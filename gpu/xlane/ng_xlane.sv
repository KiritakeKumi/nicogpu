// ng_xlane.sv -- SP cross-lane network, first cut.
//
// Executes the 37 xlane instructions of the ISA (shfl.idx/up/down/xor/rot and
// their .vi/.vu forms, shfl.bcast, quad.swizzle, writelane, readlane(.u),
// readfirst, ballot, vote.any/all/eq, xscan.add.u32, xred.* x8, and the four
// fddx/fddy fine/coarse f32 derivatives) with the semantics of
//
//
// Structure (one instruction in flight):
//   issue -> CHK (decode checks, traps) -> [UR: uniform read] -> GAT (read the
//   whole source operand(s), NBEAT beats of NL lanes, 2 read ports) -> C0 (one
//   cycle: ONE shared 32:1 lane mux per lane for every permute / readlane /
//   readfirst, vote / ballot, scan / reduction operand load, derivative stage A)
//   -> [IT: scan / reduction, one tree level per cycle on the result register,
//   5 cycles] or [C1: derivative stage B] -> POST (late traps) -> WR (NBEAT
//   beats on vector write port 0) or UW (uniform write, result lane 0)
//   -> done / trap pulse.
// A cross-lane op needs all 32 lanes of its source before any result lane, so
// the operand is gathered into a 1024-bit register first; the network then works
// on all 32 lanes at once and the result is written back beat by beat.
// (timing / area rework of the first cut, which was one 11.3 ns COMP cycle
// with six parallel 32:1 muxes per lane): the second operand is reduced to 6 bits
// per lane at gather time ({|d[31:5], d[4:0]}); the per-lane source index is
// computed by a small function and drives a single mux; scan and reduction are
// iterative (res_q doubles as the working register, 5 cycles, 32 + 16 adders /
// operators instead of 129 + 31); the f32 subtractors are split in two stages
// (ng_xlane_fsub_a / _b).  The architectural behaviour is unchanged.
//
// Register file interface = ng_sp_regfile: read ports 0 (rs1) and 1
// (rs2), write port 0, uniform read port 0, uniform write port 0.  The port
// shapes match ng_valu so the same arbitration glue can serve both units.
//
// Traps are precise, in the model's order:
//   1. IllegalInsn (cause 1, info = instruction word): unknown/reserved encoding;
//      derivatives with a reserved MODE.rm (5..7); writelane lane7 >= 32.
//   2. RegRange (cause 10, info = index): vector sources (rs2 before rs1 for
//      shfl.idx/xor/rot, as in the model), then the destination.
//   3. derivatives only: FpException (cause 4, info = raised flags) comes
//      BEFORE the destination range check (the model commits fflags first).
// fflags_o carries the instruction's raised flags in the cycle of done_valid /
// of an FpException trap (the owner ORs them into fflags); it is 0 otherwise.
module ng_xlane
  import ng_xlane_pkg::*;
#(
  parameter int NL    = 16,  // physical lanes per beat (16 or 32, must equal NG_RF_NL)
  parameter int SLOTW = 4    // warp slot id width (NG_RF_NUM_WARPS = 2**SLOTW)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- issue ---------------------------------------------------------------
  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,      // instruction word (xlane encodings are 32-bit)
  input  logic [31:0]       iss_exec,    // EXEC at issue
  input  logic [31:0]       iss_mode,    // MODE CSR at issue (rm32 / daz32 / ftz32 / trap enables)
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,   // warp's vreg allocation (<=128)
  input  logic [15:0]       iss_tag,     // opaque, returned with done

  // ---- completion (one-cycle pulses; exactly one per accepted instruction) ---
  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic [4:0]        fflags_o,    // valid with done_valid / FpException trap
  output logic              busy,

  // ---- vector register file: read ports (0 = rs1, 1 = rs2, 2 unused) ----------
  output logic [2:0]                rd_valid,
  input  logic [2:0]                rd_ready,
  output logic [SLOTW-1:0]          rd_slot [3],
  output logic [6:0]                rd_reg  [3],
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat [3],
  output logic [1:0]                rd_hsel [3],
  input  logic [2:0]                rdv_valid,
  input  logic [2:0]                rdv_err,
  input  logic [NL*32-1:0]          rdv_data [3],

  // ---- vector register file: write port --------------------------------------
  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  // ---- uniform register file --------------------------------------------------
  output logic [1:0]                ur_valid,
  output logic [SLOTW-1:0]          ur_slot [2],
  output logic [5:0]                ur_reg  [2],
  input  logic [1:0]                urd_valid,
  input  logic [31:0]               urd_data [2],

  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data
);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int DW    = NL * 32;

  // ---------------------------------------------------------------- control state
  typedef enum logic [3:0] {
    ST_IDLE, ST_CHK, ST_UR, ST_URW, ST_GAT, ST_C0, ST_IT, ST_C1, ST_POST, ST_WR, ST_UW
  } st_t;
  st_t st;

  logic [31:0]      w0_q, exec_q, mode_q;
  logic [SLOTW-1:0] slot_q;
  logic [7:0]       nregs_q;
  logic [15:0]      tag_q;
  dec_t             dq;
  logic [31:0]      uv0;
  logic             err_seen;
  logic [4:0]       first_q;     // lowest active lane (0 when EXEC = 0), registered in CHK
  logic [2:0]       it;          // scan / reduction step

  dec_t dec_c;
  ng_xlane_dec u_dec (.w0(iss_w0), .d(dec_c));

  // ---------------------------------------------------------------- CHK logic
  logic a_oor, b_oor, d_oor, rm_bad, lane_bad, chk_illegal, chk_range, d_vec;
  logic [6:0] src_idx;
  always_comb begin
    a_oor    = dq.a_vec && ({1'b0, dq.rs1} >= nregs_q);
    b_oor    = (dq.b_kind == B_VREG) && ({1'b0, dq.rs2} >= nregs_q);
    d_vec    = (dq.d_kind != D_UREG);
    d_oor    = d_vec && ({1'b0, dq.rd} >= nregs_q);
    rm_bad   = dq.fp && (mode_q[2:0] > 3'd4);
    lane_bad = (dq.uop == U_WLANE) && (dq.rs1 >= 7'd32);
    chk_illegal = !dq.hit || rm_bad || lane_bad;
    if (dq.b_first) src_idx = b_oor ? dq.rs2 : dq.rs1;
    else            src_idx = a_oor ? dq.rs1 : dq.rs2;
    // source faults first, then (non-fp) destination
    chk_range = dq.hit && !chk_illegal && (a_oor || b_oor || (d_oor && !dq.fp));
  end

  // --------------------------------------------------------------
  logic [BTW-1:0]    gb;
  logic [1:0]        pend, need, rsp_v;
  logic [32*32-1:0]  srcA;

  assign need = {dq.b_kind == B_VREG, 1'b1};

  always_comb begin
    for (int k = 0; k < 3; k++) begin
      rd_valid[k] = 1'b0;
      rd_slot[k]  = slot_q;
      rd_reg[k]   = (k == 1) ? dq.rs2 : dq.rs1;
      rd_beat[k]  = gb;
      rd_hsel[k]  = 2'd0;
    end
    rd_valid[0] = (st == ST_GAT) && pend[0];
    rd_valid[1] = (st == ST_GAT) && pend[1];
  end
  logic [2:0] gnt;
  assign gnt = rd_valid & rd_ready;

  // --------------------------------------------------------------
  logic [31:0] av [32];
  logic [5:0]  dl [32];        // per-lane index operand {|d[31:5], d[4:0]}
  logic [5:0]  dlq [32];       // ... of the gathered rs2 (B_VREG)
  always_comb begin
    for (int i = 0; i < 32; i++) begin
      av[i] = srcA[i*32 +: 32];
      case (dq.b_kind)
        B_VREG:  dl[i] = dlq[i];
        B_IMM:   dl[i] = {|dq.imm[7:5], dq.imm[4:0]};
        B_UREG:  dl[i] = {|uv0[31:5], uv0[4:0]};
        default: dl[i] = 6'd0;
      endcase
    end
  end

  // lowest active lane (first_q is its registered copy, loaded in CHK)
  wire [4:0] first_c = 5'(ng_bitops_pkg::lowest_set({32'd0, exec_q}));

  // ---- per-lane source index of every permute (one shared 32:1 mux per lane)
  logic [4:0] sidx [32];
  always_comb begin
    for (int j = 0; j < 32; j++) begin
      logic [4:0] d;
      logic       nz;
      logic [5:0] jj;
      logic [1:0] pos;
      logic [4:0] jv;
      jv = 5'(j);
      d = dl[j][4:0];
      nz = dl[j][5];
      jj = 6'(j) + {1'b0, d};
      pos = 2'(j);
      case (dq.uop)
        U_IDX:    sidx[j] = d;
        U_UP:     sidx[j] = (!nz && d <= 5'(j)) ? (5'(j) - d) : 5'(j);
        U_DOWN:   sidx[j] = (!nz && !jj[5]) ? jj[4:0] : 5'(j);
        U_XOR:    sidx[j] = 5'(j) ^ d;
        U_ROT:    sidx[j] = jj[4:0];
        U_QUAD:   sidx[j] = {jv[4:2], dq.imm[2*pos +: 2]};
        U_RFIRST: sidx[j] = first_q;
        U_RLANE:  sidx[j] = d;
        default:  sidx[j] = 5'(j);
      endcase
    end
  end

  // ---- derivatives: 16 subtractors in two register stages; unit u = (quad u/2, half u%2)
  ng_xlane_fsub_pkg::fsub_t fsq [16];     // stage A results (loaded in C0)
  logic [31:0] fs_a [16], fs_b [16], fs_r [16];
  logic [4:0]  fs_f [16];
  ng_xlane_fsub_pkg::fsub_t fs_t [16];
  for (genvar u = 0; u < 16; u++) begin : g_fsub
    localparam int Q = u / 2;
    localparam int S = u % 2;
    always_comb begin
      case (dq.uop)
        U_DDXF:  begin fs_a[u] = av[4*Q + 2*S + 1]; fs_b[u] = av[4*Q + 2*S]; end
        U_DDYF:  begin fs_a[u] = av[4*Q + 2 + S];   fs_b[u] = av[4*Q + S];   end
        U_DDYC:  begin fs_a[u] = av[4*Q + 2];       fs_b[u] = av[4*Q];       end
        default: begin fs_a[u] = av[4*Q + 1];       fs_b[u] = av[4*Q];       end  // DDXC
      endcase
    end
    ng_xlane_fsub_a u_fsa (.a(fs_a[u]), .b(fs_b[u]), .daz(mode_q[9]), .o(fs_t[u]));
    ng_xlane_fsub_b u_fsb (.i(fsq[u]), .rm(mode_q[2:0]), .ftz(mode_q[10]), .r(fs_r[u]), .flags(fs_f[u]));
  end

  function automatic logic [31:0] red_ident(input logic [4:0] uop);
    case (uop)
      U_RMINI: return 32'h7fffffff;
      U_RMAXI: return 32'h80000000;
      U_RMINU, U_RAND: return 32'hffffffff;
      default: return 32'd0;
    endcase
  endfunction
  function automatic logic [31:0] red_op(input logic [4:0] uop, input logic [31:0] x, input logic [31:0] y);
    case (uop)
      U_RADD:  return x + y;
      U_RMINI: return ($signed(x) < $signed(y)) ? x : y;
      U_RMAXI: return ($signed(x) > $signed(y)) ? x : y;
      U_RMINU: return (x < y) ? x : y;
      U_RMAXU: return (x > y) ? x : y;
      U_RAND:  return x & y;
      U_ROR:   return x | y;
      default: return x ^ y;   // RXOR
    endcase
  endfunction

  logic [32*32-1:0] res_q;
  logic [31:0]      wmask_q;
  logic [4:0]       fl_q;
  logic [31:0]      res [32];
  always_comb for (int j = 0; j < 32; j++) res[j] = res_q[j*32 +: 32];

  logic is_scan, is_red, is_vote;
  assign is_scan = (dq.uop == U_SCAN);
  assign is_red  = (dq.uop >= U_RADD) && (dq.uop <= U_RXOR);
  assign is_vote = (dq.uop >= U_BALLOT) && (dq.uop <= U_VEQ);

  // ---- C0: result-register load
  logic [31:0] c0_res [32];
  always_comb begin
    logic [31:0] nzm, zm, ref_v;
    logic        eq;
    for (int j = 0; j < 32; j++) begin
      if (is_scan || is_red)         c0_res[j] = exec_q[j] ? av[j] : red_ident(dq.uop);
      else if (dq.uop == U_WLANE)    c0_res[j] = uv0;
      else                           c0_res[j] = av[sidx[j]];
    end
    // vote / ballot -> result lane 0
    nzm = '0; zm = '0;
    for (int j = 0; j < 32; j++) begin
      nzm[j] = exec_q[j] && (av[j] != 32'd0);
      zm[j]  = exec_q[j] && (av[j] == 32'd0);
    end
    ref_v = av[first_q];
    eq = 1'b1;
    for (int j = 0; j < 32; j++) if (exec_q[j] && av[j] != ref_v) eq = 1'b0;
    if (is_vote) begin
      case (dq.uop)
        U_BALLOT: c0_res[0] = nzm;
        U_VANY:   c0_res[0] = {31'd0, nzm != 32'd0};
        U_VALL:   c0_res[0] = {31'd0, zm == 32'd0};
        default:  c0_res[0] = {31'd0, eq};     // VEQ
      endcase
    end
  end

  // ---- IT: one level of the inclusive prefix sum (Kogge-Stone) or of the reduction tree per cycle
  logic [31:0] it_res [32];
  always_comb begin
    for (int j = 0; j < 32; j++) begin
      it_res[j] = res[j];
      if (is_scan) begin
        for (int k = 0; k < 5; k++)
          if (int'(it) == k && j >= (1 << k)) it_res[j] = res[j] + res[j - (1 << k)];
      end else if (j < 16) begin
        it_res[j] = red_op(dq.uop, res[2*j], res[2*j+1]);
      end
    end
  end

  // ---- C1: derivative results; flags of the units an active lane uses
  logic [31:0] c1_res [32];
  logic [4:0]  c1_fl;
  always_comb begin
    logic [15:0] used;
    int          unit_of, q, p;
    used = '0;
    c1_fl = 5'd0;
    for (int j = 0; j < 32; j++) begin
      q = j / 4; p = j % 4;
      case (dq.uop)
        U_DDXF: unit_of = 2*q + (p / 2);
        U_DDYF: unit_of = 2*q + (p % 2);
        default: unit_of = 2*q;
      endcase
      c1_res[j] = fs_r[unit_of];
      if (exec_q[j]) used[unit_of] = 1'b1;
    end
    for (int u = 0; u < 16; u++) if (used[u]) c1_fl |= fs_f[u];
  end

  // ---------------------------------------------------------------- results / write
  logic [BTW-1:0]   wb;

  logic [DW-1:0]    wslice;
  logic [NL-1:0]    wlanes_c;
  always_comb begin
    wslice   = res_q[int'(wb) * DW +: DW];
    wlanes_c = wmask_q[int'(wb) * NL +: NL];
  end
  always_comb begin
    wr_valid = (st == ST_WR) && (wlanes_c != '0);
    wr_slot  = slot_q;
    wr_reg   = dq.rd;
    wr_beat  = wb;
    wr_lanes = wlanes_c;
    wr_hsel  = 2'b11;
    wr_data  = wslice;
  end

  // --------------------------------------------------------------
  logic fp_trap;
  assign fp_trap = dq.fp && ((fl_q & mode_q[20:16]) != 5'd0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= ST_IDLE;
      done_valid <= 1'b0; trap_valid <= 1'b0;
      trap_cause <= '0; trap_info <= '0; done_tag <= '0; fflags_o <= '0;
      pend <= '0; rsp_v <= '0; gb <= '0; wb <= '0; it <= '0;
      err_seen <= 1'b0;
    end else begin
      done_valid <= 1'b0;
      trap_valid <= 1'b0;
      fflags_o   <= 5'd0;
      rsp_v      <= gnt[1:0];

      case (st)
        ST_IDLE: begin
          if (iss_valid) begin
            w0_q    <= iss_w0;
            exec_q  <= iss_exec;
            mode_q  <= iss_mode;
            slot_q  <= iss_slot;
            nregs_q <= iss_nregs;
            tag_q   <= iss_tag;
            dq      <= dec_c;
            st      <= ST_CHK;
          end
        end

        ST_CHK: begin
          if (chk_illegal || chk_range) begin
            trap_valid <= 1'b1;
            trap_cause <= chk_illegal ? CAUSE_ILLEGAL : CAUSE_REGRANGE;
            trap_info  <= chk_illegal ? w0_q : (a_oor || b_oor) ? {25'd0, src_idx} : {25'd0, dq.rd};
            st         <= ST_IDLE;
          end else if (dq.b_kind == B_UREG) begin
            st <= ST_UR;
          end else begin
            st       <= dq.a_vec ? ST_GAT : ST_C0;
            pend     <= need;
            gb       <= '0;
            err_seen <= 1'b0;
          end
        end

        ST_UR: st <= ST_URW;   // ur_valid/ur_reg driven combinationally in this state

        ST_URW: begin
          st       <= dq.a_vec ? ST_GAT : ST_C0;
          pend     <= need;
          gb       <= '0;
          err_seen <= 1'b0;
        end

        ST_GAT: begin
          for (int k = 0; k < 2; k++) if (gnt[k]) pend[k] <= 1'b0;
          if (pend == 2'd0 && rsp_v == 2'd0) begin
            if (int'(gb) == NBEAT - 1) begin
              st <= ST_C0;
            end else begin
              gb   <= gb + 1'b1;
              pend <= need;
            end
          end
        end

        ST_C0: begin
          it <= 3'd0;
          st <= (is_scan || is_red) ? ST_IT : dq.fp ? ST_C1 : ST_POST;
        end

        ST_IT: begin
          it <= it + 3'd1;
          if (it == 3'd4) st <= ST_POST;
        end

        ST_C1: st <= ST_POST;

        ST_POST: begin
          wb <= '0;
          if (err_seen) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= 32'd0; st <= ST_IDLE;
          end else if (fp_trap) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_FPEXC; trap_info <= {27'd0, fl_q};
            fflags_o <= fl_q; st <= ST_IDLE;
          end else if (dq.fp && d_oor) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= {25'd0, dq.rd}; st <= ST_IDLE;
          end else if (dq.d_kind == D_UREG) begin
            st <= ST_UW;
          end else begin
            st <= ST_WR;
          end
        end

        ST_WR: begin
          if (!wr_valid || wr_ready) begin
            if (int'(wb) == NBEAT - 1) begin
              done_valid <= 1'b1; done_tag <= tag_q; fflags_o <= dq.fp ? fl_q : 5'd0; st <= ST_IDLE;
            end else begin
              wb <= wb + 1'b1;
            end
          end
        end

        ST_UW: begin
          done_valid <= 1'b1; done_tag <= tag_q; fflags_o <= 5'd0; st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase

      // ---- read responses (every cycle, independent of st) ----
      if (rsp_v[0] && rdv_valid[0]) begin
        if (rdv_err[0]) err_seen <= 1'b1;
      end
      if (rsp_v[1] && rdv_valid[1]) begin
        if (rdv_err[1]) err_seen <= 1'b1;
      end
    end
  end

  // ---- datapath registers without reset ----
  always_ff @(posedge clk) begin
    if (rsp_v[0] && rdv_valid[0] && !rdv_err[0]) srcA[int'(gb) * DW +: DW] <= rdv_data[0];
    if (rsp_v[1] && rdv_valid[1] && !rdv_err[1])
      for (int i = 0; i < NL; i++)
        dlq[int'(gb) * NL + i] <= {|rdv_data[1][i*32+5 +: 27], rdv_data[1][i*32 +: 5]};
    if (st == ST_URW && urd_valid[0]) uv0 <= urd_data[0];
    if (st == ST_CHK) first_q <= first_c;
    case (st)
      ST_C0: begin
        for (int j = 0; j < 32; j++) res_q[j*32 +: 32] <= c0_res[j];
        for (int u = 0; u < 16; u++) fsq[u] <= fs_t[u];
        wmask_q <= (dq.uop == U_WLANE) ? (32'd1 << dq.rs1[4:0]) : exec_q;
        fl_q    <= 5'd0;
      end
      ST_IT: for (int j = 0; j < 32; j++) res_q[j*32 +: 32] <= it_res[j];
      ST_C1: begin
        for (int j = 0; j < 32; j++) res_q[j*32 +: 32] <= c1_res[j];
        fl_q <= c1_fl;
      end
      default: ;
    endcase
  end

  // uniform read request / payload
  always_comb begin
    ur_valid = (st == ST_UR) ? 2'b01 : 2'b00;
    for (int k = 0; k < 2; k++) ur_slot[k] = slot_q;
    ur_reg[0] = dq.rs2[5:0];
    ur_reg[1] = 6'd0;
  end

  // uniform write
  always_comb begin
    uw_valid = (st == ST_UW);
    uw_slot  = slot_q;
    uw_reg   = dq.rd[5:0];
    uw_data  = res_q[31:0];     // uniform results are result lane 0
  end

  assign iss_ready = (st == ST_IDLE);
  assign busy      = (st != ST_IDLE);
endmodule
