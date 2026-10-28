// ng_fp64.sv -- SP FP64 unit, first cut.
//
// The 40 `unit: fp64` instructions of the ISA with the semantics of
//, bit for bit:
//   fadd/fsub/fmul/ffma(rm, neg/abs), fmin/fmax, fsgnj/n/x, fldexp, fclass,
//   ffloor/fceil/ftrunc/frndne/ffract, frexp.mant/exp, fcvt f64<->f32 (.m: explicit rm),
//   fcvt f64->i32/u32 (RTZ, saturating), fcvt i32/u32->f64, cmp.{eq..geu,o,u,class}.f64
//   (compare results go to a uniform register).
// NFL physical lanes (ng_fp64_lane: 3-stage pipelined FMA64 + side paths) sweep the
// 32 warp lanes in 32/NFL groups, one group per cycle.  ISA latency is `fixed:16`;
// this first cut is a multi-cycle sequencer.
//
// Structure (one instruction in flight):
//   issue -> CHK (precise traps: encoding, pair alignment, MODE rm, source range)
//         -> RDN/RDQ/RDR (operand registers, one read port, beat by beat)
//         -> RUN/DRN (lane groups through the pipeline; results and fflags collected)
//         -> FIN (FpException trap if enabled; destination range check)
//         -> WR (vector write port, beat by beat; lane mask = EXEC) or UW (uniform write)
//         -> DONE pulse.
// Nothing is written before the whole warp has been computed, so every trap is
// precise (model: any trap => no write-back; an enabled FP exception still updates
// fflags).  Trap order (model): IllegalInsn (encoding / odd pair / reserved rm)
// -> source RegRange (rs1, rs2, rs3; pair low then high) -> FpException
// -> destination RegRange.
//
// Register-file interface = ng_sp_regfile: vector read port 0, vector write port
// (any), uniform write port.  `iss_mode` is the warp's MODE CSR at issue;
// `fflags_set` is OR-ed into the warp's fflags when `fflags_valid` pulses (with done,
// or with an FpException trap).
module ng_fp64
  import ng_fp64_pkg::*;
  import ng_bitops_pkg::chk_vreg;
#(
  parameter int NL    = 16,  // register-file physical lanes per beat (16 or 32)
  parameter int SLOTW = 4,   // warp slot id width
  parameter int NFL   = 2    // FP64 physical lanes (divides 32)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- issue ---------------------------------------------------------------
  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,
  input  logic [31:0]       iss_w1,
  input  logic [31:0]       iss_exec,
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,
  input  logic [31:0]       iss_mode,
  input  logic [15:0]       iss_tag,

  // ---- completion (one-cycle pulses; exactly one per accepted instruction) ---
  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              fflags_valid,
  output logic [4:0]        fflags_set,
  output logic              busy,

  // ---- vector register file: one read port ------------------------------------
  output logic                      rd_valid,
  input  logic                      rd_ready,
  output logic [SLOTW-1:0]          rd_slot,
  output logic [6:0]                rd_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat,
  output logic [1:0]                rd_hsel,
  input  logic                      rdv_valid,
  input  logic                      rdv_err,
  input  logic [NL*32-1:0]          rdv_data,

  // ---- vector register file: one write port -------------------------------------
  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  // ---- uniform register file: write port (compare masks) ----------------------
  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data
);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int NG    = 32 / NFL;
  localparam int GW    = $clog2(NG) + 1;

  typedef enum logic [3:0] {
    ST_IDLE, ST_CHK, ST_RDN, ST_RDQ, ST_RDR, ST_RUN, ST_DRN, ST_FIN, ST_WR, ST_UW, ST_DONE
  } st_t;
  st_t st;

  logic [31:0]      w0_q, exec_q, mode_q;
  logic [SLOTW-1:0] slot_q;
  logic [7:0]       nregs_q;
  logic [15:0]      tag_q;
  dec_t             dq;

  dec_t dec_c;
  ng_fp64_dec u_dec (.w0(iss_w0), .w1(iss_w1), .d(dec_c));

  // ---------------------------------------------------------------- CHK logic
  // source: returns {fault, idx[7:0]} (first out-of-range register of the operand)
  function automatic logic [8:0] chk_reg(input logic [1:0] kind, input logic [6:0] r, input logic [7:0] n);
    return chk_vreg(kind == K_VREG || kind == K_VPAIR, kind == K_VPAIR, {1'b0, r}, n);
  endfunction

  logic        chk_illegal, chk_range, rm_bad, has_scope, c_fault;
  logic [7:0]  chk_idx;
  logic [31:0] chk_info;
  logic [2:0]  rm_eff, rm_code;
  logic        daz_en, daz32_en;
  logic [1:0]  ftz_k;
  logic [8:0]  ca, cb, cc, cd;

  always_comb begin
    has_scope = (dq.scope == SC_F64) || (dq.scope == SC_F32);
    if (dq.has_rm && dq.rm != 3'd7) rm_code = dq.rm;
    else begin
      rm_code = (dq.scope == SC_F32) ? mode_q[2:0] : mode_q[8:6];
      if (rm_code == 3'd7) rm_code = mode_q[2:0];
    end
    rm_bad = has_scope && (rm_code > 3'd4);
    rm_eff = has_scope ? rm_code : 3'd0;
    daz_en   = (dq.dazk == 2'd1) && mode_q[13];
    daz32_en = (dq.dazk == 2'd2) && mode_q[9];
    ftz_k    = (dq.ftzk == 2'd1) ? (mode_q[14] ? 2'd1 : 2'd0) : (dq.ftzk == 2'd2) ? (mode_q[10] ? 2'd2 : 2'd0) : 2'd0;

    ca = chk_reg(dq.a_kind, dq.rs1, nregs_q);
    cb = chk_reg(dq.b_kind, dq.rs2, nregs_q);
    cc = chk_reg(dq.c_kind, dq.rs3, nregs_q);
    cd = chk_reg(dq.d_kind, dq.rd,  nregs_q);   // destination (checked after the FP-exception check)
    chk_illegal = !dq.hit
                  || (dq.d_kind == K_VPAIR && dq.rd[0])
                  || (dq.a_kind == K_VPAIR && dq.rs1[0])
                  || (dq.b_kind == K_VPAIR && dq.rs2[0])
                  || (dq.c_kind == K_VPAIR && dq.rs3[0])
                  || rm_bad;
    chk_range = !chk_illegal && (ca[8] || cb[8] || cc[8]);
    chk_idx   = ca[8] ? ca[7:0] : cb[8] ? cb[7:0] : cc[7:0];
    chk_info  = chk_illegal ? w0_q : {24'd0, chk_idx};
    c_fault   = cd[8];
  end

  // ---------------------------------------------------------------- operand storage
  logic [63:0] opa [32];
  logic [63:0] opb [32];
  logic [63:0] opc [32];
  logic [63:0] resbuf [32];
  logic [2:0]  ri;
  logic [BTW-1:0] rb;
  logic        rd_err_seen;

  // item table: 0 = A lo, 1 = A hi, 2 = B lo, 3 = B hi, 4 = C lo, 5 = C hi
  function automatic logic item_ok(input dec_t d, input logic [2:0] i);
    case (i)
      3'd0:    return d.a_kind == K_VREG || d.a_kind == K_VPAIR;
      3'd1:    return d.a_kind == K_VPAIR;
      3'd2:    return d.b_kind == K_VREG || d.b_kind == K_VPAIR;
      3'd3:    return d.b_kind == K_VPAIR;
      3'd4:    return d.c_kind == K_VREG || d.c_kind == K_VPAIR;
      3'd5:    return d.c_kind == K_VPAIR;
      default: return 1'b0;
    endcase
  endfunction
  function automatic logic [6:0] item_reg(input dec_t d, input logic [2:0] i);
    case (i)
      3'd0:    return d.rs1;
      3'd1:    return d.rs1 + 7'd1;
      3'd2:    return d.rs2;
      3'd3:    return d.rs2 + 7'd1;
      3'd4:    return d.rs3;
      default: return d.rs3 + 7'd1;
    endcase
  endfunction

  always_comb begin
    rd_valid = (st == ST_RDQ);
    rd_slot  = slot_q;
    rd_reg   = item_reg(dq, ri);
    rd_beat  = rb;
    rd_hsel  = 2'd0;
  end

  // --------------------------------------------------------------
  logic [GW-1:0]    gi;                    // next group to issue
  logic             issue;
  logic [2:0]       pv;                    // pipeline valid, stages 1..3 (3 = result present)
  logic [GW-1:0]    pg1, pg2, pg3;
  logic [4:0]       fl_acc;
  logic [63:0]      ln_a [NFL], ln_b [NFL], ln_c [NFL], ln_res [NFL];
  logic [4:0]       ln_fl [NFL];

  assign issue = (st == ST_RUN) && (int'(gi) < NG);

  for (genvar j = 0; j < NFL; j++) begin : g_lane
    always_comb begin
      ln_a[j] = opa[(int'(gi) * NFL + j) % 32];
      ln_b[j] = opb[(int'(gi) * NFL + j) % 32];
      ln_c[j] = opc[(int'(gi) * NFL + j) % 32];
    end
    ng_fp64_lane u_lane (
      .clk(clk), .op(dq.lop), .pred(dq.pred), .rm(rm_eff), .daz_en(daz_en), .daz32_en(daz32_en),
      .ftz_k(ftz_k), .neg_m(dq.neg), .abs_m(dq.abs),
      .a(ln_a[j]), .b(ln_b[j]), .c(ln_c[j]), .res(ln_res[j]), .flags(ln_fl[j])
    );
  end

  // ---------------------------------------------------------------- write stage
  logic [2:0]      wi;          // destination register index (0 / 1)
  logic [BTW-1:0]  wb;
  logic [NL-1:0]   w_lanes;
  always_comb begin
    w_lanes = '0;
    for (int i = 0; i < NL; i++) w_lanes[i] = exec_q[int'(wb) * NL + i];
  end

  always_comb begin
    wr_valid = (st == ST_WR) && (w_lanes != '0);
    wr_slot  = slot_q;
    wr_reg   = dq.rd + 7'(wi);
    wr_beat  = wb;
    wr_lanes = w_lanes;
    wr_hsel  = 2'b11;
    for (int i = 0; i < NL; i++) begin
      logic [63:0] rv;
      rv = resbuf[int'(wb) * NL + i];
      wr_data[i*32 +: 32] = (wi == 3'd0) ? rv[31:0] : rv[63:32];
    end
  end

  logic [31:0] ures;
  always_comb begin
    ures = 32'd0;
    for (int l = 0; l < 32; l++) ures[l] = exec_q[l] & resbuf[l][0];
  end
  assign uw_valid = (st == ST_UW);
  assign uw_slot  = slot_q;
  assign uw_reg   = dq.rd[5:0];
  assign uw_data  = ures;

  logic fp_trap;
  assign fp_trap = (dq.scope != SC_NONE) && (|(fl_acc & mode_q[20:16]));   // cmp.* trap too (rm fixed RNE)

  // --------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= ST_IDLE;
      done_valid <= 1'b0; trap_valid <= 1'b0; fflags_valid <= 1'b0;
      trap_cause <= '0; trap_info <= '0; done_tag <= '0; fflags_set <= '0;
      pv <= '0; gi <= '0;
    end else begin
      done_valid   <= 1'b0;
      trap_valid   <= 1'b0;
      fflags_valid <= 1'b0;
      pv  <= {pv[1:0], issue};
      pg1 <= gi;
      pg2 <= pg1;
      pg3 <= pg2;

      // result capture: lane outputs of the group issued 3 cycles ago
      if (pv[2]) begin
        logic [4:0] fo;
        fo = fl_acc;
        for (int j = 0; j < NFL; j++) begin
          resbuf[int'(pg3) * NFL + j] <= ln_res[j];
          if (exec_q[int'(pg3) * NFL + j]) fo = fo | ln_fl[j];
        end
        fl_acc <= fo;
      end

      case (st)
        ST_IDLE: if (iss_valid) begin
          w0_q <= iss_w0; exec_q <= iss_exec; slot_q <= iss_slot; nregs_q <= iss_nregs;
          mode_q <= iss_mode; tag_q <= iss_tag; dq <= dec_c;
          st <= ST_CHK;
        end

        ST_CHK: begin
          if (chk_illegal || chk_range) begin
            trap_valid <= 1'b1;
            trap_cause <= chk_illegal ? CAUSE_ILLEGAL : CAUSE_REGRANGE;
            trap_info  <= chk_info;
            st <= ST_IDLE;
          end else begin
            ri <= 3'd0; rb <= '0; rd_err_seen <= 1'b0; fl_acc <= 5'd0; gi <= '0;
            for (int l = 0; l < 32; l++) begin opa[l] <= 64'd0; opb[l] <= 64'd0; opc[l] <= 64'd0; resbuf[l] <= 64'd0; end
            st <= ST_RDN;
          end
        end

        // ---- operand reads, one (register, beat) at a time
        ST_RDN: begin
          if (ri > 3'd5) st <= ST_RUN;
          else if (!item_ok(dq, ri)) ri <= ri + 3'd1;
          else begin rb <= '0; st <= ST_RDQ; end
        end
        ST_RDQ: if (rd_ready) st <= ST_RDR;
        ST_RDR: if (rdv_valid) begin
          if (rdv_err) rd_err_seen <= 1'b1;
          for (int i = 0; i < NL; i++) begin
            case (ri)
              3'd0: opa[int'(rb) * NL + i][31:0]  <= rdv_data[i*32 +: 32];
              3'd1: opa[int'(rb) * NL + i][63:32] <= rdv_data[i*32 +: 32];
              3'd2: opb[int'(rb) * NL + i][31:0]  <= rdv_data[i*32 +: 32];
              3'd3: opb[int'(rb) * NL + i][63:32] <= rdv_data[i*32 +: 32];
              3'd4: opc[int'(rb) * NL + i][31:0]  <= rdv_data[i*32 +: 32];
              default: opc[int'(rb) * NL + i][63:32] <= rdv_data[i*32 +: 32];
            endcase
          end
          if (int'(rb) == NBEAT - 1) begin ri <= ri + 3'd1; st <= ST_RDN; end
          else begin rb <= rb + 1'b1; st <= ST_RDQ; end
        end

        // ---- lane groups: one group per cycle, 3-cycle pipeline
        ST_RUN: begin
          gi <= gi + 1'b1;
          if (int'(gi) == NG - 1) st <= ST_DRN;
        end
        ST_DRN: if (pv[1:0] == 2'b00 && !pv[2]) st <= ST_FIN;

        ST_FIN: begin
          if (rd_err_seen) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= 32'd0; st <= ST_IDLE;
          end else if (fp_trap) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_FPEXC; trap_info <= {27'd0, fl_acc};
            fflags_valid <= 1'b1; fflags_set <= fl_acc;
            st <= ST_IDLE;
          end else if (c_fault) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= {24'd0, cd[7:0]};
            st <= ST_IDLE;
          end else if (dq.d_kind == K_UREG) begin
            st <= ST_UW;
          end else begin
            wi <= 3'd0; wb <= '0;
            st <= ST_WR;
          end
        end

        // ---- write-back (skips beats whose lane mask is empty)
        ST_WR: begin
          if (w_lanes == '0 || wr_ready) begin
            if (int'(wb) == NBEAT - 1) begin
              wb <= '0;
              if (dq.d_kind == K_VPAIR && wi == 3'd0) wi <= 3'd1;
              else st <= ST_DONE;
            end else begin
              wb <= wb + 1'b1;
            end
          end
        end

        ST_UW: st <= ST_DONE;

        ST_DONE: begin
          done_valid <= 1'b1; done_tag <= tag_q;
          fflags_valid <= 1'b1; fflags_set <= fl_acc;
          st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

  assign iss_ready = (st == ST_IDLE);
  assign busy      = (st != ST_IDLE);
endmodule
