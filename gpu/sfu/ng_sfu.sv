// ng_sfu.sv -- SP special-function unit (SFU), first cut.
//
// The 24 `unit: sfu` instructions of the ISA with the semantics of
//, bit for bit:
//   rcp / rsq / sqrt / exp2 / log2 / sin / cos (f16, f32; f64 for rcp rsq),
//   fsqrt.f32/.f64, fdiv.f32/.f64, div / divu / rem / remu.
// "A-tier" numerics: correctly rounded divide
// sqrt; exp2 / log2 / sin / cos / rsq via the fixed binary64 kernels of
// sfu_math.h, rounded once to the destination format.  The unit is iterative
// (ISA: "采用迭代实现"): NSL physical lanes, each an FMA64 + radix-2 div/sqrt
// engine, sweep the 32 warp lanes in 32/NSL groups.
//
// Structure (one instruction in flight):
//   issue -> CHK (precise traps: encoding, pair alignment, MODE rm, source range)
//         -> RD  (operand registers, one read port, beat by beat)
//         -> RUN (lane groups; results and fflags collected)
//         -> FIN (FpException trap if enabled; destination range check)
//         -> WR  (vector write port, beat by beat; lane mask = EXEC) -> done pulse.
// Nothing is written before the whole warp has been computed, so every trap
// is precise (model: enabled FP exception => no write-back, fflags updated;
// register-range trap => nothing changes).
//
// Register-file interface = ng_sp_regfile: one vector read port and
// ONE write port (the register file's SFU/load write port).  `iss_mode` is the
// warp's MODE CSR at issue; `fflags_set` is OR-ed into the warp's fflags when
// `fflags_valid` pulses (with done, or with an FpException trap).
module ng_sfu
  import ng_sfu_pkg::*;
  import ng_bitops_pkg::chk_vreg;
#(
  parameter int NL    = 16,  // register-file physical lanes per beat (16 or 32)
  parameter int SLOTW = 4,   // warp slot id width
  parameter int NSL   = 1    // SFU physical lanes (divides 32); M1: one lane sweeps the warp
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
  output logic [NL*32-1:0]          wr_data
);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int NG    = 32 / NSL;

  typedef enum logic [3:0] {
    ST_IDLE, ST_CHK, ST_RDN, ST_RDQ, ST_RDR, ST_RUN, ST_GO, ST_WAIT, ST_FIN, ST_WRN, ST_WRQ, ST_DONE
  } st_t;
  st_t st;

  logic [31:0]      w0_q, exec_q, mode_q;
  logic [SLOTW-1:0] slot_q;
  logic [7:0]       nregs_q;
  logic [15:0]      tag_q;
  dec_t             dq;

  dec_t dec_c;
  ng_sfu_dec u_dec (.w0(iss_w0), .w1(iss_w1), .d(dec_c));

  // ---------------------------------------------------------------- CHK logic
  // source: returns {fault, idx[7:0]} (first out-of-range register of the operand; K_VREG16 codes are {reg, half})
  function automatic logic [8:0] chk_src(input logic [1:0] kind, input logic [7:0] r, input logic [7:0] n);
    return chk_vreg(kind != K_NONE, kind == K_VPAIR, (kind == K_VREG16) ? {1'b0, r[7:1]} : {1'b0, r[6:0]}, n);
  endfunction

  logic        chk_illegal, chk_range, rm_bad;
  logic [7:0]  chk_idx;
  logic [31:0] chk_info;
  logic [2:0]  rm_eff, rm_code;
  logic        daz_c, ftz_c;
  logic        d_fault;
  logic [7:0]  d_info;
  logic [8:0]  ca, cb;
  logic [6:0]  d_reg;
  logic [7:0]  d_first;

  always_comb begin
    case (dq.fmt)
      2'd0:    rm_code = mode_q[5:3];
      2'd2:    rm_code = mode_q[8:6];
      default: rm_code = mode_q[2:0];
    endcase
    if (rm_code == 3'd7) rm_code = mode_q[2:0];
    rm_bad = dq.use_mode && (rm_code > 3'd4);
    rm_eff = dq.use_mode ? rm_code : 3'd0;
    case (dq.fmt)
      2'd0:    begin daz_c = mode_q[11]; ftz_c = mode_q[12]; end
      2'd2:    begin daz_c = mode_q[13]; ftz_c = mode_q[14]; end
      default: begin daz_c = mode_q[9];  ftz_c = mode_q[10]; end
    endcase
    if (!dq.use_mode) begin daz_c = 1'b0; ftz_c = 1'b0; end

    ca = chk_src(dq.a_kind, dq.rs1, nregs_q);
    cb = chk_src(dq.b_kind, dq.rs2, nregs_q);
    chk_illegal = !dq.hit
                  || (dq.d_kind == K_VPAIR && dq.rd[0])
                  || (dq.a_kind == K_VPAIR && dq.rs1[0])
                  || (dq.b_kind == K_VPAIR && dq.rs2[0])
                  || rm_bad;
    chk_range = !chk_illegal && (ca[8] || cb[8]);
    chk_idx   = ca[8] ? ca[7:0] : cb[7:0];
    chk_info  = chk_illegal ? w0_q : {24'd0, chk_idx};

    // destination (checked after the FP-exception check, like the model's deferred write)
    d_first = (dq.d_kind == K_VREG16) ? {1'b0, dq.rd[7:1]} : {1'b0, dq.rd[6:0]};
    d_reg   = d_first[6:0];
    d_fault = 1'b0; d_info = 8'd0;
    if (dq.d_kind == K_VPAIR) begin
      if ({1'b0, d_first} + 9'd2 > {1'b0, nregs_q}) begin
        // ERRATA v0.1.4: info = first out-of-range register (d_first < nregs here means d_first == nregs-1)
        d_fault = 1'b1; d_info = (d_first >= nregs_q) ? d_first : d_first + 8'd1;
      end
    end else begin
      if (d_first >= nregs_q) begin d_fault = 1'b1; d_info = d_first; end
    end
  end

  // ---------------------------------------------------------------- operand storage
  logic [63:0] opa [32];
  logic [63:0] opb [32];
  logic [63:0] resbuf [32];
  logic [2:0]  ri;
  logic [BTW-1:0] rb;
  logic        rd_err_seen;

  // item table: 0 = A lo, 1 = A hi, 2 = B lo, 3 = B hi
  function automatic logic item_ok(input dec_t d, input logic [2:0] i);
    case (i)
      3'd0:    return d.a_kind != K_NONE;
      3'd1:    return d.a_kind == K_VPAIR;
      3'd2:    return d.b_kind != K_NONE;
      3'd3:    return d.b_kind == K_VPAIR;
      default: return 1'b0;
    endcase
  endfunction
  function automatic logic [6:0] item_reg(input dec_t d, input logic [2:0] i);
    case (i)
      3'd0:    return (d.a_kind == K_VREG16) ? d.rs1[7:1] : d.rs1[6:0];
      3'd1:    return d.rs1[6:0] + 7'd1;
      3'd2:    return (d.b_kind == K_VREG16) ? d.rs2[7:1] : d.rs2[6:0];
      default: return d.rs2[6:0] + 7'd1;
    endcase
  endfunction
  function automatic logic [1:0] item_hsel(input dec_t d, input logic [2:0] i);
    case (i)
      3'd0:    return (d.a_kind == K_VREG16) ? (d.rs1[0] ? 2'd2 : 2'd1) : 2'd0;
      3'd2:    return (d.b_kind == K_VREG16) ? (d.rs2[0] ? 2'd2 : 2'd1) : 2'd0;
      default: return 2'd0;
    endcase
  endfunction

  always_comb begin
    rd_valid = (st == ST_RDQ);
    rd_slot  = slot_q;
    rd_reg   = item_reg(dq, ri);
    rd_beat  = rb;
    rd_hsel  = item_hsel(dq, ri);
  end

  // --------------------------------------------------------------
  int g;                                   // current group
  logic [NSL-1:0]   act_mask, pend;
  logic [NSL-1:0]   ln_start, ln_done;
  logic [63:0]      ln_a [NSL], ln_b [NSL], ln_res [NSL];
  logic [4:0]       ln_fl [NSL];
  logic [4:0]       fl_acc;

  always_comb begin
    act_mask = '0;
    for (int j = 0; j < NSL; j++) act_mask[j] = (g < NG) ? exec_q[(g * NSL + j) % 32] : 1'b0;
  end
  assign ln_start = (st == ST_GO) ? act_mask : '0;

  for (genvar j = 0; j < NSL; j++) begin : g_lane
    always_comb begin
      ln_a[j] = opa[(g * NSL + j) % 32];
      ln_b[j] = opb[(g * NSL + j) % 32];
    end
    ng_sfu_lane u_lane (
      .clk(clk), .rst_n(rst_n), .start(ln_start[j]), .op(dq.op), .iop(dq.iop), .fmt(dq.fmt),
      .rm(rm_eff), .daz(daz_c), .ftz(ftz_c), .a(ln_a[j]), .b(ln_b[j]),
      .done(ln_done[j]), .res(ln_res[j]), .fl(ln_fl[j])
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
    wr_valid = (st == ST_WRQ) && (w_lanes != '0);
    wr_slot  = slot_q;
    wr_reg   = d_reg + 7'(wi);
    wr_beat  = wb;
    wr_lanes = w_lanes;
    wr_hsel  = (dq.d_kind == K_VREG16) ? (dq.rd[0] ? 2'b10 : 2'b01) : 2'b11;
    for (int i = 0; i < NL; i++) begin
      logic [63:0] rv;
      rv = resbuf[int'(wb) * NL + i];
      case (dq.d_kind)
        K_VPAIR:  wr_data[i*32 +: 32] = (wi == 3'd0) ? rv[31:0] : rv[63:32];
        K_VREG16: wr_data[i*32 +: 32] = dq.rd[0] ? {rv[15:0], 16'd0} : {16'd0, rv[15:0]};
        default:  wr_data[i*32 +: 32] = rv[31:0];
      endcase
    end
  end

  logic [4:0] fp_en;
  assign fp_en = mode_q[20:16];
  logic fp_trap;
  assign fp_trap = |(fl_acc & fp_en);

  // --------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= ST_IDLE;
      done_valid <= 1'b0; trap_valid <= 1'b0; fflags_valid <= 1'b0;
      trap_cause <= '0; trap_info <= '0; done_tag <= '0; fflags_set <= '0;
    end else begin
      done_valid   <= 1'b0;
      trap_valid   <= 1'b0;
      fflags_valid <= 1'b0;

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
            ri <= 3'd0; rb <= '0; rd_err_seen <= 1'b0; fl_acc <= 5'd0; g <= 0;
            for (int l = 0; l < 32; l++) begin opa[l] <= 64'd0; opb[l] <= 64'd0; resbuf[l] <= 64'd0; end
            st <= ST_RDN;
          end
        end

        // ---- operand reads, one (register, beat) at a time
        ST_RDN: begin
          if (ri > 3'd3) st <= ST_RUN;
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
              default: opb[int'(rb) * NL + i][63:32] <= rdv_data[i*32 +: 32];
            endcase
          end
          if (int'(rb) == NBEAT - 1) begin ri <= ri + 3'd1; st <= ST_RDN; end
          else begin rb <= rb + 1'b1; st <= ST_RDQ; end
        end

        // ---- lane groups
        ST_RUN: begin
          if (g >= NG) st <= ST_FIN;
          else if (act_mask == '0) g <= g + 1;
          else st <= ST_GO;
        end
        ST_GO: begin
          pend <= act_mask;
          st <= ST_WAIT;
        end
        ST_WAIT: begin
          logic [NSL-1:0] np;
          logic [4:0]     fo;
          np = pend & ~ln_done;
          fo = fl_acc;
          for (int j = 0; j < NSL; j++) begin
            if (ln_done[j]) begin
              resbuf[g * NSL + j] <= ln_res[j];
              fo = fo | ln_fl[j];
            end
          end
          fl_acc <= fo;
          pend <= np;
          if (np == '0) begin g <= g + 1; st <= ST_RUN; end
        end

        ST_FIN: begin
          if (rd_err_seen) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= 32'd0; st <= ST_IDLE;
          end else if (fp_trap) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_FPEXC; trap_info <= {27'd0, fl_acc};
            fflags_valid <= 1'b1; fflags_set <= fl_acc;
            st <= ST_IDLE;
          end else if (d_fault) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= {24'd0, d_info};
            st <= ST_IDLE;
          end else begin
            wi <= 3'd0; wb <= '0;
            st <= ST_WRQ;
          end
        end

        // ---- write-back (skips beats whose lane mask is empty)
        ST_WRQ: begin
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
