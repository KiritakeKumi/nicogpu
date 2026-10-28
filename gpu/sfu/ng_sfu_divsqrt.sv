// ng_sfu_divsqrt.sv -- iterative correctly rounded divide / square root and
// 32-bit integer divide for the SFU lane (M1b: pipelined prepare / finish).
//
//   op 0: res = a / b      (binary16/32/64 selected by fmt, all five rm, DAZ/FTZ)
//   op 1: res = sqrt(a)
//   op 2: integer div/divu/rem/remu (RISC-V M semantics, never traps)
// Bit-identical to SoftFloat f{16,32,64}_div / _sqrt with the RISC-V
// specialisation: canonical quiet NaN, NV/DZ/NX/UF/OF as softfloat.
//
// Sequence: capture -> unpack (2 stages, ng_sfu_unp) -> prepare (specials,
// operand/exponent setup, one register stage) -> 55 (fp) / 32 (int) radix-2
// digit-recurrence cycles -> round/pack (ng_sfu_rnd, 6 stages) -> FTZ / result.
// Special operands skip the iteration.  `done` is a one-cycle pulse; res/fl hold
// until the next start.  Every stage is a register-to-register path; latency is
// variable (the consumer waits for `done`).
module ng_sfu_divsqrt
  import ng_sfu_fp_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [1:0]  op,        // 0 div, 1 sqrt, 2 int
  input  logic [1:0]  iop,       // int: 0 div, 1 divu, 2 rem, 3 remu
  input  logic [1:0]  fmt,
  input  logic [2:0]  rm,
  input  logic        daz,
  input  logic        ftz,
  input  logic [63:0] a,
  input  logic [63:0] b,
  output logic        done,
  output logic [63:0] res,
  output logic [4:0]  fl,
  output logic        busy
);
  typedef enum logic [3:0] {S_IDLE, S_UPW, S_PRE, S_DIV, S_SQRT, S_INT, S_IF, S_FR0, S_FR1, S_FRW, S_FR2} st_t;
  st_t st;

  // ---------------------------------------------------------------- request registers + unpack
  logic [1:0]   fmt_q, op_q, iop_q;
  logic [2:0]   rm_q;
  logic         ftz_q;
  logic [63:0]  a_r, b_r;
  logic         go;
  assign go = (st == S_IDLE) && start;

  up_t  ua, ub;
  logic ua_ov, ub_ov;
  ng_sfu_unp u_ua (.clk(clk), .rst_n(rst_n), .iv(go), .x(a), .fmt(fmt), .daz(daz), .ov(ua_ov), .u(ua));
  ng_sfu_unp u_ub (.clk(clk), .rst_n(rst_n), .iv(go), .x(b), .fmt(fmt), .daz(daz), .ov(ub_ov), .u(ub));

  // ---------------------------------------------------------------- prepare (comb on unpacked operands)
  logic        sp_v;            // special result available now
  logic [63:0] sp_bits;
  logic [4:0]  sp_fl;
  logic signed [13:0] d_e;      // div result exponent
  logic [53:0] d_rem0;
  logic        d_sign;
  logic signed [13:0] s_e;
  logic [109:0] s_n0;
  // integer
  logic        i_signed, i_isrem, i_sp;
  logic [31:0] i_sp_val, i_ma, i_mb;

  always_comb begin
    sp_v = 1'b0; sp_bits = 64'd0; sp_fl = 5'd0;
    d_sign = ua.sign ^ ub.sign;
    d_e = 14'sd0; d_rem0 = 54'd0;
    s_e = 14'sd0; s_n0 = 110'd0;
    i_signed = ~iop_q[0];
    i_isrem  = iop_q[1];
    i_sp = 1'b0; i_sp_val = 32'd0;
    i_ma = (i_signed && a_r[31]) ? (32'd0 - a_r[31:0]) : a_r[31:0];
    i_mb = (i_signed && b_r[31]) ? (32'd0 - b_r[31:0]) : b_r[31:0];

    if (op_q == 2'd2) begin
      if (b_r[31:0] == 32'd0) begin
        i_sp = 1'b1; i_sp_val = i_isrem ? a_r[31:0] : 32'hffff_ffff;
      end else if (i_signed && a_r[31:0] == 32'h8000_0000 && b_r[31:0] == 32'hffff_ffff) begin
        i_sp = 1'b1; i_sp_val = i_isrem ? 32'd0 : a_r[31:0];
      end
      sp_v = i_sp; sp_bits = {32'd0, i_sp_val};
    end else if (op_q == 2'd0) begin
      if (ua.nan || ub.nan) begin
        sp_v = 1'b1; sp_bits = nan_bits(fmt_q); sp_fl[FL_NV] = ua.snan | ub.snan;
      end else if (ua.inf) begin
        sp_v = 1'b1;
        if (ub.inf) begin sp_bits = nan_bits(fmt_q); sp_fl[FL_NV] = 1'b1; end
        else sp_bits = inf_bits(d_sign, fmt_q);
      end else if (ub.inf) begin
        sp_v = 1'b1; sp_bits = zero_bits(d_sign, fmt_q);
      end else if (ub.zero) begin
        sp_v = 1'b1;
        if (ua.zero) begin sp_bits = nan_bits(fmt_q); sp_fl[FL_NV] = 1'b1; end
        else begin sp_bits = inf_bits(d_sign, fmt_q); sp_fl[FL_DZ] = 1'b1; end
      end else if (ua.zero) begin
        sp_v = 1'b1; sp_bits = zero_bits(d_sign, fmt_q);
      end
      if (ua.m < ub.m) begin
        d_rem0 = {ua.m, 1'b0};
        d_e    = {ua.e[12], ua.e} - {ub.e[12], ub.e} - 14'sd1;
      end else begin
        d_rem0 = {1'b0, ua.m};
        d_e    = {ua.e[12], ua.e} - {ub.e[12], ub.e};
      end
    end else begin
      if (ua.nan) begin
        sp_v = 1'b1; sp_bits = nan_bits(fmt_q); sp_fl[FL_NV] = ua.snan;
      end else if (ua.zero) begin
        sp_v = 1'b1; sp_bits = zero_bits(ua.sign, fmt_q);
      end else if (ua.sign) begin
        sp_v = 1'b1; sp_bits = nan_bits(fmt_q); sp_fl[FL_NV] = 1'b1;
      end else if (ua.inf) begin
        sp_v = 1'b1; sp_bits = inf_bits(1'b0, fmt_q);
      end
      if (ua.e[0]) begin
        s_n0 = {ua.m, 1'b0, 56'd0};
        s_e  = ($signed({ua.e[12], ua.e}) - 14'sd1) >>> 1;
      end else begin
        s_n0 = {1'b0, ua.m, 56'd0};
        s_e  = $signed({ua.e[12], ua.e}) >>> 1;
      end
    end
  end

  // ---------------------------------------------------------------- iteration state
  logic [5:0]   cnt;
  logic [53:0]  rem;            // div remainder (<2*den)
  logic [52:0]  den;
  logic [54:0]  quo;
  logic signed [13:0] e_q;
  logic         sgn_q;
  logic [109:0] nreg;
  logic [54:0]  root;
  logic [58:0]  srem;
  // integer
  logic [31:0]  iq, id;
  logic [32:0]  ir;
  logic         ires_neg_q, irrem_q, irneg_q;

  // restoring step (div)
  logic [53:0] d_trial_rem;
  logic        d_bit;
  always_comb begin
    d_bit = (rem >= {1'b0, den});
    d_trial_rem = d_bit ? (rem - {1'b0, den}) : rem;
  end
  // sqrt step
  logic [58:0] q_rem2;
  logic [56:0] q_trial;
  logic        q_bit;
  always_comb begin
    q_rem2  = {srem[56:0], nreg[109:108]};
    q_trial = {root, 2'b01};
    q_bit   = (q_rem2 >= {2'b00, q_trial});
  end
  // integer step
  logic [32:0] i_sh;
  logic        i_bit;
  always_comb begin
    i_sh  = {ir[31:0], iq[31]};
    i_bit = (i_sh >= {1'b0, id});
  end

  // ---------------------------------------------------------------- round / pack
  logic [54:0] rn_m;
  logic        rn_st;
  logic        rn_iv, rn_ov;
  rnd_t        rn_r;
  assign rn_iv = (st == S_FR1);
  ng_sfu_rnd u_rnd (.clk(clk), .rst_n(rst_n), .iv(rn_iv), .sign_i(sgn_q), .e_i(e_q), .m_i(rn_m), .st_i(rn_st),
                    .fmt_i(fmt_q), .rm_i(rm_q), .ov(rn_ov), .r(rn_r));

  rnd_t        fin_r;
  logic [63:0] int_res;
  logic [31:0] qv, rv;
  always_comb begin
    fin_r = rn_r;
    if (ftz_q) fin_r = ftz_flush(rn_r, fmt_q);
    qv = ires_neg_q ? (32'd0 - iq) : iq;
    rv = irneg_q ? (32'd0 - ir[31:0]) : ir[31:0];
    int_res = {32'd0, irrem_q ? rv : qv};
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; done <= 1'b0;
    end else begin
      done <= 1'b0;
      case (st)
        S_IDLE: if (start) begin
          fmt_q <= fmt; rm_q <= rm; ftz_q <= ftz; op_q <= op; iop_q <= iop;
          a_r <= a; b_r <= b;
          st <= S_UPW;
        end
        S_UPW: if (ua_ov && ub_ov) st <= S_PRE;
        S_PRE: begin
          if (sp_v) begin
            res  <= sp_bits;
            fl   <= sp_fl;
            done <= 1'b1;
            st   <= S_IDLE;
          end else begin
            cnt <= 6'd0;
            if (op_q == 2'd0) begin
              rem <= d_rem0; den <= ub.m; quo <= 55'd0; e_q <= d_e; sgn_q <= d_sign;
              st <= S_DIV;
            end else if (op_q == 2'd1) begin
              nreg <= s_n0; root <= 55'd0; srem <= 59'd0; e_q <= s_e; sgn_q <= 1'b0;
              st <= S_SQRT;
            end else begin
              iq <= i_ma; id <= i_mb; ir <= 33'd0;
              ires_neg_q <= i_signed && (a_r[31] ^ b_r[31]);
              irneg_q    <= i_signed && a_r[31];
              irrem_q    <= i_isrem;
              st <= S_INT;
            end
          end
        end
        S_DIV: begin
          quo <= {quo[53:0], d_bit};
          rem <= {d_trial_rem[52:0], 1'b0};
          cnt <= cnt + 6'd1;
          if (cnt == 6'd54) st <= S_FR0;
        end
        S_SQRT: begin
          root <= {root[53:0], q_bit};
          srem <= q_bit ? (q_rem2 - {2'b00, q_trial}) : q_rem2;
          nreg <= {nreg[107:0], 2'b00};
          cnt  <= cnt + 6'd1;
          if (cnt == 6'd54) st <= S_FR0;
        end
        S_INT: begin
          ir <= i_bit ? (i_sh - {1'b0, id}) : i_sh;
          iq <= {iq[30:0], i_bit};
          cnt <= cnt + 6'd1;
          if (cnt == 6'd31) st <= S_IF;
        end
        S_IF: begin
          res <= int_res;
          fl  <= 5'd0;
          done <= 1'b1;
          st <= S_IDLE;
        end
        S_FR0: begin
          rn_m  <= (op_q == 2'd0) ? quo : root;
          rn_st <= (op_q == 2'd0) ? (rem != 54'd0) : (srem != 59'd0);
          st <= S_FR1;
        end
        S_FR1: st <= S_FRW;
        S_FRW: if (rn_ov) st <= S_FR2;
        S_FR2: begin
          res  <= fin_r.bits;
          fl   <= {2'b00, fin_r.of, fin_r.uf, fin_r.nx};
          done <= 1'b1;
          st   <= S_IDLE;
        end
        default: st <= S_IDLE;
      endcase
    end
  end

  assign busy = (st != S_IDLE);
endmodule
