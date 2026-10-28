// ng_sfu_lane.sv -- one SFU lane: sequences the A-tier transcendental kernels,
// correctly rounded divide / sqrt and integer divide (M1b: pipelined).
//
// One operation at a time; `done` pulses once, res/fl valid with it.  All
// arithmetic is built from bit-exact blocks that mirror SoftFloat:
//   ng_sfu_fma      binary64 fused multiply-add (RNE), pipelined
//   ng_sfu_divsqrt  binary16/32/64 divide / sqrt (all rm) + 32-bit integer divide
//   ng_sfu_unp / ng_sfu_rnd / ng_sfu_split   pipelined unpack, round-pack, rne split
//   ng_sfu_fp_pkg   remaining pure functions / coefficient tables
// The kernels  are executed
// step for step: exp2 = split + 14-term Horner + exact 2^n scale; log2 =
// (m-1)/(m+1) + 12-term Horner + final FMA; sin/cos = exact reduction + 9/10
// term Horner; rsq = RNE(1/RNE(sqrt(x))) in binary64.  Results are rounded
// once to the destination format with the instruction's rm.
//
// Every state is a register-to-register step of at most one primitive (unpack
// stage, compare, small add, table lookup); the deep arithmetic lives in the
// pipelined blocks and the lane waits for their valid pulses (the SFU interface
// has a done handshake; there is no fixed-latency assumption anywhere).
//
// op:  0 rcp  1 sqrt  2 fdiv  3 rsq  4 exp2  5 log2  6 sin  7 cos  8 int div (iop)

// ======================================================================== rne split
// x (binary64, finite) -> n = rne(x) and r = x - n (exact), bit-identical to
// ng_sfu_fp_pkg::rne_split.  r = +0 when x == n; |x| >= 2^52 or zero/NaN/Inf: r = 0, n = 0;
// |x| < 0.5 (exponent field < 1022): r = x, n = 0.  5 stages; r / n hold until the next iv.
module ng_sfu_split
  import ng_sfu_fp_pkg::*;
(
  input  logic                clk,
  input  logic                rst_n,
  input  logic                iv,
  input  logic [63:0]         x,
  output logic                ov,
  output logic [63:0]         r,
  output logic signed [11:0]  n
);
  logic [4:0] vch;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin vch <= 5'd0; ov <= 1'b0; end
    else begin vch <= {vch[3:0], iv}; ov <= vch[4]; end
  end

  // ---- A: classify, shift amount
  logic        a_sign, a_none, a_pass;
  logic [5:0]  a_sh;
  logic [53:0] a_mm;
  logic [63:0] a_x;
  always_ff @(posedge clk) begin
    if (iv) begin
      a_x    <= x;
      a_sign <= x[63];
      a_none <= ((x[62:52] == 11'd0) && (x[51:0] == 52'd0)) || (x[62:52] >= 11'd1075);
      a_pass <= !(((x[62:52] == 11'd0) && (x[51:0] == 52'd0)) || (x[62:52] >= 11'd1075)) && (x[62:52] < 11'd1022);
      a_sh   <= 6'(11'd1075 - x[62:52]);
      a_mm   <= {2'b01, x[51:0]};
    end
  end

  // ---- B: integer part, fraction, half, unit
  logic        b_sign, b_none, b_pass;
  logic [5:0]  b_sh;
  logic [63:0] b_x;
  logic [53:0] b_ip, b_frac, b_half, b_one;
  always_ff @(posedge clk) begin
    b_sign <= a_sign; b_none <= a_none; b_pass <= a_pass; b_sh <= a_sh; b_x <= a_x;
    b_ip   <= a_mm >> a_sh;
    b_frac <= a_mm & ((54'd1 << a_sh) - 54'd1);
    b_half <= 54'd1 << (a_sh - 6'd1);
    b_one  <= 54'd1 << a_sh;
  end

  // ---- C: round-to-nearest-even direction
  logic        c_sign, c_none, c_pass, c_up;
  logic [5:0]  c_sh;
  logic [63:0] c_x;
  logic [53:0] c_ip, c_frac, c_one;
  always_ff @(posedge clk) begin
    c_sign <= b_sign; c_none <= b_none; c_pass <= b_pass; c_sh <= b_sh; c_x <= b_x;
    c_ip <= b_ip; c_frac <= b_frac; c_one <= b_one;
    c_up <= (b_frac > b_half) || (b_frac == b_half && b_ip[0]);
  end

  // ---- D: rounded integer, remainder magnitude
  logic        d_sign, d_none, d_pass, d_up;
  logic [5:0]  d_sh;
  logic [63:0] d_x;
  logic [53:0] d_nmag, d_vmag;
  always_ff @(posedge clk) begin
    d_sign <= c_sign; d_none <= c_none; d_pass <= c_pass; d_sh <= c_sh; d_x <= c_x; d_up <= c_up;
    d_nmag <= c_ip + 54'(c_up);
    d_vmag <= c_up ? (c_one - c_frac) : c_frac;
  end

  // ---- E: leading zeros of the remainder, signed integer
  logic        e_sign, e_none, e_pass, e_up, e_vz;
  logic [5:0]  e_sh;
  logic [6:0]  e_lz;
  logic [63:0] e_x;
  logic [53:0] e_vmag;
  logic signed [11:0] e_n;
  always_ff @(posedge clk) begin
    e_sign <= d_sign; e_none <= d_none; e_pass <= d_pass; e_sh <= d_sh; e_x <= d_x; e_up <= d_up;
    e_vmag <= d_vmag;
    e_vz   <= (d_vmag == 54'd0);
    e_lz   <= 7'(clz128({d_vmag, 74'd0}));
    e_n    <= d_sign ? (12'd0 - {1'b0, d_nmag[10:0]}) : {1'b0, d_nmag[10:0]};
  end

  // ---- F: normalise, exponent, assemble
  logic [53:0] m54_c;
  logic [10:0] er_c;
  always_comb begin
    m54_c = e_vmag << (e_lz - 7'd1);
    er_c  = 11'(1023 + (53 - int'(e_lz)) - int'(e_sh));
  end
  always_ff @(posedge clk) begin
    if (vch[4]) begin
      if (e_none)      begin r <= 64'd0; n <= 12'sd0; end
      else if (e_pass) begin r <= e_x;   n <= 12'sd0; end
      else begin
        r <= e_vz ? 64'd0 : {e_sign ^ e_up, er_c, m54_c[51:0]};
        n <= e_n;
      end
    end
  end
endmodule

// ======================================================================== lane
module ng_sfu_lane
  import ng_sfu_fp_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [3:0]  op,
  input  logic [1:0]  iop,
  input  logic [1:0]  fmt,
  input  logic [2:0]  rm,
  input  logic        daz,
  input  logic        ftz,
  input  logic [63:0] a,
  input  logic [63:0] b,
  output logic        done,
  output logic [63:0] res,
  output logic [4:0]  fl
);
  localparam logic [3:0] SOP_RCP = 4'd0, SOP_SQRT = 4'd1, SOP_FDIV = 4'd2, SOP_RSQ = 4'd3,
                         SOP_EXP2 = 4'd4, SOP_LOG2 = 4'd5, SOP_SIN = 4'd6, SOP_COS = 4'd7, SOP_IDIV = 4'd8;
  localparam logic [63:0] F64_NONE  = 64'hbff0_0000_0000_0000;   // -1.0
  localparam logic [63:0] F64_256   = 64'h4070_0000_0000_0000;   // 256.0
  localparam logic [51:0] SQRT2_FR  = 52'h6a09e667f3bcd;         // frac of 0x3ff6a09e667f3bcd

  typedef enum logic [4:0] {S_IDLE, S_UP0, S_UPW, S_UP1, S_SP0, S_SP1, S_SPW, S_UP3, S_DEC,
                            S_EI, S_ENW, S_FI, S_FW, S_HI,
                            S_FNP, S_FN0, S_FNW, S_FN1, S_FN1B, S_FNR, S_FN2, S_FN3} st_t;
  typedef enum logic [3:0] {A_DIRECT, A_RSQ1, A_RSQ2, A_HOR, A_LG_NUM, A_LG_DEN, A_LG_S, A_LG_S2,
                            A_LG_FIN, A_SC_T2, A_SC_FIN} act_t;
  st_t  st;
  act_t act;

  // latched request
  logic [3:0]  op_q;
  logic [1:0]  iop_q, fmt_q;
  logic [2:0]  rm_q;
  logic        daz_q, ftz_q;
  logic [63:0] a_q, b_q;

  // work registers
  logic [63:0]        xv, pv, tv, sv, nv, vres;
  logic [63:0]        mreg;
  logic signed [11:0] nint;
  logic signed [13:0] eint;
  logic [3:0]         hc;
  logic [1:0]         hsel;
  logic               exact_r, ksg, kodd;
  logic [4:0]         kfl, efl;

  // fma / engine operand registers
  logic [63:0] fa, fb, fc;
  typedef struct packed {
    logic [1:0]  op;
    logic [1:0]  fmt;
    logic [2:0]  rm;
    logic        daz;
    logic        ftz;
    logic [63:0] a;
    logic [63:0] b;
  } ecfg_t;
  ecfg_t ecfg;
  function automatic ecfg_t mk_eng(input logic [1:0] o, input logic [1:0] f, input logic [2:0] r_,
                                   input logic dz, input logic fz, input logic [63:0] x, input logic [63:0] y);
    ecfg_t c;
    c.op = o; c.fmt = f; c.rm = r_; c.daz = dz; c.ftz = fz; c.a = x; c.b = y;
    return c;
  endfunction

  logic        fma_go, eng_go, fma_done, eng_done;
  logic [63:0] fma_res, eng_res;
  logic [4:0]  eng_fl;
  logic        eng_busy;

  assign fma_go = (st == S_FI);
  assign eng_go = (st == S_EI);

  ng_sfu_fma u_fma (.clk(clk), .rst_n(rst_n), .start(fma_go), .a(fa), .b(fb), .c(fc),
                    .done(fma_done), .res(fma_res));
  ng_sfu_divsqrt u_eng (.clk(clk), .rst_n(rst_n), .start(eng_go), .op(ecfg.op), .iop(iop_q), .fmt(ecfg.fmt),
                        .rm(ecfg.rm), .daz(ecfg.daz), .ftz(ecfg.ftz), .a(ecfg.a), .b(ecfg.b),
                        .done(eng_done), .res(eng_res), .fl(eng_fl), .busy(eng_busy));

  // ---------------------------------------------------------------- unpack (input and result)
  logic [63:0] ux;
  logic [1:0]  ufmt;
  logic        udaz;
  logic        unp_iv, unp_ov;
  up_t         up;
  assign unp_iv = (st == S_UP0) || (st == S_FN0);
  ng_sfu_unp u_unp (.clk(clk), .rst_n(rst_n), .iv(unp_iv), .x(ux), .fmt(ufmt), .daz(udaz), .ov(unp_ov), .u(up));

  // ---------------------------------------------------------------- derived input values (S_UP1)
  logic [63:0]  xf_r, xc_r, mb_r;
  logic         gt_r, mbone_r;
  logic         is_cos_r;
  logic [63:0]  xf_c, xc_c, mb_c;
  logic         big_c, gt_c;
  always_comb begin
    big_c = ($signed(up.e) > 13'sd8) || ($signed(up.e) == 13'sd8 && up.m[51:0] != 52'd0);
    xf_c  = up_to_f64(up);
    xc_c  = xf_c;
    if (!up.nan && !up.inf && big_c) xc_c = {up.sign, F64_256[62:0]};
    gt_c  = (up.m[51:0] > SQRT2_FR);
    mb_c  = {1'b0, gt_c ? 11'd1022 : 11'd1023, up.m[51:0]};
  end

  // ---------------------------------------------------------------- rne split (exp2 / sin / cos reduction)
  logic [63:0] sx;
  logic        sp_iv, sp_ov, sp_pass2;
  logic [63:0] sp_r;
  logic signed [11:0] sp_n;
  logic [63:0] spr;
  logic signed [11:0] spn;
  logic        rzero_r;
  logic [1:0]  k_r;
  logic [63:0] yz_c;
  assign sp_iv = (st == S_SP1);
  ng_sfu_split u_split (.clk(clk), .rst_n(rst_n), .iv(sp_iv), .x(sx), .ov(sp_ov), .r(sp_r), .n(sp_n));
  always_comb begin
    yz_c = (sp_r[62:52] == 11'd0) ? sp_r : {sp_r[63], sp_r[62:52] + 11'd2, sp_r[51:0]};
  end

  // ---------------------------------------------------------------- result rounding to the destination format
  logic        fr_spec;
  logic        fr_sign;
  logic signed [13:0] fr_e;
  logic [54:0] fr_m;
  logic [63:0] fr_sbits;
  logic        rn_iv, rn_ov;
  rnd_t        rn_r;
  assign rn_iv = (st == S_FN1B) && !fr_spec;
  ng_sfu_rnd u_rnd (.clk(clk), .rst_n(rst_n), .iv(rn_iv), .sign_i(fr_sign), .e_i(fr_e), .m_i(fr_m), .st_i(1'b0),
                    .fmt_i(fmt_q), .rm_i(rm_q), .ov(rn_ov), .r(rn_r));

  rnd_t        fin2_r, fin2_c, fin3_c;
  logic        fin_ex_sub;
  logic [4:0]  fin_fl;
  always_comb begin
    fin2_c = '0;
    if (fr_spec) fin2_c.bits = fr_sbits;
    else         fin2_c = rn_r;
    case (fmt_q)
      F16:     fin_ex_sub = (fin2_c.bits[14:10] == 5'd0);
      F32:     fin_ex_sub = (fin2_c.bits[30:23] == 8'd0);
      default: fin_ex_sub = (fin2_c.bits[62:52] == 11'd0);
    endcase
    if (!exact_r) begin
      fin2_c.nx = 1'b1;
      if (fin_ex_sub) fin2_c.uf = 1'b1;
    end
    fin3_c = fin2_r;
    if (ftz_q) fin3_c = ftz_flush(fin2_r, fmt_q);
    fin_fl = {kfl[FL_NV], kfl[FL_DZ], fin3_c.of, fin3_c.uf, fin3_c.nx};
  end

  // --------------------------------------------------------------
  logic [63:0] scaled_c;
  always_comb begin
    scaled_c = {fma_res[63], fma_res[62:52] + 11'(nint), fma_res[51:0]};
  end

  // exponent as binary64 for the final log2 FMA; eint is stable for hundreds of
  // cycles before it is consumed, this is simply one more pipeline register.
  logic [63:0] eflt;
  always_ff @(posedge clk) eflt <= itof(int'(eint));

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; done <= 1'b0;
    end else begin
      done <= 1'b0;
      case (st)
        S_IDLE: if (start) begin
          op_q <= op; iop_q <= iop; fmt_q <= fmt; rm_q <= rm; daz_q <= daz; ftz_q <= ftz;
          a_q <= a; b_q <= b;
          ux <= a; ufmt <= fmt; udaz <= daz;
          kfl <= 5'd0; efl <= 5'd0; exact_r <= 1'b1;
          st <= S_UP0;
        end

        // ---- unpack the operand and derive the kernel inputs
        S_UP0: st <= S_UPW;
        S_UPW: if (unp_ov) st <= S_UP1;
        S_UP1: begin
          xf_r <= xf_c; xc_r <= xc_c; gt_r <= gt_c; mb_r <= mb_c;
          mbone_r  <= !gt_c && (up.m[51:0] == 52'd0);
          is_cos_r <= (op_q == SOP_COS);
          if (op_q == SOP_EXP2 || op_q == SOP_SIN || op_q == SOP_COS) st <= S_SP0;
          else st <= S_DEC;
        end

        // ---- range reduction (one split for exp2, two for sin / cos)
        S_SP0: begin
          sx <= (op_q == SOP_EXP2) ? xc_r : xf_r;
          sp_pass2 <= 1'b0;
          st <= S_SP1;
        end
        S_SP1: st <= S_SPW;
        S_SPW: if (sp_ov) begin
          if (op_q != SOP_EXP2 && !sp_pass2) begin
            sx <= yz_c; sp_pass2 <= 1'b1; st <= S_SP1;
          end else begin
            spr <= sp_r; spn <= sp_n; st <= S_UP3;
          end
        end
        S_UP3: begin
          rzero_r <= (spr[62:0] == 63'd0);
          k_r     <= spn[1:0] + (is_cos_r ? 2'd1 : 2'd0);
          st <= S_DEC;
        end

        // ---- dispatch (operand classes known)
        S_DEC: begin
          case (op_q)
            SOP_RCP:  begin ecfg <= mk_eng(2'd0, fmt_q, rm_q, daz_q, ftz_q, one_bits(fmt_q), a_q); act <= A_DIRECT; st <= S_EI; end
            SOP_SQRT: begin ecfg <= mk_eng(2'd1, fmt_q, rm_q, daz_q, ftz_q, a_q, 64'd0);           act <= A_DIRECT; st <= S_EI; end
            SOP_FDIV: begin ecfg <= mk_eng(2'd0, fmt_q, rm_q, daz_q, ftz_q, a_q, b_q);             act <= A_DIRECT; st <= S_EI; end
            SOP_IDIV: begin ecfg <= mk_eng(2'd2, fmt_q, rm_q, 1'b0, 1'b0, a_q, b_q);               act <= A_DIRECT; st <= S_EI; end
            SOP_RSQ: begin
              if (fmt_q == F64) begin
                ecfg <= mk_eng(2'd1, F64, RM_RNE, 1'b0, 1'b0, a_q, 64'd0);
              end else begin
                kfl[FL_NV] <= up.snan;
                ecfg <= mk_eng(2'd1, F64, RM_RNE, 1'b0, 1'b0, xf_r, 64'd0);
              end
              act <= A_RSQ1; st <= S_EI;
            end
            SOP_EXP2: begin
              kfl[FL_NV] <= up.snan;
              if (up.nan) begin
                vres <= F64_QNAN; exact_r <= 1'b1; st <= S_FNP;
              end else if (up.inf) begin
                vres <= up.sign ? 64'd0 : F64_INF; exact_r <= 1'b1; st <= S_FNP;
              end else begin
                nint <= spn; xv <= spr;
                exact_r <= rzero_r;
                pv <= coef(2'd0, 13); hc <= 4'd12; hsel <= 2'd0;
                st <= S_HI; act <= A_HOR;
              end
            end
            SOP_LOG2: begin
              kfl[FL_NV] <= up.snan;
              if (up.nan) begin
                vres <= F64_QNAN; exact_r <= 1'b1; st <= S_FNP;
              end else if (up.zero) begin
                vres <= {1'b1, 11'h7ff, 52'd0}; kfl[FL_DZ] <= 1'b1; exact_r <= 1'b1; st <= S_FNP;
              end else if (up.sign) begin
                vres <= F64_QNAN; kfl[FL_NV] <= 1'b1; exact_r <= 1'b1; st <= S_FNP;
              end else if (up.inf) begin
                vres <= F64_INF; exact_r <= 1'b1; st <= S_FNP;
              end else begin
                mreg <= mb_r;
                eint <= {up.e[12], up.e} + (gt_r ? 14'sd1 : 14'sd0);
                exact_r <= mbone_r;
                fa <= mb_r; fb <= F64_ONE; fc <= F64_NONE;
                act <= A_LG_NUM; st <= S_FI;
              end
            end
            default: begin  // SIN / COS
              kfl[FL_NV] <= up.snan;
              if (up.nan) begin
                vres <= F64_QNAN; exact_r <= 1'b1; st <= S_FNP;
              end else if (up.inf) begin
                vres <= F64_QNAN; kfl[FL_NV] <= 1'b1; exact_r <= 1'b1; st <= S_FNP;
              end else if (rzero_r) begin
                // t == 0: exact result
                if (k_r[0]) vres <= (k_r == 2'd1) ? F64_ONE : (F64_ONE | F64_NZERO);
                else        vres <= is_cos_r ? 64'd0 : {up.sign, 63'd0};
                exact_r <= 1'b1; st <= S_FNP;
              end else begin
                exact_r <= 1'b0;
                tv <= spr; ksg <= k_r[1]; kodd <= k_r[0];
                fa <= spr; fb <= spr; fc <= F64_NZERO;
                act <= A_SC_T2; st <= S_FI;
              end
            end
          endcase
        end

        S_EI: st <= S_ENW;

        S_ENW: if (eng_done) begin
          case (act)
            A_DIRECT: begin
              res <= eng_res; fl <= eng_fl; done <= 1'b1; st <= S_IDLE;
            end
            A_RSQ1: begin
              efl <= eng_fl;
              ecfg <= mk_eng(2'd0, F64, RM_RNE, 1'b0, 1'b0, F64_ONE, eng_res);
              act <= A_RSQ2; st <= S_EI;
            end
            A_RSQ2: begin
              if (fmt_q == F64) begin
                res <= eng_res; fl <= efl | eng_fl; done <= 1'b1; st <= S_IDLE;
              end else begin
                vres <= eng_res;
                kfl[FL_DZ] <= kfl[FL_DZ] | efl[FL_DZ] | eng_fl[FL_DZ];
                kfl[FL_NV] <= kfl[FL_NV] | efl[FL_NV] | eng_fl[FL_NV];
                exact_r <= ~(efl[FL_NX] | eng_fl[FL_NX]);
                st <= S_FNP;
              end
            end
            A_LG_S: begin
              sv <= eng_res;
              fa <= eng_res; fb <= eng_res; fc <= F64_NZERO;
              act <= A_LG_S2; st <= S_FI;
            end
            default: st <= S_IDLE;
          endcase
        end

        S_FI: st <= S_FW;

        S_FW: if (fma_done) begin
          case (act)
            A_HOR: begin
              pv <= fma_res;
              if (hc == 4'd0) begin
                case (op_q)
                  SOP_EXP2: begin vres <= scaled_c; st <= S_FNP; end
                  SOP_LOG2: begin fa <= sv; fb <= fma_res; fc <= eflt; act <= A_LG_FIN; st <= S_FI; end
                  default: begin  // sin / cos: odd quadrant = cos polynomial, even = t * sin polynomial
                    if (kodd) begin vres <= {fma_res[63] ^ ksg, fma_res[62:0]}; st <= S_FNP; end
                    else begin fa <= tv; fb <= fma_res; fc <= F64_NZERO; act <= A_SC_FIN; st <= S_FI; end
                  end
                endcase
              end else begin
                hc <= hc - 4'd1;
                st <= S_HI;
              end
            end
            A_LG_NUM: begin
              nv <= fma_res;
              fa <= mreg; fb <= F64_ONE; fc <= F64_ONE;
              act <= A_LG_DEN; st <= S_FI;
            end
            A_LG_DEN: begin
              ecfg <= mk_eng(2'd0, F64, RM_RNE, 1'b0, 1'b0, nv, fma_res);
              act <= A_LG_S; st <= S_EI;
            end
            A_LG_S2: begin
              xv <= fma_res;
              pv <= coef(2'd1, 11); hc <= 4'd10; hsel <= 2'd1;
              st <= S_HI; act <= A_HOR;
            end
            A_LG_FIN: begin
              vres <= fma_res; st <= S_FNP;
            end
            A_SC_T2: begin
              xv <= fma_res;
              hsel <= kodd ? 2'd3 : 2'd2;
              pv <= kodd ? coef(2'd3, 9) : coef(2'd2, 8);
              hc <= kodd ? 4'd8 : 4'd7;
              st <= S_HI; act <= A_HOR;
            end
            A_SC_FIN: begin
              vres <= {fma_res[63] ^ ksg, fma_res[62:0]}; st <= S_FNP;
            end
            default: st <= S_IDLE;
          endcase
        end

        S_HI: begin
          fa <= pv; fb <= xv; fc <= coef(hsel, int'(hc));
          st <= S_FI;
        end

        // ---- final rounding of the binary64 kernel result to the destination format
        S_FNP: begin
          ux <= vres; ufmt <= F64; udaz <= 1'b0;
          st <= S_FN0;
        end
        S_FN0: st <= S_FNW;
        S_FNW: if (unp_ov) st <= S_FN1;
        S_FN1: begin
          fr_sign <= up.sign;
          fr_e    <= {up.e[12], up.e};
          fr_m    <= {up.m, 2'b00};
          fr_spec <= up.nan | up.inf | up.zero;
          fr_sbits <= up.nan ? nan_bits(fmt_q) : (up.inf ? inf_bits(up.sign, fmt_q) : zero_bits(up.sign, fmt_q));
          st <= S_FN1B;
        end
        S_FN1B: st <= fr_spec ? S_FN2 : S_FNR;
        S_FNR: if (rn_ov) st <= S_FN2;
        S_FN2: begin
          fin2_r <= fin2_c;
          st <= S_FN3;
        end
        S_FN3: begin
          res <= fin3_c.bits; fl <= fin_fl; done <= 1'b1; st <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end
endmodule
