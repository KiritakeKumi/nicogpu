// ng_sfu_fma.sv -- pipelined SFU arithmetic building blocks (M1b timing rework).
//
// Contains
//   ng_sfu_unp   2-stage unpack (field select + clz | normalise), the pipelined
//                form of ng_sfu_fp_pkg::unpack
//   ng_sfu_rnd   6-stage round + pack, the pipelined form of ng_sfu_fp_pkg::rnd_pack
//                (FIXED=1: binary64 / RNE only, flags unused -- used by the FMA)
//   ng_sfu_fma   binary64 fused multiply-add, RNE only
//
// Every block is a plain register pipeline with a valid chain (no stall, no
// multicycle path); `done` / `ov` is a one-cycle pulse and the result registers
// hold until the next start.  Only one operation is in flight per instance (the
// SFU lane sequences them), so the datapath stages are free-running registers
// fed by held operands.  Latency is fixed per block but nothing outside relies
// on it: consumers wait for the valid pulse.
//
// ng_sfu_fma: res = RNE(a * b + c), bit-identical to SoftFloat f64_mulAdd in RNE
// for finite operands (zeros and subnormals included; NaN / Inf are never fed in).
// The 53x53 multiply is decomposed into 26x17 partial products (DSP48E2) with
// registered inputs / product registers and a registered adder tree.

// ======================================================================== unpack
module ng_sfu_unp
  import ng_sfu_fp_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        iv,
  input  logic [63:0] x,
  input  logic [1:0]  fmt,
  input  logic        daz,
  output logic        ov,
  output up_t         u
);
  logic        v1;
  logic        sgn_c;
  logic [10:0] ef_c, emax_c;
  logic [51:0] fr_c;
  always_comb begin
    case (fmt)
      F16: begin sgn_c = x[15]; ef_c = {6'd0, x[14:10]}; fr_c = {x[9:0], 42'd0};  emax_c = 11'd31;   end
      F32: begin sgn_c = x[31]; ef_c = {3'd0, x[30:23]}; fr_c = {x[22:0], 29'd0}; emax_c = 11'd255;  end
      default: begin sgn_c = x[63]; ef_c = x[62:52];     fr_c = x[51:0];          emax_c = 11'd2047; end
    endcase
  end

  // stage 1: classify + leading-zero count of the fraction
  logic        s_sign, s_nan, s_snan, s_inf, s_zero, s_sub;
  logic [5:0]  s_lz;
  logic [51:0] s_fr;
  logic [10:0] s_ef;
  logic [1:0]  s_fmt;
  always_ff @(posedge clk) begin
    if (iv) begin
      s_sign <= sgn_c;
      s_fr   <= fr_c;
      s_ef   <= ef_c;
      s_fmt  <= fmt;
      s_nan  <= (ef_c == emax_c) && (fr_c != 52'd0);
      s_snan <= (ef_c == emax_c) && (fr_c != 52'd0) && ~fr_c[51];
      s_inf  <= (ef_c == emax_c) && (fr_c == 52'd0);
      s_zero <= (ef_c == 11'd0) && ((fr_c == 52'd0) || daz);
      s_sub  <= (ef_c == 11'd0) && (fr_c != 52'd0) && !daz;
      s_lz   <= 6'(clz128({fr_c, 76'd0}));
    end
  end

  // stage 2: normalise
  int          bias_c;
  logic [52:0] sh_c;
  up_t         u_c;
  always_comb begin
    case (s_fmt)
      F16:     bias_c = 15;
      F32:     bias_c = 127;
      default: bias_c = 1023;
    endcase
    sh_c = {s_fr, 1'b0} << s_lz;
    u_c = '0;
    u_c.sign = s_sign;
    u_c.nan  = s_nan;
    u_c.snan = s_snan;
    u_c.inf  = s_inf;
    u_c.zero = s_zero;
    if (s_sub) begin
      u_c.m = sh_c;
      u_c.e = 13'(-bias_c - int'(s_lz));
    end else if (!s_nan && !s_inf && !s_zero) begin
      u_c.m = {1'b1, s_fr};
      u_c.e = 13'(int'(s_ef) - bias_c);
    end
  end
  always_ff @(posedge clk) begin
    if (v1) u <= u_c;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin v1 <= 1'b0; ov <= 1'b0; end
    else begin v1 <= iv; ov <= v1; end
  end
endmodule

// ======================================================================== round + pack
// Inputs are sampled when iv is high.  value = (M / 2^54) * 2^e, M[54] = 1, st = sticky.
// Result ov cycles later (6 stages after the sampling edge); r holds until the next iv.
module ng_sfu_rnd
  import ng_sfu_fp_pkg::*;
#(
  parameter bit FIXED = 1'b0       // 1: fmt = binary64, rm = RNE
) (
  input  logic                clk,
  input  logic                rst_n,
  input  logic                iv,
  input  logic                sign_i,
  input  logic signed [13:0]  e_i,
  input  logic [54:0]         m_i,
  input  logic                st_i,
  input  logic [1:0]          fmt_i,
  input  logic [2:0]          rm_i,
  output logic                ov,
  output rnd_t                r
);
  logic [1:0] fm;
  logic [2:0] rmi;
  assign fm  = FIXED ? F64 : fmt_i;
  assign rmi = FIXED ? RM_RNE : rm_i;

  logic [5:0] vch;           // valid chain: stage k valid -> vch[k-1]
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin vch <= 6'd0; ov <= 1'b0; end
    else begin vch <= {vch[4:0], iv}; ov <= vch[5]; end
  end

  // ---- stage 1 (sample): format constants, tininess, shift amount, unshifted guard info
  int           p1, bias1, emin1, ei1, rpos1, sh1;
  logic         tiny1, gu1, su1, mrp1, ones1, em11, ovfe1;
  logic [54:0]  lowmask1, keptu1;
  logic [63:0]  pmask1;
  logic [11:0]  expf1;
  always_comb begin
    case (fm)
      F16:     begin p1 = 11; bias1 = 15;   end
      F32:     begin p1 = 24; bias1 = 127;  end
      default: begin p1 = 53; bias1 = 1023; end
    endcase
    emin1 = 1 - bias1;
    ei1   = int'(e_i);
    rpos1 = 55 - p1;
    tiny1 = (ei1 < emin1);
    sh1   = tiny1 ? (emin1 - ei1) : 0;
    if (sh1 > 55) sh1 = 55;
    lowmask1 = (55'd1 << (rpos1 - 1)) - 55'd1;
    gu1   = m_i[rpos1 - 1];
    su1   = (|(m_i & lowmask1)) | st_i;
    mrp1  = m_i[rpos1];
    keptu1 = m_i >> rpos1;
    pmask1 = (64'd1 << p1) - 64'd1;
    ones1 = ((64'(keptu1) & pmask1) == pmask1);
    em11  = (ei1 == emin1 - 1);
    ovfe1 = (ei1 > bias1);
    expf1 = tiny1 ? 12'd0 : 12'(ei1 + bias1);
  end
  logic         s1_sign, s1_tiny, s1_gu, s1_su, s1_mrp, s1_ones, s1_em1, s1_ovfe, s1_st;
  logic [5:0]   s1_sh;
  logic [54:0]  s1_m;
  logic [11:0]  s1_expf;
  logic [1:0]   s1_fm;
  logic [2:0]   s1_rm;
  always_ff @(posedge clk) begin
    if (iv) begin
      s1_sign <= sign_i; s1_tiny <= tiny1; s1_gu <= gu1; s1_su <= su1; s1_mrp <= mrp1; s1_ones <= ones1;
      s1_em1 <= em11; s1_ovfe <= ovfe1; s1_st <= st_i; s1_sh <= 6'(sh1); s1_m <= m_i; s1_expf <= expf1;
      s1_fm <= fm; s1_rm <= rmi;
    end
  end

  // ---- stage 2: shift right by sh; masked low part; unshifted increment / tininess
  logic         incu2, tafter2;
  logic [55:0]  tmask2;
  logic [54:0]  msh2, mlow2;
  always_comb begin
    tmask2 = (56'd1 << s1_sh) - 56'd1;
    msh2   = s1_m >> s1_sh;
    mlow2  = s1_m & tmask2[54:0];
    case (s1_rm)
      RM_RNE:  incu2 = s1_gu & (s1_su | s1_mrp);
      RM_RMM:  incu2 = s1_gu;
      RM_RDN:  incu2 = s1_sign & (s1_gu | s1_su);
      RM_RUP:  incu2 = ~s1_sign & (s1_gu | s1_su);
      default: incu2 = 1'b0;
    endcase
    tafter2 = s1_tiny && !(s1_em1 && (incu2 & s1_ones));
  end
  logic         s2_sign, s2_ovfe, s2_tafter, s2_st;
  logic [54:0]  s2_msh, s2_mlow;
  logic [11:0]  s2_expf;
  logic [1:0]   s2_fm;
  logic [2:0]   s2_rm;
  always_ff @(posedge clk) begin
    s2_sign <= s1_sign; s2_ovfe <= s1_ovfe; s2_tafter <= tafter2; s2_st <= s1_st;
    s2_msh <= msh2; s2_mlow <= mlow2; s2_expf <= s1_expf; s2_fm <= s1_fm; s2_rm <= s1_rm;
  end

  // ---- stage 3: lost / guard / sticky(high) / kept
  int           p3, rpos3;
  logic [54:0]  lowmask3, kept3;
  logic         g3, shi3, lost3;
  always_comb begin
    case (s2_fm)
      F16:     p3 = 11;
      F32:     p3 = 24;
      default: p3 = 53;
    endcase
    rpos3    = 55 - p3;
    lowmask3 = (55'd1 << (rpos3 - 1)) - 55'd1;
    lost3    = |s2_mlow;
    g3       = s2_msh[rpos3 - 1];
    shi3     = |(s2_msh & lowmask3);
    kept3    = s2_msh >> rpos3;
  end
  logic         s3_sign, s3_ovfe, s3_tafter, s3_g, s3_s;
  logic [52:0]  s3_kept;
  logic [11:0]  s3_expf;
  logic [1:0]   s3_fm;
  logic [2:0]   s3_rm;
  always_ff @(posedge clk) begin
    s3_sign <= s2_sign; s3_ovfe <= s2_ovfe; s3_tafter <= s2_tafter;
    s3_g <= g3; s3_s <= shi3 | lost3 | s2_st; s3_kept <= kept3[52:0];
    s3_expf <= s2_expf; s3_fm <= s2_fm; s3_rm <= s2_rm;
  end

  // ---- stage 4: increment decision, base word
  int           p4;
  logic         ix4, inc4;
  logic [63:0]  base4;
  always_comb begin
    case (s3_fm)
      F16:     p4 = 11;
      F32:     p4 = 24;
      default: p4 = 53;
    endcase
    ix4 = s3_g | s3_s;
    case (s3_rm)
      RM_RNE:  inc4 = s3_g & (s3_s | s3_kept[0]);
      RM_RMM:  inc4 = s3_g;
      RM_RDN:  inc4 = s3_sign & ix4;
      RM_RUP:  inc4 = ~s3_sign & ix4;
      default: inc4 = 1'b0;
    endcase
    base4 = (64'(s3_expf) << (p4 - 1)) | (64'(s3_kept) & ((64'd1 << (p4 - 1)) - 64'd1));
  end
  logic         s4_sign, s4_ovfe, s4_tafter, s4_ix, s4_inc;
  logic [63:0]  s4_base;
  logic [1:0]   s4_fm;
  logic [2:0]   s4_rm;
  always_ff @(posedge clk) begin
    s4_sign <= s3_sign; s4_ovfe <= s3_ovfe; s4_tafter <= s3_tafter; s4_ix <= ix4; s4_inc <= inc4;
    s4_base <= base4; s4_fm <= s3_fm; s4_rm <= s3_rm;
  end

  // ---- stage 5: rounding increment
  logic [63:0]  comb5;
  always_comb comb5 = s4_base + 64'(s4_inc);
  logic         s5_sign, s5_ovfe, s5_tafter, s5_ix;
  logic [63:0]  s5_comb;
  logic [1:0]   s5_fm;
  logic [2:0]   s5_rm;
  always_ff @(posedge clk) begin
    s5_sign <= s4_sign; s5_ovfe <= s4_ovfe; s5_tafter <= s4_tafter; s5_ix <= s4_ix;
    s5_comb <= comb5; s5_fm <= s4_fm; s5_rm <= s4_rm;
  end

  // ---- stage 6: overflow, pack
  int           p6, ebits6;
  logic [63:0]  maxexp6;
  logic         ovf6, use_inf6;
  rnd_t         r6;
  always_comb begin
    case (s5_fm)
      F16:     begin p6 = 11; ebits6 = 5;  end
      F32:     begin p6 = 24; ebits6 = 8;  end
      default: begin p6 = 53; ebits6 = 11; end
    endcase
    maxexp6  = (64'd1 << ebits6) - 64'd1;
    ovf6     = s5_ovfe || ((s5_comb >> (p6 - 1)) >= maxexp6);
    use_inf6 = (s5_rm == RM_RNE) || (s5_rm == RM_RMM) || (s5_rm == RM_RDN && s5_sign) || (s5_rm == RM_RUP && !s5_sign);
    r6 = '0;
    if (ovf6) begin
      if (use_inf6) r6.bits = (64'(s5_sign) << (ebits6 + p6 - 1)) | (maxexp6 << (p6 - 1));
      else          r6.bits = (64'(s5_sign) << (ebits6 + p6 - 1)) | ((maxexp6 - 64'd1) << (p6 - 1)) | ((64'd1 << (p6 - 1)) - 64'd1);
    end else begin
      r6.bits = (64'(s5_sign) << (ebits6 + p6 - 1)) | s5_comb;
    end
    r6.of = ovf6;
    r6.nx = s5_ix | ovf6;
    r6.uf = s5_tafter & s5_ix & ~ovf6;
  end
  always_ff @(posedge clk) begin
    if (vch[4]) r <= r6;
  end
endmodule

// ======================================================================== FMA
module ng_sfu_fma
  import ng_sfu_fp_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [63:0] a,
  input  logic [63:0] b,
  input  logic [63:0] c,
  output logic        done,
  output logic [63:0] res
);
  // ---------------------------------------------------------------- unpack (2 stages)
  up_t  ua, ub, uc;
  logic ov_a, ov_b, ov_c;
  ng_sfu_unp u_ua (.clk(clk), .rst_n(rst_n), .iv(start), .x(a), .fmt(F64), .daz(1'b0), .ov(ov_a), .u(ua));
  ng_sfu_unp u_ub (.clk(clk), .rst_n(rst_n), .iv(start), .x(b), .fmt(F64), .daz(1'b0), .ov(ov_b), .u(ub));
  ng_sfu_unp u_uc (.clk(clk), .rst_n(rst_n), .iv(start), .x(c), .fmt(F64), .daz(1'b0), .ov(ov_c), .u(uc));

  logic [63:0] c_hold;
  always_ff @(posedge clk) if (start) c_hold <= c;

  // valid chain
  localparam int NV = 14;
  logic [NV-1:0] vc;   // vc[0]: limb stage valid ... see stage comments
  logic          rnd_ov;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) vc <= '0;
    else vc <= {vc[NV-2:0], ov_a};
  end
  // vc[0] = limb regs valid, [1] products, [2] A1, [3] A2, [4] A3 (full product),
  // [5] normalised product, [6] swap, [7] select, [8] shift, [9] sticky, [10] add,
  // [11] clz, [12] normalise-shift, [13] sticky of the result -> rounder input

  // ---------------------------------------------------------------- D1: limbs and exponent sum
  logic [25:0] a0;  logic [25:0] a1;  logic a2;
  logic [16:0] b0, b1, b2;  logic [1:0] b3;
  logic signed [13:0] ep_r;
  always_ff @(posedge clk) begin
    a0 <= ua.m[25:0]; a1 <= ua.m[51:26]; a2 <= ua.m[52];
    b0 <= ub.m[16:0]; b1 <= ub.m[33:17]; b2 <= ub.m[50:34]; b3 <= ub.m[52:51];
    ep_r <= {ua.e[12], ua.e} + {ub.e[12], ub.e};
  end

  // ---------------------------------------------------------------- PP: DSP partial products
  logic [42:0] t00, t01, t02, t10, t11, t12;
  logic [54:0] t6;
  logic [50:0] t7;
  logic [52:0] a53;
  always_comb a53 = {a2, a1, a0};
  always_ff @(posedge clk) begin
    t00 <= a0 * b0;  t01 <= a0 * b1;  t02 <= a0 * b2;
    t10 <= a1 * b0;  t11 <= a1 * b1;  t12 <= a1 * b2;
    t6  <= (b3[0] ? {2'b00, a53} : 55'd0) + (b3[1] ? {1'b0, a53, 1'b0} : 55'd0);
    t7  <= a2 ? {b2, b1, b0} : 51'd0;
  end

  // ---------------------------------------------------------------- A1..A3: adder tree
  logic [105:0] u0, u1, u2, u3, w0, w1, pf;
  always_ff @(posedge clk) begin
    u0 <= 106'(t00) + (106'(t01) << 17);
    u1 <= (106'(t02) << 34) + (106'(t10) << 26);
    u2 <= (106'(t11) << 43) + (106'(t12) << 60);
    u3 <= (106'(t6) << 51) + (106'(t7) << 52);
    w0 <= u0 + u1;
    w1 <= u2 + u3;
    pf <= w0 + w1;
  end

  // ---------------------------------------------------------------- NRM: normalise the product
  logic [105:0]       pn_r, y_r;
  logic signed [13:0] epn_r, ec_r;
  always_ff @(posedge clk) begin
    pn_r  <= pf[105] ? pf : (pf << 1);
    epn_r <= pf[105] ? ep_r + 14'sd1 : ep_r;
    y_r   <= {uc.m, 53'd0};
    ec_r  <= {uc.e[12], uc.e};
  end

  // ---------------------------------------------------------------- SWP: which operand is larger
  logic swap_r;
  always_ff @(posedge clk) swap_r <= (epn_r < ec_r) || ((epn_r == ec_r) && (pn_r < y_r));

  // ---------------------------------------------------------------- SEL: order, exponent difference
  logic [105:0]       big_a_r;
  logic [109:0]       bfull_r;
  logic signed [13:0] ea_r, dd_c;
  logic [6:0]         dsh_r;
  logic               sgn_a_r, same_r, far_r;
  always_comb dd_c = (swap_r ? ec_r : epn_r) - (swap_r ? epn_r : ec_r);
  always_ff @(posedge clk) begin
    big_a_r <= swap_r ? y_r : pn_r;
    bfull_r <= {(swap_r ? pn_r : y_r), 4'd0};
    ea_r    <= swap_r ? ec_r : epn_r;
    sgn_a_r <= swap_r ? uc.sign : (ua.sign ^ ub.sign);
    same_r  <= (swap_r ? uc.sign : (ua.sign ^ ub.sign)) == (swap_r ? (ua.sign ^ ub.sign) : uc.sign);
    dsh_r   <= (dd_c > 14'sd120) ? 7'd120 : dd_c[6:0];
    far_r   <= (dd_c > 14'sd120) || (dd_c[6:0] >= 7'd110);
  end

  // ---------------------------------------------------------------- SHF: alignment shift
  logic [109:0] bsh_r, mlow_r;
  logic [110:0] dmask_c;
  logic [105:0] big_a_s;
  logic [13:0]  ea_s;
  logic         sgn_a_s, same_s, far_s;
  always_comb dmask_c = (111'd1 << dsh_r) - 111'd1;
  always_ff @(posedge clk) begin
    bsh_r   <= far_r ? 110'd0 : (bfull_r >> dsh_r);
    mlow_r  <= bfull_r & dmask_c[109:0];
    big_a_s <= big_a_r; ea_s <= ea_r; sgn_a_s <= sgn_a_r; same_s <= same_r; far_s <= far_r;
  end

  // ---------------------------------------------------------------- STK: sticky into bit 0
  logic [110:0] ax_r, bx_r;
  logic [13:0]  ea_k;
  logic         sgn_a_k, same_k;
  always_ff @(posedge clk) begin
    ax_r    <= {1'b0, big_a_s, 4'd0};
    bx_r    <= {1'b0, bsh_r} | {110'd0, (far_s | (|mlow_r))};
    ea_k    <= ea_s; sgn_a_k <= sgn_a_s; same_k <= same_s;
  end

  // ---------------------------------------------------------------- ADD: add / subtract + special cases
  logic               pz_c, cz_c;
  assign pz_c = ua.zero | ub.zero;
  assign cz_c = uc.zero;
  logic [110:0]       sum_c;
  always_comb sum_c = same_k ? (ax_r + bx_r) : (ax_r - bx_r);
  logic               s2_special;
  logic [63:0]        s2_bits;
  logic [110:0]       s2_sum;
  logic signed [13:0] s2_e;
  logic               s2_sign;
  always_ff @(posedge clk) begin
    s2_special <= 1'b0;
    s2_bits    <= 64'd0;
    s2_sum     <= sum_c;
    s2_e       <= ea_k;
    s2_sign    <= sgn_a_k;
    if (pz_c && cz_c) begin
      s2_special <= 1'b1;
      s2_bits    <= {((ua.sign ^ ub.sign) == uc.sign) ? uc.sign : 1'b0, 63'd0};
    end else if (pz_c) begin
      s2_special <= 1'b1;
      s2_bits    <= c_hold;
    end else if (cz_c) begin
      s2_sum  <= {1'b0, pn_r, 4'd0};
      s2_e    <= epn_r;
      s2_sign <= ua.sign ^ ub.sign;
    end
  end

  // --------------------------------------------------------------
  logic [7:0]         lz_r;
  logic               sz_r;
  logic [110:0]       s3_sum;
  logic signed [13:0] s3_e;
  logic               s3_sign, s3_special;
  logic [63:0]        s3_bits;
  always_ff @(posedge clk) begin
    lz_r <= 8'(clz128({s2_sum, 17'd0}));
    sz_r <= (s2_sum == 111'd0);
    s3_sum <= s2_sum; s3_e <= s2_e; s3_sign <= s2_sign; s3_special <= s2_special; s3_bits <= s2_bits;
  end

  // ---------------------------------------------------------------- NSH: normalise shift
  logic [110:0]       sn_r;
  logic signed [13:0] e3_r;
  logic               s4_sign, s4_special, s4_sz;
  logic [63:0]        s4_bits;
  always_ff @(posedge clk) begin
    sn_r <= s3_sum << lz_r;
    e3_r <= s3_e + 14'sd1 - 14'(lz_r);
    s4_sign <= s3_sign; s4_special <= s3_special; s4_bits <= s3_bits; s4_sz <= sz_r;
  end

  // ---------------------------------------------------------------- STC: result sticky
  logic [54:0]        m5_r;
  logic               st5_r;
  logic signed [13:0] e5_r;
  logic               s5_sign, s5_special, s5_sz;
  logic [63:0]        s5_bits;
  always_ff @(posedge clk) begin
    m5_r  <= sn_r[110:56];
    st5_r <= |sn_r[55:0];
    e5_r  <= e3_r;
    s5_sign <= s4_sign; s5_special <= s4_special; s5_bits <= s4_bits; s5_sz <= s4_sz;
  end

  // ---------------------------------------------------------------- round (6 stages) and result
  rnd_t rr;
  ng_sfu_rnd #(.FIXED(1'b1)) u_rnd (.clk(clk), .rst_n(rst_n), .iv(vc[13]), .sign_i(s5_sign), .e_i(e5_r),
                                    .m_i(m5_r), .st_i(st5_r), .fmt_i(F64), .rm_i(RM_RNE), .ov(rnd_ov), .r(rr));

  always_ff @(posedge clk) begin
    if (rnd_ov) begin
      if (s5_special)  res <= s5_bits;
      else if (s5_sz)  res <= 64'd0;
      else             res <= rr.bits;
    end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) done <= 1'b0;
    else done <= rnd_ov;
  end
endmodule
