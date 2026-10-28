// ng_valu_fp_lane.sv -- one f32 VALU lane: 7-stage pipeline.
//
// Every IEEE operation is evaluated as a single-rounding fused multiply-add
// engine (24x24 mantissa product -> align -> add -> normalise -> round) with
// SoftFloat's exact special-value and flag behaviour (RISC-V specialisation,
// tininess after rounding), plus a combinational "side" block
// (ng_valu_fp_side) for min/max/compare/convert/round-to-integer results.
//
//   in  (a,b,c,fop,...)          operand conditioning: neg/abs, DAZ, op rewiring
//   S0  registers xa/xb/xc
//   S1  unpack / normalise, special cases, side results      -> regs
//   S2  24x24 multiply                                       -> regs
//   S3  align (big/small, sticky jam)                        -> regs
//   S4  add / subtract                                       -> regs
//   S5  leading-zero count + normalise shift                 -> regs
//   S6  round / pack / FTZ / fract / sat / result select     -> regs (res, fl, cbit)
// The result of an input presented at cycle T is at the outputs at T + LAT.
// Control inputs (fop, rm, ftz, sat, ...) are sampled with the operands and
// travel down the pipeline with them (consecutive lane groups may belong
// to different instructions): S0 uses the inputs, S1 and S6 registered copies.
//
//   operation forms (fused engine  r = a*b + c):
//     add: a*1 + b   sub: a*1 - b   subr: b*1 - a   mul: a*b + (+-0)
//     fma: a*b + c   ldexp: a * 2^n + (+-0)   fract: x*1 - floor(x)
//     cvt i/u -> f: integer magnitude injected at the add stage
module ng_valu_fp_lane
  import ng_valu_fp_pkg::*;
#(
  parameter bit EN_F16 = 1'b0     // binary16 <-> binary32 conversions in the side block
) (
  input  logic        clk,
  input  logic [5:0]  fop,
  input  logic [4:0]  pred,
  input  logic [2:0]  rm,        // effective rounding mode of the instruction
  input  logic [1:0]  ub,
  input  logic        daz,       // MODE.DAZ32
  input  logic        ftz,       // MODE.FTZ32
  input  logic        daz16,     // MODE.DAZ16 (binary16 conversions)
  input  logic        ftz16,     // MODE.FTZ16
  input  logic        sat,
  input  logic [2:0]  neg,
  input  logic [2:0]  abs_m,
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic [31:0] c,
  output logic [31:0] res,
  output logic [4:0]  fl,
  output logic        cbit
);
  localparam int LAT = 7;

  // ================================================================== S0
  function automatic logic [31:0] srcmod(input logic [31:0] x, input logic n, input logic ab);
    logic [31:0] y;
    y = ab ? {1'b0, x[30:0]} : x;
    y[31] = y[31] ^ n;
    return y;
  endfunction

  logic [31:0] am, bm, cm, ad, bd, cd, flo;
  logic        dz;
  logic [32:0] fl_ri;
  logic [31:0] s0_a, s0_b, s0_c;
  always_comb begin
    am = srcmod(a, neg[0], abs_m[0]);
    bm = srcmod(b, neg[1], abs_m[1]);
    cm = srcmod(c, neg[2], abs_m[2]);
    dz = daz && fop_daz(fop);
    ad = dz ? f_flush(am) : am;
    bd = (dz && fop != FO_LDEXP) ? f_flush(bm) : bm;
    cd = dz ? f_flush(cm) : cm;
    fl_ri = round_int(ad, RM_RDN);
    flo   = fl_ri[31:0];
    s0_a = ad; s0_b = bd; s0_c = cd;
    case (fop)
      FO_ADD:   begin s0_a = ad; s0_b = F32_ONE; s0_c = bd; end
      FO_SUB:   begin s0_a = ad; s0_b = F32_ONE; s0_c = {~bd[31], bd[30:0]}; end
      FO_SUBR:  begin s0_a = bd; s0_b = F32_ONE; s0_c = {~ad[31], ad[30:0]}; end
      FO_MUL:   begin s0_a = ad; s0_b = bd; s0_c = {ad[31] ^ bd[31], 31'd0}; end
      FO_FMA:   begin s0_a = ad; s0_b = bd; s0_c = cd; end
      FO_LDEXP: begin s0_a = ad; s0_b = bd; s0_c = {ad[31], 31'd0}; end
      FO_FRACT: begin s0_a = ad; s0_b = F32_ONE; s0_c = {~flo[31], flo[30:0]}; end
      FO_CVTIF, FO_CVTUF: begin s0_a = a; s0_b = 32'd0; s0_c = 32'd0; end
      default:  begin s0_a = ad; s0_b = bd; s0_c = cd; end
    endcase
  end

  logic [31:0] x_a, x_b, x_c;
  always_ff @(posedge clk) begin
    x_a <= s0_a; x_b <= s0_b; x_c <= s0_c;
  end

  // control copies per stage (S1: fop/pred/rm/ub for the unpack + side block;
  // S2..S6: what the round / select stage needs)
  typedef struct packed {
    logic [5:0] fop;
    logic [2:0] rm;
    logic       ftz;
    logic       sat;
  } ctl6_t;
  logic [4:0]  c1_pred;
  logic [1:0]  c1_ub;
  logic        c1_dz32, c1_dz16, c1_fz16;   // MODE bits of the binary16 conversions (side block)
  ctl6_t       c_q [1:6];
  always_ff @(posedge clk) begin
    c1_pred <= pred;
    c1_ub   <= ub;
    if (EN_F16) begin
      c1_dz32 <= daz; c1_dz16 <= daz16; c1_fz16 <= ftz16;
    end else begin
      c1_dz32 <= 1'b0; c1_dz16 <= 1'b0; c1_fz16 <= 1'b0;
    end
    c_q[1]  <= '{fop: fop, rm: rm, ftz: ftz, sat: sat};
    for (int k = 2; k <= 6; k++) c_q[k] <= c_q[k-1];
  end
  logic [5:0] fop1, fop6;
  logic [2:0] rm1, rm6;
  logic       ftz6, sat6;
  assign fop1 = c_q[1].fop;
  assign rm1  = c_q[1].rm;
  assign fop6 = c_q[6].fop;
  assign rm6  = c_q[6].rm;
  assign ftz6 = c_q[6].ftz;
  assign sat6 = c_q[6].sat;

  // ================================================================== S1
  // side results (combinational on the registered operands)
  logic [31:0] sd_res;
  logic [4:0]  sd_fl;
  logic        sd_cbit;
  ng_valu_fp_side #(.EN_F16(EN_F16)) u_side (
    .fop(fop1), .pred(c1_pred), .rm(rm1), .ub(c1_ub),
    .daz32(c1_dz32), .daz16(c1_dz16), .ftz16(c1_fz16), .ra(x_a), .rb(x_b), .rc(x_c),
    .res(sd_res), .fl(sd_fl), .cbit(sd_cbit)
  );

  function automatic logic [34:0] unpk(input logic [31:0] x);
    logic [4:0]  lz;
    logic [23:0] v, m;
    logic signed [10:0] e;
    if (x[30:23] == 8'd0) begin
      v  = {1'b0, x[22:0]};
      lz = clz24(v);
      m  = v << lz;
      e  = -11'sd126 - $signed({6'd0, lz});
    end else begin
      m  = {1'b1, x[22:0]};
      e  = $signed({3'd0, x[30:23]}) - 11'sd127;
    end
    return {e, m};
  endfunction

  logic        is_cvt, is_ld, cv_neg;
  logic [31:0] cv_m;
  logic [34:0] ua, ub_, uc;
  logic        nA, nB, nC, sA, sB, sC, iA, iB, iC, zA, zB, zC;
  logic        anynan, anysnan, pinf, pnv, cdiff, sgnP;
  logic        sp_v, sp_nv;
  logic [31:0] sp_b;
  logic signed [10:0] n_cl;
  logic signed [31:0] nb_s;
  logic [23:0] ma_c, mb_c, mc_c;
  logic signed [10:0] ea_c, eb_c, ec_c;

  always_comb begin
    is_cvt = (fop1 == FO_CVTIF) || (fop1 == FO_CVTUF);
    is_ld  = (fop1 == FO_LDEXP);
    cv_neg = (fop1 == FO_CVTIF) && x_a[31];
    cv_m   = cv_neg ? (32'd0 - x_a) : x_a;

    ua  = unpk(x_a);
    ub_ = unpk(x_b);
    uc  = unpk(x_c);
    sA = x_a[31]; sB = x_b[31]; sC = x_c[31];
    nA = !is_cvt && f_nan(x_a);   iA = !is_cvt && f_inf(x_a);   zA = is_cvt ? (x_a == 32'd0) : f_zero(x_a);
    nB = !is_ld && f_nan(x_b);    iB = !is_ld && f_inf(x_b);    zB = !is_ld && f_zero(x_b);
    nC = f_nan(x_c);              iC = f_inf(x_c);              zC = f_zero(x_c);
    anynan  = nA | nB | nC;
    anysnan = (nA && !x_a[22]) | (nB && !x_b[22]) | (nC && !x_c[22]);
    sgnP    = sA ^ (is_ld ? 1'b0 : sB);
    pinf    = iA | iB;
    pnv     = (iA & zB) | (iB & zA);
    // inf - inf of the product and the addend is invalid only when the product is really an infinity: a NaN factor makes
    // the product a NaN whatever the other one is (arch_seq: fma(+inf, -qNaN, +inf) raised a spurious NV)
    cdiff   = pinf & iC & (sgnP != sC) & !nA & !nB;

    sp_v = 1'b0; sp_nv = 1'b0; sp_b = 32'd0;
    if (is_cvt) begin
      if (x_a == 32'd0) begin sp_v = 1'b1; sp_b = 32'd0; end
    end else if (anynan | pnv | cdiff) begin
      sp_v = 1'b1; sp_nv = anysnan | pnv | cdiff; sp_b = F32_QNAN;
    end else if (pinf) begin
      sp_v = 1'b1; sp_b = {sgnP, 8'hff, 23'd0};
    end else if (iC) begin
      sp_v = 1'b1; sp_b = {sC, 8'hff, 23'd0};
    end else if (zA | zB) begin
      sp_v = 1'b1;
      if (zC) sp_b = (sgnP == sC) ? {sC, 31'd0} : {(rm1 == RM_RDN), 31'd0};
      else    sp_b = x_c;
    end

    // exponent of the b operand: ldexp uses the clamped count, mantissa 1.0
    nb_s = $signed(x_b);
    if (nb_s > 32'sd300)       n_cl = 11'sd300;
    else if (nb_s < -32'sd300) n_cl = -11'sd300;
    else                       n_cl = nb_s[10:0];

    ma_c = ua[23:0];  ea_c = ua[34:24];
    mb_c = is_ld ? 24'h800000 : ub_[23:0];
    eb_c = is_ld ? n_cl : ub_[34:24];
    mc_c = zC ? 24'd0 : uc[23:0];
    ec_c = zC ? -11'sd900 : uc[34:24];
  end

  logic [23:0] r1_ma, r1_mb, r1_mc;
  logic signed [10:0] r1_ea, r1_eb, r1_ec;
  logic        r1_sp, r1_spnv, r1_sgnP, r1_sgnC, r1_cv, r1_cvs;
  logic [31:0] r1_spb, r1_cvm, r1_sd;
  logic [4:0]  r1_sdfl;
  logic        r1_cb;
  always_ff @(posedge clk) begin
    r1_ma <= ma_c; r1_mb <= mb_c; r1_mc <= mc_c;
    r1_ea <= ea_c; r1_eb <= eb_c; r1_ec <= ec_c;
    r1_sp <= sp_v; r1_spnv <= sp_nv; r1_spb <= sp_b;
    r1_sgnP <= sgnP; r1_sgnC <= sC;
    r1_cv <= is_cvt; r1_cvs <= cv_neg; r1_cvm <= cv_m;
    r1_sd <= sd_res; r1_sdfl <= sd_fl; r1_cb <= sd_cbit;
  end

  // ================================================================== S2
  logic [47:0] r2_p;
  logic [23:0] r2_mc;
  logic signed [10:0] r2_eab, r2_ec;
  logic        r2_sp, r2_spnv, r2_sgnP, r2_sgnC, r2_cv, r2_cvs;
  logic [31:0] r2_spb, r2_cvm, r2_sd;
  logic [4:0]  r2_sdfl;
  logic        r2_cb;
  always_ff @(posedge clk) begin
    r2_p   <= r1_ma * r1_mb;
    r2_mc  <= r1_mc;
    r2_eab <= r1_ea + r1_eb;
    r2_ec  <= r1_ec;
    r2_sp <= r1_sp; r2_spnv <= r1_spnv; r2_spb <= r1_spb;
    r2_sgnP <= r1_sgnP; r2_sgnC <= r1_sgnC;
    r2_cv <= r1_cv; r2_cvs <= r1_cvs; r2_cvm <= r1_cvm;
    r2_sd <= r1_sd; r2_sdfl <= r1_sdfl; r2_cb <= r1_cb;
  end

  // ================================================================== S3 (align)
  logic [47:0] pn, y;
  logic signed [11:0] ep, ecs, dd, dabs;
  logic        p_big, esub_c, sb_c, ss_c;
  logic [47:0] bigm, smallm;
  logic [50:0] big51, sm51, sm_sh;
  logic [5:0]  dsh;
  logic [63:0] lmask;
  logic        lost;
  logic signed [10:0] eb3;
  logic [50:0] big_o, small_o;
  logic        esub_o, sb_o;
  logic signed [10:0] eb_o;
  always_comb begin
    pn  = r2_p[47] ? r2_p : {r2_p[46:0], 1'b0};
    ep  = {r2_eab[10], r2_eab} + (r2_p[47] ? 12'sd1 : 12'sd0);
    ecs = {r2_ec[10], r2_ec};
    y   = {r2_mc, 24'd0};
    dd  = ep - ecs;
    if (dd > 12'sd0)      p_big = 1'b1;
    else if (dd < 12'sd0) p_big = 1'b0;
    else                  p_big = (pn >= y);
    dabs   = (dd < 12'sd0) ? -dd : dd;
    bigm   = p_big ? pn : y;
    smallm = p_big ? y : pn;
    sb_c   = p_big ? r2_sgnP : r2_sgnC;
    ss_c   = p_big ? r2_sgnC : r2_sgnP;
    eb3    = p_big ? ep[10:0] : ecs[10:0];
    esub_c = sb_c != ss_c;
    big51  = {bigm, 3'b000};
    sm51   = {smallm, 3'b000};
    dsh    = (dabs > 12'sd63) ? 6'd63 : dabs[5:0];
    lmask  = (64'd1 << dsh) - 64'd1;
    sm_sh  = sm51 >> dsh;
    lost   = |(sm51 & lmask[50:0]);
    sm_sh[0] = sm_sh[0] | lost;
    big_o = big51; small_o = sm_sh; esub_o = esub_c; sb_o = sb_c; eb_o = eb3;
    if (r2_cv) begin
      big_o = {r2_cvm, 19'd0}; small_o = 51'd0; esub_o = 1'b0; sb_o = r2_cvs; eb_o = 11'sd31;
    end
  end

  logic [50:0] r3_big, r3_small;
  logic        r3_esub, r3_sb;
  logic signed [10:0] r3_eb;
  logic        r3_sp, r3_spnv;
  logic [31:0] r3_spb, r3_sd;
  logic [4:0]  r3_sdfl;
  logic        r3_cb;
  always_ff @(posedge clk) begin
    r3_big <= big_o; r3_small <= small_o; r3_esub <= esub_o; r3_sb <= sb_o; r3_eb <= eb_o;
    r3_sp <= r2_sp; r3_spnv <= r2_spnv; r3_spb <= r2_spb;
    r3_sd <= r2_sd; r3_sdfl <= r2_sdfl; r3_cb <= r2_cb;
  end

  // ================================================================== S4 (add / subtract)
  logic [51:0] sum_c;
  always_comb begin
    sum_c = r3_esub ? ({1'b0, r3_big} - {1'b0, r3_small}) : ({1'b0, r3_big} + {1'b0, r3_small});
  end
  logic [51:0] r4_sum;
  logic        r4_sb;
  logic signed [10:0] r4_eb;
  logic        r4_sp, r4_spnv;
  logic [31:0] r4_spb, r4_sd;
  logic [4:0]  r4_sdfl;
  logic        r4_cb;
  always_ff @(posedge clk) begin
    r4_sum <= sum_c; r4_sb <= r3_sb; r4_eb <= r3_eb;
    r4_sp <= r3_sp; r4_spnv <= r3_spnv; r4_spb <= r3_spb;
    r4_sd <= r3_sd; r4_sdfl <= r3_sdfl; r4_cb <= r3_cb;
  end

  // ================================================================== S5 (normalise)
  logic [5:0]  lz5;
  logic [51:0] sn;
  logic signed [11:0] e5;
  always_comb begin
    lz5 = clz52(r4_sum);
    sn  = r4_sum << lz5;
    e5  = {r4_eb[10], r4_eb} + 12'sd1 - $signed({6'd0, lz5});
  end
  logic        r5_zero, r5_sb;
  logic [23:0] r5_sig;
  logic        r5_g, r5_st;
  logic signed [11:0] r5_e;
  logic        r5_sp, r5_spnv;
  logic [31:0] r5_spb, r5_sd;
  logic [4:0]  r5_sdfl;
  logic        r5_cb;
  always_ff @(posedge clk) begin
    r5_zero <= (r4_sum == 52'd0);
    r5_sig  <= sn[51:28];
    r5_g    <= sn[27];
    r5_st   <= |sn[26:0];
    r5_e    <= e5;
    r5_sb   <= r4_sb;
    r5_sp <= r4_sp; r5_spnv <= r4_spnv; r5_spb <= r4_spb;
    r5_sd <= r4_sd; r5_sdfl <= r4_sdfl; r5_cb <= r4_cb;
  end

  // ================================================================== S6 (round / select)
  logic [25:0] mm, ms;
  logic        tiny0, gg, ss, lsb, ix, inc, gu, su, lsbu, incu, carryu, tiny_after, ovf0, ovf, use_inf, lostb;
  logic [11:0] shamt;
  logic [25:0] lmask26;
  logic [30:0] pk;
  logic [31:0] pk_sum;
  logic [7:0]  be;
  logic [31:0] fb;
  logic [4:0]  ff_;
  logic [31:0] rb_;
  logic [4:0]  rf_;
  logic        nanr;

  function automatic logic round_inc(input logic [2:0] m, input logic sg, input logic g, input logic s, input logic l);
    case (m)
      RM_RNE:  return g & (s | l);
      RM_RMM:  return g;
      RM_RDN:  return sg & (g | s);
      RM_RUP:  return ~sg & (g | s);
      default: return 1'b0;
    endcase
  endfunction

  always_comb begin
    mm    = {r5_sig, r5_g, r5_st};
    tiny0 = (r5_e < -12'sd126);
    shamt = tiny0 ? 12'(-12'sd126 - r5_e) : 12'd0;
    lostb = 1'b0;
    if (shamt >= 12'd26) begin
      ms = 26'd1;
    end else begin
      lmask26 = (26'd1 << shamt[4:0]) - 26'd1;
      ms      = mm >> shamt[4:0];
      lostb   = |(mm & lmask26);
      ms[0]   = ms[0] | lostb;
    end
    gg  = ms[1]; ss = ms[0]; lsb = ms[2];
    ix  = gg | ss;
    inc = round_inc(rm6, r5_sb, gg, ss, lsb);
    gu  = mm[1]; su = mm[0]; lsbu = mm[2];
    incu   = round_inc(rm6, r5_sb, gu, su, lsbu);
    carryu = incu & (&mm[25:2]);
    tiny_after = tiny0 && !(r5_e == -12'sd127 && carryu);
    ovf0 = !tiny0 && (r5_e > 12'sd127);
    be   = tiny0 ? 8'd0 : 8'(r5_e + 12'sd127);
    pk   = {be, ms[24:2]};
    pk_sum = {1'b0, pk} + {31'd0, inc};
    use_inf = (rm6 == RM_RNE) || (rm6 == RM_RMM) || (rm6 == RM_RDN && r5_sb) || (rm6 == RM_RUP && !r5_sb);
    ovf  = ovf0 || (pk_sum[30:23] == 8'hff);

    rb_ = {r5_sb, pk_sum[30:0]};
    rf_ = 5'd0;
    if (ovf) begin
      rb_ = use_inf ? {r5_sb, 8'hff, 23'd0} : {r5_sb, 8'hfe, 23'h7fffff};
      rf_[FL_OF] = 1'b1; rf_[FL_NX] = 1'b1;
    end else begin
      rf_[FL_NX] = ix;
      rf_[FL_UF] = tiny_after & ix;
    end

    // result select within the FMA class
    fb = rb_; ff_ = rf_;
    if (r5_sp) begin
      fb = r5_spb; ff_ = {r5_spnv, 4'd0};
    end else if (r5_zero) begin
      fb = {(rm6 == RM_RDN), 31'd0}; ff_ = 5'd0;
    end
    // FTZ: subnormal result -> signed zero, UF | NX
    if (ftz6 && fb[30:23] == 8'd0 && fb[22:0] != 23'd0) begin
      fb = {fb[31], 31'd0};
      ff_[FL_UF] = 1'b1; ff_[FL_NX] = 1'b1;
    end
    nanr = f_nan(fb);
    if (fop6 == FO_FRACT && !nanr && !fb[31] && fb >= F32_ONE) fb = 32'h3f7fffff;
    if (sat6) begin
      if (nanr || fb[31]) fb = 32'd0;
      else if (fb >= F32_ONE) fb = F32_ONE;
    end
  end

  always_ff @(posedge clk) begin
    if (fop_fma(fop6)) begin
      res  <= fb;
      fl   <= ff_;
      cbit <= 1'b0;
    end else begin
      res  <= r5_sd;
      fl   <= r5_sdfl;
      cbit <= r5_cb;
    end
  end
endmodule
