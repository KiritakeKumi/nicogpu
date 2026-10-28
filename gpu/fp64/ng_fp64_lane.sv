// ng_fp64_lane.sv -- one FP64 lane: binary64 fma / add / sub / mul, round-to-integral,
// conversions, min/max, compare, classify, frexp, ldexp.
//
// Bit-exact with SoftFloat (RISC-V specialisation) as driven by
// all five rounding modes, DAZ / FTZ, NaN ->
// canonical quiet NaN, tininess after rounding, NV/OF/UF/NX flags.
//
// Fixed latency of 3 cycles, fully pipelined (one operation per cycle):
//   S1  source modifiers + DAZ, unpack, 53x53 multiply, special-case resolution,
//       every non-rounding operation (min/max/compare/convert/...) computed here
//   S2  align + add / subtract (wide), exact-zero result sign
//   S3  normalise + round-and-pack (ng_sfu_fp_pkg::rnd_pack), FTZ, flags
// Rounding jobs share the pipeline: ldexp and f64 -> f32 enter S2 as a "product
// only" value (an exact significand + exponent) and are rounded in S3.
// Control inputs (op, rm, daz, ftz, ...) are static for the whole instruction: S3
// reads them again two cycles after S1, so the caller must hold them until the
// last result is out.
module ng_fp64_lane
  import ng_sfu_fp_pkg::*;
  import ng_fp64_pkg::*;
  import ng_fp64_fn_pkg::*;
(
  input  logic        clk,
  // static per instruction
  input  logic [4:0]  op,        // LOP_*
  input  logic [4:0]  pred,      // compare predicate
  input  logic [2:0]  rm,        // effective rounding mode 0..4
  input  logic        daz_en,    // DAZ64 applies to the sources (decoder dazk == 1 and MODE.daz64)
  input  logic        daz32_en,  // DAZ32 applies to the f32 source (fcvt.f64.f32)
  input  logic [1:0]  ftz_k,     // 0 none, 1 flush binary64 result, 2 flush binary32 result
  input  logic [2:0]  neg_m,     // ffma source modifiers
  input  logic [2:0]  abs_m,
  // per cycle
  input  logic [63:0] a,
  input  logic [63:0] b,
  input  logic [63:0] c,
  // result, 3 cycles after the operands
  output logic [63:0] res,
  output logic [4:0]  flags
);
  // ================================================================ S1 (combinational part)
  logic [63:0] a1, b1, c1, ad, bd, cd;
  always_comb begin
    a1 = a; b1 = b; c1 = c;
    if (abs_m[0]) a1[63] = 1'b0;
    if (neg_m[0]) a1[63] = ~a1[63];
    if (abs_m[1]) b1[63] = 1'b0;
    if (neg_m[1]) b1[63] = ~b1[63];
    if (abs_m[2]) c1[63] = 1'b0;
    if (neg_m[2]) c1[63] = ~c1[63];
    ad = daz64(a1, daz_en);
    bd = daz64(b1, daz_en);
    cd = daz64(c1, daz_en);
  end

  // ---- fma-family operand selection
  logic        is_fma_job, c_none;
  logic [63:0] fa, fb, fc, flo;
  fr_t         rfloor;
  always_comb begin
    rfloor = round_to_int64(ad, 3'd2);   // floor (for ffract)
    flo    = rfloor.bits;
    is_fma_job = (op == LOP_ADD) || (op == LOP_SUB) || (op == LOP_MUL) || (op == LOP_FMA) || (op == LOP_FRACT);
    c_none = (op == LOP_MUL);
    fa = ad;
    fb = ((op == LOP_MUL) || (op == LOP_FMA)) ? bd : F64_ONE;
    case (op)
      LOP_ADD:   fc = bd;
      LOP_SUB:   fc = {~bd[63], bd[62:0]};
      LOP_FMA:   fc = cd;
      LOP_FRACT: fc = {~flo[63], flo[62:0]};
      default:   fc = 64'd0;
    endcase
  end

  up_t ua, ub, uc;
  logic [105:0]        p_full, pn_c, y_c;
  logic signed [13:0]  ep_c, epn_c;
  logic                sp_c, sc_c, pz_c, cz_c;
  always_comb begin
    ua = unpack(fa, F64, 1'b0);
    ub = unpack(fb, F64, 1'b0);
    uc = unpack(fc, F64, 1'b0);
    p_full = {53'd0, ua.m} * {53'd0, ub.m};
    ep_c   = {ua.e[12], ua.e} + {ub.e[12], ub.e};
    pn_c   = p_full[105] ? p_full : (p_full << 1);
    epn_c  = p_full[105] ? ep_c + 14'sd1 : ep_c;
    y_c    = {uc.m, 53'd0};
    sp_c   = ua.sign ^ ub.sign;
    sc_c   = uc.sign;
    pz_c   = ua.zero | ub.zero;
    cz_c   = c_none | uc.zero;
  end

  // ---- single-source operations
  up_t  ux, u32x;
  logic [63:0] xd;
  logic signed [33:0] ldk;
  logic [31:0] a32;
  fr_t  rr;
  always_comb begin
    xd   = ad;
    ux   = unpack(xd, F64, 1'b0);
    a32  = (daz32_en && a[30:23] == 8'd0) ? {a[31], 31'd0} : a[31:0];
    u32x = unpack({32'd0, a32}, F32, 1'b0);
    ldk  = 34'(ux.e) + 34'($signed(b[31:0]));
  end

  // ---- S1 result record
  logic               s1_special;
  logic [63:0]        s1_bits;
  logic [4:0]         s1_fl;
  logic               s1_prodonly;
  logic [105:0]       s1_pn, s1_y;
  logic signed [13:0] s1_epn, s1_ec;
  logic               s1_sp, s1_sc;

  logic               n_nan, n_snan, n_invp, n_pinf, n_cinf, n_infc, n_abnan;
  always_comb begin
    s1_special  = 1'b1;
    s1_bits     = 64'd0;
    s1_fl       = 5'd0;
    s1_prodonly = 1'b0;
    s1_pn       = pn_c;
    s1_epn      = epn_c;
    s1_y        = y_c;
    s1_ec       = {uc.e[12], uc.e};
    s1_sp       = sp_c;
    s1_sc       = sc_c;
    rr          = '0;
    n_nan = 1'b0; n_snan = 1'b0; n_invp = 1'b0; n_pinf = 1'b0; n_cinf = 1'b0; n_infc = 1'b0; n_abnan = 1'b0;

    if (is_fma_job) begin
      // ---------------- fma family (add / sub / mul / fma / ffract)
      n_abnan = ua.nan | ub.nan;
      n_nan   = n_abnan | (!c_none & uc.nan);
      n_snan  = ua.snan | ub.snan | (!c_none & uc.snan);
      n_invp  = (ua.inf & ub.zero) | (ua.zero & ub.inf);
      n_pinf  = ua.inf | ub.inf;
      n_cinf  = !c_none & uc.inf;
      n_infc  = n_pinf & n_cinf & (sp_c != sc_c) & !n_abnan;
      if (n_nan || n_invp || n_infc) begin
        s1_bits = F64_QNAN;
        s1_fl   = (n_snan || n_invp || n_infc) ? FLG_NV : 5'd0;
      end else if (n_pinf) begin
        s1_bits = {sp_c, 11'h7ff, 52'd0};
      end else if (n_cinf) begin
        s1_bits = {sc_c, 11'h7ff, 52'd0};
      end else if (pz_c) begin
        if (cz_c) s1_bits = {c_none ? sp_c : ((sp_c == sc_c) ? sp_c : (rm == RM_RDN)), 63'd0};
        else      s1_bits = fc;
      end else begin
        s1_special  = 1'b0;
        s1_prodonly = cz_c;
      end
    end else begin
      case (op)
        LOP_MIN:  begin rr = minmax64(ad, bd, 1'b0); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_MAX:  begin rr = minmax64(ad, bd, 1'b1); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_SGNJ:  s1_bits = {b[63], a[62:0]};
        LOP_SGNJN: s1_bits = {~b[63], a[62:0]};
        LOP_SGNJX: s1_bits = {a[63] ^ b[63], a[62:0]};
        LOP_FCLASS: s1_bits = {54'd0, fclass64(a)};
        LOP_FLOOR:  begin rr = round_to_int64(ad, 3'd2); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_CEIL:   begin rr = round_to_int64(ad, 3'd3); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_TRUNC:  begin rr = round_to_int64(ad, 3'd1); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_RNDNE:  begin rr = round_to_int64(ad, 3'd0); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_CVT_I32_F64: begin rr = f64_to_i32_rtz(ad, 1'b0); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_CVT_U32_F64: begin rr = f64_to_i32_rtz(ad, 1'b1); s1_bits = rr.bits; s1_fl = rr.fl; end
        LOP_CVT_F64_I32: s1_bits = i32_to_f64(a[31:0], 1'b0);
        LOP_CVT_F64_U32: s1_bits = i32_to_f64(a[31:0], 1'b1);
        LOP_CVT_F64_F32: begin
          s1_bits = up_to_f64(u32x);
          s1_fl   = u32x.snan ? FLG_NV : 5'd0;
        end
        LOP_FREXP_M: begin
          if (f_exp_max(ad) || f_zero(ad)) s1_bits = ad;
          else s1_bits = {ad[63], 11'h3fe, ux.m[51:0]};
        end
        LOP_FREXP_E: begin
          if (f_exp_max(ad) || f_zero(ad)) s1_bits = 64'd0;
          else s1_bits = {32'd0, 32'(int'($signed(ux.e)) + 1)};
        end
        LOP_CMP: begin
          rr = fcmp64(a, b, pred); s1_bits = rr.bits; s1_fl = rr.fl;
        end
        LOP_CMP_CLASS: s1_bits = {63'd0, |(fclass64(a) & b[9:0])} ;
        LOP_LDEXP: begin
          if (ux.nan) begin
            s1_bits = F64_QNAN; s1_fl = ux.snan ? FLG_NV : 5'd0;
          end else if (ux.inf || ux.zero) begin
            s1_bits = xd;
          end else begin
            s1_special  = 1'b0;
            s1_prodonly = 1'b1;
            s1_pn       = {ux.m, 53'd0};
            s1_epn      = (ldk > 34'sd4000) ? 14'sd4000 : (ldk < -34'sd4000) ? -14'sd4000 : ldk[13:0];
            s1_sp       = ux.sign;
          end
        end
        LOP_CVT_F32_F64: begin
          if (ux.nan) begin
            s1_bits = 64'h0000_0000_7fc0_0000; s1_fl = ux.snan ? FLG_NV : 5'd0;
          end else if (ux.inf) begin
            s1_bits = inf_bits(ux.sign, F32);
          end else if (ux.zero) begin
            s1_bits = zero_bits(ux.sign, F32);
          end else begin
            s1_special  = 1'b0;
            s1_prodonly = 1'b1;
            s1_pn       = {ux.m, 53'd0};
            s1_epn      = {ux.e[12], ux.e};
            s1_sp       = ux.sign;
          end
        end
        default: ;
      endcase
    end
  end

  // ---- S1 registers
  logic               r1_special, r1_prodonly;
  logic [63:0]        r1_bits;
  logic [4:0]         r1_fl;
  logic [105:0]       r1_pn, r1_y;
  logic signed [13:0] r1_epn, r1_ec;
  logic               r1_sp, r1_sc;
  always_ff @(posedge clk) begin
    r1_special  <= s1_special;
    r1_prodonly <= s1_prodonly;
    r1_bits     <= s1_bits;
    r1_fl       <= s1_fl;
    r1_pn       <= s1_pn;
    r1_y        <= s1_y;
    r1_epn      <= s1_epn;
    r1_ec       <= s1_ec;
    r1_sp       <= s1_sp;
    r1_sc       <= s1_sc;
  end

  // ================================================================ S2
  logic               swap_c;
  logic [105:0]       big_a, big_b;
  logic signed [13:0] e_a, e_b, dd;
  logic               sgn_a, sgn_b;
  logic [110:0]       ax, bx, sum_c;
  logic [109:0]       bfull, bsh;
  logic               blost;
  int                 dsh;
  always_comb begin
    swap_c = (r1_epn < r1_ec) || ((r1_epn == r1_ec) && (r1_pn < r1_y));
    big_a = swap_c ? r1_y : r1_pn;     e_a = swap_c ? r1_ec : r1_epn;   sgn_a = swap_c ? r1_sc : r1_sp;
    big_b = swap_c ? r1_pn : r1_y;     e_b = swap_c ? r1_epn : r1_ec;   sgn_b = swap_c ? r1_sp : r1_sc;
    dd    = e_a - e_b;
    dsh   = (dd > 14'sd120) ? 120 : int'(dd);
    ax    = {1'b0, big_a, 4'd0};
    bfull = {big_b, 4'd0};
    if (dsh >= 110) begin
      bsh   = 110'd0;
      blost = 1'b1;
    end else begin
      bsh   = bfull >> dsh;
      blost = |(bfull & ((110'd1 << dsh) - 110'd1));
    end
    bx    = {1'b0, bsh} | {110'd0, blost};
    sum_c = (sgn_a == sgn_b) ? (ax + bx) : (ax - bx);
  end

  logic               r2_special;
  logic [63:0]        r2_bits;
  logic [4:0]         r2_fl;
  logic [110:0]       r2_sum;
  logic signed [13:0] r2_e;
  logic               r2_sign;
  always_ff @(posedge clk) begin
    r2_special <= r1_special;
    r2_bits    <= r1_bits;
    r2_fl      <= r1_fl;
    r2_sum     <= sum_c;
    r2_e       <= e_a;
    r2_sign    <= sgn_a;
    if (r1_prodonly) begin
      r2_sum  <= {1'b0, r1_pn, 4'd0};
      r2_e    <= r1_epn;
      r2_sign <= r1_sp;
    end else if (!r1_special && (sum_c == 111'd0)) begin
      // exact cancellation: +0, or -0 when rounding towards -inf
      r2_special <= 1'b1;
      r2_bits    <= {(rm == RM_RDN), 63'd0};
    end
  end

  // ================================================================ S3
  int unsigned        lz3;
  logic [110:0]       sn;
  logic signed [13:0] e3;
  rnd_t               r3;
  logic [1:0]         ofmt;
  logic [63:0]        f_bits;
  logic [4:0]         f_fl;
  logic               f_sub_hit;
  always_comb begin
    ofmt = (op == LOP_CVT_F32_F64) ? F32 : F64;
    lz3  = clz128({r2_sum, 17'd0});
    sn   = r2_sum << lz3;
    e3   = r2_e + 14'sd1 - 14'(lz3);
    r3   = rnd_pack(r2_sign, e3, sn[110:56], |sn[55:0], ofmt, rm);
    if (r2_special) begin
      f_bits = r2_bits;
      f_fl   = r2_fl;
    end else begin
      f_bits = r3.bits;
      f_fl   = {2'b00, r3.of, r3.uf, r3.nx};
    end
    if (op == LOP_FRACT && f_bits == F64_ONE) f_bits = F64_BELOW_ONE;
    // FTZ: subnormal result -> signed zero with UF|NX
    f_sub_hit = 1'b0;
    if (ftz_k == 2'd1) f_sub_hit = f_sub(f_bits);
    if (ftz_k == 2'd2) f_sub_hit = (f_bits[30:23] == 8'd0) && (f_bits[22:0] != 23'd0);
    if (f_sub_hit) begin
      f_fl = f_fl | FLG_UF | FLG_NX;
      f_bits = (ftz_k == 2'd1) ? {f_bits[63], 63'd0} : {32'd0, f_bits[31], 31'd0};
    end
  end

  always_ff @(posedge clk) begin
    res   <= f_bits;
    flags <= f_fl;
  end
endmodule
