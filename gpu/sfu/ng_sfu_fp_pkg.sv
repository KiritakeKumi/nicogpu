// ng_sfu_fp_pkg.sv -- bit-exact IEEE building blocks of the SFU.
//
// Everything here mirrors what the functional model gets from SoftFloat
// (RISC-V specialisation: canonical quiet NaN, tininess detected after
// rounding) so that the SFU RTL can be compared bit for bit:
//   unpack / cvt_up        binary16/32/64 -> normalised significand + exponent
//   rnd_pack               round + pack (all five rm, subnormals, overflow,
//                          NX/UF/OF exactly as softfloat_roundPackToF*)
//   rne_split              x -> (rne(x), x - rne(x)) for |x| < 2^52 (exact)
//   coef                   Taylor / atanh polynomial tables of sfu_math.h
// Pure functions, no state; Verilator 5 and xsim clean.
package ng_sfu_fp_pkg;

  // format codes
  localparam logic [1:0] F16 = 2'd0, F32 = 2'd1, F64 = 2'd2;
  // rounding-mode codes (= SoftFloat / `rm` operand codes)
  localparam logic [2:0] RM_RNE = 3'd0, RM_RTZ = 3'd1, RM_RDN = 3'd2, RM_RUP = 3'd3, RM_RMM = 3'd4;
  // fflags bits
  localparam int FL_NX = 0, FL_UF = 1, FL_OF = 2, FL_DZ = 3, FL_NV = 4;

  localparam logic [63:0] F64_ONE   = 64'h3ff0_0000_0000_0000;
  localparam logic [63:0] F64_NZERO = 64'h8000_0000_0000_0000;
  localparam logic [63:0] F64_QNAN  = 64'h7ff8_0000_0000_0000;
  localparam logic [63:0] F64_INF   = 64'h7ff0_0000_0000_0000;

  typedef struct packed {
    logic               sign;
    logic               nan;
    logic               snan;
    logic               inf;
    logic               zero;
    logic signed [12:0] e;      // unbiased exponent of the normalised value
    logic [52:0]        m;      // significand, bit 52 = 1 (finite, non-zero)
  } up_t;

  typedef struct packed {
    logic [63:0] bits;
    logic        nx;
    logic        uf;
    logic        of;
  } rnd_t;

  typedef struct packed {
    logic [63:0]        r;      // x - rne(x) as binary64 (+0 when exact)
    logic signed [11:0] n;      // rne(x), valid for |x| < 2^11
  } split_t;

  // leading zeros of a 128-bit vector (128 when zero)
  function automatic int unsigned clz128(input logic [127:0] v);
    int unsigned n;
    logic        found;
    n = 0;
    found = 1'b0;
    for (int i = 127; i >= 0; i--) begin
      if (!found) begin
        if (v[i]) found = 1'b1;
        else n = n + 1;
      end
    end
    return n;
  endfunction

  // --------------------------------------------------------------
  // x holds the value in its low 16/32/64 bits.  daz: subnormal -> signed zero.
  function automatic up_t unpack(input logic [63:0] x, input logic [1:0] fmt, input logic daz);
    up_t         u;
    logic        sgn;
    logic [10:0] ef, emax;
    logic [51:0] fr;
    int          bias;
    int unsigned lz;
    logic [52:0] sh;
    case (fmt)
      F16: begin sgn = x[15]; ef = {6'd0, x[14:10]}; fr = {x[9:0], 42'd0};  emax = 11'd31;   bias = 15;   end
      F32: begin sgn = x[31]; ef = {3'd0, x[30:23]}; fr = {x[22:0], 29'd0}; emax = 11'd255;  bias = 127;  end
      default: begin sgn = x[63]; ef = x[62:52];     fr = x[51:0];          emax = 11'd2047; bias = 1023; end
    endcase
    u = '0;
    u.sign = sgn;
    if (ef == emax) begin
      if (fr == 52'd0) u.inf = 1'b1;
      else begin u.nan = 1'b1; u.snan = ~fr[51]; end
    end else if (ef == 11'd0) begin
      if (fr == 52'd0 || daz) u.zero = 1'b1;
      else begin
        lz  = clz128({fr, 76'd0});
        sh  = {fr, 1'b0} << lz;
        u.m = sh;
        u.e = 13'(-bias - int'(lz));
      end
    end else begin
      u.m = {1'b1, fr};
      u.e = 13'(int'(ef) - bias);
    end
    return u;
  endfunction

  // up_t -> binary64 bits (exact).  NaN -> canonical quiet NaN.
  function automatic logic [63:0] up_to_f64(input up_t u);
    logic [63:0] b;
    if (u.nan)       b = F64_QNAN;
    else if (u.inf)  b = {u.sign, 11'h7ff, 52'd0};
    else if (u.zero) b = {u.sign, 63'd0};
    else             b = {u.sign, 11'(int'($signed(u.e)) + 1023), u.m[51:0]};
    return b;
  endfunction

  function automatic logic [63:0] nan_bits(input logic [1:0] fmt);
    case (fmt)
      F16:     return 64'h0000_0000_0000_7e00;
      F32:     return 64'h0000_0000_7fc0_0000;
      default: return F64_QNAN;
    endcase
  endfunction

  function automatic logic [63:0] inf_bits(input logic sgn, input logic [1:0] fmt);
    case (fmt)
      F16:     return {48'd0, sgn, 5'h1f, 10'd0};
      F32:     return {32'd0, sgn, 8'hff, 23'd0};
      default: return {sgn, 11'h7ff, 52'd0};
    endcase
  endfunction

  function automatic logic [63:0] zero_bits(input logic sgn, input logic [1:0] fmt);
    case (fmt)
      F16:     return {48'd0, sgn, 15'd0};
      F32:     return {32'd0, sgn, 31'd0};
      default: return {sgn, 63'd0};
    endcase
  endfunction

  function automatic logic [63:0] one_bits(input logic [1:0] fmt);
    case (fmt)
      F16:     return 64'h0000_0000_0000_3c00;
      F32:     return 64'h0000_0000_3f80_0000;
      default: return F64_ONE;
    endcase
  endfunction

  // ---------------------------------------------------------------- round + pack
  // value = (M / 2^54) * 2^e, M[54] = 1 (normalised), st = sticky of discarded bits.
  function automatic rnd_t rnd_pack(input logic sign, input logic signed [13:0] e, input logic [54:0] M,
                                    input logic st, input logic [1:0] fmt, input logic [2:0] rm);
    rnd_t         r;
    int           p, ebits, bias, emin, emax, ei, sh, rpos;
    logic         tiny0, g, s, ix, inc, gu, su, ixu, incu, carryu, tiny_after, ovf, use_inf;
    logic [109:0] t;
    logic [54:0]  msh, kept, keptu, lowmask;
    logic         lost;
    logic [63:0]  base, comb, expfield, maxexp, pmask;
    case (fmt)
      F16:     begin p = 11; ebits = 5;  bias = 15;   end
      F32:     begin p = 24; ebits = 8;  bias = 127;  end
      default: begin p = 53; ebits = 11; bias = 1023; end
    endcase
    emin = 1 - bias;
    emax = bias;
    ei   = int'(e);
    rpos = 55 - p;
    tiny0 = (ei < emin);
    sh = tiny0 ? (emin - ei) : 0;
    if (sh > 55) sh = 55;
    t    = {M, 55'd0} >> sh;
    msh  = t[109:55];
    lost = |t[54:0];
    lowmask = (55'd1 << (rpos - 1)) - 55'd1;
    kept = msh >> rpos;
    g    = msh[rpos - 1];
    s    = (|(msh & lowmask)) | lost | st;
    ix   = g | s;
    case (rm)
      RM_RNE:  inc = g & (s | kept[0]);
      RM_RMM:  inc = g;
      RM_RDN:  inc = sign & ix;
      RM_RUP:  inc = ~sign & ix;
      default: inc = 1'b0;
    endcase
    // tininess after rounding (unshifted rounding with unbounded exponent)
    gu   = M[rpos - 1];
    su   = (|(M & lowmask)) | st;
    ixu  = gu | su;
    case (rm)
      RM_RNE:  incu = gu & (su | M[rpos]);
      RM_RMM:  incu = gu;
      RM_RDN:  incu = sign & ixu;
      RM_RUP:  incu = ~sign & ixu;
      default: incu = 1'b0;
    endcase
    keptu  = M >> rpos;
    pmask  = (64'd1 << p) - 64'd1;
    carryu = incu & ((64'(keptu) & pmask) == pmask);
    tiny_after = tiny0 && !(ei == emin - 1 && carryu);

    expfield = tiny0 ? 64'd0 : 64'(ei + bias);
    base = (expfield << (p - 1)) | (64'(kept) & ((64'd1 << (p - 1)) - 64'd1));
    comb = base + 64'(inc);
    maxexp = (64'd1 << ebits) - 64'd1;
    ovf = (ei > emax) || ((comb >> (p - 1)) >= maxexp);
    use_inf = (rm == RM_RNE) || (rm == RM_RMM) || (rm == RM_RDN && sign) || (rm == RM_RUP && !sign);

    if (ovf) begin
      if (use_inf) r.bits = (64'(sign) << (ebits + p - 1)) | (maxexp << (p - 1));
      else         r.bits = (64'(sign) << (ebits + p - 1)) | ((maxexp - 64'd1) << (p - 1)) | ((64'd1 << (p - 1)) - 64'd1);
    end else begin
      r.bits = (64'(sign) << (ebits + p - 1)) | comb;
    end
    r.of = ovf;
    r.nx = ix | ovf;
    r.uf = tiny_after & ix & ~ovf;
    return r;
  endfunction

  // binary64 value (any class) -> format `fmt`, rounding mode rm.  NaN input -> canonical.
  function automatic rnd_t f64_to_fmt(input logic [63:0] v, input logic [1:0] fmt, input logic [2:0] rm);
    up_t  u;
    rnd_t r;
    u = unpack(v, F64, 1'b0);
    r = '0;
    if (u.nan)       r.bits = nan_bits(fmt);
    else if (u.inf)  r.bits = inf_bits(u.sign, fmt);
    else if (u.zero) r.bits = zero_bits(u.sign, fmt);
    else             r = rnd_pack(u.sign, {u.e[12], u.e}, {u.m, 2'b00}, 1'b0, fmt, rm);
    return r;
  endfunction

  // FTZ: a subnormal result becomes a signed zero with UF|NX.
  function automatic rnd_t ftz_flush(input rnd_t r, input logic [1:0] fmt);
    rnd_t x;
    logic is_sub;
    x = r;
    case (fmt)
      F16:     is_sub = (r.bits[14:10] == 5'd0)  && (r.bits[9:0]  != 10'd0);
      F32:     is_sub = (r.bits[30:23] == 8'd0)  && (r.bits[22:0] != 23'd0);
      default: is_sub = (r.bits[62:52] == 11'd0) && (r.bits[51:0] != 52'd0);
    endcase
    if (is_sub) begin
      case (fmt)
        F16:     x.bits = {48'd0, r.bits[15], 15'd0};
        F32:     x.bits = {32'd0, r.bits[31], 31'd0};
        default: x.bits = {r.bits[63], 63'd0};
      endcase
      x.uf = 1'b1;
      x.nx = 1'b1;
    end
    return x;
  endfunction

  // exact int -> binary64
  function automatic logic [63:0] itof(input int v);
    logic        sgn;
    logic [31:0] mag;
    int unsigned lz;
    logic [63:0] sh;
    sgn = (v < 0);
    mag = sgn ? 32'(-v) : 32'(v);
    if (mag == 32'd0) return 64'd0;
    lz = clz128({mag, 96'd0});
    sh = {mag, 32'd0} << lz;           // MSB at bit 63
    return {sgn, 11'(1023 + 31 - int'(lz)), sh[62:11]};
  endfunction

  // ---------------------------------------------------------------- rne split
  // x (binary64, finite) -> n = rne(x) and r = x - n (exact).  r = +0 when x == n.
  // |x| >= 2^52 is already an integer (r = +0, n meaningless).
  function automatic split_t rne_split(input logic [63:0] x);
    split_t       s;
    up_t          u;
    int           ei, sh;
    logic [53:0]  mm, ip, frac, half, one_sh, nmag, vmag;
    logic         up;
    int unsigned  lz;
    logic [53:0]  m54;
    logic         rsign;
    logic [10:0]  er;
    int           nn;
    u = unpack(x, F64, 1'b0);
    s.r = 64'd0;
    s.n = 12'sd0;
    ei = int'($signed(u.e));
    if (u.zero || u.nan || u.inf) begin
      s.r = 64'd0;
    end else if (ei >= 52) begin
      s.r = 64'd0;
    end else if (ei < -1) begin
      s.r = x;
    end else begin
      sh     = 52 - ei;                      // 1..53
      mm     = {1'b0, u.m};
      ip     = mm >> sh;
      one_sh = 54'd1 << sh;
      frac   = mm & (one_sh - 54'd1);
      half   = 54'd1 << (sh - 1);
      up     = (frac > half) || (frac == half && ip[0]);
      nmag   = ip + 54'(up);
      vmag   = up ? (one_sh - frac) : frac;
      if (vmag != 54'd0) begin
        lz    = clz128({vmag, 74'd0});       // leading zeros inside the 54-bit field
        m54   = vmag << (lz - 1);            // MSB at bit 52 (vmag <= 2^52)
        rsign = u.sign ^ up;
        er    = 11'(1023 + (53 - int'(lz)) - sh);
        s.r   = {rsign, er, m54[51:0]};
      end
      nn  = u.sign ? -int'(nmag[10:0]) : int'(nmag[10:0]);
      s.n = 12'(nn);
    end
    return s;
  endfunction

  // ---------------------------------------------------------------- polynomial tables
  // sel: 0 = exp2 (14), 1 = log2 (12), 2 = sin (9), 3 = cos (10); constants of sfu_math.h
  function automatic logic [63:0] coef(input logic [1:0] sel, input int idx);
    logic [63:0] c;
    c = 64'd0;
    case (sel)
      2'd0: case (idx)
        0: c = 64'h3ff0000000000000;  1: c = 64'h3fe62e42fefa39ef;  2: c = 64'h3fcebfbdff82c58f;
        3: c = 64'h3fac6b08d704a0c0;  4: c = 64'h3f83b2ab6fba4e77;  5: c = 64'h3f55d87fe78a6731;
        6: c = 64'h3f2430912f86c787;  7: c = 64'h3eeffcbfc588b0c7;  8: c = 64'h3eb62c0223a5c824;
        9: c = 64'h3e7b5253d395e7c4; 10: c = 64'h3e3e4cf5158b8eca; 11: c = 64'h3dfe8cac7351bb25;
       12: c = 64'h3dbc3bd650fc2986; 13: c = 64'h3d7816193166d0f9;
        default: c = 64'd0;
      endcase
      2'd1: case (idx)
        0: c = 64'h40071547652b82fe;  1: c = 64'h3feec709dc3a03fd;  2: c = 64'h3fe2776c50ef9bfe;
        3: c = 64'h3fda61762a7aded9;  4: c = 64'h3fd484b13d7c02a9;  5: c = 64'h3fd0c9a84994022d;
        6: c = 64'h3fcc68f568d31760;  7: c = 64'h3fc89f3b1694cffe;  8: c = 64'h3fc5b9ac9b743f0d;
        9: c = 64'h3fc3703c1f4d0ffe; 10: c = 64'h3fc1964ec6fc9491; 11: c = 64'h3fc00ecd7e080215;
        default: c = 64'd0;
      endcase
      2'd2: case (idx)
        0: c = 64'h3ff921fb54442d18;  1: c = 64'hbfe4abbce625be53;  2: c = 64'h3fb466bc6775aae2;
        3: c = 64'hbf732d2cce62bd86;  4: c = 64'h3f250783487ee782;  5: c = 64'hbece3074fde8871f;
        6: c = 64'h3e6e8f434d018d63;  7: c = 64'hbe06fadb9f155744;  8: c = 64'h3d9aaec32af93359;
        default: c = 64'd0;
      endcase
      default: case (idx)
        0: c = 64'h3ff0000000000000;  1: c = 64'hbff3bd3cc9be45de;  2: c = 64'h3fd03c1f081b5ac4;
        3: c = 64'hbf955d3c7e3cbffa;  4: c = 64'h3f4e1f506891babb;  5: c = 64'hbefa6d1f2a204a8c;
        6: c = 64'h3e9f9d38a3763cc3;  7: c = 64'hbe3b6e24f44b128f;  8: c = 64'h3dd20c62c2f2d7f5;
        9: c = 64'hbd62a0c591af8314;
        default: c = 64'd0;
      endcase
    endcase
    return c;
  endfunction

endpackage
