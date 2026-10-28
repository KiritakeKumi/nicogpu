// ng_fp64_fn_pkg.sv -- bit-exact FP64 helper functions of the FP64 unit.
//
// Pure functions mirroring the model's fp_util.h helpers and the SoftFloat
// (RISC-V specialisation) routines that the fp64 semantics call: class /
// fclass / min / max / compare, roundToInt, binary64 -> i32/u32 (RTZ),
// i32/u32 -> binary64, frexp.  Flags are in fflags order {NV,DZ,OF,UF,NX}.
// Unpack / round-and-pack come from ng_sfu_fp_pkg (shared unchanged).
package ng_fp64_fn_pkg;

  localparam logic [63:0] F64_BELOW_ONE  = 64'h3fef_ffff_ffff_ffff;
  localparam logic [63:0] F64_QNAN_BITS  = 64'h7ff8_0000_0000_0000;
  localparam logic [4:0]  FLG_NV = 5'b10000;
  localparam logic [4:0]  FLG_NX = 5'b00001;
  localparam logic [4:0]  FLG_UF = 5'b00010;

  typedef struct packed {
    logic [63:0] bits;
    logic [4:0]  fl;
  } fr_t;

  // --------------------------------------------------------------
  function automatic logic f_exp_max(input logic [63:0] x);
    return x[62:52] == 11'h7ff;
  endfunction
  function automatic logic f_nan(input logic [63:0] x);
    return (x[62:52] == 11'h7ff) && (x[51:0] != 52'd0);
  endfunction
  function automatic logic f_snan(input logic [63:0] x);
    return f_nan(x) && !x[51];
  endfunction
  function automatic logic f_inf(input logic [63:0] x);
    return x[62:0] == {11'h7ff, 52'd0};
  endfunction
  function automatic logic f_zero(input logic [63:0] x);
    return x[62:0] == 63'd0;
  endfunction
  function automatic logic f_sub(input logic [63:0] x);
    return (x[62:52] == 11'd0) && (x[51:0] != 52'd0);
  endfunction

  // DAZ input flush: subnormal (and zero) -> signed zero
  function automatic logic [63:0] daz64(input logic [63:0] x, input logic en);
    return (en && x[62:52] == 11'd0) ? {x[63], 63'd0} : x;
  endfunction

  // RISC-V fclass: [0] -inf [1] -normal [2] -sub [3] -0 [4] +0 [5] +sub [6] +normal [7] +inf [8] sNaN [9] qNaN
  function automatic logic [9:0] fclass64(input logic [63:0] x);
    logic [9:0] r;
    logic       neg;
    neg = x[63];
    r = 10'd0;
    if (f_nan(x))       r = f_snan(x) ? 10'h100 : 10'h200;
    else if (f_inf(x))  r = neg ? 10'h001 : 10'h080;
    else if (f_zero(x)) r = neg ? 10'h008 : 10'h010;
    else if (f_sub(x))  r = neg ? 10'h004 : 10'h020;
    else                r = neg ? 10'h002 : 10'h040;
    return r;
  endfunction

  // unsigned key whose order is the total order of non-NaN values with -0 < +0
  function automatic logic [63:0] order_key(input logic [63:0] x);
    return x[63] ? ~x : (x | 64'h8000_0000_0000_0000);
  endfunction

  // IEEE minimumNumber / maximumNumber with RISC-V details (fp_util.h min_num / max_num)
  function automatic fr_t minmax64(input logic [63:0] a, input logic [63:0] b, input logic is_max);
    fr_t r;
    r.fl = (f_snan(a) || f_snan(b)) ? FLG_NV : 5'd0;
    if (f_nan(a))      r.bits = f_nan(b) ? F64_QNAN_BITS : b;
    else if (f_nan(b)) r.bits = a;
    else if (is_max)   r.bits = (order_key(a) >= order_key(b)) ? a : b;
    else               r.bits = (order_key(a) <= order_key(b)) ? a : b;
    return r;
  endfunction

  // compare predicate (LT=1 EQ=2 GT=4 UN=8 SIG=16); bits[0] = result
  function automatic fr_t fcmp64(input logic [63:0] a, input logic [63:0] b, input logic [4:0] p);
    fr_t        r;
    logic [3:0] rel;
    r = '0;
    if (f_nan(a) || f_nan(b)) begin
      if (p[4] || f_snan(a) || f_snan(b)) r.fl = FLG_NV;
      r.bits = {63'd0, p[3]};
    end else begin
      if (a == b || (f_zero(a) && f_zero(b))) rel = 4'd2;
      else if (order_key(a) < order_key(b))   rel = 4'd1;
      else                                    rel = 4'd4;
      r.bits = {63'd0, |(p[2:0] & rel[2:0])};
    end
    return r;
  endfunction

  // ---------------------------------------------------------------- roundToInt (exact = false)
  // dir = SoftFloat rounding enum: 0 near_even, 1 minMag, 2 min, 3 max, 4 near_maxMag
  function automatic fr_t round_to_int64(input logic [63:0] x, input logic [2:0] dir);
    fr_t          r;
    int           e;
    logic [63:0]  uz, last, rmask;
    r = '0;
    e = int'(x[62:52]);
    if (e <= 'h3fe) begin
      if (x[62:0] == 63'd0) begin
        r.bits = x;
      end else begin
        uz = {x[63], 63'd0};
        case (dir)
          3'd0:    begin if (x[51:0] != 52'd0 && e == 'h3fe) uz = uz | {1'b0, 11'h3ff, 52'd0}; end
          3'd4:    begin if (e == 'h3fe) uz = uz | {1'b0, 11'h3ff, 52'd0}; end
          3'd2:    begin if (uz != 64'd0) uz = {1'b1, 11'h3ff, 52'd0}; end
          3'd3:    begin if (uz == 64'd0) uz = {1'b0, 11'h3ff, 52'd0}; end
          default: ;
        endcase
        r.bits = uz;
      end
    end else if (e >= 'h433) begin
      if (f_nan(x)) begin
        r.bits = F64_QNAN_BITS;
        r.fl   = f_snan(x) ? FLG_NV : 5'd0;
      end else begin
        r.bits = x;
      end
    end else begin
      uz    = x;
      last  = 64'd1 << (13'h433 - e);
      rmask = last - 64'd1;
      if (dir == 3'd4) begin
        uz = uz + (last >> 1);
      end else if (dir == 3'd0) begin
        uz = uz + (last >> 1);
        if ((uz & rmask) == 64'd0) uz = uz & ~last;
      end else if (dir == (x[63] ? 3'd2 : 3'd3)) begin
        uz = uz + rmask;
      end
      uz = uz & ~rmask;
      r.bits = uz;
    end
    return r;
  endfunction

  // ---------------------------------------------------------------- binary64 -> i32 / u32 (RTZ, exact flags)
  // RISC-V specialisation: NaN -> max, overflow saturates, invalid raises NV, inexact NX.
  function automatic fr_t f64_to_i32_rtz(input logic [63:0] x, input logic is_unsigned);
    fr_t          r;
    logic         sign;
    int           e, sd;
    logic [63:0]  sig;
    logic [63:0]  lostm;
    logic         lost, inval;
    logic [31:0]  z32, zz;
    logic [11:0]  rb;
    r = '0;
    sign = x[63];
    e    = int'(x[62:52]);
    if (f_nan(x)) sign = 1'b0;
    sig = (e != 0) ? {11'd0, 1'b1, x[51:0]} : {11'd0, 1'b0, x[51:0]};
    sd = 'h427 - e;
    if (sd > 0) begin
      if (sd >= 64) begin
        sig = {63'd0, (sig != 64'd0)};
      end else begin
        lostm = (64'd1 << sd) - 64'd1;
        lost  = |(sig & lostm);
        sig   = (sig >> sd) | {63'd0, lost};
      end
    end
    rb    = sig[11:0];
    inval = (sig[63:44] != 20'd0);
    z32   = sig[43:12];
    if (!is_unsigned) begin
      zz = sign ? (32'd0 - z32) : z32;
      if (!inval && zz != 32'd0 && (zz[31] != sign)) inval = 1'b1;
      if (inval) begin
        r.bits = {32'd0, sign ? 32'h8000_0000 : 32'h7fff_ffff};
        r.fl   = FLG_NV;
      end else begin
        r.bits = {32'd0, zz};
        r.fl   = (rb != 12'd0) ? FLG_NX : 5'd0;
      end
    end else begin
      if (sign && sig == 64'd0) begin
        r.bits = 64'd0;
      end else begin
        if (!inval && sign && z32 != 32'd0) inval = 1'b1;
        if (inval) begin
          r.bits = {32'd0, sign ? 32'h0000_0000 : 32'hffff_ffff};
          r.fl   = FLG_NV;
        end else begin
          r.bits = {32'd0, z32};
          r.fl   = (rb != 12'd0) ? FLG_NX : 5'd0;
        end
      end
    end
    return r;
  endfunction

  // ---------------------------------------------------------------- 32-bit integer -> binary64 (exact)
  function automatic logic [63:0] i32_to_f64(input logic [31:0] v, input logic is_unsigned);
    logic        sgn;
    logic [31:0] mag;
    int unsigned lz;
    logic [63:0] sh;
    logic [31:0] t;
    sgn = !is_unsigned && v[31];
    mag = sgn ? (32'd0 - v) : v;
    lz  = 0;
    t   = mag;
    if (mag == 32'd0) return 64'd0;
    for (int i = 31; i >= 0; i--) begin
      if (mag[i]) begin lz = 31 - i; break; end
    end
    sh = {mag, 32'd0} << lz;               // MSB at bit 63
    return {sgn, 11'(1023 + 31 - int'(lz)), sh[62:11]};
  endfunction

endpackage
