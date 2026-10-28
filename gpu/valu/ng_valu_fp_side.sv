// ng_valu_fp_side.sv -- combinational "side" results of one f32 VALU lane.
//
// min/max/med3, sign injection, compares, fclass, round-to-integer
// (floor/ceil/trunc/rndne), fsat, f32 <-> integer conversions towards the integer
// side, byte conversions, frexp; with EN_F16  the binary16 <-> binary32
// conversions fcvt.f32.f16.lo/hi, fcvt.f16.f32.lo/hi(.m), pkcvt.f16.f32(.m).  Operands
// arrive conditioned (neg/abs modifiers and DAZ already applied where the model applies
// them; the binary16 conversions get raw operands and the MODE bits daz32 / daz16 /
// ftz16).  Semantics: and fp_semantics.cpp.
// fl = {NV, DZ, OF, UF, NX}.
module ng_valu_fp_side
  import ng_valu_fp_pkg::*;
#(
  parameter bit EN_F16 = 1'b0     // binary16 conversions; 0: those micro-ops give 0
) (
  input  logic [5:0]  fop,
  input  logic [4:0]  pred,
  input  logic [2:0]  rm,        // effective rounding mode (used by the .m conversions)
  input  logic [1:0]  ub,        // fcvt.f32.ubyteN byte index
  input  logic        daz32,     // MODE.DAZ32 (binary16 conversions only)
  input  logic        daz16,     // MODE.DAZ16
  input  logic        ftz16,     // MODE.FTZ16
  input  logic [31:0] ra,
  input  logic [31:0] rb,
  input  logic [31:0] rc,
  output logic [31:0] res,
  output logic [4:0]  fl,
  output logic        cbit
);

  // --------------------------------------------------------------
  // minimumNumber / maximumNumber, result bits only
  function automatic logic [31:0] min_val(input logic [31:0] a, input logic [31:0] b);
    if (f_nan(a)) return f_nan(b) ? F32_QNAN : b;
    if (f_nan(b)) return a;
    return (f_okey(a) <= f_okey(b)) ? a : b;
  endfunction
  function automatic logic [31:0] max_val(input logic [31:0] a, input logic [31:0] b);
    if (f_nan(a)) return f_nan(b) ? F32_QNAN : b;
    if (f_nan(b)) return a;
    return (f_okey(a) >= f_okey(b)) ? a : b;
  endfunction
  function automatic logic [31:0] med3_val(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c);
    logic [31:0] lo, hi, mid;
    if (f_nan(a) || f_nan(b) || f_nan(c)) return min_val(min_val(a, b), c);
    lo  = (f_okey(a) <= f_okey(b)) ? a : b;
    hi  = (f_okey(a) <= f_okey(b)) ? b : a;
    mid = (f_okey(hi) <= f_okey(c)) ? hi : c;
    return (f_okey(lo) >= f_okey(mid)) ? lo : mid;
  endfunction

  // fcmp: {nv, result}
  function automatic logic [1:0] fcmp_f(input logic [31:0] a, input logic [31:0] b, input logic [4:0] p);
    logic nan, eq, lt, gt, nv, r;
    nan = f_nan(a) || f_nan(b);
    eq  = (a == b) || (f_zero(a) && f_zero(b));
    lt  = !eq && (f_okey(a) < f_okey(b));
    gt  = !eq && !lt;
    if (nan) begin
      nv = p[P_SIG] || f_snan(a) || f_snan(b);
      r  = p[P_UN];
    end else begin
      nv = 1'b0;
      r  = (p[P_LT] && lt) || (p[P_EQ] && eq) || (p[P_GT] && gt);
    end
    return {nv, r};
  endfunction

  function automatic logic [9:0] fclass_f(input logic [31:0] x);
    logic [9:0] c;
    logic s;
    s = x[31];
    c = 10'd0;
    if (f_nan(x))                 c = f_snan(x) ? 10'h100 : 10'h200;
    else if (f_inf(x))            c = s ? 10'h001 : 10'h080;
    else if (f_zero(x))           c = s ? 10'h008 : 10'h010;
    else if (x[30:23] == 8'd0)    c = s ? 10'h004 : 10'h020;
    else                          c = s ? 10'h002 : 10'h040;
    return c;
  endfunction

  // shiftRightJam64
  function automatic logic [63:0] jam64(input logic [63:0] v, input int sh);
    logic [63:0] r, msk;
    if (sh <= 0) return v;
    if (sh >= 64) return {63'd0, (v != 64'd0)};
    msk = (64'd1 << sh) - 64'd1;
    r = v >> sh;
    r[0] = r[0] | (|(v & msk));
    return r;
  endfunction

  // f32 -> i32 / u32 with rounding mode m, exact: {nv, nx, bits}
  function automatic logic [33:0] cvt_f2i(input logic [31:0] x, input logic [2:0] m, input logic uns);
    logic        sg, nv, nx, early;
    logic [7:0]  e;
    logic [23:0] m24;
    logic [63:0] sig, sig2;
    logic [11:0] rbits, inc;
    logic [31:0] z, s32;
    logic        nanx;
    int          sd;
    sg = x[31]; e = x[30:23];
    nanx = (e == 8'hff) && (x[22:0] != 23'd0);
    if (nanx) sg = 1'b0;
    m24 = (e != 8'd0) ? {1'b1, x[22:0]} : {1'b0, x[22:0]};
    sig = {8'd0, m24, 32'd0};
    sd  = 170 - int'(e);
    if (sd > 0) sig = jam64(sig, sd);
    nv = 1'b0; nx = 1'b0; early = 1'b0; z = 32'd0;
    inc = 12'h800;
    if (m != RM_RMM && m != RM_RNE) begin
      inc = 12'h000;
      if (uns) begin
        if (sg) begin
          if (sig == 64'd0) early = 1'b1;
          else if (m == RM_RDN) nv = 1'b1;
        end else if (m == RM_RUP) inc = 12'hfff;
      end else begin
        if (sg ? (m == RM_RDN) : (m == RM_RUP)) inc = 12'hfff;
      end
    end
    if (!early && !nv) begin
      rbits = sig[11:0];
      sig2  = sig + {52'd0, inc};
      if (sig2[63:44] != 20'd0) nv = 1'b1;
      s32 = sig2[43:12];
      if (rbits == 12'h800 && m == RM_RNE) s32[0] = 1'b0;
      if (uns) begin
        z = s32;
        if (sg && (z != 32'd0)) nv = 1'b1;
      end else begin
        z = sg ? (32'd0 - s32) : s32;
        if ((z != 32'd0) && (z[31] ^ sg)) nv = 1'b1;
      end
      if (!nv && rbits != 12'd0) nx = 1'b1;
    end
    if (nv) z = uns ? (sg ? 32'd0 : 32'hffffffff) : (sg ? 32'h80000000 : 32'h7fffffff);
    return {nv, nx, z};
  endfunction

  // --------------------------------------------------------------
  logic [31:0] a, b, c;
  logic [32:0] ri;           // round-to-integer result of the selected mode
  logic [2:0]  ri_m, cv_m;
  logic        cv_u;
  logic [33:0] cv;           // f32 -> integer conversion of the selected flavour
  logic [1:0]  cmpr;
  logic [31:0] minv, maxv, med3v;
  logic [7:0]  ubyte_v;
  logic [31:0] ub_res;
  logic [31:0] frm, fre;
  logic [4:0]  ubmsb;
  logic [20:0] h_lo, h_hi;   // binary16 conversion results {fl, h}
  logic [32:0] f_h2f;
  logic [31:0] h_a, h_b;

  always_comb begin
    a = ra; b = rb; c = rc;
    // one shared round-to-integer / conversion datapath, mode selected by the opcode
    case (fop)
      FO_FLOOR: ri_m = RM_RDN;
      FO_CEIL:  ri_m = RM_RUP;
      FO_TRUNC: ri_m = RM_RTZ;
      default:  ri_m = RM_RNE;
    endcase
    ri   = round_int(a, ri_m);
    cv_m = (fop == FO_CVTFIM || fop == FO_CVTFUM) ? rm : RM_RTZ;
    cv_u = (fop == FO_CVTFU || fop == FO_CVTFUM);
    cv   = cvt_f2i(a, cv_m, cv_u);
    cmpr   = fcmp_f(a, b, pred);
    minv   = min_val(a, b);
    maxv   = max_val(a, b);
    med3v  = med3_val(a, b, c);

    // fcvt.f32.ubyteN: exact
    ubyte_v = a[8*ub +: 8];
    ubmsb   = 5'd0;
    for (int i = 0; i < 8; i++) if (ubyte_v[i]) ubmsb = 5'(i);
    ub_res = 32'd0;
    if (ubyte_v != 8'd0)
      ub_res = {1'b0, 8'(127 + int'(ubmsb)), 23'(({15'd0, ubyte_v} << (23 - int'(ubmsb))))};

    // frexp (raw/ DAZ'd operand a)
    frm = a; fre = 32'd0;
    if (a[30:23] != 8'hff && !f_zero(a)) begin
      logic [4:0] lz;
      logic [23:0] v24;
      logic [23:0] nm;
      v24 = {1'b0, a[22:0]};
      lz  = clz24(v24);
      nm  = v24 << lz;
      if (a[30:23] == 8'd0) begin
        frm = {a[31], 8'd126, nm[22:0]};
        fre = 32'(-126 - int'(lz) + 1);
      end else begin
        frm = {a[31], 8'd126, a[22:0]};
        fre = 32'(int'(a[30:23]) - 126);
      end
    end

    // binary16 conversions: one f32 -> f16 converter per half (pkcvt uses both),
    // one f16 -> f32 converter on the half selected by the opcode
    h_a   = daz32 ? f_flush(a) : a;
    h_b   = daz32 ? f_flush(b) : b;
    h_lo  = cvt_f2h(h_a, rm, ftz16);
    h_hi  = cvt_f2h(h_b, rm, ftz16);
    f_h2f = cvt_h2f((fop == FO_H2FH) ? a[31:16] : a[15:0], daz16);

    res = 32'd0; fl = 5'd0; cbit = 1'b0;
    case (fop)
      FO_MIN:  begin res = minv; fl[FL_NV] = f_snan(a) | f_snan(b); end
      FO_MAX:  begin res = maxv; fl[FL_NV] = f_snan(a) | f_snan(b); end
      FO_MED3: begin res = med3v; fl[FL_NV] = f_snan(a) | f_snan(b) | f_snan(c); end
      FO_SGNJ:  res = {b[31], a[30:0]};
      FO_SGNJN: res = {~b[31], a[30:0]};
      FO_SGNJX: res = {a[31] ^ b[31], a[30:0]};
      FO_FEQ, FO_FLT, FO_FLE: begin res = {31'd0, cmpr[0]}; fl[FL_NV] = cmpr[1]; end
      FO_CLASS: res = {22'd0, fclass_f(a)};
      FO_FLOOR, FO_CEIL, FO_TRUNC, FO_RNDNE: begin res = ri[31:0]; fl[FL_NV] = ri[32]; end
      FO_SAT: begin
        fl[FL_NV] = f_snan(a);
        res = (f_nan(a) || a[31]) ? 32'd0 : (a >= F32_ONE ? F32_ONE : a);
      end
      FO_CVTFI, FO_CVTFU, FO_CVTFIM, FO_CVTFUM: begin res = cv[31:0]; fl[FL_NV] = cv[33]; fl[FL_NX] = cv[32]; end
      FO_UBYTE:  res = ub_res;
      FO_FREXPM: res = frm;
      FO_FREXPE: res = fre;
      FO_CMP:    begin cbit = cmpr[0]; fl[FL_NV] = cmpr[1]; end
      FO_CMPCLS: cbit = (({22'd0, fclass_f(a)} & b) != 32'd0);
      FO_H2FL, FO_H2FH: if (EN_F16) begin res = f_h2f[31:0]; fl[FL_NV] = f_h2f[32]; end
      FO_F2HL:  if (EN_F16) begin res = {c[31:16], h_lo[15:0]}; fl = h_lo[20:16]; end
      FO_F2HH:  if (EN_F16) begin res = {h_lo[15:0], c[15:0]}; fl = h_lo[20:16]; end
      FO_PKF2H: if (EN_F16) begin res = {h_hi[15:0], h_lo[15:0]}; fl = h_lo[20:16] | h_hi[20:16]; end
      default: ;
    endcase
  end
endmodule
