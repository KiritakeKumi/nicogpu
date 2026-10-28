// ng_valu_fp_pkg.sv -- f32 VALU building blocks.
//
// Micro-op codes of the floating-point VALU subset and the bit-level helpers
// shared by ng_valu_fp_side / ng_valu_fp_lane.  Semantics mirror
// + fp_semantics.cpp (SoftFloat, RISC-V
// specialisation: canonical quiet NaN 0x7fc00000, tininess detected after
// rounding, DAZ/FTZ from MODE).  Pure functions, no state.
package ng_valu_fp_pkg;

  // ---- micro-op codes (dec_t.fop) ----------------------------------------
  // FMA-datapath class (rounded by the lane's pipeline)
  localparam logic [5:0] FO_NONE   = 6'd0;
  localparam logic [5:0] FO_ADD    = 6'd1;
  localparam logic [5:0] FO_SUB    = 6'd2;
  localparam logic [5:0] FO_SUBR   = 6'd3;
  localparam logic [5:0] FO_MUL    = 6'd4;
  localparam logic [5:0] FO_FMA    = 6'd5;   // ffma / fmac (acc = d)
  localparam logic [5:0] FO_LDEXP  = 6'd6;
  localparam logic [5:0] FO_FRACT  = 6'd7;
  localparam logic [5:0] FO_CVTIF  = 6'd8;   // i32 -> f32
  localparam logic [5:0] FO_CVTUF  = 6'd9;   // u32 -> f32
  // side class (single-stage combinational result, carried through the pipeline)
  localparam logic [5:0] FO_MIN    = 6'd16;
  localparam logic [5:0] FO_MAX    = 6'd17;
  localparam logic [5:0] FO_MED3   = 6'd18;
  localparam logic [5:0] FO_SGNJ   = 6'd19;
  localparam logic [5:0] FO_SGNJN  = 6'd20;
  localparam logic [5:0] FO_SGNJX  = 6'd21;
  localparam logic [5:0] FO_FEQ    = 6'd22;  // vector 0/1 result
  localparam logic [5:0] FO_FLT    = 6'd23;
  localparam logic [5:0] FO_FLE    = 6'd24;
  localparam logic [5:0] FO_CLASS  = 6'd25;
  localparam logic [5:0] FO_FLOOR  = 6'd26;
  localparam logic [5:0] FO_CEIL   = 6'd27;
  localparam logic [5:0] FO_TRUNC  = 6'd28;
  localparam logic [5:0] FO_RNDNE  = 6'd29;
  localparam logic [5:0] FO_SAT    = 6'd30;
  localparam logic [5:0] FO_CVTFI  = 6'd31;  // f32 -> i32, RTZ
  localparam logic [5:0] FO_CVTFU  = 6'd32;  // f32 -> u32, RTZ
  localparam logic [5:0] FO_CVTFIM = 6'd33;  // f32 -> i32, rm operand
  localparam logic [5:0] FO_CVTFUM = 6'd34;
  localparam logic [5:0] FO_UBYTE  = 6'd35;  // fcvt.f32.ubyteN (dec_t.fsub = N)
  localparam logic [5:0] FO_FREXPM = 6'd36;
  localparam logic [5:0] FO_FREXPE = 6'd37;
  localparam logic [5:0] FO_CMP    = 6'd38;  // cmp.*.f32 -> uniform mask bit (dec_t.fpred)
  localparam logic [5:0] FO_CMPCLS = 6'd39;  // cmp.class.f32
  // binary16 <-> binary32 conversions (ng_valu EN_F16CVT); side class
  localparam logic [5:0] FO_H2FL   = 6'd40;  // fcvt.f32.f16.lo: d = (float)s1.l (exact)
  localparam logic [5:0] FO_H2FH   = 6'd41;  // fcvt.f32.f16.hi: d = (float)s1.h
  localparam logic [5:0] FO_F2HL   = 6'd42;  // fcvt.f16.f32.lo(.m): d.l = (half)s1, d.h kept (c = old d)
  localparam logic [5:0] FO_F2HH   = 6'd43;  // fcvt.f16.f32.hi(.m): d.h = (half)s1, d.l kept
  localparam logic [5:0] FO_PKF2H  = 6'd44;  // pkcvt.f16.f32(.m): d = {(half)s2, (half)s1}

  // ---- compare predicate bits (fp_util.h pred:) -----------------------------
  localparam int P_LT = 0, P_EQ = 1, P_GT = 2, P_UN = 3, P_SIG = 4;

  // rounding-mode codes (rm operand codes = SoftFloat enum)
  localparam logic [2:0] RM_RNE = 3'd0, RM_RTZ = 3'd1, RM_RDN = 3'd2, RM_RUP = 3'd3, RM_RMM = 3'd4;

  localparam logic [31:0] F32_QNAN = 32'h7fc00000;
  localparam logic [31:0] F32_ONE  = 32'h3f800000;

  // fflags bits
  localparam int FL_NX = 0, FL_UF = 1, FL_OF = 2, FL_DZ = 3, FL_NV = 4;

  // ---- per-op properties ------------------------------------------------------
  // instruction constructs an FpScope (reserved rm -> IllegalInsn, flags, trap enables)
  function automatic logic fop_scope(input logic [5:0] f);
    case (f)
      FO_SGNJ, FO_SGNJN, FO_SGNJX, FO_CLASS, FO_UBYTE, FO_FREXPE, FO_CMPCLS: return 1'b0;
      default: return (f != FO_NONE);
    endcase
  endfunction
  // inputs are DAZ-flushed (fp.in32) when MODE.DAZ32 is set
  function automatic logic fop_daz(input logic [5:0] f);
    case (f)
      FO_ADD, FO_SUB, FO_SUBR, FO_MUL, FO_FMA, FO_LDEXP, FO_FRACT,
      FO_MIN, FO_MAX, FO_MED3, FO_FEQ, FO_FLT, FO_FLE,
      FO_FLOOR, FO_CEIL, FO_TRUNC, FO_RNDNE, FO_SAT, FO_CVTFI, FO_CVTFU, FO_CVTFIM, FO_CVTFUM,
      FO_FREXPM, FO_CMP: return 1'b1;
      default: return 1'b0;
    endcase
  endfunction
  // result goes through the FMA pipeline (rounded; FTZ applies)
  function automatic logic fop_fma(input logic [5:0] f);
    return (f >= FO_ADD) && (f <= FO_CVTUF);
  endfunction
  // binary16 result: the FpScope is F16 -- rounding mode MODE.rm16 (7 = follow MODE.rm) unless an rm
  // operand overrides it (model sem_fp.h).  The binary16 conversions apply DAZ / FTZ themselves
  // (fop_daz = 0): DAZ32 on f32 sources, DAZ16 on f16 sources, FTZ16 on f16 results.
  function automatic logic fop_rm16(input logic [5:0] f);
    return (f == FO_F2HL) || (f == FO_F2HH) || (f == FO_PKF2H);
  endfunction

  // ---- IEEE bit helpers ------------------------------------------------------------
  function automatic logic f_nan(input logic [31:0] x);
    return (x[30:23] == 8'hff) && (x[22:0] != 23'd0);
  endfunction
  function automatic logic f_snan(input logic [31:0] x);
    return f_nan(x) && !x[22];
  endfunction
  function automatic logic f_inf(input logic [31:0] x);
    return x[30:0] == 31'h7f800000;
  endfunction
  function automatic logic f_zero(input logic [31:0] x);
    return x[30:0] == 31'd0;
  endfunction
  function automatic logic [31:0] f_flush(input logic [31:0] x);
    return (x[30:23] == 8'd0) ? {x[31], 31'd0} : x;
  endfunction
  // integer key whose unsigned order is the IEEE order of non-NaN values, -0 < +0
  function automatic logic [31:0] f_okey(input logic [31:0] x);
    return x[31] ? ~x : (x | 32'h80000000);
  endfunction

  // clz over 24 bits (24 for zero)
  function automatic logic [4:0] clz24(input logic [23:0] v);
    logic [4:0] n;
    n = 5'd24;
    for (int i = 0; i < 24; i++) if (v[i]) n = 5'(23 - i);
    return n;
  endfunction
  // clz over 52 bits (52 for zero)
  function automatic logic [5:0] clz52(input logic [51:0] v);
    logic [5:0] n;
    n = 6'd52;
    for (int i = 0; i < 52; i++) if (v[i]) n = 6'(51 - i);
    return n;
  endfunction

  // roundToInt (exact = false) with a fixed rounding mode: {nv, bits}
  function automatic logic [32:0] round_int(input logic [31:0] x, input logic [2:0] m);
    logic [7:0]  e;
    logic        sg;
    logic [22:0] fr;
    logic [31:0] z, last, rbm;
    logic        nv;
    logic [4:0]  sh;
    e = x[30:23]; sg = x[31]; fr = x[22:0];
    z = x; nv = 1'b0;
    if (e <= 8'h7e) begin
      if (x[30:0] != 31'd0) begin
        z = {sg, 31'd0};
        case (m)
          RM_RNE: if (fr != 23'd0 && e == 8'h7e) z = {sg, 8'h7f, 23'd0};
          RM_RDN: if (sg) z = 32'hbf800000;
          RM_RUP: if (!sg) z = 32'h3f800000;
          default: ;
        endcase
      end
    end else if (e >= 8'h96) begin
      if (e == 8'hff && fr != 23'd0) begin
        z = F32_QNAN; nv = !x[22];
      end
    end else begin
      sh   = 5'(8'h96 - e);
      last = 32'd1 << sh;
      rbm  = last - 32'd1;
      case (m)
        RM_RNE: begin
          z = x + (last >> 1);
          if ((z & rbm) == 32'd0) z = z & ~last;
        end
        RM_RDN: if (sg)  z = x + rbm;
        RM_RUP: if (!sg) z = x + rbm;
        default: ;
      endcase
      z = z & ~rbm;
    end
    return {nv, z};
  endfunction

  // ---- binary16 <-> binary32  ------------------------------------------------
  // binary32 -> binary16 with rounding mode m (SoftFloat f32_to_f16 + softfloat_roundPackToF16,
  // RISC-V specialisation: default NaN 0x7e00, NV for sNaN, tininess after rounding).  x is the
  // DAZ32-conditioned source; FTZ16 (ftz) flushes a subnormal result to signed zero with UF|NX
  // (fp.out16).  Returns {fl[4:0], h[15:0]}, fl = {NV, DZ, OF, UF, NX}.
  function automatic logic [20:0] cvt_f2h(input logic [31:0] x, input logic [2:0] m, input logic ftz);
    logic        sg, tiny, rne;
    logic [7:0]  e;
    logic [22:0] fr;
    logic [14:0] sig;      // 1.10 + 4 round bits (bit 0 sticky), hidden bit at 14
    logic [15:0] sri;      // sig + round increment
    logic [3:0]  rinc, rbits;
    logic signed [9:0] ex; // binary16 biased exponent - 1
    logic [6:0]  sh;
    logic [14:0] smsk;
    logic [11:0] s2;
    logic [15:0] h;
    logic [4:0]  fl;
    sg = x[31]; e = x[30:23]; fr = x[22:0];
    fl = 5'd0;
    rne = (m == RM_RNE);
    rinc = 4'h8;
    if (m != RM_RNE && m != RM_RMM) rinc = (m == (sg ? RM_RDN : RM_RUP)) ? 4'hf : 4'h0;
    sig = {1'b1, fr[22:10], fr[9] | (|fr[8:0])};
    ex  = $signed({2'b00, e}) - 10'sd113;
    h   = 16'd0;
    if (e == 8'hff) begin
      if (fr != 23'd0) begin h = 16'h7e00; fl[FL_NV] = !fr[22]; end
      else             h = {sg, 15'h7c00};
    end else if (e == 8'd0 && sig[13:0] == 14'd0) begin
      h = {sg, 15'd0};
    end else begin
      sri = {1'b0, sig} + {12'd0, rinc};
      if (ex > 10'sd29 || (ex == 10'sd29 && sri[15])) begin
        fl[FL_OF] = 1'b1; fl[FL_NX] = 1'b1;
        h = (rinc == 4'h0) ? {sg, 15'h7bff} : {sg, 15'h7c00};
      end else begin
        if (ex < 10'sd0) begin
          tiny = (ex < -10'sd1) || !sri[15];
          // shiftRightJam32(sig, -ex)
          sh = (ex < -10'sd31) ? 7'd31 : 7'(-ex);
          if (sh >= 7'd15) begin
            sig = {14'd0, 1'b1};      // sig != 0 always here
          end else begin
            smsk = (15'd1 << sh[3:0]) - 15'd1;
            sig  = (sig >> sh[3:0]) | {14'd0, |(sig & smsk)};
          end
          ex = 10'sd0;
          if (tiny && sig[3:0] != 4'd0) fl[FL_UF] = 1'b1;
        end
        rbits = sig[3:0];
        s2 = 12'(({1'b0, sig} + {12'd0, rinc}) >> 4);
        if (rbits != 4'd0) fl[FL_NX] = 1'b1;
        if (rne && rbits == 4'h8) s2[0] = 1'b0;
        h = {sg, 15'd0} + {1'b0, ex[4:0], 10'd0} + {4'd0, s2};
        if (s2 == 12'd0) h = {sg, 15'd0};
      end
    end
    // FTZ16: subnormal result -> signed zero, UF | NX
    if (ftz && h[14:10] == 5'd0 && h[9:0] != 10'd0) begin
      h = {h[15], 15'd0};
      fl[FL_UF] = 1'b1; fl[FL_NX] = 1'b1;
    end
    return {fl, h};
  endfunction

  // binary16 -> binary32, exact (SoftFloat f16_to_f32: canonical NaN, NV for sNaN).  DAZ16 (daz)
  // flushes a subnormal source to signed zero without a flag.  The result is never subnormal (FTZ32
  // has nothing to do).  Returns {nv, f[31:0]}.
  function automatic logic [32:0] cvt_h2f(input logic [15:0] x, input logic daz);
    logic        sg;
    logic [4:0]  e;
    logic [9:0]  fr, nm;
    logic [3:0]  lz;
    sg = x[15]; e = x[14:10]; fr = x[9:0];
    if (daz && e == 5'd0) fr = 10'd0;
    if (e == 5'h1f) begin
      if (fr != 10'd0) return {!fr[9], F32_QNAN};
      return {1'b0, sg, 8'hff, 23'd0};
    end
    if (e == 5'd0) begin
      if (fr == 10'd0) return {1'b0, sg, 31'd0};
      lz = 4'd9;
      for (int i = 0; i < 10; i++) if (fr[i]) lz = 4'(9 - i);
      nm = 10'(fr << (lz + 4'd1));
      return {1'b0, sg, 8'(8'd112 - {4'd0, lz}), nm, 13'd0};
    end
    return {1'b0, sg, 8'({3'd0, e} + 8'd112), fr, 13'd0};
  endfunction

endpackage
