// ng_l2_amo_alu.sv -- L2 atomic ALU (combinational).
//
// tentative semantics, aligned
// atom.* families and the model-side choices in:
//   * operations: add/sub/and/or/xor/min[.i]/max[.i]/min[.u]/max[.u]/xchg/
//     cmpxchg/inc/dec, on 32-bit (size=4) and 64-bit (size=8) data;
//   * inputs are the *value* currently stored at A (zero-extended) and the
//     operand; byte extraction/insertion at unaligned offsets is done by the
//     slice (ng_l2_slice), so this ALU is pure;
//   * cmpxchg failure writes the old value back (write always executes --
//     lsu.md "write-back always executes"; observable memory unchanged,
//     but the write permission side effect stands);
//   * inc/dec are 32-bit only with CUDA wrap semantics:
//       inc: r = (a >= b) ? 0: a+1      (unsigned)
//       dec: r = (a==0 || a > b) ? b: a-1 (unsigned)
//   * FP32 atomics: fadd/fmin/fmax, 32-bit only.
//     Semantics = (SoftFloat, fixed env):
//       fadd: RNE, subnormals kept (no FTZ/DAZ), any NaN result (NaN input,
//             inf-inf) = canonical qNaN 0x7FC00000, no fflags;
//       fmin/fmax: minimumNumber/maximumNumber -- a single NaN operand is
//             ignored, both NaN -> 0x7FC00000, -0 < +0.
//     'a' is the old memory value, 'b' the operand (same as the integer ops).
//   * EN_FP = 0 (the LSU's integer ALU; its FP32 ops run in the pipelined ng_lsu_fatomic): no FP32 ops
//     no single-cycle FP32 adder -- AMO_F* leave 'a' unchanged like any undefined op.

module ng_l2_amo_alu #(
  parameter bit EN_FP = 1
) (
  input  logic [4:0]  op,
  input  logic        is64,     // 1: 64-bit, 0: 32-bit
  input  logic [63:0] a,        // old value at A (zero-extended)
  input  logic [63:0] b,        // operand (AMO data / swap value for CAS)
  input  logic [63:0] cmp,      // compare value (CAS only)
  output logic [63:0] r,        // new value to store at A
  output logic        cas_hit   // CAS compare result (informational)
);
  import ng_noc_pkg::*;   // AMO_F* op codes live there since

  logic [31:0] a32, b32, c32, r32;
  logic [63:0] r_c;
  assign a32 = a[31:0];
  assign b32 = b[31:0];
  assign c32 = cmp[31:0];

  logic [31:0] r_fp;   // FP32 op result (AMO_FADD / AMO_FMIN / AMO_FMAX)
  if (EN_FP) begin : g_fp
    // ---------------- FP32 adder (RNE, subnormals preserved) ----------------
    function automatic logic f32_isinf(input logic [31:0] x);
      return (x[30:23] == 8'hFF) && (x[22:0] == 23'd0);
    endfunction

    function automatic logic [31:0] f32_add(input logic [31:0] x, input logic [31:0] y);
      logic [31:0] big, sml;
      logic        sb, ss, sub;
      logic [8:0]  eb, es, d, e2;       // effective exponents (denormal -> 1)
      logic [26:0] mb, ms, ms_sh;
      logic [27:0] sum;
      logic [4:0]  p, lsh;
      logic        stky;
      logic [23:0] m;
      logic [24:0] mr;
      logic        g, rs, inc, rsign;
      logic [8:0]  ef;
      if (f32_isnan(x) || f32_isnan(y)) return F32_QNAN;
      if (f32_isinf(x) || f32_isinf(y)) begin
        if (f32_isinf(x) && f32_isinf(y) && (x[31] != y[31])) return F32_QNAN;
        return f32_isinf(x) ? x : y;
      end
      if (x[30:0] >= y[30:0]) begin big = x; sml = y; end
      else                    begin big = y; sml = x; end
      sb = big[31];
      ss = sml[31];
      sub = sb ^ ss;
      eb = (big[30:23] == 8'd0) ? 9'd1 : {1'b0, big[30:23]};
      es = (sml[30:23] == 8'd0) ? 9'd1 : {1'b0, sml[30:23]};
      mb = {(big[30:23] != 8'd0), big[22:0], 3'b000};
      ms = {(sml[30:23] != 8'd0), sml[22:0], 3'b000};
      d  = eb - es;
      if (d >= 9'd27) begin
        ms_sh = {26'd0, (ms != 27'd0)};
      end else begin
        ms_sh = ms >> d[4:0];
        stky  = 1'b0;
        for (int i = 0; i < 27; i++)
          if (i < d && ms[i]) stky = 1'b1;
        ms_sh[0] = ms_sh[0] | stky;
      end
      sum = sub ? ({1'b0, mb} - {1'b0, ms_sh}) : ({1'b0, mb} + {1'b0, ms_sh});
      if (sum == 28'd0) begin
        // exact zero: +0 unless both operands are -0 (same sign, zero)
        return {(sb & ss), 31'd0};
      end
      p = 5'd0;
      for (int i = 0; i < 28; i++) if (sum[i]) p = 5'(i);
      if (p == 5'd27) begin
        sum  = {1'b0, sum[27:2], (sum[1] | sum[0])};
        e2   = eb + 9'd1;
      end else begin
        lsh = 5'd26 - p;
        if (9'(lsh) > eb - 9'd1) lsh = 5'(eb - 9'd1);
        sum  = sum << lsh;
        e2   = eb - 9'(lsh);
      end
      m   = sum[26:3];
      g   = sum[2];
      rs  = sum[1] | sum[0];
      inc = g & (rs | m[0]);
      mr  = {1'b0, m} + 25'(inc);
      ef  = e2;
      if (mr[24]) begin       // mantissa carry-out: 0xFFFFFF + 1
        mr = mr >> 1;
        ef = e2 + 9'd1;
      end
      rsign = sb;
      if (ef >= 9'd255) return {rsign, 8'hFF, 23'd0};
      // subnormal result (hidden bit clear) has biased exponent 0; rounding up
      // into the normal range lands on e2 == 1 which is already correct
      return {rsign, (mr[23] ? ef[7:0] : 8'd0), mr[22:0]};
    endfunction

    always_comb begin
      unique case (op)
        AMO_FADD: r_fp = f32_add(a32, b32);
        AMO_FMIN: r_fp = f32_minmax(a32, b32, 1'b0);
        default:  r_fp = f32_minmax(a32, b32, 1'b1);   // AMO_FMAX
      endcase
    end
  end else begin : g_nofp
    assign r_fp = a32;
  end

  always_comb begin
    r_c    = '0;
    r32    = '0;
    cas_hit = 1'b0;
    if (is64) begin
      unique case (op)
        ng_noc_pkg::AMO_ADD:   r_c = a + b;
        ng_noc_pkg::AMO_SUB:   r_c = a - b;
        ng_noc_pkg::AMO_AND:   r_c = a & b;
        ng_noc_pkg::AMO_OR:    r_c = a | b;
        ng_noc_pkg::AMO_XOR:   r_c = a ^ b;
        ng_noc_pkg::AMO_MIN_S: r_c = ($signed(a) < $signed(b)) ? a : b;
        ng_noc_pkg::AMO_MAX_S: r_c = ($signed(a) > $signed(b)) ? a : b;
        ng_noc_pkg::AMO_MIN_U: r_c = (a < b) ? a : b;
        ng_noc_pkg::AMO_MAX_U: r_c = (a > b) ? a : b;
        ng_noc_pkg::AMO_XCHG:  r_c = b;
        ng_noc_pkg::AMO_CAS: begin
          cas_hit = (a == cmp);
          r_c = cas_hit ? b : a;
        end
        default: r_c = a;  // undefined op: no change (slice filters INC/DEC@64)
      endcase
    end else begin
      unique case (op)
        ng_noc_pkg::AMO_ADD:   r32 = a32 + b32;
        ng_noc_pkg::AMO_SUB:   r32 = a32 - b32;
        ng_noc_pkg::AMO_AND:   r32 = a32 & b32;
        ng_noc_pkg::AMO_OR:    r32 = a32 | b32;
        ng_noc_pkg::AMO_XOR:   r32 = a32 ^ b32;
        ng_noc_pkg::AMO_MIN_S: r32 = ($signed(a32) < $signed(b32)) ? a32 : b32;
        ng_noc_pkg::AMO_MAX_S: r32 = ($signed(a32) > $signed(b32)) ? a32 : b32;
        ng_noc_pkg::AMO_MIN_U: r32 = (a32 < b32) ? a32 : b32;
        ng_noc_pkg::AMO_MAX_U: r32 = (a32 > b32) ? a32 : b32;
        ng_noc_pkg::AMO_XCHG:  r32 = b32;
        ng_noc_pkg::AMO_CAS: begin
          cas_hit = (a32 == c32);
          r32 = cas_hit ? b32 : a32;
        end
        ng_noc_pkg::AMO_INC:   r32 = (a32 >= b32) ? 32'd0 : a32 + 32'd1;
        ng_noc_pkg::AMO_DEC:   r32 = (a32 == 32'd0 || a32 > b32) ? b32 : a32 - 32'd1;
        AMO_FADD, AMO_FMIN, AMO_FMAX: r32 = r_fp;
        default: r32 = a32;  // undefined op: no change
      endcase
      r_c = {32'd0, r32};
    end
  end
  assign r = r_c;
endmodule
