// ng_bitops_pkg.sv -- bit-manipulation helpers shared by the SP units (pure functions, no state).
//
//   clz32 / ctz32 / cpop32 / brev32 / bfm_mask   the model's int_semantics.cpp bit ops (SALU and VALU lanes): clz / ctz
//                                                return 32 for a zero input; bfm_mask(width, offset) uses width[5:0]
//                                                (>= 32: all ones) and offset[4:0]
//   lowest_set(m)                                index of the lowest set bit of a vector of up to 64 bits (0 when none
//                                                is set: callers that need "any" test |m themselves); callers
//                                                zero-extend their vector and truncate the result to their index width
//   chk_vreg(is_vec, is_pair, r, n)              the model's register range check of one vector operand
//                                                (ExecCtx::vreg): {fault, first out-of-range register}, the high half
//                                                r + 1 of a pair checked after r; {0, 0} for non-vector operands
package ng_bitops_pkg;

  function automatic logic [5:0] lowest_set(input logic [63:0] m);
    logic [5:0] r;
    r = '0;
    for (int i = 63; i >= 0; i--) if (m[i]) r = 6'(i);
    return r;
  endfunction

  function automatic logic [31:0] clz32(input logic [31:0] v);
    logic [5:0] n;
    n = 6'd32;
    for (int i = 0; i < 32; i++) if (v[i]) n = 6'(31 - i);
    return {26'd0, n};
  endfunction

  function automatic logic [31:0] ctz32(input logic [31:0] v);
    return (v == '0) ? 32'd32 : {26'd0, lowest_set({32'd0, v})};
  endfunction

  function automatic logic [31:0] cpop32(input logic [31:0] v);
    logic [5:0] n;
    n = '0;
    for (int i = 0; i < 32; i++) n = n + 6'(v[i]);
    return {26'd0, n};
  endfunction

  function automatic logic [31:0] brev32(input logic [31:0] v);
    logic [31:0] r;
    for (int i = 0; i < 32; i++) r[i] = v[31 - i];
    return r;
  endfunction

  function automatic logic [31:0] bfm_mask(input logic [31:0] width, input logic [31:0] offset);
    logic [31:0] m;
    m = width[5] ? 32'hffffffff : ((32'd1 << width[4:0]) - 32'd1);
    return m << offset[4:0];
  endfunction

  // r < 128 (7-bit register index zero-extended), so r + 1 does not wrap
  function automatic logic [8:0] chk_vreg(input logic is_vec, input logic is_pair, input logic [7:0] r,
                                          input logic [7:0] n);
    logic [8:0] c;
    c = '0;
    if (is_vec) begin
      if (r >= n)                         c = {1'b1, r};
      else if (is_pair && r + 8'd1 >= n)  c = {1'b1, r + 8'd1};
    end
    return c;
  endfunction

endpackage : ng_bitops_pkg
