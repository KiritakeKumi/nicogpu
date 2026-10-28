// ng_valu_mul.sv -- one lane's registered 32x32 multiplier.
//
// Signed/unsigned selectable per operand (33x33 signed multiply with the sign
// bit zeroed for unsigned operands).  The exact product of two 32-bit values
// always fits 64 bits (signed: +-2^62; unsigned: < 2^64), so the low 64 bits
// are the full result.  One pipeline register (p): written on `en`, written
// for every instruction class that needs a product (mul*, mac, mad*, mulw*).
module ng_valu_mul (
  input  logic        clk,
  input  logic        en,
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic        sa,
  input  logic        sb,
  output logic [63:0] p
);
  logic signed [32:0] ea, eb;
  logic signed [65:0] full;
  always_comb begin
    ea   = $signed({sa & a[31], a});
    eb   = $signed({sb & b[31], b});
    full = ea * eb;
  end
  always_ff @(posedge clk) if (en) p <= full[63:0];
endmodule
