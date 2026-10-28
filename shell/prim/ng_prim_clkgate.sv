// ng_prim_clkgate.sv -- glitch-free clock gate.  Behavioral latch-based model
// (ASIC: integrated clock-gating cell).  FPGA mapping: BUFGCE in the static
// region; the RM never instantiates clock buffers (DFX_BOUNDARY.md).
module ng_prim_clkgate (
  input  logic clk_i,
  input  logic en,
  output logic clk_o
);
  logic en_l;
  always_latch begin
    if (!clk_i) en_l = en;
  end
  assign clk_o = clk_i & en_l;
endmodule
