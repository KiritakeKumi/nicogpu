// ng_prim_clkdiv.sv -- clock buffer with enable and integer divide (1 or 2),
// phase aligned to its input.  FPGA mapping: BUFGCE_DIV when DIV > 1 or
// DIV_BUF is set, otherwise BUFGCE (an MMCM can only feed 4 BUFGCE_DIVs in
// its clock region).  Copies of one MMCM output that must stay phase
// aligned use the same buffer type: clk_core2x sets DIV_BUF to match the
// divided clk_core; the gated clk_l2 and the static clk_l2_s are both
// BUFGCE.  CE is CE_TYPE SYNC (UltraScale+ has no HARDSYNC).  A fabric flop
// on another global copy of the same clock cannot meet the CE setup at
// 400 MHz, and it need not: DFX_BOUNDARY 2.4 only gates/ungates while the
// RM is quiesced, isolated and in reset, so a runt edge from a late CE is
// harmless.  shell.xdc therefore cuts the CE pins.  ASIC mapping: ICG +
// divider cell.
module ng_prim_clkdiv #(
  parameter int DIV     = 1,
  parameter bit DIV_BUF = 1'b0
) (
  input  logic clk_i,
  input  logic en,
  output logic clk_o
);
`ifdef SYNTHESIS
  if (DIV > 1 || DIV_BUF) begin : g_bufgce_div
    BUFGCE_DIV #(.BUFGCE_DIVIDE(DIV)) u_bufg (
      .I   (clk_i),
      .CE  (en),
      .CLR (1'b0),
      .O   (clk_o)
    );
  end else begin : g_bufgce
    BUFGCE u_bufg (
      .I  (clk_i),
      .CE (en),
      .O  (clk_o)
    );
  end
`else
  // behavioral model (simulation)
  logic clk_g;
  ng_prim_clkgate u_gate (.clk_i(clk_i), .en(en), .clk_o(clk_g));
  if (DIV == 1) begin : g_div1
    assign clk_o = clk_g;
  end else begin : g_div2
    logic t = 1'b0;  // simulation only; real divider cells come up in a defined phase
    always_ff @(posedge clk_g) t <= ~t;
    assign clk_o = t;
  end
`endif
endmodule
