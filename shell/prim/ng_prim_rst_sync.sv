// ng_prim_rst_sync.sv -- reset synchronizer: asynchronous assertion,
// synchronous deassertion after STAGES clocks.  Behavioral model.
//
// FANOUT_STAGE=1 adds one more plain flop after the ASYNC_REG chain
// (asynchronous assertion, deassertion one clock later) with MAX_FANOUT so
// synthesis replicates it.  For a reset that feeds hundreds of pins spread over
// several SLRs: the ASYNC_REG chain flops cannot be replicated or moved, and a
// single one driving ~730 pins across an SLR crossing costs 3.2 ns of a 4 ns
// period (build28/regr: the only violating path family).  The copies are
// placed next to their loads instead.
module ng_prim_rst_sync #(
  parameter int STAGES = 3,
  parameter bit FANOUT_STAGE = 1'b0
) (
  input  logic clk,
  input  logic arst_n,
  output logic rst_n
);
  (* ASYNC_REG = "TRUE" *) logic [STAGES-1:0] s;
  always_ff @(posedge clk or negedge arst_n) begin
    if (!arst_n) s <= '0;
    else         s <= {s[STAGES-2:0], 1'b1};
  end

  if (FANOUT_STAGE) begin : g_fo
    (* MAX_FANOUT = 24 *) logic q;
    always_ff @(posedge clk or negedge arst_n) begin
      if (!arst_n) q <= 1'b0;
      else         q <= s[STAGES-1];
    end
    assign rst_n = q;
  end else begin : g_nofo
    assign rst_n = s[STAGES-1];
  end
endmodule
