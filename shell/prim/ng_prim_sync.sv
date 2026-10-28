// ng_prim_sync.sv -- multi-flop synchronizer for single bits or quasi-static
// buses (each bit synchronized independently; callers must only use it for
// Gray-coded or stable values).  Generic behavioral model; the FPGA flow adds
// ASYNC_REG, an ASIC flow maps it to a library synchronizer cell.
module ng_prim_sync #(
  parameter int W      = 1,
  parameter int STAGES = 2
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [W-1:0] d,
  output logic [W-1:0] q
);
  (* ASYNC_REG = "TRUE" *) logic [W-1:0] s [STAGES];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int i = 0; i < STAGES; i++) s[i] <= '0;
    end else begin
      s[0] <= d;
      for (int i = 1; i < STAGES; i++) s[i] <= s[i-1];
    end
  end
  assign q = s[STAGES-1];
endmodule
