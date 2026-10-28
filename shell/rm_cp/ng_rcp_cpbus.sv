// ng_rcp_cpbus.sv -- clk_ts side of the CP register bridge of the CP RM.
//
// Receives the request that ng_rcp_regs (clk_l2) holds stable behind a toggling `req`, performs it on the ng_cp_regs
// host bus for exactly one clk_ts cycle (host_wr / host_rd with host_addr), captures the read data in that cycle and
// answers by setting `ack_tog` to the value of the request toggle (so the clk_l2 side sees ack == req).  The data
// words (addr/wdata/wr and the returned rdata) cross the domains as quasi-static values: the sender does not
// change them while the handshake is open, and the toggle is synchronised before they are sampled.
module ng_rcp_cpbus (
  input  logic        clk,            // clk_ts
  input  logic        rst_n,          // rst_ts_n
  input  logic        req_tog_i,      // from clk_l2 (asynchronous)
  input  logic        wr_i,
  input  logic [12:0] addr_i,
  input  logic [31:0] wdata_i,
  output logic        ack_tog_o,      // to clk_l2 (synchronised there)
  output logic [31:0] rdata_o,
  // ng_cp_regs host bus
  output logic        host_wr,
  output logic        host_rd,
  output logic [12:0] host_addr,
  output logic [31:0] host_wdata,
  input  logic [31:0] host_rdata
);
  (* ASYNC_REG = "TRUE" *) logic r1, r2;
  logic r3, busy;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r1 <= 1'b0; r2 <= 1'b0; r3 <= 1'b0; busy <= 1'b0;
      host_wr <= 1'b0; host_rd <= 1'b0; host_addr <= '0; host_wdata <= '0;
      ack_tog_o <= 1'b0; rdata_o <= '0;
    end else begin
      r1 <= req_tog_i; r2 <= r1; r3 <= r2;
      if (busy) begin
        // the strobe cycle: capture before the register block applies the write
        rdata_o   <= host_rdata;
        host_wr   <= 1'b0;
        host_rd   <= 1'b0;
        ack_tog_o <= r2;
        busy      <= 1'b0;
      end else if (r2 != r3) begin
        host_addr  <= addr_i;
        host_wdata <= wdata_i;
        host_wr    <= wr_i;
        host_rd    <= ~wr_i;
        busy       <= 1'b1;
      end
    end
  end
endmodule
