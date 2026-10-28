// Placeholder DUT used ONLY to prove tb_rcp_dma compiles/elaborates when the real ng_rcp_dma.sv is absent.
// Behaviour: every command completes immediately with err=1. Do not synthesize.
module ng_rcp_dma #(parameter int FIFO_LOG2 = 9) (
  input  logic clk, rst_n,
  input  logic start, op_fill, input logic [3:0] vmid, input logic [47:0] src, dst, input logic [63:0] size,
  input  logic [31:0] fill, input logic abort,
  output logic busy, done, err,
  output logic t_req_valid, input logic t_req_ready, output logic [3:0] t_req_vmid, output logic [63:0] t_req_va,
  output logic [1:0] t_req_acc, input logic t_resp_valid, output logic t_resp_ready, input logic [2:0] t_resp_fault,
  input  logic t_resp_sparse, input logic [47:0] t_resp_pa, input logic [1:0] t_resp_level, input logic [63:0] t_resp_pte,
  output logic ef_valid, output logic [3:0] ef_vmid, output logic [63:0] ef_va, output logic [1:0] ef_acc,
  output logic [2:0] ef_code, output logic [1:0] ef_level, output logic [63:0] ef_pte,
  // host master (DW 64, AW 47)
  output logic h_awvalid, input logic h_awready, output logic [46:0] h_awaddr, output logic [7:0] h_awlen,
  output logic h_wvalid, input logic h_wready, output logic [63:0] h_wdata, output logic [7:0] h_wstrb, output logic h_wlast,
  input  logic h_bvalid, output logic h_bready, input logic [1:0] h_bresp,
  output logic h_arvalid, input logic h_arready, output logic [46:0] h_araddr, output logic [7:0] h_arlen,
  input  logic h_rvalid, output logic h_rready, input logic [63:0] h_rdata, input logic [1:0] h_rresp, input logic h_rlast,
  // C3 master (DW 128, AW 31)
  output logic c_awvalid, input logic c_awready, output logic [30:0] c_awaddr, output logic [7:0] c_awlen,
  output logic c_wvalid, input logic c_wready, output logic [127:0] c_wdata, output logic [15:0] c_wstrb, output logic c_wlast,
  input  logic c_bvalid, output logic c_bready, input logic [1:0] c_bresp,
  output logic c_arvalid, input logic c_arready, output logic [30:0] c_araddr, output logic [7:0] c_arlen,
  input  logic c_rvalid, output logic c_rready, input logic [127:0] c_rdata, input logic [1:0] c_rresp, input logic c_rlast
);
  always_ff @(posedge clk) begin
    if (!rst_n) begin busy <= 1'b0; done <= 1'b0; err <= 1'b0; end
    else begin done <= start && !busy; err <= 1'b1; busy <= 1'b0; end
  end
  assign t_req_valid = 1'b0; assign t_req_vmid = '0; assign t_req_va = '0; assign t_req_acc = '0; assign t_resp_ready = 1'b0;
  assign ef_valid = 1'b0; assign ef_vmid = '0; assign ef_va = '0; assign ef_acc = '0; assign ef_code = '0;
  assign ef_level = '0; assign ef_pte = '0;
  assign h_awvalid = 1'b0; assign h_awaddr = '0; assign h_awlen = '0; assign h_wvalid = 1'b0; assign h_wdata = '0;
  assign h_wstrb = '0; assign h_wlast = 1'b0; assign h_bready = 1'b1; assign h_arvalid = 1'b0; assign h_araddr = '0;
  assign h_arlen = '0; assign h_rready = 1'b1;
  assign c_awvalid = 1'b0; assign c_awaddr = '0; assign c_awlen = '0; assign c_wvalid = 1'b0; assign c_wdata = '0;
  assign c_wstrb = '0; assign c_wlast = 1'b0; assign c_bready = 1'b1; assign c_arvalid = 1'b0; assign c_araddr = '0;
  assign c_arlen = '0; assign c_rready = 1'b1;
endmodule
