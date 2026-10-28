// ng_rmt_port.sv -- boundary stage of one AXI4 master port of the test RM.
//
// * every boundary output comes from a flip-flop (skids on AW/W/AR, the
//   ready of the B/R skids) or is a constant tied at the top level
// * every boundary input reaches a flip-flop through at most one LUT
//   (B/R through ng_rmt_islice; *READY inputs only into the output skids)
// * quiesce: no new AW/AR enters the port while `quiesce` is 1; W beats of
//   already issued bursts and all responses keep flowing
// * W is never presented before its AW has been accepted by the slave: a W
//   burst may enter the W skid only after a (registered) AW handshake credit
// * outstanding limits MAX_WR (AW..B) / MAX_RD (AR..RLAST), counted from the
//   moment the address enters the port; idle = nothing outstanding
module ng_rmt_port #(
  parameter int DW     = 64,
  parameter int AW     = 32,
  parameter int MAX_WR = 32,
  parameter int MAX_RD = 32
) (
  input  logic            clk,
  input  logic            rst_n,
  input  logic            quiesce,
  output logic            idle,
  // internal side
  input  logic            s_awvalid,
  output logic            s_awready,
  input  logic [AW-1:0]   s_awaddr,
  input  logic [7:0]      s_awlen,
  input  logic            s_wvalid,
  output logic            s_wready,
  input  logic [DW-1:0]   s_wdata,
  input  logic [DW/8-1:0] s_wstrb,
  input  logic            s_wlast,
  output logic            s_bvalid,
  input  logic            s_bready,
  output logic [1:0]      s_bresp,
  input  logic            s_arvalid,
  output logic            s_arready,
  input  logic [AW-1:0]   s_araddr,
  input  logic [7:0]      s_arlen,
  output logic            s_rvalid,
  input  logic            s_rready,
  output logic [DW-1:0]   s_rdata,
  output logic [1:0]      s_rresp,
  output logic            s_rlast,
  // boundary side
  output logic [AW-1:0]   m_awaddr,
  output logic [7:0]      m_awlen,
  output logic            m_awvalid,
  input  logic            m_awready,
  output logic [DW-1:0]   m_wdata,
  output logic [DW/8-1:0] m_wstrb,
  output logic            m_wlast,
  output logic            m_wvalid,
  input  logic            m_wready,
  input  logic [1:0]      m_bresp,
  input  logic            m_bvalid,
  output logic            m_bready,
  output logic [AW-1:0]   m_araddr,
  output logic [7:0]      m_arlen,
  output logic            m_arvalid,
  input  logic            m_arready,
  input  logic [DW-1:0]   m_rdata,
  input  logic [1:0]      m_rresp,
  input  logic            m_rlast,
  input  logic            m_rvalid,
  output logic            m_rready
);
  localparam int WCW = $clog2(MAX_WR + 1) + 1;
  localparam int RCW = $clog2(MAX_RD + 1) + 1;

  logic           q_q;
  logic [WCW-1:0] wr_out, wcred;
  logic [RCW-1:0] rd_out;
  logic           aw_hs_q;
  logic           aw_sk_ready, w_sk_ready, ar_sk_ready;
  logic           aw_ok, ar_ok;

  assign aw_ok     = !q_q && (wr_out < WCW'(MAX_WR));
  assign ar_ok     = !q_q && (rd_out < RCW'(MAX_RD));
  assign s_awready = aw_ok && aw_sk_ready;
  assign s_arready = ar_ok && ar_sk_ready;
  assign s_wready  = (wcred != '0) && w_sk_ready;
  assign idle      = (wr_out == '0) && (rd_out == '0);

  wire aw_in = s_awvalid && s_awready;
  wire ar_in = s_arvalid && s_arready;
  wire w_in_last = s_wvalid && s_wready && s_wlast;
  wire b_out = s_bvalid && s_bready;
  wire r_out_last = s_rvalid && s_rready && s_rlast;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      q_q     <= 1'b0;
      wr_out  <= '0;
      rd_out  <= '0;
      wcred   <= '0;
      aw_hs_q <= 1'b0;
    end else begin
      q_q     <= quiesce;
      aw_hs_q <= m_awvalid && m_awready;
      wr_out  <= wr_out + WCW'(aw_in) - WCW'(b_out);
      rd_out  <= rd_out + RCW'(ar_in) - RCW'(r_out_last);
      wcred   <= wcred + WCW'(aw_hs_q) - WCW'(w_in_last);
    end
  end

  ng_prim_skid #(.W(AW + 8), .ATTR(1'b1)) u_aw (
    .clk(clk), .rst_n(rst_n),
    .s_valid(s_awvalid && aw_ok), .s_ready(aw_sk_ready), .s_data({s_awaddr, s_awlen}),
    .m_valid(m_awvalid), .m_ready(m_awready), .m_data({m_awaddr, m_awlen}), .occ());

  ng_prim_skid #(.W(DW + DW/8 + 1), .ATTR(1'b1)) u_w (
    .clk(clk), .rst_n(rst_n),
    .s_valid(s_wvalid && (wcred != '0)), .s_ready(w_sk_ready), .s_data({s_wdata, s_wstrb, s_wlast}),
    .m_valid(m_wvalid), .m_ready(m_wready), .m_data({m_wdata, m_wstrb, m_wlast}), .occ());

  ng_prim_skid #(.W(AW + 8), .ATTR(1'b1)) u_ar (
    .clk(clk), .rst_n(rst_n),
    .s_valid(s_arvalid && ar_ok), .s_ready(ar_sk_ready), .s_data({s_araddr, s_arlen}),
    .m_valid(m_arvalid), .m_ready(m_arready), .m_data({m_araddr, m_arlen}), .occ());

  ng_rmt_islice #(.W(2)) u_b (
    .clk(clk), .rst_n(rst_n),
    .s_valid(m_bvalid), .s_ready(m_bready), .s_data(m_bresp),
    .m_valid(s_bvalid), .m_ready(s_bready), .m_data(s_bresp));

  ng_rmt_islice #(.W(DW + 3)) u_r (
    .clk(clk), .rst_n(rst_n),
    .s_valid(m_rvalid), .s_ready(m_rready), .s_data({m_rdata, m_rresp, m_rlast}),
    .m_valid(s_rvalid), .m_ready(s_rready), .m_data({s_rdata, s_rresp, s_rlast}));
endmodule
