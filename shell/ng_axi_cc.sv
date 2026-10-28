// ng_axi_cc.sv -- parameterized AXI4 asynchronous clock converter (P0-11).
//
// Seven of these sit in the static region (BAR0, BAR2, host, and the four
// MIG channels), crossing clk_sh <-> clk_l2_s or clk_l2_s <-> ui_clk.  It is
// a straight channel-by-channel async-FIFO crossing on top of
// ng_prim_fifo_async (Gray pointers): no reordering, IDs and beat counts
// pass through, so the ordering guarantees of DFX_BOUNDARY.md 1.2 survive
// the crossing.  The Xilinx axi_clock_converter was considered and rejected
// to keep one parameterized, simulation-friendly module instead of seven
// pre-generated IP variants (the prior project measured that IP at 417 LUT
// for a 256-bit port; this design is in the same ballpark).
//
// Reset handling: each side takes its own asynchronous-active-low reset,
// and each side's reset is also synchronized INTO the other side and ORed
// in, so a reset on either clock domain flushes both sides of every FIFO
// (Gray pointers only stay consistent if both sides see a reset).
module ng_axi_cc #(
  parameter int ID_W   = 4,
  parameter int ADDR_W = 28,
  parameter int DATA_W = 64,
  parameter int DEPTH  = 16,      // per-channel FIFO depth (power of two)
  parameter int STRB_W = DATA_W / 8,
  // wide data channels (W, R) use the block-RAM, registered-read FIFO
  parameter bit WIDE_SYNC_RD = (DATA_W >= 256)
) (
  // s side (clock s_aclk)
  input  logic              s_aclk,
  input  logic              s_aresetn,
  input  logic [ID_W-1:0]   s_axi_awid,
  input  logic [ADDR_W-1:0] s_axi_awaddr,
  input  logic [7:0]        s_axi_awlen,
  input  logic [2:0]        s_axi_awsize,
  input  logic [1:0]        s_axi_awburst,
  input  logic [3:0]        s_axi_awcache,
  input  logic [2:0]        s_axi_awprot,
  input  logic [3:0]        s_axi_awqos,
  input  logic              s_axi_awvalid,
  output logic              s_axi_awready,
  input  logic [DATA_W-1:0] s_axi_wdata,
  input  logic [STRB_W-1:0] s_axi_wstrb,
  input  logic              s_axi_wlast,
  input  logic              s_axi_wvalid,
  output logic              s_axi_wready,
  output logic [ID_W-1:0]   s_axi_bid,
  output logic [1:0]        s_axi_bresp,
  output logic              s_axi_bvalid,
  input  logic              s_axi_bready,
  input  logic [ID_W-1:0]   s_axi_arid,
  input  logic [ADDR_W-1:0] s_axi_araddr,
  input  logic [7:0]        s_axi_arlen,
  input  logic [2:0]        s_axi_arsize,
  input  logic [1:0]        s_axi_arburst,
  input  logic [3:0]        s_axi_arcache,
  input  logic [2:0]        s_axi_arprot,
  input  logic [3:0]        s_axi_arqos,
  input  logic              s_axi_arvalid,
  output logic              s_axi_arready,
  output logic [ID_W-1:0]   s_axi_rid,
  output logic [DATA_W-1:0] s_axi_rdata,
  output logic [1:0]        s_axi_rresp,
  output logic              s_axi_rlast,
  output logic              s_axi_rvalid,
  input  logic              s_axi_rready,
  // m side (clock m_aclk)
  input  logic              m_aclk,
  input  logic              m_aresetn,
  output logic [ID_W-1:0]   m_axi_awid,
  output logic [ADDR_W-1:0] m_axi_awaddr,
  output logic [7:0]        m_axi_awlen,
  output logic [2:0]        m_axi_awsize,
  output logic [1:0]        m_axi_awburst,
  output logic [3:0]        m_axi_awcache,
  output logic [2:0]        m_axi_awprot,
  output logic [3:0]        m_axi_awqos,
  output logic              m_axi_awvalid,
  input  logic              m_axi_awready,
  output logic [DATA_W-1:0] m_axi_wdata,
  output logic [STRB_W-1:0] m_axi_wstrb,
  output logic              m_axi_wlast,
  output logic              m_axi_wvalid,
  input  logic              m_axi_wready,
  input  logic [ID_W-1:0]   m_axi_bid,
  input  logic [1:0]        m_axi_bresp,
  input  logic              m_axi_bvalid,
  output logic              m_axi_bready,
  output logic [ID_W-1:0]   m_axi_arid,
  output logic [ADDR_W-1:0] m_axi_araddr,
  output logic [7:0]        m_axi_arlen,
  output logic [2:0]        m_axi_arsize,
  output logic [1:0]        m_axi_arburst,
  output logic [3:0]        m_axi_arcache,
  output logic [2:0]        m_axi_arprot,
  output logic [3:0]        m_axi_arqos,
  output logic              m_axi_arvalid,
  input  logic              m_axi_arready,
  input  logic [ID_W-1:0]   m_axi_rid,
  input  logic [DATA_W-1:0] m_axi_rdata,
  input  logic [1:0]        m_axi_rresp,
  input  logic              m_axi_rlast,
  input  logic              m_axi_rvalid,
  output logic              m_axi_rready
);

  localparam int AW_W = ID_W + ADDR_W + 8 + 3 + 2 + 4 + 3 + 4;   // aw payload
  localparam int W_W  = DATA_W + STRB_W + 1;                     // w payload
  localparam int B_W  = ID_W + 2;                                // b payload
  localparam int AR_W = AW_W;
  localparam int R_W  = ID_W + DATA_W + 2 + 1;                   // r payload

  // cross-coupled resets: each side resets if its own reset asserts or the
  // other side's reset (synchronized over) asserts
  logic s_rst_n, m_rst_n, s_rst_q, m_rst_q;
  ng_prim_sync #(.W(1)) u_rst_m2s (.clk(s_aclk), .rst_n(s_aresetn), .d(!m_aresetn), .q(m_rst_q));
  ng_prim_sync #(.W(1)) u_rst_s2m (.clk(m_aclk), .rst_n(m_aresetn), .d(!s_aresetn), .q(s_rst_q));
  assign s_rst_n = s_aresetn && !m_rst_q;
  assign m_rst_n = m_aresetn && !s_rst_q;

  // ---------------------------------------------------------------- AW ------
  logic [AW_W-1:0] aw_din, aw_dout;
  assign aw_din = {s_axi_awqos, s_axi_awprot, s_axi_awcache, s_axi_awburst,
                   s_axi_awsize, s_axi_awlen, s_axi_awaddr, s_axi_awid};
  logic aw_full, aw_empty, aw_push, aw_pop;
  assign aw_push = s_axi_awvalid && !aw_full;
  assign s_axi_awready = !aw_full;
  logic aw_sk_rdy;
  assign aw_pop  = !aw_empty && aw_sk_rdy;
  ng_prim_skid #(.W(AW_W)) u_sk_aw (
    .clk(m_aclk), .rst_n(m_rst_n),
    .s_valid(!aw_empty), .s_ready(aw_sk_rdy), .s_data(aw_dout),
    .m_valid(m_axi_awvalid), .m_ready(m_axi_awready),
    .m_data({m_axi_awqos, m_axi_awprot, m_axi_awcache, m_axi_awburst,
             m_axi_awsize, m_axi_awlen, m_axi_awaddr, m_axi_awid}), .occ());

  ng_prim_fifo_async #(.W(AW_W), .DEPTH(DEPTH)) u_fifo_aw (
    .wclk(s_aclk), .wrst_n(s_rst_n), .push(aw_push), .din(aw_din),
    .full(aw_full), .wcount(),
    .rclk(m_aclk), .rrst_n(m_rst_n), .pop(aw_pop), .dout(aw_dout), .empty(aw_empty));

  // ----------------------------------------------------------------- W -------
  logic [W_W-1:0] w_din, w_dout;
  assign w_din = {s_axi_wlast, s_axi_wstrb, s_axi_wdata};
  logic w_full, w_empty, w_push, w_pop;
  assign w_push = s_axi_wvalid && !w_full;
  assign s_axi_wready = !w_full;
  logic w_sk_rdy;
  assign w_pop  = !w_empty && w_sk_rdy;
  ng_prim_skid #(.W(W_W)) u_sk_w (
    .clk(m_aclk), .rst_n(m_rst_n),
    .s_valid(!w_empty), .s_ready(w_sk_rdy), .s_data(w_dout),
    .m_valid(m_axi_wvalid), .m_ready(m_axi_wready),
    .m_data({m_axi_wlast, m_axi_wstrb, m_axi_wdata}), .occ());

  ng_prim_fifo_async #(.W(W_W), .DEPTH(DEPTH), .SYNC_RD(WIDE_SYNC_RD)) u_fifo_w (
    .wclk(s_aclk), .wrst_n(s_rst_n), .push(w_push), .din(w_din),
    .full(w_full), .wcount(),
    .rclk(m_aclk), .rrst_n(m_rst_n), .pop(w_pop), .dout(w_dout), .empty(w_empty));

  // ----------------------------------------------------------------- B -------
  logic [B_W-1:0] b_din, b_dout;
  assign b_din = {m_axi_bresp, m_axi_bid};
  logic b_full, b_empty, b_push, b_pop;
  assign b_push = m_axi_bvalid && !b_full;
  assign m_axi_bready = !b_full;
  logic b_sk_rdy;
  assign b_pop  = !b_empty && b_sk_rdy;
  ng_prim_skid #(.W(B_W)) u_sk_b (
    .clk(s_aclk), .rst_n(s_rst_n),
    .s_valid(!b_empty), .s_ready(b_sk_rdy), .s_data(b_dout),
    .m_valid(s_axi_bvalid), .m_ready(s_axi_bready),
    .m_data({s_axi_bresp, s_axi_bid}), .occ());

  ng_prim_fifo_async #(.W(B_W), .DEPTH(DEPTH)) u_fifo_b (
    .wclk(m_aclk), .wrst_n(m_rst_n), .push(b_push), .din(b_din),
    .full(b_full), .wcount(),
    .rclk(s_aclk), .rrst_n(s_rst_n), .pop(b_pop), .dout(b_dout), .empty(b_empty));

  // ---------------------------------------------------------------- AR -------
  logic [AR_W-1:0] ar_din, ar_dout;
  assign ar_din = {s_axi_arqos, s_axi_arprot, s_axi_arcache, s_axi_arburst,
                   s_axi_arsize, s_axi_arlen, s_axi_araddr, s_axi_arid};
  logic ar_full, ar_empty, ar_push, ar_pop;
  assign ar_push = s_axi_arvalid && !ar_full;
  assign s_axi_arready = !ar_full;
  logic ar_sk_rdy;
  assign ar_pop  = !ar_empty && ar_sk_rdy;
  ng_prim_skid #(.W(AR_W)) u_sk_ar (
    .clk(m_aclk), .rst_n(m_rst_n),
    .s_valid(!ar_empty), .s_ready(ar_sk_rdy), .s_data(ar_dout),
    .m_valid(m_axi_arvalid), .m_ready(m_axi_arready),
    .m_data({m_axi_arqos, m_axi_arprot, m_axi_arcache, m_axi_arburst,
             m_axi_arsize, m_axi_arlen, m_axi_araddr, m_axi_arid}), .occ());

  ng_prim_fifo_async #(.W(AR_W), .DEPTH(DEPTH)) u_fifo_ar (
    .wclk(s_aclk), .wrst_n(s_rst_n), .push(ar_push), .din(ar_din),
    .full(ar_full), .wcount(),
    .rclk(m_aclk), .rrst_n(m_rst_n), .pop(ar_pop), .dout(ar_dout), .empty(ar_empty));

  // ----------------------------------------------------------------- R -------
  logic [R_W-1:0] r_din, r_dout;
  assign r_din = {m_axi_rlast, m_axi_rresp, m_axi_rdata, m_axi_rid};
  logic r_full, r_empty, r_push, r_pop;
  assign r_push = m_axi_rvalid && !r_full;
  assign m_axi_rready = !r_full;
  logic r_sk_rdy;
  assign r_pop  = !r_empty && r_sk_rdy;
  ng_prim_skid #(.W(R_W)) u_sk_r (
    .clk(s_aclk), .rst_n(s_rst_n),
    .s_valid(!r_empty), .s_ready(r_sk_rdy), .s_data(r_dout),
    .m_valid(s_axi_rvalid), .m_ready(s_axi_rready),
    .m_data({s_axi_rlast, s_axi_rresp, s_axi_rdata, s_axi_rid}), .occ());

  ng_prim_fifo_async #(.W(R_W), .DEPTH(DEPTH), .SYNC_RD(WIDE_SYNC_RD)) u_fifo_r (
    .wclk(m_aclk), .wrst_n(m_rst_n), .push(r_push), .din(r_din),
    .full(r_full), .wcount(),
    .rclk(s_aclk), .rrst_n(s_rst_n), .pop(r_pop), .dout(r_dout), .empty(r_empty));

endmodule
