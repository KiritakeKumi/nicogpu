// ng_gpu_rm_test.sv -- test reconfigurable module (rm_kind 0x8001).
//
// Same port list as every RM (ng_gpu_rm_ports.svh); a different RM variant
// of module ng_gpu_rm.  Spec:; implementation notes
// tentative decisions:
//
// Floorplan intent (partition spans three SLRs):
//   SLR1: s_axil_ctrl registers, BAR2 adapter, DMA engine, m_axi_host, C3 unit
//   SLR2: C1 channel unit      SLR0: C2 and C4 channel units
// The SLR1 logic reaches each channel unit only through narrow (64-bit data)
// links with SLR_PIPE register stages per direction; BIST data generation /
// checking and the width conversion sit next to their channel port.
// Single clock domain clk_l2 / rst_l2_n; clk_core, clk_core2x, clk_ts unused.
module ng_gpu_rm
  import ng_dfx_pkg::*;
#(
  parameter logic [31:0] BUILD_ID = 32'h0000_0000,
  parameter logic [15:0] RM_REV   = 16'h0001,
  parameter int          SLR_PIPE = 2,
  parameter int          MUL_ST   = 4
) (
`include "ng_gpu_rm_ports.svh"
);
  localparam logic [15:0] RM_KIND_TEST = 16'h8001;
  localparam logic [3:0]  MEM_MASK     = 4'hF;
  localparam logic [1:0]  K_BW = 2'd0, K_BR = 2'd1, K_DW = 2'd2, K_DR = 2'd3;

  wire clk = clk_l2;

  // ----------------------------------------------------------------
  // rst_l2_n is registered once here (SLR1 logic) and reaches every channel
  // unit through its own SLR_PIPE-stage kept delay line (cross-SLR reset
  // fan-out is pipelined like the data links).  rm_ready rises only after
  // every unit is out of reset.
  (* keep = "true", shreg_extract = "no" *) logic rst_n;
  logic [3:0] rst_ch_n;
  always_ff @(posedge clk) rst_n <= rst_l2_n;
  for (genvar n = 0; n < 4; n++) begin : g_rst
    ng_rmt_dly #(.W(1), .N(SLR_PIPE), .RST(1'b0), .KEEP(1'b1)) u_rst (
      .clk(clk), .rst_n(1'b1), .d(rst_n), .q(rst_ch_n[n]));
  end

  // ----------------------------------------------------------------
  assign rm_info = {12'h000, MEM_MASK, RM_REV,           // dw3: reserved, mem_mask, rm_rev
                    BUILD_ID,                            // dw2
                    RM_KIND_TEST, NG_DFX_IFACE_VERSION,  // dw1
                    NG_RM_INFO_MAGIC};                   // dw0

  // ------------------------------------------------------------------ quiesce / ready
  localparam int QAGE = 2 * SLR_PIPE + 8;   // > round trip of the quiesce/qidle pipes
  logic       q_in;
  logic [4:0] q_age;
  logic [3:0] ch_qidle;
  logic       host_idle;
  logic [4:0] rdy_cnt;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rm_ready       <= 1'b0;
      rm_quiesce_ack <= 1'b0;
      q_in           <= 1'b0;
      q_age          <= '0;
      rdy_cnt        <= '0;
    end else begin
      if (rdy_cnt != 5'(SLR_PIPE + 4)) rdy_cnt <= rdy_cnt + 1'b1;
      rm_ready <= (rdy_cnt == 5'(SLR_PIPE + 4));
      q_in     <= shell_quiesce_req;
      if (!q_in)                     q_age <= '0;
      else if (q_age != 5'(QAGE))    q_age <= q_age + 1'b1;
      rm_quiesce_ack <= q_in && (q_age == 5'(QAGE)) && (&ch_qidle) && host_idle;
    end
  end

  // ----------------------------------------------------------------
  logic [3:0]  rq_valid, rs_valid;
  logic [41:0] rq_data;
  logic [31:0] rs_data [4];
  logic        d_wr;
  logic [3:0]  d_idx, d_wstrb;
  logic [31:0] d_wdata, d_rdata;
  logic [6:0]  win_sel [4];
  logic [1:0]  owner [4];
  logic [3:0]  b2_busy, ch_busy, bist_irq;
  logic        dma_busy, dma_irq, rm_irq0;
  logic [1:0]  dma_ch;

  assign ch_busy = b2_busy | (dma_busy ? (4'b0001 << dma_ch) : 4'b0000);

  ng_rmt_regs #(.RM_REV(RM_REV)) u_regs (
    .clk(clk), .rst_n(rst_n),
    .s_awaddr(s_axil_ctrl_awaddr), .s_awvalid(s_axil_ctrl_awvalid), .s_awready(s_axil_ctrl_awready),
    .s_wdata(s_axil_ctrl_wdata), .s_wstrb(s_axil_ctrl_wstrb), .s_wvalid(s_axil_ctrl_wvalid),
    .s_wready(s_axil_ctrl_wready), .s_bvalid(s_axil_ctrl_bvalid), .s_bready(s_axil_ctrl_bready),
    .s_araddr(s_axil_ctrl_araddr), .s_arvalid(s_axil_ctrl_arvalid), .s_arready(s_axil_ctrl_arready),
    .s_rdata(s_axil_ctrl_rdata), .s_rvalid(s_axil_ctrl_rvalid), .s_rready(s_axil_ctrl_rready),
    .rq_valid(rq_valid), .rq_data(rq_data), .rs_valid(rs_valid), .rs_data(rs_data),
    .d_wr(d_wr), .d_idx(d_idx), .d_wstrb(d_wstrb), .d_wdata(d_wdata), .d_rdata(d_rdata),
    .win_sel(win_sel), .owner(owner), .ch_busy(ch_busy), .mem_ready(shell_mem_ready),
    .irq_set((|bist_irq) || dma_irq), .rm_irq0(rm_irq0));
  assign s_axil_ctrl_bresp = 2'b00;
  assign s_axil_ctrl_rresp = 2'b00;
  assign rm_irq = {3'b000, rm_irq0};

  // ------------------------------------------------------------------ BAR2 adapter
  logic [3:0]   wq_valid, wq_ready, rqq_valid, rqq_ready, dq_valid, dq_ready;
  logic [106:0] wq_data, rqq_data, dq_data;
  logic [3:0]   bw_valid, bw_err, br_valid, dw_valid, dw_err, dr_valid;
  logic [65:0]  br_data [4];

  ng_rmt_bar2 u_bar2 (
    .clk(clk), .rst_n(rst_n),
    .s_awid(s_axi_bar2_awid), .s_awaddr(s_axi_bar2_awaddr), .s_awlen(s_axi_bar2_awlen),
    .s_awsize(s_axi_bar2_awsize), .s_awvalid(s_axi_bar2_awvalid), .s_awready(s_axi_bar2_awready),
    .s_wdata(s_axi_bar2_wdata), .s_wstrb(s_axi_bar2_wstrb), .s_wlast(s_axi_bar2_wlast),
    .s_wvalid(s_axi_bar2_wvalid), .s_wready(s_axi_bar2_wready),
    .s_bid(s_axi_bar2_bid), .s_bresp(s_axi_bar2_bresp), .s_bvalid(s_axi_bar2_bvalid),
    .s_bready(s_axi_bar2_bready),
    .s_arid(s_axi_bar2_arid), .s_araddr(s_axi_bar2_araddr), .s_arlen(s_axi_bar2_arlen),
    .s_arsize(s_axi_bar2_arsize), .s_arvalid(s_axi_bar2_arvalid), .s_arready(s_axi_bar2_arready),
    .s_rid(s_axi_bar2_rid), .s_rdata(s_axi_bar2_rdata), .s_rresp(s_axi_bar2_rresp),
    .s_rlast(s_axi_bar2_rlast), .s_rvalid(s_axi_bar2_rvalid), .s_rready(s_axi_bar2_rready),
    .win_sel(win_sel), .ch_busy(b2_busy),
    .wq_valid(wq_valid), .wq_ready(wq_ready), .wq_data(wq_data),
    .rq_valid(rqq_valid), .rq_ready(rqq_ready), .rq_data(rqq_data),
    .bw_valid(bw_valid), .bw_err(bw_err), .br_valid(br_valid), .br_data(br_data));

  // ------------------------------------------------------------------ DMA + host port
  logic        h_awvalid, h_awready, h_wvalid, h_wready, h_wlast, h_bvalid, h_bready;
  logic        h_arvalid, h_arready, h_rvalid, h_rready, h_rlast;
  logic [46:0] h_awaddr, h_araddr;
  logic [63:0] h_wdata, h_rdata;
  logic [1:0]  h_bresp, h_rresp;
  logic [65:0] dr_data [4];

  ng_rmt_dma u_dma (
    .clk(clk), .rst_n(rst_n),
    .rb_wr(d_wr), .rb_idx(d_idx), .rb_wstrb(d_wstrb), .rb_wdata(d_wdata), .rb_rdata(d_rdata),
    .owner(owner), .busy(dma_busy), .ch(dma_ch), .irq_pulse(dma_irq),
    .dq_valid(dq_valid), .dq_ready(dq_ready), .dq_data(dq_data),
    .dw_valid(dw_valid), .dw_err(dw_err), .dr_valid(dr_valid), .dr_data(dr_data),
    .h_awvalid(h_awvalid), .h_awready(h_awready), .h_awaddr(h_awaddr),
    .h_wvalid(h_wvalid), .h_wready(h_wready), .h_wdata(h_wdata), .h_wlast(h_wlast),
    .h_bvalid(h_bvalid), .h_bready(h_bready), .h_bresp(h_bresp),
    .h_arvalid(h_arvalid), .h_arready(h_arready), .h_araddr(h_araddr),
    .h_rvalid(h_rvalid), .h_rready(h_rready), .h_rdata(h_rdata), .h_rresp(h_rresp));

  ng_rmt_port #(.DW(64), .AW(47), .MAX_WR(M_AXI_HOST_MAX_WR), .MAX_RD(M_AXI_HOST_MAX_RD)) u_host (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in), .idle(host_idle),
    .s_awvalid(h_awvalid), .s_awready(h_awready), .s_awaddr(h_awaddr), .s_awlen(8'd63),
    .s_wvalid(h_wvalid), .s_wready(h_wready), .s_wdata(h_wdata), .s_wstrb(8'hFF), .s_wlast(h_wlast),
    .s_bvalid(h_bvalid), .s_bready(h_bready), .s_bresp(h_bresp),
    .s_arvalid(h_arvalid), .s_arready(h_arready), .s_araddr(h_araddr), .s_arlen(8'd63),
    .s_rvalid(h_rvalid), .s_rready(h_rready), .s_rdata(h_rdata), .s_rresp(h_rresp), .s_rlast(h_rlast),
    .m_awaddr(m_axi_host_awaddr), .m_awlen(m_axi_host_awlen), .m_awvalid(m_axi_host_awvalid),
    .m_awready(m_axi_host_awready),
    .m_wdata(m_axi_host_wdata), .m_wstrb(m_axi_host_wstrb), .m_wlast(m_axi_host_wlast),
    .m_wvalid(m_axi_host_wvalid), .m_wready(m_axi_host_wready),
    .m_bresp(m_axi_host_bresp), .m_bvalid(m_axi_host_bvalid), .m_bready(m_axi_host_bready),
    .m_araddr(m_axi_host_araddr), .m_arlen(m_axi_host_arlen), .m_arvalid(m_axi_host_arvalid),
    .m_arready(m_axi_host_arready),
    .m_rdata(m_axi_host_rdata), .m_rresp(m_axi_host_rresp), .m_rlast(m_axi_host_rlast),
    .m_rvalid(m_axi_host_rvalid), .m_rready(m_axi_host_rready));

  assign m_axi_host_awid = '0;  assign m_axi_host_awsize = 3'd3; assign m_axi_host_awburst = 2'b01;
  assign m_axi_host_awcache = 4'b0011; assign m_axi_host_awprot = 3'b010; assign m_axi_host_awqos = '0;
  assign m_axi_host_arid = '0;  assign m_axi_host_arsize = 3'd3; assign m_axi_host_arburst = 2'b01;
  assign m_axi_host_arcache = 4'b0011; assign m_axi_host_arprot = 3'b010; assign m_axi_host_arqos = '0;

  // ------------------------------------------------------------------ channel units
  logic [3:0]   lq_valid, lq_ready, lr_valid;
  logic [106:0] lq_data [4];
  logic [67:0]  lr_data [4];

  for (genvar n = 0; n < 4; n++) begin : g_link
    ng_rmt_arb3 #(.W(107)) u_qarb (
      .clk(clk), .rst_n(rst_n),
      .s_valid({dq_valid[n], rqq_valid[n], wq_valid[n]}),
      .s_ready({dq_ready[n], rqq_ready[n], wq_ready[n]}),
      .s_data0(wq_data), .s_data1(rqq_data), .s_data2(dq_data),
      .m_valid(lq_valid[n]), .m_ready(lq_ready[n]), .m_data(lq_data[n]));

    wire [1:0] kind = lr_data[n][67:66];
    assign bw_valid[n] = lr_valid[n] && (kind == K_BW);
    assign br_valid[n] = lr_valid[n] && (kind == K_BR);
    assign dw_valid[n] = lr_valid[n] && (kind == K_DW);
    assign dr_valid[n] = lr_valid[n] && (kind == K_DR);
    assign bw_err[n]   = (lr_data[n][65:64] != 2'b00);
    assign dw_err[n]   = (lr_data[n][65:64] != 2'b00);
    assign br_data[n]  = lr_data[n][65:0];
    assign dr_data[n]  = lr_data[n][65:0];
  end

  `define NG_RMT_CHAN(n, p, DWV, AWV, MAXRD) \
    ng_rmt_chan #(.DW(DWV), .AW(AWV), .SLR_PIPE(SLR_PIPE), .MAX_WR(32), .MAX_RD(MAXRD), .MUL_ST(MUL_ST)) u_``p ( \
      .clk(clk), .rst_n(rst_ch_n[n]), \
      .lq_valid(lq_valid[n]), .lq_ready(lq_ready[n]), .lq_data(lq_data[n]), \
      .lr_valid(lr_valid[n]), .lr_ready(1'b1), .lr_data(lr_data[n]), \
      .rq_valid(rq_valid[n]), .rq_data(rq_data), .rs_valid(rs_valid[n]), .rs_data(rs_data[n]), \
      .quiesce(q_in), .qidle(ch_qidle[n]), .irq_pulse(bist_irq[n]), \
      .m_awaddr(m_axi_``p``_awaddr), .m_awlen(m_axi_``p``_awlen), .m_awvalid(m_axi_``p``_awvalid), \
      .m_awready(m_axi_``p``_awready), .m_wdata(m_axi_``p``_wdata), .m_wstrb(m_axi_``p``_wstrb), \
      .m_wlast(m_axi_``p``_wlast), .m_wvalid(m_axi_``p``_wvalid), .m_wready(m_axi_``p``_wready), \
      .m_bresp(m_axi_``p``_bresp), .m_bvalid(m_axi_``p``_bvalid), .m_bready(m_axi_``p``_bready), \
      .m_araddr(m_axi_``p``_araddr), .m_arlen(m_axi_``p``_arlen), .m_arvalid(m_axi_``p``_arvalid), \
      .m_arready(m_axi_``p``_arready), .m_rdata(m_axi_``p``_rdata), .m_rresp(m_axi_``p``_rresp), \
      .m_rlast(m_axi_``p``_rlast), .m_rvalid(m_axi_``p``_rvalid), .m_rready(m_axi_``p``_rready)); \
    assign m_axi_``p``_awid = '0; assign m_axi_``p``_awsize = 3'($clog2(DWV / 8)); \
    assign m_axi_``p``_awburst = 2'b01; assign m_axi_``p``_awcache = 4'b0011; \
    assign m_axi_``p``_awprot = 3'b010; assign m_axi_``p``_awqos = '0; \
    assign m_axi_``p``_arid = '0; assign m_axi_``p``_arsize = 3'($clog2(DWV / 8)); \
    assign m_axi_``p``_arburst = 2'b01; assign m_axi_``p``_arcache = 4'b0011; \
    assign m_axi_``p``_arprot = 3'b010; assign m_axi_``p``_arqos = '0;

  `NG_RMT_CHAN(0, c1, M_AXI_C1_DATA_W, M_AXI_C1_ADDR_W, 32)
  `NG_RMT_CHAN(1, c2, M_AXI_C2_DATA_W, M_AXI_C2_ADDR_W, 32)
  `NG_RMT_CHAN(2, c3, M_AXI_C3_DATA_W, M_AXI_C3_ADDR_W, 32)
  `NG_RMT_CHAN(3, c4, M_AXI_C4_DATA_W, M_AXI_C4_ADDR_W, 32)
  `undef NG_RMT_CHAN

  // ----------------------------------------------------------------
  // Vivado debug hub tunnel unused.
  assign dbg_hub_tdo = 1'b0;

  // USER4 observation chain: 32-bit register, CAPTURE loads rm_info dw0.
  logic [31:0] obs_sr;
  always_ff @(posedge dbg_obs_tck) begin
    if (dbg_obs_sel && dbg_obs_capture) obs_sr <= NG_RM_INFO_MAGIC;
    else if (dbg_obs_sel && dbg_obs_shift) obs_sr <= {dbg_obs_tdi, obs_sr[31:1]};
  end
  always_ff @(negedge dbg_obs_tck) dbg_obs_tdo <= obs_sr[0];
endmodule
