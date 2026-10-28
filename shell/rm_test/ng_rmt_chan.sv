// ng_rmt_chan.sv -- one channel unit of the test RM (-).
//
// Placed next to its channel port (C1 SLR2, C2/C4 SLR0, C3 SLR1).  Talks to the
// SLR1 top only through narrow pipelined links (SLR_PIPE register stages each):
//   request link  top->chan  {kind, addr[32:0], strb, data64} valid/ready
//   response link chan->top  {kind, resp, data64}             valid/ready
//   register bus  top->chan  {we, idx, wstrb, wdata} pulse; chan->top rdata pulse
//   sideband      quiesce (top->chan), qidle / bist-done pulse (chan->top)
// Inside: BAR2 lane adapter, BIST engine, DMA gearbox, the static owner mux
// (OWNER register lives here; it changes only when everything is idle) and the
// boundary port stage.  A requester that does not own the channel talks to its
// own local error responder (SLVERR, never reaches the port).
module ng_rmt_chan #(
  parameter int DW       = 512,
  parameter int AW       = 33,
  parameter int SLR_PIPE = 2,
  parameter int MAX_WR   = 32,
  parameter int MAX_RD   = 32,
  parameter int MUL_ST   = 4
) (
  input  logic            clk,
  input  logic            rst_n,
  // request link
  input  logic            lq_valid,
  output logic            lq_ready,
  input  logic [106:0]    lq_data,
  // response link
  output logic            lr_valid,
  input  logic            lr_ready,
  output logic [67:0]     lr_data,
  // register bus
  input  logic            rq_valid,
  input  logic [41:0]     rq_data,
  output logic            rs_valid,
  output logic [31:0]     rs_data,
  // sideband
  input  logic            quiesce,
  output logic            qidle,
  output logic            irq_pulse,
  // boundary master port
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
  localparam int SB = DW / 8;
  localparam logic [1:0] K_BW = 2'd0, K_BR = 2'd1, K_DW = 2'd2, K_DR = 2'd3;

  // ================================================================ link pipes
  logic         q_valid, q_ready;
  logic [106:0] q_data;
  ng_rmt_pipe #(.W(107), .N(SLR_PIPE)) u_lq (
    .clk(clk), .rst_n(rst_n),
    .s_valid(lq_valid), .s_ready(lq_ready), .s_data(lq_data),
    .m_valid(q_valid), .m_ready(q_ready), .m_data(q_data));

  logic        r_valid, r_ready;
  logic [67:0] r_data;
  ng_rmt_pipe #(.W(68), .N(SLR_PIPE)) u_lr (
    .clk(clk), .rst_n(rst_n),
    .s_valid(r_valid), .s_ready(r_ready), .s_data(r_data),
    .m_valid(lr_valid), .m_ready(lr_ready), .m_data(lr_data));

  logic        rb_v, rb_v_q;
  logic [41:0] rb_d;
  logic [31:0] rb_rsp;
  logic        rb_rv;
  logic        q_loc, qidle_loc, irq_loc;
  ng_rmt_dly #(.W(1), .N(SLR_PIPE + 1)) u_rqv (.clk(clk), .rst_n(rst_n), .d(rq_valid), .q(rb_v));
  ng_rmt_dly #(.W(42), .N(SLR_PIPE + 1), .RST(1'b0)) u_rqd (.clk(clk), .rst_n(rst_n), .d(rq_data), .q(rb_d));
  ng_rmt_dly #(.W(1), .N(SLR_PIPE)) u_rsv (.clk(clk), .rst_n(rst_n), .d(rb_rv), .q(rs_valid));
  ng_rmt_dly #(.W(32), .N(SLR_PIPE), .RST(1'b0)) u_rsd (.clk(clk), .rst_n(rst_n), .d(rb_rsp), .q(rs_data));
  ng_rmt_dly #(.W(1), .N(SLR_PIPE + 1)) u_q (.clk(clk), .rst_n(rst_n), .d(quiesce), .q(q_loc));
  ng_rmt_dly #(.W(1), .N(SLR_PIPE)) u_qi (.clk(clk), .rst_n(rst_n), .d(qidle_loc), .q(qidle));
  ng_rmt_dly #(.W(1), .N(SLR_PIPE)) u_irq (.clk(clk), .rst_n(rst_n), .d(irq_loc), .q(irq_pulse));

  // ================================================================ request demux
  wire [1:0]    q_kind = q_data[106:105];
  wire [AW-1:0] q_addr = q_data[72 +: AW];
  wire [7:0]    q_strb = q_data[71:64];
  wire [63:0]   q_d64  = q_data[63:0];
  logic b2_iready, dm_iready;
  assign q_ready = q_kind[1] ? dm_iready : b2_iready;

  // ================================================================ requesters
  // r0: BAR2 adapter
  logic            r0_awvalid, r0_awready, r0_wvalid, r0_wready, r0_bvalid, r0_bready;
  logic            r0_arvalid, r0_arready, r0_rvalid, r0_rready;
  logic [AW-1:0]   r0_awaddr, r0_araddr;
  logic [DW-1:0]   r0_wdata, r0_rdata;
  logic [SB-1:0]   r0_wstrb;
  logic [1:0]      r0_bresp, r0_rresp;
  logic            r0_rlast;
  logic            b2_busy;
  logic            b2_bo_valid, b2_bo_ready, b2_ro_valid, b2_ro_ready;
  logic [1:0]      b2_bo_resp;
  logic [65:0]     b2_ro_data;

  ng_rmt_bar2ch #(.DW(DW), .AW(AW)) u_b2 (
    .clk(clk), .rst_n(rst_n), .busy(b2_busy),
    .i_valid(q_valid && !q_kind[1]), .i_ready(b2_iready), .i_rd(q_kind[0]),
    .i_addr(q_addr), .i_data(q_d64), .i_strb(q_strb),
    .bo_valid(b2_bo_valid), .bo_ready(b2_bo_ready), .bo_resp(b2_bo_resp),
    .ro_valid(b2_ro_valid), .ro_ready(b2_ro_ready), .ro_data(b2_ro_data),
    .m_awvalid(r0_awvalid), .m_awready(r0_awready), .m_awaddr(r0_awaddr),
    .m_wvalid(r0_wvalid), .m_wready(r0_wready), .m_wdata(r0_wdata), .m_wstrb(r0_wstrb),
    .m_bvalid(r0_bvalid), .m_bready(r0_bready), .m_bresp(r0_bresp),
    .m_arvalid(r0_arvalid), .m_arready(r0_arready), .m_araddr(r0_araddr),
    .m_rvalid(r0_rvalid), .m_rready(r0_rready), .m_rdata(r0_rdata), .m_rresp(r0_rresp));

  // r1: BIST
  logic            r1_awvalid, r1_awready, r1_wvalid, r1_wready, r1_bvalid, r1_bready;
  logic            r1_arvalid, r1_arready, r1_rvalid, r1_rready;
  logic [AW-1:0]   r1_awaddr, r1_araddr;
  logic [7:0]      r1_awlen, r1_arlen;
  logic [DW-1:0]   r1_wdata, r1_rdata;
  logic            r1_wlast;
  logic [1:0]      r1_bresp, r1_rresp;
  logic            bist_busy, bist_done;
  logic [31:0]     bist_rdata;
  logic [1:0]      own;

  wire rb_we = rb_d[41];
  wire [4:0] rb_idx = rb_d[40:36];
  wire [3:0] rb_wstrb = rb_d[35:32];
  wire [31:0] rb_wdata = rb_d[31:0];

  ng_rmt_bist #(.DW(DW), .AW(AW), .MUL_ST(MUL_ST)) u_bist (
    .clk(clk), .rst_n(rst_n), .owner_ok(own == 2'd1), .busy(bist_busy), .done_pulse(bist_done),
    .rb_wr(rb_v && rb_we && (rb_idx <= 5'd16)), .rb_idx(rb_idx), .rb_wstrb(rb_wstrb),
    .rb_wdata(rb_wdata), .rb_rdata(bist_rdata),
    .m_awvalid(r1_awvalid), .m_awready(r1_awready), .m_awaddr(r1_awaddr), .m_awlen(r1_awlen),
    .m_wvalid(r1_wvalid), .m_wready(r1_wready), .m_wdata(r1_wdata), .m_wlast(r1_wlast),
    .m_bvalid(r1_bvalid), .m_bready(r1_bready), .m_bresp(r1_bresp),
    .m_arvalid(r1_arvalid), .m_arready(r1_arready), .m_araddr(r1_araddr), .m_arlen(r1_arlen),
    .m_rvalid(r1_rvalid), .m_rready(r1_rready), .m_rdata(r1_rdata), .m_rresp(r1_rresp));

  // r2: DMA gearbox
  logic            r2_awvalid, r2_awready, r2_wvalid, r2_wready, r2_bvalid, r2_bready;
  logic            r2_arvalid, r2_arready, r2_rvalid, r2_rready;
  logic [AW-1:0]   r2_awaddr, r2_araddr;
  logic [7:0]      r2_awlen, r2_arlen;
  logic [DW-1:0]   r2_wdata, r2_rdata;
  logic            r2_wlast;
  logic [1:0]      r2_bresp, r2_rresp;
  logic            dm_busy, dm_o_valid, dm_o_ready;
  logic [66:0]     dm_o_data;

  ng_rmt_dmach #(.DW(DW), .AW(AW)) u_dm (
    .clk(clk), .rst_n(rst_n), .busy(dm_busy),
    .i_valid(q_valid && q_kind[1]), .i_ready(dm_iready), .i_rd(q_kind[0]),
    .i_addr(q_addr), .i_data(q_d64),
    .o_valid(dm_o_valid), .o_ready(dm_o_ready), .o_data(dm_o_data),
    .m_awvalid(r2_awvalid), .m_awready(r2_awready), .m_awaddr(r2_awaddr), .m_awlen(r2_awlen),
    .m_wvalid(r2_wvalid), .m_wready(r2_wready), .m_wdata(r2_wdata), .m_wlast(r2_wlast),
    .m_bvalid(r2_bvalid), .m_bready(r2_bready), .m_bresp(r2_bresp),
    .m_arvalid(r2_arvalid), .m_arready(r2_arready), .m_araddr(r2_araddr), .m_arlen(r2_arlen),
    .m_rvalid(r2_rvalid), .m_rready(r2_rready), .m_rdata(r2_rdata), .m_rresp(r2_rresp));

  // ================================================================ error responders
  logic e0_awready, e0_wready, e0_bvalid, e0_arready, e0_rvalid, e0_rlast, e0_busy;
  logic e2_awready, e2_wready, e2_bvalid, e2_arready, e2_rvalid, e2_rlast, e2_busy;
  logic [1:0] e0_bresp, e0_rresp, e2_bresp, e2_rresp;
  logic [DW-1:0] e0_rdata, e2_rdata;

  ng_rmt_errslv #(.DW(DW)) u_e0 (
    .clk(clk), .rst_n(rst_n), .busy(e0_busy),
    .s_awvalid(r0_awvalid && own != 2'd0), .s_awready(e0_awready),
    .s_wvalid(r0_wvalid && own != 2'd0), .s_wready(e0_wready), .s_wlast(1'b1),
    .s_bvalid(e0_bvalid), .s_bready(r0_bready && own != 2'd0), .s_bresp(e0_bresp),
    .s_arvalid(r0_arvalid && own != 2'd0), .s_arready(e0_arready), .s_arlen(8'd0),
    .s_rvalid(e0_rvalid), .s_rready(r0_rready && own != 2'd0), .s_rdata(e0_rdata),
    .s_rresp(e0_rresp), .s_rlast(e0_rlast));

  ng_rmt_errslv #(.DW(DW)) u_e2 (
    .clk(clk), .rst_n(rst_n), .busy(e2_busy),
    .s_awvalid(r2_awvalid && own != 2'd2), .s_awready(e2_awready),
    .s_wvalid(r2_wvalid && own != 2'd2), .s_wready(e2_wready), .s_wlast(r2_wlast),
    .s_bvalid(e2_bvalid), .s_bready(r2_bready && own != 2'd2), .s_bresp(e2_bresp),
    .s_arvalid(r2_arvalid && own != 2'd2), .s_arready(e2_arready), .s_arlen(r2_arlen),
    .s_rvalid(e2_rvalid), .s_rready(r2_rready && own != 2'd2), .s_rdata(e2_rdata),
    .s_rresp(e2_rresp), .s_rlast(e2_rlast));

  // ================================================================ owner mux -> port
  logic            p_awvalid, p_awready, p_wvalid, p_wready, p_wlast, p_bvalid, p_bready;
  logic            p_arvalid, p_arready, p_rvalid, p_rready, p_rlast;
  logic [AW-1:0]   p_awaddr, p_araddr;
  logic [7:0]      p_awlen, p_arlen;
  logic [DW-1:0]   p_wdata, p_rdata;
  logic [SB-1:0]   p_wstrb;
  logic [1:0]      p_bresp, p_rresp;
  logic            port_idle;

  always_comb begin
    p_awvalid = 1'b0; p_awaddr = r0_awaddr; p_awlen = 8'd0;
    p_wvalid  = 1'b0; p_wdata  = r0_wdata;  p_wstrb = r0_wstrb; p_wlast = 1'b1;
    p_arvalid = 1'b0; p_araddr = r0_araddr; p_arlen = 8'd0;
    p_bready  = 1'b1; p_rready = 1'b1;
    unique case (own)
      2'd0: begin
        p_awvalid = r0_awvalid; p_wvalid = r0_wvalid; p_arvalid = r0_arvalid;
        p_bready  = r0_bready;  p_rready = r0_rready;
      end
      2'd1: begin
        p_awvalid = r1_awvalid; p_awaddr = r1_awaddr; p_awlen = r1_awlen;
        p_wvalid  = r1_wvalid;  p_wdata  = r1_wdata;  p_wstrb = '1; p_wlast = r1_wlast;
        p_arvalid = r1_arvalid; p_araddr = r1_araddr; p_arlen = r1_arlen;
        p_bready  = r1_bready;  p_rready = r1_rready;
      end
      2'd2: begin
        p_awvalid = r2_awvalid; p_awaddr = r2_awaddr; p_awlen = r2_awlen;
        p_wvalid  = r2_wvalid;  p_wdata  = r2_wdata;  p_wstrb = '1; p_wlast = r2_wlast;
        p_arvalid = r2_arvalid; p_araddr = r2_araddr; p_arlen = r2_arlen;
        p_bready  = r2_bready;  p_rready = r2_rready;
      end
      default: ;
    endcase
  end

  // r0 returns
  assign r0_awready = (own == 2'd0) ? p_awready : e0_awready;
  assign r0_wready  = (own == 2'd0) ? p_wready  : e0_wready;
  assign r0_arready = (own == 2'd0) ? p_arready : e0_arready;
  assign r0_bvalid  = (own == 2'd0) ? p_bvalid  : e0_bvalid;
  assign r0_bresp   = (own == 2'd0) ? p_bresp   : e0_bresp;
  assign r0_rvalid  = (own == 2'd0) ? p_rvalid  : e0_rvalid;
  assign r0_rresp   = (own == 2'd0) ? p_rresp   : e0_rresp;
  assign r0_rdata   = (own == 2'd0) ? p_rdata   : e0_rdata;
  assign r0_rlast   = 1'b1;
  // r1 returns (BIST only runs as owner; otherwise held off)
  assign r1_awready = (own == 2'd1) && p_awready;
  assign r1_wready  = (own == 2'd1) && p_wready;
  assign r1_arready = (own == 2'd1) && p_arready;
  assign r1_bvalid  = (own == 2'd1) && p_bvalid;
  assign r1_bresp   = p_bresp;
  assign r1_rvalid  = (own == 2'd1) && p_rvalid;
  assign r1_rresp   = p_rresp;
  assign r1_rdata   = p_rdata;
  // r2 returns
  assign r2_awready = (own == 2'd2) ? p_awready : e2_awready;
  assign r2_wready  = (own == 2'd2) ? p_wready  : e2_wready;
  assign r2_arready = (own == 2'd2) ? p_arready : e2_arready;
  assign r2_bvalid  = (own == 2'd2) ? p_bvalid  : e2_bvalid;
  assign r2_bresp   = (own == 2'd2) ? p_bresp   : e2_bresp;
  assign r2_rvalid  = (own == 2'd2) ? p_rvalid  : e2_rvalid;
  assign r2_rresp   = (own == 2'd2) ? p_rresp   : e2_rresp;
  assign r2_rdata   = (own == 2'd2) ? p_rdata   : e2_rdata;

  ng_rmt_port #(.DW(DW), .AW(AW), .MAX_WR(MAX_WR), .MAX_RD(MAX_RD)) u_port (
    .clk(clk), .rst_n(rst_n), .quiesce(q_loc), .idle(port_idle),
    .s_awvalid(p_awvalid), .s_awready(p_awready), .s_awaddr(p_awaddr), .s_awlen(p_awlen),
    .s_wvalid(p_wvalid), .s_wready(p_wready), .s_wdata(p_wdata), .s_wstrb(p_wstrb), .s_wlast(p_wlast),
    .s_bvalid(p_bvalid), .s_bready(p_bready), .s_bresp(p_bresp),
    .s_arvalid(p_arvalid), .s_arready(p_arready), .s_araddr(p_araddr), .s_arlen(p_arlen),
    .s_rvalid(p_rvalid), .s_rready(p_rready), .s_rdata(p_rdata), .s_rresp(p_rresp), .s_rlast(p_rlast),
    .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid), .m_wready(m_wready),
    .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready));

  // ================================================================ response merge
  logic [2:0] rm_v, rm_r;
  assign rm_v = {dm_o_valid, b2_ro_valid, b2_bo_valid};
  assign b2_bo_ready = rm_r[0];
  assign b2_ro_ready = rm_r[1];
  assign dm_o_ready  = rm_r[2];

  ng_rmt_arb3 #(.W(68)) u_rarb (
    .clk(clk), .rst_n(rst_n),
    .s_valid(rm_v), .s_ready(rm_r),
    .s_data0({K_BW, b2_bo_resp, 64'h0}),
    .s_data1({K_BR, b2_ro_data}),
    .s_data2({(dm_o_data[66] ? K_DR : K_DW), dm_o_data[65:0]}),
    .m_valid(r_valid), .m_ready(r_ready), .m_data(r_data));

  // ================================================================ OWNER + register bus
  wire all_idle = port_idle && !bist_busy && !b2_busy && !dm_busy && !e0_busy && !e2_busy;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      own       <= 2'd0;
      rb_rv     <= 1'b0;
      qidle_loc <= 1'b0;
      irq_loc   <= 1'b0;
    end else begin
      rb_rv     <= rb_v;
      qidle_loc <= q_loc && port_idle;
      irq_loc   <= bist_done;
      if (rb_v && rb_we && rb_idx == 5'd17 && rb_wstrb[0] && all_idle) own <= rb_wdata[1:0];
    end
  end
  always_ff @(posedge clk) begin
    if (rb_idx == 5'd17) begin
      if (rb_we) rb_rsp <= {29'h0, (all_idle || !rb_wstrb[0]), (rb_wstrb[0] && all_idle) ? rb_wdata[1:0] : own};
      else       rb_rsp <= {29'h0, 1'b1, own};
    end else begin
      rb_rsp <= bist_rdata;
    end
  end
endmodule
