// tb_rsp_fab_n.sv -- unit test of the SP fabric port with several requests in flight.
//
// DUT chain (the wiring of ng_gpu_rm_sp.sv, two clocks): TB requester (clk_ts, the slot port) -> ng_rsp_fab_ts ->
// asynchronous FIFOs / slot RAMs -> ng_rsp_fab_l2 (clk_l2) -> { burst masters -> ng_rcp_axmux (s1) -> ng_rcp_hosteng /
// ng_rcp_c3eng b_* ; operation path -> ng_rsp_oparb -> engine requester 1 } -> ng_rmt_port -> tb_rmt_slave, with two
// random burst masters (DMA stand-ins, tb_bmaster from tb_rsp_fab.sv) on s0 of the muxes.
// clk_ts ~10 ns with a random half-period (4.85..5.15 ns), clk_l2 4 ns with a 1.37 ns offset: no integer ratio, the
// phase drifts.
// Checks (the numbers of 9.9):
//   (1) every answer (data / old value / err) equals a reference that executes the requests in ACCEPT order (the
//       conflict rule makes the out-of-order execution equivalent); final slave memory == reference
//   (2) one answer per accepted request, on its slot; answered M_RD slots keep their line until mem_free (read back
//       0..several clocks later)
//   (3) same-line WR->RD / RD->WR / WR->WR / WR->AMO sequences issued back to back (directed + a small random line pool);
//       at every accept: no outstanding request on the same line where either side is a write
//   (4) fences (AMO / maintenance): accepted only with nothing outstanding, nothing accepted until they answer
//   (5) random sp_rst: no answer for a request that was in flight at the reset, an orphan slot is not reused before
//       its completion came back (DUT completion pop), out_cnt returns to 0, orphan writes still land
//   (6) random quiesce (as tb_rsp_fab: RM-idle condition 24 clocks, then the slaves are frozen)
//   (7) R / B error windows: ERR_DDR on exactly the requests to those lines (reads 0, memory unchanged)
//   (8) tb_rmt_slave protocol assertions (INCR, 4 KiB, W not before AW, outstanding limits, valid held)
// plus a throughput phase (64 host / C3 line reads, fixed slave latency) that reports lines per microsecond and checks
// that NOUT > 1 really keeps several lines in flight.
// Parameters: NOUT, BURST, HOST_WR_FLUSH, WR_RUNS, NREQ, SEED.  Pass: NG_RSP_FAB_N_TB_PASS.
`timescale 1ns / 1ps

module tb_rsp_fab_n;
  import ng_lsu_pkg::*;

  parameter int NOUT          = 8;
  parameter bit BURST         = 1'b1;
  parameter bit HOST_WR_FLUSH = 1'b0;
  parameter bit WR_RUNS       = 1'b1;
  parameter int NREQ          = 3000;
  parameter int SEED          = 1;
  localparam int SW = (NOUT > 1) ? $clog2(NOUT) : 1;
  localparam int HW = SW + 124;

  int errors = 0;
  int checks = 0;
  task automatic check(input bit cond, input string what);
    checks++;
    if (!cond) begin errors++; if (errors < 40) $display("FAIL @%0t: %s", $time, what); end
  endtask

  // ---------------------------------------------------------------- clocks / resets
  logic clk = 1'b0;          // clk_l2
  logic clk_ts = 1'b0;
  logic rst_n = 1'b0, rst_ts_n = 1'b0;
  initial begin #1.37; forever #2.0 clk = ~clk; end
  initial begin
    int unsigned s;
    s = $urandom(SEED);
    forever begin #(4.85 + 0.01 * $urandom_range(0, 30)); clk_ts = ~clk_ts; end
  end
  longint unsigned tcyc = 0;
  always @(posedge clk_ts) tcyc++;

  localparam longint unsigned SYS_BASE  = 64'h10_0000_0000;
  localparam longint unsigned HOST_BASE = 64'h8000_0000_0000;
  localparam longint unsigned POOL      = 64'h4_0000;       // random line pool (both spaces)
  localparam int              NL        = 16;               // lines in the pool; lines 14, 15 are the error window
  localparam longint unsigned ERR_LO    = POOL + 128 * 14;
  localparam longint unsigned ERR_HI    = POOL + 128 * 16;
  localparam longint unsigned TPUT      = 64'h8_0000;       // throughput phase lines

  // ---------------------------------------------------------------- DUT: fab_ts (clk_ts) + fab_l2 (clk_l2)
  logic          quiesce, sp_rst;
  logic          mreq_v, mreq_rdy, mrsp_v, mrsp_rd, mfree;
  mreq_t         mreq;
  logic [SW-1:0] mreq_slot, mrsp_slot, mrd_slot;
  logic [1:0]    mrsp_err;
  logic [63:0]   mrsp_old;
  logic [1023:0] mrd_data;
  logic [SW:0]   out_cnt, max_out, x_hgray, y_cgray;
  logic          fab_busy;
  logic [SW-1:0] x_hra, y_cra, y_rra, y_ora;
  logic [HW-1:0] x_hd;
  logic [SW+1:0] x_wra, y_cd;
  logic [143:0]  x_wde, x_wdo;
  logic [1023:0] y_rdh, y_rdc;
  logic [63:0]   y_old;
  logic [31:0]   t_nreq, t_nerr, t_rdn, t_rdmax, t_wrn, t_wrmax, t_orph, t_cst, t_sst, t_fen, t_fst;
  logic [47:0]   t_lpa, t_rdsum, t_wrsum;
  logic [2:0]    t_lkind;
  logic [1:0]    t_lerr;

  logic         sf_c_valid, sf_c_ready, sf_c_rs_valid, sf_h_valid, sf_h_ready, sf_h_rs_valid, sf_wr;
  logic [46:0]  sf_addr;
  logic [7:0]   sf_wstrb;
  logic [63:0]  sf_wdata;
  logic [2:0]   sf_st;
  logic [31:0]  sf_ops, sf_bad, sf_bursts, sf_wfb;
  logic [47:0]  sf_bad_pa;
  logic         e_rs_err_c, e_rs_err_h;
  logic [63:0]  e_rs_rdata_c, e_rs_rdata_h;

  logic        sbh_awvalid, sbh_awready, sbh_wvalid, sbh_wready, sbh_wlast, sbh_bvalid, sbh_bready, sbh_arvalid,
               sbh_arready, sbh_rvalid, sbh_rready, sbh_rlast;
  logic [46:0] sbh_awaddr, sbh_araddr;
  logic [7:0]  sbh_awlen, sbh_arlen, sbh_wstrb;
  logic [63:0] sbh_wdata, sbh_rdata;
  logic [1:0]  sbh_bresp, sbh_rresp;
  logic        mh_awvalid, mh_awready, mh_wvalid, mh_wready, mh_wlast, mh_bvalid, mh_bready, mh_arvalid,
               mh_arready, mh_rvalid, mh_rready, mh_rlast;
  logic [46:0] mh_awaddr, mh_araddr;
  logic [7:0]  mh_awlen, mh_arlen, mh_wstrb;
  logic [63:0] mh_wdata, mh_rdata;
  logic [1:0]  mh_bresp, mh_rresp;
  logic        dh_awvalid, dh_awready, dh_wvalid, dh_wready, dh_wlast, dh_bvalid, dh_bready, dh_arvalid,
               dh_arready, dh_rvalid, dh_rready, dh_rlast;
  logic [46:0] dh_awaddr, dh_araddr;
  logic [7:0]  dh_awlen, dh_arlen, dh_wstrb;
  logic [63:0] dh_wdata, dh_rdata;
  logic [1:0]  dh_bresp, dh_rresp;
  logic        sbc_awvalid, sbc_awready, sbc_wvalid, sbc_wready, sbc_wlast, sbc_bvalid, sbc_bready, sbc_arvalid,
               sbc_arready, sbc_rvalid, sbc_rready, sbc_rlast;
  logic [30:0] sbc_awaddr, sbc_araddr;
  logic [7:0]  sbc_awlen, sbc_arlen;
  logic [127:0] sbc_wdata, sbc_rdata;
  logic [15:0] sbc_wstrb;
  logic [1:0]  sbc_bresp, sbc_rresp;
  logic        mc_awvalid, mc_awready, mc_wvalid, mc_wready, mc_wlast, mc_bvalid, mc_bready, mc_arvalid,
               mc_arready, mc_rvalid, mc_rready, mc_rlast;
  logic [30:0] mc_awaddr, mc_araddr;
  logic [7:0]  mc_awlen, mc_arlen;
  logic [127:0] mc_wdata, mc_rdata;
  logic [15:0] mc_wstrb;
  logic [1:0]  mc_bresp, mc_rresp;
  logic        dc_awvalid, dc_awready, dc_wvalid, dc_wready, dc_wlast, dc_bvalid, dc_bready, dc_arvalid,
               dc_arready, dc_rvalid, dc_rready, dc_rlast;
  logic [30:0] dc_awaddr, dc_araddr;
  logic [7:0]  dc_awlen, dc_arlen;
  logic [127:0] dc_wdata, dc_rdata;
  logic [15:0] dc_wstrb;
  logic [1:0]  dc_bresp, dc_rresp;
  logic        hbm_idle, c3bm_idle, c3_idle, host_idle;
  logic        dma_en;
  int          dh_nerr, dh_ntrans, dc_nerr, dc_ntrans;
  wire rm_idle = c3_idle && host_idle && hbm_idle && c3bm_idle;

  ng_rsp_fab_ts #(.NOUT(NOUT)) u_ts (
    .clk(clk_ts), .rst_n(rst_ts_n), .sp_rst(sp_rst),
    .mem_req_valid(mreq_v), .mem_req_ready(mreq_rdy), .mem_req(mreq), .mem_req_slot(mreq_slot),
    .mem_rsp_valid(mrsp_v), .mem_rsp_slot(mrsp_slot), .mem_rsp_err(mrsp_err), .mem_rsp_rd(mrsp_rd),
    .mem_rsp_old(mrsp_old), .mem_rd_slot(mrd_slot), .mem_rd_data(mrd_data), .mem_free(mfree),
    .out_cnt(out_cnt), .fab_busy(fab_busy),
    .x_hgray(x_hgray), .x_hra(x_hra), .x_hd(x_hd), .x_wra(x_wra), .x_wde(x_wde), .x_wdo(x_wdo),
    .y_cgray(y_cgray), .y_cra(y_cra), .y_cd(y_cd), .y_rra(y_rra), .y_rdh(y_rdh), .y_rdc(y_rdc),
    .y_ora(y_ora), .y_old(y_old),
    .n_req(t_nreq), .n_err(t_nerr), .last_pa(t_lpa), .last_kind(t_lkind), .last_err(t_lerr),
    .rd_lat_sum(t_rdsum), .rd_lat_n(t_rdn), .rd_lat_max(t_rdmax), .wr_lat_sum(t_wrsum), .wr_lat_n(t_wrn),
    .wr_lat_max(t_wrmax), .max_out(max_out), .n_orphan(t_orph), .n_cstall(t_cst), .n_sstall(t_sst),
    .n_fence(t_fen), .n_fstall(t_fst));

  ng_rsp_fab_l2 #(.NOUT(NOUT), .BURST(BURST), .HOST_WR_FLUSH(HOST_WR_FLUSH), .WR_RUNS(WR_RUNS)) u_fab (
    .clk(clk), .rst_n(rst_n), .quiesce(quiesce),
    .x_hgray(x_hgray), .x_hra(x_hra), .x_hd(x_hd), .x_wra(x_wra), .x_wde(x_wde), .x_wdo(x_wdo),
    .y_cgray(y_cgray), .y_cra(y_cra), .y_cd(y_cd), .y_rra(y_rra), .y_rdh(y_rdh), .y_rdc(y_rdc),
    .y_ora(y_ora), .y_old(y_old),
    .c_valid(sf_c_valid), .c_ready(sf_c_ready), .c_rs_valid(sf_c_rs_valid), .c_rs_err(e_rs_err_c),
    .c_rs_rdata(e_rs_rdata_c),
    .h_valid(sf_h_valid), .h_ready(sf_h_ready), .h_rs_valid(sf_h_rs_valid), .h_rs_err(e_rs_err_h),
    .h_rs_rdata(e_rs_rdata_h),
    .op_wr(sf_wr), .op_addr(sf_addr), .op_wstrb(sf_wstrb), .op_wdata(sf_wdata),
    .bh_awvalid(sbh_awvalid), .bh_awready(sbh_awready), .bh_awaddr(sbh_awaddr), .bh_awlen(sbh_awlen),
    .bh_wvalid(sbh_wvalid), .bh_wready(sbh_wready), .bh_wdata(sbh_wdata), .bh_wstrb(sbh_wstrb), .bh_wlast(sbh_wlast),
    .bh_bvalid(sbh_bvalid), .bh_bready(sbh_bready), .bh_bresp(sbh_bresp), .bh_arvalid(sbh_arvalid),
    .bh_arready(sbh_arready), .bh_araddr(sbh_araddr), .bh_arlen(sbh_arlen), .bh_rvalid(sbh_rvalid),
    .bh_rready(sbh_rready), .bh_rdata(sbh_rdata), .bh_rresp(sbh_rresp), .bh_rlast(sbh_rlast),
    .bc_awvalid(sbc_awvalid), .bc_awready(sbc_awready), .bc_awaddr(sbc_awaddr), .bc_awlen(sbc_awlen),
    .bc_wvalid(sbc_wvalid), .bc_wready(sbc_wready), .bc_wdata(sbc_wdata), .bc_wstrb(sbc_wstrb), .bc_wlast(sbc_wlast),
    .bc_bvalid(sbc_bvalid), .bc_bready(sbc_bready), .bc_bresp(sbc_bresp), .bc_arvalid(sbc_arvalid),
    .bc_arready(sbc_arready), .bc_araddr(sbc_araddr), .bc_arlen(sbc_arlen), .bc_rvalid(sbc_rvalid),
    .bc_rready(sbc_rready), .bc_rdata(sbc_rdata), .bc_rresp(sbc_rresp), .bc_rlast(sbc_rlast),
    .dbg_st(sf_st), .n_ops(sf_ops), .n_bad(sf_bad), .bad_pa(sf_bad_pa), .n_burst(sf_bursts), .n_wfb(sf_wfb));

  // ---- operation path: oparb (CP requester tied off) -> engine requester 1
  logic [1:0]  c3a_valid, c3a_ready, c3a_rs, ha_valid, ha_ready, ha_rs;
  logic [1:0]  c3a_st, ha_st;
  logic [31:0] c3a_gnt [2];
  logic [31:0] ha_gnt [2];
  logic        e1_valid, e1_ready, e1_wr, eh_valid, eh_ready, eh_wr, eh_rs_valid;
  logic [30:0] e1_addr;
  logic [46:0] eh_addr;
  logic [7:0]  e1_wstrb, eh_wstrb;
  logic [63:0] e1_wdata, eh_wdata;
  logic [2:0]  e_valid, e_ready, e_rs_valid;

  assign c3a_valid     = {sf_c_valid, 1'b0};
  assign sf_c_ready    = c3a_ready[1];
  assign sf_c_rs_valid = c3a_rs[1];
  ng_rsp_oparb #(.AW(31)) u_c3arb (
    .clk(clk), .rst_n(rst_n),
    .op_valid(c3a_valid), .op_ready(c3a_ready), .op_wr({sf_wr, 1'b0}),
    .op_addr('{31'h0, sf_addr[30:0]}), .op_wstrb('{8'h0, sf_wstrb}), .op_wdata('{64'h0, sf_wdata}),
    .rs_valid(c3a_rs),
    .e_valid(e1_valid), .e_ready(e1_ready), .e_wr(e1_wr), .e_addr(e1_addr), .e_wstrb(e1_wstrb), .e_wdata(e1_wdata),
    .e_rs_valid(e_rs_valid[1]), .dbg_st(c3a_st), .grants(c3a_gnt));
  assign ha_valid      = {sf_h_valid, 1'b0};
  assign sf_h_ready    = ha_ready[1];
  assign sf_h_rs_valid = ha_rs[1];
  ng_rsp_oparb #(.AW(47)) u_harb (
    .clk(clk), .rst_n(rst_n),
    .op_valid(ha_valid), .op_ready(ha_ready), .op_wr({sf_wr, 1'b0}),
    .op_addr('{47'h0, sf_addr}), .op_wstrb('{8'h0, sf_wstrb}), .op_wdata('{64'h0, sf_wdata}),
    .rs_valid(ha_rs),
    .e_valid(eh_valid), .e_ready(eh_ready), .e_wr(eh_wr), .e_addr(eh_addr), .e_wstrb(eh_wstrb), .e_wdata(eh_wdata),
    .e_rs_valid(eh_rs_valid), .dbg_st(ha_st), .grants(ha_gnt));

  // ---- the engines, the muxes, the DMA stand-ins and the slaves (as tb_rsp_fab)
  logic [30:0]  m_c3_awaddr, m_c3_araddr;
  logic [7:0]   m_c3_awlen, m_c3_arlen;
  logic         m_c3_awvalid, m_c3_awready, m_c3_wlast, m_c3_wvalid, m_c3_wready, m_c3_bvalid, m_c3_bready;
  logic         m_c3_arvalid, m_c3_arready, m_c3_rlast, m_c3_rvalid, m_c3_rready;
  logic [127:0] m_c3_wdata, m_c3_rdata;
  logic [15:0]  m_c3_wstrb;
  logic [1:0]   m_c3_bresp, m_c3_rresp;
  logic [46:0]  m_h_awaddr, m_h_araddr;
  logic [7:0]   m_h_awlen, m_h_arlen;
  logic         m_h_awvalid, m_h_awready, m_h_wlast, m_h_wvalid, m_h_wready, m_h_bvalid, m_h_bready;
  logic         m_h_arvalid, m_h_arready, m_h_rlast, m_h_rvalid, m_h_rready;
  logic [63:0]  m_h_wdata, m_h_rdata;
  logic [7:0]   m_h_wstrb;
  logic [1:0]   m_h_bresp, m_h_rresp;
  logic [63:0]  e_rs_rdata;
  logic         e_rs_err, h_rs_err;
  logic [63:0]  h_rs_rdata;
  assign e_rs_err_c = e_rs_err;   assign e_rs_rdata_c = e_rs_rdata;
  assign e_rs_err_h = h_rs_err;   assign e_rs_rdata_h = h_rs_rdata;
  assign e_valid = {1'b0, e1_valid, 1'b0};
  assign e1_ready = e_ready[1];

  ng_rcp_c3eng #(.DW(128), .AW(31)) u_c3 (
    .clk(clk), .rst_n(rst_n), .quiesce(quiesce), .mem_ready(1'b1), .idle(c3_idle),
    .op_valid(e_valid), .op_ready(e_ready), .op_wr({e1_wr, 1'b0}),
    .op_addr0(31'h0), .op_addr1(e1_addr), .op_addr2(31'h0),
    .op_wstrb0(8'h0), .op_wstrb1(e1_wstrb), .op_wdata0(64'h0), .op_wdata1(e1_wdata),
    .rs_valid(e_rs_valid), .rs_err(e_rs_err), .rs_rdata(e_rs_rdata),
    .m_awaddr(m_c3_awaddr), .m_awlen(m_c3_awlen), .m_awvalid(m_c3_awvalid), .m_awready(m_c3_awready),
    .m_wdata(m_c3_wdata), .m_wstrb(m_c3_wstrb), .m_wlast(m_c3_wlast), .m_wvalid(m_c3_wvalid),
    .m_wready(m_c3_wready), .m_bresp(m_c3_bresp), .m_bvalid(m_c3_bvalid), .m_bready(m_c3_bready),
    .m_araddr(m_c3_araddr), .m_arlen(m_c3_arlen), .m_arvalid(m_c3_arvalid), .m_arready(m_c3_arready),
    .m_rdata(m_c3_rdata), .m_rresp(m_c3_rresp), .m_rlast(m_c3_rlast), .m_rvalid(m_c3_rvalid),
    .m_rready(m_c3_rready),
    .b_awvalid(mc_awvalid), .b_awready(mc_awready), .b_awaddr(mc_awaddr), .b_awlen(mc_awlen), .b_wvalid(mc_wvalid),
    .b_wready(mc_wready), .b_wdata(mc_wdata), .b_wstrb(mc_wstrb), .b_wlast(mc_wlast), .b_bvalid(mc_bvalid),
    .b_bready(mc_bready), .b_bresp(mc_bresp), .b_arvalid(mc_arvalid), .b_arready(mc_arready), .b_araddr(mc_araddr),
    .b_arlen(mc_arlen), .b_rvalid(mc_rvalid), .b_rready(mc_rready), .b_rdata(mc_rdata), .b_rresp(mc_rresp),
    .b_rlast(mc_rlast));

  ng_rcp_hosteng u_host (
    .clk(clk), .rst_n(rst_n), .quiesce(quiesce), .idle(host_idle),
    .op_valid(eh_valid), .op_ready(eh_ready), .op_wr(eh_wr), .op_addr(eh_addr), .op_wstrb(eh_wstrb),
    .op_wdata(eh_wdata),
    .rs_valid(eh_rs_valid), .rs_err(h_rs_err), .rs_rdata(h_rs_rdata),
    .m_awaddr(m_h_awaddr), .m_awlen(m_h_awlen), .m_awvalid(m_h_awvalid), .m_awready(m_h_awready),
    .m_wdata(m_h_wdata), .m_wstrb(m_h_wstrb), .m_wlast(m_h_wlast), .m_wvalid(m_h_wvalid), .m_wready(m_h_wready),
    .m_bresp(m_h_bresp), .m_bvalid(m_h_bvalid), .m_bready(m_h_bready),
    .m_araddr(m_h_araddr), .m_arlen(m_h_arlen), .m_arvalid(m_h_arvalid), .m_arready(m_h_arready),
    .m_rdata(m_h_rdata), .m_rresp(m_h_rresp), .m_rlast(m_h_rlast), .m_rvalid(m_h_rvalid), .m_rready(m_h_rready),
    .b_awvalid(mh_awvalid), .b_awready(mh_awready), .b_awaddr(mh_awaddr), .b_awlen(mh_awlen), .b_wvalid(mh_wvalid),
    .b_wready(mh_wready), .b_wdata(mh_wdata), .b_wstrb(mh_wstrb), .b_wlast(mh_wlast), .b_bvalid(mh_bvalid),
    .b_bready(mh_bready), .b_bresp(mh_bresp), .b_arvalid(mh_arvalid), .b_arready(mh_arready), .b_araddr(mh_araddr),
    .b_arlen(mh_arlen), .b_rvalid(mh_rvalid), .b_rready(mh_rready), .b_rdata(mh_rdata), .b_rresp(mh_rresp),
    .b_rlast(mh_rlast));

  ng_rcp_axmux #(.DW(64), .AW(47), .DEPTH(10), .REG_A(1'b1)) u_hbmux (
    .clk(clk), .rst_n(rst_n), .idle(hbm_idle),
    .s0_awvalid(dh_awvalid), .s0_awready(dh_awready), .s0_awaddr(dh_awaddr), .s0_awlen(dh_awlen),
    .s0_wvalid(dh_wvalid), .s0_wready(dh_wready), .s0_wdata(dh_wdata), .s0_wstrb(dh_wstrb), .s0_wlast(dh_wlast),
    .s0_bvalid(dh_bvalid), .s0_bready(dh_bready), .s0_bresp(dh_bresp), .s0_arvalid(dh_arvalid),
    .s0_arready(dh_arready), .s0_araddr(dh_araddr), .s0_arlen(dh_arlen), .s0_rvalid(dh_rvalid),
    .s0_rready(dh_rready), .s0_rdata(dh_rdata), .s0_rresp(dh_rresp), .s0_rlast(dh_rlast),
    .s1_awvalid(sbh_awvalid), .s1_awready(sbh_awready), .s1_awaddr(sbh_awaddr), .s1_awlen(sbh_awlen),
    .s1_wvalid(sbh_wvalid), .s1_wready(sbh_wready), .s1_wdata(sbh_wdata), .s1_wstrb(sbh_wstrb), .s1_wlast(sbh_wlast),
    .s1_bvalid(sbh_bvalid), .s1_bready(sbh_bready), .s1_bresp(sbh_bresp), .s1_arvalid(sbh_arvalid),
    .s1_arready(sbh_arready), .s1_araddr(sbh_araddr), .s1_arlen(sbh_arlen), .s1_rvalid(sbh_rvalid),
    .s1_rready(sbh_rready), .s1_rdata(sbh_rdata), .s1_rresp(sbh_rresp), .s1_rlast(sbh_rlast),
    .m_awvalid(mh_awvalid), .m_awready(mh_awready), .m_awaddr(mh_awaddr), .m_awlen(mh_awlen),
    .m_wvalid(mh_wvalid), .m_wready(mh_wready), .m_wdata(mh_wdata), .m_wstrb(mh_wstrb), .m_wlast(mh_wlast),
    .m_bvalid(mh_bvalid), .m_bready(mh_bready), .m_bresp(mh_bresp), .m_arvalid(mh_arvalid),
    .m_arready(mh_arready), .m_araddr(mh_araddr), .m_arlen(mh_arlen), .m_rvalid(mh_rvalid),
    .m_rready(mh_rready), .m_rdata(mh_rdata), .m_rresp(mh_rresp), .m_rlast(mh_rlast));

  ng_rcp_axmux #(.DW(128), .AW(31), .DEPTH(18), .REG_A(1'b1)) u_c3bmux (
    .clk(clk), .rst_n(rst_n), .idle(c3bm_idle),
    .s0_awvalid(dc_awvalid), .s0_awready(dc_awready), .s0_awaddr(dc_awaddr), .s0_awlen(dc_awlen),
    .s0_wvalid(dc_wvalid), .s0_wready(dc_wready), .s0_wdata(dc_wdata), .s0_wstrb(dc_wstrb), .s0_wlast(dc_wlast),
    .s0_bvalid(dc_bvalid), .s0_bready(dc_bready), .s0_bresp(dc_bresp), .s0_arvalid(dc_arvalid),
    .s0_arready(dc_arready), .s0_araddr(dc_araddr), .s0_arlen(dc_arlen), .s0_rvalid(dc_rvalid),
    .s0_rready(dc_rready), .s0_rdata(dc_rdata), .s0_rresp(dc_rresp), .s0_rlast(dc_rlast),
    .s1_awvalid(sbc_awvalid), .s1_awready(sbc_awready), .s1_awaddr(sbc_awaddr), .s1_awlen(sbc_awlen),
    .s1_wvalid(sbc_wvalid), .s1_wready(sbc_wready), .s1_wdata(sbc_wdata), .s1_wstrb(sbc_wstrb), .s1_wlast(sbc_wlast),
    .s1_bvalid(sbc_bvalid), .s1_bready(sbc_bready), .s1_bresp(sbc_bresp), .s1_arvalid(sbc_arvalid),
    .s1_arready(sbc_arready), .s1_araddr(sbc_araddr), .s1_arlen(sbc_arlen), .s1_rvalid(sbc_rvalid),
    .s1_rready(sbc_rready), .s1_rdata(sbc_rdata), .s1_rresp(sbc_rresp), .s1_rlast(sbc_rlast),
    .m_awvalid(mc_awvalid), .m_awready(mc_awready), .m_awaddr(mc_awaddr), .m_awlen(mc_awlen),
    .m_wvalid(mc_wvalid), .m_wready(mc_wready), .m_wdata(mc_wdata), .m_wstrb(mc_wstrb), .m_wlast(mc_wlast),
    .m_bvalid(mc_bvalid), .m_bready(mc_bready), .m_bresp(mc_bresp), .m_arvalid(mc_arvalid),
    .m_arready(mc_arready), .m_araddr(mc_araddr), .m_arlen(mc_arlen), .m_rvalid(mc_rvalid),
    .m_rready(mc_rready), .m_rdata(mc_rdata), .m_rresp(mc_rresp), .m_rlast(mc_rlast));

  tb_bmaster #(.DW(64), .AW(47), .BASE(64'h10_0000), .MAXB(16)) u_dmah (
    .clk(clk), .rst_n(rst_n), .en(dma_en),
    .awvalid(dh_awvalid), .awready(dh_awready), .awaddr(dh_awaddr), .awlen(dh_awlen),
    .wvalid(dh_wvalid), .wready(dh_wready), .wdata(dh_wdata), .wstrb(dh_wstrb), .wlast(dh_wlast),
    .bvalid(dh_bvalid), .bready(dh_bready), .bresp(dh_bresp), .arvalid(dh_arvalid), .arready(dh_arready),
    .araddr(dh_araddr), .arlen(dh_arlen), .rvalid(dh_rvalid), .rready(dh_rready), .rdata(dh_rdata),
    .rresp(dh_rresp), .rlast(dh_rlast), .nerr(dh_nerr), .ntrans(dh_ntrans));
  tb_bmaster #(.DW(128), .AW(31), .BASE(64'h10_0000), .MAXB(8)) u_dmac (
    .clk(clk), .rst_n(rst_n), .en(dma_en),
    .awvalid(dc_awvalid), .awready(dc_awready), .awaddr(dc_awaddr), .awlen(dc_awlen),
    .wvalid(dc_wvalid), .wready(dc_wready), .wdata(dc_wdata), .wstrb(dc_wstrb), .wlast(dc_wlast),
    .bvalid(dc_bvalid), .bready(dc_bready), .bresp(dc_bresp), .arvalid(dc_arvalid), .arready(dc_arready),
    .araddr(dc_araddr), .arlen(dc_arlen), .rvalid(dc_rvalid), .rready(dc_rready), .rdata(dc_rdata),
    .rresp(dc_rresp), .rlast(dc_rlast), .nerr(dc_nerr), .ntrans(dc_ntrans));

  logic [2:0] c3_awsize, c3_arsize;
  assign c3_awsize = 3'd4;  assign c3_arsize = 3'd4;
  tb_rmt_slave #(.DW(128), .AW(31), .IW(6), .MAXLEN(15), .MAXWR(16), .MAXRD(16), .NAME("c3")) u_c3s (
    .clk(clk), .rst_n(rst_n),
    .awid(6'h0), .awaddr(m_c3_awaddr), .awlen(m_c3_awlen), .awsize(c3_awsize), .awburst(2'b01), .awcache(4'b0011),
    .awprot(3'b010), .awqos(4'h0), .awvalid(m_c3_awvalid), .awready(m_c3_awready),
    .wdata(m_c3_wdata), .wstrb(m_c3_wstrb), .wlast(m_c3_wlast), .wvalid(m_c3_wvalid), .wready(m_c3_wready),
    .bid(), .bresp(m_c3_bresp), .bvalid(m_c3_bvalid), .bready(m_c3_bready),
    .arid(6'h0), .araddr(m_c3_araddr), .arlen(m_c3_arlen), .arsize(c3_arsize), .arburst(2'b01), .arcache(4'b0011),
    .arprot(3'b010), .arqos(4'h0), .arvalid(m_c3_arvalid), .arready(m_c3_arready),
    .rid(), .rdata(m_c3_rdata), .rresp(m_c3_rresp), .rlast(m_c3_rlast), .rvalid(m_c3_rvalid), .rready(m_c3_rready));
  tb_rmt_slave #(.DW(64), .AW(47), .IW(4), .MAXLEN(63), .MAXWR(8), .MAXRD(8), .NAME("host"), .XDMA_STRB(1)) u_hs (
    .clk(clk), .rst_n(rst_n),
    .awid(4'h0), .awaddr(m_h_awaddr), .awlen(m_h_awlen), .awsize(3'd3), .awburst(2'b01), .awcache(4'b0011),
    .awprot(3'b010), .awqos(4'h0), .awvalid(m_h_awvalid), .awready(m_h_awready),
    .wdata(m_h_wdata), .wstrb(m_h_wstrb), .wlast(m_h_wlast), .wvalid(m_h_wvalid), .wready(m_h_wready),
    .bid(), .bresp(m_h_bresp), .bvalid(m_h_bvalid), .bready(m_h_bready),
    .arid(4'h0), .araddr(m_h_araddr), .arlen(m_h_arlen), .arsize(3'd3), .arburst(2'b01), .arcache(4'b0011),
    .arprot(3'b010), .arqos(4'h0), .arvalid(m_h_arvalid), .arready(m_h_arready),
    .rid(), .rdata(m_h_rdata), .rresp(m_h_rresp), .rlast(m_h_rlast), .rvalid(m_h_rvalid), .rready(m_h_rready));

  // ---------------------------------------------------------------- reference model
  logic [7:0] refc [longint unsigned];
  logic [7:0] refh [longint unsigned];
  function automatic logic [7:0] rbyte(input bit host, input longint unsigned a);
    if (host) return refh.exists(a) ? refh[a] : 8'h00;
    else      return refc.exists(a) ? refc[a] : 8'h00;
  endfunction
  function automatic logic [7:0] mbyte(input bit host, input longint unsigned a);
    logic [127:0] c;
    logic [63:0]  h;
    if (host) begin h = u_hs.peek(a >> 3); return h[8 * a[2:0] +: 8]; end
    c = u_c3s.peek(a >> 4); return c[8 * a[3:0] +: 8];
  endfunction
  task automatic mset(input bit host, input longint unsigned a, input logic [7:0] v);
    logic [127:0] c;
    logic [63:0]  h;
    if (host) begin refh[a] = v; h = u_hs.peek(a >> 3); h[8 * a[2:0] +: 8] = v; u_hs.poke(a >> 3, h); end
    else      begin refc[a] = v; c = u_c3s.peek(a >> 4); c[8 * a[3:0] +: 8] = v; u_c3s.poke(a >> 4, c); end
  endtask
  task automatic fill_line(input bit host, input longint unsigned lb);
    for (int i = 0; i < 128; i++) mset(host, lb + i, 8'($urandom));
  endtask
  task automatic chk_mem(input bit host, input longint unsigned lb, input string what);
    bit ok = 1'b1;
    for (int i = 0; i < 128; i++) if (mbyte(host, lb + i) !== rbyte(host, lb + i)) begin
      if (ok) $display("  mem mismatch (%s) %s line %h byte %0d: got %h exp %h", what, host ? "host" : "c3", lb, i,
                       mbyte(host, lb + i), rbyte(host, lb + i));
      ok = 1'b0;
    end
    check(ok, {what, ": slave memory == reference"});
  endtask
  bit err_on = 1'b0;            // the error window of both slaves is active
  function automatic bit in_errw(input longint unsigned off);
    return err_on && (off >= ERR_LO) && (off < ERR_HI);
  endfunction

  function automatic mreq_t mk(input logic [2:0] kind, input bit host, input longint unsigned off);
    mreq_t r;
    r = '0;
    r.kind = kind; r.pa = 48'(host ? (HOST_BASE + off) : (SYS_BASE + off)); r.uc = 1'b1;
    return r;
  endfunction
  function automatic logic [127:0] mk_be(input int cls);
    logic [127:0] b;
    int lo, hi;
    b = '0;
    unique case (cls)
      0: b = '1;
      1: begin lo = $urandom_range(0, 15); hi = $urandom_range(lo, 15); for (int q = lo; q <= hi; q++) b[8*q +: 8] = 8'hFF; end
      2: begin lo = $urandom_range(0, 127); hi = $urandom_range(lo, 127); for (int i = lo; i <= hi; i++) b[i] = 1'b1; end
      3: begin b[$urandom_range(0, 127)] = 1'b1; end
      4: begin for (int k = 0; k < 4; k++) b[32*k +: 32] = $urandom; end
      5: begin for (int q = 0; q < 16; q += 2) b[8*q +: 8] = 8'hFF; end
      7: begin                                       // XDMA hole patterns per qword (as tb_rsp_fab xdma_q)
        for (int q = 0; q < 16; q++) begin
          unique case ($urandom_range(0, 10))
            0, 1, 2, 3: b[8*q +: 8] = 8'h11 << $urandom_range(0, 3);
            4:          b[8*q +: 8] = 8'h55;
            5:          b[8*q +: 8] = 8'hAA;
            6:          b[8*q +: 8] = 8'h81;
            7:          b[8*q +: 8] = 8'h3C;
            8:          b[8*q +: 8] = 8'hFF;
            9:          b[8*q +: 8] = 8'h00;
            default:    b[8*q +: 8] = 8'($urandom);
          endcase
        end
      end
      default: b = '0;
    endcase
    return b;
  endfunction
  function automatic mreq_t mk_wr(input bit host, input longint unsigned lb, input logic [127:0] be);
    mreq_t r;
    r = mk(M_WR, host, lb);
    r.be = be;
    for (int k = 0; k < 32; k++) r.wdata[32*k +: 32] = $urandom;
    return r;
  endfunction
  function automatic mreq_t mk_amo(input bit host, input longint unsigned off, input bit w64, input logic [4:0] op,
                                   input logic [63:0] a, input logic [63:0] c);
    mreq_t r;
    r = mk(M_AMO, host, off);
    r.amo_op = op; r.amo64 = w64; r.amo_a = a; r.amo_cmp = c;
    return r;
  endfunction

  // ---------------------------------------------------------------- outstanding table (TB view, by slot)
  typedef struct {
    bit              v;          // allocated in the TB's view
    bit              orph;       // in flight at an sp_rst: must never be answered
    bit              answered;   // M_RD answered, line not yet read back / freed
    logic [2:0]      kind;
    bit              host;
    bit              fence;
    bit              wr;
    longint unsigned line;       // line offset
    logic [1:0]      eerr;
    logic [1023:0]   edata;
    logic [63:0]     eold;
    int              id;
  } ot_t;
  ot_t ot [NOUT];
  bit  orph_pend [NOUT];
  int  rbq [$];                  // answered M_RD slots waiting for the read-back
  int  n_acc = 0, n_ans = 0, n_orph = 0, n_rst = 0, n_fence_tb = 0, n_conf_seq = 0;
  int  nxt_id = 0;

  // reference execution at accept (accept order == execution order for every pair the conflict rule orders)
  function automatic ot_t accept_ref(input mreq_t r);
    ot_t e;
    bit host, sys, maint, bad, amo_ok;
    longint unsigned off;
    e = '{default: '0};
    e.kind = r.kind; e.v = 1'b1; e.id = nxt_id;
    host  = r.pa[47];
    sys   = (r.pa[47:31] == 17'h00020);
    off   = host ? 64'(r.pa[46:0]) : 64'(r.pa[30:0]);
    e.host = host; e.line = off & ~64'h7F;
    maint = (r.kind == M_INV_L1) || (r.kind == M_INV_L2) || (r.kind == M_WB_L2);
    e.fence = maint || (r.kind == M_AMO);
    e.wr    = (r.kind == M_WR);
    amo_ok = (r.amo_op <= 5'd12) && (r.amo64 ? (r.pa[2:0] == 3'b000) && (r.amo_op <= 5'd10) : (r.pa[1:0] == 2'b00));
    bad = !maint && (!r.uc || !(sys || host) || ((r.kind == M_AMO) && !amo_ok) ||
                     !((r.kind == M_RD) || (r.kind == M_WR) || (r.kind == M_AMO)));
    if (maint) begin
      e.eerr = 2'd0;
    end else if (bad) begin
      e.eerr = 2'd1;
    end else if (r.kind == M_RD) begin
      if (in_errw(e.line)) begin e.eerr = 2'd3; e.edata = '0; end
      else for (int i = 0; i < 128; i++) e.edata[8*i +: 8] = rbyte(host, e.line + i);
    end else if (r.kind == M_WR) begin
      if (r.be == '0) e.eerr = 2'd0;
      else if (in_errw(e.line)) e.eerr = 2'd3;
      else for (int i = 0; i < 128; i++) if (r.be[i]) begin
        if (host) refh[e.line + i] = r.wdata[8*i +: 8]; else refc[e.line + i] = r.wdata[8*i +: 8];
      end
    end else begin                                         // AMO (never on the error window)
      logic [63:0] cur, nw;
      bit w64;
      w64 = r.amo64;
      cur = '0;
      for (int i = 0; i < (w64 ? 8 : 4); i++) cur[8*i +: 8] = rbyte(host, off + i);
      e.eold = w64 ? cur : {32'h0, cur[31:0]};
      unique case (r.amo_op)
        5'd0:  nw = cur + r.amo_a;
        5'd4:  nw = cur ^ r.amo_a;
        5'd9:  nw = r.amo_a;
        default: nw = (w64 ? (cur == r.amo_cmp) : (cur[31:0] == r.amo_cmp[31:0])) ? r.amo_a : cur;
      endcase
      for (int i = 0; i < (w64 ? 8 : 4); i++) begin
        if (host) refh[off + i] = nw[8*i +: 8]; else refc[off + i] = nw[8*i +: 8];
      end
    end
    return e;
  endfunction

  function automatic mreq_t rnd_req();
    int k;
    bit host;
    longint unsigned lb;
    k = $urandom_range(0, 99);
    host = $urandom_range(0, 1);
    lb = POOL + 128 * $urandom_range(0, NL - 1);
    if (k < 35) return mk(M_RD, host, lb);
    if (k < 75) return mk_wr(host, lb, mk_be($urandom_range(0, 7)));
    if (k < 83) begin
      longint unsigned ao;
      bit w64;
      logic [4:0] op;
      lb = POOL + 128 * $urandom_range(0, NL - 3);              // not the error window
      ao = lb + 8 * $urandom_range(0, 15);
      w64 = $urandom_range(0, 1);
      unique case ($urandom_range(0, 3))
        0: op = 5'd0;  1: op = 5'd4;  2: op = 5'd9;  default: op = 5'd10;
      endcase
      return mk_amo(host, w64 ? ao : ao + 4 * $urandom_range(0, 1), w64, op, {$urandom, $urandom},
                    ($urandom_range(0, 1) ? 64'h0 : {$urandom, $urandom}));
    end
    if (k < 87) begin
      mreq_t r;
      unique case ($urandom_range(0, 2))
        0: r = mk(M_INV_L1, 1'b0, 0);  1: r = mk(M_INV_L2, 1'b0, 0);  default: r = mk(M_WB_L2, 1'b0, 0);
      endcase
      r.pa = '0;
      return r;
    end
    if (k < 92) begin
      mreq_t r;
      unique case ($urandom_range(0, 3))
        0: begin r = mk(M_RD, 1'b0, 0); r.pa = 48'h1000; end                         // VRAM
        1: begin r = mk_wr(1'b0, 0, '1); r.pa = 48'h20_0000_0000; end                // hole
        2: begin r = mk(M_RD, host, lb); r.uc = 1'b0; end                            // not uc
        default: begin r = mk_amo(host, lb + 2, 1'b0, 5'd0, 64'h1, 64'h0); end      // misaligned atomic
      endcase
      return r;
    end
    return mk_wr(host, lb, '0);                                                       // be == 0
  endfunction

  // ---------------------------------------------------------------- requester (clk_ts, drives at negedge)
  mreq_t dq [$];               // directed requests, issued before random ones
  bit    gen_en = 1'b0;        // generate random requests
  int    gen_left = 0;
  int    issue_pct = 70;
  bit    rst_en = 1'b0;
  int    rst_left = 0;
  bit    acc_pend = 1'b0;
  bit    hold = 1'b0;          // a request is presented (valid held until accepted)

  // DUT completion pops: an orphan's answer came back, its slot may be reused from now on
  always @(posedge clk_ts) if (rst_ts_n && u_ts.c_pop) begin
    int s;
    s = int'(u_ts.c_slot);
    if (orph_pend[s]) begin orph_pend[s] = 1'b0; ot[s].v = 1'b0; n_orph++; end
  end

  task automatic ts_step();
    // (a) answer pulse of the previous clock
    if (mrsp_v) begin
      int s;
      s = int'(mrsp_slot);
      if (!ot[s].v || ot[s].answered) begin
        check(1'b0, $sformatf("answer on slot %0d that is not outstanding", s));
      end else if (ot[s].orph) begin
        check(1'b0, $sformatf("stale answer on slot %0d (request %0d was in flight at an sp_rst)", s, ot[s].id));
      end else begin
        n_ans++;
        check(mrsp_err == ot[s].eerr, $sformatf("request %0d kind %0d %s line %h: err %0d expected %0d", ot[s].id,
              ot[s].kind, ot[s].host ? "host" : "c3", ot[s].line, mrsp_err, ot[s].eerr));
        check(mrsp_rd == (ot[s].kind == M_RD), "mem_rsp_rd == the request was M_RD");
        if (ot[s].kind == M_AMO && ot[s].eerr == 2'd0)
          check(mrsp_old == ot[s].eold, $sformatf("request %0d AMO old %h expected %h", ot[s].id, mrsp_old, ot[s].eold));
        if (ot[s].kind == M_RD) begin ot[s].answered = 1'b1; rbq.push_back(s); end
        else ot[s].v = 1'b0;
      end
    end
    // (b) read back one answered line: random delay, random order among the answered M_RD slots (the port allows
    //     mem_rd_slot = any answered, not yet freed read slot and an arbitrarily late mem_free), free its slot
    mfree = 1'b0;
    if (rbq.size() != 0 && !sp_rst && $urandom_range(0, 1) != 0) begin
      int s, ix;
      ix = $urandom_range(0, rbq.size() - 1);
      s = rbq[ix];
      rbq.delete(ix);
      mrd_slot = SW'(s); mfree = 1'b1;
      #0.1;
      if (ot[s].eerr != 2'd1) begin
        bit ok;
        ok = (mrd_data === ot[s].edata);
        if (!ok) for (int i = 0; i < 128; i++) if (mrd_data[8*i +: 8] !== ot[s].edata[8*i +: 8]) begin
          $display("  RD mismatch request %0d %s line %h byte %0d: got %h exp %h", ot[s].id, ot[s].host ? "host" : "c3",
                   ot[s].line, i, mrd_data[8*i +: 8], ot[s].edata[8*i +: 8]);
          break;
        end
        check(ok, $sformatf("request %0d RD data == reference", ot[s].id));
      end
      ot[s].v = 1'b0; ot[s].answered = 1'b0;
    end
    // (c) sp_rst
    if (sp_rst) begin
      if (rst_left > 0) rst_left--; else sp_rst = 1'b0;
    end else if (rst_en && $urandom_range(0, 599) == 0) begin
      sp_rst = 1'b1; rst_left = $urandom_range(0, 20); n_rst++;
      for (int i = 0; i < NOUT; i++) if (ot[i].v) begin
        if (ot[i].answered) begin ot[i].v = 1'b0; ot[i].answered = 1'b0; end   // freed by the reset
        else if (!ot[i].orph) begin ot[i].orph = 1'b1; orph_pend[i] = 1'b1; end
      end
      rbq.delete();
      mfree = 1'b0;
    end
    // (d) the request presented in the previous clock was accepted at the posedge: present the next one
    if (acc_pend) begin acc_pend = 1'b0; hold = 1'b0; mreq_v = 1'b0; end
    if (!hold) begin
      if (dq.size() != 0) begin
        mreq = dq.pop_front(); mreq_v = 1'b1; hold = 1'b1;
      end else if (gen_en && gen_left > 0 && $urandom_range(0, 99) < issue_pct) begin
        mreq = rnd_req(); mreq_v = 1'b1; hold = 1'b1; gen_left--;
      end
    end
    #0.1;
    // (e) will the presented request be accepted at the next posedge?
    if (mreq_v && mreq_rdy) begin
      int s, nout;
      ot_t e;
      s = int'(mreq_slot);
      check(!ot[s].v && !orph_pend[s], $sformatf("accept into slot %0d that is still in flight (TB view)", s));
      nout = 0;
      for (int i = 0; i < NOUT; i++) if (ot[i].v) begin
        nout++;
        if (ot[i].fence) check(1'b0, $sformatf("request accepted while fence request %0d is in flight", ot[i].id));
      end
      e = accept_ref(mreq);
      if (e.fence) begin
        n_fence_tb++;
        check(nout == 0, $sformatf("fence request accepted with %0d requests in flight", nout));
      end
      if (e.kind == M_RD || e.kind == M_WR) for (int i = 0; i < NOUT; i++)
        if (ot[i].v && ot[i].host == e.host && ot[i].line == e.line && (ot[i].kind == M_RD || ot[i].kind == M_WR)
            && e.eerr != 2'd1 && (ot[i].wr || e.wr)) begin
          check(1'b0, $sformatf("same-line conflict not blocked: request %0d with %0d in flight", e.id, ot[i].id));
        end
      ot[s] = e;
      nxt_id++; n_acc++;
      acc_pend = 1'b1;
    end
  endtask

  // after the quiesce acknowledge (RM idle for 24 clocks, slaves frozen) no AR / AW may reach a slave (CI report on
  // Verilator 5.052: "AR/AW handshake after rm_quiesce_ack"; the quiesce agent's locals were static there, see below)
  int q_hs_viol = 0;
  always @(posedge clk) if (u_hs.freeze || u_c3s.freeze) begin
    if ((m_h_arvalid && m_h_arready) || (m_h_awvalid && m_h_awready) || (m_c3_arvalid && m_c3_arready) ||
        (m_c3_awvalid && m_c3_awready)) begin
      q_hs_viol++;
      if (q_hs_viol < 5) $display("FAIL @%0t: AR/AW handshake at a slave after the quiesce acknowledge", $time);
    end
  end

  // ---------------------------------------------------------------- quiesce agent (clk_l2, as tb_rsp_fab)
  bit q_enable = 1'b0;
  int q_count = 0;
  initial begin
    quiesce = 1'b0;
    forever begin
      @(posedge clk);
      if (q_enable && $urandom_range(0, 399) == 0) begin
        // automatic: a static local (LRM default here; xsim, newer Verilator) keeps 24 from the first episode, the
        // slaves were then frozen without waiting for the RM-idle condition
        automatic int idle_run = 0;
        automatic int guard = 0;
        quiesce <= 1'b1;
        while (idle_run < 24 && guard < 100000) begin
          @(posedge clk);
          guard++;
          if (rm_idle) idle_run++; else idle_run = 0;
        end
        if (guard >= 100000) check(1'b0, "quiesce: RM never became idle");
        u_c3s.freeze = 1'b1; u_hs.freeze = 1'b1;
        repeat ($urandom_range(40, 200)) @(posedge clk);
        u_c3s.freeze = 1'b0; u_hs.freeze = 1'b0;
        quiesce <= 1'b0;
        q_count++;
      end
    end
  end

  // --------------------------------------------------------------
  task automatic drain(input string what, input int maxcyc = 400000);
    int g = 0;
    bit busy;
    do begin
      @(negedge clk_ts); ts_step(); g++;
      busy = hold || (rbq.size() != 0) || mreq_v;
      for (int i = 0; i < NOUT; i++) if (ot[i].v || orph_pend[i]) busy = 1'b1;
    end while (busy && g < maxcyc);
    check(!busy, $sformatf("%s: everything answered (%0d clk_ts)", what, g));
    repeat (40) begin @(negedge clk_ts); ts_step(); end
    check(out_cnt == 0 && !fab_busy, $sformatf("%s: out_cnt back to 0 (%0d)", what, out_cnt));
  endtask

  task automatic run_reqs(input int n);
    gen_left = n; gen_en = 1'b1;
    while (gen_left > 0 || hold) begin @(negedge clk_ts); ts_step(); end
    gen_en = 1'b0;
  endtask

  initial begin
    int unsigned sd;
    sd = $urandom(SEED);
    sp_rst = 1'b0; mreq_v = 1'b0; mreq = '0; mrd_slot = '0; mfree = 1'b0; dma_en = 1'b0;
    for (int i = 0; i < NOUT; i++) begin ot[i] = '{default: '0}; orph_pend[i] = 1'b0; end
    repeat (6) @(posedge clk_ts);
    rst_n = 1'b1; rst_ts_n = 1'b1;
    repeat (6) @(posedge clk_ts);
    for (int h = 0; h < 2; h++) for (int l = 0; l < NL; l++) fill_line(h[0], POOL + 128 * l);

    // ---- throughput: 64 line reads per target, fixed slave latency, no other traffic
    $display("TEST throughput (NOUT=%0d)", NOUT);
    u_c3s.ready_pct = 100; u_c3s.gap_pct = 0; u_c3s.lat_min = 20;  u_c3s.lat_max = 20;
    u_hs.ready_pct  = 100; u_hs.gap_pct  = 0; u_hs.lat_min  = 100; u_hs.lat_max  = 100;
    for (int h = 0; h < 2; h++) begin
      longint unsigned t0, t1;
      for (int l = 0; l < 64; l++) fill_line(h[0], TPUT + 128 * l);
      for (int l = 0; l < 64; l++) dq.push_back(mk(M_RD, h[0], TPUT + 128 * l));
      t0 = tcyc;
      drain($sformatf("throughput %s", h ? "host" : "c3"));
      t1 = tcyc - 40;
      $display("  %s line reads: 64 in %0d clk_ts = %0d.%01d ns/line (slave latency %0d clk_l2), max in flight %0d",
               h ? "host" : "c3", t1 - t0, (t1 - t0) * 10 / 64, ((t1 - t0) * 100 / 64) % 10, h ? 100 : 20, max_out);
    end
    if (NOUT > 1) check(max_out >= 2, $sformatf("several lines in flight (max %0d)", max_out));
    if (NOUT == 1) check(max_out == 1, "NOUT = 1: one line in flight");
    u_c3s.ready_pct = 75; u_c3s.gap_pct = 15; u_c3s.lat_min = 2; u_c3s.lat_max = 30;
    u_hs.ready_pct  = 75; u_hs.gap_pct  = 15; u_hs.lat_min  = 2; u_hs.lat_max  = 60;

    // ---- directed same-line sequences, issued back to back
    $display("TEST same-line sequences");
    for (int h = 0; h < 2; h++) for (int rep = 0; rep < 6; rep++) begin
      longint unsigned lb;
      lb = POOL + 128 * rep;
      dq.push_back(mk_wr(h[0], lb, '1));                         // WR -> RD
      dq.push_back(mk(M_RD, h[0], lb));
      dq.push_back(mk_wr(h[0], lb, mk_be(2)));                   // RD -> WR
      dq.push_back(mk_wr(h[0], lb, mk_be(4)));                   // WR -> WR
      dq.push_back(mk(M_RD, h[0], lb));
      dq.push_back(mk(M_RD, h[0], lb));                          // RD -> RD (not ordered)
      dq.push_back(mk_wr(h[0], lb, mk_be(5)));
      dq.push_back(mk_amo(h[0], lb + 16, 1'b1, 5'd0, 64'h1234_5678_9ABC_DEF0, 64'h0));   // WR -> AMO
      dq.push_back(mk(M_RD, h[0], lb));
      dq.push_back(mk_wr(h[0], lb + 128, mk_be(1)));             // other line, other target in between
      dq.push_back(mk(M_RD, !h[0], lb));
      dq.push_back(mk_wr(h[0], lb, mk_be(3)));
      dq.push_back(mk(M_RD, h[0], lb));
      dq.push_back(mk_wr(h[0], lb, mk_be(7)));                   // holey qwords, back to back
      dq.push_back(mk_wr(h[0], lb, mk_be(7)));
      dq.push_back(mk(M_RD, h[0], lb));
    end
    drain("same-line sequences");
    for (int h = 0; h < 2; h++) for (int l = 0; l < NL; l++) chk_mem(h[0], POOL + 128 * l, "after the sequences");

    // ---- random: errors, DMA, quiesce, sp_rst
    $display("TEST random (%0d requests) + errors + dma + quiesce + sp_rst", NREQ);
    err_on = 1'b1;
    u_c3s.err_en = 1; u_c3s.err_lo = ERR_LO; u_c3s.err_hi = ERR_HI;
    u_hs.err_en  = 1; u_hs.err_lo  = ERR_LO; u_hs.err_hi  = ERR_HI;
    dma_en = 1'b1; q_enable = 1'b1; rst_en = 1'b1;
    run_reqs(NREQ);
    rst_en = 1'b0;
    drain("random");
    q_enable = 1'b0;
    repeat (2000) @(posedge clk);
    dma_en = 1'b0;
    repeat (2000) @(posedge clk);
    for (int h = 0; h < 2; h++) for (int l = 0; l < NL; l++) chk_mem(h[0], POOL + 128 * l, "after random");

    $display("  accepted %0d answered %0d orphans %0d (DUT %0d) sp_rst %0d fences %0d (DUT %0d) quiesce %0d",
             n_acc, n_ans, n_orph, t_orph, n_rst, n_fence_tb, t_fen, q_count);
    $display("  DUT: max in flight %0d, stalls: conflict %0d slot %0d fence %0d; bursts %0d ops %0d bad %0d fallback %0d",
             max_out, t_cst, t_sst, t_fst, sf_bursts, sf_ops, sf_bad, sf_wfb);
    $display("  dma bursts: host %0d c3 %0d", dh_ntrans, dc_ntrans);
    check(n_acc == n_ans + n_orph, $sformatf("accepted %0d == answered %0d + orphans %0d", n_acc, n_ans, n_orph));
    check(n_orph == int'(t_orph), "orphan count == DUT FAB_ORPHANS");
    check(n_fence_tb == int'(t_fen), "fence count == DUT FAB_FENCES");
    check(n_rst > 0 && q_count > 0, "sp_rst and quiesce episodes happened");
    check(dh_nerr == 0 && dc_nerr == 0 && dh_ntrans > 10 && dc_ntrans > 10, "dma bursts clean and concurrent");
    check(rm_idle, "RM idle at the end");
    check(u_c3s.errors == 0 && u_hs.errors == 0, "slave protocol assertions");
    check(u_hs.n_holey == 0, $sformatf(" no host write beat with a holey strobe (%0d)", u_hs.n_holey));
    check(q_hs_viol == 0, $sformatf("no AR / AW handshake after the quiesce acknowledge (%0d)", q_hs_viol));
    if (u_c3s.errors != 0 || u_hs.errors != 0) $display("  slave errors: c3 %0d host %0d", u_c3s.errors, u_hs.errors);
    if (NOUT > 1 && BURST) check(t_cst != 0, "same-line conflicts happened");
    $display("tb_rsp_fab_n: %0d checks, %0d errors (NOUT=%0d BURST=%0d HOST_WR_FLUSH=%0d WR_RUNS=%0d NREQ=%0d)", checks,
             errors, NOUT, BURST, HOST_WR_FLUSH, WR_RUNS, NREQ);
    if (errors == 0) $display("NG_RSP_FAB_N_TB_PASS"); else $display("NG_RSP_FAB_N_TB_FAIL");
    $finish;
  end

  initial begin
    #400_000_000;
    $display("FAIL: tb_rsp_fab_n global timeout");
    $display("NG_RSP_FAB_N_TB_FAIL");
    $finish;
  end
endmodule
