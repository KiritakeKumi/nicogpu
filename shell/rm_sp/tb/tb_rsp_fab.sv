// tb_rsp_fab.sv -- unit test of the SP fabric port memory side with real engines (the TB drives the
// slot port of ng_rsp_fab_ts one request at a time, the multi-request test is tb_rsp_fab_n.sv).
//
// DUT chain (the wiring of ng_gpu_rm_sp.sv, ts and l2 on ONE clock here):
//   TB (one request at a time, the ng_sp_top serial port) -> ng_rsp_mport1 -> ng_rsp_fab_ts -> CDC FIFOs / RAMs ->
//   ng_rsp_fab_l2 -> { burst masters -> ng_rcp_axmux (s1) ->
//   ng_rcp_hosteng / ng_rcp_c3eng b_* ; operation path -> ng_rsp_oparb -> engine requester 1 } -> ng_rmt_port ->
//   tb_rmt_slave (host DW 64 / C3 DW 128; protocol assertions, random ready / latency, SLVERR injection, freeze).
// A random burst master (stands in for ng_rcp_dma) drives s0 of both new muxes against a disjoint region while the SP
// requests run.  Reference model: byte-accurate memory per space.
// Tests: line RD / WR on both apertures (full, partial, holey, all-zero byte enables; route check burst vs fallback),
// WR -> RD / WR -> AMO / AMO -> WR RAW sequences, RRESP / BRESP errors (data and memory), bad PA, concurrent DMA
// bursts, quiesce in the middle (slave freeze after the RM-idle condition), cycles per line.
// Parameters (verilator -G / xelab -generic_top): BURST, HOST_WR_FLUSH, NRAND, NOUT (slots), WR_RUNS (
// 1 = host partial / holey writes as bursts of runs, 0 = the rule with the per-qword fallback).
// NOUT = 1, WR_RUNS = 0 is the configuration.  Pass: NG_RSP_FAB_TB_PASS.
`timescale 1ns / 1ps

// random burst master (DMA stand-in): write a random burst, read it back, compare
module tb_bmaster #(
  parameter int DW   = 64,
  parameter int AW   = 47,
  parameter longint unsigned BASE = 64'h10_0000,
  parameter int MAXB = 16
) (
  input  logic            clk,
  input  logic            rst_n,
  input  logic            en,
  output logic            awvalid,
  input  logic            awready,
  output logic [AW-1:0]   awaddr,
  output logic [7:0]      awlen,
  output logic            wvalid,
  input  logic            wready,
  output logic [DW-1:0]   wdata,
  output logic [DW/8-1:0] wstrb,
  output logic            wlast,
  input  logic            bvalid,
  output logic            bready,
  input  logic [1:0]      bresp,
  output logic            arvalid,
  input  logic            arready,
  output logic [AW-1:0]   araddr,
  output logic [7:0]      arlen,
  input  logic            rvalid,
  output logic            rready,
  input  logic [DW-1:0]   rdata,
  input  logic [1:0]      rresp,
  input  logic            rlast,
  output int              nerr,
  output int              ntrans
);
  localparam int BPB = DW / 8;
  typedef enum int {M_IDLE, M_WR, M_B, M_AR, M_R} ms_t;
  ms_t ms;
  logic [DW-1:0] ref_mem [longint unsigned];
  logic [DW-1:0] wd [MAXB];
  int            beats, wb, rb;
  longint unsigned a0;
  logic          aw_done, ar_done;

  always @(posedge clk) begin
    if (!rst_n) begin
      ms <= M_IDLE; awvalid <= 1'b0; wvalid <= 1'b0; arvalid <= 1'b0; bready <= 1'b0; rready <= 1'b0;
      nerr <= 0; ntrans <= 0; wlast <= 1'b0; awaddr <= '0; araddr <= '0; awlen <= '0; arlen <= '0;
      wdata <= '0; wstrb <= '0;
    end else begin
      rready <= ($urandom_range(0, 99) < 70);
      unique case (ms)
        M_IDLE: if (en && $urandom_range(0, 99) < 40) begin
          int pg, off;
          beats = $urandom_range(1, MAXB);
          pg  = $urandom_range(0, 15);
          off = $urandom_range(0, 4096 / BPB - beats);
          a0  = BASE + 4096 * pg + BPB * off;
          for (int i = 0; i < beats; i++) begin
            for (int k = 0; k < DW / 32; k++) wd[i][32*k +: 32] = $urandom;
            ref_mem[(a0 / BPB) + i] = wd[i];
          end
          awvalid <= 1'b1; awaddr <= AW'(a0); awlen <= 8'(beats - 1);
          wvalid <= 1'b1; wdata <= wd[0]; wstrb <= '1; wlast <= (beats == 1);
          wb = 0; aw_done = 1'b0;
          ms <= M_WR;
        end
        M_WR: begin
          if (awvalid && awready) begin awvalid <= 1'b0; end
          if (wvalid && wready) begin
            if (wb == beats - 1) begin
              wvalid <= 1'b0; wlast <= 1'b0; bready <= 1'b1; ms <= M_B;
            end else begin
              wb = wb + 1;
              wdata <= wd[wb]; wlast <= (wb == beats - 1);
            end
          end
        end
        M_B: begin
          if (awvalid && awready) awvalid <= 1'b0;
          if (bvalid && bready) begin
            if (bresp != 2'b00) begin nerr <= nerr + 1; $display("FAIL: bmaster bresp %0d", bresp); end
            bready <= 1'b0;
            arvalid <= 1'b1; araddr <= AW'(a0); arlen <= 8'(beats - 1);
            rb = 0;
            ms <= M_R;
          end
        end
        M_R: begin
          if (arvalid && arready) arvalid <= 1'b0;
          if (rvalid && rready) begin
            if (rdata !== wd[rb] || rresp != 2'b00) begin
              nerr <= nerr + 1;
              $display("FAIL: bmaster read beat %0d of burst at %h: %h vs %h resp %0d", rb, a0, rdata, wd[rb], rresp);
            end
            if (rlast != (rb == beats - 1)) begin nerr <= nerr + 1; $display("FAIL: bmaster rlast"); end
            rb = rb + 1;
            if (rlast) begin ntrans <= ntrans + 1; ms <= M_IDLE; end
          end
        end
        default: ms <= M_IDLE;
      endcase
    end
  end
endmodule

// ================================================================================================================
module tb_rsp_fab;
  import ng_lsu_pkg::*;

  parameter bit BURST         = 1'b1;
  parameter bit HOST_WR_FLUSH = 1'b0;
  parameter int NRAND         = 400;
  parameter int NOUT          = 8;
  parameter bit WR_RUNS       = 1'b1;
  localparam int SW = (NOUT > 1) ? $clog2(NOUT) : 1;
  localparam int HW = SW + 124;

  int errors = 0;
  int checks = 0;
  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #2.0 clk = ~clk;
  longint unsigned cycles = 0;
  always @(posedge clk) cycles++;

  localparam longint unsigned SYS_BASE  = 64'h10_0000_0000;
  localparam longint unsigned HOST_BASE = 64'h8000_0000_0000;

  task automatic check(input bit cond, input string what);
    checks++;
    if (!cond) begin errors++; if (errors < 30) $display("FAIL @%0t: %s", $time, what); end
  endtask

  // ---- fab_ts / fab_l2 and their ports
  logic         quiesce;
  logic          mreq_v, mreq_rdy, mrsp_v, mrsp_rd, mfree;
  mreq_t         mreq;
  logic [SW-1:0] mreq_slot, mrsp_slot, mrd_slot;
  logic [1:0]    mrsp_err;
  logic [63:0]   mrsp_old;
  logic [1023:0] mrd_data;
  logic          sp_v, sp_rdy, sp_rv, tb_sp_rst;      // the serial port of ng_sp_top (through ng_rsp_mport1)
  mreq_t         sp_rq;
  mrsp_t         sp_rs;
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

  // the RM-idle condition of ng_gpu_rm_sp.sv rm_quiesce_ack (without the QAGE delay: the quiesce agent waits 24 clocks)
  // (ng_rcp_hosteng's idle no longer counts a flush read held off by quiesce as busy --; the TB uses the real idle.)
  wire rm_idle = c3_idle && host_idle && hbm_idle && c3bm_idle;

  ng_rsp_fab_ts #(.NOUT(NOUT)) u_ts (
    .clk(clk), .rst_n(rst_n), .sp_rst(tb_sp_rst),
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

  ng_rsp_mport1 #(.SW(SW)) u_mp (
    .clk(clk), .rst_n(rst_n), .sp_rst(tb_sp_rst),
    .sp_req_valid(sp_v), .sp_req_ready(sp_rdy), .sp_req(sp_rq), .sp_rsp_valid(sp_rv), .sp_rsp(sp_rs),
    .f_req_valid(mreq_v), .f_req_ready(mreq_rdy), .f_req(mreq), .f_rsp_valid(mrsp_v), .f_rsp_slot(mrsp_slot),
    .f_rsp_err(mrsp_err), .f_rsp_rd(mrsp_rd), .f_rsp_old(mrsp_old), .f_rd_slot(mrd_slot), .f_rd_data(mrd_data),
    .f_free(mfree));

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

  // ---- the engines and the slaves
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

  // ---------------------------------------------------------------- reference model + helpers
  logic [7:0] refc [longint unsigned];     // C3 byte reference (offset)
  logic [7:0] refh [longint unsigned];     // host byte reference

  function automatic logic [7:0] rbyte(input bit host, input longint unsigned a);
    if (host) return refh.exists(a) ? refh[a] : 8'h00;
    else      return refc.exists(a) ? refc[a] : 8'h00;
  endfunction
  function automatic logic [7:0] mbyte(input bit host, input longint unsigned a);   // slave memory
    logic [127:0] c;
    logic [63:0]  h;
    if (host) begin h = u_hs.peek(a >> 3); return h[8 * a[2:0] +: 8]; end
    c = u_c3s.peek(a >> 4); return c[8 * a[3:0] +: 8];
  endfunction
  task automatic mset(input bit host, input longint unsigned a, input logic [7:0] v);   // both models
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

  function automatic longint unsigned pa_of(input bit host, input longint unsigned off);
    return host ? (HOST_BASE + off) : (SYS_BASE + off);
  endfunction

  function automatic mreq_t mk(input logic [2:0] kind, input longint unsigned pa);
    mreq_t r;
    r = '0;
    r.kind = kind; r.pa = 48'(pa); r.uc = 1'b1;
    return r;
  endfunction

  // one SP request through the serial port (ng_sp_top fa_st semantics: valid held until ready, then wait for the
  // answer pulse); ng_rsp_mport1 turns it into the slot port of ng_rsp_fab_ts.  Returns the number of clocks used.
  task automatic sp_issue(input mreq_t r);
    int guard = 0;
    @(negedge clk);
    sp_rq = r; sp_v = 1'b1;
    #0.1;                                                   // ready is combinational from the request: let it settle
    while (!sp_rdy) begin
      @(negedge clk);
      #0.1;
      guard++;
      if (guard > 200000) begin check(1'b0, "sp_req: never accepted"); break; end
    end
    @(negedge clk);                                         // accepted at the posedge in between
    sp_v = 1'b0;
  endtask
  task automatic sp_wait(output logic [1023:0] rd, output logic [63:0] old, output logic [1:0] err);
    int guard = 0;
    while (!sp_rv) begin
      @(negedge clk);
      guard++;
      if (guard > 200000) begin check(1'b0, "sp_req timeout"); break; end
    end
    rd = sp_rs.rdata; old = sp_rs.old; err = sp_rs.err;
  endtask
  task automatic sp_req(input mreq_t r, output logic [1023:0] rd, output logic [63:0] old, output logic [1:0] err,
                        output int cyc);
    longint unsigned t0;
    logic [31:0] nr0;
    nr0 = t_nreq;
    t0 = cycles;
    sp_issue(r);
    sp_wait(rd, old, err);
    cyc = int'(cycles - t0);
    check(t_nreq == nr0 + 32'd1 && out_cnt <= 1, "sp_req: exactly one request accepted, one slot in use");
  endtask

  // line write: update the reference (be bytes), run, check err and the three lines around it
  task automatic wr_line(input bit host, input longint unsigned lb, input logic [127:0] be, output int cyc,
                         input bit exp_err = 1'b0);
    mreq_t r;
    logic [1023:0] rd; logic [63:0] old; logic [1:0] err;
    r = mk(M_WR, pa_of(host, lb));
    r.be = be;
    for (int k = 0; k < 32; k++) r.wdata[32*k +: 32] = $urandom;
    sp_req(r, rd, old, err, cyc);
    if (exp_err) begin
      check(err == 2'd3, $sformatf("WR err expected ERR_DDR, got %0d", err));
    end else begin
      check(err == 2'd0, $sformatf("WR %s %h err %0d", host ? "host" : "c3", lb, err));
      for (int i = 0; i < 128; i++) if (be[i]) begin
        if (host) refh[lb + i] = r.wdata[8*i +: 8]; else refc[lb + i] = r.wdata[8*i +: 8];
      end
    end
    chk_mem(host, lb, "after WR");
    if (lb >= 128) chk_mem(host, lb - 128, "after WR (prev line)");
    chk_mem(host, lb + 128, "after WR (next line)");
  endtask

  task automatic rd_line(input bit host, input longint unsigned lb, output int cyc, input bit exp_err = 1'b0);
    mreq_t r;
    logic [1023:0] rd; logic [63:0] old; logic [1:0] err;
    bit ok = 1'b1;
    r = mk(M_RD, pa_of(host, lb));
    sp_req(r, rd, old, err, cyc);
    if (exp_err) begin
      check(err == 2'd3, $sformatf("RD err expected ERR_DDR, got %0d", err));
      for (int i = 0; i < 128; i++) if (rd[8*i +: 8] != 8'h00) ok = 1'b0;
      check(ok, "RD error: all data beats read as 0");
    end else begin
      check(err == 2'd0, $sformatf("RD %s %h err %0d", host ? "host" : "c3", lb, err));
      for (int i = 0; i < 128; i++) if (rd[8*i +: 8] !== rbyte(host, lb + i)) begin
        if (ok) $display("  RD mismatch %s line %h byte %0d: got %h exp %h", host ? "host" : "c3", lb, i, rd[8*i +: 8],
                         rbyte(host, lb + i));
        ok = 1'b0;
      end
      check(ok, "RD data == reference");
    end
  endtask

  // AMO at exact byte address (aligned); ops: ADD 0, XOR 4, XCHG 9, CAS 10
  task automatic amo(input bit host, input longint unsigned off, input bit w64, input logic [4:0] op,
                     input logic [63:0] a, input logic [63:0] cmp, output int cyc);
    mreq_t r;
    logic [1023:0] rd; logic [63:0] old, e_old, e_new, cur;
    logic [1:0] err;
    r = mk(M_AMO, pa_of(host, off));
    r.amo_op = op; r.amo64 = w64; r.amo_a = a; r.amo_cmp = cmp;
    cur = '0;
    for (int i = 0; i < (w64 ? 8 : 4); i++) cur[8*i +: 8] = rbyte(host, off + i);
    sp_req(r, rd, old, err, cyc);
    e_old = w64 ? cur : {32'h0, cur[31:0]};
    unique case (op)
      5'd0:  e_new = cur + a;
      5'd4:  e_new = cur ^ a;
      5'd9:  e_new = a;
      default: e_new = (w64 ? (cur == cmp) : (cur[31:0] == cmp[31:0])) ? a : cur;
    endcase
    check(err == 2'd0, $sformatf("AMO err %0d", err));
    check(old == e_old, $sformatf("AMO old %h exp %h", old, e_old));
    for (int i = 0; i < (w64 ? 8 : 4); i++) begin
      if (host) refh[off + i] = e_new[8*i +: 8]; else refc[off + i] = e_new[8*i +: 8];
    end
    chk_mem(host, off & ~64'h7F, "after AMO");
  endtask

  // byte-enable pattern generators: 0 full, 1 contiguous qword run, 2 byte-contiguous run (partial edge qwords),
  // 3 single bytes, 4 random holey, 5 half the lanes every other qword, 7  per qword one of the XDMA hole
  // patterns (xdma_q), 6 all zero
  function automatic logic [7:0] xdma_q(input int k);   // 0x11 << r, 0x55, 0xAA, 0x81, 0x3C, 0xFF, 0, random
    unique case (k)
      0, 1, 2, 3: return 8'h11 << k;
      4:          return 8'h55;
      5:          return 8'hAA;
      6:          return 8'h81;
      7:          return 8'h3C;
      8:          return 8'hFF;
      9:          return 8'h00;
      default:    return 8'($urandom);
    endcase
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
      7: begin for (int q = 0; q < 16; q++) b[8*q +: 8] = xdma_q($urandom_range(0, 10)); end
      default: b = '0;
    endcase
    return b;
  endfunction
  function automatic int byte_runs(input logic [7:0] s);   // runs of enabled bytes in one qword
    bit prev;
    byte_runs = 0; prev = 1'b0;
    for (int i = 0; i < 8; i++) begin if (s[i] && !prev) byte_runs++; prev = s[i]; end
  endfunction
  // does ng_rsp_fab_l2 use bursts for this write (host rule: whole qwords, one contiguous run; WR_RUNS: any
  // non-zero; C3: any non-zero)
  function automatic bit exp_burst(input bit host, input logic [127:0] be);
    logic [15:0] m, f;
    int runs;
    bit prev;
    if (!BURST || be == '0) return 1'b0;
    if (!host || WR_RUNS) return 1'b1;
    for (int q = 0; q < 16; q++) begin m[q] = |be[8*q +: 8]; f[q] = &be[8*q +: 8]; end
    if ((m & ~f) != 16'h0) return 1'b0;
    runs = 0; prev = 1'b0;
    for (int q = 0; q < 16; q++) begin if (m[q] && !prev) runs++; prev = m[q]; end
    return (runs == 1);
  endfunction
  // number of bursts of a bursted write (runs: consecutive whole qwords, every partial qword alone --
  // one 1-beat write per byte run of a holey qword; C3: 1), + the flush read with HOST_WR_FLUSH
  function automatic int exp_nburst(input bit host, input logic [127:0] be);
    logic [15:0] m, f;
    int runs;
    if (!host) return 1;
    for (int q = 0; q < 16; q++) begin m[q] = |be[8*q +: 8]; f[q] = &be[8*q +: 8]; end
    runs = 0;
    for (int q = 0; q < 16; q++)
      if (m[q] && (!f[q] || q == 0 || !(m[q-1] && f[q-1]))) runs += f[q] ? 1 : byte_runs(be[8*q +: 8]);
    return runs + (HOST_WR_FLUSH ? 1 : 0);
  endfunction

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

  // ---------------------------------------------------------------- quiesce agent (background)
  bit     q_enable = 1'b0;
  int     q_count = 0;
  int     q_viol = 0;
  logic   frozen_chk;
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
        if (guard >= 100000) begin
          check(1'b0, "quiesce: RM never became idle");
          $display("  idle flags: c3 %0b host %0b hbm %0b c3bm %0b, fab st %0d, c3 slave out wr %0d rd %0d, host wr %0d rd %0d, hosteng st %0d flushing %0b",
                   c3_idle, host_idle, hbm_idle, c3bm_idle, sf_st, u_c3s.wr_out, u_c3s.rd_out, u_hs.wr_out,
                   u_hs.rd_out, u_host.st, u_host.flushing);
        end
        u_c3s.freeze = 1'b1; u_hs.freeze = 1'b1;
        repeat ($urandom_range(40, 200)) @(posedge clk);
        u_c3s.freeze = 1'b0; u_hs.freeze = 1'b0;
        quiesce <= 1'b0;
        q_count++;
      end
    end
  end

  // --------------------------------------------------------------
  int  c;
  logic [1023:0] rd;
  logic [63:0] old;
  logic [1:0]  err;
  int  b0, f0;
  logic [127:0] be;
  longint unsigned lb;
  int  tot_rd_h, tot_rd_c, tot_wr_h, tot_wr_c;

  task automatic wr_check_route(input bit host, input longint unsigned off, input logic [127:0] be_, input string nm);
    int bb, ff;
    bb = int'(sf_bursts); ff = int'(sf_wfb);
    wr_line(host, off, be_, c);
    if (be_ != '0) begin
      if (exp_burst(host, be_)) begin
        check(int'(sf_bursts) - bb == exp_nburst(host, be_) && int'(sf_wfb) == ff,
              $sformatf("%s: burst route (%0d bursts, expected %0d)", nm, int'(sf_bursts) - bb, exp_nburst(host, be_)));
      end else if (BURST) begin
        check(int'(sf_wfb) - ff == 1, {nm, ": fallback route"});
      end
    end
  endtask

  initial begin
    sp_v = 1'b0; sp_rq = '0; tb_sp_rst = 1'b0; dma_en = 1'b0;
    rst_n = 1'b0;
    repeat (8) @(posedge clk);
    rst_n <= 1'b1;
    repeat (4) @(posedge clk);

    // ---- directed: reads
    $display("TEST rd");
    for (int h = 0; h < 2; h++) begin
      fill_line(h[0], 64'h1000); fill_line(h[0], 64'h1080);
      rd_line(h[0], 64'h1000, c);
      rd_line(h[0], 64'h1080, c);
    end

    // ---- directed: writes of every byte-enable class, both apertures
    $display("TEST wr");
    for (int h = 0; h < 2; h++) begin
      for (int cls = 0; cls <= 7; cls++) begin
        for (int rep = 0; rep < 6; rep++) begin
          lb = 64'h2000 + 128 * (rep + 8 * cls);
          fill_line(h[0], lb);                       // old content that must survive where be == 0
          fill_line(h[0], lb + 128);
          if (lb >= 128) fill_line(h[0], lb - 128);
          be = mk_be(cls);
          wr_check_route(h[0], lb, be, $sformatf("cls %0d", cls));
          rd_line(h[0], lb, c);
        end
      end
    end
    // edge cases: first / last qword only, last byte of the line, upper half only
    for (int h = 0; h < 2; h++) begin
      lb = 64'h4000;
      fill_line(h[0], lb);
      be = '0; be[7:0] = 8'hFF;                         wr_check_route(h[0], lb, be, "q0");
      be = '0; be[127:120] = 8'hFF;                     wr_check_route(h[0], lb, be, "q15");
      be = '0; be[127] = 1'b1;                          wr_check_route(h[0], lb, be, "last byte");
      be = '0; be[127:64] = '1;                         wr_check_route(h[0], lb, be, "upper half");
      be = '0; be[63:0] = '1;                           wr_check_route(h[0], lb, be, "lower half");
      be = '0; be[15:0] = 16'hFFFF; be[111:96] = '1;    wr_check_route(h[0], lb, be, "two runs");
    end

    // ---- holey byte strobes inside a qword (the host slave is the XDMA model: a holey beat FAILs and is
    //      written as the range [first .. last byte]); every pattern alone in one qword, in all 16 qwords, and mixed
    //      rows of whole + holey qwords
    $display("TEST xdma strobes");
    for (int h = 0; h < 2; h++) begin
      for (int k = 0; k <= 8; k++) begin
        automatic logic [7:0] pq = xdma_q(k);
        lb = 64'h5000 + 128 * k;
        fill_line(h[0], lb); fill_line(h[0], lb + 128);
        be = '0; be[8 * (k % 16) +: 8] = pq;            wr_check_route(h[0], lb, be, $sformatf("one qword %02h", pq));
        be = '0; be[8 * (15 - k) +: 8] = pq;            wr_check_route(h[0], lb, be, $sformatf("qword hi %02h", pq));
        for (int q = 0; q < 16; q++) be[8*q +: 8] = pq; wr_check_route(h[0], lb, be, $sformatf("all qwords %02h", pq));
        rd_line(h[0], lb, c);
      end
      for (int rep = 0; rep < 8; rep++) begin
        lb = 64'h5800 + 128 * rep;
        fill_line(h[0], lb); fill_line(h[0], lb + 128); fill_line(h[0], lb - 128);
        // whole qwords around holey ones: FF FF 55 FF FF FF 81 3C AA FF FF 44 00 FF 88 FF, rotated by rep
        for (int q = 0; q < 16; q++) begin
          automatic int sel [16] = '{8, 8, 4, 8, 8, 8, 6, 7, 5, 8, 8, 2, 9, 8, 3, 8};
          be[8*q +: 8] = xdma_q(sel[(q + rep) % 16]);
        end
        wr_check_route(h[0], lb, be, $sformatf("mixed row %0d", rep));
        rd_line(h[0], lb, c);
        be = mk_be(7);                                  wr_check_route(h[0], lb, be, $sformatf("xdma row %0d", rep));
        rd_line(h[0], lb, c);
      end
    end
    check(u_hs.n_holey == 0, $sformatf("xdma strobes: no host write beat with a holey strobe (%0d)", u_hs.n_holey));

    // ---- RAW: write then read / atomic, atomic then write then read, same address
    $display("TEST raw");
    for (int h = 0; h < 2; h++) begin
      for (int rep = 0; rep < 8; rep++) begin
        lb = 64'h6000 + 128 * rep;
        fill_line(h[0], lb);
        be = '1;
        wr_line(h[0], lb, be, c);
        amo(h[0], lb + 8 * $urandom_range(0, 15), 1'b1, 5'd0, 64'h0123_4567_89AB_CDEF, 64'h0, c);
        amo(h[0], lb + 4 * $urandom_range(0, 31), 1'b0, 5'd0, 64'h77, 64'h0, c);
        rd_line(h[0], lb, c);
        amo(h[0], lb + 8, 1'b1, 5'd9, 64'hDEAD_BEEF_0000_0001, 64'h0, c);
        be = mk_be(1);
        wr_line(h[0], lb, be, c);
        rd_line(h[0], lb, c);
        amo(h[0], lb + 16, 1'b0, 5'd10, 64'h55, 64'h0, c);           // CAS usually fails
        amo(h[0], lb + 16, 1'b1, 5'd4, 64'hFFFF_FFFF_FFFF_FFFF, 64'h0, c);
        wr_line(h[0], lb, mk_be(rep % 5), c);
        rd_line(h[0], lb, c);
      end
    end

    // ---- errors on R / B and bad addresses
    $display("TEST errors");
    for (int h = 0; h < 2; h++) begin
      lb = 64'h8000;
      fill_line(h[0], lb); fill_line(h[0], lb + 128);
      // read error: whole line in the error window
      if (h == 0) begin u_c3s.err_en = 1; u_c3s.err_lo = lb; u_c3s.err_hi = lb + 128; end
      else        begin u_hs.err_en = 1;  u_hs.err_lo = lb;  u_hs.err_hi = lb + 128; end
      rd_line(h[0], lb, c, 1'b1);
      rd_line(h[0], lb + 128, c);                        // next line is fine
      // write error: memory must stay as it was (slave drops an erroneous burst), request answers ERR_DDR
      wr_line(h[0], lb, '1, c, 1'b1);
      wr_line(h[0], lb, mk_be(2), c, 1'b1);
      wr_line(h[0], lb + 128, '1, c);                    // no error beyond the window
      u_c3s.err_en = 0; u_hs.err_en = 0;
      rd_line(h[0], lb, c);                              // recovered
      wr_line(h[0], lb, '1, c);
    end
    begin
      mreq_t r;
      int a0, r0, a1, r1;
      a0 = u_c3s.n_aw + u_hs.n_aw; r0 = u_c3s.n_ar + u_hs.n_ar;
      r = mk(M_RD, 64'h1000);                       sp_req(r, rd, old, err, c); check(err == 2'd1, "VRAM RD -> BAD_PA");
      r = mk(M_WR, 64'h20_0000_0000); r.be = '1;    sp_req(r, rd, old, err, c); check(err == 2'd1, "hole WR -> BAD_PA");
      r = mk(M_RD, SYS_BASE + 64'h1000); r.uc = 1'b0; sp_req(r, rd, old, err, c); check(err == 2'd1, "non-uc -> BAD_PA");
      r = mk(M_AMO, SYS_BASE + 64'h1002); r.amo_op = 5'd0; sp_req(r, rd, old, err, c); check(err == 2'd1, "misaligned AMO");
      r = mk(M_INV_L1, 64'h0);                      sp_req(r, rd, old, err, c); check(err == 2'd0, "INV_L1 acked");
      r = mk(M_WR, SYS_BASE + 64'h1000); r.be = '0; sp_req(r, rd, old, err, c); check(err == 2'd0, "be == 0 acked");
      a1 = u_c3s.n_aw + u_hs.n_aw; r1 = u_c3s.n_ar + u_hs.n_ar;
      check(a1 == a0 && r1 == r0, "bad / maintenance / be==0 requests touch no bus");
    end

    // ---- SP reset with a request in flight -> the adapter answers it once (dummy, dropped by the SP), the
    //      real answer is reclaimed as an orphan, a write still lands, the next request to the line waits for it
    $display("TEST sp_rst in flight");
    u_c3s.lat_min = 150; u_c3s.lat_max = 150; u_hs.lat_min = 150; u_hs.lat_max = 150;
    for (int h = 0; h < 2; h++) for (int k = 0; k < 2; k++) begin
      mreq_t r;
      int g, nrv;
      logic [31:0] or0;
      lb = 64'h9000 + 128 * (2 * h + k);
      fill_line(h[0], lb);
      or0 = t_orph;
      if (k == 0) r = mk(M_RD, pa_of(h[0], lb));
      else begin
        r = mk(M_WR, pa_of(h[0], lb)); r.be = '1;
        for (int q = 0; q < 32; q++) r.wdata[32*q +: 32] = $urandom;
        for (int i = 0; i < 128; i++) if (h) refh[lb + i] = r.wdata[8*i +: 8]; else refc[lb + i] = r.wdata[8*i +: 8];
      end
      sp_issue(r);
      repeat (20) @(negedge clk);
      check(!sp_rv && out_cnt == 1, "sp_rst test: request in flight");
      tb_sp_rst = 1'b1;
      repeat (3) @(negedge clk);
      tb_sp_rst = 1'b0;
      nrv = 0; g = 0;
      while (g < 8000) begin @(negedge clk); g++; if (sp_rv) nrv++; end
      check(nrv == 1, $sformatf("sp_rst test: exactly one (dummy) answer to the SP (%0d)", nrv));
      check(t_orph == or0 + 32'd1, "sp_rst test: the real answer was reclaimed as an orphan");
      check(out_cnt == 0, "sp_rst test: slot free again");
      if (k == 1) chk_mem(h[0], lb, "sp_rst test: the orphan write landed");
      rd_line(h[0], lb, c);
    end
    u_c3s.lat_min = 2; u_c3s.lat_max = 30; u_hs.lat_min = 2; u_hs.lat_max = 30;

    // ---- cycles per line (no contention, fast slave)
    $display("TEST cycles");
    u_c3s.ready_pct = 100; u_c3s.gap_pct = 0; u_c3s.lat_min = 8; u_c3s.lat_max = 8;
    u_hs.ready_pct = 100;  u_hs.gap_pct = 0;  u_hs.lat_min = 8;  u_hs.lat_max = 8;
    for (int h = 0; h < 2; h++) begin
      automatic int sr = 0, sw = 0, sp = 0;
      for (int rep = 0; rep < 8; rep++) begin
        rd_line(h[0], 64'hA000 + 128 * rep, c); sr += c;
        wr_line(h[0], 64'hB000 + 128 * rep, '1, c); sw += c;
        wr_line(h[0], 64'hC000 + 128 * rep, mk_be(2), c); sp += c;
      end
      $display("  cycles/line (%s, slave latency 8, BURST=%0d NOUT=%0d WR_RUNS=%0d): RD %0d  WR full %0d  WR byte-run %0d",
               h ? "host" : "c3", BURST, NOUT, WR_RUNS, sr / 8, sw / 8, sp / 8);
    end
    u_c3s.ready_pct = 75; u_c3s.gap_pct = 15; u_c3s.lat_min = 2; u_c3s.lat_max = 30;
    u_hs.ready_pct = 75;  u_hs.gap_pct = 15;  u_hs.lat_min = 2;  u_hs.lat_max = 30;

    // ---- random mix with concurrent DMA bursts and quiesce
    $display("TEST random + dma + quiesce");
    dma_en = 1'b1;
    q_enable = 1'b1;
    for (int n = 0; n < NRAND; n++) begin
      automatic bit h = $urandom_range(0, 1);
      automatic int k = $urandom_range(0, 9);
      lb = 64'hE000 + 128 * $urandom_range(0, 63);
      if (k < 3) begin
        if ($urandom_range(0, 3) == 0) fill_line(h, lb);
        rd_line(h, lb, c);
      end else if (k < 8) begin
        wr_line(h, lb, mk_be($urandom_range(0, 7)), c);
      end else begin
        automatic longint unsigned ao = lb + 8 * $urandom_range(0, 15);
        automatic bit w64 = $urandom_range(0, 1);
        amo(h, w64 ? ao : ao + 4 * $urandom_range(0, 1), w64, ($urandom_range(0, 1) ? 5'd0 : 5'd9),
            {$urandom, $urandom}, 64'h0, c);
      end
    end
    q_enable = 1'b0;
    repeat (4000) @(posedge clk);
    dma_en = 1'b0;
    repeat (2000) @(posedge clk);
    $display("  dma bursts: host %0d c3 %0d, quiesce episodes %0d, fab bursts %0d, fallback writes %0d, ops %0d",
             dh_ntrans, dc_ntrans, q_count, sf_bursts, sf_wfb, sf_ops);
    check(dh_nerr == 0 && dc_nerr == 0, "dma bursts clean");
    check(dh_ntrans > 20 && dc_ntrans > 20, "dma bursts ran concurrently");
    check(q_count > 0, "quiesce episodes happened");
    check(rm_idle, "RM idle at the end");
    check(u_c3s.errors == 0 && u_hs.errors == 0, "slave protocol assertions");
    check(u_hs.n_holey == 0, $sformatf(" no host write beat with a holey strobe (%0d)", u_hs.n_holey));
    check(q_hs_viol == 0, $sformatf("no AR / AW handshake after the quiesce acknowledge (%0d)", q_hs_viol));
    if (u_c3s.errors != 0 || u_hs.errors != 0) $display("  slave errors: c3 %0d host %0d", u_c3s.errors, u_hs.errors);
    if (BURST) begin
      check(u_c3s.n_aw > 0 && u_c3s.aw_len_hist[7] > 0 && u_c3s.ar_len_hist[7] > 0, "C3 8-beat bursts seen");
      check(u_hs.aw_len_hist[15] > 0 && u_hs.ar_len_hist[15] > 0, "host 16-beat bursts seen");
    end
    check(out_cnt == 0 && !fab_busy, "all slots free at the end");
    check(t_orph == 4, "orphans only from the sp_rst test (4)");
    $display("  slots: NOUT %0d max in use %0d (serial TB: 1), fences %0d", NOUT, max_out, t_fen);
    $display("tb_rsp_fab: %0d checks, %0d errors (BURST=%0d HOST_WR_FLUSH=%0d NOUT=%0d WR_RUNS=%0d)", checks, errors, BURST,
             HOST_WR_FLUSH, NOUT, WR_RUNS);
    if (errors == 0) $display("NG_RSP_FAB_TB_PASS"); else $display("NG_RSP_FAB_TB_FAIL");
    $finish;
  end

  initial begin
    #200_000_000;
    $display("FAIL: tb_rsp_fab global timeout");
    $display("NG_RSP_FAB_TB_FAIL");
    $finish;
  end
endmodule
