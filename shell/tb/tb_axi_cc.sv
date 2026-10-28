// tb_axi_cc.sv -- self-checking random testbench for ng_axi_cc (P0-11).
//
// Two unrelated clocks (periods in ps, jitter-free, non-integer ratio).  Every
// channel is an independent stream of random payloads:
//   AW/W/AR  source BFM on the s side  -> sink BFM on the m side
//   B/R      source BFM on the m side  -> sink BFM on the s side
// Payload n of a channel is a pure function of (channel salt, n), so the sink
// checks that each payload arrives exactly once, in order, bit-exact without
// a cross-domain queue.  Sources insert random valid gaps and idle bursts;
// sinks drop ready randomly and take long stalls (100..600 cycles) so every
// FIFO fills.  A protocol checker on all ten interfaces flags X on
// valid/ready, X payload while valid, valid dropped before ready, and payload
// changing while valid && !ready.
// Phases:
//   A  main soak (N_MAIN per channel), each FIFO must see backpressure
//   B  drained reset of s side alone, then of m side alone; all valids low
//      during reset, traffic passes afterwards
//   C  in-flight flush: every FIFO filled (all five DUT input readies low),
//      then one side reset -> both sides must come back empty (cross-coupled
//      reset), readies high, traffic passes afterwards; done for s and for m
// Pass token: NG_AXI_CC_TB_PASS
`timescale 1ps / 1ps

package tbcc_pkg;
  function automatic logic [31:0] h32(input logic [31:0] x);
    x = x ^ (x >> 16); x = x * 32'h7feb352d;
    x = x ^ (x >> 15); x = x * 32'h846ca68b;
    x = x ^ (x >> 16);
    return x;
  endfunction
endpackage

`define TBCC_FAIL(msg) begin $display("NG_AXI_CC_TB FAIL @%0t ps: %s", $time, msg); $fatal(1, "NG_AXI_CC_TB FAIL"); end

`define TBCC_GEN \
  localparam int NCH = (W + 31) / 32; \
  function automatic logic [W-1:0] gen(input int unsigned i); \
    logic [NCH*32-1:0] t; \
    for (int k = 0; k < NCH; k++) \
      t[k*32 +: 32] = tbcc_pkg::h32(i * 32'h9E3779B9 ^ SALT * 32'h85EBCA6B ^ (k + 1) * 32'hC2B2AE35); \
    return t[W-1:0]; \
  endfunction

// ---------------------------------------------------------------- source ---
module tbcc_src #(parameter int W = 8, parameter int SALT = 0, parameter int GAP_PCT = 30) (
  input  logic        clk,
  input  logic        flush,     // drop valid immediately (reset phases only)
  input  int unsigned tgt,       // send payloads 0..tgt-1
  output logic        valid,
  input  logic        ready,
  output logic [W-1:0] data
);
  `TBCC_GEN
  int unsigned sent = 0, bp_cycles = 0, idle = 0, s;
  initial begin valid = 1'b0; data = '0; end
  always @(posedge clk) begin
    if (flush) begin
      valid <= 1'b0; idle <= 0;
    end else begin
      s = sent;
      if (valid && ready) s = s + 1;
      if (valid && !ready) bp_cycles <= bp_cycles + 1;
      sent <= s;
      if (valid && !ready) ;                                   // hold
      else if (idle != 0) begin valid <= 1'b0; idle <= idle - 1; end
      else if (s < tgt && ($urandom % 100) >= GAP_PCT) begin valid <= 1'b1; data <= gen(s); end
      else begin
        valid <= 1'b0;
        data  <= {NCH{$urandom}};                              // garbage when idle
        if ($urandom % 400 == 0) idle <= 20 + $urandom % 200;
      end
    end
  end
endmodule

// ------------------------------------------------------------------ sink ---
module tbcc_snk #(parameter int W = 8, parameter int SALT = 0, parameter string NAME = "") (
  input  logic        clk,
  input  logic        flush,     // hold ready low
  input  logic        stall_all, // hold ready low (fill test)
  input  logic        valid,
  output logic        ready,
  input  logic [W-1:0] data
);
  `TBCC_GEN
  int unsigned rcvd = 0, stall = 0, long_stalls = 0;
  initial ready = 1'b0;
  always @(posedge clk) begin
    if (valid === 1'b1 && ready === 1'b1) begin
      if (data !== gen(rcvd)) begin
        $display("  %s beat %0d: got %h", NAME, rcvd, data);
        $display("  %s beat %0d: exp %h", NAME, rcvd, gen(rcvd));
        `TBCC_FAIL($sformatf("%s scoreboard mismatch (lost/duplicated/reordered/corrupt payload)", NAME))
      end
      rcvd <= rcvd + 1;
    end
    if (flush || stall_all) ready <= 1'b0;
    else if (stall != 0) begin ready <= 1'b0; stall <= stall - 1; end
    else if ($urandom % 300 == 0) begin
      ready <= 1'b0; stall <= 100 + $urandom % 500; long_stalls <= long_stalls + 1;
    end else ready <= ($urandom % 100) < 60;
  end
endmodule

// --------------------------------------------------------- AXI checker ----
module tbcc_chk #(parameter int W = 8, parameter string NAME = "") (
  input logic clk, input logic en,
  input logic valid, input logic ready, input logic [W-1:0] data
);
  logic pend = 1'b0;
  logic [W-1:0] pdata;
  always @(posedge clk) begin
    if (!en) pend <= 1'b0;
    else begin
      if ($isunknown(valid) || $isunknown(ready))
        `TBCC_FAIL($sformatf("%s X on valid/ready (valid=%b ready=%b)", NAME, valid, ready))
      if (valid && $isunknown(data)) `TBCC_FAIL($sformatf("%s X payload while valid", NAME))
      if (pend && !valid) `TBCC_FAIL($sformatf("%s valid dropped before ready", NAME))
      if (pend && data !== pdata) `TBCC_FAIL($sformatf("%s payload changed while valid && !ready", NAME))
      pend  <= valid && !ready;
      pdata <= data;
    end
  end
endmodule

// ------------------------------------------------------------------- top ---
module tb_axi_cc #(
  parameter int ID_W   = 4,
  parameter int ADDR_W = 32,
  parameter int DATA_W = 64,
  parameter int DEPTH  = 4,
  parameter int S_PER  = 4000,   // ps
  parameter int M_PER  = 7519,   // ps
  parameter int M_PH   = 1237,   // m clock start offset, ps
  parameter int N_MAIN = 4000,
  parameter int N_POST = 500,
  parameter bit SYNC_RD = (DATA_W >= 256)   // DUT WIDE_SYNC_RD (default = DUT's)
);
  localparam int STRB_W = DATA_W / 8;
  localparam int AXP = ID_W + ADDR_W + 24;
  localparam int WP  = DATA_W + STRB_W + 1;
  localparam int BP  = ID_W + 2;
  localparam int RP  = ID_W + DATA_W + 3;

  logic s_aclk = 0, m_aclk = 0, s_aresetn = 0, m_aresetn = 0;
  initial forever begin #(S_PER / 2) s_aclk = 1; #(S_PER - S_PER / 2) s_aclk = 0; end
  initial begin #(M_PH); forever begin #(M_PER / 2) m_aclk = 1; #(M_PER - M_PER / 2) m_aclk = 0; end end

  logic flush = 1, stall_all = 0, chk_en = 0;
  int unsigned tgt = 0;

  // payload buses (same concatenation order on both sides)
  logic [AXP-1:0] s_aw, m_aw, s_ar, m_ar;
  logic [WP-1:0]  s_w, m_w;
  logic [BP-1:0]  s_b, m_b;
  logic [RP-1:0]  s_r, m_r;
  logic s_awv, s_awr, s_wv, s_wr, s_bv, s_br, s_arv, s_arr, s_rv, s_rr;
  logic m_awv, m_awr, m_wv, m_wr, m_bv, m_br, m_arv, m_arr, m_rv, m_rr;

  logic [ID_W-1:0] m_awid, m_arid, s_bid, s_rid;
  logic [ADDR_W-1:0] m_awaddr, m_araddr;
  logic [7:0] m_awlen, m_arlen; logic [2:0] m_awsize, m_arsize, m_awprot, m_arprot;
  logic [1:0] m_awburst, m_arburst, s_bresp, s_rresp;
  logic [3:0] m_awcache, m_arcache, m_awqos, m_arqos;
  logic [DATA_W-1:0] m_wdata, s_rdata; logic [STRB_W-1:0] m_wstrb; logic m_wlast, s_rlast;
  assign m_aw = {m_awqos, m_awprot, m_awcache, m_awburst, m_awsize, m_awlen, m_awaddr, m_awid};
  assign m_ar = {m_arqos, m_arprot, m_arcache, m_arburst, m_arsize, m_arlen, m_araddr, m_arid};
  assign m_w  = {m_wlast, m_wstrb, m_wdata};
  assign s_b  = {s_bresp, s_bid};
  assign s_r  = {s_rlast, s_rresp, s_rdata, s_rid};

  ng_axi_cc #(.ID_W(ID_W), .ADDR_W(ADDR_W), .DATA_W(DATA_W), .DEPTH(DEPTH), .WIDE_SYNC_RD(SYNC_RD)) dut (
    .s_aclk(s_aclk), .s_aresetn(s_aresetn),
    .s_axi_awid(s_aw[0 +: ID_W]), .s_axi_awaddr(s_aw[ID_W +: ADDR_W]),
    .s_axi_awlen(s_aw[ID_W+ADDR_W +: 8]), .s_axi_awsize(s_aw[ID_W+ADDR_W+8 +: 3]),
    .s_axi_awburst(s_aw[ID_W+ADDR_W+11 +: 2]), .s_axi_awcache(s_aw[ID_W+ADDR_W+13 +: 4]),
    .s_axi_awprot(s_aw[ID_W+ADDR_W+17 +: 3]), .s_axi_awqos(s_aw[ID_W+ADDR_W+20 +: 4]),
    .s_axi_awvalid(s_awv), .s_axi_awready(s_awr),
    .s_axi_wdata(s_w[0 +: DATA_W]), .s_axi_wstrb(s_w[DATA_W +: STRB_W]), .s_axi_wlast(s_w[WP-1]),
    .s_axi_wvalid(s_wv), .s_axi_wready(s_wr),
    .s_axi_bid(s_bid), .s_axi_bresp(s_bresp), .s_axi_bvalid(s_bv), .s_axi_bready(s_br),
    .s_axi_arid(s_ar[0 +: ID_W]), .s_axi_araddr(s_ar[ID_W +: ADDR_W]),
    .s_axi_arlen(s_ar[ID_W+ADDR_W +: 8]), .s_axi_arsize(s_ar[ID_W+ADDR_W+8 +: 3]),
    .s_axi_arburst(s_ar[ID_W+ADDR_W+11 +: 2]), .s_axi_arcache(s_ar[ID_W+ADDR_W+13 +: 4]),
    .s_axi_arprot(s_ar[ID_W+ADDR_W+17 +: 3]), .s_axi_arqos(s_ar[ID_W+ADDR_W+20 +: 4]),
    .s_axi_arvalid(s_arv), .s_axi_arready(s_arr),
    .s_axi_rid(s_rid), .s_axi_rdata(s_rdata), .s_axi_rresp(s_rresp), .s_axi_rlast(s_rlast),
    .s_axi_rvalid(s_rv), .s_axi_rready(s_rr),
    .m_aclk(m_aclk), .m_aresetn(m_aresetn),
    .m_axi_awid(m_awid), .m_axi_awaddr(m_awaddr), .m_axi_awlen(m_awlen), .m_axi_awsize(m_awsize),
    .m_axi_awburst(m_awburst), .m_axi_awcache(m_awcache), .m_axi_awprot(m_awprot),
    .m_axi_awqos(m_awqos), .m_axi_awvalid(m_awv), .m_axi_awready(m_awr),
    .m_axi_wdata(m_wdata), .m_axi_wstrb(m_wstrb), .m_axi_wlast(m_wlast),
    .m_axi_wvalid(m_wv), .m_axi_wready(m_wr),
    .m_axi_bid(m_b[0 +: ID_W]), .m_axi_bresp(m_b[ID_W +: 2]), .m_axi_bvalid(m_bv), .m_axi_bready(m_br),
    .m_axi_arid(m_arid), .m_axi_araddr(m_araddr), .m_axi_arlen(m_arlen), .m_axi_arsize(m_arsize),
    .m_axi_arburst(m_arburst), .m_axi_arcache(m_arcache), .m_axi_arprot(m_arprot),
    .m_axi_arqos(m_arqos), .m_axi_arvalid(m_arv), .m_axi_arready(m_arr),
    .m_axi_rid(m_r[0 +: ID_W]), .m_axi_rdata(m_r[ID_W +: DATA_W]), .m_axi_rresp(m_r[ID_W+DATA_W +: 2]),
    .m_axi_rlast(m_r[RP-1]), .m_axi_rvalid(m_rv), .m_axi_rready(m_rr)
  );

  // ---- BFMs ------------------------------------------------------------------
  tbcc_src #(.W(AXP), .SALT(1), .GAP_PCT(25)) u_aw_src (s_aclk, flush, tgt, s_awv, s_awr, s_aw);
  tbcc_src #(.W(WP),  .SALT(2), .GAP_PCT(15)) u_w_src  (s_aclk, flush, tgt, s_wv,  s_wr,  s_w);
  tbcc_src #(.W(AXP), .SALT(3), .GAP_PCT(35)) u_ar_src (s_aclk, flush, tgt, s_arv, s_arr, s_ar);
  tbcc_src #(.W(BP),  .SALT(4), .GAP_PCT(30)) u_b_src  (m_aclk, flush, tgt, m_bv,  m_br,  m_b);
  tbcc_src #(.W(RP),  .SALT(5), .GAP_PCT(10)) u_r_src  (m_aclk, flush, tgt, m_rv,  m_rr,  m_r);
  tbcc_snk #(.W(AXP), .SALT(1), .NAME("AW")) u_aw_snk (m_aclk, flush, stall_all, m_awv, m_awr, m_aw);
  tbcc_snk #(.W(WP),  .SALT(2), .NAME("W"))  u_w_snk  (m_aclk, flush, stall_all, m_wv,  m_wr,  m_w);
  tbcc_snk #(.W(AXP), .SALT(3), .NAME("AR")) u_ar_snk (m_aclk, flush, stall_all, m_arv, m_arr, m_ar);
  tbcc_snk #(.W(BP),  .SALT(4), .NAME("B"))  u_b_snk  (s_aclk, flush, stall_all, s_bv,  s_br,  s_b);
  tbcc_snk #(.W(RP),  .SALT(5), .NAME("R"))  u_r_snk  (s_aclk, flush, stall_all, s_rv,  s_rr,  s_r);

  // ---- protocol checkers on all ten interfaces --------------------------------
  tbcc_chk #(.W(AXP), .NAME("s_AW")) c0 (s_aclk, chk_en, s_awv, s_awr, s_aw);
  tbcc_chk #(.W(WP),  .NAME("s_W"))  c1 (s_aclk, chk_en, s_wv,  s_wr,  s_w);
  tbcc_chk #(.W(AXP), .NAME("s_AR")) c2 (s_aclk, chk_en, s_arv, s_arr, s_ar);
  tbcc_chk #(.W(BP),  .NAME("s_B"))  c3 (s_aclk, chk_en, s_bv,  s_br,  s_b);
  tbcc_chk #(.W(RP),  .NAME("s_R"))  c4 (s_aclk, chk_en, s_rv,  s_rr,  s_r);
  tbcc_chk #(.W(AXP), .NAME("m_AW")) c5 (m_aclk, chk_en, m_awv, m_awr, m_aw);
  tbcc_chk #(.W(WP),  .NAME("m_W"))  c6 (m_aclk, chk_en, m_wv,  m_wr,  m_w);
  tbcc_chk #(.W(AXP), .NAME("m_AR")) c7 (m_aclk, chk_en, m_arv, m_arr, m_ar);
  tbcc_chk #(.W(BP),  .NAME("m_B"))  c8 (m_aclk, chk_en, m_bv,  m_br,  m_b);
  tbcc_chk #(.W(RP),  .NAME("m_R"))  c9 (m_aclk, chk_en, m_rv,  m_rr,  m_r);

  // ---- helpers -----------------------------------------------------------------
  function automatic int unsigned max_sent();
    int unsigned m = u_aw_src.sent;
    if (u_w_src.sent  > m) m = u_w_src.sent;
    if (u_ar_src.sent > m) m = u_ar_src.sent;
    if (u_b_src.sent  > m) m = u_b_src.sent;
    if (u_r_src.sent  > m) m = u_r_src.sent;
    return m;
  endfunction

  function automatic bit all_done();
    return u_aw_src.sent == tgt && u_w_src.sent == tgt && u_ar_src.sent == tgt &&
           u_b_src.sent == tgt && u_r_src.sent == tgt &&
           u_aw_snk.rcvd == tgt && u_w_snk.rcvd == tgt && u_ar_snk.rcvd == tgt &&
           u_b_snk.rcvd == tgt && u_r_snk.rcvd == tgt;
  endfunction

  task automatic idle_both(input int n);
    repeat (n) @(posedge s_aclk);
    repeat (n) @(posedge m_aclk);
  endtask

  task automatic run_n(input string ph, input int unsigned n);
    int unsigned to = 0;
    tgt = max_sent() + n;
    while (!all_done()) begin
      @(posedge s_aclk);
      if (++to > 400 * n + 200000)
        `TBCC_FAIL($sformatf("%s: deadlock/timeout (sent %0d/%0d/%0d/%0d/%0d rcvd %0d/%0d/%0d/%0d/%0d tgt %0d)",
          ph, u_aw_src.sent, u_w_src.sent, u_ar_src.sent, u_b_src.sent, u_r_src.sent,
          u_aw_snk.rcvd, u_w_snk.rcvd, u_ar_snk.rcvd, u_b_snk.rcvd, u_r_snk.rcvd, tgt))
    end
    idle_both(4);
    $display("  phase %s: %0d payloads per channel OK (total per channel %0d) @%0t ps", ph, n, tgt, $time);
  endtask

  task automatic expect_valids_low(input string ph);
    if (s_bv !== 1'b0 || s_rv !== 1'b0 || m_awv !== 1'b0 || m_wv !== 1'b0 || m_arv !== 1'b0)
      `TBCC_FAIL($sformatf("%s: DUT valid not low (s_b %b s_r %b m_aw %b m_w %b m_ar %b)",
                           ph, s_bv, s_rv, m_awv, m_wv, m_arv))
  endtask

  task automatic expect_readies(input string ph, input logic v);
    if (s_awr !== v || s_wr !== v || s_arr !== v || m_br !== v || m_rr !== v)
      `TBCC_FAIL($sformatf("%s: DUT readies expected %b (s_aw %b s_w %b s_ar %b m_b %b m_r %b)",
                           ph, v, s_awr, s_wr, s_arr, m_br, m_rr))
  endtask

  // assert one side's reset at a random point, hold, check, release
  task automatic pulse_reset(input string ph, input bit side_s);
    int unsigned hold = 8 + $urandom % 20;
    chk_en = 0;
    #($urandom % (side_s ? S_PER : M_PER));
    if (side_s) s_aresetn = 0; else m_aresetn = 0;
    for (int i = 0; i < hold; i++) begin
      @(posedge s_aclk); @(posedge m_aclk);
      if (i >= 3) expect_valids_low({ph, " (in reset)"});
    end
    #($urandom % (side_s ? S_PER : M_PER));
    if (side_s) s_aresetn = 1; else m_aresetn = 1;
    idle_both(12);
    expect_valids_low({ph, " (after reset)"});
    expect_readies({ph, " (after reset)"}, 1'b1);
  endtask

  task automatic drained_reset(input string ph, input bit side_s);
    flush = 1;                               // all idle already; keep BFMs off
    pulse_reset(ph, side_s);
    flush = 0; chk_en = 1;
    run_n({ph, " post"}, N_POST);
  endtask

  task automatic flush_reset(input string ph, input bit side_s);
    stall_all = 1;
    tgt = max_sent() + 1000000;
    // sources keep pushing (random gaps / idle bursts) until every FIFO is
    // full: all five DUT input readies low at the same time
    for (int i = 0; !(s_awr === 1'b0 && s_wr === 1'b0 && s_arr === 1'b0 &&
                      m_br === 1'b0 && m_rr === 1'b0); i++) begin
      @(posedge s_aclk);
      if (i > 20000) expect_readies({ph, " (filled)"}, 1'b0);
    end
    idle_both(4);
    expect_readies({ph, " (filled, stable)"}, 1'b0);  // stays full while stalled
    chk_en = 0; flush = 1;                    // sources drop valid, sinks ready low
    idle_both(4);
    pulse_reset(ph, side_s);
    // in-flight payloads were flushed; resync scoreboards
    tgt = max_sent();
    u_aw_snk.rcvd = u_aw_src.sent; u_w_snk.rcvd = u_w_src.sent; u_ar_snk.rcvd = u_ar_src.sent;
    u_b_snk.rcvd  = u_b_src.sent;  u_r_snk.rcvd = u_r_src.sent;
    stall_all = 0; flush = 0; chk_en = 1;
    run_n({ph, " post"}, N_POST);
  endtask

  initial begin
    $display("tb_axi_cc: ID_W=%0d ADDR_W=%0d DATA_W=%0d DEPTH=%0d s_aclk=%0dps m_aclk=%0dps",
             ID_W, ADDR_W, DATA_W, DEPTH, S_PER, M_PER);
    idle_both(10);
    s_aresetn = 1; m_aresetn = 1;
    idle_both(10);
    expect_valids_low("init");
    flush = 0; chk_en = 1;

    run_n("A main soak", N_MAIN);
    if (u_aw_src.bp_cycles == 0 || u_w_src.bp_cycles == 0 || u_ar_src.bp_cycles == 0 ||
        u_b_src.bp_cycles == 0 || u_r_src.bp_cycles == 0)
      `TBCC_FAIL($sformatf("coverage: a FIFO never filled (bp cycles aw %0d w %0d ar %0d b %0d r %0d)",
        u_aw_src.bp_cycles, u_w_src.bp_cycles, u_ar_src.bp_cycles, u_b_src.bp_cycles, u_r_src.bp_cycles))
    $display("  backpressure cycles (FIFO full): aw %0d w %0d ar %0d b %0d r %0d; long stalls aw %0d b %0d",
      u_aw_src.bp_cycles, u_w_src.bp_cycles, u_ar_src.bp_cycles, u_b_src.bp_cycles, u_r_src.bp_cycles,
      u_aw_snk.long_stalls, u_b_snk.long_stalls);

    drained_reset("B1 s-reset (drained)", 1);
    drained_reset("B2 m-reset (drained)", 0);
    flush_reset("C1 s-reset (in-flight flush)", 1);
    flush_reset("C2 m-reset (in-flight flush)", 0);

    $display("NG_AXI_CC_TB_PASS DATA_W=%0d DEPTH=%0d s=%0dps m=%0dps payloads/channel=%0d",
             DATA_W, DEPTH, S_PER, M_PER, tgt);
    $finish;
  end
endmodule
