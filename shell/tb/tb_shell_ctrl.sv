// tb_shell_ctrl.sv -- testbench for the static shell control block.
//
// Regression cases (each maps to a 2026-09-26 board bug):
//   * T0  reset state: DECOUPLED+IN_RESET, SHELL_ID/VERSION/STATUS sanity
//   * T1  quiesce timeout -> SHELL_IRQ_STATUS[2]; exactly one MSI per arm;
//         W1C clears the status (was dead code: a second always_ff assignment
//         overrode the clear); EOI after W1C does NOT re-fire (storm check)
//   * T2  release interlock with rm_ready driven high *while decoupled*
//         (rm_ready must pass through the boundary, not be clamped)
//   * T0b PR_OPT resets to 1 (ORDER 1, SHELL_REV 4); T3a a stream under that order does not sync the
//         order-0 model (busy, no DONE); the rest of T3 runs with PR_OPT = 0
//   * T3  PR smoke: START, two words into the PR_DATA page in file byte order,
//         FINISH -> PR_STATUS.DONE with no error, PR_DONE irq bit set
//   * T4   CLK_FREQ_* readback (CORE = CORE2X/2), SHELL_STATUS.FENCE_STUCK,
//         MMCM DRP write/read through CLK_DRP_ADDR/DATA against a model with a
//         slow DRDY: single DEN pulse, ready bit, busy ignores writes, both
//         strobes set = no transaction
//   * T5  (PR diagnostics) PR_OPT readback, PR_STATUS raw/EOS bits, ICAPE3.O
//         capture registers through a full sync..DESYNC stream (model order 0):
//         default order done + O log 9B/DB/9B; ORDER=2 mismatch -> no DONE,
//         O log only 9B; CSIB_CONT and STORE+CSIB_FORCE variants complete
//   * T6   SHELL_CAPS.ICAP_RB, PR_OPT[8:6] readback, ICAP readback
//         through PR_RB_* (idle and inside a PR session; matching / mismatching
//         ORDER; REJECTED while busy and while the FIFO holds words; SEQ),
//         DONE_SEL 1/2/3 (PRDONE edge, EOS edge, drain only; each also in the
//         case that must NOT complete), PR_STATUS AVAIL/AVAIL_LOST/PRDONE_FELL/
//         EOS_FELL, PR_ICAP_WORDS, AVAIL_GATE holding the FIFO while AVAIL is
//         low, pr_dbg black box (marker, word counts, heartbeat)
// Pass token: NG_CTRL_TB_PASS
`timescale 1ns / 1ps

module tb_shell_ctrl;
  // SH_* offsets from ng_dfx_pkg (nicogpu_dfx_ports.yaml)
  localparam logic [15:0] A_ID      = 16'h0000, A_VER = 16'h0004, A_STATUS = 16'h0020;
  localparam logic [15:0] A_ISTAT   = 16'h0040, A_IEN = 16'h0044, A_EOI    = 16'h0048;
  localparam logic [15:0] A_RMCTRL  = 16'h0100, A_RMSTAT = 16'h0104;
  localparam logic [15:0] A_QTO     = 16'h0108, A_INFO0  = 16'h0110;
  localparam logic [15:0] A_PRCTRL  = 16'h0200, A_PRSTAT = 16'h0204, A_PRWORDS = 16'h0208;
  localparam logic [15:0] A_PREXP   = 16'h020C, A_PRFREE = 16'h0210;
  localparam logic [15:0] A_PROPT   = 16'h0214, A_PROUT = 16'h0218, A_PROST = 16'h021C;
  localparam logic [15:0] A_PROH0   = 16'h0220, A_PROH1 = 16'h0224, A_PROH2 = 16'h0228, A_PROH3 = 16'h022C;
  localparam logic [15:0] A_PRDATA  = 16'h1000;
  localparam logic [15:0] A_CAPS    = 16'h0008, A_PRWICAP = 16'h0230, A_RBCTRL = 16'h0234, A_RBSTAT = 16'h0238;
  localparam logic [15:0] A_RBD0    = 16'h0240;
  localparam logic [15:0] A_DRPA = 16'h2008, A_DRPD = 16'h200C;
  localparam logic [15:0] A_FCORE = 16'h2010, A_FL2 = 16'h2014, A_FTS = 16'h2018, A_FC2X = 16'h201C;

  logic clk_sh = 0, rst_sh_n = 0;
  logic clk_icap = 0, rst_icap_n = 0;
  always #4 clk_sh   = ~clk_sh;    // 125 MHz
  always #5 clk_icap = ~clk_icap;  // 100 MHz

  logic [15:0] s_awaddr, s_araddr;
  logic        s_awvalid, s_awready, s_wvalid, s_wready, s_bvalid, s_bready;
  logic [31:0] s_wdata, s_rdata;
  logic [3:0]  s_wstrb;
  logic [1:0]  s_bresp, s_rresp;
  logic        s_arvalid, s_arready, s_rvalid, s_rready;

  logic si_done, si_err, mmcm_locked, link_up;
  logic [3:0]  mem_calib;
  logic        quiesce_req, decouple, rm_reset, info_sample, rm_clk_en, mmcm_reset;
  logic        l2_rm_quiesce_ack, l2_m_drained, l2_iso_idle, l2_rm_ready, l2_rm_irq0;
  logic        l2_err_pulse_toggle;
  logic [31:0] l2_outstanding;
  logic [127:0] l2_rm_info;
  logic [1:0]  msi_req, msi_ack = 2'b11;   // ack same cycle -> 1-cycle pulses
  logic        fence_stuck = 1'b0;
  logic        eos = 1'b0;
  logic [31:0] freq_core2x = '0, freq_l2 = '0, freq_ts = '0;
  logic        drp_en, drp_we, drp_rdy = 1'b0;
  logic [6:0]  drp_a;
  logic [15:0] drp_di, drp_do = '0;
  logic [159:0] pr_dbg;

  // MMCM DRP model: 128 x 16, DRDY 6 cycles after DEN (real MMCM is slower than 1)
  logic [15:0] drp_mem [128];
  int          drp_en_cnt = 0, drp_wr_cnt = 0, drp_rd_cnt = 0, drp_pipe = 0;
  logic        drp_pend_we;
  logic [6:0]  drp_pend_a;
  logic [15:0] drp_pend_d;
  always @(posedge clk_sh) begin
    drp_rdy <= 1'b0;
    if (drp_en) begin
      drp_en_cnt++;
      drp_pipe <= 6; drp_pend_we <= drp_we; drp_pend_a <= drp_a; drp_pend_d <= drp_di;
    end else if (drp_pipe != 0) begin
      drp_pipe <= drp_pipe - 1;
      if (drp_pipe == 1) begin
        drp_rdy <= 1'b1;
        if (drp_pend_we) begin drp_mem[drp_pend_a] <= drp_pend_d; drp_wr_cnt++; end
        else             begin drp_do <= drp_mem[drp_pend_a];     drp_rd_cnt++; end
      end
    end
  end

  ng_shell_ctrl #(
    .SHELL_UID(64'h0000_2026_98FE_668C), .SHELL_REV(16'h0001),
    .US_DIV(125), .PR_DEPTH(512)
  ) dut (
    .clk_sh(clk_sh), .rst_sh_n(rst_sh_n),
    .s_awaddr(s_awaddr), .s_awvalid(s_awvalid), .s_awready(s_awready),
    .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wvalid(s_wvalid), .s_wready(s_wready),
    .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
    .s_araddr(s_araddr), .s_arvalid(s_arvalid), .s_arready(s_arready),
    .s_rdata(s_rdata), .s_rresp(s_rresp), .s_rvalid(s_rvalid), .s_rready(s_rready),
    .si_done(si_done), .si_err(si_err), .mmcm_locked(mmcm_locked),
    .mem_calib(mem_calib), .link_up(link_up),
    .fence_stuck(fence_stuck), .eos(eos), .freq_core2x(freq_core2x), .freq_l2(freq_l2), .freq_ts(freq_ts),
    .drp_en(drp_en), .drp_we(drp_we), .drp_a(drp_a), .drp_di(drp_di), .drp_do(drp_do), .drp_rdy(drp_rdy),
    .quiesce_req(quiesce_req), .decouple(decouple), .rm_reset(rm_reset),
    .info_sample(info_sample), .rm_clk_en(rm_clk_en), .mmcm_reset(mmcm_reset),
    .l2_rm_quiesce_ack(l2_rm_quiesce_ack), .l2_m_drained(l2_m_drained),
    .l2_iso_idle(l2_iso_idle), .l2_rm_ready(l2_rm_ready),
    .l2_rm_irq0(l2_rm_irq0), .l2_err_pulse_toggle(l2_err_pulse_toggle),
    .l2_outstanding(l2_outstanding), .l2_rm_info(l2_rm_info),
    .msi_req(msi_req), .msi_ack(msi_ack),
    .clk_icap(clk_icap), .rst_icap_n(rst_icap_n), .pr_dbg(pr_dbg)
  );

  assign s_bready = 1'b1;
  assign s_rready = 1'b1;

  int errors = 0;
  task automatic fail(input string w);
    $display("TB_FAIL %s", w); errors++;
  endtask

  // ---- AXI-Lite BFM (aw/w handshakes may complete in the same cycle)
  task automatic lite_wr(input logic [15:0] a, input logic [31:0] d);
    s_awaddr <= a; s_wdata <= d; s_wstrb <= 4'hF; s_awvalid <= 1'b1; s_wvalid <= 1'b1;
    while (s_awready !== 1'b1 || s_wready !== 1'b1) @(posedge clk_sh);
    @(posedge clk_sh);                       // the handshake cycle
    s_awvalid <= 1'b0; s_wvalid <= 1'b0;
    s_wdata <= ~d;                           // like ng_axi64_to_lite32 after the high-half handshake: the bus changes after the W handshake
    while (s_bvalid !== 1'b1) @(posedge clk_sh);
  endtask

  task automatic lite_rd(input logic [15:0] a, output logic [31:0] d);
    s_araddr <= a; s_arvalid <= 1'b1;
    while (s_arready !== 1'b1) @(posedge clk_sh);
    @(posedge clk_sh);
    s_arvalid <= 1'b0;
    while (s_rvalid !== 1'b1) @(posedge clk_sh);
    d = s_rdata;
  endtask

  task automatic rd_expect(input logic [15:0] a, input logic [31:0] mask,
                           input logic [31:0] want, input string w);
    logic [31:0] r;
    lite_rd(a, r);
    if ((r & mask) !== want) begin
      fail($sformatf("%s: read %08x want(mask) %08x", w, r, want));
    end
  endtask

  // ---- MSI pulse counters
  int msi0_cnt = 0, msi1_cnt = 0;
  always @(posedge clk_sh) begin
    if (msi_req[0]) msi0_cnt++;
    if (msi_req[1]) msi1_cnt++;
  end

  function automatic logic [31:0] bitrev(input logic [31:0] w);
    logic [31:0] r;
    for (int i = 0; i < 32; i++) r[i] = w[31 - i];
    return r;
  endfunction

  // helpers
  task automatic rb_wait(input string w);
    logic [31:0] r;
    for (int i = 0; i < 400; i++) begin
      lite_rd(A_RBSTAT, r);
      if (!r[0] && r[1]) return;
    end
    fail({w, ": readback did not complete"});
  endtask

  // PR_OPT = opt, START, push the BE words in file byte order (eos_pulse: EOS
  // low for a while in the middle), FINISH
  task automatic pr_stream(input logic [31:0] opt, input int n, input logic [31:0] be [9], input bit eos_pulse = 0);
    lite_wr(A_PROPT, opt);
    lite_wr(A_PREXP, n);
    lite_wr(A_PRCTRL, 32'h1);
    for (int k = 0; k < n; k++) begin
      lite_wr(A_PRDATA, {be[k][7:0], be[k][15:8], be[k][23:16], be[k][31:24]});
      if (eos_pulse && k == 4) begin eos = 1'b0; repeat (10) @(posedge clk_sh); eos = 1'b1; end
    end
    repeat (100) @(posedge clk_sh);
    lite_wr(A_PRCTRL, 32'h2);
  endtask

  // want_done = 1: PR_STATUS must reach DONE (no error); 0: must stay BUSY
  task automatic pr_finish_wait(input bit want_done, input string w);
    logic [31:0] r;
    for (int i = 0; i < 300; i++) begin
      lite_rd(A_PRSTAT, r);
      if (r[2] || !r[0]) break;
    end
    if (want_done && (r[2:0] !== 3'b010)) fail($sformatf("%s: PR_STATUS %08x, want DONE", w, r));
    if (!want_done && (r[2:0] !== 3'b001)) fail($sformatf("%s: PR_STATUS %08x, want still BUSY", w, r));
  endtask

  // watchdog
  initial begin
    #2_000_000;
    fail("watchdog expired");
    $finish;
  end

  // valid rm_info: magic "NGRM", iface 0x0001, kind blank, mem_mask 0
  localparam logic [127:0] RM_INFO_GOOD =
    {32'h0000_0001, 32'hDEAD_BEEF, 32'h0000_0001, 32'h4D52_474E};

  initial begin
    si_done = 1'b1; si_err = 1'b0; mmcm_locked = 1'b1; link_up = 1'b1;
    mem_calib = 4'hF;
    l2_rm_quiesce_ack = 1'b0; l2_m_drained = 1'b0; l2_iso_idle = 1'b0;
    l2_rm_ready = 1'b0; l2_rm_irq0 = 1'b0; l2_err_pulse_toggle = 1'b0;
    l2_outstanding = '0; l2_rm_info = '0;
    repeat (10) @(posedge clk_sh);
    rst_sh_n = 1'b1; rst_icap_n = 1'b1;
    repeat (10) @(posedge clk_sh);

    // ---------------- T0: reset state and identity
    rd_expect(A_ID,   32'hFFFF_FFFF, 32'h4C48_5358, "T0 SHELL_ID");
    rd_expect(A_VER,  32'hFFFF_FFFF, 32'h0001_0001, "T0 SHELL_VERSION");
    rd_expect(A_STATUS, 32'hFFFF_FFFF, 32'h0000_01F5, "T0 SHELL_STATUS");
    rd_expect(A_RMSTAT, 32'h0000_000C, 32'h0000_000C, "T0 reset DECOUPLED+IN_RESET");
    rd_expect(A_ISTAT, 32'hFFFF_FFFF, 32'h0, "T0 irq_status clear at reset");
    rd_expect(A_PROPT, 32'hFFFF_FFFF, 32'h1, "T0 PR_OPT resets to 1 (ORDER 1, other knobs 0)");

    // ---------------- T1: quiesce timeout irq, W1C, EOI re-arm, no storm
    lite_wr(16'h000C, 32'hDEAD_BEEF);                    // scratch side path
    rd_expect(16'h000C, 32'hFFFF_FFFF, 32'hDEAD_BEEF, "T1 scratch");
    lite_wr(A_QTO, 32'd1);                               // 1 us timeout
    lite_wr(A_RMCTRL, 32'h7);                            // quiesce+decouple+reset
    repeat (500) @(posedge clk_sh);                      // > 1 us
    rd_expect(A_ISTAT, 32'h4, 32'h4, "T1 QUIESCE_TIMEOUT set");
    if (msi1_cnt != 0) fail("T1 MSI fired before enable");
    lite_wr(A_IEN, 32'h4);                               // enable bit2 -> vector 1
    repeat (20) @(posedge clk_sh);
    if (msi1_cnt != 1) fail($sformatf("T1 want exactly 1 MSI per arm, got %0d", msi1_cnt));
    repeat (100) @(posedge clk_sh);
    if (msi1_cnt != 1) fail("T1 MSI repeated without EOI");
    lite_wr(A_ISTAT, 32'hFFFF_FFFF);                     // W1C all
    rd_expect(A_ISTAT, 32'hFFFF_FFFF, 32'h0, "T1 W1C clears status");   // board bug
    lite_wr(A_EOI, 32'h2);                               // re-arm vector 1
    repeat (1000) @(posedge clk_sh);
    if (msi1_cnt != 1) fail($sformatf("T1 MSI storm after EOI: count %0d", msi1_cnt));

    // ---------------- T2: release with rm_ready high while decoupled
    l2_rm_ready = 1'b1;                                  // unclamped pass-through
    l2_rm_quiesce_ack = 1'b1; l2_m_drained = 1'b1; l2_iso_idle = 1'b1;
    l2_rm_info = RM_INFO_GOOD;
    repeat (5) @(posedge clk_sh);
    lite_wr(A_RMCTRL, 32'h0);                            // release everything
    repeat (10) @(posedge clk_sh);
    rd_expect(A_RMSTAT, 32'h0000_01FC, 32'h0000_0170,    // READY|INFO|IFACE|MEM_OK set, !DECOUPLED, !IN_RESET (ACK/DRAINED unmasked)
              "T2 released");
    if (decouple !== 1'b0) fail("T2 decouple output still high");
    if (rm_reset  !== 1'b0) fail("T2 rm_reset output still high");
    rd_expect(A_INFO0, 32'hFFFF_FFFF, 32'h4D52_474E, "T2 RM_INFO0 sampled");
    lite_wr(A_RMCTRL, 32'h2);                            // re-decouple, keep RM out of reset
    repeat (5) @(posedge clk_sh);
    rd_expect(A_RMSTAT, 32'h0000_008C, 32'h0000_0004, "T2 re-decoupled, not in reset, not forced");
    lite_wr(A_RMCTRL, 32'h6);                            // decouple + reset for PR
    repeat (5) @(posedge clk_sh);
    rd_expect(A_RMSTAT, 32'h0000_000C, 32'h0000_000C, "T2 decoupled+in reset");

    // ---------------- T3: PR smoke through the behavioral ICAP
    rd_expect(A_PRFREE, 32'hFFFF_FFFF, 32'd512, "T3 FIFO free at start");
    // T3a: a stream pushed under the PR_OPT reset value (ORDER 1) does not sync the model, whose "silicon"
    // expects order 0: busy, no DONE, no error (end-to-end check of the reset order, SHELL_REV 4)
    lite_wr(A_PREXP, 32'd2);
    lite_wr(A_PRCTRL, 32'h1);
    lite_wr(A_PRDATA, 32'h0180_0030);
    lite_wr(A_PRDATA, 32'h0D00_0000);
    repeat (200) @(posedge clk_sh);
    lite_wr(A_PRCTRL, 32'h2);
    repeat (300) @(posedge clk_sh);
    rd_expect(A_PRSTAT, 32'h7, 32'h1, "T3a reset ORDER 1 vs model order 0: still busy, no DONE");
    lite_wr(A_PRCTRL, 32'h4);                            // ABORT
    repeat (20) @(posedge clk_sh);
    lite_wr(A_PROPT, 32'h0);                             // the model expects order 0
    lite_wr(A_PREXP, 32'd2);
    lite_wr(A_PRCTRL, 32'h1);                            // START
    rd_expect(A_PRSTAT, 32'h1, 32'h1, "T3 PR busy");
    // two words in file byte order: BE 30008001 (type-1 write CMD) then
    // BE 0000000D (DESYNC); the ctrl byte-swaps each dword on the way in
    lite_wr(A_PRDATA, 32'h0180_0030);
    lite_wr(A_PRDATA, 32'h0D00_0000);
    rd_expect(A_PRWORDS, 32'hFFFF_FFFF, 32'd2, "T3 words counted");
    repeat (200) @(posedge clk_sh);                      // FIFO -> ICAP drain
    lite_wr(A_PRCTRL, 32'h2);                            // FINISH
    begin : pr_poll
      int i; logic [31:0] r;
      for (i = 0; i < 2000; i++) begin
        lite_rd(A_PRSTAT, r);
        if (r[2] || (r[1] && !r[0])) begin
          rd_expect(A_PRSTAT, 32'h7, 32'h2, "T3 PR done, no error");
          disable pr_poll;
        end
        @(posedge clk_sh);
      end
      fail("T3 PR never completed");
    end
    rd_expect(A_ISTAT, 32'h3, 32'h1, "T3 PR_DONE irq bit set (no error)");

    // ---------------- T4: clock meters, FENCE_STUCK, MMCM DRP
    begin : t4
      logic [31:0] r;
      int k;
      freq_core2x = 32'd399767; freq_l2 = 32'd249854; freq_ts = 32'd99942;
      rd_expect(A_FCORE, 32'hFFFF_FFFF, 32'd199883, "T4 CLK_FREQ_CORE = core2x/2");
      rd_expect(A_FC2X,  32'hFFFF_FFFF, 32'd399767, "T4 CLK_FREQ_CORE2X");
      rd_expect(A_FL2,   32'hFFFF_FFFF, 32'd249854, "T4 CLK_FREQ_L2");
      rd_expect(A_FTS,   32'hFFFF_FFFF, 32'd99942,  "T4 CLK_FREQ_TS");
      rd_expect(A_STATUS, 32'h0000_0200, 32'h0, "T4 FENCE_STUCK clear");
      fence_stuck = 1'b1;
      repeat (8) @(posedge clk_sh);
      rd_expect(A_STATUS, 32'h0000_0200, 32'h0000_0200, "T4 FENCE_STUCK set");
      fence_stuck = 1'b0;
      repeat (8) @(posedge clk_sh);
      rd_expect(A_STATUS, 32'h0000_0200, 32'h0, "T4 FENCE_STUCK clear again");

      for (k = 0; k < 128; k++) drp_mem[k] = 16'h0;
      drp_mem[7'h29] = 16'hBEEF;
      rd_expect(A_DRPD, 32'h8000_0000, 32'h8000_0000, "T4 DRP idle ready");
      // write 0x1234 -> reg 0x28
      lite_wr(A_DRPD, 32'h1234);
      lite_wr(A_DRPA, 32'h8000_0028);
      rd_expect(A_DRPD, 32'h8000_0000, 32'h0, "T4 DRP busy after write strobe");
      lite_wr(A_DRPD, 32'h5555);                        // ignored while busy
      lite_wr(A_DRPA, 32'h8000_0030);                   // ignored while busy
      for (k = 0; k < 40; k++) @(posedge clk_sh);
      rd_expect(A_DRPD, 32'hFFFF_FFFF, 32'h8000_1234, "T4 DRP ready, data unchanged");
      if (drp_mem[7'h28] !== 16'h1234) fail($sformatf("T4 DRP reg28=%04x", drp_mem[7'h28]));
      if (drp_mem[7'h30] !== 16'h0)    fail("T4 busy-time strobe leaked into the DRP");
      if (drp_en_cnt != 1 || drp_wr_cnt != 1) fail($sformatf("T4 DEN pulses %0d writes %0d", drp_en_cnt, drp_wr_cnt));
      rd_expect(A_DRPA, 32'hFFFF_FFFF, 32'h0000_0028, "T4 DRP_ADDR reads back address only");
      // read reg 0x29
      lite_wr(A_DRPA, 32'h4000_0029);
      for (k = 0; k < 40; k++) @(posedge clk_sh);
      rd_expect(A_DRPD, 32'hFFFF_FFFF, 32'h8000_BEEF, "T4 DRP read data");
      if (drp_en_cnt != 2 || drp_rd_cnt != 1) fail($sformatf("T4 DEN pulses %0d reads %0d", drp_en_cnt, drp_rd_cnt));
      // both strobes: no transaction
      lite_wr(A_DRPA, 32'hC000_0029);
      for (k = 0; k < 20; k++) @(posedge clk_sh);
      if (drp_en_cnt != 2) fail("T4 both strobes started a transaction");
      rd_expect(A_DRPD, 32'h8000_0000, 32'h8000_0000, "T4 ready after rejected strobe");
    end

    // ---------------- T5: PR diagnostics knobs and ICAP O capture
    begin : t5
      localparam int NW = 9;
      logic [31:0] be [NW];
      logic [31:0] r;
      int phase, k;
      be[0] = 32'hFFFF_FFFF; be[1] = 32'h0000_00BB; be[2] = 32'h1122_0044; be[3] = 32'hFFFF_FFFF;
      be[4] = 32'hAA99_5566; be[5] = 32'h2000_0000; be[6] = 32'h3000_8001; be[7] = 32'h0000_000D;
      be[8] = 32'h2000_0000;
      rd_expect(A_PROPT, 32'hFFFF_FFFF, 32'h0, "T5 PR_OPT = 0 (written by T3)");
      lite_wr(A_PROPT, 32'h3F);
      rd_expect(A_PROPT, 32'hFFFF_FFFF, 32'h3F, "T5 PR_OPT write/readback");
      lite_wr(A_PROPT, 32'h0);
      eos = 1'b1;
      repeat (8) @(posedge clk_sh);
      rd_expect(A_PRSTAT, 32'h1000, 32'h1000, "T5 PR_STATUS.EOS follows the pin");
      eos = 1'b0;
      // phase 0: default knobs; 1: ORDER=2 (mismatch with model order 0); 2: CSIB_CONT; 3: STORE+FORCE
      for (phase = 0; phase < 4; phase++) begin
        case (phase)
          0: lite_wr(A_PROPT, 32'h00);
          1: lite_wr(A_PROPT, 32'h02);
          2: lite_wr(A_PROPT, 32'h04);
          default: lite_wr(A_PROPT, 32'h30);
        endcase
        lite_wr(A_PREXP, NW);
        lite_wr(A_PRCTRL, 32'h1);                        // START
        for (k = 0; k < NW; k++)
          lite_wr(A_PRDATA, {be[k][7:0], be[k][15:8], be[k][23:16], be[k][31:24]});
        repeat (300) @(posedge clk_sh);                  // FIFO -> ICAP drain (STORE: waits for FINISH)
        lite_wr(A_PRCTRL, 32'h2);                        // FINISH
        repeat (300) @(posedge clk_sh);
        if (phase == 1) begin
          rd_expect(A_PRSTAT, 32'h7, 32'h1, "T5 ORDER mismatch: still busy, no DONE");
          lite_wr(A_PRCTRL, 32'h4);                      // ABORT
          repeat (20) @(posedge clk_sh);
        end else begin
          rd_expect(A_PRSTAT, 32'h7, 32'h2, "T5 PR done, no error");
          rd_expect(A_PRSTAT, 32'h0C00, 32'h0800, "T5 PRDONE_RAW (model: 1 idle), PRERROR_RAW 0");
        end
        // wait for a snapshot to propagate, then freeze and read a coherent set
        repeat (80) @(posedge clk_sh);
        lite_wr(A_PROPT, 32'h08 | ((phase == 1) ? 32'h02 : (phase == 2) ? 32'h04 : (phase == 3) ? 32'h30 : 32'h0));
        repeat (4) @(posedge clk_sh);
        rd_expect(A_PROUT, 32'hFFFF_FFFF, 32'h0000_009B, $sformatf("T5.%0d last non-zero O", phase));
        rd_expect(A_PROH0, 32'hFFFF_FFFF, 32'h0000_009B, $sformatf("T5.%0d O log h0", phase));
        if (phase == 1) begin
          rd_expect(A_PROST, 32'hFFFF_FFFF, 32'h0000_0101, "T5.1 O valid, 1 change (never synced)");
          rd_expect(A_PROH1, 32'hFFFF_FFFF, 32'h0, "T5.1 O log h1 empty");
        end else begin
          rd_expect(A_PROST, 32'hFFFF_FFFF, 32'h0000_0301, $sformatf("T5.%0d O valid, 3 changes", phase));
          rd_expect(A_PROH1, 32'hFFFF_FFFF, 32'h0000_00DB, $sformatf("T5.%0d O log h1 = synced value", phase));
          rd_expect(A_PROH2, 32'hFFFF_FFFF, 32'h0000_009B, $sformatf("T5.%0d O log h2", phase));
          rd_expect(A_PROH3, 32'hFFFF_FFFF, 32'h0, $sformatf("T5.%0d O log h3", phase));
        end
        lite_wr(A_PROPT, 32'h0);
      end
    end

    // ---------------- T6: readback, completion selectors, AVAIL
    begin : t6
      localparam int NW = 9;
      logic [31:0] be [NW];
      logic [31:0] r, r2;
      int k;
      be[0] = 32'hFFFF_FFFF; be[1] = 32'h0000_00BB; be[2] = 32'h1122_0044; be[3] = 32'hFFFF_FFFF;
      be[4] = 32'hAA99_5566; be[5] = 32'h2000_0000; be[6] = 32'h3000_8001; be[7] = 32'h0000_000D;
      be[8] = 32'h2000_0000;
      eos = 1'b1;                                         // configuration finished (idle level)
      repeat (8) @(posedge clk_sh);
      rd_expect(A_CAPS, 32'h0000_000F, 32'h0000_000B, "T6 SHELL_CAPS ICAP|CLK_DRP|ICAP_RB");
      lite_wr(A_PROPT, 32'h1FF);
      rd_expect(A_PROPT, 32'hFFFF_FFFF, 32'h1FF, "T6 PR_OPT[8:0] readback");
      lite_wr(A_PROPT, 32'h0);
      rd_expect(A_PRSTAT, 32'h0000_2000, 32'h0000_2000, "T6 PR_STATUS.ICAP_AVAIL (model 1)");
      rd_expect(A_RBSTAT, 32'hFFFF_FFFF, 32'h0, "T6 PR_RB_STATUS reset");

      // -- readback, PR idle, ORDER 0 (= the model's order): IDCODE in read cycle 3
      lite_wr(A_RBCTRL, 32'h8000_010C);                   // GO, COUNT 1, REG 0x0C
      rb_wait("T6 rb idle");
      rd_expect(A_RBSTAT, 32'hFFFF_FFFF, 32'h0000_0102, "T6 rb DONE, SEQ 1");
      rd_expect(A_RBD0 + 16'd12, 32'hFFFF_FFFF, bitrev(32'h04B3_5093), "T6 rb DATA3 = IDCODE (order 0)");
      rd_expect(A_RBD0,          32'hFFFF_FFFF, 32'h0000_00DB, "T6 rb DATA0 = synced status");
      rd_expect(A_RBD0 + 16'd28, 32'hFFFF_FFFF, 32'h0000_00DB, "T6 rb DATA7 = synced status");
      rd_expect(A_RBCTRL, 32'hFFFF_FFFF, 32'h0000_010C, "T6 PR_RB_CTRL readback, not busy");
      // SKIP 3: the data word moves to DATA0
      lite_wr(A_RBCTRL, 32'h8003_010C);
      rb_wait("T6 rb skip");
      rd_expect(A_RBD0, 32'hFFFF_FFFF, bitrev(32'h04B3_5093), "T6 rb SKIP 3 -> DATA0 = IDCODE");
      rd_expect(A_RBSTAT, 32'h0000_FF00, 32'h0000_0200, "T6 rb SEQ 2");
      // ORDER 2: the model never syncs, all words are the unsynced status
      lite_wr(A_PROPT, 32'h2);
      lite_wr(A_RBCTRL, 32'h8000_010C);
      rb_wait("T6 rb mismatch");
      for (k = 0; k < 8; k++)
        rd_expect(A_RBD0 + 16'(4 * k), 32'hFFFF_FFFF, 32'h0000_009B, $sformatf("T6 rb mismatch DATA%0d", k));
      lite_wr(A_PROPT, 32'h0);
      // GO while busy -> REJECTED, the running one still completes
      lite_wr(A_RBCTRL, 32'h8000_0107);
      lite_wr(A_RBCTRL, 32'h8000_010C);
      rd_expect(A_RBSTAT, 32'h0000_0004, 32'h0000_0004, "T6 rb GO while busy rejected");
      rb_wait("T6 rb after reject");
      rd_expect(A_RBD0 + 16'd12, 32'hFFFF_FFFF, bitrev(32'h1090_79FC), "T6 rb STAT (the first GO) completed");

      // -- DONE_SEL 1 (PRDONE fell and rose): matching order completes
      pr_stream(32'h40, NW, be);
      pr_finish_wait(1, "T6 DONE_SEL 1 match");
      rd_expect(A_PRSTAT, 32'h0001_E000, 32'h0000_A000, "T6 DONE_SEL 1: AVAIL, PRDONE_FELL, no AVAIL_LOST/EOS_FELL");
      repeat (80) @(posedge clk_sh);
      rd_expect(A_PRWICAP, 32'hFFFF_FFFF, NW, "T6 PR_ICAP_WORDS = 9");
      // -- DONE_SEL 1, ORDER 2: PRDONE never falls -> stays busy, ABORT
      pr_stream(32'h42, NW, be);
      pr_finish_wait(0, "T6 DONE_SEL 1 mismatch");
      rd_expect(A_PRSTAT, 32'h0000_8007, 32'h0000_0001, "T6 DONE_SEL 1 mismatch: busy, no PRDONE_FELL");
      lite_wr(A_PRCTRL, 32'h4);
      repeat (20) @(posedge clk_sh);
      // -- DONE_SEL 2 (EOS fell and rose): EOS pulsed low during the session
      pr_stream(32'h80, NW, be, 1);
      pr_finish_wait(1, "T6 DONE_SEL 2 EOS pulse");
      rd_expect(A_PRSTAT, 32'h0001_0000, 32'h0001_0000, "T6 EOS_FELL set");
      // -- DONE_SEL 2 without an EOS pulse: busy, ABORT
      pr_stream(32'h80, NW, be);
      pr_finish_wait(0, "T6 DONE_SEL 2 no pulse");
      rd_expect(A_PRSTAT, 32'h0001_0007, 32'h0000_0001, "T6 DONE_SEL 2 no pulse: busy, EOS_FELL clear");
      lite_wr(A_PRCTRL, 32'h4);
      repeat (20) @(posedge clk_sh);
      // -- DONE_SEL 3 (drain only) with the mismatching order: completes anyway
      pr_stream(32'hC2, NW, be);
      pr_finish_wait(1, "T6 DONE_SEL 3 drain only");

      // -- readback inside a PR session (FIFO empty), and refused while the FIFO holds words (STORE)
      lite_wr(A_PROPT, 32'h20);                          // STORE: the ICAP waits for FINISH
      lite_wr(A_PREXP, NW);
      lite_wr(A_PRCTRL, 32'h1);
      lite_wr(A_RBCTRL, 32'h8000_010C);
      rb_wait("T6 rb in session");
      rd_expect(A_RBD0 + 16'd12, 32'hFFFF_FFFF, bitrev(32'h04B3_5093), "T6 rb in session: IDCODE");
      for (k = 0; k < 3; k++) lite_wr(A_PRDATA, {be[k][7:0], be[k][15:8], be[k][23:16], be[k][31:24]});
      repeat (20) @(posedge clk_sh);
      lite_wr(A_RBCTRL, 32'h8000_010C);
      rd_expect(A_RBSTAT, 32'h0000_0005, 32'h0000_0004, "T6 rb GO refused while the FIFO holds words");
      for (k = 3; k < NW; k++) lite_wr(A_PRDATA, {be[k][7:0], be[k][15:8], be[k][23:16], be[k][31:24]});
      lite_wr(A_PRCTRL, 32'h2);
      pr_finish_wait(1, "T6 STORE session after readbacks");

      // -- AVAIL low: AVAIL_GATE holds the FIFO; without the gate AVAIL_LOST is flagged
      force dut.u_icap.model_avail = 1'b0;
      repeat (10) @(posedge clk_sh);
      rd_expect(A_PRSTAT, 32'h0000_2000, 32'h0, "T6 ICAP_AVAIL follows the primitive");
      pr_stream(32'h1C0, NW, be);                        // AVAIL_GATE + DONE_SEL 3
      repeat (100) @(posedge clk_sh);
      rd_expect(A_PRFREE, 32'hFFFF_FFFF, 32'd512 - NW, "T6 AVAIL_GATE: FIFO not popped");
      rd_expect(A_PRWICAP, 32'hFFFF_FFFF, 32'd0, "T6 AVAIL_GATE: no ICAP words");
      release dut.u_icap.model_avail; dut.u_icap.model_avail = 1'b1;   // a released variable keeps the forced value
      pr_finish_wait(1, "T6 AVAIL_GATE drains after AVAIL returns");
      rd_expect(A_PRSTAT, 32'h0000_4000, 32'h0, "T6 AVAIL_GATE: no AVAIL_LOST");
      force dut.u_icap.model_avail = 1'b0;
      pr_stream(32'hC0, NW, be);                         // DONE_SEL 3, no gate
      pr_finish_wait(1, "T6 no gate");
      rd_expect(A_PRSTAT, 32'h0000_4000, 32'h0000_4000, "T6 AVAIL_LOST without the gate");
      release dut.u_icap.model_avail; dut.u_icap.model_avail = 1'b1;   // a released variable keeps the forced value

      // -- black box
      repeat (80) @(posedge clk_sh);
      if (pr_dbg[159:152] !== 8'h5A) fail($sformatf("T6 pr_dbg marker %02x", pr_dbg[159:152]));
      if (pr_dbg[31:0] !== NW) fail($sformatf("T6 pr_dbg icap words %0d", pr_dbg[31:0]));
      if (pr_dbg[95:64] !== NW) fail($sformatf("T6 pr_dbg PR_WORDS %0d", pr_dbg[95:64]));
      if (pr_dbg[136] !== 1'b1) fail("T6 pr_dbg AVAIL_LOST");
      if (pr_dbg[151:144] !== 8'd5) fail($sformatf("T6 pr_dbg rb SEQ %0d", pr_dbg[151:144]));
      r = pr_dbg[111:96];
      repeat (3000) @(posedge clk_sh);
      if (pr_dbg[111:96] === r[15:0]) fail("T6 pr_dbg clk_sh heartbeat does not move");
      lite_wr(A_PROPT, 32'h0);
    end

    if (errors == 0) $display("NG_CTRL_TB_PASS");
    else             $display("NG_CTRL_TB_FAIL %0d errors", errors);
    $finish;
  end
endmodule
