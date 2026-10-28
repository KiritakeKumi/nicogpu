// ng_shell_ctrl.sv -- static shell control block (BAR1 register file).
//
// Runs on clk_sh (the PCIe bridge AXI clock) so it stays reachable while the
// RM clocks are gated or the MMCM is being retuned.  Implements the RM life
// cycle interlocks of DFX_BOUNDARY.md: quiesce -> decouple -> reset ->
// partial reconfiguration -> compatibility check -> release, plus MSI arming.
// Offsets come from the generated ng_dfx_pkg (SH_*).
// (SHELL_REV 3): PR_OPT.DONE_SEL / AVAIL_GATE, PR_STATUS[16:13],
// PR_ICAP_WORDS, the ICAP readback sequencer (PR_RB_*) and the pr_dbg
// black box for BSCAN USER1.
// SHELL_REV 4 (b19): PR_OPT.ORDER resets to 1 (was 0 in SHELL_REV 2/3, where
// the all-zero PR_OPT reproduces the P0-11 order that never syncs ICAPE3).
module ng_shell_ctrl
  import ng_dfx_pkg::*;
#(
  parameter logic [63:0] SHELL_UID  = 64'h0,
  parameter logic [15:0] SHELL_REV,                 // no default: ng_board_top is the single place that sets the revision
  parameter int          US_DIV     = 125,    // clk_sh cycles per microsecond (XDMA axi_aclk 125 MHz)
  parameter int          PR_DEPTH   = 512,
  parameter logic [1:0]  PR_ORDER_RESET = 2'd1  // PR_OPT.ORDER reset value: 1 = the ICAPE3 pin order that syncs the config engine (SHELL_REV 4; SHELL_REV 3 / b18 reset to 0)
) (
  input  logic        clk_sh,
  input  logic        rst_sh_n,
  // BAR1 AXI4-Lite slave
  input  logic [15:0] s_awaddr,
  input  logic        s_awvalid,
  output logic        s_awready,
  input  logic [31:0] s_wdata,
  input  logic [3:0]  s_wstrb,
  input  logic        s_wvalid,
  output logic        s_wready,
  output logic [1:0]  s_bresp,
  output logic        s_bvalid,
  input  logic        s_bready,
  input  logic [15:0] s_araddr,
  input  logic        s_arvalid,
  output logic        s_arready,
  output logic [31:0] s_rdata,
  output logic [1:0]  s_rresp,
  output logic        s_rvalid,
  input  logic        s_rready,
  // board status (asynchronous, synchronized here)
  input  logic        si_done,
  input  logic        si_err,
  input  logic        mmcm_locked,
  input  logic [3:0]  mem_calib,
  input  logic        link_up,
  input  logic        fence_stuck,          // BAR router: orphan responses not yet swallowed (clk_sh)
  input  logic        eos,                  // STARTUPE3 EOS (async; 1 = end of configuration startup)
  // clock meters, kHz, clk_sh domain (tear-free, see ng_clk_meas)
  input  logic [31:0] freq_core2x,
  input  logic [31:0] freq_l2,
  input  logic [31:0] freq_ts,
  // MMCM DRP (DCLK = clk_sh): one-cycle DEN, single transaction at a time
  output logic        drp_en,
  output logic        drp_we,
  output logic [6:0]  drp_a,
  output logic [15:0] drp_di,
  input  logic [15:0] drp_do,
  input  logic        drp_rdy,
  // RM control, clk_sh domain (the top synchronizes them into clk_l2)
  output logic        quiesce_req,
  output logic        decouple,
  output logic        rm_reset,
  output logic        info_sample,
  output logic        rm_clk_en,
  output logic        mmcm_reset,
  // RM status from the boundary (clk_l2 domain, synchronized here)
  input  logic        l2_rm_quiesce_ack,
  input  logic        l2_m_drained,
  input  logic        l2_iso_idle,
  input  logic        l2_rm_ready,
  input  logic        l2_rm_irq0,
  input  logic        l2_err_pulse_toggle,
  input  logic [31:0] l2_outstanding,       // debug only, may be torn
  input  logic [127:0] l2_rm_info,          // quasi-static while info_sample = 1
  // MSI request/ack towards the PCIe bridge (vector 0 = RM, vector 1 = shell)
  output logic [1:0]  msi_req,
  input  logic [1:0]  msi_ack,
  // ICAP domain
  input  logic        clk_icap,
  input  logic        rst_icap_n,
  // PR/ICAP black box for the BSCAN USER1 chain (read over JTAG after
  // the PCIe link is gone).  Mixed clk_sh / clk_icap bits sampled raw by TCK:
  // may tear while they move, exact once everything has stopped.
  output logic [159:0] pr_dbg
);
  // ----------------------------------------------------------------
  logic si_done_s, si_err_s, locked_s, link_s;
  logic [3:0] calib_s;
  logic ack_s, drained_s, idle_s, ready_s, irq0_s, errt_s, errt_q;
  ng_prim_sync #(.W(13)) u_sync_in (.clk(clk_sh), .rst_n(rst_sh_n),
    .d({si_done, si_err, mmcm_locked, mem_calib, link_up, l2_rm_quiesce_ack, l2_m_drained, l2_iso_idle, l2_rm_ready, l2_rm_irq0}),
    .q({si_done_s, si_err_s, locked_s, calib_s, link_s, ack_s, drained_s, idle_s, ready_s, irq0_s}));
  ng_prim_sync #(.W(1)) u_sync_err (.clk(clk_sh), .rst_n(rst_sh_n), .d(l2_err_pulse_toggle), .q(errt_s));

  // ----------------------------------------------------------------
  logic [31:0] scratch, irq_status, irq_enable, q_timeout, pr_expect, pr_words, err_count;
  logic        c_quiesce, c_decouple, c_reset, c_irqfwd, c_force;
  logic        dec_state, rst_state, forced;
  logic        pr_busy, pr_finish, pr_done, pr_err;
  logic [3:0]  pr_ecode;
  logic [1:0]  armed;
  logic [127:0] info;
  logic [15:0] drp_addr;
  logic [15:0] drp_data;
  logic        drp_ready;
  logic        c_gate, c_mmcm_rst;
  logic        fence_s;
  ng_prim_sync #(.W(1)) u_sync_fence (.clk(clk_sh), .rst_n(rst_sh_n), .d(fence_stuck), .q(fence_s));
  assign drp_a  = drp_addr[6:0];
  assign drp_di = drp_data;

  // rm_info check (DFX_BOUNDARY.md)
  logic info_valid, iface_ok, mem_ok;
  assign info_valid = (info[31:0] == NG_RM_INFO_MAGIC);
  assign iface_ok   = info_valid && (info[47:40] == NG_DFX_IFACE_VERSION[15:8])
                                 && (info[39:32] <= NG_DFX_IFACE_VERSION[7:0]);
  assign mem_ok     = ((info[115:112] & ~calib_s) == 4'h0);

  // ------------------------------------------------------------------ PR data path
  logic        pr_push, pr_full, pr_empty_icap, pr_pop, icap_done, icap_err;
  logic [31:0] w_data;
  logic [$clog2(PR_DEPTH):0] pr_fill;
  logic [31:0] pr_word_icap;
  logic        icap_done_s, icap_err_s, pr_busy_icap;
  ng_prim_fifo_async #(.W(32), .DEPTH(PR_DEPTH)) u_pr_fifo (
    .wclk(clk_sh), .wrst_n(rst_sh_n), .push(pr_push),
    .din({w_data[7:0], w_data[15:8], w_data[23:16], w_data[31:24]}),  // file byte order -> BE word; w_data = the word latched at the W handshake (pr_push is wr_go, >= 1 cycle later; the live s_wdata is already the next beat's lane)
    .full(pr_full), .wcount(pr_fill),
    .rclk(clk_icap), .rrst_n(rst_icap_n), .pop(pr_pop), .dout(pr_word_icap), .empty(pr_empty_icap));
  ng_prim_sync #(.W(1)) u_sync_prb (.clk(clk_icap), .rst_n(rst_icap_n), .d(pr_busy), .q(pr_busy_icap));
  // PR diagnostics (PR_OPT; ORDER resets to PR_ORDER_RESET, the other knobs to 0): word order, continuous CSIB
  logic [1:0]  pr_order, pr_order_icap;
  logic        pr_cont, pr_cont_icap, pr_force, pr_force_icap, pr_store, pr_store_icap, pr_fin_icap, pr_hold;
  logic        icap_done_raw, icap_err_raw, icap_done_raw_s, icap_err_raw_s, eos_s;
  logic        snap_req_t, snap_req_icap, snap_ack_t, snap_ack_s;
  logic [200:0] icap_obs, obs_sh;
  // completion selector, AVAIL gate/observation, readback sequencer
  logic [1:0]  pr_done_sel;
  logic        pr_agate, pr_agate_icap;
  logic        icap_avail, icap_avail_lost, icap_prdone_fell;
  logic        icap_avail_s, icap_avail_lost_s, icap_prdone_fell_s, eos_fell;
  logic [31:0] icap_words_live, icap_o_live;
  logic [4:0]  rb_reg;
  logic [7:0]  rb_cnt;
  logic [5:0]  rb_skip;
  logic [3:0]  rb_nop;
  logic        rb_req_t, rb_req_icap, rb_ack_t, rb_ack_s, rb_ack_q, rb_busy_icap, rb_busy_sh;
  logic        rb_done, rb_rej;
  logic [7:0]  rb_seq;
  logic [255:0] rb_data;     // clk_icap registers, stable while rb_busy_sh == 0 (read directly, see below)
  ng_prim_sync #(.W(7)) u_sync_opt (.clk(clk_icap), .rst_n(rst_icap_n),
    .d({pr_agate, pr_finish, pr_store, pr_force, pr_cont, pr_order}),
    .q({pr_agate_icap, pr_fin_icap, pr_store_icap, pr_force_icap, pr_cont_icap, pr_order_icap}));
  // STORE: store-and-forward, the ICAP side starts popping only after FINISH
  // (short test streams <= PR_DEPTH words: gapless delivery to the ICAP).
  // Never while the readback sequencer owns the ICAP pins; with AVAIL_GATE not
  // while ICAPE3 AVAIL is low.
  assign pr_pop = !pr_empty_icap && (!pr_store_icap || pr_fin_icap) && !rb_busy_icap &&
                  (!pr_agate_icap || icap_avail);
  ng_prim_sync #(.W(1)) u_sync_snq (.clk(clk_icap), .rst_n(rst_icap_n), .d(snap_req_t), .q(snap_req_icap));
  ng_prim_sync #(.W(1)) u_sync_rbq (.clk(clk_icap), .rst_n(rst_icap_n), .d(rb_req_t), .q(rb_req_icap));
  ng_prim_icap u_icap (.clk(clk_icap), .rst_n(rst_icap_n), .enable(pr_busy_icap),
    .valid(pr_pop), .ready(), .data(pr_word_icap),
    .order(pr_order_icap), .csib_cont(pr_cont_icap), .csib_force(pr_force_icap),
    .prdone(icap_done), .prerror(icap_err), .prdone_raw(icap_done_raw), .prerror_raw(icap_err_raw),
    .avail_raw(icap_avail), .avail_lost(icap_avail_lost), .prdone_fell(icap_prdone_fell),
    .snap_req(snap_req_icap), .snap_ack(snap_ack_t), .obs(icap_obs),
    .icap_words(icap_words_live), .o_live(icap_o_live),
    .rb_req(rb_req_icap), .rb_reg(rb_reg), .rb_cnt(rb_cnt), .rb_skip(rb_skip), .rb_nop(rb_nop),
    .rb_ack(rb_ack_t), .rb_busy(rb_busy_icap), .rb_data(rb_data));
  ng_prim_sync #(.W(9)) u_sync_icap (.clk(clk_sh), .rst_n(rst_sh_n),
    .d({rb_ack_t, icap_prdone_fell, icap_avail_lost, icap_avail,
        icap_done, icap_err, icap_done_raw, icap_err_raw, snap_ack_t}),
    .q({rb_ack_s, icap_prdone_fell_s, icap_avail_lost_s, icap_avail_s,
        icap_done_s, icap_err_s, icap_done_raw_s, icap_err_raw_s, snap_ack_s}));
  ng_prim_sync #(.W(1)) u_sync_eos (.clk(clk_sh), .rst_n(rst_sh_n), .d(eos), .q(eos_s));
  assign rb_busy_sh = (rb_req_t != rb_ack_q);    // drops the cycle after DONE/SEQ update

  // heartbeats for the JTAG black box: if a counter stops between two
  // USER1 reads, its clock (or GWE) stopped.
  logic [25:0] hb_sh, hb_icap;
  always_ff @(posedge clk_icap) begin
    if (!rst_icap_n) hb_icap <= '0;
    else             hb_icap <= hb_icap + 26'd1;
  end

  // ------------------------------------------------------------------ AXI-Lite write
  logic        aw_pend, w_pend;
  logic [15:0] aw_addr;
  logic        wr_go, is_pr_data;
  // W1C mask from the write channel.  It must be folded into the single
  // irq_status assignment below: a separate always_ff assignment here was
  // silently overridden by that later one, so status bits never cleared and
  // each EOI re-sent MSI forever (2026-09-26 board bug, MSI storm).
  logic [31:0] irq_w1c;
  assign s_awready  = !aw_pend;
  assign s_wready   = !w_pend;
  assign is_pr_data = (aw_addr[15:12] == SH_PR_DATA[15:12]);
  assign wr_go      = aw_pend && w_pend && !s_bvalid && !(is_pr_data && pr_busy && pr_full);
  assign irq_w1c    = (wr_go && {aw_addr[15:2], 2'b00} == SH_SHELL_IRQ_STATUS) ? w_data : '0;
  assign pr_push    = wr_go && is_pr_data && pr_busy && !pr_finish;
  assign s_bresp    = 2'b00;
  assign s_rresp    = 2'b00;

  // ----------------------------------------------------------------
  logic [$clog2(US_DIV)-1:0] us_div;
  logic        us_tick;
  logic [31:0] q_timer;
  assign us_tick = (us_div == '0);

  logic [31:0] set_irq;
  // PR completion (DFX_BOUNDARY 6.2 step 6): FINISH, FIFO drained (pr_fill
  // is the write-side view through the synchronized read pointer, so 0 means
  // the ICAP side has popped every word). DONE_SEL picks what counts
  // as "the ICAP finished"; PRERROR / word-count mismatch are errors always.
  logic pr_drained, pr_bad, pr_ok;
  assign pr_drained = pr_busy && pr_finish && pr_fill == '0;
  assign pr_bad     = icap_err_s || pr_words != pr_expect;
  always_comb begin
    case (pr_done_sel)
      2'd0:    pr_ok = icap_done_s;                          // latched PRDONE (SHELL_REV 2)
      2'd1:    pr_ok = icap_prdone_fell_s && icap_done_raw_s; // PRDONE fell and rose again
      2'd2:    pr_ok = eos_fell && eos_s;                     // EOS fell and rose again
      default: pr_ok = 1'b1;                                  // drain only: the host verifies
    endcase
  end
  always_comb begin
    set_irq = '0;
    if (pr_drained && (pr_ok || pr_bad)) begin
      if (pr_bad) set_irq[1] = 1'b1;
      else        set_irq[0] = 1'b1;
    end
    if (c_quiesce && !(ack_s && drained_s) && q_timer == 32'd1 && us_tick) set_irq[2] = 1'b1;
    if (!c_decouple && dec_state && !pr_busy && !rst_state && ready_s && !(iface_ok && mem_ok)) set_irq[3] = 1'b1;
  end

  always_ff @(posedge clk_sh) begin
    if (!rst_sh_n) begin
      aw_pend <= 1'b0; w_pend <= 1'b0; s_bvalid <= 1'b0; s_rvalid <= 1'b0;
      scratch <= '0; irq_status <= '0; irq_enable <= '0; q_timeout <= 32'd10000;
      c_quiesce <= 1'b0; c_decouple <= 1'b1; c_reset <= 1'b1; c_irqfwd <= 1'b0; c_force <= 1'b0;
      dec_state <= 1'b1; rst_state <= 1'b1; forced <= 1'b0;
      pr_busy <= 1'b0; pr_finish <= 1'b0; pr_done <= 1'b0; pr_err <= 1'b0; pr_ecode <= '0;
      pr_expect <= '0; pr_words <= '0; err_count <= '0;
      pr_order <= PR_ORDER_RESET; pr_cont <= 1'b0; pr_force <= 1'b0; pr_store <= 1'b0; pr_hold <= 1'b0; snap_req_t <= 1'b0; obs_sh <= '0;
      pr_done_sel <= '0; pr_agate <= 1'b0; eos_fell <= 1'b0;
      rb_reg <= '0; rb_cnt <= '0; rb_skip <= '0; rb_nop <= '0; rb_req_t <= 1'b0; rb_ack_q <= 1'b0;
      rb_done <= 1'b0; rb_rej <= 1'b0; rb_seq <= '0; hb_sh <= '0;
      armed <= 2'b11; msi_req <= 2'b00; info <= '0;
      us_div <= '0; q_timer <= '0; errt_q <= 1'b0;
      drp_addr <= '0; drp_data <= '0; drp_ready <= 1'b1; drp_en <= 1'b0; drp_we <= 1'b0;
      c_gate <= 1'b0; c_mmcm_rst <= 1'b0;
    end else begin
      // ---- MMCM DRP: DEN is a single-cycle strobe one clock after the
      // CLK_DRP_ADDR write that carries the write/read bit; the read result
      // lands in CLK_DRP_DATA and DATA[31] (ready) comes back with DRDY
      drp_en <= 1'b0;
      if (drp_rdy) begin
        drp_ready <= 1'b1;
        if (!drp_we) drp_data <= drp_do;
      end
      // ---- write channel
      if (s_awvalid && s_awready) begin aw_pend <= 1'b1; aw_addr <= s_awaddr; end
      if (s_wvalid && s_wready)   begin w_pend  <= 1'b1; w_data  <= s_wdata;  end
      if (s_bvalid && s_bready) s_bvalid <= 1'b0;
      if (wr_go) begin
        aw_pend  <= 1'b0;
        w_pend   <= 1'b0;
        s_bvalid <= 1'b1;
        if (pr_push) pr_words <= pr_words + 32'd1;
        case ({aw_addr[15:2], 2'b00})
          SH_SHELL_SCRATCH:    scratch <= w_data;
          SH_SHELL_IRQ_STATUS: ;   // W1C applied via irq_w1c in the update below
          SH_SHELL_IRQ_ENABLE: irq_enable <= w_data & 32'h1F;
          SH_SHELL_IRQ_EOI:    armed <= armed | w_data[1:0];
          SH_RM_CTRL: begin
            c_quiesce  <= w_data[0];
            c_decouple <= w_data[1];
            c_reset    <= w_data[2];
            c_irqfwd   <= w_data[3];
            c_force    <= w_data[4];
          end
          SH_RM_QUIESCE_TIMEOUT: q_timeout <= w_data;
          SH_PR_CTRL: begin
            if (w_data[2]) begin                       // ABORT: RP content undefined, stay isolated
              pr_busy <= 1'b0; pr_finish <= 1'b0; pr_err <= 1'b1; pr_ecode <= 4'd7;
            end else if (w_data[0] && !pr_busy) begin  // START
              if (dec_state && rst_state) begin
                pr_busy <= 1'b1; pr_finish <= 1'b0; pr_done <= 1'b0; pr_err <= 1'b0;
                pr_ecode <= 4'd0; pr_words <= '0; eos_fell <= 1'b0;
              end else begin
                pr_err <= 1'b1; pr_ecode <= 4'd1;      // not decoupled / not in reset
              end
            end else if (w_data[1] && pr_busy) begin
              pr_finish <= 1'b1;
            end
          end
          SH_PR_EXPECT_WORDS: pr_expect <= w_data;
          SH_PR_OPT: begin pr_order <= w_data[1:0]; pr_cont <= w_data[2]; pr_hold <= w_data[3];
                                 pr_force <= w_data[4]; pr_store <= w_data[5];
                                 pr_done_sel <= w_data[7:6]; pr_agate <= w_data[8]; end
          // readback: parameters and the request toggle move together;
          // the ICAP side latches the parameters when it sees the toggle (2
          // synchronizer stages later), so they are stable by then.
          SH_PR_RB_CTRL: begin
            if (w_data[31]) begin
              if (!rb_busy_sh && pr_fill == '0) begin
                rb_reg <= w_data[4:0]; rb_cnt <= w_data[15:8]; rb_skip <= w_data[21:16]; rb_nop <= w_data[27:24];
                rb_req_t <= ~rb_req_t; rb_done <= 1'b0; rb_rej <= 1'b0;
              end else begin
                rb_rej <= 1'b1;
              end
            end else if (!rb_busy_sh) begin
              rb_reg <= w_data[4:0]; rb_cnt <= w_data[15:8]; rb_skip <= w_data[21:16]; rb_nop <= w_data[27:24];
            end
          end
          SH_CLK_CTRL: begin c_gate <= w_data[0]; c_mmcm_rst <= w_data[1]; end
          SH_CLK_DRP_ADDR: if (drp_ready) begin                // busy: the whole write is ignored
            drp_addr <= {9'h0, w_data[6:0]};
            if (w_data[31] != w_data[30]) begin                // exactly one strobe
              drp_en    <= 1'b1;
              drp_we    <= w_data[31];
              drp_ready <= 1'b0;
            end
          end
          SH_CLK_DRP_DATA: if (drp_ready) drp_data <= w_data[15:0];
          default: ;
        endcase
      end

      // ---- read channel
      if (s_rvalid && s_rready) s_rvalid <= 1'b0;
      if (s_arvalid && s_arready) begin
        s_rvalid <= 1'b1;
        case ({s_araddr[15:2], 2'b00})
          SH_SHELL_ID:         s_rdata <= NG_SHELL_ID;
          SH_SHELL_VERSION:    s_rdata <= {NG_DFX_IFACE_VERSION, SHELL_REV};
          SH_SHELL_CAPS:       s_rdata <= {16'h0, 4'd2, 4'hF, 4'h0, 1'b1, 3'b011};   // [3] ICAP_RB
          SH_SHELL_SCRATCH:    s_rdata <= scratch;
          SH_SHELL_UID_LO:     s_rdata <= SHELL_UID[31:0];
          SH_SHELL_UID_HI:     s_rdata <= SHELL_UID[63:32];
          SH_SHELL_STATUS:     s_rdata <= {22'h0, fence_s, link_s, calib_s, 1'b0, locked_s, si_err_s, si_done_s};
          SH_SHELL_IRQ_STATUS: s_rdata <= irq_status;
          SH_SHELL_IRQ_ENABLE: s_rdata <= irq_enable;
          SH_RM_CTRL:          s_rdata <= {27'h0, c_force, c_irqfwd, c_reset, c_decouple, c_quiesce};
          SH_RM_STATUS:        s_rdata <= {23'h0, mem_ok, forced, iface_ok, info_valid, ready_s, rst_state,
                                           dec_state, drained_s, ack_s};
          SH_RM_QUIESCE_TIMEOUT: s_rdata <= q_timeout;
          SH_RM_OUTSTANDING:   s_rdata <= l2_outstanding;
          SH_RM_INFO0:         s_rdata <= info[31:0];
          SH_RM_INFO1:         s_rdata <= info[63:32];
          SH_RM_INFO2:         s_rdata <= info[95:64];
          SH_RM_INFO3:         s_rdata <= info[127:96];
          SH_RM_ERR_COUNT:     s_rdata <= err_count;
          SH_PR_CTRL:          s_rdata <= {29'h0, 1'b0, pr_finish, pr_busy};
          SH_PR_STATUS:        s_rdata <= {15'h0, eos_fell, icap_prdone_fell_s, icap_avail_lost_s, icap_avail_s,
                                           eos_s, icap_done_raw_s, icap_err_raw_s, icap_done_s, icap_err_s,
                                           pr_ecode, 1'b0, pr_err, pr_done, pr_busy};
          SH_PR_WORDS:         s_rdata <= pr_words;
          SH_PR_EXPECT_WORDS:  s_rdata <= pr_expect;
          SH_PR_FIFO_FREE:     s_rdata <= 32'(PR_DEPTH) - 32'(pr_fill);
          SH_PR_OPT:           s_rdata <= {23'h0, pr_agate, pr_done_sel, pr_store, pr_force, pr_hold, pr_cont, pr_order};
          SH_PR_ICAP_O:        s_rdata <= obs_sh[31:0];
          SH_PR_ICAP_OSTAT:    s_rdata <= {16'h0, obs_sh[168:161], 7'h0, obs_sh[160]};
          SH_PR_ICAP_O_H0:     s_rdata <= obs_sh[63:32];
          SH_PR_ICAP_O_H1:     s_rdata <= obs_sh[95:64];
          SH_PR_ICAP_O_H2:     s_rdata <= obs_sh[127:96];
          SH_PR_ICAP_O_H3:     s_rdata <= obs_sh[159:128];
          SH_PR_ICAP_WORDS:    s_rdata <= obs_sh[200:169];
          SH_PR_RB_CTRL:       s_rdata <= {rb_busy_sh, 3'h0, rb_nop, 2'h0, rb_skip, rb_cnt, 3'h0, rb_reg};
          SH_PR_RB_STATUS:     s_rdata <= {16'h0, rb_seq, 5'h0, rb_rej, rb_done, rb_busy_sh};
          // rb_data lives in clk_icap and only changes while a readback runs
          // (cleared at its start, filled during its read window, then held);
          // the done toggle passes 2 synchronizer stages after the last
          // write, so a read after PR_RB_STATUS.DONE samples stable bits.
          SH_PR_RB_DATA0:      s_rdata <= rb_data[31:0];
          SH_PR_RB_DATA1:      s_rdata <= rb_data[63:32];
          SH_PR_RB_DATA2:      s_rdata <= rb_data[95:64];
          SH_PR_RB_DATA3:      s_rdata <= rb_data[127:96];
          SH_PR_RB_DATA4:      s_rdata <= rb_data[159:128];
          SH_PR_RB_DATA5:      s_rdata <= rb_data[191:160];
          SH_PR_RB_DATA6:      s_rdata <= rb_data[223:192];
          SH_PR_RB_DATA7:      s_rdata <= rb_data[255:224];
          SH_CLK_CTRL:         s_rdata <= {30'h0, c_mmcm_rst, c_gate};
          SH_CLK_STATUS:       s_rdata <= {30'h0, c_gate, locked_s};
          SH_CLK_DRP_ADDR:     s_rdata <= {16'h0, drp_addr};
          SH_CLK_DRP_DATA:     s_rdata <= {drp_ready, 15'h0, drp_data};
          SH_CLK_FREQ_CORE:    s_rdata <= {1'b0, freq_core2x[31:1]};   // clk_core = clk_core2x / 2
          SH_CLK_FREQ_L2:      s_rdata <= freq_l2;
          SH_CLK_FREQ_TS:      s_rdata <= freq_ts;
          SH_CLK_FREQ_CORE2X:  s_rdata <= freq_core2x;
          default:             s_rdata <= 32'h0;
        endcase
      end

      // ---- ICAP O snapshot: request toggle; the ICAP side copies live -> obs
      // and toggles ack; obs is stable from then until the next request, so
      // it is sampled tear-free here (PR_OPT.OBS_HOLD stops the refresh).
      if (!pr_hold && snap_ack_s == snap_req_t) begin
        obs_sh     <= icap_obs;
        snap_req_t <= ~snap_req_t;
      end

      // ---- PR completion (DONE_SEL, see pr_ok above)
      if (pr_drained) begin
        if (pr_bad) begin
          pr_busy <= 1'b0; pr_finish <= 1'b0; pr_err <= 1'b1;
          pr_ecode <= icap_err_s ? 4'd2 : 4'd5;
        end else if (pr_ok) begin
          pr_busy <= 1'b0; pr_finish <= 1'b0; pr_done <= 1'b1;
        end
      end
      if (pr_busy && !eos_s) eos_fell <= 1'b1;

      // ---- readback completion and heartbeat
      rb_ack_q <= rb_ack_s;
      if (rb_ack_s != rb_ack_q) begin rb_done <= 1'b1; rb_seq <= rb_seq + 8'd1; end
      hb_sh <= hb_sh + 26'd1;

      // ---- RM life cycle interlocks
      if (pr_busy) begin
        dec_state <= 1'b1;
        rst_state <= 1'b1;
      end else begin
        if (c_decouple && !dec_state && (drained_s || c_force)) begin
          dec_state <= 1'b1;
          forced    <= !idle_s;      // something (master or host side) was still in flight
        end else if (!c_decouple && dec_state && !rst_state && !forced && ready_s && iface_ok && mem_ok && idle_s) begin
          dec_state <= 1'b0;
        end
        if (c_reset && dec_state) rst_state <= 1'b1;
        else if (!c_reset)        rst_state <= 1'b0;
        if (rst_state) forced <= 1'b0;   // FORCED = forced decouple not yet followed by a reset
      end
      if (!pr_busy) info <= l2_rm_info;

      // ---- quiesce timeout (microseconds)
      us_div <= us_tick ? ($clog2(US_DIV))'(US_DIV - 1) : us_div - 1'b1;
      if (!c_quiesce || (ack_s && drained_s)) q_timer <= q_timeout;
      else if (us_tick && q_timer != 32'd0) q_timer <= q_timer - 32'd1;

      // ---- interrupts and MSI arming
      // single update: a W1C write and a same-cycle event combine, set wins
      irq_status <= (irq_status & ~irq_w1c) | set_irq;
      errt_q <= errt_s;
      if (errt_s != errt_q && err_count != 32'hFFFF_FFFF) err_count <= err_count + 32'd1;
      for (int v = 0; v < 2; v++) begin
        if (msi_req[v] && msi_ack[v]) begin
          msi_req[v] <= 1'b0;
          armed[v]   <= 1'b0;
        end else if (armed[v] && !msi_req[v]) begin
          msi_req[v] <= (v == 0) ? (irq0_s && c_irqfwd && !dec_state) : |(irq_status & irq_enable);
        end
      end
    end
  end

  assign s_arready   = !s_rvalid;
  assign quiesce_req = c_quiesce;
  assign decouple    = dec_state;
  assign rm_reset    = rst_state;
  assign info_sample = !pr_busy;
  assign rm_clk_en   = !c_gate;
  assign mmcm_reset  = c_mmcm_rst;

  // black box layout (ng_bscan_status USER1 bits [287:128]):
  //   [31:0] ICAP-side data words since START   [63:32] live ICAPE3.O
  //   [95:64] PR_WORDS (host side)               [111:96] hb_sh[25:10]   [127:112] hb_icap[25:10]
  //   [128] AVAIL [129] PRDONE raw [130] PRERROR raw [131] EOS (unsynchronized) [132] pr_busy
  //   [133] pr_finish [134] pr_done [135] pr_err [136] AVAIL_LOST [137] PRDONE_FELL [138] EOS_FELL
  //   [139] readback busy (ICAP side) [140] decoupled [141] RM in reset [143:142] PR_OPT.ORDER
  //   [151:144] readback SEQ [159:152] 0x5A (marks the extended chain)
  assign pr_dbg = {8'h5A, rb_seq, pr_order, rst_state, dec_state, rb_busy_icap, eos_fell, icap_prdone_fell,
                   icap_avail_lost, pr_err, pr_done, pr_finish, pr_busy, eos, icap_err_raw, icap_done_raw,
                   icap_avail, hb_icap[25:10], hb_sh[25:10], pr_words, icap_o_live, icap_words_live};
endmodule
