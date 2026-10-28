// tb_rm_test.sv -- unit testbench of the test RM (ng_gpu_rm, rm_kind 0x8001).
//
// DUT: rtl/shell/rm_test/*.sv (module ng_gpu_rm).  Models:
//   u_c1..u_c4, u_host  tb_rmt_slave memories with random ready/latency, in-order
//                       responses, SLVERR injection and master-side AXI
//                       protocol assertions (see tb_rmt_slave.sv)
//   s_axil_ctrl         serialized AXI4-Lite driver (lw / lr)
//   s_axi_bar2          pipelined AXI4 driver: AW, W, AR issued from queues by
//                       independent threads (W may run ahead of AW), random
//                       BREADY/RREADY; B/R checked in order against expected
//                       queues (ID, RESP, data, RLAST) plus VALID/payload
//                       stability on the RM's slave outputs
// Tests (each prints "TEST <name>"; any mismatch prints FAIL):
//   ident      ID/REV/CAPS/T_*/rm_info/rm_ready, undefined offsets read 0,
//              WIN_SEL truncation, OWNER reset values
//   bar2       per channel: random bursts 1..256 beats, AxADDR[2:0] != 0 with
//              AxSIZE=3, random byte strobes, write -> memory check -> read
//              back; WIN_SEL 0 and top slice (sliding window)
//   bar2err    narrow AxSIZE (1, 2) served as plain INCR (lane strobes, word read);
//              AxSIZE > 3 -> SLVERR (no channel access); non-owned channel
//              -> SLVERR / all-ones, pipelined with owned traffic, in order;
//              channel SLVERR -> BAR2 B SLVERR and per-beat RRESP SLVERR
//   bist       per channel x {PRBS, address pattern}: write+check, memory vs
//              TB model (pinned to tools/p0test/rmtest_model.h vectors),
//              BURST_BEATS 16/4/3, CFG_ERR cases (no bus traffic)
//   bisterr    corrupted words -> ERRCNT / FIRST_ERR_ADDR / FIRST_GOT / FIRST_EXP;
//              seed mismatch -> ERRCNT = all beats; slave SLVERR -> AXI_ERR /
//              AXI_ERRCNT, no hang; CLEAR
//   conc       four BISTs concurrently
//   cross      BAR2-written PRBS verified by BIST DO_CHECK; BIST DO_WRITE read
//              back over BAR2 (both patterns, non-zero WIN_SEL)
//   dma        host->Cn and Cn->host for all channels, multi-block, burst
//              shapes (C1 8 beats, C2/C4 16, C3 2x16, host 64), CFG_ERR
//              cases, error blocks with and without STOP_ON_ERR
//   irq        IRQ_STATUS set by BIST/DMA(IRQ_EN), IRQ_ENABLE gating, W1C
//   owner      OWNER change while BIST / DMA / BAR2 read busy -> OWNER_ERR
//   quiesce    traffic on every port, shell_quiesce_req: ack only when all
//              ports drained, no AW/AR afterwards, BAR0 and BAR2 (non-owned)
//              still answered, BAR2 write to an owned channel accepted but
//              deferred; release -> everything completes correctly
// Pass token: NG_RM_TEST_TB_PASS.  Plusargs: +quick (smaller sizes).
`timescale 1ns / 1ps

module tb_rm_test;
  // ---- DUT ports (ng_gpu_rm_ports.svh)
  logic clk_core; logic clk_core2x; logic clk_l2; logic clk_ts; logic rst_core_n; logic rst_l2_n;
  logic rst_ts_n; logic [19:0] s_axil_ctrl_awaddr; logic [2:0] s_axil_ctrl_awprot;
  logic s_axil_ctrl_awvalid; logic s_axil_ctrl_awready; logic [31:0] s_axil_ctrl_wdata;
  logic [3:0] s_axil_ctrl_wstrb; logic s_axil_ctrl_wvalid; logic s_axil_ctrl_wready;
  logic [1:0] s_axil_ctrl_bresp; logic s_axil_ctrl_bvalid; logic s_axil_ctrl_bready;
  logic [19:0] s_axil_ctrl_araddr; logic [2:0] s_axil_ctrl_arprot; logic s_axil_ctrl_arvalid;
  logic s_axil_ctrl_arready; logic [31:0] s_axil_ctrl_rdata; logic [1:0] s_axil_ctrl_rresp;
  logic s_axil_ctrl_rvalid; logic s_axil_ctrl_rready; logic [3:0] s_axi_bar2_awid;
  logic [27:0] s_axi_bar2_awaddr; logic [7:0] s_axi_bar2_awlen; logic [2:0] s_axi_bar2_awsize;
  logic [1:0] s_axi_bar2_awburst; logic [3:0] s_axi_bar2_awcache; logic [2:0] s_axi_bar2_awprot;
  logic [3:0] s_axi_bar2_awqos; logic s_axi_bar2_awvalid; logic s_axi_bar2_awready;
  logic [63:0] s_axi_bar2_wdata; logic [7:0] s_axi_bar2_wstrb; logic s_axi_bar2_wlast;
  logic s_axi_bar2_wvalid; logic s_axi_bar2_wready; logic [3:0] s_axi_bar2_bid;
  logic [1:0] s_axi_bar2_bresp; logic s_axi_bar2_bvalid; logic s_axi_bar2_bready;
  logic [3:0] s_axi_bar2_arid; logic [27:0] s_axi_bar2_araddr; logic [7:0] s_axi_bar2_arlen;
  logic [2:0] s_axi_bar2_arsize; logic [1:0] s_axi_bar2_arburst; logic [3:0] s_axi_bar2_arcache;
  logic [2:0] s_axi_bar2_arprot; logic [3:0] s_axi_bar2_arqos; logic s_axi_bar2_arvalid;
  logic s_axi_bar2_arready; logic [3:0] s_axi_bar2_rid; logic [63:0] s_axi_bar2_rdata;
  logic [1:0] s_axi_bar2_rresp; logic s_axi_bar2_rlast; logic s_axi_bar2_rvalid;
  logic s_axi_bar2_rready; logic [3:0] m_axi_host_awid; logic [46:0] m_axi_host_awaddr;
  logic [7:0] m_axi_host_awlen; logic [2:0] m_axi_host_awsize; logic [1:0] m_axi_host_awburst;
  logic [3:0] m_axi_host_awcache; logic [2:0] m_axi_host_awprot; logic [3:0] m_axi_host_awqos;
  logic m_axi_host_awvalid; logic m_axi_host_awready; logic [63:0] m_axi_host_wdata;
  logic [7:0] m_axi_host_wstrb; logic m_axi_host_wlast; logic m_axi_host_wvalid;
  logic m_axi_host_wready; logic [3:0] m_axi_host_bid; logic [1:0] m_axi_host_bresp;
  logic m_axi_host_bvalid; logic m_axi_host_bready; logic [3:0] m_axi_host_arid;
  logic [46:0] m_axi_host_araddr; logic [7:0] m_axi_host_arlen; logic [2:0] m_axi_host_arsize;
  logic [1:0] m_axi_host_arburst; logic [3:0] m_axi_host_arcache; logic [2:0] m_axi_host_arprot;
  logic [3:0] m_axi_host_arqos; logic m_axi_host_arvalid; logic m_axi_host_arready;
  logic [3:0] m_axi_host_rid; logic [63:0] m_axi_host_rdata; logic [1:0] m_axi_host_rresp;
  logic m_axi_host_rlast; logic m_axi_host_rvalid; logic m_axi_host_rready;
  logic [5:0] m_axi_c1_awid; logic [32:0] m_axi_c1_awaddr; logic [7:0] m_axi_c1_awlen;
  logic [2:0] m_axi_c1_awsize; logic [1:0] m_axi_c1_awburst; logic [3:0] m_axi_c1_awcache;
  logic [2:0] m_axi_c1_awprot; logic [3:0] m_axi_c1_awqos; logic m_axi_c1_awvalid;
  logic m_axi_c1_awready; logic [511:0] m_axi_c1_wdata; logic [63:0] m_axi_c1_wstrb;
  logic m_axi_c1_wlast; logic m_axi_c1_wvalid; logic m_axi_c1_wready; logic [5:0] m_axi_c1_bid;
  logic [1:0] m_axi_c1_bresp; logic m_axi_c1_bvalid; logic m_axi_c1_bready;
  logic [5:0] m_axi_c1_arid; logic [32:0] m_axi_c1_araddr; logic [7:0] m_axi_c1_arlen;
  logic [2:0] m_axi_c1_arsize; logic [1:0] m_axi_c1_arburst; logic [3:0] m_axi_c1_arcache;
  logic [2:0] m_axi_c1_arprot; logic [3:0] m_axi_c1_arqos; logic m_axi_c1_arvalid;
  logic m_axi_c1_arready; logic [5:0] m_axi_c1_rid; logic [511:0] m_axi_c1_rdata;
  logic [1:0] m_axi_c1_rresp; logic m_axi_c1_rlast; logic m_axi_c1_rvalid; logic m_axi_c1_rready;
  logic [5:0] m_axi_c2_awid; logic [31:0] m_axi_c2_awaddr; logic [7:0] m_axi_c2_awlen;
  logic [2:0] m_axi_c2_awsize; logic [1:0] m_axi_c2_awburst; logic [3:0] m_axi_c2_awcache;
  logic [2:0] m_axi_c2_awprot; logic [3:0] m_axi_c2_awqos; logic m_axi_c2_awvalid;
  logic m_axi_c2_awready; logic [255:0] m_axi_c2_wdata; logic [31:0] m_axi_c2_wstrb;
  logic m_axi_c2_wlast; logic m_axi_c2_wvalid; logic m_axi_c2_wready; logic [5:0] m_axi_c2_bid;
  logic [1:0] m_axi_c2_bresp; logic m_axi_c2_bvalid; logic m_axi_c2_bready;
  logic [5:0] m_axi_c2_arid; logic [31:0] m_axi_c2_araddr; logic [7:0] m_axi_c2_arlen;
  logic [2:0] m_axi_c2_arsize; logic [1:0] m_axi_c2_arburst; logic [3:0] m_axi_c2_arcache;
  logic [2:0] m_axi_c2_arprot; logic [3:0] m_axi_c2_arqos; logic m_axi_c2_arvalid;
  logic m_axi_c2_arready; logic [5:0] m_axi_c2_rid; logic [255:0] m_axi_c2_rdata;
  logic [1:0] m_axi_c2_rresp; logic m_axi_c2_rlast; logic m_axi_c2_rvalid; logic m_axi_c2_rready;
  logic [5:0] m_axi_c3_awid; logic [30:0] m_axi_c3_awaddr; logic [7:0] m_axi_c3_awlen;
  logic [2:0] m_axi_c3_awsize; logic [1:0] m_axi_c3_awburst; logic [3:0] m_axi_c3_awcache;
  logic [2:0] m_axi_c3_awprot; logic [3:0] m_axi_c3_awqos; logic m_axi_c3_awvalid;
  logic m_axi_c3_awready; logic [127:0] m_axi_c3_wdata; logic [15:0] m_axi_c3_wstrb;
  logic m_axi_c3_wlast; logic m_axi_c3_wvalid; logic m_axi_c3_wready; logic [5:0] m_axi_c3_bid;
  logic [1:0] m_axi_c3_bresp; logic m_axi_c3_bvalid; logic m_axi_c3_bready;
  logic [5:0] m_axi_c3_arid; logic [30:0] m_axi_c3_araddr; logic [7:0] m_axi_c3_arlen;
  logic [2:0] m_axi_c3_arsize; logic [1:0] m_axi_c3_arburst; logic [3:0] m_axi_c3_arcache;
  logic [2:0] m_axi_c3_arprot; logic [3:0] m_axi_c3_arqos; logic m_axi_c3_arvalid;
  logic m_axi_c3_arready; logic [5:0] m_axi_c3_rid; logic [127:0] m_axi_c3_rdata;
  logic [1:0] m_axi_c3_rresp; logic m_axi_c3_rlast; logic m_axi_c3_rvalid; logic m_axi_c3_rready;
  logic [5:0] m_axi_c4_awid; logic [31:0] m_axi_c4_awaddr; logic [7:0] m_axi_c4_awlen;
  logic [2:0] m_axi_c4_awsize; logic [1:0] m_axi_c4_awburst; logic [3:0] m_axi_c4_awcache;
  logic [2:0] m_axi_c4_awprot; logic [3:0] m_axi_c4_awqos; logic m_axi_c4_awvalid;
  logic m_axi_c4_awready; logic [255:0] m_axi_c4_wdata; logic [31:0] m_axi_c4_wstrb;
  logic m_axi_c4_wlast; logic m_axi_c4_wvalid; logic m_axi_c4_wready; logic [5:0] m_axi_c4_bid;
  logic [1:0] m_axi_c4_bresp; logic m_axi_c4_bvalid; logic m_axi_c4_bready;
  logic [5:0] m_axi_c4_arid; logic [31:0] m_axi_c4_araddr; logic [7:0] m_axi_c4_arlen;
  logic [2:0] m_axi_c4_arsize; logic [1:0] m_axi_c4_arburst; logic [3:0] m_axi_c4_arcache;
  logic [2:0] m_axi_c4_arprot; logic [3:0] m_axi_c4_arqos; logic m_axi_c4_arvalid;
  logic m_axi_c4_arready; logic [5:0] m_axi_c4_rid; logic [255:0] m_axi_c4_rdata;
  logic [1:0] m_axi_c4_rresp; logic m_axi_c4_rlast; logic m_axi_c4_rvalid; logic m_axi_c4_rready;
  logic [3:0] shell_mem_ready; logic shell_quiesce_req; logic rm_quiesce_ack; logic rm_ready;
  logic [3:0] rm_irq; logic [127:0] rm_info; logic dbg_hub_drck; logic dbg_hub_tck;
  logic dbg_hub_tms; logic dbg_hub_tdi; logic dbg_hub_sel; logic dbg_hub_shift;
  logic dbg_hub_capture; logic dbg_hub_update; logic dbg_hub_reset; logic dbg_hub_runtest;
  logic dbg_hub_bscanid_en; logic dbg_hub_tdo; logic dbg_obs_tck; logic dbg_obs_tdi;
  logic dbg_obs_sel; logic dbg_obs_capture; logic dbg_obs_shift; logic dbg_obs_update;
  logic dbg_obs_tdo;

  // ------------------------------------------------------------------ clock / reset / tie-offs
  initial clk_l2 = 1'b0;
  always #2 clk_l2 = ~clk_l2;   // 250 MHz
  assign clk_core = 1'b0; assign clk_core2x = 1'b0; assign clk_ts = 1'b0;
  assign rst_core_n = rst_l2_n; assign rst_ts_n = rst_l2_n;
  assign dbg_hub_drck = 1'b0; assign dbg_hub_tck = 1'b0; assign dbg_hub_tms = 1'b0;
  assign dbg_hub_tdi = 1'b0; assign dbg_hub_sel = 1'b0; assign dbg_hub_shift = 1'b0;
  assign dbg_hub_capture = 1'b0; assign dbg_hub_update = 1'b0; assign dbg_hub_reset = 1'b0;
  assign dbg_hub_runtest = 1'b0; assign dbg_hub_bscanid_en = 1'b0;
  assign dbg_obs_tck = 1'b0; assign dbg_obs_tdi = 1'b0; assign dbg_obs_sel = 1'b0;
  assign dbg_obs_capture = 1'b0; assign dbg_obs_shift = 1'b0; assign dbg_obs_update = 1'b0;
  assign s_axil_ctrl_awprot = 3'b000; assign s_axil_ctrl_arprot = 3'b000;
  assign s_axi_bar2_awburst = 2'b01; assign s_axi_bar2_awcache = 4'b0011;
  assign s_axi_bar2_awprot = 3'b010; assign s_axi_bar2_awqos = 4'h0;
  assign s_axi_bar2_arburst = 2'b01; assign s_axi_bar2_arcache = 4'b0011;
  assign s_axi_bar2_arprot = 3'b010; assign s_axi_bar2_arqos = 4'h0;

  localparam logic [31:0] BUILD_ID = 32'hC0FF_EE01;
  localparam logic [15:0] RM_REV   = 16'h0001;

  ng_gpu_rm #(.BUILD_ID(BUILD_ID), .RM_REV(RM_REV)) dut (.*);

  // ------------------------------------------------------------------ slave models
  `define SLV(p, DWV, AWV, IWV, ML, MW, MR) \
    tb_rmt_slave #(.DW(DWV), .AW(AWV), .IW(IWV), .MAXLEN(ML), .MAXWR(MW), .MAXRD(MR), .NAME(`"p`")) u_``p ( \
      .clk(clk_l2), .rst_n(rst_l2_n), \
      .awid(m_axi_``p``_awid), .awaddr(m_axi_``p``_awaddr), .awlen(m_axi_``p``_awlen), \
      .awsize(m_axi_``p``_awsize), .awburst(m_axi_``p``_awburst), .awcache(m_axi_``p``_awcache), \
      .awprot(m_axi_``p``_awprot), .awqos(m_axi_``p``_awqos), .awvalid(m_axi_``p``_awvalid), \
      .awready(m_axi_``p``_awready), .wdata(m_axi_``p``_wdata), .wstrb(m_axi_``p``_wstrb), \
      .wlast(m_axi_``p``_wlast), .wvalid(m_axi_``p``_wvalid), .wready(m_axi_``p``_wready), \
      .bid(m_axi_``p``_bid), .bresp(m_axi_``p``_bresp), .bvalid(m_axi_``p``_bvalid), \
      .bready(m_axi_``p``_bready), .arid(m_axi_``p``_arid), .araddr(m_axi_``p``_araddr), \
      .arlen(m_axi_``p``_arlen), .arsize(m_axi_``p``_arsize), .arburst(m_axi_``p``_arburst), \
      .arcache(m_axi_``p``_arcache), .arprot(m_axi_``p``_arprot), .arqos(m_axi_``p``_arqos), \
      .arvalid(m_axi_``p``_arvalid), .arready(m_axi_``p``_arready), .rid(m_axi_``p``_rid), \
      .rdata(m_axi_``p``_rdata), .rresp(m_axi_``p``_rresp), .rlast(m_axi_``p``_rlast), \
      .rvalid(m_axi_``p``_rvalid), .rready(m_axi_``p``_rready));
  `SLV(c1, 512, 33, 6, 15, 32, 64)
  `SLV(c2, 256, 32, 6, 15, 32, 64)
  `SLV(c3, 128, 31, 6, 15, 32, 32)
  `SLV(c4, 256, 32, 6, 15, 32, 64)
  `SLV(host, 64, 47, 4, 63, 8, 8)
  `undef SLV

  // ----------------------------------------------------------------
  int  errors = 0;
  bit  quick  = 1'b0;
  function automatic void fail(string s);
    errors++;
    if (errors <= 60) $display("FAIL t=%0t %s", $time, s);
  endfunction
  function automatic void chk(bit ok, string s);
    if (!ok) fail(s);
  endfunction

  function automatic int slave_errors();
    return u_c1.errors + u_c2.errors + u_c3.errors + u_c4.errors + u_host.errors;
  endfunction

  // channel geometry (n = 0..3 <-> C1..C4)
  function automatic int ch_bpb(int n);
    return (n == 0) ? 64 : (n == 2) ? 16 : 32;
  endfunction
  function automatic int ch_winbits(int n);
    return (n == 0) ? 7 : (n == 2) ? 5 : 6;
  endfunction

  // ------------------------------------------------------------------ reference model
  function automatic logic [63:0] mix64(logic [63:0] z);
    z = (z ^ (z >> 30)) * 64'hBF58_476D_1CE4_E5B9;
    z = (z ^ (z >> 27)) * 64'h94D0_49BB_1331_11EB;
    return z ^ (z >> 31);
  endfunction
  function automatic logic [63:0] prbs_lane(logic [31:0] seed, logic [31:0] beat, int k);
    return mix64({24'h0, beat, 8'(k)} ^ {seed, ~seed});
  endfunction
  // 64-bit word at 8-byte aligned channel byte address a
  function automatic logic [63:0] pat_word(int pat, logic [31:0] seed, int n, longint unsigned a);
    longint unsigned beat;
    int k;
    beat = a / ch_bpb(n);
    k    = (a / 8) % (ch_bpb(n) / 8);
    if (pat != 0) return {16'hA5A5 ^ seed[15:0], beat[15:0], 32'(beat * ch_bpb(n))};
    return prbs_lane(seed, 32'(beat), k);
  endfunction

  // channel memory access (64-bit lane words)
  function automatic logic [63:0] ch_peek64(int n, longint unsigned a);
    logic [511:0] b;
    case (n)
      0: b = 512'(u_c1.peek(a >> 6));
      1: b = 512'(u_c2.peek(a >> 5));
      2: b = 512'(u_c3.peek(a >> 4));
      default: b = 512'(u_c4.peek(a >> 5));
    endcase
    return b[64 * ((a / 8) % (ch_bpb(n) / 8)) +: 64];
  endfunction
  function automatic void ch_poke64(int n, longint unsigned a, logic [63:0] d);
    int k;
    k = (a / 8) % (ch_bpb(n) / 8);
    case (n)
      0: begin logic [511:0] b; b = u_c1.peek(a >> 6); b[64*k +: 64] = d; u_c1.poke(a >> 6, b); end
      1: begin logic [255:0] b; b = u_c2.peek(a >> 5); b[64*k +: 64] = d; u_c2.poke(a >> 5, b); end
      2: begin logic [127:0] b; b = u_c3.peek(a >> 4); b[64*k +: 64] = d; u_c3.poke(a >> 4, b); end
      default: begin logic [255:0] b; b = u_c4.peek(a >> 5); b[64*k +: 64] = d; u_c4.poke(a >> 5, b); end
    endcase
  endfunction
  function automatic int ch_naw(int n);
    case (n) 0: return u_c1.n_aw; 1: return u_c2.n_aw; 2: return u_c3.n_aw; default: return u_c4.n_aw; endcase
  endfunction
  function automatic int ch_nar(int n);
    case (n) 0: return u_c1.n_ar; 1: return u_c2.n_ar; 2: return u_c3.n_ar; default: return u_c4.n_ar; endcase
  endfunction
  function automatic void ch_err(int n, bit en, longint unsigned lo, longint unsigned hi);
    case (n)
      0: begin u_c1.err_en = en; u_c1.err_lo = lo; u_c1.err_hi = hi; end
      1: begin u_c2.err_en = en; u_c2.err_lo = lo; u_c2.err_hi = hi; end
      2: begin u_c3.err_en = en; u_c3.err_lo = lo; u_c3.err_hi = hi; end
      default: begin u_c4.err_en = en; u_c4.err_lo = lo; u_c4.err_hi = hi; end
    endcase
  endfunction
  function automatic int ch_awlen_hist(int n, int len);
    case (n)
      0: return u_c1.aw_len_hist[len]; 1: return u_c2.aw_len_hist[len];
      2: return u_c3.aw_len_hist[len]; default: return u_c4.aw_len_hist[len];
    endcase
  endfunction
  function automatic int ch_arlen_hist(int n, int len);
    case (n)
      0: return u_c1.ar_len_hist[len]; 1: return u_c2.ar_len_hist[len];
      2: return u_c3.ar_len_hist[len]; default: return u_c4.ar_len_hist[len];
    endcase
  endfunction
  function automatic bit all_slaves_idle();
    return u_c1.idle() && u_c2.idle() && u_c3.idle() && u_c4.idle() && u_host.idle();
  endfunction

  // ------------------------------------------------------------------ AXI4-Lite driver
  semaphore lite_sem = new(1);

  task automatic lw(input logic [19:0] a, input logic [31:0] d, input logic [3:0] s = 4'hF);
    lite_sem.get(1);
    s_axil_ctrl_awaddr  <= a;
    s_axil_ctrl_awvalid <= 1'b1;
    s_axil_ctrl_wdata   <= d;
    s_axil_ctrl_wstrb   <= s;
    s_axil_ctrl_wvalid  <= 1'b1;
    fork
      begin do @(posedge clk_l2); while (!s_axil_ctrl_awready); s_axil_ctrl_awvalid <= 1'b0; end
      begin do @(posedge clk_l2); while (!s_axil_ctrl_wready);  s_axil_ctrl_wvalid  <= 1'b0; end
    join
    s_axil_ctrl_bready <= 1'b1;
    do @(posedge clk_l2); while (!s_axil_ctrl_bvalid);
    chk(s_axil_ctrl_bresp == 2'b00, $sformatf("lite write %h bresp %0d", a, s_axil_ctrl_bresp));
    s_axil_ctrl_bready <= 1'b0;
    lite_sem.put(1);
  endtask

  task automatic lr(input logic [19:0] a, output logic [31:0] d);
    lite_sem.get(1);
    s_axil_ctrl_araddr  <= a;
    s_axil_ctrl_arvalid <= 1'b1;
    do @(posedge clk_l2); while (!s_axil_ctrl_arready);
    s_axil_ctrl_arvalid <= 1'b0;
    s_axil_ctrl_rready  <= 1'b1;
    do @(posedge clk_l2); while (!s_axil_ctrl_rvalid);
    d = s_axil_ctrl_rdata;
    chk(s_axil_ctrl_rresp == 2'b00, $sformatf("lite read %h rresp %0d", a, s_axil_ctrl_rresp));
    s_axil_ctrl_rready <= 1'b0;
    lite_sem.put(1);
  endtask

  task automatic lr_exp(input logic [19:0] a, input logic [31:0] e, input string what);
    logic [31:0] d;
    lr(a, d);
    chk(d === e, $sformatf("%s @%h: got %h want %h", what, a, d, e));
  endtask

  function automatic logic [19:0] BR(int n, int off);
    return 20'h08100 + 20'(64 * n) + 20'(off);
  endfunction
  function automatic logic [19:0] DR(int off);
    return 20'h08600 + 20'(off);
  endfunction

  // ------------------------------------------------------------------ BAR2 driver
  typedef struct { logic [3:0] id; logic [27:0] addr; logic [7:0] len; logic [2:0] size; } ax_t;
  typedef struct { logic [63:0] d; logic [7:0] s; logic last; } wb_t;
  typedef struct { logic [3:0] id; logic [1:0] resp; } eb_t;
  typedef struct { logic [3:0] id; int len; } er_t;
  ax_t         b2aw_q [$];
  ax_t         b2ar_q [$];
  wb_t         b2w_q [$];
  eb_t         b2b_exp [$];
  er_t         b2r_exp [$];
  logic [63:0] b2r_dexp [$];
  logic [1:0]  b2r_rexp [$];
  int          b2_gap = 20, b2_rdy = 75;
  int          b2_rbeat = 0, b2_nb = 0, b2_nr = 0, b2_aw_acc = 0;
  logic        b2_bwait, b2_rwait;
  logic [5:0]  b2_bhold;
  logic [70:0] b2_rhold;

  always @(posedge clk_l2) begin
    if (!rst_l2_n) begin
      s_axi_bar2_awvalid <= 1'b0; s_axi_bar2_wvalid <= 1'b0; s_axi_bar2_arvalid <= 1'b0;
      s_axi_bar2_bready <= 1'b0; s_axi_bar2_rready <= 1'b0;
      b2_bwait = 1'b0; b2_rwait = 1'b0;
    end else begin
      // AW
      if (s_axi_bar2_awvalid && s_axi_bar2_awready) b2_aw_acc++;
      if (!s_axi_bar2_awvalid || s_axi_bar2_awready) begin
        if (b2aw_q.size() != 0 && $urandom_range(0, 99) >= b2_gap) begin
          ax_t a;
          a = b2aw_q.pop_front();
          s_axi_bar2_awvalid <= 1'b1;
          s_axi_bar2_awid <= a.id; s_axi_bar2_awaddr <= a.addr;
          s_axi_bar2_awlen <= a.len; s_axi_bar2_awsize <= a.size;
        end else begin
          s_axi_bar2_awvalid <= 1'b0;
        end
      end
      // W (independent of AW: may run ahead)
      if (!s_axi_bar2_wvalid || s_axi_bar2_wready) begin
        if (b2w_q.size() != 0 && $urandom_range(0, 99) >= b2_gap) begin
          wb_t w;
          w = b2w_q.pop_front();
          s_axi_bar2_wvalid <= 1'b1;
          s_axi_bar2_wdata <= w.d; s_axi_bar2_wstrb <= w.s; s_axi_bar2_wlast <= w.last;
        end else begin
          s_axi_bar2_wvalid <= 1'b0;
        end
      end
      // AR
      if (!s_axi_bar2_arvalid || s_axi_bar2_arready) begin
        if (b2ar_q.size() != 0 && $urandom_range(0, 99) >= b2_gap) begin
          ax_t a;
          a = b2ar_q.pop_front();
          s_axi_bar2_arvalid <= 1'b1;
          s_axi_bar2_arid <= a.id; s_axi_bar2_araddr <= a.addr;
          s_axi_bar2_arlen <= a.len; s_axi_bar2_arsize <= a.size;
        end else begin
          s_axi_bar2_arvalid <= 1'b0;
        end
      end
      // RM slave outputs: VALID held / payload stable until READY
      if (b2_bwait && (!s_axi_bar2_bvalid || {s_axi_bar2_bid, s_axi_bar2_bresp} != b2_bhold))
        fail("BAR2 B dropped/changed before BREADY");
      if (b2_rwait && (!s_axi_bar2_rvalid ||
                       {s_axi_bar2_rid, s_axi_bar2_rdata, s_axi_bar2_rresp, s_axi_bar2_rlast} != b2_rhold))
        fail("BAR2 R dropped/changed before RREADY");
      b2_bwait = s_axi_bar2_bvalid && !s_axi_bar2_bready;
      b2_rwait = s_axi_bar2_rvalid && !s_axi_bar2_rready;
      b2_bhold = {s_axi_bar2_bid, s_axi_bar2_bresp};
      b2_rhold = {s_axi_bar2_rid, s_axi_bar2_rdata, s_axi_bar2_rresp, s_axi_bar2_rlast};
      // B
      if (s_axi_bar2_bvalid && s_axi_bar2_bready) begin
        b2_nb++;
        if (b2b_exp.size() == 0) begin
          fail("BAR2 unexpected B");
        end else begin
          eb_t e;
          e = b2b_exp.pop_front();
          chk(s_axi_bar2_bid == e.id && s_axi_bar2_bresp == e.resp,
              $sformatf("BAR2 B id %0d resp %0d, want id %0d resp %0d", s_axi_bar2_bid, s_axi_bar2_bresp, e.id, e.resp));
        end
      end
      // R
      if (s_axi_bar2_rvalid && s_axi_bar2_rready) begin
        b2_nr++;
        if (b2r_exp.size() == 0) begin
          fail("BAR2 unexpected R");
        end else begin
          logic [63:0] d;
          logic [1:0]  r;
          d = b2r_dexp.pop_front();
          r = b2r_rexp.pop_front();
          chk(s_axi_bar2_rid == b2r_exp[0].id, $sformatf("BAR2 RID %0d want %0d", s_axi_bar2_rid, b2r_exp[0].id));
          chk(s_axi_bar2_rdata === d && s_axi_bar2_rresp == r,
              $sformatf("BAR2 R beat %0d id %0d: data %h resp %0d, want %h resp %0d",
                        b2_rbeat, s_axi_bar2_rid, s_axi_bar2_rdata, s_axi_bar2_rresp, d, r));
          chk(s_axi_bar2_rlast == (b2_rbeat == b2r_exp[0].len),
              $sformatf("BAR2 RLAST %0d at beat %0d of len %0d", s_axi_bar2_rlast, b2_rbeat, b2r_exp[0].len));
          if (b2_rbeat == b2r_exp[0].len) begin
            void'(b2r_exp.pop_front());
            b2_rbeat = 0;
          end else begin
            b2_rbeat++;
          end
        end
      end
      s_axi_bar2_bready <= ($urandom_range(0, 99) < b2_rdy);
      s_axi_bar2_rready <= ($urandom_range(0, 99) < b2_rdy);
    end
  end

  // golden BAR2-visible bytes, key = {n, channel byte address}
  byte unsigned gold [longint unsigned];
  logic [6:0]   win_tb [4] = '{default: '0};

  function automatic longint unsigned gkey(int n, longint unsigned ca);
    return (longint'(n) << 40) | ca;
  endfunction
  function automatic byte unsigned gold_get(int n, longint unsigned ca);
    logic [63:0] w;
    if (gold.exists(gkey(n, ca))) return gold[gkey(n, ca)];
    w = ch_peek64(n, ca & ~64'h7);
    return w[8 * (ca % 8) +: 8];
  endfunction
  function automatic longint unsigned chaddr(int n, longint unsigned off);
    return (longint'(win_tb[n]) << 26) | (off & 64'h3FF_FFFF);
  endfunction

  // BAR2 write: beats of 64 bits from the 8-aligned start (off[2:0] ignored by the RM)
  task automatic b2_write(input int n, input longint unsigned off, input int beats, input logic [2:0] size,
                          input logic [63:0] d [$], input logic [7:0] s [$], input logic [1:0] exp,
                          input bit upd, input logic [3:0] id = 4'h1);
    ax_t a;
    longint unsigned base;
    a.id = id; a.addr = {2'(n), 26'(off)}; a.len = 8'(beats - 1); a.size = size;
    b2aw_q.push_back(a);
    base = (off & ~64'h7);
    for (int i = 0; i < beats; i++) begin
      wb_t w;
      w.d = d[i]; w.s = s[i]; w.last = (i == beats - 1);
      b2w_q.push_back(w);
      if (upd)
        for (int b = 0; b < 8; b++)
          if (s[i][b]) gold[gkey(n, chaddr(n, base + 8 * i + b))] = d[i][8*b +: 8];
    end
    b2b_exp.push_back('{id, exp});
  endtask

  // BAR2 read: mode 0 expect golden data OKAY, 1 expect all-ones SLVERR
  task automatic b2_read(input int n, input longint unsigned off, input int beats, input logic [2:0] size,
                         input int mode, input logic [3:0] id = 4'h2);
    ax_t a;
    longint unsigned base;
    a.id = id; a.addr = {2'(n), 26'(off)}; a.len = 8'(beats - 1); a.size = size;
    b2ar_q.push_back(a);
    b2r_exp.push_back('{id, beats - 1});
    base = (off & ~64'h7);
    for (int i = 0; i < beats; i++) begin
      logic [63:0] w;
      if (mode == 1) begin
        w = '1;
        b2r_rexp.push_back(2'b10);
      end else begin
        for (int b = 0; b < 8; b++) w[8*b +: 8] = gold_get(n, chaddr(n, base + 8 * i + b));
        b2r_rexp.push_back(2'b00);
      end
      b2r_dexp.push_back(w);
    end
  endtask

  // narrow (AxSIZE < 3) BAR2 write, start aligned to 1<<size: beat i sits at the
  // INCR address, data/strobes on that beat's byte lanes; golden updated per byte
  task automatic b2_write_n(input int n, input longint unsigned off, input int beats, input logic [2:0] size,
                            input logic [3:0] id);
    ax_t a;
    longint unsigned ai;
    a.id = id; a.addr = {2'(n), 26'(off)}; a.len = 8'(beats - 1); a.size = size;
    b2aw_q.push_back(a);
    for (int i = 0; i < beats; i++) begin
      wb_t w;
      int lane, nb;
      ai = off + (longint'(i) << size);
      lane = int'(ai & 7);
      nb = 1 << size;
      w.d = '0; w.s = '0;
      for (int b = 0; b < nb; b++) begin
        logic [7:0] v;
        v = 8'($urandom);
        w.d[8*(lane + b) +: 8] = v;
        w.s[lane + b] = 1'b1;
        gold[gkey(n, chaddr(n, (ai & ~64'h7) + lane + b))] = v;
      end
      w.last = (i == beats - 1);
      b2w_q.push_back(w);
    end
    b2b_exp.push_back('{id, 2'b00});
  endtask

  // narrow BAR2 read: the RM returns the whole 64-bit word holding each beat address
  task automatic b2_read_n(input int n, input longint unsigned off, input int beats, input logic [2:0] size,
                           input logic [3:0] id);
    ax_t a;
    longint unsigned ai;
    a.id = id; a.addr = {2'(n), 26'(off)}; a.len = 8'(beats - 1); a.size = size;
    b2ar_q.push_back(a);
    b2r_exp.push_back('{id, beats - 1});
    for (int i = 0; i < beats; i++) begin
      logic [63:0] w;
      ai = off + (longint'(i) << size);
      for (int b = 0; b < 8; b++) w[8*b +: 8] = gold_get(n, chaddr(n, (ai & ~64'h7) + b));
      b2r_rexp.push_back(2'b00);
      b2r_dexp.push_back(w);
    end
  endtask

  task automatic b2_drain(input string what, input int max_cyc = 400000);
    int c;
    c = 0;
    while ((b2aw_q.size() || b2w_q.size() || b2ar_q.size() || b2b_exp.size() || b2r_exp.size()) && c < max_cyc) begin
      @(posedge clk_l2);
      c++;
    end
    if (c >= max_cyc) fail($sformatf("%s: BAR2 drain timeout (aw %0d w %0d ar %0d b %0d r %0d)", what,
                                     b2aw_q.size(), b2w_q.size(), b2ar_q.size(), b2b_exp.size(), b2r_exp.size()));
    repeat (4) @(posedge clk_l2);
  endtask

  task automatic set_win(input int n, input logic [31:0] v);
    lw(20'h08020 + 20'(4 * n), v);
    win_tb[n] = 7'(v & ((1 << ch_winbits(n)) - 1));
  endtask
  task automatic set_owner(input int n, input int o, input bit expect_ok = 1'b1);
    logic [31:0] st;
    lw(20'h0800C, 32'h1);   // clear OWNER_ERR
    lw(20'h08010 + 20'(4 * n), 32'(o));
    lr(20'h0800C, st);
    if (expect_ok) chk(st[0] == 1'b0, $sformatf("OWNER[%0d]=%0d unexpectedly refused", n, o));
    else           chk(st[0] == 1'b1, $sformatf("OWNER[%0d]=%0d change while busy not refused", n, o));
    if (expect_ok) lr_exp(20'h08010 + 20'(4 * n), 32'(o), "OWNER readback");
    lw(20'h0800C, 32'h1);
  endtask

  // ------------------------------------------------------------------ BIST helpers
  task automatic bist_prog(input int n, input int mode, input logic [31:0] seed, input longint unsigned base,
                           input logic [31:0] len, input int bb);
    lw(BR(n, 'h00), 32'h2);          // CLEAR
    lw(BR(n, 'h04), 32'h1E);         // W1C
    lw(BR(n, 'h08), 32'(mode));
    lw(BR(n, 'h0C), seed);
    lw(BR(n, 'h10), base[31:0]);
    lw(BR(n, 'h14), 32'(base[32]));
    lw(BR(n, 'h18), len);
    lw(BR(n, 'h1C), 32'(bb));
  endtask
  task automatic bist_wait(input int n, output logic [31:0] st, input int max_polls = 20000);
    int p;
    p = 0;
    do begin
      repeat (20) @(posedge clk_l2);
      lr(BR(n, 'h04), st);
      p++;
    end while (!((st & 32'h6) != 0 && st[0] == 1'b0) && p < max_polls);
    if (p >= max_polls) fail($sformatf("BIST%0d wait timeout, status %h", n, st));
  endtask
  task automatic bist_run(input int n, input int mode, input logic [31:0] seed, input longint unsigned base,
                          input logic [31:0] len, input int bb, output logic [31:0] st);
    bist_prog(n, mode, seed, base, len, bb);
    lw(BR(n, 'h00), 32'h1);
    bist_wait(n, st);
  endtask
  task automatic bist_expect_clean(input int n, input string what);
    logic [31:0] st, e, a, cw, cr;
    lr(BR(n, 'h04), st);
    lr(BR(n, 'h20), e);
    lr(BR(n, 'h24), a);
    chk(st[4:0] == 5'b00010 && e == 0 && a == 0,
        $sformatf("%s C%0d: status %h errcnt %0d axierr %0d", what, n + 1, st, e, a));
  endtask
  // memory holds pattern pat/seed on [base, base+len)
  task automatic mem_expect_pat(input int n, input int pat, input logic [31:0] seed, input longint unsigned base,
                                input longint unsigned len, input string what);
    int bad;
    bad = 0;
    for (longint unsigned a = base; a < base + len; a += 8) begin
      logic [63:0] g, w;
      g = ch_peek64(n, a);
      w = pat_word(pat, seed, n, a);
      if (g !== w) begin
        bad++;
        if (bad <= 3) fail($sformatf("%s C%0d mem @%h: %h want %h", what, n + 1, a, g, w));
      end
    end
    if (bad > 3) fail($sformatf("%s C%0d: %0d bad words", what, n + 1, bad));
  endtask

  // ------------------------------------------------------------------ DMA helpers
  task automatic dma_run(input int src_sel, input int dst_sel, input logic [46:0] src, input logic [46:0] dst,
                         input logic [31:0] len, input logic [31:0] ctrl_extra, output logic [31:0] st);
    int p;
    lw(DR('h00), 32'h2 | ctrl_extra);
    lw(DR('h04), 32'hE);
    lw(DR('h08), 32'(src_sel));
    lw(DR('h0C), 32'(dst_sel));
    lw(DR('h10), src[31:0]);
    lw(DR('h14), 32'(src[46:32]));
    lw(DR('h18), dst[31:0]);
    lw(DR('h1C), 32'(dst[46:32]));
    lw(DR('h20), len);
    lw(DR('h00), 32'h1 | ctrl_extra);
    p = 0;
    do begin
      repeat (20) @(posedge clk_l2);
      lr(DR('h04), st);
      p++;
    end while (!((st & 32'h6) != 0 && st[0] == 1'b0) && p < 20000);
    if (p >= 20000) fail($sformatf("DMA wait timeout, status %h", st));
  endtask

  // ------------------------------------------------------------------ always-on checks
  logic [1:0] q_req_d;   // the ack may lag the request by two cycles (input flop + ack flop)
  always @(posedge clk_l2) begin
    q_req_d <= {q_req_d[0], shell_quiesce_req};
    if (rst_l2_n) begin
      if (rm_irq[3:1] != 3'b000) fail("rm_irq[3:1] != 0");
      if (!shell_quiesce_req && q_req_d == 2'b00 && rm_quiesce_ack) fail("rm_quiesce_ack without request");
    end
  end

  // ================================================================== tests
  task automatic t_ident();
    logic [31:0] d;
    $display("TEST ident");
    chk(rm_ready === 1'b1, "rm_ready not 1 after reset");
    chk(rm_info[31:0] == 32'h4D52_474E, "rm_info magic");
    chk(rm_info[47:32] == 16'h0001, "rm_info iface_version");
    chk(rm_info[63:48] == 16'h8001, "rm_info rm_kind");
    chk(rm_info[95:64] == BUILD_ID, "rm_info build_id");
    chk(rm_info[111:96] == RM_REV, "rm_info rm_rev");
    chk(rm_info[115:112] == 4'hF, "rm_info mem_mask");
    chk(rm_info[127:116] == 12'h0, "rm_info reserved");
    lr_exp(20'h00000, 32'h5550_474E, "ID");
    lr_exp(20'h00004, {16'h0000, RM_REV}, "REV");
    lr_exp(20'h00008, 32'h0, "CAPS");
    lw(20'h00010, 32'hA5A5_1234);
    lr_exp(20'h00010, 32'hA5A5_1234, "SCRATCH");
    lw(20'h00010, 32'h0000_00FF, 4'b0001);
    lr_exp(20'h00010, 32'hA5A5_12FF, "SCRATCH byte strobe");
    lr_exp(20'h08000, 32'h3154_5354, "T_ID");
    lr_exp(20'h08004, 32'h0001_0000, "T_VERSION");
    lr_exp(20'h08008, 32'h0000_000F, "T_MEM_READY");
    shell_mem_ready <= 4'h5;
    repeat (4) @(posedge clk_l2);
    lr_exp(20'h08008, 32'h0000_0005, "T_MEM_READY follows input");
    shell_mem_ready <= 4'hF;
    lr_exp(20'h0800C, 32'h0, "T_STATUS");
    lr_exp(20'h08030, 32'h0, "T_FREQ_L2");
    for (int n = 0; n < 4; n++) begin
      lr_exp(20'h08010 + 20'(4 * n), 32'h0, "OWNER reset");
      lr_exp(20'h08020 + 20'(4 * n), 32'h0, "WIN_SEL reset");
      lw(20'h08020 + 20'(4 * n), 32'hFFFF_FFFF);
      lr_exp(20'h08020 + 20'(4 * n), (32'h1 << ch_winbits(n)) - 1, "WIN_SEL truncation");
      lw(20'h08020 + 20'(4 * n), 32'h0);
      lr_exp(BR(n, 'h1C), 32'd16, "BURST_BEATS default");
      lr_exp(BR(n, 'h04), 32'h0, "BIST STATUS reset");
      lr_exp(20'h08500 + 20'(4 * n), 32'h0, "FIRST_EXP_HI reset");
    end
    lr_exp(DR('h04), 32'h0, "DMA STATUS reset");
    lr_exp(20'h00020, 32'h0, "undefined 0x20");
    lr_exp(20'h08034, 32'h0, "undefined 0x8034");
    lr_exp(20'h08630, 32'h0, "undefined 0x8630");
    lr_exp(20'hFF000, 32'h0, "undefined 0xFF000 (no blank test page)");
    lw(20'hFF000, 32'h7);
    lr_exp(20'h00040, 32'h0, "0xFF000 write does not raise IRQ");
    lw(20'h08000, 32'h1234);
    lr_exp(20'h08000, 32'h3154_5354, "T_ID read-only");
  endtask

  // random BAR2 traffic on channel n inside [rbase, rbase + rsize)
  task automatic t_bar2_chan(input int n, input longint unsigned rbase, input longint unsigned rsize, input int nb);
    typedef struct { longint unsigned off; int beats; } rg_t;
    rg_t regs [$];
    for (int i = 0; i < nb; i++) begin
      longint unsigned off, a8;
      int beats, maxb, lsel;
      logic [63:0] d [$];
      logic [7:0]  s [$];
      off  = rbase + $urandom_range(0, 32'(rsize - 8));
      a8   = off & ~64'h7;
      maxb = (4096 - (a8 % 4096)) / 8;
      lsel = $urandom_range(0, 9);
      beats = (lsel == 0) ? 1 : (lsel == 1) ? 256 : (lsel == 2) ? 64 : $urandom_range(1, 40);
      if (i == 0) begin off = rbase + 3; a8 = rbase; beats = 256; maxb = (4096 - (a8 % 4096)) / 8; end
      if (beats > maxb) beats = maxb;
      if (a8 + 8 * beats > rbase + rsize) beats = (rbase + rsize - a8) / 8;
      if (beats < 1) beats = 1;
      for (int b = 0; b < beats; b++) begin
        int m;
        d.push_back({$urandom, $urandom});
        m = $urandom_range(0, 5);
        s.push_back((m == 0) ? 8'h0F : (m == 1) ? 8'hF0 : (m == 2) ? 8'(1 << $urandom_range(0, 7)) :
                    (m == 3) ? 8'($urandom) : 8'hFF);
      end
      // unaligned start: leading bytes below off are not written by a real host;
      // model the typical PCIe first-DW strobe
      if (off[2:0] != 3'b000) s[0] = s[0] & (8'hFF << off[2:0]);
      b2_write(n, off, beats, 3'd3, d, s, 2'b00, 1'b1, 4'(i));
      regs.push_back('{a8, beats});
    end
    b2_drain($sformatf("bar2 C%0d write", n + 1));
    // memory must equal the golden bytes
    foreach (regs[r]) begin
      for (int b = 0; b < regs[r].beats; b++) begin
        longint unsigned ca;
        logic [63:0] g, w;
        ca = chaddr(n, regs[r].off + 8 * b);
        g  = ch_peek64(n, ca);
        for (int k = 0; k < 8; k++) w[8*k +: 8] = gold_get(n, ca + k);
        chk(g === w, $sformatf("bar2 C%0d mem @%h = %h want %h", n + 1, ca, g, w));
      end
    end
    // read back: the written regions plus random reads
    foreach (regs[r]) b2_read(n, regs[r].off + $urandom_range(0, 7), regs[r].beats, 3'd3, 0, 4'(r));
    for (int i = 0; i < nb / 2; i++) begin
      longint unsigned off, a8;
      int beats;
      off   = rbase + $urandom_range(0, 32'(rsize - 8));
      a8    = off & ~64'h7;
      beats = $urandom_range(1, 256);
      if (beats > (4096 - (a8 % 4096)) / 8) beats = (4096 - (a8 % 4096)) / 8;
      if (a8 + 8 * beats > rbase + rsize) beats = (rbase + rsize - a8) / 8;
      if (beats < 1) beats = 1;
      b2_read(n, off, beats, 3'd3, 0, 4'(i + 7));
    end
    b2_drain($sformatf("bar2 C%0d read", n + 1));
  endtask

  task automatic t_bar2();
    int nb;
    $display("TEST bar2");
    nb = quick ? 8 : 24;
    for (int n = 0; n < 4; n++) begin
      set_win(n, 0);
      t_bar2_chan(n, 64'h0010_0000, 64'h4000, nb);
      set_win(n, 32'hFFFF_FFFF);              // top slice (sliding window)
      t_bar2_chan(n, 64'h03FF_C000, 64'h4000, nb / 2);
      set_win(n, 3);
      t_bar2_chan(n, 64'h0000_0000, 64'h2000, nb / 2);
      set_win(n, 0);
    end
    // explicit window mapping check on C1: top slice + offset lands at {127, off}
    begin
      logic [63:0] d [$];
      logic [7:0]  s [$];
      d.push_back(64'h0123_4567_89AB_CDEF); s.push_back(8'hFF);
      set_win(0, 127);
      b2_write(0, 26'h000_1238, 1, 3'd3, d, s, 2'b00, 1'b1);
      b2_drain("window map");
      chk(ch_peek64(0, 64'h1_FC00_1238) == 64'h0123_4567_89AB_CDEF, "C1 WIN_SEL=127 mapping");
      set_win(0, 0);
    end
  endtask

  task automatic t_bar2err();
    logic [63:0] d [$];
    logic [7:0]  s [$];
    logic [63:0] mem_before;
    int naw0;
    $display("TEST bar2err");
    for (int i = 0; i < 4; i++) begin d.push_back({$urandom, $urandom}); s.push_back(8'hFF); end
    // narrow AxSIZE on an owned channel: served as plain INCR (also across 8-byte words)
    b2_write_n(1, 26'h020_0004, 6, 3'd2, 4'h3);
    b2_write_n(1, 26'h020_0102, 8, 3'd1, 4'h5);
    b2_write_n(1, 26'h020_0203, 9, 3'd0, 4'h6);
    b2_drain("narrow w");
    b2_read_n(1, 26'h020_0004, 6, 3'd2, 4'h4);
    b2_read_n(1, 26'h020_0102, 8, 3'd1, 4'h7);
    b2_read_n(1, 26'h020_0203, 9, 3'd0, 4'h8);
    b2_drain("narrow");
    // AxSIZE > 3 cannot occur on a 64-bit port: SLVERR, channel untouched
    naw0 = u_c2.n_aw + u_c2.n_ar;
    mem_before = ch_peek64(1, 64'h0020_0000);
    b2_write(1, 26'h020_0000, 4, 3'd4, d, s, 2'b10, 1'b0, 4'h3);
    b2_read(1, 26'h020_0000, 3, 3'd5, 1, 4'h4);
    b2_drain("bad size");
    chk(ch_peek64(1, 64'h0020_0000) == mem_before, "bad-size write modified memory");
    chk(u_c2.n_aw + u_c2.n_ar == naw0, "bad-size access reached the channel");
    // non-owned channel C2 (OWNER=BIST) pipelined with owned C1 traffic
    set_owner(1, 1);
    naw0 = u_c2.n_aw + u_c2.n_ar;
    for (int i = 0; i < 6; i++) begin
      b2_read(0, 26'h010_0000 + 64 * i, 4 + i, 3'd3, 0, 4'(2 * i));
      b2_read(1, 26'h010_0000 + 64 * i, 3 + i, 3'd3, 1, 4'(2 * i + 1));
      b2_write(1, 26'h010_0000 + 64 * i, 4, 3'd3, d, s, 2'b10, 1'b0, 4'(i));
      b2_write(0, 26'h011_0000 + 64 * i, 4, 3'd3, d, s, 2'b00, 1'b1, 4'(i + 8));
    end
    b2_drain("non-owned");
    chk(u_c2.n_aw + u_c2.n_ar == naw0, "non-owned BAR2 access reached the channel");
    set_owner(1, 0);
    // DMA-owned channel also answers locally
    set_owner(3, 2);
    naw0 = u_c4.n_aw + u_c4.n_ar;
    b2_write(3, 26'h000_0100, 4, 3'd3, d, s, 2'b10, 1'b0);
    b2_read(3, 26'h000_0100, 2, 3'd3, 1);
    b2_drain("dma-owned");
    chk(u_c4.n_aw + u_c4.n_ar == naw0, "DMA-owned BAR2 access reached the channel");
    set_owner(3, 0);
    // channel SLVERR: C3 range [0x30_0040, 0x30_0080) = beats 8..15 of a 16-beat read from 0x30_0000
    ch_err(2, 1'b1, 64'h30_0040, 64'h30_0080);
    b2_write(2, 26'h030_0040, 4, 3'd3, d, s, 2'b10, 1'b0);
    b2_write(2, 26'h030_0100, 4, 3'd3, d, s, 2'b00, 1'b1);
    begin
      ax_t a;
      a.id = 4'h9; a.addr = {2'd2, 26'h030_0000}; a.len = 8'd15; a.size = 3'd3;
      b2ar_q.push_back(a);
      b2r_exp.push_back('{4'h9, 15});
      for (int i = 0; i < 16; i++) begin
        if (i < 8) begin
          logic [63:0] w;
          for (int b = 0; b < 8; b++) w[8*b +: 8] = gold_get(2, 64'h30_0000 + 8 * i + b);
          b2r_dexp.push_back(w);
          b2r_rexp.push_back(2'b00);
        end else begin
          b2r_dexp.push_back({2{32'hBADBAD00}});
          b2r_rexp.push_back(2'b10);
        end
      end
    end
    b2_drain("channel slverr");
    ch_err(2, 1'b0, 0, 0);
  endtask

  task automatic t_bist();
    logic [31:0] st, cw, cr, v;
    longint unsigned base, len;
    int naw0;
    $display("TEST bist");
    for (int n = 0; n < 4; n++) set_owner(n, 1);
    for (int n = 0; n < 4; n++) begin
      for (int pat = 0; pat < 2; pat++) begin
        base = 64'h0400_0000 + 64'h1_0000 * pat;          // slice 1
        if (n == 0 && pat == 1) base = 64'h1_8000_0000;    // C1: bit 32 via BASE_HI
        len  = quick ? 64'h1000 : 64'h4000;
        bist_run(n, 3 | (pat << 2), 32'h1234_5678 + n, base, 32'(len), 16, st);
        bist_expect_clean(n, $sformatf("bist pat%0d", pat));
        lr(BR(n, 'h2C), cw);
        lr(BR(n, 'h30), cr);
        chk(cw > 0 && cr > 0, $sformatf("BIST C%0d CYC_W %0d CYC_R %0d", n + 1, cw, cr));
        mem_expect_pat(n, pat, 32'h1234_5678 + n, base, len, $sformatf("bist pat%0d", pat));
      end
      // BURST_BEATS 4 and a non-power-of-2 (3) inside one 4 KiB page
      naw0 = ch_awlen_hist(n, 3);
      bist_run(n, 3, 32'hCAFE_0000 + n, 64'h0800_0000, 32'(8 * 4 * ch_bpb(n)), 4, st);
      bist_expect_clean(n, "bist bb4");
      chk(ch_awlen_hist(n, 3) == naw0 + 8, $sformatf("C%0d bb4: AWLEN=3 bursts %0d", n + 1, ch_awlen_hist(n, 3) - naw0));
      mem_expect_pat(n, 0, 32'hCAFE_0000 + n, 64'h0800_0000, 8 * 4 * ch_bpb(n), "bist bb4");
      begin
        longint unsigned bb3, b3;
        bb3 = 3 * ch_bpb(n);
        b3  = bb3 * ((4096 + bb3 - 1) / bb3);
        bist_run(n, 3, 32'h0BB3_0000, b3, 32'(4 * bb3), 3, st);
        bist_expect_clean(n, "bist bb3");
        mem_expect_pat(n, 0, 32'h0BB3_0000, b3, 4 * bb3, "bist bb3");
      end
      // CFG_ERR: no DONE, no traffic
      naw0 = ch_naw(n) + ch_nar(n);
      bist_run(n, 3, 0, 64'h0, 32'(16 * ch_bpb(n) * 2 + ch_bpb(n)), 16, st);   // LEN not multiple of BB
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR len: status %h", n + 1, st));
      bist_run(n, 3, 0, 64'(ch_bpb(n) * 3), 32'(16 * ch_bpb(n)), 16, st);        // BASE not BB aligned
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR base: status %h", n + 1, st));
      bist_run(n, 3, 0, 64'h0, 32'(ch_bpb(n) * 16), 0, st);                     // BURST_BEATS 0
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR bb0: status %h", n + 1, st));
      bist_run(n, 3, 0, 64'h0, 32'(ch_bpb(n) * 17), 17, st);                    // BURST_BEATS 17
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR bb17: status %h", n + 1, st));
      bist_run(n, 3, 0, 64'h0, 32'h0, 16, st);                                  // LEN 0
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR len0: status %h", n + 1, st));
      bist_run(n, 3, 0, 64'h0, 32'(ch_bpb(n) * 16 + 4), 16, st);                // LEN not beat multiple
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR len4: status %h", n + 1, st));
      set_owner(n, 0);
      bist_run(n, 3, 0, 64'h0, 32'(ch_bpb(n) * 16), 16, st);                    // OWNER != BIST
      chk(st[2:0] == 3'b100, $sformatf("C%0d CFG_ERR owner: status %h", n + 1, st));
      set_owner(n, 1);
      chk(ch_naw(n) + ch_nar(n) == naw0, $sformatf("C%0d CFG_ERR runs made bus traffic", n + 1));
      // W1C of CFG_ERR, ERR bit RO
      lw(BR(n, 'h04), 32'h4);
      lr_exp(BR(n, 'h04), 32'h0, "STATUS after W1C");
      // register read-back
      lw(BR(n, 'h08), 32'h5);
      lr_exp(BR(n, 'h08), 32'h5, "MODE rb");
      lw(BR(n, 'h14), 32'hFFFF_FFFF);
      lr_exp(BR(n, 'h14), 32'h1, "BASE_HI rb");
    end
    for (int n = 0; n < 4; n++) set_owner(n, 0);
  endtask

  task automatic t_bisterr();
    logic [31:0] st, v, lo, hi;
    longint unsigned base, len, a1, a2;
    int L, j1, j2, k1;
    logic [63:0] orig, bad;
    $display("TEST bisterr");
    for (int n = 0; n < 4; n++) begin
      set_owner(n, 1);
      L    = ch_bpb(n) / 8;
      base = 64'h0C00_0000;
      len  = 64'h1000;
      bist_run(n, 1, 32'h5EED_0000 + n, base, 32'(len), 16, st);   // DO_WRITE only
      bist_expect_clean(n, "bisterr write");
      // corrupt: beat j1 lanes k1 and L-1 (FIRST must pick k1), beat j2 lane 0
      j1 = 5; j2 = 9; k1 = (L > 2) ? 1 : 0;
      a1 = base + j1 * ch_bpb(n) + 8 * k1;
      orig = ch_peek64(n, a1);
      bad  = orig ^ 64'h0000_0100_0000_8001;
      ch_poke64(n, a1, bad);
      ch_poke64(n, base + j1 * ch_bpb(n) + 8 * (L - 1), ~ch_peek64(n, base + j1 * ch_bpb(n) + 8 * (L - 1)));
      a2 = base + j2 * ch_bpb(n);
      ch_poke64(n, a2, ch_peek64(n, a2) ^ 64'h1);
      bist_run(n, 2, 32'h5EED_0000 + n, base, 32'(len), 16, st);   // DO_CHECK only
      chk(st[4:0] == 5'b10010, $sformatf("bisterr C%0d status %h", n + 1, st));
      lr_exp(BR(n, 'h20), 32'd2, $sformatf("bisterr C%0d ERRCNT", n + 1));
      lr_exp(BR(n, 'h24), 32'd0, "bisterr AXI_ERRCNT");
      lr_exp(BR(n, 'h28), 32'(base + j1 * ch_bpb(n)), $sformatf("bisterr C%0d FIRST_ERR_ADDR", n + 1));
      lr_exp(BR(n, 'h34), bad[31:0], "FIRST_GOT_LO");
      lr_exp(BR(n, 'h38), bad[63:32], "FIRST_GOT_HI");
      lr_exp(BR(n, 'h3C), orig[31:0], "FIRST_EXP_LO");
      lr_exp(20'h08500 + 20'(4 * n), orig[63:32], "FIRST_EXP_HI");
      // CLEAR, then seed mismatch: every beat wrong, FIRST at base lane 0
      lr(BR(n, 'h30), v);
      chk(v > 0, "CYC_R after check");
      lw(BR(n, 'h00), 32'h2);
      lr_exp(BR(n, 'h20), 32'd0, "ERRCNT after CLEAR");
      lr_exp(BR(n, 'h30), 32'd0, "CYC_R after CLEAR");
      lr_exp(BR(n, 'h28), 32'd0, "FIRST_ERR_ADDR after CLEAR");
      lr_exp(BR(n, 'h04), 32'h0, "STATUS after CLEAR (DONE cleared, ERR 0)");
      bist_run(n, 2, 32'h5EED_0001 + n, base, 32'(len), 8, st);
      lr_exp(BR(n, 'h20), 32'(len / ch_bpb(n)), "seed mismatch ERRCNT");
      lr_exp(BR(n, 'h28), 32'(base), "seed mismatch FIRST_ERR_ADDR");
      lr_exp(BR(n, 'h3C), 32'(prbs_lane(32'h5EED_0001 + n, 32'(base / ch_bpb(n)), 0)), "seed mismatch FIRST_EXP_LO");
      lr_exp(BR(n, 'h34), 32'(prbs_lane(32'h5EED_0000 + n, 32'(base / ch_bpb(n)), 0)), "seed mismatch FIRST_GOT_LO");
      // AXI error path: bursts 2 and 3 (bb 16) fail on write and read
      lw(BR(n, 'h00), 32'h2);
      ch_err(n, 1'b1, base + 2 * 16 * ch_bpb(n), base + 4 * 16 * ch_bpb(n));
      bist_run(n, 3, 32'h5EED_0002, base, 32'(16 * 16 * ch_bpb(n)), 16, st);
      ch_err(n, 1'b0, 0, 0);
      chk(st[3:0] == 4'b1010, $sformatf("axierr C%0d status %h", n + 1, st));
      lr_exp(BR(n, 'h24), 32'd2 + 32'd32, $sformatf("axierr C%0d AXI_ERRCNT (2 B + 32 R)", n + 1));
      lr(BR(n, 'h20), v);
      chk(v == 0, $sformatf("axierr C%0d data ERRCNT %0d (error beats must not be compared)", n + 1, v));
      lw(BR(n, 'h04), 32'h8);
      lr_exp(BR(n, 'h04), 32'h2, "AXI_ERR W1C");
      set_owner(n, 0);
    end
  endtask

  task automatic t_conc();
    logic [31:0] st;
    longint unsigned len;
    $display("TEST conc");
    len = quick ? 64'h2000 : 64'h8000;
    for (int n = 0; n < 4; n++) begin
      set_owner(n, 1);
      bist_prog(n, 3, 32'hC0C0_0000 + n, 64'h1000_0000, 32'(len), 16);
    end
    for (int n = 0; n < 4; n++) lw(BR(n, 'h00), 32'h1);
    for (int n = 0; n < 4; n++) begin
      bist_wait(n, st);
      bist_expect_clean(n, "conc");
      mem_expect_pat(n, 0, 32'hC0C0_0000 + n, 64'h1000_0000, len, "conc");
    end
    $display("INFO conc: max outstanding W/R bursts C1 %0d/%0d C2 %0d/%0d C3 %0d/%0d C4 %0d/%0d",
             u_c1.max_wr_out, u_c1.max_rd_out, u_c2.max_wr_out, u_c2.max_rd_out,
             u_c3.max_wr_out, u_c3.max_rd_out, u_c4.max_wr_out, u_c4.max_rd_out);
    chk(u_c1.max_wr_out <= 8 && u_c1.max_rd_out <= 16, "BIST outstanding limits");
    for (int n = 0; n < 4; n++) set_owner(n, 0);
  endtask

  task automatic t_cross();
    logic [31:0] st;
    $display("TEST cross");
    for (int n = 0; n < 4; n++) begin
      longint unsigned off, len, pos;
      logic [31:0] seed;
      seed = 32'hA5A5_A5A5 ^ n;
      set_win(n, 2);
      off  = 64'h0004_0000;
      len  = 64'h1000;
      // (a) host writes PRBS through BAR2 in random-size pieces, BIST DO_CHECK
      pos = 0;
      while (pos < len) begin
        int beats;
        logic [63:0] d [$];
        logic [7:0]  s [$];
        beats = $urandom_range(1, 64);
        if (pos + 8 * beats > len) beats = (len - pos) / 8;
        for (int b = 0; b < beats; b++) begin
          d.push_back(pat_word(0, seed, n, chaddr(n, off + pos + 8 * b)));
          s.push_back(8'hFF);
        end
        b2_write(n, off + pos, beats, 3'd3, d, s, 2'b00, 1'b1);
        pos += 8 * beats;
      end
      b2_drain("cross a");
      set_owner(n, 1);
      bist_run(n, 2, seed, chaddr(n, off), 32'(len), 16, st);
      bist_expect_clean(n, "cross a (BAR2 write -> BIST check)");
      // (b) BIST DO_WRITE both patterns, host reads back through BAR2
      for (int pat = 0; pat < 2; pat++) begin
        bist_run(n, 1 | (pat << 2), seed + 1, chaddr(n, off + len), 32'(len), 16, st);
        bist_expect_clean(n, "cross b write");
        set_owner(n, 0);
        pos = 0;
        while (pos < len) begin
          int beats;
          ax_t a;
          beats = $urandom_range(1, 128);
          if (pos + 8 * beats > len) beats = (len - pos) / 8;
          a.id = 4'(pos); a.addr = {2'(n), 26'(off + len + pos)}; a.len = 8'(beats - 1); a.size = 3'd3;
          b2ar_q.push_back(a);
          b2r_exp.push_back('{4'(pos), beats - 1});
          for (int b = 0; b < beats; b++) begin
            b2r_dexp.push_back(pat_word(pat, seed + 1, n, chaddr(n, off + len + pos + 8 * b)));
            b2r_rexp.push_back(2'b00);
          end
          pos += 8 * beats;
        end
        b2_drain("cross b");
        set_owner(n, 1);
      end
      set_owner(n, 0);
      set_win(n, 0);
      // gold is stale for the BIST-written range: forget it
      gold.delete();
    end
  endtask

  task automatic t_dma();
    logic [31:0] st, v;
    logic [46:0] hsrc, hdst;
    longint unsigned caddr;
    int nblk, h63, c15, c7;
    $display("TEST dma");
    for (int n = 0; n < 4; n++) set_owner(n, 2);
    for (int n = 0; n < 4; n++) begin
      nblk  = (n == 2) ? 4 : 3;
      hsrc  = 47'h4000_0000_0000 + 47'(n) * 47'h1_0000;
      hdst  = 47'h7FFF_0000_0000 + 47'(n) * 47'h1_0000;
      caddr = (n == 0) ? 64'h1_0020_0000 : 64'h0020_0000 + 64'(n) * 64'h1000;
      for (int i = 0; i < nblk * 64; i++) u_host.poke((hsrc >> 3) + i, {$urandom, $urandom});
      h63 = u_host.ar_len_hist[63];
      c15 = ch_awlen_hist(n, 15);
      c7  = ch_awlen_hist(n, 7);
      dma_run(0, n + 1, hsrc, 47'(caddr), 32'(512 * nblk), 32'h0, st);
      chk(st[3:0] == 4'b0010, $sformatf("dma h->C%0d status %h", n + 1, st));
      lr_exp(DR('h2C), 32'(nblk), "dma BLOCKS_DONE");
      lr(DR('h24), v);
      chk(v > 0, "dma CYCLES");
      for (int i = 0; i < nblk * 64; i++)
        chk(ch_peek64(n, caddr + 8 * i) == u_host.peek((hsrc >> 3) + i),
            $sformatf("dma h->C%0d word %0d: %h want %h", n + 1, i, ch_peek64(n, caddr + 8 * i), u_host.peek((hsrc >> 3) + i)));
      chk(u_host.ar_len_hist[63] == h63 + nblk, "dma host AR count/len");
      if (n == 0) chk(ch_awlen_hist(0, 7) == c7 + nblk, "dma C1 burst shape (1x8 beats)");
      else if (n == 2) chk(ch_awlen_hist(2, 15) == c15 + 2 * nblk, "dma C3 burst shape (2x16 beats)");
      else chk(ch_awlen_hist(n, 15) == c15 + nblk, $sformatf("dma C%0d burst shape (1x16 beats)", n + 1));
      // channel -> host
      h63 = u_host.aw_len_hist[63];
      dma_run(n + 1, 0, 47'(caddr), hdst, 32'(512 * nblk), 32'h0, st);
      chk(st[3:0] == 4'b0010, $sformatf("dma C%0d->h status %h", n + 1, st));
      for (int i = 0; i < nblk * 64; i++)
        chk(u_host.peek((hdst >> 3) + i) == u_host.peek((hsrc >> 3) + i),
            $sformatf("dma C%0d->h word %0d: %h want %h", n + 1, i, u_host.peek((hdst >> 3) + i), u_host.peek((hsrc >> 3) + i)));
      chk(u_host.aw_len_hist[63] == h63 + nblk, "dma host AW count/len");
      if (n == 2) chk(ch_arlen_hist(2, 15) >= 2 * nblk, "dma C3 read burst shape");
    end
    // CFG_ERR cases
    begin
      int h0;
      h0 = u_host.n_aw + u_host.n_ar;
      dma_run(0, 0, 47'h0, 47'h1000, 32'd512, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma host->host CFG_ERR: %h", st));
      dma_run(1, 2, 47'h0, 47'h1000, 32'd512, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma C1->C2 CFG_ERR: %h", st));
      dma_run(0, 5, 47'h0, 47'h1000, 32'd512, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma sel 5 CFG_ERR: %h", st));
      dma_run(0, 1, 47'h0, 47'h1000, 32'd0, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma LEN 0 CFG_ERR: %h", st));
      dma_run(0, 1, 47'h0, 47'h1000, 32'd100, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma LEN 100 CFG_ERR: %h", st));
      dma_run(0, 1, 47'h40, 47'h1000, 32'd512, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma SRC misaligned CFG_ERR: %h", st));
      dma_run(0, 1, 47'h0, 47'h1100, 32'd512, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma DST misaligned CFG_ERR: %h", st));
      set_owner(0, 1);
      dma_run(0, 1, 47'h0, 47'h1000, 32'd512, 32'h0, st);
      chk(st[3:0] == 4'b0100, $sformatf("dma OWNER!=DMA CFG_ERR: %h", st));
      set_owner(0, 2);
      chk(u_host.n_aw + u_host.n_ar == h0, "dma CFG_ERR made host traffic");
    end
    // error blocks: host read error on block 1 of 3
    u_host.err_en = 1'b1; u_host.err_lo = 64'h5000_0000_0200; u_host.err_hi = 64'h5000_0000_0400;
    dma_run(0, 2, 47'h5000_0000_0000, 47'h0040_0000, 32'd1536, 32'h0, st);
    chk(st[3:0] == 4'b1010, $sformatf("dma err continue status %h", st));
    lr_exp(DR('h28), 32'd1, "dma AXI_ERRCNT (blocks)");
    lr_exp(DR('h2C), 32'd3, "dma continue BLOCKS_DONE");
    dma_run(0, 2, 47'h5000_0000_0000, 47'h0040_0000, 32'd1536, 32'h8, st);
    chk(st[3:0] == 4'b1010, $sformatf("dma STOP_ON_ERR status %h", st));
    lr_exp(DR('h2C), 32'd2, "dma STOP_ON_ERR BLOCKS_DONE");
    lr_exp(DR('h00), 32'h8, "dma CTRL readback (STOP_ON_ERR)");
    u_host.err_en = 1'b0;
    // channel-side write error (C4) and host write error
    ch_err(3, 1'b1, 64'h0050_0000, 64'h0050_0200);
    dma_run(0, 4, 47'h4000_0000_0000, 47'h0050_0000, 32'd1024, 32'h0, st);
    chk(st[3:0] == 4'b1010, $sformatf("dma channel write err status %h", st));
    lr_exp(DR('h28), 32'd1, "dma channel err AXI_ERRCNT");
    ch_err(3, 1'b0, 0, 0);
    u_host.err_en = 1'b1; u_host.err_lo = 64'h6000_0000_0000; u_host.err_hi = 64'h6000_0000_0200;
    dma_run(4, 0, 47'h0050_0200, 47'h6000_0000_0000, 32'd512, 32'h0, st);
    chk(st[3:0] == 4'b1010, $sformatf("dma host write err status %h", st));
    u_host.err_en = 1'b0;
    lw(DR('h00), 32'h2);
    lr_exp(DR('h28), 32'd0, "dma CLEAR AXI_ERRCNT");
    lr_exp(DR('h2C), 32'd0, "dma CLEAR BLOCKS_DONE");
    for (int n = 0; n < 4; n++) set_owner(n, 0);
  endtask

  task automatic t_irq();
    logic [31:0] st;
    $display("TEST irq");
    lw(20'h00040, 32'hFFFF_FFFF);
    lw(20'h00044, 32'hFFFF_FFFF);
    lr_exp(20'h00044, 32'h7, "IRQ_ENABLE mask");
    lw(20'h00044, 32'h0);
    set_owner(2, 1);
    bist_run(2, 1, 32'h1, 64'h0, 32'h400, 16, st);
    lr_exp(20'h00040, 32'h1, "IRQ_STATUS after BIST");
    repeat (4) @(posedge clk_l2);
    chk(rm_irq == 4'h0, "rm_irq with IRQ_ENABLE=0");
    lw(20'h00044, 32'h1);
    repeat (4) @(posedge clk_l2);
    chk(rm_irq == 4'h1, "rm_irq with IRQ_ENABLE=1");
    lw(20'h00040, 32'h1);
    repeat (4) @(posedge clk_l2);
    chk(rm_irq == 4'h0, "rm_irq after W1C");
    lr_exp(20'h00040, 32'h0, "IRQ_STATUS after W1C");
    set_owner(2, 2);
    dma_run(0, 3, 47'h0, 47'h0, 32'd512, 32'h0, st);
    repeat (4) @(posedge clk_l2);
    lr_exp(20'h00040, 32'h0, "DMA without IRQ_EN leaves IRQ_STATUS");
    dma_run(0, 3, 47'h0, 47'h0, 32'd512, 32'h4, st);
    lr_exp(DR('h00), 32'h4, "DMA CTRL IRQ_EN readback");
    lr_exp(20'h00040, 32'h1, "IRQ_STATUS after DMA with IRQ_EN");
    repeat (4) @(posedge clk_l2);
    chk(rm_irq == 4'h1, "rm_irq after DMA");
    lw(20'h00040, 32'h1);
    lw(20'h00044, 32'h0);
    lw(DR('h00), 32'h0);
    set_owner(2, 0);
  endtask

  task automatic t_owner();
    logic [31:0] st;
    $display("TEST owner");
    // BIST busy
    set_owner(1, 1);
    bist_prog(1, 3, 32'h77, 64'h0, 32'h8000, 16);
    lw(BR(1, 'h00), 32'h1);
    set_owner(1, 0, 1'b0);
    lr_exp(20'h08014, 32'h1, "OWNER unchanged after refused write");
    bist_wait(1, st);
    bist_expect_clean(1, "owner bist");
    set_owner(1, 0);
    // DMA busy on C3
    set_owner(2, 2);
    lw(DR('h00), 32'h2);
    lw(DR('h08), 32'h0); lw(DR('h0C), 32'h3);
    lw(DR('h10), 32'h0); lw(DR('h14), 32'h0); lw(DR('h18), 32'h0); lw(DR('h1C), 32'h0);
    lw(DR('h20), 32'h4000);
    lw(DR('h00), 32'h1);
    set_owner(2, 0, 1'b0);
    lr_exp(20'h08018, 32'h2, "OWNER C3 unchanged during DMA");
    do begin repeat (50) @(posedge clk_l2); lr(DR('h04), st); end while (st[0]);
    chk(st[3:0] == 4'b0010, $sformatf("owner dma status %h", st));
    set_owner(2, 0);
    // BAR2 read in progress on C4 (slow slave)
    u_c4.lat_min = 1500; u_c4.lat_max = 1600;
    b2_read(3, 26'h000_0000, 64, 3'd3, 0);
    repeat (100) @(posedge clk_l2);
    set_owner(3, 1, 1'b0);
    lr_exp(20'h0801C, 32'h0, "OWNER C4 unchanged during BAR2 read");
    b2_drain("owner bar2");
    u_c4.lat_min = 2; u_c4.lat_max = 30;
  endtask

  task automatic t_quiesce();
    logic [31:0] st, d;
    int c, aw0;
    bit acked;
    $display("TEST quiesce");
    set_owner(0, 1); set_owner(1, 1); set_owner(2, 2); set_owner(3, 0);
    for (int i = 0; i < 64 * 16; i++) u_host.poke((64'h4100_0000_0000 >> 3) + i, {$urandom, $urandom});
    bist_prog(0, 3, 32'h9001, 64'h0, 32'h10000, 16);
    bist_prog(1, 3, 32'h9002, 64'h0, 32'h10000, 16);
    lw(BR(0, 'h00), 32'h1);
    lw(BR(1, 'h00), 32'h1);
    // DMA host -> C3, 16 blocks, then BAR2 traffic to C4
    lw(DR('h00), 32'h2);
    lw(DR('h08), 32'h0); lw(DR('h0C), 32'h3);
    lw(DR('h10), 32'h0); lw(DR('h14), 32'h4100); lw(DR('h18), 32'h0010_0000); lw(DR('h1C), 32'h0);
    lw(DR('h20), 32'h2000);
    lw(DR('h00), 32'h1);
    for (int i = 0; i < 6; i++) begin
      logic [63:0] dd [$];
      logic [7:0]  ss [$];
      for (int b = 0; b < 32; b++) begin dd.push_back({$urandom, $urandom}); ss.push_back(8'hFF); end
      b2_write(3, 26'h000_4000 + 256 * i, 32, 3'd3, dd, ss, 2'b00, 1'b1);
      b2_read(3, 26'h000_2000 + 256 * i, 32, 3'd3, 0);
    end
    repeat (300) @(posedge clk_l2);
    // request quiesce while everything is running
    shell_quiesce_req <= 1'b1;
    c = 0;
    while (!rm_quiesce_ack && c < 20000) begin @(posedge clk_l2); c++; end
    chk(rm_quiesce_ack, "no rm_quiesce_ack");
    chk(all_slaves_idle(), "rm_quiesce_ack while master transactions outstanding");
    $display("INFO quiesce: ack after %0d cycles", c);
    u_c1.freeze = 1; u_c2.freeze = 1; u_c3.freeze = 1; u_c4.freeze = 1; u_host.freeze = 1;
    // slaves keep answering during quiesce (BAR0)
    lr(BR(0, 'h04), st);
    chk(st[0] == 1'b1, "BIST0 should still be busy (paused) during quiesce");
    lr_exp(20'h00000, 32'h5550_474E, "ID during quiesce");
    lr_exp(DR('h04), 32'h1, "DMA busy (paused) during quiesce");
    repeat (2000) @(posedge clk_l2);
    chk(rm_quiesce_ack, "rm_quiesce_ack dropped during quiesce");
    // release: everything resumes and completes
    u_c1.freeze = 0; u_c2.freeze = 0; u_c3.freeze = 0; u_c4.freeze = 0; u_host.freeze = 0;
    shell_quiesce_req <= 1'b0;
    repeat (3) @(posedge clk_l2);
    chk(!rm_quiesce_ack, "rm_quiesce_ack after release");
    b2_drain("quiesce release");
    bist_wait(0, st);
    bist_expect_clean(0, "quiesce bist");
    mem_expect_pat(0, 0, 32'h9001, 64'h0, 64'h10000, "quiesce bist");
    bist_wait(1, st);
    bist_expect_clean(1, "quiesce bist");
    do begin repeat (50) @(posedge clk_l2); lr(DR('h04), st); end while (st[0]);
    chk(st[3:0] == 4'b0010, $sformatf("quiesce dma status %h", st));
    for (int i = 0; i < 16 * 64; i++)
      chk(ch_peek64(2, 64'h0010_0000 + 8 * i) == u_host.peek((64'h4100_0000_0000 >> 3) + i),
          $sformatf("quiesce dma word %0d", i));
    // phase B (idle RM): BAR2 to a non-owned channel is answered during quiesce,
    // BAR2 write to an owned channel is accepted but deferred until release
    shell_quiesce_req <= 1'b1;
    c = 0;
    while (!rm_quiesce_ack && c < 20000) begin @(posedge clk_l2); c++; end
    chk(rm_quiesce_ack, "no rm_quiesce_ack (phase B)");
    u_c1.freeze = 1; u_c2.freeze = 1; u_c3.freeze = 1; u_c4.freeze = 1; u_host.freeze = 1;
    b2_read(0, 26'h000_0000, 4, 3'd3, 1);    // C1 owned by BIST -> local SLVERR
    begin
      logic [63:0] dd [$];
      logic [7:0]  ss [$];
      dd.push_back(64'h1); ss.push_back(8'hFF);
      dd.push_back(64'h2); ss.push_back(8'hFF);
      b2_write(0, 26'h000_0000, 2, 3'd3, dd, ss, 2'b10, 1'b0);
      b2_drain("quiesce non-owned");
      aw0 = b2_aw_acc;
      dd.delete(); ss.delete();
      dd.push_back(64'hFEED_FACE_0000_0001); ss.push_back(8'hFF);
      b2_write(3, 26'h000_8000, 1, 3'd3, dd, ss, 2'b00, 1'b1);
    end
    repeat (2000) @(posedge clk_l2);
    chk(rm_quiesce_ack, "rm_quiesce_ack dropped during quiesce (phase B)");
    chk(b2_aw_acc > aw0, "BAR2 AW not accepted during quiesce");
    chk(b2b_exp.size() == 1, "BAR2 write completed during quiesce (should be deferred)");
    u_c1.freeze = 0; u_c2.freeze = 0; u_c3.freeze = 0; u_c4.freeze = 0; u_host.freeze = 0;
    shell_quiesce_req <= 1'b0;
    b2_drain("quiesce B release");
    chk(ch_peek64(3, 64'h8000) == 64'hFEED_FACE_0000_0001, "deferred BAR2 write data");
    for (int n = 0; n < 4; n++) set_owner(n, 0);
    // quiesce while idle: immediate ack
    shell_quiesce_req <= 1'b1;
    c = 0;
    while (!rm_quiesce_ack && c < 200) begin @(posedge clk_l2); c++; end
    chk(rm_quiesce_ack, "idle quiesce ack");
    shell_quiesce_req <= 1'b0;
    repeat (4) @(posedge clk_l2);
  endtask

  // ================================================================== main
  initial begin
    int tot;
    if ($test$plusargs("quick")) quick = 1'b1;
    rst_l2_n = 1'b0;
    shell_mem_ready = 4'hF;
    shell_quiesce_req = 1'b0;
    s_axil_ctrl_awvalid = 1'b0; s_axil_ctrl_wvalid = 1'b0; s_axil_ctrl_arvalid = 1'b0;
    s_axil_ctrl_bready = 1'b0; s_axil_ctrl_rready = 1'b0;
    // model self-check against tools/p0test/rmtest_model.h pinned vectors
    chk(mix64(64'h1) == 64'h5692_161d_100b_05e5, "model mix64(1)");
    chk(mix64(64'h9E37_79B9_7F4A_7C15) == 64'he220_a839_7b1d_cdaf, "model mix64(golden)");
    chk(prbs_lane(32'h0, 32'h0, 0) == 64'h8b32_c408_e8c2_c97c, "model lane(0,0,0)");
    chk(prbs_lane(32'h1234_5678, 32'hDEAD_BEEF, 3) == 64'h93d7_fd64_15e1_ad75, "model lane(12345678,DEADBEEF,3)");
    chk(pat_word(0, 32'hA5A5_A5A5, 0, 64'h1_FC00_0048) == 64'h576c_fe7e_5761_1dbd, "model C1 word");
    chk(pat_word(0, 32'hA5A5_A5A5, 2, 64'h7FF_FFF8) == 64'h58ce_fdbb_bd93_d9d1, "model C3 word");
    chk(pat_word(1, 32'h1234, 0, 3 * 64 + 8) == {16'hA5A5 ^ 16'h1234, 16'd3, 32'd192}, "model PAT1");
    repeat (20) @(posedge clk_l2);
    rst_l2_n = 1'b1;
    repeat (20) @(posedge clk_l2);
    t_ident();
    t_bar2();
    t_bar2err();
    t_bist();
    t_bisterr();
    t_conc();
    t_cross();
    t_dma();
    t_irq();
    t_owner();
    t_quiesce();
    repeat (100) @(posedge clk_l2);
    tot = errors + slave_errors();
    $display("SUMMARY tb_errors %0d slave_errors %0d (c1 %0d c2 %0d c3 %0d c4 %0d host %0d)", errors, slave_errors(),
             u_c1.errors, u_c2.errors, u_c3.errors, u_c4.errors, u_host.errors);
    $display("SUMMARY BAR2 B %0d R beats %0d; channel AW/AR C1 %0d/%0d C2 %0d/%0d C3 %0d/%0d C4 %0d/%0d host %0d/%0d",
             b2_nb, b2_nr, u_c1.n_aw, u_c1.n_ar, u_c2.n_aw, u_c2.n_ar, u_c3.n_aw, u_c3.n_ar,
             u_c4.n_aw, u_c4.n_ar, u_host.n_aw, u_host.n_ar);
    if (tot == 0) $display("NG_RM_TEST_TB_PASS");
    else          $display("NG_RM_TEST_TB_FAIL %0d", tot);
    $finish;
  end

  initial begin
    #(400ms);
    $display("FAIL global timeout");
    $display("NG_RM_TEST_TB_FAIL timeout");
    $finish;
  end
endmodule
