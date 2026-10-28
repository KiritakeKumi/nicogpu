// ng_board_top.sv -- XE310 board top for the P0-11 static shell.
//
// Everything device-specific lives here and in the ng_prim_* wrappers; the
// DFX boundary (ng_shell_static_top + the generated boundary) stays
// portable.  Contents:
//   * STARTUPE3 CFGMCLK -> si_init (SI5381 register replay; the board has
//     no other free-running fabric clock before SI5381 is programmed)
//   * IBUFDS_GTE4 on the PCIe 100 MHz reference (AG9) feeding XDMA sys_clk
//     and sys_clk_gt (Gen3 x1, PCIE40E4_X1Y2 / GTY quad 227)
//   * XDMA AXI Bridge: BAR0/BAR1/BAR2 through pciebar2axibar windows into
//     the bar router (ordering fence + clock-stop local response), then
//     clock converters into the DFX boundary / shell registers
//   * RM -> host memory through XDMA s_axib (64-bit addresses)
//   * SI5381 OUT4 (299.825 MHz) shared between the C3 MIG (No_Buffer) and
//     the fabric MMCM (DFX_BOUNDARY.md 2.1); OUT3/5/6 feed the C1/C2/C4
//     MIGs differentially
//   * 4x MIG DDR4-2400 in AXI mode.  The four instantiations are generated
//     from the IP .veo files by syn/shell/gen_ip.tcl into
//     build/ipgen/ng_mig_<ch>.vh (net declarations + instance), so the
//     exact MIG port list never has to be transcribed by hand; this file
//     includes them and passes -include_dirs build/ipgen at elaboration.
//   * BSCAN USER1 status chain; USER3/USER4 raw tunnels to the RM
//   * MSI: XDMA bridge mode exposes one usr_irq line, so shell vectors 0
//     and 1 are ORed onto it (DFX_BOUNDARY.md 4.1; the driver reads both
//     status registers on every interrupt)
// MMCM DRP and the CLK_FREQ_* meters are live since SHELL_REV 2.
// SHELL_REV 3: ICAP readback + PR completion options in BAR1, PR/ICAP
// black box appended to the USER1 chain (288 bits).
// Still pending: MIG USER2 calibration detail chain (MIG debug disabled),
// debug-hub bridge IP on USER3 (raw BSCANE2 tunnel only; verify with a real
// RM later).
module ng_board_top
  import ng_dfx_pkg::*;
#(
  parameter logic [63:0] SHELL_UID = 64'h0,
  parameter logic [15:0] SHELL_REV = 16'h0006,          // 3: ICAP readback / PR diagnostics; 4: PR_OPT.ORDER resets to 1, PR FIFO din = latched w_data; 5 (b20): RM_INFO magic NGRM, ng_ names; 6 (b21): shared ng_prim FIFO / skid, generated sh_* nets (static re-implemented, b20 RMs do not fit)
  parameter SI_MEM = "rtl/shell/si/si_config_out4c.mem"
) (
  // PCIe (MGTREFCLK0_226 AG9/AG8, lane quad 227 ch3, PERST# AM17)
  input  wire sys_clk_p, sys_clk_n, sys_reset,
  input  wire [0:0] pci_exp_rxp, pci_exp_rxn,
  output wire [0:0] pci_exp_txp, pci_exp_txn,
  // SI5381 control (bank 70)
  inout  wire si_scl, si_sda, si_csb,
  output wire si_rstn,
  // MIG reference clocks: SI5381 OUT3 (C1, G22/G21), OUT4 (C3, AY9/BA9,
  // shared with the fabric MMCM), OUT5 (C2, AL32/AM32), OUT6 (C4, AV33/AW33)
  input  wire c1_sys_clk_p, c1_sys_clk_n,
  input  wire c3_sys_clk_p, c3_sys_clk_n,
  input  wire c2_sys_clk_p, c2_sys_clk_n,
  input  wire c4_sys_clk_p, c4_sys_clk_n,
  // DDR4 C1: 64-bit
  inout  wire [63:0] c1_ddr4_dq,
  inout  wire [7:0]  c1_ddr4_dqs_t, c1_ddr4_dqs_c,
  output wire [7:0]  c1_ddr4_dm_dbi_n,
  output wire [16:0] c1_ddr4_adr,
  output wire [1:0]  c1_ddr4_ba,
  output wire [0:0]  c1_ddr4_bg, c1_ddr4_cke, c1_ddr4_cs_n, c1_ddr4_odt,
  output wire [0:0]  c1_ddr4_ck_t, c1_ddr4_ck_c,
  output wire        c1_ddr4_act_n, c1_ddr4_reset_n,
  // DDR4 C2: 32-bit
  inout  wire [31:0] c2_ddr4_dq,
  inout  wire [3:0]  c2_ddr4_dqs_t, c2_ddr4_dqs_c,
  output wire [3:0]  c2_ddr4_dm_dbi_n,
  output wire [16:0] c2_ddr4_adr,
  output wire [1:0]  c2_ddr4_ba,
  output wire [0:0]  c2_ddr4_bg, c2_ddr4_cke, c2_ddr4_cs_n, c2_ddr4_odt,
  output wire [0:0]  c2_ddr4_ck_t, c2_ddr4_ck_c,
  output wire        c2_ddr4_act_n, c2_ddr4_reset_n,
  // DDR4 C3: 16-bit
  inout  wire [15:0] c3_ddr4_dq,
  inout  wire [1:0]  c3_ddr4_dqs_t, c3_ddr4_dqs_c,
  output wire [1:0]  c3_ddr4_dm_dbi_n,
  output wire [16:0] c3_ddr4_adr,
  output wire [1:0]  c3_ddr4_ba,
  output wire [0:0]  c3_ddr4_bg, c3_ddr4_cke, c3_ddr4_cs_n, c3_ddr4_odt,
  output wire [0:0]  c3_ddr4_ck_t, c3_ddr4_ck_c,
  output wire        c3_ddr4_act_n, c3_ddr4_reset_n,
  // DDR4 C4: 32-bit
  inout  wire [31:0] c4_ddr4_dq,
  inout  wire [3:0]  c4_ddr4_dqs_t, c4_ddr4_dqs_c,
  output wire [3:0]  c4_ddr4_dm_dbi_n,
  output wire [16:0] c4_ddr4_adr,
  output wire [1:0]  c4_ddr4_ba,
  output wire [0:0]  c4_ddr4_bg, c4_ddr4_cke, c4_ddr4_cs_n, c4_ddr4_odt,
  output wire [0:0]  c4_ddr4_ck_t, c4_ddr4_ck_c,
  output wire        c4_ddr4_act_n, c4_ddr4_reset_n
);

  // ---------------------------------------------------------------- clocks --
  logic cfgmclk_raw, clk_cfg;
  logic eos;                                          // STARTUPE3 EOS -> shell PR_STATUS.EOS (async, synced in ctrl)
  logic clk_sh, rst_sh_n;                             // XDMA axi_aclk / axi_aresetn
  logic c1_calib, c2_calib, c3_calib, c4_calib;       // MIG init_calib_complete
  // UltraScale+ primitive; tie-offs as in the board-verified rescue tops:
  // flash data/FCS_B tristated, KEYCLEARB inactive (it is active low).
  STARTUPE3 #(
    .PROG_USR      ("FALSE"),
    .SIM_CCLK_FREQ (0.0)
  ) u_startup (
    .CFGCLK    (),
    .CFGMCLK   (cfgmclk_raw),
    .DI        (),
    .EOS       (eos),
    .PREQ      (),
    .DO        (4'b0000),
    .DTS       (4'b1111),
    .FCSBO     (1'b0),
    .FCSBTS    (1'b1),
    .GSR       (1'b0),
    .GTS       (1'b0),
    .KEYCLEARB (1'b1),
    .PACK      (1'b0),
    .USRCCLKO  (1'b0),
    .USRCCLKTS (1'b1),
    .USRDONEO  (1'b1),
    .USRDONETS (1'b0)
  );
  BUFG bufg_cfg (.I(cfgmclk_raw), .O(clk_cfg));

  // GT refclk for the PCIe hard IP; ODIV2 feeds the XDMA sys_clk input,
  // whose internal BUFG_GT (bufg_gt_sysclk) can only be driven from ODIV2
  // (the O pin is reserved for GTREFCLK loads -- placer DRC).
  logic gt_refclk, gt_refclk_div2;
  IBUFDS_GTE4 u_gtclk (.O(gt_refclk), .ODIV2(gt_refclk_div2), .I(sys_clk_p), .IB(sys_clk_n), .CEB(1'b0));

  // SI5381 OUT4 shared by the C3 MIG and the fabric MMCM (DFX_BOUNDARY 2.1):
  // one input buffer, one global net, MMCM CLKIN and MIG C3 (No_Buffer mode).
  // The BUFG hop is intentional (commodity IBUFDS -> global -> MMCM CLKIN),
  // constrained with CLOCK_DEDICATED_ROUTE in shell.xdc.
  logic clk_ref_ibuf, clk_ref_fabric;
  IBUFDS u_c3ref (.O(clk_ref_ibuf), .I(c3_sys_clk_p), .IB(c3_sys_clk_n));
  BUFG u_bg_ref (.I(clk_ref_ibuf), .O(clk_ref_fabric));

  logic mmcm_clk_core2x, mmcm_clk_l2, mmcm_clk_ts, mmcm_locked, mmcm_reset;
  // DRP from the BAR1 CLK_DRP_* registers (clk_sh = DCLK)
  logic        mmcm_drp_en, mmcm_drp_we, mmcm_drp_rdy;
  logic [6:0]  mmcm_drp_addr;
  logic [15:0] mmcm_drp_di, mmcm_drp_do;
  ng_prim_mmcm u_mmcm (
    .clk_ref  (clk_ref_fabric),
    .rst      (mmcm_reset),
    .locked   (mmcm_locked),
    .clk_out0 (mmcm_clk_core2x),
    .clk_out1 (mmcm_clk_l2),
    .clk_out2 (mmcm_clk_ts),
    .drp_clk  (clk_sh),
    .drp_en   (mmcm_drp_en),
    .drp_we   (mmcm_drp_we),
    .drp_addr (mmcm_drp_addr),
    .drp_di   (mmcm_drp_di),
    .drp_do   (mmcm_drp_do),
    .drp_rdy  (mmcm_drp_rdy)
  );

  // ------------------------------------------------------------------ SI5381 --
  logic si_done_raw, si_err_raw, si_done, si_err;
  si_init #(.SI_MEM(SI_MEM)) u_si (
    .clk (clk_cfg),
    .rstn(si_rstn),
    .done(si_done_raw),
    .err (si_err_raw),
    .scl (si_scl),
    .sda (si_sda),
    .csb (si_csb)
  );

  // -------------------------------------------------------------------- XDMA --
  logic link_up, link_up_q;
  logic [5:0] ltssm;
  logic msi_enable;
  logic [2:0] msi_vw;

  // router <-> XDMA (m_axib: one AXI4-64 master, 32-bit bridge addresses)
  logic [3:0]  x_awid;
  logic [31:0] x_awaddr;
  logic [7:0]  x_awlen;
  logic [2:0]  x_awsize;
  logic [1:0]  x_awburst;
  logic [3:0]  x_awcache;
  logic [2:0]  x_awprot;
  logic        x_awvalid, x_awready;
  logic [63:0] x_wdata;
  logic [7:0]  x_wstrb;
  logic        x_wlast, x_wvalid, x_wready;
  logic [3:0]  x_bid;
  logic [1:0]  x_bresp;
  logic        x_bvalid, x_bready;
  logic [3:0]  x_arid;
  logic [31:0] x_araddr;
  logic [7:0]  x_arlen;
  logic [2:0]  x_arsize;
  logic [1:0]  x_arburst;
  logic [3:0]  x_arcache;
  logic [2:0]  x_arprot;
  logic        x_arvalid, x_arready;
  logic [3:0]  x_rid;
  logic [63:0] x_rdata;
  logic [1:0]  x_rresp;
  logic        x_rlast, x_rvalid, x_rready;

  // host path (RM -> host memory), XDMA s_axib side
  logic [3:0]  h_awid;
  logic [63:0] h_awaddr, h_araddr;
  logic [7:0]  h_awlen, h_arlen;
  logic [2:0]  h_awsize, h_arsize;
  logic [1:0]  h_awburst, h_arburst;
  logic [3:0]  h_awcache, h_arcache;
  logic [2:0]  h_awprot, h_arprot;
  logic        h_awvalid, h_awready;
  logic [63:0] h_wdata;
  logic [7:0]  h_wstrb;
  logic        h_wlast, h_wvalid, h_wready;
  logic [3:0]  h_bid;
  logic [1:0]  h_bresp;
  logic        h_bvalid, h_bready;
  logic [3:0]  h_arid;
  logic        h_arvalid, h_arready;
  logic [3:0]  h_rid;
  logic [63:0] h_rdata;
  logic [1:0]  h_rresp;
  logic        h_rlast, h_rvalid, h_rready;

  logic [1:0] msi_req, msi_ack;
  logic usr_irq_ack;

  xdma_nicogpu u_xdma (
    .sys_clk         (gt_refclk_div2),
    .sys_clk_gt      (gt_refclk),
    .sys_rst_n       (sys_reset),   // AM17 is PERST# itself (active low), as in the board-verified pciedma top
    .cfg_ltssm_state (ltssm),
    .user_lnk_up     (link_up),
    .pci_exp_txp     (pci_exp_txp),
    .pci_exp_txn     (pci_exp_txn),
    .pci_exp_rxp     (pci_exp_rxp),
    .pci_exp_rxn     (pci_exp_rxn),
    .axi_aclk        (clk_sh),
    .axi_aresetn     (rst_sh_n),
    .axi_ctl_aresetn (),
    .usr_irq_req     (msi_req[0] | msi_req[1]),
    .usr_irq_ack     (usr_irq_ack),
    .msi_enable      (msi_enable),
    .msi_vector_width(msi_vw),
    // BAR window master -> router
    .m_axib_awid     (x_awid),
    .m_axib_awaddr   (x_awaddr),
    .m_axib_awlen    (x_awlen),
    .m_axib_awsize   (x_awsize),
    .m_axib_awburst  (x_awburst),
    .m_axib_awlock   (),
    .m_axib_awcache  (x_awcache),
    .m_axib_awprot   (x_awprot),
    .m_axib_awvalid  (x_awvalid),
    .m_axib_awready  (x_awready),
    .m_axib_wdata    (x_wdata),
    .m_axib_wstrb    (x_wstrb),
    .m_axib_wlast    (x_wlast),
    .m_axib_wvalid   (x_wvalid),
    .m_axib_wready   (x_wready),
    .m_axib_bid      (x_bid),
    .m_axib_bresp    (x_bresp),
    .m_axib_bvalid   (x_bvalid),
    .m_axib_bready   (x_bready),
    .m_axib_arid     (x_arid),
    .m_axib_araddr   (x_araddr),
    .m_axib_arlen    (x_arlen),
    .m_axib_arsize   (x_arsize),
    .m_axib_arburst  (x_arburst),
    .m_axib_arlock   (),
    .m_axib_arcache  (x_arcache),
    .m_axib_arprot   (x_arprot),
    .m_axib_arvalid  (x_arvalid),
    .m_axib_arready  (x_arready),
    .m_axib_rid      (x_rid),
    .m_axib_rdata    (x_rdata),
    .m_axib_rresp    (x_rresp),
    .m_axib_rlast    (x_rlast),
    .m_axib_rvalid   (x_rvalid),
    .m_axib_rready   (x_rready),
    // host slave port (64-bit AXI addresses)
    .s_axib_awid     (h_awid),
    .s_axib_awaddr   (h_awaddr),
    .s_axib_awregion (4'h0),
    .s_axib_awlen    (h_awlen),
    .s_axib_awsize   (h_awsize),
    .s_axib_awburst  (h_awburst),
    .s_axib_awvalid  (h_awvalid),
    .s_axib_awready  (h_awready),
    .s_axib_wdata    (h_wdata),
    .s_axib_wstrb    (h_wstrb),
    .s_axib_wlast    (h_wlast),
    .s_axib_wvalid   (h_wvalid),
    .s_axib_wready   (h_wready),
    .s_axib_bid      (h_bid),
    .s_axib_bresp    (h_bresp),
    .s_axib_bvalid   (h_bvalid),
    .s_axib_bready   (h_bready),
    .s_axib_arid     (h_arid),
    .s_axib_araddr   (h_araddr),
    .s_axib_arregion (4'h0),
    .s_axib_arlen    (h_arlen),
    .s_axib_arsize   (h_arsize),
    .s_axib_arburst  (h_arburst),
    .s_axib_arvalid  (h_arvalid),
    .s_axib_arready  (h_arready),
    .s_axib_rid      (h_rid),
    .s_axib_rdata    (h_rdata),
    .s_axib_rresp    (h_rresp),
    .s_axib_rlast    (h_rlast),
    .s_axib_rvalid   (h_rvalid),
    .s_axib_rready   (h_rready),
    // XDMA CSR unused in bridge mode
    .s_axil_awaddr   (32'h0),
    .s_axil_awprot   (3'h0),
    .s_axil_awvalid  (1'b0),
    .s_axil_awready  (),
    .s_axil_wdata    (32'h0),
    .s_axil_wstrb    (4'h0),
    .s_axil_wvalid   (1'b0),
    .s_axil_wready   (),
    .s_axil_bvalid   (),
    .s_axil_bresp    (),
    .s_axil_bready   (1'b1),
    .s_axil_araddr   (32'h0),
    .s_axil_arprot   (3'h0),
    .s_axil_arvalid  (1'b0),
    .s_axil_arready  (),
    .s_axil_rdata    (),
    .s_axil_rresp    (),
    .s_axil_rvalid   (),
    .s_axil_rready   (1'b1),
    .interrupt_out   ()
  );
  assign msi_ack = {2{usr_irq_ack}};
  ng_prim_sync #(.W(1)) u_sync_lnk (.clk(clk_sh), .rst_n(rst_sh_n), .d(link_up), .q(link_up_q));

  // -------------------------------------------------------------- BAR router --
  logic l2_clk_ok, fence_stuck;

  // m0 (BAR0) full-64
  logic [3:0]  b0_awid;
  logic [19:0] b0_awaddr;
  logic [7:0]  b0_awlen;
  logic [2:0]  b0_awsize;
  logic [1:0]  b0_awburst;
  logic [3:0]  b0_awcache;
  logic [2:0]  b0_awprot;
  logic        b0_awvalid, b0_awready;
  logic [63:0] b0_wdata;
  logic [7:0]  b0_wstrb;
  logic        b0_wlast, b0_wvalid, b0_wready;
  logic [3:0]  b0_bid;
  logic [1:0]  b0_bresp;
  logic        b0_bvalid, b0_bready;
  logic [3:0]  b0_arid;
  logic [19:0] b0_araddr;
  logic [7:0]  b0_arlen;
  logic [2:0]  b0_arsize;
  logic [1:0]  b0_arburst;
  logic [3:0]  b0_arcache;
  logic [2:0]  b0_arprot;
  logic        b0_arvalid, b0_arready;
  logic [3:0]  b0_rid;
  logic [63:0] b0_rdata;
  logic [1:0]  b0_rresp;
  logic        b0_rlast, b0_rvalid, b0_rready;

  // m1 (BAR1) full-64
  logic [3:0]  b1_awid;
  logic [15:0] b1_awaddr;
  logic [7:0]  b1_awlen;
  logic [2:0]  b1_awsize;
  logic [1:0]  b1_awburst;
  logic [3:0]  b1_awcache;
  logic [2:0]  b1_awprot;
  logic        b1_awvalid, b1_awready;
  logic [63:0] b1_wdata;
  logic [7:0]  b1_wstrb;
  logic        b1_wlast, b1_wvalid, b1_wready;
  logic [3:0]  b1_bid;
  logic [1:0]  b1_bresp;
  logic        b1_bvalid, b1_bready;
  logic [3:0]  b1_arid;
  logic [15:0] b1_araddr;
  logic [7:0]  b1_arlen;
  logic [2:0]  b1_arsize;
  logic [1:0]  b1_arburst;
  logic [3:0]  b1_arcache;
  logic [2:0]  b1_arprot;
  logic        b1_arvalid, b1_arready;
  logic [3:0]  b1_rid;
  logic [63:0] b1_rdata;
  logic [1:0]  b1_rresp;
  logic        b1_rlast, b1_rvalid, b1_rready;

  // m2 (BAR2) full-64
  logic [3:0]  b2_awid;
  logic [27:0] b2_awaddr;
  logic [7:0]  b2_awlen;
  logic [2:0]  b2_awsize;
  logic [1:0]  b2_awburst;
  logic [3:0]  b2_awcache;
  logic [2:0]  b2_awprot;
  logic        b2_awvalid, b2_awready;
  logic [63:0] b2_wdata;
  logic [7:0]  b2_wstrb;
  logic        b2_wlast, b2_wvalid, b2_wready;
  logic [3:0]  b2_bid;
  logic [1:0]  b2_bresp;
  logic        b2_bvalid, b2_bready;
  logic [3:0]  b2_arid;
  logic [27:0] b2_araddr;
  logic [7:0]  b2_arlen;
  logic [2:0]  b2_arsize;
  logic [1:0]  b2_arburst;
  logic [3:0]  b2_arcache;
  logic [2:0]  b2_arprot;
  logic        b2_arvalid, b2_arready;
  logic [3:0]  b2_rid;
  logic [63:0] b2_rdata;
  logic [1:0]  b2_rresp;
  logic        b2_rlast, b2_rvalid, b2_rready;

  // l2_clk_ok: the MMCM is the only thing that can stop clk_l2_s (the RM
  // copies are gated by the shell; the static copy never is).  While the RM
  // is gated the iso blocks keep answering locally, so the router only
  // needs to fence on MMCM loss (DFX_BOUNDARY 2.4).
  ng_prim_sync #(.W(1)) u_sync_lock (.clk(clk_sh), .rst_n(rst_sh_n), .d(mmcm_locked), .q(l2_clk_ok));

  ng_bar_router u_router (
    .clk_sh   (clk_sh),
    .rst_sh_n (rst_sh_n),
    .l2_clk_ok(l2_clk_ok),
    .fence_stuck(fence_stuck),
    .s_awid   (x_awid),
    .s_awaddr (x_awaddr),
    .s_awlen  (x_awlen),
    .s_awsize (x_awsize),
    .s_awburst(x_awburst),
    .s_awcache(x_awcache),
    .s_awprot (x_awprot),
    .s_awvalid(x_awvalid),
    .s_awready(x_awready),
    .s_wdata  (x_wdata),
    .s_wstrb  (x_wstrb),
    .s_wlast  (x_wlast),
    .s_wvalid (x_wvalid),
    .s_wready (x_wready),
    .s_bid    (x_bid),
    .s_bresp  (x_bresp),
    .s_bvalid (x_bvalid),
    .s_bready (x_bready),
    .s_arid   (x_arid),
    .s_araddr (x_araddr),
    .s_arlen  (x_arlen),
    .s_arsize (x_arsize),
    .s_arburst(x_arburst),
    .s_arcache(x_arcache),
    .s_arprot (x_arprot),
    .s_arvalid(x_arvalid),
    .s_arready(x_arready),
    .s_rid    (x_rid),
    .s_rdata  (x_rdata),
    .s_rresp  (x_rresp),
    .s_rlast  (x_rlast),
    .s_rvalid (x_rvalid),
    .s_rready (x_rready),
    .m0_awid   (b0_awid),
    .m0_awaddr (b0_awaddr),
    .m0_awlen  (b0_awlen),
    .m0_awsize (b0_awsize),
    .m0_awburst(b0_awburst),
    .m0_awcache(b0_awcache),
    .m0_awprot (b0_awprot),
    .m0_awvalid(b0_awvalid),
    .m0_awready(b0_awready),
    .m0_wdata  (b0_wdata),
    .m0_wstrb  (b0_wstrb),
    .m0_wlast  (b0_wlast),
    .m0_wvalid (b0_wvalid),
    .m0_wready (b0_wready),
    .m0_bid    (b0_bid),
    .m0_bresp  (b0_bresp),
    .m0_bvalid (b0_bvalid),
    .m0_bready (b0_bready),
    .m0_arid   (b0_arid),
    .m0_araddr (b0_araddr),
    .m0_arlen  (b0_arlen),
    .m0_arsize (b0_arsize),
    .m0_arburst(b0_arburst),
    .m0_arcache(b0_arcache),
    .m0_arprot (b0_arprot),
    .m0_arvalid(b0_arvalid),
    .m0_arready(b0_arready),
    .m0_rid    (b0_rid),
    .m0_rdata  (b0_rdata),
    .m0_rresp  (b0_rresp),
    .m0_rlast  (b0_rlast),
    .m0_rvalid (b0_rvalid),
    .m0_rready (b0_rready),
    .m1_awid   (b1_awid),
    .m1_awaddr (b1_awaddr),
    .m1_awlen  (b1_awlen),
    .m1_awsize (b1_awsize),
    .m1_awburst(b1_awburst),
    .m1_awcache(b1_awcache),
    .m1_awprot (b1_awprot),
    .m1_awvalid(b1_awvalid),
    .m1_awready(b1_awready),
    .m1_wdata  (b1_wdata),
    .m1_wstrb  (b1_wstrb),
    .m1_wlast  (b1_wlast),
    .m1_wvalid (b1_wvalid),
    .m1_wready (b1_wready),
    .m1_bid    (b1_bid),
    .m1_bresp  (b1_bresp),
    .m1_bvalid (b1_bvalid),
    .m1_bready (b1_bready),
    .m1_arid   (b1_arid),
    .m1_araddr (b1_araddr),
    .m1_arlen  (b1_arlen),
    .m1_arsize (b1_arsize),
    .m1_arburst(b1_arburst),
    .m1_arcache(b1_arcache),
    .m1_arprot (b1_arprot),
    .m1_arvalid(b1_arvalid),
    .m1_arready(b1_arready),
    .m1_rid    (b1_rid),
    .m1_rdata  (b1_rdata),
    .m1_rresp  (b1_rresp),
    .m1_rlast  (b1_rlast),
    .m1_rvalid (b1_rvalid),
    .m1_rready (b1_rready),
    .m2_awid   (b2_awid),
    .m2_awaddr (b2_awaddr),
    .m2_awlen  (b2_awlen),
    .m2_awsize (b2_awsize),
    .m2_awburst(b2_awburst),
    .m2_awcache(b2_awcache),
    .m2_awprot (b2_awprot),
    .m2_awvalid(b2_awvalid),
    .m2_awready(b2_awready),
    .m2_wdata  (b2_wdata),
    .m2_wstrb  (b2_wstrb),
    .m2_wlast  (b2_wlast),
    .m2_wvalid (b2_wvalid),
    .m2_wready (b2_wready),
    .m2_bid    (b2_bid),
    .m2_bresp  (b2_bresp),
    .m2_bvalid (b2_bvalid),
    .m2_bready (b2_bready),
    .m2_arid   (b2_arid),
    .m2_araddr (b2_araddr),
    .m2_arlen  (b2_arlen),
    .m2_arsize (b2_arsize),
    .m2_arburst(b2_arburst),
    .m2_arcache(b2_arcache),
    .m2_arprot (b2_arprot),
    .m2_arvalid(b2_arvalid),
    .m2_arready(b2_arready),
    .m2_rid    (b2_rid),
    .m2_rdata  (b2_rdata),
    .m2_rresp  (b2_rresp),
    .m2_rlast  (b2_rlast),
    .m2_rvalid (b2_rvalid),
    .m2_rready (b2_rready)
  );

  // -------------------------------------------------- static top (DFX core) ---
  // signal names match ng_shell_static_top ports exactly; (.*) connects
  logic clk_l2_s, rst_l2_s_n;
  logic [3:0] mem_calib;

  // BAR1 registers (clk_sh)
  logic [15:0] bar1_awaddr;
  logic        bar1_awvalid, bar1_awready;
  logic [31:0] bar1_wdata;
  logic [3:0]  bar1_wstrb;
  logic        bar1_wvalid, bar1_wready, bar1_bvalid, bar1_bready;
  logic [1:0]  bar1_bresp;
  logic [15:0] bar1_araddr;
  logic        bar1_arvalid, bar1_arready;
  logic [31:0] bar1_rdata;
  logic [1:0]  bar1_rresp;
  logic        bar1_rvalid, bar1_rready;

  // sh_* nets of the DFX boundary (control port, BAR2 window, host / DDR4 master ports, BSCAN tunnels), generated from
  // the same yaml as the static top's ports (tools/dfxgen)
`include "ng_dfx_shell_nets.svh"
  logic [159:0] pr_dbg;                               // PR/ICAP black box -> USER1 [287:128]


  ng_shell_static_top #(
    .SHELL_UID (SHELL_UID),
    .SHELL_REV (SHELL_REV)
  ) u_static (
    .*
  );

  // status into the shell: si and per-channel calibration are not clk_sh
  ng_prim_sync #(.W(2)) u_sync_si (.clk(clk_sh), .rst_n(rst_sh_n), .d({si_err_raw, si_done_raw}), .q({si_err, si_done}));
  ng_prim_sync #(.W(4)) u_sync_cal (.clk(clk_sh), .rst_n(rst_sh_n), .d({c4_calib, c3_calib, c2_calib, c1_calib}), .q(mem_calib));  // bit0 = C1

  // --------------------------------------------------------------- BAR0 path --
  logic [3:0]  b0c_awid;
  logic [19:0] b0c_awaddr;
  logic [7:0]  b0c_awlen;
  logic [2:0]  b0c_awsize;
  logic [1:0]  b0c_awburst;
  logic [3:0]  b0c_awcache;
  logic [2:0]  b0c_awprot;
  logic [3:0]  b0c_awqos;
  logic        b0c_awvalid, b0c_awready;
  logic [63:0] b0c_wdata;
  logic [7:0]  b0c_wstrb;
  logic        b0c_wlast, b0c_wvalid, b0c_wready;
  logic [3:0]  b0c_bid;
  logic [1:0]  b0c_bresp;
  logic        b0c_bvalid, b0c_bready;
  logic [3:0]  b0c_arid;
  logic [19:0] b0c_araddr;
  logic [7:0]  b0c_arlen;
  logic [2:0]  b0c_arsize;
  logic [1:0]  b0c_arburst;
  logic [3:0]  b0c_arcache;
  logic [2:0]  b0c_arprot;
  logic [3:0]  b0c_arqos;
  logic        b0c_arvalid, b0c_arready;
  logic [3:0]  b0c_rid;
  logic [63:0] b0c_rdata;
  logic [1:0]  b0c_rresp;
  logic        b0c_rlast, b0c_rvalid, b0c_rready;

  ng_axi_cc #(.ID_W(S_AXI_BAR2_ID_W), .ADDR_W(S_AXIL_CTRL_ADDR_W), .DATA_W(S_AXI_BAR2_DATA_W)) u_cc_bar0 (
    .s_aclk(clk_sh), .s_aresetn(rst_sh_n),
    .s_axi_awid(b0_awid), .s_axi_awaddr(b0_awaddr), .s_axi_awlen(b0_awlen),
    .s_axi_awsize(b0_awsize), .s_axi_awburst(b0_awburst), .s_axi_awcache(b0_awcache),
    .s_axi_awprot(b0_awprot), .s_axi_awqos(4'h0),
    .s_axi_awvalid(b0_awvalid), .s_axi_awready(b0_awready),
    .s_axi_wdata(b0_wdata), .s_axi_wstrb(b0_wstrb), .s_axi_wlast(b0_wlast),
    .s_axi_wvalid(b0_wvalid), .s_axi_wready(b0_wready),
    .s_axi_bid(b0_bid), .s_axi_bresp(b0_bresp), .s_axi_bvalid(b0_bvalid), .s_axi_bready(b0_bready),
    .s_axi_arid(b0_arid), .s_axi_araddr(b0_araddr), .s_axi_arlen(b0_arlen),
    .s_axi_arsize(b0_arsize), .s_axi_arburst(b0_arburst), .s_axi_arcache(b0_arcache),
    .s_axi_arprot(b0_arprot), .s_axi_arqos(4'h0),
    .s_axi_arvalid(b0_arvalid), .s_axi_arready(b0_arready),
    .s_axi_rid(b0_rid), .s_axi_rdata(b0_rdata), .s_axi_rresp(b0_rresp),
    .s_axi_rlast(b0_rlast), .s_axi_rvalid(b0_rvalid), .s_axi_rready(b0_rready),
    .m_aclk(clk_l2_s), .m_aresetn(rst_l2_s_n),
    .m_axi_awid(b0c_awid), .m_axi_awaddr(b0c_awaddr), .m_axi_awlen(b0c_awlen),
    .m_axi_awsize(b0c_awsize), .m_axi_awburst(b0c_awburst), .m_axi_awcache(b0c_awcache),
    .m_axi_awprot(b0c_awprot), .m_axi_awqos(b0c_awqos),
    .m_axi_awvalid(b0c_awvalid), .m_axi_awready(b0c_awready),
    .m_axi_wdata(b0c_wdata), .m_axi_wstrb(b0c_wstrb), .m_axi_wlast(b0c_wlast),
    .m_axi_wvalid(b0c_wvalid), .m_axi_wready(b0c_wready),
    .m_axi_bid(b0c_bid), .m_axi_bresp(b0c_bresp), .m_axi_bvalid(b0c_bvalid), .m_axi_bready(b0c_bready),
    .m_axi_arid(b0c_arid), .m_axi_araddr(b0c_araddr), .m_axi_arlen(b0c_arlen),
    .m_axi_arsize(b0c_arsize), .m_axi_arburst(b0c_arburst), .m_axi_arcache(b0c_arcache),
    .m_axi_arprot(b0c_arprot), .m_axi_arqos(b0c_arqos),
    .m_axi_arvalid(b0c_arvalid), .m_axi_arready(b0c_arready),
    .m_axi_rid(b0c_rid), .m_axi_rdata(b0c_rdata), .m_axi_rresp(b0c_rresp),
    .m_axi_rlast(b0c_rlast), .m_axi_rvalid(b0c_rvalid), .m_axi_rready(b0c_rready)
  );

  ng_axi64_to_lite32 #(.AW_W(20)) u_lite_bar0 (
    .clk(clk_l2_s), .rst_n(rst_l2_s_n),
    .s_awid(b0c_awid), .s_awaddr(b0c_awaddr), .s_awlen(b0c_awlen), .s_awsize(b0c_awsize),
    .s_awvalid(b0c_awvalid), .s_awready(b0c_awready),
    .s_wdata(b0c_wdata), .s_wstrb(b0c_wstrb), .s_wlast(b0c_wlast),
    .s_wvalid(b0c_wvalid), .s_wready(b0c_wready),
    .s_bid(b0c_bid), .s_bresp(b0c_bresp), .s_bvalid(b0c_bvalid), .s_bready(b0c_bready),
    .s_arid(b0c_arid), .s_araddr(b0c_araddr), .s_arlen(b0c_arlen), .s_arsize(b0c_arsize),
    .s_arvalid(b0c_arvalid), .s_arready(b0c_arready),
    .s_rid(b0c_rid), .s_rdata(b0c_rdata), .s_rresp(b0c_rresp),
    .s_rlast(b0c_rlast), .s_rvalid(b0c_rvalid), .s_rready(b0c_rready),
    .m_awaddr(sh_s_axil_ctrl_awaddr), .m_awprot(sh_s_axil_ctrl_awprot),
    .m_awvalid(sh_s_axil_ctrl_awvalid), .m_awready(sh_s_axil_ctrl_awready),
    .m_wdata(sh_s_axil_ctrl_wdata), .m_wstrb(sh_s_axil_ctrl_wstrb),
    .m_wvalid(sh_s_axil_ctrl_wvalid), .m_wready(sh_s_axil_ctrl_wready),
    .m_bresp(sh_s_axil_ctrl_bresp), .m_bvalid(sh_s_axil_ctrl_bvalid), .m_bready(sh_s_axil_ctrl_bready),
    .m_araddr(sh_s_axil_ctrl_araddr), .m_arprot(sh_s_axil_ctrl_arprot),
    .m_arvalid(sh_s_axil_ctrl_arvalid), .m_arready(sh_s_axil_ctrl_arready),
    .m_rdata(sh_s_axil_ctrl_rdata), .m_rresp(sh_s_axil_ctrl_rresp),
    .m_rvalid(sh_s_axil_ctrl_rvalid), .m_rready(sh_s_axil_ctrl_rready)
  );

  // --------------------------------------------------------------- BAR1 path --
  ng_axi64_to_lite32 #(.AW_W(16)) u_lite_bar1 (
    .clk(clk_sh), .rst_n(rst_sh_n),
    .s_awid(b1_awid), .s_awaddr(b1_awaddr), .s_awlen(b1_awlen), .s_awsize(b1_awsize),
    .s_awvalid(b1_awvalid), .s_awready(b1_awready),
    .s_wdata(b1_wdata), .s_wstrb(b1_wstrb), .s_wlast(b1_wlast),
    .s_wvalid(b1_wvalid), .s_wready(b1_wready),
    .s_bid(b1_bid), .s_bresp(b1_bresp), .s_bvalid(b1_bvalid), .s_bready(b1_bready),
    .s_arid(b1_arid), .s_araddr(b1_araddr), .s_arlen(b1_arlen), .s_arsize(b1_arsize),
    .s_arvalid(b1_arvalid), .s_arready(b1_arready),
    .s_rid(b1_rid), .s_rdata(b1_rdata), .s_rresp(b1_rresp),
    .s_rlast(b1_rlast), .s_rvalid(b1_rvalid), .s_rready(b1_rready),
    .m_awaddr(bar1_awaddr), .m_awprot(),
    .m_awvalid(bar1_awvalid), .m_awready(bar1_awready),
    .m_wdata(bar1_wdata), .m_wstrb(bar1_wstrb),
    .m_wvalid(bar1_wvalid), .m_wready(bar1_wready),
    .m_bresp(bar1_bresp), .m_bvalid(bar1_bvalid), .m_bready(bar1_bready),
    .m_araddr(bar1_araddr), .m_arprot(),
    .m_arvalid(bar1_arvalid), .m_arready(bar1_arready),
    .m_rdata(bar1_rdata), .m_rresp(bar1_rresp),
    .m_rvalid(bar1_rvalid), .m_rready(bar1_rready)
  );

  // --------------------------------------------------------------- BAR2 path --
  ng_axi_cc #(.ID_W(S_AXI_BAR2_ID_W), .ADDR_W(S_AXI_BAR2_ADDR_W), .DATA_W(S_AXI_BAR2_DATA_W)) u_cc_bar2 (
    .s_aclk(clk_sh), .s_aresetn(rst_sh_n),
    .s_axi_awid(b2_awid), .s_axi_awaddr(b2_awaddr), .s_axi_awlen(b2_awlen),
    .s_axi_awsize(b2_awsize), .s_axi_awburst(b2_awburst), .s_axi_awcache(b2_awcache),
    .s_axi_awprot(b2_awprot), .s_axi_awqos(4'h0),
    .s_axi_awvalid(b2_awvalid), .s_axi_awready(b2_awready),
    .s_axi_wdata(b2_wdata), .s_axi_wstrb(b2_wstrb), .s_axi_wlast(b2_wlast),
    .s_axi_wvalid(b2_wvalid), .s_axi_wready(b2_wready),
    .s_axi_bid(b2_bid), .s_axi_bresp(b2_bresp), .s_axi_bvalid(b2_bvalid), .s_axi_bready(b2_bready),
    .s_axi_arid(b2_arid), .s_axi_araddr(b2_araddr), .s_axi_arlen(b2_arlen),
    .s_axi_arsize(b2_arsize), .s_axi_arburst(b2_arburst), .s_axi_arcache(b2_arcache),
    .s_axi_arprot(b2_arprot), .s_axi_arqos(4'h0),
    .s_axi_arvalid(b2_arvalid), .s_axi_arready(b2_arready),
    .s_axi_rid(b2_rid), .s_axi_rdata(b2_rdata), .s_axi_rresp(b2_rresp),
    .s_axi_rlast(b2_rlast), .s_axi_rvalid(b2_rvalid), .s_axi_rready(b2_rready),
    .m_aclk(clk_l2_s), .m_aresetn(rst_l2_s_n),
    .m_axi_awid(sh_s_axi_bar2_awid), .m_axi_awaddr(sh_s_axi_bar2_awaddr), .m_axi_awlen(sh_s_axi_bar2_awlen),
    .m_axi_awsize(sh_s_axi_bar2_awsize), .m_axi_awburst(sh_s_axi_bar2_awburst), .m_axi_awcache(sh_s_axi_bar2_awcache),
    .m_axi_awprot(sh_s_axi_bar2_awprot), .m_axi_awqos(sh_s_axi_bar2_awqos),
    .m_axi_awvalid(sh_s_axi_bar2_awvalid), .m_axi_awready(sh_s_axi_bar2_awready),
    .m_axi_wdata(sh_s_axi_bar2_wdata), .m_axi_wstrb(sh_s_axi_bar2_wstrb), .m_axi_wlast(sh_s_axi_bar2_wlast),
    .m_axi_wvalid(sh_s_axi_bar2_wvalid), .m_axi_wready(sh_s_axi_bar2_wready),
    .m_axi_bid(sh_s_axi_bar2_bid), .m_axi_bresp(sh_s_axi_bar2_bresp), .m_axi_bvalid(sh_s_axi_bar2_bvalid), .m_axi_bready(sh_s_axi_bar2_bready),
    .m_axi_arid(sh_s_axi_bar2_arid), .m_axi_araddr(sh_s_axi_bar2_araddr), .m_axi_arlen(sh_s_axi_bar2_arlen),
    .m_axi_arsize(sh_s_axi_bar2_arsize), .m_axi_arburst(sh_s_axi_bar2_arburst), .m_axi_arcache(sh_s_axi_bar2_arcache),
    .m_axi_arprot(sh_s_axi_bar2_arprot), .m_axi_arqos(sh_s_axi_bar2_arqos),
    .m_axi_arvalid(sh_s_axi_bar2_arvalid), .m_axi_arready(sh_s_axi_bar2_arready),
    .m_axi_rid(sh_s_axi_bar2_rid), .m_axi_rdata(sh_s_axi_bar2_rdata), .m_axi_rresp(sh_s_axi_bar2_rresp),
    .m_axi_rlast(sh_s_axi_bar2_rlast), .m_axi_rvalid(sh_s_axi_bar2_rvalid), .m_axi_rready(sh_s_axi_bar2_rready)
  );

  // --------------------------------------------------------------- host path --
  // 47-bit boundary address zero-extended onto the XDMA 64-bit slave port
  logic [46:0] h47_awaddr, h47_araddr;
  assign h_awaddr = {17'd0, h47_awaddr};
  assign h_araddr = {17'd0, h47_araddr};

  ng_axi_cc #(.ID_W(M_AXI_HOST_ID_W), .ADDR_W(M_AXI_HOST_ADDR_W), .DATA_W(M_AXI_HOST_DATA_W)) u_cc_host (
    .s_aclk(clk_l2_s), .s_aresetn(rst_l2_s_n),
    .s_axi_awid(sh_m_axi_host_awid), .s_axi_awaddr(sh_m_axi_host_awaddr), .s_axi_awlen(sh_m_axi_host_awlen),
    .s_axi_awsize(sh_m_axi_host_awsize), .s_axi_awburst(sh_m_axi_host_awburst), .s_axi_awcache(sh_m_axi_host_awcache),
    .s_axi_awprot(sh_m_axi_host_awprot), .s_axi_awqos(sh_m_axi_host_awqos),
    .s_axi_awvalid(sh_m_axi_host_awvalid), .s_axi_awready(sh_m_axi_host_awready),
    .s_axi_wdata(sh_m_axi_host_wdata), .s_axi_wstrb(sh_m_axi_host_wstrb), .s_axi_wlast(sh_m_axi_host_wlast),
    .s_axi_wvalid(sh_m_axi_host_wvalid), .s_axi_wready(sh_m_axi_host_wready),
    .s_axi_bid(sh_m_axi_host_bid), .s_axi_bresp(sh_m_axi_host_bresp), .s_axi_bvalid(sh_m_axi_host_bvalid), .s_axi_bready(sh_m_axi_host_bready),
    .s_axi_arid(sh_m_axi_host_arid), .s_axi_araddr(sh_m_axi_host_araddr), .s_axi_arlen(sh_m_axi_host_arlen),
    .s_axi_arsize(sh_m_axi_host_arsize), .s_axi_arburst(sh_m_axi_host_arburst), .s_axi_arcache(sh_m_axi_host_arcache),
    .s_axi_arprot(sh_m_axi_host_arprot), .s_axi_arqos(sh_m_axi_host_arqos),
    .s_axi_arvalid(sh_m_axi_host_arvalid), .s_axi_arready(sh_m_axi_host_arready),
    .s_axi_rid(sh_m_axi_host_rid), .s_axi_rdata(sh_m_axi_host_rdata), .s_axi_rresp(sh_m_axi_host_rresp),
    .s_axi_rlast(sh_m_axi_host_rlast), .s_axi_rvalid(sh_m_axi_host_rvalid), .s_axi_rready(sh_m_axi_host_rready),
    .m_aclk(clk_sh), .m_aresetn(rst_sh_n),
    .m_axi_awid(h_awid[3:0]), .m_axi_awaddr(h47_awaddr), .m_axi_awlen(h_awlen),
    .m_axi_awsize(h_awsize), .m_axi_awburst(h_awburst), .m_axi_awcache(h_awcache),
    .m_axi_awprot(h_awprot), .m_axi_awqos(),
    .m_axi_awvalid(h_awvalid), .m_axi_awready(h_awready),
    .m_axi_wdata(h_wdata), .m_axi_wstrb(h_wstrb), .m_axi_wlast(h_wlast),
    .m_axi_wvalid(h_wvalid), .m_axi_wready(h_wready),
    .m_axi_bid(h_bid), .m_axi_bresp(h_bresp), .m_axi_bvalid(h_bvalid), .m_axi_bready(h_bready),
    .m_axi_arid(h_arid), .m_axi_araddr(h47_araddr), .m_axi_arlen(h_arlen),
    .m_axi_arsize(h_arsize), .m_axi_arburst(h_arburst), .m_axi_arcache(h_arcache),
    .m_axi_arprot(h_arprot), .m_axi_arqos(),
    .m_axi_arvalid(h_arvalid), .m_axi_arready(h_arready),
    .m_axi_rid(h_rid), .m_axi_rdata(h_rdata), .m_axi_rresp(h_rresp),
    .m_axi_rlast(h_rlast), .m_axi_rvalid(h_rvalid), .m_axi_rready(h_rready)
  );

  // ---------------------------------------------------------------- MIG C1 ---
  logic c1_ui_clk, c1_ui_rst, c1_axi_rstn;
  assign c1_axi_rstn = ~c1_ui_rst;
  // c1_axi_*: MIG AXI slave nets (declared in the generated include)
`include "ng_mig_c1.vh"
  ng_axi_cc #(.ID_W(M_AXI_C1_ID_W), .ADDR_W(M_AXI_C1_ADDR_W), .DATA_W(M_AXI_C1_DATA_W)) u_cc_c1 (
    .s_aclk(clk_l2_s), .s_aresetn(rst_l2_s_n),
    .s_axi_awid(sh_m_axi_c1_awid), .s_axi_awaddr(sh_m_axi_c1_awaddr), .s_axi_awlen(sh_m_axi_c1_awlen),
    .s_axi_awsize(sh_m_axi_c1_awsize), .s_axi_awburst(sh_m_axi_c1_awburst), .s_axi_awcache(sh_m_axi_c1_awcache),
    .s_axi_awprot(sh_m_axi_c1_awprot), .s_axi_awqos(sh_m_axi_c1_awqos),
    .s_axi_awvalid(sh_m_axi_c1_awvalid), .s_axi_awready(sh_m_axi_c1_awready),
    .s_axi_wdata(sh_m_axi_c1_wdata), .s_axi_wstrb(sh_m_axi_c1_wstrb), .s_axi_wlast(sh_m_axi_c1_wlast),
    .s_axi_wvalid(sh_m_axi_c1_wvalid), .s_axi_wready(sh_m_axi_c1_wready),
    .s_axi_bid(sh_m_axi_c1_bid), .s_axi_bresp(sh_m_axi_c1_bresp), .s_axi_bvalid(sh_m_axi_c1_bvalid), .s_axi_bready(sh_m_axi_c1_bready),
    .s_axi_arid(sh_m_axi_c1_arid), .s_axi_araddr(sh_m_axi_c1_araddr), .s_axi_arlen(sh_m_axi_c1_arlen),
    .s_axi_arsize(sh_m_axi_c1_arsize), .s_axi_arburst(sh_m_axi_c1_arburst), .s_axi_arcache(sh_m_axi_c1_arcache),
    .s_axi_arprot(sh_m_axi_c1_arprot), .s_axi_arqos(sh_m_axi_c1_arqos),
    .s_axi_arvalid(sh_m_axi_c1_arvalid), .s_axi_arready(sh_m_axi_c1_arready),
    .s_axi_rid(sh_m_axi_c1_rid), .s_axi_rdata(sh_m_axi_c1_rdata), .s_axi_rresp(sh_m_axi_c1_rresp),
    .s_axi_rlast(sh_m_axi_c1_rlast), .s_axi_rvalid(sh_m_axi_c1_rvalid), .s_axi_rready(sh_m_axi_c1_rready),
    .m_aclk(c1_ui_clk), .m_aresetn(c1_axi_rstn),
    .m_axi_awid(c1_axi_awid), .m_axi_awaddr(c1_axi_awaddr), .m_axi_awlen(c1_axi_awlen),
    .m_axi_awsize(c1_axi_awsize), .m_axi_awburst(c1_axi_awburst), .m_axi_awcache(c1_axi_awcache),
    .m_axi_awprot(c1_axi_awprot), .m_axi_awqos(c1_axi_awqos),
    .m_axi_awvalid(c1_axi_awvalid), .m_axi_awready(c1_axi_awready),
    .m_axi_wdata(c1_axi_wdata), .m_axi_wstrb(c1_axi_wstrb), .m_axi_wlast(c1_axi_wlast),
    .m_axi_wvalid(c1_axi_wvalid), .m_axi_wready(c1_axi_wready),
    .m_axi_bid(c1_axi_bid), .m_axi_bresp(c1_axi_bresp), .m_axi_bvalid(c1_axi_bvalid), .m_axi_bready(c1_axi_bready),
    .m_axi_arid(c1_axi_arid), .m_axi_araddr(c1_axi_araddr), .m_axi_arlen(c1_axi_arlen),
    .m_axi_arsize(c1_axi_arsize), .m_axi_arburst(c1_axi_arburst), .m_axi_arcache(c1_axi_arcache),
    .m_axi_arprot(c1_axi_arprot), .m_axi_arqos(c1_axi_arqos),
    .m_axi_arvalid(c1_axi_arvalid), .m_axi_arready(c1_axi_arready),
    .m_axi_rid(c1_axi_rid), .m_axi_rdata(c1_axi_rdata), .m_axi_rresp(c1_axi_rresp),
    .m_axi_rlast(c1_axi_rlast), .m_axi_rvalid(c1_axi_rvalid), .m_axi_rready(c1_axi_rready)
  );

  // ---------------------------------------------------------------- MIG C2 ---
  logic c2_ui_clk, c2_ui_rst, c2_axi_rstn;
  assign c2_axi_rstn = ~c2_ui_rst;
`include "ng_mig_c2.vh"
  ng_axi_cc #(.ID_W(M_AXI_C2_ID_W), .ADDR_W(M_AXI_C2_ADDR_W), .DATA_W(M_AXI_C2_DATA_W)) u_cc_c2 (
    .s_aclk(clk_l2_s), .s_aresetn(rst_l2_s_n),
    .s_axi_awid(sh_m_axi_c2_awid), .s_axi_awaddr(sh_m_axi_c2_awaddr), .s_axi_awlen(sh_m_axi_c2_awlen),
    .s_axi_awsize(sh_m_axi_c2_awsize), .s_axi_awburst(sh_m_axi_c2_awburst), .s_axi_awcache(sh_m_axi_c2_awcache),
    .s_axi_awprot(sh_m_axi_c2_awprot), .s_axi_awqos(sh_m_axi_c2_awqos),
    .s_axi_awvalid(sh_m_axi_c2_awvalid), .s_axi_awready(sh_m_axi_c2_awready),
    .s_axi_wdata(sh_m_axi_c2_wdata), .s_axi_wstrb(sh_m_axi_c2_wstrb), .s_axi_wlast(sh_m_axi_c2_wlast),
    .s_axi_wvalid(sh_m_axi_c2_wvalid), .s_axi_wready(sh_m_axi_c2_wready),
    .s_axi_bid(sh_m_axi_c2_bid), .s_axi_bresp(sh_m_axi_c2_bresp), .s_axi_bvalid(sh_m_axi_c2_bvalid), .s_axi_bready(sh_m_axi_c2_bready),
    .s_axi_arid(sh_m_axi_c2_arid), .s_axi_araddr(sh_m_axi_c2_araddr), .s_axi_arlen(sh_m_axi_c2_arlen),
    .s_axi_arsize(sh_m_axi_c2_arsize), .s_axi_arburst(sh_m_axi_c2_arburst), .s_axi_arcache(sh_m_axi_c2_arcache),
    .s_axi_arprot(sh_m_axi_c2_arprot), .s_axi_arqos(sh_m_axi_c2_arqos),
    .s_axi_arvalid(sh_m_axi_c2_arvalid), .s_axi_arready(sh_m_axi_c2_arready),
    .s_axi_rid(sh_m_axi_c2_rid), .s_axi_rdata(sh_m_axi_c2_rdata), .s_axi_rresp(sh_m_axi_c2_rresp),
    .s_axi_rlast(sh_m_axi_c2_rlast), .s_axi_rvalid(sh_m_axi_c2_rvalid), .s_axi_rready(sh_m_axi_c2_rready),
    .m_aclk(c2_ui_clk), .m_aresetn(c2_axi_rstn),
    .m_axi_awid(c2_axi_awid), .m_axi_awaddr(c2_axi_awaddr), .m_axi_awlen(c2_axi_awlen),
    .m_axi_awsize(c2_axi_awsize), .m_axi_awburst(c2_axi_awburst), .m_axi_awcache(c2_axi_awcache),
    .m_axi_awprot(c2_axi_awprot), .m_axi_awqos(c2_axi_awqos),
    .m_axi_awvalid(c2_axi_awvalid), .m_axi_awready(c2_axi_awready),
    .m_axi_wdata(c2_axi_wdata), .m_axi_wstrb(c2_axi_wstrb), .m_axi_wlast(c2_axi_wlast),
    .m_axi_wvalid(c2_axi_wvalid), .m_axi_wready(c2_axi_wready),
    .m_axi_bid(c2_axi_bid), .m_axi_bresp(c2_axi_bresp), .m_axi_bvalid(c2_axi_bvalid), .m_axi_bready(c2_axi_bready),
    .m_axi_arid(c2_axi_arid), .m_axi_araddr(c2_axi_araddr), .m_axi_arlen(c2_axi_arlen),
    .m_axi_arsize(c2_axi_arsize), .m_axi_arburst(c2_axi_arburst), .m_axi_arcache(c2_axi_arcache),
    .m_axi_arprot(c2_axi_arprot), .m_axi_arqos(c2_axi_arqos),
    .m_axi_arvalid(c2_axi_arvalid), .m_axi_arready(c2_axi_arready),
    .m_axi_rid(c2_axi_rid), .m_axi_rdata(c2_axi_rdata), .m_axi_rresp(c2_axi_rresp),
    .m_axi_rlast(c2_axi_rlast), .m_axi_rvalid(c2_axi_rvalid), .m_axi_rready(c2_axi_rready)
  );

  // ---------------------------------------------------------------- MIG C3 ---
  logic c3_ui_clk, c3_ui_rst, c3_axi_rstn;
  assign c3_axi_rstn = ~c3_ui_rst;
`include "ng_mig_c3.vh"
  ng_axi_cc #(.ID_W(M_AXI_C3_ID_W), .ADDR_W(M_AXI_C3_ADDR_W), .DATA_W(M_AXI_C3_DATA_W)) u_cc_c3 (
    .s_aclk(clk_l2_s), .s_aresetn(rst_l2_s_n),
    .s_axi_awid(sh_m_axi_c3_awid), .s_axi_awaddr(sh_m_axi_c3_awaddr), .s_axi_awlen(sh_m_axi_c3_awlen),
    .s_axi_awsize(sh_m_axi_c3_awsize), .s_axi_awburst(sh_m_axi_c3_awburst), .s_axi_awcache(sh_m_axi_c3_awcache),
    .s_axi_awprot(sh_m_axi_c3_awprot), .s_axi_awqos(sh_m_axi_c3_awqos),
    .s_axi_awvalid(sh_m_axi_c3_awvalid), .s_axi_awready(sh_m_axi_c3_awready),
    .s_axi_wdata(sh_m_axi_c3_wdata), .s_axi_wstrb(sh_m_axi_c3_wstrb), .s_axi_wlast(sh_m_axi_c3_wlast),
    .s_axi_wvalid(sh_m_axi_c3_wvalid), .s_axi_wready(sh_m_axi_c3_wready),
    .s_axi_bid(sh_m_axi_c3_bid), .s_axi_bresp(sh_m_axi_c3_bresp), .s_axi_bvalid(sh_m_axi_c3_bvalid), .s_axi_bready(sh_m_axi_c3_bready),
    .s_axi_arid(sh_m_axi_c3_arid), .s_axi_araddr(sh_m_axi_c3_araddr), .s_axi_arlen(sh_m_axi_c3_arlen),
    .s_axi_arsize(sh_m_axi_c3_arsize), .s_axi_arburst(sh_m_axi_c3_arburst), .s_axi_arcache(sh_m_axi_c3_arcache),
    .s_axi_arprot(sh_m_axi_c3_arprot), .s_axi_arqos(sh_m_axi_c3_arqos),
    .s_axi_arvalid(sh_m_axi_c3_arvalid), .s_axi_arready(sh_m_axi_c3_arready),
    .s_axi_rid(sh_m_axi_c3_rid), .s_axi_rdata(sh_m_axi_c3_rdata), .s_axi_rresp(sh_m_axi_c3_rresp),
    .s_axi_rlast(sh_m_axi_c3_rlast), .s_axi_rvalid(sh_m_axi_c3_rvalid), .s_axi_rready(sh_m_axi_c3_rready),
    .m_aclk(c3_ui_clk), .m_aresetn(c3_axi_rstn),
    .m_axi_awid(c3_axi_awid), .m_axi_awaddr(c3_axi_awaddr), .m_axi_awlen(c3_axi_awlen),
    .m_axi_awsize(c3_axi_awsize), .m_axi_awburst(c3_axi_awburst), .m_axi_awcache(c3_axi_awcache),
    .m_axi_awprot(c3_axi_awprot), .m_axi_awqos(c3_axi_awqos),
    .m_axi_awvalid(c3_axi_awvalid), .m_axi_awready(c3_axi_awready),
    .m_axi_wdata(c3_axi_wdata), .m_axi_wstrb(c3_axi_wstrb), .m_axi_wlast(c3_axi_wlast),
    .m_axi_wvalid(c3_axi_wvalid), .m_axi_wready(c3_axi_wready),
    .m_axi_bid(c3_axi_bid), .m_axi_bresp(c3_axi_bresp), .m_axi_bvalid(c3_axi_bvalid), .m_axi_bready(c3_axi_bready),
    .m_axi_arid(c3_axi_arid), .m_axi_araddr(c3_axi_araddr), .m_axi_arlen(c3_axi_arlen),
    .m_axi_arsize(c3_axi_arsize), .m_axi_arburst(c3_axi_arburst), .m_axi_arcache(c3_axi_arcache),
    .m_axi_arprot(c3_axi_arprot), .m_axi_arqos(c3_axi_arqos),
    .m_axi_arvalid(c3_axi_arvalid), .m_axi_arready(c3_axi_arready),
    .m_axi_rid(c3_axi_rid), .m_axi_rdata(c3_axi_rdata), .m_axi_rresp(c3_axi_rresp),
    .m_axi_rlast(c3_axi_rlast), .m_axi_rvalid(c3_axi_rvalid), .m_axi_rready(c3_axi_rready)
  );

  // ---------------------------------------------------------------- MIG C4 ---
  logic c4_ui_clk, c4_ui_rst, c4_axi_rstn;
  assign c4_axi_rstn = ~c4_ui_rst;
`include "ng_mig_c4.vh"
  ng_axi_cc #(.ID_W(M_AXI_C4_ID_W), .ADDR_W(M_AXI_C4_ADDR_W), .DATA_W(M_AXI_C4_DATA_W)) u_cc_c4 (
    .s_aclk(clk_l2_s), .s_aresetn(rst_l2_s_n),
    .s_axi_awid(sh_m_axi_c4_awid), .s_axi_awaddr(sh_m_axi_c4_awaddr), .s_axi_awlen(sh_m_axi_c4_awlen),
    .s_axi_awsize(sh_m_axi_c4_awsize), .s_axi_awburst(sh_m_axi_c4_awburst), .s_axi_awcache(sh_m_axi_c4_awcache),
    .s_axi_awprot(sh_m_axi_c4_awprot), .s_axi_awqos(sh_m_axi_c4_awqos),
    .s_axi_awvalid(sh_m_axi_c4_awvalid), .s_axi_awready(sh_m_axi_c4_awready),
    .s_axi_wdata(sh_m_axi_c4_wdata), .s_axi_wstrb(sh_m_axi_c4_wstrb), .s_axi_wlast(sh_m_axi_c4_wlast),
    .s_axi_wvalid(sh_m_axi_c4_wvalid), .s_axi_wready(sh_m_axi_c4_wready),
    .s_axi_bid(sh_m_axi_c4_bid), .s_axi_bresp(sh_m_axi_c4_bresp), .s_axi_bvalid(sh_m_axi_c4_bvalid), .s_axi_bready(sh_m_axi_c4_bready),
    .s_axi_arid(sh_m_axi_c4_arid), .s_axi_araddr(sh_m_axi_c4_araddr), .s_axi_arlen(sh_m_axi_c4_arlen),
    .s_axi_arsize(sh_m_axi_c4_arsize), .s_axi_arburst(sh_m_axi_c4_arburst), .s_axi_arcache(sh_m_axi_c4_arcache),
    .s_axi_arprot(sh_m_axi_c4_arprot), .s_axi_arqos(sh_m_axi_c4_arqos),
    .s_axi_arvalid(sh_m_axi_c4_arvalid), .s_axi_arready(sh_m_axi_c4_arready),
    .s_axi_rid(sh_m_axi_c4_rid), .s_axi_rdata(sh_m_axi_c4_rdata), .s_axi_rresp(sh_m_axi_c4_rresp),
    .s_axi_rlast(sh_m_axi_c4_rlast), .s_axi_rvalid(sh_m_axi_c4_rvalid), .s_axi_rready(sh_m_axi_c4_rready),
    .m_aclk(c4_ui_clk), .m_aresetn(c4_axi_rstn),
    .m_axi_awid(c4_axi_awid), .m_axi_awaddr(c4_axi_awaddr), .m_axi_awlen(c4_axi_awlen),
    .m_axi_awsize(c4_axi_awsize), .m_axi_awburst(c4_axi_awburst), .m_axi_awcache(c4_axi_awcache),
    .m_axi_awprot(c4_axi_awprot), .m_axi_awqos(c4_axi_awqos),
    .m_axi_awvalid(c4_axi_awvalid), .m_axi_awready(c4_axi_awready),
    .m_axi_wdata(c4_axi_wdata), .m_axi_wstrb(c4_axi_wstrb), .m_axi_wlast(c4_axi_wlast),
    .m_axi_wvalid(c4_axi_wvalid), .m_axi_wready(c4_axi_wready),
    .m_axi_bid(c4_axi_bid), .m_axi_bresp(c4_axi_bresp), .m_axi_bvalid(c4_axi_bvalid), .m_axi_bready(c4_axi_bready),
    .m_axi_arid(c4_axi_arid), .m_axi_araddr(c4_axi_araddr), .m_axi_arlen(c4_axi_arlen),
    .m_axi_arsize(c4_axi_arsize), .m_axi_arburst(c4_axi_arburst), .m_axi_arcache(c4_axi_arcache),
    .m_axi_arprot(c4_axi_arprot), .m_axi_arqos(c4_axi_arqos),
    .m_axi_arvalid(c4_axi_arvalid), .m_axi_arready(c4_axi_arready),
    .m_axi_rid(c4_axi_rid), .m_axi_rdata(c4_axi_rdata), .m_axi_rresp(c4_axi_rresp),
    .m_axi_rlast(c4_axi_rlast), .m_axi_rvalid(c4_axi_rvalid), .m_axi_rready(c4_axi_rready)
  );

  // ------------------------------------------------------------------ BSCAN --
  ng_bscan_status #(
    .SHELL_REV (SHELL_REV),
    .SHELL_UID (SHELL_UID)
  ) u_bscan_status (
    .si_done          (si_done),
    .si_err           (si_err),
    .mmcm_locked      (mmcm_locked),
    .fence_stuck      (fence_stuck),
    .link_up          (link_up_q),
    .ltssm_state      (ltssm),
    .mem_calib        (mem_calib),
    .msi_enable       (msi_enable),
    .msi_vector_width (msi_vw),
    .pr_dbg           (pr_dbg)
  );

  // USER3: raw tunnel to the RM debug-hub port
  BSCANE2 #(.JTAG_CHAIN(3)) u_bscan_hub (
    .CAPTURE (sh_dbg_hub_capture),
    .DRCK    (sh_dbg_hub_drck),
    .RESET   (sh_dbg_hub_reset),
    .RUNTEST (sh_dbg_hub_runtest),
    .SEL     (sh_dbg_hub_sel),
    .SHIFT   (sh_dbg_hub_shift),
    .TCK     (sh_dbg_hub_tck),
    .TDI     (sh_dbg_hub_tdi),
    .TMS     (sh_dbg_hub_tms),
    .UPDATE  (sh_dbg_hub_update),
    .TDO     (sh_dbg_hub_tdo)
  );
  assign sh_dbg_hub_bscanid_en = 1'b0;  // BSCANE2 has no id-enable output

  // USER4: raw tunnel to the RM observation chain
  BSCANE2 #(.JTAG_CHAIN(4)) u_bscan_obs (
    .CAPTURE (sh_dbg_obs_capture),
    .DRCK    (),
    .RESET   (),
    .RUNTEST (),
    .SEL     (sh_dbg_obs_sel),
    .SHIFT   (sh_dbg_obs_shift),
    .TCK     (sh_dbg_obs_tck),
    .TDI     (sh_dbg_obs_tdi),
    .TMS     (),
    .UPDATE  (sh_dbg_obs_update),
    .TDO     (sh_dbg_obs_tdo)
  );

endmodule
