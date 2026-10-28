// ng_gpu_rm_sp.sv -- M1 reconfigurable module (rm_kind 0x8003): the CP RM (rm_cp,: Ibex + boot loader
// + copy engine + register block + GPU MMU) plus ONE shader processor (rtl/gpu/sp ng_sp_top + ng_wd + L0 I$, LSU,
// register file, scheduler, VALU, SALU; SFU + XLANE instantiated, FP64 not: SP_EN_* below, published in CAPS[27:24]) behind the DFX boundary.
//
// Same port list as every RM (ng_gpu_rm_ports.svh); a different RM variant of module ng_gpu_rm.  Design, CDC, ABI and
// the debug-page register map:  The rm_cp helper modules (rtl/shell/rm_cp/ng_rcp_*.sv) are used by
// reference, unchanged; the BAR0 front end is ng_rcp_regs with its CAPS and DBG_PAGE parameters (a modified copy,
// ng_rsp_regs, before).
//   clk_l2   boundary side: BAR0 registers, BAR2 C3 window, C3 / host engines, quiesce / ready, GPU MMU, and the
//            memory side of the SP bridges: ng_rsp_fab_l2 (line request -> 64-bit operations), ng_rsp_mmu_l2
//            (translation), ng_rsp_flt_l2 (LSU BAD_PHYS report); arbiters ng_rsp_mmuarb (MMU: CP / SP),
//            ng_rsp_oparb (C3 engine requester 1: CP / SP, host engine: CP / SP), round-robin
//   clk_ts   CP (99.94 MHz) and the SP on the same clock: the CP work-distributor port (wd_*) connects directly;
//            SP fabric / MMU / fault ports cross to clk_l2 with toggle handshakes (ng_rsp_*_ts)
// No MMCM / BUFG in the RM (DFX_BOUNDARY.md 2.2): the SP runs on the boundary's clk_ts.
// SP_PRESENT = 0 ties the SP off (wd_done = wd_start as rm_cp, sp_idle = 1, no bridges) -- the rm_cp behaviour with the
// rm_sp identity, for bisecting on the board.
module ng_gpu_rm
  import ng_dfx_pkg::*;
  import ng_cp_pkg::*;
#(
  parameter logic [31:0] BUILD_ID   = 32'h0000_0000,
  parameter logic [15:0] RM_REV     = 16'h0001,
  parameter bit          SP_PRESENT = 1'b1,
  parameter bit          SP_BURST   = 1'b1,      // line RD / WR as one AXI burst (0 =-bit operations)
  parameter bit          SP_HOST_WR_FLUSH = 1'b0, // flush read after each host write burst (see ng_rsp_fab_l2)
  parameter int          SP_NOUT    = 8,         // fabric request slots (1 = one request at a time, as)
  parameter int          SP_MEM_OUT = SP_NOUT,   // requests the SP may keep in flight: 1 = the serial port
                                                 // through the MEM_OUT = 1 adapter ng_rsp_mport1 (the 26c1 RM), or
                                                 // SP_NOUT = ng_sp_top's slot port straight onto ng_rsp_fab_ts
  parameter int          SP_LSU_K   = 4,         // LSU instruction contexts (1 = ng_lsu, the pre-26c3 LSU
                                                 // cycle for cycle with SP_MEM_OUT = 1; 2 / 4 / 8 = ng_lsu_kn)
  parameter bit          SP_EN_XLANE = 1'b1,     // XLANE (shfl / quad / vote / scan / reduce / derivatives, CAPS[25]);
                                                 // 0 = the previous SP (build_rm.tcl NG_RM_GENERICS="SP_EN_XLANE=0")
  parameter int          SP_SH_LPB  = 4,         // R3 (b20): shared-memory lanes per round (ng_sp_top default 8; 32 = one round)
  parameter int          SP_L1_LPB  = 2,         // R3 (b20): global-path scatter / merge lanes per beat (ng_sp_top default 4)
  parameter bit          SP_WR_RUNS = 1'b1,      // host partial / holey line writes as bursts of runs
                                                 // (0 = the rule: operation path, FAB_WR_FALLBACK)
  parameter int unsigned ABORT_WDOG_LOG2 = 19    // WDIST abort watchdog 2**N clk_ts (19: ~5.2 ms
                                                 //  a small value builds the timeout-path debug RM)
) (
`include "ng_gpu_rm_ports.svh"
);
  localparam logic [15:0] RM_KIND_SP = 16'h8003;
  localparam logic [3:0]  MEM_MASK   = 4'b0100;       // C3 only
  localparam int          QAGE       = 20;            // > the round trip of the quiesce / idle paths
  localparam int          FSW        = ng_noc_pkg::clog2_1(SP_NOUT);           // fabric slot (tag) width
  localparam int          FHW        = FSW + 124;                              // fabric head FIFO entry
  // Optional SP execution units of this build (the ng_sp_top parameters below); also published in CAPS[26:24]
  localparam bit          SP_EN_SFU   = 1'b1;
  localparam bit          SP_EN_FP64  = 1'b0;
  localparam bit          SP_EN_F16CVT = 1'b1;   // VALU binary16 <-> binary32 conversions (CAPS[29])
  // CAPS: [3:0] cores 1, [7:4] SP per core 1, [15:8] warp size 32, [23:16] VMIDs 16,
  // [24] EN_SFU, [25] EN_XLANE, [26] EN_FP64 (the SP's unit parameters; 0 when the SP is not instantiated),
  // [27] UNITS_VALID = 1 (this RM reports [26:24]; an RM that leaves bit 27 clear predates the unit bits, so
  // [26:24] = 0 there means "unknown", not "no optional units"),
  // [28] SCRATCH_ILV (ISA ERRATA v0.1.6): lane scratch is dword-interleaved (ng_lsu); 0 = an older RM with the
  // v0.1.2 linear layout.  Invisible to programs; only tools that read scratch memory directly need it.
  // [29] F16CVT (ruled): the VALU executes fcvt.f32.f16.lo/hi, fcvt.f16.f32.lo/hi(.m), pkcvt.f16.f32(.m)
  // (ng_sp_top EN_VALU_F16CVT); 0 = an RM whose VALU decodes them as illegal (b4 .. b13).  Read with UNITS_VALID.
  localparam logic [3:0]  CAPS_UNITS = {1'b1, SP_PRESENT & SP_EN_FP64, SP_PRESENT & SP_EN_XLANE, SP_PRESENT & SP_EN_SFU};
  localparam logic [31:0] RM_CAPS    = {2'h0, SP_PRESENT & SP_EN_F16CVT, SP_PRESENT, CAPS_UNITS, 24'h10_2011};

  wire clk = clk_l2;
  if (SP_NOUT != 1 && SP_NOUT != 2 && SP_NOUT != 4 && SP_NOUT != 8 && SP_NOUT != 16) begin : g_bad_nout
    $error("SP_NOUT must be 1, 2, 4, 8 or 16");
  end
  if (SP_MEM_OUT != 1 && SP_MEM_OUT != SP_NOUT) begin : g_bad_mem_out
    $error("SP_MEM_OUT must be 1 or SP_NOUT");
  end
  if (SP_LSU_K != 1 && SP_LSU_K != 2 && SP_LSU_K != 4 && SP_LSU_K != 8) begin : g_bad_lsu_k
    $error("SP_LSU_K must be 1, 2, 4 or 8");
  end
  localparam int          SMO        = (SP_MEM_OUT == 1) ? 1 : SP_NOUT;          // ng_sp_top MEM_OUT
  localparam int          SMW        = ng_noc_pkg::clog2_1(SMO);                 // its slot width (= FSW for SMO > 1)
  (* keep = "true", shreg_extract = "no" *) logic rst_n;
  always_ff @(posedge clk) rst_n <= rst_l2_n;

  assign rm_info = {12'h000, MEM_MASK, RM_REV, BUILD_ID, RM_KIND_SP, NG_DFX_IFACE_VERSION, NG_RM_INFO_MAGIC};

  // ------------------------------------------------------------------ BAR0 registers + CP bridge
  logic        win_en;
  logic [4:0]  win_log2;
  logic [30:0] win_base;
  logic        cp_req, cp_wr, cp_ack_ts;
  logic [12:0] cp_addr;
  logic [31:0] cp_wdata, cp_rdata;
  (* ASYNC_REG = "TRUE" *) logic ack_s1, ack_s2;
  logic        csr_wr;
  logic [11:0] csr_waddr, csr_raddr;
  logic [31:0] csr_wdata, csr_rdata;
  logic [31:0] l2dbg [16];
  always_ff @(posedge clk) begin ack_s1 <= cp_ack_ts; ack_s2 <= ack_s1; end

  ng_rcp_regs #(.RM_REV(RM_REV), .CAPS(RM_CAPS), .DBG_PAGE(1'b1)) u_regs (
    .clk(clk), .rst_n(rst_n),
    .s_awaddr(s_axil_ctrl_awaddr), .s_awvalid(s_axil_ctrl_awvalid), .s_awready(s_axil_ctrl_awready),
    .s_wdata(s_axil_ctrl_wdata), .s_wstrb(s_axil_ctrl_wstrb), .s_wvalid(s_axil_ctrl_wvalid),
    .s_wready(s_axil_ctrl_wready), .s_bvalid(s_axil_ctrl_bvalid), .s_bready(s_axil_ctrl_bready),
    .s_araddr(s_axil_ctrl_araddr), .s_arvalid(s_axil_ctrl_arvalid), .s_arready(s_axil_ctrl_arready),
    .s_rdata(s_axil_ctrl_rdata), .s_rvalid(s_axil_ctrl_rvalid), .s_rready(s_axil_ctrl_rready),
    .win_en(win_en), .win_log2(win_log2), .win_base(win_base),
    .cp_req(cp_req), .cp_wr(cp_wr), .cp_addr(cp_addr), .cp_wdata(cp_wdata), .cp_ack(ack_s2), .cp_rdata(cp_rdata),
    .csr_wr(csr_wr), .csr_waddr(csr_waddr), .csr_wdata(csr_wdata), .csr_raddr(csr_raddr), .csr_rdata(csr_rdata),
    .l2dbg(l2dbg));
  assign s_axil_ctrl_bresp = 2'b00;
  assign s_axil_ctrl_rresp = 2'b00;

  // ------------------------------------------------------------------ quiesce / ready
  logic        q_in;
  logic [5:0]  q_age;
  logic [3:0]  rdy_cnt;
  logic        c3_idle, host_idle, hbm_idle, c3bm_idle;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rm_ready <= 1'b0; rm_quiesce_ack <= 1'b0; q_in <= 1'b0; q_age <= '0; rdy_cnt <= '0;
    end else begin
      if (rdy_cnt != 4'd10) rdy_cnt <= rdy_cnt + 1'b1;
      rm_ready <= (rdy_cnt == 4'd10);
      q_in     <= shell_quiesce_req;
      if (!q_in)                  q_age <= '0;
      else if (q_age != 6'(QAGE)) q_age <= q_age + 1'b1;
      rm_quiesce_ack <= q_in && (q_age == 6'(QAGE)) && c3_idle && host_idle && hbm_idle && c3bm_idle;
    end
  end

  // ------------------------------------------------------------------ CP (clk_ts)
  logic        host_wr, host_rd;
  logic [12:0] host_addr;
  logic [31:0] host_wdata, host_rdata, cp_host_rdata, dbg_rdata;
  logic        m_req_valid, m_req_ready, m_rsp_valid, m_rsp_fault;
  mem_req_t    m_req;
  logic [63:0] m_rsp_data;
  logic        wd_start, wd_done, wd_fault, sp_idle, cp_irq, core_reset, cpu_quiet, mmu_irq_ts, wd_abort;
  logic        fb_busy;      // SP fabric slots allocated (ng_rsp_fab_ts) -> ng_cp_port abort wait
  logic [63:0] wd_instr;
  wd_req_t     wd_req;
  logic        dbg_valid;
  logic [31:0] dbg_code;
  logic        dbg_sel;      // host_addr in the clk_ts debug page (0x0800..0x08BF + 0x0900..0x093F,
                             // + 0x0940..0x095F + 0x0960..0x097F + 0x0980..0x09BF)

  // bulk copy: CP Copy engine -> memif (one handshake per command) -> ng_rcp_dma -> engine port 1
  logic        cb_valid, cb_fill, cb_abort, cb_done, cb_fault;
  logic [3:0]  cb_vmid;
  logic [47:0] cb_src, cb_dst, x_src;
  logic [63:0] cb_size, x_size;
  logic [31:0] cb_fillv, x_fillv;
  logic        x_bulk, x_fill, x_abort;
  logic        dh_awvalid, dh_awready, dh_wvalid, dh_wready, dh_wlast, dh_bvalid, dh_bready, dh_arvalid,
               dh_arready, dh_rvalid, dh_rready, dh_rlast;
  logic [46:0] dh_awaddr, dh_araddr;
  logic [7:0]  dh_awlen, dh_arlen;
  logic [63:0] dh_wdata, dh_rdata;
  logic [7:0] dh_wstrb;
  logic [1:0]  dh_bresp, dh_rresp;
  logic        dc_awvalid, dc_awready, dc_wvalid, dc_wready, dc_wlast, dc_bvalid, dc_bready, dc_arvalid,
               dc_arready, dc_rvalid, dc_rready, dc_rlast;
  logic [30:0] dc_awaddr, dc_araddr;
  logic [7:0]  dc_awlen, dc_arlen;
  logic [127:0] dc_wdata, dc_rdata;
  logic [15:0] dc_wstrb;
  logic [1:0]  dc_bresp, dc_rresp;

  // the SP fabric bursts (ng_rsp_fab_l2 sbh_* / sbc_*) share the engines' burst port with the DMA (dh_* / dc_*)
  // through one ng_rcp_axmux per engine; mh_* / mc_* are the muxed ports into ng_rcp_hosteng / ng_rcp_c3eng b_*
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

  // 99.94 MHz: tick just under 1 ms; COPY / FILL go to the clk_l2 DMA as whole commands
  // ABORT_FAB_WAIT -- WDIST_STATUS.ABORTING lasts until the fabric slots drained (fb_busy) or the watchdog
  // (2**ABORT_WDOG_LOG2 clk_ts) expires
  // every MMU CTX_CTRL write (BAR0 0x0200 + 0x10 n + 8 = ctx_enable / ctx_disable of VMID n != 0, the
  // driver does one per vm_create / vm_destroy) may change the owner of VMID n: a toggle per VMID in clk_l2, crossed to
  // clk_ts with two flops and edge-detected into a one-cycle pulse for ng_cp_port (scr_pend, sticky until the VMID's next
  // dispatch).  Two writes of one VMID closer than ~3 clk_ts cycles cannot occur (BAR0 write rate).
  logic [15:1]   scr_tog;
  (* ASYNC_REG = "TRUE" *) logic [15:1] scr_s1, scr_s2;
  logic [15:1]   scr_s3;
  logic [2:0]    sp_inv;                                         // CACHE packet invalidates (ng_cp_port P_SP_INV)
  logic [15:0]   scrub_set_ts;                                   // MMU CTX_CTRL write -> owner change (clk_ts pulses)
  always_ff @(posedge clk) begin
    if (!rst_n) scr_tog <= '0;
    else for (int n = 1; n < 16; n++)
      if (csr_wr && csr_waddr[11:8] == 4'h2 && csr_waddr[3:2] == 2'd2 && csr_waddr[7:4] == 4'(n)) scr_tog[n] <= ~scr_tog[n];
  end
  always_ff @(posedge clk_ts) begin
    scr_s1 <= scr_tog; scr_s2 <= scr_s1; scr_s3 <= scr_s2;
  end
  assign scrub_set_ts = {scr_s2 ^ scr_s3, 1'b0};

  ng_cp_top #(.COPY_BULK(1'b1), .BOOT_LOADER(1'b1), .IBEX_IMAGE(""), .TICK_CYCLES(99_000), .ABORT_FAB_WAIT(1'b1),
              .ABORT_WDOG_LOG2(ABORT_WDOG_LOG2), .SCRUB(SP_PRESENT)) u_cp (
    .clk(clk_ts), .rst_n(rst_ts_n),
    .host_wr(host_wr && !dbg_sel), .host_rd(host_rd && !dbg_sel), .host_addr(host_addr), .host_wdata(host_wdata),
    .host_rdata(cp_host_rdata),
    .irq_ext_i({29'h0, mmu_irq_ts, 2'b00}), .rm_irq_o(cp_irq),
    .m_req_valid(m_req_valid), .m_req_ready(m_req_ready), .m_req(m_req),
    .m_rsp_valid(m_rsp_valid), .m_rsp_fault(m_rsp_fault), .m_rsp_data(m_rsp_data),
    .cb_valid(cb_valid), .cb_fill(cb_fill), .cb_vmid(cb_vmid), .cb_src(cb_src), .cb_dst(cb_dst),
    .cb_size(cb_size), .cb_fillv(cb_fillv), .cb_abort(cb_abort), .cb_done(cb_done), .cb_fault(cb_fault),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .sp_idle_i(sp_idle), .wd_abort_o(wd_abort), .fab_busy_i(fb_busy),
    .scrub_set_i(scrub_set_ts), .sp_inv_o(sp_inv),
    .dbg_valid_o(dbg_valid), .dbg_code_o(dbg_code), .cpu_quiet_o(cpu_quiet), .core_reset_o(core_reset));

  // BAR0 0x0800..0x08BF, 0x0900..0x093F, 0x0940..0x095F, 0x0960..0x097F  and 0x0980..0x09BF
  // the clk_ts debug page instead of the CP block (same split as ng_rcp_regs is_cp; 0x09C0..0x09FF never
  // reach the CP bridge)
  assign dbg_sel = ((host_addr[12:8] == 5'h08) && (host_addr[7:6] != 2'b11)) ||
                   ((host_addr[12:8] == 5'h09) && (host_addr[7:6] != 2'b11));
  assign host_rdata = dbg_sel ? dbg_rdata : cp_host_rdata;

  logic cpb_ack;
  ng_rcp_cpbus u_cpbus (
    .clk(clk_ts), .rst_n(rst_ts_n),
    .req_tog_i(cp_req), .wr_i(cp_wr), .addr_i(cp_addr), .wdata_i(cp_wdata),
    .ack_tog_o(cpb_ack), .rdata_o(cp_rdata),
    .host_wr(host_wr), .host_rd(host_rd), .host_addr(host_addr), .host_wdata(host_wdata), .host_rdata(host_rdata));
  assign cp_ack_ts = cpb_ack;

  // CP interrupt level -> clk_l2
  (* ASYNC_REG = "TRUE" *) logic irq_s1, irq_s2;
  always_ff @(posedge clk) begin irq_s1 <= cp_irq; irq_s2 <= irq_s1; end
  assign rm_irq = {3'b000, irq_s2};

  // ------------------------------------------------------------------ SP (clk_ts)
  logic          s_mmu_req_valid, s_mmu_req_ready, s_mmu_req_atomic, s_mmu_resp_valid, s_mmu_resp_ready;
  logic          s_mmu_resp_sparse;
  logic [3:0]    s_mmu_req_vmid;
  logic [63:0]   s_mmu_req_va, s_mmu_resp_pte;
  logic [1:0]    s_mmu_req_acc;
  logic [7:0]    s_mmu_req_tag;
  logic [2:0]    s_mmu_resp_fault;
  logic [47:0]   s_mmu_resp_pa;
  logic          s_flt_valid;
  logic [3:0]    s_flt_vmid;
  logic [63:0]   s_flt_va;
  logic [1:0]    s_flt_acc;
  logic          s_mem_req_valid, s_mem_req_ready, s_mem_rsp_valid;
  ng_lsu_pkg::mreq_t s_mem_req;
  ng_lsu_pkg::mrsp_t s_mem_rsp;
  logic [SMW-1:0] s_req_slot, s_rsp_slot, s_rd_slot;              // slot port of ng_sp_top (SMO > 1)
  logic          s_rsp_rd, s_free;
  logic [1023:0] s_rd_data;
  logic          cm_valid, tr_valid;
  logic [3:0]    cm_slot, tr_slot, tr_cause;
  logic [31:0]   cm_pc, cm_w0, cm_exec, cm_wg, tr_pc, tr_info, tr_wg, ic_hit, ic_miss, sp_status;
  logic [31:0]   ls_l1_rd, ls_l1_wr, ls_instr_mem, ls_groups;   //
  logic [255:0]  sp_perf;                                        //
  logic [351:0]  sp_lsu;                                         // LSU counters (debug page 0x980..0x9A8)
  logic [159:0]  sp_scrub;                                       // counters (debug page 0x9AC..0x9BC)
  logic [4:0]    cm_warp, tr_warp;
  logic [2:0]    tr_unit;

  // bridge bundles.  Fabric: ng_sp_top -> ng_rsp_mport1 (MEM_OUT = 1) -> ng_rsp_fab_ts (f_*) -> CDC (fx_* / fy_*)
  // -> ng_rsp_fab_l2
  logic          f_req_valid, f_req_ready, f_rsp_valid, f_rsp_rd, f_free, f_sp_rst;
  ng_lsu_pkg::mreq_t f_req;
  logic [FSW-1:0] f_req_slot, f_rsp_slot, f_rd_slot;
  logic [1:0]    f_rsp_err;
  logic [63:0]   f_rsp_old;
  logic [1023:0] f_rd_data;
  logic [FSW:0]  fb_out, fb_max_out, fx_hgray, fy_cgray;
  logic [FSW-1:0] fx_hra, fy_cra, fy_rra, fy_ora;
  logic [FHW-1:0] fx_hd;
  logic [FSW+1:0] fx_wra, fy_cd;
  logic [143:0]  fx_wde, fx_wdo;
  logic [1023:0] fy_rdh, fy_rdc;
  logic [63:0]   fy_old;
  logic [31:0]   fb_orphans, fb_cstall, fb_sstall, fb_fences, fb_fstall;
  logic [31:0]   fb_reqs, fb_errs;
  logic [47:0]   fb_last_pa;
  logic [2:0]    fb_last_kind;
  logic [1:0]    fb_last_err;
  logic [47:0]   fb_rd_lat_sum, fb_wr_lat_sum;                  //
  logic [31:0]   fb_rd_lat_n, fb_rd_lat_max, fb_wr_lat_n, fb_wr_lat_max;
  logic          mb_req, mb_ack, mb_x_atomic, mb_y_sparse;
  logic [3:0]    mb_x_vmid;
  logic [63:0]   mb_x_va, mb_y_pte, mb_last_va;
  logic [1:0]    mb_x_acc, mb_st_ts, mb_st_l2;
  logic [2:0]    mb_y_fault, mb_last_fault;
  logic [47:0]   mb_y_pa;
  logic [31:0]   mb_reqs, mb_flts;
  logic          fl_tog, fl_ack, fl_last_valid;
  logic [3:0]    fl_x_vmid, fl_last_vmid;
  logic [63:0]   fl_x_va, fl_last_va;
  logic [1:0]    fl_x_acc, fl_last_acc;

  generate
    if (SP_PRESENT) begin : g_sp
      ng_sp_top #(.EN_SFU(SP_EN_SFU), .EN_XLANE(SP_EN_XLANE), .EN_FP64(SP_EN_FP64), .EN_VALU_FP(1'b1), .VALU_NFL(4), .EN_VALU_F16CVT(SP_EN_F16CVT), .SP_ID(0), .CORE_ID(0),
                  .GSLOT_BASE(0), .LSU_K(SP_LSU_K), .MEM_OUT(SMO), .SH_LPB(SP_SH_LPB), .L1_LPB(SP_L1_LPB)) u_sp (
        .clk(clk_ts), .rst_n(rst_ts_n), .core_reset(core_reset | wd_abort),   // WDIST_CTRL.ABORT resets the SP only
        .inv_l1_ext(sp_inv[0] | sp_inv[1]), .inv_i_ext(sp_inv[2]), .stat_scrub(sp_scrub),
        .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
        .sp_idle(sp_idle),
        .mmu_req_valid(s_mmu_req_valid), .mmu_req_ready(s_mmu_req_ready), .mmu_req_vmid(s_mmu_req_vmid),
        .mmu_req_va(s_mmu_req_va), .mmu_req_acc(s_mmu_req_acc), .mmu_req_atomic(s_mmu_req_atomic),
        .mmu_req_tag(s_mmu_req_tag), .mmu_resp_valid(s_mmu_resp_valid), .mmu_resp_ready(s_mmu_resp_ready),
        .mmu_resp_fault(s_mmu_resp_fault), .mmu_resp_sparse(s_mmu_resp_sparse), .mmu_resp_pa(s_mmu_resp_pa),
        .mmu_resp_pte(s_mmu_resp_pte),
        .ext_flt_valid(s_flt_valid), .ext_flt_vmid(s_flt_vmid), .ext_flt_va(s_flt_va), .ext_flt_acc(s_flt_acc),
        .mem_req_valid(s_mem_req_valid), .mem_req_ready(s_mem_req_ready), .mem_req(s_mem_req),
        .mem_rsp_valid(s_mem_rsp_valid), .mem_rsp(s_mem_rsp),
        .mem_req_slot(s_req_slot), .mem_rsp_slot(s_rsp_slot), .mem_rsp_rd(s_rsp_rd), .mem_rd_slot(s_rd_slot),
        .mem_rd_data(s_rd_data), .mem_free(s_free),
        .dbg_commit_valid(cm_valid), .dbg_commit_slot(cm_slot), .dbg_commit_pc(cm_pc), .dbg_commit_w0(cm_w0),
        .dbg_commit_exec(cm_exec), .dbg_commit_wg(cm_wg), .dbg_commit_warp(cm_warp),
        .dbg_trap_valid(tr_valid), .dbg_trap_unit(tr_unit), .dbg_trap_slot(tr_slot), .dbg_trap_pc(tr_pc),
        .dbg_trap_cause(tr_cause), .dbg_trap_info(tr_info), .dbg_trap_wg(tr_wg), .dbg_trap_warp(tr_warp),
        .stat_ic_hit(ic_hit), .stat_ic_miss(ic_miss), .stat_l1_rd(ls_l1_rd), .stat_l1_wr(ls_l1_wr),
        .stat_instr_mem(ls_instr_mem), .stat_groups(ls_groups), .stat_perf(sp_perf), .stat_lsu(sp_lsu),
        .dbg_status(sp_status));

      // sp_rst = everything that resets the SP datapath: CP CORE_RESET, WDIST abort and the WD's post-trap
      // flush (sp_status[31] = !crst_n of ng_sp_top, which covers core_reset and wd_flush)
      assign f_sp_rst = core_reset | wd_abort | sp_status[31];
      if (SMO == 1) begin : g_mp1
        // SP_MEM_OUT = 1: ng_sp_top's serial port through the MEM_OUT = 1 adapter (the 26c1 RM; with SP_LSU_K = 1 the
        // pre-26c3 SP cycle for cycle).  Its dummy answer after sp_rst also releases the 1-slot adapter of an
        // SP_LSU_K >= 2 SP (ng_sp_top g_ser: the answer of the orphaned request never comes)
        ng_rsp_mport1 #(.SW(FSW)) u_smport (
          .clk(clk_ts), .rst_n(rst_ts_n), .sp_rst(f_sp_rst),
          .sp_req_valid(s_mem_req_valid), .sp_req_ready(s_mem_req_ready), .sp_req(s_mem_req),
          .sp_rsp_valid(s_mem_rsp_valid), .sp_rsp(s_mem_rsp),
          .f_req_valid(f_req_valid), .f_req_ready(f_req_ready), .f_req(f_req),
          .f_rsp_valid(f_rsp_valid), .f_rsp_slot(f_rsp_slot), .f_rsp_err(f_rsp_err), .f_rsp_rd(f_rsp_rd),
          .f_rsp_old(f_rsp_old), .f_rd_slot(f_rd_slot), .f_rd_data(f_rd_data), .f_free(f_free));
        assign s_req_slot = '0; assign s_rsp_slot = '0; assign s_rsp_rd = 1'b0; assign s_rd_data = '0;
      end else begin : g_mpn
        // ng_sp_top's mport (I$ / L1D / LSU uc channel, owner table) speaks the slot protocol of fab_ts
        // directly; the line stays in rbuf until the SP frees the slot (no dummy answer needed: the owner table is
        // cleared by the SP's datapath reset)
        assign f_req_valid     = s_mem_req_valid;
        assign s_mem_req_ready = f_req_ready;
        assign f_req           = s_mem_req;
        assign s_req_slot      = f_req_slot;
        assign s_mem_rsp_valid = f_rsp_valid;
        assign s_rsp_slot      = f_rsp_slot;
        assign s_rsp_rd        = f_rsp_rd;
        always_comb begin
          s_mem_rsp       = '0;
          s_mem_rsp.err   = f_rsp_err;
          s_mem_rsp.old   = f_rsp_old;
        end
        assign f_rd_slot       = s_rd_slot;
        assign s_rd_data       = f_rd_data;
        assign f_free          = s_free;
      end

      ng_rsp_fab_ts #(.NOUT(SP_NOUT)) u_sfab_ts (
        .clk(clk_ts), .rst_n(rst_ts_n), .sp_rst(f_sp_rst),
        .mem_req_valid(f_req_valid), .mem_req_ready(f_req_ready), .mem_req(f_req), .mem_req_slot(f_req_slot),
        .mem_rsp_valid(f_rsp_valid), .mem_rsp_slot(f_rsp_slot), .mem_rsp_err(f_rsp_err), .mem_rsp_rd(f_rsp_rd),
        .mem_rsp_old(f_rsp_old), .mem_rd_slot(f_rd_slot), .mem_rd_data(f_rd_data), .mem_free(f_free),
        .out_cnt(fb_out), .fab_busy(fb_busy),
        .x_hgray(fx_hgray), .x_hra(fx_hra), .x_hd(fx_hd), .x_wra(fx_wra), .x_wde(fx_wde), .x_wdo(fx_wdo),
        .y_cgray(fy_cgray), .y_cra(fy_cra), .y_cd(fy_cd), .y_rra(fy_rra), .y_rdh(fy_rdh), .y_rdc(fy_rdc),
        .y_ora(fy_ora), .y_old(fy_old),
        .n_req(fb_reqs), .n_err(fb_errs), .last_pa(fb_last_pa), .last_kind(fb_last_kind), .last_err(fb_last_err),
        .rd_lat_sum(fb_rd_lat_sum), .rd_lat_n(fb_rd_lat_n), .rd_lat_max(fb_rd_lat_max),
        .wr_lat_sum(fb_wr_lat_sum), .wr_lat_n(fb_wr_lat_n), .wr_lat_max(fb_wr_lat_max),
        .max_out(fb_max_out), .n_orphan(fb_orphans), .n_cstall(fb_cstall), .n_sstall(fb_sstall),
        .n_fence(fb_fences), .n_fstall(fb_fstall));

      ng_rsp_mmu_ts u_smmu_ts (
        .clk(clk_ts), .rst_n(rst_ts_n),
        .req_valid(s_mmu_req_valid), .req_ready(s_mmu_req_ready), .req_vmid(s_mmu_req_vmid), .req_va(s_mmu_req_va),
        .req_acc(s_mmu_req_acc), .req_atomic(s_mmu_req_atomic),
        .resp_valid(s_mmu_resp_valid), .resp_ready(s_mmu_resp_ready), .resp_fault(s_mmu_resp_fault),
        .resp_sparse(s_mmu_resp_sparse), .resp_pa(s_mmu_resp_pa), .resp_pte(s_mmu_resp_pte),
        .req_tog_o(mb_req), .x_vmid(mb_x_vmid), .x_va(mb_x_va), .x_acc(mb_x_acc), .x_atomic(mb_x_atomic),
        .ack_tog_i(mb_ack), .y_fault(mb_y_fault), .y_sparse(mb_y_sparse), .y_pa(mb_y_pa), .y_pte(mb_y_pte),
        .dbg_st(mb_st_ts), .n_req(mb_reqs), .n_flt(mb_flts), .last_va(mb_last_va), .last_fault(mb_last_fault));

      ng_rsp_flt_ts u_sflt_ts (
        .clk(clk_ts), .rst_n(rst_ts_n),
        .flt_valid(s_flt_valid), .flt_vmid(s_flt_vmid), .flt_va(s_flt_va), .flt_acc(s_flt_acc),
        .tog_o(fl_tog), .x_vmid(fl_x_vmid), .x_va(fl_x_va), .x_acc(fl_x_acc), .ack_tog_i(fl_ack),
        .last_valid(fl_last_valid), .last_va(fl_last_va), .last_vmid(fl_last_vmid), .last_acc(fl_last_acc));
    end else begin : g_no_sp
      // no shader processor: a DISPATCH is acknowledged as done at once (rm_cp behaviour)
      always_ff @(posedge clk_ts or negedge rst_ts_n) begin
        if (!rst_ts_n) wd_done <= 1'b0;
        else           wd_done <= wd_start;
      end
      assign wd_fault = 1'b0; assign wd_instr = '0; assign sp_idle = 1'b1;
      assign cm_valid = 1'b0; assign cm_slot = '0; assign cm_pc = '0; assign cm_w0 = '0; assign cm_exec = '0;
      assign cm_wg = '0; assign cm_warp = '0;
      assign tr_valid = 1'b0; assign tr_unit = '0; assign tr_slot = '0; assign tr_pc = '0; assign tr_cause = '0;
      assign tr_info = '0; assign tr_wg = '0; assign tr_warp = '0; assign ic_hit = '0; assign ic_miss = '0;
      assign ls_l1_rd = '0; assign ls_l1_wr = '0; assign ls_instr_mem = '0; assign ls_groups = '0;
      assign sp_perf = '0;
      assign sp_lsu = '0;
      assign sp_scrub = '0;
      assign sp_status = 32'h4000_0000;
      assign s_mmu_req_tag = '0;
      assign fb_busy = 1'b0; assign fb_reqs = '0; assign fb_errs = '0; assign fb_out = '0; assign fb_max_out = '0;
      assign fx_hgray = '0; assign fx_hd = '0; assign fx_wde = '0; assign fx_wdo = '0;
      assign fy_cra = '0; assign fy_rra = '0; assign fy_ora = '0; assign f_req_slot = '0; assign f_sp_rst = 1'b0;
      assign fb_orphans = '0; assign fb_cstall = '0; assign fb_sstall = '0; assign fb_fences = '0; assign fb_fstall = '0;
      assign fb_last_pa = '0; assign fb_last_kind = '0; assign fb_last_err = '0;
      assign fb_rd_lat_sum = '0; assign fb_rd_lat_n = '0; assign fb_rd_lat_max = '0;
      assign fb_wr_lat_sum = '0; assign fb_wr_lat_n = '0; assign fb_wr_lat_max = '0;
      assign mb_req = 1'b0; assign mb_x_vmid = '0; assign mb_x_va = '0; assign mb_x_acc = '0; assign mb_x_atomic = 1'b0;
      assign mb_st_ts = '0; assign mb_reqs = '0; assign mb_flts = '0; assign mb_last_va = '0; assign mb_last_fault = '0;
      assign fl_tog = 1'b0; assign fl_x_vmid = '0; assign fl_x_va = '0; assign fl_x_acc = '0;
      assign fl_last_valid = 1'b0; assign fl_last_va = '0; assign fl_last_vmid = '0; assign fl_last_acc = '0;
    end
  endgenerate

  ng_rsp_dbg u_dbg (
    .clk(clk_ts), .rst_n(rst_ts_n), .addr(host_addr[8:0]), .rdata(dbg_rdata),
    .sp_status(sp_status), .wd_start(wd_start), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .commit_valid(cm_valid), .commit_slot(cm_slot), .commit_pc(cm_pc), .commit_wg(cm_wg), .commit_warp(cm_warp),
    .trap_valid(tr_valid), .trap_unit(tr_unit), .trap_slot(tr_slot), .trap_pc(tr_pc), .trap_cause(tr_cause),
    .trap_info(tr_info), .trap_wg(tr_wg), .trap_warp(tr_warp), .ic_hit(ic_hit), .ic_miss(ic_miss),
    .core_reset(core_reset),
    .fab_busy(fb_busy), .fab_reqs(fb_reqs), .fab_errs(fb_errs), .fab_last_pa(fb_last_pa),
    .fab_last_kind(fb_last_kind), .fab_last_err(fb_last_err),
    .mmu_st(mb_st_ts), .mmu_reqs(mb_reqs), .mmu_flts(mb_flts), .mmu_last_va(mb_last_va),
    .mmu_last_fault(mb_last_fault),
    .flt_valid(fl_last_valid), .flt_va(fl_last_va), .flt_vmid(fl_last_vmid), .flt_acc(fl_last_acc),
    .rd_lat_sum(fb_rd_lat_sum), .rd_lat_n(fb_rd_lat_n), .rd_lat_max(fb_rd_lat_max),
    .wr_lat_sum(fb_wr_lat_sum), .wr_lat_n(fb_wr_lat_n), .wr_lat_max(fb_wr_lat_max),
    .ls_l1_rd(ls_l1_rd), .ls_l1_wr(ls_l1_wr), .ls_instr_mem(ls_instr_mem), .ls_groups(ls_groups),
    .fab_nout(8'(SP_PRESENT ? SP_NOUT : 0)), .fab_out(8'(fb_out)), .fab_max_out(8'(fb_max_out)),
    .fab_orphans(fb_orphans), .fab_cstall(fb_cstall), .fab_sstall(fb_sstall), .fab_fences(fb_fences),
    .fab_fstall(fb_fstall), .sp_perf(sp_perf), .lsu_k(sp_lsu), .scrub_k(sp_scrub));

  // ------------------------------------------------------------------ CP memory port -> C3 / host
  logic        x_req, x_host, x_wr, x_ack, y_err;
  logic [3:0]  x_vmid;
  logic [47:0] x_addr;
  logic [46:0] o_addr;
  logic [7:0]  x_wstrb;
  logic [63:0] x_wdata, y_rdata;
  logic        ml_busy;
  logic        c1_valid, c1_ready, c1_rs_valid, c1_rs_err;
  logic [63:0] c1_rs_rdata;
  logic        h_valid, h_ready, h_rs_valid, h_rs_err;
  logic [63:0] h_rs_rdata;

  // ------------------------------------------------------------------ GPU MMU (clk_l2)
  logic        t_req_valid, t_req_ready, t_resp_valid, t_resp_ready, t_resp_sparse;   // CP client (memif_l2)
  logic [3:0]  t_req_vmid;
  logic [63:0] t_req_va, t_resp_pte;
  logic [1:0]  t_req_acc, t_resp_level;
  logic [2:0]  t_resp_fault, t_resp_size;
  logic [47:0] t_resp_pa;
  logic [7:0]  t_resp_tag;
  logic        mu_req_valid, mu_req_ready, mu_req_atomic, mu_resp_valid, mu_resp_ready;   // ng_mmu client port
  logic [3:0]  mu_req_vmid;
  logic [63:0] mu_req_va;
  logic [1:0]  mu_req_acc;
  logic        st_req_valid, st_req_ready, st_req_atomic, st_resp_valid, st_resp_ready;   // SP client (mmu_l2)
  logic [3:0]  st_req_vmid;
  logic [63:0] st_req_va;
  logic [1:0]  st_req_acc;
  logic        ef_valid, cp_ef_valid, sp_ef_take;
  logic [3:0]  ef_vmid, cp_ef_vmid;
  logic [63:0] ef_va, ef_pte, cp_ef_va, cp_ef_pte;
  logic [1:0]  ef_acc, ef_level, cp_ef_acc, cp_ef_level;
  logic [2:0]  ef_code, cp_ef_code;
  logic        fl_pend;
  logic        mm_rd_valid, mm_rd_ready, mm_rsp_valid, mm_rsp_err, mmu_irq;
  logic [47:0] mm_rd_addr;
  logic [63:0] mm_rsp_data;
  logic [31:0] mm_walks, mm_hits;
  logic        c2_valid, c2_ready, c2_rs_valid;
  logic [30:0] c2_addr;

  ng_rcp_memif_ts u_mts (
    .clk(clk_ts), .rst_n(rst_ts_n),
    .m_req_valid(m_req_valid), .m_req_ready(m_req_ready), .m_req(m_req),
    .m_rsp_valid(m_rsp_valid), .m_rsp_fault(m_rsp_fault), .m_rsp_data(m_rsp_data),
    .b_valid(cb_valid), .b_fill(cb_fill), .b_vmid(cb_vmid), .b_src(cb_src), .b_dst(cb_dst), .b_size(cb_size),
    .b_fillv(cb_fillv), .b_abort(cb_abort), .b_done(cb_done), .b_fault(cb_fault),
    .x_bulk(x_bulk), .x_fill(x_fill), .x_src(x_src), .x_size(x_size), .x_fillv(x_fillv), .x_abort(x_abort),
    .req_tog_o(x_req), .x_vmid(x_vmid), .x_host(x_host), .x_wr(x_wr), .x_addr(x_addr), .x_wstrb(x_wstrb), .x_wdata(x_wdata),
    .ack_tog_i(x_ack), .y_err(y_err), .y_rdata(y_rdata));

  ng_rcp_memif_l2 u_ml2 (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in),
    .req_tog_i(x_req), .x_vmid(x_vmid), .x_host(x_host), .x_wr(x_wr), .x_addr(x_addr), .x_wstrb(x_wstrb),
    .x_wdata(x_wdata), .x_bulk(x_bulk), .x_fill(x_fill), .x_src(x_src), .x_size(x_size), .x_fillv(x_fillv),
    .x_abort(x_abort),
    .ack_tog_o(x_ack), .y_err(y_err), .y_rdata(y_rdata), .busy(ml_busy), .o_addr(o_addr),
    .c_valid(c1_valid), .c_ready(c1_ready), .c_rs_valid(c1_rs_valid), .c_rs_err(c1_rs_err), .c_rs_rdata(c1_rs_rdata),
    .h_valid(h_valid), .h_ready(h_ready), .h_rs_valid(h_rs_valid), .h_rs_err(h_rs_err), .h_rs_rdata(h_rs_rdata),
    .t_req_valid(t_req_valid), .t_req_ready(t_req_ready), .t_req_vmid(t_req_vmid), .t_req_va(t_req_va),
    .t_req_acc(t_req_acc), .t_resp_valid(t_resp_valid), .t_resp_ready(t_resp_ready), .t_resp_fault(t_resp_fault),
    .t_resp_sparse(t_resp_sparse), .t_resp_pa(t_resp_pa), .t_resp_level(t_resp_level), .t_resp_pte(t_resp_pte),
    .ef_valid(cp_ef_valid), .ef_vmid(cp_ef_vmid), .ef_va(cp_ef_va), .ef_acc(cp_ef_acc), .ef_code(cp_ef_code),
    .ef_level(cp_ef_level), .ef_pte(cp_ef_pte),
    .d_h_awvalid(dh_awvalid), .d_h_awready(dh_awready), .d_h_awaddr(dh_awaddr), .d_h_awlen(dh_awlen),
    .d_h_wvalid(dh_wvalid), .d_h_wready(dh_wready), .d_h_wdata(dh_wdata), .d_h_wstrb(dh_wstrb), .d_h_wlast(dh_wlast),
    .d_h_bvalid(dh_bvalid), .d_h_bready(dh_bready), .d_h_bresp(dh_bresp), .d_h_arvalid(dh_arvalid),
    .d_h_arready(dh_arready), .d_h_araddr(dh_araddr), .d_h_arlen(dh_arlen), .d_h_rvalid(dh_rvalid),
    .d_h_rready(dh_rready), .d_h_rdata(dh_rdata), .d_h_rresp(dh_rresp), .d_h_rlast(dh_rlast),
    .d_c_awvalid(dc_awvalid), .d_c_awready(dc_awready), .d_c_awaddr(dc_awaddr), .d_c_awlen(dc_awlen),
    .d_c_wvalid(dc_wvalid), .d_c_wready(dc_wready), .d_c_wdata(dc_wdata), .d_c_wstrb(dc_wstrb), .d_c_wlast(dc_wlast),
    .d_c_bvalid(dc_bvalid), .d_c_bready(dc_bready), .d_c_bresp(dc_bresp), .d_c_arvalid(dc_arvalid),
    .d_c_arready(dc_arready), .d_c_araddr(dc_araddr), .d_c_arlen(dc_arlen), .d_c_rvalid(dc_rvalid),
    .d_c_rready(dc_rready), .d_c_rdata(dc_rdata), .d_c_rresp(dc_rresp), .d_c_rlast(dc_rlast));

  // ---- SP memory side (clk_l2)
  logic        sf_c_valid, sf_c_ready, sf_c_rs_valid, sf_h_valid, sf_h_ready, sf_h_rs_valid, sf_wr;
  logic [46:0] sf_addr;
  logic [7:0]  sf_wstrb;
  logic [63:0] sf_wdata;
  logic [2:0]  sf_st;
  logic [31:0] sf_ops, sf_bad, sf_bursts, sf_wfb;
  logic [47:0] sf_bad_pa;
  logic        e1_valid, e1_ready, e1_wr, eh_valid, eh_ready, eh_wr;
  logic [30:0] e1_addr;
  logic [46:0] eh_addr;
  logic [7:0]  e1_wstrb, eh_wstrb;
  logic [63:0] e1_wdata, eh_wdata;
  logic [1:0]  c3a_valid, c3a_ready, c3a_rs, ha_valid, ha_ready, ha_rs;
  logic [1:0]  ma_st, c3a_st, ha_st;
  logic [31:0] ma_gnt [2];
  logic [31:0] c3a_gnt [2];
  logic [31:0] ha_gnt [2];
  logic [1:0]  ma_req_valid, ma_req_ready, ma_resp_valid, ma_resp_ready;

  // (with SP_PRESENT = 0 the SP side never requests: the head FIFO pointer is constant)
  ng_rsp_fab_l2 #(.NOUT(SP_NOUT), .BURST(SP_BURST), .HOST_WR_FLUSH(SP_HOST_WR_FLUSH), .WR_RUNS(SP_WR_RUNS)) u_sfab_l2 (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in),
    .x_hgray(fx_hgray), .x_hra(fx_hra), .x_hd(fx_hd), .x_wra(fx_wra), .x_wde(fx_wde), .x_wdo(fx_wdo),
    .y_cgray(fy_cgray), .y_cra(fy_cra), .y_cd(fy_cd), .y_rra(fy_rra), .y_rdh(fy_rdh), .y_rdc(fy_rdc),
    .y_ora(fy_ora), .y_old(fy_old),
    .c_valid(sf_c_valid), .c_ready(sf_c_ready), .c_rs_valid(sf_c_rs_valid), .c_rs_err(c1_rs_err),
    .c_rs_rdata(c1_rs_rdata),
    .h_valid(sf_h_valid), .h_ready(sf_h_ready), .h_rs_valid(sf_h_rs_valid), .h_rs_err(h_rs_err),
    .h_rs_rdata(h_rs_rdata),
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

  // burst port of each engine = DMA (s0) + SP fabric (s1).  REG_A: AW / AR leave through a register slice (the
  // ready chain port -> engine mux -> this mux -> master would otherwise be combinational through both mux levels), so
  // DEPTH = the ng_rmt_port outstanding limit behind the engine (host 8, C3 16) + 2 slice entries; idle counts only
  // transactions that have left the slice.  The mux has no write-before-read ordering between its masters: the SP
  // completes a write line only after its last B (and ng_rsp_fab_ts orders same-line requests after it), the DMA
  // completes whole commands. the SP master keeps several bursts in flight; the mux FIFOs / port limits
  // back-pressure its address channels, its R / B are always accepted.
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

  ng_rsp_mmu_l2 u_smmu_l2 (
    .clk(clk), .rst_n(rst_n),
    .req_tog_i(mb_req), .x_vmid(mb_x_vmid), .x_va(mb_x_va), .x_acc(mb_x_acc), .x_atomic(mb_x_atomic),
    .ack_tog_o(mb_ack), .y_fault(mb_y_fault), .y_sparse(mb_y_sparse), .y_pa(mb_y_pa), .y_pte(mb_y_pte),
    .t_req_valid(st_req_valid), .t_req_ready(st_req_ready), .t_req_vmid(st_req_vmid), .t_req_va(st_req_va),
    .t_req_acc(st_req_acc), .t_req_atomic(st_req_atomic),
    .t_resp_valid(st_resp_valid), .t_resp_ready(st_resp_ready), .t_resp_fault(t_resp_fault),
    .t_resp_sparse(t_resp_sparse), .t_resp_pa(t_resp_pa), .t_resp_pte(t_resp_pte), .dbg_st(mb_st_l2));

  ng_rsp_flt_l2 u_sflt_l2 (
    .clk(clk), .rst_n(rst_n), .tog_i(fl_tog), .ack_tog_o(fl_ack), .pend(fl_pend), .take(sp_ef_take));

  // external fault report: the CP's BAD_PHYS pulse first, a pending SP report in a cycle without one
  assign sp_ef_take = fl_pend && !cp_ef_valid;
  assign ef_valid = cp_ef_valid || sp_ef_take;
  assign ef_vmid  = cp_ef_valid ? cp_ef_vmid  : fl_x_vmid;
  assign ef_va    = cp_ef_valid ? cp_ef_va    : fl_x_va;
  assign ef_acc   = cp_ef_valid ? cp_ef_acc   : fl_x_acc;
  assign ef_code  = cp_ef_valid ? cp_ef_code  : ng_mmu_pkg::F_BAD_PHYS;
  assign ef_level = cp_ef_valid ? cp_ef_level : 2'd0;
  assign ef_pte   = cp_ef_valid ? cp_ef_pte   : 64'h0;

  // MMU arbiter: requester 0 = CP (memif_l2), 1 = SP
  assign ma_req_valid  = {st_req_valid, t_req_valid};
  assign t_req_ready   = ma_req_ready[0];
  assign st_req_ready  = ma_req_ready[1];
  assign t_resp_valid  = ma_resp_valid[0];
  assign st_resp_valid = ma_resp_valid[1];
  assign ma_resp_ready = {st_resp_ready, t_resp_ready};
  ng_rsp_mmuarb u_marb (
    .clk(clk), .rst_n(rst_n),
    .req_valid(ma_req_valid), .req_ready(ma_req_ready), .req_vmid('{t_req_vmid, st_req_vmid}),
    .req_va('{t_req_va, st_req_va}), .req_acc('{t_req_acc, st_req_acc}), .req_atomic({st_req_atomic, 1'b0}),
    .resp_valid(ma_resp_valid), .resp_ready(ma_resp_ready),
    .m_req_valid(mu_req_valid), .m_req_ready(mu_req_ready), .m_req_vmid(mu_req_vmid), .m_req_va(mu_req_va),
    .m_req_acc(mu_req_acc), .m_req_atomic(mu_req_atomic), .m_resp_valid(mu_resp_valid), .m_resp_ready(mu_resp_ready),
    .dbg_st(ma_st), .grants(ma_gnt));

  // C3 engine requester 1: 0 = CP (memif_l2), 1 = SP (fab_l2)
  assign c3a_valid     = {sf_c_valid, c1_valid};
  assign c1_ready      = c3a_ready[0];
  assign sf_c_ready    = c3a_ready[1];
  assign c1_rs_valid   = c3a_rs[0];
  assign sf_c_rs_valid = c3a_rs[1];
  ng_rsp_oparb #(.AW(31)) u_c3arb (
    .clk(clk), .rst_n(rst_n),
    .op_valid(c3a_valid), .op_ready(c3a_ready), .op_wr({sf_wr, x_wr}),
    .op_addr('{o_addr[30:0], sf_addr[30:0]}), .op_wstrb('{x_wstrb, sf_wstrb}), .op_wdata('{x_wdata, sf_wdata}),
    .rs_valid(c3a_rs),
    .e_valid(e1_valid), .e_ready(e1_ready), .e_wr(e1_wr), .e_addr(e1_addr), .e_wstrb(e1_wstrb), .e_wdata(e1_wdata),
    .e_rs_valid(e_rs_valid[1]), .dbg_st(c3a_st), .grants(c3a_gnt));

  // host engine: 0 = CP, 1 = SP
  logic eh_rs_valid;
  assign ha_valid      = {sf_h_valid, h_valid};
  assign h_ready       = ha_ready[0];
  assign sf_h_ready    = ha_ready[1];
  assign h_rs_valid    = ha_rs[0];
  assign sf_h_rs_valid = ha_rs[1];
  ng_rsp_oparb #(.AW(47)) u_harb (
    .clk(clk), .rst_n(rst_n),
    .op_valid(ha_valid), .op_ready(ha_ready), .op_wr({sf_wr, x_wr}),
    .op_addr('{o_addr, sf_addr}), .op_wstrb('{x_wstrb, sf_wstrb}), .op_wdata('{x_wdata, sf_wdata}),
    .rs_valid(ha_rs),
    .e_valid(eh_valid), .e_ready(eh_ready), .e_wr(eh_wr), .e_addr(eh_addr), .e_wstrb(eh_wstrb), .e_wdata(eh_wdata),
    .e_rs_valid(eh_rs_valid), .dbg_st(ha_st), .grants(ha_gnt));

  // ------------------------------------------------------------------ BAR2 (C3 window) + C3 engine
  localparam int C3_AW = M_AXI_C3_ADDR_W;
  localparam int C3_DW = M_AXI_C3_DATA_W;
  logic        b2_valid, b2_ready, b2_wr, b2_rs_valid, b2_rs_err, b2_busy;
  logic [C3_AW-1:0] b2_addr;
  logic [7:0]  b2_wstrb;
  logic [63:0] b2_wdata, b2_rs_rdata;
  logic [2:0]  e_valid, e_ready, e_rs_valid;
  logic        e_rs_err;
  logic [63:0] e_rs_rdata;

  ng_rcp_bar2 #(.AW(C3_AW)) u_bar2 (
    .clk(clk), .rst_n(rst_n), .win_en(win_en), .win_log2(win_log2), .win_base(win_base),
    .s_awid(s_axi_bar2_awid), .s_awaddr(s_axi_bar2_awaddr), .s_awlen(s_axi_bar2_awlen), .s_awsize(s_axi_bar2_awsize),
    .s_awvalid(s_axi_bar2_awvalid), .s_awready(s_axi_bar2_awready),
    .s_wdata(s_axi_bar2_wdata), .s_wstrb(s_axi_bar2_wstrb), .s_wlast(s_axi_bar2_wlast),
    .s_wvalid(s_axi_bar2_wvalid), .s_wready(s_axi_bar2_wready),
    .s_bid(s_axi_bar2_bid), .s_bresp(s_axi_bar2_bresp), .s_bvalid(s_axi_bar2_bvalid), .s_bready(s_axi_bar2_bready),
    .s_arid(s_axi_bar2_arid), .s_araddr(s_axi_bar2_araddr), .s_arlen(s_axi_bar2_arlen), .s_arsize(s_axi_bar2_arsize),
    .s_arvalid(s_axi_bar2_arvalid), .s_arready(s_axi_bar2_arready),
    .s_rid(s_axi_bar2_rid), .s_rdata(s_axi_bar2_rdata), .s_rresp(s_axi_bar2_rresp), .s_rlast(s_axi_bar2_rlast),
    .s_rvalid(s_axi_bar2_rvalid), .s_rready(s_axi_bar2_rready),
    .op_valid(b2_valid), .op_ready(b2_ready), .op_wr(b2_wr), .op_addr(b2_addr), .op_wstrb(b2_wstrb),
    .op_wdata(b2_wdata), .rs_valid(b2_rs_valid), .rs_err(b2_rs_err), .rs_rdata(b2_rs_rdata), .busy(b2_busy));

  // requester 0 = BAR2, 1 = CP / SP (ng_rsp_oparb), 2 = MMU table reads
  assign b2_rs_valid = e_rs_valid[0];
  assign b2_rs_err = e_rs_err;   assign b2_rs_rdata = e_rs_rdata;
  assign c1_rs_err = e_rs_err;   assign c1_rs_rdata = e_rs_rdata;
  assign e1_ready  = e_ready[1];
  assign b2_ready  = e_ready[0];
  assign c2_ready  = e_ready[2];
  assign c2_rs_valid = e_rs_valid[2];
  assign e_valid   = {c2_valid, e1_valid, b2_valid};

  ng_rcp_c3eng #(.DW(C3_DW), .AW(C3_AW)) u_c3 (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in), .mem_ready(shell_mem_ready[2]), .idle(c3_idle),
    .op_valid(e_valid), .op_ready(e_ready), .op_wr({e1_wr, b2_wr}),
    .op_addr0(b2_addr), .op_addr1(e1_addr[C3_AW-1:0]), .op_addr2(c2_addr[C3_AW-1:0]),
    .op_wstrb0(b2_wstrb), .op_wstrb1(e1_wstrb), .op_wdata0(b2_wdata), .op_wdata1(e1_wdata),
    .rs_valid(e_rs_valid), .rs_err(e_rs_err), .rs_rdata(e_rs_rdata),
    .m_awaddr(m_axi_c3_awaddr), .m_awlen(m_axi_c3_awlen), .m_awvalid(m_axi_c3_awvalid), .m_awready(m_axi_c3_awready),
    .m_wdata(m_axi_c3_wdata), .m_wstrb(m_axi_c3_wstrb), .m_wlast(m_axi_c3_wlast), .m_wvalid(m_axi_c3_wvalid),
    .m_wready(m_axi_c3_wready), .m_bresp(m_axi_c3_bresp), .m_bvalid(m_axi_c3_bvalid), .m_bready(m_axi_c3_bready),
    .m_araddr(m_axi_c3_araddr), .m_arlen(m_axi_c3_arlen), .m_arvalid(m_axi_c3_arvalid), .m_arready(m_axi_c3_arready),
    .m_rdata(m_axi_c3_rdata), .m_rresp(m_axi_c3_rresp), .m_rlast(m_axi_c3_rlast), .m_rvalid(m_axi_c3_rvalid),
    .m_rready(m_axi_c3_rready),
    .b_awvalid(mc_awvalid), .b_awready(mc_awready), .b_awaddr(mc_awaddr), .b_awlen(mc_awlen), .b_wvalid(mc_wvalid),
    .b_wready(mc_wready), .b_wdata(mc_wdata), .b_wstrb(mc_wstrb), .b_wlast(mc_wlast), .b_bvalid(mc_bvalid),
    .b_bready(mc_bready), .b_bresp(mc_bresp), .b_arvalid(mc_arvalid), .b_arready(mc_arready), .b_araddr(mc_araddr),
    .b_arlen(mc_arlen), .b_rvalid(mc_rvalid), .b_rready(mc_rready), .b_rdata(mc_rdata), .b_rresp(mc_rresp),
    .b_rlast(mc_rlast));

  ng_mmu u_mmu (
    .clk(clk), .rst_n(rst_n),
    .csr_wr(csr_wr), .csr_waddr(csr_waddr), .csr_wdata(csr_wdata), .csr_raddr(csr_raddr), .csr_rdata(csr_rdata),
    .irq_fault(mmu_irq),
    .req_valid(mu_req_valid), .req_ready(mu_req_ready), .req_vmid(mu_req_vmid), .req_va(mu_req_va),
    .req_acc(mu_req_acc), .req_atomic(mu_req_atomic), .req_tag(8'h00),
    .resp_valid(mu_resp_valid), .resp_ready(mu_resp_ready), .resp_fault(t_resp_fault), .resp_sparse(t_resp_sparse),
    .resp_pa(t_resp_pa), .resp_size(t_resp_size), .resp_level(t_resp_level), .resp_pte(t_resp_pte),
    .resp_tag(t_resp_tag),
    .ext_flt_valid(ef_valid), .ext_flt_vmid(ef_vmid), .ext_flt_va(ef_va), .ext_flt_acc(ef_acc),
    .ext_flt_code(ef_code), .ext_flt_level(ef_level), .ext_flt_pte(ef_pte),
    .rd_valid(mm_rd_valid), .rd_ready(mm_rd_ready), .rd_addr(mm_rd_addr),
    .rsp_valid(mm_rsp_valid), .rsp_data(mm_rsp_data), .rsp_err(mm_rsp_err),
    .stat_walks(mm_walks), .stat_tlb_hits(mm_hits));

  // page-table reads -> C3 engine requester 2
  ng_rcp_mmurd u_mmurd (
    .clk(clk), .rst_n(rst_n),
    .rd_valid(mm_rd_valid), .rd_ready(mm_rd_ready), .rd_addr(mm_rd_addr),
    .rsp_valid(mm_rsp_valid), .rsp_data(mm_rsp_data), .rsp_err(mm_rsp_err),
    .op_valid(c2_valid), .op_ready(c2_ready), .op_addr(c2_addr),
    .rs_valid(c2_rs_valid), .rs_err(e_rs_err), .rs_rdata(e_rs_rdata));

  // MMU fault interrupt -> CP register block (clk_ts), IRQ_STATUS bit 2
  ng_rcp_irq_l2ts u_mmu_irq (
    .clk_l2(clk), .rst_l2_n(rst_n), .pulse_l2(mmu_irq), .clk_ts(clk_ts), .rst_ts_n(rst_ts_n), .pulse_ts(mmu_irq_ts));

  assign m_axi_c3_awid = '0; assign m_axi_c3_awsize = 3'($clog2(C3_DW / 8)); assign m_axi_c3_awburst = 2'b01;
  assign m_axi_c3_awcache = 4'b0011; assign m_axi_c3_awprot = 3'b010; assign m_axi_c3_awqos = '0;
  assign m_axi_c3_arid = '0; assign m_axi_c3_arsize = 3'($clog2(C3_DW / 8)); assign m_axi_c3_arburst = 2'b01;
  assign m_axi_c3_arcache = 4'b0011; assign m_axi_c3_arprot = 3'b010; assign m_axi_c3_arqos = '0;

  // ------------------------------------------------------------------ host engine
  ng_rcp_hosteng u_host (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in), .idle(host_idle),
    .op_valid(eh_valid), .op_ready(eh_ready), .op_wr(eh_wr), .op_addr(eh_addr), .op_wstrb(eh_wstrb),
    .op_wdata(eh_wdata),
    .rs_valid(eh_rs_valid), .rs_err(h_rs_err), .rs_rdata(h_rs_rdata),
    .m_awaddr(m_axi_host_awaddr), .m_awlen(m_axi_host_awlen), .m_awvalid(m_axi_host_awvalid),
    .m_awready(m_axi_host_awready), .m_wdata(m_axi_host_wdata), .m_wstrb(m_axi_host_wstrb),
    .m_wlast(m_axi_host_wlast), .m_wvalid(m_axi_host_wvalid), .m_wready(m_axi_host_wready),
    .m_bresp(m_axi_host_bresp), .m_bvalid(m_axi_host_bvalid), .m_bready(m_axi_host_bready),
    .m_araddr(m_axi_host_araddr), .m_arlen(m_axi_host_arlen), .m_arvalid(m_axi_host_arvalid),
    .m_arready(m_axi_host_arready), .m_rdata(m_axi_host_rdata), .m_rresp(m_axi_host_rresp),
    .m_rlast(m_axi_host_rlast), .m_rvalid(m_axi_host_rvalid), .m_rready(m_axi_host_rready),
    .b_awvalid(mh_awvalid), .b_awready(mh_awready), .b_awaddr(mh_awaddr), .b_awlen(mh_awlen), .b_wvalid(mh_wvalid),
    .b_wready(mh_wready), .b_wdata(mh_wdata), .b_wstrb(mh_wstrb), .b_wlast(mh_wlast), .b_bvalid(mh_bvalid),
    .b_bready(mh_bready), .b_bresp(mh_bresp), .b_arvalid(mh_arvalid), .b_arready(mh_arready), .b_araddr(mh_araddr),
    .b_arlen(mh_arlen), .b_rvalid(mh_rvalid), .b_rready(mh_rready), .b_rdata(mh_rdata), .b_rresp(mh_rresp),
    .b_rlast(mh_rlast));
  assign m_axi_host_awid = '0; assign m_axi_host_awsize = 3'd3; assign m_axi_host_awburst = 2'b01;
  assign m_axi_host_awcache = 4'b0011; assign m_axi_host_awprot = 3'b010; assign m_axi_host_awqos = '0;
  assign m_axi_host_arid = '0; assign m_axi_host_arsize = 3'd3; assign m_axi_host_arburst = 2'b01;
  assign m_axi_host_arcache = 4'b0011; assign m_axi_host_arprot = 3'b010; assign m_axi_host_arqos = '0;

  // ------------------------------------------------------------------ clk_l2 debug registers (BAR0 0x08C0..0x08FF)
  always_comb begin
    for (int i = 0; i < 16; i++) l2dbg[i] = 32'h0;
    l2dbg[0]  = {13'h0, q_in, host_idle, c3_idle, 2'b00, ha_st, c3a_st, ma_st, 2'b00, mb_st_l2, 1'b0, sf_st};
    l2dbg[1]  = sf_ops;
    l2dbg[2]  = sf_bad;
    l2dbg[3]  = sf_bad_pa[31:0];
    l2dbg[4]  = {16'h0, sf_bad_pa[47:32]};
    l2dbg[5]  = ma_gnt[0];
    l2dbg[6]  = ma_gnt[1];
    l2dbg[7]  = c3a_gnt[0];
    l2dbg[8]  = c3a_gnt[1];
    l2dbg[9]  = ha_gnt[0];
    l2dbg[10] = ha_gnt[1];
    l2dbg[11] = mm_walks;
    l2dbg[12] = mm_hits;
    l2dbg[13] = sf_bursts;
    l2dbg[14] = sf_wfb;
  end

  // ------------------------------------------------------------------ unused channels C1, C2, C4: never issue
  `define NG_RSP_TIE(p) \
    assign m_axi_``p``_awid = '0; assign m_axi_``p``_awaddr = '0; assign m_axi_``p``_awlen = '0; \
    assign m_axi_``p``_awsize = '0; assign m_axi_``p``_awburst = 2'b01; assign m_axi_``p``_awcache = 4'b0011; \
    assign m_axi_``p``_awprot = 3'b010; assign m_axi_``p``_awqos = '0; assign m_axi_``p``_awvalid = 1'b0; \
    assign m_axi_``p``_wdata = '0; assign m_axi_``p``_wstrb = '0; assign m_axi_``p``_wlast = 1'b0; \
    assign m_axi_``p``_wvalid = 1'b0; assign m_axi_``p``_bready = 1'b1; \
    assign m_axi_``p``_arid = '0; assign m_axi_``p``_araddr = '0; assign m_axi_``p``_arlen = '0; \
    assign m_axi_``p``_arsize = '0; assign m_axi_``p``_arburst = 2'b01; assign m_axi_``p``_arcache = 4'b0011; \
    assign m_axi_``p``_arprot = 3'b010; assign m_axi_``p``_arqos = '0; assign m_axi_``p``_arvalid = 1'b0; \
    assign m_axi_``p``_rready = 1'b1;
  `NG_RSP_TIE(c1)
  `NG_RSP_TIE(c2)
  `NG_RSP_TIE(c4)
  `undef NG_RSP_TIE

  // ----------------------------------------------------------------
  assign dbg_hub_tdo = 1'b0;
  logic [31:0] obs_sr;
  always_ff @(posedge dbg_obs_tck) begin
    if (dbg_obs_sel && dbg_obs_capture) obs_sr <= NG_RM_INFO_MAGIC;
    else if (dbg_obs_sel && dbg_obs_shift) obs_sr <= {dbg_obs_tdi, obs_sr[31:1]};
  end
  always_ff @(negedge dbg_obs_tck) dbg_obs_tdo <= obs_sr[0];

  logic unused;
  assign unused = &{1'b0, t_resp_size, t_resp_tag, ml_busy, b2_busy, dbg_valid, dbg_code, cpu_quiet,
                    s_mmu_req_tag, cm_w0, cm_exec, mb_st_l2, f_req_slot,
                    s_axil_ctrl_awprot, s_axil_ctrl_arprot, s_axi_bar2_awburst, s_axi_bar2_awcache,
                    s_axi_bar2_awprot, s_axi_bar2_awqos, s_axi_bar2_arburst, s_axi_bar2_arcache,
                    s_axi_bar2_arprot, s_axi_bar2_arqos, clk_core, clk_core2x, rst_core_n,
                    m_axi_c1_awready, m_axi_c1_wready, m_axi_c1_bid, m_axi_c1_bresp, m_axi_c1_bvalid,
                    m_axi_c1_arready, m_axi_c1_rid, m_axi_c1_rdata, m_axi_c1_rresp, m_axi_c1_rlast, m_axi_c1_rvalid,
                    m_axi_c2_awready, m_axi_c2_wready, m_axi_c2_bid, m_axi_c2_bresp, m_axi_c2_bvalid,
                    m_axi_c2_arready, m_axi_c2_rid, m_axi_c2_rdata, m_axi_c2_rresp, m_axi_c2_rlast, m_axi_c2_rvalid,
                    m_axi_c4_awready, m_axi_c4_wready, m_axi_c4_bid, m_axi_c4_bresp, m_axi_c4_bvalid,
                    m_axi_c4_arready, m_axi_c4_rid, m_axi_c4_rdata, m_axi_c4_rresp, m_axi_c4_rlast, m_axi_c4_rvalid,
                    m_axi_c3_bid, m_axi_c3_rid, m_axi_host_bid, m_axi_host_rid, shell_mem_ready[3], shell_mem_ready[1:0],
                    dbg_hub_drck, dbg_hub_tck, dbg_hub_tms, dbg_hub_tdi, dbg_hub_sel, dbg_hub_shift,
                    dbg_hub_capture, dbg_hub_update, dbg_hub_reset, dbg_hub_runtest, dbg_hub_bscanid_en,
                    dbg_obs_update};
endmodule
