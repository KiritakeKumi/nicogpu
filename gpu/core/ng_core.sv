// ng_core.sv -- "L configuration" core top level (simulation scope):
// NSP x ng_sp_top (NL = 16 lanes) + work distributor + per-SP L1D -> NoC -> 4 L2 slices -> 4 AXI ports.
//
//                 CP wd_* ──> ng_core_wd ──> SP0 .. SP(NSP-1)   (ng_sp_top, black boxes: ng_wd, I$, LSU + L1D +
//                                              │ │                shared array, scheduler, units)
//              MMU client <── ng_core_mmuarb <─┘ │ mem (mreq_t / mrsp_t, one line request in flight per SP)
//                                                ▼
//          uc port <── ng_core_ucarb <── byp  ng_lsu_noc_adapter (per SP + one for the CP port) ─> ng_noc_ep
//                                                                                                     │
//                                            ng_noc_mesh GW x GH (XY routing, VC0 requests, VC1 responses)
//                                                                                                     │
//                          4 x (ng_noc_ep + ng_l2_slice) ─> m_axi[s]  (slice 0/1 = C1 512 b, 2 = C2, 3 = C4 256 b)
//
// Half cores (design doc 4.1 / 8.3): SP 2h and 2h+1 form half core h.  In this version every SP keeps its own LSU,
// L1D and 64 KiB shared array (ng_sp_top unchanged), and ng_core_wd places a workgroup on exactly one SP, so a
// workgroup sits inside one half core and shares one L1D (ERRATA v0.1.4); the largest workgroup is therefore 16 warps
// (512 threads, as the single-SP RM), not the model's 1024 over the 2 SPs of a half core.  The "half core" is
// physical grouping only (NoC row / SLR); a shared half-core L1D / shared array / cross-SP barrier is future work
//
//
// Memory map (ng_lsu_pkg / model interleave.h): VRAM PA < 16 GiB -> L2 slice l2_slice_idx(pa) (4 KiB granules,
// slots C1 C2 C1 C4, ng_l2_addrmap); SYS / HOST PAs (mreq.uc) bypass the NoC to the uc port.  The CP port is a second
// line-granular requester on its own NoC node (the TB uses it for cache.wb.l2 after the dispatch; on a board the CP /
// copy engine VRAM path would go through it).
//
// Resets: rst_n resets everything; core_reset (CP CORE_RESET | WDIST abort, level) goes to every SP and the
// distributor; the adapters / NoC / slices / arbiters only see rst_n, so a transaction in flight across a core reset
// completes and the SP drops the response (the ng_sp_top contract).  After a trap the distributor aborts the other
// SPs through their core_reset (sp_abort).
//
// Debug: per-SP commit / trap ports (arrays) and the core's first trap record (first SP whose dbg_trap_valid rose,
// lowest index on a tie; cleared at wd_start).
//
// placement, node layout, one MMU shared by all SPs with one translation in flight,
// one uc transaction in flight, static workgroup interleave.
module ng_core
  import ng_lsu_pkg::mreq_t;
  import ng_lsu_pkg::mrsp_t;
  import ng_cp_pkg::wd_req_t;
  import ng_noc_pkg::flit_t;
#(
  parameter int          NSP         = 4,
  parameter int          CORE_ID     = 0,
  // SP (ng_sp_top) configuration
  parameter int          NL          = 16,
  parameter int          L1_SETS     = 16,
  parameter int          L1_WAYS     = 2,
  parameter int          IC_LINES    = 32,
  parameter bit          EN_SFU      = 1'b1,
  parameter bit          EN_XLANE    = 1'b0,
  parameter bit          EN_FP64     = 1'b0,
  parameter bit          EN_VALU_FP  = 1'b1,
  parameter bit          CLEAR_VREGS = 1'b0,
  // L2 slices (full configuration 1024 x 8 = 1 MiB per slice; the TB uses small slices to force evictions)
  parameter int          L2_SETS     = 1024,
  parameter int          L2_WAYS     = 8,
  // NoC: GW x GH mesh, node = y * GW + x
  parameter int          GW          = 3,
  parameter int          GH          = 3,
  parameter int          BUF_DEPTH   = 4,
  parameter int          LOC_CREDITS = 4,
  parameter int          TXQ_DEPTH   = 2,
  parameter logic [23:0] SP_NODES    = {6'd7, 6'd6, 6'd1, 6'd0},   // SP i -> node [6i+:6] (SP 0..3)
  parameter logic [5:0]  CP_NODE     = 6'd4,
  parameter logic [23:0] SLICE_NODES = {6'd3, 6'd2, 6'd8, 6'd5}    // slice i -> node [6i+:6]
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          core_reset,

  // ---- CP work-distributor port (ng_cp_top wd_*) ----
  input  logic          wd_start,
  input  wd_req_t       wd_req,
  output logic          wd_done,
  output logic          wd_fault,
  output logic [63:0]   wd_instr,
  output logic          sp_idle,

  // ---- MMU client (one ng_mmu for the core) ----
  output logic          mmu_req_valid,
  input  logic          mmu_req_ready,
  output logic [3:0]    mmu_req_vmid,
  output logic [63:0]   mmu_req_va,
  output logic [1:0]    mmu_req_acc,
  output logic          mmu_req_atomic,
  output logic [7:0]    mmu_req_tag,
  input  logic          mmu_resp_valid,
  output logic          mmu_resp_ready,
  input  logic [2:0]    mmu_resp_fault,
  input  logic          mmu_resp_sparse,
  input  logic [47:0]   mmu_resp_pa,
  input  logic [63:0]   mmu_resp_pte,
  output logic          ext_flt_valid,
  output logic [3:0]    ext_flt_vmid,
  output logic [63:0]   ext_flt_va,
  output logic [1:0]    ext_flt_acc,

  // ---- uc (SYS / HOST) port: one request in flight, response pulse >= 1 cycle after the handshake ----
  output logic          uc_req_valid,
  input  logic          uc_req_ready,
  output mreq_t         uc_req,
  input  logic          uc_rsp_valid,
  input  mrsp_t         uc_rsp,

  // ---- CP line port into the L2 (same contract as the L1D mem port) ----
  input  logic          cp_req_valid,
  output logic          cp_req_ready,
  input  mreq_t         cp_req,
  output logic          cp_rsp_valid,
  output mrsp_t         cp_rsp,

  // ---- L2 slice AXI4 masters (slice s; C1 slices use 33 / 512 bits, C2 / C4 the low 32 / 256 bits) ----
  output logic          m_arvalid [4],
  input  logic          m_arready [4],
  output logic [32:0]   m_araddr  [4],
  output logic [7:0]    m_arlen   [4],
  output logic [2:0]    m_arsize  [4],
  output logic [1:0]    m_arburst [4],
  output logic [5:0]    m_arid    [4],
  input  logic          m_rvalid  [4],
  output logic          m_rready  [4],
  input  logic [511:0]  m_rdata   [4],
  input  logic          m_rlast   [4],
  input  logic [5:0]    m_rid     [4],
  input  logic [1:0]    m_rresp   [4],
  output logic          m_awvalid [4],
  input  logic          m_awready [4],
  output logic [32:0]   m_awaddr  [4],
  output logic [7:0]    m_awlen   [4],
  output logic [2:0]    m_awsize  [4],
  output logic [1:0]    m_awburst [4],
  output logic [5:0]    m_awid    [4],
  output logic          m_wvalid  [4],
  input  logic          m_wready  [4],
  output logic [511:0]  m_wdata   [4],
  output logic [63:0]   m_wstrb   [4],
  output logic          m_wlast   [4],
  input  logic          m_bvalid  [4],
  output logic          m_bready  [4],
  input  logic [5:0]    m_bid     [4],
  input  logic [1:0]    m_bresp   [4],

  // ---- debug / trace (testbench and DFX) ----
  output logic [NSP-1:0] dbg_commit_valid,
  output logic [31:0]    dbg_commit_pc   [NSP],
  output logic [31:0]    dbg_commit_w0   [NSP],
  output logic [31:0]    dbg_commit_exec [NSP],
  output logic [31:0]    dbg_commit_wg   [NSP],
  output logic [4:0]     dbg_commit_warp [NSP],
  output logic           dbg_trap_valid,       // level: first trap of the dispatch (any SP)
  output logic [1:0]     dbg_trap_sp,
  output logic [2:0]     dbg_trap_unit,
  output logic [3:0]     dbg_trap_slot,
  output logic [31:0]    dbg_trap_pc,
  output logic [3:0]     dbg_trap_cause,
  output logic [31:0]    dbg_trap_info,
  output logic [31:0]    dbg_trap_wg,
  output logic [4:0]     dbg_trap_warp,
  output logic [NSP-1:0] dbg_sp_idle,
  output logic [NSP-1:0] dbg_sp_aborted,      // SPs the distributor aborted after a trap (current dispatch)
  output logic           fab_busy             // a line / uc / translation transaction is in flight (ng_cp_port fab_busy_i)
);
  localparam int NN = GW * GH;
  localparam int NREQ = NSP + 1;        // NoC requesters: SPs + the CP port

  if (NSP < 1 || NSP > 4) begin : g_bad_nsp
    $error("ng_core: NSP must be 1..4 (SP_NODES has 4 entries)");
  end

  // ================================================================ work distributor
  logic [NSP-1:0] sp_start, sp_abort, sp_done, sp_fault, sp_idle_v;
  wd_req_t        sp_req   [NSP];
  logic [63:0]    sp_instr [NSP];
  logic           cwd_busy;
  ng_core_wd #(.NSP(NSP)) u_wd (
    .clk(clk), .rst_n(rst_n), .core_reset(core_reset),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .busy(cwd_busy),
    .sp_start(sp_start), .sp_req(sp_req), .sp_abort(sp_abort), .sp_done(sp_done), .sp_fault(sp_fault),
    .sp_instr(sp_instr), .dbg_done(), .dbg_aborted(dbg_sp_aborted)
  );
  assign sp_idle     = (&sp_idle_v) && !cwd_busy;
  assign dbg_sp_idle = sp_idle_v;

  // ================================================================ NoC
  logic [NN-1:0]        ep_inj_valid;
  flit_t [NN-1:0]       ep_inj_flit;
  logic [NN-1:0][1:0]   ep_inj_credit;
  logic [NN-1:0]        ep_recv_valid;
  flit_t [NN-1:0]       ep_recv_flit;
  logic [NN-1:0][1:0]   ep_recv_credit;

  ng_noc_mesh #(.GW(GW), .GH(GH), .NUM_VC(2), .BUF_DEPTH(BUF_DEPTH), .LOC_CREDITS(LOC_CREDITS)) u_mesh (
    .clk(clk), .rst_n(rst_n),
    .ep_inj_valid(ep_inj_valid), .ep_inj_flit(ep_inj_flit), .ep_inj_credit(ep_inj_credit),
    .ep_recv_valid(ep_recv_valid), .ep_recv_flit(ep_recv_flit), .ep_recv_credit(ep_recv_credit)
  );

  // nodes without an agent (NSP < 4, or a mesh larger than the agent count) are tied off
  function automatic bit node_used(input int n);
    bit u;
    u = (int'(CP_NODE) == n);
    for (int i = 0; i < NSP; i++) if (int'(SP_NODES[6*i +: 6]) == n) u = 1'b1;
    for (int k = 0; k < 4; k++) if (int'(SLICE_NODES[6*k +: 6]) == n) u = 1'b1;
    return u;
  endfunction
  for (genvar n = 0; n < NN; n++) begin : g_node
    if (!node_used(n)) begin : g_free
      assign ep_inj_valid[n]   = 1'b0;
      assign ep_inj_flit[n]    = '0;
      assign ep_recv_credit[n] = '0;
    end
  end

  // ================================================================ requester nodes: SPs + CP port
  logic          rq_valid [NREQ], rq_ready [NREQ], rs_valid [NREQ];
  mreq_t         rq       [NREQ];
  mrsp_t         rs       [NREQ];
  logic [NREQ-1:0] byp_valid, byp_ready, byp_rsp_valid;
  mreq_t         byp_req  [NREQ];
  mrsp_t         byp_rsp;
  logic          uc_busy, mmu_busy;

  for (genvar r = 0; r < NREQ; r++) begin : g_rq
    localparam logic [5:0] NODE = (r < NSP) ? SP_NODES[6*((r < NSP) ? r : 0) +: 6] : CP_NODE;
    logic  ti_valid, ti_ready, to_valid, to_ready;
    flit_t ti_flit, to_flit;
    ng_lsu_noc_adapter #(.SRC_ID(NODE), .SLICE_NODES(SLICE_NODES), .TAG_HI(4'h0)) u_ad (
      .clk(clk), .rst_n(rst_n),
      .req_valid(rq_valid[r]), .req_ready(rq_ready[r]), .req(rq[r]),
      .rsp_valid(rs_valid[r]), .rsp(rs[r]),
      .ti_valid(ti_valid), .ti_ready(ti_ready), .ti_flit(ti_flit),
      .to_valid(to_valid), .to_ready(to_ready), .to_flit(to_flit),
      .byp_req_valid(byp_valid[r]), .byp_req_ready(byp_ready[r]), .byp_req(byp_req[r]),
      .byp_rsp_valid(byp_rsp_valid[r]), .byp_rsp(byp_rsp)
    );
    ng_noc_ep #(.NUM_VC(2), .INJ_CREDITS(BUF_DEPTH), .RX_DEPTH(2 * LOC_CREDITS), .TXQ_DEPTH(TXQ_DEPTH)) u_ep (
      .clk(clk), .rst_n(rst_n),
      .ti_valid(ti_valid), .ti_ready(ti_ready), .ti_flit(ti_flit),
      .to_valid(to_valid), .to_ready(to_ready), .to_flit(to_flit),
      .inj_valid(ep_inj_valid[NODE]), .inj_flit(ep_inj_flit[NODE]), .inj_credit(ep_inj_credit[NODE]),
      .recv_valid(ep_recv_valid[NODE]), .recv_flit(ep_recv_flit[NODE]), .recv_credit(ep_recv_credit[NODE])
    );
  end

  assign rq_valid[NSP] = cp_req_valid;
  assign rq[NSP]       = cp_req;
  assign cp_req_ready  = rq_ready[NSP];
  assign cp_rsp_valid  = rs_valid[NSP];
  assign cp_rsp        = rs[NSP];

  ng_core_ucarb #(.N(NREQ)) u_ucarb (
    .clk(clk), .rst_n(rst_n),
    .req_valid(byp_valid), .req_ready(byp_ready), .req(byp_req), .rsp_valid(byp_rsp_valid), .rsp(byp_rsp),
    .m_req_valid(uc_req_valid), .m_req_ready(uc_req_ready), .m_req(uc_req),
    .m_rsp_valid(uc_rsp_valid), .m_rsp(uc_rsp), .busy(uc_busy), .grants()
  );

  always_comb begin
    fab_busy = 1'b0;
    for (int r = 0; r < NREQ; r++) if (!rq_ready[r] || rq_valid[r]) fab_busy = 1'b1;
    if (uc_busy || mmu_busy) fab_busy = 1'b1;
  end

  // ================================================================ L2 slices
  for (genvar s = 0; s < 4; s++) begin : g_sl
    localparam logic [5:0] NODE = SLICE_NODES[6*s +: 6];
    localparam int AW = (s < 2) ? 33 : 32;
    localparam int DW = (s < 2) ? 512 : 256;
    logic  req_v, req_r, resp_v, resp_r;
    flit_t req_f, resp_f;
    ng_noc_ep #(.NUM_VC(2), .INJ_CREDITS(BUF_DEPTH), .RX_DEPTH(2 * LOC_CREDITS), .TXQ_DEPTH(TXQ_DEPTH)) u_ep (
      .clk(clk), .rst_n(rst_n),
      .ti_valid(resp_v), .ti_ready(resp_r), .ti_flit(resp_f),
      .to_valid(req_v), .to_ready(req_r), .to_flit(req_f),
      .inj_valid(ep_inj_valid[NODE]), .inj_flit(ep_inj_flit[NODE]), .inj_credit(ep_inj_credit[NODE]),
      .recv_valid(ep_recv_valid[NODE]), .recv_flit(ep_recv_flit[NODE]), .recv_credit(ep_recv_credit[NODE])
    );
    logic [AW-1:0]   araddr, awaddr;
    logic [DW-1:0]   wdata;
    logic [DW/8-1:0] wstrb;
    ng_l2_slice #(.SETS(L2_SETS), .WAYS(L2_WAYS), .AXI_AW(AW), .AXI_DW(DW), .AXI_IW(6), .AXI_ID(s),
                  .SLICE_ID(int'(NODE))) u_sl (
      .clk(clk), .rst_n(rst_n),
      .s_req_valid(req_v), .s_req_ready(req_r), .s_req_flit(req_f),
      .m_resp_valid(resp_v), .m_resp_ready(resp_r), .m_resp_flit(resp_f),
      .arvalid(m_arvalid[s]), .arready(m_arready[s]), .araddr(araddr), .arlen(m_arlen[s]),
      .arsize(m_arsize[s]), .arburst(m_arburst[s]), .arid(m_arid[s]),
      .rvalid(m_rvalid[s]), .rready(m_rready[s]), .rdata(m_rdata[s][DW-1:0]), .rlast(m_rlast[s]),
      .rid(m_rid[s]), .rresp(m_rresp[s]),
      .awvalid(m_awvalid[s]), .awready(m_awready[s]), .awaddr(awaddr), .awlen(m_awlen[s]),
      .awsize(m_awsize[s]), .awburst(m_awburst[s]), .awid(m_awid[s]),
      .wvalid(m_wvalid[s]), .wready(m_wready[s]), .wdata(wdata), .wstrb(wstrb), .wlast(m_wlast[s]),
      .bvalid(m_bvalid[s]), .bready(m_bready[s]), .bid(m_bid[s]), .bresp(m_bresp[s])
    );
    assign m_araddr[s] = 33'(araddr);
    assign m_awaddr[s] = 33'(awaddr);
    assign m_wdata[s]  = 512'(wdata);
    assign m_wstrb[s]  = 64'(wstrb);
  end

  // ================================================================ shared MMU client / fault report
  logic [NSP-1:0] s_mreq_valid, s_mreq_ready, s_mreq_atomic, s_mresp_valid, s_mresp_ready, s_flt_valid;
  logic [3:0]     s_mreq_vmid [NSP], s_flt_vmid [NSP];
  logic [63:0]    s_mreq_va   [NSP], s_flt_va   [NSP];
  logic [1:0]     s_mreq_acc  [NSP], s_flt_acc  [NSP];
  logic [7:0]     s_mreq_tag  [NSP];
  ng_core_mmuarb #(.N(NSP)) u_mmuarb (
    .clk(clk), .rst_n(rst_n),
    .req_valid(s_mreq_valid), .req_ready(s_mreq_ready), .req_vmid(s_mreq_vmid), .req_va(s_mreq_va),
    .req_acc(s_mreq_acc), .req_atomic(s_mreq_atomic), .req_tag(s_mreq_tag),
    .resp_valid(s_mresp_valid), .resp_ready(s_mresp_ready),
    .m_req_valid(mmu_req_valid), .m_req_ready(mmu_req_ready), .m_req_vmid(mmu_req_vmid), .m_req_va(mmu_req_va),
    .m_req_acc(mmu_req_acc), .m_req_atomic(mmu_req_atomic), .m_req_tag(mmu_req_tag),
    .m_resp_valid(mmu_resp_valid), .m_resp_ready(mmu_resp_ready), .busy(mmu_busy), .grants()
  );
  ng_core_fltarb #(.N(NSP)) u_fltarb (
    .clk(clk), .rst_n(rst_n),
    .in_valid(s_flt_valid), .in_vmid(s_flt_vmid), .in_va(s_flt_va), .in_acc(s_flt_acc),
    .out_valid(ext_flt_valid), .out_vmid(ext_flt_vmid), .out_va(ext_flt_va), .out_acc(ext_flt_acc), .dropped()
  );

  // ================================================================ SPs
  logic [NSP-1:0] t_valid;
  logic [2:0]     t_unit  [NSP];
  logic [3:0]     t_slot  [NSP], t_cause [NSP];
  logic [31:0]    t_pc    [NSP], t_info  [NSP], t_wg [NSP];
  logic [4:0]     t_warp  [NSP];

  for (genvar i = 0; i < NSP; i++) begin : g_sp
    ng_sp_top #(.NL(NL), .L1_SETS(L1_SETS), .L1_WAYS(L1_WAYS), .IC_LINES(IC_LINES),
                .EN_SFU(EN_SFU), .EN_XLANE(EN_XLANE), .EN_FP64(EN_FP64), .EN_VALU_FP(EN_VALU_FP),
                .SP_ID(i), .CORE_ID(CORE_ID), .GSLOT_BASE((CORE_ID * NSP + i) * 16),
                .CLEAR_VREGS(CLEAR_VREGS)) u_sp (
      .clk(clk), .rst_n(rst_n), .core_reset(core_reset | sp_abort[i]), .inv_l1_ext(1'b0), .inv_i_ext(1'b0),
      .wd_start(sp_start[i]), .wd_req(sp_req[i]), .wd_done(sp_done[i]), .wd_fault(sp_fault[i]),
      .wd_instr(sp_instr[i]), .sp_idle(sp_idle_v[i]),
      .mmu_req_valid(s_mreq_valid[i]), .mmu_req_ready(s_mreq_ready[i]), .mmu_req_vmid(s_mreq_vmid[i]),
      .mmu_req_va(s_mreq_va[i]), .mmu_req_acc(s_mreq_acc[i]), .mmu_req_atomic(s_mreq_atomic[i]),
      .mmu_req_tag(s_mreq_tag[i]), .mmu_resp_valid(s_mresp_valid[i]), .mmu_resp_ready(s_mresp_ready[i]),
      .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
      .mmu_resp_pte(mmu_resp_pte),
      .ext_flt_valid(s_flt_valid[i]), .ext_flt_vmid(s_flt_vmid[i]), .ext_flt_va(s_flt_va[i]),
      .ext_flt_acc(s_flt_acc[i]),
      .mem_req_valid(rq_valid[i]), .mem_req_ready(rq_ready[i]), .mem_req(rq[i]),
      .mem_rsp_valid(rs_valid[i]), .mem_rsp(rs[i]),
      .dbg_commit_valid(dbg_commit_valid[i]), .dbg_commit_slot(), .dbg_commit_pc(dbg_commit_pc[i]),
      .dbg_commit_w0(dbg_commit_w0[i]), .dbg_commit_exec(dbg_commit_exec[i]), .dbg_commit_wg(dbg_commit_wg[i]),
      .dbg_commit_warp(dbg_commit_warp[i]),
      .dbg_trap_valid(t_valid[i]), .dbg_trap_unit(t_unit[i]), .dbg_trap_slot(t_slot[i]), .dbg_trap_pc(t_pc[i]),
      .dbg_trap_cause(t_cause[i]), .dbg_trap_info(t_info[i]), .dbg_trap_wg(t_wg[i]), .dbg_trap_warp(t_warp[i]),
      .stat_ic_hit(), .stat_ic_miss(), .stat_l1_rd(), .stat_l1_wr(), .stat_instr_mem(), .stat_groups(),
      .stat_perf(), .dbg_status()
    );
  end

  // ================================================================ first trap of the dispatch (core level)
  always_ff @(posedge clk) begin
    if (!rst_n || core_reset) begin
      dbg_trap_valid <= 1'b0;
      dbg_trap_sp <= '0; dbg_trap_unit <= '0; dbg_trap_slot <= '0; dbg_trap_pc <= '0; dbg_trap_cause <= '0;
      dbg_trap_info <= '0; dbg_trap_wg <= '0; dbg_trap_warp <= '0;
    end else if (wd_start) begin
      dbg_trap_valid <= 1'b0;
    end else if (!dbg_trap_valid && |t_valid) begin
      int k;
      k = int'(ng_bitops_pkg::lowest_set(64'(t_valid)));
      dbg_trap_valid <= 1'b1;
      dbg_trap_sp    <= 2'(k);
      dbg_trap_unit  <= t_unit[k];
      dbg_trap_slot  <= t_slot[k];
      dbg_trap_pc    <= t_pc[k];
      dbg_trap_cause <= t_cause[k];
      dbg_trap_info  <= t_info[k];
      dbg_trap_wg    <= t_wg[k];
      dbg_trap_warp  <= t_warp[k];
    end
  end
endmodule
