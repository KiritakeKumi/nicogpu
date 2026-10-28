// tb_core.sv -- the multi-SP core (ng_core: NSP x ng_sp_top + work distributor + per-SP L1D -> NoC -> 4 L2
// slices) against a functional-model dispatch bundle exported with the model running as ONE core of NSP SPs
// (tools/dispatch_bundle/mk_core_bundles.sh, bundle_capture.cpp NG_BUNDLE_NUM_SP; format).
//
// The TB plays CP + MMU + memory:
//   * CP: the bundle's request goes through a real ng_cp_port (WDIST_REQ dwords + GO) into the core's wd_* port;
//     after wd_done the TB issues cache.wb.l2 (M_WB_L2) through the core's CP line port (NoC node of the CP) so the
//     VRAM image is complete in the DDR models;
//   * MMU stub on the core's single (shared) translation port: the bundle's page map, fault codes as ng_mmu;
//   * VRAM (PA < 16 GiB): 4 AXI DDR models behind the real L2 slices (tb_l2_axi_mem_model, HASH_INIT = 0, random
//     latency), preloaded through a backdoor with the bundle's pre-image (channel offsets per ng_l2_addrmap);
//   * SYS / HOST (GTT) pages: a PA-addressed memory on the core's uc port (M_RD / M_WR / M_AMO with ng_l2_amo_alu).
// Checks:
//   * every committed instruction of every SP against the model's trace of the same warp (warp key = linear WG id
//     * 32 + warp index; pc, w0, exec) -- per warp exact, the interleave across warps / SPs is not compared;
//   * placement: all warps of a workgroup commit on one SP, and that SP is (linear WG id mod NSP) (static interleave,
//     ng_core_wd);
//   * ok bundles: no fault, every warp's trace consumed exactly, WDIST instruction count = model count, final memory
//     byte-identical (bundle pages; mapped VRAM pages and RTL-created host pages the model left zero must be zero;
//     the scratch buffer is excluded: its layout follows the implementation-defined GlobalWarpSlot), every SP ran
//     work when the grid has >= NSP workgroups;
//   * trap bundles: wd_fault, the core's first trap record = the model's (wg, warp, pc, cause, info), every warp's
//     commits are a prefix of the model's (ERRATA v0.1.5 "traps are precise per warp"); memory only for 1-warp traps;
//   * sp_idle: 1 before, 0 while running, 1 after wd_done.
// Reset mode (tb_core_run.txt): core_reset after each listed number of commits (held until a line / uc / translation
// transaction is in flight), no commits afterwards, no wd_done, WDIST busy clear, sp_idle back; then the in-flight
// transactions drain, cache.inv.l2 through the CP port, the memory image is reloaded, and finally the full checked run.
//
// All DUT inputs are driven at negedge or from posedge state machines with non-blocking assignments.
`timescale 1ns/1ps
`include "tb_core_cfg.svh"
`ifndef NG_TB_NSP
`define NG_TB_NSP 4
`endif
`ifndef NG_TB_MAXLAT
`define NG_TB_MAXLAT 3
`endif
`ifndef NG_TB_EN_SFU
`define NG_TB_EN_SFU 1
`endif
`ifndef NG_TB_L2_SETS
`define NG_TB_L2_SETS 16
`endif
`ifndef NG_TB_L2_WAYS
`define NG_TB_L2_WAYS 2
`endif
`ifndef NG_TB_MAXPAGES
`define NG_TB_MAXPAGES 1024
`endif
`ifndef NG_TB_MAXMAP
`define NG_TB_MAXMAP 65536
`endif
`ifndef NG_TB_MAXTR
`define NG_TB_MAXTR 65536
`endif
module tb_core;
  import ng_lsu_pkg::*;
  import ng_cp_pkg::*;
  import ng_l2_addrmap::*;

  localparam int NSP     = `NG_TB_NSP;
  localparam int MAXLAT  = `NG_TB_MAXLAT;
  localparam int MAXPG   = `NG_TB_MAXPAGES;
  localparam int MAXMAP  = `NG_TB_MAXMAP;
  localparam int MAXTR   = `NG_TB_MAXTR;
  localparam int TIMEOUT = 8000000;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic core_reset = 1'b0;
  always #5 clk = ~clk;

  // --------------------------------------------------------------
  logic          wd_start;
  wd_req_t       wd_req;
  logic          wd_done, wd_fault, sp_idle, fab_busy;
  logic [63:0]   wd_instr;
  logic          mmu_req_valid, mmu_req_ready, mmu_req_atomic, mmu_resp_valid, mmu_resp_ready, mmu_resp_sparse;
  logic [3:0]    mmu_req_vmid;
  logic [63:0]   mmu_req_va;
  logic [1:0]    mmu_req_acc;
  logic [7:0]    mmu_req_tag;
  logic [2:0]    mmu_resp_fault;
  logic [47:0]   mmu_resp_pa;
  logic          ext_flt_valid;
  logic [3:0]    ext_flt_vmid;
  logic [63:0]   ext_flt_va;
  logic [1:0]    ext_flt_acc;
  logic          uc_req_valid, uc_req_ready, uc_rsp_valid;
  mreq_t         uc_req;
  mrsp_t         uc_rsp;
  logic          cp_req_valid = 1'b0, cp_req_ready, cp_rsp_valid;
  mreq_t         cp_req = '0;
  mrsp_t         cp_rsp;
  logic          m_arvalid [4], m_arready [4], m_rvalid [4], m_rready [4], m_rlast [4];
  logic [32:0]   m_araddr  [4], m_awaddr [4];
  logic [7:0]    m_arlen   [4], m_awlen  [4];
  logic [2:0]    m_arsize  [4], m_awsize [4];
  logic [1:0]    m_arburst [4], m_awburst [4], m_rresp [4], m_bresp [4];
  logic [5:0]    m_arid    [4], m_rid [4], m_awid [4], m_bid [4];
  logic [511:0]  m_rdata   [4], m_wdata [4];
  logic [63:0]   m_wstrb   [4];
  logic          m_awvalid [4], m_awready [4], m_wvalid [4], m_wready [4], m_wlast [4], m_bvalid [4], m_bready [4];
  logic [NSP-1:0] c_valid;
  logic [31:0]   c_pc [NSP], c_w0 [NSP], c_exec [NSP], c_wg [NSP];
  logic [4:0]    c_warp [NSP];
  logic          dbg_trap_valid;
  logic [1:0]    dbg_trap_sp;
  logic [2:0]    dbg_trap_unit;
  logic [3:0]    dbg_trap_slot, dbg_trap_cause;
  logic [31:0]   dbg_trap_pc, dbg_trap_info, dbg_trap_wg;
  logic [4:0]    dbg_trap_warp;
  logic [NSP-1:0] dbg_sp_idle, dbg_sp_aborted;

  ng_core #(.NSP(NSP), .EN_SFU(`NG_TB_EN_SFU), .L2_SETS(`NG_TB_L2_SETS), .L2_WAYS(`NG_TB_L2_WAYS)) dut (
    .clk(clk), .rst_n(rst_n), .core_reset(core_reset),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .sp_idle(sp_idle),
    .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
    .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic), .mmu_req_tag(mmu_req_tag),
    .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready), .mmu_resp_fault(mmu_resp_fault),
    .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa), .mmu_resp_pte(64'd0),
    .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
    .uc_req_valid(uc_req_valid), .uc_req_ready(uc_req_ready), .uc_req(uc_req), .uc_rsp_valid(uc_rsp_valid),
    .uc_rsp(uc_rsp),
    .cp_req_valid(cp_req_valid), .cp_req_ready(cp_req_ready), .cp_req(cp_req), .cp_rsp_valid(cp_rsp_valid),
    .cp_rsp(cp_rsp),
    .m_arvalid(m_arvalid), .m_arready(m_arready), .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arsize(m_arsize),
    .m_arburst(m_arburst), .m_arid(m_arid), .m_rvalid(m_rvalid), .m_rready(m_rready), .m_rdata(m_rdata),
    .m_rlast(m_rlast), .m_rid(m_rid), .m_rresp(m_rresp),
    .m_awvalid(m_awvalid), .m_awready(m_awready), .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awsize(m_awsize),
    .m_awburst(m_awburst), .m_awid(m_awid), .m_wvalid(m_wvalid), .m_wready(m_wready), .m_wdata(m_wdata),
    .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_bvalid(m_bvalid), .m_bready(m_bready), .m_bid(m_bid), .m_bresp(m_bresp),
    .dbg_commit_valid(c_valid), .dbg_commit_pc(c_pc), .dbg_commit_w0(c_w0), .dbg_commit_exec(c_exec),
    .dbg_commit_wg(c_wg), .dbg_commit_warp(c_warp),
    .dbg_trap_valid(dbg_trap_valid), .dbg_trap_sp(dbg_trap_sp), .dbg_trap_unit(dbg_trap_unit),
    .dbg_trap_slot(dbg_trap_slot), .dbg_trap_pc(dbg_trap_pc), .dbg_trap_cause(dbg_trap_cause),
    .dbg_trap_info(dbg_trap_info), .dbg_trap_wg(dbg_trap_wg), .dbg_trap_warp(dbg_trap_warp),
    .dbg_sp_idle(dbg_sp_idle), .dbg_sp_aborted(dbg_sp_aborted), .fab_busy(fab_busy)
  );

  // ---------------------------------------------------------------- DDR models (VRAM channels behind the slices)
  int ddr_ar [4], ddr_aw [4];
  for (genvar s = 0; s < 4; s++) begin : g_m
    localparam int AW = (s < 2) ? 33 : 32;
    localparam int DW = (s < 2) ? 512 : 256;
    logic [DW-1:0] rdata;
    tb_l2_axi_mem_model #(.AW(AW), .DW(DW), .IW(6), .MAX_DELAY(MAXLAT), .HASH_INIT(1'b0)) u_m (
      .clk(clk), .rst_n(rst_n),
      .arvalid(m_arvalid[s]), .arready(m_arready[s]), .araddr(m_araddr[s][AW-1:0]), .arlen(m_arlen[s]),
      .arsize(m_arsize[s]), .arburst(m_arburst[s]), .arid(m_arid[s]),
      .rvalid(m_rvalid[s]), .rready(m_rready[s]), .rdata(rdata), .rlast(m_rlast[s]), .rid(m_rid[s]),
      .rresp(m_rresp[s]),
      .awvalid(m_awvalid[s]), .awready(m_awready[s]), .awaddr(m_awaddr[s][AW-1:0]), .awlen(m_awlen[s]),
      .awsize(m_awsize[s]), .awburst(m_awburst[s]), .awid(m_awid[s]),
      .wvalid(m_wvalid[s]), .wready(m_wready[s]), .wdata(m_wdata[s][DW-1:0]), .wstrb(m_wstrb[s][DW/8-1:0]),
      .wlast(m_wlast[s]), .bvalid(m_bvalid[s]), .bready(m_bready[s]), .bid(m_bid[s]), .bresp(m_bresp[s])
    );
    assign m_rdata[s] = 512'(rdata);
    always @(posedge clk) begin
      if (m_arvalid[s] && m_arready[s]) ddr_ar[s]++;
      if (m_awvalid[s] && m_awready[s]) ddr_aw[s]++;
    end
  end

  function automatic logic [7:0] ddr_rd8(input logic [47:0] pa);
    ch_addr_s ca;
    ca = l2_ch_of(pa);
    case (l2_slice_idx(pa))
      0:       return g_m[0].u_m.peek(ca.off[32:0]);
      1:       return g_m[1].u_m.peek(ca.off[32:0]);
      2:       return g_m[2].u_m.peek(ca.off[31:0]);
      default: return g_m[3].u_m.peek(ca.off[31:0]);
    endcase
  endfunction
  function automatic void ddr_wr8(input logic [47:0] pa, input logic [7:0] v);
    ch_addr_s ca;
    ca = l2_ch_of(pa);
    case (l2_slice_idx(pa))
      0:       g_m[0].u_m.poke(ca.off[32:0], v);
      1:       g_m[1].u_m.poke(ca.off[32:0], v);
      2:       g_m[2].u_m.poke(ca.off[31:0], v);
      default: g_m[3].u_m.poke(ca.off[31:0], v);
    endcase
  endfunction
  function automatic void ddr_clear();
    g_m[0].u_m.clear_all();
    g_m[1].u_m.clear_all();
    g_m[2].u_m.clear_all();
    g_m[3].u_m.clear_all();
  endfunction

  // NoC traffic per slice (flits accepted by the slice)
  int sl_flits [4];
  for (genvar s = 0; s < 4; s++) begin : g_cnt
    always @(posedge clk) if (dut.g_sl[s].req_v && dut.g_sl[s].req_r) sl_flits[s]++;
  end

  // ---------------------------------------------------------------- CP port (WDIST path only)
  logic        d_req = 1'b0, d_we = 1'b0;
  logic [31:0] d_addr = 32'h0, d_wdata = 32'h0, d_rdata;
  logic        d_gnt, d_rvalid, d_err, p_fw_wr, p_irq, p_dbg_v, p_tick;
  logic [11:0] p_fw_waddr, p_fw_raddr;
  logic [31:0] p_fw_wdata, p_irq_set, p_dbg_code;
  logic [3:0]  p_ev_clr;
  logic        p_mreq_valid;
  mem_req_t    p_mreq;
  ng_cp_port u_cpport (
    .clk(clk), .rst_n(rst_n), .core_reset_i(core_reset),
    .d_req(d_req), .d_gnt(d_gnt), .d_rvalid(d_rvalid), .d_we(d_we), .d_be(4'hf), .d_addr(d_addr),
    .d_wdata(d_wdata), .d_rdata(d_rdata), .d_err(d_err),
    .fw_wr(p_fw_wr), .fw_waddr(p_fw_waddr), .fw_wdata(p_fw_wdata), .fw_raddr(p_fw_raddr), .fw_rdata(32'h0),
    .wptr_i(32'h0), .ev_pend_i(4'h0), .ev_clr_o(p_ev_clr), .irq_set_o(p_irq_set),
    .m_req_valid(p_mreq_valid), .m_req_ready(1'b0), .m_req(p_mreq),
    .m_rsp_valid(1'b0), .m_rsp_fault(1'b0), .m_rsp_data(64'h0),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .sp_idle_i(sp_idle), .wd_abort_o(), .fab_busy_i(1'b0), .scrub_set_i(16'h0), .sp_inv_o(), .tick_o(p_tick), .dbg_valid_o(p_dbg_v),
    .dbg_code_o(p_dbg_code), .busy_o(),
    .cb_valid(), .cb_fill(), .cb_vmid(), .cb_src(), .cb_dst(), .cb_size(), .cb_fillv(), .cb_abort(),
    .cb_done(1'b0), .cb_fault(1'b0)
  );

  task automatic port_wr(input logic [7:0] off, input logic [31:0] data);
    @(negedge clk);
    d_req = 1'b1; d_we = 1'b1; d_addr = PORT_BASE | 32'(off); d_wdata = data;
    @(negedge clk);
    d_req = 1'b0; d_we = 1'b0;
  endtask
  task automatic port_rd(input logic [7:0] off, output logic [31:0] data);
    @(negedge clk);
    d_req = 1'b1; d_we = 1'b0; d_addr = PORT_BASE | 32'(off);
    @(negedge clk);
    d_req = 1'b0;
    @(negedge clk);
    data = d_rdata;
  endtask

  // --------------------------------------------------------------
  logic [63:0] meta   [64];
  logic [31:0] wdw    [42];
  logic [63:0] pmap   [2 * MAXMAP];
  logic [63:0] mpages [MAXPG];
  logic [31:0] mpre   [MAXPG * 1024];
  logic [31:0] mpost  [MAXPG * 1024];
  logic [63:0] trc    [2 * MAXTR];
  logic [63:0] tidx   [MAXTR];

  int unsigned n_map, n_pages, n_trace, n_warps, m_status, m_vmid, m_memcmp;
  longint unsigned m_instr;
  longint unsigned n_wgs;

  // host (SYS / HOST) working memory: bundle pages by index, pages created by RTL writes appended
  logic [31:0] mem [MAXPG * 1024];
  int unsigned pidx [longint unsigned];   // PA page -> index (bundle pages: VRAM ones are kept in the DDR models)
  int unsigned n_alloc;
  int pg_words = 1024;
  int unsigned vidx [longint unsigned];   // VA page -> page-map entry
  bit          skip_pg [longint unsigned];  // PA pages excluded from the memory compare (scratch buffer)

  int errors = 0;
  function automatic void err(input string s);
    errors++;
    if (errors <= 30) $display("TB_ERROR %s", s);
  endfunction

  function automatic bit is_vram(input logic [47:0] pa);
    return pa < 48'h0004_0000_0000;
  endfunction

  string bdir;
  string rmode;
  int    n_resets;
  int    reset_at [16];
  task automatic read_runcfg();
    int fd, r;
    fd = $fopen("tb_core_run.txt", "r");
    if (fd == 0) begin $display("TB_ERROR cannot open tb_core_run.txt"); $finish; end
    r = $fscanf(fd, "%s", bdir);
    r = $fscanf(fd, "%s", rmode);
    n_resets = 0;
    r = $fscanf(fd, "%d", n_resets);
    if (n_resets > 16) n_resets = 16;
    for (int i = 0; i < n_resets; i++) r = $fscanf(fd, "%d", reset_at[i]);
    $fclose(fd);
  endtask

  task automatic load_bundle();
    longint unsigned scr_va, scr_end;
    for (int i = 0; i < 64; i++) meta[i] = '0;
    $readmemh({bdir, "/meta.hex"}, meta);
    if (meta[0] != 64'h5847_4255_4e44_0001) begin
      $display("TB_ERROR bad bundle magic %h in %s", meta[0], bdir);
      $finish;
    end
    n_map    = int'(meta[1]);
    n_pages  = int'(meta[2]);
    n_trace  = int'(meta[3]);
    n_warps  = int'(meta[4]);
    m_status = int'(meta[5]);
    m_instr  = meta[11];
    m_vmid   = int'(meta[12]);
    m_memcmp = int'(meta[13]);
    if (n_map > MAXMAP || n_pages > MAXPG || n_trace > MAXTR || n_warps > MAXTR) begin
      $display("TB_ERROR bundle too large (map %0d pages %0d trace %0d): raise NG_TB_MAX*", n_map, n_pages, n_trace);
      $finish;
    end
    $readmemh({bdir, "/wd.hex"}, wdw);
    $readmemh({bdir, "/pagemap.hex"}, pmap);
    $readmemh({bdir, "/mem_pages.hex"}, mpages);
    $readmemh({bdir, "/mem_pre.hex"}, mpre);
    $readmemh({bdir, "/mem_post.hex"}, mpost);
    $readmemh({bdir, "/trace.hex"}, trc);
    $readmemh({bdir, "/trace_idx.hex"}, tidx);
    vidx.delete();
    for (int i = 0; i < int'(n_map); i++) vidx[longint'(pmap[2 * i][35:0])] = i;
    n_wgs = longint'(wdw[9]) * longint'(wdw[10]) * longint'(wdw[11]);
    // scratch buffer (SCRATCH_VA, bytes per lane x 32 lanes x 64 warp slots, bundle_run.cpp): its contents are laid
    // out by GlobalWarpSlot, which is implementation defined (model placement != ng_core_wd placement)
    skip_pg.delete();
    scr_va  = {wdw[7], wdw[6]};
    scr_end = scr_va + longint'(wdw[8]) * 32 * 64;
    if (wdw[8] != 0)
      for (int i = 0; i < int'(n_map); i++) begin
        longint unsigned va;
        va = longint'(pmap[2 * i][35:0]) << 12;
        if (va >= scr_va && va < scr_end && !pmap[2 * i][59]) skip_pg[longint'(pmap[2 * i + 1][35:0])] = 1'b1;
      end
    $display("NOTE bundle %s: map=%0d pages=%0d trace=%0d warps=%0d status=%0d instr=%0d vmid=%0d workgroups=%0d scratch_pages=%0d",
             bdir, n_map, n_pages, n_trace, n_warps, m_status, m_instr, m_vmid, n_wgs, skip_pg.num());
  endtask

  int n_vram_pages = 0;
  task automatic reset_memory();
    pidx.delete();
    ddr_clear();
    n_vram_pages = 0;
    for (int i = 0; i < int'(n_pages); i++) begin
      logic [47:0] pa;
      pa = {mpages[i][35:0], 12'd0};
      pidx[longint'(mpages[i][35:0])] = i;
      if (is_vram(pa)) begin
        n_vram_pages++;
        for (int w = 0; w < 1024; w++) begin
          logic [31:0] v;
          v = mpre[i * 1024 + w];
          if (v != 32'd0)
            for (int b = 0; b < 4; b++) ddr_wr8(pa + 48'(4 * w + b), v[8 * b +: 8]);
          mem[i * 1024 + w] = 32'hxxxx_xxxx;       // never used for VRAM pages
        end
      end else begin
        for (int w = 0; w < 1024; w++) mem[i * 1024 + w] = mpre[i * 1024 + w];
      end
    end
    n_alloc = n_pages;
  endtask

  // ---------------------------------------------------------------- host memory helpers (uc path)
  function automatic logic [7:0] rd8(input logic [47:0] pa);
    longint unsigned pg;
    pg = longint'(pa[47:12]);
    if (!pidx.exists(pg)) return 8'h00;
    return mem[pidx[pg] * 1024 + int'(pa[11:2])][8 * pa[1:0] +: 8];
  endfunction
  function automatic void wr8(input logic [47:0] pa, input logic [7:0] v);
    longint unsigned pg;
    int unsigned ix;
    pg = longint'(pa[47:12]);
    if (!pidx.exists(pg)) begin
      if (n_alloc >= MAXPG) begin err($sformatf("memory pages exhausted at pa %h", pa)); return; end
      pidx[pg] = n_alloc;
      for (int w = 0; w < pg_words; w++) mem[n_alloc * 1024 + w] = '0;
      n_alloc++;
    end
    ix = pidx[pg] * 1024 + int'(pa[11:2]);
    mem[ix][8 * pa[1:0] +: 8] = v;
  endfunction

  // ---------------------------------------------------------------- MMU stub (the core's shared translation port)
  typedef enum logic [1:0] {M_IDLE, M_LAT, M_RSP} mst_t;
  mst_t        ms = M_IDLE;
  int          mlat;
  logic [63:0] m_va;
  logic [1:0]  m_acc;
  logic        m_atom;
  logic [3:0]  m_vm;
  int          n_mmu = 0;
  assign mmu_req_ready  = (ms == M_IDLE);
  assign mmu_resp_valid = (ms == M_RSP);
  always @(posedge clk) begin
    if (!rst_n) ms <= M_IDLE;
    else case (ms)
      M_IDLE: if (mmu_req_valid) begin
        m_va <= mmu_req_va; m_acc <= mmu_req_acc; m_atom <= mmu_req_atomic; m_vm <= mmu_req_vmid;
        mlat <= (MAXLAT > 0) ? $urandom_range(0, MAXLAT) : 0;
        n_mmu <= n_mmu + 1;
        ms <= M_LAT;
      end
      M_LAT: begin
        if (mlat > 0) mlat <= mlat - 1;
        else begin
          logic [2:0]  f;
          logic        sp;
          logic [47:0] pa;
          f = 3'd0; sp = 1'b0; pa = '0;
          if (m_va[63:48] != 16'd0) begin
            f = 3'd2;
          end else if (m_vm == 4'd0) begin
            pa = m_va[47:0];
          end else if (!vidx.exists(longint'(m_va[47:12]))) begin
            f = 3'd1;
          end else begin
            logic [63:0] e0, e1;
            logic        need_w;
            e0 = pmap[2 * vidx[longint'(m_va[47:12])]];
            e1 = pmap[2 * vidx[longint'(m_va[47:12])] + 1];
            need_w = (m_acc == 2'd1) || m_atom;
            if (e0[59]) sp = 1'b1;
            else if ((m_acc == 2'd2 && !e0[58]) || (need_w && !e0[57]) || (!need_w && m_acc != 2'd2 && !e0[56]))
              f = 3'd3;
            else pa = {e1[35:0], m_va[11:0]};
          end
          mmu_resp_fault <= f; mmu_resp_sparse <= sp; mmu_resp_pa <= pa;
          ms <= M_RSP;
        end
      end
      M_RSP: if (mmu_resp_ready) ms <= M_IDLE;
      default: ms <= M_IDLE;
    endcase
  end

  // ---------------------------------------------------------------- uc stub (SYS / HOST memory)
  logic [4:0]  amo_op_s;
  logic        amo64_s;
  logic [63:0] amo_a_s, amo_b_s, amo_cmp_s, amo_r_s;
  logic        amo_hit_s;
  ng_l2_amo_alu u_amo (.op(amo_op_s), .is64(amo64_s), .a(amo_a_s), .b(amo_b_s), .cmp(amo_cmp_s), .r(amo_r_s),
                       .cas_hit(amo_hit_s));
  mst_t   fs = M_IDLE;
  int     flat;
  mreq_t  f_rq;
  logic [63:0] f_old;
  int     n_uc_rd = 0, n_uc_wr = 0, n_uc_amo = 0;
  assign uc_req_ready = (fs == M_IDLE);
  assign uc_rsp_valid = (fs == M_RSP);
  assign amo_op_s  = f_rq.amo_op;
  assign amo64_s   = f_rq.amo64;
  assign amo_a_s   = f_rq.amo64 ? f_old : {32'd0, f_old[31:0]};
  assign amo_b_s   = f_rq.amo_a;
  assign amo_cmp_s = f_rq.amo_cmp;
  always @(posedge clk) begin
    if (!rst_n) fs <= M_IDLE;
    else case (fs)
      M_IDLE: if (uc_req_valid) begin
        logic [63:0] o;
        for (int b = 0; b < 8; b++) o[8 * b +: 8] = rd8(uc_req.pa + 48'(b));
        f_old <= o;
        f_rq <= uc_req;
        flat <= (MAXLAT > 0) ? $urandom_range(0, MAXLAT) : 0;
        if (is_vram(uc_req.pa) || !uc_req.uc)
          err($sformatf("uc port: request kind %0d uc %0d at VRAM / cached pa %h", uc_req.kind, uc_req.uc, uc_req.pa));
        fs   <= M_LAT;
      end
      M_LAT: begin
        if (flat > 0) flat <= flat - 1;
        else begin
          mrsp_t r;
          r = '0;
          case (f_rq.kind)
            M_RD: begin
              n_uc_rd <= n_uc_rd + 1;
              for (int b = 0; b < 128; b++) r.rdata[8 * b +: 8] = rd8({f_rq.pa[47:7], 7'd0} + 48'(b));
            end
            M_WR: begin
              n_uc_wr <= n_uc_wr + 1;
              for (int b = 0; b < 128; b++)
                if (f_rq.be[b]) wr8({f_rq.pa[47:7], 7'd0} + 48'(b), f_rq.wdata[8 * b +: 8]);
            end
            M_AMO: begin
              int nb;
              n_uc_amo <= n_uc_amo + 1;
              nb = f_rq.amo64 ? 8 : 4;
              r.old = amo_a_s;
              for (int b = 0; b < nb; b++) wr8(f_rq.pa + 48'(b), amo_r_s[8 * b +: 8]);
            end
            default: err($sformatf("uc port: unexpected request kind %0d", f_rq.kind));
          endcase
          uc_rsp <= r;
          fs <= M_RSP;
        end
      end
      M_RSP: fs <= M_IDLE;
      default: fs <= M_IDLE;
    endcase
  end

  // ---------------------------------------------------------------- CP line port (maintenance after the dispatch)
  task automatic cp_line(input logic [2:0] kind, output logic [1:0] e);
    int c;
    @(negedge clk);
    cp_req = '0;
    cp_req.kind = kind;
    cp_req_valid = 1'b1;
    c = 0;
    while (!cp_req_ready && c < 100000) begin @(negedge clk); c++; end
    @(negedge clk);
    cp_req_valid = 1'b0;
    c = 0;
    while (!cp_rsp_valid && c < 2000000) begin @(negedge clk); c++; end
    e = cp_rsp.err;
    if (c >= 2000000) err($sformatf("CP port: no response to kind %0d", kind));
  endtask

  // ---------------------------------------------------------------- commit trace check (all SPs)
  int unsigned tstart [int unsigned];
  int unsigned tcnt   [int unsigned];
  int unsigned tcur   [int unsigned];
  int          wg_sp  [int unsigned];    // linear WG id -> SP that committed its warps
  int          n_commit = 0;
  int          n_extra = 0;
  int          n_commit_raw = 0;
  int          sp_commits [NSP];
  int          multi_cyc = 0;            // cycles with commits on >= 2 SPs
  int          max_conc = 0;
  bit          trace_on = 1'b0;
  longint      t_c_first = -1, t_c_last = 0;

  task automatic reset_trace();
    int unsigned s;
    tstart.delete(); tcnt.delete(); tcur.delete(); wg_sp.delete();
    s = 0;
    for (int i = 0; i < int'(n_warps); i++) begin
      tstart[tidx[i][63:32]] = s;
      tcnt[tidx[i][63:32]]   = tidx[i][31:0];
      tcur[tidx[i][63:32]]   = 0;
      s += tidx[i][31:0];
    end
    n_commit = 0;
    n_extra = 0;
    for (int i = 0; i < NSP; i++) sp_commits[i] = 0;
    multi_cyc = 0;
    max_conc = 0;
    t_c_first = -1;
    t_c_last = 0;
  endtask

  always @(posedge clk) begin
    int nc;
    nc = 0;
    for (int i = 0; i < NSP; i++) if (c_valid[i]) begin
      n_commit_raw++;
      nc++;
    end
    if (trace_on) begin
      if (nc >= 2) multi_cyc++;
      if (nc > max_conc) max_conc = nc;
      if (nc > 0) begin
        if (t_c_first < 0) t_c_first = $time;
        t_c_last = $time;
      end
      for (int i = 0; i < NSP; i++) if (c_valid[i]) begin
        int unsigned key, ix;
        key = c_wg[i] * 32 + {27'd0, c_warp[i]};
        n_commit++;
        sp_commits[i]++;
        if (!wg_sp.exists(c_wg[i])) begin
          wg_sp[c_wg[i]] = i;
          if (int'(c_wg[i] % NSP) != i)
            err($sformatf("placement: workgroup %0d ran on SP %0d, the static interleave puts it on SP %0d",
                          c_wg[i], i, c_wg[i] % NSP));
        end else if (wg_sp[c_wg[i]] != i) begin
          err($sformatf("placement: workgroup %0d committed on SP %0d and SP %0d", c_wg[i], wg_sp[c_wg[i]], i));
        end
        if (!tcnt.exists(key) || tcur[key] >= tcnt[key]) begin
          n_extra++;
          if (m_status == 0)
            err($sformatf("extra commit (SP %0d) wg %0d warp %0d pc %h w0 %h (model trace of this warp has %0d)",
                          i, c_wg[i], c_warp[i], c_pc[i], c_w0[i], tcnt.exists(key) ? tcnt[key] : 0));
        end else begin
          ix = tstart[key] + tcur[key];
          if (trc[2 * ix][31:0] != c_pc[i] || trc[2 * ix + 1][31:0] != c_w0[i] || trc[2 * ix + 1][63:32] != c_exec[i])
            err($sformatf("MISMATCH (SP %0d) commit #%0d of wg %0d warp %0d: rtl pc %h w0 %h exec %h, model pc %h w0 %h exec %h",
                          i, tcur[key], c_wg[i], c_warp[i], c_pc[i], c_w0[i], c_exec[i],
                          trc[2 * ix][31:0], trc[2 * ix + 1][31:0], trc[2 * ix + 1][63:32]));
          tcur[key] = tcur[key] + 1;
        end
      end
    end
  end

  // ---------------------------------------------------------------- CP driver helpers
  task automatic port_send_request();
    for (int i = 0; i < WD_NWORDS - 1; i++) port_wr(P_WDIST_REQ, wdw[i]);
    port_wr(P_WDIST_REQ, 32'hffff_ffff);
  endtask

  int          done_seen = 0;
  logic        done_fault;
  logic [63:0] done_instr;
  always @(posedge clk) if (wd_done) begin
    done_seen <= done_seen + 1;
    done_fault <= wd_fault;
    done_instr <= wd_instr;
  end

  task automatic start_dispatch();
    @(negedge clk);
    if (!sp_idle) err("sp_idle is 0 before the dispatch");
    port_send_request();
  endtask

  task automatic wait_done(input int n_before, output bit ok);
    int c;
    bit idle_low;
    c = 0;
    idle_low = 1'b0;
    ok = 1'b0;
    while (c < TIMEOUT) begin
      @(negedge clk);
      c++;
      if (!sp_idle) idle_low = 1'b1;
      if (done_seen != n_before) begin ok = 1'b1; break; end
    end
    if (ok && !idle_low && n_warps != 0) err("sp_idle never dropped during the dispatch");
    if (!ok) err($sformatf("timeout: no wd_done after %0d cycles (commits %0d)", TIMEOUT, n_commit));
  endtask

  task automatic wait_idle(input int max);
    int c;
    c = 0;
    while (!sp_idle && c < max) begin @(negedge clk); c++; end
    if (!sp_idle) err($sformatf("sp_idle not back within %0d cycles", max));
  endtask

  task automatic wait_quiet(input int max);
    int c;
    c = 0;
    while ((fab_busy || fs != M_IDLE || ms != M_IDLE) && c < max) begin @(negedge clk); c++; end
    if (fab_busy) err($sformatf("fabric still busy after %0d cycles", max));
  endtask

  // ---------------------------------------------------------------- final checks
  task automatic check_result();
    int mism, n_zero_pages;
    logic [1:0] e;
    bit do_mem;
    if (m_status == 0) begin
      if (done_fault) err($sformatf("wd_fault set (rtl trap SP %0d unit %0d slot %0d pc %h cause %0d info %h)",
                                    dbg_trap_sp, dbg_trap_unit, dbg_trap_slot, dbg_trap_pc, dbg_trap_cause, dbg_trap_info));
      foreach (tcnt[k])
        if (tcur[k] != tcnt[k])
          err($sformatf("warp key %0d (wg %0d warp %0d): rtl committed %0d of %0d model instructions",
                        k, k / 32, k % 32, tcur[k], tcnt[k]));
      if (n_wgs >= NSP)
        for (int i = 0; i < NSP; i++) if (sp_commits[i] == 0) err($sformatf("SP %0d committed nothing", i));
    end else begin
      if (!done_fault) err("model trapped but wd_fault is 0");
      else begin
        if (dbg_trap_wg != meta[6][31:0] || dbg_trap_warp != meta[7][4:0] || dbg_trap_pc != meta[8][31:0] ||
            dbg_trap_cause != meta[9][3:0] || dbg_trap_info != meta[10][31:0])
          err($sformatf("trap record rtl SP %0d wg %0d warp %0d pc %h cause %0d info %h / model wg %0d warp %0d pc %h cause %0d info %h",
                        dbg_trap_sp, dbg_trap_wg, dbg_trap_warp, dbg_trap_pc, dbg_trap_cause, dbg_trap_info,
                        meta[6][31:0], meta[7][4:0], meta[8][31:0], meta[9][3:0], meta[10][31:0]));
        else $display("NOTE trap matches: SP %0d unit %0d wg %0d warp %0d pc %h cause %0d info %h (SPs aborted %b)",
                      dbg_trap_sp, dbg_trap_unit, dbg_trap_wg, dbg_trap_warp, dbg_trap_pc, dbg_trap_cause,
                      dbg_trap_info, dbg_sp_aborted);
      end
    end
    if (m_status != 0 && n_warps > 1)
      $display("NOTE WDIST instruction count not compared (trap bundle with %0d warps): rtl %0d model %0d",
               n_warps, done_instr, m_instr);
    else if (done_instr != m_instr)
      err($sformatf("WDIST instruction count rtl %0d model %0d", done_instr, m_instr));

    // memory: write the L2 back through the CP port, then compare the DDR models / host memory
    cp_line(M_WB_L2, e);
    if (e != 2'd0) err($sformatf("CP port wb.l2: err %0d", e));
    do_mem = (m_memcmp != 0) && (m_status == 0 || n_warps == 1);
    if (do_mem) begin
      mism = 0;
      for (int i = 0; i < int'(n_pages); i++) begin
        logic [47:0] pa;
        pa = {mpages[i][35:0], 12'd0};
        if (skip_pg.exists(longint'(mpages[i][35:0]))) continue;
        for (int w = 0; w < 1024; w++) begin
          logic [31:0] v;
          if (is_vram(pa)) for (int b = 0; b < 4; b++) v[8 * b +: 8] = ddr_rd8(pa + 48'(4 * w + b));
          else v = mem[i * 1024 + w];
          if (v !== mpost[i * 1024 + w]) begin
            mism++;
            if (mism <= 10)
              err($sformatf("MISMATCH memory pa %h (%s): rtl %h model %h", pa | 48'(4 * w),
                            is_vram(pa) ? "VRAM" : "host", v, mpost[i * 1024 + w]));
          end
        end
      end
      // mapped VRAM pages the model left zero (not in the bundle) must still be zero
      n_zero_pages = 0;
      for (int i = 0; i < int'(n_map); i++) begin
        logic [47:0] pa;
        pa = {pmap[2 * i + 1][35:0], 12'd0};
        if (pmap[2 * i][59] || !is_vram(pa) || pidx.exists(longint'(pmap[2 * i + 1][35:0]))) continue;
        if (skip_pg.exists(longint'(pmap[2 * i + 1][35:0]))) continue;
        n_zero_pages++;
        for (int b = 0; b < 4096; b++)
          if (ddr_rd8(pa + 48'(b)) != 8'h00) begin
            mism++;
            if (mism <= 10) err($sformatf("MISMATCH memory: rtl wrote %h to VRAM pa %h the model left zero",
                                          ddr_rd8(pa + 48'(b)), pa + 48'(b)));
            break;
          end
      end
      for (int i = int'(n_pages); i < int'(n_alloc); i++) begin
        bit scr_pg;
        scr_pg = 1'b0;
        foreach (pidx[k]) if (int'(pidx[k]) == i && skip_pg.exists(k)) scr_pg = 1'b1;
        if (scr_pg) continue;   // host (GTT) scratch pages: layout follows GlobalWarpSlot, not compared
        for (int w = 0; w < 1024; w++)
          if (mem[i * 1024 + w] != 32'd0) begin
            mism++;
            if (mism <= 10) err($sformatf("MISMATCH memory: rtl wrote nonzero word %h to a host page the model left zero",
                                          mem[i * 1024 + w]));
          end
      end
      $display("NOTE memory compare: %0d pages (%0d VRAM via L2 + DDR, %0d scratch skipped), %0d zero VRAM pages, +%0d rtl-created host pages, %0d mismatching words",
               n_pages, n_vram_pages, skip_pg.num(), n_zero_pages, n_alloc - n_pages, mism);
    end else begin
      $display("NOTE memory compare skipped (%s)", m_memcmp == 0 ? "bundle flag" : "multi-warp trap bundle");
    end
    $display("NOTE commits %0d (extra beyond model %0d), per SP %0d %0d %0d %0d, wd_instr %0d, model %0d, cycles with >= 2 SPs committing %0d (max %0d / cycle)",
             n_commit, n_extra, sp_commits[0], NSP > 1 ? sp_commits[NSP > 1 ? 1 : 0] : 0,
             NSP > 2 ? sp_commits[NSP > 2 ? 2 : 0] : 0, NSP > 3 ? sp_commits[NSP > 3 ? 3 : 0] : 0,
             done_instr, m_instr, multi_cyc, max_conc);
    $display("NOTE mmu %0d, uc rd %0d wr %0d amo %0d, slice flits %0d %0d %0d %0d, DDR reads %0d %0d %0d %0d writes %0d %0d %0d %0d, first-to-last commit %0d cycles",
             n_mmu, n_uc_rd, n_uc_wr, n_uc_amo, sl_flits[0], sl_flits[1], sl_flits[2], sl_flits[3],
             ddr_ar[0], ddr_ar[1], ddr_ar[2], ddr_ar[3], ddr_aw[0], ddr_aw[1], ddr_aw[2], ddr_aw[3],
             (t_c_last - t_c_first) / 10);
  endtask

  // --------------------------------------------------------------
  initial begin
    bit ok;
    int n_before;
    logic [31:0] st0;
    logic [1:0] e;
    for (int s = 0; s < 4; s++) begin ddr_ar[s] = 0; ddr_aw[s] = 0; sl_flits[s] = 0; end
    read_runcfg();
    load_bundle();
    reset_memory();
    reset_trace();
    repeat (5) @(negedge clk);
    rst_n = 1'b1;
    repeat (5) @(negedge clk);

    // ---- configuration fault: 16 warps x 128 registers cannot fit an SP's register file (every SP faults) ----
    begin
      logic [31:0] save0, save12, save13, save14;
      int nc0;
      save0 = wdw[0]; save12 = wdw[12]; save13 = wdw[13]; save14 = wdw[14];
      wdw[0] = (wdw[0] & 32'hff) | (32'd128 << 8);
      wdw[12] = 32'd512; wdw[13] = 32'd1; wdw[14] = 32'd1;
      n_before = done_seen;
      nc0 = n_commit_raw;
      start_dispatch();
      wait_done(n_before, ok);
      repeat (4) @(negedge clk);
      if (ok && !done_fault) err("16 warps x 128 vregs: expected wd_fault");
      if (n_commit_raw != nc0) err("16 warps x 128 vregs: instructions committed");
      wait_idle(50);
      wdw[0] = save0; wdw[12] = save12; wdw[13] = save13; wdw[14] = save14;
      port_rd(P_WDIST_STATUS, st0);
      if (st0[0] !== 1'b0) err("WDIST busy after the configuration fault");
      $display("NOTE config-fault case (16 warps x 128 vregs): wd_done fault=%0d", done_fault);
    end

    // ---- aborted runs (reset mode) ----
    for (int ri = 0; ri < (rmode == "reset" ? n_resets : 0); ri++) begin
      int c, commits_after, base;
      logic [31:0] st32;
      trace_on = 1'b0;
      n_before = done_seen;
      base = n_commit_raw;
      start_dispatch();
      c = 0;
      while (c < TIMEOUT && n_commit_raw - base < reset_at[ri] && done_seen == n_before) begin @(negedge clk); c++; end
      if (done_seen != n_before) err("dispatch finished before the core reset point (raise the kernel length)");
      c = 0;
      while (c < 400 && done_seen == n_before && !((ri % 2 == 0) ? fab_busy : (ms != M_IDLE))) begin
        @(negedge clk); c++;
      end
      if (sp_idle) err("sp_idle is 1 while the dispatch runs");
      $display("NOTE core_reset after %0d commits (in flight: mmu %0d uc %0d fabric %0d)", n_commit_raw - base,
               ms != M_IDLE, fs != M_IDLE, fab_busy);
      port_rd(P_WDIST_STATUS, st32);
      if (st32[0] !== 1'b1 && done_seen == n_before) err("WDIST busy not set while the dispatch runs");
      core_reset = 1'b1;
      repeat (2) @(negedge clk);
      commits_after = n_commit_raw;
      repeat (6) @(negedge clk);
      if (n_commit_raw != commits_after) err("commits continued during core_reset");
      core_reset = 1'b0;
      repeat (2) @(negedge clk);
      wait_idle(20);
      repeat (50) @(negedge clk);
      if (n_commit_raw != commits_after) err("commits after core_reset release");
      if (done_seen != n_before)
        err($sformatf("aborted dispatch: %0d wd_done pulses (expected none)", done_seen - n_before));
      port_rd(P_WDIST_STATUS, st32);
      if (st32[0] !== 1'b0) err($sformatf("aborted dispatch: WDIST_STATUS %h, busy still set after core_reset", st32));
      // transactions in flight across the reset complete (and are dropped by the SPs); then empty the L2 and reload
      wait_quiet(200000);
      cp_line(M_INV_L2, e);
      if (e != 2'd0) err($sformatf("CP port inv.l2: err %0d", e));
      reset_memory();
    end

    // ---- checked run ----
    reset_trace();
    trace_on = 1'b1;
    n_before = done_seen;
    start_dispatch();
    wait_done(n_before, ok);
    repeat (2) @(negedge clk);
    if (ok) begin
      wait_idle(200);
      wait_quiet(200000);
      check_result();
    end
    trace_on = 1'b0;
    if (errors == 0) $display("NG_CORE_PASS %s", bdir);
    else             $display("NG_CORE_FAIL %s errors=%0d", bdir, errors);
    $finish;
  end

  logic unused;
  assign unused = ^{ext_flt_valid, ext_flt_vmid, ext_flt_va, ext_flt_acc, mmu_req_tag, amo_hit_s, dbg_sp_idle,
                    p_mreq_valid, p_mreq, p_fw_wr, p_irq_set, p_ev_clr, p_dbg_v, p_dbg_code, p_tick, d_gnt,
                    d_rvalid, d_err};
endmodule
