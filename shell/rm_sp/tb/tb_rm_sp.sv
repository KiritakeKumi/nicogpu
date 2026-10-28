// tb_rm_sp.sv -- system test of the M1 SP RM (ng_gpu_rm of rm_sp = rm_cp + one SP) with the real Ibex firmware.
//
// Part 1 is tb_rm_cp.sv (/) unchanged except the identity (rm_kind 0x8003, CAPS 0x3B102011 since): every rm_cp
// check runs against the new top with the SP present.
// Part 2  runs functional-model dispatch bundles (tools/dispatch_bundle; directory list
// rm_sp_bundles.txt, written by run_tb_rm_sp.sh) through the whole stack: the bundle's PA pages are relocated into C3
// (SYS), page tables for the bundle's VA map are built in C3, everything is written through the BAR2 window, the
// MMU context of the bundle's VMID is enabled through BAR0, and an IB with SET_SHADER / SET_USER_REGS / DISPATCH
// (rebuilt from the bundle's WDIST words) is submitted through the ring.  Checks: the work-distributor request the
// firmware builds equals the bundle's 42 WDIST words; ok bundles: no CP error, retired instructions = model, final
// memory = model on every page; trap bundles: CP_ERROR = SHADER_FAULT, the debug page trap record = model, CP RESET
// recovery.  Plus directed cases: a buffer page in host memory, BAR2 traffic concurrent with a dispatch, a code page
// mapped to VRAM (fabric error -> fetch fault -> SHADER_FAULT), the BAR0 debug page.
//
// Original header (tb_rm_cp.sv):
// tb_rm_cp.sv -- system test of the CP RM (ng_gpu_rm of rm_cp) with the real Ibex firmware.
//
// The TB plays the kernel: window C3 via BAR0, NGCF image + ring into C3 through BAR2, FW_BASE/FW_SIZE, release
// CORE_RESET, wait for READY, ring + doorbell, FENCE jobs whose fence lands in host memory (tb_rmt_slave on
// m_axi_host) and raises IRQ_STATUS.FENCE -> rm_irq.  Also: boot errors (BOOT_ERR, IRQ_CP_FW), a CP error from an
// INDIRECT with a user VMID that has no MMU context, RESET recovery, the MMU (page tables in C3, IB at a VA,
// TLB hit / stale entry / MMU_INVALIDATE, sparse, NOT_PRESENT / PERMISSION / BAD_PHYS faults, IRQ_STATUS bit 2), C3 window rules (ERR, VRAM SLVERR), BAR2
// bursts / narrow beats, and quiesce.  Build: run_tb_rm_cp.sh (Verilator + --timing, Ibex).  Pass: NG_RM_CP_TB_PASS.
`timescale 1ns / 1ps

module tb_rm_sp;
`include "tb_rm_sp_decls.svh"

  int errors = 0;
  int checks = 0;

  // clocks: clk_l2 250 MHz, clk_ts 100 MHz (asynchronous)
  initial begin clk_l2 = 0; forever #2.0 clk_l2 = ~clk_l2; end
  initial begin clk_ts = 0; #1.3; forever #5.0 clk_ts = ~clk_ts; end
  initial begin clk_core = 0; forever #2.5 clk_core = ~clk_core; end
  initial begin clk_core2x = 0; forever #1.25 clk_core2x = ~clk_core2x; end

  parameter bit SP_BURST = 1'b1;      // -GSP_BURST=0 (VOPTS) runs the operation path through the same top
  parameter int SP_NOUT  = 8;         // fabric slots (1 = one request at a time)
  parameter int SP_MEM_OUT = SP_NOUT; // 1 = serial SP port through ng_rsp_mport1, SP_NOUT = slot port
  parameter bit SP_WR_RUNS = 1'b1;    // host partial writes as bursts of runs (0 = rule)
  parameter int SP_LSU_K   = 4;       // LSU contexts (1 = ng_lsu; the RM default 4)
  ng_gpu_rm #(.BUILD_ID(32'h1234abcd), .RM_REV(16'h0007), .SP_BURST(SP_BURST), .SP_NOUT(SP_NOUT),
              .SP_MEM_OUT(SP_MEM_OUT), .SP_WR_RUNS(SP_WR_RUNS), .SP_LSU_K(SP_LSU_K)) dut (.*);

  // clk_ts cycles WDIST_STATUS.ABORTING waited for the fabric (ng_cp_port ab_wait), abort watchdog expiries
  int ab_wait_cyc = 0, ab_wait_max = 0, ab_wait_cur = 0, ab_tmo = 0;
  bit ab_tmo_d = 1'b0;
  always @(posedge clk_ts) begin
    if (dut.u_cp.u_port.ab_wait) begin ab_wait_cyc++; ab_wait_cur++; if (ab_wait_cur > ab_wait_max) ab_wait_max = ab_wait_cur; end
    else ab_wait_cur = 0;
    if (dut.u_cp.u_port.ab_tmo_q && !ab_tmo_d) ab_tmo++;
    ab_tmo_d = dut.u_cp.u_port.ab_tmo_q;
  end

  // --
  tb_rmt_slave #(.DW(128), .AW(31), .IW(6), .MAXLEN(15), .MAXWR(16), .MAXRD(16), .NAME("c3")) u_c3 (
    .clk(clk_l2), .rst_n(rst_l2_n),
    .awid(m_axi_c3_awid), .awaddr(m_axi_c3_awaddr), .awlen(m_axi_c3_awlen), .awsize(m_axi_c3_awsize),
    .awburst(m_axi_c3_awburst), .awcache(m_axi_c3_awcache), .awprot(m_axi_c3_awprot), .awqos(m_axi_c3_awqos),
    .awvalid(m_axi_c3_awvalid), .awready(m_axi_c3_awready),
    .wdata(m_axi_c3_wdata), .wstrb(m_axi_c3_wstrb), .wlast(m_axi_c3_wlast), .wvalid(m_axi_c3_wvalid),
    .wready(m_axi_c3_wready), .bid(m_axi_c3_bid), .bresp(m_axi_c3_bresp), .bvalid(m_axi_c3_bvalid),
    .bready(m_axi_c3_bready),
    .arid(m_axi_c3_arid), .araddr(m_axi_c3_araddr), .arlen(m_axi_c3_arlen), .arsize(m_axi_c3_arsize),
    .arburst(m_axi_c3_arburst), .arcache(m_axi_c3_arcache), .arprot(m_axi_c3_arprot), .arqos(m_axi_c3_arqos),
    .arvalid(m_axi_c3_arvalid), .arready(m_axi_c3_arready),
    .rid(m_axi_c3_rid), .rdata(m_axi_c3_rdata), .rresp(m_axi_c3_rresp), .rlast(m_axi_c3_rlast),
    .rvalid(m_axi_c3_rvalid), .rready(m_axi_c3_rready));

  tb_rmt_slave #(.DW(64), .AW(47), .IW(4), .MAXLEN(63), .MAXWR(8), .MAXRD(8), .NAME("host"), .XDMA_STRB(1)) u_host (
    .clk(clk_l2), .rst_n(rst_l2_n),
    .awid(m_axi_host_awid), .awaddr(m_axi_host_awaddr), .awlen(m_axi_host_awlen), .awsize(m_axi_host_awsize),
    .awburst(m_axi_host_awburst), .awcache(m_axi_host_awcache), .awprot(m_axi_host_awprot), .awqos(m_axi_host_awqos),
    .awvalid(m_axi_host_awvalid), .awready(m_axi_host_awready),
    .wdata(m_axi_host_wdata), .wstrb(m_axi_host_wstrb), .wlast(m_axi_host_wlast), .wvalid(m_axi_host_wvalid),
    .wready(m_axi_host_wready), .bid(m_axi_host_bid), .bresp(m_axi_host_bresp), .bvalid(m_axi_host_bvalid),
    .bready(m_axi_host_bready),
    .arid(m_axi_host_arid), .araddr(m_axi_host_araddr), .arlen(m_axi_host_arlen), .arsize(m_axi_host_arsize),
    .arburst(m_axi_host_arburst), .arcache(m_axi_host_arcache), .arprot(m_axi_host_arprot), .arqos(m_axi_host_arqos),
    .arvalid(m_axi_host_arvalid), .arready(m_axi_host_arready),
    .rid(m_axi_host_rid), .rdata(m_axi_host_rdata), .rresp(m_axi_host_rresp), .rlast(m_axi_host_rlast),
    .rvalid(m_axi_host_rvalid), .rready(m_axi_host_rready));

`include "tb_rm_cp_common.svh"

  function automatic logic [7:0] host_b(input longint unsigned a);
    logic [63:0] q = host_q(a & ~64'h7);
    return q[8 * a[2:0] +: 8];
  endfunction
  function automatic logic [7:0] c3_b(input longint unsigned off);
    logic [63:0] q = c3_q(off & ~64'h7);
    return q[8 * off[2:0] +: 8];
  endfunction
  // COPY (op 4, 7 dwords) / FILL (op 11, 5 dwords) on the ring (VMID 0: SYS / HOST physical addresses)
  task automatic copy_pkt(input longint unsigned src, input longint unsigned dst, input longint unsigned n);
    ring_put(32'h0007_0004);
    ring_put(src[31:0]); ring_put(src[63:32]); ring_put(dst[31:0]); ring_put(dst[63:32]);
    ring_put(n[31:0]); ring_put(n[63:32]); ring_put(32'h0);
  endtask
  task automatic fill_pkt(input longint unsigned dst, input longint unsigned n, input logic [31:0] val);
    ring_put(32'h0005_000B);
    ring_put(dst[31:0]); ring_put(dst[63:32]); ring_put(n[31:0]); ring_put(n[63:32]); ring_put(val);
  endtask

  // ================================================================================================================
  // SP part
  // ================================================================================================================
  localparam int MAXPG = 64, MAXMAP = 64;
  localparam longint unsigned PT_POOL   = 64'h10_0000;            // C3: page tables of the bundle runs
  localparam longint unsigned DATA_POOL = 64'h20_0000;            // C3: relocated bundle pages + IB pages
  localparam longint unsigned HOST_POOL = 64'h10_0000;            // host memory: pages of the host variant
  localparam longint unsigned IB_VA     = 64'h2_0000_0000;        // VA of the IB page (not used by the bundles)
  localparam longint unsigned VRAM_PA   = 64'h4000_0000;          // a VRAM PA (not reachable in this RM)
  localparam int V_PLAIN = 0, V_HOST = 1, V_VRAM_CODE = 2, V_ABORT = 3, V_ABORT2 = 4;   // V_ABORT*: CP_CTRL.RESET mid-dispatch

  logic [63:0] b_meta  [64];
  logic [31:0] b_wd    [42];
  logic [63:0] b_pmap  [2 * MAXMAP];
  logic [63:0] b_pages [MAXPG];
  logic [31:0] b_pre   [MAXPG * 1024];
  logic [31:0] b_post  [MAXPG * 1024];
  longint unsigned pt_next = PT_POOL, data_next = DATA_POOL, host_next = HOST_POOL, pt_root;
  logic [63:0]     ptq   [longint unsigned];                      // page-table qwords (C3 offset -> value)
  longint unsigned reloc [longint unsigned];                      // bundle PA page number -> relocated PA
  bit rd_stop, rd_done;                                           // concurrent BAR2 reader
  longint unsigned last_ib_off;                                   // C3 offset / VMID / size of the last bundle's IB
  int last_vmid, last_nw;
  int n_conc;

  // work-distributor port monitor (clk_ts, the CP <-> SP interface inside the RM)
  int                 wd_starts = 0, wd_dones = 0;
  ng_cp_pkg::wd_req_t wd_cap;
  logic               wd_cap_fault;
  logic [63:0]        wd_cap_instr;
  always @(posedge clk_ts) begin
    if (dut.wd_start) begin wd_starts <= wd_starts + 1; wd_cap <= dut.wd_req; end
    if (dut.wd_done) begin wd_dones <= wd_dones + 1; wd_cap_fault <= dut.wd_fault; wd_cap_instr <= dut.wd_instr; end
  end

  function automatic longint unsigned pt_alloc();
    longint unsigned r = pt_next;
    pt_next += 64'h1000;
    return r;
  endfunction

  // 4-level table walk (9 bits per level, 4 KiB leaves), tables allocated on demand in the PT pool
  function automatic void pt_map(input longint unsigned va, input logic [63:0] leaf);
    longint unsigned t = pt_root;
    for (int lv = 3; lv >= 1; lv--) begin
      longint unsigned e = t + 8 * ((va >> (12 + 9 * lv)) & 64'h1FF);
      if (!ptq.exists(e)) ptq[e] = (SYS_BASE + pt_alloc()) | 64'h1;
      t = (ptq[e] & 64'h0000_FFFF_FFFF_F000) - SYS_BASE;
    end
    ptq[t + 8 * ((va >> 12) & 64'h1FF)] = leaf;
  endfunction

  task automatic load_bundle(input string dir);
    for (int i = 0; i < 64; i++) b_meta[i] = '0;
    $readmemh({dir, "/meta.hex"}, b_meta);
    $readmemh({dir, "/wd.hex"}, b_wd);
    $readmemh({dir, "/pagemap.hex"}, b_pmap);
    $readmemh({dir, "/mem_pages.hex"}, b_pages);
    $readmemh({dir, "/mem_pre.hex"}, b_pre);
    $readmemh({dir, "/mem_post.hex"}, b_post);
    if (b_meta[0] != 64'h5847_4255_4e44_0001) begin
      errors++; $display("FAIL: bad bundle %s (meta[0] %h)", dir, b_meta[0]);
    end
  endtask

  // relocated qword read (C3 or host memory, straight from the slave models)
  function automatic logic [63:0] phys_q(input longint unsigned pa);
    if (pa >= HOST_BASE) return host_q(pa - HOST_BASE);
    return c3_q(pa - SYS_BASE);
  endfunction

  task automatic dbg_rd(input logic [11:0] off, output logic [31:0] d); lr({8'h0, off}, d); endtask

  // run one bundle; variant: V_PLAIN, V_HOST (pages with W permission in host memory), V_VRAM_CODE (code page
  // mapped to a VRAM PA: expected fetch fault).  conc = 1: BAR2 reads of the IB page run during the dispatch.
  task automatic run_bundle(input string name, input int variant, input bit conc);
    int n_map, n_pages, status, vmid, ib_n0, starts0, dones0, mism, nw;
    longint unsigned instr, ib_off, va, pa_pn, root_pa;
    logic [7:0]  perm;
    logic [31:0] w32, cm0, cm1, ds0, dn0, d;
    logic [63:0] q;
    bit ok, okj, expect_trap;
    logic [31:0] ib [64];
    $display("TEST sp bundle %s variant %0d%s", name, variant, conc ? " (concurrent BAR2)" : "");
    load_bundle({"bundles/", name});
    n_map = int'(b_meta[1]); n_pages = int'(b_meta[2]); status = int'(b_meta[5]);
    instr = b_meta[11]; vmid = int'(b_meta[12]);
    expect_trap = (status != 0) || (variant == V_VRAM_CODE);
    if (n_map > MAXMAP || n_pages > MAXPG) begin errors++; $display("FAIL: bundle %s too large", name); return; end

    // ---- relocation + page tables
    ptq.delete(); reloc.delete();
    pt_root = pt_alloc();
    for (int i = 0; i < n_map; i++) begin
      perm = b_pmap[2 * i][63:56];
      pa_pn = b_pmap[2 * i + 1];
      va = {b_pmap[2 * i][35:0], 12'h0};
      if (perm[3]) begin pt_map(va, 64'h100); continue; end                 // sparse-null
      if (!reloc.exists(pa_pn)) begin
        if (variant == V_HOST && perm[1]) begin reloc[pa_pn] = HOST_BASE + host_next; host_next += 64'h1000; end
        else if (variant == V_VRAM_CODE && perm[2]) reloc[pa_pn] = VRAM_PA;
        else begin reloc[pa_pn] = SYS_BASE + data_next; data_next += 64'h1000; end
      end
      pt_map(va, reloc[pa_pn] | 64'h3 | (perm[0] ? 64'h4 : 0) | (perm[1] ? 64'h8 : 0) | (perm[2] ? 64'h10 : 0));
    end
    for (int j = 0; j < n_pages; j++)
      if (!reloc.exists(b_pages[j])) begin reloc[b_pages[j]] = SYS_BASE + data_next; data_next += 64'h1000; end
    ib_off = data_next; data_next += 64'h1000;
    last_ib_off = ib_off; last_vmid = vmid;
    pt_map(IB_VA, (SYS_BASE + ib_off) | 64'h7);
    foreach (ptq[k]) put_q(k, ptq[k]);

    // ---- pre-image (SYS pages through the BAR2 window, host pages into the host model, VRAM: unreachable)
    for (int j = 0; j < n_pages; j++) begin
      longint unsigned p = reloc[b_pages[j]];
      if (p >= HOST_BASE) begin
        for (int k = 0; k < 512; k++)
          u_host.poke((p - HOST_BASE) / 8 + k, {b_pre[j * 1024 + 2 * k + 1], b_pre[j * 1024 + 2 * k]});
      end else if (p >= SYS_BASE) begin
        for (int c = 0; c < 64; c++) begin
          logic [63:0] dd [];
          logic [1:0] r;
          bit nz = 0;
          dd = new[8];
          for (int k = 0; k < 8; k++) begin
            dd[k] = {b_pre[j * 1024 + 16 * c + 2 * k + 1], b_pre[j * 1024 + 16 * c + 2 * k]};
            if (dd[k] != 0) nz = 1;
          end
          if (nz) begin
            b2_write(WIN_LO + 28'(p - SYS_BASE + 64 * c), dd, 8'hFF, 3'd3, r);
            check(r == 2'b00, "bundle pre-image BAR2 write");
          end
        end
      end
    end

    // ---- IB: SET_SHADER, SET_USER_REGS (u0..u25), DISPATCH -- rebuilt from the WDIST words
    ib[0] = 32'h000B_0006;
    ib[1] = b_wd[4]; ib[2] = b_wd[5] & 32'hFFFF; ib[3] = b_wd[2] - b_wd[4];
    ib[4] = b_wd[0] >> 8; ib[5] = b_wd[1]; ib[6] = b_wd[8]; ib[7] = b_wd[6]; ib[8] = b_wd[7];
    ib[9] = b_wd[12]; ib[10] = b_wd[13]; ib[11] = b_wd[14];
    ib[12] = 32'h001B_0007; ib[13] = 32'h0;
    for (int i = 0; i < 26; i++) ib[14 + i] = b_wd[15 + i];
    ib[40] = 32'h0003_0005; ib[41] = b_wd[9]; ib[42] = b_wd[10]; ib[43] = b_wd[11];
    nw = 44;                                                               // even: no NOP padding needed
    last_nw = nw;
    begin
      logic [31:0] t [];
      t = new[nw];
      for (int i = 0; i < nw; i++) t[i] = ib[i];
      put_c3(ib_off, t, nw);
    end

    // ---- MMU context of the bundle's VMID (CTRL latches the root and invalidates the VMID)
    root_pa = SYS_BASE + pt_root;
    lw(20'h200 + 20'(16 * vmid), root_pa[31:0]); lw(20'h204 + 20'(16 * vmid), root_pa[47:32]);
    lw(20'h208 + 20'(16 * vmid), 32'h1);
    okj = 0;
    for (int t = 0; t < 50 && !okj; t++) begin lr(20'h300, v); okj = (v == 0); end
    check(okj, "bundle: MMU context enable / invalidate completes");
    lr(20'h310, v); check(v[0] == 1'b0, $sformatf("bundle %s: no stale MMU fault before the run (%08h)", name, v));

    dbg_rd(12'h824, cm0); dbg_rd(12'h828, ds0); dbg_rd(12'h82C, dn0);
    starts0 = wd_starts; dones0 = wd_dones;

    // ---- submit, wait (concurrent BAR2 reader optional)
    rd_stop = 0; rd_done = !conc; n_conc = 0;
    run_ib_noerrwait(vmid, IB_VA, nw);
    if (variant == V_ABORT || variant == V_ABORT2) begin
      // CP_CTRL.RESET while the long kernel runs -> the firmware aborts the dispatch in the work distributor,
      // keeps its heartbeat, and the CP is usable again (the caller runs a normal bundle next)
      logic [31:0] hb0, hb1, cmx, cmy, st_x;
      int thr = (variant == V_ABORT) ? 100 : 350;   // abort depth in committed instructions (the loop is memory-bound, ~400 commits in 12000 polls)
      ok = 0;
      for (int t = 0; t < 12000 && !ok; t++) begin
        dbg_rd(12'h824, cm1); ok = (cm1 - cm0 >= thr);
        if (!ok) repeat (20) @(posedge clk_l2);
      end
      check(ok, $sformatf("abort %s: kernel running (%0d commits)", name, cm1 - cm0));
      dbg_rd(12'h804, d); check(!d[30] && d[20], $sformatf("abort %s: SP / WD busy before the reset (%08h)", name, d));
      check(wd_starts == starts0 + 1 && wd_dones == dones0, $sformatf("abort %s: one wd_start, no wd_done yet", name));
      cpr(13'h012c, hb0);
      cpw(13'h0114, 32'h3);                                          // ENABLE | RESET (what the driver's L1 writes)
      wait_reg(13'h0118, 32'h1, 32'h1, 4000, ok);
      check(ok, $sformatf("abort %s: CP_STATUS.IDLE after CP_CTRL.RESET", name));
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, okj);
      check(okj, $sformatf("abort %s: RPTR == WPTR after the reset", name));
      // the work distributor drained: SP idle, nothing resident, no done / fault report
      okj = 0;
      for (int t = 0; t < 4000 && !okj; t++) begin
        dbg_rd(12'h804, st_x); okj = st_x[30] && st_x[15:0] == 0 && !st_x[22];
        if (!okj) repeat (20) @(posedge clk_l2);
      end
      check(okj, $sformatf("abort %s: SP idle after the abort (%08h)", name, st_x));
      check(wd_dones == dones0, $sformatf("abort %s: no wd_done for the aborted dispatch", name));
      dbg_rd(12'h824, cmx); repeat (2000) @(posedge clk_l2); dbg_rd(12'h824, cmy);
      check(cmx == cmy, $sformatf("abort %s: commits stopped (%0d -> %0d)", name, cmx, cmy));
      cpr(13'h011c, v); check(v == 0, $sformatf("abort %s: no CP error (%0d)", name, v));
      cpr(13'h0118, v); check(!v[1], $sformatf("abort %s: CP not halted (%08h)", name, v));
      repeat (400000) @(posedge clk_l2);                             // > a few firmware ticks (1 ms = 99000 clk_ts)
      cpr(13'h012c, hb1); check(hb1 != hb0, $sformatf("abort %s: heartbeat moves (%0d -> %0d)", name, hb0, hb1));
      lr(20'h310, v); check(!v[0], $sformatf("abort %s: no MMU fault (%08h)", name, v));
      $display("  abort %s: dispatch aborted after %0d commits, heartbeat %0d -> %0d", name, cm1 - cm0, hb0, hb1);
      // the abort waited for the fabric slots (no watchdog expiry), every slot is free again
      dbg_rd(12'h960, v); dbg_rd(12'h964, d);
      $display("  abort %s: FAB_SLOTS %08h FAB_ORPHANS %0d, ABORTING waited %0d clk_ts for the fabric (longest %0d)", name,
               v, d, ab_wait_cyc, ab_wait_max);
      check(v[7:0] == 0, $sformatf("abort %s: no fabric slot allocated after the abort (%08h)", name, v));
      check(ab_tmo == 0 && !dut.u_cp.u_port.ab_tmo_q, $sformatf("abort %s: no ABORT_TIMEOUT (WDIST_STATUS[4])", name));
      return;
    end
    if (conc) fork
      begin
        logic [63:0] rq [];
        logic [1:0] rr;
        while (!rd_stop) begin
          b2_read(WIN_LO + 28'(ib_off), 8, rq, rr);
          check(rr == 2'b00, "concurrent BAR2 read resp");
          for (int k = 0; k < 8; k++) check(rq[k] == {ib[2 * k + 1], ib[2 * k]}, "concurrent BAR2 read data");
          n_conc++;
        end
        rd_done = 1;
      end
    join_none
    if (expect_trap) wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd4, 40000, ok);
    else             wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 40000, ok);
    rd_stop = 1;
    while (!rd_done) @(negedge clk_l2);
    if (conc) begin
      $display("  concurrent BAR2 reads during the dispatch: %0d bursts", n_conc);
      check(n_conc >= 2, "concurrent BAR2 reads happened");
    end
    check(ok, $sformatf("bundle %s: %s", name, expect_trap ? "CP_ERROR = SHADER_FAULT" : "RPTR reaches WPTR"));
    if (!ok) begin
      logic [31:0] a1, a2, a3, a4;
      cpr(13'h011c, a1); cpr(13'h010c, a2); dbg_rd(12'h804, a3); dbg_rd(12'h840, a4);
      $display("  bundle %s stuck: CP_ERROR %0d RPTR %0d (wp %0d) SP_STATUS %08h FAB_REQS %0d", name, a1, a2, ring_wp, a3, a4);
    end

    // ---- WD request = the bundle's WDIST words
    check(wd_starts == starts0 + 1, $sformatf("bundle %s: one wd_start (%0d)", name, wd_starts - starts0));
    check(wd_dones == dones0 + 1, $sformatf("bundle %s: one wd_done (%0d)", name, wd_dones - dones0));
    check(wd_cap.vmid == b_wd[0][3:0] && wd_cap.num_vregs == {8'h0, b_wd[0][31:8]} && wd_cap.shared_bytes == b_wd[1] &&
          wd_cap.code_va == {b_wd[3], b_wd[2]} && wd_cap.code_base == {b_wd[5], b_wd[4]} &&
          wd_cap.scratch_va == {b_wd[7], b_wd[6]} && wd_cap.scratch_bytes == b_wd[8],
          $sformatf("bundle %s: WDIST request header = bundle", name));
    for (int i = 0; i < 3; i++)
      check(wd_cap.grid[i] == b_wd[9 + i] && wd_cap.wg[i] == b_wd[12 + i], $sformatf("bundle %s: grid / wg %0d", name, i));
    for (int i = 0; i < 26; i++)
      check(wd_cap.uregs[i] == b_wd[15 + i], $sformatf("bundle %s: u%0d", name, i));

    // --
    if (variant != V_VRAM_CODE) begin
      check(wd_cap_fault == (status != 0), $sformatf("bundle %s: wd_fault %0d", name, wd_cap_fault));
      check(wd_cap_instr == instr, $sformatf("bundle %s: retired %0d, model %0d", name, wd_cap_instr, instr));
      dbg_rd(12'h81C, d); check(d == instr[31:0], $sformatf("bundle %s: debug WD_INSTR_LO %0d", name, d));
      dbg_rd(12'h824, cm1);
      check(cm1 - cm0 == b_meta[3][31:0] || status != 0,
            $sformatf("bundle %s: debug COMMIT_TOTAL +%0d, model commits %0d", name, cm1 - cm0, b_meta[3]));
      // memory, every page of the bundle (relocated)
      mism = 0;
      if (b_meta[13] != 0)
        for (int j = 0; j < n_pages; j++)
          for (int k = 0; k < 512; k++) begin
            q = phys_q(reloc[b_pages[j]] + 8 * k);
            if (q != {b_post[j * 1024 + 2 * k + 1], b_post[j * 1024 + 2 * k]}) begin
              mism++;
              if (mism <= 8) $display("  MISMATCH %s pa page %h qword %0d: rtl %h model %h", name, b_pages[j], k, q,
                                      {b_post[j * 1024 + 2 * k + 1], b_post[j * 1024 + 2 * k]});
            end
          end
      check(mism == 0, $sformatf("bundle %s: memory = model (%0d pages, %0d mismatching qwords)", name, n_pages, mism));
    end
    dbg_rd(12'h828, d); check(d == ds0 + 1, "debug DISPATCHES +1");
    dbg_rd(12'h82C, d); check(d[15:0] == dn0[15:0] + 1 && d[31:16] == dn0[31:16] + 16'(expect_trap),
                              $sformatf("debug DISPATCH_DONE %08h -> %08h", dn0, d));
    dbg_rd(12'h804, d); check(d[30] && d[15:0] == 0 && !d[22], $sformatf("bundle %s: SP idle after the run (%08h)", name, d));

    if (expect_trap) begin
      logic [31:0] tc, tpc, tinf, twg;
      dbg_rd(12'h80C, tc); dbg_rd(12'h810, tpc); dbg_rd(12'h814, tinf); dbg_rd(12'h818, twg);
      if (variant == V_VRAM_CODE) begin
        check(tc[0] && tc[11:8] == 4'd5, $sformatf("vram code: trap record valid, MEMFAULT (%08h)", tc));
        dbg_rd(12'h844, d); check(d != 0, "vram code: FAB_ERRS counted");
        lr(20'h8C8, d); check(d != 0, "vram code: FAB_BAD counted (l2 page)");
        lr(20'h8CC, d); check(d == VRAM_PA[31:0], $sformatf("vram code: FAB_BAD_PA %08h", d));
        dbg_rd(12'h84C, d); check(d[21:20] == 2'd1 && d[18:16] == 3'd0, $sformatf("vram code: FAB_LAST_HI %08h", d));
      end else begin
        check(tc[0] && tc[11:8] == b_meta[9][3:0] && tc[20:16] == b_meta[7][4:0] && tpc == b_meta[8][31:0] &&
              tinf == b_meta[10][31:0] && twg == b_meta[6][31:0],
              $sformatf("bundle %s: trap record ctrl %08h pc %h info %h wg %0d / model cause %0d warp %0d pc %h info %h wg %0d",
                        name, tc, tpc, tinf, twg, b_meta[9], b_meta[7], b_meta[8], b_meta[10], b_meta[6]));
      end
      wait_reg(13'h0118, 32'h2, 32'h2, 200, okj);                      // the firmware halts after recording the error
      check(okj, $sformatf("bundle %s: STATUS.HALTED after SHADER_FAULT", name));
      cpr(13'h0040, v); check(v[1], "IRQ_STATUS.CP_ERROR after SHADER_FAULT");
      lr(20'h310, v);
      if (variant == V_VRAM_CODE) begin
        // the fetch error (fabric error on the VRAM page, no VRAM behind it) is latched like an LSU one:
        // BAD_PHYS_ADDR, access Exec, LEVEL 0 (SP path), the bundle's VMID, VA = the faulting fetch (= the trap info)
        logic [31:0] fva_lo;
        check(v[0] && v[7:4] == 4'd5 && v[9:8] == 2'd2 && v[13:12] == 2'd0 && v[19:16] == 4'(vmid),
              $sformatf("vram code: MMU_FAULT_STATUS %08h (BAD_PHYS_ADDR, Exec, level 0)", v));
        lr(20'h314, fva_lo); check(fva_lo == tinf, $sformatf("vram code: MMU_FAULT_VA_LO %08h = trap info %08h", fva_lo, tinf));
        dbg_rd(12'h854, d); check(d != 0, "vram code: debug MMU_FAULTS counted");
        cpr(13'h0040, d); check(d[2], "vram code: IRQ_STATUS.MMU_FAULT");
      end else if (b_meta[9][3:0] == 4'd5) begin                         // memory fault in the LSU: MMU record
        check(v[0] && v[7:4] == 4'd1 && v[19:16] == 4'(vmid), $sformatf("bundle %s: MMU_FAULT_STATUS %08h", name, v));
        dbg_rd(12'h854, d); check(d != 0, "debug MMU_FAULTS counted");
        dbg_rd(12'h85C, d); check(d[18:16] == 3'd1, $sformatf("debug MMU_LAST_VA_HI %08h", d));
      end else begin
        check(!v[0], $sformatf("bundle %s: no MMU fault (%08h)", name, v));
      end
      // recovery: clear, CP RESET, a fence job runs
      lw(20'h310, 32'h1);
      cpw(13'h0040, 32'hFFFF_FFFF);
      cpw(13'h0114, 32'h3);
      wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd0, 4000, okj);
      check(okj, $sformatf("bundle %s: RESET clears CP_ERROR", name));
      fence_job(64'h5150_0000 + 64'(wd_dones));
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, okj);
      check(okj && host_q(FENCE_HOST) == 64'h5150_0000 + 64'(wd_dones), $sformatf("bundle %s: CP works after RESET", name));
      cpw(13'h0040, 32'hFFFF_FFFF);
    end else begin
      cpr(13'h011c, v); check(v == 0, $sformatf("bundle %s: no CP error", name));
      lr(20'h310, v); check(!v[0], $sformatf("bundle %s: no MMU fault (%08h)", name, v));
    end
    // the context stays enabled; the next bundle re-roots it (CTRL write invalidates)
  endtask

  task automatic sp_tests();
    logic [31:0] d, g0, g1, nb, nf;
    $display("TEST sp");
    cpw(13'h0040, 32'hFFFF_FFFF);
    cpw(13'h0044, 32'h1);
    run_bundle("k_vadd", V_PLAIN, 0);
    run_bundle("k_ids", V_PLAIN, 0);
    run_bundle("k_intops", V_PLAIN, 0);
    run_bundle("k_loop", V_PLAIN, 0);
    run_bundle("k_atom", V_PLAIN, 0);
    begin   // cache.inv.l1 / cache.inv.l2 / cache.wb.l2 reach the LSU; inv.l2 / wb.l2 are fence requests
      logic [31:0] f0, f1;
      dbg_rd(12'h968, f0);
      run_bundle("k_cacheop", V_PLAIN, 0);
      dbg_rd(12'h968, f1);
      $display("  debug: k_cacheop: FAB_FENCES +%0d", f1 - f0);
      check(f1 != f0, "k_cacheop: cache.inv.l2 / cache.wb.l2 counted as fabric fences (FAB_FENCES)");
    end
    begin   // dword-interleaved scratch (ERRATA v0.1.6); 8 warps x (28 scratch + 7 OUT) instructions; per warp the
            // scratch side is 18 store + 25 load lines (one line per aligned b32 of 32 lanes; the v0.1.2 linear layout
            // would need ~12 per b32 at 48 B per lane), the OUT stores 7 x 32 lines
      logic [31:0] i0, g0_, i1, g1_;
      dbg_rd(12'h928, i0); dbg_rd(12'h92C, g0_);
      run_bundle("k_scratch", V_PLAIN, 0);
      dbg_rd(12'h928, i1); dbg_rd(12'h92C, g1_);
      $display("  debug: k_scratch: LSU instr %0d groups %0d", i1 - i0, g1_ - g0_);
      check(i1 - i0 == 32'd280 && g1_ - g0_ == 32'd2136, "k_scratch: LSU_INSTR_MEM 280, LSU_GROUPS 2136 (8 x (43 + 224))");
    end
    run_bundle("k_f16cvt", V_PLAIN, 0);                 // VALU binary16 conversions (EN_VALU_F16CVT, CAPS[29])
    run_bundle("k_xlane", V_PLAIN, 0);                  // XLANE in the RM (CAPS[25]): shfl / quad / vote / scan / reduce / derivatives
    run_bundle("k_vadd_big", V_PLAIN, 1);
    run_bundle("k_trap_explicit", V_PLAIN, 0);
    run_bundle("k_trap_lsu", V_PLAIN, 0);
    run_bundle("k_vadd", V_HOST, 0);
    run_bundle("k_scratch", V_HOST, 0);                 // sub-dword scratch stores to host memory (XDMA model)
    run_bundle("k_vadd", V_VRAM_CODE, 0);
    run_bundle("k_vadd", V_PLAIN, 0);                   // after the faults: a clean run again
    // CP_CTRL.RESET in the middle of a long dispatch (twice, at different depths), a normal dispatch after each
    run_bundle("k_spin", V_ABORT, 0);
    run_bundle("k_vadd", V_PLAIN, 0);
    run_bundle("k_spin", V_ABORT2, 0);
    begin   // LSU / fabric-latency counters across one kernel (uc data: one LSU line request = one fabric request)
      logic [31:0] i0, g0_, r0, w0, rn0, wn0, i1, g1_, r1, w1, rn1, wn1;
      dbg_rd(12'h928, i0); dbg_rd(12'h92C, g0_); dbg_rd(12'h920, r0); dbg_rd(12'h924, w0);
      dbg_rd(12'h908, rn0); dbg_rd(12'h918, wn0);
      run_bundle("k_vadd", V_PLAIN, 0);
      dbg_rd(12'h928, i1); dbg_rd(12'h92C, g1_); dbg_rd(12'h920, r1); dbg_rd(12'h924, w1);
      dbg_rd(12'h908, rn1); dbg_rd(12'h918, wn1);
      $display("  debug: k_vadd: LSU instr %0d groups %0d l1 rd %0d wr %0d, fabric rd %0d wr %0d", i1 - i0, g1_ - g0_,
               r1 - r0, w1 - w0, rn1 - rn0, wn1 - wn0);
      check(i1 != i0 && g1_ != g0_, "k_vadd: LSU_INSTR_MEM / LSU_GROUPS counted");
      check(g1_ - g0_ <= (r1 - r0) + (w1 - w0), "k_vadd: LSU_GROUPS <= LSU_L1_RD + LSU_L1_WR");
      check(wn1 - wn0 == w1 - w0 && w1 != w0, "k_vadd: one fabric write (FAB_WR_LAT_N) per LSU line write");
      check(rn1 - rn0 >= r1 - r0 && r1 != r0, "k_vadd: fabric reads (FAB_RD_LAT_N, + I$ fills) >= LSU line reads");
    end
    begin   // SP performance counters across one kernel (loop with bnz / bexecz beside VALU work)
      logic [31:0] p0 [8], p1 [8], d [8];
      for (int i = 0; i < 8; i++) dbg_rd(12'h940 + 12'(4 * i), p0[i]);
      run_bundle("k_loop", V_PLAIN, 0);
      for (int i = 0; i < 8; i++) begin dbg_rd(12'h940 + 12'(4 * i), p1[i]); d[i] = p1[i] - p0[i]; end
      $display("  debug: k_loop perf: active %0d valu_back_busy %0d br_wait %0d front_starved %0d valu %0d branches %0d own_wait %0d salu_busy %0d",
               d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
      check(d[0] != 0 && d[1] != 0 && d[4] != 0 && d[5] != 0 && d[7] != 0,
            "k_loop: SP_ACTIVE / VALU_BACK_BUSY / VALU_ISSUED / BR_ISSUED / SALU_BUSY counted");
      check(d[1] + d[3] <= d[0], "k_loop: VALU_BACK_BUSY + FRONT_STARVED <= SP_ACTIVE");
      check(d[2] <= d[0] && d[6] <= d[0] && d[7] <= d[0], "k_loop: stall counts <= SP_ACTIVE");
      check(d[5] <= d[7], "k_loop: BR_ISSUED <= SALU_BUSY");
      check(d[5] == 44, "k_loop: BR_ISSUED = 4 warps x (10 bnz + 1 bexecz); jal / jr are not pure branches");
    end
    // debug page summary
    dbg_rd(12'h830, d); check(d != 0, "IC_HIT counted");
    dbg_rd(12'h834, d); check(d != 0, "IC_MISS counted");
    dbg_rd(12'h840, d); check(d != 0, "FAB_REQS counted");
    dbg_rd(12'h850, d); check(d != 0, "MMU_REQS counted");
    begin   // SP translation requests per fabric line request (2.0 before the LSU translation memo)
      logic [31:0] nf, nm;
      dbg_rd(12'h840, nf); dbg_rd(12'h850, nm);
      $display("  debug: MMU/FAB requests: MMU_REQS %0d FAB_REQS %0d ratio %0d.%02d", nm, nf, nm / nf, (nm * 100 / nf) % 100);
    end
    lr(20'h8D4, g0); lr(20'h8D8, g1);
    $display("  debug: MMU grants CP %0d SP %0d", g0, g1);
    check(g0 != 0 && g1 != 0, "MMU arbiter served both requesters");
    lr(20'h8DC, g0); lr(20'h8E0, g1);
    $display("  debug: C3 requester-1 grants CP %0d SP %0d", g0, g1);
    check(g0 != 0 && g1 != 0, "C3 arbiter served both requesters");
    lr(20'h8E4, g0); lr(20'h8E8, g1);
    $display("  debug: host grants CP %0d SP %0d", g0, g1);
    check(g0 != 0, "host arbiter served the CP");
    // SP line reads / writes are AXI bursts (not oparb operations); only atomics and the byte-partial / holey
    // host line writes still use the 64-bit operation path (the V_HOST bundle's host data page is bursted)
    lr(20'h8F4, nb); lr(20'h8F8, nf);
    $display("  debug: SP fabric bursts %0d, line-write fallbacks %0d", nb, nf);
    if (SP_BURST) check(nb != 0, "SP line requests ran as AXI bursts");
    else          check(nb == 0 && g1 != 0, "SP_BURST=0: operation path only");
    lr(20'h8C4, d); $display("  debug: SP 64-bit fabric operations %0d", d);
    for (int s = 0; s < 4; s++) begin dbg_rd(12'h880 + 12'(4 * s), d); $display("  debug: SLOT_PC[%0d] %08h", s, d); end
    dbg_rd(12'h880, d); check(d != 0, "SLOT_PC[0] recorded");
    begin   // fabric round trip in clk_ts cycles (10 ns), accept in ng_rsp_fab_ts -> response
      logic [31:0] sl, sh, n, mx, fr;
      longint unsigned sum, nrw;
      dbg_rd(12'h840, fr);
      nrw = 0;
      for (int k = 0; k < 2; k++) begin
        dbg_rd(12'h900 + 12'(16 * k), sl); dbg_rd(12'h904 + 12'(16 * k), sh);
        dbg_rd(12'h908 + 12'(16 * k), n); dbg_rd(12'h90C + 12'(16 * k), mx);
        sum = {16'h0, sh[15:0], sl};
        $display("  debug: fabric %s latency: n %0d sum %0d avg %0d.%01d max %0d clk_ts", k ? "WR" : "RD", n, sum,
                 n ? sum / n : 0, n ? (sum * 10 / n) % 10 : 0, mx);
        check(n != 0 && sh[31:16] == 0, $sformatf("FAB_%s_LAT counted", k ? "WR" : "RD"));
        // lower bound: two 2-flop toggle synchronisers and the slave's minimum latency; upper bound: the slave's
        // 30-cycle clk_l2 latency, 16 beats at 75 % ready / 15 % gaps and a CP / DMA burst ahead in the mux
        // (SP_BURST = 0: a line is 16 serial 64-bit operations, about 12 x the burst latency)
        check(n != 0 && sum >= 64'(n) * 6 && sum <= 64'(n) * (SP_BURST ? 150 : 600),
              $sformatf("FAB_%s_LAT average in the slave-latency range (%0d / %0d)", k ? "WR" : "RD", sum, n));
        check(64'(mx) * 64'(n) >= sum && 64'(mx) <= 64'd5000, $sformatf("FAB_%s_LAT_MAX >= average (%0d)", k ? "WR" : "RD", mx));
        nrw += n;
      end
      check(nrw <= fr, "FAB_RD_LAT_N + FAB_WR_LAT_N <= FAB_REQS (the rest are atomics / cache maintenance)");
      dbg_rd(12'h928, n); dbg_rd(12'h92C, mx);
      $display("  debug: LSU global memory instructions %0d, line groups %0d (since the last SP reset)", n, mx);
      check(n != 0 && mx != 0, "LSU_INSTR_MEM / LSU_GROUPS after the runs");
    end
    begin   // fabric slot block 0x960..0x97F
      logic [31:0] sl, orph, fen, fst, sst, cst;
      dbg_rd(12'h960, sl); dbg_rd(12'h964, orph); dbg_rd(12'h968, fen); dbg_rd(12'h96C, fst);
      dbg_rd(12'h970, sst); dbg_rd(12'h974, cst);
      $display("  debug: fabric slots: NOUT %0d in use %0d max %0d, orphans %0d, fences %0d, stall clk_ts: fence %0d slot %0d conflict %0d",
               sl[23:16], sl[7:0], sl[15:8], orph, fen, fst, sst, cst);
      check(sl[23:16] == 8'(SP_NOUT) && sl[7:0] == 0 && sl[15:8] >= 1 && sl[15:8] <= 8'(SP_NOUT),
            $sformatf("FAB_SLOTS after the runs (%08h)", sl));
      check(fen != 0, "FAB_FENCES counted (k_atom atomics)");
      dbg_rd(12'h978, sl); dbg_rd(12'h97C, orph);
      check(sl == 0 && orph == 0, "0x978 / 0x97C reserved (read 0)");
    end
    begin   // LSU block 0x980..0x9A8 (since the last SP reset)
      logic [31:0] c [11];
      for (int i = 0; i < 11; i++) dbg_rd(12'h980 + 12'(4 * i), c[i]);
      $display("  debug: LSU K=%0d MEM_OUT=%0d IOF=%0d: fe_busy %0d ralloc %0d iss_stall %0d sync %0d ring_occ %0d max %0d be_wait %0d wb_stall %0d late_err %0d",
               c[10][7:0], c[10][15:8], c[10][23:16], c[0], c[1], c[2], c[3], {c[5][15:0], c[4]}, c[6], c[7], c[8], c[9]);
      check(c[10][7:0] == 8'(SP_LSU_K), "LSU_CFG.K");
      if (SP_LSU_K > 1) begin
        check(c[0] != 0 && c[6] >= 1 && c[6] <= SP_LSU_K, "LSU_FE_BUSY / LSU_RING_MAX after the runs");
        check(c[9] == 0, "LSU_LATE_ERR: no fabric-error trap in these bundles");
      end else begin
        check(c[0] != 0, "LSU_FE_BUSY (LSU busy cycles, LSU_K = 1) after the runs");
      end
    end
    begin   // owner-change scrub and CACHE invalidates, end to end through the real firmware / CDC
      logic [31:0] s0, c0, e0, l0, i0, s1, c1, e1, l1, i1;
      logic [31:0] t [];
      int st0;
      bit ok2;
      localparam int NBEAT_RM = 2;                              // rm_sp NL = 16: 32 / 16 beats per warp register
      // first re-enable the context (the last bundle may have been an abort / fault variant), then run the same IB
      lw(20'h208 + 20'(16 * last_vmid), 32'h1);
      repeat (40) @(posedge clk_ts);                            // clk_l2 toggle -> clk_ts pulse -> scr_pend
      st0 = wd_starts;
      run_ib(last_vmid, IB_VA, last_nw, ok2);
      check(ok2 && wd_starts == st0 + 1, "S5b: dispatch after the context enable ran");
      check(wd_cap.scrub === 1'b1, "S5b: the dispatch after an MMU context enable carries scrub");
      dbg_rd(12'h9AC, s0); dbg_rd(12'h9B0, c0); dbg_rd(12'h9B4, e0); dbg_rd(12'h9B8, l0); dbg_rd(12'h9BC, i0);
      $display("  debug: S5b counters: scrubs %0d, scrub clk %0d, L1D ext inv %0d, CP INV_L1 %0d INV_I %0d", s0, c0, e0, l0, i0);
      check(s0 != 0 && c0 >= 32'(1024 * NBEAT_RM), "S5b: SCRUB_N / SCRUB_CYC counted");
      // same owner, same IB again: no scrub (the implicit C-INV of the INDIRECT still invalidates L1 / K / I)
      st0 = wd_starts;
      run_ib(last_vmid, IB_VA, last_nw, ok2);
      check(ok2 && wd_starts == st0 + 1, "S5b: second dispatch of the same owner ran");
      check(wd_cap.scrub === 1'b0, "S5b: no scrub for the same owner back to back");
      dbg_rd(12'h9AC, s1); dbg_rd(12'h9B0, c1); dbg_rd(12'h9B4, e1); dbg_rd(12'h9B8, l1); dbg_rd(12'h9BC, i1);
      check(s1 == s0 && c1 == c0, "S5b: SCRUB_N / SCRUB_CYC unchanged without an owner change");
      check(e1 == e0 + 1 && l1 == l0 + 1 && i1 == i0 + 1, $sformatf("S5b: INDIRECT implicit INV_L1 / INV_K / INV_I reach the SP once (%0d %0d %0d)", e1 - e0, l1 - l0, i1 - i0));
      // MMU context re-enable of the same VMID (what vm_destroy + vm_create of a reused VMID do): the next dispatch scrubs
      s0 = s1; c0 = c1; e0 = e1; l0 = l1; i0 = i1;
      lw(20'h208 + 20'(16 * last_vmid), 32'h1);
      repeat (40) @(posedge clk_ts);                            // clk_l2 toggle -> clk_ts pulse -> scr_pend
      st0 = wd_starts;
      run_ib(last_vmid, IB_VA, last_nw, ok2);
      check(ok2 && wd_starts == st0 + 1, "S5b: dispatch after the context re-enable ran");
      check(wd_cap.scrub === 1'b1, "S5b: MMU CTX_CTRL write of the VMID between two dispatches -> scrub");
      dbg_rd(12'h9AC, s1); dbg_rd(12'h9B0, c1); dbg_rd(12'h9B4, e1); dbg_rd(12'h9B8, l1);
      $display("  debug: S5b re-enable: scrub +%0d, scrub step %0d clk (register file %0d beats), L1D ext inv +%0d, CP INV_L1 +%0d",
               s1 - s0, c1 - c0, 1024 * NBEAT_RM, e1 - e0, l1 - l0);
      check(s1 == s0 + 1, "S5b: SCRUB_N +1");
      check(c1 - c0 >= 32'(1024 * NBEAT_RM) && c1 - c0 <= 32'(1024 * NBEAT_RM + 1024), "S5b: SCRUB_CYC = register file write + shared rows + a few clk");
      check(e1 == e0 + 2 && l1 == l0 + 1, "S5b: L1D invalidates: scrub + implicit INV_L1");
      cpr(13'h011c, v); check(v == 0, "S5b: no CP error");
      // CACHE(INV_L1) packet in an IB: the L1D invalidate request reaches the SP (debug counters)
      s0 = s1; e0 = e1; l0 = l1; dbg_rd(12'h9BC, i0);
      t = new[2]; t[0] = 32'h0001_000A; t[1] = 32'h0000_0002;    // CACHE, FLAGS = INV_L1
      put_c3(last_ib_off, t, 2);
      run_ib(last_vmid, IB_VA, 2, ok2);
      check(ok2, "S5b: CACHE(INV_L1) IB completed");
      dbg_rd(12'h9AC, s1); dbg_rd(12'h9B4, e1); dbg_rd(12'h9B8, l1); dbg_rd(12'h9BC, i1);
      $display("  debug: S5b CACHE(INV_L1): L1D ext inv +%0d, CP INV_L1 +%0d, CP INV_I +%0d", e1 - e0, l1 - l0, i1 - i0);
      check(s1 == s0, "S5b: a CACHE packet does not scrub");
      check(l1 == l0 + 2 && e1 == e0 + 2, "S5b: CACHE(INV_L1) = implicit C-INV + the packet, 2 L1D invalidates");
      check(i1 == i0 + 1, "S5b: CACHE(INV_L1) does not request an I$ invalidate (only the implicit one)");
      t[1] = 32'h0000_0008;                                       // INV_I
      put_c3(last_ib_off, t, 2);
      e0 = e1; l0 = l1; i0 = i1;
      run_ib(last_vmid, IB_VA, 2, ok2);
      dbg_rd(12'h9B4, e1); dbg_rd(12'h9B8, l1); dbg_rd(12'h9BC, i1);
      check(ok2 && i1 == i0 + 2 && e1 == e0 + 1 && l1 == l0 + 1, $sformatf("S5b: CACHE(INV_I): I$ +2, L1D +1 (%0d %0d %0d)", i1 - i0, e1 - e0, l1 - l0));
      cpr(13'h011c, v); check(v == 0, "S5b: no CP error after the CACHE packets");
    end
  endtask

  initial begin
    bit ok;
    logic [1:0] r;
    logic [63:0] d [];
    logic [63:0] rd [];
    logic [31:0] fwv;
    int fd, n;

    // idle values
    s_axil_ctrl_awvalid = 0; s_axil_ctrl_wvalid = 0; s_axil_ctrl_bready = 0; s_axil_ctrl_arvalid = 0;
    s_axil_ctrl_rready = 0; s_axil_ctrl_awprot = 0; s_axil_ctrl_arprot = 0;
    s_axi_bar2_awvalid = 0; s_axi_bar2_wvalid = 0; s_axi_bar2_bready = 0; s_axi_bar2_arvalid = 0; s_axi_bar2_rready = 0;
    s_axi_bar2_awcache = 0; s_axi_bar2_awprot = 0; s_axi_bar2_awqos = 0; s_axi_bar2_arcache = 0;
    s_axi_bar2_arprot = 0; s_axi_bar2_arqos = 0; s_axi_bar2_arid = 0; s_axi_bar2_awid = 0;
    s_axi_bar2_arburst = 2'b01; s_axi_bar2_awburst = 2'b01;
    shell_mem_ready = 4'hF; shell_quiesce_req = 0;
    dbg_hub_drck = 0; dbg_hub_tck = 0; dbg_hub_tms = 0; dbg_hub_tdi = 0; dbg_hub_sel = 0; dbg_hub_shift = 0;
    dbg_hub_capture = 0; dbg_hub_update = 0; dbg_hub_reset = 0; dbg_hub_runtest = 0; dbg_hub_bscanid_en = 0;
    dbg_obs_tck = 0; dbg_obs_tdi = 0; dbg_obs_sel = 0; dbg_obs_capture = 0; dbg_obs_shift = 0; dbg_obs_update = 0;
    rst_core_n = 0; rst_l2_n = 0; rst_ts_n = 0;
    repeat (20) @(posedge clk_l2);
    rst_core_n = 1; rst_l2_n = 1; rst_ts_n = 1;
    repeat (40) @(posedge clk_l2);

    // --------------------------------------------------------------
    $display("TEST ident");
    lr(20'h0, v);   check(v == 32'h5550_474E, "ID");
    lr(20'h4, v);   check(v == 32'h0000_0007, "REV");
    lr(20'h8, v);   check(v == 32'h3B10_2011, "CAPS (1 core, 1 SP, warp 32, 16 VMIDs, SFU + XLANE, units valid, scratch interleaved, f16cvt)");
    lw(20'h10, 32'hCAFE_F00D); lr(20'h10, v); check(v == 32'hCAFE_F00D, "SCRATCH");
    lr(20'h400, v); check(v == 32'h00_1b_14_1c, "BAR2_INFO");
    check(rm_info[31:0] == 32'h4d52474e && rm_info[63:48] == 16'h8003, "rm_info");
    check(rm_ready === 1'b1, "rm_ready");
    lr(20'h2000, v); check(v == 0, "undefined offset reads 0");
    cpr(13'h0138, v); check(v == 0, "FW_STATUS reset");
    cpr(13'h0114, v); check(v == 32'h8, "CTRL reset value CORE_RESET");
    cpr(13'h0058, v); check(v == 32'd100_000_000, "TS_FREQ");
    // debug page at reset: SP held by CORE_RESET (CP_CTRL reset value), idle, bridges idle
    lr(20'h800, v); check(v == 32'h5350_0005, "DBG_ID (page version 5)");
    lr(20'h804, v); check(v[30] && v[31] && v[15:0] == 0, $sformatf("SP_STATUS at reset %08h", v));
    lr(20'h808, v); check(v == 32'h20, $sformatf("BRIDGE_TS at reset %08h (CORE_RESET only)", v));
    lw(20'h804, 32'hFFFF_FFFF); lr(20'h804, v); check(v[30] && v[31], "debug page ignores writes");
    lr(20'h8C0, v); check(v[17:16] == 2'b11 && v[13:0] == 0, $sformatf("BR_L2_STATUS at reset %08h", v));
    lr(20'h8C4, v); check(v == 0, "FAB_OPS at reset");
    lr(20'h8FC, v); check(v == 0, "debug page reserved l2 word");
    lr(20'h8BC, v); check(v == 0, "SLOT_PC[15] at reset");
    for (int i = 0; i < 16; i++) begin                                  // block 0x900..0x93F
      lr(20'h900 + 20'(4 * i), v); check(v == 0, $sformatf(" counter 0x%03h at reset (%08h)", 'h900 + 4 * i, v));
    end
    for (int i = 0; i < 8; i++) begin                                   // block 0x940..0x95F (SP held in reset)
      lr(20'h940 + 20'(4 * i), v); check(v == 0, $sformatf(" counter 0x%03h at reset (%08h)", 'h940 + 4 * i, v));
    end
    lr(20'h960, v); check(v == (32'(SP_NOUT) << 16), $sformatf("FAB_SLOTS at reset %08h (NOUT, nothing allocated)", v));
    for (int i = 1; i < 8; i++) begin                                   // block 0x960..0x97F
      lr(20'h960 + 20'(4 * i), v); check(v == 0, $sformatf(" counter 0x%03h at reset (%08h)", 'h960 + 4 * i, v));
    end
    for (int i = 0; i < 10; i++) begin                                  // LSU block 0x980..0x9A4 (SP in reset)
      lr(20'h980 + 20'(4 * i), v); check(v == 0, $sformatf(" counter 0x%03h at reset (%08h)", 'h980 + 4 * i, v));
    end
    lr(20'h9A8, v);
    check(v[7:0] == 8'(SP_LSU_K) && v[15:8] == ((SP_LSU_K > 1) ? 8'((SP_MEM_OUT == 1) ? 1 : SP_NOUT) : 8'((SP_MEM_OUT == 1) ? 1 : SP_NOUT)),
          $sformatf("LSU_CFG at reset %08h", v));
    lr(20'h9BC, v); check(v == 0, "0x9BC ( block, unused) reads 0");

    // ---------------------------------------------------------------- C3 window
    $display("TEST window");
    lw(20'h404, 32'h1 | (24 << 8)); lw(20'h408, 0);
    lr(20'h404, v); check(v == (32'h1 | (24 << 8)), "C3_WIN_CTRL readback");
    lw(20'h404, 32'h1 | (21 << 8)); lw(20'h408, 32'h0010_0000);         // base not aligned to 2 MiB -> illegal
    lr(20'h404, v); check(v[31] && !v[0], "illegal window: ERR set, ENABLE cleared");
    lw(20'h408, 0); lw(20'h404, 32'h1 | (24 << 8));
    lr(20'h404, v); check(!v[31] && v[0] && v[12:8] == 24, "legal window clears ERR");
    d = new[1]; d[0] = 64'h1122_3344_5566_7788;
    b2_write(28'h000_0100, d, 8'hFF, 3'd3, r);                           // VRAM (below the window): SLVERR
    check(r == 2'b10, "VRAM write SLVERR");
    b2_read(28'h000_0100, 1, rd, r);   check(r == 2'b10 && rd[0] == 0, "VRAM read SLVERR");
    d = new[8];
    for (int i = 0; i < 8; i++) d[i] = {32'hA0A0_0000 + 2 * i + 1, 32'hB0B0_0000 + 2 * i};
    b2_write(WIN_LO + 28'h200, d, 8'hFF, 3'd3, r);
    check(r == 2'b00, "window burst write ok");
    for (int i = 0; i < 8; i++) check(c3_q(64'h200 + 8 * i) == d[i], $sformatf("C3 mem qword %0d", i));
    b2_read(WIN_LO + 28'h200, 8, rd, r);
    check(r == 2'b00, "window burst read ok");
    for (int i = 0; i < 8; i++) check(rd[i] == d[i], $sformatf("read back qword %0d", i));
    // narrow beats (size 2): a 4-byte store into the upper lane
    d = new[1]; d[0] = 64'h5555_6666_0000_0000;
    b2_write(WIN_LO + 28'h204, d, 8'hF0, 3'd2, r);
    check(c3_q(64'h200) == {32'h5555_6666, 32'hB0B0_0000}, "narrow write keeps the other lane");
    // sliding window
    lw(20'h408, 32'h0100_0000);
    lr(20'h404, v); check(!v[31] && v[0], "slide to 16 MiB");
    d = new[1]; d[0] = 64'hDEAD_BEEF_0BAD_F00D;
    b2_write(WIN_LO + 28'h40, d, 8'hFF, 3'd3, r);
    check(c3_q(64'h0100_0040) == d[0], "slid window write");
    lw(20'h408, 0);

    // ---------------------------------------------------------------- boot: bad images
    $display("TEST boot errors");
    begin
      logic [31:0] bad [];
      logic [31:0] st;
      bad = new[16];
      for (int i = 0; i < 16; i++) bad[i] = 32'h1111_0000 + i;
      put_c3(FW_OFF, bad, 16);
      boot_fw(FW_OFF, 64, ok, st, 200);
      check(!ok && st[15:8] == 8'd1, "BOOT_ERR 1 (magic)");
      cpr(13'h0040, v); check(v[3], "IRQ_STATUS.CP_FW after boot error");
      cpw(13'h0044, 32'h8); repeat (20) @(posedge clk_l2); check(rm_irq[0] === 1'b1, "rm_irq from CP_FW");
      cpw(13'h0040, 32'h8); cpw(13'h0044, 32'h0);
    end

    // ---------------------------------------------------------------- boot: real firmware
    $display("TEST boot firmware");
    fd = $fopen("fw_ngcf.size", "r");
    void'($fscanf(fd, "%d", fw_bytes));
    $fclose(fd);
    $readmemh("fw_ngcf.hex", fw_img);
    begin
      logic [31:0] w [];
      logic [31:0] st;
      w = new[fw_bytes / 4];
      for (int i = 0; i < fw_bytes / 4; i++) w[i] = fw_img[i];
      put_c3(FW_OFF, w, fw_bytes / 4);
      boot_fw(FW_OFF, fw_bytes, ok, st);
      check(ok, $sformatf("firmware READY (FW_STATUS %08h)", st));
      cpr(13'h0130, v); check(v == 32'h0000_0003, "CMD_VERSION");   // cmd v0.3 (P2-00b)
      cpr(13'h0118, v); check(v[4], "STATUS.FW_READY");
      cpr(13'h0040, v); check(v[3], "IRQ_CP_FW after READY");
      cpw(13'h0040, 32'hFFFF_FFFF);
    end

    // ---------------------------------------------------------------- heartbeat while idle (1 ms hardware tick)
    $display("TEST heartbeat");
    begin
      logic [31:0] h0, h1;
      cpr(13'h012c, h0);
      repeat (1500) @(posedge clk_ts);          // ~15 us per 1500 cycles ... wait about 3 ms of CP clock below
      for (int k = 0; k < 40; k++) repeat (10_000) @(posedge clk_ts);   // 40 x 100 us = 4 ms
      cpr(13'h012c, h1);
      check(h1 - h0 >= 3, $sformatf("CP_HEARTBEAT advances while idle: %0d -> %0d", h0, h1));
    end

    // ---------------------------------------------------------------- FENCE jobs
    $display("TEST fence jobs");
    ring_wp = 0;
    cpw(13'h0100, (SYS_BASE + RING_OFF) & 32'hFFFF_FFFF); cpw(13'h0104, (SYS_BASE + RING_OFF) >> 32);
    cpw(13'h0108, 32'd10);
    cpw(13'h0044, 32'h1);                                 // enable the fence interrupt
    cpw(13'h0114, 32'h1);                                 // ENABLE
    fence_job(64'h0000_0001_0000_0007);
    wait_reg(13'h010c, 32'hFFFF_FFFF, 32'd6, 4000, ok);
    check(ok, "RPTR reaches 6");
    check(host_q(FENCE_HOST) == 64'h0000_0001_0000_0007, "fence value written to host memory");
    cpr(13'h0040, v); check(v[0], "IRQ_STATUS.FENCE");
    repeat (20) @(posedge clk_l2); check(rm_irq[0] === 1'b1, "rm_irq asserted");
    cpw(13'h0040, 32'h1); repeat (20) @(posedge clk_l2); check(rm_irq[0] === 1'b0, "rm_irq drops after W1C");
    for (int j = 2; j <= 5; j++) begin
      fence_job(64'(j) * 64'h100);
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(6 * j), 4000, ok);
      check(ok && host_q(FENCE_HOST) == 64'(j) * 64'h100, $sformatf("fence job %0d", j));
    end
    cpw(13'h0040, 32'hFFFF_FFFF);

    // ---------------------------------------------------------------- COPY / FILL through the clk_l2 DMA
    $display("TEST copy");
    begin
      localparam longint unsigned HS = 64'h4_0003, HD = 64'h6_0005, HD2 = 64'h8_0006, HF = 64'h9_0004;
      localparam longint unsigned CD = 64'h40_0009;
      localparam int N1 = 20000, N2 = 5003, N3 = 4108;
      int s0, bad;
      for (longint unsigned q = HS >> 3; q <= (HS + N1) >> 3; q++)
        u_host.poke(q, {q[31:0] * 32'h9E37_79B1, ~q[31:0] * 32'h85EB_CA6B});
      s0 = ring_wp;
      copy_pkt(HOST_BASE + HS, HOST_BASE + HD, N1);     // host -> host, unaligned both ends
      copy_pkt(HOST_BASE + HS, SYS_BASE + CD, N2);      // host -> C3
      copy_pkt(SYS_BASE + CD, HOST_BASE + HD2, N2);     // C3 -> host (packets run in order)
      fill_pkt(HOST_BASE + HF, N3, 32'hA5A5_5A5A);
      ring_flush(s0, ring_wp);
      cpw(13'h1000, ring_wp);
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 40000, ok);
      check(ok, "COPY / FILL packets retire");
      cpr(13'h011c, v); check(v == 0, "no CP_ERROR after COPY / FILL");
      bad = 0;
      for (int i = 0; i < N1; i++) if (host_b(HD + i) !== host_b(HS + i)) bad++;
      check(bad == 0, $sformatf("host -> host copy: %0d bytes differ", bad));
      check(host_b(HD - 1) == 8'h0 && host_b(HD + N1) == 8'h0, "host -> host copy: no bytes outside the range");
      bad = 0;
      for (int i = 0; i < N2; i++) if (c3_b(CD + i) !== host_b(HS + i)) bad++;
      check(bad == 0, $sformatf("host -> C3 copy: %0d bytes differ", bad));
      bad = 0;
      for (int i = 0; i < N2; i++) if (host_b(HD2 + i) !== host_b(HS + i)) bad++;
      check(bad == 0, $sformatf("C3 -> host copy: %0d bytes differ", bad));
      bad = 0;
      for (int i = 0; i < N3; i++) if (host_b(HF + i) !== 8'(32'hA5A5_5A5A >> (8 * ((HF + i) % 4)))) bad++;
      check(bad == 0 && host_b(HF + N3) == 8'h0, $sformatf("FILL: %0d bytes differ", bad));
      $display("  copy: %0d + %0d + %0d bytes, fill %0d bytes", N1, N2, N2, N3);
    end

    // ---------------------------------------------------------------- CP error: INDIRECT with a user VMID
    $display("TEST cp error and recovery");
    begin
      int s = ring_wp;
      ring_put(32'h0004_0001);                            // INDIRECT, 4 dwords
      ring_put(32'h0000_1000); ring_put(32'h1); ring_put(32'd4); ring_put(32'h1);   // va, size 4 dw, VMID 1
      ring_flush(s, ring_wp);
      cpw(13'h1000, ring_wp);
      wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd3, 4000, ok);   // MEM_FAULT
      check(ok, "CP_ERROR = MEM_FAULT (VMID 1 has no enabled context -> NON_CANONICAL)");
      lr(20'h310, v);
      check(v[0] && v[7:4] == 4'd2 && v[9:8] == 2'd0 && v[19:16] == 4'd1, "MMU_FAULT_STATUS: NON_CANONICAL read, VMID 1");
      cpr(13'h0040, v); check(v[2], "IRQ_STATUS.MMU_FAULT");
      lw(20'h310, 32'h1);
      wait_reg(13'h0118, 32'h2, 32'h2, 200, ok);
      check(ok, "STATUS.HALTED");
      cpr(13'h0040, v); check(v[1], "IRQ_STATUS.CP_ERROR");
      cpw(13'h0040, 32'hFFFF_FFFF);
      cpw(13'h0114, 32'h3);                               // ENABLE | RESET
      wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd0, 4000, ok);
      check(ok, "RESET clears CP_ERROR");
      fence_job(64'hF00D);
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, ok);
      check(ok && host_q(FENCE_HOST) == 64'hF00D, "fence works after RESET");
      if (!ok) begin
        logic [31:0] a1, a2, a3, a4, a5, a6;
        cpr(13'h0118, a1); cpr(13'h011c, a2); cpr(13'h010c, a3); cpr(13'h0110, a4); cpr(13'h0138, a5); cpr(13'h012c, a6);
        $display("  after RESET: STATUS %08h ERROR %08h RPTR %0d WPTR %0d FW_STATUS %08h HB %0d ring_wp %0d host %h",
                 a1, a2, a3, a4, a5, a6, ring_wp, host_q(FENCE_HOST));
      end
    end

    // ---------------------------------------------------------------- MMU: user IB through a VMID-1 context
    $display("TEST mmu");
    begin
      longint unsigned root_pa, v1, v2, v3, v4, va;
      logic [31:0] w32, wlo, whi;
      int walks0, walks1, hits0, hits1;
      bit okj;

      root_pa = SYS_BASE + PT_ROOT;
      cpw(13'h0040, 32'hFFFF_FFFF);                           // drop leftover FENCE / error interrupts
      // page tables: root[0] -> L2, L2[4] -> L1, L1[0] -> L0, L0[1..7]:
      //   1 IB (SYS, R)   2 host page 1 (RW)   3 SYS data (RW)   4 SYS page (R only)   5 not present
      //   6 VRAM page (RW, unreachable in this RM)   7 sparse-null
      put_q(PT_ROOT, (SYS_BASE + PT_L2) | 64'h1);
      put_q(PT_L2 + 8 * 4, (SYS_BASE + PT_L1) | 64'h1);
      put_q(PT_L1, (SYS_BASE + PT_L0) | 64'h1);
      put_q(PT_L0 + 8 * 1, (SYS_BASE + IB_PG) | 64'h7);
      put_q(PT_L0 + 8 * 2, (HOST_BASE + HPG1) | 64'hF);
      put_q(PT_L0 + 8 * 3, (SYS_BASE + DATA_PG) | 64'hF);
      put_q(PT_L0 + 8 * 4, (SYS_BASE + RO_PG) | 64'h7);
      put_q(PT_L0 + 8 * 6, 64'h0000_0000_4000_0000 | 64'hF);
      put_q(PT_L0 + 8 * 7, 64'h100);

      // BAR0: context for VMID 1 (ROOT_LO/HI shadow, CTRL latches + enables), poll MMU_INVALIDATE like the driver
      lr(20'h200, v); check(v == 0, "VMID 0 slot reads 0");
      lw(20'h210, root_pa[31:0]); lw(20'h214, root_pa[47:32]);
      lr(20'h210, v); check(v == root_pa[31:0], "MMU_CTX_ROOT_LO readback");
      lr(20'h214, v); check(v == root_pa[47:32], "MMU_CTX_ROOT_HI readback");
      lr(20'h218, v); check(v == 0, "ctx not enabled before CTRL");
      lw(20'h218, 32'h1);
      okj = 0;
      for (int t = 0; t < 50 && !okj; t++) begin lr(20'h300, v); okj = (v == 0); end
      check(okj, "MMU_INVALIDATE pending drains");
      lr(20'h218, v); check(v == 1, "ctx ENABLE readback");
      lr(20'h310, v); check(v[1:0] == 2'b00, "MMU_FAULT_STATUS clear");

      // ---- run 1: IB = FENCE -> host VA, WRITE_DATA -> SYS VA
      v1 = 64'hFEED_0001_0000_0001;
      ib_n = 0;
      ib_fence(VA0 + 64'h2008, v1);
      ib_write2(VA0 + 64'h3010, 32'h1111_1111, 32'h2222_2222);
      ib_pad();
      ib_commit();
      walks0 = dut.u_mmu.stat_walks; hits0 = dut.u_mmu.stat_tlb_hits;
      run_ib(1, VA0 + 64'h1000, ib_n, okj);
      check(okj, "mmu run 1: RPTR reaches WPTR");
      check(host_q(HPG1 + 8) == v1, "fence through a mapped HOST page lands at the translated address");
      check(c3_q(DATA_PG + 64'h10) == {32'h2222_2222, 32'h1111_1111}, "WRITE_DATA through a mapped SYS page");
      cpr(13'h011c, v); check(v == 0, "no CP error");
      walks1 = dut.u_mmu.stat_walks;
      check(walks1 - walks0 >= 3, $sformatf("run 1 walked at least 3 pages (%0d)", walks1 - walks0));
      lr(20'h310, v); check(v[1:0] == 2'b00, "no MMU fault recorded");

      // ---- run 2: same pages again -> TLB hits only
      v2 = 64'hFEED_0002_0000_0002;
      ib_n = 0;
      ib_fence(VA0 + 64'h2008, v2);
      ib_write2(VA0 + 64'h3010, 32'h3333_3333, 32'h4444_4444);
      ib_pad();
      ib_commit();
      hits0 = dut.u_mmu.stat_tlb_hits;
      run_ib(1, VA0 + 64'h1000, ib_n, okj);
      check(okj, "mmu run 2: RPTR reaches WPTR");
      check(host_q(HPG1 + 8) == v2, "run 2 fence");
      check(c3_q(DATA_PG + 64'h10) == {32'h4444_4444, 32'h3333_3333}, "run 2 WRITE_DATA");
      hits1 = dut.u_mmu.stat_tlb_hits;
      check(dut.u_mmu.stat_walks == walks1, "run 2: no new walks (TLB hits)");
      check(hits1 - hits0 >= 3, $sformatf("run 2: TLB hits (%0d)", hits1 - hits0));

      // ---- remap the host page without an invalidate: the stale TLB entry still wins
      put_q(PT_L0 + 8 * 2, (HOST_BASE + HPG2) | 64'hF);
      v3 = 64'hFEED_0003_0000_0003;
      ib_n = 0; ib_fence(VA0 + 64'h2008, v3); ib_pad(); ib_commit();
      run_ib(1, VA0 + 64'h1000, ib_n, okj);
      check(okj, "mmu run 3: RPTR reaches WPTR");
      check(host_q(HPG1 + 8) == v3 && host_q(HPG2 + 8) != v3, "stale TLB entry used before the invalidate");
      // ---- MMU_INVALIDATE VMID 1, then the new mapping is walked
      walks1 = dut.u_mmu.stat_walks;
      lw(20'h300, 32'h2);
      okj = 0;
      for (int t = 0; t < 50 && !okj; t++) begin lr(20'h300, v); okj = (v == 0); end
      check(okj, "MMU_INVALIDATE completes");
      v4 = 64'hFEED_0004_0000_0004;
      ib_n = 0; ib_fence(VA0 + 64'h2008, v4); ib_pad(); ib_commit();
      run_ib(1, VA0 + 64'h1000, ib_n, okj);
      check(okj, "mmu run 4: RPTR reaches WPTR");
      check(host_q(HPG2 + 8) == v4 && host_q(HPG1 + 8) == v3, "after MMU_INVALIDATE the fence lands in the new page");
      check(dut.u_mmu.stat_walks > walks1, "walks after the invalidate");
      lr(20'h310, v); check(v[1:0] == 2'b00, "still no MMU fault");

      // ---- sparse-null: read 0 (WAIT_MEM EQ 0 completes), write dropped, no fault
      v1 = 64'hFEED_0005_0000_0005;
      ib_n = 0;
      ib_waitmem0(VA0 + 64'h7000);
      ib_write2(VA0 + 64'h7008, 32'hDEAD_BEEF, 32'hDEAD_BEEF);
      ib_fence(VA0 + 64'h2008, v1);
      ib_pad();
      ib_commit();
      run_ib(1, VA0 + 64'h1000, ib_n, okj);
      check(okj, "sparse run: RPTR reaches WPTR");
      check(host_q(HPG2 + 8) == v1, "sparse run: fence after sparse read / write");
      cpr(13'h011c, v); check(v == 0, "sparse: no CP error");
      lr(20'h310, v); check(v[1:0] == 2'b00, "sparse: no MMU fault record");

      // ---- faults: unmapped (NOT_PRESENT), write to a read-only page (PERMISSION), PTE -> VRAM (BAD_PHYS_ADDR)
      cpw(13'h0044, 32'h5);                                   // FENCE | MMU_FAULT interrupt
      for (int k = 0; k < 3; k++) begin
        logic [3:0] ecode;
        logic [31:0] ept_lo;
        case (k)
          0: begin va = VA0 + 64'h5008; ecode = 4'd1; ept_lo = 32'h0; end
          1: begin va = VA0 + 64'h4008; ecode = 4'd3; ept_lo = ((SYS_BASE + RO_PG) | 64'h7) & 32'hFFFF_FFFF; end
          default: begin va = VA0 + 64'h6008; ecode = 4'd5; ept_lo = 32'h4000_000F; end
        endcase
        ib_n = 0; ib_fence(va, 64'h1234_5678_9ABC_DEF0 + k); ib_pad(); ib_commit();
        run_ib_noerrwait(1, VA0 + 64'h1000, ib_n);
        wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd3, 4000, okj);
        check(okj, $sformatf("fault %0d: CP_ERROR = MEM_FAULT", k));
        lr(20'h310, v);
        check(v[0] && !v[1] && v[7:4] == ecode && v[9:8] == 2'd1 && v[13:12] == 2'd0 && v[19:16] == 4'd1,
              $sformatf("fault %0d: MMU_FAULT_STATUS %08h (code %0d)", k, v, ecode));
        lr(20'h314, wlo); lr(20'h318, whi);
        check({whi, wlo} == va, $sformatf("fault %0d: MMU_FAULT_VA %h_%h", k, whi, wlo));
        lr(20'h31c, v); check(v == ept_lo, $sformatf("fault %0d: MMU_FAULT_PTE_LO %08h", k, v));
        cpr(13'h0040, v); check(v[2], $sformatf("fault %0d: IRQ_STATUS.MMU_FAULT", k));
        repeat (20) @(posedge clk_l2);
        check(rm_irq[0] === 1'b1, $sformatf("fault %0d: rm_irq from the MMU fault", k));
        check(host_q(HPG2 + 8) != 64'h1234_5678_9ABC_DEF0 + k, $sformatf("fault %0d: write did not happen", k));
        lw(20'h310, 32'h1);                                   // W1C
        lr(20'h310, v); check(v[1:0] == 2'b00, $sformatf("fault %0d: W1C clears the record", k));
        cpw(13'h0040, 32'hFFFF_FFFF);
        repeat (20) @(posedge clk_l2);
        check(rm_irq[0] === 1'b0, $sformatf("fault %0d: rm_irq drops", k));
        cpw(13'h0114, 32'h3);                                 // ENABLE | RESET
        wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd0, 4000, okj);
        check(okj, $sformatf("fault %0d: RESET clears CP_ERROR", k));
        fence_job(64'hC0DE + k);
        wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, okj);
        check(okj && host_q(FENCE_HOST) == 64'hC0DE + k, $sformatf("fault %0d: CP works after RESET", k));
        cpw(13'h0040, 32'hFFFF_FFFF);                         // the recovery fence raised IRQ_STATUS.FENCE
      end
      cpw(13'h0044, 32'h1);
      // ---- COPY with translation in an IB: VA page 2 (host page 2 since the remap above) -> SYS data page
      //      (VA page 3)
      begin
        int bad;
        for (int k = 0; k < 64; k++) u_host.poke((HPG2 >> 3) + k, {32'hC0B1_0000 + k, 32'h5EED_0000 + k});
        ib_n = 0;
        ib_buf[0] = 32'h0007_0004;
        ib_buf[1] = 32'(VA0 + 64'h2005); ib_buf[2] = 32'((VA0 + 64'h2005) >> 32);
        ib_buf[3] = 32'(VA0 + 64'h3103); ib_buf[4] = 32'((VA0 + 64'h3103) >> 32);
        ib_buf[5] = 32'd300; ib_buf[6] = 32'h0; ib_buf[7] = 32'h0;
        ib_n = 8;
        ib_pad(); ib_commit();
        run_ib(1, VA0 + 64'h1000, ib_n, okj);
        check(okj, "translated COPY: IB retires");
        cpr(13'h011c, v); check(v == 0, "translated COPY: no CP_ERROR");
        bad = 0;
        for (int i = 0; i < 300; i++) if (c3_b(DATA_PG + 64'h103 + i) !== host_b(HPG2 + 5 + i)) bad++;
        check(bad == 0, $sformatf("translated COPY: %0d bytes differ", bad));
      end
      // ---- a context that is disabled again faults NON_CANONICAL
      lw(20'h218, 32'h0);
      okj = 0;
      for (int t = 0; t < 50 && !okj; t++) begin lr(20'h300, v); okj = (v == 0); end
      ib_n = 0; ib_fence(VA0 + 64'h2008, 64'h77); ib_pad(); ib_commit();
      run_ib_noerrwait(1, VA0 + 64'h1000, ib_n);
      wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd3, 4000, okj);
      check(okj, "disabled context: CP_ERROR = MEM_FAULT");
      lr(20'h310, v); check(v[0] && v[7:4] == 4'd2, "disabled context: NON_CANONICAL");
      lw(20'h310, 32'h1); cpw(13'h0040, 32'hFFFF_FFFF);
      cpw(13'h0114, 32'h3);
      wait_reg(13'h011c, 32'hFFFF_FFFF, 32'd0, 4000, okj);
      fence_job(64'hBEEF);
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, okj);
      check(okj && host_q(FENCE_HOST) == 64'hBEEF, "CP works after the last fault");
      $display("  mmu: walks %0d tlb hits %0d", dut.u_mmu.stat_walks, dut.u_mmu.stat_tlb_hits);
    end

    // ---------------------------------------------------------------- SP debug page, dispatch bundles
    sp_tests();

    // --------------------------------------------------------------
    $display("TEST quiesce");
    fence_prep(64'hAAAA_0001);                            // ring contents first (BAR2 writes are held off by quiesce)
    shell_quiesce_req = 1'b1;
    repeat (4) @(posedge clk_l2);
    cpw(13'h1000, ring_wp);                               // the doorbell still reaches the CP; its ring fetch waits
    begin
      int t = 0;
      while (!rm_quiesce_ack && t < 20000) begin @(posedge clk_l2); t++; end
      check(rm_quiesce_ack === 1'b1, "rm_quiesce_ack");
      u_c3.freeze = 1'b1; u_host.freeze = 1'b1;
      repeat (500) @(posedge clk_l2);                       // CP traffic is held off: freeze would fail any handshake
      u_c3.freeze = 1'b0; u_host.freeze = 1'b0;
      shell_quiesce_req = 1'b0;
      wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, ok);
      check(ok && host_q(FENCE_HOST) == 64'hAAAA_0001, "job completes after the quiesce is released");
    end

    check(u_c3.errors == 0, "c3 slave protocol checks");
    check(u_host.errors == 0, "host slave protocol checks");
    $display("tb_rm_sp: %0d checks, %0d errors; C3: %0d AW %0d AR; host: %0d AW %0d AR", checks, errors,
             u_c3.n_aw, u_c3.n_ar, u_host.n_aw, u_host.n_ar);
    if (errors == 0) $display("NG_RM_SP_TB_PASS");
    else $display("NG_RM_SP_TB_FAIL");
    $finish;
  end

  initial begin
    #2_000_000_000;
    $display("TB_FAIL global timeout; regs st=%0d bar2 st=%0d left=%0d awvalid=%b awready=%b wvalid=%b wready=%b bvalid=%b arvalid=%b arready=%b rvalid=%b c3 st=%0d",
             dut.u_regs.st, dut.u_bar2.st, dut.u_bar2.left, s_axi_bar2_awvalid, s_axi_bar2_awready,
             s_axi_bar2_wvalid, s_axi_bar2_wready, s_axi_bar2_bvalid, s_axi_bar2_arvalid, s_axi_bar2_arready,
             s_axi_bar2_rvalid, dut.u_c3.st);
    $finish;
  end
endmodule
