// tb_rm_cp.sv -- system test of the CP RM (ng_gpu_rm of rm_cp) with the real Ibex firmware.
//
// The TB plays the kernel: window C3 via BAR0, NGCF image + ring into C3 through BAR2, FW_BASE/FW_SIZE, release
// CORE_RESET, wait for READY, ring + doorbell, FENCE jobs whose fence lands in host memory (tb_rmt_slave on
// m_axi_host) and raises IRQ_STATUS.FENCE -> rm_irq.  Also: boot errors (BOOT_ERR, IRQ_CP_FW), a CP error from an
// INDIRECT with a user VMID that has no MMU context, RESET recovery, the MMU (page tables in C3, IB at a VA,
// TLB hit / stale entry / MMU_INVALIDATE, sparse, NOT_PRESENT / PERMISSION / BAD_PHYS faults, IRQ_STATUS bit 2), C3 window rules (ERR, VRAM SLVERR), BAR2
// bursts / narrow beats, and quiesce.  Build: run_tb_rm_cp.sh (Verilator + --timing, Ibex).  Pass: NG_RM_CP_TB_PASS.
`timescale 1ns / 1ps

module tb_rm_cp;
`include "tb_rm_cp_decls.svh"

  int errors = 0;
  int checks = 0;

  // clocks: clk_l2 250 MHz, clk_ts 100 MHz (asynchronous)
  initial begin clk_l2 = 0; forever #2.0 clk_l2 = ~clk_l2; end
  initial begin clk_ts = 0; #1.3; forever #5.0 clk_ts = ~clk_ts; end
  initial begin clk_core = 0; forever #2.5 clk_core = ~clk_core; end
  initial begin clk_core2x = 0; forever #1.25 clk_core2x = ~clk_core2x; end

  ng_gpu_rm #(.BUILD_ID(32'h1234abcd), .RM_REV(16'h0007)) dut (.*);

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

  tb_rmt_slave #(.DW(64), .AW(47), .IW(4), .MAXLEN(63), .MAXWR(8), .MAXRD(8), .NAME("host")) u_host (
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
    lr(20'h8, v);   check(v == 0, "CAPS");
    lw(20'h10, 32'hCAFE_F00D); lr(20'h10, v); check(v == 32'hCAFE_F00D, "SCRATCH");
    lr(20'h400, v); check(v == 32'h00_1b_14_1c, "BAR2_INFO");
    check(rm_info[31:0] == 32'h4d52474e && rm_info[63:48] == 16'h8002, "rm_info");
    check(rm_ready === 1'b1, "rm_ready");
    lr(20'h2000, v); check(v == 0, "undefined offset reads 0");
    cpr(13'h0138, v); check(v == 0, "FW_STATUS reset");
    cpr(13'h0114, v); check(v == 32'h8, "CTRL reset value CORE_RESET");
    cpr(13'h0058, v); check(v == 32'd100_000_000, "TS_FREQ");

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
    $display("tb_rm_cp: %0d checks, %0d errors; C3: %0d AW %0d AR; host: %0d AW %0d AR", checks, errors,
             u_c3.n_aw, u_c3.n_ar, u_host.n_aw, u_host.n_ar);
    if (errors == 0) $display("NG_RM_CP_TB_PASS");
    else $display("NG_RM_CP_TB_FAIL");
    $finish;
  end

  initial begin
    #400_000_000;
    $display("TB_FAIL global timeout; regs st=%0d bar2 st=%0d left=%0d awvalid=%b awready=%b wvalid=%b wready=%b bvalid=%b arvalid=%b arready=%b rvalid=%b c3 st=%0d",
             dut.u_regs.st, dut.u_bar2.st, dut.u_bar2.left, s_axi_bar2_awvalid, s_axi_bar2_awready,
             s_axi_bar2_wvalid, s_axi_bar2_wready, s_axi_bar2_bvalid, s_axi_bar2_arvalid, s_axi_bar2_arready,
             s_axi_bar2_rvalid, dut.u_c3.st);
    $finish;
  end
endmodule
