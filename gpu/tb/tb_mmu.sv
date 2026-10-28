// tb_mmu.sv -- MMU TB: ng_mmu (TLB + walker + registers + fault latch).
//
//  1) directed tests (reset state, TLB hit/miss + VMID isolation, per-VMID
//     flush, invalidate racing a walk, shadow-root / CTRL semantics, walker
//     read error, fault latch MULTI / W1C / external report, IRQ pulses);
//  2) replay of "mmu_vec.hex" produced from the functional model
//     (rtl/gpu/tb/gen_golden/gen_mmu_vectors.cpp): model page tables
//     (incl. splits / demotions / sparse / malformed entries), context and
//     invalidate operations, and ~2000 translations whose result (fault,
//     level, PTE, sparse, size, PA) and fault-latch contents must match the
//     model exactly.
// Randomisation: memory latency, rd_ready, client gaps, resp_ready backpressure
// (-sv_seed varies them; the vector file varies the page tables).
// Expect NG_MMU_TB_PASS.
`timescale 1ns/1ps
module tb_mmu;
  import ng_mmu_pkg::*;

  localparam int ENT = 8;   // small TLB: forces evictions
  localparam int MAXREC = 400000;

  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  // ---------------- DUT ------------------------------------------------------
  logic        drv_wr = 0;
  logic [11:0] drv_waddr = 0;
  logic [31:0] drv_wdata = 0;
  logic [11:0] csr_raddr = 0;
  logic [31:0] csr_rdata;
  logic        irq_fault;
  logic        hook_wr;
  logic [31:0] hook_mask;

  logic        req_valid = 0, req_ready;
  logic [3:0]  req_vmid = 0;
  logic [63:0] req_va = 0;
  logic [1:0]  req_acc = 0;
  logic        req_atomic = 0;
  logic [7:0]  req_tag = 0;
  logic        resp_valid, resp_ready = 0;
  logic [2:0]  resp_fault;
  logic        resp_sparse;
  logic [47:0] resp_pa;
  logic [2:0]  resp_size;
  logic [1:0]  resp_level;
  logic [63:0] resp_pte;
  logic [7:0]  resp_tag;

  logic        ext_flt_valid = 0;
  logic [3:0]  ext_flt_vmid = 0;
  logic [63:0] ext_flt_va = 0;
  logic [1:0]  ext_flt_acc = 0;
  logic [2:0]  ext_flt_code = 0;
  logic [1:0]  ext_flt_level = 0;
  logic [63:0] ext_flt_pte = 0;

  logic        rd_valid, rd_ready;
  logic [47:0] rd_addr;
  logic        rsp_valid;
  logic [63:0] rsp_data;
  logic        rsp_err;
  logic [31:0] stat_walks, stat_tlb_hits;

  ng_mmu #(.TLB_ENTRIES(ENT)) dut (
    .clk, .rst_n,
    .csr_wr(drv_wr | hook_wr),
    .csr_waddr(hook_wr ? 12'h300 : drv_waddr),
    .csr_wdata(hook_wr ? hook_mask : drv_wdata),
    .csr_raddr, .csr_rdata, .irq_fault,
    .req_valid, .req_ready, .req_vmid, .req_va, .req_acc, .req_atomic, .req_tag,
    .resp_valid, .resp_ready, .resp_fault, .resp_sparse, .resp_pa, .resp_size,
    .resp_level, .resp_pte, .resp_tag,
    .ext_flt_valid, .ext_flt_vmid, .ext_flt_va, .ext_flt_acc, .ext_flt_code,
    .ext_flt_level, .ext_flt_pte,
    .rd_valid, .rd_ready, .rd_addr, .rsp_valid, .rsp_data, .rsp_err,
    .stat_walks, .stat_tlb_hits
  );

  // ---------------- table memory model --------------------------------------
  logic [63:0] tmem [bit [44:0]];
  function automatic logic [63:0] mem_rd(input logic [47:0] a);
    if (tmem.exists(a[47:3])) return tmem[a[47:3]];
    return 64'h0;
  endfunction
  task automatic mem_wr(input logic [47:0] a, input logic [63:0] d);
    if (d == 64'h0) begin
      if (tmem.exists(a[47:3])) tmem.delete(a[47:3]);
    end else tmem[a[47:3]] = d;
  endtask

  int  hook_delay_v[2] = '{0, 1};
  int  lat_max  = 4;        // latency 1..lat_max
  int  rdy_pct  = 70;       // rd_ready probability
  int  rrdy_pct = 70;       // resp_ready probability
  int  gap_max  = 3;        // idle cycles between client requests

  bit          err_en = 0;
  logic [47:0] err_addr = 0;
  bit          hook_en = 0;
  logic [47:0] hook_addr = 0;
  logic [63:0] hook_new = 0;
  int          hook_delay = 0;
  logic [31:0] hook_mask_r = 0;
  bit          hook_f0 = 0, hook_f1 = 0;
  assign hook_mask = hook_mask_r;
  assign hook_wr   = (hook_delay == 0) ? hook_f0 : hook_f1;

  bit          m_pend = 0;
  int          m_cnt = 0;
  logic [47:0] m_addr = 0;
  bit          rdy_rand = 0;
  assign rd_ready = rdy_rand && !m_pend && !rsp_valid;

  task automatic serve(input logic [47:0] a);
    rsp_valid <= 1'b1;
    rsp_data  <= mem_rd(a);
    rsp_err   <= err_en && (a == err_addr);
    if (hook_en && a == hook_addr) begin
      mem_wr(a, hook_new);
      hook_f0 <= 1'b1;
      hook_en = 0;
    end
  endtask

  always @(posedge clk) begin
    rsp_valid <= 1'b0;
    rsp_err   <= 1'b0;
    hook_f0   <= 1'b0;
    hook_f1   <= hook_f0;
    rdy_rand  <= ($urandom_range(99) < rdy_pct);
    if (!rst_n) begin
      m_pend <= 1'b0;
    end else if (m_pend) begin
      if (m_cnt <= 0) begin
        serve(m_addr);
        m_pend <= 1'b0;
      end else m_cnt <= m_cnt - 1;
    end else if (rd_valid && rd_ready) begin
      automatic int l = 1 + $urandom_range(lat_max - 1);
      if (l == 1) serve(rd_addr);
      else begin
        m_pend <= 1'b1;
        m_addr <= rd_addr;
        m_cnt  <= l - 2;
      end
    end
  end

  // client-side resp_ready randomisation
  always @(posedge clk) resp_ready <= ($urandom_range(99) < rrdy_pct);

  // irq pulse counter
  int irq_count = 0;
  always @(posedge clk) if (irq_fault) irq_count++;

  // ---------------- checking -------------------------------------------------
  int errors = 0, checks = 0;
  task automatic chk(input bit cond, input string msg);
    checks++;
    if (!cond) begin
      errors++;
      if (errors <= 40) $display("TB_FAIL @%0t: %s", $time, msg);
    end
  endtask

  // ---------------- register access ------------------------------------------
  task automatic csr_write(input logic [11:0] a, input logic [31:0] d);
    @(posedge clk); #1;
    drv_wr = 1; drv_waddr = a; drv_wdata = d;
    @(posedge clk); #1;
    drv_wr = 0;
  endtask
  task automatic csr_read(input logic [11:0] a, output logic [31:0] d);
    csr_raddr = a; #1;
    d = csr_rdata;
  endtask

  task automatic wait_inv_done(input string who);
    logic [31:0] v;
    int n = 0;
    do begin
      csr_read(12'h300, v);
      @(posedge clk); #1;
      n++;
    end while (v != 0 && n < 16);
    chk(v == 0, {"invalidate never completes: ", who});
  endtask

  task automatic set_ctx(input int vmid, input logic [47:0] root, input bit en);
    logic [31:0] v;
    csr_write(12'h200 + 12'(vmid) * 16 + 0, {root[31:12], 12'h0});
    csr_write(12'h200 + 12'(vmid) * 16 + 4, {16'h0, root[47:32]});
    csr_write(12'h200 + 12'(vmid) * 16 + 8, {31'h0, en});
    wait_inv_done("ctx");
    csr_read(12'h200 + 12'(vmid) * 16 + 0, v);
    chk(v == {root[31:12], 12'h0}, "ROOT_LO readback");
    csr_read(12'h200 + 12'(vmid) * 16 + 4, v);
    chk(v == {16'h0, root[47:32]}, "ROOT_HI readback");
    csr_read(12'h200 + 12'(vmid) * 16 + 8, v);
    chk(v == {31'h0, en}, "CTRL readback");
  endtask

  task automatic do_inv(input logic [15:0] mask);
    logic [31:0] v;
    csr_write(12'h300, {16'h0, mask});
    // pending must be visible right after the write and gone within a few cycles
    wait_inv_done("inv");
  endtask

  // ---------------- client query ---------------------------------------------
  typedef struct {
    logic [2:0]  fault;
    logic        sparse;
    logic [47:0] pa;
    logic [2:0]  size;
    logic [1:0]  level;
    logic [63:0] pte;
    logic [7:0]  tag;
  } resp_t;

  task automatic query(input logic [3:0] vmid, input logic [63:0] va, input logic [1:0] acc,
                       input bit atomic, output resp_t r);
    static logic [7:0] tagc = 0;
    int w;
    tagc++;
    repeat ($urandom_range(gap_max)) @(posedge clk);
    #1;
    req_valid = 1; req_vmid = vmid; req_va = va; req_acc = acc; req_atomic = atomic; req_tag = tagc;
    w = 0;
    forever begin
      @(posedge clk);
      if (req_ready) break;
      if (++w > 1000) begin chk(0, "req_ready timeout"); break; end
    end
    #1; req_valid = 0;
    w = 0;
    forever begin
      @(posedge clk);
      if (resp_valid && resp_ready) begin
        r.fault = resp_fault; r.sparse = resp_sparse; r.pa = resp_pa; r.size = resp_size;
        r.level = resp_level; r.pte = resp_pte; r.tag = resp_tag;
        break;
      end
      if (++w > 5000) begin chk(0, "resp timeout"); r = '{default: '0}; break; end
    end
    chk(r.tag == tagc, "tag echo");
  endtask

  // expected fault-latch contents after a query (status cleared before it)
  task automatic chk_latch(input bit exp_valid, input logic [3:0] vmid, input logic [63:0] va,
                           input logic [1:0] acc, input logic [2:0] code, input logic [1:0] level,
                           input logic [63:0] pte, input string tag);
    logic [31:0] s, lo, hi, plo, phi;
    repeat (3) @(posedge clk);
    #1;
    csr_read(12'h310, s);
    chk(s[0] == exp_valid, {tag, ": FAULT_STATUS.VALID"});
    if (exp_valid) begin
      chk(s[1] == 1'b0, {tag, ": MULTI unexpectedly set"});
      chk(s[7:4] == {1'b0, code}, $sformatf("%s: CODE %0d/%0d", tag, s[7:4], code));
      chk(s[9:8] == acc, $sformatf("%s: ACCESS %0d/%0d", tag, s[9:8], acc));
      chk(s[13:12] == level, $sformatf("%s: LEVEL %0d/%0d", tag, s[13:12], level));
      chk(s[23:16] == {4'h0, vmid}, {tag, ": VMID"});
      csr_read(12'h314, lo); csr_read(12'h318, hi);
      chk({hi, lo} == va, $sformatf("%s: FAULT_VA %h/%h", tag, {hi, lo}, va));
      csr_read(12'h31C, plo); csr_read(12'h320, phi);
      chk({phi, plo} == pte, $sformatf("%s: FAULT_PTE %h/%h", tag, {phi, plo}, pte));
    end
  endtask

  // =========================================================================
  // directed tests
  // =========================================================================
  localparam logic [47:0] SB = 48'h0010_0000_0000;
  function automatic logic [63:0] pde(input logic [47:0] pa);
    return {16'h0, pa[47:12], 12'h001};
  endfunction
  function automatic logic [63:0] pte_rw(input logic [47:0] pa);
    return {16'h0, pa[47:12], 12'h00F};   // V LEAF R W
  endfunction

  task automatic build_basic;
    // root SB+0x1000 -> L2 SB+0x2000 -> L1 SB+0x3000 -> L0 SB+0x4000
    mem_wr(SB + 48'h1000, pde(SB + 48'h2000));
    mem_wr(SB + 48'h2000, pde(SB + 48'h3000));
    mem_wr(SB + 48'h3000, pde(SB + 48'h4000));
    mem_wr(SB + 48'h4000, pte_rw(48'h1234_5000));
    mem_wr(SB + 48'h4008, pte_rw(48'h5000_0000));
  endtask

  task automatic directed;
    resp_t r;
    logic [31:0] v;
    int w0, h0, ic0;

    // ---- reset state: every context disabled -> NON_CANONICAL ----
    query(4'd1, 64'h123, ACC_R, 0, r);
    chk(r.fault == F_NON_CANONICAL && r.level == 0 && r.pte == 0, "reset: ctx disabled");
    query(4'd0, 64'h0000_0003_2100_5678, ACC_W, 0, r);   // VMID 0 bypass (VRAM)
    chk(r.fault == F_NONE && r.pa == 48'h0003_2100_5678 && !r.sparse && r.size == SZ_4K, "bypass");
    query(4'd0, 64'h0000_4321_0000_5678, ACC_W, 0, r);   // VMID 0 bypass into a hole
    chk(r.fault == F_BAD_PHYS && r.level == 0 && r.pte == 0 && !r.sparse, "bypass hole: BAD_PHYS_ADDR");
    csr_read(12'h310, v);
    chk(v == 0 || v[0], "status readable");   // NON_CANONICAL above was latched
    csr_write(12'h310, 32'h1);

    // ---- VMID 0 bypass with VA[63:48] != 0 -> NON_CANONICAL (no truncation) ----
    begin
      logic [63:0] bva [4];
      logic [1:0]  bacc [4];
      bit          bat [4];
      string       bnm [4];
      bva[0] = 64'h0001_0000_0000_1000; bacc[0] = ACC_R; bat[0] = 0; bnm[0] = "bypass hiVA read";
      bva[1] = 64'h8000_0000_0000_0040; bacc[1] = ACC_W; bat[1] = 0; bnm[1] = "bypass hiVA write";
      bva[2] = 64'hFFFF_0000_dead_0000; bacc[2] = ACC_W; bat[2] = 1; bnm[2] = "bypass hiVA atomic";
      bva[3] = 64'h0100_0000_0000_0000; bacc[3] = ACC_X; bat[3] = 0; bnm[3] = "bypass hiVA fetch";
      foreach (bva[i]) begin
        query(4'd0, bva[i], bacc[i], bat[i], r);
        chk(r.fault == F_NON_CANONICAL && r.level == 0 && r.pte == 0 && !r.sparse, {bnm[i], ": fault"});
        chk_latch(1, 0, bva[i], bacc[i], F_NON_CANONICAL, 0, 64'h0, {bnm[i], " latch"});
        csr_write(12'h310, 32'h1);
      end
      // just below the boundary stays legal; bit 47 set is a valid PA
      query(4'd0, 64'h0000_FFFF_FFFF_F000, ACC_R, 0, r);
      chk(r.fault == F_NONE && r.pa == 48'hFFFF_FFFF_F000 && r.size == SZ_4K, "bypass VA<2^48 max");
      query(4'd0, 64'h0000_8000_0000_0000, ACC_X, 0, r);
      chk(r.fault == F_NONE && r.pa == 48'h8000_0000_0000, "bypass VA<2^48 fetch");
      chk_latch(0, 0, 64'h0, ACC_R, F_NONE, 0, 64'h0, "bypass legal: no latch");
    end

    // ---- VMID 0 bypass PA must lie in VRAM [0,16 GiB) / SYS [0x10_0000_0000,+2 GiB) / HOST [2^47,2^48) ----
    begin
      logic [63:0] pva [14];
      bit          pok [14];
      logic [1:0]  pacc [4];
      bit          pat [4];
      string       pnm [4];
      pacc[0] = ACC_R; pat[0] = 0; pnm[0] = "read";
      pacc[1] = ACC_W; pat[1] = 0; pnm[1] = "write";
      pacc[2] = ACC_W; pat[2] = 1; pnm[2] = "atomic";
      pacc[3] = ACC_X; pat[3] = 0; pnm[3] = "fetch";
      pva[0]  = 64'h0;                          pok[0]  = 1;   // VA 0 / VA 1: VRAM, legal
      pva[1]  = 64'h1;                          pok[1]  = 1;
      pva[2]  = 64'h0000_0003_FFFF_FFFF;        pok[2]  = 1;   // last VRAM byte
      pva[3]  = 64'h0000_0004_0000_0000;        pok[3]  = 0;   // first byte past VRAM
      pva[4]  = 64'h0000_000F_FFFF_FFFF;        pok[4]  = 0;
      pva[5]  = 64'h0000_0010_0000_0000;        pok[5]  = 1;   // SYS base
      pva[6]  = 64'h0000_0010_7FFF_FFFF;        pok[6]  = 1;   // last SYS byte
      pva[7]  = 64'h0000_0010_8000_0000;        pok[7]  = 0;   // first byte past SYS
      pva[8]  = 64'h0000_0020_0000_0100;        pok[8]  = 0;
      pva[9]  = 64'h0000_7FFF_FFFF_FFFF;        pok[9]  = 0;   // last byte below HOST
      pva[10] = 64'h0000_8000_0000_0000;        pok[10] = 1;   // HOST base
      pva[11] = 64'h0000_FFFF_FFFF_FFFF;        pok[11] = 1;   // top of the 48-bit space
      pva[12] = 64'h0000_0010_0000_0000 - 1;    pok[12] = 0;   // just below SYS
      pva[13] = 64'h0000_0000_4000_0000;        pok[13] = 1;   // 1 GiB: VRAM, legal
      for (int k = 0; k < 4; k++) begin
        foreach (pva[i]) begin
          query(4'd0, pva[i], pacc[k], pat[k], r);
          if (pok[i]) begin
            chk(r.fault == F_NONE && r.pa == pva[i][47:0] && !r.sparse, $sformatf("bypass window %h %s: passes", pva[i], pnm[k]));
          end else begin
            chk(r.fault == F_BAD_PHYS && r.level == 0 && r.pte == 0 && !r.sparse,
                $sformatf("bypass window %h %s: BAD_PHYS_ADDR", pva[i], pnm[k]));
            chk_latch(1, 0, pva[i], pacc[k], F_BAD_PHYS, 0, 64'h0, $sformatf("bypass window %h %s latch", pva[i], pnm[k]));
            csr_write(12'h310, 32'h1);
          end
        end
      end
      // NON_CANONICAL still has priority; a hole does not wedge the next translation
      query(4'd0, 64'h0001_0000_0000_0000, ACC_R, 0, r);
      chk(r.fault == F_NON_CANONICAL, "bypass 2^48 stays NON_CANONICAL");
      csr_write(12'h310, 32'h1);
      query(4'd0, 64'h0000_0010_0000_1000, ACC_W, 0, r);
      chk(r.fault == F_NONE && r.pa == 48'h0010_0000_1000, "bypass legal after a hole");
    end

    build_basic();
    // ---- hit / miss / VMID isolation / per-VMID flush ----
    set_ctx(1, SB + 48'h1000, 1);
    set_ctx(2, SB + 48'h1000, 1);       // same tables, separate VMID
    w0 = stat_walks; h0 = stat_tlb_hits;
    query(4'd1, 64'h123, ACC_R, 0, r);
    chk(r.fault == F_NONE && r.pa == 48'h1234_5123 && r.size == SZ_4K && !r.sparse && r.level == 0, "basic translate");
    chk(stat_walks == w0 + 1 && stat_tlb_hits == h0, "first access walks");
    query(4'd1, 64'h7F8, ACC_W, 0, r);
    chk(r.fault == F_NONE && r.pa == 48'h1234_57F8, "same page offset");
    chk(stat_walks == w0 + 1 && stat_tlb_hits == h0 + 1, "second access hits");
    query(4'd2, 64'h123, ACC_R, 0, r);
    chk(stat_walks == w0 + 2, "VMID 2 does not hit VMID 1's entry");
    do_inv(16'h0002);                   // flush VMID 1 only
    query(4'd2, 64'h124, ACC_R, 0, r);
    chk(stat_walks == w0 + 2, "VMID 2 entry survives flush of VMID 1");
    query(4'd1, 64'h124, ACC_R, 0, r);
    chk(stat_walks == w0 + 3, "VMID 1 walks again after flush");
    do_inv(16'h0001);                   // bit 0 ignored
    query(4'd1, 64'h125, ACC_R, 0, r);
    chk(stat_walks == w0 + 3, "invalidate bit0 is ignored");
    // permission: page is RW (no X), atomics need R and W
    query(4'd1, 64'h10, ACC_X, 0, r);
    chk(r.fault == F_PERMISSION && r.level == 0 && r.pte == pte_rw(48'h1234_5000), "exec on RW page");
    chk_latch(1, 1, 64'h10, ACC_X, F_PERMISSION, 0, pte_rw(48'h1234_5000), "perm latch");
    csr_write(12'h310, 32'h1);
    mem_wr(SB + 48'h4010, {16'h0, 36'h6_0000_0, 12'h00B});   // V LEAF W only (write-only page)
    do_inv(16'h0002);
    query(4'd1, 64'h2000, ACC_W, 0, r);
    chk(r.fault == F_NONE, "write-only page: store ok");
    query(4'd1, 64'h2000, ACC_W, 1, r);
    chk(r.fault == F_PERMISSION, "write-only page: atomic needs R");
    query(4'd1, 64'h2000, ACC_R, 0, r);
    chk(r.fault == F_PERMISSION, "write-only page: load denied");
    csr_write(12'h310, 32'h1);

    // ---- invalidate racing a walk (hook fires in the cycle the leaf is read) ----
    foreach (hook_delay_v[i]) begin
      automatic int dly = hook_delay_v[i];
      mem_wr(SB + 48'h4008, pte_rw(48'h5000_0000));
      set_ctx(3, SB + 48'h1000, 1);
      hook_delay = dly; hook_en = 1; hook_addr = SB + 48'h4008; hook_new = pte_rw(48'h6000_0000);
      hook_mask_r = 32'h0008;
      w0 = stat_walks;
      query(4'd3, 64'h1040, ACC_R, 0, r);
      chk(r.fault == F_NONE, "race: no fault");
      if (dly == 0) begin
        chk(r.pa == 48'h6000_0040, $sformatf("race d0: stale walk result used (pa=%h)", r.pa));
        chk(stat_walks == w0 + 2, "race d0: walk aborted and restarted after the invalidate");
      end else
        chk(r.pa == 48'h6000_0040 || r.pa == 48'h5000_0040, "race d1: old or new translation");
      wait_inv_done("race");
      query(4'd3, 64'h1040, ACC_R, 0, r);
      chk(r.pa == 48'h6000_0040, $sformatf("race d%0d: stale entry survived the invalidate (pa=%h)", dly, r.pa));
    end

    // ---- shadow root / CTRL semantics / disable ----
    mem_wr(SB + 48'h5000, pde(SB + 48'h6000));
    mem_wr(SB + 48'h6000, pde(SB + 48'h7000));
    mem_wr(SB + 48'h7000, pde(SB + 48'h8000));
    mem_wr(SB + 48'h8000, pte_rw(48'h7777_0000));
    set_ctx(5, SB + 48'h1000, 1);
    query(4'd5, 64'h33, ACC_R, 0, r);
    chk(r.fault == F_NONE && r.pa == 48'h1234_5033, "ctx5 old root");
    csr_write(12'h250, {SB[31:12] + 20'h5, 12'h0});            // new shadow root = SB+0x5000 (LO)
    csr_write(12'h254, {16'h0, SB[47:32]});
    do_inv(16'h0000);
    query(4'd5, 64'h34, ACC_R, 0, r);
    chk(r.fault == F_NONE && r.pa == 48'h1234_5034, "shadow write does not change the active root");
    csr_write(12'h258, 32'h1);
    wait_inv_done("ctrl");
    query(4'd5, 64'h35, ACC_R, 0, r);
    chk(r.fault == F_NONE && r.pa == 48'h7777_0035, "CTRL latches the new root and flushes");
    csr_write(12'h258, 32'h0);
    wait_inv_done("ctrl0");
    query(4'd5, 64'h35, ACC_R, 0, r);
    chk(r.fault == F_NON_CANONICAL, "ENABLE=0 -> NON_CANONICAL");
    csr_write(12'h310, 32'h1);
    csr_write(12'h208, 32'h1);                                  // VMID 0 slot: ignored
    query(4'd0, 64'h99, ACC_R, 0, r);
    chk(r.fault == F_NONE && r.pa == 48'h99, "VMID 0 slot write ignored");

    // ---- walker read error ----
    set_ctx(4, SB + 48'h1000, 1);
    err_en = 1; err_addr = SB + 48'h3000;
    csr_write(12'h310, 32'h1);
    query(4'd4, 64'h40, ACC_R, 0, r);
    chk(r.fault == F_BAD_PHYS && r.level == 1 && r.pte == 0, "walker read error -> BAD_PHYS");
    chk_latch(1, 4, 64'h40, ACC_R, F_BAD_PHYS, 1, 0, "read-error latch");
    err_en = 0;
    csr_write(12'h310, 32'h1);
    query(4'd4, 64'h40, ACC_R, 0, r);
    chk(r.fault == F_NONE, "error result was not cached");

    // ---- fault latch: first fault kept, MULTI, W1C re-arm, IRQ pulse ----
    csr_write(12'h310, 32'h1);
    ic0 = irq_count;
    query(4'd1, 64'hA000_0000, ACC_R, 0, r);     // NOT_PRESENT at L2? (root idx0 -> L2 idx2 empty)
    chk(r.fault == F_NOT_PRESENT, "latch: first fault is NOT_PRESENT");
    query(4'd1, 64'h10, ACC_X, 0, r);            // PERMISSION
    query(4'd1, 64'h0001_0000_0000_0000, ACC_R, 0, r);   // NON_CANONICAL
    repeat (3) @(posedge clk);
    csr_read(12'h310, v);
    chk(v[0] && v[1], "MULTI set by later faults");
    chk(v[7:4] == F_NOT_PRESENT && v[9:8] == ACC_R, "first fault kept");
    csr_read(12'h314, v);
    chk(v == 32'hA000_0000, "first fault VA kept");
    chk(irq_count == ic0 + 1, "one IRQ pulse per VALID rise");
    csr_write(12'h310, 32'h2);                   // bit0=0: no clear
    csr_read(12'h310, v);
    chk(v[0] == 1'b1, "W1C ignores bit0=0");
    csr_write(12'h310, 32'h1);
    csr_read(12'h310, v);
    chk(v[1:0] == 2'b00, "W1C clears VALID and MULTI");
    query(4'd1, 64'h10, ACC_X, 0, r);
    chk_latch(1, 1, 64'h10, ACC_X, F_PERMISSION, 0, pte_rw(48'h1234_5000), "re-armed latch");
    chk(irq_count == ic0 + 2, "IRQ again after re-arm");
    query(4'd1, 64'h0001_0000_0000_0000, ACC_W, 0, r);
    repeat (3) @(posedge clk);
    csr_read(12'h310, v);
    chk(v[0] && v[1] && v[7:4] == F_PERMISSION, "first fault still kept after re-arm + 1 more");

    // ---- external fault report (BAD_PHYS_ADDR / MISALIGNED share the latch) ----
    csr_write(12'h310, 32'h1);
    @(posedge clk); #1;
    ext_flt_valid = 1; ext_flt_vmid = 4'd9; ext_flt_va = 64'hDEAD_BEEF_1234; ext_flt_acc = ACC_W;
    ext_flt_code = F_MISALIGNED; ext_flt_level = 2'd1; ext_flt_pte = 64'h55;
    @(posedge clk); #1;
    ext_flt_valid = 0;
    chk_latch(1, 9, 64'hDEAD_BEEF_1234, ACC_W, F_MISALIGNED, 1, 64'h55, "external report");
    csr_write(12'h310, 32'h1);

    // ---- sparse & size codes via hand tables: 2M leaf, sparse L1, F64K group ----
    mem_wr(SB + 48'h3008, {16'h0, 36'h0_0040_0, 12'h00F});        // L1 idx1: 2M leaf pa 0x40_0000? (aligned)
    mem_wr(SB + 48'h3010, 64'h100);                                // L1 idx2: 2M sparse
    for (int i = 0; i < 16; i++) mem_wr(SB + 48'h4000 + 8 * (32 + i), {16'h0, 36'h9_0000_0, 12'h08F}); // 64K group
    do_inv(16'h0002);
    query(4'd1, 64'h20_1234, ACC_R, 0, r);
    chk(r.fault == F_NONE && r.size == SZ_2M && r.pa == 48'h0040_1234, $sformatf("2M leaf pa=%h", r.pa));
    query(4'd1, 64'h40_0008, ACC_W, 0, r);
    chk(r.fault == F_NONE && r.sparse && r.size == SZ_2M && r.level == 1, "2M sparse-null");
    query(4'd1, 64'h40_0008, ACC_X, 0, r);
    chk(r.fault == F_PERMISSION && r.level == 1 && r.pte == 64'h100, "exec on sparse-null");
    query(4'd1, 64'h2_1FE8, ACC_R, 0, r);                     // L0 idx 32+1, 64K group member
    chk(r.fault == F_NONE && r.size == SZ_64K && r.pa == 48'h0009_0000_1FE8, $sformatf("64K pa=%h", r.pa));
    csr_write(12'h310, 32'h1);
  endtask

  // =========================================================================
  // vector replay
  // =========================================================================
  logic [63:0] vec [4*MAXREC];

  task automatic replay;
    int nq = 0, nflt = 0, nrec = 0;
    int ic_start;
    int fault_expected = 0;
    resp_t r;
    logic [31:0] v;
    ic_start = irq_count;
    $readmemh("mmu_vec.hex", vec);
    for (int i = 0; i < MAXREC; i++) begin
      logic [63:0] w0, w1, w2, w3;
      logic [7:0] op;
      w0 = vec[4*i]; w1 = vec[4*i+1]; w2 = vec[4*i+2]; w3 = vec[4*i+3];
      if (^w0 === 1'bx) begin chk(0, "vector file ended without END"); break; end
      op = w0[63:56];
      nrec++;
      case (op)
        8'd0: break;
        8'd1: tmem.delete();
        8'd2: mem_wr(w1[47:0], w2);
        8'd3: set_ctx(int'(w0[55:48]), w1[47:0], w0[0]);
        8'd4: do_inv(w1[15:0]);
        8'd6: $display("  vector phase %0d (%0d queries so far, walks %0d hits %0d)", w1, nq, stat_walks, stat_tlb_hits);
        8'd5: begin
          logic [3:0] vmid;
          logic [1:0] acc;
          bit atomic;
          logic [2:0] ef;
          bit esp;
          string m;
          vmid = w0[51:48]; acc = w0[41:40]; atomic = w0[44];
          ef = w0[34:32]; esp = w0[24];
          csr_write(12'h310, 32'h1);
          query(vmid, w1, acc, atomic, r);
          nq++;
          m = $sformatf("q%0d vmid %0d va %h acc %0d%s", nq, vmid, w1, acc, atomic ? "A" : "");
          chk(r.fault == ef, $sformatf("%s: fault %0d/%0d", m, r.fault, ef));
          chk(r.level == w0[17:16], $sformatf("%s: level %0d/%0d", m, r.level, w0[17:16]));
          chk(r.pte == w3, $sformatf("%s: pte %h/%h", m, r.pte, w3));
          if (ef == F_NONE) begin
            chk(r.sparse == esp, $sformatf("%s: sparse %0d/%0d", m, r.sparse, esp));
            chk(r.size == w0[10:8], $sformatf("%s: size %0d/%0d", m, r.size, w0[10:8]));
            if (!esp) chk(r.pa == w2[47:0], $sformatf("%s: pa %h/%h", m, r.pa, w2[47:0]));
          end
          if (ef != F_NONE) begin
            nflt++;
            chk_latch(1, vmid, w1, atomic ? ACC_W : acc, ef, w0[17:16], w3, m);
          end else chk_latch(0, 0, 0, 0, 0, 0, 0, m);
        end
        default: chk(0, $sformatf("bad vector op %0d", op));
      endcase
    end
    chk(irq_count - ic_start >= nflt, $sformatf("IRQ pulses %0d < faults %0d", irq_count - ic_start, nflt));
    $display("  replay: %0d records, %0d queries (%0d faulting), walks %0d, tlb hits %0d",
             nrec, nq, nflt, stat_walks, stat_tlb_hits);
    chk(nq > 1000, "too few queries replayed");
    chk(stat_tlb_hits > 100, "TLB hit path barely exercised");
  endtask

  // =========================================================================
  initial begin
    lat_max  = 1 + $urandom_range(6);
    rdy_pct  = 30 + $urandom_range(70);
    rrdy_pct = 30 + $urandom_range(70);
    gap_max  = $urandom_range(4);
    $display("tb_mmu knobs: lat_max=%0d rdy=%0d%% rrdy=%0d%% gap_max=%0d", lat_max, rdy_pct, rrdy_pct, gap_max);
    repeat (4) @(posedge clk);
    #1 rst_n = 1;
    repeat (2) @(posedge clk);
    directed();
    $display("  directed done: %0d checks, %0d errors", checks, errors);
    replay();
    if (errors == 0) $display("NG_MMU_TB_PASS: %0d checks, 0 errors", checks);
    else $display("NG_MMU_TB_FAIL: %0d errors / %0d checks", errors, checks);
    $finish;
  end

  // watchdog
  initial begin
    #2_000_000_000;
    $display("NG_MMU_TB_FAIL: watchdog");
    $finish;
  end
endmodule
