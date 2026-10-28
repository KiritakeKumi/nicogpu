// tb_sp_top.sv -- ng_sp_top against a functional-model dispatch bundle.
//
// The bundle (tools/dispatch_bundle, format in) holds one
// DISPATCH as the HAL sim backend ran it: the 42 WDIST words, the VMID's page
// map (VA page -> PA page + R/W/X/sparse), every non-zero memory page before
// and after the dispatch, the model's commit trace (per warp: pc, w0, exec)
// and its final status (ok / trap record / instruction count).
//
// The TB plays CP + MMU + fabric:
//   * CP: the bundle's request is written into a real ng_cp_port (WDIST_REQ dwords + GO), whose wd_start /
//     wd_req drive the SP; waits for wd_done;
//   * MMU stub: translates through the page map (fault codes as ng_mmu:
//     1 NotPresent, 2 NonCanonical, 3 Permission; sparse pages);
//   * fabric: PA-addressed memory (bundle pre-image, other pages zero) for
//     M_RD / M_WR / M_AMO (ng_l2_amo_alu) / L2 maintenance (ack only);
//     MMU and fabric responses take 0..MAXLAT random extra cycles.
// Checks:
//   * every committed instruction against the model's trace of the same warp
//     (warp key = linear WG id * 32 + warp index; pc, w0, exec);
//   * ok bundles: no fault, every warp's trace consumed exactly, final memory
//     byte-identical on every page (bundle pages and pages the RTL created),
//     WDIST instruction count = model count;
//   * trap bundles: wd_fault, first trap record = model's (wg, warp, pc,
//     cause, info), commits before it equal the model's prefix;
//   * sp_idle: 1 before the dispatch, 0 while it runs, 1 after wd_done.
// Reset mode (tb_sp_top_run.txt): core_reset after each listed number of
// commits of a fresh start of the dispatch (held until a fabric / MMU
// transaction is in flight); checks that nothing commits afterwards, that the
// abort produces no wd_done, the CP port (ng_cp_port, in the TB path) reports WDIST busy 0, sp_idle returns; the memory image is restored after
// each abort and finally the dispatch runs to completion with the full checks
// (the MMU / fabric stubs are not reset, so a transaction in flight across the
// reset exercises the SP's drop logic).
//
// All DUT inputs are driven at negedge or from posedge state machines with
// non-blocking assignments only (Verilator --timing NBA-in-task issue).
`timescale 1ns/1ps
`include "tb_sp_top_cfg.svh"
`ifndef NG_TB_CLEAR
`define NG_TB_CLEAR 0
`endif
`ifndef NG_TB_BDEPTH
`define NG_TB_BDEPTH 3
`endif
`ifndef NG_TB_BRND
`define NG_TB_BRND 1
`endif
`ifndef NG_TB_IC_LINES
`define NG_TB_IC_LINES 32
`endif
`ifndef NG_TB_SCRUB
`define NG_TB_SCRUB 0
`endif
`ifndef NG_TB_TIMEOUT
`define NG_TB_TIMEOUT 4000000
`endif
`ifndef NG_TB_LSU_K
`define NG_TB_LSU_K 1
`endif
`ifndef NG_TB_MEMOUT
`define NG_TB_MEMOUT 1
`endif
`ifndef NG_TB_FLAT_LO
`define NG_TB_FLAT_LO 0
`endif
`ifndef NG_TB_FLAT_HI
`define NG_TB_FLAT_HI `NG_TB_MAXLAT
`endif
`ifndef NG_TB_ORPHAN
`define NG_TB_ORPHAN 1
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
module tb_sp_top;
  import ng_lsu_pkg::*;
  import ng_cp_pkg::*;

  localparam int NL      = `NG_TB_NL;
  localparam int MAXLAT  = `NG_TB_MAXLAT;
  localparam bit ENX     = `NG_TB_EN_EXTRA;
  localparam bit CLRV    = `NG_TB_CLEAR;
  localparam int MAXPG   = `NG_TB_MAXPAGES;
  localparam int MAXMAP  = `NG_TB_MAXMAP;
  localparam int MAXTR   = `NG_TB_MAXTR;
  localparam int TIMEOUT = `NG_TB_TIMEOUT;
  // LSU contexts / fabric slots (MEMOUT > 1: the tagged multi-outstanding fabric model g_fslot below)
  localparam int LSU_K   = `NG_TB_LSU_K;
  localparam int MEMOUT  = `NG_TB_MEMOUT;
  localparam int TSW     = ng_noc_pkg::clog2_1(MEMOUT);

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic core_reset = 1'b0;
  always #5 clk = ~clk;

  // --------------------------------------------------------------
  logic          wd_start;
  wd_req_t       wd_req;
  logic          wd_done, wd_fault, sp_idle;
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
  logic          mem_req_valid, mem_req_ready, mem_rsp_valid;
  mreq_t         mem_req;
  mrsp_t         mem_rsp;
  // slot-protocol side (MEMOUT > 1, driven by g_fslot; tied off for the serial stub)
  logic          sl_ready, sl_rsp_valid, sl_rsp_rd, mem_free;
  mrsp_t         sl_rsp;
  logic [TSW-1:0] sl_fslot, sl_rsp_slot, mem_rd_slot;
  logic [1023:0] sl_rd_data;
  logic          sl_busy;          // any fabric slot allocated (reset mode: wait for it before restoring memory)
  int            sl_n_acc = 0, sl_max_out = 0, sl_n_orph = 0, sl_n_late_free = 0, sl_n_cstall = 0, sl_n_fstall = 0;
  logic          dbg_commit_valid, dbg_trap_valid;
  logic [3:0]    dbg_commit_slot, dbg_trap_slot, dbg_trap_cause;
  logic [31:0]   dbg_commit_pc, dbg_commit_w0, dbg_commit_exec, dbg_commit_wg;
  logic [4:0]    dbg_commit_warp, dbg_trap_warp;
  logic [2:0]    dbg_trap_unit;
  logic [31:0]   dbg_trap_pc, dbg_trap_info, dbg_trap_wg, st_ic_hit, st_ic_miss;
  logic [31:0]   st_l1_rd, st_l1_wr, st_instr_mem, st_groups;
  logic [255:0]  st_perf;   // performance counters
  logic [159:0]  st_scrub;  // counters (scrub dispatches / clocks, L1D external invalidates)
  logic [15:0]   scrub_set = '0;   // MMU CTX_CTRL write pulses into ng_cp_port (driven by the scrub mode)
  logic [2:0]    sp_inv;           // P_SP_INV pulses (CACHE packet path) from ng_cp_port
  longint        n_valu_acc_tb = 0;   // VALU accepts since the last datapath reset

  ng_sp_top #(.NL(NL), .EN_SFU(ENX), .EN_XLANE(ENX), .EN_FP64(ENX), .EN_VALU_FP(1'b1), .CLEAR_VREGS(CLRV),
              .VALU_BDEPTH(`NG_TB_BDEPTH), .BR_NODRAIN(`NG_TB_BRND), .IC_LINES(`NG_TB_IC_LINES),
              .EN_VALU_F16CVT(`NG_TB_F16CVT), .LSU_K(LSU_K), .MEM_OUT(MEMOUT)) dut (
    .clk(clk), .rst_n(rst_n), .core_reset(core_reset),
    .inv_l1_ext(sp_inv[0] | sp_inv[1]), .inv_i_ext(sp_inv[2]),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .sp_idle(sp_idle),
    .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
    .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic),
    .mmu_req_tag(mmu_req_tag), .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready),
    .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
    .mmu_resp_pte(64'd0),
    .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
    .mem_req_valid(mem_req_valid), .mem_req_ready((MEMOUT > 1) ? sl_ready : mem_req_ready), .mem_req(mem_req),
    .mem_rsp_valid((MEMOUT > 1) ? sl_rsp_valid : mem_rsp_valid), .mem_rsp((MEMOUT > 1) ? sl_rsp : mem_rsp),
    .mem_req_slot(sl_fslot), .mem_rsp_slot(sl_rsp_slot), .mem_rsp_rd(sl_rsp_rd), .mem_rd_slot(mem_rd_slot),
    .mem_rd_data(sl_rd_data), .mem_free(mem_free),
    .dbg_commit_valid(dbg_commit_valid), .dbg_commit_slot(dbg_commit_slot), .dbg_commit_pc(dbg_commit_pc),
    .dbg_commit_w0(dbg_commit_w0), .dbg_commit_exec(dbg_commit_exec), .dbg_commit_wg(dbg_commit_wg),
    .dbg_commit_warp(dbg_commit_warp),
    .dbg_trap_valid(dbg_trap_valid), .dbg_trap_unit(dbg_trap_unit), .dbg_trap_slot(dbg_trap_slot),
    .dbg_trap_pc(dbg_trap_pc), .dbg_trap_cause(dbg_trap_cause), .dbg_trap_info(dbg_trap_info),
    .dbg_trap_wg(dbg_trap_wg), .dbg_trap_warp(dbg_trap_warp),
    .stat_ic_hit(st_ic_hit), .stat_ic_miss(st_ic_miss), .stat_l1_rd(st_l1_rd), .stat_l1_wr(st_l1_wr),
    .stat_instr_mem(st_instr_mem), .stat_groups(st_groups), .stat_perf(st_perf), .stat_scrub(st_scrub)
  );

  // ---------------------------------------------------------------- CP port (WDIST path only)
  logic        d_req = 1'b0, d_we = 1'b0;
  logic [31:0] d_addr = 32'h0, d_wdata = 32'h0, d_rdata;
  logic        d_gnt, d_rvalid, d_err, p_fw_wr, p_irq, p_dbg_v, p_tick;
  logic [11:0] p_fw_waddr, p_fw_raddr;
  logic [31:0] p_fw_wdata, p_irq_set, p_dbg_code;
  logic [3:0]  p_ev_clr;
  logic        p_mreq_valid;
  mem_req_t    p_mreq;
  ng_cp_port #(.SCRUB(`NG_TB_SCRUB)) u_cpport (
    .clk(clk), .rst_n(rst_n), .core_reset_i(core_reset),
    .d_req(d_req), .d_gnt(d_gnt), .d_rvalid(d_rvalid), .d_we(d_we), .d_be(4'hf), .d_addr(d_addr),
    .d_wdata(d_wdata), .d_rdata(d_rdata), .d_err(d_err),
    .fw_wr(p_fw_wr), .fw_waddr(p_fw_waddr), .fw_wdata(p_fw_wdata), .fw_raddr(p_fw_raddr), .fw_rdata(32'h0),
    .wptr_i(32'h0), .ev_pend_i(4'h0), .ev_clr_o(p_ev_clr), .irq_set_o(p_irq_set),
    .m_req_valid(p_mreq_valid), .m_req_ready(1'b0), .m_req(p_mreq),
    .m_rsp_valid(1'b0), .m_rsp_fault(1'b0), .m_rsp_data(64'h0),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(wd_fault), .wd_instr(wd_instr),
    .sp_idle_i(sp_idle), .wd_abort_o(), .fab_busy_i(1'b0), .scrub_set_i(scrub_set), .sp_inv_o(sp_inv), .tick_o(p_tick), .dbg_valid_o(p_dbg_v), .dbg_code_o(p_dbg_code), .busy_o(),
    .cb_valid(), .cb_fill(), .cb_vmid(), .cb_src(), .cb_dst(), .cb_size(), .cb_fillv(), .cb_abort(),
    .cb_done(1'b0), .cb_fault(1'b0)
  );

  // one bus write / read of the private port (drives at negedge)
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

  // working memory: bundle pages first, pages created by RTL writes appended
  logic [31:0] mem [MAXPG * 1024];
  int unsigned pidx [longint unsigned];   // PA page -> index into mem
  int unsigned n_alloc;
  // Page size in words for the clear loop in wr8, deliberately a variable: wr8 is inlined at each of its call sites
  // (128 in the fabric line write) and with --unroll-count 4096 Verilator unrolled the constant 1024-word loop into
  // every one of them (a single 55 MB C++ function, ~15 GB for g++).  A variable bound keeps the loop rolled.
  int pg_words = 1024;
  int unsigned vidx [longint unsigned];   // VA page -> page-map entry

  int errors = 0;
  int notes = 0;
  function automatic void err(input string s);
    errors++;
    if (errors <= 30) $display("TB_ERROR %s", s);
  endfunction

  string bdir;
  string rmode;
  int    n_resets;
  int    reset_at [16];
  // run-time configuration (xsim.bat cannot pass valued plusargs): the run
  // script writes tb_sp_top_run.txt in the simulation directory:
  //   <bundle dir>
  //   run | reset
  //   <n> <commit count 1> ... <commit count n>     (reset mode: abort points)
  task automatic read_runcfg();
    int fd, r;
    fd = $fopen("tb_sp_top_run.txt", "r");
    if (fd == 0) begin $display("TB_ERROR cannot open tb_sp_top_run.txt"); $finish; end
    r = $fscanf(fd, "%s", bdir);
    r = $fscanf(fd, "%s", rmode);
    n_resets = 0;
    r = $fscanf(fd, "%d", n_resets);
    if (n_resets > 16) n_resets = 16;
    for (int i = 0; i < n_resets; i++) r = $fscanf(fd, "%d", reset_at[i]);
    $fclose(fd);
  endtask

  task automatic load_bundle();
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
    $display("NOTE bundle %s: map=%0d pages=%0d trace=%0d warps=%0d status=%0d instr=%0d vmid=%0d",
             bdir, n_map, n_pages, n_trace, n_warps, m_status, m_instr, m_vmid);
  endtask

  task automatic reset_memory();
    pidx.delete();
    for (int i = 0; i < int'(n_pages); i++) begin
      pidx[longint'(mpages[i][35:0])] = i;
      for (int w = 0; w < 1024; w++) mem[i * 1024 + w] = mpre[i * 1024 + w];
    end
    n_alloc = n_pages;
  endtask

  // ---------------------------------------------------------------- memory helpers
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

  // ---------------------------------------------------------------- MMU stub
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
    case (ms)
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
            f = 3'd2;                                    // NonCanonical
          end else if (m_vm == 4'd0) begin
            pa = m_va[47:0];                             // VMID 0: bypass
          end else if (!vidx.exists(longint'(m_va[47:12]))) begin
            f = 3'd1;                                    // NotPresent
          end else begin
            logic [63:0] e0, e1;
            logic        need_w;
            e0 = pmap[2 * vidx[longint'(m_va[47:12])]];
            e1 = pmap[2 * vidx[longint'(m_va[47:12])] + 1];
            need_w = (m_acc == 2'd1) || m_atom;
            if (e0[59]) sp = 1'b1;                       // sparse-null
            else if ((m_acc == 2'd2 && !e0[58]) || (need_w && !e0[57]) || (!need_w && m_acc != 2'd2 && !e0[56]))
              f = 3'd3;                                  // Permission
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

  // ---------------------------------------------------------------- fabric stub
  logic [4:0]  amo_op_s;
  logic        amo64_s;
  logic [63:0] amo_a_s, amo_b_s, amo_cmp_s, amo_r_s;
  logic        amo_hit_s;
  ng_l2_amo_alu u_amo (.op(amo_op_s), .is64(amo64_s), .a(amo_a_s), .b(amo_b_s), .cmp(amo_cmp_s), .r(amo_r_s),
                       .cas_hit(amo_hit_s));
  mst_t   fs = M_IDLE;
  int     flat;
  mreq_t  f_rq;
  logic [63:0] f_old;     // AMO: memory value at the request address (captured with the request)
  int     n_rd = 0, n_wr = 0, n_amo = 0, n_maint = 0;
  // cache maintenance instructions accepted by the LSU in the checked run (inv.l1 / inv.l2 / wb.l2)
  int          co_n [3];
  int unsigned co_inv0 = 0;
  int          co_maint0 = 0;
  assign mem_req_ready = (fs == M_IDLE);
  assign mem_rsp_valid = (fs == M_RSP);
  assign amo_op_s  = f_rq.amo_op;
  assign amo64_s   = f_rq.amo64;
  assign amo_a_s   = f_rq.amo64 ? f_old : {32'd0, f_old[31:0]};
  assign amo_b_s   = f_rq.amo_a;
  assign amo_cmp_s = f_rq.amo_cmp;
  always @(posedge clk) begin
    case (fs)
      M_IDLE: if (mem_req_valid && MEMOUT == 1) begin
        logic [63:0] o;
        for (int b = 0; b < 8; b++) o[8 * b +: 8] = rd8(mem_req.pa + 48'(b));
        f_old <= o;
        f_rq <= mem_req;
        flat <= (MAXLAT > 0) ? $urandom_range(0, MAXLAT) : 0;
        fs   <= M_LAT;
      end
      M_LAT: begin
        if (flat > 0) flat <= flat - 1;
        else begin
          mrsp_t r;
          r = '0;
          case (f_rq.kind)
            M_RD: begin
              n_rd <= n_rd + 1;
              for (int b = 0; b < 128; b++) r.rdata[8 * b +: 8] = rd8({f_rq.pa[47:7], 7'd0} + 48'(b));
            end
            M_WR: begin
              n_wr <= n_wr + 1;
              for (int b = 0; b < 128; b++)
                if (f_rq.be[b]) wr8({f_rq.pa[47:7], 7'd0} + 48'(b), f_rq.wdata[8 * b +: 8]);
            end
            M_AMO: begin
              int nb;
              n_amo <= n_amo + 1;
              nb = f_rq.amo64 ? 8 : 4;
              r.old = amo_a_s;
              for (int b = 0; b < nb; b++) wr8(f_rq.pa + 48'(b), amo_r_s[8 * b +: 8]);
            end
            default: n_maint <= n_maint + 1;
          endcase
          mem_rsp <= r;
          fs <= M_RSP;
        end
      end
      M_RSP: fs <= M_IDLE;
      default: fs <= M_IDLE;
    endcase
  end

  function automatic bit fab_inflight();
    return (MEMOUT > 1) ? sl_busy : (fs != M_IDLE);
  endfunction

  // ---------------------------------------------------------------- commit trace check
  int unsigned tstart [int unsigned];
  int unsigned tcnt   [int unsigned];
  int unsigned tcur   [int unsigned];
  int          n_commit = 0;
  int          n_extra = 0;
  int          n_commit_raw = 0;   // reset-mode trigger, independent of the trace checker
  bit          trace_on = 1'b0;
  longint      t_c_first = -1, t_c_last = 0;

  function automatic bit bdir_is(input string name);
    return (bdir.len() >= name.len()) && (bdir.substr(bdir.len() - name.len(), bdir.len() - 1) == name);
  endfunction

  task automatic reset_trace();
    int unsigned s;
    tstart.delete(); tcnt.delete(); tcur.delete();
    s = 0;
    for (int i = 0; i < int'(n_warps); i++) begin
      tstart[tidx[i][63:32]] = s;
      tcnt[tidx[i][63:32]]   = tidx[i][31:0];
      tcur[tidx[i][63:32]]   = 0;
      s += tidx[i][31:0];
    end
    n_commit = 0;
    n_extra = 0;
    t_c_first = -1;
    t_c_last = 0;
  endtask

  always @(posedge clk) begin
    if (trace_on && dbg_commit_valid) begin
      int unsigned key, ix;
      key = dbg_commit_wg * 32 + {27'd0, dbg_commit_warp};
      n_commit++;
      if (!tcnt.exists(key) || tcur[key] >= tcnt[key]) begin
        n_extra++;
        if (m_status == 0)
          err($sformatf("extra commit wg %0d warp %0d pc %h w0 %h (model trace of this warp has %0d)",
                        dbg_commit_wg, dbg_commit_warp, dbg_commit_pc, dbg_commit_w0,
                        tcnt.exists(key) ? tcnt[key] : 0));
      end else begin
        ix = tstart[key] + tcur[key];
        if (trc[2 * ix][31:0] != dbg_commit_pc || trc[2 * ix + 1][31:0] != dbg_commit_w0 ||
            trc[2 * ix + 1][63:32] != dbg_commit_exec)
          err($sformatf("MISMATCH commit #%0d of wg %0d warp %0d: rtl pc %h w0 %h exec %h, model pc %h w0 %h exec %h",
                        tcur[key], dbg_commit_wg, dbg_commit_warp, dbg_commit_pc, dbg_commit_w0, dbg_commit_exec,
                        trc[2 * ix][31:0], trc[2 * ix + 1][31:0], trc[2 * ix + 1][63:32]));
        tcur[key] = tcur[key] + 1;
      end
    end
  end

  // ---------------------------------------------------------------- CP driver helpers
  // writes the WD_NWORDS-1 request dwords from wdw[] + the GO marker
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

  // wait for wd_done; returns 0 on timeout
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

  // ---------------------------------------------------------------- final checks
  task automatic check_result();
    int mism;
    if (m_status == 0) begin
      if (done_fault) err($sformatf("wd_fault set (rtl trap unit %0d slot %0d pc %h cause %0d info %h)",
                                    dbg_trap_unit, dbg_trap_slot, dbg_trap_pc, dbg_trap_cause, dbg_trap_info));
      foreach (tcnt[k])
        if (tcur[k] != tcnt[k])
          err($sformatf("warp key %0d (wg %0d warp %0d): rtl committed %0d of %0d model instructions",
                        k, k / 32, k % 32, tcur[k], tcnt[k]));
    end else begin
      if (!done_fault) err("model trapped but wd_fault is 0");
      else begin
        if (dbg_trap_wg != meta[6][31:0] || dbg_trap_warp != meta[7][4:0] || dbg_trap_pc != meta[8][31:0] ||
            dbg_trap_cause != meta[9][3:0] || dbg_trap_info != meta[10][31:0])
          err($sformatf("trap record rtl wg %0d warp %0d pc %h cause %0d info %h / model wg %0d warp %0d pc %h cause %0d info %h",
                        dbg_trap_wg, dbg_trap_warp, dbg_trap_pc, dbg_trap_cause, dbg_trap_info,
                        meta[6][31:0], meta[7][4:0], meta[8][31:0], meta[9][3:0], meta[10][31:0]));
        else $display("NOTE trap matches: unit %0d wg %0d warp %0d pc %h cause %0d info %h",
                      dbg_trap_unit, dbg_trap_wg, dbg_trap_warp, dbg_trap_pc, dbg_trap_cause, dbg_trap_info);
      end
    end
    // retired instruction count (a trapping instruction is not counted, in the model and in the RTL).
    // traps are precise per warp only -- with several warps the other warps' progress at the trap
    // differs from the model's quantum-1 schedule, so the total is only compared for single-warp traps.
    if (m_status != 0 && n_warps > 1)
      $display("NOTE WDIST instruction count not compared (trap bundle with %0d warps): rtl %0d model %0d",
               n_warps, done_instr, m_instr);
    else if (done_instr != m_instr)
      err($sformatf("WDIST instruction count rtl %0d model %0d", done_instr, m_instr));
    // memory
    if (m_memcmp != 0) begin
      mism = 0;
      for (int i = 0; i < int'(n_pages); i++)
        for (int w = 0; w < 1024; w++)
          if (mem[i * 1024 + w] !== mpost[i * 1024 + w]) begin
            logic [47:0] pa;
            pa = {mpages[i][35:0], 12'd0} | 48'(4 * w);
            mism++;
            if (mism <= 10)
              err($sformatf("MISMATCH memory pa %h: rtl %h model %h", pa, mem[i * 1024 + w], mpost[i * 1024 + w]));
            // the complete list (capped), independent of the 30-line TB_ERROR budget; tools/archseq/archseq.py
            // diagnose maps it to program / slot / instruction
            if (mism <= 20000) $display("MEMDIFF pa %h rtl %h model %h", pa, mem[i * 1024 + w], mpost[i * 1024 + w]);
          end
      for (int i = int'(n_pages); i < int'(n_alloc); i++)
        for (int w = 0; w < 1024; w++)
          if (mem[i * 1024 + w] != 32'd0) begin
            mism++;
            if (mism <= 10) err($sformatf("MISMATCH memory: rtl wrote nonzero word %h to a page the model left zero (page index %0d word %0d)",
                                          mem[i * 1024 + w], i, w));
          end
      $display("NOTE memory compare: %0d pages (+%0d rtl-created), %0d mismatching words", n_pages,
               n_alloc - n_pages, mism);
    end else begin
      $display("NOTE memory compare skipped (bundle flag)");
    end
    $display("NOTE commits %0d (extra beyond model %0d), wd_instr %0d, model %0d, icache hit %0d miss %0d, mmu %0d, fabric rd %0d wr %0d amo %0d maint %0d",
             n_commit, n_extra, done_instr, m_instr, st_ic_hit, st_ic_miss, n_mmu, n_rd, n_wr, n_amo, n_maint);
    // LSU counters (since the last datapath reset)
    $display("NOTE lsu global memory instructions %0d, line groups %0d, line reads %0d writes %0d",
             st_instr_mem, st_groups, st_l1_rd, st_l1_wr);
    if (st_groups > st_l1_rd + st_l1_wr)
      err($sformatf("LSU line groups %0d > line reads + writes %0d", st_groups, st_l1_rd + st_l1_wr));
    if ((st_groups != 0) && (st_instr_mem == 0)) err("LSU line groups without a global memory instruction");
    if (MEMOUT > 1 && sl_busy) err("fabric slots still allocated after the dispatch (slot leak)");
  endtask

  // ---------------------------------------------------------------- scrub mode (NG_MODE=scrub, bundle k_scrub)
  // Cross-owner residue: v registers, shared memory and L1D contents of the previous dispatch must not be visible to a
  // dispatch whose owner changed (ng_cp_port scrub -> ng_wd), and must stay visible when it did not (the previous behaviour).
  // The residue is planted with back-door writes into the register file / shared array and by changing the global
  // source page behind the L1D's back (what a BAR2 write of a new buffer does).  k_scrub reads shared word l, v20 + v21
  // and global word g of its source page and stores the three values per thread.
  localparam int RF_NB    = 4;                                  // ng_sp_top NG_RF_NBANK
  localparam int RF_NBEAT = 32 / NL;
  localparam int RF_DEPTH = (1024 / RF_NB) * RF_NBEAT;          // ng_sp_top VREGS = 1024
  localparam int RF_USED  = 8 * 24;                             // k_scrub: 8 warps x 24 vregs = warp windows 0 .. 191
  localparam logic [31:0] RES_V = 32'hC3C3_C3C3;                // planted in every register file word
  longint sc_t_start = -1, sc_t_first = -1;                     // wd_start / first commit of the last scrub-mode dispatch
  always @(posedge clk) begin
    if (wd_start) begin sc_t_start <= $time; sc_t_first <= -1; end
    if (dbg_commit_valid && sc_t_first < 0 && sc_t_start >= 0) sc_t_first <= $time;
  end

  function automatic logic [47:0] va2pa(input logic [63:0] va);
    return {pmap[2 * vidx[longint'(va[47:12])] + 1][35:0], va[11:0]};
  endfunction
  function automatic logic [31:0] rd32(input logic [47:0] pa);
    return {rd8(pa + 48'd3), rd8(pa + 48'd2), rd8(pa + 48'd1), rd8(pa)};
  endfunction
  function automatic logic [31:0] shp(input int b, input int r);   // shared residue pattern of bank b, row r
    return 32'hA5A5_0000 | 32'((b << 9) | r);
  endfunction

  task automatic plant_residue();
    logic [NL*32-1:0] d;
    d = {NL{RES_V}};
    for (int e = 0; e < RF_DEPTH; e++) begin
      dut.u_rf.g_bank[0].mem[e] = d; dut.u_rf.g_bank[1].mem[e] = d;
      dut.u_rf.g_bank[2].mem[e] = d; dut.u_rf.g_bank[3].mem[e] = d;
    end
    for (int b = 0; b < 32; b++) for (int r = 0; r < 512; r++) dut.u_lsu.g_core.u_lsu.u_core.u_shmem.mem[b][r] = shp(b, r);
  endtask

  // number of nonzero register file words of warp registers [lo, 1024)
  function automatic int rf_nonzero_from(input int lo);
    int n;
    logic [NL*32-1:0] d;
    n = 0;
    for (int p = lo; p < 1024; p++)
      for (int bt = 0; bt < RF_NBEAT; bt++) begin
        case (p % RF_NB)
          0: d = dut.u_rf.g_bank[0].mem[(p / RF_NB) * RF_NBEAT + bt];
          1: d = dut.u_rf.g_bank[1].mem[(p / RF_NB) * RF_NBEAT + bt];
          2: d = dut.u_rf.g_bank[2].mem[(p / RF_NB) * RF_NBEAT + bt];
          default: d = dut.u_rf.g_bank[3].mem[(p / RF_NB) * RF_NBEAT + bt];
        endcase
        if (d !== '0) n++;
      end
    return n;
  endfunction

  task automatic xor_src(input logic [63:0] src_va, input logic [31:0] m);   // new data behind the L1D's back
    logic [47:0] pa;
    logic [31:0] w;
    pa = va2pa(src_va);
    for (int i = 0; i < 256; i++) begin
      w = rd32(pa + 48'(4 * i)) ^ m;
      for (int b = 0; b < 4; b++) wr8(pa + 48'(4 * i + b), w[8 * b +: 8]);
    end
  endtask

  task automatic run_scrub_test();
    logic [63:0] src_va, out_va;
    logic [47:0] src_pa, out_pa;
    logic [31:0] orig [256];
    logic [31:0] cur [256];
    logic [31:0] o0, o1, o2;
    logic [31:0] cyc0, inv0, cpl10, cpi0, n0, cyc1;
    int n_before, n_stale, n_fresh, n_shres, n_vres, nz, rows_bad, rows_kept;
    bit ok;
    longint lat_noscrub, lat_scrub;
    if (!bdir_is("k_scrub")) begin err("NG_MODE=scrub needs the k_scrub bundle"); return; end
    src_va = {wdw[26], wdw[25]}; out_va = {wdw[30], wdw[29]};
    src_pa = va2pa(src_va); out_pa = va2pa(out_va);
    for (int i = 0; i < 256; i++) orig[i] = rd32(src_pa + 48'(4 * i));
    cur = orig;
    $display("NOTE S5b scrub test: src pa %h out pa %h, owner %0d, NL=%0d", src_pa, out_pa, wdw[0][3:0], NL);

    // ---- A. same owner, no scrub: the previous dispatch's residue is visible (the previous behaviour) ----
    n0 = st_scrub[31:0]; cyc0 = st_scrub[63:32]; inv0 = st_scrub[95:64];
    plant_residue();
    xor_src(src_va, 32'h0F0F_F0F0);
    for (int i = 0; i < 256; i++) cur[i] = cur[i] ^ 32'h0F0F_F0F0;           // what the fabric holds now (L1D: orig)
    n_before = done_seen;
    start_dispatch();
    wait_done(n_before, ok);
    repeat (2) @(negedge clk);
    wait_idle(200);
    lat_noscrub = (sc_t_first - sc_t_start) / 10;
    if (st_scrub[31:0] != n0) err($sformatf("A: scrub counter moved (%0d -> %0d) without an owner change", n0, st_scrub[31:0]));
    if (st_scrub[63:32] != cyc0) err("A: scrub cycles counted without a scrub");
    if (st_scrub[95:64] != inv0) err("A: external L1D invalidate without a request");
    n_stale = 0; n_shres = 0; n_vres = 0;
    for (int g = 0; g < 256; g++) begin
      o0 = rd32(out_pa + 48'(12 * g)); o1 = rd32(out_pa + 48'(12 * g + 4)); o2 = rd32(out_pa + 48'(12 * g + 8));
      if (o0 != 32'd0) n_shres++;
      if (o1 == RES_V + RES_V) n_vres++;
      if (o2 == orig[g]) n_stale++;
    end
    $display("NOTE S5b A (no scrub): shared residue %0d / 256 threads, v residue %0d / 256, stale L1D words %0d / 256, wd_start -> first commit %0d clk",
             n_shres, n_vres, n_stale, lat_noscrub);
    if (n_shres != 256) err($sformatf("A: shared residue visible to %0d of 256 threads (expected all: no scrub)", n_shres));
    if (n_vres != 256) err($sformatf("A: v register residue visible to %0d of 256 threads (expected all: no scrub)", n_vres));
    if (n_stale == 0) err("A: no stale L1D word seen (the source page was refetched without an invalidate)");

    // ---- B. owner change (MMU CTX_CTRL write of the VMID): scrub ----
    plant_residue();
    xor_src(src_va, 32'h00FF_FF00);
    for (int i = 0; i < 256; i++) cur[i] = cur[i] ^ 32'h00FF_FF00;
    n0 = st_scrub[31:0]; cyc0 = st_scrub[63:32]; inv0 = st_scrub[95:64];
    @(negedge clk); scrub_set[wdw[0][3:0]] = 1'b1;
    @(negedge clk); scrub_set = '0;
    repeat (2) @(negedge clk);
    n_before = done_seen;
    start_dispatch();
    wait_done(n_before, ok);
    repeat (2) @(negedge clk);
    wait_idle(200);
    lat_scrub = (sc_t_first - sc_t_start) / 10;
    cyc1 = st_scrub[63:32] - cyc0;
    if (st_scrub[31:0] != n0 + 1) err($sformatf("B: scrub counter %0d -> %0d (expected +1)", n0, st_scrub[31:0]));
    if (st_scrub[95:64] != inv0 + 1) err($sformatf("B: external L1D invalidates %0d -> %0d (expected +1)", inv0, st_scrub[95:64]));
    n_stale = 0; n_fresh = 0; n_shres = 0; n_vres = 0;
    for (int g = 0; g < 256; g++) begin
      o0 = rd32(out_pa + 48'(12 * g)); o1 = rd32(out_pa + 48'(12 * g + 4)); o2 = rd32(out_pa + 48'(12 * g + 8));
      if (o0 != 32'd0) n_shres++;
      if (o1 != 32'd0) n_vres++;
      if (o2 == cur[g]) n_fresh++;
      else n_stale++;
    end
    $display("NOTE S5b B (owner change): shared residue %0d, v residue %0d, stale L1D words %0d, fresh %0d (of 256); scrub step %0d clk, wd_start -> first commit %0d clk (no-scrub run %0d)",
             n_shres, n_vres, n_stale, n_fresh, cyc1, lat_scrub, lat_noscrub);
    if (n_shres != 0) err($sformatf("B: shared residue visible to %0d threads after the scrub", n_shres));
    if (n_vres != 0) err($sformatf("B: v register residue visible to %0d threads after the scrub", n_vres));
    if (n_stale != 0) err($sformatf("B: %0d stale L1D words after the scrub", n_stale));
    // everything outside the warp windows of the dispatch is zero (the whole register file was cleared, not just the windows used)
    nz = rf_nonzero_from(RF_USED);
    if (nz != 0) err($sformatf("B: %0d nonzero register file words beyond the warp windows after the scrub", nz));
    // shared: 4 workgroups x 1 KiB windows used; the scrub cleared min(NWG, 64 KiB / 1 KiB) = 8 windows = rows 0..63; the
    // kernel wrote rows 8 w + 0..1 (its 256 B of residue per workgroup); rows 64.. keep the planted pattern
    rows_bad = 0; rows_kept = 0;
    for (int r = 0; r < 512; r++)
      for (int b = 0; b < 32; b++) begin
        logic [31:0] v;
        v = dut.u_lsu.g_core.u_lsu.u_core.u_shmem.mem[b][r];
        if (r < 64) begin
          if (!(r % 8 < 2 && r < 32) && v != 32'd0) rows_bad++;
        end else begin
          if (v == shp(b, r)) rows_kept++;
        end
      end
    if (rows_bad != 0) err($sformatf("B: %0d nonzero shared words in the cleared windows outside the kernel's own writes", rows_bad));
    if (rows_kept != 448 * 32) err($sformatf("B: %0d of %0d shared words beyond the cleared windows lost their pattern", 448 * 32 - rows_kept, 448 * 32));
    if (cyc1 < 32'(1024 * RF_NBEAT)) err($sformatf("B: scrub step %0d clk is shorter than the register file write (%0d)", cyc1, 1024 * RF_NBEAT));

    // ---- D. CP CACHE(INV_L1) packet path (ng_cp_port P_SP_INV): L1D invalidated without a scrub ----
    plant_residue();
    xor_src(src_va, 32'hF000_000F);
    for (int i = 0; i < 256; i++) cur[i] = cur[i] ^ 32'hF000_000F;
    n0 = st_scrub[31:0]; inv0 = st_scrub[95:64]; cpl10 = st_scrub[127:96]; cpi0 = st_scrub[159:128];
    port_wr(P_SP_INV, 32'h1);              // INV_L1
    port_wr(P_SP_INV, 32'h4);              // INV_I (counted; the I$ is invalidated at every dispatch start anyway)
    port_wr(P_SP_INV, 32'h2);              // INV_K (the L1D too)
    repeat (4) @(negedge clk);
    n_before = done_seen;
    start_dispatch();
    wait_done(n_before, ok);
    repeat (2) @(negedge clk);
    wait_idle(200);
    if (st_scrub[31:0] != n0) err("D: scrub without an owner change");
    if (st_scrub[127:96] != cpl10 + 2) err($sformatf("D: CP INV_L1/INV_K pulses %0d -> %0d (expected +2)", cpl10, st_scrub[127:96]));
    if (st_scrub[159:128] != cpi0 + 1) err($sformatf("D: CP INV_I pulses %0d -> %0d (expected +1)", cpi0, st_scrub[159:128]));
    if (st_scrub[95:64] < inv0 + 1) err($sformatf("D: L1D external invalidates %0d -> %0d", inv0, st_scrub[95:64]));
    n_stale = 0; n_fresh = 0; n_shres = 0; n_vres = 0;
    for (int g = 0; g < 256; g++) begin
      o0 = rd32(out_pa + 48'(12 * g)); o1 = rd32(out_pa + 48'(12 * g + 4)); o2 = rd32(out_pa + 48'(12 * g + 8));
      if (o0 != 32'd0) n_shres++;
      if (o1 == RES_V + RES_V) n_vres++;
      if (o2 == cur[g]) n_fresh++; else n_stale++;
    end
    $display("NOTE S5b D (CACHE INV_L1 packet, no scrub): fresh L1D words %0d, stale %0d, shared residue %0d, v residue %0d (of 256); CP INV_L1 +%0d INV_I +%0d, L1D invalidates +%0d",
             n_fresh, n_stale, n_shres, n_vres, st_scrub[127:96] - cpl10, st_scrub[159:128] - cpi0, st_scrub[95:64] - inv0);
    if (n_stale != 0) err($sformatf("D: %0d stale L1D words after CACHE(INV_L1)", n_stale));
    if (n_shres != 256 || n_vres != 256) err("D: the CACHE packet must not scrub v registers / shared memory");

    // ---- E. a scrubbing dispatch against the model (trace, final memory), after residue everywhere ----
    reset_memory();
    plant_residue();
    @(negedge clk); scrub_set[wdw[0][3:0]] = 1'b1;
    @(negedge clk); scrub_set = '0;
    repeat (2) @(negedge clk);
    reset_trace();
    n0 = st_scrub[31:0];
    trace_on = 1'b1;
    n_before = done_seen;
    start_dispatch();
    wait_done(n_before, ok);
    repeat (2) @(negedge clk);
    if (ok) begin wait_idle(200); check_result(); end
    trace_on = 1'b0;
    if (st_scrub[31:0] != n0 + 1) err("E: the dispatch did not scrub");
    $display("NOTE S5b E (scrub dispatch vs model): ok=%0d, scrub counter %0d -> %0d", ok, n0, st_scrub[31:0]);
  endtask

  // --------------------------------------------------------------
  initial begin
    bit ok;
    int n_before;
    logic [31:0] st0;
    read_runcfg();
    load_bundle();
    reset_memory();
    reset_trace();
    repeat (5) @(negedge clk);
    rst_n = 1'b1;
    repeat (5) @(negedge clk);

    // ---- configuration fault: 16 warps x 128 registers cannot fit the register file (no hang, wd_done + wd_fault) ----
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

    // ---- aborted runs (reset mode): core_reset at each abort point ----
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
      // then (up to 400 cycles) until a fabric (even abort index) or MMU (odd)
      // transaction is in flight, so the SP's response-drop path is exercised
      c = 0;
      while (c < 400 && done_seen == n_before && !((ri % 2 == 0) ? fab_inflight() : (ms != M_IDLE))) begin
        @(negedge clk); c++;
      end
      if (sp_idle) err("sp_idle is 1 while the dispatch runs");
      $display("NOTE core_reset after %0d commits (in flight: mmu %0d fabric %0d)", n_commit_raw - base,
               ms != M_IDLE, fab_inflight());
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
      // an aborted dispatch produces no wd_done, and the CP port's WDIST busy flag is cleared by core_reset
      if (done_seen != n_before)
        err($sformatf("aborted dispatch: %0d wd_done pulses (expected none)", done_seen - n_before));
      port_rd(P_WDIST_STATUS, st32);
      if (st32[0] !== 1'b0) err($sformatf("aborted dispatch: WDIST_STATUS %h, busy still set after core_reset", st32));
      // requests accepted before the reset still complete in the slot model (writes land, as on the
      // board); wait for them before the memory image is restored
      c = 0;
      while (MEMOUT > 1 && sl_busy && c < 200000) begin @(negedge clk); c++; end
      if (MEMOUT > 1 && sl_busy) err("fabric slots still allocated long after the core reset");
      reset_memory();
    end

    // ---- checked run ----
    reset_trace();
    co_n = '{0, 0, 0};
    co_inv0 = dut.ls_st[7];
    co_maint0 = n_maint;
    trace_on = 1'b1;
    n_before = done_seen;
    start_dispatch();
    wait_done(n_before, ok);
    repeat (2) @(negedge clk);
    if (ok) begin
      wait_idle(200);
      check_result();
    end
    trace_on = 1'b0;
    // INV-FX / per-warp in-flight invariants (simulation-only checks in ng_sp_top)
    $display("NOTE inv_err=%0d lsu_sfu_accepted_under_fixed_op=%0d overlap_cycles=%0d",
             dut.sim_inv_err, dut.sim_acc_fx, dut.sim_ovl_cyc);
    $display("NOTE commits=%0d first_to_last_commit_cycles=%0d valu_busy_cycles=%0d valu2_cycles=%0d",
             n_commit, (t_c_last - t_c_first) / 10, dut.sim_valu_busy, dut.sim_valu2);
    $display("NOTE VALU: chk_trap_deferred=%0d xfer_wait=%0d excl_issue_blocked=%0d excl_instr=%0d late_traps=%0d prep_drop_deferred=%0d",
             dut.u_valu.sim_chk_defer, dut.u_valu.sim_xfer_wait, dut.u_valu.sim_excl_blk, dut.u_valu.sim_excl_n,
             dut.u_valu.sim_late_trap, dut.u_valu.sim_prep_defer);
    $display("NOTE VALU (BDEPTH=%0d): ctx2_cycles=%0d ctx3_cycles=%0d valu3_cycles=%0d queued_f32=%0d x2_stall=%0d gsafe_hold=%0d valu_err=%0d",
             `NG_TB_BDEPTH, dut.u_valu.sim_ctx2, dut.u_valu.sim_ctx3, dut.sim_valu3, dut.u_valu.sim_q_push,
             dut.u_valu.sim_x2_stall, dut.u_valu.sim_gs_blk, dut.u_valu.sim_err);
    perf_report();
    begin   // SP performance counters (whole run since the last datapath reset) and BR_NODRAIN coverage
      logic [31:0] pc [8];
      for (int i = 0; i < 8; i++) pc[i] = st_perf[32 * i +: 32];
      $display("NOTE counters (BR_NODRAIN=%0d): active=%0d valu_back_busy=%0d br_wait_drain=%0d front_starved=%0d valu_issued=%0d br_issued=%0d ib_wait_own=%0d salu_busy=%0d",
               `NG_TB_BRND, pc[0], pc[1], pc[2], pc[3], pc[4], pc[5], pc[6], pc[7]);
      $display("NOTE overlap: br_valu_cycles=%0d br_accepted_under_valu=%0d valu_accepted_under_br=%0d salu_ur_held=%0d",
               dut.sim_br_ovl, dut.sim_br_acc_v, dut.sim_va_acc_br, dut.sim_ur_hold);
      if (pc[1] + pc[3] > pc[0]) err("perf counters: VALU_BACK_BUSY + FRONT_STARVED > SP_ACTIVE");
      if (pc[2] > pc[0] || pc[6] > pc[0] || pc[7] > pc[0]) err("perf counters: a stall count exceeds SP_ACTIVE");
      if (pc[5] > pc[7]) err("perf counters: BR_ISSUED > SALU_BUSY");
      if (m_status == 0 && pc[0] == 0) err("perf counters: SP_ACTIVE did not count");
      if (n_valu_acc_tb != longint'(pc[4]))
        err($sformatf("perf counters: VALU_ISSUED %0d, TB counted %0d VALU accepts", pc[4], n_valu_acc_tb));
      if (`NG_TB_BRND == 0 && (dut.sim_br_ovl != 0 || dut.sim_ur_hold != 0))
        err("BR_NODRAIN=0: a branch overlapped the VALU");
      if (`NG_TB_BRND != 0 && bdir_is("k_brnd") && (dut.sim_br_acc_v == 0 || dut.sim_va_acc_br == 0))
        err("k_brnd: no pure branch accepted under a busy VALU / no VALU accept under a branch");
      // k_trap_brur is timed for the default configuration (16 lanes, latency bound 3, BDEPTH 3, no CLEAR_VREGS):
      // a branch's uniform read must have waited behind the trapping .vu instruction stuck in the VALU's CHK
      if (`NG_TB_BRND != 0 && bdir_is("k_trap_brur")) begin
        if (dut.sim_ur_hold != 0) $display("NOTE k_trap_brur: SALU uniform read held %0d cycles behind the VALU", dut.sim_ur_hold);
        else if (NL == 16 && MAXLAT == 3 && `NG_TB_BDEPTH == 3 && !CLRV)
          err("k_trap_brur: the branch's uniform read never yielded to the VALU (sa_ur_gnt not exercised)");
        else $display("NOTE k_trap_brur: sa_ur_gnt not exercised in this configuration");
      end
    end
    if (dut.sim_inv_err != 0) err($sformatf("ng_sp_top invariant violations: %0d", dut.sim_inv_err));
    // LSU ring statistics (ng_lsu_kn, LSU_K >= 2) and the fabric slot model
    $display("NOTE LSU (K=%0d MEMOUT=%0d): lsu_in_flight_max=%0d ring_max=%0d iof_max=%0d ring_occ_avg=%0.2f fe_busy=%0d ralloc_wait=%0d iss_stall=%0d sync_wait=%0d twait=%0d be_wait=%0d late_err=%0d err=%0d",
             LSU_K, MEMOUT, dut.sim_lsu_max, dut.u_lsu.sim_ring_max, dut.u_lsu.sim_iof_max,
             (dut.u_lsu.sim_cyc != 0) ? real'(dut.u_lsu.sim_ring_occ) / real'(dut.u_lsu.sim_cyc) : 0.0,
             dut.u_lsu.sim_fe_busy, dut.u_lsu.sim_ralloc, dut.u_lsu.sim_iss_stall, dut.u_lsu.sim_sync,
             dut.u_lsu.sim_twait, dut.u_lsu.sim_be_wait, dut.u_lsu.sim_late_err, dut.u_lsu.sim_err);
    if (MEMOUT > 1)
      $display("NOTE fabric slot model: latency %0d..%0d orphan=%0d accepted=%0d max_out=%0d orphans=%0d dropped_free=%0d conflict_stall=%0d fence_stall=%0d",
               `NG_TB_FLAT_LO, `NG_TB_FLAT_HI, `NG_TB_ORPHAN, sl_n_acc, sl_max_out, sl_n_orph, sl_n_late_free,
               sl_n_cstall, sl_n_fstall);
    if (dut.u_lsu.sim_err != 0) err($sformatf("ng_lsu_kn invariant violations: %0d", dut.u_lsu.sim_err));
    if (LSU_K > 1 && MEMOUT > 1 && bdir_is("k_stream") && dut.u_lsu.sim_ring_max < 2)
      err("k_stream: the LSU ring never held two instructions (no cross-instruction concurrency)");
    if (dut.u_valu.sim_err != 0) err($sformatf("ng_valu invariant violations: %0d", dut.u_valu.sim_err));
    // (integer-only kernels such as k_mw2/3 drain each instruction before the next one's first beat
    // is ready, so the multi-instruction back half is checked on an f32 kernel)
    if (`NG_TB_BDEPTH > 1 && bdir_is("k_fsaxpy") && dut.u_valu.sim_ctx2 == 0)
      err("k_fsaxpy: the VALU back half never held two instructions");
    if (bdir_is("k_trap_mwchk") && dut.u_valu.sim_chk_defer == 0)
      err("k_trap_mwchk: no CHK-stage trap was deferred behind a busy back half");
    if (bdir_is("k_trap_mwchk") && dut.u_valu.sim_prep_defer == 0)
      err("k_trap_mwchk: no movrel drop was deferred behind a busy back half");
    if ((bdir_is("k_trap_mwfp") || bdir_is("k_trap_mwrr")) && dut.u_valu.sim_late_trap == 0)
      err("late-trap kernel: no late trap taken in the VALU back half");
    if ((bdir_is("k_mw2") || bdir_is("k_mw3")) && dut.sim_valu2 == 0)
      err("k_mw2/3: the VALU never held two instructions");
    if (bdir.len() >= 4 && bdir.substr(bdir.len() - 4, bdir.len() - 1) == "k_mw") begin
      if (dut.sim_acc_fx == 0) err("k_mw: no LSU/SFU accept under a running fixed-group op (no cross-warp overlap)");
    end
    // every accepted cache.inv.l1 reaches the L1D (whole-cache invalidate), every cache.inv.l2 /
    // cache.wb.l2 reaches the fabric as one maintenance request (the only maintenance source on this port)
    $display("NOTE cache ops accepted by the LSU: inv.l1 %0d inv.l2 %0d wb.l2 %0d; L1D invalidates %0d, fabric maint %0d",
             co_n[0], co_n[1], co_n[2], dut.ls_st[7] - co_inv0, n_maint - co_maint0);
    if (ok) begin
      if (dut.ls_st[7] - co_inv0 != co_n[0])
        err($sformatf("cache.inv.l1: %0d accepted, %0d L1D invalidates", co_n[0], dut.ls_st[7] - co_inv0));
      if (n_maint - co_maint0 != co_n[1] + co_n[2])
        err($sformatf("cache.inv.l2 + cache.wb.l2: %0d accepted, %0d fabric maintenance requests",
                      co_n[1] + co_n[2], n_maint - co_maint0));
    end
    if (bdir_is("k_cacheop") && (co_n[0] == 0 || co_n[1] == 0 || co_n[2] == 0))
      err("k_cacheop: a cache op kind never reached the LSU");
    if (rmode == "scrub") run_scrub_test();
    if (errors == 0) $display("NG_SP_TOP_PASS %s", bdir);
    else             $display("NG_SP_TOP_FAIL %s errors=%0d", bdir, errors);
    $finish;
  end

  // raw commit counter
  always @(posedge clk) if (dbg_commit_valid) n_commit_raw <= n_commit_raw + 1;

  // VALU accepts since the last datapath reset (reference for the VALU_ISSUED counter)
  always @(posedge clk) begin
    if (!dut.crst_n) n_valu_acc_tb <= 0;
    else if (dut.va_push) n_valu_acc_tb <= n_valu_acc_tb + 1;
  end

  // cache maintenance instructions accepted by the LSU in the checked run (inv.l1 / inv.l2 / wb.l2)
  always @(posedge clk)
    if (trace_on && dut.u_iss_valid[ng_sp_pkg::UNIT_LSU] && dut.u_iss_ready[ng_sp_pkg::UNIT_LSU])
      case (dut.is_insn[31:0])
        32'h0000_04ce: co_n[0]++;
        32'h0000_054e: co_n[1]++;
        32'h0000_05ce: co_n[2]++;
        default: ;
      endcase

  // dispatch length (first to last commit, cycles); reset by reset_trace
  always @(posedge clk) if (trace_on && dbg_commit_valid) begin
    if (t_c_first < 0) t_c_first = $time;
    t_c_last = $time;
  end

  // ---------------------------------------------------------------- cycle accounting (simulation only, probes only)
  // Window = first to last commit of the checked run.  Two independent per-cycle classifications:
  //   VALU view: what the VALU back half (bst) and front half (st/cs) do;
  //   front view: what the scheduler / issue buffer does (one bucket per cycle).
  // Printed as "PERF ..." lines after the checked run (tb_sp_top_run.txt unchanged).
  localparam int PB = 16, PF = 20;
  longint pv_cnt [PB];
  longint pf_cnt [PF];
  longint p_cyc = 0, p_valu_acc = 0, p_salu_acc = 0, p_lsu_acc = 0, p_other_acc = 0, p_commit = 0;
  longint p_resident_sum = 0, p_inflight_sum = 0, p_ready_sum = 0, p_brwait_sum = 0, p_pend_sum = 0;
  longint p_issue_gap_max = 0, p_last_valu_acc = -1, p_gap_sum = 0, p_gap_n = 0;
  longint p_wr_busy_cyc = 0, p_x1_cyc = 0, p_fpeng_cyc = 0, p_launch_cyc = 0, p_bkconf = 0;
  logic p_win = 1'b0;
  longint p_start_cyc = 0, p_wd_t0 = -1;
  logic p_end = 1'b0;   // wd_done seen in the checked run
  always @(posedge clk) begin
    if (!trace_on) p_end <= 1'b0;
    else if (wd_done) p_end <= 1'b1;
  end
  string pv_name [PB] = '{"back_issue_beats", "back_drain_tail", "back_fw_write", "back_idle_front_idle_ibempty",
                          "back_idle_front_idle_ib_wait", "back_idle_front_setup(CHK/URW/PREP)", "back_idle_front_collect",
                          "back_idle_front_other", "backbusy_front_idle", "backbusy_front_setup", "backbusy_front_collect_wait",
                          "backbusy_front_collect_launching", "unused12", "unused13", "unused14", "unused15"};
  string pf_name [PF] = '{"accept_valu", "accept_salu_br", "accept_lsu_other", "ib_wait_own_inflight",
                          "ib_branch_wait_valu_drain", "ib_valu_wait_salu", "ib_valu_wait_front_busy", "ib_wait_other",
                          "ibempty_fsm_SEL_nogrant", "ibempty_fsm_SEL_grant", "ibempty_fsm_FETCH_wait", "ibempty_fsm_FETCH_hit",
                          "ibempty_fsm_PD_writes_ib", "ibempty_fsm_PD_local_insn", "ibempty_fsm_PD_hold", "ibempty_PD_other",
                          "unused16", "unused17", "unused18", "unused19"};
  initial begin
    for (int i = 0; i < PB; i++) pv_cnt[i] = 0;
    for (int i = 0; i < PF; i++) pf_cnt[i] = 0;
  end
  always @(posedge clk) begin
    p_win = trace_on && (t_c_first >= 0) && !p_end;
    if (p_win) begin
      int vb, fb, nres, ninf, nrdy, nbr, npend;
      logic acc, v_front_idle;
      p_cyc++;
      acc = dut.is_ready;
      // ---- VALU view
      v_front_idle = (dut.u_valu.st == 3'd0);
      if (!dut.u_valu.b_idle) begin
        if (dut.u_valu.fw_on) vb = 2;
        else if (!dut.u_valu.k_all[dut.u_valu.xk]) vb = 0; else vb = 1;
      end
      else vb = -1;
      if (vb >= 0) begin
        pv_cnt[vb]++;
        // front half while the back half is busy
        if (v_front_idle) pv_cnt[8]++;
        else if (dut.u_valu.st != 3'd4) pv_cnt[9]++;
        else if (dut.u_valu.cs == 3'd2 && dut.u_valu.pend == 7'd0 && !dut.u_valu.launch) pv_cnt[10]++;
        else pv_cnt[11]++;
      end else begin
        if (v_front_idle) begin
          if (!dut.u_sched.ib_v) pv_cnt[3]++; else pv_cnt[4]++;
        end else if (dut.u_valu.st != 3'd4) pv_cnt[5]++;
        else pv_cnt[6]++;
      end
      if (dut.u_valu.x1_v) p_x1_cyc++;
      if (dut.u_valu.launch) p_launch_cyc++;
      // ---- front view
      if (acc) begin
        if (dut.tgt == ng_sp_pkg::UNIT_VALU) begin
          pf_cnt[0]++; p_valu_acc++;
          if (p_last_valu_acc >= 0) begin p_gap_sum += p_cyc - p_last_valu_acc; p_gap_n++; end
          p_last_valu_acc = p_cyc;
        end else if (dut.tgt == ng_sp_pkg::UNIT_SALU) begin pf_cnt[1]++; p_salu_acc++; end
        else begin pf_cnt[2]++; p_other_acc++; end
      end else if (dut.u_sched.ib_v) begin
        if (dut.u_sched.infl_ib_q) pf_cnt[3]++;
        else if (dut.tgt == ng_sp_pkg::UNIT_SALU) pf_cnt[4]++;
        else if (dut.tgt == ng_sp_pkg::UNIT_VALU && dut.fx_oth_busy) pf_cnt[5]++;
        else if (dut.tgt == ng_sp_pkg::UNIT_VALU) pf_cnt[6]++;
        else pf_cnt[7]++;
      end else begin
        case (dut.u_sched.fsm)
          2'd0: if (dut.u_sched.grant_valid) pf_cnt[9]++; else pf_cnt[8]++;
          2'd1: if (dut.u_sched.hit_buf || dut.u_sched.hit_rsp) pf_cnt[11]++; else pf_cnt[10]++;
          default: begin   // S_PD
            if (dut.u_sched.ib_wr) pf_cnt[12]++;
            else if (dut.u_sched.hold && !dut.u_sched.pd_pipe) pf_cnt[14]++;
            else if (!dut.u_sched.pd_pipe) pf_cnt[13]++;
            else pf_cnt[15]++;
          end
        endcase
      end
      // --
      nres = 0; ninf = 0; nrdy = 0; nbr = 0; npend = 0;
      for (int s = 0; s < 16; s++) begin
        if (dut.u_sched.state[s] != ng_sp_sched_pkg::W_EMPTY) nres++;
        if (dut.u_sched.state[s] == ng_sp_sched_pkg::W_READY) nrdy++;
        if (dut.u_sched.state[s] == ng_sp_sched_pkg::W_BRWAIT) nbr++;
        if (dut.u_sched.state[s] == ng_sp_sched_pkg::W_ISSUE_PEND) npend++;
        if (dut.u_sched.inflight[s]) ninf++;
      end
      p_resident_sum += nres; p_inflight_sum += ninf; p_ready_sum += nrdy; p_brwait_sum += nbr; p_pend_sum += npend;
      if (dbg_commit_valid) p_commit++;
    end
  end

  task automatic perf_report();
    longint tot;
    $display("PERF bundle %s cycles=%0d valu_instr=%0d salu_br_instr=%0d other_instr=%0d commits=%0d cyc_per_valu_instr=%0.2f mean_gap_between_valu_accepts=%0.2f",
             bdir, p_cyc, p_valu_acc, p_salu_acc, p_other_acc, p_commit,
             (p_valu_acc != 0) ? real'(p_cyc) / real'(p_valu_acc) : 0.0,
             (p_gap_n != 0) ? real'(p_gap_sum) / real'(p_gap_n) : 0.0);
    $display("PERF occupancy avg resident=%0.2f ready=%0.2f issue_pend=%0.2f brwait=%0.2f inflight=%0.2f",
             real'(p_resident_sum) / real'(p_cyc == 0 ? 1 : p_cyc), real'(p_ready_sum) / real'(p_cyc == 0 ? 1 : p_cyc),
             real'(p_pend_sum) / real'(p_cyc == 0 ? 1 : p_cyc), real'(p_brwait_sum) / real'(p_cyc == 0 ? 1 : p_cyc),
             real'(p_inflight_sum) / real'(p_cyc == 0 ? 1 : p_cyc));
    $display("PERF x1_cycles=%0d launch_cycles=%0d", p_x1_cyc, p_launch_cyc);
    tot = p_cyc == 0 ? 1 : p_cyc;
    for (int i = 0; i < PB; i++) if (pv_cnt[i] != 0)
      $display("PERF valu  %-40s %9d %6.2f%%", pv_name[i], pv_cnt[i], 100.0 * real'(pv_cnt[i]) / real'(tot));
    for (int i = 0; i < PF; i++) if (pf_cnt[i] != 0)
      $display("PERF front %-40s %9d %6.2f%%", pf_name[i], pf_cnt[i], 100.0 * real'(pf_cnt[i]) / real'(tot));
    $display("PERF startup_cycles_wdstart_to_first_commit=%0d", p_start_cyc);
  endtask

  always @(posedge clk) begin
    if (wd_start && p_wd_t0 < 0) p_wd_t0 = $time;
    if (dbg_commit_valid && p_start_cyc == 0 && p_wd_t0 >= 0) p_start_cyc = ($time - p_wd_t0) / 10;
  end

  // ---------------------------------------------------------------- tagged multi-outstanding fabric model
  // MEMOUT > 1: the slot protocol of ng_rsp_fab_ts  as a behavioural model --
  //   MEMOUT slots, the slot is the tag (lowest free slot, mem_req_slot valid with ready);
  //   same 128 B line with either side a write -> the later request waits (RAW / WAR / WAW, answered read slots
  //   waiting for mem_free and orphans count); M_AMO / cache maintenance are fences (accepted with no slot allocated,
  //   nothing else accepted until they completed);
  //   an M_WR is accepted after its request was held 4 clocks (fab_ts copies it into wbuf), random 1/16 ready stalls;
  //   latency NG_TB_FLAT_LO..HI clocks (+ a 2 % long tail of 4 * HI + 50), two targets (PA[47]: host, else C3) in order
  //   per target, out of order across targets, one answer per clock; the memory operation happens at the answer;
  //   M_RD keeps its slot (line in s_line) until mem_free, the other kinds are freed by the answer;
  //   NG_TB_ORPHAN = 1: sp_rst = !crst_n as rm_sp (answered reads are freed, requests in flight become orphans whose
  //   answers are not delivered); 0: answers of requests from before a reset are delivered (the mport drops them).
  // Every mem_free must name an answered read slot (outside sp_rst); no slot may stay allocated after the dispatch.
  if (MEMOUT > 1) begin : g_fslot
    localparam int NS = MEMOUT;
    localparam int LO = `NG_TB_FLAT_LO;
    localparam int HI = `NG_TB_FLAT_HI;
    bit [NS-1:0]   s_al = '0, s_ans = '0, s_orph = '0;
    mreq_t         s_rq   [NS];
    longint        s_due  [NS];
    logic [1023:0] s_line [NS];
    logic [63:0]   s_old  [NS];
    longint        tl [2] = '{0, 0};
    longint        cyc = 0;
    int            wstg = 0;
    bit            stall = 1'b0;
    int            amo_s = 0;
    logic [63:0]   amo2_r;
    logic          amo2_hit;
    wire           sp_rst = (`NG_TB_ORPHAN != 0) && !dut.crst_n;
    logic          k_wr, k_fence, conflict, fwait, any_free, can;
    logic [TSW-1:0] fsl;
    initial for (int i = 0; i < NS; i++) begin s_rq[i] = '0; s_due[i] = 0; s_line[i] = '0; s_old[i] = '0; end
    ng_l2_amo_alu u_amo2 (.op(s_rq[amo_s].amo_op), .is64(s_rq[amo_s].amo64),
                          .a(s_rq[amo_s].amo64 ? s_old[amo_s] : {32'd0, s_old[amo_s][31:0]}),
                          .b(s_rq[amo_s].amo_a), .cmp(s_rq[amo_s].amo_cmp), .r(amo2_r), .cas_hit(amo2_hit));
    always_comb begin
      k_wr    = (mem_req.kind == M_WR);
      k_fence = (mem_req.kind == M_AMO) || (mem_req.kind == M_INV_L1) || (mem_req.kind == M_INV_L2) ||
                (mem_req.kind == M_WB_L2);
      conflict = 1'b0; fwait = 1'b0; any_free = 1'b0; fsl = '0;
      for (int i = NS - 1; i >= 0; i--) if (!s_al[i]) begin any_free = 1'b1; fsl = TSW'(i); end
      for (int i = 0; i < NS; i++) if (s_al[i]) begin
        if ((s_rq[i].pa[47:7] == mem_req.pa[47:7]) && (s_rq[i].kind == M_WR || k_wr)) conflict = 1'b1;
        if ((s_rq[i].kind == M_AMO) || (s_rq[i].kind == M_INV_L1) || (s_rq[i].kind == M_INV_L2) ||
            (s_rq[i].kind == M_WB_L2)) fwait = 1'b1;
        if (k_fence) fwait = 1'b1;
      end
      can        = mem_req_valid && !sp_rst && any_free && !conflict && !fwait && !stall;
      sl_ready   = can && (!k_wr || wstg >= 3);
      sl_fslot   = fsl;
      sl_rd_data = s_line[mem_rd_slot];
      sl_busy    = (s_al != '0);
    end
    always @(posedge clk) begin
      int bi;
      longint bd;
      cyc <= cyc + 1;
      sl_rsp_valid <= 1'b0;
      stall <= (HI > 0) && ($urandom_range(0, 15) == 0);
      if (mem_req_valid && k_wr && can && !sl_ready) wstg <= wstg + 1; else wstg <= 0;
      // --
      if (mem_req_valid && sl_ready) begin
        int lat, t;
        logic [63:0] o;
        lat = (HI > LO) ? $urandom_range(LO, HI) : LO;
        if (HI > 0 && $urandom_range(0, 49) == 0) lat = lat + 4 * HI + 50;
        t = mem_req.pa[47] ? 1 : 0;
        if (cyc + longint'(lat) <= tl[t]) tl[t] = tl[t] + 1; else tl[t] = cyc + longint'(lat);
        s_due[fsl] <= tl[t];
        s_al[fsl] <= 1'b1; s_ans[fsl] <= 1'b0; s_orph[fsl] <= 1'b0; s_rq[fsl] <= mem_req;
        if (mem_req.kind == M_AMO) begin
          for (int b = 0; b < 8; b++) o[8 * b +: 8] = rd8(mem_req.pa + 48'(b));
          s_old[fsl] <= o; amo_s <= int'(fsl);
        end
        sl_n_acc <= sl_n_acc + 1;
      end else if (mem_req_valid && !sp_rst) begin
        if (fwait) sl_n_fstall <= sl_n_fstall + 1;
        else if (conflict) sl_n_cstall <= sl_n_cstall + 1;
      end
      // ---- one answer per clock: the earliest due allocated, unanswered slot
      bi = -1; bd = 0;
      for (int i = 0; i < NS; i++)
        if (s_al[i] && !s_ans[i] && s_due[i] <= cyc && (bi < 0 || s_due[i] < bd)) begin bi = i; bd = s_due[i]; end
      if (bi >= 0) begin
        mrsp_t r;
        logic [1023:0] line;
        r = '0; line = '0;
        case (s_rq[bi].kind)
          M_RD: begin
            n_rd <= n_rd + 1;
            for (int b = 0; b < 128; b++) line[8 * b +: 8] = rd8({s_rq[bi].pa[47:7], 7'd0} + 48'(b));
          end
          M_WR: begin
            n_wr <= n_wr + 1;
            for (int b = 0; b < 128; b++)
              if (s_rq[bi].be[b]) wr8({s_rq[bi].pa[47:7], 7'd0} + 48'(b), s_rq[bi].wdata[8 * b +: 8]);
          end
          M_AMO: begin
            int nb;
            n_amo <= n_amo + 1;
            nb = s_rq[bi].amo64 ? 8 : 4;
            r.old = s_rq[bi].amo64 ? s_old[bi] : {32'd0, s_old[bi][31:0]};
            for (int b = 0; b < nb; b++) wr8(s_rq[bi].pa + 48'(b), amo2_r[8 * b +: 8]);
          end
          default: n_maint <= n_maint + 1;
        endcase
        if (s_orph[bi] || sp_rst) begin
          s_al[bi] <= 1'b0;
          sl_n_orph <= sl_n_orph + 1;
        end else begin
          sl_rsp_valid <= 1'b1; sl_rsp_slot <= TSW'(bi); sl_rsp <= r; sl_rsp_rd <= (s_rq[bi].kind == M_RD);
          if (s_rq[bi].kind == M_RD) begin s_ans[bi] <= 1'b1; s_line[bi] <= line; end
          else s_al[bi] <= 1'b0;
        end
      end
      // ---- free (M_RD slots)
      if (mem_free) begin
        if (s_al[mem_rd_slot] && s_ans[mem_rd_slot]) begin s_al[mem_rd_slot] <= 1'b0; s_ans[mem_rd_slot] <= 1'b0; end
        else if (!sp_rst) err($sformatf("fabric model: mem_free of slot %0d that holds no answered read", mem_rd_slot));
        else sl_n_late_free <= sl_n_late_free + 1;
      end
      // ---- sp_rst (orphan mode)
      if (sp_rst)
        for (int i = 0; i < NS; i++) begin
          if (s_al[i] && s_ans[i]) begin s_al[i] <= 1'b0; s_ans[i] <= 1'b0; end
          else if (s_al[i] && i != bi) s_orph[i] <= 1'b1;
        end
      if ($countones(s_al) > sl_max_out) sl_max_out <= $countones(s_al);
    end
  end else begin : g_fser
    assign sl_ready = 1'b0; assign sl_rsp_valid = 1'b0; assign sl_rsp_rd = 1'b0; assign sl_rsp = '0;
    assign sl_fslot = '0; assign sl_rsp_slot = '0; assign sl_rd_data = '0; assign sl_busy = 1'b0;
  end

  logic unused;
  assign unused = ^{ext_flt_valid, ext_flt_vmid, ext_flt_va, ext_flt_acc, mmu_req_tag, dbg_commit_slot, amo_hit_s,
                    notes};
endmodule
