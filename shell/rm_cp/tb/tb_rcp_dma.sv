// testbench for ng_rcp_dma (clk_l2 bulk COPY/FILL DMA engine) 2.2/2.3.
//   rtl/shell/rm_cp/tb/run_tb_rcp_dma.sh [vivado_bin_dir] [work_dir] ["seed ..."]   expect NG_RCP_DMA_TB_PASS
// Contents: AXI slave model (tb_axi_slv, used for host DW64 and C3 DW128), the top tb with MMU model, reference
// model, directed tests and a random loop.  Parameters come from params.txt ("seed iters", written by the runner) or +seed=N +iters=N.
`timescale 1ns/1ps

// ----------------------------------------------------------------------------------------------------------------
// AXI slave: sparse byte memory, random ready/latency, in-order single-ID responses, protocol checks, SLVERR injection.
module tb_axi_slv #(
  parameter int DW = 64, parameter int AW = 47, parameter int MAXLEN = 63, parameter int MAXOUT = 8,
  parameter string NM = "h"
) (
  input  logic clk, input logic rst_n,
  input  logic awvalid, output logic awready, input logic [AW-1:0] awaddr, input logic [7:0] awlen,
  input  logic wvalid, output logic wready, input logic [DW-1:0] wdata, input logic [DW/8-1:0] wstrb, input logic wlast,
  output logic bvalid, input logic bready, output logic [1:0] bresp,
  input  logic arvalid, output logic arready, input logic [AW-1:0] araddr, input logic [7:0] arlen,
  output logic rvalid, input logic rready, output logic [DW-1:0] rdata, output logic [1:0] rresp, output logic rlast
);
  typedef longint unsigned u64_t;
  localparam int BB = DW / 8;
  typedef struct { longint unsigned addr; int len; longint due; int bad; longint cyc; } bt_t;
  typedef struct { logic [DW-1:0] d; logic [BB-1:0] s; logic l; } wb_t;

  bit [7:0] mem [longint unsigned];
  bit       touched [longint unsigned];
  bt_t arq[$], awq[$], bq[$], arlog[$], awlog[$];
  wb_t wq[$];
  longint ar_first[$], aw_first[$];

  // configuration (set by the tb)
  int rdy_pct = 100, lat_max = 0, inj_r = 0, inj_b = 0;
  // wr_block_rd: no AR accepted and no R returned while an accepted AW still waits for W data (the board's host
  // path behaves like this: a write burst waiting for data held a read back, b7)
  int wr_block_rd = 1;
  // statistics
  longint cyc = 0, last_b_cyc = -1, bad_resp_cyc = -1;
  int nproto = 0, n_arb = 0, n_awb = 0, n_wb = 0, n_wbursts = 0;
  bit inj_hit = 0;
  // state
  longint unsigned rs = 64'h2545F4914F6CDD1D;
  int wk = 0, rbeat = 0;
  bit ar_pend = 0, aw_pend = 0, w_pend = 0;
  logic [AW-1:0] p_awaddr, p_araddr; logic [7:0] p_awlen, p_arlen;
  logic [DW-1:0] p_wdata; logic [BB-1:0] p_wstrb; logic p_wlast;

  function automatic int rn(int n);
    rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
    return (n <= 1) ? 0 : int'((rs >> 8) % u64_t'(n));
  endfunction
  task automatic seed_rng(longint unsigned s); rs = s ^ 64'h9E3779B97F4A7C15; if (rs == 0) rs = 1; endtask
  function automatic bit [7:0] dflt(longint unsigned a);
    logic [63:0] x;
    x = a ^ 64'hA5A51234DEADBEEF; x = x * 64'h9E3779B97F4A7C15; x = x ^ (x >> 29); x = x * 64'hBF58476D1CE4E5B9;
    return x[47:40];
  endfunction
  function automatic bit [7:0] rd(longint unsigned a); return mem.exists(a) ? mem[a] : dflt(a); endfunction
  function automatic int pend(); return arq.size() + awq.size() + wq.size() + bq.size(); endfunction
  task automatic sfail(string m);
    nproto++;
    if (nproto <= 20) $display("TB_FAIL [%s] t=%0t: %s", NM, $time, m);
  endtask
  task automatic cmd_reset();
    touched.delete(); arlog.delete(); awlog.delete(); ar_first.delete(); aw_first.delete();
    n_arb = 0; n_awb = 0; n_wb = 0; n_wbursts = 0; inj_hit = 0; bad_resp_cyc = -1; last_b_cyc = -1;
  endtask

  initial begin awready = 0; wready = 0; arready = 0; bvalid = 0; rvalid = 0; bresp = 0; rresp = 0; rlast = 0; rdata = '0; end

  always @(posedge clk) begin : slv
    bt_t t; wb_t b; bt_t a; longint unsigned base; logic [DW-1:0] dd; int j; longint unsigned ad;
    cyc++;
    if (!rst_n) begin
      arq.delete(); awq.delete(); bq.delete(); wq.delete(); wk = 0; rbeat = 0;
      ar_pend = 0; aw_pend = 0; w_pend = 0;
      awready <= 0; wready <= 0; arready <= 0; bvalid <= 0; rvalid <= 0; bresp <= 0; rresp <= 0; rlast <= 0; rdata <= '0;
    end else begin
      // --
      if (aw_pend && !(awvalid && awaddr === p_awaddr && awlen === p_awlen)) sfail("AW dropped/changed before AWREADY");
      if (awvalid && !aw_pend) aw_first.push_back(cyc);
      if (awvalid && awready) begin
        ad = awaddr;
        if ($isunknown(awaddr) || $isunknown(awlen)) sfail("AW X");
        if ((ad & u64_t'(BB - 1)) != 0) sfail($sformatf("AWADDR %h not beat aligned", ad));
        if (awlen > MAXLEN) sfail($sformatf("AWLEN %0d > max %0d", awlen, MAXLEN));
        if (((ad & 12'hFFF) + (awlen + 1) * BB) > 4096) sfail($sformatf("AW burst crosses 4 KiB addr=%h len=%0d", ad, awlen));
        t = '{addr: ad, len: int'(awlen), due: 0, bad: 0, cyc: cyc};
        awq.push_back(t); awlog.push_back(t); n_awb++;
      end
      aw_pend = awvalid && !awready; p_awaddr = awaddr; p_awlen = awlen;
      // --
      if (w_pend && !(wvalid && wdata === p_wdata && wstrb === p_wstrb && wlast === p_wlast)) sfail("W dropped/changed before WREADY");
      if (wvalid && wready) begin
        if ($isunknown(wstrb) || $isunknown(wlast)) sfail("W strobe/last X");
        b.d = wdata; b.s = wstrb; b.l = wlast; wq.push_back(b); n_wb++;
      end
      w_pend = wvalid && !wready; p_wdata = wdata; p_wstrb = wstrb; p_wlast = wlast;
      // --
      if (bvalid && bready) begin
        if (bq.size() == 0) sfail("internal: B with empty queue");
        else begin
          if (bq[0].bad != 0) begin inj_hit = 1; if (bad_resp_cyc < 0) bad_resp_cyc = cyc; end
          bq.pop_front();
        end
        last_b_cyc = cyc;
      end
      // --
      if (rvalid && rready) begin
        if (arq.size() == 0) sfail("internal: R with empty queue");
        else begin
          if (arq[0].bad != 0) begin inj_hit = 1; if (bad_resp_cyc < 0) bad_resp_cyc = cyc; end
          if (rbeat == arq[0].len) begin arq.pop_front(); rbeat = 0; end else rbeat++;
        end
      end
      // --
      if (ar_pend && !(arvalid && araddr === p_araddr && arlen === p_arlen)) sfail("AR dropped/changed before ARREADY");
      if (arvalid && !ar_pend) ar_first.push_back(cyc);
      if (arvalid && arready) begin
        ad = araddr;
        if ($isunknown(araddr) || $isunknown(arlen)) sfail("AR X");
        if ((ad & u64_t'(BB - 1)) != 0) sfail($sformatf("ARADDR %h not beat aligned", ad));
        if (arlen > MAXLEN) sfail($sformatf("ARLEN %0d > max %0d", arlen, MAXLEN));
        if (((ad & 12'hFFF) + (arlen + 1) * BB) > 4096) sfail($sformatf("AR burst crosses 4 KiB addr=%h len=%0d", ad, arlen));
        n_arb++;
        t = '{addr: ad, len: int'(arlen), due: cyc + 1 + rn(lat_max + 1), bad: ((inj_r != 0 && n_arb == inj_r) ? 1 : 0), cyc: cyc};
        arq.push_back(t); arlog.push_back(t);
      end
      ar_pend = arvalid && !arready; p_araddr = araddr; p_arlen = arlen;
      // ---- apply write data against queued AW (W may arrive before AW)
      while (awq.size() > 0 && wq.size() > 0) begin
        b = wq.pop_front(); a = awq[0];
        if (b.l !== (wk == a.len)) sfail($sformatf("WLAST wrong at beat %0d of burst len %0d (wlast=%b)", wk, a.len, b.l));
        base = (a.addr & ~u64_t'(BB - 1)) + u64_t'(wk) * BB;
        for (j = 0; j < BB; j++) if (b.s[j]) begin mem[base + j] = b.d[8*j +: 8]; touched[base + j] = 1'b1; end
        wk++;
        if (wk > a.len) begin
          t.addr = 0; t.len = 0; t.cyc = cyc; t.due = cyc + 1 + rn(lat_max + 1);
          n_wbursts++;
          t.bad = (inj_b != 0 && n_wbursts == inj_b) ? 1 : 0;
          bq.push_back(t); void'(awq.pop_front()); wk = 0;
        end
      end
      // ---- drive ready / responses for the next cycle
      awready <= (rn(100) < rdy_pct) && ((awq.size() + bq.size()) < MAXOUT);
      wready  <= (rn(100) < rdy_pct) && (wq.size() < 64);
      arready <= (rn(100) < rdy_pct) && (arq.size() < MAXOUT) && !(wr_block_rd && awq.size() != 0);
      if (!(bvalid && !bready)) begin
        if (bq.size() > 0 && bq[0].due <= cyc) begin bvalid <= 1'b1; bresp <= (bq[0].bad != 0) ? 2'b10 : 2'b00; end
        else bvalid <= 1'b0;
      end
      if (!(rvalid && !rready)) begin
        if (arq.size() > 0 && arq[0].due <= cyc && rn(100) < rdy_pct && !(wr_block_rd && awq.size() != 0)) begin
          base = (arq[0].addr & ~u64_t'(BB - 1)) + u64_t'(rbeat) * BB;
          for (j = 0; j < BB; j++) dd[8*j +: 8] = rd(base + j);
          rvalid <= 1'b1; rdata <= dd; rresp <= (arq[0].bad != 0) ? 2'b10 : 2'b00; rlast <= (rbeat == arq[0].len);
        end else begin
          rvalid <= 1'b0;
        end
      end
    end
  end
endmodule

// ----------------------------------------------------------------------------------------------------------------
module tb_rcp_dma;
  import ng_mmu_pkg::*;
  typedef longint unsigned u64_t;

  // ---------------- clock / reset / DUT signals
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic start = 0, op_fill = 0, abort = 0;
  logic [3:0] vmid = 0; logic [47:0] src = 0, dst = 0; logic [63:0] size = 0; logic [31:0] fill = 0;
  logic busy, done, err;
  logic t_req_valid, t_resp_ready; logic t_req_ready = 0, t_resp_valid = 0;
  logic [3:0] t_req_vmid; logic [63:0] t_req_va; logic [1:0] t_req_acc;
  logic [2:0] t_resp_fault = 0; logic t_resp_sparse = 0; logic [47:0] t_resp_pa = 0; logic [1:0] t_resp_level = 0;
  logic [63:0] t_resp_pte = 0;
  logic ef_valid; logic [3:0] ef_vmid; logic [63:0] ef_va; logic [1:0] ef_acc; logic [2:0] ef_code; logic [1:0] ef_level;
  logic [63:0] ef_pte;

  logic h_awvalid, h_awready, h_wvalid, h_wready, h_wlast, h_bvalid, h_bready, h_arvalid, h_arready, h_rvalid, h_rready, h_rlast;
  logic [46:0] h_awaddr, h_araddr; logic [7:0] h_awlen, h_arlen; logic [63:0] h_wdata, h_rdata; logic [7:0] h_wstrb;
  logic [1:0] h_bresp, h_rresp;
  logic c_awvalid, c_awready, c_wvalid, c_wready, c_wlast, c_bvalid, c_bready, c_arvalid, c_arready, c_rvalid, c_rready, c_rlast;
  logic [30:0] c_awaddr, c_araddr; logic [7:0] c_awlen, c_arlen; logic [127:0] c_wdata, c_rdata; logic [15:0] c_wstrb;
  logic [1:0] c_bresp, c_rresp;

  ng_rcp_dma dut (
    .clk, .rst_n, .start, .op_fill, .vmid, .src, .dst, .size, .fill, .abort, .busy, .done, .err,
    .t_req_valid, .t_req_ready, .t_req_vmid, .t_req_va, .t_req_acc, .t_resp_valid, .t_resp_ready, .t_resp_fault,
    .t_resp_sparse, .t_resp_pa, .t_resp_level, .t_resp_pte,
    .ef_valid, .ef_vmid, .ef_va, .ef_acc, .ef_code, .ef_level, .ef_pte,
    .h_awvalid, .h_awready, .h_awaddr, .h_awlen, .h_wvalid, .h_wready, .h_wdata, .h_wstrb, .h_wlast,
    .h_bvalid, .h_bready, .h_bresp, .h_arvalid, .h_arready, .h_araddr, .h_arlen, .h_rvalid, .h_rready, .h_rdata,
    .h_rresp, .h_rlast,
    .c_awvalid, .c_awready, .c_awaddr, .c_awlen, .c_wvalid, .c_wready, .c_wdata, .c_wstrb, .c_wlast,
    .c_bvalid, .c_bready, .c_bresp, .c_arvalid, .c_arready, .c_araddr, .c_arlen, .c_rvalid, .c_rready, .c_rdata,
    .c_rresp, .c_rlast
  );

  tb_axi_slv #(.DW(64), .AW(47), .MAXLEN(63), .MAXOUT(8), .NM("host")) u_h (
    .clk, .rst_n,
    .awvalid(h_awvalid), .awready(h_awready), .awaddr(h_awaddr), .awlen(h_awlen),
    .wvalid(h_wvalid), .wready(h_wready), .wdata(h_wdata), .wstrb(h_wstrb), .wlast(h_wlast),
    .bvalid(h_bvalid), .bready(h_bready), .bresp(h_bresp),
    .arvalid(h_arvalid), .arready(h_arready), .araddr(h_araddr), .arlen(h_arlen),
    .rvalid(h_rvalid), .rready(h_rready), .rdata(h_rdata), .rresp(h_rresp), .rlast(h_rlast));
  tb_axi_slv #(.DW(128), .AW(31), .MAXLEN(15), .MAXOUT(16), .NM("c3")) u_c (
    .clk, .rst_n,
    .awvalid(c_awvalid), .awready(c_awready), .awaddr(c_awaddr), .awlen(c_awlen),
    .wvalid(c_wvalid), .wready(c_wready), .wdata(c_wdata), .wstrb(c_wstrb), .wlast(c_wlast),
    .bvalid(c_bvalid), .bready(c_bready), .bresp(c_bresp),
    .arvalid(c_arvalid), .arready(c_arready), .araddr(c_araddr), .arlen(c_arlen),
    .rvalid(c_rvalid), .rready(c_rready), .rdata(c_rdata), .rresp(c_rresp), .rlast(c_rlast));

  // ---------------- random numbers (deterministic from +seed)
  longint unsigned rs = 64'h1234567;
  function automatic longint unsigned rnd64();
    rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return rs;
  endfunction
  function automatic int rn(int n);   // 0 .. n-1
    return (n <= 1) ? 0 : int'((rnd64() >> 8) % u64_t'(n));
  endfunction
  function automatic int rr(int lo, int hi); return lo + rn(hi - lo + 1); endfunction

  // ---------------- page table / address space
  localparam int K_NONE = 0, K_HOST = 1, K_C3 = 2, K_SPARSE = 3, K_FAULT = 4, K_VRAM = 5, K_INVAL = 6;
  localparam int M_HOST = 1, M_C3 = 2, M_MIX = 3, M_MIXS = 4;
  localparam longint unsigned SYS_BASE = 48'h10_0000_0000;
  localparam longint unsigned HOST_BIT = 48'h8000_0000_0000;

  typedef struct { int kind; longint unsigned pbase; int fault; int level; logic [63:0] pte; } pe_t;
  pe_t pt [longint unsigned];
  bit  used_h [longint unsigned];
  bit  used_c [longint unsigned];
  int  role [longint unsigned];

  function automatic pe_t new_page(int kind, int fault);
    pe_t e; longint unsigned x;
    e.kind = kind; e.pbase = 0; e.fault = fault; e.level = rn(4); e.pte = {rnd64()};
    if (kind == K_HOST) begin
      do x = rnd64() % (64'd1 << 34); while (used_h.exists(x));
      used_h[x] = 1'b1; e.pbase = x << 12;
    end else if (kind == K_C3) begin
      do x = rnd64() % 64'd524288; while (used_c.exists(x));
      used_c[x] = 1'b1; e.pbase = x << 12;
    end
    return e;
  endfunction
  function automatic int pick_kind(int mode);
    int r;
    case (mode)
      M_HOST: return K_HOST;
      M_C3:   return K_C3;
      M_MIX:  return rn(2) ? K_HOST : K_C3;
      default: begin r = rn(10); return (r < 4) ? K_HOST : (r < 8) ? K_C3 : K_SPARSE; end
    endcase
  endfunction
  task automatic map_range(longint unsigned va, longint unsigned n, int mode);
    longint unsigned p;
    for (p = va >> 12; p <= ((va + n - 1) >> 12); p++)
      if (!pt.exists(p)) pt[p] = new_page(pick_kind(mode), F_NOT_PRESENT);
  endtask
  task automatic set_page(longint unsigned va, int kind, int fault);
    pt[va >> 12] = new_page(kind, fault);
  endtask

  typedef struct { int kind; longint unsigned pa; } res_t;   // pa: C3 offset (31 b) or host address (47 b)
  function automatic res_t resolve(int vm, longint unsigned va);
    res_t r; pe_t e;
    r.kind = K_NONE; r.pa = 0;
    if (vm == 0) begin
      if ((va >> 48) != 0) r.kind = K_INVAL;
      else if (va[47]) begin r.kind = K_HOST; r.pa = va & 64'h7FFF_FFFF_FFFF; end
      else if (va[47:31] == 17'h20) begin r.kind = K_C3; r.pa = va & 64'h7FFF_FFFF; end
      else r.kind = K_INVAL;
    end else if (!pt.exists(va >> 12)) begin
      r.kind = K_INVAL;
    end else begin
      e = pt[va >> 12];
      r.kind = e.kind;
      if (e.kind == K_HOST || e.kind == K_C3) r.pa = e.pbase + (va & 12'hFFF);
    end
    return r;
  endfunction

  // ---------------- test bookkeeping
  string cur_test = "";
  int test_fail = 0, total_fail = 0, test_cmds = 0, total_cmds = 0, tests_run = 0, tests_failed = 0;
  int proto_seen = 0;
  task automatic fail(string m);
    test_fail++; total_fail++;
    if (test_fail <= 12) $display("TB_FAIL [%s] t=%0t: %s", cur_test, $time, m);
  endtask
  task automatic begin_test(string n); cur_test = n; test_fail = 0; test_cmds = 0; endtask
  task automatic end_test();
    tests_run++;
    if (test_fail == 0) $display("TEST %-28s PASS (%0d cmds)", cur_test, test_cmds);
    else begin tests_failed++; $display("TEST %-28s FAIL (%0d failures, %0d cmds)", cur_test, test_fail, test_cmds); end
  endtask

  // ---------------- current command / reference model
  int c_vmid = 0; bit c_fill = 0; longint unsigned c_src = 0, c_dst = 0, c_size = 0; logic [31:0] c_fillv = 0;
  bit [7:0] ew_h [longint unsigned];
  bit [7:0] ew_c [longint unsigned];
  bit pg_h [longint unsigned];
  bit pg_c [longint unsigned];
  bit exp_err = 0, has_vram = 0, has_oth = 0;
  longint cyc = 0, stop_cyc = 64'h0FFF_FFFF_FFFF_FFFF;
  bit active = 0, done_seen = 0, done_err_v = 0, done_q = 0, aborted = 0, abort_seen = 0;
  longint done_cyc = 0;
  int mmu_n = 0, ef_cnt = 0;
  int mmu_rdy_pct = 100, mmu_lat_max = 0;
  localparam int SLACK = 6;

  function automatic bit [7:0] rdm(bit is_h, longint unsigned a); return is_h ? u_h.rd(a) : u_c.rd(a); endfunction

  task automatic mark_bad(int k); exp_err = 1; if (k == K_VRAM) has_vram = 1; else has_oth = 1; endtask

  task automatic build_ref();
    longint unsigned i, p; res_t d, s; bit [7:0] v; bit sok;
    ew_h.delete(); ew_c.delete(); pg_h.delete(); pg_c.delete(); role.delete();
    exp_err = 0; has_vram = 0; has_oth = 0;
    for (p = c_dst >> 12; p <= ((c_dst + c_size - 1) >> 12); p++) role[p] = (role.exists(p) ? role[p] : 0) | 2;
    if (!c_fill) for (p = c_src >> 12; p <= ((c_src + c_size - 1) >> 12); p++) role[p] = (role.exists(p) ? role[p] : 0) | 1;
    for (i = 0; i < c_size; i++) begin
      d = resolve(c_vmid, c_dst + i);
      sok = 1; s.kind = K_SPARSE; s.pa = 0;
      if (!c_fill) s = resolve(c_vmid, c_src + i);
      if (d.kind == K_FAULT || d.kind == K_VRAM || d.kind == K_INVAL) mark_bad(d.kind);
      if (!c_fill && (s.kind == K_FAULT || s.kind == K_VRAM || s.kind == K_INVAL)) begin mark_bad(s.kind); sok = 0; end
      if (!c_fill) begin
        if (s.kind == K_HOST) pg_h[s.pa >> 12] = 1'b1;
        if (s.kind == K_C3)   pg_c[s.pa >> 12] = 1'b1;
      end
      if (d.kind == K_HOST || d.kind == K_C3) begin
        if (d.kind == K_HOST) pg_h[d.pa >> 12] = 1'b1; else pg_c[d.pa >> 12] = 1'b1;
        if (c_fill) v = c_fillv[8*(i & 3) +: 8];
        else if (!sok || s.kind == K_SPARSE) v = 8'h00;
        else v = rdm(s.kind == K_HOST, s.pa);
        if (d.kind == K_HOST) ew_h[d.pa] = v; else ew_c[d.pa] = v;
      end
    end
  endtask

  // ---------------- MMU model + monitors (single posedge process; shared state is touched by the main flow only
  // at negedge)
  int m_state = 0, m_cnt = 0;
  bit rq_pend = 0;
  logic [3:0] p_rvmid; logic [63:0] p_rva; logic [1:0] p_racc;
  int lr_kind = 0, lr_vmid = 0; longint unsigned lr_va = 0; int lr_acc = 0, lr_level = 0; logic [63:0] lr_pte = 0;
  pe_t pe_cur; longint unsigned rq_va;

  always @(posedge clk) begin : mon
    longint unsigned pg; int ro; res_t dummy; logic [47:0] garbage;
    cyc++;
    if (!rst_n) begin
      m_state = 0; rq_pend = 0; t_req_ready <= 0; t_resp_valid <= 0; done_q = 0;
    end else begin
      // ---- done / err monitor
      if (done) begin
        if (done_q) fail("done asserted for more than one cycle");
        else if (!active) fail("spurious done while no command is active");
        else if (done_seen) fail("second done for one command");
        else begin
          done_seen = 1; done_err_v = err; done_cyc = cyc;
          if (u_h.pend() != 0 || u_c.pend() != 0 || h_rvalid || c_rvalid || h_bvalid || c_bvalid)
            fail($sformatf("done with outstanding AXI work (host pend=%0d, c3 pend=%0d)", u_h.pend(), u_c.pend()));
          if (m_state != 0 || t_req_valid) fail("done with an MMU request outstanding");
        end
      end
      done_q = done;
      if (abort && !abort_seen) begin abort_seen = 1; if (cyc < stop_cyc) stop_cyc = cyc; end
      if (!abort) abort_seen = 0;
      // ---- ef monitor
      if (ef_valid) begin
        if (!active) fail("spurious ef_valid while no command is active");
        else if (lr_kind != K_VRAM) fail("ef_valid without a BAD_PHYS translation response");
        else begin
          ef_cnt++;
          if (ef_code !== F_BAD_PHYS) fail($sformatf("ef_code %0d != F_BAD_PHYS", ef_code));
          if (ef_vmid !== lr_vmid[3:0]) fail("ef_vmid mismatch");
          if (ef_va[63:12] !== lr_va[63:12]) fail($sformatf("ef_va %h mismatch (req page %h)", ef_va, lr_va));
          if (ef_acc !== lr_acc[1:0]) fail("ef_acc mismatch");
          if (ef_level !== lr_level[1:0]) fail("ef_level mismatch");
          if (ef_pte !== lr_pte) fail("ef_pte mismatch");
        end
      end
      // ---- MMU request channel
      if (rq_pend && !(t_req_valid && t_req_vmid === p_rvmid && t_req_va === p_rva && t_req_acc === p_racc))
        fail("MMU request dropped/changed before ready");
      rq_pend = t_req_valid && !t_req_ready; p_rvmid = t_req_vmid; p_rva = t_req_va; p_racc = t_req_acc;
      if (t_req_valid && t_req_ready) begin
        mmu_n++;
        lr_vmid = t_req_vmid; lr_va = t_req_va; lr_acc = t_req_acc;
        pg = t_req_va >> 12;
        if (!active) fail("MMU request while no command is active");
        if (t_req_vmid === 4'd0) fail("MMU request with VMID 0 (physical commands must not translate)");
        if (c_vmid != 0 && t_req_vmid !== c_vmid[3:0]) fail("MMU request vmid != command vmid");
        if (t_req_va[11:0] !== 12'h0 || t_req_va[63:48] !== 16'h0) fail($sformatf("MMU request VA %h not a page address", t_req_va));
        if (t_req_acc !== ACC_R && t_req_acc !== ACC_W) fail("MMU request acc is not R/W");
        if (!role.exists(pg)) begin
          fail($sformatf("MMU request for page %h outside the command's ranges", t_req_va));
          pe_cur = new_page(K_FAULT, F_NOT_PRESENT); lr_kind = K_FAULT;
        end else begin
          ro = role[pg];
          if (ro == 1 && t_req_acc !== ACC_R) fail($sformatf("source page %h translated with acc=%0d, expected ACC_R", t_req_va, t_req_acc));
          if (ro == 2 && t_req_acc !== ACC_W) fail($sformatf("dest page %h translated with acc=%0d, expected ACC_W", t_req_va, t_req_acc));
          pe_cur = pt[pg]; lr_kind = pe_cur.kind;
        end
        lr_level = pe_cur.level; lr_pte = pe_cur.pte;
        m_state = 1; m_cnt = rn(mmu_lat_max + 1);
      end
      // ---- MMU response channel
      if (rq_pend == 0 && 0) begin end
      if (t_resp_valid && t_resp_ready) begin
        m_state = 0; t_resp_valid <= 1'b0;
        // the DUT can only stop once it has the faulting response (writes for other pages may go on until then)
        if ((lr_kind == K_FAULT || lr_kind == K_VRAM) && cyc < stop_cyc) stop_cyc = cyc;
      end
      case (m_state)
        0: t_req_ready <= (rn(100) < mmu_rdy_pct);
        1: begin
          t_req_ready <= 1'b0;
          if (m_cnt == 0) begin
            m_state = 2;
            garbage = {rnd64()};
            t_resp_valid <= 1'b1; t_resp_level <= pe_cur.level[1:0]; t_resp_pte <= pe_cur.pte;
            t_resp_fault <= F_NONE; t_resp_sparse <= 1'b0; t_resp_pa <= garbage;
            case (pe_cur.kind)
              K_HOST:   t_resp_pa <= {1'b1, pe_cur.pbase[46:0]};
              K_C3:     t_resp_pa <= {17'h20, pe_cur.pbase[30:0]};
              K_SPARSE: begin t_resp_sparse <= 1'b1; end
              K_FAULT:  begin t_resp_fault <= pe_cur.fault[2:0]; t_resp_sparse <= garbage[0]; end
              default: begin   // K_VRAM: not SYS, bit 47 clear
                t_resp_pa <= {1'b0, garbage[46:12], 12'h0};
                if (garbage[46:30] == 17'h20) t_resp_pa <= {1'b0, garbage[46:41], 1'b1, garbage[39:12], 12'h0};
              end
            endcase
          end else m_cnt--;
        end
        default: t_req_ready <= 1'b0;
      endcase
    end
  end

  // ---------------- command runner and checker
  int inj_start_at = -1;   // pulse a bogus second start while busy at this many cycles after start

  task automatic do_reset();
    @(negedge clk); rst_n = 0; start = 0; abort = 0; active = 0;
    repeat (4) @(negedge clk);
    rst_n = 1; repeat (2) @(negedge clk);
    done_seen = 0;
  endtask

  task automatic scramble_cmd();
    vmid = rn(16); op_fill = rn(2); src = {rnd64()}; dst = {rnd64()}; size = {rnd64()}; fill = {rnd64()};
  endtask

  task automatic check_cmd(bit exp_e_inj, int h_ar0, int h_aw0, int c_ar0, int c_aw0);
    bit exp_e; bit do_cmp; longint unsigned k; int bad, un, n; bit ok;
    longint last_b; int flush_n, flush_ok; int i; longint stp; bit exp_ef;
    exp_e = exp_err | u_h.inj_hit | u_c.inj_hit;
    do_cmp = !exp_e && !aborted;
    // done / err
    if (aborted && !exp_e) begin
      if (done_err_v) fail("err set on abort (no error condition present)");
    end else if (!aborted) begin
      if (done_err_v !== exp_e) fail($sformatf("err=%0b, expected %0b (ref=%0b injH=%0b injC=%0b)", done_err_v, exp_e, exp_err, u_h.inj_hit, u_c.inj_hit));
    end
    // protocol violations in the slaves
    if (u_h.nproto + u_c.nproto > proto_seen) begin
      fail($sformatf("%0d AXI protocol violations reported by slave models", u_h.nproto + u_c.nproto - proto_seen));
      proto_seen = u_h.nproto + u_c.nproto;
    end
    // nothing after done
    if (u_h.arlog.size() != h_ar0 || u_h.awlog.size() != h_aw0 || u_c.arlog.size() != c_ar0 || u_c.awlog.size() != c_aw0)
      fail("AXI AR/AW issued after done");
    // memory: no byte outside the destination written, and (success) every destination byte written correctly
    bad = 0; un = 0;
    if (u_h.touched.first(k)) do begin
      if (!ew_h.exists(k)) begin bad++; if (bad <= 4) fail($sformatf("host write outside destination, byte %h", k)); end
    end while (u_h.touched.next(k));
    if (u_c.touched.first(k)) do begin
      if (!ew_c.exists(k)) begin bad++; if (bad <= 4) fail($sformatf("C3 write outside destination, byte %h", k)); end
    end while (u_c.touched.next(k));
    if (do_cmp) begin
      if (ew_h.first(k)) do begin
        if (!u_h.touched.exists(k)) un++;
        if (u_h.rd(k) !== ew_h[k]) begin un++; if (un <= 6) fail($sformatf("host byte %h = %h, expected %h", k, u_h.rd(k), ew_h[k])); end
      end while (ew_h.next(k));
      if (ew_c.first(k)) do begin
        if (!u_c.touched.exists(k)) un++;
        if (u_c.rd(k) !== ew_c[k]) begin un++; if (un <= 6) fail($sformatf("C3 byte %h = %h, expected %h", k, u_c.rd(k), ew_c[k])); end
      end while (ew_c.next(k));
      if (un != 0 && un > 6) fail($sformatf("%0d destination byte errors/unwritten bytes in total", un));
    end
    // AR addresses must be within pages of the command (source pages, destination pages for the flush read)
    for (i = 0; i < u_h.arlog.size(); i++) if (!pg_h.exists(u_h.arlog[i].addr >> 12))
      fail($sformatf("host AR %h outside the command's pages", u_h.arlog[i].addr));
    for (i = 0; i < u_c.arlog.size(); i++) if (!pg_c.exists(u_c.arlog[i].addr >> 12))
      fail($sformatf("C3 AR %h outside the command's pages", u_c.arlog[i].addr));
    // flush read: exactly one read inside a written span after the last host B (success only)
    if (do_cmp && u_h.awlog.size() > 0) begin
      flush_n = 0;
      for (i = 0; i < u_h.arlog.size(); i++) if (u_h.arlog[i].cyc > u_h.last_b_cyc) begin
        ok = 0;
        for (int j = 0; j < u_h.awlog.size(); j++)
          if (u_h.arlog[i].addr >= u_h.awlog[j].addr && u_h.arlog[i].addr < u_h.awlog[j].addr + (u_h.awlog[j].len + 1) * 8) ok = 1;
        if (ok) flush_n++;
      end
      if (flush_n != 1) fail($sformatf("expected exactly 1 flush read after the last host write response, saw %0d", flush_n));
    end
    // no new AR/AW after an error / abort was seen
    stp = stop_cyc;
    if (u_h.bad_resp_cyc >= 0 && u_h.bad_resp_cyc < stp) stp = u_h.bad_resp_cyc;
    if (u_c.bad_resp_cyc >= 0 && u_c.bad_resp_cyc < stp) stp = u_c.bad_resp_cyc;
    if (stp < 64'h0FFF_FFFF_FFFF_FFFF) begin
      for (i = 0; i < u_h.ar_first.size(); i++) if (u_h.ar_first[i] > stp + SLACK) fail("new host AR after error/abort");
      for (i = 0; i < u_h.aw_first.size(); i++) if (u_h.aw_first[i] > stp + SLACK) fail("new host AW after error/abort");
      for (i = 0; i < u_c.ar_first.size(); i++) if (u_c.ar_first[i] > stp + SLACK) fail("new C3 AR after error/abort");
      for (i = 0; i < u_c.aw_first.size(); i++) if (u_c.aw_first[i] > stp + SLACK) fail("new C3 AW after error/abort");
    end
    // BAD_PHYS report
    exp_ef = has_vram && !has_oth && !u_h.inj_hit && !u_c.inj_hit && !aborted;
    if (exp_ef && ef_cnt == 0) fail("BAD_PHYS page in range but no ef_valid report");
    if (!has_vram && ef_cnt != 0) fail("ef_valid reported without a BAD_PHYS page");
    if (c_vmid == 0 && mmu_n != 0) fail("MMU used for VMID 0");
  endtask

  // abort_at: cycles after start at which abort goes high (-1 none)
  task automatic run_cmd(input int vm, input bit fl, input longint unsigned s, input longint unsigned d,
                         input longint unsigned sz, input logic [31:0] fv, input int abort_at);
    int ts, limit, h_ar0, h_aw0, c_ar0, c_aw0, w; bit timed_out;
    @(negedge clk);
    w = 0;
    while (busy && w < 300) begin @(negedge clk); w++; end
    if (busy) begin fail("busy stuck high before command; resetting"); do_reset(); end
    c_vmid = vm; c_fill = fl; c_src = s; c_dst = d; c_size = sz; c_fillv = fv;
    build_ref();
    u_h.cmd_reset(); u_c.cmd_reset();
    mmu_n = 0; ef_cnt = 0; stop_cyc = 64'h0FFF_FFFF_FFFF_FFFF; done_seen = 0; aborted = 0; lr_kind = 0;
    active = 1;
    repeat (2) @(negedge clk);
    start = 1; vmid = vm[3:0]; op_fill = fl; src = s[47:0]; dst = d[47:0]; size = sz; fill = fv;
    @(negedge clk);
    start = 0; scramble_cmd();
    test_cmds++; total_cmds++;
    limit = 100000 + int'(sz) * 60;
    ts = 0; timed_out = 0;
    while (!done_seen && !timed_out) begin
      @(negedge clk); ts++;
      if (abort_at >= 0 && ts == abort_at && !done_seen) begin abort = 1; aborted = 1; end
      if (inj_start_at >= 0 && ts == inj_start_at && busy && !done_seen) begin
        start = 1; vmid = rn(16); op_fill = rn(2); src = {rnd64()}; dst = {rnd64()}; size = 64'd77; fill = {rnd64()};
        @(negedge clk); start = 0; scramble_cmd();
      end
      if (ts > limit) timed_out = 1;
    end
    if (timed_out) begin
      fail($sformatf("TIMEOUT: no done after %0d cycles (vmid=%0d fill=%0b src=%h dst=%h size=%0d)", limit, vm, fl, s, d, sz));
      $display("  stuck state: busy=%0b t_req_valid=%0b mmu_state=%0d t_resp_ready=%0b | host pend=%0d ar/aw/w/b q=%0d/%0d/%0d/%0d arvalid=%0b awvalid=%0b wvalid=%0b rready=%0b bready=%0b | c3 pend=%0d ar/aw/w/b q=%0d/%0d/%0d/%0d arvalid=%0b awvalid=%0b wvalid=%0b rready=%0b bready=%0b",
        busy, t_req_valid, m_state, t_resp_ready, u_h.pend(), u_h.arq.size(), u_h.awq.size(), u_h.wq.size(), u_h.bq.size(),
        h_arvalid, h_awvalid, h_wvalid, h_rready, h_bready, u_c.pend(), u_c.arq.size(), u_c.awq.size(), u_c.wq.size(), u_c.bq.size(),
        c_arvalid, c_awvalid, c_wvalid, c_rready, c_bready);
      abort = 0; active = 0; do_reset();
    end else begin
      h_ar0 = u_h.arlog.size(); h_aw0 = u_h.awlog.size(); c_ar0 = u_c.arlog.size(); c_aw0 = u_c.awlog.size();
      repeat (6) @(negedge clk);
      if (busy) fail("busy still high 6 cycles after done");
      abort = 0;
      begin
        int e0;
        e0 = total_fail;
        check_cmd(0, h_ar0, h_aw0, c_ar0, c_aw0);
        if (total_fail != e0)
          $display("  failing command: vmid=%0d fill=%0b src=%h dst=%h size=%0d abort_at=%0d err=%0b",
                   vm, fl, s, d, sz, abort_at, err);
      end
      repeat (2) @(negedge clk);
      active = 0;
    end
    inj_start_at = -1;
  endtask

  task automatic cp(input int vm, input longint unsigned s, input longint unsigned d, input longint unsigned sz);
    run_cmd(vm, 0, s, d, sz, 32'h0, -1);
  endtask
  task automatic fl_cmd(input int vm, input longint unsigned d, input longint unsigned sz, input logic [31:0] fv);
    run_cmd(vm, 1, 48'h0, d, sz, fv, -1);
  endtask
  task automatic no_inj();
    u_h.inj_r = 0; u_h.inj_b = 0; u_c.inj_r = 0; u_c.inj_b = 0;
  endtask
  task automatic cfg_fast();
    u_h.rdy_pct = 100; u_h.lat_max = 0; u_c.rdy_pct = 100; u_c.lat_max = 0; mmu_rdy_pct = 100; mmu_lat_max = 0;
  endtask
  task automatic cfg_random();
    int pc [5]; int lt [4]; int ml [4];
    pc = '{100, 100, 70, 40, 15}; lt = '{0, 3, 12, 40}; ml = '{0, 2, 10, 40};
    u_h.rdy_pct = pc[rn(5)]; u_c.rdy_pct = pc[rn(5)]; u_h.lat_max = lt[rn(4)]; u_c.lat_max = lt[rn(4)];
    u_h.wr_block_rd = (rn(4) != 0); u_c.wr_block_rd = (rn(4) != 0);
    mmu_rdy_pct = pc[rn(5)]; mmu_lat_max = ml[rn(4)];
  endtask
  // a small clean copy that must work after an error/abort test (state recovery)
  task automatic recover();
    longint unsigned a, b;
    no_inj(); cfg_fast();
    a = 48'h0000_0300_0000 + (rn(1000) << 12) + 3; b = 48'h0000_0310_0000 + (rn(1000) << 12) + 5;
    pt.delete();
    map_range(a, 300, M_HOST); map_range(b, 300, M_C3);
    cp(5, a, b, 300);
    pt.delete();
  endtask

  // ---------------- directed tests
  localparam longint unsigned VS = 48'h0000_0010_0000, VD = 48'h0000_0200_0000;

  task automatic t_basic();
    begin_test("copy_aligned8_h2h");  pt.delete(); map_range(VS, 8, M_HOST); map_range(VD, 8, M_HOST);
    cp(3, VS, VD, 8); end_test();
    begin_test("copy_1byte_variants");
    pt.delete(); map_range(VS, 64, M_MIX); map_range(VD, 64, M_MIX);
    for (int so = 0; so < 8; so++) for (int dof = 0; dof < 8; dof++) cp(2, VS + so, VD + dof, 1);
    end_test();
    begin_test("copy_4k_aligned");
    pt.delete(); map_range(VS, 4096, M_HOST); map_range(VD, 4096, M_HOST);
    cp(4, VS, VD, 4096); end_test();
    begin_test("copy_align_sweep_h2h");
    pt.delete(); map_range(VS, 100, M_HOST); map_range(VD, 100, M_HOST);
    for (int so = 0; so < 8; so++) for (int dof = 0; dof < 8; dof++) begin
      cp(6, VS + so, VD + dof, 1); cp(6, VS + so, VD + dof, 7); cp(6, VS + so, VD + dof, 9); cp(6, VS + so, VD + dof, 37);
    end
    end_test();
    begin_test("copy_align_sweep_c2c");
    pt.delete(); map_range(VS, 200, M_C3); map_range(VD, 200, M_C3);
    for (int so = 0; so < 16; so += (so < 2 ? 1 : 5)) for (int dof = 0; dof < 16; dof += (dof < 2 ? 1 : 5)) begin
      cp(7, VS + so, VD + dof, 1); cp(7, VS + so, VD + dof, 15); cp(7, VS + so, VD + dof, 17); cp(7, VS + so, VD + dof, 71);
    end
    end_test();
  endtask

  task automatic t_cross();
    begin_test("copy_cross_page");
    pt.delete(); map_range(VS, 40000, M_HOST); map_range(VD, 40000, M_HOST);
    cp(3, VS + 4090, VD + 100, 20);          // src crosses a page, dst does not
    cp(3, VS + 100, VD + 4090, 20);          // dst crosses, src does not
    cp(3, VS + 4093, VD + 4091, 4200);       // both cross, > 1 page
    cp(3, VS + 4095, VD + 4096, 8192 + 13);
    cp(3, VS + 1, VD + 3, 3 * 4096);
    end_test();
    begin_test("copy_burst_boundaries");   // sizes around host 512 B / C3 256 B bursts and 4 KiB
    pt.delete(); map_range(VS, 40000, M_HOST); map_range(VD, 40000, M_C3);
    for (int sz = 250; sz <= 262; sz++) cp(3, VS + 4096 - 8, VD + 4096 - 16, sz);
    for (int sz = 508; sz <= 516; sz++) cp(3, VS + 5, VD + 3, sz);
    cp(3, VS + 7, VD + 9, 4095); cp(3, VS, VD, 4097);
    end_test();
  endtask

  task automatic t_routes();
    begin_test("copy_h2c");   pt.delete(); map_range(VS, 30000, M_HOST); map_range(VD, 30000, M_C3);
    cp(2, VS + 3, VD + 11, 20000); cp(2, VS, VD, 4096); end_test();
    begin_test("copy_c2h");   pt.delete(); map_range(VS, 30000, M_C3); map_range(VD, 30000, M_HOST);
    cp(2, VS + 13, VD + 5, 20000); cp(2, VS, VD, 4096); end_test();
    begin_test("copy_c2c");   pt.delete(); map_range(VS, 30000, M_C3); map_range(VD, 30000, M_C3);
    cp(2, VS + 1, VD + 15, 20000); cp(2, VS, VD, 4096); end_test();
    begin_test("copy_h2h_big"); pt.delete(); map_range(VS, 70000, M_HOST); map_range(VD, 70000, M_HOST);
    cp(2, VS + 2, VD + 6, 65536); end_test();
    begin_test("copy_mixed_pages"); pt.delete(); map_range(VS, 60000, M_MIX); map_range(VD, 60000, M_MIX);
    cp(9, VS + 9, VD + 1, 50000); end_test();
  endtask

  task automatic t_fill();
    begin_test("fill");
    pt.delete(); map_range(VD, 30000, M_HOST); map_range(VD + 32'h100000, 30000, M_C3);
    fl_cmd(3, VD, 4, 32'hDEADBEEF); fl_cmd(3, VD + 4, 8, 32'h01020304); fl_cmd(3, VD + 8, 12, 32'hA5A5A5A5);
    fl_cmd(3, VD + 4092, 16, 32'h11223344); fl_cmd(3, VD, 4096, 32'h55AA55AA); fl_cmd(3, VD + 12, 20000, 32'hCAFEF00D);
    fl_cmd(3, VD + 32'h100000, 4, 32'h0BADF00D); fl_cmd(3, VD + 32'h100004, 12, 32'h99999999);
    fl_cmd(3, VD + 32'h100000 + 4088, 4100, 32'h31415926); fl_cmd(3, VD + 32'h100000, 24000, 32'h27182818);
    for (int dof = 0; dof < 16; dof += 4) for (int sz = 4; sz <= 68; sz += 4) fl_cmd(3, VD + 32'h100000 + 256 + dof, sz, {rnd64()});
    for (int dof = 0; dof < 16; dof += 4) for (int sz = 4; sz <= 68; sz += 4) fl_cmd(3, VD + 1024 + dof, sz, {rnd64()});
    end_test();
  endtask

  task automatic t_sparse();
    begin_test("sparse_src_dst");
    pt.delete(); map_range(VS, 5 * 4096, M_HOST); map_range(VD, 5 * 4096, M_C3);
    set_page(VS + 1 * 4096, K_SPARSE, 0); set_page(VS + 3 * 4096, K_SPARSE, 0);
    cp(3, VS + 100, VD + 7, 4 * 4096 + 300);                  // src page 1 and 3 read as 0
    set_page(VD + 2 * 4096, K_SPARSE, 0);
    cp(3, VS + 100, VD + 7, 4 * 4096 + 300);                  // plus dst page 2 dropped
    set_page(VS, K_SPARSE, 0);
    cp(3, VS + 4000, VD + 4000, 400);                         // both sparse at the start
    pt.delete(); map_range(VS, 3 * 4096, M_C3); map_range(VD, 3 * 4096, M_HOST);
    set_page(VD + 4096, K_SPARSE, 0);
    cp(3, VS + 50, VD + 4090, 4200);                          // dst: host, sparse, host; first host page then sparse
    fl_cmd(3, VD, 3 * 4096, 32'h12345678);                    // FILL over a sparse middle page
    end_test();
  endtask

  task automatic t_faults();
    int codes [5];
    codes = '{F_NOT_PRESENT, F_NON_CANONICAL, F_PERMISSION, F_BAD_PTE, F_MISALIGNED};
    begin_test("xlat_fault_src_mid");
    for (int k = 0; k < 5; k++) begin
      pt.delete(); map_range(VS, 5 * 4096, M_HOST); map_range(VD, 5 * 4096, M_C3);
      set_page(VS + 2 * 4096, K_FAULT, codes[k]);
      cp(3, VS + 7, VD + 9, 4 * 4096 + 100);
    end
    recover(); end_test();
    begin_test("xlat_fault_dst_mid");
    for (int k = 0; k < 5; k++) begin
      pt.delete(); map_range(VS, 5 * 4096, M_C3); map_range(VD, 5 * 4096, M_HOST);
      set_page(VD + 3 * 4096, K_FAULT, codes[k]);
      cp(3, VS + 1, VD + 15, 4 * 4096 + 17);
    end
    pt.delete(); map_range(VD, 5 * 4096, M_HOST); set_page(VD + 4096, K_FAULT, F_NOT_PRESENT);
    fl_cmd(3, VD + 4000, 5000, 32'hFEEDFACE);
    pt.delete(); map_range(VS, 5 * 4096, M_HOST); map_range(VD, 5 * 4096, M_HOST);
    set_page(VS, K_FAULT, F_NOT_PRESENT);                      // first page faults
    cp(3, VS + 3, VD + 3, 100);
    set_page(VD, K_FAULT, F_PERMISSION);
    cp(3, VS + 4096, VD + 3, 100);
    recover(); end_test();
  endtask

  task automatic t_badphys();
    begin_test("bad_phys_src_dst");
    pt.delete(); map_range(VS, 5 * 4096, M_HOST); map_range(VD, 5 * 4096, M_C3);
    set_page(VS + 2 * 4096, K_VRAM, 0);
    cp(3, VS + 7, VD + 9, 4 * 4096 + 100);
    pt.delete(); map_range(VS, 5 * 4096, M_C3); map_range(VD, 5 * 4096, M_HOST);
    set_page(VD + 4096, K_VRAM, 0);
    cp(3, VS, VD + 4000, 5000);
    set_page(VD, K_VRAM, 0);
    cp(3, VS, VD, 300);                                         // first page BAD_PHYS
    fl_cmd(3, VD, 4000, 32'h0F0F0F0F);
    pt.delete(); map_range(VS, 5 * 4096, M_HOST); map_range(VD, 5 * 4096, M_HOST);
    set_page(VS, K_VRAM, 0);
    cp(3, VS, VD, 4096);
    recover(); end_test();
  endtask

  task automatic t_vmid0();
    longint unsigned H, S;
    H = HOST_BIT | 48'h0000_0012_3000; S = SYS_BASE | 48'h0000_0100_0000;
    begin_test("vmid0_physical_ok");
    pt.delete();
    cp(0, H + 5, H + 64'h10_0000 + 3, 10000);           // host -> host
    cp(0, H + 64'h20_0000, S + 64'h1000 + 1, 9000);     // host -> C3
    cp(0, S + 64'h8000 + 7, H + 64'h30_0000 + 1, 9000); // C3 -> host
    cp(0, S + 64'h20000 + 7, S + 64'h30000 + 5, 9000);  // C3 -> C3
    fl_cmd(0, H + 64'h40_0000, 4100, 32'hABCD1234);
    fl_cmd(0, S + 64'h40000, 4100, 32'h4321DCBA);
    cp(0, S + 64'h7FFF_F000, H + 64'h50_0000, 4096);    // last SYS page
    cp(0, H + 64'h60_0000, S + 64'h7FFF_F000 + 8, 4088); // ends at the SYS top
    end_test();
    begin_test("vmid0_invalid_address");
    cp(0, 48'h0000_1234_5000, H + 64'h70_0000, 100);              // src invalid (low memory)
    cp(0, H + 64'h70_0000, 48'h0000_1234_5000, 100);              // dst invalid
    fl_cmd(0, 48'h0000_2000_0000, 400, 32'h1);                    // fill to invalid
    cp(0, H + 64'h70_0000, SYS_BASE + 64'h8000_0000 - 100, 300);  // dst runs past the end of SYS
    cp(0, SYS_BASE + 64'h8000_0000 - 100, H + 64'h70_0000, 300);  // src runs past the end of SYS
    cp(0, H + 64'h70_0000, 48'h0000_FFFF_F000, 5000);             // not host, not SYS
    cp(0, 48'h000F_0000_0000, 48'h7FFF_FFFF_F000, 64);            // below SYS_BASE region? (bit 47 clear)
    recover(); end_test();
  endtask

  task automatic t_abort();
    int at;
    begin_test("abort_mid_copy");
    for (int k = 0; k < 24; k++) begin
      cfg_random();
      pt.delete();
      case (k % 4)
        0: begin map_range(VS, 40000, M_HOST); map_range(VD, 40000, M_HOST); end
        1: begin map_range(VS, 40000, M_C3);   map_range(VD, 40000, M_C3);   end
        2: begin map_range(VS, 40000, M_HOST); map_range(VD, 40000, M_C3);   end
        default: begin map_range(VS, 40000, M_C3); map_range(VD, 40000, M_HOST); end
      endcase
      at = (k < 8) ? rr(1, 12) : (k < 16) ? rr(10, 150) : rr(100, 900);
      run_cmd(4, 0, VS + 3, VD + 5, 32768, 32'h0, at);
      if (k % 6 == 5) begin
        run_cmd(4, 1, 48'h0, VD, 20000, 32'h13572468, rr(1, 300));
      end
      recover();     // a normal copy works after an abort (also resets slave config)
    end
    // abort with VMID 0
    pt.delete();
    run_cmd(0, 0, SYS_BASE | 64'h1000, HOST_BIT | 64'h50_0000, 30000, 32'h0, 40);
    recover();
    end_test();
  endtask

  task automatic t_slverr();
    begin_test("slverr_on_R");
    for (int k = 0; k < 6; k++) begin
      pt.delete(); no_inj(); cfg_fast();
      case (k % 3)
        0: begin map_range(VS, 8192, M_HOST); map_range(VD, 8192, M_HOST); u_h.inj_r = (k < 3) ? 1 : 3; end
        1: begin map_range(VS, 8192, M_C3);   map_range(VD, 8192, M_HOST); u_c.inj_r = (k < 3) ? 1 : 4; end
        default: begin map_range(VS, 8192, M_C3); map_range(VD, 8192, M_C3); u_c.inj_r = (k < 3) ? 2 : 5; end
      endcase
      if (k == 2) cfg_random();
      cp(3, VS + 5, VD + 3, 8000);
      cfg_fast(); no_inj();
    end
    pt.delete(); map_range(VS, 100, M_HOST); map_range(VD, 100, M_HOST); u_h.inj_r = 1; cp(3, VS + 1, VD + 2, 5);   // single burst
    recover(); end_test();
    begin_test("slverr_on_B");
    for (int k = 0; k < 6; k++) begin
      pt.delete(); no_inj(); cfg_fast();
      case (k % 3)
        0: begin map_range(VS, 8192, M_C3);   map_range(VD, 8192, M_HOST); u_h.inj_b = (k < 3) ? 1 : 3; end
        1: begin map_range(VS, 8192, M_HOST); map_range(VD, 8192, M_C3);   u_c.inj_b = (k < 3) ? 1 : 4; end
        default: begin map_range(VS, 8192, M_HOST); map_range(VD, 8192, M_HOST); u_h.inj_b = (k < 3) ? 2 : 6; end
      endcase
      if (k == 5) cfg_random();
      cp(3, VS + 5, VD + 3, 8000);
      cfg_fast(); no_inj();
    end
    pt.delete(); map_range(VD, 20000, M_HOST); u_h.inj_b = 2; fl_cmd(3, VD, 16000, 32'h77777777);
    pt.delete(); map_range(VD, 20000, M_C3);   no_inj(); u_c.inj_b = 3; fl_cmd(3, VD, 16000, 32'h77777777);
    recover(); end_test();
  endtask

  task automatic t_busy_start();
    begin_test("start_while_busy_ignored");
    for (int k = 0; k < 3; k++) begin
      pt.delete(); map_range(VS, 20000, M_HOST); map_range(VD, 20000, M_C3);
      cfg_random();
      inj_start_at = rr(2, 40);
      cp(3, VS + 1, VD + 2, 16000);
    end
    cfg_fast(); recover(); end_test();
  endtask

  // ---------------- random loop
  function automatic int rsize();
    int r; r = rn(100);
    if (r < 40) return rr(1, 16);
    if (r < 65) return rr(17, 512);
    if (r < 85) return rr(513, 8192);
    if (r < 95) return rr(8193, 24000);
    return rr(24001, 65536);
  endfunction
  function automatic int roff();
    int r; r = rn(10);
    if (r < 3) return rn(4096);
    if (r < 5) return 4096 - rr(1, 40);
    if (r < 8) return rn(16);
    return rn(4096);
  endfunction

  task automatic t_random(int iters);
    int it, vm, sz, so, dof, kind, pgn; bit fl; longint unsigned s, d, pidx, a; int smode, dmode, r; int c0, c1;
    begin_test("random_loop");
    for (it = 0; it < iters; it++) begin
      cfg_random(); no_inj();
      pt.delete();
      fl = (rn(100) < 20);
      sz = rsize(); if (fl) sz = (sz + 3) & ~3; if (sz == 0) sz = 4;
      vm = (rn(100) < 20) ? 0 : rr(1, 15);
      so = roff(); dof = roff(); if (fl) dof &= ~3;
      if (vm != 0) begin
        s = (64'(rn(1 << 20)) + 1) << 12;
        d = (64'(rn(1 << 20)) + (1 << 21)) << 12;
        if (rn(100) < 5) d = s + so + sz + rn(200);   // same page region (non-overlapping)
        else d = d + dof;
        s = s + so;
        if (rn(100) < 5) begin          // any page offset, unless that makes the ranges overlap (undefined)
          longint unsigned d2;
          d2 = (d & ~64'hFFF) | (dof & 64'hFFF);
          if (fl || d2 >= s + sz || d2 + sz <= s) d = d2;
        end
        if (fl) d &= ~64'h3;              // FILL: dst is a multiple of 4 (the firmware rejects anything else)
        r = rn(6); smode = (r < 2) ? M_HOST : (r < 4) ? M_C3 : (r < 5) ? M_MIX : M_MIXS;
        r = rn(6); dmode = (r < 2) ? M_HOST : (r < 4) ? M_C3 : (r < 5) ? M_MIX : M_MIXS;
        if (!fl) map_range(s, sz, smode);
        map_range(d, sz, dmode);
        if (rn(100) < 8) begin     // one faulty page
          if (fl || rn(2)) begin pidx = (d >> 12) + rn(((d + sz - 1) >> 12) - (d >> 12) + 1); a = pidx << 12; end
          else begin pidx = (s >> 12) + rn(((s + sz - 1) >> 12) - (s >> 12) + 1); a = pidx << 12; end
          if (rn(2)) begin
            int fc [5]; fc = '{F_NOT_PRESENT, F_NON_CANONICAL, F_PERMISSION, F_BAD_PTE, F_MISALIGNED};
            set_page(a, K_FAULT, fc[rn(5)]);
          end else set_page(a, K_VRAM, 0);
        end
      end else begin
        // physical addresses: host / SYS, occasionally invalid
        r = rn(4);
        s = (r < 2) ? (HOST_BIT | (64'(rn(1 << 20)) << 12)) : (SYS_BASE + ((64'(rn(1 << 18))) << 12));
        r = rn(4);
        d = (r < 2) ? (HOST_BIT | ((64'(rn(1 << 20)) + (1 << 21)) << 12)) : (SYS_BASE + ((64'(rn(1 << 18)) + (1 << 18)) << 12));
        s = s + so; d = d + dof;
        if (rn(100) < 6) begin
          r = rn(3);
          if (r == 0) s = 48'h0000_1234_5000 + so;
          else if (r == 1) d = 48'h0000_2000_0000 + dof;
          else d = SYS_BASE + 64'h8000_0000 - rr(1, sz) - (fl ? 0 : 0);
          if (fl) d &= ~64'h3;
        end
      end
      run_cmd(vm, fl, s, d, sz, {rnd64()}, -1);
    end
    cfg_fast();
    end_test();
  endtask

  // --------------
  int seed_v = 1, iters_v = 150;
  initial begin
    if (!$value$plusargs("seed=%d", seed_v)) seed_v = 1;
    if (!$value$plusargs("iters=%d", iters_v)) iters_v = 150;
    begin   // xsim.bat splits "seed=N" arguments, so the runner passes them in params.txt ("seed iters") instead
      int fd, a, b;
      fd = $fopen("params.txt", "r");
      if (fd != 0) begin if ($fscanf(fd, "%d %d", a, b) == 2) begin seed_v = a; iters_v = b; end $fclose(fd); end
    end
    rs = 64'h1234567 + longint'(seed_v) * 64'h9E3779B97F4A7C15; if (rs == 0) rs = 1;
    repeat (4) void'(rnd64());
    u_h.seed_rng(longint'(seed_v) * 3 + 1); u_c.seed_rng(longint'(seed_v) * 5 + 2);
    $display("tb_rcp_dma: seed=%0d iters=%0d", seed_v, iters_v);
    rst_n = 0; repeat (6) @(negedge clk); rst_n = 1; repeat (4) @(negedge clk);
    cfg_fast();
    t_basic(); t_cross(); t_routes(); t_fill(); t_sparse(); t_faults(); t_badphys(); t_vmid0();
    t_abort(); t_slverr(); t_busy_start();
    // directed tests again under random back-pressure / latency
    cfg_random(); begin_test("backpressure_copy_mix");
    pt.delete(); map_range(VS, 40000, M_MIX); map_range(VD, 40000, M_MIX);
    for (int k = 0; k < 6; k++) begin cfg_random(); cp(2, VS + rn(300), VD + rn(300), 20000 + rn(5000)); end
    cfg_fast(); end_test();
    t_random(iters_v);
    $display("SUMMARY: %0d tests, %0d failed, %0d commands, %0d check failures", tests_run, tests_failed, total_cmds, total_fail);
    if (total_fail == 0 && tests_failed == 0) $display("NG_RCP_DMA_TB_PASS");
    else $display("NG_RCP_DMA_TB_FAIL");
    $finish;
  end

  // global watchdog (simulation time)
  initial begin
    #(64'd4_000_000_000);
    $display("TB_FAIL global watchdog"); $display("NG_RCP_DMA_TB_FAIL"); $finish;
  end
endmodule
