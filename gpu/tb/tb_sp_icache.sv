// tb_sp_icache.sv -- unit test of ng_sp_icache against a reference model.
//
// One requester (the scheduler's fetch port: one fetch at a time, random gaps), an MMU stub and a fabric stub with
// random ready / latency, a reference model of the cache organisation (fully associative FIFO or set associative
// tree pseudo-LRU, re-implemented here) and a reference memory:
//   * "dispatches": each picks a random page-aligned CODE_BASE, a random VA -> PA page map over a window of NPG + 1
//     code pages (normal VRAM / host pages, sparse-null pages, MMU faults 1..3, PAs outside the physical map) and
//     invalidates the cache (inv, as the WD does at dispatch start);
//   * fetch streams: loops of 1..70 lines (sequential pcs, 64-bit instructions, straddling line / page ends),
//     random jumps, and repeats; icache.inv pulses at random cycles (also with a fill in flight / installing);
//   * fabric read errors injected at random; asynchronous resets at random points (a transaction in flight
//     is dropped by the stubs).
// Checks (every fetch, every cycle):
//   * response: {w1, w0} = reference memory (w1 only for a 64-bit w0; sparse = 0), fault flag and info (= low 32
//     bits of the faulting word's VA) for MMU faults, BAD_PHYS_ADDR and fabric errors;
//   * every lookup (S_LK cycle): address = the expected word, hit / miss = the reference model's prediction
//     (replacement exactness), hit word = reference memory;
//   * every miss: one MMU request (VA = the word, access Exec) and, if translated, one M_RD of the right line PA
//     (uc = outside VRAM); nothing else on either port;
//   * ext_flt_*: one single-cycle report exactly in the response cycle of a fetch that failed AFTER translation
//     (translated PA outside the physical map, or a fabric read error): vmid 5, access Exec, VA = the faulting
//     word's VA; none for MMU-detected faults (ng_mmu latches those itself), none for successful fetches ;
//   * latency: a fetch without a miss takes 3 (32-bit) / 4 (64-bit) cycles: accept, lookup(s), respond;
//   * stat_hit / stat_miss = the reference counts (after every fetch).
// Compile-time configuration: tb_sp_icache_cfg.svh (NG_TB_IC_LINES, NG_TB_IC_WAYS, NG_TB_SEED, NG_TB_FETCHES).
`timescale 1ns/1ps
`include "tb_sp_icache_cfg.svh"
module tb_sp_icache;
  import ng_lsu_pkg::*;

  localparam int LINES   = `NG_TB_IC_LINES;
  localparam int WAYS    = `NG_TB_IC_WAYS;
  localparam int NFETCH  = `NG_TB_FETCHES;
  localparam int WE      = (WAYS > 0) ? ((WAYS < LINES) ? WAYS : LINES) : ((LINES <= 8) ? LINES : 4);
  localparam bit FA      = (WE >= LINES);
  localparam int SETS    = FA ? 1 : LINES / WE;
  localparam int NW      = FA ? LINES : WE;     // ways of the model
  localparam int NPG     = 12;                  // code window pages (+1 for a w1 past the end)

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // --------------------------------------------------------------
  logic          inv;
  logic [47:0]   code_base;
  logic          req_valid, req_ready, rsp_valid, rsp_fault;
  logic [31:0]   req_pc, rsp_fault_info;
  logic [63:0]   rsp_data;
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
  logic          busy;
  logic [31:0]   stat_hit, stat_miss;

  ng_sp_icache #(.LINES(LINES), .WAYS(WAYS)) dut (
    .clk(clk), .rst_n(rst_n), .inv(inv), .code_base(code_base), .vmid(4'd5),
    .req_valid(req_valid), .req_ready(req_ready), .req_pc(req_pc), .rsp_valid(rsp_valid), .rsp_fault(rsp_fault),
    .rsp_data(rsp_data), .rsp_fault_info(rsp_fault_info),
    .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
    .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic), .mmu_req_tag(mmu_req_tag),
    .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready), .mmu_resp_fault(mmu_resp_fault),
    .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
    .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
    .mem_req_valid(mem_req_valid), .mem_req_ready(mem_req_ready), .mem_req(mem_req),
    .mem_rsp_valid(mem_rsp_valid), .mem_rsp(mem_rsp),
    .busy(busy), .stat_hit(stat_hit), .stat_miss(stat_miss));

  // ---------------------------------------------------------------- reference memory / page map
  int unsigned seed = `NG_TB_SEED;
  int          errors = 0;
  // page attribute: 0 normal, 1 sparse, 2..4 MMU fault 1..3, 5 PA outside the physical map
  int          pg_attr [NPG + 1];
  logic [35:0] pg_pa   [NPG + 1];
  int          disp_id = 0;

  function automatic int unsigned rnd(input int unsigned n);   // 0..n-1
    return (n == 0) ? 0 : ($urandom % n);
  endfunction

  function automatic logic [31:0] pa_word(input logic [47:0] pa);
    logic [31:0] h;
    h = pa[31:0] ^ {pa[47:32], 16'h9e37} ^ (32'h85eb_ca6b * (pa[33:2] + 32'(disp_id)));
    h = h ^ (h >> 15);
    h = h * 32'h2c1b_3c6d;
    h = h ^ (h >> 12);
    // about one word in four encodes a 64-bit instruction (w0[1:0] == 01)
    return (h[9:8] == 2'b00) ? {h[31:2], 2'b01} : {h[31:2], (h[1:0] == 2'b01) ? 2'b11 : h[1:0]};
  endfunction

  function automatic int va_page(input logic [63:0] va);   // window page of a VA, -1 outside
    longint d;
    d = longint'(va[48:12]) - longint'(code_base[47:12]);
    return (d >= 0 && d <= NPG) ? int'(d) : -1;
  endfunction

  function automatic logic [47:0] va_pa(input logic [63:0] va);
    int p;
    p = va_page(va);
    return {pg_pa[p], va[11:0]};
  endfunction

  function automatic logic [31:0] va_word(input logic [63:0] va);   // reference instruction word (mapped pages)
    int p;
    p = va_page(va);
    return (pg_attr[p] == 1) ? 32'd0 : pa_word(va_pa({va[63:2], 2'b00}));
  endfunction

  task automatic new_dispatch();
    disp_id++;
    code_base <= {12'h000 + 12'(rnd(16)), 24'(rnd(1 << 24)), 12'h000};
    for (int p = 0; p <= NPG; p++) begin
      int r;
      r = rnd(100);
      pg_attr[p] = (r < 80) ? 0 : (r < 88) ? 1 : (r < 90) ? 2 : (r < 92) ? 3 : (r < 94) ? 4 : (r < 96) ? 5 : 0;
      case (rnd(3))
        0: pg_pa[p] = 36'(rnd(1 << 22));                                   // VRAM (< 16 GiB)
        1: pg_pa[p] = {16'h8000, 20'(rnd(1 << 20))};                       // host (PA[47])
        default: pg_pa[p] = {16'h0100, 20'(rnd(1 << 19))};                 // system region 0x10_0000_0000..
      endcase
      if (pg_attr[p] == 5) pg_pa[p] = {16'h0050, 20'(rnd(1 << 20))};      // 0x50_0000_0000: unmapped PA
    end
  endtask

  // ---------------------------------------------------------------- reference cache model
  longint      m_tag [SETS][NW];      // line VA >> 7 (FA) / full line number (SA)
  bit          m_val [SETS][NW];
  int          m_fifo;
  bit          m_tree [SETS][2 * NW]; // node n (1..NW-1): 1 = victim on the right
  longint      m_hits = 0, m_misses = 0, t_hits = 0, t_misses = 0;   // since reset / in total
  int          inv_rate = 4;                // icache.inv pulses per 100000 cycles (per dispatch)

  function automatic int m_set(input longint line);
    return FA ? 0 : int'(line % SETS);
  endfunction
  function automatic int m_find(input longint line);
    int s;
    s = m_set(line);
    for (int w = 0; w < NW; w++) if (m_val[s][w] && m_tag[s][w] == line) return w;
    return -1;
  endfunction
  task automatic m_touch(input int s, input int w);
    int node, lo, sz;
    node = 1; lo = 0; sz = NW;
    while (sz > 1) begin
      sz = sz / 2;
      if (w < lo + sz) begin m_tree[s][node] = 1'b1; node = 2 * node; end
      else begin m_tree[s][node] = 1'b0; node = 2 * node + 1; lo = lo + sz; end
    end
  endtask
  task automatic m_install(input longint line);
    int s, v, node, lo, sz;
    s = m_set(line);
    if (FA) begin
      v = m_fifo;
      m_fifo = (m_fifo + 1) % LINES;
    end else begin
      v = -1;
      for (int w = 0; w < NW; w++) if (!m_val[s][w] && v < 0) v = w;
      if (v < 0) begin
        node = 1; lo = 0; sz = NW;
        while (sz > 1) begin
          sz = sz / 2;
          if (m_tree[s][node]) begin node = 2 * node + 1; lo = lo + sz; end else node = 2 * node;
        end
        v = lo;
      end
      m_touch(s, v);
    end
    m_val[s][v] = 1'b1;
    m_tag[s][v] = line;
  endtask
  task automatic m_reset();
    for (int s = 0; s < SETS; s++) for (int w = 0; w < NW; w++) begin m_val[s][w] = 1'b0; m_tree[s][w] = 1'b0; m_tree[s][w + NW] = 1'b0; end
    m_fifo = 0;
  endtask
  task automatic m_inv();
    for (int s = 0; s < SETS; s++) for (int w = 0; w < NW; w++) m_val[s][w] = 1'b0;
  endtask

  // ---------------------------------------------------------------- stubs (posedge state machines, NBA only)
  // MMU: one translation; fabric: one line read.  Responses after 0..MAXLAT idle cycles.
  int          mmu_st = 0, mmu_wait = 0, mem_st = 0, mem_wait = 0;
  logic [63:0] mmu_va_q;
  logic [47:0] mem_pa_q;
  bit          mem_err_q;
  longint      n_mmu = 0, n_mem = 0, n_err = 0, n_inv = 0, n_rst = 0, n_flt = 0, n_sparse = 0, n_xflt = 0, n_xflt_bp = 0;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mmu_st <= 0; mmu_req_ready <= 1'b0; mmu_resp_valid <= 1'b0;
      mmu_resp_fault <= '0; mmu_resp_sparse <= 1'b0; mmu_resp_pa <= '0;
      mem_st <= 0; mem_req_ready <= 1'b0; mem_rsp_valid <= 1'b0; mem_rsp <= '0;
    end else begin
      // MMU
      case (mmu_st)
        0: begin
          mmu_req_ready <= (rnd(3) != 0);
          if (mmu_req_valid && mmu_req_ready) begin
            mmu_req_ready <= 1'b0; mmu_va_q <= mmu_req_va; mmu_wait <= rnd(6); mmu_st <= 1;
          end
        end
        1: if (mmu_wait > 0) mmu_wait <= mmu_wait - 1;
           else begin
             int p;
             p = va_page(mmu_va_q);
             mmu_resp_valid  <= 1'b1;
             mmu_resp_fault  <= (p < 0) ? 3'd1 : (pg_attr[p] >= 2 && pg_attr[p] <= 4) ? 3'(pg_attr[p] - 1) : 3'd0;
             mmu_resp_sparse <= (p >= 0) && (pg_attr[p] == 1);
             mmu_resp_pa     <= (p >= 0) ? {pg_pa[p], 12'(rnd(4096))} : '0;   // the I$ must use PA[47:12] only
             mmu_st <= 2;
           end
        2: if (mmu_resp_ready) begin mmu_resp_valid <= 1'b0; mmu_st <= 0; end
      endcase
      // fabric
      case (mem_st)
        0: begin
          mem_req_ready <= (rnd(3) != 0);
          if (mem_req_valid && mem_req_ready) begin
            mem_req_ready <= 1'b0; mem_pa_q <= mem_req.pa; mem_wait <= rnd(9); mem_err_q <= (rnd(100) < 4);
            mem_st <= 1;
          end
        end
        1: if (mem_wait > 0) mem_wait <= mem_wait - 1;
           else begin
             mrsp_t r;
             r = '0;
             for (int k = 0; k < 32; k++) r.rdata[32*k +: 32] = pa_word({mem_pa_q[47:7], 5'(k), 2'b00});
             r.err = mem_err_q ? 2'(1 + rnd(3)) : 2'd0;
             if (mem_err_q) r.rdata = {32{32'hdead_beef}};
             mem_rsp <= r; mem_rsp_valid <= 1'b1; mem_st <= 2;
           end
        2: begin mem_rsp_valid <= 1'b0; mem_rsp <= '0; mem_st <= 0; end    // no ready: the I$ takes it in one cycle
      endcase
    end
  end

  // ---------------------------------------------------------------- requester + checker
  // expectation of the fetch in flight
  bit          f_act = 0;          // a fetch is in flight (accepted, no response yet)
  logic [63:0] f_va0;
  bit          f_ph;               // next lookup is w1
  logic [31:0] f_w0;
  int          f_edges;            // edges since accept
  bit          f_missed;           // a miss happened in this fetch
  bit          f_exp_fault;
  bit          f_exp_xflt;         // the fault is a post-translation one (BAD_PHYS_ADDR / fabric error): ext_flt report due
  logic [31:0] f_exp_info;
  int          f_stage = 0;        // 0 lookup, 1 waiting MMU request, 2 waiting MMU resp, 3 waiting fabric req,
                                   // 4 waiting fabric rsp, 5 response due
  logic [63:0] f_lva;              // VA of the word of the current lookup / miss
  logic [47:0] f_lpa;
  longint      n_fetch = 0, n_rsp = 0, n_hitf = 0;
  int          gap = 0;
  // fetch stream generator
  int          s_kind = 0, s_left = 0, s_lo = 0, s_len = 0, s_pc = 0, s_rep = 0;
  bit          inv_now;

  function automatic int next_pc();
    int pc;
    if (s_left <= 0) begin
      s_kind = rnd(10);
      if (s_kind < 6) begin               // loop of 1..70 lines, 2..6 times
        s_len = 1 + rnd(70);
        s_lo  = 128 * rnd(NPG * 32 - s_len + 1) + 4 * rnd(4);
        s_rep = 2 + rnd(5);
        s_left = s_rep * s_len * 32;
        s_pc  = s_lo;
      end else if (s_kind < 9) begin      // random jumps
        s_left = 1 + rnd(40);
      end else begin                      // line / page ends (straddling 64-bit instructions)
        s_left = 1 + rnd(20);
      end
    end
    s_left--;
    if (s_kind < 6) begin
      pc = s_pc;
      s_pc += (rnd(4) == 0) ? 8 : 4;
      if (s_pc >= s_lo + 128 * s_len) s_pc = s_lo;
    end else if (s_kind < 9) begin
      pc = 4 * rnd(NPG * 1024);
    end else begin
      pc = (rnd(2) ? 4096 * (1 + rnd(NPG - 1)) : 128 * (1 + rnd(NPG * 32 - 1))) - 4;
    end
    return pc;
  endfunction

  task automatic err(input string m);
    errors++;
    if (errors <= 20) $display("TB_ERROR t=%0t fetch %0d: %s", $time, n_fetch, m);
  endtask

  always @(posedge clk) begin
    if (rst_n) begin
      // ---- requester (NBA drives)
      inv_now = inv;
      if (req_valid && req_ready) begin
        req_valid <= 1'b0;
        f_act = 1; f_va0 = {16'd0, code_base} + {32'd0, req_pc}; f_ph = 0; f_edges = 0; f_missed = 0;
        f_exp_fault = 0; f_exp_xflt = 0; f_exp_info = 0; f_stage = 0;
        n_fetch++;
      end else if (!f_act && !req_valid) begin
        if (gap > 0) gap--;
        else begin req_valid <= 1'b1; req_pc <= next_pc(); gap = rnd(4); end
      end
      if (f_act) f_edges++;

      // ---- lookup cycle
      if (int'(dut.st) == 1) begin                    // S_LK
        longint line;
        int     p;
        bit     mh;
        if (!f_act || f_stage != 0) err("lookup without a fetch expecting one");
        f_lva = f_va0 + (f_ph ? 64'd4 : 64'd0);
        if (dut.fva !== f_lva) err($sformatf("lookup VA %h, expected %h", dut.fva, f_lva));
        line = longint'(f_lva[48:7]);
        mh = (m_find(line) >= 0);
        if (dut.hit !== mh) err($sformatf("lookup of line %h: dut hit %0d, model %0d", line, dut.hit, mh));
        if (mh) begin
          logic [31:0] wexp;
          wexp = va_word(f_lva);
          if (dut.hword !== wexp) err($sformatf("hit word at %h: %h, expected %h", f_lva, dut.hword, wexp));
          if (!FA) m_touch(m_set(line), m_find(line));
          if (!f_ph) begin m_hits++; t_hits++; end
          if (!f_ph && wexp[1:0] == 2'b01) begin f_w0 = wexp; f_ph = 1; end
          else begin
            f_stage = 5;
          end
        end else begin
          m_misses++; t_misses++;
          f_missed = 1;
          f_stage = 1;
        end
      end
      // ---- MMU request / response
      if (mmu_req_valid && mmu_req_ready) begin
        n_mmu++;
        if (f_stage != 1) err("unexpected MMU request");
        if (mmu_req_va !== f_lva || mmu_req_acc != 2'd2 || mmu_req_atomic)
          err($sformatf("MMU request va %h acc %0d, expected %h Exec", mmu_req_va, mmu_req_acc, f_lva));
        f_stage = 2;
      end
      if (mmu_resp_valid && mmu_resp_ready) begin
        int p;
        if (f_stage != 2) err("MMU response without a request");
        p = va_page(f_lva);
        if (mmu_resp_fault != 0) begin
          f_exp_fault = 1; f_exp_info = f_lva[31:0]; f_stage = 5; n_flt++;
        end else if (mmu_resp_sparse) begin
          m_install(longint'(f_lva[48:7])); f_stage = 0; n_sparse++;
        end else if (!pa_valid({mmu_resp_pa[47:12], f_lva[11:0]})) begin
          f_exp_fault = 1; f_exp_xflt = 1; f_exp_info = f_lva[31:0]; f_stage = 5; n_flt++; n_xflt_bp++;
        end else begin
          f_lpa = {mmu_resp_pa[47:12], f_lva[11:7], 7'd0}; f_stage = 3;
        end
      end
      if (mem_req_valid && mem_req_ready) begin
        n_mem++;
        if (f_stage != 3) err("unexpected fabric request");
        if (mem_req.kind != M_RD || mem_req.pa !== f_lpa || mem_req.uc != !pa_is_vram(f_lpa))
          err($sformatf("fabric request kind %0d pa %h uc %0d, expected M_RD %h", mem_req.kind, mem_req.pa,
                        mem_req.uc, f_lpa));
        f_stage = 4;
      end
      if (mem_rsp_valid) begin
        if (f_stage != 4) err("fabric response without a request");
        if (mem_rsp.err != 0) begin
          f_exp_fault = 1; f_exp_xflt = 1; f_exp_info = f_lva[31:0]; f_stage = 5; n_err++;
        end else begin
          m_install(longint'(f_lva[48:7])); f_stage = 0;
        end
      end
      // ---- invalidation (after a same-cycle install: inv wins)
      if (inv_now) begin m_inv(); n_inv++; end
      // ---- external fault report: exactly in the response cycle of a post-translation fault
      if (ext_flt_valid !== (rsp_valid && f_exp_xflt))
        err($sformatf("ext_flt_valid %b with rsp_valid %b, expected post-translation fault %0d", ext_flt_valid, rsp_valid, f_exp_xflt));
      if (ext_flt_valid === 1'b1) begin
        n_xflt++;
        if (ext_flt_vmid !== 4'd5 || ext_flt_acc !== 2'd2 || ext_flt_va !== f_lva)
          err($sformatf("ext_flt vmid %0d acc %0d va %h, expected 5 / Exec / %h", ext_flt_vmid, ext_flt_acc, ext_flt_va, f_lva));
      end
      // --
      if (rsp_valid) begin
        logic [63:0] dexp;
        n_rsp++;
        if (!f_act || f_stage != 5) err("unexpected response");
        if (rsp_fault !== f_exp_fault) err($sformatf("fault %0d, expected %0d", rsp_fault, f_exp_fault));
        if (f_exp_fault) begin
          if (rsp_fault_info !== f_exp_info) err($sformatf("fault info %h, expected %h", rsp_fault_info, f_exp_info));
          s_left = 0;                               // a faulting warp stops: next stream
        end else begin
          logic [31:0] w0;
          w0 = va_word(f_va0);
          dexp = (w0[1:0] == 2'b01) ? {va_word(f_va0 + 64'd4), w0} : {32'd0, w0};
          if (rsp_data !== dexp) err($sformatf("data %h, expected %h (va %h)", rsp_data, dexp, f_va0));
          if (!f_missed) begin
            n_hitf++;
            if (f_edges != ((w0[1:0] == 2'b01) ? 4 : 3)) err($sformatf("hit latency %0d edges", f_edges));
          end
        end
        f_act = 0;
      end
      if (stat_hit != 32'(m_hits) || stat_miss != 32'(m_misses))
        if (!f_act) err($sformatf("stat hit %0d miss %0d, model %0d %0d", stat_hit, stat_miss, m_hits, m_misses));
    end
  end

  // inv pulses: random single cycles (driven at negedge so they are stable for the edge)
  always @(negedge clk) inv <= rst_n && (rnd(100000) < inv_rate);

  // --------------------------------------------------------------
  initial begin
    seed = $urandom(seed);
    inv = 1'b0; req_valid = 1'b0; req_pc = '0; code_base = '0;
    for (int p = 0; p <= NPG; p++) begin pg_attr[p] = 0; pg_pa[p] = '0; end
    m_reset();
    repeat (3) @(posedge clk);
    #1 rst_n = 1'b1;
    while (n_rsp < NFETCH) begin
      // a dispatch: wait until idle, new map + invalidation, then a few thousand fetches
      int nf;
      @(negedge clk);
      while (f_act || req_valid) @(negedge clk);
      gap = 1000000;                                  // hold the requester
      @(negedge clk);
      while (f_act || req_valid) @(negedge clk);
      new_dispatch();
      case (rnd(4)) 0: inv_rate = 0; 1: inv_rate = 2; 2: inv_rate = 20; default: inv_rate = 300; endcase
      @(negedge clk);
      force inv = 1'b1;
      @(negedge clk);
      release inv;
      gap = 0;
      nf = 500 + rnd(4000);
      while (nf > 0 && n_rsp < NFETCH) begin
        @(negedge clk);
        if (rsp_valid) nf--;
        // occasional asynchronous reset in the middle of anything
        if (rnd(200000) == 0) begin
          rst_n = 1'b0;
          n_rst++;
          f_act = 0; f_stage = 0; req_valid = 1'b0; gap = 2;
          m_reset(); m_hits = 0; m_misses = 0;
          #3 rst_n = 1'b1;
        end
      end
    end
    $display("NOTE LINES %0d WAYS %0d (%s, %0d sets x %0d ways): fetches %0d (no-miss %0d), lookups hit %0d miss %0d, mmu %0d (faults %0d, sparse %0d), fabric %0d (errors %0d), inv %0d, resets %0d, dispatches %0d, ext_flt reports %0d (bad PA %0d)",
             LINES, WAYS, FA ? "fully associative FIFO" : "set associative PLRU", SETS, NW, n_rsp, n_hitf, t_hits,
             t_misses, n_mmu, n_flt, n_sparse, n_mem, n_err, n_inv, n_rst, disp_id, n_xflt, n_xflt_bp);
    if (NFETCH >= 2000 && (n_xflt_bp == 0 || n_xflt <= n_xflt_bp)) err("no bad-PA / fabric-error fetch fault was exercised");
    if (errors == 0 && n_rsp >= NFETCH) $display("NG_SP_ICACHE_TB_PASS");
    else $display("NG_SP_ICACHE_TB_FAIL errors %0d", errors);
    $finish;
  end
endmodule
