// tb_lsu_l1.sv -- ng_l1d semantics: write-through, non-coherent, cpol policy, inv / wb.
//
// Behavioural fabric (byte-addressed associative array + the real L2 AMO ALU)
// behind a small L1D (4 sets x 2 ways) so evictions happen constantly.  The TB
// keeps a shadow image of every line the L1 is allowed to hold stale
// (`img`), updated only by the rules of the spec (fill = memory at fill time,
// own write = merge on hit, atomic / inv = invalidate), and uses read-only
// hierarchical peeks of the valid/tag/data arrays only to know WHICH lines are
// resident (replacement is the DUT's business) -- data of resident lines must
// equal the shadow, so staleness vs the fabric is checked exactly:
//   * def read hit returns the (possibly stale) shadow, miss returns memory;
//   * l2 / sys policy and uc always read the fabric (fresh), never allocate;
//   * nt never allocates; mreq.noalloc never allocates;
//   * write: memory updated (write-through), resident line merged, miss does
//     not allocate; atomic: executed at the fabric, resident line invalidated;
//   * cache.inv.l1 empties the cache; cache.inv.l2 / wb.l2 are forwarded exactly
//     once and leave the L1 untouched.
// Directed D1..D9 pin the headline behaviours, then ~20000 random operations.
`timescale 1ns / 1ps

module tb_lsu_l1;
  import ng_lsu_pkg::*;
  parameter int SETS = 4;
  parameter int WAYS = 2;
  parameter int MAXLAT = 3;
  localparam logic [4:0] AMO_ADD_L = 5'd0;
  localparam int SET_LB = $clog2(SETS);

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic     req_valid = 1'b0, req_ready;
  mreq_t    req = '0;
  logic     rsp_valid;
  mrsp_t    rsp;
  logic     mem_req_valid, mem_req_ready = 1'b0, mem_rsp_valid = 1'b0;
  mreq_t    mem_req;
  mrsp_t    mem_rsp = '0;
  logic [31:0] st_hit, st_miss, st_fill, st_inv;

  ng_l1d #(.SETS(SETS), .WAYS(WAYS)) dut (
    .clk(clk), .rst_n(rst_n), .ext_inv(1'b0), .req_valid(req_valid), .req_ready(req_ready), .req(req),
    .rsp_valid(rsp_valid), .rsp(rsp),
    .mem_req_valid(mem_req_valid), .mem_req_ready(mem_req_ready), .mem_req(mem_req),
    .mem_rsp_valid(mem_rsp_valid), .mem_rsp(mem_rsp),
    .stat_hit(st_hit), .stat_miss(st_miss), .stat_fill(st_fill), .stat_inv(st_inv)
  );

  // ----------------------------------------------------------
  logic [7:0] fmem [longint unsigned];
  int  n_f_rd = 0, n_f_wr = 0, n_f_amo = 0, n_f_inv2 = 0, n_f_wb2 = 0, n_f_uc = 0;
  logic [4:0]  amo_op_s;  logic amo64_s;
  logic [63:0] amo_a_s, amo_b_s, amo_cmp_s, amo_r_s;  logic amo_hit_s;
  ng_l2_amo_alu u_alu (.op(amo_op_s), .is64(amo64_s), .a(amo_a_s), .b(amo_b_s), .cmp(amo_cmp_s), .r(amo_r_s), .cas_hit(amo_hit_s));

  function automatic logic [7:0] frd(input longint unsigned a);
    return fmem.exists(a) ? fmem[a] : 8'd0;
  endfunction

  logic [31:0] lfsr = 32'h13579bdf;
  function automatic int rnd(input int n);
    lfsr = lfsr ^ (lfsr << 13); lfsr = lfsr ^ (lfsr >> 17); lfsr = lfsr ^ (lfsr << 5);
    return (n <= 1) ? 0 : int'(lfsr % n);
  endfunction

  typedef enum int {F_IDLE, F_ACC, F_LAT} fst_t;
  fst_t fst = F_IDLE;
  mreq_t f_rq;  mrsp_t f_rs;  int f_lat;
  always @(negedge clk) begin
    mem_req_ready <= 1'b0;
    mem_rsp_valid <= 1'b0;
    case (fst)
      F_IDLE: if (mem_req_valid) begin f_rq = mem_req; mem_req_ready <= 1'b1; fst <= F_ACC; end
      F_ACC: begin
        f_rs = '0;
        case (f_rq.kind)
          M_RD: begin
            n_f_rd++; if (f_rq.uc) n_f_uc++;
            for (int b = 0; b < 128; b++) f_rs.rdata[8*b +: 8] = frd(longint'(f_rq.pa) + b);
          end
          M_WR: begin
            n_f_wr++; if (f_rq.uc) n_f_uc++;
            for (int b = 0; b < 128; b++) if (f_rq.be[b]) fmem[longint'(f_rq.pa) + b] = f_rq.wdata[8*b +: 8];
          end
          M_AMO: begin
            logic [63:0] o; int nb;
            n_f_amo++; if (f_rq.uc) n_f_uc++;
            nb = f_rq.amo64 ? 8 : 4; o = '0;
            for (int b = 0; b < nb; b++) o[8*b +: 8] = frd(longint'(f_rq.pa) + b);
            amo_op_s = f_rq.amo_op; amo64_s = f_rq.amo64; amo_a_s = o; amo_b_s = f_rq.amo_a; amo_cmp_s = f_rq.amo_cmp;
            #1;
            for (int b = 0; b < nb; b++) fmem[longint'(f_rq.pa) + b] = amo_r_s[8*b +: 8];
            f_rs.old = o;
          end
          M_INV_L2: n_f_inv2++;
          M_WB_L2:  n_f_wb2++;
          default: ;
        endcase
        f_lat = rnd(MAXLAT + 1);
        if (f_lat == 0) begin mem_rsp <= f_rs; mem_rsp_valid <= 1'b1; fst <= F_IDLE; end
        else fst <= F_LAT;
      end
      F_LAT: begin
        f_lat = f_lat - 1;
        if (f_lat <= 0) begin mem_rsp <= f_rs; mem_rsp_valid <= 1'b1; fst <= F_IDLE; end
      end
      default: fst <= F_IDLE;
    endcase
  end

  // ------------------------------------------------------------ shadow + peeks
  logic [1023:0] img [longint unsigned];     // shadow of lines the L1 may hold (possibly stale)
  int errors = 0, checks = 0;
  int e_hit = 0, e_miss = 0, e_fill = 0, e_inv = 0;
  int ops_done = 0;

  function automatic int set_of(input longint unsigned line);   // line = pa >> 7
    return int'(line % SETS);
  endfunction
  function automatic bit resident(input longint unsigned line);
    int s;
    s = set_of(line);
    for (int w = 0; w < WAYS; w++)
      if (dut.val_a[w][s] && dut.tag_a[w][s] == (line >> SET_LB)) return 1;
    return 0;
  endfunction
  function automatic logic [1023:0] peek_data(input longint unsigned line);
    int s;
    s = set_of(line);
    for (int w = 0; w < WAYS; w++)
      if (dut.val_a[w][s] && dut.tag_a[w][s] == (line >> SET_LB)) return dut.data_a[w][s];
    return '0;
  endfunction
  function automatic logic [1023:0] mem_line(input longint unsigned line);
    logic [1023:0] d;
    for (int b = 0; b < 128; b++) d[8*b +: 8] = frd((line << 7) + b);
    return d;
  endfunction
  function automatic int n_resident();
    int n; n = 0;
    for (int w = 0; w < WAYS; w++) for (int s = 0; s < SETS; s++) if (dut.val_a[w][s]) n++;
    return n;
  endfunction

  // every resident line must equal its shadow; every shadowed line either resident or dropped
  task automatic audit(input string where_);
    foreach (img[ln]) begin
      if (resident(ln)) begin
        checks++;
        if (peek_data(ln) !== img[ln]) begin
          errors++; if (errors < 12) $display("FAIL(%s): resident line %h data != shadow", where_, ln);
        end
      end
    end
    // residency count must match shadow-resident count
    begin
      int cnt;
      cnt = 0;
      foreach (img[ln]) if (resident(ln)) cnt++;
      checks++;
      if (cnt != n_resident()) begin
        errors++; if (errors < 12) $display("FAIL(%s): %0d resident lines in DUT but %0d known to the shadow", where_, n_resident(), cnt);
      end
    end
  endtask

  // drop shadow lines the DUT no longer holds (evictions, invalidations); keys are collected first,
  // never delete while iterating an associative array
  task automatic prune_shadow();
    longint unsigned keys [$];
    foreach (img[ln]) keys.push_back(ln);
    foreach (keys[i]) if (!resident(keys[i])) img.delete(keys[i]);
  endtask

  // ----------------------------------------------------------
  task automatic issue(input mreq_t r, output mrsp_t p);
    int guard;
    guard = 0;
    while (!req_ready) @(negedge clk);
    req <= r; req_valid <= 1'b1;
    @(negedge clk);
    req_valid <= 1'b0;
    guard = 0;
    while (!rsp_valid && guard < 2000) begin @(negedge clk); guard++; end
    if (guard >= 2000) begin errors++; $display("FAIL: response timeout"); end
    p = rsp;
    @(negedge clk);
  endtask

  function automatic mreq_t mk(input logic [2:0] kind, input longint unsigned pa, input logic [1:0] pol, input bit uc, input bit na);
    mreq_t r;
    r = '0; r.kind = kind; r.pa = 48'(pa); r.pol = pol; r.uc = uc; r.noalloc = na;
    return r;
  endfunction

  // ---- operations with checking
  task automatic op_rd(input longint unsigned line, input logic [1:0] pol, input bit uc, input bit na, input string tag);
    mreq_t r; mrsp_t p;
    bit pres, cacheable_lookup, will_alloc;
    logic [1023:0] expv;
    pres = resident(line);
    cacheable_lookup = !uc && pol < 2'd2;
    will_alloc = cacheable_lookup && !na && pol != 2'd1;
    if (cacheable_lookup && pres) begin expv = img[line]; e_hit++; end
    else begin expv = mem_line(line); if (cacheable_lookup) e_miss++; end
    r = mk(M_RD, line << 7, pol, uc, na);
    issue(r, p);
    checks++;
    if (p.rdata !== expv) begin
      errors++; if (errors < 12) $display("FAIL(%s): RD line %h pol=%0d uc=%0b na=%0b resident=%0b: data mismatch (stale rule)", tag, line, pol, uc, na, pres);
    end
    if (cacheable_lookup && !pres) begin
      if (will_alloc) begin img[line] = expv; e_fill++; end
    end
    // residency afterwards
    checks++;
    if (will_alloc && !pres && !resident(line)) begin errors++; if (errors < 12) $display("FAIL(%s): RD miss did not allocate line %h", tag, line); end
    if (!will_alloc && !pres && resident(line)) begin errors++; if (errors < 12) $display("FAIL(%s): non-allocating RD allocated line %h", tag, line); end
    // eviction of other lines: shadow follows residency
    prune_shadow();
    ops_done++;
    audit(tag);
  endtask

  task automatic op_wr(input longint unsigned line, input logic [127:0] be, input logic [1023:0] data, input logic [1:0] pol,
                       input bit uc, input string tag);
    mreq_t r; mrsp_t p;
    bit pres;
    pres = resident(line);
    r = mk(M_WR, line << 7, pol, uc, 1'b0);
    r.be = be; r.wdata = data;
    // expected effects
    for (int b = 0; b < 128; b++) if (be[b]) fmem[(line << 7) + b] = data[8*b +: 8];
    // (fabric model applies the same write; apply to a scratch copy first would double-apply: idempotent)
    if (!uc && pres) for (int b = 0; b < 128; b++) if (be[b]) img[line][8*b +: 8] = data[8*b +: 8];
    issue(r, p);
    checks++;
    if (!uc && pres && !resident(line)) begin errors++; if (errors < 12) $display("FAIL(%s): write hit dropped the line %h", tag, line); end
    if (!pres && resident(line)) begin errors++; if (errors < 12) $display("FAIL(%s): write miss allocated line %h (write-no-allocate)", tag, line); end
    prune_shadow();
    ops_done++;
    audit(tag);
  endtask

  task automatic op_amo(input longint unsigned pa, input logic [4:0] op, input bit w64, input logic [63:0] a, input logic [63:0] cmp,
                        input bit uc, input string tag);
    mreq_t r; mrsp_t p;
    longint unsigned line;
    logic [63:0] o, exp_new;
    int nb;
    bit pres;
    line = pa >> 7;
    pres = resident(line);
    nb = w64 ? 8 : 4;
    o = '0;
    for (int b = 0; b < nb; b++) o[8*b +: 8] = frd(pa + b);
    r = mk(M_AMO, pa, 2'd0, uc, 1'b0);
    r.amo_op = op; r.amo64 = w64; r.amo_rtn = 1'b1; r.amo_a = a; r.amo_cmp = cmp;
    issue(r, p);
    checks++;
    if (p.old !== o) begin errors++; if (errors < 12) $display("FAIL(%s): AMO old %h expected %h", tag, p.old, o); end
    if (!uc && resident(line)) begin errors++; if (errors < 12) $display("FAIL(%s): AMO did not invalidate the resident line %h", tag, line); end
    prune_shadow();
    ops_done++;
    audit(tag);
  endtask

  task automatic op_inv_l1(input string tag);
    mreq_t r; mrsp_t p;
    r = mk(M_INV_L1, 0, 2'd0, 1'b0, 1'b0);
    issue(r, p);
    checks++;
    if (n_resident() != 0) begin errors++; if (errors < 12) $display("FAIL(%s): lines resident after cache.inv.l1", tag); end
    img.delete();
    e_inv++;
    ops_done++;
  endtask

  task automatic op_maint(input logic [2:0] kind, input string tag);
    mreq_t r; mrsp_t p;
    int cnt0, nres;
    nres = n_resident();
    cnt0 = (kind == M_INV_L2) ? n_f_inv2 : n_f_wb2;
    r = mk(kind, 0, 2'd0, 1'b0, 1'b0);
    issue(r, p);
    checks++;
    if (((kind == M_INV_L2) ? n_f_inv2 : n_f_wb2) != cnt0 + 1) begin errors++; $display("FAIL(%s): maintenance not forwarded exactly once", tag); end
    if (n_resident() != nres) begin errors++; $display("FAIL(%s): maintenance changed L1 residency", tag); end
    ops_done++;
    audit(tag);
  endtask

  // external writer (another agent writing the fabric behind the L1's back)
  task automatic ext_write(input longint unsigned line, input int off, input logic [31:0] v);
    for (int b = 0; b < 4; b++) fmem[(line << 7) + off + b] = v[8*b +: 8];
  endtask

  function automatic logic [1023:0] rnd_line();
    logic [1023:0] d;
    for (int i = 0; i < 32; i++) d[32*i +: 32] = {$urandom()};
    return d;
  endfunction

  initial begin
    int seed;
    if ($value$plusargs("SEED=%d", seed)) begin int dummy; dummy = $urandom(seed); lfsr = lfsr ^ seed; end
    repeat (4) @(negedge clk);
    rst_n <= 1'b1;
    repeat (3) @(negedge clk);

    // seed the fabric
    for (int ln = 0; ln < 40; ln++) for (int b = 0; b < 128; b++) fmem[(longint'(ln) << 7) + b] = $urandom();

    // ---- D1: read allocates, second read hits, external write is NOT seen (non-coherent) ...
    op_rd(5, 2'd0, 0, 0, "D1a");
    op_rd(5, 2'd0, 0, 0, "D1b hit");
    ext_write(5, 8, 32'hcafe0001);
    op_rd(5, 2'd0, 0, 0, "D1c stale");              // expect stale shadow (checked by the shadow rule)
    // ... until cache.inv.l1
    op_inv_l1("D1d");
    op_rd(5, 2'd0, 0, 0, "D1e fresh");
    begin
      mrsp_t pp; mreq_t rr;
      checks++;
      rr = mk(M_RD, 5 << 7, 2'd0, 0, 0);
      issue(rr, pp);
      e_hit++;                 // line 5 was allocated by D1e, this read hits
      if (pp.rdata[8*8 +: 32] !== 32'hcafe0001) begin errors++; $display("FAIL D1: inv.l1 did not expose the external write"); end
    end
    // ---- D2: l2 / sys policy always reads the fabric even on a stale resident line
    ext_write(5, 16, 32'hcafe0002);
    op_rd(5, 2'd2, 0, 0, "D2 l2");
    op_rd(5, 2'd3, 0, 0, "D2 sys");
    // ---- D3: write-through: hit updates L1 + fabric, miss writes fabric only
    begin
      logic [127:0] be; logic [1023:0] d;
      be = 128'hff << 24; d = '0; d[8*24 +: 64] = 64'h1122334455667788;
      op_wr(5, be, d, 2'd0, 0, "D3 hit");
      op_wr(20, be, d, 2'd0, 0, "D3 miss");        // write-no-allocate
    end
    // ---- D4: nt never allocates; noalloc never allocates
    op_rd(21, 2'd1, 0, 0, "D4 nt");
    op_rd(21, 2'd1, 0, 0, "D4 nt again (still miss)");
    op_rd(22, 2'd0, 0, 1, "D4 noalloc");
    // ---- D5: atomics at the fabric, resident line invalidated
    op_rd(7, 2'd0, 0, 0, "D5 fill");
    op_amo((7 << 7) + 12, AMO_ADD_L, 0, 64'd5, 64'd0, 0, "D5 amo");
    op_rd(7, 2'd0, 0, 0, "D5 refill");
    // ---- D6: uc bypasses everything (never resident)
    op_rd(9, 2'd0, 1, 0, "D6 uc rd");
    op_wr(9, 128'hf, 1024'h1234, 2'd0, 1, "D6 uc wr");
    op_amo((9 << 7) + 4, AMO_ADD_L, 0, 64'd1, 64'd0, 1, "D6 uc amo");
    // ---- D7: maintenance forwarding
    op_maint(M_INV_L2, "D7 inv.l2");
    op_maint(M_WB_L2, "D7 wb.l2");
    // ---- D8: fill every way of every set, then evict
    for (int ln = 0; ln < SETS * WAYS + 3; ln++) op_rd(ln, 2'd0, 0, 0, "D8 sweep");
    op_inv_l1("D8 inv");

    // --
    for (int it = 0; it < 20000; it++) begin
      int k, lnn;
      longint unsigned line;
      logic [1:0] pol;
      bit uc;
      lnn = $urandom_range(0, 15);
      line = lnn;
      uc = ($urandom_range(0, 11) == 0);
      pol = $urandom_range(0, 3);
      k = $urandom_range(0, 99);
      if (k < 40) op_rd(line, ($urandom_range(0, 3) == 0) ? pol : 2'd0, uc, ($urandom_range(0, 9) == 0), "R rd");
      else if (k < 62) begin
        logic [127:0] be; logic [1023:0] d;
        be = 128'd0;
        for (int b = 0; b < 128; b++) be[b] = ($urandom_range(0, 3) == 0);
        if ($urandom_range(0, 2) == 0) be = '1;
        d = rnd_line();
        op_wr(line, be, d, pol, uc, "R wr");
      end else if (k < 72) begin
        bit w64;
        int off;
        w64 = $urandom_range(0, 1);
        off = $urandom_range(0, 15) * 8;
        op_amo((line << 7) + off, 5'($urandom_range(0, 4)), w64, {$urandom(), $urandom()}, {$urandom(), $urandom()}, uc, "R amo");
      end else if (k < 86) ext_write(line, $urandom_range(0, 31) * 4, $urandom());
      else if (k < 90) op_inv_l1("R inv");
      else if (k < 92) op_maint(M_INV_L2, "R inv2");
      else if (k < 94) op_maint(M_WB_L2, "R wb2");
      else op_rd(line, 2'd0, 0, 0, "R rd2");
    end

    // ---- counters vs TB expectation
    checks += 4;
    if (st_hit !== e_hit)   begin errors++; $display("FAIL: stat_hit %0d expected %0d", st_hit, e_hit); end
    if (st_miss !== e_miss) begin errors++; $display("FAIL: stat_miss %0d expected %0d", st_miss, e_miss); end
    if (st_fill !== e_fill) begin errors++; $display("FAIL: stat_fill %0d expected %0d", st_fill, e_fill); end
    if (st_inv !== e_inv)   begin errors++; $display("FAIL: stat_inv %0d expected %0d", st_inv, e_inv); end
    $display("NG_LSU_L1 SETS=%0d WAYS=%0d ops=%0d checks=%0d errors=%0d | hit=%0d miss=%0d fill=%0d inv=%0d | fabric rd=%0d wr=%0d amo=%0d uc=%0d inv2=%0d wb2=%0d",
             SETS, WAYS, ops_done, checks, errors, st_hit, st_miss, st_fill, st_inv, n_f_rd, n_f_wr, n_f_amo, n_f_uc, n_f_inv2, n_f_wb2);
    if (errors == 0) $display("NG_LSU_L1_TB_PASS");
    else $display("NG_LSU_L1_TB_FAIL");
    $finish;
  end

  initial begin
    repeat (8) #1000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
