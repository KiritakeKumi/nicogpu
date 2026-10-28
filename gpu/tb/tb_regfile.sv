// tb_regfile.sv -- SP register file testbench.
//
// The reference is a behavioural copy of the functional model's register
// semantics (apply_pending: lane_mask + bit_mask merge,
// num_vregs range check -> RegRange, 64 uniform registers per warp) plus the
// documented port rules (lowest port wins a bank / a uniform register, reads
// are write-first bypassed, response one cycle after the grant).  A cycle-level
// scoreboard compares every grant (ready), every response (valid/err/data) and
// every error pulse against it.
//
//   * directed: bank conflict serialisation, conflict-free consecutive regs,
//     same-cycle write->read bypass, half (lo/hi) writes + half reads,
//     write-write bank conflict, range errors, allocation errors
//   * random: two phases (different port pressure and slot layouts, second
//     one after freeing/reallocating slots), 3 read + 2 write vector ports and
//     3 read + 2 write uniform ports all busy, biased to few registers so
//     conflicts and bypasses are frequent
//
// Seed via `NG_TB_SEED and lane count via `NG_TB_NL (run script regenerates
// tb_regfile_cfg.svh per run).  Prints NG_RF_TB_PASS on success.
`timescale 1ns/1ps
`ifdef NG_TB_CFG_FILE
  `include "tb_regfile_cfg.svh"
`endif
`ifndef NG_TB_SEED
  `define NG_TB_SEED 1
`endif
`ifndef NG_TB_NL
  `define NG_TB_NL 16
`endif
`ifndef NG_TB_CYCLES
  `define NG_TB_CYCLES 12000
`endif

module tb_regfile;
  localparam int NW   = 16;
  localparam int NL   = `NG_TB_NL;
  localparam int NBEAT = 32 / NL;
  localparam int BTW  = (NL < 32) ? $clog2(32 / NL) : 1;
  localparam int NB   = 4;
  localparam int VREGS = 1024;
  localparam int NRP = 3, NWP = 2, NURP = 3, NUWP = 2;
  localparam int DW  = NL * 32;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // --------------------------------------------------------------
  logic        alloc_valid = 1'b0;
  logic [3:0]  alloc_slot = '0;
  logic [9:0]  alloc_base = '0;
  logic [7:0]  alloc_nregs = '0;
  logic        alloc_err;

  logic [NRP-1:0] rd_valid = '0, rd_ready, rdv_valid, rdv_err;
  logic [3:0]     rd_slot [NRP];
  logic [6:0]     rd_reg  [NRP];
  logic [BTW-1:0] rd_beat [NRP];
  logic [1:0]     rd_hsel [NRP];
  logic [DW-1:0]  rdv_data [NRP];
  logic           perf_rd_conflict;

  logic [NWP-1:0] wr_valid = '0, wr_ready, wr_err;
  logic [3:0]     wr_slot [NWP];
  logic [6:0]     wr_reg  [NWP];
  logic [BTW-1:0] wr_beat [NWP];
  logic [NL-1:0]  wr_lanes [NWP];
  logic [1:0]     wr_hsel [NWP];
  logic [DW-1:0]  wr_data [NWP];

  logic [NURP-1:0] ur_valid = '0, urd_valid;
  logic [3:0]      ur_slot [NURP];
  logic [5:0]      ur_reg  [NURP];
  logic [31:0]     urd_data [NURP];
  logic [NUWP-1:0] uw_valid = '0;
  logic [3:0]      uw_slot [NUWP];
  logic [5:0]      uw_reg  [NUWP];
  logic [31:0]     uw_data [NUWP];

  ng_sp_regfile #(.NG_RF_NL(NL)) dut (.*);

  // --------------------------------------------------------------
  int seed = `NG_TB_SEED;
  function automatic int rnd();  // xorshift32 (xsim has no seeded $urandom)
    seed = seed ^ (seed << 13);
    seed = seed ^ (seed >> 17);
    seed = seed ^ (seed << 5);
    return (seed < 0) ? -seed : seed;
  endfunction
  function automatic int rr(input int hi);  // 0..hi
    return rnd() % (hi + 1);
  endfunction
  function automatic logic [DW-1:0] rdata();
    logic [DW-1:0] d;
    for (int i = 0; i < NL; i++) d[i*32 +: 32] = rnd() ^ (rnd() << 16);
    return d;
  endfunction

  // --------------------------------------------------------------
  logic [31:0] rv [NW][128][32];
  logic [31:0] ru [NW][64];
  int          ref_n [NW];
  int          ref_base [NW];

  int errors = 0;
  task automatic err(input string m);
    errors++;
    if (errors < 30) $display("FAIL @%0t: %s", $time, m);
  endtask

  // coverage
  int cov_conflict = 0, cov_bypass = 0, cov_rerr = 0, cov_werr = 0, cov_wconf = 0, cov_uw2 = 0;
  int cov_half = 0, n_reads = 0, n_writes = 0;

  // pending response expectations (grant edge N -> checked at edge N+1)
  bit             pv [NRP], pe [NRP];
  logic [DW-1:0]  pd [NRP];
  bit             pwe [NWP];
  bit             puv [NURP];
  logic [31:0]    pud [NURP];
  bit             perr_alloc;
  bit             took_r [NRP], took_w [NWP];

  function automatic int phys_of(input int s, input int r);
    return ref_base[s] + r;
  endfunction
  function automatic int entry_of(input int ph, input int beat);
    return (ph / NB) * NBEAT + beat;
  endfunction

  // --------------------------------------------------------------
  always @(posedge clk) if (rst_n) begin : mon
    bit e_rdy;
    // ---- check last cycle's responses
    for (int p = 0; p < NRP; p++) begin
      if (rdv_valid[p] !== pv[p]) err($sformatf("rdv_valid[%0d]=%b exp %b", p, rdv_valid[p], pv[p]));
      else if (pv[p]) begin
        if (rdv_err[p] !== pe[p]) err($sformatf("rdv_err[%0d]=%b exp %b", p, rdv_err[p], pe[p]));
        if (rdv_data[p] !== pd[p]) err($sformatf("rdv_data[%0d]=%h exp %h", p, rdv_data[p], pd[p]));
      end
    end
    for (int p = 0; p < NWP; p++)
      if (wr_err[p] !== pwe[p]) err($sformatf("wr_err[%0d]=%b exp %b", p, wr_err[p], pwe[p]));
    for (int p = 0; p < NURP; p++) begin
      if (urd_valid[p] !== puv[p]) err($sformatf("urd_valid[%0d]=%b exp %b", p, urd_valid[p], puv[p]));
      else if (puv[p] && urd_data[p] !== pud[p]) err($sformatf("urd_data[%0d]=%h exp %h", p, urd_data[p], pud[p]));
    end
    if (alloc_err !== perr_alloc) err($sformatf("alloc_err=%b exp %b", alloc_err, perr_alloc));

    // ---- allocation (effective for next cycle's accesses)
    perr_alloc = 1'b0;
    if (alloc_valid) begin
      if (alloc_nregs > 128 || int'(alloc_base) + int'(alloc_nregs) > VREGS) perr_alloc = 1'b1;
      else begin
        ref_n[alloc_slot] = alloc_nregs;
        ref_base[alloc_slot] = alloc_base;
        for (int r = 0; r < 128; r++) for (int l = 0; l < 32; l++) rv[alloc_slot][r][l] = 32'd0;
      end
    end

    // ---- vector writes: grants (independent bank rule), then apply
    for (int p = 0; p < NWP; p++) begin
      pwe[p] = 1'b0;
      took_w[p] = 1'b0;
      if (wr_valid[p]) begin
        if (wr_reg[p] >= ref_n[wr_slot[p]]) begin
          pwe[p] = 1'b1;
          took_w[p] = 1'b1;
          cov_werr++;
          if (wr_ready[p] !== 1'b1) err($sformatf("wr_ready[%0d] low on range error", p));
        end else begin
          e_rdy = 1'b1;
          for (int q = 0; q < p; q++)
            if (wr_valid[q] && wr_reg[q] < ref_n[wr_slot[q]] &&
                phys_of(wr_slot[q], wr_reg[q]) % NB == phys_of(wr_slot[p], wr_reg[p]) % NB) e_rdy = 1'b0;
          if (!e_rdy) cov_wconf++;
          if (wr_ready[p] !== e_rdy) err($sformatf("wr_ready[%0d]=%b exp %b", p, wr_ready[p], e_rdy));
          if (e_rdy) begin
            took_w[p] = 1'b1;
            n_writes++;
            if (wr_hsel[p] != 2'b11 && wr_hsel[p] != 2'b00) cov_half++;
            for (int l = 0; l < NL; l++) begin
              if (wr_lanes[p][l]) begin
                logic [31:0] m, o;
                m = {{16{wr_hsel[p][1]}}, {16{wr_hsel[p][0]}}};
                o = rv[wr_slot[p]][wr_reg[p]][int'(wr_beat[p]) * NL + l];
                rv[wr_slot[p]][wr_reg[p]][int'(wr_beat[p]) * NL + l] = (o & ~m) | (wr_data[p][l*32 +: 32] & m);
              end
            end
          end
        end
      end
    end

    // ---- vector reads: expected grant, expected data (after this cycle's writes)
    for (int p = 0; p < NRP; p++) begin
      pv[p] = 1'b0; pe[p] = 1'b0; pd[p] = '0;
      took_r[p] = 1'b0;
      if (rd_valid[p]) begin
        pv[p] = 1'b1;
        if (rd_reg[p] >= ref_n[rd_slot[p]]) begin
          pe[p] = 1'b1;
          took_r[p] = 1'b1;
          cov_rerr++;
          if (rd_ready[p] !== 1'b1) err($sformatf("rd_ready[%0d] low on range error", p));
        end else begin
          e_rdy = 1'b1;
          for (int q = 0; q < p; q++)
            if (rd_valid[q] && rd_reg[q] < ref_n[rd_slot[q]] &&
                phys_of(rd_slot[q], rd_reg[q]) % NB == phys_of(rd_slot[p], rd_reg[p]) % NB) e_rdy = 1'b0;
          if (rd_ready[p] !== e_rdy) err($sformatf("rd_ready[%0d]=%b exp %b", p, rd_ready[p], e_rdy));
          if (!e_rdy) begin
            pv[p] = 1'b0;
            cov_conflict++;
          end else begin
            took_r[p] = 1'b1;
            n_reads++;
            for (int l = 0; l < NL; l++) begin
              logic [31:0] v;
              v = rv[rd_slot[p]][rd_reg[p]][int'(rd_beat[p]) * NL + l];
              case (rd_hsel[p])
                2'd1: v = {16'd0, v[15:0]};
                2'd2: v = {16'd0, v[31:16]};
                default: ;
              endcase
              pd[p][l*32 +: 32] = v;
            end
            for (int q = 0; q < NWP; q++)
              if (took_w[q] && phys_of(wr_slot[q], wr_reg[q]) % NB == phys_of(rd_slot[p], rd_reg[p]) % NB &&
                  entry_of(phys_of(wr_slot[q], wr_reg[q]), int'(wr_beat[q])) ==
                  entry_of(phys_of(rd_slot[p], rd_reg[p]), int'(rd_beat[p])))
                cov_bypass++;
          end
        end
      end
    end
    if (perf_rd_conflict !== (|(rd_valid & ~rd_ready) )) begin
      // perf pulse counts only bank losers (range errors are ready), so the
      // two agree whenever ready is the only reason for a stall.
      err($sformatf("perf_rd_conflict=%b vs stalled %b", perf_rd_conflict, |(rd_valid & ~rd_ready)));
    end

    // ---- uniform: writes in port order (highest wins), then reads (bypass)
    for (int p = 0; p < NUWP; p++)
      if (uw_valid[p]) begin
        ru[uw_slot[p]][uw_reg[p]] = uw_data[p];
        for (int q = 0; q < p; q++)
          if (uw_valid[q] && uw_slot[q] == uw_slot[p] && uw_reg[q] == uw_reg[p]) cov_uw2++;
      end
    for (int p = 0; p < NURP; p++) begin
      puv[p] = ur_valid[p];
      pud[p] = ru[ur_slot[p]][ur_reg[p]];
    end
  end

  // ---------------------------------------------------------------- stimulus helpers
  bit rand_en = 1'b0;
  int pv_rd, pv_wr, pv_ur, pv_uw;   // percent valid
  int small_pct;                     // percent of accesses drawn from a small register window
  int win_reg;                       // window base

  task automatic nedge();
    @(negedge clk);
  endtask
  task automatic idle_all();
    rd_valid = '0; wr_valid = '0; ur_valid = '0; uw_valid = '0; alloc_valid = 1'b0;
  endtask

  task automatic do_alloc(input int slot, input int base, input int n);
    nedge();
    alloc_valid = 1'b1; alloc_slot = slot; alloc_base = base; alloc_nregs = n;
    nedge();
    alloc_valid = 1'b0;
  endtask

  // Clear every allocated register through port 0 (the DUT does not clear on
  // allocation; a real launch sequencer would).
  task automatic zero_fill();
    for (int s = 0; s < NW; s++) begin
      for (int r = 0; r < ref_n[s]; r++)
        for (int b = 0; b < NBEAT; b++) begin
          nedge();
          wr_valid = 2'b01; wr_slot[0] = s; wr_reg[0] = r; wr_beat[0] = b;
          wr_lanes[0] = '1; wr_hsel[0] = 2'b11; wr_data[0] = '0;
        end
      if (ref_n[s] > 0)
        for (int u = 0; u < 64; u++) begin
          nedge();
          wr_valid = '0;
          uw_valid = 2'b01; uw_slot[0] = s; uw_reg[0] = u; uw_data[0] = 32'd0;
        end
      nedge();
      idle_all();
    end
  endtask

  int lay_slots [NW];
  task automatic random_layout(input int free_pct);
    int pos, rem, n;
    pos = rr(16);
    for (int s = 0; s < NW; s++) begin
      rem = VREGS - pos;
      n = 0;
      if (rem > 0 && rr(99) >= free_pct) n = 1 + rr((rem < 64 ? rem : 64) - 1);
      if (rr(9) == 0 && rem > 100) n = 100 + rr(28);   // occasional big warp
      if (n > rem) n = rem;
      if (n > 128) n = 128;
      do_alloc(s, pos, n);
      // TB-side visibility: mirror immediately (the scoreboard also mirrors)
      pos += n + rr(3);
      if (pos > VREGS) pos = VREGS;
    end
  endtask

  // random request generators (called at negedge)
  function automatic int pick_slot(input bit allow_bad);
    int s, tries;
    s = rr(NW - 1);
    tries = 0;
    while (!allow_bad && ref_n[s] == 0 && tries < 40) begin s = rr(NW - 1); tries++; end
    if (!allow_bad && ref_n[s] == 0)
      for (int k = 0; k < NW; k++) if (ref_n[k] != 0) s = k;
    return s;
  endfunction
  function automatic int pick_reg(input int s);
    int n;
    n = ref_n[s];
    if (n == 0) return rr(127);
    if (rr(99) < 4) return n + rr(3);            // just out of range
    if (rr(99) < small_pct) return (win_reg + rr(7)) % n;
    return rr(n - 1);
  endfunction

  always @(negedge clk) if (rand_en && rst_n) begin : drv
    for (int p = 0; p < NRP; p++)
      if (!(rd_valid[p] && !took_r[p])) begin   // hold until granted
        rd_valid[p] = (rr(99) < pv_rd);
        rd_slot[p] = pick_slot(rr(19) == 0);
        rd_reg[p]  = pick_reg(rd_slot[p]);
        rd_beat[p] = rr(NBEAT - 1);
        rd_hsel[p] = (rr(3) == 0) ? 1 + rr(1) : 0;
      end
    for (int p = 0; p < NWP; p++)
      if (!(wr_valid[p] && !took_w[p] && !wr_err_pending(p))) begin
        wr_valid[p] = (rr(99) < pv_wr);
        wr_slot[p] = pick_slot(rr(19) == 0);
        wr_reg[p]  = pick_reg(wr_slot[p]);
        wr_beat[p] = rr(NBEAT - 1);
        wr_lanes[p] = (rr(2) == 0) ? rdata() : '1;
        wr_hsel[p] = (rr(3) == 0) ? 1 + rr(2) : 3;
        wr_data[p] = rdata();
      end
    for (int p = 0; p < NURP; p++) begin
      ur_valid[p] = (rr(99) < pv_ur);
      ur_slot[p] = pick_slot(1'b0);
      ur_reg[p] = (rr(99) < small_pct) ? (win_reg + rr(3)) % 64 : rr(63);
    end
    for (int p = 0; p < NUWP; p++) begin
      uw_valid[p] = (rr(99) < pv_uw);
      uw_slot[p] = pick_slot(1'b0);
      uw_reg[p] = (rr(99) < small_pct) ? (win_reg + rr(3)) % 64 : rr(63);
      uw_data[p] = rnd() ^ (rnd() << 16);
    end
    if (rr(199) == 0) win_reg = rr(127);
  end
  // A range-error write is accepted (ready=1) in the same cycle, so it is
  // never held; wr_err_pending is thus constant 0 (kept for readability).
  function automatic bit wr_err_pending(input int p);
    return 1'b0;
  endfunction

  // --------------------------------------------------------------
  int base_d;
  task automatic drive_rd(input int p, input int s, input int r, input int beat, input int hs);
    rd_valid[p] = 1'b1; rd_slot[p] = s; rd_reg[p] = r; rd_beat[p] = beat; rd_hsel[p] = hs;
  endtask
  task automatic drive_wr(input int p, input int s, input int r, input int beat, input logic [NL-1:0] lanes,
                          input int hs, input logic [DW-1:0] d);
    wr_valid[p] = 1'b1; wr_slot[p] = s; wr_reg[p] = r; wr_beat[p] = beat;
    wr_lanes[p] = lanes; wr_hsel[p] = hs; wr_data[p] = d;
  endtask
  task automatic expect_rdy(input logic [NRP-1:0] exp, input string m);
    #1;
    if (rd_ready !== exp) err($sformatf("%s: rd_ready=%b exp %b", m, rd_ready, exp));
  endtask

  task automatic directed();
    logic [DW-1:0] d0, d1;
    logic [DW-1:0] exp_d;
    // slot 0: 32 regs at base 5 (unaligned), slot 1: 8 regs at 40
    do_alloc(0, 5, 32);
    do_alloc(1, 40, 8);
    zero_fill();

    // -- D1: three same-bank reads (regs 0,4,8) serialise in port order
    nedge(); idle_all();
    drive_rd(0, 0, 0, 0, 0); drive_rd(1, 0, 4, 0, 0); drive_rd(2, 0, 8, 0, 0);
    expect_rdy(3'b001, "D1 cycle0");
    nedge(); rd_valid = 3'b110; expect_rdy(3'b010, "D1 cycle1");
    nedge(); rd_valid = 3'b100; expect_rdy(3'b100, "D1 cycle2");
    // consecutive registers: all three banks distinct, one cycle
    nedge(); idle_all();
    drive_rd(0, 0, 1, 0, 0); drive_rd(1, 0, 2, 0, 0); drive_rd(2, 0, 3, 0, 0);
    expect_rdy(3'b111, "D1 no-conflict");
    // two of three collide
    nedge(); idle_all();
    drive_rd(0, 0, 1, 0, 0); drive_rd(1, 0, 2, 0, 0); drive_rd(2, 0, 5, 0, 0);
    expect_rdy(3'b011, "D1 partial");
    nedge(); rd_valid = 3'b100; expect_rdy(3'b100, "D1 partial-2");

    // -- D2: bypass: old = D0 written, then lo-half write and full read same cycle
    d0 = rdata(); d1 = rdata();
    nedge(); idle_all();
    drive_wr(0, 0, 7, 0, '1, 3, d0);
    nedge(); idle_all();
    drive_wr(0, 0, 7, 0, '1, 1, d1);       // low halves only
    drive_rd(0, 0, 7, 0, 0);
    nedge(); idle_all();                     // response visible now (before next posedge)
    #1;
    for (int l = 0; l < NL; l++) exp_d[l*32 +: 32] = {d0[l*32+16 +: 16], d1[l*32 +: 16]};
    if (!rdv_valid[0] || rdv_data[0] !== exp_d) err("D2 bypass merge mismatch");
    if (cov_bypass == 0) err("D2 no bypass observed by scoreboard");
    // hi-half read of the same reg, lane-masked hi write, back to back
    drive_wr(1, 0, 7, 0, {(NL/2){2'b10}}, 2, d1);   // odd lanes hi <- d1.hi
    drive_rd(2, 0, 7, 0, 2);
    nedge(); idle_all(); #1;
    for (int l = 0; l < NL; l++) exp_d[l*32 +: 32] = {16'd0, (l % 2 ? d1[l*32+16 +: 16] : d0[l*32+16 +: 16])};
    if (rdv_data[2] !== exp_d) err("D2 hi-half read mismatch");

    // -- D3: write-write same bank (regs 2 and 6 of slot 0) serialise
    nedge(); idle_all();
    drive_wr(0, 0, 2, 0, '1, 3, rdata()); drive_wr(1, 0, 6, 0, '1, 3, rdata());
    #1;
    if (wr_ready !== 2'b01) err("D3 wr_ready cycle0");
    nedge(); wr_valid = 2'b10; #1;
    if (wr_ready !== 2'b10) err("D3 wr_ready cycle1");
    // different banks: both go
    nedge(); idle_all();
    drive_wr(0, 0, 2, 0, '1, 3, rdata()); drive_wr(1, 0, 3, 0, '1, 3, rdata());
    #1;
    if (wr_ready !== 2'b11) err("D3 wr_ready parallel");

    // -- D5: range: reg n-1 ok, reg n error, unallocated slot error, write dropped
    nedge(); idle_all();
    drive_rd(0, 1, 7, 0, 0); drive_rd(1, 1, 8, 0, 0); drive_rd(2, 9, 0, 0, 0);
    drive_wr(0, 1, 8, 0, '1, 3, rdata());
    nedge(); idle_all(); #1;
    if (rdv_err !== 3'b110) err($sformatf("D5 rdv_err=%b", rdv_err));
    if (wr_err !== 2'b01) err($sformatf("D5 wr_err=%b", wr_err));
    nedge(); #1;
    if (wr_err !== 2'b00) err("D5 wr_err not a pulse");

    // -- D6: allocation error (does not disturb the table)
    nedge(); alloc_valid = 1'b1; alloc_slot = 1; alloc_base = 1020; alloc_nregs = 8;
    nedge(); alloc_valid = 1'b0; #1;
    if (!alloc_err) err("D6 alloc_err missing");
    nedge(); #1;
    if (alloc_err) err("D6 alloc_err not a pulse");
    nedge(); idle_all();
    drive_rd(0, 1, 7, 0, 0);
    nedge(); idle_all(); #1;
    if (rdv_err[0]) err("D6 slot 1 allocation lost");
    nedge(); idle_all();
  endtask

  // --------------------------------------------------------------
  initial begin : main
    for (int s = 0; s < NW; s++) begin
      ref_n[s] = 0; ref_base[s] = 0;
      for (int r = 0; r < 128; r++) for (int l = 0; l < 32; l++) rv[s][r][l] = 32'd0;
      for (int u = 0; u < 64; u++) ru[s][u] = 32'd0;
    end
    for (int p = 0; p < NRP; p++) begin
      rd_slot[p] = 0; rd_reg[p] = 0; rd_beat[p] = 0; rd_hsel[p] = 0; pv[p] = 0; pe[p] = 0; pd[p] = '0;
      took_r[p] = 0;
    end
    for (int p = 0; p < NWP; p++) begin
      wr_slot[p] = 0; wr_reg[p] = 0; wr_beat[p] = 0; wr_lanes[p] = '0; wr_hsel[p] = 0; wr_data[p] = '0;
      pwe[p] = 0; took_w[p] = 0;
    end
    for (int p = 0; p < NURP; p++) begin ur_slot[p] = 0; ur_reg[p] = 0; puv[p] = 0; pud[p] = 0; end
    for (int p = 0; p < NUWP; p++) begin uw_slot[p] = 0; uw_reg[p] = 0; uw_data[p] = 0; end
    perr_alloc = 0;
    pv_rd = 50; pv_wr = 50; pv_ur = 50; pv_uw = 50; small_pct = 50; win_reg = 0;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    directed();

    // -------- random phase A: moderate pressure
    random_layout(10);
    zero_fill();
    pv_rd = 60; pv_wr = 50; pv_ur = 50; pv_uw = 40; small_pct = 50;
    nedge(); rand_en = 1'b1;
    repeat (`NG_TB_CYCLES) @(negedge clk);
    rand_en = 1'b0; nedge(); idle_all();
    repeat (3) nedge();

    // -------- random phase B: free some slots, reallocate, hammer a few registers
    random_layout(40);
    zero_fill();
    pv_rd = 95; pv_wr = 90; pv_ur = 90; pv_uw = 80; small_pct = 85;
    nedge(); rand_en = 1'b1;
    repeat (`NG_TB_CYCLES) @(negedge clk);
    rand_en = 1'b0; nedge(); idle_all();
    repeat (4) nedge();

    $display("cov: reads=%0d writes=%0d conflicts=%0d wconf=%0d bypass=%0d halfw=%0d rerr=%0d werr=%0d uw2=%0d",
             n_reads, n_writes, cov_conflict, cov_wconf, cov_bypass, cov_half, cov_rerr, cov_werr, cov_uw2);
    if (cov_conflict == 0 || cov_bypass == 0 || cov_wconf == 0 || cov_half == 0 || cov_rerr == 0 ||
        cov_werr == 0 || cov_uw2 == 0)
      err("coverage hole");
    if (errors == 0) $display("NG_RF_TB_PASS seed=%0d NL=%0d", `NG_TB_SEED, NL);
    else $display("NG_RF_TB_FAIL errors=%0d", errors);
    $finish;
  end

  initial begin : watchdog
    #(64'd400_000_000);
    $display("FAIL: timeout");
    $display("NG_RF_TB_FAIL");
    $finish;
  end
endmodule
