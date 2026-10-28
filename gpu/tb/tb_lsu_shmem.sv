// tb_lsu_shmem.sv -- ng_lsu_shmem (banked shared memory, conflict serialisation) .
//
// Independent reference: a byte array + a TB-side re-implementation of the round
// rule ("maximal prefix of pending lanes whose dword footprints do not collide;
// load lanes may share a dword (broadcast), store lanes may not share a bank").
// Checks per round: the served-lane mask, the load data of every served lane,
// the store effect (final memory equals the reference).  Directed cases pin the
// textbook conflict counts (stride 1 -> 1 round, stride 2 -> 2, stride 32 -> 32,
// broadcast -> 1, aligned b128 -> 4 rounds, same-address store -> 32 ...).
// Random: pattern families x access sizes 1..16 B x load/store x unaligned, many
// seeds (+SEED=<n>, xsim -testplusarg SEED=<n>).
`timescale 1ns / 1ps

module tb_lsu_shmem;
  localparam int AW    = 14;               // 16 KiB array
  localparam int BYTES = 1 << AW;
`ifdef NG_TB_LPB
  localparam int LPB   = `NG_TB_LPB;       // lanes per beat of the DUT
`else
  localparam int LPB   = 8;
`endif
  localparam int NG    = 32 / LPB;         // groups (= rounds of a conflict-free 32-lane access)
  localparam int GW    = (LPB < 32) ? $clog2(32 / LPB) : 1;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic                req_valid = 1'b0;
  logic                req_wr = 1'b0;
  logic [LPB-1:0]      req_pend = '0;
  logic [GW-1:0]       req_grp = '0;
  logic [AW-1:0]       req_addr [LPB];
  logic [4:0]          req_nb = '0;
  logic [127:0]        req_wdata [LPB];
  logic                rsp_valid;
  logic [GW-1:0]       rsp_grp;
  logic [LPB-1:0]      rsp_sel;
  logic [127:0]        rsp_rdata [LPB];

  ng_lsu_shmem #(.NB(32), .BYTES(BYTES), .AW(AW), .LPB(LPB)) dut (
    .clk(clk), .rst_n(rst_n), .req_valid(req_valid), .req_wr(req_wr), .req_grp(req_grp), .req_pend(req_pend),
    .req_addr(req_addr), .req_nb(req_nb), .req_wdata(req_wdata),
    .rsp_valid(rsp_valid), .rsp_grp(rsp_grp), .rsp_sel(rsp_sel), .rsp_rdata(rsp_rdata),
    .clr_en(1'b0), .clr_row('0)
  );

  byte unsigned refm [BYTES];
  int errors = 0, checks = 0, rounds_total = 0, instr_total = 0;

  // ---- reference round selection -------------------------------------------------
  // lowest group (of LPB lanes) with a pending lane; -1 when nothing is pending
  function automatic int lowest_group(input logic [31:0] pend);
    for (int g = 0; g < NG; g++)
      for (int i = 0; i < LPB; i++)
        if (pend[g * LPB + i]) return g;
    return -1;
  endfunction

  // A round serves the maximal conflict-free PREFIX of the pending lanes of the lowest pending group.
  function automatic logic [31:0] ref_sel(input logic [31:0] pend, input logic wr, input int nb1,
                                          input int addr [32]);
    logic [31:0] sel;
    bit claimed [32];
    int claim_d [32];
    bit stop;
    int g;
    sel = '0;
    stop = 0;
    for (int b = 0; b < 32; b++) begin claimed[b] = 0; claim_d[b] = 0; end
    g = lowest_group(pend);
    for (int l = 0; l < 32; l++) if (g < 0 || l / LPB != g) pend[l] = 1'b0;
    for (int l = 0; l < 32; l++) begin
      if (pend[l] && !stop) begin
        int d0, d1;
        bit conflict;
        d0 = addr[l] / 4;
        d1 = (addr[l] + nb1 - 1) / 4;
        conflict = 0;
        for (int d = d0; d <= d1; d++)
          if (claimed[d % 32] && (wr || claim_d[d % 32] != d)) conflict = 1;
        if (conflict) stop = 1;
        else begin
          sel[l] = 1;
          for (int d = d0; d <= d1; d++) begin claimed[d % 32] = 1; claim_d[d % 32] = d; end
        end
      end
    end
    return sel;
  endfunction

  // one access = all pending lanes until done; returns the number of rounds
  int  addr_i [32];
  task automatic run_access(input bit wr, input logic [31:0] exec, input int nb1, input int addrs [32],
                            input logic [127:0] wd [32], output int rounds, input bit expect_rounds_en, input int expect_rounds);
    logic [31:0] pend, esel;
    int guard;
    pend = exec;
    rounds = 0;
    guard = 0;
    while (pend != 0 && guard < 200) begin
      int g;
      guard++;
      g = lowest_group(pend);
      // Inputs are driven with BLOCKING assignments at the falling edge (half a cycle before the
      // sampling rising edge): back-to-back rounds re-drive req_valid in the same time step, and
      // two NBAs to one variable in one step are not applied in program order by every Verilator
      // version (Verilator 5.050 dropped the second one -> "no response").
      for (int i = 0; i < LPB; i++) begin
        req_addr[i] = AW'(addrs[g * LPB + i]);
        req_wdata[i] = wd[g * LPB + i];
      end
      req_wr = wr; req_grp = GW'(g); req_pend = pend[g * LPB +: LPB]; req_nb = 5'(nb1 - 1);
      req_valid = 1'b1;
      @(negedge clk);   // the request is sampled at the rising edge in between
      req_valid = 1'b0;
      // the response is a one-cycle pulse a few cycles later (pipelined unit)
      for (int w = 0; w < 20 && !rsp_valid; w++) @(negedge clk);
      if (!rsp_valid) begin errors++; $display("FAIL: no response"); end
      if (rsp_grp !== GW'(g)) begin errors++; $display("FAIL: rsp_grp %0d expected %0d", rsp_grp, g); end
      esel = ref_sel(pend, wr, nb1, addrs);
      checks++;
      if (32'(rsp_sel) << (g * LPB) !== esel) begin
        errors++;
        if (errors < 10) $display("FAIL: sel got %h expected %h (wr=%0d nb=%0d pend=%h)", 32'(rsp_sel) << (g * LPB), esel, wr, nb1, pend);
      end
      if (!wr) begin
        for (int l = 0; l < 32; l++) if (esel[l]) begin
          for (int q = 0; q < nb1; q++) begin
            checks++;
            if (rsp_rdata[l % LPB][8*q +: 8] !== refm[addrs[l] + q]) begin
              errors++;
              if (errors < 10) $display("FAIL: load lane %0d byte %0d got %02x expected %02x (addr %0d)", l, q,
                                        rsp_rdata[l % LPB][8*q +: 8], refm[addrs[l] + q], addrs[l]);
            end
          end
        end
      end else begin
        for (int l = 0; l < 32; l++) if (esel[l])
          for (int q = 0; q < nb1; q++) refm[addrs[l] + q] = wd[l][8*q +: 8];
      end
      pend = pend & ~esel;
      rounds++;
      @(negedge clk);   // idle gap (the pulse is gone)
    end
    rounds_total += rounds;
    instr_total++;
    if (expect_rounds_en && rounds != expect_rounds) begin
      errors++;
      $display("FAIL: expected %0d rounds, got %0d", expect_rounds, rounds);
    end
  endtask

  task automatic fill_ref_and_dut_random();
    // write the whole array through the DUT with an aligned stride-1 b32 store, 32 lanes per round
    logic [127:0] wd [32];
    int ad [32];
    int r;
    for (int base = 0; base < BYTES; base += 128) begin
      for (int l = 0; l < 32; l++) begin
        wd[l] = {96'd0, $urandom()};
        ad[l] = base + 4 * l;
      end
      run_access(1'b1, 32'hffffffff, 4, ad, wd, r, 1'b1, NG);
    end
  endtask

  task automatic readback_check();
    logic [127:0] wd [32];
    int ad [32];
    int r;
    for (int l = 0; l < 32; l++) wd[l] = '0;
    for (int base = 0; base < BYTES; base += 128) begin
      for (int l = 0; l < 32; l++) ad[l] = base + 4 * l;
      run_access(1'b0, 32'hffffffff, 4, ad, wd, r, 1'b1, NG);
    end
  endtask

  function automatic int rounds_expected(input logic [31:0] exec, input bit wr, input int nb1, input int addrs [32]);
    logic [31:0] pend, sel;
    int n;
    pend = exec; n = 0;
    while (pend != 0) begin sel = ref_sel(pend, wr, nb1, addrs); pend = pend & ~sel; n++; end
    return n;
  endfunction

  int dir_addr [32];
  logic [127:0] wdz [32];
  int rr;

  initial begin
    for (int i = 0; i < BYTES; i++) refm[i] = 8'd0;
    for (int l = 0; l < 32; l++) begin req_addr[l] = '0; req_wdata[l] = '0; wdz[l] = '0; end
    begin
      int seed;
      if ($value$plusargs("SEED=%d", seed)) begin int dummy; dummy = $urandom(seed); end
    end
    repeat (4) @(negedge clk);
    rst_n <= 1'b1;
    repeat (3) @(negedge clk);

    // ---- initialise memory (reference and DUT agree: zero, then random fill through the DUT)
    fill_ref_and_dut_random();
    readback_check();

    // ---- directed conflict counts (loads)
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 4 * l;
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, NG);                  // stride-1 dword: no conflict
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 8 * l;
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, NG * ((LPB * 2 + 31) / 32));                  // stride-2 dword: 2-way
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 16 * l;
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, NG * ((LPB * 4 + 31) / 32));                  // stride-4: 4-way
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 128 * l;
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, 32);                 // stride-32 dwords: all one bank
    for (int l = 0; l < 32; l++) dir_addr[l] = 512;
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, NG);                  // broadcast read
    for (int l = 0; l < 32; l++) dir_addr[l] = 512 + 4 * (l / 2);
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, NG);                  // pairs share a dword
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 16 * l;
    run_access(1'b0, 32'hffffffff, 16, dir_addr, wdz, rr, 1'b1, 32 / (LPB < 8 ? LPB : 8));                 // b128, 16 B stride: 8 lanes per round
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 16 * l;
    run_access(1'b0, 32'h000000ff, 16, dir_addr, wdz, rr, 1'b1, 8 / (LPB < 8 ? LPB : 8));                 // 8 lanes x 4 dwords = 32 banks
    for (int l = 0; l < 32; l++) dir_addr[l] = 256 + 4 * l;
    run_access(1'b0, 32'h0, 4, dir_addr, wdz, rr, 1'b1, 0);                         // no active lane
    for (int l = 0; l < 32; l++) dir_addr[l] = 257 + 4 * l;
    run_access(1'b0, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, (LPB == 32) ? 2 : NG);                  // unaligned: each lane 2 dwords
    // ---- directed (stores)
    for (int l = 0; l < 32; l++) begin dir_addr[l] = 1024 + 4 * l; wdz[l] = {96'd0, 32'hdead0000 + l}; end
    run_access(1'b1, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, NG);
    for (int l = 0; l < 32; l++) dir_addr[l] = 2048;                                // same address from every lane
    run_access(1'b1, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b1, 32);                 // ordered: last lane wins
    for (int l = 0; l < 32; l++) dir_addr[l] = 2048 + 2 * l;                        // overlapping 4 B stores
    run_access(1'b1, 32'hffffffff, 4, dir_addr, wdz, rr, 1'b0, 0);
    for (int l = 0; l < 32; l++) dir_addr[l] = 4096 + 5 * l;                        // odd stride bytes
    run_access(1'b1, 32'hffffffff, 1, dir_addr, wdz, rr, 1'b0, 0);
    readback_check();

    // --
    for (int it = 0; it < 4000; it++) begin
      int nb1, fam, stride, base;
      bit wr;
      logic [31:0] ex;
      logic [127:0] wd [32];
      int ad [32];
      int er;
      case ($urandom_range(0, 5)) 0: nb1 = 1; 1: nb1 = 2; 2: nb1 = 4; 3: nb1 = 8; 4: nb1 = 12; default: nb1 = 16; endcase
      wr = ($urandom_range(0, 2) == 0);
      fam = $urandom_range(0, 5);
      stride = $urandom_range(0, 40);
      base = $urandom_range(0, BYTES - 1 - 32 * 48 - 16) & ((nb1 >= 4 && $urandom_range(0,1)) ? ~3 : ~0);
      ex = ($urandom_range(0, 3) == 0) ? 32'($urandom()) : 32'hffffffff;
      if ($urandom_range(0, 9) == 0) ex = 32'd1 << $urandom_range(0, 31);
      for (int l = 0; l < 32; l++) begin
        case (fam)
          0: ad[l] = base + stride * l;                         // strided
          1: ad[l] = base + (l * 4) % 128 + 128 * $urandom_range(0, 3);   // random line, in-line lanes
          2: ad[l] = base + $urandom_range(0, 2000);            // random
          3: ad[l] = base + 4 * $urandom_range(0, 8);           // heavy sharing
          4: ad[l] = base + (l % 4) * 4 + 128 * (l / 4);        // 4-lane groups, one bank-set per group
          default: ad[l] = base + ((l * 37) % 256);
        endcase
        if (ad[l] + 16 >= BYTES) ad[l] = BYTES - 16 - l;
        wd[l] = {$urandom(), $urandom(), $urandom(), $urandom()};
      end
      run_access(wr, ex, nb1, ad, wd, er, 1'b1, rounds_expected(ex, wr, nb1, ad));
    end
    readback_check();

    $display("NG_LSU_SHMEM instrs=%0d rounds=%0d checks=%0d errors=%0d", instr_total, rounds_total, checks, errors);
    if (errors == 0) $display("NG_LSU_SHMEM_TB_PASS");
    else $display("NG_LSU_SHMEM_TB_FAIL");
    $finish;
  end

  initial begin
    repeat (8) #1000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
