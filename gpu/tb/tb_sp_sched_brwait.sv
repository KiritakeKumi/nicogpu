// tb_sp_sched_brwait.sv -- a branch only parks its own warp (W_BRWAIT).
//
// WG0 (slot 0) runs  bnz u29 (not taken for warp 0) ; other; endpgm  and the fake
// branch unit resolves its branch br_dly[0] cycles late.  WG1 (slot 1) runs a
// straight line of NOTHER pipe ops.  While slot 0 waits for its branch the front end
// must keep issuing slot 1: the slot-1 commits made before the slot-0 branch commit
// are counted (the previous S_BR state stalled every warp: count was 0).  Also
// checks the per-warp commit sequences, with the branch commit landing in the same
// cycle as a scheduler commit (parked-commit path) covered by the nop stream of slot 1.
// Prints NG_SP_SCHED_BRWAIT_TB_PASS on success.
`timescale 1ns/1ps
module tb_sp_sched_brwait;
  `include "tb_sp_sched_common.svh"

  localparam int NOTHER = 8;
  longint t_br0 = -1;
  int     n1_before = 0;
  int     n1 = 0;
  int     seq0 [$];
  int     seq1 [$];

  always @(posedge clk) if (rst_n && ev_commit_valid_d) begin
    if (ev_slot_d == 0) begin
      seq0.push_back(int'(ev_pc_d));
      if (ev_class_d == CLS_BNZ) t_br0 = $time;
    end
    if (ev_slot_d == 1) begin
      seq1.push_back(int'(ev_pc_d));
      if (t_br0 < 0) n1_before++;
      n1++;
    end
  end

  initial begin : watchdog
    repeat (20000) @(posedge clk);
    $display("FAIL: timeout");
    $display("NG_SP_SCHED_BRWAIT_TB_FAIL");
    $finish;
  end

  task automatic run_case(input int dly, input int nnop);
    t_br0 = -1; n1_before = 0; n1 = 0;
    seq0.delete(); seq1.delete();
    foreach (imem[i]) imem[i] = '0;
    imem_base = 32'h1000; imem_words = 64;
    imem[0] = w_bop(1, 29, 16);         // bnz u29, +16 (warp 0: u29 == 0 -> not taken)
    imem[1] = 32'h0000_0102;            // other
    imem[2] = w_sys(1, 0);              // endpgm
    for (int i = 0; i < NOTHER; i++) imem[8 + i] = 32'h0000_0102;   // WG1 stream at 0x1020
    for (int i = 0; i < nnop; i++) imem[8 + NOTHER + i] = w_sys(0, 0);
    imem[8 + NOTHER + nnop] = w_sys(1, 0);
    do_reset();
    for (int i = 0; i < 16; i++) br_dly[i] = 0;
    br_dly[0] = dly;
    for (int s = 0; s < 16; s++) slot_busy[s] = 1'b0;
    n_dones = 0; n_commits = 0;
    tb_launch(0, 1, 32'h1000);
    tb_launch(1, 1, 32'h1020);
    wait (n_dones >= 2);
    repeat (6) @(posedge clk);
    check(seq0.size() == 3, $sformatf("slot0 commits %0d != 3", seq0.size()));
    if (seq0.size() == 3)
      check(seq0[0] == 32'h1000 && seq0[1] == 32'h1004 && seq0[2] == 32'h1008, "slot0 pc sequence");
    check(seq1.size() == NOTHER + nnop + 1, $sformatf("slot1 commits %0d", seq1.size()));
    for (int i = 0; i < seq1.size(); i++)
      check(seq1[i] == 32'h1020 + 4 * i, $sformatf("slot1 pc[%0d] = %h", i, seq1[i]));
    $display("BRWAIT dly=%0d: slot-1 commits before the slot-0 branch commit = %0d", dly, n1_before);
    if (dly >= 40) check(n1_before >= 6, $sformatf("slot 1 stalled by slot 0's pending branch (%0d)", n1_before));
  endtask

  initial begin
    foreach (imem[i]) imem[i] = '0;
    run_case(0, 0);
    run_case(1, 3);
    run_case(2, 0);
    run_case(40, 4);
    run_case(40, 0);
    if (errors == 0) $display("NG_SP_SCHED_BRWAIT_TB_PASS");
    else             $display("NG_SP_SCHED_BRWAIT_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
