// tb_sp_sched_barrier.sv -- workgroup barrier rendezvous:
//   1. two warps of one WG arrive together; release; both continue
//   2. staggered arrival: bz u29 splits the warps onto paths of different
//      length; the early warp parks in W_BAR while the other catches up
//   3. partial arrival: two of three warps endpgm before the barrier; the
//      barrier releases with the single remaining arrival (n - done rule)
//   4. two WGs with independent barriers running interleaved
//   5. workgroup retire frees the tag for relaunch
// Prints NG_SP_SCHED_BARRIER_TB_PASS on success.
`timescale 1ns/1ps
module tb_sp_sched_barrier;
  `include "tb_sp_sched_common.svh"

  task automatic expect_commit(input int slot, input logic [31:0] pc, input int cls,
                               input string msg);
    // warp-selective issue changes the global interleaving; commits are checked
    // per warp (the next commit of `slot` in arrival order), not as one global sequence.
    int ix;
    ix = -1;
    for (int i = 0; i < commit_slot_q.size(); i++)
      if (commit_slot_q[i] == slot) begin ix = i; break; end
    check(ix >= 0, {msg, ": no pending commit of this slot"});
    if (ix >= 0) begin
      check(commit_pc_q[ix] == pc, $sformatf("%s: pc %08h != %08h", msg, commit_pc_q[ix], pc));
      check(commit_cls_q[ix] == cls, $sformatf("%s: cls %0d != %0d", msg, commit_cls_q[ix], cls));
      commit_slot_q.delete(ix);
      commit_pc_q.delete(ix);
      commit_cls_q.delete(ix);
    end
  endtask

  initial begin : watchdog
    repeat (20000) @(posedge clk);
    $display("FAIL: timeout");
    $display("NG_SP_SCHED_BARRIER_TB_FAIL");
    $finish;
  end

  initial begin
    imem_base = 32'h1000;
    // A @0x00: nop / barrier / nop / endpgm                       (scenario 1)
    imem[0] = w_sys(0, 0);
    imem[1] = w_sys(6, 0);         // barrier
    imem[2] = w_sys(0, 0);
    imem[3] = w_sys(1, 0);
    // B @0x20: rewritten in scenario 2 (bz u29 splits, staggered arrival)
    // C @0x50: bz u29 -> +16; nop nop endpgm @..; barrier; endpgm (scenario 3)
    imem[20] = w_bop(0, 29, 16);   // bz u29 -> +16
    imem[21] = w_sys(0, 0);
    imem[22] = w_sys(0, 0);
    imem[23] = w_sys(1, 0);        // endpgm @+12
    imem[24] = w_sys(6, 0);        // barrier @+16
    imem[25] = w_sys(1, 0);        // endpgm @+20
    imem_words = 26;

    do_reset();

    // ---- 1: symmetric rendezvous --------------------------------------------
    tb_launch(0, 2, 32'h1000);
    wait (n_dones >= 2);
    expect_commit(0, 32'h1000, CLS_NOP, "b1-w0-nop");
    expect_commit(1, 32'h1000, CLS_NOP, "b1-w1-nop");
    expect_commit(0, 32'h1004, CLS_BARRIER, "b1-w0-bar");
    expect_commit(1, 32'h1004, CLS_BARRIER, "b1-w1-bar");
    expect_commit(0, 32'h1008, CLS_NOP, "b1-w0-nop2");
    expect_commit(1, 32'h1008, CLS_NOP, "b1-w1-nop2");
    expect_commit(0, 32'h100c, CLS_ENDPGM, "b1-w0-end");
    expect_commit(1, 32'h100c, CLS_ENDPGM, "b1-w1-end");

    // ---- 2: staggered arrival via bz u29 -------------------------------------
    // program B at 0x1020 (words 8..15): bz +24 -> barrier at 0x1038; the
    // fall-through path walks 5 nops into the same barrier.  warp 0 (u29=0)
    // jumps straight to the barrier; warp 1 catches up later.
    for (int i = 0; i < 8; i++) imem[8 + i] = w_sys(0, 0);
    imem[8]  = w_bop(0, 29, 24);   // bz u29 -> 0x1020+24 = 0x1038
    imem[14] = w_sys(6, 0);        // barrier at word 14 = 0x1038
    imem[15] = w_sys(1, 0);        // endpgm at 0x103c
    tb_launch(0, 2, 32'h1020);
    wait (n_dones >= 4);
    expect_commit(0, 32'h1020, CLS_BZ, "b2-w0-bz");
    expect_commit(1, 32'h1020, CLS_BZ, "b2-w1-bz");
    expect_commit(0, 32'h1038, CLS_BARRIER, "b2-w0-bar");     // warp 0 first
    expect_commit(1, 32'h1024, CLS_NOP, "b2-w1-nop1");
    expect_commit(1, 32'h1028, CLS_NOP, "b2-w1-nop2");
    expect_commit(1, 32'h102c, CLS_NOP, "b2-w1-nop3");
    expect_commit(1, 32'h1030, CLS_NOP, "b2-w1-nop4");
    expect_commit(1, 32'h1034, CLS_NOP, "b2-w1-nop5");
    expect_commit(1, 32'h1038, CLS_BARRIER, "b2-w1-bar");     // release
    expect_commit(0, 32'h103c, CLS_ENDPGM, "b2-w0-end");
    expect_commit(1, 32'h103c, CLS_ENDPGM, "b2-w1-end");

    // ---- 3: partial arrival (2 of 3 warps end before the barrier) ------------
    // The RR pointer carries over from scenario 2 (last grant slot 1), so the
    // cyclic issue order starts at slot 2: C, A, B.  (Trace-equality
    // scenarios launch everything at t=0 where cyclic order == the model's
    // per-step order; mid-run launches keep the hardware rotation.)
    tb_launch(0, 3, 32'h1050);
    wait (n_dones >= 7);
    expect_commit(2, 32'h1050, CLS_BZ, "b3-w2-bz");
    expect_commit(0, 32'h1050, CLS_BZ, "b3-w0-bz");
    expect_commit(1, 32'h1050, CLS_BZ, "b3-w1-bz");
    expect_commit(2, 32'h1054, CLS_NOP, "b3-w2-nop");
    expect_commit(0, 32'h1060, CLS_BARRIER, "b3-w0-bar");
    expect_commit(1, 32'h1054, CLS_NOP, "b3-w1-nop");
    expect_commit(2, 32'h1058, CLS_NOP, "b3-w2-nop2");
    expect_commit(1, 32'h1058, CLS_NOP, "b3-w1-nop2");
    expect_commit(2, 32'h105c, CLS_ENDPGM, "b3-w2-end");
    expect_commit(1, 32'h105c, CLS_ENDPGM, "b3-w1-end");
    expect_commit(0, 32'h1064, CLS_ENDPGM, "b3-w0-end");

    // ---- 4+5: two independent WGs, tag reuse ---------------------------------
    tb_launch(0, 1, 32'h1000);
    tb_launch(1, 2, 32'h1000);
    wait (n_dones >= 7 + 3);
    tb_launch(1, 1, 32'h1000);     // tag 1 free again after retire
    wait (n_dones >= 7 + 4);
    check(n_dones == 7 + 4, "tag reuse after retire completes");

    if (errors == 0) $display("NG_SP_SCHED_BARRIER_TB_PASS");
    else             $display("NG_SP_SCHED_BARRIER_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
