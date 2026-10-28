// tb_sp_sched_trap.sv -- trap detection and reporting:
//   1. reserved length encoding (w0[1:0]=11): illegal-instruction trap with
//      info = the raw word
//   2. SYS with a reserved f5, and nop/endpgm with nonzero imm: illegal
//   3. explicit `trap imm`: cause 2, info = imm; the word is not committed
//   4. `brk`: cause 3
//   5. fetch outside the mapped code window: MemFault with info = pc
//   6. a trapped warp stays halted; its WG tag stays allocated; sibling
//      warps of the same SP keep issuing
//   7. (after a reset) cache.inv.l1 / cache.inv.l2 / cache.wb.l2
//      (SYS f5 9/0xa/0xb) issue to the pipe as CLS_OTHER; with a nonzero imm
//      they are illegal; interlock.begin/end (f5 7/8, no gfx unit) and the
//      reserved f5 0xd / 0x1f stay illegal
// Prints NG_SP_SCHED_TRAP_TB_PASS on success.
`timescale 1ns/1ps
module tb_sp_sched_trap;
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

  task automatic expect_trap(input int slot, input logic [31:0] pc, input int cause,
                             input logic [31:0] info, input string msg);
    check(n_traps >= 1, {msg, ": no trap reported"});
    check(last_trap_slot == slot, $sformatf("%s: slot %0d != %0d", msg, last_trap_slot, slot));
    check(last_trap_pc == pc, $sformatf("%s: pc %08h != %08h", msg, last_trap_pc, pc));
    check(last_trap_cause == cause, $sformatf("%s: cause %0d != %0d", msg, last_trap_cause, cause));
    check(last_trap_info == info, $sformatf("%s: info %08h != %08h", msg, last_trap_info, info));
    n_traps = 0;
  endtask

  initial begin : watchdog
    repeat (20000) @(posedge clk);
    $display("FAIL: timeout");
    $display("NG_SP_SCHED_TRAP_TB_FAIL");
    $finish;
  end

  int commits0;
  always @(posedge clk) if (rst_n && ev_commit_valid_d && ev_slot_d == 4'd0) commits0++;

  initial begin
    imem_base = 32'h1000;
    imem_words = 40;
    // A @0x00: nop / raw 0x3 (reserved length)                    (cases 1/6)
    imem[0] = w_sys(0, 0);
    imem[1] = 32'h0000_0003;
    // B @0x20: SYS f5=0x7 reserved / nop-with-imm / endpgm-with-imm (case 2)
    imem[8]  = w_sys(7, 0);
    imem[9]  = w_sys(0, 5);
    imem[10] = w_sys(1, 1);
    // C @0x40: nop / trap 0x15 / nop                               (case 3)
    imem[16] = w_sys(0, 0);
    imem[17] = w_sys(2, 32'h15);
    imem[18] = w_sys(0, 0);
    // D @0x5c: brk                                                  (case 4)
    imem[23] = w_sys(3, 0);
    // E @0x60: jmp +0x100 -> 0x1160 (outside the window)            (case 5)
    imem[24] = w_jmp(32'h100);
    // F @0x60..: bz u29 splits: warp0 -> raw 0x3, warp1 finishes    (case 6)
    // (E and F share word 24; F is written in case 6)
    // G @0x80: cache.inv.l1 / cache.inv.l2 / cache.wb.l2 / endpgm   (case 7)
    imem[32] = w_sys(5'h9, 0);
    imem[33] = w_sys(5'ha, 0);
    imem[34] = w_sys(5'hb, 0);
    imem[35] = w_sys(1, 0);
    // H @0x90..: cache.inv.l1 imm=1 / interlock.end / f5 0xd / f5 0x1f (case 7, one launch each)
    imem[36] = w_sys(5'h9, 1);
    imem[37] = w_sys(5'h8, 0);
    imem[38] = w_sys(5'hd, 0);
    imem[39] = w_sys(5'h1f, 0);

    do_reset();

    // ---- 1: reserved length encoding ----------------------------------------
    tb_launch(0, 1, 32'h1000);
    wait (n_traps >= 1);
    expect_trap(0, 32'h1004, CAUSE_ILLEGAL, 32'h0000_0003, "t1-ill-len");
    expect_commit(0, 32'h1000, CLS_NOP, "t1-nop");
    repeat (20) @(posedge clk);
    check(commits0 == 1, "trapped warp stays halted");

    // ---- 2: reserved SYS encodings (new tags: 0 stays allocated) -------------
    tb_launch(1, 1, 32'h1020);
    wait (n_traps >= 1);
    expect_trap(1, 32'h1020, CAUSE_ILLEGAL, w_sys(7, 0), "t2-res-f5");
    tb_launch(2, 1, 32'h1024);
    wait (n_traps >= 1);
    expect_trap(2, 32'h1024, CAUSE_ILLEGAL, w_sys(0, 5), "t2-nop-imm");
    tb_launch(3, 1, 32'h1028);
    wait (n_traps >= 1);
    expect_trap(3, 32'h1028, CAUSE_ILLEGAL, w_sys(1, 1), "t2-endpgm-imm");

    // ---- 3: explicit trap ----------------------------------------------------
    tb_launch(4, 1, 32'h1040);
    wait (n_traps >= 1);
    expect_trap(4, 32'h1044, CAUSE_EXPLICIT, 32'h15, "t3-explicit");
    expect_commit(4, 32'h1040, CLS_NOP, "t3-nop");

    // ---- 4: brk ---------------------------------------------------------------
    tb_launch(5, 1, 32'h105c);
    wait (n_traps >= 1);
    expect_trap(5, 32'h105c, CAUSE_BRK, 32'h0, "t4-brk");

    // ---- 5: fetch fault ------------------------------------------------------
    tb_launch(6, 1, 32'h1060);
    wait (n_traps >= 1);
    expect_commit(6, 32'h1060, CLS_JMP, "t5-jmp");
    expect_trap(6, 32'h1160, CAUSE_MEMFAULT, 32'h0000_1160, "t5-fault");
    // trapped warps never retire their workgroup: the tag stays allocated
    launch_valid  = 1'b1;
    launch_wg     = 3'd0;
    launch_nwarps = 5'd1;
    launch_pc     = 32'h1000;
    @(posedge clk);
    check(launch_ready_d == 1'b0, "trapped WG keeps its tag allocated");
    launch_valid = 1'b0;

    // ---- 6: sibling warps keep issuing after a trap ---------------------------
    // warp0 (u29=0) branches to the raw word and traps; warp1 runs to endpgm
    imem[24] = w_bop(0, 29, 16);    // bz u29 -> 0x1070
    imem[25] = w_sys(0, 0);
    imem[26] = w_sys(0, 0);
    imem[27] = w_sys(1, 0);         // warp1 endpgm @0x106c
    imem[28] = 32'h0000_0003;       // warp0 traps here @0x1070
    tb_launch(7, 2, 32'h1060);
    wait (n_traps >= 1);
    wait (n_dones >= 1);          // warp1 finishes after the sibling traps
    repeat (4) @(posedge clk);
    expect_trap(7, 32'h1070, CAUSE_ILLEGAL, 32'h0000_0003, "t6-w0-trap");
    expect_commit(7, 32'h1060, CLS_BZ, "t6-w0-bz");
    expect_commit(8, 32'h1060, CLS_BZ, "t6-w1-bz");
    expect_commit(8, 32'h1064, CLS_NOP, "t6-w1-nop1");
    expect_commit(8, 32'h1068, CLS_NOP, "t6-w1-nop2");
    expect_commit(8, 32'h106c, CLS_ENDPGM, "t6-w1-end");
    check(n_dones == 1, "one warp finished, the trapped one did not");

    // ---- 7: cache ops (every WG tag is held by a trapped warp: reset first) ----
    do_reset();
    for (int s = 0; s < 16; s++) slot_busy[s] = 1'b0;
    commit_slot_q.delete(); commit_pc_q.delete(); commit_cls_q.delete();
    n_traps = 0;
    tb_launch(0, 1, 32'h1080);
    wait (n_dones >= 2);
    repeat (2) @(posedge clk);
    check(n_traps == 0, "t7: cache ops raised a trap");
    expect_commit(0, 32'h1080, CLS_OTHER, "t7-inv-l1");
    expect_commit(0, 32'h1084, CLS_OTHER, "t7-inv-l2");
    expect_commit(0, 32'h1088, CLS_OTHER, "t7-wb-l2");
    expect_commit(0, 32'h108c, CLS_ENDPGM, "t7-end");
    tb_launch(1, 1, 32'h1090);
    wait (n_traps >= 1);
    expect_trap(0, 32'h1090, CAUSE_ILLEGAL, w_sys(5'h9, 1), "t7-inv-l1-imm");
    tb_launch(2, 1, 32'h1094);
    wait (n_traps >= 1);
    expect_trap(1, 32'h1094, CAUSE_ILLEGAL, w_sys(5'h8, 0), "t7-interlock-end");
    tb_launch(3, 1, 32'h1098);
    wait (n_traps >= 1);
    expect_trap(2, 32'h1098, CAUSE_ILLEGAL, w_sys(5'hd, 0), "t7-res-f5-d");
    tb_launch(4, 1, 32'h109c);
    wait (n_traps >= 1);
    expect_trap(3, 32'h109c, CAUSE_ILLEGAL, w_sys(5'h1f, 0), "t7-res-f5-1f");

    if (errors == 0) $display("NG_SP_SCHED_TRAP_TB_PASS");
    else             $display("NG_SP_SCHED_TRAP_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
