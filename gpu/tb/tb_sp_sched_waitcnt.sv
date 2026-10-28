// tb_sp_sched_waitcnt.sv -- waitcnt blocking and recovery.
// Async-op accounting is driven manually (cnt_inc/cnt_dec pulses on empty or
// running slots -- the counters are per-slot storage, independent of warp
// state), so every check is deterministic:
//   1. vm=0 with outstanding ops blocks the warp; another warp keeps issuing
//      (no head-of-line blocking); stalled_all asserts once only the waiter
//      remains; in-order cnt_dec release makes the waitcnt commit once
//   2. threshold: vm=1 passes with one outstanding op, blocks with two
//   3. all-ones fields never block
//   4. sm=1..2 act as 0 (wait for empty); sm=3 does not wait
//   5. counter independence: sfu outstanding does not block a vm=0 waitcnt
//   6. saturation: 70 incs cap at 63 (no wrap); 63 decs drain to zero
// Prints NG_SP_SCHED_WAITCNT_TB_PASS on success.
`timescale 1ns/1ps
module tb_sp_sched_waitcnt;
  `include "tb_sp_sched_common.svh"

  int c0;  // commits seen from slot 0
  always @(posedge clk) if (rst_n && ev_commit_valid_d && ev_slot_d == 4'd0) c0++;

  task automatic pulse_inc(input int slot, input int cnt);
    cnt_inc_valid_r <= 1'b1;
    cnt_inc_slot_r  <= 4'(slot);
    cnt_inc_id_r    <= 3'(cnt);
    @(posedge clk);
    cnt_inc_valid_r <= 1'b0;
    @(posedge clk);
  endtask

  task automatic pulse_dec(input int slot, input int cnt);
    cnt_dec_valid_r <= 1'b1;
    cnt_dec_slot_r  <= 4'(slot);
    cnt_dec_id_r    <= 3'(cnt);
    @(posedge clk);
    cnt_dec_valid_r <= 1'b0;
    @(posedge clk);
  endtask

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
    $display("NG_SP_SCHED_WAITCNT_TB_FAIL");
    $finish;
  end

  initial begin
    imem_base = 32'h1000;
    // A @0x00: wait vm=0 / nop / endpgm          (scenario 1, slot 0)
    imem[0] = w_waitcnt(0, 31, 31, 15, 3, 7);
    imem[1] = w_sys(0, 0);
    imem[2] = w_sys(1, 0);
    // B @0x20: nop x5 / endpgm                    (slot 1 companion)
    for (int i = 0; i < 5; i++) imem[8 + i] = w_sys(0, 0);
    imem[13] = w_sys(1, 0);
    // C @0x40: wait vm=1 / nop / endpgm           (scenario 2)
    imem[16] = w_waitcnt(1, 31, 31, 15, 3, 7);
    imem[17] = w_sys(0, 0);
    imem[18] = w_sys(1, 0);
    // E @0x60: wait all-ones / endpgm             (scenario 3)
    imem[24] = w_waitcnt(63, 31, 31, 15, 3, 7);
    imem[25] = w_sys(1, 0);
    // F @0x70: wait sm=1 / nop / endpgm           (scenario 4a)
    imem[28] = w_waitcnt(63, 31, 31, 15, 1, 7);
    imem[29] = w_sys(0, 0);
    imem[30] = w_sys(1, 0);
    // F2 @0x80: wait sm=3 / endpgm                (scenario 4b)
    imem[32] = w_waitcnt(63, 31, 31, 15, 3, 7);
    imem[33] = w_sys(1, 0);
    // G @0x90: wait vm=0 / endpgm                 (scenario 5)
    imem[36] = w_waitcnt(0, 31, 31, 15, 3, 7);
    imem[37] = w_sys(1, 0);
    // H @0xa0: wait vm=0 / endpgm                 (scenario 6)
    imem[40] = w_waitcnt(0, 31, 31, 15, 3, 7);
    imem[41] = w_sys(1, 0);
    imem_words = 42;

    do_reset();

    // ---- 1: block, no head-of-line blocking, release ----------------------
    // The pipe is single-issue, so slot 1 only runs after slot 0 blocks.
    // Hold slot 0's first fetch (if_req_ready = 0) until the vm counter is
    // loaded, so the waitcnt decision happens with vm = 3 outstanding.
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1000);
    tb_launch(1, 1, 32'h1020);
    repeat (3) pulse_inc(0, CNT_VM);
    if_req_ready_r = 1'b1;
    wait (n_dones >= 1);               // slot 1 runs to completion meanwhile
    repeat (2) @(posedge clk);
    check(c0 == 0, "waitcnt blocked while vm outstanding");
    check(stalled_all_d == 1'b1, "stalled_all with only the waiter left");
    repeat (3) pulse_dec(0, CNT_VM);   // drain in order -> unblock
    wait (n_dones >= 2);
    repeat (2) @(posedge clk);
    // slot 1 committed its five nops + endpgm while slot 0 was blocked
    expect_commit(1, 32'h1020, CLS_NOP, "s1-nop1");
    expect_commit(1, 32'h1024, CLS_NOP, "s1-nop2");
    expect_commit(1, 32'h1028, CLS_NOP, "s1-nop3");
    expect_commit(1, 32'h102c, CLS_NOP, "s1-nop4");
    expect_commit(1, 32'h1030, CLS_NOP, "s1-nop5");
    expect_commit(1, 32'h1034, CLS_ENDPGM, "s1-end");
    expect_commit(0, 32'h1000, CLS_WAITCNT, "w1-wait");
    expect_commit(0, 32'h1004, CLS_NOP, "w1-nop");
    expect_commit(0, 32'h1008, CLS_ENDPGM, "w1-end");
    check(c0 == 3, "waitcnt committed exactly once");

    // ---- 2: threshold vm=1 --------------------------------------------------
    // (fetch held until the counter is loaded, as in scenario 1)
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1040);
    pulse_inc(0, CNT_VM);                   // 1 outstanding
    if_req_ready_r = 1'b1;
    wait (n_dones >= 3);
    expect_commit(0, 32'h1040, CLS_WAITCNT, "w2-pass");
    expect_commit(0, 32'h1044, CLS_NOP, "w2-pass-nop");
    expect_commit(0, 32'h1048, CLS_ENDPGM, "w2-pass-end");
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1040);
    pulse_inc(0, CNT_VM);
    pulse_inc(0, CNT_VM);                   // 2 outstanding total
    if_req_ready_r = 1'b1;
    repeat (20) @(posedge clk);
    check(c0 == 6, "vm=1 blocks with two outstanding");
    pulse_dec(0, CNT_VM);                   // back to 1: <= threshold
    wait (n_dones >= 4);
    expect_commit(0, 32'h1040, CLS_WAITCNT, "w2-release");
    expect_commit(0, 32'h1044, CLS_NOP, "w2-release-nop");
    expect_commit(0, 32'h1048, CLS_ENDPGM, "w2-release-end");
    pulse_dec(0, CNT_VM);

    // ---- 3: all-ones never blocks -------------------------------------------
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1060);
    repeat (5) pulse_inc(0, CNT_VM);
    if_req_ready_r = 1'b1;
    wait (n_dones >= 5);
    expect_commit(0, 32'h1060, CLS_WAITCNT, "w3-skip");
    expect_commit(0, 32'h1064, CLS_ENDPGM, "w3-skip-end");
    repeat (5) pulse_dec(0, CNT_VM);

    // ---- 4: sm special values ------------------------------------------------
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1070);
    pulse_inc(0, CNT_SM);
    pulse_inc(0, CNT_SM);
    if_req_ready_r = 1'b1;
    repeat (20) @(posedge clk);
    check(c0 == 11, "sm=1 acts as 0 and blocks");
    pulse_dec(0, CNT_SM);
    pulse_dec(0, CNT_SM);
    wait (n_dones >= 6);
    expect_commit(0, 32'h1070, CLS_WAITCNT, "w4a-release");
    expect_commit(0, 32'h1074, CLS_NOP, "w4a-release-nop");
    expect_commit(0, 32'h1078, CLS_ENDPGM, "w4a-release-end");
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1080);
    pulse_inc(0, CNT_SM);
    pulse_inc(0, CNT_SM);
    if_req_ready_r = 1'b1;                  // sm=3 field: skip despite sm ops
    wait (n_dones >= 7);
    expect_commit(0, 32'h1080, CLS_WAITCNT, "w4b-skip");
    expect_commit(0, 32'h1084, CLS_ENDPGM, "w4b-skip-end");
    pulse_dec(0, CNT_SM);
    pulse_dec(0, CNT_SM);

    // ---- 5: counter independence ---------------------------------------------
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h1090);              // vm=0: passes despite sfu ops
    repeat (3) pulse_inc(0, CNT_SFU);
    if_req_ready_r = 1'b1;
    wait (n_dones >= 8);
    expect_commit(0, 32'h1090, CLS_WAITCNT, "w5-indep");
    expect_commit(0, 32'h1094, CLS_ENDPGM, "w5-indep-end");
    repeat (3) pulse_dec(0, CNT_SFU);

    // ---- 6: saturation at 63 ---------------------------------------------------
    // 70 incs cap the counter at the ISA maximum (vm: 62+1); 63 decs bring it
    // back to 0 (the 7 lost incs are the saturation margin), so the vm=0
    // waitcnt passes immediately afterwards.
    if_req_ready_r = 1'b0;
    tb_launch(0, 1, 32'h10a0);
    for (int i = 0; i < 70; i++) pulse_inc(0, CNT_VM);
    for (int i = 0; i < 63; i++) pulse_dec(0, CNT_VM);
    if_req_ready_r = 1'b1;
    wait (n_dones >= 9);
    expect_commit(0, 32'h10a0, CLS_WAITCNT, "w6-sat");
    expect_commit(0, 32'h10a4, CLS_ENDPGM, "w6-sat-end");

    if (errors == 0) $display("NG_SP_SCHED_WAITCNT_TB_PASS");
    else             $display("NG_SP_SCHED_WAITCNT_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
