// tb_sp_sched_basic.sv -- directed basics for ng_sp_sched:
//   1. launch handshake, slot assignment (lowest free), warp_done events
//   2. round-robin commit order over two warps of one workgroup
//   3. endpgm finishes warps; slots and the workgroup tag are reusable
//   4. launch backpressure when the table is full, and while the WG tag is
//      active even with free slots
//   5. an out-of-predecode opcode passes through the issue port as CLS_OTHER
//      and commits (pc += 4, or 8 for a 64-bit encoding)
//   6. issue backpressure stalls the pipeline without losing events
// Prints NG_SP_SCHED_BASIC_TB_PASS on success.
`timescale 1ns/1ps
module tb_sp_sched_basic;
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
    $display("NG_SP_SCHED_BASIC_TB_FAIL");
    $finish;
  end

  initial begin
    // program at 0x1000: nop / nop / endpgm; a second region for the OTHER
    // op test: OTHER(0x00000102) / 64-bit OTHER (w0[1:0]=01) / nop / endpgm
    imem_base  = 32'h1000;
    imem_words = 12;
    imem[0]  = w_sys(0, 0);    // nop
    imem[1]  = w_sys(0, 0);
    imem[2]  = w_sys(1, 0);    // endpgm
    imem[4]  = 32'h0000_0102;  // opcode 0x02: outside the predecoded set
    imem[5]  = 32'h0000_0111;  // w0[1:0]=01: 64-bit encoding, opcode 0x11
    imem[6]  = 32'h0;          // second half of the 64-bit word
    imem[7]  = w_sys(0, 0);
    imem[8]  = w_sys(1, 0);

    do_reset();
    // launch_ready is payload-dependent ("would accept this launch"), so the
    // payload must be presented before checking it
    launch_nwarps <= 5'd1;
    launch_wg     <= 3'd0;
    launch_pc     <= 32'h1000;
    @(posedge clk);
    check(launch_ready_d, "launch_ready after reset");

    // ---- 1+2: one WG, two warps, round-robin ------------------------------
    tb_launch(0, 2, 32'h1000);
    check(last_slot0 == 0, "first launch starts at slot 0");
    wait (n_commits >= 6);
    expect_commit(0, 32'h1000, CLS_NOP, "rr0");
    expect_commit(1, 32'h1000, CLS_NOP, "rr1");
    expect_commit(0, 32'h1004, CLS_NOP, "rr2");
    expect_commit(1, 32'h1004, CLS_NOP, "rr3");
    expect_commit(0, 32'h1008, CLS_ENDPGM, "rr4");
    expect_commit(1, 32'h1008, CLS_ENDPGM, "rr5");
    check(n_dones == 2, "two warps done");

    // ---- 3: reuse of slots and the WG tag after retire ---------------------
    tb_launch(0, 2, 32'h1000);
    check(last_slot0 == 0, "slots reused after retire");
    wait (n_dones >= 4);

    // ---- 4: fill the table; backpressure ----------------------------------
    tb_launch(1, 16, 32'h1000);  // 16 warps x [nop,nop,endpgm]
    repeat (2) @(posedge clk);   // let the post-launch state settle
    check(launch_ready_d == 1'b0, "no slot left after 16-warp launch");
    wait (n_dones >= 4 + 16);
    repeat (2) @(posedge clk);   // last commit event lands with the done event
    // 48 commits from the 16-warp WG plus the 6 unpopped commits of the
    // relaunch in test 3
    check(commit_slot_q.size() == 54, "commit count after the 16-warp WG");
    commit_slot_q.delete();
    commit_pc_q.delete();
    commit_cls_q.delete();

    // ---- 5: CLS_OTHER pass-through, 32- and 64-bit --------------------------
    tb_launch(2, 1, 32'h1010);
    wait (n_commits >= 6 + 6 + 48 + 4);
    expect_commit(0, 32'h1010, CLS_OTHER, "other32");
    expect_commit(0, 32'h1014, CLS_OTHER, "other64");
    expect_commit(0, 32'h101c, CLS_NOP, "other-nop");
    expect_commit(0, 32'h1020, CLS_ENDPGM, "other-end");
    check(n_dones == 4 + 16 + 1, "warp done after OTHER stream");

    // ---- 6: issue backpressure + tag-active launch refusal -----------------
    is_ready_r = 1'b0;
    tb_launch(3, 1, 32'h1010);  // OTHER ops need the issue port
    repeat (6) @(posedge clk);
    check(is_valid_d == 1'b1, "issue request held under backpressure");
    check(n_commits == 6 + 6 + 48 + 4, "no OTHER commit while stalled");
    // tag 3 is active (warp stalled at issue): launch must be refused even
    // though 15 slots are free
    launch_valid  = 1'b1;
    launch_wg     = 3'd3;
    launch_nwarps = 5'd1;
    launch_pc     = 32'h1000;
    @(posedge clk);
    check(launch_ready_d == 1'b0, "launch refused while WG tag active");
    launch_valid = 1'b0;
    is_ready_r = 1'b1;
    wait (n_dones >= 4 + 16 + 1 + 1);
    expect_commit(0, 32'h1010, CLS_OTHER, "bp-other32");
    expect_commit(0, 32'h1014, CLS_OTHER, "bp-other64");
    expect_commit(0, 32'h101c, CLS_NOP, "bp-other-nop");
    expect_commit(0, 32'h1020, CLS_ENDPGM, "bp-other-end");

    if (errors == 0) $display("NG_SP_SCHED_BASIC_TB_PASS");
    else             $display("NG_SP_SCHED_BASIC_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
