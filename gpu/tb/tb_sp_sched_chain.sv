// tb_sp_sched_chain.sv -- front-end throughput of ng_sp_sched.
//
// One warp runs a chain of N independent single-issue "OTHER" ops (the shape
// of a dependent ALU chain as the scheduler sees it) followed by endpgm.  The
// execution pipe is modelled as in ng_sp_top for a fixed-latency unit: after an
// accepted issue it is busy for E cycles (is_ready low, ex_hold high).  The TB
// prints the issue-to-issue interval in cycles per instruction for several
// (E, fetch latency) points and fails if the interval is not bounded by
// max(E, ~front-end floor) + 2 -- the regression bar of the pipelined front end.
// With -d NG_OLD_SCHED the previous RTL is measured instead: there the old
// ng_sp_top also gated fetch while the pipe was busy, modelled here (GATE).
// Prints NG_SP_SCHED_CHAIN_TB_PASS on success.
`timescale 1ns/1ps
module tb_sp_sched_chain;
  `include "tb_sp_sched_common.svh"

  localparam int N = 64;
  int E_list [5] = '{0, 1, 4, 8, 16};
  int L_list [2] = '{0, 2};
  int E = 0;
  bit gate = 1'b0;           // old top: no fetch while the pipe is busy
  int busy_cnt = 0;
  longint t_first, t_last;
  int ncommit_l;

  always @(posedge clk) begin
    if (busy_cnt > 0) busy_cnt = busy_cnt - 1;
    if (is_valid_d && is_rdy_w && E > 0) busy_cnt = E;
    ex_hold_r      <= (busy_cnt > 0);
    is_ready_r     <= !(busy_cnt > 0);
    if_req_ready_r <= !(gate && busy_cnt > 0);
  end

  always @(posedge clk) if (rst_n && ev_commit_valid_d) begin
    if (ncommit_l == 0) t_first = $time;
    t_last = $time;
    ncommit_l++;
  end

  task automatic run_point(input int e, input int lat);
    real cpi;
    E = e;
    done_lat = (e > 0) ? e - 1 : 0;
    fetch_lat = lat;
    do_reset();
    n_commits = 0; n_dones = 0; ncommit_l = 0;
    commit_slot_q.delete(); commit_pc_q.delete(); commit_cls_q.delete();
    for (int i = 0; i < N; i++) imem[i] = 32'h0000_0102;
    imem[N] = w_sys(1, 0);
    imem_base = 32'h1000; imem_words = N + 1;
    for (int s = 0; s < 16; s++) slot_busy[s] = 1'b0;
    tb_launch(0, 1, 32'h1000);
    wait (n_dones >= 1);
    repeat (3) @(posedge clk);
    check(n_commits == N + 1, $sformatf("chain: %0d commits != %0d", n_commits, N + 1));
    cpi = real'(t_last - t_first) / 10.0 / real'(N);
    $display("CHAIN E=%0d fetch_lat=%0d : %0.2f cycles/insn (%0d insns)", e, lat, cpi, N);
    // bar: back-to-back issue is limited by the pipe (E+1: accept..done..next)
    // plus a small constant; the old serial front end needed E + ~8
`ifndef NG_OLD_SCHED
    check(cpi <= real'((e > 4 ? e : 4)) + 2.5,
          $sformatf("chain E=%0d lat=%0d: %0.2f cycles/insn too slow", e, lat, cpi));
`endif
  endtask

  initial begin : watchdog
    repeat (400000) @(posedge clk);
    $display("FAIL: timeout");
    $display("NG_SP_SCHED_CHAIN_TB_FAIL");
    $finish;
  end

  initial begin
`ifdef NG_OLD_SCHED
    gate = 1'b1;
`endif
    foreach (imem[i]) imem[i] = '0;
    foreach (E_list[i])
      foreach (L_list[j]) run_point(E_list[i], L_list[j]);
    if (errors == 0) $display("NG_SP_SCHED_CHAIN_TB_PASS");
    else             $display("NG_SP_SCHED_CHAIN_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
