// tb_sp_sched_trace.sv -- scenario-driven trace TB for ng_sp_sched.
//
// Runs one scenario produced by the model-side generator
// (rtl/gpu/tb/model_trace/gen_scenarios.py):
//   <dir>/scen.cfg   "base <hex>" / "words <n>" / "launch <wg> <nwarps> <hexpc>"
//                    lines (launches applied in order at t=0) / "end DONE|TRAP"
//   <dir>/code.hex   32-bit little-endian words from base, $readmemh format
// and writes one text line per warp commit / trap event:
//   "C <slot> <pc> <mnemonic>" / "T <slot> <pc> <cause> <info>"
// The run script diffs this file against the functional model's trace
// (commit hooks) for the same launches.
//
// Scenario selection: `NG_TRACE_SCEN "<path>" (via tb_sp_sched_trace_scen.svh
// written by the run script; xsim.bat cannot forward valued plusargs on
// Windows).  Trace path: <scen>/rtl.trace.
// Prints NG_SP_SCHED_TRACE_TB_PASS on success.
`timescale 1ns/1ps
`ifdef NG_TRACE_SCEN_FILE
  `include "tb_sp_sched_trace_scen.svh"
`endif
module tb_sp_sched_trace;
  `include "tb_sp_sched_common.svh"

  string      scen = "scen";
  string      trace_path = "";
  string      endkind = "?";
  string      tok;
  int         fd_cfg, fd_t;
  int         nwords = 0;
  logic [31:0] base = 32'h1000;
  int         lwg_q [$], lnw_q [$];
  logic [31:0] lpc_q [$];
  int         total_warps = 0;
  bit         trap_seen = 0, done_all = 0, failed = 0;
  int         stall_cnt = 0;
  int         maxcycles = 200000;
`ifdef NG_TRACE_SCEN
  initial begin
    scen = `NG_TRACE_SCEN;
  end
`endif

  // ------------------------------------------------------------ trace writer
  // After the first trap no further commits are recorded: the model aborts
  // the whole dispatch at the first trap, so its trace ends there too.
  always @(posedge clk) begin
    if (rst_n && !trap_seen && ev_commit_valid_d)
      $fwrite(fd_t, "C %02d %08h %s\n", ev_slot_d, ev_pc_d, CLS_NAME[ev_class_d]);
    if (rst_n && !trap_seen && ev_trap_valid_d) begin
      trap_seen = 1;
      $fwrite(fd_t, "T %02d %08h %0d %08h\n", ev_trap_slot_d, ev_trap_pc_d,
              ev_trap_cause_d, ev_trap_info_d);
      if (endkind == "DONE") begin
        failed = 1;
        $display("FAIL: unexpected trap (scenario expects DONE) @%0t", $time);
      end
    end
  end

  // ----------------------------------------------------------
  always @(posedge clk) begin
    if (stalled_all_d) stall_cnt++;
    else stall_cnt = 0;
    if (stall_cnt > 512) begin
      $display("FAIL: all resident warps blocked for >512 cycles (deadlock) @%0t", $time);
      failed = 1;
    end
  end

  initial begin : watchdog
    repeat (maxcycles) @(posedge clk);
    $display("FAIL: timeout after %0d cycles", maxcycles);
    failed = 1;
  end

  // ----------------------------------------------------------
  initial begin
    trace_path = {scen, "/rtl.trace"};

    fd_cfg = $fopen({scen, "/scen.cfg"}, "r");
    if (fd_cfg == 0) begin
      $display("FAIL: cannot open %s/scen.cfg", scen);
      $finish;
    end
    while ($fscanf(fd_cfg, "%s", tok) == 1) begin
      case (tok)
        "base":   void'($fscanf(fd_cfg, "%h", base));
        "words":  void'($fscanf(fd_cfg, "%d", nwords));
        "launch": begin
          int wg, nw;
          logic [31:0] pc;
          void'($fscanf(fd_cfg, "%d %d %h", wg, nw, pc));
          lwg_q.push_back(wg);
          lnw_q.push_back(nw);
          lpc_q.push_back(pc);
        end
        "end":    void'($fscanf(fd_cfg, "%s", endkind));
        default:  $display("FAIL: bad scen.cfg token %s", tok);
      endcase
    end
    $fclose(fd_cfg);

    load_imem({scen, "/code.hex"}, base, nwords);
    do_reset();
    fd_t = $fopen(trace_path, "w");

    for (int i = 0; i < lwg_q.size(); i++) begin
      tb_launch(lwg_q[i], lnw_q[i], lpc_q[i]);
      total_warps += lnw_q[i];
    end

    // run to completion: every warp ends, or the first trap stops the test
    // (model policy: a trap aborts the whole dispatch).
    while (!failed && !trap_seen && n_dones < total_warps) @(posedge clk);
    repeat (16) @(posedge clk);  // drain event pulses
    $fclose(fd_t);

    if (!failed && endkind == "TRAP" && !trap_seen) begin
      $display("FAIL: scenario expects a trap but none was reported");
      failed = 1;
    end
    if (!failed && endkind == "DONE" && n_dones != total_warps) begin
      $display("FAIL: %0d/%0d warps finished", n_dones, total_warps);
      failed = 1;
    end
    if (failed) begin
      $display("NG_SP_SCHED_TRACE_TB_FAIL");
      $finish;
    end
    $display("NG_SP_SCHED_TRACE_TB_PASS %s (%0d commits)", scen, n_commits);
    $finish;
  end
endmodule
