// tb_sp_sched_soak.sv -- randomized soak for ng_sp_sched.
//
// Each iteration builds a random forward-jump program (pc never moves back,
// so every warp terminates), launches 1-3 workgroups of 1-3 warps at random
// 4-aligned entry PCs, marks a random subset of OTHER ops async (cnt_inc at
// issue acceptance, cnt_dec after a random delay -- always eventually, so a
// blocked waitcnt always recovers), applies random issue backpressure and
// checks:
//   * every commit's pc equals the warp's independently predicted next pc
//     (prediction from the reported class + the instruction word: jmp offset,
//     branch condition via the TB's uniform view, 4/8-byte length)
//   * a warp stops exactly at endpgm or a trap; no commit after termination
//   * all launched warps reach a terminal state (liveness; stall watchdog)
//   * no X on the primary control outputs
// A per-warp instruction budget catches runaway forward-jump mistakes.
//
// Seeding: +seed=<n> (default 1) and +iters=<n> (default 40) when the
// simulator accepts valued plusargs; multi-seed runs on Windows xsim.bat
// (which cannot pass valued plusargs through the .bat wrapper) instead write
// an optional tb_sp_sched_seed.svh next to the compile (run script does it)
// with `NG_SOAK_SEED / `NG_SOAK_ITERS defines.
// Prints NG_SP_SCHED_SOAK_TB_PASS on success.
`timescale 1ns/1ps
`ifdef NG_SOAK_SEED_FILE
  `include "tb_sp_sched_seed.svh"
`endif
module tb_sp_sched_soak;
  `include "tb_sp_sched_common.svh"

  int seed  = 1;
  int iters = 40;
`ifdef NG_SOAK_SEED
  initial begin
    seed  = `NG_SOAK_SEED;
    iters = `NG_SOAK_ITERS;
  end
`endif

  // xsim-compatible seeded RNG (no $urandom seed argument): xorshift32.
  function automatic int rnd();
    seed = seed ^ (seed << 13);
    seed = seed ^ (seed >> 17);
    seed = seed ^ (seed << 5);
    return (seed < 0) ? -seed : seed;
  endfunction
  function automatic int rnd_range(input int hi);  // 0..hi inclusive
    return rnd() % (hi + 1);
  endfunction

  // per-warp reference state
  logic [31:0] exp_pc [16];
  bit          exp_live [16];   // launched and not terminal
  bit          exp_term [16];   // reached endpgm or trap
  int          exp_budget [16];
  int          live_warps = 0;

  // async completion pool
  typedef struct packed {
    int cycles;
    int slot;
    int cnt;
  } dec_ev_t;
  dec_ev_t dec_q [$];
  int p_async = 0;  // percent of OTHER ops made async
  int bp_pct_l = 0;
  bit bp_en = 1'b0;

  // Front-end pipelining stress: random fetch latency / fetch-port
  // backpressure, and an execution-busy model of the fixed-latency pipes: after
  // each accepted issue the pipe is busy for 0..exec_max cycles (is_ready low,
  // ex_hold_r high -- what ng_sp_top derives from fixed_busy).
  int  exec_max = 0;
  int  fe_rdy_pct = 100;
  int  busy_cnt = 0;
  bit  ex_en = 1'b0;
  logic bp_ok;
  // sole driver of is_ready_r / ex_hold_r / if_req_ready_r
  always @(posedge clk) begin
    bp_ok = bp_en ? ((rnd() % 100) >= bp_pct_l) : 1'b1;
    if (busy_cnt > 0) busy_cnt = busy_cnt - 1;
    if (ex_en && is_valid_d && is_rdy_w && exec_max > 0) busy_cnt = rnd_range(exec_max);
    ex_hold_r      <= ex_en && (busy_cnt > 0);
    is_ready_r     <= bp_ok && !(ex_en && (busy_cnt > 0));
    if_req_ready_r <= !ex_en || ((rnd() % 100) < fe_rdy_pct);
  end

  // ---------------------------------------------------------------- X checker
  always @(posedge clk) if (rst_n) begin
    check(ev_commit_valid_d !== 1'bx, "ev_commit_valid_d X");
    check(is_valid_d !== 1'bx, "is_valid_d X");
    check(if_req_valid_d !== 1'bx, "if_req_valid_d X");
    check(br_v !== 1'bx, "br_valid X");
  end

  // ---------------------------------------------------------------- async inject
  always @(posedge clk) begin
    cnt_inc_valid_r <= 1'b0;
    cnt_dec_valid_r <= 1'b0;
    if (rst_n) begin
      if (is_valid_d && is_rdy_w && is_class_d == CLS_OTHER && p_async > 0 &&
          (rnd() % 100) < p_async) begin
        automatic int new_id = rnd_range(5);
        cnt_inc_valid_r <= 1'b1;
        cnt_inc_slot_r  <= is_slot_d;
        cnt_inc_id_r    <= 3'(new_id);
        dec_q.push_back('{int'(rnd_range(30)), int'(is_slot_d), new_id});
      end
      foreach (dec_q[i]) dec_q[i].cycles--;
      while (dec_q.size() > 0 && dec_q[0].cycles <= 0) begin
        cnt_dec_valid_r <= 1'b1;
        cnt_dec_slot_r  <= dec_q[0].slot;
        cnt_dec_id_r    <= dec_q[0].cnt;
        void'(dec_q.pop_front());
        break;  // one completion per cycle
      end
    end
  end

  // --------------------------------------------------------------
  function automatic logic [31:0] ref_next_pc(input logic [31:0] pc, input int cls,
                                              input logic [31:0] w0, input int slot);
    logic [31:0] uv;
    case (cls)
      CLS_JMP: return pc + {{8{w0[31]}}, w0[31:10], 2'b00};
      CLS_BZ, CLS_BNZ, CLS_BANY, CLS_BNONE: begin
        uv = ureg_view(slot, w0[12:7]);
        if (cls == CLS_BZ || cls == CLS_BNONE) begin
          if (uv == 0) return pc + {{13{w0[31]}}, w0[31:15], 2'b00};
        end else begin
          if (uv != 0) return pc + {{13{w0[31]}}, w0[31:15], 2'b00};
        end
        return pc + 32'd4;
      end
      CLS_BEXECZ, CLS_BEXECNZ: begin
        // EXEC is a full warp in this TB: bexecz never taken, bexecnz always
        if (cls == CLS_BEXECNZ) return pc + {{8{w0[31]}}, w0[31:10], 2'b00};
        return pc + 32'd4;
      end
      default: return pc + ((w0[1:0] == 2'b01) ? 32'd8 : 32'd4);
    endcase
  endfunction

  always @(posedge clk) begin
    if (rst_n && ev_commit_valid_d) begin
      int s;
      logic [31:0] w0;
      s  = int'(ev_slot_d);
      w0 = ((ev_pc_d >= imem_base) && ((ev_pc_d - imem_base) < 4 * 4096))
           ? imem[int'((ev_pc_d - imem_base) >> 2)] : 32'h0;
      check(exp_live[s] && !exp_term[s], $sformatf("soak: commit from dead warp %0d", s));
      check(ev_pc_d == exp_pc[s],
            $sformatf("soak: w%0d pc %08h != exp %08h", s, ev_pc_d, exp_pc[s]));
      exp_budget[s]--;
      check(exp_budget[s] >= 0, $sformatf("soak: warp %0d exceeded insn budget", s));
      if (ev_class_d == CLS_ENDPGM) begin
        exp_term[s] = 1'b1;
        exp_live[s] = 1'b0;
        live_warps--;
      end else begin
        exp_pc[s] = ref_next_pc(ev_pc_d, int'(ev_class_d), w0, s);
      end
    end
    if (rst_n && ev_trap_valid_d) begin
      int s;
      s = int'(ev_trap_slot_d);
      check(exp_live[s] && !exp_term[s], $sformatf("soak: trap from dead warp %0d", s));
      check(ev_trap_pc_d == exp_pc[s],
            $sformatf("soak: w%0d trap pc %08h != exp %08h", s, ev_trap_pc_d, exp_pc[s]));
      exp_term[s]  = 1'b1;
      exp_live[s]  = 1'b0;
      live_warps--;
    end
  end

  // ---------------------------------------------------------------- program gen
  // Forward jumps only (byte offsets 4..60, kept inside the program): warp
  // termination is structural.  Two modes so a trapping warp can never
  // strand siblings at a barrier (a trapped warp neither arrives nor exits,
  // so its workgroup would hang -- legitimate hardware behavior, but not a
  // soak pass condition):
  //   mode 0 "trappy":  no barrier words (traps + illegal encodings instead)
  //   mode 1 "barrier": no trap / illegal words
  task automatic gen_program(input int mode);
    int nw;
    for (int i = 0; i < 4096; i++) imem[i] = w_sys(0, 0);
    nw = 16 + (rnd() % 48);  // 16..63 words
    for (int i = 0; i < nw; i++) begin
      int r, off, f;
      r   = rnd() % 100;
      off = 4 * (1 + (rnd() % 15));
      f   = rnd_range(1);
      if (r < 32)      imem[i] = w_sys(0, 0);
      else if (r < 40) imem[i] = w_sys(4, rnd_range(100));
      else if (r < 48) imem[i] = w_sys(5, rnd_range(4));
      else if (r < 60) imem[i] = w_jmp((i + (off >> 2) < nw) ? off : 4);
      else if (r < 72) imem[i] = w_bop(f, 29, (i + (off >> 2) < nw) ? off : 4);
      else if (r < 80) imem[i] = w_waitcnt(rnd_range(63), rnd_range(31),
                                           rnd_range(31), rnd_range(15),
                                           rnd_range(3), rnd_range(7));
      else if (r < 88) imem[i] = 32'h0000_0102;
      else if (mode == 0 && r < 92) imem[i] = w_sys(2, rnd_range(40));
      else if (mode == 0 && r < 95) imem[i] = 32'h0000_0003;
      else if (mode == 1 && r < 96) imem[i] = w_sys(6, 0);
      else             imem[i] = w_sys(1, 0);
    end
    imem[nw - 1] = w_sys(1, 0);
    imem_words   = nw;
  endtask

  task automatic clear_state();
    n_commits = 0;
    n_dones   = 0;
    n_traps   = 0;
    commit_slot_q.delete();
    commit_pc_q.delete();
    commit_cls_q.delete();
    dec_q.delete();
    for (int s = 0; s < 16; s++) begin
      exp_live[s]   = 1'b0;
      exp_term[s]   = 1'b0;
      exp_budget[s] = 500;
      slot_busy[s]  = 1'b0;
    end
    live_warps = 0;
  endtask

  task automatic run_one_iteration();
    int nwg, nw, entry_w, mode;
    bp_en    = 1'b0;
    // half of the iterations run with the front-end stress knobs on
    ex_en      = (rnd_range(1) == 1);
    exec_max   = rnd_range(12);
    fe_rdy_pct = 30 + rnd_range(70);
    fetch_lat  = ex_en ? rnd_range(4) : 0;
    done_lat   = rnd_range(3);
    for (int k = 0; k < 16; k++) br_dly[k] = (rnd_range(2) == 0) ? 0 : rnd_range(8);  // late branch resolution
    done_jit   = ex_en ? rnd_range(exec_max) : rnd_range(6);
    p_async  = rnd_range(60);
    bp_pct_l = rnd_range(30);
    mode     = rnd_range(1);
    gen_program(mode);
    do_reset();
    clear_state();
    nwg = 1 + (rnd() % 3);
    for (int g = 0; g < nwg; g++) begin
      nw = 1 + (rnd() % 3);
      entry_w = rnd() % (int'(imem_words) - 7);
      tb_launch(g, nw, 32'(imem_base + 4 * entry_w));
      for (int k = 0; k < 16; k++)
        if (slot_busy[k] && !exp_live[k]) begin
          exp_live[k]   = 1'b1;
          exp_term[k]   = 1'b0;
          exp_pc[k]     = 32'(imem_base + 4 * entry_w);
          exp_budget[k] = 500;
          live_warps++;
        end
    end
    bp_en = 1'b1;
    // wait for every warp to reach a terminal state
    begin : waitdone
      int guard, stall;
      guard = 0;
      stall = 0;
      while (live_warps > 0 && guard < 20000) begin
        @(posedge clk);
        guard++;
        if (stalled_all_d) stall++;
        else stall = 0;
        check(stall < 400, "soak: deadlock (all warps blocked too long)");
      end
      check(live_warps == 0, $sformatf("soak: %0d warp(s) never terminated", live_warps));
    end
    repeat (10) @(posedge clk);
    bp_en = 1'b0;
    repeat (4) @(posedge clk);
  endtask

  initial begin : watchdog
    repeat (3000000) @(posedge clk);
    $display("FAIL: global timeout");
    $display("NG_SP_SCHED_SOAK_TB_FAIL");
    $finish;
  end

  int seed0 = 0;  // initial seed, for the pass message
  int n_parked = 0;   // branch commits that had to wait behind another commit event
  int n_lparked = 0;  // scheduler-local commits that had to wait behind a pipe-accept commit
  always @(posedge clk) if (rst_n && dut.rp_v && !$past(dut.rp_v)) n_parked++;
  always @(posedge clk) if (rst_n && dut.lp_v && !$past(dut.lp_v)) n_lparked++;

  initial begin
    int plus_seed;
    if ($value$plusargs("seed=%d", plus_seed)) seed = plus_seed;
    if (seed == 0) seed = 1;  // xorshift needs a nonzero state
    void'($value$plusargs("iters=%d", iters));
    seed0 = seed;
    $display("NG_SOAK_CFG seed=%0d iters=%0d", seed, iters);
    for (int it = 0; it < iters; it++) run_one_iteration();
    $display("NG_SOAK_STAT parked_branch_commits=%0d parked_local_commits=%0d", n_parked, n_lparked);
    if (errors == 0) $display("NG_SP_SCHED_SOAK_TB_PASS (%0d iterations, initial seed %0d)",
                              iters, seed0);
    else             $display("NG_SP_SCHED_SOAK_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
