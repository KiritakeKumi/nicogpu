// ng_sp_sched.sv -- SP warp scheduler, first cut.
//
// One SP's fetch/issue front end: a warp slot table with the state machine of
// (Ready/Barrier/Done/Trapped plus waitcnt Wait),
// deterministic round-robin selection, a single outstanding instruction
// fetch, a scheduler predecoder (ng_sp_predec) and a decoupled issue port
// towards the execution pipes (VALU/LSU/SALU-branch in later units).
//
// Scheduling semantics follow run with CoreConfig
// quantum=1: one instruction per ready warp per cyclic round, warps visited
// in launch order (slot order).  With an in-order single-issue pipe the
// commit order is therefore identical to the model's commit trace for the
// same launches; the trace TBs compare the two textually.
//
//
//   * fetch port: simple one-outstanding valid/ready request + untagged
//     response (becomes an L0 I$ burst interface later);
//   * issue port: one op at a time, in-order; branch classes are resolved by
//     the pipe through br_*; jmp is folded into the scheduler;
//   * async-op counter classification comes from the pipe side (cnt_inc_*),
//     not from decode;
//   * a blocked waitcnt is re-fetched when the counters drain (idempotent);
//   * warp slots free per warp (the model frees a workgroup's slots only at
//     workgroup retire; visible only with mid-run relaunch).
module ng_sp_sched #(
  parameter int NG_SP_NUM_WARPS = 16,  // warp slots per SP
  parameter int NG_SP_NUM_WGS   = 8,   // simultaneously resident workgroup tags
  parameter int NG_SP_PC_W      = 32   // code offset width (ISA.md 1: 32-bit PC)
) (
  input  logic clk,
  input  logic rst_n,

  // Work-distributor launch: one workgroup atomically (nwarps warps, shared
  // entry PC).  Slots are the lowest free ones; the WG tag must be unused.
  input  logic                              launch_valid,
  output logic                              launch_ready,
  input  logic [$clog2(NG_SP_NUM_WGS)-1:0]  launch_wg,
  input  logic [4:0]                        launch_nwarps,
  input  logic [NG_SP_PC_W-1:0]             launch_pc,
  output logic [3:0]                        launch_slot0,

  // Instruction fetch (one outstanding).
  output logic                  if_req_valid,
  input  logic                  if_req_ready,
  output logic [NG_SP_PC_W-1:0] if_req_pc,
  input  logic                  if_rsp_valid,
  input  logic                  if_rsp_fault,
  input  logic [63:0]           if_rsp_data,   // {w1, w0}

  // Execution-side hold: while high, instructions the scheduler
  // resolves itself (nop/waitcnt/jmp/barrier/endpgm/traps) are not processed.
  // the top only raises it for a pending trap; the ordering after the
  // warp's own in-flight instruction comes from the scheduler's per-warp bit.
  // Pipe instructions are not held here (the top's issue interlock handles them).
  input  logic                  ex_hold,

  // Per-warp in-flight tracking: the scheduler sets inflight[slot]
  // when the pipe accepts an instruction of that warp; the top clears it
  // (one bit per warp, any number per cycle) when that instruction's unit
  // reports done / trap.  A warp with an instruction in flight is not
  // preferred by the selection, and never issues its next instruction before
  // the previous one has completed (same-warp RAW/WAW, ERRATA v0.1.3).
  input  logic [NG_SP_NUM_WARPS-1:0] inflight_clr,
  output logic [NG_SP_NUM_WARPS-1:0] inflight_o,

  // Issue towards the execution pipes.
  output logic                  is_valid,
  input  logic                  is_ready,
  output logic [3:0]            is_slot,
  output logic [NG_SP_PC_W-1:0] is_pc,
  output logic [63:0]           is_insn,
  output logic [3:0]            is_class,

  // Branch resolution from the pipe (bz/bnz/bany/bnone/bexecz/bexecnz).
  input  logic                  br_valid,
  input  logic [3:0]            br_slot,
  input  logic                  br_taken,
  input  logic [NG_SP_PC_W-1:0] br_target,

  // Async-op accounting driven by the pipe: +1 when an issued op enters a
  // waitcnt counter queue, -1 when it completes (in order per counter).
  input  logic       cnt_inc_valid,
  input  logic [3:0] cnt_inc_slot,
  input  logic [2:0] cnt_inc_id,
  input  logic       cnt_dec_valid,
  input  logic [3:0] cnt_dec_slot,
  input  logic [2:0] cnt_dec_id,

  // Commit event (one per retired instruction, in scheduler order).
  output logic                  ev_commit_valid,
  output logic [3:0]            ev_slot,
  output logic [NG_SP_PC_W-1:0] ev_pc,
  output logic [3:0]            ev_class,
  output logic [31:0]           ev_w0,         // first word of the committed instruction

  // Warp termination (endpgm) and trap report (v0.1 policy: halt and report).
  output logic                               ev_warp_done_valid,
  output logic [3:0]                         ev_warp_done_slot,
  output logic [$clog2(NG_SP_NUM_WGS)-1:0]   ev_warp_done_wg,
  output logic                               ev_trap_valid,
  output logic [3:0]                         ev_trap_slot,
  output logic [NG_SP_PC_W-1:0]              ev_trap_pc,
  output logic [3:0]                         ev_trap_cause,
  output logic [31:0]                        ev_trap_info,

  // Status: resident-but-blocked warps exist and none can issue.
  output logic stalled_all,

  // A commit event is parked or being emitted (the dispatch must not retire before it is out).
  output logic ev_busy,

  // perf counters: the issue buffer holds an instruction (presented or waiting for its warp's in-flight one).
  output logic ib_busy
);
  import ng_sp_sched_pkg::*;

  localparam int NW  = NG_SP_NUM_WARPS;
  localparam int SW  = $clog2(NW);
  localparam int WGW = $clog2(NG_SP_NUM_WGS);
  localparam int PCW = NG_SP_PC_W;

  // --------------------------------------------------------------
  warp_state_e            state [NW];
  logic [PCW-1:0]         wpc [NW];
  logic [WGW-1:0]         wgrp [NW];
  logic [5:0]             ocnt [NW][CNT_N];  // outstanding async ops per counter
  logic [5:0]             thr  [NW][CNT_N];  // waitcnt thresholds while blocked

  logic        wg_active  [NG_SP_NUM_WGS];
  logic [4:0]  wg_nwarps  [NG_SP_NUM_WGS];
  logic [4:0]  wg_arrived [NG_SP_NUM_WGS];
  logic [4:0]  wg_done    [NG_SP_NUM_WGS];

  // the issue request goes into a one-entry issue buffer (is_*); the FSM does
  // not wait for the pipe to accept it (S_ISSUE is gone), so the next warp is selected,
  // fetched and predecoded while the buffered instruction waits for / executes in the pipe.
  typedef enum logic [1:0] { S_SEL, S_FETCH, S_PD } fsm_e;
  fsm_e fsm;

  logic [SW-1:0]        sel_slot;
  logic [PCW-1:0]       sel_pc;
  logic [63:0]          rsp_data;
  logic                 rsp_fault;

  // ---------------------------------------------------------------- fetch engine
  // One outstanding fetch, decoupled from the issue FSM, plus a two-entry result
  // buffer keyed by PC.  Demand fetches (S_FETCH) and speculative prefetches of the
  // predicted next instructions share it.  Prefetching is free-running (no longer tied to
  // an FSM state): whenever the engine is idle, one of the two predicted candidates
  // (the next two warps in round-robin order, or the sequential successor of the
  // current warp) that is neither buffered nor in flight is requested.  A buffered
  // or in-flight word is only ever *used* when its PC equals the actual selection's
  // PC, so a wrong guess costs nothing but a refetch.  Entries are kept after use
  // (loops / re-fetched waitcnt hit again) and replaced oldest-first; they are all
  // dropped on launch and icache.inv.  Faulting prefetch responses are not buffered
  // (the fault VA of the last response is only valid for a directly consumed one);
  // the faulting PC is remembered (nf_*) so it is not prefetched again.
  logic                  e_busy;      // request issued, response not yet received
  logic [PCW-1:0]        e_pc;
  logic                  e_kill;      // drop the response (launch / icache.inv flush)
  logic [1:0]            b_v;
  logic [PCW-1:0]        b_pc   [2];
  logic [63:0]           b_data [2];
  logic                  b_fault [2];
  logic                  b_wp;        // replacement pointer (oldest entry)
  logic                  nf_v;        // negative entry: this PC faulted when prefetched
  logic [PCW-1:0]        nf_pc;
  logic [1:0]            pred_v_q;    // stage A: next two round-robin candidates
  logic [SW-1:0]         pred_slot_q [2];
  logic [PCW-1:0]        pred_pc_q [2];   // stage B: their (predicted) fetch PCs

  // ---------------------------------------------------------------- issue buffer + in-flight bits
  // inflight[s]: warp s has an instruction accepted by a pipe and not yet done/trapped
  // (set at accept, cleared by inflight_clr).  infl_ib_q is a registered copy of
  // inflight[is_slot] (it sees a clear arriving in the same cycle as the bit itself does,
  // so it is never later than the bit); it self-gates is_valid so that the single-warp
  // case keeps overlapping SEL/FETCH/PD of the next instruction with the execution of the
  // previous one  without ever issuing before completion.
  logic [NW-1:0] inflight, inflight_nxt, acc_mask;
  logic          infl_ib_q;
  logic          ib_v, ib_br;    // issue buffer holds an instruction (ib_br: a branch)
  logic          ib_wr;          // S_PD writes the issue buffer in this cycle
  logic          is_acc;         // the pipe accepts the presented instruction this cycle
  assign is_valid   = ib_v && !infl_ib_q;
  assign is_acc     = is_valid && is_ready;
  assign inflight_o = inflight;
  always_comb begin
    acc_mask = '0;
    if (is_acc) acc_mask[is_slot] = 1'b1;
    inflight_nxt = (inflight & ~inflight_clr) | acc_mask;
  end

  // branch bookkeeping: the branch in the SALU is the only one outstanding (the SALU is a
  // fixed-group unit), so one record is enough.
  logic [PCW-1:0]        brw_pc;
  logic [3:0]            brw_cls;
  logic [31:0]           brw_w0;

  // ---------------------------------------------------------------- commit events
  // Three sources: A = accept of a pipe instruction (never delayed: the SALU's clock CSR and
  // the top's EXEC latch for the trace count on its timing), L = scheduler-local instruction
  // (S_PD), R = branch resolution.  One event per cycle; priority A > parked L > parked R >
  // L > R, the losers are parked in lp_* / rp_* (every source is >= 3 cycles apart from
  // itself, so one parking register each is enough, and a warp's next event cannot be
  // created within 3 cycles of its previous one, so per-warp program order is preserved).
  logic                  lp_v, rp_v;
  logic [3:0]            lp_slot, rp_slot, lp_cls, rp_cls;
  logic [PCW-1:0]        lp_pc, rp_pc;
  logic [31:0]           lp_w0, rp_w0;
  logic                  ic_h1, ic_h2;     // icache.inv decided 1 / 2 cycles ago (its commit may have been parked)
  logic                  l_v, a_v, r_v;     // blocking temporaries (this cycle's requests)
  logic                  em_l, em_a, em_r;
  logic [3:0]            l_cls;

  // --------------------------------------------------------------
  // Prefer warps without an instruction in flight; if every READY warp has one
  // (e.g. a single resident warp) still select among them -- the front end of
  // the next instruction then overlaps the current execution and is_valid is
  // self-gated by infl_ib_q.
  logic [NW-1:0] ready_mask, ready_nif, ready_eff, grant;
  logic          grant_valid;
  always_comb begin
    for (int s = 0; s < NW; s++) ready_mask[s] = (state[s] == W_READY);
    ready_nif = ready_mask & ~inflight;
    ready_eff = (|ready_nif) ? ready_nif : ready_mask;
  end
  ng_sp_prim_rr_arb #(.N(NW)) u_arb (
    .clk   (clk),
    .rst_n (rst_n),
    .req   (ready_eff & {NW{fsm == S_SEL}}),
    .grant (grant),
    .valid (grant_valid)
  );

  // waitcnt unblock: a blocked warp whose counters all drained returns to
  // READY (checked continuously; thr[] is only meaningful in W_WAIT).
  function automatic logic wait_ok(input int s);
    for (int c = 0; c < CNT_N; c++)
      if (ocnt[s][c] > thr[s][c]) return 1'b0;
    return 1'b1;
  endfunction

  // --------------------------------------------------------------
  logic [3:0]  pd_cls;
  logic        pd_ill, pd_tvalid, pd_len8, pd_wbr;
  logic [3:0]  pd_tcause;
  logic [31:0] pd_tinfo;
  logic [23:0] pd_boff;
  logic [5:0]  pd_wfield [CNT_N];
  ng_sp_predec u_predec (
    .insn   (rsp_data),
    .cls    (pd_cls),
    .ill    (pd_ill),
    .tvalid (pd_tvalid),
    .tcause (pd_tcause),
    .tinfo  (pd_tinfo),
    .boff   (pd_boff),
    .len8   (pd_len8),
    .wbr    (pd_wbr),
    .wfield (pd_wfield)
  );

  // Effective waitcnt thresholds (ISA.md 5): all-ones field = do not wait;
  // sm accepts only 0 (wait for empty) or 3 (skip); sm 1..2 act as 0.
  logic [5:0] thr_eff [CNT_N];
  logic       wait_block;
  always_comb begin
    wait_block = 1'b0;
    for (int c = 0; c < CNT_N; c++) begin
      if (pd_wfield[c] == 6'((1 << CNT_FIELD_W[c]) - 1))
        thr_eff[c] = CNT_SAT[c];
      else if (c == CNT_SM && pd_wfield[c] != 0)
        thr_eff[c] = 6'd0;
      else
        thr_eff[c] = pd_wfield[c];
      if (ocnt[sel_slot][c] > thr_eff[c]) wait_block = 1'b1;
    end
  end

  // pd_wbr: waits for br_* after issue: bz/bnz/bnone/bany/bexecz/bexecnz
  // jal / jr / jalr, which are CLS_OTHER for the trace but SALU-resolved.

  // --------------------------------------------------------------
  // Parallel allocation (timing addendum): pre_cnt[s] = number of free
  // slots below s (prefix popcount, 5 bits); a free slot is assigned when
  // fewer than nwarps free slots lie below it ("lowest free slots").
  logic [4:0]     free_count;
  logic [4:0]     pre_cnt [NW];
  logic [NW-1:0]  free_mask, assign_mask;
  always_comb begin
    for (int s = 0; s < NW; s++) free_mask[s] = (state[s] == W_EMPTY);
    for (int s = 0; s < NW; s++) begin
      pre_cnt[s] = '0;
      for (int t = 0; t < s; t++) pre_cnt[s] = pre_cnt[s] + 5'(free_mask[t]);
    end
    free_count = pre_cnt[NW-1] + 5'(free_mask[NW-1]);
  end
  always_comb begin
    assign_mask = '0;
    if (launch_valid && launch_nwarps >= 5'd1)
      for (int s = 0; s < NW; s++)
        assign_mask[s] = free_mask[s] && (pre_cnt[s] < launch_nwarps);
  end
  assign launch_ready = (free_count >= launch_nwarps) && (launch_nwarps >= 5'd1) &&
                        !wg_active[launch_wg];
  always_comb begin
    launch_slot0 = '0;
    for (int s = NW - 1; s >= 0; s--)
      if (assign_mask[s]) launch_slot0 = 4'(s);  // lowest assigned slot
  end

  // --------------------------------------------------------------
  // Registered one-cycle pulses; payload registers hold until the next event.
  // Scheduler-local instructions wait while the warp itself still has an instruction in
  // flight (so they keep their order after it: e.g. endpgm after the last write) or a trap
  // is pending; other warps' instructions do not hold them.
  logic           hold;
  logic [WGW-1:0] sel_wg;
  logic [5:0]     arr_n, done_n;
  logic           do_barrier, do_endpgm, pd_release;
  assign hold       = ex_hold || inflight[sel_slot];
  assign sel_wg     = wgrp[sel_slot];
  assign do_barrier = (fsm == S_PD) && !hold && !rsp_fault && !pd_ill && !pd_tvalid && (pd_cls == CLS_BARRIER);
  assign do_endpgm  = (fsm == S_PD) && !hold && !rsp_fault && !pd_ill && !pd_tvalid && (pd_cls == CLS_ENDPGM);
  assign arr_n      = {1'b0, wg_arrived[sel_wg]} + {5'b0, do_barrier};
  assign done_n     = {1'b0, wg_done[sel_wg]} + {5'b0, do_endpgm};
  // icache.inv commits flush the fetch buffers (the code may have changed)
  logic do_icache, pd_pipe, flush_fe;
  assign do_icache  = (fsm == S_PD) && !hold && !rsp_fault && !pd_ill && !pd_tvalid && (pd_cls == CLS_ICACHE);
  // instructions that go to the execution pipe are not held
  assign pd_pipe    = !rsp_fault && !pd_ill && !pd_tvalid &&
                      (pd_cls == CLS_OTHER || pd_cls == CLS_BZ || pd_cls == CLS_BNZ ||
                       pd_cls == CLS_BNONE || pd_cls == CLS_BANY || pd_cls == CLS_BEXECZ ||
                       pd_cls == CLS_BEXECNZ);
  assign ib_wr      = (fsm == S_PD) && pd_pipe && !ib_v;
  assign flush_fe   = (launch_valid && launch_ready) || do_icache;

  // ---------------------------------------------------------------- next-fetch prediction
  // Stage A (registered): the next two warps the round-robin arbiter will pick after the
  // current one (cyclic order sel+1 .. sel+NW-1, then sel itself).  A warp whose
  // instruction waits in the issue buffer counts (it becomes READY at accept; its wpc
  // already points at the successor).
  // Stage B (registered): their PCs; for the warp being predecoded that is the sequential
  // next PC.
  logic [NW-1:0] cand_mask, cand_nif, cand_eff;
  logic [SW-1:0] pred_c [2];
  logic [1:0]    pred_cv;
  always_comb begin
    cand_mask = ready_mask;
    if (ib_v && !ib_br) cand_mask[is_slot] = 1'b1;
    cand_nif  = cand_mask & ~inflight;
    cand_eff  = (|cand_nif) ? cand_nif : cand_mask;
    pred_c[0] = sel_slot; pred_c[1] = sel_slot; pred_cv = 2'b00;
    for (int k = NW; k >= 1; k--)
      if (cand_eff[(int'(sel_slot) + k) % NW]) begin
        pred_c[1] = pred_c[0]; pred_cv[1] = pred_cv[0];
        pred_c[0] = SW'((int'(sel_slot) + k) % NW); pred_cv[0] = 1'b1;
      end
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pred_v_q <= 2'b00;
      for (int j = 0; j < 2; j++) begin pred_slot_q[j] <= '0; pred_pc_q[j] <= '0; end
    end else begin
      pred_v_q <= pred_cv;
      for (int j = 0; j < 2; j++) begin
        pred_slot_q[j] <= pred_c[j];
        pred_pc_q[j]   <= (pred_slot_q[j] == sel_slot && fsm == S_PD)
                          ? sel_pc + (pd_len8 ? PCW'(8) : PCW'(4))
                          : wpc[pred_slot_q[j]];
      end
    end
  end

  // engine start / consume decisions (combinational, used by the FSM block)
  logic hit_b0, hit_b1, hit_buf, hit_rsp, demand_go, pf_go, eng_go, pf_want;
  logic have_p [2];
  logic [PCW-1:0] pf_pc;
  assign hit_b0    = (fsm == S_FETCH) && b_v[0] && (b_pc[0] == sel_pc);
  assign hit_b1    = (fsm == S_FETCH) && b_v[1] && (b_pc[1] == sel_pc);
  assign hit_buf   = hit_b0 || hit_b1;
  assign hit_rsp   = (fsm == S_FETCH) && !hit_buf && e_busy && !e_kill && if_rsp_valid && (e_pc == sel_pc);
  assign demand_go = (fsm == S_FETCH) && !hit_buf && !e_busy;
  always_comb begin
    for (int j = 0; j < 2; j++)
      have_p[j] = (b_v[0] && b_pc[0] == pred_pc_q[j]) || (b_v[1] && b_pc[1] == pred_pc_q[j]) ||
                  (nf_v && nf_pc == pred_pc_q[j]);
  end
  assign pf_want = (pred_v_q[0] && !have_p[0]) || (pred_v_q[1] && !have_p[1]);
  assign pf_pc   = (pred_v_q[0] && !have_p[0]) ? pred_pc_q[0] : pred_pc_q[1];
  // (no prefetch request while an icache.inv commit may still be on its way to the I$)
  assign pf_go   = pf_want && !e_busy && !demand_go && !flush_fe && !ic_h1 && !ic_h2;
  assign eng_go  = demand_go || pf_go;

  assign pd_release = (do_barrier || do_endpgm) && wg_active[sel_wg] && (arr_n != 0) &&
                      (arr_n == {1'b0, wg_nwarps[sel_wg]} - done_n);

  // ---------------------------------------------------------------- FSM + state updates
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int s = 0; s < NW; s++) begin
        state[s] <= W_EMPTY;
        wpc[s]   <= '0;
        wgrp[s]  <= '0;
        for (int c = 0; c < CNT_N; c++) begin
          ocnt[s][c] <= '0;
          thr[s][c]  <= '0;
        end
      end
      for (int w = 0; w < NG_SP_NUM_WGS; w++) begin
        wg_active[w]  <= 1'b0;
        wg_nwarps[w]  <= '0;
        wg_arrived[w] <= '0;
        wg_done[w]    <= '0;
      end
      fsm          <= S_SEL;
      sel_slot     <= '0;
      sel_pc       <= '0;
      rsp_data     <= '0;
      rsp_fault    <= 1'b0;
      e_busy       <= 1'b0;
      e_pc         <= '0;
      e_kill       <= 1'b0;
      b_v          <= 2'b00;
      b_wp         <= 1'b0;
      for (int j = 0; j < 2; j++) begin b_pc[j] <= '0; b_data[j] <= '0; b_fault[j] <= 1'b0; end
      nf_v         <= 1'b0;
      nf_pc        <= '0;
      ev_w0        <= '0;
      if_req_valid <= 1'b0;
      if_req_pc    <= '0;
      ib_v         <= 1'b0;
      ib_br        <= 1'b0;
      infl_ib_q    <= 1'b0;
      inflight     <= '0;
      is_slot      <= '0;
      is_pc        <= '0;
      is_insn      <= '0;
      is_class     <= CLS_NONE;
      ev_commit_valid    <= 1'b0;
      ev_slot            <= '0;
      ev_pc              <= '0;
      ev_class           <= CLS_NONE;
      ev_warp_done_valid <= 1'b0;
      ev_warp_done_slot  <= '0;
      ev_warp_done_wg    <= '0;
      ev_trap_valid      <= 1'b0;
      ev_trap_slot       <= '0;
      ev_trap_pc         <= '0;
      ev_trap_cause      <= CAUSE_ILLEGAL;
      ev_trap_info       <= '0;
      brw_pc <= '0; brw_cls <= CLS_NONE; brw_w0 <= '0;
      lp_v <= 1'b0; lp_slot <= '0; lp_pc <= '0; lp_cls <= CLS_NONE; lp_w0 <= '0;
      ic_h1 <= 1'b0; ic_h2 <= 1'b0;
      rp_v <= 1'b0; rp_slot <= '0; rp_pc <= '0; rp_cls <= CLS_NONE; rp_w0 <= '0;
    end else begin
      l_v = 1'b0; a_v = 1'b0; r_v = 1'b0; l_cls = CLS_NONE;
      // default: single-cycle pulses deassert
      ev_commit_valid    <= 1'b0;
      ev_warp_done_valid <= 1'b0;
      ev_trap_valid      <= 1'b0;
      ev_w0              <= rsp_data[31:0];  // pairs with a scheduler-local commit decided this cycle

      ic_h1 <= do_icache;
      ic_h2 <= ic_h1;
      inflight  <= inflight_nxt;
      infl_ib_q <= inflight_nxt[ib_wr ? sel_slot : is_slot];

      // ---- fetch engine ---------------------------------------------------------
      if (if_req_valid && if_req_ready) if_req_valid <= 1'b0;   // request accepted
      if (if_rsp_valid && e_busy) begin
        e_busy <= 1'b0;
        e_kill <= 1'b0;
        if (!e_kill) begin   // (a response consumed straight by S_FETCH is kept too: sibling warps run the same PCs)
          if (if_rsp_fault) begin
            nf_v  <= 1'b1;
            nf_pc <= e_pc;
          end else begin
            // free entry first, else the oldest
            if (!b_v[0] || (b_v[1] && !b_wp)) begin
              b_v[0] <= 1'b1; b_pc[0] <= e_pc; b_data[0] <= if_rsp_data; b_fault[0] <= 1'b0;
              b_wp   <= 1'b1;
            end else begin
              b_v[1] <= 1'b1; b_pc[1] <= e_pc; b_data[1] <= if_rsp_data; b_fault[1] <= 1'b0;
              b_wp   <= 1'b0;
            end
          end
        end
      end
      if (eng_go) begin
        e_busy       <= 1'b1;
        e_kill       <= 1'b0;
        e_pc         <= demand_go ? sel_pc : pf_pc;
        if_req_pc    <= demand_go ? sel_pc : pf_pc;
        if_req_valid <= 1'b1;
      end
      if (flush_fe) begin
        b_v  <= 2'b00;
        nf_v <= 1'b0;
        if (e_busy && !if_rsp_valid) e_kill <= 1'b1;
      end

      // ---- launch (independent of the pipe) --------------------------------
      if (launch_valid && launch_ready) begin
        for (int s = 0; s < NW; s++)
          if (assign_mask[s]) begin
            state[s] <= W_READY;
            wpc[s]   <= launch_pc;
            wgrp[s]  <= launch_wg;
            for (int c = 0; c < CNT_N; c++) begin
              ocnt[s][c] <= '0;  // fresh warp: no outstanding async ops
              thr[s][c]  <= '0;
            end
          end
        wg_active[launch_wg]  <= 1'b1;
        wg_nwarps[launch_wg]  <= launch_nwarps;
        wg_arrived[launch_wg] <= '0;
        wg_done[launch_wg]    <= '0;
      end

      // ---- waitcnt unblock --------------------------------------------------
      for (int s = 0; s < NW; s++)
        if (state[s] == W_WAIT && wait_ok(s)) state[s] <= W_READY;

      // ---- async counters ----------------------------------------------------
      // Same-cycle inc+dec on one counter nets out (a dec against an empty
      // counter pairs with the simultaneous inc instead of being dropped).
      if (cnt_inc_valid && cnt_dec_valid && cnt_inc_slot == cnt_dec_slot && cnt_inc_id == cnt_dec_id) begin
        // saturated: both are dropped; otherwise +1 and -1 cancel
        if (ocnt[cnt_inc_slot][cnt_inc_id] < CNT_SAT[cnt_inc_id])
          ocnt[cnt_inc_slot][cnt_inc_id] <= ocnt[cnt_inc_slot][cnt_inc_id];
      end else begin
        if (cnt_inc_valid && ocnt[cnt_inc_slot][cnt_inc_id] < CNT_SAT[cnt_inc_id])
          ocnt[cnt_inc_slot][cnt_inc_id] <= ocnt[cnt_inc_slot][cnt_inc_id] + 6'd1;
        if (cnt_dec_valid && ocnt[cnt_dec_slot][cnt_dec_id] != 6'd0)
          ocnt[cnt_dec_slot][cnt_dec_id] <= ocnt[cnt_dec_slot][cnt_dec_id] - 6'd1;
      end

      // ---- barrier release / workgroup retire --------------------------------
      if (pd_release) begin
        wg_arrived[sel_wg] <= '0;
        for (int s = 0; s < NW; s++)
          if (state[s] == W_BAR && wgrp[s] == sel_wg) state[s] <= W_READY;
      end else if (do_barrier || do_endpgm) begin
        wg_arrived[sel_wg] <= arr_n[4:0];
      end
      if (do_endpgm) begin
        wg_done[sel_wg] <= done_n[4:0];
        if (done_n == {1'b0, wg_nwarps[sel_wg]}) wg_active[sel_wg] <= 1'b0;
      end

      // ---- issue buffer: accept ------------------------------------------------
      if (is_acc) begin
        ib_v <= 1'b0;
        if (ib_br) begin
          // only this warp waits for br_* (the front end moves on)
          state[is_slot] <= W_BRWAIT;
          brw_pc         <= is_pc;
          brw_cls        <= is_class;
          brw_w0         <= is_insn[31:0];
        end else begin
          state[is_slot] <= W_READY;     // wpc already points at the successor
          a_v = 1'b1;
        end
      end

      // ---- pipeline ------------------------------------------------------------
      unique case (fsm)
        S_SEL: begin
          if (grant_valid) begin
            for (int s = 0; s < NW; s++)
              if (grant[s]) begin
                sel_slot <= SW'(s);
                sel_pc   <= wpc[s];
              end
            fsm <= S_FETCH;
          end
        end
        S_FETCH: begin
          if (hit_b0) begin
            rsp_data  <= b_data[0];
            rsp_fault <= b_fault[0];
            fsm       <= S_PD;
          end else if (hit_b1) begin
            rsp_data  <= b_data[1];
            rsp_fault <= b_fault[1];
            fsm       <= S_PD;
          end else if (hit_rsp) begin
            rsp_data  <= if_rsp_data;
            rsp_fault <= if_rsp_fault;
            fsm       <= S_PD;
          end
        end
        S_PD: begin
          if (hold && !pd_pipe) begin
            // wait for this warp's in-flight instruction to finish / no trap pending
          end else if (rsp_fault) begin
            state[sel_slot] <= W_TRAP;
            ev_trap_valid   <= 1'b1;
            ev_trap_slot    <= sel_slot;
            ev_trap_pc      <= sel_pc;
            ev_trap_cause   <= CAUSE_MEMFAULT;
            ev_trap_info    <= sel_pc;
            fsm             <= S_SEL;
          end else if (pd_ill) begin
            state[sel_slot] <= W_TRAP;
            ev_trap_valid   <= 1'b1;
            ev_trap_slot    <= sel_slot;
            ev_trap_pc      <= sel_pc;
            ev_trap_cause   <= CAUSE_ILLEGAL;
            ev_trap_info    <= rsp_data[31:0];
            fsm             <= S_SEL;
          end else if (pd_tvalid) begin
            state[sel_slot] <= W_TRAP;
            ev_trap_valid   <= 1'b1;
            ev_trap_slot    <= sel_slot;
            ev_trap_pc      <= sel_pc;
            ev_trap_cause   <= pd_tcause;
            ev_trap_info    <= pd_tinfo;
            fsm             <= S_SEL;
          end else begin
            unique case (pd_cls)
              CLS_WAITCNT: begin
                if (wait_block) begin
                  state[sel_slot] <= W_WAIT;
                  for (int c = 0; c < CNT_N; c++) thr[sel_slot][c] <= thr_eff[c];
                  fsm <= S_SEL;  // re-fetch the waitcnt once the counters drain
                end else begin
                  l_v = 1'b1; l_cls = CLS_WAITCNT;
                  wpc[sel_slot]   <= sel_pc + PCW'(4);
                  fsm             <= S_SEL;
                end
              end
              CLS_JMP: begin
                l_v = 1'b1; l_cls = CLS_JMP;
                wpc[sel_slot]   <= sel_pc + PCW'($signed(pd_boff));
                fsm             <= S_SEL;
              end
              CLS_BARRIER: begin
                l_v = 1'b1; l_cls = CLS_BARRIER;
                wpc[sel_slot]   <= sel_pc + PCW'(4);
                // if this arrival completes the barrier, the same-cycle
                // release must win over parking in W_BAR
                state[sel_slot] <= pd_release ? W_READY : W_BAR;
                fsm             <= S_SEL;
              end
              CLS_ENDPGM: begin
                l_v = 1'b1; l_cls = CLS_ENDPGM;
                // the slot frees immediately: "Done" is the ev_warp_done
                // event, not a parked state (context harvesting arrives
                // with the register-file scoreboard).  Like the
                // model's retire_all, remaining outstanding ops are
                // dropped with the warp (their dec pulses arrive later and
                // are ignored on an empty counter).
                state[sel_slot]    <= W_EMPTY;
                for (int c = 0; c < CNT_N; c++) ocnt[sel_slot][c] <= '0;
                ev_warp_done_valid <= 1'b1;
                ev_warp_done_slot  <= sel_slot;
                ev_warp_done_wg    <= sel_wg;
                fsm                <= S_SEL;
              end
              CLS_BZ, CLS_BNZ, CLS_BNONE, CLS_BANY, CLS_BEXECZ, CLS_BEXECNZ,
              CLS_OTHER: begin
                // into the issue buffer (waits here while it is occupied)
                if (!ib_v) begin
                  ib_v     <= 1'b1;
                  ib_br    <= pd_wbr;
                  is_slot  <= sel_slot;
                  is_pc    <= sel_pc;
                  is_insn  <= rsp_data;
                  is_class <= pd_cls;
                  state[sel_slot] <= W_ISSUE_PEND;
                  // a non-branch successor is known now (branches get it from br_*)
                  if (!pd_wbr) wpc[sel_slot] <= sel_pc + (pd_len8 ? PCW'(8) : PCW'(4));
                  fsm      <= S_SEL;
                end
              end
              default: begin  // nop / sleep / setprio / icache.inv
                l_v = 1'b1; l_cls = pd_cls;
                wpc[sel_slot]   <= sel_pc + PCW'(4);
                fsm             <= S_SEL;
              end
            endcase
          end
        end
        default: fsm <= S_SEL;
      endcase

      // ---- branch resolution -------------------------------------------------------
      // br_valid pulses with the SALU's done; the warp parked in W_BRWAIT gets its new PC
      // and returns to READY.
      if (br_valid && state[br_slot] == W_BRWAIT) begin
        wpc[br_slot]   <= br_taken ? br_target : (brw_pc + PCW'(4));
        state[br_slot] <= W_READY;
        r_v = 1'b1;
      end

      // ---- commit event arbitration ------------------------------------------------
      em_l = 1'b0; em_a = 1'b0; em_r = 1'b0;
      if (a_v) begin
        ev_commit_valid <= 1'b1; ev_slot <= is_slot; ev_pc <= is_pc; ev_class <= CLS_OTHER;
        ev_w0 <= is_insn[31:0];
        em_a = 1'b1;
      end else if (lp_v) begin
        ev_commit_valid <= 1'b1; ev_slot <= lp_slot; ev_pc <= lp_pc; ev_class <= lp_cls; ev_w0 <= lp_w0;
        lp_v <= 1'b0;
      end else if (rp_v) begin
        ev_commit_valid <= 1'b1; ev_slot <= rp_slot; ev_pc <= rp_pc; ev_class <= rp_cls; ev_w0 <= rp_w0;
        rp_v <= 1'b0;
      end else if (l_v) begin
        ev_commit_valid <= 1'b1; ev_slot <= sel_slot; ev_pc <= sel_pc; ev_class <= l_cls;
        em_l = 1'b1;
      end else if (r_v) begin
        ev_commit_valid <= 1'b1; ev_slot <= br_slot; ev_pc <= brw_pc; ev_class <= brw_cls; ev_w0 <= brw_w0;
        em_r = 1'b1;
      end
      if (l_v && !em_l) begin
        lp_v <= 1'b1; lp_slot <= sel_slot; lp_pc <= sel_pc; lp_cls <= l_cls; lp_w0 <= rsp_data[31:0];
      end
      if (r_v && !em_r) begin
        rp_v <= 1'b1; rp_slot <= br_slot; rp_pc <= brw_pc; rp_cls <= brw_cls; rp_w0 <= brw_w0;
      end
    end
  end

  // --------------------------------------------------------------
  // PEND / BRWAIT warps are progressing (their instruction is in the issue buffer / the SALU)
  logic any_blocked, any_ready;
  always_comb begin
    any_blocked = 1'b0;
    any_ready   = 1'b0;
    for (int s = 0; s < NW; s++) begin
      if (state[s] == W_WAIT || state[s] == W_BAR) any_blocked = 1'b1;
      if (state[s] == W_READY || state[s] == W_ISSUE_PEND || state[s] == W_BRWAIT) any_ready = 1'b1;
    end
  end
  assign stalled_all = any_blocked && !any_ready;
  assign ev_busy     = lp_v || rp_v || ev_commit_valid;
  assign ib_busy     = ib_v;
endmodule
