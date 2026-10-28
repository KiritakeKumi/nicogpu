// ng_core_wd.sv -- core work distributor: fans one CP DISPATCH (ng_cp_pkg::wd_req_t, ng_cp_port wd_*) out to
// the NSP SPs of an ng_core and merges their completions.
//
// Placement (static interleave.  SP i runs the workgroups whose linear id
// (x fastest, the model's order) is first + i*step, first + (i + NSP)*step, ...  where first / step are the request's
// own wg_first / wg_step (0 / 0 from ng_cp_port = 0 / 1).  Each SP's ng_wd enumerates its share itself (it passes
// over the other SPs' workgroups in W_PICK, one per cycle), so a workgroup always lives on exactly one SP
// one half core, sharing one L1D / shared array / barrier, which is what ERRATA v0.1.4 "workgroup and half core"
// requires.  The functional model places dynamically (least loaded half, Core::place); the two only differ in the
// implementation-defined spid / warpid / wgslot / GlobalWarpSlot values and in timing.
//
// Completion: wd_done once every SP reported wd_done; wd_fault = OR of the SP faults; wd_instr = sum of the SPs'
// WDIST instruction counters (frozen per SP at its wd_done).
// Trap (any SP reports wd_done + wd_fault while others still run): the other SPs are aborted with sp_abort (folded
// into their core_reset, ABORT_CYC cycles) -- ERRATA v0.1.5 "traps are precise per warp" allows the other warps to
// stop anywhere; their instruction counts up to the abort are kept.  The model aborts every core at the first trap.
// core_reset: aborts the distributor (no wd_done for the aborted dispatch, like ng_wd); the SPs get it directly.
module ng_core_wd
  import ng_cp_pkg::*;
#(
  parameter int NSP       = 4,
  parameter int ABORT_CYC = 4      // sp_abort pulse length (ng_sp_top registers core_reset once)
) (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              core_reset,

  // ---- CP side (ng_cp_port / ng_cp_top wd_*) ----
  input  logic              wd_start,
  input  wd_req_t           wd_req,
  output logic              wd_done,
  output logic              wd_fault,
  output logic [63:0]       wd_instr,
  output logic              busy,

  // ---- SP side ----
  output logic [NSP-1:0]    sp_start,
  output wd_req_t           sp_req   [NSP],
  output logic [NSP-1:0]    sp_abort,
  input  logic [NSP-1:0]    sp_done,
  input  logic [NSP-1:0]    sp_fault,
  input  logic [63:0]       sp_instr [NSP],

  output logic [NSP-1:0]    dbg_done,     // SPs finished in the running dispatch
  output logic [NSP-1:0]    dbg_aborted   // SPs aborted after another SP's trap
);
  typedef enum logic [1:0] {C_IDLE, C_RUN, C_ABORT, C_DONE} cst_t;
  cst_t st;

  logic [NSP-1:0] done_q, abort_set;
  logic           fault_q;
  logic [63:0]    instr_q [NSP];
  logic [3:0]     ab_cnt;

  // per-SP request: the CP request with this SP's share of the workgroup stride
  logic [7:0] step0;
  assign step0 = (wd_req.wg_step == 8'd0) ? 8'd1 : wd_req.wg_step;
  always_comb begin
    for (int i = 0; i < NSP; i++) begin
      sp_req[i]          = wd_req;
      sp_req[i].wg_first = wd_req.wg_first + 8'(i * int'(step0));
      sp_req[i].wg_step  = 8'(int'(step0) * NSP);
    end
  end
  assign sp_start = (st == C_IDLE && wd_start) ? '1 : '0;
  // sp_abort is held for ABORT_CYC cycles, then released for 2 cycles before C_DONE
  logic ab_on;
  assign ab_on    = (st == C_ABORT) && (ab_cnt > 4'd2);
  assign sp_abort = ab_on ? abort_set : '0;
  assign busy     = (st != C_IDLE);
  assign dbg_done = done_q;
  assign dbg_aborted = abort_set;

  logic [NSP-1:0] done_nx;
  logic           any_fault_now;
  always_comb begin
    done_nx = done_q | sp_done;
    any_fault_now = |(sp_done & sp_fault);
  end

  always_ff @(posedge clk) begin
    if (!rst_n || core_reset) begin
      st <= C_IDLE;
      done_q <= '0; abort_set <= '0; fault_q <= 1'b0; ab_cnt <= '0;
      wd_done <= 1'b0; wd_fault <= 1'b0; wd_instr <= '0;
      for (int i = 0; i < NSP; i++) instr_q[i] <= '0;
    end else begin
      wd_done <= 1'b0;
      // live / frozen per-SP counts and their sum (the SP counters restart at wd_start)
      for (int i = 0; i < NSP; i++)
        if (st == C_RUN && !done_q[i]) instr_q[i] <= sp_instr[i];
      begin
        logic [63:0] s;
        s = '0;
        for (int i = 0; i < NSP; i++) s = s + instr_q[i];
        if (st != C_IDLE) wd_instr <= s;
      end
      case (st)
        C_IDLE: if (wd_start) begin
          done_q <= '0; abort_set <= '0; fault_q <= 1'b0;
          for (int i = 0; i < NSP; i++) instr_q[i] <= '0;
          wd_instr <= '0;
          st <= C_RUN;
        end
        C_RUN: begin
          done_q  <= done_nx;
          fault_q <= fault_q | any_fault_now;
          if (done_nx == '1) begin
            st <= C_DONE;
          end else if (any_fault_now || fault_q) begin
            abort_set <= ~done_nx;
            ab_cnt <= 4'(ABORT_CYC + 2);   // + 2: the SPs' registered reset is released before wd_done
            st <= C_ABORT;
          end
        end
        C_ABORT: begin
          if (ab_cnt != 4'd0) ab_cnt <= ab_cnt - 4'd1;
          else begin
            done_q <= '1;
            st <= C_DONE;
          end
        end
        C_DONE: begin       // instr_q final since the cycle before: wd_instr is the dispatch total now
          wd_done  <= 1'b1;
          wd_fault <= fault_q;
          st <= C_IDLE;
        end
        default: st <= C_IDLE;
      endcase
    end
  end
endmodule
