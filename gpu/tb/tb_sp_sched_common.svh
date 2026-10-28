// tb_sp_sched_common.svh -- shared harness for the SP-scheduler TBs.
//
// Included (not compiled standalone) by tb_sp_sched_*.sv.  Provides the
// clock/reset, one DUT instance, a behavioral instruction memory on the fetch
// port (1-cycle response, out-of-range or misaligned fetch faults), a fake
// execution pipe on the issue port (is_ready programmable, branch responder
// evaluating uniform branches with u26 = workgroup id x, u29 = warp index in
// workgroup, all other uniforms 0, EXEC = full warp) and commit/done/trap
// event capture queues.
`ifndef TB_SP_SCHED_COMMON_SVH
`define TB_SP_SCHED_COMMON_SVH

import ng_sp_sched_pkg::*;

// ---------------------------------------------------------------- stim regs
logic         clk = 1'b0;
logic         rst_n = 1'b0;
always #5 clk = ~clk;

logic        launch_valid = 1'b0;
logic [2:0]  launch_wg = '0;
logic [4:0]  launch_nwarps = '0;
logic [31:0] launch_pc = '0;

logic        if_req_ready_r = 1'b1;
logic        is_ready_r = 1'b1;
logic        ex_hold_r = 1'b0;  // execution-side hold (0 unless a TB drives it)
int          done_lat = 0;     // accept -> inflight clear delay (cycles, 0 = next cycle)
int          done_jit = 0;     // plus a random 0..done_jit
bit          br_rand = 1'b0;   // 1: random branch resolution (soak)
int          bp_pct = 0;       // issue backpressure percent (soak)
int          br_dly [16];      // per-slot extra branch-resolution delay (cycles after accept)
bit          br_out = 1'b0;    // a branch is outstanding in the (fixed-latency) branch unit: it takes one at a time

// ---------------------------------------------------------------- TB slot view
// Mirrors the DUT's lowest-free-slot allocation to know each slot's
// workgroup id and warp index (needed to evaluate uniform branches).
int  slot_wg [16];
int  tb_wg_xid [16];
int  tb_slot_iw [16];
bit  slot_busy [16];

task automatic tb_free_slot(input int s);
  slot_busy[s] = 1'b0;
endtask

// --------------------------------------------------------------
logic        launch_ready_d;
logic [3:0]  launch_slot0_d;
logic        if_req_valid_d;
logic [31:0] if_req_pc_d;
logic        if_rsp_v, if_rsp_f;
logic [63:0] if_rsp_d;
logic        is_valid_d;
logic [3:0]  is_slot_d;
logic [31:0] is_pc_d;
logic [63:0] is_insn_d;
logic [3:0]  is_class_d;
logic        br_v = 1'b0;
logic [3:0]  br_slot_w;
logic        br_taken_w;
logic [31:0] br_target_w;
logic        cnt_inc_valid_r = 1'b0;
logic [3:0]  cnt_inc_slot_r = '0;
logic [2:0]  cnt_inc_id_r = '0;
logic        cnt_dec_valid_r = 1'b0;
logic [3:0]  cnt_dec_slot_r = '0;
logic [2:0]  cnt_dec_id_r = '0;
logic        ev_commit_valid_d;
logic [3:0]  ev_slot_d;
logic [31:0] ev_pc_d;
logic [3:0]  ev_class_d;
logic        ev_warp_done_valid_d;
logic [3:0]  ev_warp_done_slot_d;
logic [2:0]  ev_warp_done_wg_d;
logic        ev_trap_valid_d;
logic [3:0]  ev_trap_slot_d;
logic [31:0] ev_trap_pc_d;
logic [3:0]  ev_trap_cause_d;
logic [31:0] ev_trap_info_d;
logic        stalled_all_d;
logic        ev_busy_d;
logic [15:0] infl_clr_r = '0;
logic [15:0] infl_d;
// the branch unit is a single fixed-latency unit: a second branch is not accepted while one is outstanding
function automatic logic is_branch_cls_f(input logic [3:0] c);
  return (c >= CLS_BZ && c <= CLS_BEXECNZ);
endfunction
wire is_rdy_w = is_ready_r && !(br_out && is_branch_cls_f(is_class_d));

ng_sp_sched #(
  .NG_SP_NUM_WARPS (16),
  .NG_SP_NUM_WGS   (8),
  .NG_SP_PC_W      (32)
) dut (
  .clk                (clk),
  .rst_n              (rst_n),
  .launch_valid       (launch_valid),
  .launch_ready       (launch_ready_d),
  .launch_wg          (launch_wg),
  .launch_nwarps      (launch_nwarps),
  .launch_pc          (launch_pc),
  .launch_slot0       (launch_slot0_d),
  .if_req_valid       (if_req_valid_d),
  .if_req_ready       (if_req_ready_r),
  .if_req_pc          (if_req_pc_d),
  .if_rsp_valid       (if_rsp_v),
  .if_rsp_fault       (if_rsp_f),
  .if_rsp_data        (if_rsp_d),
`ifndef NG_OLD_SCHED
  .ex_hold            (ex_hold_r),
`endif
  .inflight_clr       (infl_clr_r),
  .inflight_o         (infl_d),
  .is_valid           (is_valid_d),
  .is_ready           (is_rdy_w),
  .is_slot            (is_slot_d),
  .is_pc              (is_pc_d),
  .is_insn            (is_insn_d),
  .is_class           (is_class_d),
  .br_valid           (br_v),
  .br_slot            (br_slot_w),
  .br_taken           (br_taken_w),
  .br_target          (br_target_w),
  .cnt_inc_valid      (cnt_inc_valid_r),
  .cnt_inc_slot       (cnt_inc_slot_r),
  .cnt_inc_id         (cnt_inc_id_r),
  .cnt_dec_valid      (cnt_dec_valid_r),
  .cnt_dec_slot       (cnt_dec_slot_r),
  .cnt_dec_id         (cnt_dec_id_r),
  .ev_commit_valid    (ev_commit_valid_d),
  .ev_slot            (ev_slot_d),
  .ev_pc              (ev_pc_d),
  .ev_class           (ev_class_d),
  .ev_warp_done_valid (ev_warp_done_valid_d),
  .ev_warp_done_slot  (ev_warp_done_slot_d),
  .ev_warp_done_wg    (ev_warp_done_wg_d),
  .ev_trap_valid      (ev_trap_valid_d),
  .ev_trap_slot       (ev_trap_slot_d),
  .ev_trap_pc         (ev_trap_pc_d),
  .ev_trap_cause      (ev_trap_cause_d),
  .ev_trap_info       (ev_trap_info_d),
  .stalled_all        (stalled_all_d),
  .ev_busy            (ev_busy_d)
);

// ---------------------------------------------------------------- fetch memory
logic [31:0] imem [0:4095];
int unsigned imem_base = 32'h0000_1000;
int unsigned imem_words = 0;

task automatic load_imem(input string path, input int unsigned base, input int unsigned words);
  imem_base = base;
  imem_words = words;
  foreach (imem[i]) imem[i] = '0;
  $readmemh(path, imem);
endtask

int fr_idx;
int fetch_lat = 0;               // extra response latency in cycles (0 = next cycle)
int dly_cnt = 0;
bit dly_pend = 1'b0;
logic [63:0] dly_d;
logic        dly_f;
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    if_rsp_v <= 1'b0;
    if_rsp_f <= 1'b0;
    if_rsp_d <= '0;
    dly_pend <= 1'b0;
  end else begin
    if_rsp_v <= 1'b0;
    if (if_req_valid_d && if_req_ready_r) begin
      fr_idx = int'((if_req_pc_d - imem_base) >> 2);
      dly_f  = (if_req_pc_d < imem_base) || (fr_idx >= int'(imem_words)) ||
               (if_req_pc_d[1:0] != 2'b00);
      dly_d  = {((fr_idx + 1 < int'(imem_words)) ? imem[fr_idx + 1] : 32'h0),
                ((fr_idx >= 0 && fr_idx < 4096) ? imem[fr_idx] : 32'h0)};
      if (fetch_lat <= 0) begin
        if_rsp_f <= dly_f;
        if_rsp_d <= dly_d;
        if_rsp_v <= 1'b1;
      end else begin
        dly_pend <= 1'b1;
        dly_cnt  <= fetch_lat;
      end
    end else if (dly_pend) begin
      if (dly_cnt <= 1) begin
        if_rsp_f <= dly_f;
        if_rsp_d <= dly_d;
        if_rsp_v <= 1'b1;
        dly_pend <= 1'b0;
      end else dly_cnt <= dly_cnt - 1;
    end
  end
end

// ---------------------------------------------------------------- fake pipe
function automatic logic is_branch_cls(input logic [3:0] c);
  return (c >= CLS_BZ && c <= CLS_BEXECNZ);
endfunction

// Uniform-register view used to evaluate uniform branches: u26 = workgroup
// id x, u29 = warp index in workgroup (ABI.md 2), everything else 0.
function automatic logic [31:0] ureg_view(input int slot, input logic [6:0] r);
  if (r == 7'd26) return 32'(tb_wg_xid[slot]);
  if (r == 7'd29) return 32'(tb_slot_iw[slot]);
  return 32'h0;
endfunction

function automatic logic eval_branch(input logic [3:0] cls, input logic [63:0] insn,
                                     input int slot);
  logic [31:0] uv;
  uv = ureg_view(slot, insn[12:7]);
  case (cls)
    CLS_BZ:      return (uv == 32'h0);
    CLS_BNZ:     return (uv != 32'h0);
    CLS_BANY:    return (uv != 32'h0);   // (u & EXEC) != 0, EXEC = full warp
    CLS_BNONE:   return (uv == 32'h0);   // (u & EXEC) == 0
    CLS_BEXECZ:  return 1'b0;            // EXEC never zero here
    CLS_BEXECNZ: return 1'b1;
    default:     return 1'b0;
  endcase
endfunction

function automatic logic [31:0] branch_off(input logic [3:0] cls, input logic [63:0] insn);
  if (cls == CLS_BEXECZ || cls == CLS_BEXECNZ)
    return {{8{insn[31]}}, insn[31:10], 2'b00};  // br22
  return {{13{insn[31]}}, insn[31:15], 2'b00};   // br17
endfunction

// Unit-completion model for the per-warp in-flight bits: every accepted issue
// clears its warp's bit done_lat (+ random 0..done_jit) cycles later.
typedef struct { int slot; int cyc; } infl_ev_t;
infl_ev_t infl_q_tb [$];
int lat_n;
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    infl_clr_r <= '0;
    infl_q_tb.delete();
  end else begin
    infl_clr_r <= '0;
    foreach (infl_q_tb[i]) infl_q_tb[i].cyc--;
    for (int i = infl_q_tb.size() - 1; i >= 0; i--)
      if (infl_q_tb[i].cyc <= 0) begin
        infl_clr_r[infl_q_tb[i].slot] <= 1'b1;
        infl_q_tb.delete(i);
      end
    if (is_valid_d && is_rdy_w) begin
      lat_n = done_lat + ((done_jit > 0) ? $urandom_range(done_jit) : 0);
      if (lat_n <= 0) infl_clr_r[is_slot_d] <= 1'b1;
      else            infl_q_tb.push_back('{int'(is_slot_d), lat_n});
    end
  end
end

int          br_cnt = 0;
initial foreach (br_dly[i]) br_dly[i] = 0;
always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    br_v <= 1'b0;
    br_out <= 1'b0;
    br_cnt <= 0;
  end else begin
    br_v <= 1'b0;
    if (br_out) begin
      if (br_cnt <= 0) begin
        br_v   <= 1'b1;
        br_out <= 1'b0;
      end else br_cnt <= br_cnt - 1;
    end
    if (is_valid_d && is_rdy_w && is_branch_cls(is_class_d)) begin
      br_out     <= 1'b1;
      br_cnt     <= br_dly[is_slot_d];
      br_slot_w  <= is_slot_d;
      br_taken_w <= br_rand ? $urandom : eval_branch(is_class_d, is_insn_d, is_slot_d);
      br_target_w <= is_pc_d + branch_off(is_class_d, is_insn_d);
    end
  end
end

// ---------------------------------------------------------------- event capture
int          n_commits = 0, n_dones = 0, n_traps = 0;
int          commit_slot_q [$];
logic [31:0] commit_pc_q [$];
int          commit_cls_q [$];
logic [3:0]  last_trap_slot;
logic [31:0] last_trap_pc;
int          last_trap_cause;
logic [31:0] last_trap_info;

always @(posedge clk) begin
  if (rst_n && ev_commit_valid_d) begin
    n_commits++;
    commit_slot_q.push_back(ev_slot_d);
    commit_pc_q.push_back(ev_pc_d);
    commit_cls_q.push_back(ev_class_d);
  end
  if (rst_n && ev_warp_done_valid_d) begin
    n_dones++;
    tb_free_slot(ev_warp_done_slot_d);
  end
  if (rst_n && ev_trap_valid_d) begin
    n_traps++;
    last_trap_slot  = ev_trap_slot_d;
    last_trap_pc    = ev_trap_pc_d;
    last_trap_cause = ev_trap_cause_d;
    last_trap_info  = ev_trap_info_d;
  end
end

// --------------------------------------------------------------
// Encoders for the predecoded subset (bitmaps per ISA.md 3).
function automatic logic [31:0] w_sys(input int f5, input int imm20);
  return ((32'(imm20) << 12) | (32'(f5) << 7)) | 32'h4e;
endfunction

function automatic logic [31:0] w_waitcnt(input int vm, input int vst, input int tex,
                                          input int ls, input int sm, input int sfu);
  return ((32'(sm) << 30) | (32'(sfu) << 27) | (32'(ls) << 23) | (32'(vst) << 18) |
          (32'(tex) << 13) | (32'(vm) << 7)) | 32'h50;
endfunction

function automatic logic [31:0] w_jmp(input int byte_off);
  return ((32'(byte_off >> 2) << 10)) | 32'h46;  // br22, multiples of 4
endfunction

function automatic logic [31:0] w_bop(input int f2, input int rs1, input int byte_off);
  return ((32'(byte_off >> 2) << 15) | (32'(f2) << 13) | (32'(rs1) << 7)) | 32'h4a;
endfunction

int last_slot0 = 0;  // slot0 of the most recent tb_launch (pre-consumption)

task automatic tb_launch(input int wg, input int nw, input logic [31:0] pc);
  int picked;
  if (nw < 1) return;
  launch_valid  <= 1'b1;
  launch_wg     <= 3'(wg);
  launch_nwarps <= 5'(nw);
  launch_pc     <= pc;
  // The inputs above take effect after this edge's NBA region, so the DUT first
  // samples valid at the NEXT edge.  Sample launch_ready_d at each edge wake-up
  // (still the pre-NBA value the DUT sees): the first edge with ready high is
  // the one that accepted the launch.
  do begin
    @(posedge clk);
    last_slot0 = launch_slot0_d;  // pre-consumption view (valid still pending)
  end while (!launch_ready_d);
  launch_valid <= 1'b0;
  // Never end with 'valid <= 0' in the same time step as the next launch's
  // 'valid <= 1': under Verilator the later launch was observed lost (root
  // cause not isolated); keeping valid low for a full cycle avoids it.
  picked = 0;
  for (int s = 0; s < 16; s++)
    if (!slot_busy[s] && picked < nw) begin
      slot_busy[s]  = 1'b1;
      slot_wg[s]    = wg;
      tb_wg_xid[s]  = wg;
      tb_slot_iw[s] = picked;
      picked++;
    end
  @(posedge clk);  // valid is low for one full cycle between launches
endtask

task automatic do_reset();
  rst_n <= 1'b0;
  repeat (4) @(posedge clk);
  rst_n <= 1'b1;
  repeat (2) @(posedge clk);
endtask

int errors = 0;
task automatic check(input bit c, input string msg);
  if (!c) begin
    errors++;
    $display("FAIL: %s @%0t", msg, $time);
  end
endtask

`endif  // TB_SP_SCHED_COMMON_SVH
