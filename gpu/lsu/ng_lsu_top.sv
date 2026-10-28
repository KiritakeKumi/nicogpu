// ng_lsu_top.sv -- LSU core + L1D wired together.  The integration
// view of one half-core's memory pipe: instruction issue / register-file ports
// / MMU client on one side, the line-granular fabric port (to the L2 NoC
// adapter, or to the SYS/HOST bypass for uc requests) on the other.
// LSU_K selects the core --
//   LSU_K = 1   ng_lsu, unchanged (one instruction in flight; cycle for cycle the pre-26c3 LSU, the bisection switch);
//               every request goes through the L1D, the uc channel (u_*) is idle;
//   LSU_K >= 2  ng_lsu_kn (front end / K-entry ring / back end): uc = 1 line requests of the asynchronous class go to
//               the uc channel (slot protocol, MEM_OUT slots: ng_sp_top's mport), everything else through the L1D.
module ng_lsu_top
  import ng_lsu_pkg::*;
#(
  parameter int NL    = 16,
  parameter int SLOTW = 4,
  parameter int SH_AW = 16,
  parameter int SH_LPB = 8,
  parameter int L1_LPB = 4,
  parameter int L1_SETS = 256,
  parameter int L1_WAYS = 4,
  parameter int LSU_K   = 1,          // 1 = ng_lsu, 2 / 4 / 8 = ng_lsu_kn ring entries
  parameter int MEM_OUT = 1,          // fabric slots behind the uc channel (slot tag width)
  localparam int MSW    = ng_noc_pkg::clog2_1(MEM_OUT)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- scrub / CACHE packet: external L1D invalidate pulse (OR-ed into the cache.inv.l1 path,
  // performed as soon as the L1D is idle between two requests); shared-memory row clear (ng_wd, between dispatches) ----
  input  logic              inv_l1_ext,
  input  logic              sh_clr_en,
  input  logic [SH_AW-7-1:0] sh_clr_row,

  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,
  input  logic [31:0]       iss_w1,
  input  logic [31:0]       iss_exec,
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,
  input  logic [15:0]       iss_tag,
  input  ctx_t              iss_ctx,

  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              busy,

  output logic              cnt_inc_valid,
  output logic [SLOTW-1:0]  cnt_inc_slot,
  output logic [2:0]        cnt_inc_id,
  output logic              cnt_dec_valid,
  output logic [SLOTW-1:0]  cnt_dec_slot,
  output logic [2:0]        cnt_dec_id,

  output logic                      rd_valid,
  input  logic                      rd_ready,
  output logic [SLOTW-1:0]          rd_slot,
  output logic [6:0]                rd_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat,
  output logic [1:0]                rd_hsel,
  input  logic                      rdv_valid,
  input  logic                      rdv_err,
  input  logic [NL*32-1:0]          rdv_data,

  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  output logic                      ur_valid,
  output logic [SLOTW-1:0]          ur_slot,
  output logic [5:0]                ur_reg,
  input  logic                      urd_valid,
  input  logic [31:0]               urd_data,
  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data,

  output logic                      mmu_req_valid,
  input  logic                      mmu_req_ready,
  output logic [3:0]                mmu_req_vmid,
  output logic [63:0]               mmu_req_va,
  output logic [1:0]                mmu_req_acc,
  output logic                      mmu_req_atomic,
  output logic [7:0]                mmu_req_tag,
  input  logic                      mmu_resp_valid,
  output logic                      mmu_resp_ready,
  input  logic [2:0]                mmu_resp_fault,
  input  logic                      mmu_resp_sparse,
  input  logic [47:0]               mmu_resp_pa,
  input  logic [63:0]               mmu_resp_pte,
  output logic                      ext_flt_valid,
  output logic [3:0]                ext_flt_vmid,
  output logic [63:0]               ext_flt_va,
  output logic [1:0]                ext_flt_acc,

  // ---- fabric side (L1 miss / write-through / atomic / maintenance) ----
  output logic                      mem_req_valid,
  input  logic                      mem_req_ready,
  output mreq_t                     mem_req,
  input  logic                      mem_rsp_valid,
  input  mrsp_t                     mem_rsp,

  // ---- uc channel (LSU_K >= 2; slot protocol, ng_rsp_fab_ts 9.2 / 9.11.3; idle for LSU_K = 1) ----
  output logic                      u_req_valid,
  input  logic                      u_req_ready,
  output mreq_t                     u_req,
  input  logic [MSW-1:0]            u_req_slot,
  input  logic                      u_rsp_valid,
  input  logic [MSW-1:0]            u_rsp_slot,
  input  logic [1:0]                u_rsp_err,
  output logic                      u_rd_req,
  output logic [MSW-1:0]            u_rd_slot,
  input  logic                      u_rd_gnt,
  input  logic [LINE_BITS-1:0]      u_rd_data,
  output logic                      u_free,

  output logic [31:0]               stat_l1_rd,
  output logic [31:0]               stat_l1_wr,
  output logic [31:0]               stat_l1_amo,
  output logic [31:0]               stat_sh_rounds,
  output logic [31:0]               stat_instr_mem,   // see ng_lsu
  output logic [31:0]               stat_groups,
  // debug counters, words 0..9 = ng_lsu_kn stat_kn (LSU_K = 1: word 0 = cycles with the LSU busy, the
  // rest 0), word 10 = LSU_CFG {8'(NQ), 8'(MEM_OUT), 8'(LSU_K)} (RM debug page 0x980..0x9A8)
  output logic [351:0]              stat_k,
  output logic [31:0]               stat_hit,
  output logic [31:0]               stat_miss,
  output logic [31:0]               stat_fill,
  output logic [31:0]               stat_inv
);
`ifndef SYNTHESIS
  // simulation-only statistics of ng_lsu_kn (0 for LSU_K = 1): TB reads dut.u_lsu.sim_* (no hierarchical
  // reference into the generate branch that may not exist)
  int unsigned     sim_err, sim_ring_max, sim_iof_max, sim_fe_busy, sim_ralloc, sim_iss_stall, sim_sync, sim_twait;
  int unsigned     sim_be_wait, sim_late_err;
  longint unsigned sim_ring_occ, sim_cyc;
`endif
  logic   l1_req_valid, l1_req_ready, l1_rsp_valid;
  mreq_t  l1_req;
  mrsp_t  l1_rsp;

  if (LSU_K <= 1) begin : g_core   // (same block and instance name for both cores: dut.g_core.u_lsu)
    ng_lsu #(.NL(NL), .SLOTW(SLOTW), .SH_AW(SH_AW), .SH_LPB(SH_LPB), .L1_LPB(L1_LPB)) u_lsu (
      .clk(clk), .rst_n(rst_n),
      .sh_clr_en(sh_clr_en), .sh_clr_row(sh_clr_row),
      .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1), .iss_exec(iss_exec),
      .iss_slot(iss_slot), .iss_nregs(iss_nregs), .iss_tag(iss_tag), .iss_ctx(iss_ctx),
      .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid), .trap_cause(trap_cause),
      .trap_info(trap_info), .busy(busy),
      .cnt_inc_valid(cnt_inc_valid), .cnt_inc_slot(cnt_inc_slot), .cnt_inc_id(cnt_inc_id),
      .cnt_dec_valid(cnt_dec_valid), .cnt_dec_slot(cnt_dec_slot), .cnt_dec_id(cnt_dec_id),
      .rd_valid(rd_valid), .rd_ready(rd_ready), .rd_slot(rd_slot), .rd_reg(rd_reg), .rd_beat(rd_beat),
      .rd_hsel(rd_hsel), .rdv_valid(rdv_valid), .rdv_err(rdv_err), .rdv_data(rdv_data),
      .wr_valid(wr_valid), .wr_ready(wr_ready), .wr_slot(wr_slot), .wr_reg(wr_reg), .wr_beat(wr_beat),
      .wr_lanes(wr_lanes), .wr_hsel(wr_hsel), .wr_data(wr_data),
      .ur_valid(ur_valid), .ur_slot(ur_slot), .ur_reg(ur_reg), .urd_valid(urd_valid), .urd_data(urd_data),
      .uw_valid(uw_valid), .uw_slot(uw_slot), .uw_reg(uw_reg), .uw_data(uw_data),
      .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
      .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic),
      .mmu_req_tag(mmu_req_tag), .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready),
      .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
      .mmu_resp_pte(mmu_resp_pte),
      .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
      .l1_req_valid(l1_req_valid), .l1_req_ready(l1_req_ready), .l1_req(l1_req),
      .l1_rsp_valid(l1_rsp_valid), .l1_rsp(l1_rsp),
      .stat_l1_rd(stat_l1_rd), .stat_l1_wr(stat_l1_wr), .stat_l1_amo(stat_l1_amo), .stat_sh_rounds(stat_sh_rounds),
      .stat_instr_mem(stat_instr_mem), .stat_groups(stat_groups)
    );

    assign u_req_valid = 1'b0; assign u_req = '0; assign u_rd_req = 1'b0; assign u_rd_slot = '0; assign u_free = 1'b0;
    logic [31:0] c_busy;
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) c_busy <= '0;
      else if (busy) c_busy <= c_busy + 32'd1;
    end
    assign stat_k = {16'h0, 8'(MEM_OUT), 8'd1, 288'd0, c_busy};
`ifndef SYNTHESIS
    always_comb begin
      sim_err = 0; sim_ring_max = 0; sim_iof_max = 0; sim_ring_occ = 0; sim_cyc = 0; sim_fe_busy = 0; sim_ralloc = 0;
      sim_iss_stall = 0; sim_sync = 0; sim_twait = 0; sim_be_wait = 0; sim_late_err = 0;
    end
`endif
  end else begin : g_core
    logic [319:0] stat_kn;
    ng_lsu_kn #(.NL(NL), .SLOTW(SLOTW), .SH_AW(SH_AW), .SH_LPB(SH_LPB), .L1_LPB(L1_LPB), .K(LSU_K), .MEM_OUT(MEM_OUT)) u_lsu (
      .clk(clk), .rst_n(rst_n),
      .sh_clr_en(sh_clr_en), .sh_clr_row(sh_clr_row),
      .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1), .iss_exec(iss_exec),
      .iss_slot(iss_slot), .iss_nregs(iss_nregs), .iss_tag(iss_tag), .iss_ctx(iss_ctx),
      .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid), .trap_cause(trap_cause),
      .trap_info(trap_info), .busy(busy),
      .cnt_inc_valid(cnt_inc_valid), .cnt_inc_slot(cnt_inc_slot), .cnt_inc_id(cnt_inc_id),
      .cnt_dec_valid(cnt_dec_valid), .cnt_dec_slot(cnt_dec_slot), .cnt_dec_id(cnt_dec_id),
      .rd_valid(rd_valid), .rd_ready(rd_ready), .rd_slot(rd_slot), .rd_reg(rd_reg), .rd_beat(rd_beat),
      .rd_hsel(rd_hsel), .rdv_valid(rdv_valid), .rdv_err(rdv_err), .rdv_data(rdv_data),
      .wr_valid(wr_valid), .wr_ready(wr_ready), .wr_slot(wr_slot), .wr_reg(wr_reg), .wr_beat(wr_beat),
      .wr_lanes(wr_lanes), .wr_hsel(wr_hsel), .wr_data(wr_data),
      .ur_valid(ur_valid), .ur_slot(ur_slot), .ur_reg(ur_reg), .urd_valid(urd_valid), .urd_data(urd_data),
      .uw_valid(uw_valid), .uw_slot(uw_slot), .uw_reg(uw_reg), .uw_data(uw_data),
      .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
      .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic),
      .mmu_req_tag(mmu_req_tag), .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready),
      .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
      .mmu_resp_pte(mmu_resp_pte),
      .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
      .l1_req_valid(l1_req_valid), .l1_req_ready(l1_req_ready), .l1_req(l1_req),
      .l1_rsp_valid(l1_rsp_valid), .l1_rsp(l1_rsp),
      .stat_l1_rd(stat_l1_rd), .stat_l1_wr(stat_l1_wr), .stat_l1_amo(stat_l1_amo), .stat_sh_rounds(stat_sh_rounds),
      .stat_instr_mem(stat_instr_mem), .stat_groups(stat_groups),
      .u_req_valid(u_req_valid), .u_req_ready(u_req_ready), .u_req(u_req), .u_req_slot(u_req_slot),
      .u_rsp_valid(u_rsp_valid), .u_rsp_slot(u_rsp_slot), .u_rsp_err(u_rsp_err),
      .u_rd_req(u_rd_req), .u_rd_slot(u_rd_slot), .u_rd_gnt(u_rd_gnt), .u_rd_data(u_rd_data), .u_free(u_free),
      .stat_kn(stat_kn)
    );
    assign stat_k = {8'h0, 8'(2 * MEM_OUT), 8'(MEM_OUT), 8'(LSU_K), stat_kn};
`ifndef SYNTHESIS
    always_comb begin
      sim_err = u_lsu.u_core.sim_err; sim_ring_max = u_lsu.u_core.sim_ring_max; sim_iof_max = u_lsu.u_core.sim_iof_max;
      sim_ring_occ = u_lsu.u_core.sim_ring_occ; sim_cyc = u_lsu.u_core.sim_cyc; sim_fe_busy = u_lsu.u_core.sim_fe_busy;
      sim_ralloc = u_lsu.u_core.sim_ralloc; sim_iss_stall = u_lsu.u_core.sim_iss_stall; sim_sync = u_lsu.u_core.sim_sync;
      sim_twait = u_lsu.u_core.sim_twait; sim_be_wait = u_lsu.u_core.sim_be_wait; sim_late_err = u_lsu.u_core.sim_late_err;
    end
`endif

  end

  ng_l1d #(.SETS(L1_SETS), .WAYS(L1_WAYS)) u_l1 (
    .clk(clk), .rst_n(rst_n), .ext_inv(inv_l1_ext),
    .req_valid(l1_req_valid), .req_ready(l1_req_ready), .req(l1_req),
    .rsp_valid(l1_rsp_valid), .rsp(l1_rsp),
    .mem_req_valid(mem_req_valid), .mem_req_ready(mem_req_ready), .mem_req(mem_req),
    .mem_rsp_valid(mem_rsp_valid), .mem_rsp(mem_rsp),
    .stat_hit(stat_hit), .stat_miss(stat_miss), .stat_fill(stat_fill), .stat_inv(stat_inv)
  );
endmodule
