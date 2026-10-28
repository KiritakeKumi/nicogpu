// ng_sp_top.sv -- one SP: scheduler + register file + execution units + L0
// instruction cache + work distributor.
//
//   CP wd_* --> ng_wd --launch--> ng_sp_sched --fetch--> ng_sp_icache --+--> MMU port
//                 |  regfile alloc / init writes   | issue                 +--> fabric port
//                 v                                v                       |
//            ng_sp_regfile <---- VALU / LSU(+L1D+shared) / SALU / [SFU] [XLANE] [FP64]
//
// Issue routing (one instruction from the scheduler at a time):
//   branch classes (bz/bnz/bnone/bany/bexecz/bexecnz)          -> SALU (branch)
//   ng_valu_dec hit                                            -> VALU
//   ng_lsu_dec hit                                             -> LSU
//   ng_salu_dec hit (SALU, jal/jr/jalr, CSR)                   -> SALU
//   ng_sfu_dec / ng_xlane_dec / ng_fp64_dec hit (when enabled)  -> SFU / XLANE / FP64
//   anything else                                              -> SALU (IllegalInsn, info = w0)
// (nop/endpgm/trap/brk/sleep/setprio/barrier/icache.inv/waitcnt/jmp are
// consumed by the scheduler front end and never routed.)
// Interlocks (the units hold one instruction each and trust the dispatcher):
//   * fixed-latency group {VALU, SALU, XLANE, FP64}: at most one instruction in
//     flight (INV-FX); while one runs no other fixed-group instruction is issued
//     (the group shares register-file ports 0..2 / uniform ports 0..1 / write
//     port 0). LSU / SFU instructions of other warps do issue under a
//     running fixed op; a warp never has two instructions in flight (the
//     scheduler's per-warp in-flight bit, cleared on unit done/trap).
// (BR_NODRAIN): the one exception is a pure branch (bz/bnz/bnone/bany/bexecz/bexecnz) in the SALU,
//     which runs beside the VALU: it writes no register and reads u[rs1] on uniform read port 0 only when the
//     VALU does not (sa_ur_gnt); b2.
//   * asynchronous units {LSU, SFU}: ERRATA v0.1.3 -- async instructions read
//     all sources at issue; the units read them a few cycles after accepting,
//     so a later instruction of the same warp is not issued until the unit is
//     done (conservative; a "sources read" hook would relax it).
//     Destination RAW/WAW is the program's job (waitcnt); cnt_inc at accept,
//     cnt_dec at completion.
// Register-file ports: read 0..2 fixed group, 3 LSU, 4 SFU; write 0 fixed
// group, 1 asynchronous write-back (LSU/SFU round-robin), 2 work distributor;
// uniform read 0..1 fixed group, 2 LSU; uniform write 0 fixed group (SALU low
// word), 1 LSU, 2..3 work distributor, 4 SALU high word.
// Per-warp EXEC / MODE / fflags / trap CSRs live in ng_salu (q_* query port):
// the issuing slot reads them while the scheduler presents an instruction,
// the commit slot otherwise.
// Traps (v0.1: halt and report): the first trap of any source is latched,
// forwarded to the SALU trap CSRs (xtrap_*, unless the SALU raised it), issue
// and fetch stop in the same cycle; the WD drains the units, reports
// wd_done + wd_fault, then flushes the SP.
// Reset: rst_n (global) and core_reset (CP CORE_RESET, level) both reset the
// datapath through a registered internal reset; the MMU / fabric arbiters
// only see rst_n so a transaction in flight across a core reset completes on
// the fabric and its response is dropped.
//
//
module ng_sp_top
  import ng_lsu_pkg::mreq_t;
  import ng_lsu_pkg::mrsp_t;
  import ng_lsu_pkg::ctx_t;
  import ng_lsu_pkg::LINE_BITS;
  import ng_cp_pkg::wd_req_t;
  import ng_sp_pkg::*;
#(
  parameter int NL          = 16,      // register-file lanes per beat (16 = L, 32 = XL)
  parameter int NWG         = 8,       // resident workgroup tags
  parameter int VREGS       = 1024,    // warp-registers per SP
  parameter int SH_AW       = 16,      // shared array: 2**SH_AW bytes
  parameter int L1_SETS     = 16,      // L1D shrunk for M1 (4 KiB at 16 x 2)
  parameter int L1_WAYS     = 2,
  parameter int IC_LINES    = 32,      // L0 I$ lines of 128 B (4 KiB; 8 = the KiB I$, cycle for cycle)
  parameter int IC_WAYS     = 0,       // 0 = auto (LINES <= 8: fully associative FIFO, else 4-way set associative)
  parameter bit EN_SFU      = 1'b0,
  parameter bit EN_XLANE    = 1'b0,
  parameter bit EN_FP64     = 1'b0,
  parameter bit EN_VALU_FP  = 1'b0,    // f32 VALU subset in ng_valu
  parameter bit EN_VALU_F16CVT = 1'b0, // VALU binary16 <-> binary32 conversions (needs EN_VALU_FP; 0 = illegal, as before)
  parameter int VALU_NFL    = 4,       // f32 physical lanes of the VALU (divides NL)
  parameter int VALU_BDEPTH = 3,       // VALU back-half instruction contexts; 1 = behaviour
  parameter bit BR_NODRAIN  = 1'b1,    // pure branches (bz/bnz/bnone/bany/bexecz/bexecnz) issue under a busy VALU; 0 =
  parameter int SP_ID       = 0,       // spid CSR
  parameter int CORE_ID     = 0,       // coreid CSR
  parameter int GSLOT_BASE  = 0,       // GlobalWarpSlot of slot 0 ((core * num_sp + sp) * 16)
  parameter bit CLEAR_VREGS = 1'b0,    // 1: WD zero-fills v1..v(n-1) (default 0: v1..v127 are undefined at warp start; u0..u63 are always initialised)
  // LSU instruction contexts and fabric requests in flight.
  //   LSU_K = 1, MEM_OUT = 1: the pre-26c3 SP, cycle for cycle (ng_lsu, serial fabric arbiter fa_st);
  //   LSU_K >= 2: ng_lsu_kn (FE / K-entry ring / BE); MEM_OUT > 1: the fabric port is the slot protocol of
  //   ng_rsp_fab_ts (mem_req_slot / mem_rsp_slot / mem_rd_slot / mem_free, NOUT = MEM_OUT), shared by the I$, the
  //   L1D and the LSU uc channel through the mport below; LSU_K >= 2 with MEM_OUT = 1 keeps the serial port (one
  //   request in flight; a 1-slot adapter with a line hold register in front of the mport).
  parameter int LSU_K       = 1,
  parameter int MEM_OUT     = 1,
  // LSU lanes per beat (ng_lsu_top pass-through; defaults = the ng_lsu_top defaults, unchanged behaviour):
  //   SH_LPB shared-memory round width, L1_LPB global-path scatter / merge width
  parameter int SH_LPB      = 8,
  parameter int L1_LPB      = 4,
  localparam int MSW        = ng_noc_pkg::clog2_1(MEM_OUT)
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          core_reset,     // CP CORE_RESET (level): aborts everything

  // ---- CP CACHE packet invalidates (ng_cp_top sp_inv_o, 1-cycle pulses; tie 0 without a CP) ----
  input  logic          inv_l1_ext,     // INV_L1 / INV_K: L1D invalidate (the K cache is the L1D: uld goes through it)
  input  logic          inv_i_ext,      // INV_I: I$ invalidate

  // ---- CP work-distributor port (ng_cp_top wd_*) ----
  input  logic          wd_start,
  input  wd_req_t       wd_req,
  output logic          wd_done,
  output logic          wd_fault,
  output logic [63:0]   wd_instr,       // instructions committed by the dispatch
  output logic          sp_idle,

  // ---- MMU client (ng_mmu) ----
  output logic          mmu_req_valid,
  input  logic          mmu_req_ready,
  output logic [3:0]    mmu_req_vmid,
  output logic [63:0]   mmu_req_va,
  output logic [1:0]    mmu_req_acc,
  output logic          mmu_req_atomic,
  output logic [7:0]    mmu_req_tag,
  input  logic          mmu_resp_valid,
  output logic          mmu_resp_ready,
  input  logic [2:0]    mmu_resp_fault,
  input  logic          mmu_resp_sparse,
  input  logic [47:0]   mmu_resp_pa,
  input  logic [63:0]   mmu_resp_pte,
  output logic          ext_flt_valid,
  output logic [3:0]    ext_flt_vmid,
  output logic [63:0]   ext_flt_va,
  output logic [1:0]    ext_flt_acc,

  // ---- fabric (L2 NoC adapter / uc bypass).  MEM_OUT = 1: one request in flight, the answer carries the line;
  // MEM_OUT > 1: slot protocol of ng_rsp_fab_ts  -- mem_rsp.err / .old
  // are valid with mem_rsp_valid, the line is read with mem_rd_slot / mem_rd_data (mem_rsp.rdata unused) ----
  output logic          mem_req_valid,
  input  logic          mem_req_ready,
  output mreq_t         mem_req,
  input  logic          mem_rsp_valid,
  input  mrsp_t         mem_rsp,
  input  logic [MSW-1:0] mem_req_slot,    // MEM_OUT > 1: slot of the request (valid with mem_req_ready)
  input  logic [MSW-1:0] mem_rsp_slot,    // MEM_OUT > 1: slot of the answer
  input  logic          mem_rsp_rd,       // MEM_OUT > 1: the answered slot is an M_RD (waits for mem_free)
  output logic [MSW-1:0] mem_rd_slot,     // MEM_OUT > 1: line read port / slot to free
  input  logic [LINE_BITS-1:0] mem_rd_data,
  output logic          mem_free,

  // ---- debug / trace (testbench and DFX) ----
  output logic          dbg_commit_valid,
  output logic [3:0]    dbg_commit_slot,
  output logic [31:0]   dbg_commit_pc,
  output logic [31:0]   dbg_commit_w0,
  output logic [31:0]   dbg_commit_exec,
  output logic [31:0]   dbg_commit_wg,      // linear workgroup id (x fastest)
  output logic [4:0]    dbg_commit_warp,    // warp index in the workgroup
  output logic          dbg_trap_valid,     // level: first trap of the dispatch latched
  output logic [2:0]    dbg_trap_unit,
  output logic [3:0]    dbg_trap_slot,
  output logic [31:0]   dbg_trap_pc,
  output logic [3:0]    dbg_trap_cause,
  output logic [31:0]   dbg_trap_info,
  output logic [31:0]   dbg_trap_wg,
  output logic [4:0]    dbg_trap_warp,
  output logic [31:0]   stat_ic_hit,
  output logic [31:0]   stat_ic_miss,
  // LSU counters (ng_lsu, datapath reset like stat_ic_*): line reads / writes issued, global ld / st / uld
  // instructions on the line-group path, line groups issued
  output logic [31:0]   stat_l1_rd,
  output logic [31:0]   stat_l1_wr,
  output logic [31:0]   stat_instr_mem,
  output logic [31:0]   stat_groups,
  // SP performance counters (datapath reset like stat_ic_*), 8 x 32 bit, word i = [32*i +: 32]:
  // 0 cycles with a warp slot occupied, 1 VALU back half busy, 2 pure branch presented but held (no trap),
  // 3 front starved (warp resident, VALU back half idle, issue buffer empty), 4 VALU instructions accepted,
  // 5 pure branches accepted, 6 issue buffer waiting for its warp's in-flight instruction, 7 SALU busy
  output logic [255:0]  stat_perf,
  // LSU counters (ng_lsu_top stat_k, datapath reset): RM debug page 0x980..0x9A8
  output logic [351:0]  stat_lsu,
  // counters (datapath reset), 5 x 32 bit, word i = [32*i +: 32] (RM debug page 0x9AC..0x9BC):
  // 0 dispatches that scrubbed, 1 clocks spent in the scrub step (L1D / shared / register file), 2 L1D invalidate
  // requests from outside the LSU (scrub + CP), 3 CP INV_L1 / INV_K pulses, 4 CP INV_I pulses
  output logic [159:0]  stat_scrub,
  // status summary for the RM debug page: [15:0] occupied warp slots, [19:16] WD state (ng_wd wst_t),
  // [20] WD busy, [21] units busy, [22] trap latched (halt), [23] icache busy, [24] LSU busy,
  // [26:25] fabric arbiter state, [28:27] MMU arbiter state, [29] scheduler stalled_all, [30] sp_idle,
  // [31] datapath held in reset (core_reset / flush)
  output logic [31:0]   dbg_status
);
  import ng_sp_sched_pkg::*;

  localparam int NW    = 16;            // fixed: ng_sp_sched slot ports are [3:0]
  localparam int SW    = 4;
  localparam int WGW   = $clog2(NWG);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int DW    = NL * 32;
  localparam int NU    = NUNIT;         // unit index space = ng_sp_pkg UNIT_*

  // ================================================================ resets
  logic wd_flush;
  logic crst_n;     // datapath: rst_n & !core_reset & !flush (registered)
  logic wrst_n;     // work distributor: rst_n & !core_reset (registered)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crst_n <= 1'b0;
      wrst_n <= 1'b0;
    end else begin
      crst_n <= !core_reset && !wd_flush;
      wrst_n <= !core_reset;
    end
  end

  // ================================================================ work distributor
  wd_req_t          cfg;
  logic             wd_busy;
  logic             launch_valid, launch_ready;
  logic [WGW-1:0]   launch_wg;
  logic [4:0]       launch_nwarps;
  logic [31:0]      launch_pc;
  logic [3:0]       launch_slot0;
  logic             ev_wdone_v;
  logic [3:0]       ev_wdone_slot;
  logic [WGW-1:0]   ev_wdone_wg;
  logic             trap_v;               // latched first trap (halt)
  logic             units_busy;
  logic [NW-1:0]    slot_busy;
  logic             ic_inv_wd;
  logic             wd_l1_inv, wd_sh_clr_en, wd_scrub_ev, wd_scrub_busy;
  logic [SH_AW-7-1:0] wd_sh_clr_row;
  logic [NW-1:0]    occ;

  logic             wd_done_i, wd_fault_i;
  logic             wd_alloc_valid;
  logic [SW-1:0]    wd_alloc_slot;
  logic [$clog2(VREGS)-1:0] wd_alloc_base;
  logic [7:0]       wd_alloc_nregs;
  logic             wd_wr_valid, wd_wr_ready;
  logic [SW-1:0]    wd_wr_slot;
  logic [6:0]       wd_wr_reg;
  logic [BTW-1:0]   wd_wr_beat;
  logic [NL-1:0]    wd_wr_lanes;
  logic [1:0]       wd_wr_hsel;
  logic [DW-1:0]    wd_wr_data;
  logic [1:0]       wd_uw_valid;
  logic [SW-1:0]    wd_uw_slot [2];
  logic [5:0]       wd_uw_reg  [2];
  logic [31:0]      wd_uw_data [2];
  logic             tbl_we;
  logic [SW-1:0]    tbl_slot;
  logic [31:0]      tbl_exec, tbl_wg_lin;
  logic [4:0]       tbl_widx;
  logic [WGW-1:0]   tbl_tag;
  logic [31:0]      tbl_wgid [3];
  logic [15:0]      tbl_sh_base;
  logic [3:0]       wd_dbg_st;

  ng_wd #(.NW(NW), .NWG(NWG), .NL(NL), .VREGS(VREGS), .SH_BYTES(1 << SH_AW), .CLEAR_VREGS(CLEAR_VREGS)) u_wd (
    .clk(clk), .rst_n(wrst_n),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done_i), .wd_fault(wd_fault_i), .busy(wd_busy), .cfg(cfg),
    .launch_valid(launch_valid), .launch_ready(launch_ready), .launch_wg(launch_wg),
    .launch_nwarps(launch_nwarps), .launch_pc(launch_pc), .launch_slot0(launch_slot0),
    .ev_warp_done_valid(ev_wdone_v), .ev_warp_done_slot(ev_wdone_slot), .ev_warp_done_wg(ev_wdone_wg),
    .trap_any(trap_v), .units_busy(units_busy), .slot_async_busy(slot_busy),
    .flush(wd_flush), .ic_inv(ic_inv_wd), .occ(occ), .dbg_st(wd_dbg_st),
    .l1_inv(wd_l1_inv), .sh_clr_en(wd_sh_clr_en), .sh_clr_row(wd_sh_clr_row), .scrub_ev(wd_scrub_ev),
    .scrub_busy(wd_scrub_busy),
    .alloc_valid(wd_alloc_valid), .alloc_slot(wd_alloc_slot), .alloc_base(wd_alloc_base), .alloc_nregs(wd_alloc_nregs),
    .wr_valid(wd_wr_valid), .wr_ready(wd_wr_ready), .wr_slot(wd_wr_slot), .wr_reg(wd_wr_reg), .wr_beat(wd_wr_beat),
    .wr_lanes(wd_wr_lanes), .wr_hsel(wd_wr_hsel), .wr_data(wd_wr_data),
    .uw_valid(wd_uw_valid), .uw_slot(wd_uw_slot), .uw_reg(wd_uw_reg), .uw_data(wd_uw_data),
    .tbl_we(tbl_we), .tbl_slot(tbl_slot), .tbl_exec(tbl_exec), .tbl_wg_lin(tbl_wg_lin), .tbl_widx(tbl_widx),
    .tbl_tag(tbl_tag), .tbl_wgid(tbl_wgid), .tbl_sh_base(tbl_sh_base)
  );

  // wd_done / wd_fault come only from the WD state machine.  A core reset that
  // aborts a running dispatch produces no wd_done: ng_cp_port clears its own
  // request/busy state on core_reset.
  assign wd_done  = wd_done_i;
  assign wd_fault = wd_fault_i;

  // ================================================================ per-slot tables (SP top side)
  logic [31:0]    wglin_tbl   [NW];
  logic [4:0]     widx_tbl    [NW];
  logic [15:0]    shb_tbl     [NW];
  logic [31:0]    retired_tbl [NW];   // HwInfo::Clock source
  logic [63:0]    time_ctr;           // HwInfo::Time source

  logic           ev_commit_v;
  logic [3:0]     ev_commit_slot;

  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      for (int s = 0; s < NW; s++) begin
        wglin_tbl[s] <= '0; widx_tbl[s] <= '0; shb_tbl[s] <= '0; retired_tbl[s] <= '0;
      end
      time_ctr <= '0;
    end else begin
      if (ev_commit_v) begin
        retired_tbl[ev_commit_slot] <= retired_tbl[ev_commit_slot] + 32'd1;
        time_ctr <= time_ctr + 64'd1;
      end
      if (tbl_we) begin
        wglin_tbl[tbl_slot]   <= tbl_wg_lin;
        widx_tbl[tbl_slot]    <= tbl_widx;
        shb_tbl[tbl_slot]     <= tbl_sh_base;
        retired_tbl[tbl_slot] <= '0;
      end
    end
  end

  // ================================================================ scheduler
  logic             sc_if_req_valid, sc_if_req_ready, sc_if_rsp_valid, sc_if_rsp_fault;
  logic [31:0]      sc_if_req_pc;
  logic [63:0]      sc_if_rsp_data;
  logic             is_valid, is_ready;
  logic [3:0]       is_slot, is_class;
  logic [31:0]      is_pc;
  logic [63:0]      is_insn;
  logic             br_valid, br_taken;
  logic [3:0]       br_slot;
  logic [31:0]      br_target;
  logic             cnt_inc_v, cnt_dec_v;
  logic [3:0]       cnt_inc_slot, cnt_dec_slot;
  logic [2:0]       cnt_inc_id, cnt_dec_id;
  logic [31:0]      ev_commit_pc;
  logic [3:0]       ev_commit_class;
  logic [31:0]      ev_commit_w0;
  logic             fixed_busy, halt;   // declared early: feed the scheduler's ex_hold
  logic             ev_trap_v;
  logic [3:0]       ev_trap_slot, ev_trap_cause;
  logic [31:0]      ev_trap_pc, ev_trap_info;
  logic             stalled_all, sc_ev_busy, sc_ib_busy;
  logic [NW-1:0]    infl_clr, infl_q;   // per-warp in-flight bits (owned by the scheduler)

  ng_sp_sched #(.NG_SP_NUM_WARPS(NW), .NG_SP_NUM_WGS(NWG), .NG_SP_PC_W(32)) u_sched (
    .clk(clk), .rst_n(crst_n),
    .launch_valid(launch_valid), .launch_ready(launch_ready), .launch_wg(launch_wg),
    .launch_nwarps(launch_nwarps), .launch_pc(launch_pc), .launch_slot0(launch_slot0),
    .if_req_valid(sc_if_req_valid), .if_req_ready(sc_if_req_ready), .if_req_pc(sc_if_req_pc),
    .if_rsp_valid(sc_if_rsp_valid), .if_rsp_fault(sc_if_rsp_fault), .if_rsp_data(sc_if_rsp_data),
    .ex_hold(trap_v),
    .inflight_clr(infl_clr), .inflight_o(infl_q),
    .is_valid(is_valid), .is_ready(is_ready), .is_slot(is_slot), .is_pc(is_pc), .is_insn(is_insn),
    .is_class(is_class),
    .br_valid(br_valid), .br_slot(br_slot), .br_taken(br_taken), .br_target(br_target),
    .cnt_inc_valid(cnt_inc_v), .cnt_inc_slot(cnt_inc_slot), .cnt_inc_id(cnt_inc_id),
    .cnt_dec_valid(cnt_dec_v), .cnt_dec_slot(cnt_dec_slot), .cnt_dec_id(cnt_dec_id),
    .ev_commit_valid(ev_commit_v), .ev_slot(ev_commit_slot), .ev_pc(ev_commit_pc), .ev_class(ev_commit_class), .ev_w0(ev_commit_w0),
    .ev_warp_done_valid(ev_wdone_v), .ev_warp_done_slot(ev_wdone_slot), .ev_warp_done_wg(ev_wdone_wg),
    .ev_trap_valid(ev_trap_v), .ev_trap_slot(ev_trap_slot), .ev_trap_pc(ev_trap_pc),
    .ev_trap_cause(ev_trap_cause), .ev_trap_info(ev_trap_info),
    .stalled_all(stalled_all), .ev_busy(sc_ev_busy), .ib_busy(sc_ib_busy)
  );

  // ================================================================ in-flight records
  // one per unit: set at accept, cleared at done/trap
  logic [NU-1:0]  u_iss_valid, u_iss_ready, u_done, u_trap;
  logic [3:0]     u_trap_cause [NU];
  logic [31:0]    u_trap_info  [NU];
  logic [NU-1:0]  rec_v;        // per-unit "instruction in flight" (VALU: any of its up to VRD)
  logic [SW-1:0]  rec_slot [NU];  // ... slot of the oldest one
  logic [31:0]    rec_pc   [NU];
  logic [NU-1:0]  rec_v_q;
  logic [SW-1:0]  rec_slot_q [NU];
  logic [31:0]    rec_pc_q   [NU];
  // the VALU holds up to two instructions (front/back half); the back half holds
  // VALU_BDEPTH of them.  VRD-entry FIFO in acceptance order, head = oldest
  localparam int VRD = VALU_BDEPTH + 1;
  localparam int VCW = $clog2(VRD + 1);
  logic [VCW-1:0] vr_cnt;
  logic [SW-1:0]  vr_slot [VRD];
  logic [31:0]    vr_pc   [VRD];
  // with LSU_K >= 2 the LSU holds up to LSU_K + 1 instructions (FE + ring) and returns their done / trap
  // pulses in acceptance order (ng_lsu_kn): LRD-entry FIFO like the VALU's, head = oldest.  LSU_K = 1: rec_v_q.
  localparam int LRD = LSU_K + 1;
  localparam int LCW = $clog2(LRD + 1);
  logic [LCW-1:0] lr_cnt;
  logic [SW-1:0]  lr_slot [LRD];
  logic [31:0]    lr_pc   [LRD];
  always_comb begin
    for (int u = 0; u < NU; u++) begin
      rec_v[u] = rec_v_q[u]; rec_slot[u] = rec_slot_q[u]; rec_pc[u] = rec_pc_q[u];
    end
    rec_v[UNIT_VALU]    = (vr_cnt != '0);
    rec_slot[UNIT_VALU] = vr_slot[0];
    rec_pc[UNIT_VALU]   = vr_pc[0];
    if (LSU_K > 1) begin
      rec_v[UNIT_LSU]    = (lr_cnt != '0);
      rec_slot[UNIT_LSU] = lr_slot[0];
      rec_pc[UNIT_LSU]   = lr_pc[0];
    end
  end

  function automatic logic is_fixed(input logic [2:0] u);
    return (u == UNIT_VALU) || (u == UNIT_SALU) || (u == UNIT_XLANE) || (u == UNIT_FP64);
  endfunction

  logic [SW-1:0] fixed_slot;
  always_comb begin
    fixed_busy = 1'b0;
    fixed_slot = '0;
    for (int u = 0; u < NU; u++)
      if (rec_v[u] && is_fixed(3'(u))) begin fixed_busy = 1'b1; fixed_slot = rec_slot[u]; end
  end

  // ================================================================ trap halt
  logic any_trap_pulse;
  assign halt = trap_v || any_trap_pulse;

  // ================================================================ L0 instruction cache
  logic          ic_mmu_req_valid, ic_mmu_req_ready, ic_mmu_resp_valid, ic_mmu_resp_ready;
  logic [3:0]    ic_mmu_req_vmid;
  logic [63:0]   ic_mmu_req_va;
  logic [1:0]    ic_mmu_req_acc;
  logic          ic_mmu_req_atomic;
  logic [7:0]    ic_mmu_req_tag;
  logic          ic_mem_req_valid, ic_mem_req_ready, ic_mem_rsp_valid;
  mrsp_t         fab_rsp;         // fabric answer seen by the I$ / L1D (serial port, or built by the mport)
  mreq_t         ic_mem_req;
  logic          ic_busy;
  logic [31:0]   ic_fault_info;
  logic          ic_req_valid, ic_req_ready;
  logic          fetch_ok;
  logic          ic_ef_valid, ls_ef_valid;                  // BAD_PHYS_ADDR reports: I$  and LSU, single-cycle pulses
  logic [3:0]    ic_ef_vmid, ls_ef_vmid;
  logic [63:0]   ic_ef_va, ls_ef_va;
  logic [1:0]    ic_ef_acc, ls_ef_acc;

  // One ext_flt port towards the MMU fault latch: the LSU's report wins a same-cycle collision; the I$'s is dropped
  // then (the downstream report paths keep one pending entry and drop later reports too, ng_rsp_flt_ts / ng_core_fltarb;
  // only MULTI is lost, the first fault is latched either way).
  assign ext_flt_valid = ls_ef_valid || ic_ef_valid;
  assign ext_flt_vmid  = ls_ef_valid ? ls_ef_vmid : ic_ef_vmid;
  assign ext_flt_va    = ls_ef_valid ? ls_ef_va   : ic_ef_va;
  assign ext_flt_acc   = ls_ef_valid ? ls_ef_acc  : ic_ef_acc;

  // fetch (incl. prefetch of the next instruction) proceeds while a
  // fixed-latency op executes; what stays serialized is *processing*: the
  // scheduler holds scheduler-resolved instructions via ex_hold, and issue
  // is blocked below.  Only a trap stops fetching.
  assign fetch_ok        = !halt;
  assign ic_req_valid    = sc_if_req_valid && fetch_ok;
  assign sc_if_req_ready = ic_req_ready && fetch_ok;

  ng_sp_icache #(.LINES(IC_LINES), .WAYS(IC_WAYS)) u_icache (
    .clk(clk), .rst_n(crst_n),
    .inv(ic_inv_wd || inv_i_ext || (ev_commit_v && ev_commit_class == CLS_ICACHE)),
    .code_base(cfg.code_base[47:0]), .vmid(cfg.vmid),
    .req_valid(ic_req_valid), .req_ready(ic_req_ready), .req_pc(sc_if_req_pc),
    .rsp_valid(sc_if_rsp_valid), .rsp_fault(sc_if_rsp_fault), .rsp_data(sc_if_rsp_data),
    .rsp_fault_info(ic_fault_info),
    .mmu_req_valid(ic_mmu_req_valid), .mmu_req_ready(ic_mmu_req_ready), .mmu_req_vmid(ic_mmu_req_vmid),
    .mmu_req_va(ic_mmu_req_va), .mmu_req_acc(ic_mmu_req_acc), .mmu_req_atomic(ic_mmu_req_atomic),
    .mmu_req_tag(ic_mmu_req_tag), .mmu_resp_valid(ic_mmu_resp_valid), .mmu_resp_ready(ic_mmu_resp_ready),
    .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
    .ext_flt_valid(ic_ef_valid), .ext_flt_vmid(ic_ef_vmid), .ext_flt_va(ic_ef_va), .ext_flt_acc(ic_ef_acc),
    .mem_req_valid(ic_mem_req_valid), .mem_req_ready(ic_mem_req_ready), .mem_req(ic_mem_req),
    .mem_rsp_valid(ic_mem_rsp_valid), .mem_rsp(fab_rsp),
    .busy(ic_busy), .stat_hit(stat_ic_hit), .stat_miss(stat_ic_miss)
  );

  // fetch-fault VA of the last response (the scheduler reports the code offset
  // as trapinfo; the model reports the faulting word's VA).  A response is
  // only held in the scheduler's buffer, never overwritten before it is
  // consumed or dropped, so this is the faulting word's info when the trap fires.
  // (The committed instruction's w0 now comes from the scheduler, ev_commit_w0:
  // prefetching means the latest response is not the committed instruction.)
  logic [31:0] last_finfo;
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      last_finfo <= '0;
    end else if (sc_if_rsp_valid) begin
      last_finfo <= ic_fault_info;
    end
  end

  // ================================================================ issue routing
  ng_valu_pkg::dec_t rt_vd;
  ng_lsu_pkg::dec_t  rt_ld;
  ng_salu_pkg::dec_t rt_sd;
  ng_valu_dec #(.EN_FP(EN_VALU_FP), .EN_F16CVT(EN_VALU_F16CVT)) u_rt_valu (.w0(is_insn[31:0]), .w1(is_insn[63:32]), .d(rt_vd));
  ng_lsu_dec  u_rt_lsu  (.w0(is_insn[31:0]), .w1(is_insn[63:32]), .d(rt_ld));
  ng_salu_dec u_rt_salu (.w0(is_insn[31:0]), .w1(is_insn[63:32]), .d(rt_sd));
  logic rt_sfu_hit, rt_xl_hit, rt_fp_hit;

  wire is_brcls = (is_class == CLS_BZ) || (is_class == CLS_BNZ) || (is_class == CLS_BNONE) ||
                  (is_class == CLS_BANY) || (is_class == CLS_BEXECZ) || (is_class == CLS_BEXECNZ);
  logic [2:0] tgt;
  always_comb begin
    if (is_brcls)                   tgt = UNIT_SALU;
    else if (rt_vd.hit)             tgt = UNIT_VALU;
    else if (rt_ld.hit)             tgt = UNIT_LSU;
    else if (rt_sd.hit)             tgt = UNIT_SALU;
    else if (EN_SFU && rt_sfu_hit)  tgt = UNIT_SFU;
    else if (EN_XLANE && rt_xl_hit) tgt = UNIT_XLANE;
    else if (EN_FP64 && rt_fp_hit)  tgt = UNIT_FP64;
    else                            tgt = UNIT_SALU;   // IllegalInsn from the SALU decoder
  end

  logic blk, iss_go, fx_oth_busy, fx_nv_busy, sa_br_q;
  // sa_br_q = the SALU's instruction in flight is a pure branch (bz/bnz/bnone/bany/bexecz/bexecnz: reads
  // at most u[rs1] + EXEC, writes nothing but br_*).  With BR_NODRAIN such a branch and the VALU run side by side.
  assign fx_nv_busy  = rec_v[UNIT_SALU] || rec_v[UNIT_XLANE] || rec_v[UNIT_FP64];
  assign fx_oth_busy = (rec_v[UNIT_SALU] && !(BR_NODRAIN && sa_br_q)) || rec_v[UNIT_XLANE] || rec_v[UNIT_FP64];
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) sa_br_q <= 1'b0;
    else if (u_iss_valid[UNIT_SALU] && u_iss_ready[UNIT_SALU]) sa_br_q <= is_brcls;
  end
  always_comb begin
    // only a fixed-latency target waits for a running fixed-latency op
    // (they share regfile ports 0..2 / write port 0 / uniform ports, INV-FX).
    // LSU / SFU targets issue under a running VALU/SALU/XLANE/FP64 op of
    // another warp (own regfile / uniform ports).  Same-warp ordering is the
    // scheduler's per-warp in-flight bit (no instruction of a warp is presented
    // while its previous one is in flight), so no same-slot check is needed here.
    // the VALU takes a second instruction behind the first (its own iss_ready decides),
    // but not while SALU/XLANE/FP64 run; those wait for the whole VALU (both halves) to drain.
    // (BR_NODRAIN): a pure branch only waits for SALU / XLANE / FP64, not for the VALU (same-warp RAW on its
    // u register is the scheduler's in-flight bit); the VALU then also accepts under that branch (fx_oth_busy).
    if (tgt == UNIT_VALU)             blk = halt || fx_oth_busy;
    else if (BR_NODRAIN && is_brcls)  blk = halt || fx_nv_busy;
    else                              blk = halt || (is_fixed(tgt) && fixed_busy);
    iss_go = is_valid && !blk;
    for (int u = 0; u < NU; u++) u_iss_valid[u] = iss_go && (tgt == 3'(u));
    is_ready = iss_go && u_iss_ready[tgt];
  end
  assign u_iss_ready[UNIT_SCHED] = 1'b0;
  assign u_done[UNIT_SCHED] = 1'b0;
  assign u_trap[UNIT_SCHED] = 1'b0;
  assign u_trap_cause[UNIT_SCHED] = '0;
  assign u_trap_info[UNIT_SCHED] = '0;

  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      rec_v_q <= '0;
      for (int u = 0; u < NU; u++) begin rec_slot_q[u] <= '0; rec_pc_q[u] <= '0; end
    end else begin
      for (int u = 0; u < NU; u++) begin
        if (u_done[u] || u_trap[u]) rec_v_q[u] <= 1'b0;
        if (u_iss_valid[u] && u_iss_ready[u]) begin
          rec_v_q[u]    <= 1'b1;
          rec_slot_q[u] <= is_slot;
          rec_pc_q[u]   <= is_pc;
        end
      end
    end
  end

  // VALU in-flight FIFO (acceptance order; the unit returns exactly one done / trap pulse per
  // instruction in that order, see ng_valu).
  logic va_push, va_pop;
  assign va_push = u_iss_valid[UNIT_VALU] && u_iss_ready[UNIT_VALU];
  assign va_pop  = u_done[UNIT_VALU] || u_trap[UNIT_VALU];
  ng_sp_infl_q #(.DEPTH(VRD), .SW(SW), .CW(VCW)) u_vq (
    .clk(clk), .rst_n(crst_n), .push(va_push), .pop(va_pop), .in_slot(is_slot), .in_pc(is_pc),
    .cnt(vr_cnt), .slot(vr_slot), .pc(vr_pc));

  // LSU in-flight FIFO (used for LSU_K >= 2 only; acceptance order, one done / trap per instruction)
  logic lr_push, lr_pop;
  assign lr_push = u_iss_valid[UNIT_LSU] && u_iss_ready[UNIT_LSU];
  assign lr_pop  = u_done[UNIT_LSU] || u_trap[UNIT_LSU];
  ng_sp_infl_q #(.DEPTH(LRD), .SW(SW), .CW(LCW)) u_lq (
    .clk(clk), .rst_n(crst_n), .push(lr_push), .pop(lr_pop), .in_slot(is_slot), .in_pc(is_pc),
    .cnt(lr_cnt), .slot(lr_slot), .pc(lr_pc));

  // per-warp in-flight clear: unit done/trap for the slot recorded at its accept
  always_comb begin
    infl_clr = '0;
    for (int u = 1; u < NU; u++)
      if (rec_v[u] && (u_done[u] || u_trap[u])) infl_clr[rec_slot[u]] = 1'b1;
  end

  // ================================================================ SALU state query (EXEC / MODE / fflags)
  logic [SW-1:0] q_slot;
  logic [31:0]   q_exec, q_mode, q_trapctl;
  logic [4:0]    q_fflags;
  // the issue query port always follows the presented instruction's slot;
  // the commit trace has its own EXEC read port (qc_*), so a (branch) commit pulse may
  // coincide with a presented instruction of another warp.
  assign q_slot = is_slot;
  logic [31:0] qc_exec;


  // EXEC of the issued instruction (commit trace: exec_before)
  logic [31:0] iss_exec_q;
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) iss_exec_q <= '0;
    else if (is_valid && is_ready) iss_exec_q <= q_exec;
  end

  ctx_t lsu_ctx;
  always_comb begin
    lsu_ctx           = '0;
    lsu_ctx.vmid      = cfg.vmid;
    lsu_ctx.sh_base   = shb_tbl[is_slot];
    lsu_ctx.sh_size   = cfg.shared_bytes;
    lsu_ctx.scr_base  = cfg.scratch_va[47:0];
    lsu_ctx.scr_bytes = cfg.scratch_bytes;
    lsu_ctx.gslot     = 32'(GSLOT_BASE) + {28'd0, is_slot};
  end

  // ================================================================ register file
  localparam int NRP = 5, NWP = 3, NURP = 3, NUWP = 5;
  logic [NRP-1:0]  rf_rd_valid, rf_rd_ready, rf_rdv_valid, rf_rdv_err;
  logic [SW-1:0]   rf_rd_slot [NRP];
  logic [6:0]      rf_rd_reg  [NRP];
  logic [BTW-1:0]  rf_rd_beat [NRP];
  logic [1:0]      rf_rd_hsel [NRP];
  logic [DW-1:0]   rf_rdv_data [NRP];
  logic [NWP-1:0]  rf_wr_valid, rf_wr_ready, rf_wr_err;
  logic [SW-1:0]   rf_wr_slot [NWP];
  logic [6:0]      rf_wr_reg  [NWP];
  logic [BTW-1:0]  rf_wr_beat [NWP];
  logic [NL-1:0]   rf_wr_lanes [NWP];
  logic [1:0]      rf_wr_hsel [NWP];
  logic [DW-1:0]   rf_wr_data [NWP];
  logic [NURP-1:0] rf_ur_valid, rf_urd_valid;
  logic [SW-1:0]   rf_ur_slot [NURP];
  logic [5:0]      rf_ur_reg  [NURP];
  logic [31:0]     rf_urd_data [NURP];
  logic [NUWP-1:0] rf_uw_valid;
  logic [SW-1:0]   rf_uw_slot [NUWP];
  logic [5:0]      rf_uw_reg  [NUWP];
  logic [31:0]     rf_uw_data [NUWP];
  logic            rf_alloc_err, rf_perf_conflict;

  ng_sp_regfile #(.NG_RF_NUM_WARPS(NW), .NG_RF_NL(NL), .NG_RF_VREGS(VREGS), .NG_RF_NBANK(4), .NG_RF_NUREG(64),
                  .NG_RF_NRP(NRP), .NG_RF_NWP(NWP), .NG_RF_NURP(NURP), .NG_RF_NUWP(NUWP)) u_rf (
    .clk(clk), .rst_n(crst_n),
    .alloc_valid(wd_alloc_valid), .alloc_slot(wd_alloc_slot), .alloc_base(wd_alloc_base),
    .alloc_nregs(wd_alloc_nregs), .alloc_err(rf_alloc_err),
    .rd_valid(rf_rd_valid), .rd_ready(rf_rd_ready), .rd_slot(rf_rd_slot), .rd_reg(rf_rd_reg),
    .rd_beat(rf_rd_beat), .rd_hsel(rf_rd_hsel), .rdv_valid(rf_rdv_valid), .rdv_err(rf_rdv_err),
    .rdv_data(rf_rdv_data), .perf_rd_conflict(rf_perf_conflict),
    .wr_valid(rf_wr_valid), .wr_ready(rf_wr_ready), .wr_slot(rf_wr_slot), .wr_reg(rf_wr_reg),
    .wr_beat(rf_wr_beat), .wr_lanes(rf_wr_lanes), .wr_hsel(rf_wr_hsel), .wr_data(rf_wr_data), .wr_err(rf_wr_err),
    .ur_valid(rf_ur_valid), .ur_slot(rf_ur_slot), .ur_reg(rf_ur_reg), .urd_valid(rf_urd_valid),
    .urd_data(rf_urd_data),
    .uw_valid(rf_uw_valid), .uw_slot(rf_uw_slot), .uw_reg(rf_uw_reg), .uw_data(rf_uw_data)
  );

  // ================================================================ unit port signals
  // VALU
  logic [2:0]      va_rd_valid;
  logic [SW-1:0]   va_rd_slot [3];
  logic [6:0]      va_rd_reg  [3];
  logic [BTW-1:0]  va_rd_beat [3];
  logic [1:0]      va_rd_hsel [3];
  logic            va_wr_valid;
  logic [SW-1:0]   va_wr_slot;
  logic [6:0]      va_wr_reg;
  logic [BTW-1:0]  va_wr_beat;
  logic [NL-1:0]   va_wr_lanes;
  logic [1:0]      va_wr_hsel;
  logic [DW-1:0]   va_wr_data;
  logic [1:0]      va_ur_valid;
  logic [SW-1:0]   va_ur_slot [2];
  logic [5:0]      va_ur_reg  [2];
  logic            va_uw_valid;
  logic [SW-1:0]   va_uw_slot;
  logic [5:0]      va_uw_reg;
  logic [31:0]     va_uw_data;
  logic            va_busy, va_bbusy;
  logic [15:0]     va_done_tag;
  // XLANE
  logic [2:0]      xl_rd_valid;
  logic [SW-1:0]   xl_rd_slot [3];
  logic [6:0]      xl_rd_reg  [3];
  logic [BTW-1:0]  xl_rd_beat [3];
  logic [1:0]      xl_rd_hsel [3];
  logic            xl_wr_valid;
  logic [SW-1:0]   xl_wr_slot;
  logic [6:0]      xl_wr_reg;
  logic [BTW-1:0]  xl_wr_beat;
  logic [NL-1:0]   xl_wr_lanes;
  logic [1:0]      xl_wr_hsel;
  logic [DW-1:0]   xl_wr_data;
  logic [1:0]      xl_ur_valid;
  logic [SW-1:0]   xl_ur_slot [2];
  logic [5:0]      xl_ur_reg  [2];
  logic            xl_uw_valid;
  logic [SW-1:0]   xl_uw_slot;
  logic [5:0]      xl_uw_reg;
  logic [31:0]     xl_uw_data;
  // FP64
  logic            fp_rd_valid;
  logic [SW-1:0]   fp_rd_slot;
  logic [6:0]      fp_rd_reg;
  logic [BTW-1:0]  fp_rd_beat;
  logic [1:0]      fp_rd_hsel;
  logic            fp_wr_valid;
  logic [SW-1:0]   fp_wr_slot;
  logic [6:0]      fp_wr_reg;
  logic [BTW-1:0]  fp_wr_beat;
  logic [NL-1:0]   fp_wr_lanes;
  logic [1:0]      fp_wr_hsel;
  logic [DW-1:0]   fp_wr_data;
  logic            fp_uw_valid;
  logic [SW-1:0]   fp_uw_slot;
  logic [5:0]      fp_uw_reg;
  logic [31:0]     fp_uw_data;
  // SALU
  logic [1:0]      sa_ur_valid;
  logic [SW-1:0]   sa_ur_slot [2];
  logic [5:0]      sa_ur_reg  [2];
  logic [1:0]      sa_uw_valid;
  logic [SW-1:0]   sa_uw_slot [2];
  logic [5:0]      sa_uw_reg  [2];
  logic [31:0]     sa_uw_data [2];
  logic            sa_busy;
  logic            sa_ur_gnt;   // uniform read ports 0..1 granted to the SALU (the VALU has priority)
  // LSU
  logic            ls_rd_valid, ls_rd_ready;
  logic [SW-1:0]   ls_rd_slot;
  logic [6:0]      ls_rd_reg;
  logic [BTW-1:0]  ls_rd_beat;
  logic [1:0]      ls_rd_hsel;
  logic            ls_wr_valid, ls_wr_ready;
  logic [SW-1:0]   ls_wr_slot;
  logic [6:0]      ls_wr_reg;
  logic [BTW-1:0]  ls_wr_beat;
  logic [NL-1:0]   ls_wr_lanes;
  logic [1:0]      ls_wr_hsel;
  logic [DW-1:0]   ls_wr_data;
  logic            ls_ur_valid;
  logic [SW-1:0]   ls_ur_slot;
  logic [5:0]      ls_ur_reg;
  logic            ls_uw_valid;
  logic [SW-1:0]   ls_uw_slot;
  logic [5:0]      ls_uw_reg;
  logic [31:0]     ls_uw_data;
  logic            ls_cnt_inc_v, ls_cnt_dec_v;
  logic [SW-1:0]   ls_cnt_inc_slot, ls_cnt_dec_slot;
  logic [2:0]      ls_cnt_inc_id, ls_cnt_dec_id;
  logic            ls_busy;
  logic [15:0]     ls_done_tag;
  // SFU
  logic            sf_rd_valid, sf_rd_ready;
  logic [SW-1:0]   sf_rd_slot;
  logic [6:0]      sf_rd_reg;
  logic [BTW-1:0]  sf_rd_beat;
  logic [1:0]      sf_rd_hsel;
  logic            sf_wr_valid, sf_wr_ready;
  logic [SW-1:0]   sf_wr_slot;
  logic [6:0]      sf_wr_reg;
  logic [BTW-1:0]  sf_wr_beat;
  logic [NL-1:0]   sf_wr_lanes;
  logic [1:0]      sf_wr_hsel;
  logic [DW-1:0]   sf_wr_data;
  // fflags sources: [0] SFU (async), [1] XLANE, [2] FP64, [3] VALU (f32)
  logic            ff_v    [4];
  logic [4:0]      ff_bits [4];

  // ================================================================ register-file port muxing
  always_comb begin
    // read ports 0..2: fixed group (at most one unit active)
    for (int k = 0; k < 3; k++) begin
      rf_rd_valid[k] = va_rd_valid[k] | xl_rd_valid[k] | ((k == 0) && fp_rd_valid);
      if (xl_rd_valid[k]) begin
        rf_rd_slot[k] = xl_rd_slot[k]; rf_rd_reg[k] = xl_rd_reg[k]; rf_rd_beat[k] = xl_rd_beat[k];
        rf_rd_hsel[k] = xl_rd_hsel[k];
      end else if ((k == 0) && fp_rd_valid) begin
        rf_rd_slot[k] = fp_rd_slot; rf_rd_reg[k] = fp_rd_reg; rf_rd_beat[k] = fp_rd_beat; rf_rd_hsel[k] = fp_rd_hsel;
      end else begin
        rf_rd_slot[k] = va_rd_slot[k]; rf_rd_reg[k] = va_rd_reg[k]; rf_rd_beat[k] = va_rd_beat[k];
        rf_rd_hsel[k] = va_rd_hsel[k];
      end
    end
    // read port 3: LSU, read port 4: SFU
    rf_rd_valid[3] = ls_rd_valid; rf_rd_slot[3] = ls_rd_slot; rf_rd_reg[3] = ls_rd_reg;
    rf_rd_beat[3]  = ls_rd_beat;  rf_rd_hsel[3] = ls_rd_hsel;
    rf_rd_valid[4] = sf_rd_valid; rf_rd_slot[4] = sf_rd_slot; rf_rd_reg[4] = sf_rd_reg;
    rf_rd_beat[4]  = sf_rd_beat;  rf_rd_hsel[4] = sf_rd_hsel;
  end
  assign ls_rd_ready = rf_rd_ready[3];
  assign sf_rd_ready = rf_rd_ready[4];

  // write port 1 arbitration: asynchronous write-back (LSU / SFU), round-robin
  logic wb_pref_sfu, wb_sel_sfu;
  assign wb_sel_sfu  = sf_wr_valid && (!ls_wr_valid || wb_pref_sfu);
  assign ls_wr_ready = rf_wr_ready[1] && !wb_sel_sfu;
  assign sf_wr_ready = rf_wr_ready[1] && wb_sel_sfu;
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) wb_pref_sfu <= 1'b0;
    else if (rf_wr_valid[1] && rf_wr_ready[1]) wb_pref_sfu <= !wb_sel_sfu;
  end

  always_comb begin
    // write port 0: fixed group
    rf_wr_valid[0] = va_wr_valid | xl_wr_valid | fp_wr_valid;
    if (xl_wr_valid) begin
      rf_wr_slot[0] = xl_wr_slot; rf_wr_reg[0] = xl_wr_reg; rf_wr_beat[0] = xl_wr_beat;
      rf_wr_lanes[0] = xl_wr_lanes; rf_wr_hsel[0] = xl_wr_hsel; rf_wr_data[0] = xl_wr_data;
    end else if (fp_wr_valid) begin
      rf_wr_slot[0] = fp_wr_slot; rf_wr_reg[0] = fp_wr_reg; rf_wr_beat[0] = fp_wr_beat;
      rf_wr_lanes[0] = fp_wr_lanes; rf_wr_hsel[0] = fp_wr_hsel; rf_wr_data[0] = fp_wr_data;
    end else begin
      rf_wr_slot[0] = va_wr_slot; rf_wr_reg[0] = va_wr_reg; rf_wr_beat[0] = va_wr_beat;
      rf_wr_lanes[0] = va_wr_lanes; rf_wr_hsel[0] = va_wr_hsel; rf_wr_data[0] = va_wr_data;
    end
    // write port 1: asynchronous write-back
    rf_wr_valid[1] = ls_wr_valid | sf_wr_valid;
    if (wb_sel_sfu) begin
      rf_wr_slot[1] = sf_wr_slot; rf_wr_reg[1] = sf_wr_reg; rf_wr_beat[1] = sf_wr_beat;
      rf_wr_lanes[1] = sf_wr_lanes; rf_wr_hsel[1] = sf_wr_hsel; rf_wr_data[1] = sf_wr_data;
    end else begin
      rf_wr_slot[1] = ls_wr_slot; rf_wr_reg[1] = ls_wr_reg; rf_wr_beat[1] = ls_wr_beat;
      rf_wr_lanes[1] = ls_wr_lanes; rf_wr_hsel[1] = ls_wr_hsel; rf_wr_data[1] = ls_wr_data;
    end
    // write port 2: work distributor
    rf_wr_valid[2] = wd_wr_valid; rf_wr_slot[2] = wd_wr_slot; rf_wr_reg[2] = wd_wr_reg;
    rf_wr_beat[2] = wd_wr_beat; rf_wr_lanes[2] = wd_wr_lanes; rf_wr_hsel[2] = wd_wr_hsel;
    rf_wr_data[2] = wd_wr_data;
  end
  assign wd_wr_ready = rf_wr_ready[2];

  always_comb begin
    // uniform read ports 0..1: fixed group (VALU / XLANE / SALU); 2: LSU
    // a pure branch in the SALU may run beside the VALU; the VALU's uniform reads (one ST_CHK cycle per
    // instruction) win and the SALU's S_RD0 waits (ur_ready).  XLANE never runs beside either of them.
    for (int k = 0; k < 2; k++) begin
      rf_ur_valid[k] = va_ur_valid[k] | xl_ur_valid[k] | (sa_ur_valid[k] & sa_ur_gnt);
      if (xl_ur_valid[k])                   begin rf_ur_slot[k] = xl_ur_slot[k]; rf_ur_reg[k] = xl_ur_reg[k]; end
      else if (sa_ur_valid[k] && sa_ur_gnt) begin rf_ur_slot[k] = sa_ur_slot[k]; rf_ur_reg[k] = sa_ur_reg[k]; end
      else                     begin rf_ur_slot[k] = va_ur_slot[k]; rf_ur_reg[k] = va_ur_reg[k]; end
    end
    rf_ur_valid[2] = ls_ur_valid; rf_ur_slot[2] = ls_ur_slot; rf_ur_reg[2] = ls_ur_reg;
    // uniform write ports: 0 fixed group (SALU low word), 1 LSU, 2..3 WD, 4 SALU high word
    rf_uw_valid[0] = va_uw_valid | xl_uw_valid | fp_uw_valid | sa_uw_valid[0];
    if (xl_uw_valid)         begin rf_uw_slot[0] = xl_uw_slot; rf_uw_reg[0] = xl_uw_reg; rf_uw_data[0] = xl_uw_data; end
    else if (fp_uw_valid)    begin rf_uw_slot[0] = fp_uw_slot; rf_uw_reg[0] = fp_uw_reg; rf_uw_data[0] = fp_uw_data; end
    else if (sa_uw_valid[0]) begin rf_uw_slot[0] = sa_uw_slot[0]; rf_uw_reg[0] = sa_uw_reg[0]; rf_uw_data[0] = sa_uw_data[0]; end
    else                     begin rf_uw_slot[0] = va_uw_slot; rf_uw_reg[0] = va_uw_reg; rf_uw_data[0] = va_uw_data; end
    rf_uw_valid[1] = ls_uw_valid; rf_uw_slot[1] = ls_uw_slot; rf_uw_reg[1] = ls_uw_reg; rf_uw_data[1] = ls_uw_data;
    for (int p = 0; p < 2; p++) begin
      rf_uw_valid[2 + p] = wd_uw_valid[p]; rf_uw_slot[2 + p] = wd_uw_slot[p];
      rf_uw_reg[2 + p]   = wd_uw_reg[p];   rf_uw_data[2 + p] = wd_uw_data[p];
    end
    rf_uw_valid[4] = sa_uw_valid[1]; rf_uw_slot[4] = sa_uw_slot[1]; rf_uw_reg[4] = sa_uw_reg[1];
    rf_uw_data[4] = sa_uw_data[1];
  end

  // read-response fan-out (units only look at a port they requested)
  logic [2:0]    fx_rdv_valid, fx_rdv_err;
  logic [DW-1:0] fx_rdv_data [3];
  logic [1:0]    fx_urd_valid;
  logic [31:0]   fx_urd_data [2];
  always_comb begin
    for (int k = 0; k < 3; k++) begin
      fx_rdv_valid[k] = rf_rdv_valid[k]; fx_rdv_err[k] = rf_rdv_err[k]; fx_rdv_data[k] = rf_rdv_data[k];
    end
    for (int k = 0; k < 2; k++) begin fx_urd_valid[k] = rf_urd_valid[k]; fx_urd_data[k] = rf_urd_data[k]; end
  end

  // ================================================================ VALU
  ng_valu #(.NL(NL), .SLOTW(SW), .EN_FP(EN_VALU_FP), .NFL(VALU_NFL), .BDEPTH(VALU_BDEPTH), .EN_F16CVT(EN_VALU_F16CVT)) u_valu (
    .clk(clk), .rst_n(crst_n),
    .iss_valid(u_iss_valid[UNIT_VALU]), .iss_ready(u_iss_ready[UNIT_VALU]),
    .iss_w0(is_insn[31:0]), .iss_w1(is_insn[63:32]), .iss_exec(q_exec), .iss_slot(is_slot),
    .iss_nregs(cfg.num_vregs[7:0]), .iss_mode(q_mode), .iss_tag(16'(is_pc)),
    .done_valid(u_done[UNIT_VALU]), .done_tag(va_done_tag), .trap_valid(u_trap[UNIT_VALU]),
    .trap_cause(u_trap_cause[UNIT_VALU]), .trap_info(u_trap_info[UNIT_VALU]),
    .fflags_valid(ff_v[3]), .fflags_set(ff_bits[3]), .busy(va_busy), .bbusy(va_bbusy),
    .rd_valid(va_rd_valid), .rd_ready(rf_rd_ready[2:0]), .rd_slot(va_rd_slot), .rd_reg(va_rd_reg),
    .rd_beat(va_rd_beat), .rd_hsel(va_rd_hsel), .rdv_valid(fx_rdv_valid), .rdv_err(fx_rdv_err),
    .rdv_data(fx_rdv_data),
    .wr_valid(va_wr_valid), .wr_ready(rf_wr_ready[0]), .wr_slot(va_wr_slot), .wr_reg(va_wr_reg),
    .wr_beat(va_wr_beat), .wr_lanes(va_wr_lanes), .wr_hsel(va_wr_hsel), .wr_data(va_wr_data),
    .ur_valid(va_ur_valid), .ur_slot(va_ur_slot), .ur_reg(va_ur_reg), .urd_valid(fx_urd_valid),
    .urd_data(fx_urd_data),
    .uw_valid(va_uw_valid), .uw_slot(va_uw_slot), .uw_reg(va_uw_reg), .uw_data(va_uw_data)
  );

  // ================================================================ LSU + L1D + shared memory
  logic          ls_mmu_req_valid, ls_mmu_req_ready, ls_mmu_resp_valid, ls_mmu_resp_ready;
  logic [3:0]    ls_mmu_req_vmid;
  logic [63:0]   ls_mmu_req_va;
  logic [1:0]    ls_mmu_req_acc;
  logic          ls_mmu_req_atomic;
  logic [7:0]    ls_mmu_req_tag;
  logic          l1_mem_req_valid, l1_mem_req_ready, l1_mem_rsp_valid;
  mreq_t         l1_mem_req;
  logic [31:0]   ls_st [8];
  assign stat_l1_rd = ls_st[0];
  assign stat_l1_wr = ls_st[1];

  // uc channel of the LSU (LSU_K >= 2) -> mport
  logic            lu_req_valid, lu_req_ready, lu_rsp_valid, lu_rd_req, lu_rd_gnt, lu_free;
  mreq_t           lu_req;
  logic [MSW-1:0]  lu_req_slot, lu_rsp_slot, lu_rd_slot;
  logic [1:0]      lu_rsp_err;
  logic [LINE_BITS-1:0] lu_rd_data;

  ng_lsu_top #(.NL(NL), .SLOTW(SW), .SH_AW(SH_AW), .SH_LPB(SH_LPB), .L1_LPB(L1_LPB), .L1_SETS(L1_SETS), .L1_WAYS(L1_WAYS), .LSU_K(LSU_K),
               .MEM_OUT(MEM_OUT)) u_lsu (
    .clk(clk), .rst_n(crst_n),
    .inv_l1_ext(inv_l1_ext || wd_l1_inv), .sh_clr_en(wd_sh_clr_en), .sh_clr_row(wd_sh_clr_row),
    .iss_valid(u_iss_valid[UNIT_LSU]), .iss_ready(u_iss_ready[UNIT_LSU]),
    .iss_w0(is_insn[31:0]), .iss_w1(is_insn[63:32]), .iss_exec(q_exec), .iss_slot(is_slot),
    .iss_nregs(cfg.num_vregs[7:0]), .iss_tag(16'(is_pc)), .iss_ctx(lsu_ctx),
    .done_valid(u_done[UNIT_LSU]), .done_tag(ls_done_tag), .trap_valid(u_trap[UNIT_LSU]),
    .trap_cause(u_trap_cause[UNIT_LSU]), .trap_info(u_trap_info[UNIT_LSU]), .busy(ls_busy),
    .cnt_inc_valid(ls_cnt_inc_v), .cnt_inc_slot(ls_cnt_inc_slot), .cnt_inc_id(ls_cnt_inc_id),
    .cnt_dec_valid(ls_cnt_dec_v), .cnt_dec_slot(ls_cnt_dec_slot), .cnt_dec_id(ls_cnt_dec_id),
    .rd_valid(ls_rd_valid), .rd_ready(ls_rd_ready), .rd_slot(ls_rd_slot), .rd_reg(ls_rd_reg),
    .rd_beat(ls_rd_beat), .rd_hsel(ls_rd_hsel), .rdv_valid(rf_rdv_valid[3]), .rdv_err(rf_rdv_err[3]),
    .rdv_data(rf_rdv_data[3]),
    .wr_valid(ls_wr_valid), .wr_ready(ls_wr_ready), .wr_slot(ls_wr_slot), .wr_reg(ls_wr_reg),
    .wr_beat(ls_wr_beat), .wr_lanes(ls_wr_lanes), .wr_hsel(ls_wr_hsel), .wr_data(ls_wr_data),
    .ur_valid(ls_ur_valid), .ur_slot(ls_ur_slot), .ur_reg(ls_ur_reg), .urd_valid(rf_urd_valid[2]),
    .urd_data(rf_urd_data[2]),
    .uw_valid(ls_uw_valid), .uw_slot(ls_uw_slot), .uw_reg(ls_uw_reg), .uw_data(ls_uw_data),
    .mmu_req_valid(ls_mmu_req_valid), .mmu_req_ready(ls_mmu_req_ready), .mmu_req_vmid(ls_mmu_req_vmid),
    .mmu_req_va(ls_mmu_req_va), .mmu_req_acc(ls_mmu_req_acc), .mmu_req_atomic(ls_mmu_req_atomic),
    .mmu_req_tag(ls_mmu_req_tag), .mmu_resp_valid(ls_mmu_resp_valid), .mmu_resp_ready(ls_mmu_resp_ready),
    .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
    .mmu_resp_pte(mmu_resp_pte),
    .ext_flt_valid(ls_ef_valid), .ext_flt_vmid(ls_ef_vmid), .ext_flt_va(ls_ef_va), .ext_flt_acc(ls_ef_acc),
    .mem_req_valid(l1_mem_req_valid), .mem_req_ready(l1_mem_req_ready), .mem_req(l1_mem_req),
    .mem_rsp_valid(l1_mem_rsp_valid), .mem_rsp(fab_rsp),
    .u_req_valid(lu_req_valid), .u_req_ready(lu_req_ready), .u_req(lu_req), .u_req_slot(lu_req_slot),
    .u_rsp_valid(lu_rsp_valid), .u_rsp_slot(lu_rsp_slot), .u_rsp_err(lu_rsp_err),
    .u_rd_req(lu_rd_req), .u_rd_slot(lu_rd_slot), .u_rd_gnt(lu_rd_gnt), .u_rd_data(lu_rd_data), .u_free(lu_free),
    .stat_l1_rd(ls_st[0]), .stat_l1_wr(ls_st[1]), .stat_l1_amo(ls_st[2]), .stat_sh_rounds(ls_st[3]),
    .stat_instr_mem(stat_instr_mem), .stat_groups(stat_groups),
    .stat_k(stat_lsu), .stat_hit(ls_st[4]), .stat_miss(ls_st[5]), .stat_fill(ls_st[6]), .stat_inv(ls_st[7])
  );

  // ================================================================ counters
  logic [31:0] scr_n, scr_cyc, scr_inv, scr_cp_l1, scr_cp_i;
  assign stat_scrub = {scr_cp_i, scr_cp_l1, scr_inv, scr_cyc, scr_n};
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      scr_n <= '0; scr_cyc <= '0; scr_inv <= '0; scr_cp_l1 <= '0; scr_cp_i <= '0;
    end else begin
      if (wd_scrub_ev)   scr_n   <= scr_n + 32'd1;
      if (wd_scrub_busy) scr_cyc <= scr_cyc + 32'd1;
      if (inv_l1_ext || wd_l1_inv) scr_inv <= scr_inv + 32'd1;
      if (inv_l1_ext)    scr_cp_l1 <= scr_cp_l1 + 32'd1;
      if (inv_i_ext)     scr_cp_i  <= scr_cp_i + 32'd1;
    end
  end

  // ================================================================ trap aggregation (declared early for xtrap)
  logic          tr_v;
  logic [2:0]    tr_unit;
  logic [SW-1:0] tr_slot;
  logic [31:0]   tr_pc, tr_info;
  logic [3:0]    tr_cause;

  // ================================================================ fflags merge into the SALU
  // XLANE / FP64 (fixed group, mutually exclusive) go straight through; an
  // SFU report colliding with one of them waits one cycle in a pending slot.
  logic          ffp_v;
  logic [SW-1:0] ffp_slot;
  logic [4:0]    ffp_bits;
  logic          ffs_v;
  logic [SW-1:0] ffs_slot;
  logic [4:0]    ffs_bits;
  logic          ff_fx_v;
  always_comb begin
    ff_fx_v = ff_v[1] || ff_v[2] || ff_v[3];
    if (ff_fx_v) begin
      ffs_v = 1'b1;
      ffs_slot = ff_v[1] ? rec_slot[UNIT_XLANE] : ff_v[2] ? rec_slot[UNIT_FP64] : rec_slot[UNIT_VALU];
      ffs_bits = ff_v[1] ? ff_bits[1] : ff_v[2] ? ff_bits[2] : ff_bits[3];
    end else if (ffp_v) begin
      ffs_v = 1'b1; ffs_slot = ffp_slot; ffs_bits = ffp_bits;
    end else begin
      ffs_v = ff_v[0]; ffs_slot = rec_slot[UNIT_SFU]; ffs_bits = ff_bits[0];
    end
  end
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      ffp_v <= 1'b0; ffp_slot <= '0; ffp_bits <= '0;
    end else begin
      if (!ff_fx_v && ffp_v) ffp_v <= 1'b0;
      if (ff_v[0] && (ff_fx_v || ffp_v)) begin
        ffp_v <= 1'b1; ffp_slot <= rec_slot[UNIT_SFU]; ffp_bits <= ff_bits[0];
      end
    end
  end

  // ================================================================ SALU / branch / CSR (ng_salu)
  logic [SW-1:0] sa_cur_slot;
  logic [15:0]   sa_done_tag;
  logic          sa_ev_valid;
  logic [SW-1:0] sa_ev_slot;
  logic [2:0]    sa_ev_kind;
  logic [19:0]   sa_ev_imm;
  logic [24:0]   sa_ev_wait;
  logic [SW-1:0] sa_br_slot;
  logic          xtrap_v;
  assign xtrap_v = tr_v && !trap_v && (tr_unit != UNIT_SALU);

  ng_salu #(.SLOTW(SW)) u_salu (
    .clk(clk), .rst_n(crst_n),
    .iss_valid(u_iss_valid[UNIT_SALU]), .iss_ready(u_iss_ready[UNIT_SALU]),
    .iss_w0(is_insn[31:0]), .iss_w1(is_insn[63:32]), .iss_pc(is_pc), .iss_slot(is_slot), .iss_tag(16'(is_pc)),
    .done_valid(u_done[UNIT_SALU]), .done_tag(sa_done_tag), .trap_valid(u_trap[UNIT_SALU]),
    .trap_cause(u_trap_cause[UNIT_SALU]), .trap_info(u_trap_info[UNIT_SALU]), .busy(sa_busy),
    .br_valid(br_valid), .br_slot(sa_br_slot), .br_taken(br_taken), .br_target(br_target),
    .ev_valid(sa_ev_valid), .ev_slot(sa_ev_slot), .ev_kind(sa_ev_kind), .ev_imm(sa_ev_imm), .ev_wait(sa_ev_wait),
    .ur_valid(sa_ur_valid), .ur_ready(sa_ur_gnt), .ur_slot(sa_ur_slot), .ur_reg(sa_ur_reg),
    .urd_valid(fx_urd_valid), .urd_data(fx_urd_data),
    .uw_valid(sa_uw_valid), .uw_ready(1'b1), .uw_slot(sa_uw_slot), .uw_reg(sa_uw_reg), .uw_data(sa_uw_data),
    .winit_valid(tbl_we), .winit_slot(tbl_slot), .winit_exec(tbl_exec),
    .winit_code_lo(cfg.code_base[31:0]), .winit_code_hi({16'd0, cfg.code_base[47:32]}),
    .winit_scratch_lo(cfg.scratch_va[31:0]), .winit_scratch_hi({16'd0, cfg.scratch_va[47:32]}),
    .winit_scratch_size(cfg.scratch_bytes), .winit_wgslot({{(32 - WGW){1'b0}}, tbl_tag}),
    .sp_id(32'(SP_ID)), .core_id(32'(CORE_ID)), .cur_slot(sa_cur_slot),
    // the instruction's own commit is already counted when the SALU reads these
    .hw_clock({32'd0, retired_tbl[sa_cur_slot]} - 64'd1), .hw_time(time_ctr - 64'd1),
    .q_slot(q_slot), .q_exec(q_exec), .q_mode(q_mode), .q_fflags(q_fflags), .q_trapctl(q_trapctl),
    .qc_slot(ev_commit_slot), .qc_exec(qc_exec),
    .fflags_set_valid(ffs_v), .fflags_set_slot(ffs_slot), .fflags_set_bits(ffs_bits),
    .xtrap_valid(xtrap_v), .xtrap_slot(tr_slot), .xtrap_cause(tr_cause), .xtrap_pc(tr_pc), .xtrap_info(tr_info)
  );
  assign br_slot = sa_br_slot;
  assign sa_ur_gnt = !BR_NODRAIN || (va_ur_valid == 2'b00);

  // ================================================================ optional units
  generate
    if (EN_SFU) begin : g_sfu
      ng_sfu_pkg::dec_t rt_fd;
      ng_sfu_dec u_rt_sfu (.w0(is_insn[31:0]), .w1(is_insn[63:32]), .d(rt_fd));
      assign rt_sfu_hit = rt_fd.hit;
      logic        sf_busy;
      logic [15:0] sf_done_tag;
      ng_sfu #(.NL(NL), .SLOTW(SW)) u_sfu (
        .clk(clk), .rst_n(crst_n),
        .iss_valid(u_iss_valid[UNIT_SFU]), .iss_ready(u_iss_ready[UNIT_SFU]),
        .iss_w0(is_insn[31:0]), .iss_w1(is_insn[63:32]), .iss_exec(q_exec), .iss_slot(is_slot),
        .iss_nregs(cfg.num_vregs[7:0]), .iss_mode(q_mode), .iss_tag(16'(is_pc)),
        .done_valid(u_done[UNIT_SFU]), .done_tag(sf_done_tag), .trap_valid(u_trap[UNIT_SFU]),
        .trap_cause(u_trap_cause[UNIT_SFU]), .trap_info(u_trap_info[UNIT_SFU]),
        .fflags_valid(ff_v[0]), .fflags_set(ff_bits[0]), .busy(sf_busy),
        .rd_valid(sf_rd_valid), .rd_ready(sf_rd_ready), .rd_slot(sf_rd_slot), .rd_reg(sf_rd_reg),
        .rd_beat(sf_rd_beat), .rd_hsel(sf_rd_hsel), .rdv_valid(rf_rdv_valid[4]), .rdv_err(rf_rdv_err[4]),
        .rdv_data(rf_rdv_data[4]),
        .wr_valid(sf_wr_valid), .wr_ready(sf_wr_ready), .wr_slot(sf_wr_slot), .wr_reg(sf_wr_reg),
        .wr_beat(sf_wr_beat), .wr_lanes(sf_wr_lanes), .wr_hsel(sf_wr_hsel), .wr_data(sf_wr_data)
      );
    end else begin : g_no_sfu
      assign rt_sfu_hit = 1'b0;
      assign u_iss_ready[UNIT_SFU] = 1'b0;
      assign u_done[UNIT_SFU] = 1'b0;
      assign u_trap[UNIT_SFU] = 1'b0;
      assign u_trap_cause[UNIT_SFU] = '0;
      assign u_trap_info[UNIT_SFU] = '0;
      assign sf_rd_valid = 1'b0; assign sf_rd_slot = '0; assign sf_rd_reg = '0; assign sf_rd_beat = '0;
      assign sf_rd_hsel = '0;
      assign sf_wr_valid = 1'b0; assign sf_wr_slot = '0; assign sf_wr_reg = '0; assign sf_wr_beat = '0;
      assign sf_wr_lanes = '0; assign sf_wr_hsel = '0; assign sf_wr_data = '0;
      assign ff_v[0] = 1'b0; assign ff_bits[0] = '0;
    end

    if (EN_XLANE) begin : g_xlane
      ng_xlane_pkg::dec_t rt_xd;
      ng_xlane_dec u_rt_xlane (.w0(is_insn[31:0]), .d(rt_xd));
      assign rt_xl_hit = rt_xd.hit && (is_insn[0] == 1'b0);   // xlane encodings are 32-bit
      logic        xl_busy;
      logic [4:0]  xl_ff;
      logic [15:0] xl_done_tag;
      ng_xlane #(.NL(NL), .SLOTW(SW)) u_xlane (
        .clk(clk), .rst_n(crst_n),
        .iss_valid(u_iss_valid[UNIT_XLANE]), .iss_ready(u_iss_ready[UNIT_XLANE]),
        .iss_w0(is_insn[31:0]), .iss_exec(q_exec), .iss_mode(q_mode),
        .iss_slot(is_slot), .iss_nregs(cfg.num_vregs[7:0]), .iss_tag(16'(is_pc)),
        .done_valid(u_done[UNIT_XLANE]), .done_tag(xl_done_tag), .trap_valid(u_trap[UNIT_XLANE]),
        .trap_cause(u_trap_cause[UNIT_XLANE]), .trap_info(u_trap_info[UNIT_XLANE]),
        .fflags_o(xl_ff), .busy(xl_busy),
        .rd_valid(xl_rd_valid), .rd_ready(rf_rd_ready[2:0]), .rd_slot(xl_rd_slot), .rd_reg(xl_rd_reg),
        .rd_beat(xl_rd_beat), .rd_hsel(xl_rd_hsel), .rdv_valid(fx_rdv_valid), .rdv_err(fx_rdv_err),
        .rdv_data(fx_rdv_data),
        .wr_valid(xl_wr_valid), .wr_ready(rf_wr_ready[0]), .wr_slot(xl_wr_slot), .wr_reg(xl_wr_reg),
        .wr_beat(xl_wr_beat), .wr_lanes(xl_wr_lanes), .wr_hsel(xl_wr_hsel), .wr_data(xl_wr_data),
        .ur_valid(xl_ur_valid), .ur_slot(xl_ur_slot), .ur_reg(xl_ur_reg), .urd_valid(fx_urd_valid),
        .urd_data(fx_urd_data),
        .uw_valid(xl_uw_valid), .uw_slot(xl_uw_slot), .uw_reg(xl_uw_reg), .uw_data(xl_uw_data)
      );
      assign ff_v[1] = (u_done[UNIT_XLANE] || u_trap[UNIT_XLANE]) && (xl_ff != 5'd0);
      assign ff_bits[1] = xl_ff;
    end else begin : g_no_xlane
      assign rt_xl_hit = 1'b0;
      assign u_iss_ready[UNIT_XLANE] = 1'b0;
      assign u_done[UNIT_XLANE] = 1'b0;
      assign u_trap[UNIT_XLANE] = 1'b0;
      assign u_trap_cause[UNIT_XLANE] = '0;
      assign u_trap_info[UNIT_XLANE] = '0;
      assign xl_rd_valid = '0;
      for (genvar k = 0; k < 3; k++) begin : g_z
        assign xl_rd_slot[k] = '0; assign xl_rd_reg[k] = '0; assign xl_rd_beat[k] = '0; assign xl_rd_hsel[k] = '0;
      end
      assign xl_wr_valid = 1'b0; assign xl_wr_slot = '0; assign xl_wr_reg = '0; assign xl_wr_beat = '0;
      assign xl_wr_lanes = '0; assign xl_wr_hsel = '0; assign xl_wr_data = '0;
      assign xl_ur_valid = '0;
      assign xl_ur_slot[0] = '0; assign xl_ur_slot[1] = '0; assign xl_ur_reg[0] = '0; assign xl_ur_reg[1] = '0;
      assign xl_uw_valid = 1'b0; assign xl_uw_slot = '0; assign xl_uw_reg = '0; assign xl_uw_data = '0;
      assign ff_v[1] = 1'b0; assign ff_bits[1] = '0;
    end

    if (EN_FP64) begin : g_fp64
      ng_fp64_pkg::dec_t rt_pd;
      ng_fp64_dec u_rt_fp64 (.w0(is_insn[31:0]), .w1(is_insn[63:32]), .d(rt_pd));
      assign rt_fp_hit = rt_pd.hit;
      logic        fp_busy;
      logic [15:0] fp_done_tag;
      ng_fp64 #(.NL(NL), .SLOTW(SW)) u_fp64 (
        .clk(clk), .rst_n(crst_n),
        .iss_valid(u_iss_valid[UNIT_FP64]), .iss_ready(u_iss_ready[UNIT_FP64]),
        .iss_w0(is_insn[31:0]), .iss_w1(is_insn[63:32]), .iss_exec(q_exec), .iss_slot(is_slot),
        .iss_nregs(cfg.num_vregs[7:0]), .iss_mode(q_mode), .iss_tag(16'(is_pc)),
        .done_valid(u_done[UNIT_FP64]), .done_tag(fp_done_tag), .trap_valid(u_trap[UNIT_FP64]),
        .trap_cause(u_trap_cause[UNIT_FP64]), .trap_info(u_trap_info[UNIT_FP64]),
        .fflags_valid(ff_v[2]), .fflags_set(ff_bits[2]), .busy(fp_busy),
        .rd_valid(fp_rd_valid), .rd_ready(rf_rd_ready[0]), .rd_slot(fp_rd_slot), .rd_reg(fp_rd_reg),
        .rd_beat(fp_rd_beat), .rd_hsel(fp_rd_hsel), .rdv_valid(rf_rdv_valid[0]), .rdv_err(rf_rdv_err[0]),
        .rdv_data(rf_rdv_data[0]),
        .wr_valid(fp_wr_valid), .wr_ready(rf_wr_ready[0]), .wr_slot(fp_wr_slot), .wr_reg(fp_wr_reg),
        .wr_beat(fp_wr_beat), .wr_lanes(fp_wr_lanes), .wr_hsel(fp_wr_hsel), .wr_data(fp_wr_data),
        .uw_valid(fp_uw_valid), .uw_slot(fp_uw_slot), .uw_reg(fp_uw_reg), .uw_data(fp_uw_data)
      );
    end else begin : g_no_fp64
      assign rt_fp_hit = 1'b0;
      assign u_iss_ready[UNIT_FP64] = 1'b0;
      assign u_done[UNIT_FP64] = 1'b0;
      assign u_trap[UNIT_FP64] = 1'b0;
      assign u_trap_cause[UNIT_FP64] = '0;
      assign u_trap_info[UNIT_FP64] = '0;
      assign fp_rd_valid = 1'b0; assign fp_rd_slot = '0; assign fp_rd_reg = '0; assign fp_rd_beat = '0;
      assign fp_rd_hsel = '0;
      assign fp_wr_valid = 1'b0; assign fp_wr_slot = '0; assign fp_wr_reg = '0; assign fp_wr_beat = '0;
      assign fp_wr_lanes = '0; assign fp_wr_hsel = '0; assign fp_wr_data = '0;
      assign fp_uw_valid = 1'b0; assign fp_uw_slot = '0; assign fp_uw_reg = '0; assign fp_uw_data = '0;
      assign ff_v[2] = 1'b0; assign ff_bits[2] = '0;
    end
  endgenerate

  // ================================================================ waitcnt accounting
  // inc: LSU at accept (its own cnt_inc), SFU at accept (counter sfu); never
  // both in one cycle (single issue).  dec: LSU / SFU at completion, staged in
  // one pending register per source (a source produces at most one dec per
  // instruction) and delivered one per cycle, LSU first.
  logic          sfu_inc;
  assign sfu_inc      = u_iss_valid[UNIT_SFU] && u_iss_ready[UNIT_SFU];
  assign cnt_inc_v    = ls_cnt_inc_v || sfu_inc;
  assign cnt_inc_slot = ls_cnt_inc_v ? ls_cnt_inc_slot : is_slot;
  assign cnt_inc_id   = ls_cnt_inc_v ? ls_cnt_inc_id : 3'(CNT_SFU);

  logic          dq_lv, dq_sv;
  logic [SW-1:0] dq_ls, dq_ss;
  logic [2:0]    dq_li;
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      dq_lv <= 1'b0; dq_sv <= 1'b0; dq_ls <= '0; dq_ss <= '0; dq_li <= '0;
    end else begin
      if (dq_lv) dq_lv <= 1'b0;
      else if (dq_sv) dq_sv <= 1'b0;
      if (ls_cnt_dec_v) begin dq_lv <= 1'b1; dq_ls <= ls_cnt_dec_slot; dq_li <= ls_cnt_dec_id; end
      if (u_done[UNIT_SFU] || u_trap[UNIT_SFU]) begin dq_sv <= 1'b1; dq_ss <= rec_slot[UNIT_SFU]; end
    end
  end
  assign cnt_dec_v    = dq_lv || dq_sv;
  assign cnt_dec_slot = dq_lv ? dq_ls : dq_ss;
  assign cnt_dec_id   = dq_lv ? dq_li : 3'(CNT_SFU);

  // ================================================================ trap aggregation
  // First trap wins; same-cycle priority: scheduler, then unit index order.
  always_comb begin
    tr_v = 1'b0; tr_unit = UNIT_SCHED; tr_slot = '0; tr_pc = '0; tr_info = '0; tr_cause = '0;
    for (int u = NU - 1; u >= 1; u--)
      if (u_trap[u]) begin
        tr_v = 1'b1; tr_unit = 3'(u); tr_slot = rec_slot[u]; tr_pc = rec_pc[u];
        tr_cause = u_trap_cause[u]; tr_info = u_trap_info[u];
      end
    if (ev_trap_v) begin
      tr_v = 1'b1; tr_unit = UNIT_SCHED; tr_slot = ev_trap_slot; tr_pc = ev_trap_pc; tr_cause = ev_trap_cause;
      tr_info = (ev_trap_cause == CAUSE_MEMFAULT) ? last_finfo : ev_trap_info;
    end
  end
  assign any_trap_pulse = tr_v;

  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n)                trap_v <= 1'b0;
    else if (tr_v && !trap_v)   trap_v <= 1'b1;
  end
  // diagnostic copy of the first trap: survives the post-fault flush (the CP /
  // DFX read it after wd_done), cleared by core_reset and at the next wd_start
  logic dbg_tv;
  always_ff @(posedge clk or negedge wrst_n) begin
    if (!wrst_n) begin
      dbg_tv <= 1'b0;
      dbg_trap_unit <= '0; dbg_trap_slot <= '0; dbg_trap_pc <= '0; dbg_trap_cause <= '0;
      dbg_trap_info <= '0; dbg_trap_wg <= '0; dbg_trap_warp <= '0;
    end else if (wd_start) begin
      dbg_tv <= 1'b0;
    end else if (tr_v && !trap_v) begin
      dbg_tv         <= 1'b1;
      dbg_trap_unit  <= tr_unit;
      dbg_trap_slot  <= tr_slot;
      dbg_trap_pc    <= tr_pc;
      dbg_trap_cause <= tr_cause;
      dbg_trap_info  <= tr_info;
      dbg_trap_wg    <= wglin_tbl[tr_slot];
      dbg_trap_warp  <= widx_tbl[tr_slot];
    end
  end
  assign dbg_trap_valid = dbg_tv;

  // ================================================================ status
  always_comb begin
    for (int s = 0; s < NW; s++) begin
      slot_busy[s] = (dq_lv && dq_ls == SW'(s)) || (dq_sv && dq_ss == SW'(s)) || (ffp_v && ffp_slot == SW'(s));
      for (int j = 1; j < VRD; j++)
        if (j < int'(vr_cnt) && vr_slot[j] == SW'(s)) slot_busy[s] = 1'b1;
      if (LSU_K > 1)
        for (int j = 1; j < LRD; j++)
          if (j < int'(lr_cnt) && lr_slot[j] == SW'(s)) slot_busy[s] = 1'b1;
      for (int u = 1; u < NU; u++)
        if (rec_v[u] && rec_slot[u] == SW'(s)) slot_busy[s] = 1'b1;
    end
  end
  assign units_busy = (|rec_v) || sc_ev_busy || ic_busy || dq_lv || dq_sv || ffp_v || ls_busy || va_busy || sa_busy;
  assign sp_idle    = !wd_busy && (occ == '0) && !units_busy;

  // ================================================================ performance counters (debug page 0x940..0x95F)
  // Datapath reset (crst_n) like stat_ic_* / the LSU counters: cleared by core_reset and the post-fault flush;
  // free-running 32-bit, the host subtracts snapshots.  See b2.6.
  logic [31:0] pc_cnt [8];
  logic [7:0]  pc_ev;
  always_comb begin
    pc_ev[0] = (occ != '0);                                                    // SP_ACTIVE
    pc_ev[1] = va_bbusy;                                                       // VALU_BACK_BUSY
    pc_ev[2] = is_valid && is_brcls && !is_ready && !halt;                     // BR_WAIT_DRAIN
    pc_ev[3] = (occ != '0) && !va_bbusy && !sc_ib_busy;                        // FRONT_STARVED
    pc_ev[4] = va_push;                                                        // VALU_ISSUED
    pc_ev[5] = u_iss_valid[UNIT_SALU] && u_iss_ready[UNIT_SALU] && is_brcls;   // BR_ISSUED
    pc_ev[6] = sc_ib_busy && !is_valid;                                        // IB_WAIT_OWN
    pc_ev[7] = rec_v[UNIT_SALU];                                               // SALU_BUSY
  end
  always_ff @(posedge clk or negedge crst_n) begin
    if (!crst_n) begin
      for (int i = 0; i < 8; i++) pc_cnt[i] <= '0;
    end else begin
      for (int i = 0; i < 8; i++) if (pc_ev[i]) pc_cnt[i] <= pc_cnt[i] + 32'd1;
    end
  end
  always_comb for (int i = 0; i < 8; i++) stat_perf[32*i +: 32] = pc_cnt[i];

  // instructions retired by the running dispatch (WDIST_INSTR): the scheduler commits at issue, an
  // instruction that then traps in its unit (u_trap) is taken back out, like the model (trapping insn not counted)
  logic [2:0] ntrap;
  always_comb begin
    ntrap = '0;
    for (int u = 1; u < NU; u++) ntrap = ntrap + 3'(u_trap[u]);
  end
  always_ff @(posedge clk or negedge wrst_n) begin
    if (!wrst_n)          wd_instr <= '0;
    else if (wd_start)    wd_instr <= '0;
    else                  wd_instr <= wd_instr + 64'(ev_commit_v) - 64'(ntrap);
  end

  // ================================================================ commit trace
  // only plain pipe instructions use the EXEC latched at their accept; branch commits (resolved later,
  // other warps may have been accepted meanwhile) and scheduler-local ones read the warp's EXEC at commit
  // time (a branch does not change EXEC; the warp has no other instruction in flight)
  // (arch_seq): jal / jr / jalr are CLS_OTHER for the trace but SALU-resolved (parked in BRWAIT, committed when
  // br_* resolves, as for the pure branches), so by then iss_exec_q may belong to another warp's later accept:
  // they read the warp's EXEC at commit time like the pure branches (found by the random-stream comparison: the exec
  // of a jal / jr / jalr commit differed from the model whenever warps of different EXEC interleaved)
  wire ev_commit_jlr    = (ev_commit_w0[1:0] != 2'b11) && (ev_commit_w0[6:0] == 7'h48 || ev_commit_w0[6:0] == 7'h4c);
  wire commit_from_pipe = (ev_commit_class == CLS_OTHER) && !ev_commit_jlr;
  assign dbg_commit_valid = ev_commit_v;
  assign dbg_commit_slot  = ev_commit_slot;
  assign dbg_commit_pc    = ev_commit_pc;
  assign dbg_commit_w0    = ev_commit_w0;
  assign dbg_commit_exec  = commit_from_pipe ? iss_exec_q : qc_exec;
  assign dbg_commit_wg    = wglin_tbl[ev_commit_slot];
  assign dbg_commit_warp  = widx_tbl[ev_commit_slot];

  // ================================================================ MMU arbitration (icache > LSU)
  // One translation in flight; the request is registered so it stays stable
  // even if its owner is reset meanwhile (the response is then dropped).
  typedef enum logic [1:0] {A_IDLE, A_SEND, A_WAIT} arb_t;
  arb_t        ma_st;
  logic        ma_own;      // 1 = icache
  logic        ma_drop;
  logic [3:0]  ma_vmid;
  logic [63:0] ma_va;
  logic [1:0]  ma_acc;
  logic        ma_atomic;
  logic [7:0]  ma_tag;
  assign ic_mmu_req_ready  = (ma_st == A_IDLE);
  assign ls_mmu_req_ready  = (ma_st == A_IDLE) && !ic_mmu_req_valid;
  assign mmu_req_valid     = (ma_st == A_SEND);
  assign mmu_req_vmid      = ma_vmid;
  assign mmu_req_va        = ma_va;
  assign mmu_req_acc       = ma_acc;
  assign mmu_req_atomic    = ma_atomic;
  assign mmu_req_tag       = ma_tag;
  assign ic_mmu_resp_valid = (ma_st == A_WAIT) && !ma_drop && ma_own && mmu_resp_valid;
  assign ls_mmu_resp_valid = (ma_st == A_WAIT) && !ma_drop && !ma_own && mmu_resp_valid;
  assign mmu_resp_ready    = (ma_st == A_WAIT) && (ma_drop || (ma_own ? ic_mmu_resp_ready : ls_mmu_resp_ready));
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ma_st <= A_IDLE; ma_own <= 1'b0; ma_drop <= 1'b0;
      ma_vmid <= '0; ma_va <= '0; ma_acc <= '0; ma_atomic <= 1'b0; ma_tag <= '0;
    end else begin
      case (ma_st)
        A_IDLE: begin
          ma_drop <= 1'b0;
          if (ic_mmu_req_valid) begin
            ma_st <= A_SEND; ma_own <= 1'b1;
            ma_vmid <= ic_mmu_req_vmid; ma_va <= ic_mmu_req_va; ma_acc <= ic_mmu_req_acc;
            ma_atomic <= ic_mmu_req_atomic; ma_tag <= ic_mmu_req_tag;
          end else if (ls_mmu_req_valid) begin
            ma_st <= A_SEND; ma_own <= 1'b0;
            ma_vmid <= ls_mmu_req_vmid; ma_va <= ls_mmu_req_va; ma_acc <= ls_mmu_req_acc;
            ma_atomic <= ls_mmu_req_atomic; ma_tag <= ls_mmu_req_tag;
          end
        end
        A_SEND: if (mmu_req_ready) ma_st <= A_WAIT;
        A_WAIT: if (mmu_resp_valid && mmu_resp_ready) ma_st <= A_IDLE;
        default: ma_st <= A_IDLE;
      endcase
      if (!crst_n && ma_st != A_IDLE) ma_drop <= 1'b1;
    end
  end

  // ================================================================ fabric arbitration
  // LSU_K = 1 and MEM_OUT = 1: the pre-26c3 arbiter (icache > L1D, one request in flight; the request is registered
  // and the answer of a request in flight across a datapath reset is dropped).  Otherwise the mport.
  localparam bit MPORT = (LSU_K > 1) || (MEM_OUT > 1);
  logic [1:0] fa_dbg;
  if (!MPORT) begin : g_fa
    arb_t   fa_st;
    logic   fa_own;
    logic   fa_drop;
    mreq_t  fa_req;
    assign ic_mem_req_ready = (fa_st == A_IDLE);
    assign l1_mem_req_ready = (fa_st == A_IDLE) && !ic_mem_req_valid;
    assign mem_req_valid    = (fa_st == A_SEND);
    assign mem_req          = fa_req;
    assign ic_mem_rsp_valid = (fa_st == A_WAIT) && !fa_drop && fa_own && mem_rsp_valid;
    assign l1_mem_rsp_valid = (fa_st == A_WAIT) && !fa_drop && !fa_own && mem_rsp_valid;
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        fa_st <= A_IDLE; fa_own <= 1'b0; fa_drop <= 1'b0; fa_req <= '0;
      end else begin
        case (fa_st)
          A_IDLE: begin
            fa_drop <= 1'b0;
            if (ic_mem_req_valid) begin
              fa_st <= A_SEND; fa_own <= 1'b1; fa_req <= ic_mem_req;
            end else if (l1_mem_req_valid) begin
              fa_st <= A_SEND; fa_own <= 1'b0; fa_req <= l1_mem_req;
            end
          end
          A_SEND: if (mem_req_ready) fa_st <= A_WAIT;
          A_WAIT: if (mem_rsp_valid) fa_st <= A_IDLE;
          default: fa_st <= A_IDLE;
        endcase
        if (!crst_n && fa_st != A_IDLE) fa_drop <= 1'b1;
      end
    end
    assign fab_rsp = mem_rsp;
    assign mem_rd_slot = '0;
    assign mem_free = 1'b0;
    assign lu_req_ready = 1'b0; assign lu_req_slot = '0; assign lu_rsp_valid = 1'b0; assign lu_rsp_slot = '0;
    assign lu_rsp_err = '0; assign lu_rd_gnt = 1'b0; assign lu_rd_data = '0;
    assign fa_dbg = fa_st;
  end else begin : g_mp
    // ---- mport: requesters I$ > L1D > LSU uc channel onto the slot protocol.
    // * Lock: once a requester is presented it stays presented until accepted (a write is staged into the fabric's
    //   wbuf over several clocks; the presented request must not change).
    // * Owner table (rst_n domain): own_v / own_w per slot, set at the accept, cleared at the answer and by the datapath
    //   reset (!crst_n).  An answer of a slot that is not owned (requested before a reset) or arriving while crst_n is
    //   low is dropped; a dropped read answer is freed at once (mem_free).  This does not rely on the fabric's own
    //   orphan handling (ng_rsp_fab_ts sp_rst); with it (rm_sp) such answers never arrive.
    // * Read port (mem_rd_slot / mem_rd_data / mem_free): an I$ / L1D / dropped answer uses it in its pulse clock
    //   (the line is taken in that clock), then the reset clean-up (lw: LSU read slots answered but not yet freed --
    //   freed one per clock while crst_n is low), otherwise the LSU back end (lu_rd_gnt).
    logic           p_req_valid, p_req_ready, p_rsp_valid, p_rsp_rd, p_free;
    mreq_t          p_req;
    logic [MSW-1:0] p_req_slot, p_rsp_slot, p_rd_slot;
    logic [1:0]     p_rsp_err;
    logic [63:0]    p_rsp_old;
    logic [LINE_BITS-1:0] p_rd_data;
    if (MEM_OUT > 1) begin : g_slot
      assign mem_req_valid = p_req_valid;
      assign p_req_ready   = mem_req_ready;
      assign mem_req       = p_req;
      assign p_req_slot    = mem_req_slot;
      assign p_rsp_valid   = mem_rsp_valid;
      assign p_rsp_slot    = mem_rsp_slot;
      assign p_rsp_err     = mem_rsp.err;
      assign p_rsp_old     = mem_rsp.old;
      assign p_rsp_rd      = mem_rsp_rd;
      assign mem_rd_slot   = p_rd_slot;
      assign p_rd_data     = mem_rd_data;
      assign mem_free      = p_free;
    end else begin : g_ser
      // 1-slot adapter (LSU_K >= 2 on the serial port): one request in flight; the answer is registered (the line
      // in sa_rsp) and delivered one clock later as slot 0; a read slot stays busy until it is freed
      logic  sa_busy, sa_rd, sa_rv, sa_rrd;
      mrsp_t sa_rsp;
      assign mem_req_valid = p_req_valid && !sa_busy;
      assign mem_req       = p_req;
      assign p_req_ready   = mem_req_ready && !sa_busy;
      assign p_req_slot    = '0;
      assign p_rsp_valid   = sa_rv;
      assign p_rsp_slot    = '0;
      assign p_rsp_err     = sa_rsp.err;
      assign p_rsp_old     = sa_rsp.old;
      assign p_rsp_rd      = sa_rrd;
      assign p_rd_data     = sa_rsp.rdata;
      assign mem_rd_slot   = '0;
      assign mem_free      = 1'b0;
      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          sa_busy <= 1'b0; sa_rd <= 1'b0; sa_rv <= 1'b0; sa_rrd <= 1'b0; sa_rsp <= '0;
        end else begin
          sa_rv <= 1'b0;
          if (p_req_valid && p_req_ready) begin sa_busy <= 1'b1; sa_rd <= (p_req.kind == ng_lsu_pkg::M_RD); end
          if (mem_rsp_valid && sa_busy) begin
            sa_rsp <= mem_rsp; sa_rv <= 1'b1; sa_rrd <= sa_rd;
            if (!sa_rd) sa_busy <= 1'b0;
          end
          if (p_free) sa_busy <= 1'b0;
        end
      end
    end

    localparam int NSL = 1 << MSW;
    logic           mp_held;
    logic [1:0]     mp_hsel, mp_sel;     // 0 I$, 1 L1D, 2 LSU uc channel
    logic [NSL-1:0] own_v, lw;
    logic [1:0]     own_w [NSL];
    always_comb begin
      if (mp_held)               mp_sel = mp_hsel;
      else if (ic_mem_req_valid) mp_sel = 2'd0;
      else if (l1_mem_req_valid) mp_sel = 2'd1;
      else                       mp_sel = 2'd2;
    end
    assign p_req_valid      = (mp_sel == 2'd0) ? ic_mem_req_valid : (mp_sel == 2'd1) ? l1_mem_req_valid : lu_req_valid;
    assign p_req            = (mp_sel == 2'd0) ? ic_mem_req : (mp_sel == 2'd1) ? l1_mem_req : lu_req;
    assign ic_mem_req_ready = (mp_sel == 2'd0) && p_req_ready;
    assign l1_mem_req_ready = (mp_sel == 2'd1) && p_req_ready;
    assign lu_req_ready     = (mp_sel == 2'd2) && p_req_ready;
    assign lu_req_slot      = p_req_slot;

    wire       r_own  = own_v[p_rsp_slot] && crst_n;
    wire [1:0] r_w    = own_w[p_rsp_slot];
    wire       r_uc   = p_rsp_valid && r_own && (r_w == 2'd2);
    wire       rp_ans = p_rsp_valid && !r_uc;          // I$ / L1D / dropped answer: read port in its pulse clock
    assign ic_mem_rsp_valid = p_rsp_valid && r_own && (r_w == 2'd0);
    assign l1_mem_rsp_valid = p_rsp_valid && r_own && (r_w == 2'd1);
    assign lu_rsp_valid     = r_uc;
    assign lu_rsp_slot      = p_rsp_slot;
    assign lu_rsp_err       = p_rsp_err;
    always_comb begin
      fab_rsp       = '0;
      fab_rsp.rdata = p_rsp_rd ? p_rd_data : '0;
      fab_rsp.old   = p_rsp_old;
      fab_rsp.err   = p_rsp_err;
    end
    wire           lw_any = |lw;
    wire [MSW-1:0] lw_lo  = MSW'(ng_bitops_pkg::lowest_set(64'(lw)));
    wire rp_cln = !crst_n && lw_any && !rp_ans;
    assign lu_rd_gnt  = crst_n && !rp_ans;
    assign lu_rd_data = p_rd_data;
    assign p_rd_slot  = rp_ans ? p_rsp_slot : (rp_cln ? lw_lo : lu_rd_slot);
    assign p_free     = rp_ans ? p_rsp_rd : (rp_cln || (lu_free && lu_rd_gnt));

    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        mp_held <= 1'b0; mp_hsel <= '0; own_v <= '0; lw <= '0;
        for (int i = 0; i < NSL; i++) own_w[i] <= '0;
      end else begin
        mp_held <= p_req_valid && !p_req_ready && crst_n;
        mp_hsel <= mp_sel;
        if (p_rsp_valid) begin
          own_v[p_rsp_slot] <= 1'b0;
          if (r_uc && p_rsp_rd) lw[p_rsp_slot] <= 1'b1;
        end
        if (p_free && !rp_ans) lw[p_rd_slot] <= 1'b0;
        if (p_req_valid && p_req_ready) begin
          own_v[p_req_slot] <= 1'b1; own_w[p_req_slot] <= mp_sel; lw[p_req_slot] <= 1'b0;
        end
        if (!crst_n) own_v <= '0;
      end
    end
    assign fa_dbg = {mp_held, |own_v};
`ifndef SYNTHESIS
    always @(posedge clk) if (rst_n) begin
      if (p_req_valid && p_req_ready && own_v[p_req_slot])
        $error("ng_sp_top mport: accept into an owned slot %0d", p_req_slot);
      if (lu_free && lu_rd_gnt && !lw[lu_rd_slot])
        $error("ng_sp_top mport: LSU frees slot %0d that holds no answered LSU read", lu_rd_slot);
    end
`endif
  end

  // unused
  assign dbg_status = {!crst_n, sp_idle, stalled_all, ma_st, fa_dbg, ls_busy, ic_busy, trap_v, units_busy, wd_busy,
                       wd_dbg_st, occ};

  logic unused;
  assign unused = ^{va_done_tag, ls_done_tag, sa_done_tag, stalled_all, rf_alloc_err, rf_perf_conflict,
                    rf_wr_err, ls_st[0], sa_ev_valid, sa_ev_slot, sa_ev_kind, sa_ev_imm, sa_ev_wait,
                    q_fflags, q_trapctl, br_slot};

`ifdef NG_SP_ASSERT
  always @(posedge clk) if (crst_n) begin
    if ($countones({va_rd_valid != 3'd0, xl_rd_valid != 3'd0, fp_rd_valid}) > 1)
      $error("ng_sp_top: two fixed-group units drive register-file read ports");
    if (rf_alloc_err) $error("ng_sp_top: register-file allocation error");
    if (ff_v[0] && ffp_v && (ff_v[1] || ff_v[2] || ff_v[3])) $error("ng_sp_top: SFU fflags pending overflow");
  end
`endif
`ifndef SYNTHESIS
  // ---- simulation-only invariants (INV-FX and per-warp in-flight) -----
  // INV-FX: at most one of {VALU, SALU, XLANE, FP64} is in flight / drives the shared
  // fixed-group register-file and uniform ports (they are OR-ed, not arbitrated).
  // INV-WARP: a warp has at most one instruction in flight across all units, and every
  // unit record's warp has its scheduler in-flight bit set.
  int unsigned sim_inv_err = 0;   // TB reads dut.sim_inv_err (must stay 0)
  int unsigned sim_acc_fx  = 0;   // LSU/SFU instructions accepted while a fixed-group op was in flight (new in)
  int unsigned sim_ovl_cyc = 0;   // cycles with a fixed-group op and an LSU/SFU op of another warp both in flight
  int unsigned sim_valu_busy = 0;  // cycles with a VALU instruction in flight
  int unsigned sim_valu2     = 0;  // cycles with two VALU instructions in flight (split VALU)
  int unsigned sim_valu3     = 0;  // cycles with three or more VALU instructions in flight
  int unsigned sim_br_ovl    = 0;  // cycles with a pure branch in the SALU and a VALU instruction in flight
  int unsigned sim_br_acc_v  = 0;  // pure branches accepted while the VALU was busy
  int unsigned sim_va_acc_br = 0;  // VALU instructions accepted while a pure branch was in the SALU
  int unsigned sim_ur_hold   = 0;  // cycles the SALU's uniform read waited for the VALU's
  int unsigned sim_lsu_max   = 0;  // largest number of LSU instructions in flight (LSU_K >= 2)
  always @(posedge clk) if (crst_n) begin
    if (rec_v[UNIT_SALU] && rec_v[UNIT_VALU]) sim_br_ovl <= sim_br_ovl + 1;
    if (u_iss_valid[UNIT_SALU] && u_iss_ready[UNIT_SALU] && rec_v[UNIT_VALU]) sim_br_acc_v <= sim_br_acc_v + 1;
    if (va_push && rec_v[UNIT_SALU]) sim_va_acc_br <= sim_va_acc_br + 1;
    if ((sa_ur_valid != 2'b00) && !sa_ur_gnt) sim_ur_hold <= sim_ur_hold + 1;
    // INV-FX: SALU / XLANE / FP64 pairwise exclusive; the VALU only overlaps a pure branch in the SALU
    if ($countones({rec_v[UNIT_SALU], rec_v[UNIT_XLANE], rec_v[UNIT_FP64]}) > 1)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: two of SALU/XLANE/FP64 in flight"); end
    if (rec_v[UNIT_VALU] && rec_v[UNIT_SALU] && !(BR_NODRAIN && sa_br_q))
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: VALU in flight beside a SALU instruction that is not a pure branch"); end
    if (rec_v[UNIT_VALU] && rec_v[UNIT_SALU] && (sa_uw_valid != 2'b00))
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: SALU writes a uniform register beside the VALU"); end
    if (u_iss_valid[UNIT_SALU] && u_iss_ready[UNIT_SALU] && rec_v[UNIT_VALU] && !(BR_NODRAIN && is_brcls))
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: SALU accepted a non-branch (or BR_NODRAIN=0) under a busy VALU"); end
    if (rec_v[UNIT_VALU]) sim_valu_busy <= sim_valu_busy + 1;
    if ((rec_v[UNIT_VALU] || rec_v[UNIT_SALU] || rec_v[UNIT_XLANE] || rec_v[UNIT_FP64]) &&
        (rec_v[UNIT_LSU] || rec_v[UNIT_SFU]))
      sim_ovl_cyc <= sim_ovl_cyc + 1;
    if (fixed_busy && ((u_iss_valid[UNIT_LSU] && u_iss_ready[UNIT_LSU]) || (u_iss_valid[UNIT_SFU] && u_iss_ready[UNIT_SFU])))
      sim_acc_fx <= sim_acc_fx + 1;
    if ($countones({rec_v[UNIT_VALU], rec_v[UNIT_SALU] && !(BR_NODRAIN && sa_br_q), rec_v[UNIT_XLANE], rec_v[UNIT_FP64]}) > 1)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: two fixed-group units in flight"); end
    if ($countones({va_wr_valid, xl_wr_valid, fp_wr_valid}) > 1)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: two fixed-group units drive regfile write port 0"); end
    if ($countones({va_uw_valid, xl_uw_valid, fp_uw_valid, sa_uw_valid[0]}) > 1)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: two fixed-group units drive uniform write port 0"); end
    for (int k = 0; k < 2; k++)
      if ($countones({va_ur_valid[k], xl_ur_valid[k], sa_ur_valid[k] && sa_ur_gnt}) > 1)
        begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: two fixed-group units drive uniform read port"); end
    if ($countones({|va_rd_valid, |xl_rd_valid, fp_rd_valid}) > 1)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: two fixed-group units drive regfile read ports"); end
    if (int'(vr_cnt) >= 2) sim_valu2 <= sim_valu2 + 1;
    if (int'(vr_cnt) >= 3) sim_valu3 <= sim_valu3 + 1;
    for (int j = 1; j < VRD; j++) if (j < int'(vr_cnt)) begin
      if (!infl_q[vr_slot[j]])
        begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: VALU record %0d without scheduler in-flight bit", j); end
      for (int i = 0; i < j; i++)
        if (vr_slot[i] == vr_slot[j])
          begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: VALU records %0d and %0d hold the same warp", i, j); end
      for (int u = 2; u < NU; u++)
        if (rec_v[u] && rec_slot[u] == vr_slot[j])
          begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: unit %0d and VALU record %0d hold the same warp", u, j); end
    end
    if (va_push && !va_pop && int'(vr_cnt) == VRD)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top: VALU in-flight FIFO overflow"); end
    if (vr_cnt == '0 && va_pop)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top: VALU done/trap pulse without an in-flight record"); end
    // (LSU_K >= 2): LSU FIFO records -- each has its warp's in-flight bit, no two hold the same warp (plan A:
    // the scheduler's per-warp in-flight bit allows one LSU instruction per warp), no other unit holds that warp
    if (LSU_K > 1) begin
      for (int j = 1; j < LRD; j++) if (j < int'(lr_cnt)) begin
        if (!infl_q[lr_slot[j]])
          begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: LSU record %0d without scheduler in-flight bit", j); end
        for (int i = 0; i < j; i++)
          if (lr_slot[i] == lr_slot[j])
            begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: LSU records %0d and %0d hold the same warp", i, j); end
        for (int u = 1; u < NU; u++)
          if (u != UNIT_LSU && rec_v[u] && rec_slot[u] == lr_slot[j])
            begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: unit %0d and LSU record %0d hold the same warp", u, j); end
        for (int i = 1; i < VRD; i++)
          if (i < int'(vr_cnt) && vr_slot[i] == lr_slot[j])
            begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: VALU record %0d and LSU record %0d hold the same warp", i, j); end
      end
      if (lr_push && !lr_pop && int'(lr_cnt) == LRD)
        begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top: LSU in-flight FIFO overflow"); end
      if (lr_cnt == '0 && lr_pop)
        begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top: LSU done/trap pulse without an in-flight record"); end
    end
    if (int'(lr_cnt) > int'(sim_lsu_max)) sim_lsu_max <= lr_cnt;
    // INV-FX: while a VALU instruction is in flight no SALU / XLANE / FP64 one runs and vice versa
    // (already covered by the one-hot check on rec_v above; except a pure branch in the SALU);
    // the VALU accepts only without them:
    if (va_push && fx_oth_busy)
      begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-FX: VALU accepted while SALU/XLANE/FP64 in flight"); end
    for (int u = 1; u < NU; u++) begin
      if (rec_v[u] && !infl_q[rec_slot[u]])
        begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: unit %0d record without scheduler in-flight bit", u); end
      for (int v = u + 1; v < NU; v++)
        if (rec_v[u] && rec_v[v] && rec_slot[u] == rec_slot[v])
          begin sim_inv_err <= sim_inv_err + 1; $error("ng_sp_top INV-WARP: units %0d and %0d hold the same warp", u, v); end
    end
  end
`endif
endmodule

// ng_sp_infl_q -- in-flight record FIFO of one unit that returns several done / trap pulses in acceptance order:
// a completion shifts the queue down (head = oldest), an accept in the same cycle lands behind the survivors.
// ng_sp_top instantiates it for the VALU  and the LSU (ng_lsu_kn).
module ng_sp_infl_q #(
  parameter int DEPTH = 2,
  parameter int SW    = 4,
  parameter int CW    = $clog2(DEPTH + 1)
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          push,
  input  logic          pop,
  input  logic [SW-1:0] in_slot,
  input  logic [31:0]   in_pc,
  output logic [CW-1:0] cnt,
  output logic [SW-1:0] slot [DEPTH],
  output logic [31:0]   pc   [DEPTH]
);
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cnt <= '0;
      for (int j = 0; j < DEPTH; j++) begin slot[j] <= '0; pc[j] <= '0; end
    end else begin
      if (pop)
        for (int j = 0; j < DEPTH - 1; j++) begin slot[j] <= slot[j+1]; pc[j] <= pc[j+1]; end
      if (push) begin
        slot[int'(cnt) - int'(pop)] <= in_slot;
        pc[int'(cnt) - int'(pop)]   <= in_pc;
      end
      cnt <= cnt + CW'(push) - CW'(pop);
    end
  end
endmodule
