// ng_rsp_dbg.sv -- clk_ts half of the BAR0 debug page of the M1 SP RM: read-only registers
// 0x0800..0x08BF,  0x0900..0x093F,  0x0940..0x095F,  0x0960..0x097F
// 0x0980..0x09BF, read through the CP
// register bridge (ng_rcp_cpbus performs the read for one clk_ts cycle and captures `rdata` combinationally from `addr`).
// The board has no ILA: this page is the window into the SP.
//
//   0x800 DBG_ID          0x5350_0005 ("SP", page version 5: version 4 (0x900 + 0x940 + 0x960 + 0x980 blocks) + the words 0x9AC..0x9BC)
//   0x804 SP_STATUS       ng_sp_top dbg_status: [15:0] occupied warp slots, [19:16] WD state, [20] WD busy,
//                         [21] units busy, [22] trap latched, [23] icache busy, [24] LSU busy, [26:25] fabric arb,
//                         [28:27] MMU arb, [29] scheduler stalled_all, [30] sp_idle, [31] datapath in reset
//   0x808 BRIDGE_TS       [0] fabric request open, [3:2] MMU bridge state (0 idle, 1 wait, 2 resp), [4] LSU external
//                         fault seen, [5] CP CORE_RESET (SP held in reset), [6] wd_start seen since reset
//   0x80C TRAP_CTRL       first trap of the last dispatch: [0] valid, [3:1] unit, [7:4] slot, [11:8] cause, [20:16] warp
//   0x810 TRAP_PC         byte offset from CODE_BASE        0x814 TRAP_INFO        0x818 TRAP_WG (linear workgroup)
//   0x81C/820 WD_INSTR    instructions retired by the running / last dispatch (lo / hi)
//   0x824 COMMIT_TOTAL    instructions committed since reset (scheduler commit events, 32-bit wrap)
//   0x828 DISPATCHES      wd_start count                    0x82C DISPATCH_DONE   [15:0] wd_done, [31:16] with wd_fault
//   0x830 IC_HIT  0x834 IC_MISS (L0 instruction cache)
//   0x838/83C LSU_FLT     last LSU BAD_PHYS report: VA lo; hi [15:0] VA[47:32], [19:16] VMID, [21:20] acc, [31] valid
//   0x840 FAB_REQS        fabric line requests              0x844 FAB_ERRS        responses with err != 0
//   0x848/84C FAB_LAST    last fabric request: PA lo; hi [15:0] PA[47:32], [18:16] kind, [21:20] err of the response
//   0x850 MMU_REQS        SP translations                   0x854 MMU_FAULTS      translations with fault != 0
//   0x858/85C MMU_LAST    last SP translation: VA lo; hi [15:0] VA[47:32], [18:16] fault
//   0x860 LAST_PC         last committed PC (byte offset)   0x864 LAST_COMMIT [3:0] slot, [8:4] warp, [31:16] WG[15:0]
//   0x880 + 4n            SLOT_PC[n] (n = 0..15): last committed PC of warp slot n
// measurement block.  FAB_*_LAT: per M_RD / M_WR fabric request, clk_ts cycles
// the accept in ng_rsp_fab_ts to its response pulse (CDC + fab_l2 + engine + AXI round trip), free-running from the
// RM reset like FAB_REQS (sum 48 bit: read HI, LO, HI again if the SP may be running).  LSU_*: ng_lsu counters, cleared
// by the SP datapath reset like IC_HIT / IC_MISS (CP CORE_RESET, WDIST abort, fault flush).
//   0x900/904 FAB_RD_LAT_SUM lo / hi[15:0]    0x908 FAB_RD_LAT_N       0x90C FAB_RD_LAT_MAX
//   0x910/914 FAB_WR_LAT_SUM lo / hi[15:0]    0x918 FAB_WR_LAT_N       0x91C FAB_WR_LAT_MAX
//   0x920 LSU_L1_RD  line reads issued by the LSU (load groups + FP-atomic CAS line reads)
//   0x924 LSU_L1_WR  line writes issued by the LSU (store groups + FP-atomic CAS line writes)
//   0x928 LSU_INSTR_MEM  global ld / st / uld instructions on the line-group path (not shared, not atomic)
//   0x92C LSU_GROUPS     line-group requests of those instructions (LSU_GROUPS / LSU_INSTR_MEM = lines per instruction)
// SP performance block: ng_sp_top stat_perf, cycle / event counts of the SP clock, cleared
// by the SP datapath reset like LSU_*:
//   0x940 SP_ACTIVE       cycles with a warp slot occupied     0x944 VALU_BACK_BUSY  cycles the VALU back half is busy
//   0x948 BR_WAIT_DRAIN   cycles a pure branch waits in the issue buffer (VALU drain / fixed group busy; 23b2: SALU busy)
//   0x94C FRONT_STARVED   warp resident, VALU back half idle and issue buffer empty
//   0x950 VALU_ISSUED     VALU instructions accepted           0x954 BR_ISSUED       pure branches accepted
//   0x958 IB_WAIT_OWN     issue buffer waits for its warp's in-flight instruction
//   0x95C SALU_BUSY       cycles with a SALU instruction in flight
// fabric slot block, ng_rsp_fab_ts, free-running from the RM reset:
//   0x960 FAB_SLOTS      [7:0] slots allocated now (orphans included), [15:8] largest value seen, [23:16] NOUT
//   0x964 FAB_ORPHANS    answers reclaimed without delivery (orphans of an SP reset / WDIST abort)
//   0x968 FAB_FENCES     fence requests accepted (atomics, cache maintenance)
//   0x96C FAB_FENCE_STALL   clk_ts cycles a request waited for a fence
//   0x970 FAB_SLOT_STALL    clk_ts cycles a request waited for a free slot
//   0x974 FAB_CONFL_STALL   clk_ts cycles a request waited for a same-line conflict (RAW / WAR / WAW)
// LSU block, ng_lsu_top stat_k, cleared by the SP datapath reset like LSU_*:
//   0x980 LSU_FE_BUSY      cycles the LSU front end is not idle (LSU_K = 1: cycles the LSU is busy)
//   0x984 LSU_FE_RALLOC    cycles the FE waits for a free ring entry
//   0x988 LSU_FE_ISS_STALL cycles a line request of the FE waits (no slot / conflict / fence / L1D / IOF full)
//   0x98C LSU_FE_SYNC      cycles a synchronous instruction (shared / atomic / cache.*) or an FE trap waits for the drain
//   0x990/994 LSU_RING_OCC lo / hi[15:0]  sum of the ring occupancy per cycle (48 bit; / cycles = instructions in flight)
//   0x998 LSU_RING_MAX     largest ring occupancy
//   0x99C LSU_BE_WAIT      cycles the back end waits for an answer of the ring head
//   0x9A0 LSU_BE_WB_STALL  cycles the back end's register write waits for wr_ready
//   0x9A4 LSU_LATE_ERR     fabric-error (late) traps
//   0x9A8 LSU_CFG          [7:0] LSU_K, [15:8] MEM_OUT, [23:16] IOF entries (0 for LSU_K = 1)
//   (LSU_K = 1: only 0x980 and 0x9A8 count / are set, the rest read 0)
// scrub / invalidate counters, ng_sp_top stat_scrub, cleared by the SP datapath reset:
//   0x9AC SCRUB_N          dispatches that scrubbed (the owner differed from the previous dispatch's)
//   0x9B0 SCRUB_CYC        clk_ts cycles spent in the scrub step (L1D inv + shared rows + register file; ~2.1 K cycles with the
//                          whole register file, VREGS * NBEAT + a few)
//   0x9B4 L1_INV_EXT       L1D invalidates requested from outside the LSU (scrub + CP CACHE packets)
//   0x9B8 CP_INV_L1        CACHE(INV_L1 | INV_K) pulses from the CP (ng_cp_port P_SP_INV)
//   0x9BC CP_INV_I         CACHE(INV_I) pulses from the CP
// other offsets read 0
module ng_rsp_dbg (
  input  logic        clk,                // clk_ts
  input  logic        rst_n,
  input  logic [8:0]  addr,               // byte offset from 0x800 (host_addr[8:0]; [8] = the 0x900 block)
  output logic [31:0] rdata,
  // SP
  input  logic [31:0] sp_status,
  input  logic        wd_start,
  input  logic        wd_done,
  input  logic        wd_fault,
  input  logic [63:0] wd_instr,
  input  logic        commit_valid,
  input  logic [3:0]  commit_slot,
  input  logic [31:0] commit_pc,
  input  logic [31:0] commit_wg,
  input  logic [4:0]  commit_warp,
  input  logic        trap_valid,
  input  logic [2:0]  trap_unit,
  input  logic [3:0]  trap_slot,
  input  logic [31:0] trap_pc,
  input  logic [3:0]  trap_cause,
  input  logic [31:0] trap_info,
  input  logic [31:0] trap_wg,
  input  logic [4:0]  trap_warp,
  input  logic [31:0] ic_hit,
  input  logic [31:0] ic_miss,
  input  logic        core_reset,
  // bridges
  input  logic        fab_busy,
  input  logic [31:0] fab_reqs,
  input  logic [31:0] fab_errs,
  input  logic [47:0] fab_last_pa,
  input  logic [2:0]  fab_last_kind,
  input  logic [1:0]  fab_last_err,
  input  logic [1:0]  mmu_st,
  input  logic [31:0] mmu_reqs,
  input  logic [31:0] mmu_flts,
  input  logic [63:0] mmu_last_va,
  input  logic [2:0]  mmu_last_fault,
  input  logic        flt_valid,
  input  logic [63:0] flt_va,
  input  logic [3:0]  flt_vmid,
  input  logic [1:0]  flt_acc,
  //
  input  logic [47:0] rd_lat_sum,
  input  logic [31:0] rd_lat_n,
  input  logic [31:0] rd_lat_max,
  input  logic [47:0] wr_lat_sum,
  input  logic [31:0] wr_lat_n,
  input  logic [31:0] wr_lat_max,
  input  logic [31:0] ls_l1_rd,
  input  logic [31:0] ls_l1_wr,
  input  logic [31:0] ls_instr_mem,
  input  logic [31:0] ls_groups,
  //
  input  logic [7:0]  fab_nout,
  input  logic [7:0]  fab_out,
  input  logic [7:0]  fab_max_out,
  input  logic [31:0] fab_orphans,
  input  logic [31:0] fab_cstall,
  input  logic [31:0] fab_sstall,
  input  logic [31:0] fab_fences,
  input  logic [31:0] fab_fstall,
  //
  input  logic [255:0] sp_perf,
  input  logic [351:0] lsu_k,                 // 11 words at 0x980..0x9A8
  input  logic [159:0] scrub_k                // 5 words at 0x9AC..0x9BC
);
  logic [31:0] commits, starts, last_pc;
  logic [15:0] dones, faults;
  logic [3:0]  last_slot;
  logic [4:0]  last_warp;
  logic [15:0] last_wg;
  logic        started;
  logic [31:0] slot_pc [16];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      commits <= '0; starts <= '0; dones <= '0; faults <= '0; started <= 1'b0;
      last_pc <= '0; last_slot <= '0; last_warp <= '0; last_wg <= '0;
      for (int i = 0; i < 16; i++) slot_pc[i] <= '0;
    end else begin
      if (commit_valid) begin
        commits <= commits + 32'd1;
        last_pc <= commit_pc; last_slot <= commit_slot; last_warp <= commit_warp; last_wg <= commit_wg[15:0];
        slot_pc[commit_slot] <= commit_pc;
      end
      if (wd_start) begin starts <= starts + 32'd1; started <= 1'b1; end
      if (wd_done) begin
        dones <= dones + 16'd1;
        if (wd_fault) faults <= faults + 16'd1;
      end
    end
  end

  always_comb begin
    rdata = 32'h0;
    if (addr[8]) begin
      if (addr[7:6] == 2'b00) begin
        unique case (addr[5:2])
          4'd0:  rdata = rd_lat_sum[31:0];
          4'd1:  rdata = {16'h0, rd_lat_sum[47:32]};
          4'd2:  rdata = rd_lat_n;
          4'd3:  rdata = rd_lat_max;
          4'd4:  rdata = wr_lat_sum[31:0];
          4'd5:  rdata = {16'h0, wr_lat_sum[47:32]};
          4'd6:  rdata = wr_lat_n;
          4'd7:  rdata = wr_lat_max;
          4'd8:  rdata = ls_l1_rd;
          4'd9:  rdata = ls_l1_wr;
          4'd10: rdata = ls_instr_mem;
          4'd11: rdata = ls_groups;
          default: rdata = 32'h0;
        endcase
      end else if (addr[7:5] == 3'b010) begin
        rdata = sp_perf[32 * addr[4:2] +: 32];
      end else if (addr[7:6] == 2'b10) begin
        if (addr[5:2] < 4'd11) rdata = lsu_k[32 * addr[5:2] +: 32];
        else begin
          case (addr[5:2])
            4'd11:   rdata = scrub_k[31:0];
            4'd12:   rdata = scrub_k[63:32];
            4'd13:   rdata = scrub_k[95:64];
            4'd14:   rdata = scrub_k[127:96];
            4'd15:   rdata = scrub_k[159:128];
            default: rdata = 32'h0;
          endcase
        end
      end else if (addr[7:5] == 3'b011) begin
        unique case (addr[4:2])
          3'd0:    rdata = {8'h0, fab_nout, fab_max_out, fab_out};
          3'd1:    rdata = fab_orphans;
          3'd2:    rdata = fab_fences;
          3'd3:    rdata = fab_fstall;
          3'd4:    rdata = fab_sstall;
          3'd5:    rdata = fab_cstall;
          default: rdata = 32'h0;
        endcase
      end
    end else if (addr[7]) begin
      if (!addr[6]) rdata = slot_pc[addr[5:2]];
    end else begin
      unique case (addr[6:2])
        5'd0:  rdata = 32'h5350_0005;
        5'd1:  rdata = sp_status;
        5'd2:  rdata = {25'h0, started, core_reset, flt_valid, mmu_st, 1'b0, fab_busy};
        5'd3:  rdata = {11'h0, trap_warp, 4'h0, trap_cause, trap_slot, trap_unit, trap_valid};
        5'd4:  rdata = trap_pc;
        5'd5:  rdata = trap_info;
        5'd6:  rdata = trap_wg;
        5'd7:  rdata = wd_instr[31:0];
        5'd8:  rdata = wd_instr[63:32];
        5'd9:  rdata = commits;
        5'd10: rdata = starts;
        5'd11: rdata = {faults, dones};
        5'd12: rdata = ic_hit;
        5'd13: rdata = ic_miss;
        5'd14: rdata = flt_va[31:0];
        5'd15: rdata = {flt_valid, 9'h0, flt_acc, flt_vmid, flt_va[47:32]};
        5'd16: rdata = fab_reqs;
        5'd17: rdata = fab_errs;
        5'd18: rdata = fab_last_pa[31:0];
        5'd19: rdata = {10'h0, fab_last_err, 1'b0, fab_last_kind, fab_last_pa[47:32]};
        5'd20: rdata = mmu_reqs;
        5'd21: rdata = mmu_flts;
        5'd22: rdata = mmu_last_va[31:0];
        5'd23: rdata = {13'h0, mmu_last_fault, mmu_last_va[47:32]};
        5'd24: rdata = last_pc;
        5'd25: rdata = {last_wg, 7'h0, last_warp, last_slot};
        default: rdata = 32'h0;
      endcase
    end
  end
endmodule
