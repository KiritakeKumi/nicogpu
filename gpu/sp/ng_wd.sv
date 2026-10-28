// ng_wd.sv -- SP work distributor.
//
// Consumes one CP work-distributor request (ng_cp_pkg::wd_req_t, captured by
// ng_cp_port from the 42-dword WDIST sequence; wd_start is a one-cycle pulse,
// wd_req stays stable until wd_done) and runs the DISPATCH on one SP:
//
//   CHK     validate the request (workgroup <= 1024 threads and <= NW warps,
//           1 <= num_vregs <= 128, nwarps * num_vregs <= VREGS, shared <= SH_BYTES); a zero grid dimension
//           is a no-op (wd_done at once); an unrunnable request reports
//           wd_done + wd_fault without launching anything.
//   SCRUB   (only when the request carries scrub = 1, see below; runs inside W_CHK4, the debug state stays CHK4)
//   PICK    per workgroup, in the model's linear order (x fastest, then y, z;
// only every wg_step-th workgroup starting at wg_first when the
//           request carries a stride -- the multi-SP core ng_core_wd gives SP i
//           of N wg_first = i, wg_step = N; the skipped ones are passed over
//           in PICK at one per cycle; 0 / 0 = every workgroup, unchanged timing):
//           wait for a free WG tag whose shared window fits, and for the
//           scheduler's slot choice -- ng_sp_sched always assigns the lowest
//           free slots, so the WD mirrors the slot occupancy and predicts the
//           exact slot set -- to be usable: register window inside VREGS,
//           no asynchronous unit (LSU/SFU) still working for a previous
//           warp of that slot.
//   PREP    per warp k (ascending slot = ascending warp index): register-file
//           allocation (base = slot * num_vregs), then in 32 cycles the
//           packed local ids of the 32 lanes (ABI v0.1 2: x | y << 10 |
//           z << 20, model Core::launch) and all 64 uniforms over two write
//           ports (u0..u25 = SET_USER_REGS, u26..u28 = workgroup id, u29 =
//           warp index, u30..u63 = 0), then v0 (+ v1..v(n-1) = 0 when
//           CLEAR_VREGS=1; default 0: v1..v127 are undefined at warp start), then the
//           SP-top slot table entry (EXEC = valid-thread mask, ids, shared
//           window).
//   LAUNCH  launch the workgroup in the scheduler if the slot prediction
//           still holds (a slot freed during PREP changes the scheduler's
//           choice: PREP is redone).
//   DRAIN   all workgroups launched -> wait until every warp ended and every
//           unit is idle (stores of finished warps reach memory) -> wd_done.
//
// Scrub (a request with `scrub` set (the dispatch's owner differs from the previous
// dispatch's, ng_cp_port) makes the WD erase what the previous owner may have left in the SP before the first
// workgroup, in W_CHK4 (also for a no-op / invalid request, which consumes the owner change like any other):
//   (1) the L1D is invalidated (l1_inv pulse, 1 clk);
//   (2) the shared array windows this dispatch can address (min(NWG, SH_BYTES / stride) windows of the 128 B aligned
//       stride; the whole array for a no-op / invalid request; nothing for shared_bytes = 0) are zeroed one 128 B row
//       (a dword row of all 32 banks) per clock through sh_clr_* (ng_lsu_shmem);
//   (3) the whole register file is zeroed through the WD write port: for r = 0 .. ceil(VREGS / 128) - 1 slot 0 is
//       allocated at base 128 r (nregs <= 128) and written with zero, 128 registers x NBEAT beats each (VREGS * NBEAT
//       clks in all), then slot 0 is released.  (The CLEAR_VREGS path writes only the slot windows of the dispatch,
//       which would leave the previous owner's values in the slots it does not use but a later same-owner dispatch
//       does; the whole file is cleared once per owner change instead.)
// (2) and (3) run concurrently; the scrub step ends when both are done.  scrub = 0 changes nothing (cycle for cycle the
// previous WD).  The WD starts a dispatch with the SP quiescent, so the shared RAM port, the register-file write port
// and the L1D are free.
//
// Trap: any trap (trap_any from ng_sp_top, which halts issue at once) stops
// launching; once every unit is idle the WD reports wd_done + wd_fault and
// pulses `flush`, a soft reset of the SP datapath (warps, register-file
// allocations, tables), so sp_idle returns without host action.
// core_reset (folded into rst_n by ng_sp_top) aborts everything at any time;
// no wd_done is generated for an aborted dispatch (ng_cp_port clears its own
// busy state on core_reset).
//
// one SP only (no core/SP placement; adds the static
// wg_first / wg_step stride a multi-SP core uses),
// fixed per-slot register windows, WG tag = shared window index, 64 uniform
// writes per warp, ids generated one lane per cycle.
module ng_wd
  import ng_cp_pkg::*;
#(
  parameter int NW          = 16,     // warp slots (ng_sp_sched NG_SP_NUM_WARPS)
  parameter int NWG         = 8,      // workgroup tags (NG_SP_NUM_WGS)
  parameter int NL          = 16,     // register-file lanes per beat
  parameter int VREGS       = 1024,   // warp-registers per SP
  parameter int SH_BYTES    = 65536,  // shared array bytes (ng_lsu SH_AW)
  parameter bit CLEAR_VREGS = 1'b0
) (
  input  logic                     clk,
  input  logic                     rst_n,

  // ---- CP work-distributor port ----
  input  logic                     wd_start,
  input  wd_req_t                  wd_req,
  output logic                     wd_done,
  output logic                     wd_fault,
  output logic                     busy,
  output wd_req_t                  cfg,          // latched request (stable while busy)

  // ---- scheduler ----
  output logic                     launch_valid,
  input  logic                     launch_ready,
  output logic [$clog2(NWG)-1:0]   launch_wg,
  output logic [4:0]               launch_nwarps,
  output logic [31:0]              launch_pc,
  input  logic [3:0]               launch_slot0,
  input  logic                     ev_warp_done_valid,
  input  logic [$clog2(NW)-1:0]    ev_warp_done_slot,
  input  logic [$clog2(NWG)-1:0]   ev_warp_done_wg,

  // ---- SP status / control ----
  input  logic                     trap_any,      // latched first trap (SP halted)
  input  logic                     units_busy,    // any execution unit / fetch busy
  input  logic [NW-1:0]            slot_async_busy,
  output logic                     flush,         // soft reset of the SP datapath (after a fault)
  output logic                     ic_inv,        // icache invalidate (dispatch start)
  output logic                     l1_inv,        // L1D invalidate pulse (scrub)
  output logic                     sh_clr_en,     // zero shared row sh_clr_row in every bank (registered, scrub)
  output logic [$clog2(SH_BYTES)-7-1:0] sh_clr_row,
  output logic                     scrub_ev,      // pulse, a scrubbing dispatch started its scrub
  output logic                     scrub_busy,    // level, WD inside the scrub step (L1D / shared part)
  output logic [NW-1:0]            occ,           // occupied slots (mirror of the scheduler)
  output logic [3:0]               dbg_st,        // state (wst_t encoding, debug only; BAR0 debug page)

  // ---- register file ----
  output logic                     alloc_valid,
  output logic [$clog2(NW)-1:0]    alloc_slot,
  output logic [$clog2(VREGS)-1:0] alloc_base,
  output logic [7:0]               alloc_nregs,
  output logic                     wr_valid,
  input  logic                     wr_ready,
  output logic [$clog2(NW)-1:0]    wr_slot,
  output logic [6:0]               wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]            wr_lanes,
  output logic [1:0]               wr_hsel,
  output logic [NL*32-1:0]         wr_data,
  output logic [1:0]               uw_valid,
  output logic [$clog2(NW)-1:0]    uw_slot [2],
  output logic [5:0]               uw_reg  [2],
  output logic [31:0]              uw_data [2],

  // ---- SP-top slot table (one write per prepared warp) ----
  output logic                     tbl_we,
  output logic [$clog2(NW)-1:0]    tbl_slot,
  output logic [31:0]              tbl_exec,
  output logic [31:0]              tbl_wg_lin,
  output logic [4:0]               tbl_widx,
  output logic [$clog2(NWG)-1:0]   tbl_tag,
  output logic [31:0]              tbl_wgid [3],
  output logic [15:0]              tbl_sh_base
);
  localparam int SW    = $clog2(NW);
  localparam int WGW   = $clog2(NWG);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int PW    = $clog2(VREGS);
  localparam int SRW   = $clog2(SH_BYTES) - 7;      // 128 B shared rows (one row = dword row of all 32 banks)

  typedef enum logic [3:0] {
    W_IDLE, W_CHK, W_PICK, W_ALLOC, W_INIT, W_VWR, W_TBL, W_LAUNCH,
    W_DRAIN, W_DONE, W_CFGF, W_FWAIT, W_FLUSH, W_CHK2, W_CHK3, W_CHK4
  } wst_t;
  wst_t st;
  assign dbg_st = st;

  wd_req_t       rq;
  assign cfg = rq;

  // ---------------------------------------------------------------- request derived values
  logic [31:0] wx, wy, wz, gx, gy, gz;
  assign wx = rq.wg[0];   assign wy = rq.wg[1];   assign wz = rq.wg[2];
  assign gx = rq.grid[0]; assign gy = rq.grid[1]; assign gz = rq.grid[2];
  logic [7:0]  nregs;
  assign nregs = rq.num_vregs[7:0];

  // Timing (addendum): the request is stable while busy, so the derived
  // values are computed over three registered stages after the request is
  // captured (W_CHK: dims, w*w product, shared stride; W_CHK2: nthreads;
  // W_CHK3: nwarps) instead of one long wx*wy*wz -> launch combinational path.
  logic [31:0] nthreads;     // valid only when every dimension <= 1024
  logic [5:0]  nwarps;       // ceil(nthreads / 32), <= 32
  logic [16:0] sh_stride;    // shared window stride (128 B aligned)
  logic [21:0] pxy;          // wx * wy (valid when dims_ok)
  logic        dims_ok, grid_zero, cfg_ok;
  always_comb begin
    cfg_ok    = dims_ok && (nthreads <= 1024) && (int'(nwarps) <= NW) &&
                (rq.num_vregs >= 1) && (rq.num_vregs <= 128) && (rq.shared_bytes <= SH_BYTES) &&
                (int'(nwarps) * int'(rq.num_vregs) <= VREGS);   // else W_PICK could never find a usable slot set
  end

  // ---------------------------------------------------------------- scrub bookkeeping
  logic [1:0]     sc_ph;                  // 0 not started, 1 started (L1D inv sent, rows known), 2 clearing, 3 done
  logic [SRW:0]   sc_cnt, sc_rows;
  typedef enum logic [2:0] {RF_OFF, RF_ALLOC, RF_GAP, RF_WR, RF_REL, RF_DONE} rf_t;
  rf_t            rf_st;                  // register-file clear sequencer
  logic [PW-1:0]  rf_base;
  logic [6:0]     rf_reg;
  logic [BTW-1:0] rf_beat;
  logic [7:0]     rf_n;
  assign rf_n = (VREGS - int'(rf_base) > 128) ? 8'd128 : 8'(VREGS - int'(rf_base));
  logic [SRW:0]   sh_rows_c;
  always_comb begin
    int nwin;
    int prod;
    nwin = 0;
    for (int t = 0; t < NWG; t++) if ((t + 1) * int'(sh_stride) <= SH_BYTES) nwin++;
    prod = (rq.shared_bytes == 0) ? 0 : nwin * int'(sh_stride);
    sh_rows_c = (cfg_ok && !grid_zero) ? (SRW + 1)'(prod >> 7) : (SRW + 1)'(SH_BYTES >> 7);
  end

  // ---------------------------------------------------------------- occupancy mirror
  // occ follows ng_sp_sched's W_EMPTY: set at launch, cleared by ev_warp_done
  // (the scheduler frees the slot one cycle before the event is visible, so
  // the live view clears the event's slot combinationally).
  logic [NW-1:0] occ_live;
  logic [4:0]    tag_cnt [NWG];
  always_comb begin
    occ_live = occ;
    if (ev_warp_done_valid) occ_live[ev_warp_done_slot] = 1'b0;
  end

  // lowest nwarps free slots == the scheduler's assign_mask
  // (parallel prefix popcount: a free slot is picked when fewer than nwarps
  // free slots lie below it)
  logic [NW-1:0] pick_mask;
  logic [4:0]    pre_cnt [NW];
  logic [4:0]    free_n;
  always_comb begin
    for (int s = 0; s < NW; s++) begin
      pre_cnt[s] = '0;
      for (int t = 0; t < s; t++) pre_cnt[s] = pre_cnt[s] + 5'(!occ_live[t]);
    end
    free_n = pre_cnt[NW-1] + 5'(!occ_live[NW-1]);
    for (int s = 0; s < NW; s++)
      pick_mask[s] = !occ_live[s] && ({1'b0, pre_cnt[s]} < nwarps);
  end

  logic pick_slots_ok;
  always_comb begin
    pick_slots_ok = ({1'b0, free_n} >= nwarps) && (nwarps != 0);
    for (int s = 0; s < NW; s++)
      if (pick_mask[s] && ((s + 1) * int'(nregs) > VREGS || slot_async_busy[s])) pick_slots_ok = 1'b0;
  end

  // lowest free tag whose shared window fits
  logic           tag_ok;
  logic [WGW-1:0] tag_pick;
  always_comb begin
    tag_ok = 1'b0;
    tag_pick = '0;
    for (int t = NWG - 1; t >= 0; t--)
      if (tag_cnt[t] == 5'd0 && (rq.shared_bytes == 0 || (t + 1) * int'(sh_stride) <= SH_BYTES)) begin
        tag_ok = 1'b1;
        tag_pick = WGW'(t);
      end
  end

  // ---------------------------------------------------------------- per-workgroup / per-warp state
  logic [31:0]    cx, cy, cz;      // current workgroup id
  logic [31:0]    wg_lin;
  logic           last_wg;
  assign last_wg = (cx == gx - 1) && (cy == gy - 1) && (cz == gz - 1);
  // next workgroup id in linear order (x fastest)
  logic [31:0]    nx, ny, nz;
  always_comb begin
    nx = cx + 32'd1; ny = cy; nz = cz;
    if (cx + 1 == gx) begin
      nx = '0;
      if (cy + 1 == gy) begin ny = '0; nz = cz + 32'd1; end
      else ny = cy + 32'd1;
    end
  end
  logic [7:0]     skip;            // workgroups still to pass over before the next PICK

  logic [NW-1:0]  prep_mask, rem_mask;
  logic [WGW-1:0] cur_tag;
  logic [4:0]     k;               // warp index in the workgroup
  wire [SW-1:0]   cur_slot = SW'(ng_bitops_pkg::lowest_set(64'(rem_mask)));

  // thread enumeration (x fastest) across the warps of the workgroup
  logic [10:0]    tx, ty, tz;
  logic [10:0]    tcount;          // threads enumerated so far in this workgroup
  logic [31:0]    idbuf [32];
  logic [4:0]     c32;             // INIT cycle = lane / uniform pair index
  logic [6:0]     vr;              // vector register being written
  logic [BTW-1:0] vb;              // beat

  // uniform value of register r for warp k
  function automatic logic [31:0] uval(input int r);
    if (r < WD_NUREGS) return rq.uregs[r];
    case (r)
      26: return cx;
      27: return cy;
      28: return cz;
      29: return {27'd0, k};
      default: return 32'd0;
    endcase
  endfunction

  logic [31:0] exec_mask;
  always_comb begin
    logic [31:0] left;
    left = nthreads - {22'd0, k, 5'd0};
    exec_mask = (left >= 32) ? 32'hffff_ffff : ((32'd1 << left[4:0]) - 32'd1);
  end

  // ---------------------------------------------------------------- outputs (combinational)
  assign busy          = (st != W_IDLE);
  assign launch_valid  = (st == W_LAUNCH) && (pick_mask == prep_mask) && !trap_any;
  assign launch_wg     = cur_tag;
  assign launch_nwarps = nwarps[4:0];
  assign launch_pc     = rq.code_va[31:0] - rq.code_base[31:0];
  assign flush         = (st == W_FLUSH);

  // register-file clear of the scrub: slot 0 allocated at base rf_base (RF_ALLOC), written (RF_WR), released (RF_REL)
  wire   rf_al = (rf_st == RF_ALLOC) || (rf_st == RF_REL);
  assign alloc_valid = (st == W_ALLOC) || rf_al;
  assign alloc_slot  = (rf_st != RF_OFF) ? '0 : cur_slot;
  assign alloc_base  = (rf_st != RF_OFF) ? rf_base : PW'(int'(cur_slot) * int'(nregs));
  assign alloc_nregs = (rf_st == RF_REL) ? 8'd0 : (rf_st != RF_OFF) ? rf_n : nregs;

  always_comb begin
    wr_valid = (st == W_VWR) || (rf_st == RF_WR);
    wr_slot  = (rf_st != RF_OFF) ? '0 : cur_slot;
    wr_reg   = (rf_st != RF_OFF) ? rf_reg : vr;
    wr_beat  = (rf_st != RF_OFF) ? rf_beat : vb;
    wr_lanes = '1;
    wr_hsel  = 2'b11;
    wr_data  = '0;
    if (vr == 7'd0 && rf_st == RF_OFF)
      for (int l = 0; l < NL; l++) wr_data[l*32 +: 32] = idbuf[int'(vb) * NL + l];
  end

  always_comb begin
    for (int p = 0; p < 2; p++) begin
      uw_valid[p] = (st == W_INIT);
      uw_slot[p]  = cur_slot;
      uw_reg[p]   = {c32, 1'(p)};
      uw_data[p]  = uval(int'({c32, 1'(p)}));
    end
  end

  assign tbl_we      = (st == W_TBL);
  assign tbl_slot    = cur_slot;
  assign tbl_exec    = exec_mask;
  assign tbl_wg_lin  = wg_lin;
  assign tbl_widx    = k;
  assign tbl_tag     = cur_tag;
  assign tbl_wgid[0] = cx;
  assign tbl_wgid[1] = cy;
  assign tbl_wgid[2] = cz;
  assign tbl_sh_base = 16'(int'(cur_tag) * int'(sh_stride));

  // --------------------------------------------------------------
  wire vlast = (vr == (CLEAR_VREGS ? 7'(nregs - 8'd1) : 7'd0)) && (int'(vb) == NBEAT - 1);
  assign scrub_busy = (st == W_CHK4) && rq.scrub && (sc_ph != 2'd3);
  wire abort_trap = trap_any && (st == W_PICK || st == W_ALLOC || st == W_INIT || st == W_VWR ||
                                 st == W_TBL || st == W_LAUNCH || st == W_DRAIN);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= W_IDLE;
      rq <= '0;
      wd_done <= 1'b0; wd_fault <= 1'b0; ic_inv <= 1'b0;
      l1_inv <= 1'b0; sh_clr_en <= 1'b0; sh_clr_row <= '0; scrub_ev <= 1'b0;
      sc_ph <= '0; sc_cnt <= '0; sc_rows <= '0; rf_st <= RF_OFF; rf_base <= '0; rf_reg <= '0; rf_beat <= '0;
      occ <= '0;
      for (int t = 0; t < NWG; t++) tag_cnt[t] <= '0;
      cx <= '0; cy <= '0; cz <= '0; wg_lin <= '0; skip <= '0;
      dims_ok <= 1'b0; grid_zero <= 1'b0; pxy <= '0; nthreads <= '0; nwarps <= '0; sh_stride <= '0;
      prep_mask <= '0; rem_mask <= '0; cur_tag <= '0; k <= '0;
      tx <= '0; ty <= '0; tz <= '0; tcount <= '0;
      c32 <= '0; vr <= '0; vb <= '0;
      for (int l = 0; l < 32; l++) idbuf[l] <= '0;
    end else begin
      wd_done <= 1'b0;
      ic_inv  <= 1'b0;
      l1_inv  <= 1'b0; sh_clr_en <= 1'b0; scrub_ev <= 1'b0;

      // occupancy / tag bookkeeping (independent of the FSM)
      if (ev_warp_done_valid) begin
        occ[ev_warp_done_slot] <= 1'b0;
        if (tag_cnt[ev_warp_done_wg] != 5'd0) tag_cnt[ev_warp_done_wg] <= tag_cnt[ev_warp_done_wg] - 5'd1;
      end

      if (abort_trap) begin
        st <= W_FWAIT;
      end else begin
        case (st)
          W_IDLE: begin
            if (wd_start) begin
              rq     <= wd_req;
              ic_inv <= 1'b1;
              sc_ph  <= '0; rf_st <= RF_OFF;
              st     <= W_CHK;
            end
          end
          W_CHK: begin
            cx <= '0; cy <= '0; cz <= '0; wg_lin <= '0;
            dims_ok   <= (wx != 0) && (wy != 0) && (wz != 0) && (wx <= 1024) && (wy <= 1024) && (wz <= 1024);
            grid_zero <= (gx == 0) || (gy == 0) || (gz == 0);
            pxy       <= wx[10:0] * wy[10:0];
            sh_stride <= 17'((rq.shared_bytes + 32'd127) & ~32'd127);
            st        <= W_CHK2;
          end
          W_CHK2: begin
            nthreads <= dims_ok ? (32'(pxy) * {21'd0, wz[10:0]}) : 32'd0;
            st       <= W_CHK3;
          end
          W_CHK3: begin
            nwarps <= 6'((nthreads + 31) >> 5);
            st     <= W_CHK4;
          end
          W_CHK4: begin
            // judged once the scrub step (below) is done or not requested
            skip <= rq.wg_first;
            if (!scrub_busy) st <= grid_zero ? W_DONE : !cfg_ok ? W_CFGF : W_PICK;
          end
          W_PICK: begin
            if (skip != 8'd0) begin
              // stride: pass over a workgroup of another SP (none left for this SP: drain)
              if (last_wg) st <= W_DRAIN;
              else begin
                cx <= nx; cy <= ny; cz <= nz;
                wg_lin <= wg_lin + 32'd1;
                skip <= skip - 8'd1;
              end
            end else if (tag_ok && pick_slots_ok) begin
              prep_mask <= pick_mask;
              rem_mask  <= pick_mask;
              cur_tag   <= tag_pick;
              k         <= '0;
              tx <= '0; ty <= '0; tz <= '0; tcount <= '0;
              st        <= W_ALLOC;
            end
          end
          W_ALLOC: begin
            c32 <= '0;
            st  <= W_INIT;
          end
          W_INIT: begin
            // lane c32 of warp k: thread index k*32 + c32 (enumerated in order)
            if (tcount < nthreads[10:0]) begin
              idbuf[c32] <= {2'b00, tz[9:0], ty[9:0], tx[9:0]};
              tcount <= tcount + 11'd1;
              if (tx + 11'd1 == wx[10:0]) begin
                tx <= '0;
                if (ty + 11'd1 == wy[10:0]) begin ty <= '0; tz <= tz + 11'd1; end
                else ty <= ty + 11'd1;
              end else begin
                tx <= tx + 11'd1;
              end
            end else begin
              idbuf[c32] <= '0;
            end
            c32 <= c32 + 5'd1;
            if (c32 == 5'd31) begin vr <= '0; vb <= '0; st <= W_VWR; end
          end
          W_VWR: begin
            if (wr_ready) begin
              if (vlast) st <= W_TBL;
              else if (int'(vb) == NBEAT - 1) begin vb <= '0; vr <= vr + 7'd1; end
              else vb <= vb + 1'b1;
            end
          end
          W_TBL: begin
            rem_mask[cur_slot] <= 1'b0;
            k <= k + 5'd1;
            if ((rem_mask & ~(NW'(1) << cur_slot)) == '0) st <= W_LAUNCH;
            else st <= W_ALLOC;
          end
          W_LAUNCH: begin
            if (pick_mask != prep_mask) begin
              st <= W_PICK;                     // a slot freed meanwhile: redo PREP
            end else if (launch_ready) begin
              occ <= (occ & ~(ev_warp_done_valid ? (NW'(1) << ev_warp_done_slot) : '0)) | prep_mask;
              tag_cnt[cur_tag] <= 5'(nwarps);
              if (last_wg) begin
                st <= W_DRAIN;
              end else begin
                wg_lin <= wg_lin + 32'd1;
                cx <= nx; cy <= ny; cz <= nz;
                skip <= (rq.wg_step > 8'd1) ? rq.wg_step - 8'd1 : 8'd0;
                st <= W_PICK;
              end
            end
          end
          W_DRAIN: begin
            if (occ_live == '0 && !units_busy) st <= W_DONE;
          end
          W_DONE: begin
            wd_done <= 1'b1; wd_fault <= 1'b0; st <= W_IDLE;
          end
          W_CFGF: begin
            wd_done <= 1'b1; wd_fault <= 1'b1; st <= W_IDLE;
          end
          W_FWAIT: begin
            if (!units_busy) begin
              wd_done <= 1'b1; wd_fault <= 1'b1; st <= W_FLUSH;
            end
          end
          W_FLUSH: begin
            occ <= '0;
            for (int t = 0; t < NWG; t++) tag_cnt[t] <= '0;
            st <= W_IDLE;
          end
          default: st <= W_IDLE;
        endcase
      end

      // scrub step (scrub_busy: the FSM stays in W_CHK4, before the request is judged, so a no-op / invalid
      // request consumes the owner change too): L1D invalidate, then the shared rows and the register file,
      // concurrently
      if (scrub_busy && sc_ph == 2'd0) begin
        l1_inv <= 1'b1; scrub_ev <= 1'b1;
        sc_rows <= sh_rows_c; sc_cnt <= '0; sc_ph <= 2'd1;
        rf_st <= RF_ALLOC; rf_base <= '0;
      end
      if (scrub_busy && sc_ph != 2'd0) begin
        if (sc_cnt != sc_rows) begin
          sh_clr_en <= 1'b1; sh_clr_row <= sc_cnt[SRW-1:0]; sc_cnt <= sc_cnt + 1'b1;
        end
        case (rf_st)
          RF_ALLOC: rf_st <= RF_GAP;
          RF_GAP:   begin rf_st <= RF_WR; rf_reg <= '0; rf_beat <= '0; end
          RF_WR: if (wr_ready) begin
            if (int'(rf_beat) != NBEAT - 1) rf_beat <= rf_beat + 1'b1;
            else begin
              rf_beat <= '0;
              if (int'(rf_reg) + 1 < int'(rf_n)) rf_reg <= rf_reg + 7'd1;
              else if (int'(rf_base) + int'(rf_n) >= VREGS) rf_st <= RF_REL;
              else begin rf_base <= rf_base + PW'(128); rf_st <= RF_ALLOC; end
            end
          end
          RF_REL: rf_st <= RF_DONE;
          default: ;
        endcase
        if (sc_cnt == sc_rows && rf_st == RF_DONE) begin sc_ph <= 2'd3; rf_st <= RF_OFF; end
      end
    end
  end

`ifdef NG_SP_ASSERT
  always @(posedge clk) if (rst_n && launch_valid && launch_ready) begin
    int lo;
    lo = -1;
    for (int s = NW - 1; s >= 0; s--) if (prep_mask[s]) lo = s;
    if (lo != int'(launch_slot0))
      $error("ng_wd: scheduler slot0 %0d != predicted %0d", launch_slot0, lo);
  end
`endif
endmodule
