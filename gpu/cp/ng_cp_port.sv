// ng_cp_port.sv -- CP private port: the Ibex data-bus slave.
//
// Decodes the Ibex LSU port into
//   REG_BASE  window onto the BAR0 CP register block (ng_cp_regs fw side),
//   PORT_BASE the private port (ng_cp_pkg::P_*): doorbell latch, events,
//             IRQ_SET, platform time, one-access memory port (MEM_*),
//             work-distributor request port (WDIST_*), Copy engine control
//             (COPY_*), debug.
// and owns the shared memory request port towards the MMU/L2/C3 adapter
// (arbiter: the Copy engine owns it while busy; MEM_* accesses wait).
//
// Ibex bus: req/gnt/rvalid, 32-bit.  gnt is combinational (always accepts);
// rvalid is one cycle after req for reads and writes.  Only full-word
// accesses are used by the firmware (be is ignored).
//
module ng_cp_port
  import ng_cp_pkg::*;
#(
  parameter int unsigned TICK_CYCLES = 0,     // 0: no tick; else one tick every TICK_CYCLES clocks (1 ms, firmware heartbeat)
  parameter bit          COPY_BULK   = 1'b0,  // Copy engine hands whole commands to the clk_l2 DMA (ng_cp_copy BULK)
  parameter int unsigned ABORT_RST_CYCLES = 16,     // WDIST_CTRL.ABORT: clocks the SP datapath is held in reset
  parameter int unsigned ABORT_SETTLE      = 16384, // ... and clocks WDIST_STATUS.ABORTING stays up afterwards (in-flight fabric drains)
  // ABORT_FAB_WAIT = 1 replaces the fixed settle window by "minimum window + fabric drained (fab_busy_i = 0)
  // + watchdog": ABORTING stays up ABORT_RST_CYCLES + ABORT_MIN clocks, then until fab_busy_i falls or
  // 2**ABORT_WDOG_LOG2 more clocks have passed (then WDIST_STATUS[4] ABORT_TIMEOUT).  0 = the behaviour (rm_cp,
  // tb_cp, tb_sp_top: fab_busy_i tied 0, nothing changes there).  Watchdog default 2**19 clocks (~5.2 ms at 99.94 MHz,
  // below the driver's 10 ms L1 IDLE wait, so an abort timeout is reported
  // (CP_FW_STATUS[2]) before the driver gives up on L1 (the original design value was 2**20, ~10.5 ms).
  parameter bit          ABORT_FAB_WAIT   = 1'b0,
  parameter int unsigned ABORT_MIN        = 256,
  parameter int unsigned ABORT_WDOG_LOG2  = 19,
  // 1 = generate wd_req.scrub (cross-owner residue scrub, see below); 0 = never set (the previous
  // behaviour, cycle for cycle)
  parameter bit          SCRUB            = 1'b0
) (

  input  logic        clk,
  input  logic        rst_n,
  input  logic        core_reset_i,   // CP_CTRL.CORE_RESET (level): clears the work-distributor request state, aborts
                                      // the Copy engine
  output logic        busy_o,         // a Copy engine command or a MEM_* access is still in flight

  // ---- Ibex data port ----
  input  logic        d_req,
  output logic        d_gnt,
  output logic        d_rvalid,
  input  logic        d_we,
  input  logic [3:0]  d_be,
  input  logic [31:0] d_addr,
  input  logic [31:0] d_wdata,
  output logic [31:0] d_rdata,
  output logic        d_err,

  // ---- register block (ng_cp_regs) ----
  output logic        fw_wr,
  output logic [11:0] fw_waddr,
  output logic [31:0] fw_wdata,
  output logic [11:0] fw_raddr,
  input  logic [31:0] fw_rdata,
  input  logic [31:0] wptr_i,
  input  logic [3:0]  ev_pend_i,
  output logic [3:0]  ev_clr_o,
  output logic [31:0] irq_set_o,

  // ---- shared memory request port (CP -> MMU/L2/C3 adapter) ----
  output logic        m_req_valid,
  input  logic        m_req_ready,
  output mem_req_t    m_req,
  input  logic        m_rsp_valid,
  input  logic        m_rsp_fault,
  input  logic [63:0] m_rsp_data,

  // ---- work distributor (SP complex) ----
  output logic        wd_start,       // 1-cycle pulse, wd_req stable until wd_done
  output wd_req_t     wd_req,
  input  logic        wd_done,        // 1-cycle pulse
  input  logic        wd_fault,
  input  logic [63:0] wd_instr,
  input  logic        sp_idle_i,
  output logic        wd_abort_o,     // WDIST_CTRL.ABORT (level, ABORT_RST_CYCLES clocks): the SP side treats it like CORE_RESET
  input  logic        fab_busy_i,     // SP fabric slots still allocated (ng_rsp_fab_ts; tie 0 without one)
  input  logic [15:0] scrub_set_i,    // 1-cycle pulse per VMID = MMU CTX_CTRL write (ctx enable / disable): owner may have changed
  output logic [2:0]  sp_inv_o,       // CACHE packet invalidates (P_SP_INV write): [0] L1D, [1] K (= L1D, uld goes through it), [2] I$

  // ---- COPY_BULK: whole-command port of the Copy engine (ng_cp_copy b_*) ----
  output logic        cb_valid,
  output logic        cb_fill,
  output logic [3:0]  cb_vmid,
  output logic [47:0] cb_src,
  output logic [47:0] cb_dst,
  output logic [63:0] cb_size,
  output logic [31:0] cb_fillv,
  output logic        cb_abort,
  input  logic        cb_done,
  input  logic        cb_fault,

  // ---- 1 ms tick towards the Ibex machine-timer interrupt (firmware heartbeat while idle) ----
  output logic        tick_o,

  // ---- debug ----
  output logic        dbg_valid_o,
  output logic [31:0] dbg_code_o
);
  // ---------------------------------------------------------------------------
  // address decode
  wire in_reg  = (d_addr[31:12] == REG_BASE[31:12]);
  wire in_port = (d_addr[31:12] == PORT_BASE[31:12]);
  wire [7:0] poff = d_addr[7:0];

  assign d_gnt = d_req;
  assign d_err = 1'b0;

  // register-block window
  assign fw_wr    = d_req & d_we & in_reg;
  assign fw_waddr = d_addr[11:0];
  assign fw_wdata = d_wdata;
  assign fw_raddr = d_addr[11:0];

  wire pw = d_req & d_we & in_port;

  // ---------------------------------------------------------------------------
  // platform time
  logic [63:0] time_q;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n) time_q <= '0; else time_q <= time_q + 64'd1;

  // ---------------------------------------------------------------------------
  // one-access memory port
  logic [47:0] mem_addr;
  logic [3:0]  mem_vmid;
  logic [1:0]  mem_size;       // log2 bytes (0..3)
  logic        mem_wr;
  logic [63:0] mem_data;
  logic        mem_busy, mem_fault, mem_issued;
  logic        copy_busy, copy_fault;

  // copy engine registers
  logic [47:0] c_src, c_dst;
  logic [63:0] c_size;
  logic [3:0]  c_vmid;
  logic [31:0] c_fill;
  logic        c_start, c_op_fill;
  logic        c_fault_clr, c_abort;

  logic        cp_req_valid, cp_req_ready;
  mem_req_t    cp_req;
  logic        port_req_valid;
  mem_req_t    port_req;

  wire [2:0]  lane = mem_addr[2:0];
  wire [7:0]  size_mask = (mem_size == 2'd0) ? 8'h01 : (mem_size == 2'd1) ? 8'h03 :
                          (mem_size == 2'd2) ? 8'h0f : 8'hff;

  always_comb begin
    port_req_valid = mem_busy & ~mem_issued & ~copy_busy;
    port_req       = '0;
    port_req.write = mem_wr;
    port_req.vmid  = mem_vmid;
    port_req.addr  = {mem_addr[47:3], 3'b000};
    port_req.wstrb = size_mask << lane;
    port_req.wdata = mem_data << (8 * lane);
  end

  assign busy_o      = copy_busy | mem_busy;
  assign m_req_valid = copy_busy ? cp_req_valid : port_req_valid;
  assign m_req       = copy_busy ? cp_req : port_req;
  assign cp_req_ready = copy_busy & m_req_ready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mem_addr <= '0; mem_vmid <= '0; mem_size <= '0; mem_wr <= 1'b0; mem_data <= '0;
      mem_busy <= 1'b0; mem_fault <= 1'b0; mem_issued <= 1'b0;
    end else begin
      // issue handshake and response
      if (port_req_valid && m_req_ready) mem_issued <= 1'b1;
      if (mem_busy && mem_issued && m_rsp_valid && !copy_busy) begin
        mem_busy <= 1'b0; mem_issued <= 1'b0;
        if (m_rsp_fault) mem_fault <= 1'b1;
        else if (!mem_wr) mem_data <= m_rsp_data >> (8 * lane);
      end
      if (pw) begin
        unique case (poff)
          P_MEM_ADDR_LO: mem_addr[31:0] <= d_wdata;
          P_MEM_ADDR_HI: mem_addr[47:32] <= d_wdata[15:0];
          P_MEM_VMID:    mem_vmid <= d_wdata[3:0];
          P_MEM_DATA:    mem_data[31:0] <= d_wdata;
          P_MEM_DATA_HI: mem_data[63:32] <= d_wdata;
          P_MEM_CTRL: if (!mem_busy) begin   // starts the access
            mem_wr <= d_wdata[0]; mem_size <= d_wdata[3:2];
            mem_busy <= 1'b1; mem_issued <= 1'b0;
          end
          P_MEM_STATUS: if (d_wdata[1]) mem_fault <= 1'b0;
          default: ;
        endcase
      end
      // S5: CORE_RESET kills the firmware that asked for the access.  One that has not gone out yet is cancelled
      // (it would otherwise reach memory after the reset, possibly while the boot loader owns the port); one that is
      // out keeps mem_busy until its response is back, which is what holds the loader (ng_cp_top boot_hold) off.
      // The sticky fault of the old firmware is forgotten like every other firmware-visible state.
      if (core_reset_i) begin
        if (mem_busy && !mem_issued && !(port_req_valid && m_req_ready)) mem_busy <= 1'b0;
        mem_fault <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Copy engine
  ng_cp_copy #(.BULK(COPY_BULK)) u_copy (
    .clk, .rst_n,
    .start(c_start), .op_fill(c_op_fill), .vmid(c_vmid), .src(c_src), .dst(c_dst),
    .size(c_size), .fill_value(c_fill), .busy(copy_busy), .fault(copy_fault),
    .fault_clr(c_fault_clr), .abort(c_abort),
    .req_valid(cp_req_valid), .req_ready(cp_req_ready), .req(cp_req),
    .rsp_valid(m_rsp_valid & copy_busy), .rsp_fault(m_rsp_fault), .rsp_data(m_rsp_data),
    .b_valid(cb_valid), .b_fill(cb_fill), .b_vmid(cb_vmid), .b_src(cb_src), .b_dst(cb_dst), .b_size(cb_size),
    .b_fillv(cb_fillv), .b_abort(cb_abort), .b_done(cb_done), .b_fault(cb_fault)
  );

  always_comb begin
    c_start     = pw && (poff == P_COPY_CTRL) && !mem_busy && !copy_busy && (d_wdata[1:0] != 2'd0) && !d_wdata[2];
    c_op_fill   = d_wdata[1];
    c_fault_clr = pw && (poff == P_COPY_STATUS) && d_wdata[1];
    c_abort     = core_reset_i || (pw && (poff == P_COPY_CTRL) && d_wdata[2]);
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      c_src <= '0; c_dst <= '0; c_size <= '0; c_vmid <= '0; c_fill <= '0;
    end else if (pw) begin
      unique case (poff)
        P_COPY_SRC_LO:  c_src[31:0] <= d_wdata;
        P_COPY_SRC_HI:  c_src[47:32] <= d_wdata[15:0];
        P_COPY_DST_LO:  c_dst[31:0] <= d_wdata;
        P_COPY_DST_HI:  c_dst[47:32] <= d_wdata[15:0];
        P_COPY_SIZE_LO: c_size[31:0] <= d_wdata;
        P_COPY_SIZE_HI: c_size[63:32] <= d_wdata;
        P_COPY_VMID:    c_vmid <= d_wdata[3:0];
        P_COPY_FILL:    c_fill <= d_wdata;
        default: ;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // work distributor request port
  logic [31:0] wd_w [WD_NWORDS];
  logic [5:0]  wd_idx;
  logic        wd_busy_q, wd_fault_q;
  logic [63:0] wd_instr_q;
  // WDIST_CTRL.ABORT: hold the SP in reset for ABORT_RST_CYCLES clocks (the SP side resets its datapath
  // work distributor exactly as for CORE_RESET, but the Ibex keeps running), then keep ABORTING up for ABORT_SETTLE
  // more clocks so requests the SP had on the fabric complete (and are dropped) before a new dispatch can start.
  // (ABORT_FAB_WAIT = 1): the window after the reset is only ABORT_MIN clocks (both CDC FIFO directions
  // drain), then ABORTING stays up while fab_busy_i (slots of the SP fabric port still allocated, including the
  // orphans of the reset: their answers are reclaimed, never delivered) until it falls or the watchdog
  // (2**ABORT_WDOG_LOG2 clocks) expires; expiry sets ab_tmo_q = WDIST_STATUS[4] ABORT_TIMEOUT (cleared by the next
  // ABORT write and by CORE_RESET) and ends ABORTING anyway -- the orphan bits keep later dispatches correct, the
  // link just has fewer free slots until the late answers arrive.
  localparam int unsigned AB_SET = ABORT_FAB_WAIT ? ABORT_MIN : ABORT_SETTLE;
  localparam int unsigned AB_W = $clog2(ABORT_RST_CYCLES + AB_SET + 1) + 1;
  logic [AB_W-1:0] ab_cnt;
  logic            wd_aborted_q, ab_wait, ab_tmo_q;
  logic [ABORT_WDOG_LOG2:0] ab_wdog;
  wire   aborting   = (ab_cnt != '0) || ab_wait;
  assign wd_abort_o = (ab_cnt > AB_W'(AB_SET));
  wire   wd_ab_wr   = pw && poff == P_WDIST_CTRL && d_wdata[0] && !aborting;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ab_cnt <= '0; wd_aborted_q <= 1'b0; ab_wait <= 1'b0; ab_tmo_q <= 1'b0; ab_wdog <= '0;
    end else begin
      if (ab_cnt != '0) ab_cnt <= ab_cnt - 1'b1;
      if (ABORT_FAB_WAIT) begin
        if (ab_cnt == AB_W'(1) && fab_busy_i) begin             // the minimum window ends with slots still out
          ab_wait <= 1'b1; ab_wdog <= '0;
        end else if (ab_wait) begin
          if (!fab_busy_i) begin
            ab_wait <= 1'b0;
          end else if (ab_wdog[ABORT_WDOG_LOG2]) begin
            ab_wait <= 1'b0; ab_tmo_q <= 1'b1;
          end else begin
            ab_wdog <= ab_wdog + 1'b1;
          end
        end
      end
      if (wd_ab_wr) begin
        ab_cnt <= AB_W'(ABORT_RST_CYCLES + AB_SET); wd_aborted_q <= 1'b1; ab_tmo_q <= 1'b0;
      end
      if (core_reset_i) begin
        ab_cnt <= '0; wd_aborted_q <= 1'b0; ab_wait <= 1'b0; ab_tmo_q <= 1'b0;
      end else if (wd_start) begin
        wd_aborted_q <= 1'b0;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wd_idx <= '0; wd_busy_q <= 1'b0; wd_fault_q <= 1'b0; wd_instr_q <= '0; wd_start <= 1'b0;
      for (int i = 0; i < WD_NWORDS; i++) wd_w[i] <= '0;
    end else begin
      wd_start <= 1'b0;
      if (wd_done) begin
        wd_busy_q <= 1'b0; wd_fault_q <= wd_fault; wd_instr_q <= wd_instr;
      end
      if (pw && poff == P_WDIST_REQ && !wd_busy_q && !aborting) begin
        if (wd_idx == 6'(WD_NWORDS - 1)) begin
          // last dword is the GO marker (value 0xffffffff)
          wd_idx <= '0;
          if (d_wdata == 32'hffff_ffff) begin
            wd_busy_q <= 1'b1; wd_fault_q <= 1'b0; wd_start <= 1'b1;
          end
        end else begin
          wd_w[wd_idx] <= d_wdata;
          wd_idx <= wd_idx + 6'd1;
        end
      end
      // CORE_RESET aborts a running dispatch (the SP side produces no wd_done for it): drop the request state,
      // so WDIST_STATUS.busy is 0 again when the firmware restarts.  Last, so it has priority.
      // WDIST_CTRL.ABORT does the same, from the write on: the SP is reset a clock later.
      if (core_reset_i || wd_ab_wr || wd_abort_o) begin
        wd_idx <= '0; wd_busy_q <= 1'b0; wd_fault_q <= 1'b0; wd_instr_q <= '0; wd_start <= 1'b0;
      end
    end
  end

  // owner-change scrub (scheme A, conditional): a dispatch scrubs when its owner is not the previous
  // dispatch's -- the VMID changed, an MMU CTX_CTRL write of the VMID happened since (ctx_enable / ctx_disable of a
  // reused VMID: vm_create / vm_destroy) or nothing has been dispatched since reset / CORE_RESET (SP RAMs keep their
  // contents).  The decision is taken at GO and latched (wd_req must stay stable while the dispatch runs).
  logic [15:0] scr_pend;
  logic [3:0]  last_vmid;
  logic        last_vld, wd_scrub_q;
  wire         wd_go    = pw && poff == P_WDIST_REQ && !wd_busy_q && !aborting && wd_idx == 6'(WD_NWORDS - 1) &&
                          d_wdata == 32'hffff_ffff;
  wire  [3:0]  go_vmid  = wd_w[0][3:0];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      scr_pend <= '0; last_vmid <= '0; last_vld <= 1'b0; wd_scrub_q <= 1'b0;
    end else begin
      if (wd_go) begin
        wd_scrub_q <= scr_pend[go_vmid] || !last_vld || (go_vmid != last_vmid);
        last_vmid <= go_vmid; last_vld <= 1'b1;
      end
      scr_pend <= (wd_go ? (scr_pend & ~(16'd1 << go_vmid)) : scr_pend) | scrub_set_i;   // a set in the GO cycle is kept
      if (core_reset_i) last_vld <= 1'b0;
    end
  end

  always_comb begin
    wd_req = '0;
    wd_req.scrub        = SCRUB && wd_scrub_q;
    wd_req.vmid         = wd_w[0][3:0];
    wd_req.num_vregs    = {8'h0, wd_w[0][31:8]};
    wd_req.shared_bytes = wd_w[1];
    wd_req.code_va      = {wd_w[3], wd_w[2]};
    wd_req.code_base    = {wd_w[5], wd_w[4]};
    wd_req.scratch_va   = {wd_w[7], wd_w[6]};
    wd_req.scratch_bytes = wd_w[8];
    for (int i = 0; i < 3; i++) begin
      wd_req.grid[i] = wd_w[9 + i];
      wd_req.wg[i]   = wd_w[12 + i];
    end
    for (int i = 0; i < WD_NUREGS; i++) wd_req.uregs[i] = wd_w[15 + i];
  end

  // ---------------------------------------------------------------------------
  // simple registers / strobes
  assign ev_clr_o  = (pw && poff == P_EVENT) ? d_wdata[3:0] : 4'h0;
  assign irq_set_o = (pw && poff == P_IRQ_SET) ? d_wdata : 32'h0;
  // 1 ms tick: sets a sticky pending bit that wakes the Ibex (irq_timer_i); the firmware clears it through P_TICK.
  // Without it a firmware sleeping in wfi never advances CP_HEARTBEAT (wants >= 1 per ms).
  logic [31:0] tick_cnt;
  logic        tick_q;
  assign tick_o = tick_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tick_cnt <= '0; tick_q <= 1'b0;
    end else begin
      if (TICK_CYCLES != 0) begin
        if (tick_cnt >= 32'(TICK_CYCLES - 1)) begin
          tick_cnt <= '0; tick_q <= 1'b1;
        end else begin
          tick_cnt <= tick_cnt + 32'd1;
        end
      end
      if (pw && poff == P_TICK && d_wdata[0]) tick_q <= 1'b0;
    end
  end
  assign sp_inv_o    = (pw && poff == P_SP_INV) ? d_wdata[2:0] : 3'h0;
  assign dbg_valid_o = pw && poff == P_DEBUG;
  assign dbg_code_o  = d_wdata;

  // ---------------------------------------------------------------------------
  // read mux
  logic [31:0] rmux;
  always_comb begin
    rmux = 32'h0;
    if (in_reg) begin
      rmux = fw_rdata;
    end else if (in_port) begin
      unique case (poff)
        P_DOORBELL:      rmux = wptr_i;
        P_EVENT:         rmux = {28'h0, ev_pend_i};
        P_TIME_LO:       rmux = time_q[31:0];
        P_TIME_HI:       rmux = time_q[63:32];
        P_ENGINES_IDLE:  rmux = {30'h0, ~copy_busy, sp_idle_i};
        P_MEM_ADDR_LO:   rmux = mem_addr[31:0];
        P_MEM_ADDR_HI:   rmux = {16'h0, mem_addr[47:32]};
        P_MEM_VMID:      rmux = {28'h0, mem_vmid};
        P_MEM_STATUS:    rmux = {30'h0, mem_fault, mem_busy};
        P_MEM_DATA:      rmux = mem_data[31:0];
        P_MEM_DATA_HI:   rmux = mem_data[63:32];
        P_WDIST_STATUS:  rmux = {27'h0, ab_tmo_q, wd_aborted_q, aborting, wd_fault_q, wd_busy_q};
        P_WDIST_INSTR_LO: rmux = wd_instr_q[31:0];
        P_WDIST_INSTR_HI: rmux = wd_instr_q[63:32];
        P_COPY_STATUS:   rmux = {30'h0, copy_fault, copy_busy};
        P_TICK:          rmux = {31'h0, tick_q};
        default:         rmux = 32'h0;
      endcase
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      d_rvalid <= 1'b0; d_rdata <= '0;
    end else begin
      d_rvalid <= d_req;
      if (d_req && !d_we) d_rdata <= rmux;
    end
  end
endmodule
