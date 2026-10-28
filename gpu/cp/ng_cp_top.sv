// ng_cp_top.sv -- CP subsystem: hardware register block + Ibex private
// port + Copy engine + CPU socket.
//
//   host (BAR0)  -> ng_cp_regs  <- register window ->  ng_cp_port <- Ibex data bus
//                                                       |-- memory port (MMU/L2/C3 adapter)
//                                                       |-- work-distributor port
//                                                       `-- Copy engine
//
// The CPU socket is the behavioural DPI stub in simulation (ng_cp_cpu_stub);
// for the real core define NG_CP_IBEX and add third_party/ibex (see
// ng_cp_ibex_wrap.sv).
module ng_cp_top
  import ng_cp_pkg::*;
#(
  parameter logic [31:0] TS_FREQ_HZ = 32'd100_000_000,
  parameter int unsigned TICK_CYCLES = 0,       // clocks per firmware heartbeat tick (1 ms); 0 = none (older testbenches)
  parameter int unsigned IBEX_QUIET_WORD = 0,   // NG_CP_IBEX: DMEM word of ngfw_ibex_quiet (nm of the image)
  parameter              IBEX_IMAGE = "fw_imem.hex",  // NG_CP_IBEX simulation IMEM preload (BOOT_LOADER = 0)
  parameter bit          BOOT_LOADER = 1'b0,          // NG_CP_IBEX: 1 = hardware boot loader ng_cp_boot loads the NGCF
  parameter bit          COPY_BULK = 1'b0,            // Copy engine hands whole commands to the clk_l2 DMA (cb_*)
                                                      // image from CP_FW_BASE/SIZE (hardware default); 0 = IMEM preload
  parameter bit          ABORT_FAB_WAIT = 1'b0,       // WDIST abort waits for fab_busy_i (ng_cp_port); 0 =
  parameter int unsigned ABORT_WDOG_LOG2 = 19,        // abort watchdog 2**N clocks (ABORT_FAB_WAIT = 1 only)
  parameter bit          SCRUB = 1'b0                 // ng_cp_port generates wd_req.scrub
) (
  input  logic        clk,
  input  logic        rst_n,

  // host BAR0 register bus (CP page, IRQ regs, timestamp, doorbell page)
  input  logic        host_wr,
  input  logic        host_rd,
  input  logic [12:0] host_addr,
  input  logic [31:0] host_wdata,
  output logic [31:0] host_rdata,

  // device interrupt
  input  logic [31:0] irq_ext_i,      // other IRQ_STATUS sources (bit 2 = MMU fault)
  output logic        rm_irq_o,

  // memory request port (one outstanding request; response >= 1 cycle later)
  output logic        m_req_valid,
  input  logic        m_req_ready,
  output mem_req_t    m_req,
  input  logic        m_rsp_valid,
  input  logic        m_rsp_fault,
  input  logic [63:0] m_rsp_data,

  // COPY_BULK: whole COPY / FILL commands for the clk_l2 DMA (ng_cp_copy b_*)
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

  // work distributor
  output logic        wd_start,
  output wd_req_t     wd_req,
  input  logic        wd_done,
  input  logic        wd_fault,
  input  logic [63:0] wd_instr,
  input  logic        sp_idle_i,
  output logic        wd_abort_o,       // WDIST_CTRL.ABORT, to be OR-ed into the SP's core_reset
  input  logic        fab_busy_i,       // SP fabric port busy (used with ABORT_FAB_WAIT); tie 0 without an SP
  input  logic [15:0] scrub_set_i,      // per-VMID pulse, MMU CTX_CTRL write (this clock domain); tie 0 without an SP
  output logic [2:0]  sp_inv_o,         // CACHE packet L1D / K / I$ invalidate pulses towards the SP

  // debug / settle detection (tb)
  output logic        dbg_valid_o,
  output logic [31:0] dbg_code_o,
  output logic        cpu_quiet_o,
  output logic        core_reset_o
);
  logic        p_req_valid, p_req_ready;       // ng_cp_port's side of the memory port (the boot loader can own it)
  mem_req_t    p_req;
  logic        p_rsp_valid;
  logic [63:0] fw_base;
  logic [31:0] fw_size;
  logic        boot_err_vld;
  logic [7:0]  boot_err;
  logic        d_req, d_gnt, d_rvalid, d_we, d_err;
  logic [3:0]  d_be;
  logic [31:0] d_addr, d_wdata, d_rdata;

  logic        fw_wr;
  logic [11:0] fw_waddr, fw_raddr;
  logic [31:0] fw_wdata, fw_rdata, wptr;
  logic [3:0]  ev_pend, ev_clr;
  logic [31:0] irq_set_p, irq_set;
  logic        cpu_irq;

  ng_cp_regs #(.TS_FREQ_HZ(TS_FREQ_HZ)) u_regs (
    .clk, .rst_n,
    .host_wr, .host_rd, .host_addr, .host_wdata, .host_rdata,
    .fw_wr, .fw_waddr, .fw_wdata, .fw_raddr, .fw_rdata,
    .wptr_o(wptr), .ev_pend_o(ev_pend), .ev_clr_i(ev_clr),
    .irq_set_i(irq_set), .irq_ext_i,
    .fw_base_o(fw_base), .fw_size_o(fw_size), .boot_err_vld_i(boot_err_vld), .boot_err_i(boot_err),
    .core_reset_o, .cpu_irq_o(cpu_irq), .rm_irq_o
  );

  logic cpu_tick, port_busy;
  ng_cp_port #(.TICK_CYCLES(TICK_CYCLES), .COPY_BULK(COPY_BULK), .ABORT_FAB_WAIT(ABORT_FAB_WAIT),
               .ABORT_WDOG_LOG2(ABORT_WDOG_LOG2), .SCRUB(SCRUB)) u_port (
    .clk, .rst_n, .core_reset_i(core_reset_o), .busy_o(port_busy),
    .d_req, .d_gnt, .d_rvalid, .d_we, .d_be, .d_addr, .d_wdata, .d_rdata, .d_err,
    .fw_wr, .fw_waddr, .fw_wdata, .fw_raddr, .fw_rdata,
    .wptr_i(wptr), .ev_pend_i(ev_pend), .ev_clr_o(ev_clr), .irq_set_o(irq_set_p),
    .m_req_valid(p_req_valid), .m_req_ready(p_req_ready), .m_req(p_req), .m_rsp_valid(p_rsp_valid),
    .m_rsp_fault, .m_rsp_data,
    .wd_start, .wd_req, .wd_done, .wd_fault, .wd_instr, .sp_idle_i, .wd_abort_o, .fab_busy_i, .scrub_set_i, .sp_inv_o,
    .cb_valid, .cb_fill, .cb_vmid, .cb_src, .cb_dst, .cb_size, .cb_fillv, .cb_abort, .cb_done, .cb_fault,
    .tick_o(cpu_tick),
    .dbg_valid_o, .dbg_code_o
  );

`ifdef NG_CP_IBEX
  logic        ibex_sleep;
  logic        ld_we, ld_dmem;
  logic [31:0] ld_addr, ld_data, boot_addr;
  logic        boot_busy, boot_ok;
  logic        b_req_valid;
  mem_req_t    b_req;
  // CORE_RESET aborts the Copy engine, but a request it (or a MEM_* access) already issued is still in flight: the
  // loader starts only once the port has drained, so that response cannot land in the loader's stream.
  logic boot_hold;
  always_ff @(posedge clk or negedge rst_n)
    if (!rst_n)            boot_hold <= 1'b0;
    else if (core_reset_o) boot_hold <= 1'b1;
    else if (!port_busy)   boot_hold <= 1'b0;
  if (BOOT_LOADER) begin : g_boot
    ng_cp_boot u_boot (
      .clk, .rst_n, .run_i(~core_reset_o & ~boot_hold), .fw_base_i(fw_base), .fw_size_i(fw_size),
      .m_req_valid_o(b_req_valid), .m_req_ready_i(m_req_ready), .m_req_o(b_req),
      .m_rsp_valid_i(m_rsp_valid), .m_rsp_fault_i(m_rsp_fault), .m_rsp_data_i(m_rsp_data),
      .ld_we_o(ld_we), .ld_dmem_o(ld_dmem), .ld_addr_o(ld_addr), .ld_data_o(ld_data),
      .boot_addr_o(boot_addr), .busy_o(boot_busy), .ok_o(boot_ok), .err_vld_o(boot_err_vld), .err_o(boot_err)
    );
  end else begin : g_noboot
    assign ld_we = 1'b0;  assign ld_dmem = 1'b0;  assign ld_addr = 32'h0;  assign ld_data = 32'h0;
    assign boot_addr = 32'h0;  assign boot_busy = 1'b0;  assign boot_ok = 1'b1;  assign b_req_valid = 1'b0;
    assign b_req = '0;  assign boot_err_vld = 1'b0;  assign boot_err = 8'h0;
  end
  // memory port: the loader owns it while it runs (the Ibex and the Copy engine are idle then)
  assign m_req_valid = boot_busy ? b_req_valid : p_req_valid;
  assign m_req       = boot_busy ? b_req : p_req;
  assign p_req_ready = ~boot_busy & m_req_ready;
  assign p_rsp_valid = m_rsp_valid & ~boot_busy;
  // a boot error raises IRQ_CP_FW (IRQ_STATUS bit 3) so the kernel does not have to wait out the READY timeout
  assign irq_set = irq_set_p | (boot_err_vld ? 32'h8 : 32'h0);

  ng_cp_ibex_wrap #(.QUIET_WORD(IBEX_QUIET_WORD), .IMAGE_FILE(IBEX_IMAGE)) u_cpu (
    .clk, .rst_n(rst_n & ~core_reset_o & boot_ok), .irq_i(cpu_irq), .timer_irq_i(cpu_tick),
    .ld_we_i(ld_we), .ld_dmem_i(ld_dmem), .ld_addr_i(ld_addr), .ld_data_i(ld_data), .boot_addr_i(boot_addr),
    .d_req, .d_gnt, .d_rvalid, .d_we, .d_be, .d_addr, .d_wdata, .d_rdata, .d_err,
    .quiet_o(cpu_quiet_o), .sleep_o(ibex_sleep)
  );
`else
  assign m_req_valid = p_req_valid;
  assign m_req       = p_req;
  assign p_req_ready = m_req_ready;
  assign p_rsp_valid = m_rsp_valid;
  assign irq_set     = irq_set_p;
  assign boot_err_vld = 1'b0;
  assign boot_err     = 8'h0;
  ng_cp_cpu_stub u_cpu (
    .clk, .rst_n(rst_n & ~core_reset_o), .irq_i(cpu_irq),
    .d_req, .d_gnt, .d_rvalid, .d_we, .d_be, .d_addr, .d_wdata, .d_rdata, .d_err,
    .quiet_o(cpu_quiet_o)
  );
`endif
endmodule
