// ng_cp_regs.sv -- CP hardware register block.
//
// The BAR0 CP page (0x0100..0x01FF) plus IRQ_STATUS/IRQ_ENABLE, the
// timestamp registers and DOORBELL0 are served here, in hardware: host
// reads and writes never wait for the Ibex.  Ownership (7.9 item 2):
//   host-owned RW   RING_BASE/SIZE_LOG2, CTRL, FW_BASE/SIZE, RPTR_WB
//   hardware RO     RING_WPTR (doorbell latch), STATUS.FW_READY (= FW_STATUS.READY)
//   firmware RO     RPTR, STATUS[3:0], ERROR*, HEARTBEAT, CMD_/FW_VERSION,
//                   FW_STATUS, TIMESTAMP_LO/HI -- written by the Ibex through
//                   the register window (fw_wr/fw_waddr), cleared to their
//                   power-on values while CORE_RESET is 1.
// Host writes that need the firmware raise EVENT bits (level-pending, W1C by
// the firmware through ev_clr); their OR drives the Ibex interrupt line.
//
// event bit layout; WPTR latch and event bits
// cleared at the CORE_RESET rising edge; CP_ERROR:= 0 also clears
// ERROR_RPTR/INFO/IB_OFFSET (the firmware's reset publishes only CP_ERROR);
// TIMESTAMP_LO/HI are plain firmware-published values.
module ng_cp_regs
  import ng_cp_pkg::*;
#(
  parameter logic [31:0] TS_FREQ_HZ = 32'd100_000_000
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- host side (BAR0): single-cycle register bus, combinational read ----
  input  logic        host_wr,
  input  logic        host_rd,        // read strobe (TIMESTAMP_LO latches HI)
  input  logic [12:0] host_addr,
  input  logic [31:0] host_wdata,
  output logic [31:0] host_rdata,

  // ---- firmware register window (Ibex data port, offset = BAR0 offset) ----
  input  logic        fw_wr,
  input  logic [11:0] fw_waddr,
  input  logic [31:0] fw_wdata,
  input  logic [11:0] fw_raddr,
  output logic [31:0] fw_rdata,

  // ---- private-port connections ----
  output logic [31:0] wptr_o,          // DOORBELL (hardware WPTR latch)
  output logic [3:0]  ev_pend_o,       // pending host events
  input  logic [3:0]  ev_clr_i,        // W1C pulse from the firmware
  input  logic [31:0] irq_set_i,       // IRQ_SET pulse bits (firmware)
  input  logic [31:0] irq_ext_i,       // other device sources (e.g. MMU fault bit 2)

  // ---- boot loader (ng_cp_boot): image location out, BOOT_ERR in ----
  output logic [63:0] fw_base_o,       // CP_FW_BASE
  output logic [31:0] fw_size_o,       // CP_FW_SIZE
  input  logic        boot_err_vld_i,  // loader decided BOOT_ERR (one cycle): FW_STATUS[15:8]:= boot_err_i
  input  logic [7:0]  boot_err_i,

  // ---- outputs ----
  output logic        core_reset_o,    // holds the Ibex in reset
  output logic        cpu_irq_o,       // Ibex external interrupt (level)
  output logic        rm_irq_o         // device MSI request (IRQ_STATUS & IRQ_ENABLE)
);
  // host-owned
  logic [63:0] ring_base, rptr_wb;
  logic [31:0] ring_size, fw_size, fw_base_lo, fw_base_hi;
  logic        c_enable, c_halt, c_core_reset;
  logic [31:0] irq_status, irq_enable;
  // hardware
  logic [31:0] wptr;
  logic [3:0]  ev_pend;
  // firmware-owned
  logic [31:0] f_rptr, f_status, f_error, f_err_rptr, f_err_info, f_err_ib;
  logic [31:0] f_heartbeat, f_cmd_ver, f_fw_ver, f_fw_status, f_ts_lo, f_ts_hi;
  logic [31:0] ts_hi_latch;
  logic        core_reset_q;

  assign core_reset_o = c_core_reset;
  assign fw_base_o    = {fw_base_hi, fw_base_lo};
  assign fw_size_o    = fw_size;
  assign wptr_o       = wptr;
  assign ev_pend_o    = ev_pend;
  assign cpu_irq_o    = |ev_pend;
  assign rm_irq_o     = |(irq_status & irq_enable);

  wire core_rise = c_core_reset & ~core_reset_q;

  // ---- read mux shared by host and firmware windows ---------------------------
  function automatic logic [31:0] rd(input logic [12:0] a);
    logic [31:0] v;
    v = 32'h0;
    unique case (a)
      R_IRQ_STATUS:   v = irq_status;
      R_IRQ_ENABLE:   v = irq_enable;
      R_TS_LO:        v = f_ts_lo;
      R_TS_HI:        v = ts_hi_latch;
      R_TS_FREQ:      v = TS_FREQ_HZ;
      R_RING_BASE_LO: v = ring_base[31:0];
      R_RING_BASE_HI: v = ring_base[63:32];
      R_RING_SIZE:    v = ring_size;
      R_RING_RPTR:    v = f_rptr;
      R_RING_WPTR:    v = wptr;
      R_CTRL:         v = {28'h0, c_core_reset, c_halt, 1'b0, c_enable};
      R_STATUS:       v = {27'h0, f_fw_status[0], f_status[3:0]};
      R_ERROR:        v = f_error;
      R_ERROR_RPTR:   v = f_err_rptr;
      R_ERROR_INFO:   v = f_err_info;
      R_ERROR_IB_OFF: v = f_err_ib;
      R_HEARTBEAT:    v = f_heartbeat;
      R_CMD_VERSION:  v = f_cmd_ver;
      R_FW_VERSION:   v = f_fw_ver;
      R_FW_STATUS:    v = f_fw_status;
      R_FW_BASE_LO:   v = fw_base_lo;
      R_FW_BASE_HI:   v = fw_base_hi;
      R_FW_SIZE:      v = fw_size;
      R_RPTR_WB_LO:   v = rptr_wb[31:0];
      R_RPTR_WB_HI:   v = rptr_wb[63:32];
      default:        v = 32'h0;
    endcase
    return v;
  endfunction

  // always_comb (not assign): the function reads module state that is not in its argument list
  always_comb begin
    host_rdata = rd(host_addr);
    fw_rdata   = rd({1'b0, fw_raddr});
  end

  // ---- state ------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ring_base <= '0; rptr_wb <= '0;
      ring_size <= '0; fw_size <= '0; fw_base_lo <= '0; fw_base_hi <= '0;
      c_enable <= 1'b0; c_halt <= 1'b0; c_core_reset <= 1'b1;   // CORE_RESET resets to 1
      core_reset_q <= 1'b1;
      irq_status <= '0; irq_enable <= '0;
      wptr <= '0; ev_pend <= '0;
      f_rptr <= '0; f_status <= 32'h1; f_error <= '0; f_err_rptr <= '0; f_err_info <= '0;
      f_err_ib <= '0; f_heartbeat <= '0; f_cmd_ver <= '0; f_fw_ver <= '0; f_fw_status <= '0;
      f_ts_lo <= '0; f_ts_hi <= '0; ts_hi_latch <= '0;
    end else begin
      core_reset_q <= c_core_reset;

      // events: firmware clears, host writes set (set wins)
      ev_pend <= ev_pend & ~ev_clr_i;

      // IRQ_STATUS: W1C from the host, set from the firmware / other sources
      irq_status <= irq_status | irq_set_i | irq_ext_i;
      if (host_wr && host_addr == R_IRQ_STATUS)
        irq_status <= (irq_status & ~host_wdata) | irq_set_i | irq_ext_i;

      // firmware-owned registers: cleared to power-on values while CORE_RESET=1
      if (c_core_reset) begin
        f_rptr <= '0; f_status <= 32'h1; f_error <= '0; f_err_rptr <= '0; f_err_info <= '0;
        f_err_ib <= '0; f_heartbeat <= '0; f_cmd_ver <= '0; f_fw_ver <= '0; f_fw_status <= '0;
        f_ts_lo <= '0; f_ts_hi <= '0;
      end else if (fw_wr) begin
        unique case ({1'b0, fw_waddr})
          R_RING_RPTR:    f_rptr <= fw_wdata;
          R_STATUS:       f_status <= fw_wdata;
          R_ERROR: begin
            f_error <= fw_wdata;
            if (fw_wdata == 32'h0) begin
              f_err_rptr <= '0; f_err_info <= '0; f_err_ib <= '0;
            end
          end
          R_ERROR_RPTR:   f_err_rptr <= fw_wdata;
          R_ERROR_INFO:   f_err_info <= fw_wdata;
          R_ERROR_IB_OFF: f_err_ib <= fw_wdata;
          R_HEARTBEAT:    f_heartbeat <= fw_wdata;
          R_CMD_VERSION:  f_cmd_ver <= fw_wdata;
          R_FW_VERSION:   f_fw_ver <= fw_wdata;
          R_FW_STATUS:    f_fw_status <= fw_wdata;
          R_TS_LO:        f_ts_lo <= fw_wdata;
          R_TS_HI:        f_ts_hi <= fw_wdata;
          default: ;
        endcase
      end

      // boot loader error (the Ibex never ran, so nobody else writes FW_STATUS)
      if (boot_err_vld_i && !c_core_reset) f_fw_status[15:8] <= boot_err_i;

      // CORE_RESET rising edge: drop the doorbell latch and stale events
      if (core_rise) begin
        wptr <= '0;
        ev_pend <= '0;
      end

      // TIMESTAMP_LO read latches HI (consistent 64-bit read)
      if (host_rd && host_addr == R_TS_LO) ts_hi_latch <= f_ts_hi;

      // host writes
      if (host_wr) begin
        unique case (host_addr)
          R_IRQ_ENABLE:   irq_enable <= host_wdata;
          R_RING_BASE_LO: begin ring_base[31:0] <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_RING_BASE_HI: begin ring_base[63:32] <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_RING_SIZE:    begin ring_size <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_FW_BASE_LO:   begin fw_base_lo <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_FW_BASE_HI:   begin fw_base_hi <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_FW_SIZE:      begin fw_size <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_RPTR_WB_LO:   begin rptr_wb[31:0] <= host_wdata & ~32'h2; ev_pend[EV_RING] <= 1'b1; end
          R_RPTR_WB_HI:   begin rptr_wb[63:32] <= host_wdata; ev_pend[EV_RING] <= 1'b1; end
          R_CTRL: begin
            c_enable     <= host_wdata[CTRL_ENABLE];
            c_halt       <= host_wdata[CTRL_HALT_REQ];
            c_core_reset <= host_wdata[CTRL_CORE_RESET];
            ev_pend[EV_CTRL] <= 1'b1;
            if (host_wdata[CTRL_RESET]) ev_pend[EV_RESET] <= 1'b1;
          end
          R_DOORBELL0: begin wptr <= host_wdata; ev_pend[EV_DOORBELL] <= 1'b1; end
          default: ;   // RO / undefined offsets: ignored (7.9 item 1)
        endcase
      end
    end
  end
endmodule
