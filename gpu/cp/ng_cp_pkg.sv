// ng_cp_pkg.sv -- CP subsystem constants (BAR0 CP page offsets, private
// port map, work-distributor request).  Mirrors
// `registers` (v0.2) and fw/cp/src/ngfw_ibex_port.h; tb_cp runs the real
// firmware against this package, so a mismatch shows up as a test failure.
// tentative as a whole:
package ng_cp_pkg;

  // ---- BAR0 offsets owned by the CP block  -------------
  localparam logic [12:0] R_IRQ_STATUS    = 13'h0040;
  localparam logic [12:0] R_IRQ_ENABLE    = 13'h0044;
  localparam logic [12:0] R_TS_LO         = 13'h0050;
  localparam logic [12:0] R_TS_HI         = 13'h0054;
  localparam logic [12:0] R_TS_FREQ       = 13'h0058;
  localparam logic [12:0] R_RING_BASE_LO  = 13'h0100;
  localparam logic [12:0] R_RING_BASE_HI  = 13'h0104;
  localparam logic [12:0] R_RING_SIZE     = 13'h0108;
  localparam logic [12:0] R_RING_RPTR     = 13'h010c;
  localparam logic [12:0] R_RING_WPTR     = 13'h0110;
  localparam logic [12:0] R_CTRL          = 13'h0114;
  localparam logic [12:0] R_STATUS        = 13'h0118;
  localparam logic [12:0] R_ERROR         = 13'h011c;
  localparam logic [12:0] R_ERROR_RPTR    = 13'h0120;
  localparam logic [12:0] R_ERROR_INFO    = 13'h0124;
  localparam logic [12:0] R_ERROR_IB_OFF  = 13'h0128;
  localparam logic [12:0] R_HEARTBEAT     = 13'h012c;
  localparam logic [12:0] R_CMD_VERSION   = 13'h0130;
  localparam logic [12:0] R_FW_VERSION    = 13'h0134;
  localparam logic [12:0] R_FW_STATUS     = 13'h0138;
  localparam logic [12:0] R_FW_BASE_LO    = 13'h0140;
  localparam logic [12:0] R_FW_BASE_HI    = 13'h0144;
  localparam logic [12:0] R_FW_SIZE       = 13'h0148;
  localparam logic [12:0] R_RPTR_WB_LO    = 13'h0150;
  localparam logic [12:0] R_RPTR_WB_HI    = 13'h0154;
  localparam logic [12:0] R_DOORBELL0     = 13'h1000;

  // CP_CTRL bits
  localparam int CTRL_ENABLE = 0, CTRL_RESET = 1, CTRL_HALT_REQ = 2, CTRL_CORE_RESET = 3;

  // Host events (EVENT register bits item 3)
  localparam int EV_DOORBELL = 0, EV_CTRL = 1, EV_RING = 2, EV_RESET = 3;

  // ---- Private port (Ibex side), byte offsets from PORT_BASE -----------------
  localparam logic [7:0] P_DOORBELL      = 8'h00;
  localparam logic [7:0] P_IRQ_SET       = 8'h04;
  localparam logic [7:0] P_EVENT         = 8'h08;
  localparam logic [7:0] P_TIME_LO       = 8'h0c;
  localparam logic [7:0] P_TIME_HI       = 8'h10;
  localparam logic [7:0] P_ENGINES_IDLE  = 8'h14;
  localparam logic [7:0] P_MEM_ADDR_LO   = 8'h20;
  localparam logic [7:0] P_MEM_ADDR_HI   = 8'h24;
  localparam logic [7:0] P_MEM_VMID      = 8'h28;
  localparam logic [7:0] P_MEM_CTRL      = 8'h2c;
  localparam logic [7:0] P_MEM_STATUS    = 8'h30;
  localparam logic [7:0] P_MEM_DATA      = 8'h34;
  localparam logic [7:0] P_MEM_DATA_HI   = 8'h38;
  localparam logic [7:0] P_WDIST_REQ     = 8'h40;
  localparam logic [7:0] P_WDIST_STATUS  = 8'h44;
  localparam logic [7:0] P_WDIST_INSTR_LO = 8'h48;
  localparam logic [7:0] P_WDIST_INSTR_HI = 8'h4c;
  localparam logic [7:0] P_DEBUG         = 8'h50;
  localparam logic [7:0] P_WDIST_CTRL    = 8'h54;   // write bit0 = ABORT the running dispatch
  localparam logic [7:0] P_SP_INV        = 8'h58;   // write: cache invalidate of the SP(s): bit0 INV_L1, bit1 INV_K, bit2 INV_I (1-cycle pulses)
  localparam logic [7:0] P_COPY_SRC_LO   = 8'h60;
  localparam logic [7:0] P_COPY_SRC_HI   = 8'h64;
  localparam logic [7:0] P_COPY_DST_LO   = 8'h68;
  localparam logic [7:0] P_COPY_DST_HI   = 8'h6c;
  localparam logic [7:0] P_COPY_SIZE_LO  = 8'h70;
  localparam logic [7:0] P_COPY_SIZE_HI  = 8'h74;
  localparam logic [7:0] P_COPY_VMID     = 8'h78;
  localparam logic [7:0] P_COPY_FILL     = 8'h7c;
  localparam logic [7:0] P_COPY_CTRL     = 8'h80;
  localparam logic [7:0] P_COPY_STATUS   = 8'h84;
  localparam logic [7:0] P_TICK          = 8'h90;   // read: bit0 = tick pending; write 1 to bit0 clears

  // Address map seen by the Ibex data port (upper bits select the region)
  localparam logic [31:0] REG_BASE  = 32'h4000_0000;   // BAR0 CP page window, 4 KiB
  localparam logic [31:0] PORT_BASE = 32'h4002_0000;   // private port, 4 KiB

  // ---- Work-distributor request (42-dword sequence, last dword = GO) ---------
  localparam int WD_NWORDS = 42;
  localparam int WD_NUREGS = 26;

  // wg_first / wg_step (not part of the 42-dword sequence; ng_cp_port leaves them 0.  A
  // multi-SP core (rtl/gpu/core/ng_core_wd) sets them per SP: that SP's ng_wd runs only the workgroups whose linear
  // id (x fastest) is wg_first, wg_first + wg_step, ...  (0 / 0 = every workgroup, the single-SP behaviour).
  // scrub (internal, not part of the 42-dword sequence; ng_cp_port sets it when the
  // dispatch's owner differs from the previous dispatch's (MMU CTX write of the VMID since, VMID changed, first dispatch
  // after reset); ng_wd then clears v registers / shared memory and invalidates the L1D before the first workgroup.
  typedef struct packed {
    logic                        scrub;
    logic [7:0]                  wg_first;
    logic [7:0]                  wg_step;
    logic [31:0]                 shared_bytes;
    logic [31:0]                 num_vregs;
    logic [3:0]                  vmid;
    logic [63:0]                 code_va;
    logic [63:0]                 code_base;
    logic [63:0]                 scratch_va;
    logic [31:0]                 scratch_bytes;
    logic [2:0][31:0]            grid;
    logic [2:0][31:0]            wg;
    logic [WD_NUREGS-1:0][31:0]  uregs;
  } wd_req_t;

  // ---- Memory request / response (CP -> MMU/L2/C3 adapter) --------------------
  // One outstanding request per master; addr is naturally aligned to the
  // access size (1/2/4/8 bytes); wstrb selects the bytes inside the 64-bit lane.
  typedef struct packed {
    logic        write;
    logic [3:0]  vmid;      // 0 = physical address
    logic [47:0] addr;      // 8-byte aligned (lane select is in wstrb/rsize)
    logic [7:0]  wstrb;     // write: byte enables, read: bytes wanted
    logic [63:0] wdata;
  } mem_req_t;

endpackage
