// ng_prim_mmcm.sv -- system MMCM wrapper for the static shell (P0-11).
//
// Generates the three RM clocks from the 299.825 MHz SI5381 reference
// (DFX_BOUNDARY.md section 2.1) and exposes the DRP for runtime retuning
// through the shell's CLK_DRP_ADDR/DATA registers (section 2.4).
//
// This is the first ng_prim_* in the repository that maps to a real vendor
// primitive.  House rule (DFX_BOUNDARY.md section 8): vendor primitives are
// instantiated only inside ng_prim_* wrappers, never elsewhere in the shell
// or RM RTL.  The wrapper therefore carries two variants, selected by the
// SYNTHESIS macro (auto-defined by Vivado synth_design; simulators see the
// behavioral model):
//   * SYNTHESIS: one MMCME4_ADV with the frozen clock-table ratios --
//     CLKIN1 299.825 MHz, M = 5.0, D = 1 -> VCO 1499.125 MHz, CLKOUT0/1/2
//     divide 3.75/6/15 -> 399.7667/249.8542/99.9417 MHz.  The frequency
//     parameters below feed the behavioral model only; overriding them
//     desynchronizes simulation from synthesis, so board builds keep the
//     defaults and retune through the DRP instead.
//   * default: behavioral model.  Exact frequencies via absolute-scheduled
//     realtime half periods (timescale 1ns/1fs: rounding cannot accumulate,
//     the error is orders of magnitude below 0.01%).  While rst is high the
//     outputs stop (clamped low within one half period) and locked is 0;
//     after rst release, locked rises with the LOCK_REF_CYCLES-th clk_ref
//     edge (~1 us), synchronous to clk_ref.  The DRP is a 128 x 16-bit
//     register file: write on (drp_en & drp_we), read on (drp_en & !drp_we),
//     and drp_rdy pulses for exactly one drp_clk cycle, one cycle after
//     drp_en.  Writing divide registers does not change the model
//     frequencies -- on the real device it does; the driver verifies the
//     result with CLK_FREQ_*.
//
// DRP clocking: the DRP interface runs on drp_clk, separate from clk_ref, so
// the board top can drive it from clk_sh where ng_shell_ctrl's CLK_DRP
// registers live: a BAR1 write to CLK_DRP_ADDR with bit [31]/[30] set is the
// write/read strobe; the board top turns it into a single-cycle drp_en
// (together with drp_we and drp_addr from the register) with no clock-domain
// crossing, and merges drp_rdy/drp_do back into CLK_DRP_DATA [31]/[15:0].
// drp_clk must respect the DCLK frequency limit of the device data sheet
// (DS923); use a divided clk_sh if clk_sh itself is too fast.  ASIC mapping:
// a PLL with the same ratio table.
`timescale 1ns/1fs

module ng_prim_mmcm #(
  parameter real REF_CLK_MHZ = 299.825,  // SI5381 reference (clock table)
  parameter real CLKOUT0_MHZ = 399.767,  // nominal clk_core2x
  parameter real CLKOUT1_MHZ = 249.854,  // nominal clk_l2
  parameter real CLKOUT2_MHZ =  99.942   // nominal clk_ts
) (
  input  logic        clk_ref,  // reference clock, REF_CLK_MHZ
  input  logic        rst,      // asynchronous, active high (MMCME4 RST)
  output logic        locked,
  output logic        clk_out0, // clk_core2x
  output logic        clk_out1, // clk_l2
  output logic        clk_out2, // clk_ts
  // dynamic reconfiguration port, all drp_* synchronous to drp_clk
  input  logic        drp_clk,
  input  logic        drp_en,   // single-cycle strobe (DEN)
  input  logic        drp_we,   // 1 = write, 0 = read
  input  logic [6:0]  drp_addr,
  input  logic [15:0] drp_di,
  output logic [15:0] drp_do,
  output logic        drp_rdy   // single-cycle pulse, one cycle after drp_en
);

`ifdef SYNTHESIS

  logic clkfb;
  MMCME4_ADV #(
    .BANDWIDTH          ("OPTIMIZED"),
    .CLKFBOUT_MULT_F    (5.0),
    .CLKFBOUT_PHASE     (0.0),
    .DIVCLK_DIVIDE      (1),
    .CLKIN1_PERIOD      (1000.0 / REF_CLK_MHZ),  // ns, 3.335278
    .CLKOUT0_DIVIDE_F   (3.75),
    .CLKOUT0_DUTY_CYCLE (0.5),
    .CLKOUT0_PHASE      (0.0),
    .CLKOUT1_DIVIDE     (6),
    .CLKOUT1_DUTY_CYCLE (0.5),
    .CLKOUT1_PHASE      (0.0),
    .CLKOUT2_DIVIDE     (15),
    .CLKOUT2_DUTY_CYCLE (0.5),
    .CLKOUT2_PHASE      (0.0),
    .COMPENSATION       ("AUTO"),   // let the tool pick the compensation network
    .REF_JITTER1        (0.010),
    .STARTUP_WAIT       ("FALSE")
  ) u_mmcm (
    .CLKOUT0   (clk_out0),
    .CLKOUT1   (clk_out1),
    .CLKOUT2   (clk_out2),
    .CLKOUT3   (),                   // unused outputs stay open
    .CLKOUT4   (),
    .CLKOUT5   (),
    .CLKOUT6   (),
    .CLKOUT0B  (), .CLKOUT1B (), .CLKOUT2B (), .CLKOUT3B (),
    .CLKFBOUT  (clkfb),              // direct feedback; buffers go downstream
    .CLKFBOUTB (),
    .CLKFBIN   (clkfb),
    .CLKIN1    (clk_ref),
    .CLKIN2    (1'b0),
    .CLKINSEL  (1'b1),
    .CLKINSTOPPED (),
    .CLKFBSTOPPED (),
    .RST       (rst),
    .PWRDWN    (1'b0),
    .LOCKED    (locked),
    // DRP, fully wired (retuning, DFX_BOUNDARY.md section 2.4)
    .DADDR     (drp_addr),
    .DCLK      (drp_clk),
    .DEN       (drp_en),
    .DWE       (drp_we),
    .DI        (drp_di),
    .DO        (drp_do),
    .DRDY      (drp_rdy),
    // dynamic phase shift and clock divide request: not used
    .PSCLK     (), .PSEN (), .PSINCDEC (), .PSDONE (),
    .CDDCREQ   (), .CDDCDONE ()
  );

`else

  // ---------------------------------------------------------- output clocks
  // Exact frequencies: half periods in ns, every edge scheduled at the
  // absolute time (base + n*HP), so simulator time rounding never
  // accumulates (stays at ~1 fs per edge).
  localparam real HP0 = 500.0 / CLKOUT0_MHZ;
  localparam real HP1 = 500.0 / CLKOUT1_MHZ;
  localparam real HP2 = 500.0 / CLKOUT2_MHZ;
  localparam int  LOCK_REF_CYCLES = 300;  // ~1 us at 299.825 MHz

  always begin
    realtime base;
    int      n;
    clk_out0 = 1'b0;
    @(negedge rst);
    base = $realtime;
    n    = 0;
    forever begin
      n++;
      #(base + n * HP0 - $realtime);
      if (rst) break;          // stop: output clamps low within one HP0
      clk_out0 = ~clk_out0;
    end
  end

  always begin
    realtime base;
    int      n;
    clk_out1 = 1'b0;
    @(negedge rst);
    base = $realtime;
    n    = 0;
    forever begin
      n++;
      #(base + n * HP1 - $realtime);
      if (rst) break;
      clk_out1 = ~clk_out1;
    end
  end

  always begin
    realtime base;
    int      n;
    clk_out2 = 1'b0;
    @(negedge rst);
    base = $realtime;
    n    = 0;
    forever begin
      n++;
      #(base + n * HP2 - $realtime);
      if (rst) break;
      clk_out2 = ~clk_out2;
    end
  end

  // ---------------------------------------------------------- lock detector
  // Asynchronous drop on rst; after release, locked rises with the
  // LOCK_REF_CYCLES-th clk_ref edge (~1 us), synchronous to clk_ref.
  int lock_cnt;
  always begin
    locked = 1'b0;
    @(negedge rst);
    lock_cnt = 0;
    forever begin
      @(posedge clk_ref or posedge rst);
      if (rst) break;
      if (lock_cnt < LOCK_REF_CYCLES) lock_cnt++;
      if (lock_cnt == LOCK_REF_CYCLES) locked = 1'b1;
    end
  end

  // ---------------------------------------------------------- DRP model
  // 128 x 16-bit register file (the primitive's DRP address space).  Write
  // on (drp_en & drp_we), read on (drp_en & !drp_we); drp_rdy pulses one
  // cycle after drp_en.  Contents survive rst: the real device keeps its
  // DRP configuration across an MMCM reset.  Divide writes do not retune the
  // model (see header).
  logic [15:0] drp_mem [0:127];
  logic        drdy_q;
  logic [15:0] do_q;
  initial begin
    for (int i = 0; i < 128; i++) drp_mem[i] = '0;
    drdy_q = 1'b0;
    do_q   = '0;
  end
  always @(posedge drp_clk) begin
    drdy_q <= drp_en;
    if (drp_en && drp_we)       drp_mem[drp_addr] <= drp_di;
    else if (drp_en && !drp_we) do_q <= drp_mem[drp_addr];
  end
  assign drp_do  = do_q;
  assign drp_rdy = drdy_q;

`endif

endmodule
