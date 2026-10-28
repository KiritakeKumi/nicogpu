// ng_clk_meas.sv -- reciprocal frequency meter for the shell CLK_FREQ_*
// registers (DFX_BOUNDARY.md section 2.4).
//
// The clk_ref domain (clk_sh, 250 MHz from PCIe) opens a REF_TICKS-period
// gate window; the clk_meas domain counts its own rising edges while the
// 2-FF synchronized window is high; the result
//   freq_khz = round(count * REF_FREQ_KHZ / REF_TICKS)
// crosses back into the clk_ref domain with a toggle handshake, so the
// 32-bit output is tear-free for slow readers.  Because the gate edges are
// synchronized, the count is exact to a couple of clk_meas cycles; the
// quantization step is REF_FREQ_KHZ / REF_TICKS (1.25 kHz at the defaults).
// freq_khz is 0 after reset and stays 0 until the first measurement
// completes; if clk_meas stops (MMCM in reset), the last value is held.
module ng_clk_meas #(
  parameter int REF_TICKS    = 200000,  // clk_ref periods per window (800 us)
  parameter int REF_FREQ_KHZ = 250000,  // clk_ref frequency in kHz
  // width of the edge counter in the clk_meas domain; must hold the count of
  // one window (the board uses 20 bits: 400 MHz x 1 ms = 400,000)
  parameter int CNT_W        = 32
) (
  input  logic        clk_meas,  // clock under test (e.g. an MMCM output)
  input  logic        clk_ref,   // reference clock (clk_sh)
  input  logic        rst_n,     // asynchronous, active low
  output logic [31:0] freq_khz   // measured frequency in kHz, clk_ref domain
);
  // clk_ref cycles with the gate low between windows: covers the 2-FF gate
  // sync, the two-cycle compute and the toggle handshake, with margin
  localparam int GAP = 32;
  // REF_TICKS == REF_FREQ_KHZ means a 1 ms window: the edge count is already
  // the frequency in kHz and the multiply/divide below are pruned (they would
  // be a 64-bit constant divide in a 400 MHz domain otherwise)
  localparam bit DIRECT = (REF_TICKS == REF_FREQ_KHZ);

  // --------------------------------------------------------------
  logic [31:0] phase;
  logic        gate, req_s, ack;
  logic [31:0] data_s;

  // free-running window phase: gate high for REF_TICKS cycles, then a GAP
  assign gate = (phase < REF_TICKS);
  always_ff @(posedge clk_ref or negedge rst_n) begin
    if (!rst_n)                            phase <= REF_TICKS;  // start in the gap
    else if (phase == REF_TICKS + GAP - 1) phase <= '0;
    else                                   phase <= phase + 32'd1;
  end

  // result latch: the toggle handshake guarantees data_s is stable here
  always_ff @(posedge clk_ref or negedge rst_n) begin
    if (!rst_n) begin
      freq_khz <= '0;
      ack      <= 1'b0;
    end else if (req_s != ack) begin
      freq_khz <= data_s;
      ack      <= ~ack;
    end
  end

  // --------------------------------------------------------------
  logic        win, win_d, calc0, calc1, req_t, ack_s;
  logic [CNT_W-1:0] cnt;
  logic [31:0] freq_m;
  logic [63:0] prod;

  ng_prim_sync #(.W(1))  u_gate (.clk(clk_meas), .rst_n(rst_n), .d(gate),   .q(win));
  ng_prim_sync #(.W(1))  u_ack  (.clk(clk_meas), .rst_n(rst_n), .d(ack),    .q(ack_s));
  ng_prim_sync #(.W(1))  u_req  (.clk(clk_ref),  .rst_n(rst_n), .d(req_t),  .q(req_s));
  ng_prim_sync #(.W(32)) u_data (.clk(clk_ref),  .rst_n(rst_n), .d(freq_m), .q(data_s));

  always_ff @(posedge clk_meas or negedge rst_n) begin
    if (!rst_n) begin
      win_d <= 1'b0; calc0 <= 1'b0; calc1 <= 1'b0; req_t <= 1'b0;
      cnt <= '0; prod <= '0; freq_m <= '0;
    end else begin
      win_d <= win;
      calc1 <= calc0;
      calc0 <= win_d && !win;              // window just closed
      if (win_d && !win) begin             // snapshot and restart the counter
        prod <= DIRECT ? 64'(cnt) : 64'(cnt) * 64'(REF_FREQ_KHZ);
        cnt  <= '0;
      end else if (win) begin
        cnt <= cnt + 1'b1;                 // one count per clk_meas edge
      end
      // two-cycle compute: register the product, then the constant divide
      if (calc0) freq_m <= DIRECT ? prod[31:0] : 32'((prod + 64'(REF_TICKS / 2)) / REF_TICKS);
      // publish: the previous handshake completed long ago (GAP covers it),
      // otherwise this result would be dropped -- there is no backpressure
      if (calc1 && req_t == ack_s) req_t <= ~req_t;
    end
  end
endmodule
