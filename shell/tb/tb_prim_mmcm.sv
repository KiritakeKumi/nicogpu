// tb_prim_mmcm.sv -- self-checking testbench for ng_prim_mmcm (behavioral
// model) and ng_clk_meas (xsim / any SV simulator).  Checks:
//   1. reset behavior: outputs stopped and locked low while rst is high,
//      locked drops asynchronously on re-reset, re-lock ~1 us after release
//   2. frequency accuracy: 10 ms window, error < 0.01% on all three outputs
//      (clk_ref generated here at exactly 299.825 MHz)
//   3. DRP: drdy timing (one cycle after drp_en, single-cycle pulse, silent
//      when idle), write/read-back on several addresses, unwritten addresses
//      read 0, contents survive an MMCM reset, divide writes leave the model
//      frequencies unchanged
//   4. ng_clk_meas: 249.8542 MHz against a 250.000 MHz reference gives
//      freq_khz in [249700, 250000] kHz and the value stays put afterwards
// Prints NG_PRIM_MMCM_TB_PASS on success.
`timescale 1ns/1fs
module tb_prim_mmcm;
  int errors = 0;
  task automatic check(input bit c, input string msg);
    if (!c) begin errors++; $display("FAIL: %s @%0t", msg, $time); end
  endtask
  function automatic real rel_err(input real f, input real f_nom);
    return (f > f_nom ? f - f_nom : f_nom - f) / f_nom;
  endfunction

  // ----------------------------------------------------------
  // Absolute-scheduled generators (edge at tn, tn += half period): rounding
  // cannot accumulate at 1 fs precision.  clk_ref runs from t = 0 like the
  // real SI5381; the clk_meas pair starts late to keep the long frequency
  // window cheap.
  logic clk_ref = 1'b0;  // 299.825 MHz (SI5381 stand-in)
  logic drp_clk = 1'b0;  // 100 MHz DRP clock
  logic clk_m   = 1'b0;  // 249.8542 MHz (ng_clk_meas DUT input)
  logic clk_r2  = 1'b0;  // 250.000 MHz (ng_clk_meas DUT reference)
  bit   meas_run = 1'b0;

  initial begin : gen_ref
    real tn; tn = 0.0;
    forever begin
      tn += 500.0 / 299.825;
      #(tn - $realtime) clk_ref = ~clk_ref;
    end
  end
  initial begin : gen_drp
    real tn; tn = 0.0;
    forever begin
      tn += 5.0;
      #(tn - $realtime) drp_clk = ~drp_clk;
    end
  end
  initial begin : gen_meas
    real tn;
    wait (meas_run);
    tn = $realtime;
    forever begin
      tn += 500.0 / 249.8542;
      #(tn - $realtime) clk_m = ~clk_m;
    end
  end
  initial begin : gen_ref2
    real tn;
    wait (meas_run);
    tn = $realtime;
    forever begin
      tn += 2.0;
      #(tn - $realtime) clk_r2 = ~clk_r2;
    end
  end

  // ----------------------------------------------------------
  logic rst = 1'b1, locked, clk_out0, clk_out1, clk_out2;
  logic drp_en, drp_we, drp_rdy;
  logic [6:0]  drp_addr;
  logic [15:0] drp_di, drp_do;

  ng_prim_mmcm dut_mmcm (
    .clk_ref(clk_ref), .rst(rst), .locked(locked),
    .clk_out0(clk_out0), .clk_out1(clk_out1), .clk_out2(clk_out2),
    .drp_clk(drp_clk), .drp_en(drp_en), .drp_we(drp_we),
    .drp_addr(drp_addr), .drp_di(drp_di), .drp_do(drp_do), .drp_rdy(drp_rdy)
  );

  logic rst_n_m = 1'b0;
  logic [31:0] freq_khz;
  ng_clk_meas dut_meas (
    .clk_meas(clk_m), .clk_ref(clk_r2), .rst_n(rst_n_m), .freq_khz(freq_khz)
  );

  // DIRECT mode (1 ms window: REF_TICKS == REF_FREQ_KHZ), as used on the board
  logic [31:0] freq_khz_d;
  ng_clk_meas #(.REF_TICKS(250000), .REF_FREQ_KHZ(250000), .CNT_W(20)) dut_meas_d (
    .clk_meas(clk_m), .clk_ref(clk_r2), .rst_n(rst_n_m), .freq_khz(freq_khz_d)
  );

  // ----------------------------------------------------------
  longint unsigned c0, c1, c2;
  bit cnt_en = 1'b0;
  int meas_guard;
  always @(posedge clk_out0) if (cnt_en) c0++;
  always @(posedge clk_out1) if (cnt_en) c1++;
  always @(posedge clk_out2) if (cnt_en) c2++;

  task automatic freq_check(input longint unsigned cnt, input real win,
                            input real f_nom, input real tol, input string name);
    real f;
    f = 1000.0 * cnt / win;  // edges per ns -> MHz
    check(rel_err(f, f_nom) < tol,
          $sformatf("%s: %.6f MHz measured, %.6f nominal", name, f, f_nom));
  endtask

  task automatic freq_window(input real win_ns, input real tol);
    real t0;
    c0 = 0; c1 = 0; c2 = 0;
    t0 = $realtime;
    cnt_en = 1'b1;
    #(win_ns);
    cnt_en = 1'b0;
    freq_check(c0, $realtime - t0, dut_mmcm.CLKOUT0_MHZ, tol, "clk_out0 frequency");
    freq_check(c1, $realtime - t0, dut_mmcm.CLKOUT1_MHZ, tol, "clk_out1 frequency");
    freq_check(c2, $realtime - t0, dut_mmcm.CLKOUT2_MHZ, tol, "clk_out2 frequency");
  endtask

  // one DRP transfer, checking the drdy timing: signals change at negedge,
  // the model samples at posedge, so drdy must be high on the negedge after
  // the en cycle and low again on the next one
  task automatic drp_wr(input logic [6:0] a, input logic [15:0] d);
    @(negedge drp_clk);
    drp_en = 1'b1; drp_we = 1'b1; drp_addr = a; drp_di = d;
    @(negedge drp_clk);
    drp_en = 1'b0; drp_we = 1'b0;
    check(drp_rdy === 1'b1, "DRP write: drdy one cycle after drp_en");
    @(negedge drp_clk);
    check(drp_rdy === 1'b0, "DRP write: drdy is a single-cycle pulse");
  endtask

  task automatic drp_rd(input logic [6:0] a, input logic [15:0] exp);
    @(negedge drp_clk);
    drp_en = 1'b1; drp_we = 1'b0; drp_addr = a; drp_di = '0;
    @(negedge drp_clk);
    drp_en = 1'b0;
    check(drp_rdy === 1'b1, "DRP read: drdy one cycle after drp_en");
    check(drp_do === exp, $sformatf("DRP read %02h: %04h, expected %04h",
                                    a, drp_do, exp));
    @(negedge drp_clk);
    check(drp_rdy === 1'b0, "DRP read: drdy is a single-cycle pulse");
  endtask

  // ----------------------------------------------------------
  initial begin : main
    // ---------------- in reset
    rst = 1'b1;
    drp_en = 1'b0; drp_we = 1'b0; drp_addr = '0; drp_di = '0;
    #100;
    check(locked === 1'b0, "locked low while in reset");
    check(clk_out0 === 1'b0 && clk_out1 === 1'b0 && clk_out2 === 1'b0,
          "outputs stopped while in reset");

    // ---------------- release: locked after ~1 us (300 clk_ref edges)
    rst = 1'b0;
    #500  check(locked === 1'b0, "still unlocked 0.5 us after release");
    #1000 check(locked === 1'b1, "locked ~1 us after release");

    // ---------------- clocks run at the right frequency
    freq_window(1000.0, 0.01);             // 1 us sanity window, 1%
    freq_window(10_000_000.0, 0.0001);     // 10 ms window, 0.01%

    // ---------------- DRP: unwritten addresses read 0
    drp_rd(7'h7F, 16'h0000);
    // write/read-back on a few addresses (on the real device 0x08/0x09 hold
    // the CLKOUT0 divide; the model treats them like any other address)
    drp_wr(7'h08, 16'hA55A); drp_rd(7'h08, 16'hA55A);
    drp_wr(7'h09, 16'h1234); drp_rd(7'h09, 16'h1234);
    drp_wr(7'h14, 16'h00FF); drp_rd(7'h14, 16'h00FF);
    drp_wr(7'h14, 16'h55AA); drp_rd(7'h14, 16'h55AA);  // overwrite
    repeat (10) begin
      @(negedge drp_clk);
      check(drp_rdy === 1'b0, "no drdy while drp_en is idle");
    end

    // ---------------- divide writes do not change the model frequencies
    freq_window(1_000_000.0, 0.0001);      // 1 ms, still 0.01%

    // ---------------- re-reset: asynchronous locked drop, clocks stop
    rst = 1'b1;
    #5 check(locked === 1'b0, "locked drops asynchronously on rst");
    #50;
    check(clk_out0 === 1'b0 && clk_out1 === 1'b0 && clk_out2 === 1'b0,
          "outputs clamp low within one half period of rst");
    c0 = 0; c1 = 0; c2 = 0;
    cnt_en = 1'b1;
    #200;
    cnt_en = 1'b0;
    check(c0 == 0 && c1 == 0 && c2 == 0, "no output edges while in reset");
    // DRP contents survive the reset (model keeps its registers)
    drp_rd(7'h08, 16'hA55A);

    // ---------------- release again
    rst = 1'b0;
    #1500 check(locked === 1'b1, "re-locks ~1 us after the second release");
    rst = 1'b1;   // nothing else needs the MMCM model: stop its outputs

    // ---------------- ng_clk_meas: 249.8542 MHz against 250.000 MHz
    meas_run = 1'b1;
    #200;
    @(negedge clk_r2);
    #1;
    rst_n_m = 1'b1;
    #700_000;    // first window is 800 us: no result yet
    check(freq_khz == 32'd0, "freq_khz 0 before the first measurement completes");
    begin : poll
      meas_guard = 0;
      while (freq_khz == 32'd0 && meas_guard < 1100) begin
        #1000;
        meas_guard++;
      end
      check(freq_khz != 32'd0, "first measurement completes");
    end
    $display("freq_khz = %0d (expected ~249854)", freq_khz);
    begin : poll_d
      meas_guard = 0;
      while (freq_khz_d == 32'd0 && meas_guard < 1100) begin
        #1000;
        meas_guard++;
      end
      $display("freq_khz_d (direct, 1 ms window) = %0d", freq_khz_d);
      check(freq_khz_d >= 32'd249852 && freq_khz_d <= 32'd249856,
            "direct-mode freq_khz exact to +-2 kHz");
    end
    check(freq_khz >= 32'd249700 && freq_khz <= 32'd250000,
          "freq_khz within [249700, 250000]");

    // ---------------- value stays put (within measurement resolution)
    begin : stab
      logic [31:0] last;
      last = freq_khz;
      repeat (3200) begin          // ~3.2 ms = 4 more measurement windows
        #1000;
        check(freq_khz >= 32'd249700 && freq_khz <= 32'd250000,
              "freq_khz stays within range");
        if (freq_khz > last) check(freq_khz - last <= 32'd4,
                                   "freq_khz changes by at most a few LSBs");
        else                 check(last - freq_khz <= 32'd4,
                                   "freq_khz changes by at most a few LSBs");
        last = freq_khz;
      end
    end

    if (errors == 0) $display("NG_PRIM_MMCM_TB_PASS");
    else             $display("NG_PRIM_MMCM_TB_FAIL errors=%0d", errors);
    $finish;
  end

  // watchdog
  initial begin
    #30_000_000;
    $display("FAIL: watchdog timeout @%0t", $time);
    $finish;
  end
endmodule
