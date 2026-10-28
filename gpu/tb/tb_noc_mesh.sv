// tb_noc_mesh.sv -- NoC testbench: all-to-all random traffic on a 3x3
// mesh with random receive-side backpressure (stall clocks), both virtual
// channels (request and response classes) exercised simultaneously, payload
// and per-(src,dst,vc) ordering checked, multi-seed, deadlock watchdog.
// Expect NG_NOC_TB_PASS.
// NG_NOC_TB_BUF / NG_NOC_TB_LOC / NG_NOC_TB_TXQ override the router FIFO depth, the endpoint receive credits
// and the endpoint TX queue depth (run_tb_noc.sh also runs 3 / 3 / 1: non-power-of-two FIFOs and TXQ_DEPTH = 1).
`timescale 1ns/1ps
`ifndef NG_NOC_TB_BUF
`define NG_NOC_TB_BUF 4
`endif
`ifndef NG_NOC_TB_LOC
`define NG_NOC_TB_LOC 4
`endif
`ifndef NG_NOC_TB_TXQ
`define NG_NOC_TB_TXQ 4
`endif
module tb_noc_mesh;
  import ng_noc_pkg::*;

  localparam int GW = 3, GH = 3, NN = GW * GH;
  localparam int BUF_DEPTH = `NG_NOC_TB_BUF, LOC_CREDITS = `NG_NOC_TB_LOC, TXQ_DEPTH = `NG_NOC_TB_TXQ;
  localparam int PKTS = 96;      // per node per seed
  localparam int SEEDS = 4;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  // mesh endpoint-facing links
  logic [NN-1:0]            ep_inj_valid;
  flit_t [NN-1:0]           ep_inj_flit;
  logic [NN-1:0][1:0]       ep_inj_credit;
  logic [NN-1:0]            ep_recv_valid;
  flit_t [NN-1:0]           ep_recv_flit;
  logic [NN-1:0][1:0]       ep_recv_credit;

  ng_noc_mesh #(
    .GW(GW), .GH(GH), .NUM_VC(2),
    .BUF_DEPTH(BUF_DEPTH), .LOC_CREDITS(LOC_CREDITS)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .ep_inj_valid(ep_inj_valid), .ep_inj_flit(ep_inj_flit),
    .ep_inj_credit(ep_inj_credit),
    .ep_recv_valid(ep_recv_valid), .ep_recv_flit(ep_recv_flit),
    .ep_recv_credit(ep_recv_credit)
  );

  // per-node agent-side streams (driven from processes)
  logic     ti_v [NN];
  logic     ti_r [NN];
  flit_t    ti_f [NN];
  logic     to_v [NN];
  logic     to_r [NN];
  flit_t    to_f [NN];

  for (genvar i = 0; i < NN; i++) begin : g_ep
    logic  inj_v;
    flit_t inj_f;
    ng_noc_ep #(
      .NUM_VC(2), .INJ_CREDITS(BUF_DEPTH), .RX_DEPTH(2 * LOC_CREDITS), .TXQ_DEPTH(TXQ_DEPTH)
    ) u_ep (
      .clk(clk), .rst_n(rst_n),
      .ti_valid(ti_v[i]), .ti_ready(ti_r[i]), .ti_flit(ti_f[i]),
      .to_valid(to_v[i]), .to_ready(to_r[i]), .to_flit(to_f[i]),
      .inj_valid(inj_v), .inj_flit(inj_f), .inj_credit(ep_inj_credit[i]),
      .recv_valid(ep_recv_valid[i]), .recv_flit(ep_recv_flit[i]),
      .recv_credit(ep_recv_credit[i])
    );
    assign ep_inj_valid[i] = inj_v;
    assign ep_inj_flit[i]  = inj_f;
  end

  // ---- scoreboarding --------------------------------------------------------
  longint   exp_seq [NN][NN][2];
  longint   send_seq [NN][NN][2];
  int       delivered;
  int       errors, checks;
  int       active;
  int       idle_cycles;
  longint   last_delivered;

  task automatic check_recv(input int n, input flit_t f);
    int s, c;
    s = f.src;
    c = f.cls;
    checks++;
    if (f.dst != 6'(n)) begin
      errors++;
      $display("TB_FAIL noc: node %0d got flit for %0d", n, f.dst);
    end else if (s == n) begin
      errors++;
      $display("TB_FAIL noc: node %0d got flit from itself", n);
    end else if (f.data != (64'(exp_seq[s][n][c]) ^ 64'hC0FFEE_00000000)) begin
      errors++;
      $display("TB_FAIL noc: node %0d src %0d vc %0d seq mismatch: %h want seq %0d",
               n, s, c, f.data, exp_seq[s][n][c]);
    end else begin
      exp_seq[s][n][c]++;
      delivered++;
    end
  endtask

  // receiver: random ready deassertion every cycle
  initial begin
    for (int i = 0; i < NN; i++) to_r[i] = 1'b0;
    forever begin
      @(negedge clk);
      for (int i = 0; i < NN; i++) begin
        to_r[i] = active && ($urandom_range(0, 3) != 0);
        if (active && to_v[i] && to_r[i]) check_recv(i, to_f[i]);
      end
    end
  end

  // sender for node i
  task automatic sender(input int i, input int seed);
    for (int k = 0; k < PKTS; k++) begin
      int dst, c;
      flit_t f;
      dst = $urandom_range(0, NN - 1);
      if (dst == i) dst = (i + 1) % NN;
      c = $urandom_range(0, 1);
      f = '{cls: 2'(c), dst: 6'(dst), src: 6'(i),
           data: (64'(send_seq[i][dst][c]) ^ 64'hC0FFEE_00000000),
           default: '0};
      send_seq[i][dst][c]++;
      ti_v[i] = 1'b1;
      ti_f[i] = f;
      while (!ti_r[i]) @(negedge clk);
      @(negedge clk);
      ti_v[i] = 1'b0;
      if ($urandom_range(0, 2) == 0) repeat ($urandom_range(0, 2)) @(negedge clk);
    end
  endtask

  // watchdog: no delivery progress for 20000 cycles while active = deadlock
  initial begin
    idle_cycles = 0;
    last_delivered = 0;
    forever begin
      @(posedge clk);
      if (active) begin
        if (delivered == last_delivered) idle_cycles++;
        else begin
          idle_cycles   = 0;
          last_delivered = delivered;
        end
        if (idle_cycles > 20000) begin
          $display("NG_NOC_TB_FAIL: deadlock watchdog (delivered=%0d)", delivered);
          $finish;
        end
      end
    end
  end

  // ---- main ----------------------------------------------------------------
  initial begin
    errors = 0; checks = 0; delivered = 0; active = 0;
    for (int i = 0; i < NN; i++) begin
      ti_v[i] = 1'b0;
      for (int j = 0; j < NN; j++) begin
        exp_seq[i][j][0] = 0; exp_seq[i][j][1] = 0;
        send_seq[i][j][0] = 0; send_seq[i][j][1] = 0;
      end
    end

    for (int sd = 1; sd <= SEEDS; sd++) begin
      rst_n = 1'b0;
      delivered = 0; active = 0;
      for (int i = 0; i < NN; i++) begin
        ti_v[i] = 1'b0;
        for (int j = 0; j < NN; j++) begin
          exp_seq[i][j][0] = 0; exp_seq[i][j][1] = 0;
          send_seq[i][j][0] = 0; send_seq[i][j][1] = 0;
        end
      end
      repeat (4) @(negedge clk);
      rst_n = 1'b1;
      repeat (2) @(negedge clk);
      active = 1;
      fork
        sender(0, sd); sender(1, sd); sender(2, sd);
        sender(3, sd); sender(4, sd); sender(5, sd);
        sender(6, sd); sender(7, sd); sender(8, sd);
      join
      wait (delivered == NN * PKTS);
      active = 0;
      $display("TB: noc seed %0d done (%0d flits, %0d checks)", sd, delivered, checks);
    end

    if (errors == 0)
      $display("NG_NOC_TB_PASS: %0d checks, 0 errors", checks);
    else
      $display("NG_NOC_TB_FAIL: %0d errors of %0d checks", errors, checks);
    $finish;
  end

  initial begin
    #50_000_000;
    $display("NG_NOC_TB_FAIL: absolute watchdog timeout");
    $finish;
  end
endmodule
