// tb_prim_icap.sv -- ng_prim_icap behavioral model and diagnostics knobs.
//
// Four model instances, each whose "silicon" wants a different pin word order
// (MODEL_ORDER = 0..3).  For every order setting x CSIB mode the same stream
// (dummy, bus-width pattern, sync, NOP, write-CMD, DESYNC, NOP) is pushed in
// BE file-word form; only the instance whose MODEL_ORDER equals the selected
// order may see "synced" / report prdone.  CSIB modes:
//   0 off            idle gaps release CSIB
//   1 cont           CSIB low except inside a write packet (no NOP between
//                    the CMD header and its payload: prdone still happens)
//   2 force, gaps    NOP injected inside the packet -> the load is corrupted
//                    (no prdone anywhere): demonstrates why force is gapless-only
//   3 force, gapless back-to-back words: CSIB never high, prdone as mode 0
// Also checked: exact pin words, NOP transform, O capture + snapshot.
// ICAP word count in the snapshot, PRDONE_FELL (only the matching model
// leaves the idle PRDONE level), AVAIL_LOST (AVAIL forced low in one model),
// and the readback sequencer: per ORDER x model, a read of IDCODE (count 1 /
// 2, SKIP 0 / 2) with and without a PR session; only the matching model
// returns the register in the expected read cycle, the others stay unsynced;
// RDWRB never changes while CSIB is low; the engine is desynced afterwards.
// Pass token: NG_PRIM_ICAP_TB_PASS
`timescale 1ns / 1ps

module tb_prim_icap;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic        enable = 0, valid = 0;
  logic [31:0] data = 0;
  logic [1:0]  order = 0;
  logic        cont = 0, force_nop = 0;
  logic        snap_req = 0;

  logic [3:0]  prdone, prerror, prdone_raw, prerror_raw, snap_ack;
  logic [3:0]  avail_raw, avail_lost, prdone_fell, rb_ack, rb_busy;
  logic [200:0] obs [4];
  logic [255:0] rb_data [4];
  logic [31:0] icap_words [4];
  logic [3:0]  synced_seen;
  logic        rb_req = 0;
  logic [4:0]  rb_reg = 0;
  logic [7:0]  rb_cnt = 0;
  logic [5:0]  rb_skip = 0;
  logic [3:0]  rb_nop = 0;

  for (genvar k = 0; k < 4; k++) begin : g
    ng_prim_icap #(.MODEL_ORDER(k[1:0])) dut (
      .clk(clk), .rst_n(rst_n), .enable(enable), .valid(valid), .ready(), .data(data),
      .order(order), .csib_cont(cont), .csib_force(force_nop),
      .prdone(prdone[k]), .prerror(prerror[k]), .prdone_raw(prdone_raw[k]), .prerror_raw(prerror_raw[k]),
      .avail_raw(avail_raw[k]), .avail_lost(avail_lost[k]), .prdone_fell(prdone_fell[k]),
      .snap_req(snap_req), .snap_ack(snap_ack[k]), .obs(obs[k]),
      .icap_words(icap_words[k]), .o_live(),
      .rb_req(rb_req), .rb_reg(rb_reg), .rb_cnt(rb_cnt), .rb_skip(rb_skip), .rb_nop(rb_nop),
      .rb_ack(rb_ack[k]), .rb_busy(rb_busy[k]), .rb_data(rb_data[k]));
    always @(posedge clk) if (dut.synced) synced_seen[k] <= 1'b1;
  end

  // RDWRB may only change while CSIB is high (before and after the change)
  int rdwrb_viol = 0, rd_cycles = 0;
  logic csib_prev = 1'b1, rdwrb_prev = 1'b0;
  always @(posedge clk) begin
    if (rst_n && g[0].dut.rdwrb_q !== rdwrb_prev && (csib_prev !== 1'b1 || g[0].dut.csib_q !== 1'b1)) rdwrb_viol++;
    if (g[0].dut.rdwrb_q === 1'b1 && g[0].dut.csib_q === 1'b0) rd_cycles++;
    csib_prev  <= g[0].dut.csib_q;
    rdwrb_prev <= g[0].dut.rdwrb_q;
  end

  int errors = 0;
  task automatic fail(input string w);
    $display("TB_FAIL %s", w); errors++;
  endtask

  int  csib_high_cnt = 0;
  logic csib_count_en = 1'b0;
  always @(posedge clk) if (csib_count_en && g[0].dut.csib_q) csib_high_cnt++;

  localparam int NW = 9;
  localparam logic [31:0] STREAM [NW] = '{
    32'hFFFF_FFFF, 32'h0000_00BB, 32'h1122_0044, 32'hFFFF_FFFF, 32'hAA99_5566,
    32'h2000_0000, 32'h3000_8001, 32'h0000_000D, 32'h2000_0000 };

  task automatic run_stream(input logic [1:0] o, input int mode);
    int exp_synced;
    order = o;
    cont = (mode == 1); force_nop = (mode >= 2);
    enable = 1'b0; valid = 1'b0;
    repeat (4) @(posedge clk);
    synced_seen = '0;
    enable = 1'b1;
    repeat (4) @(posedge clk);
    csib_high_cnt = 0;
    csib_count_en = 1'b1;
    for (int w = 0; w < NW; w++) begin
      #1; data = STREAM[w]; valid = 1'b1;
      @(posedge clk);
      if (mode != 3) begin
        #1; valid = 1'b0; data = 32'hDEAD_BEEF;     // gap (garbage data while !valid)
        if (w == 2 || w == 5) repeat (3) @(posedge clk);
        else                  @(posedge clk);
      end
    end
    #1; valid = 1'b0; data = 32'hDEAD_BEEF;
    if (mode == 3) begin
      // gapless: CSIB must have stayed low for the whole stream, then
      // force keeps feeding NOPs while enabled
      if (csib_high_cnt != 0) fail($sformatf("order %0d gapless: CSIB high %0d cycles", o, csib_high_cnt));
    end
    repeat (8) @(posedge clk);
    csib_count_en = 1'b0;
    if (mode >= 2 && csib_high_cnt != 0)
      fail($sformatf("order %0d force: CSIB went high %0d cycles in the window", o, csib_high_cnt));
    if (mode == 1 && csib_high_cnt == 0)
      fail($sformatf("order %0d cont: CSIB never released inside the packet", o));
    if (mode == 0 && csib_high_cnt == 0)
      fail($sformatf("order %0d off: CSIB never idled across gaps", o));
    for (int k = 0; k < 4; k++) begin
      exp_synced = (k == o);
      if (synced_seen[k] !== exp_synced[0])
        fail($sformatf("order %0d mode %0d: model %0d synced_seen=%0b want %0d", o, mode, k, synced_seen[k], exp_synced));
      if (prdone[k] !== (exp_synced[0] && mode != 2))
        fail($sformatf("order %0d mode %0d: model %0d prdone=%0b", o, mode, k, prdone[k]));
      if (prerror[k] !== 1'b0) fail($sformatf("order %0d model %0d unexpected prerror", o, k));
      if (prdone_fell[k] !== exp_synced[0])
        fail($sformatf("order %0d mode %0d: model %0d prdone_fell=%0b", o, mode, k, prdone_fell[k]));
      if (icap_words[k] !== NW) fail($sformatf("order %0d mode %0d: model %0d icap_words=%0d", o, mode, k, icap_words[k]));
      if (avail_lost[k] !== 1'b0) fail($sformatf("order %0d model %0d unexpected avail_lost", o, k));
    end
    if (mode != 2) begin
      // O capture: matching model went 9B -> DB -> 9B, mismatching stays 9B
      snap_req = ~snap_req;
      repeat (4) @(posedge clk);
      for (int k = 0; k < 4; k++) begin
        if (snap_ack[k] !== snap_req) fail($sformatf("snap_ack model %0d did not follow", k));
        if (obs[k][200:169] !== NW) fail($sformatf("order %0d mode %0d: snapshot word count %0d", o, mode, obs[k][200:169]));
        if (k == o) begin
          if (obs[k][31:0] !== 32'h9B || obs[k][160] !== 1'b1 || obs[k][168:161] !== 8'd3 ||
              obs[k][63:32] !== 32'h9B || obs[k][95:64] !== 32'hDB || obs[k][127:96] !== 32'h9B ||
              obs[k][159:128] !== 32'h0)
            fail($sformatf("order %0d mode %0d: O log of matching model: %h", o, mode, obs[k]));
        end else begin
          if (obs[k][31:0] !== 32'h9B || obs[k][160] !== 1'b1 || obs[k][168:161] !== 8'd1 ||
              obs[k][63:32] !== 32'h9B || obs[k][95:64] !== 32'h0)
            fail($sformatf("order %0d mode %0d: O log of mismatching model %0d: %h", o, mode, k, obs[k]));
        end
      end
      if (prdone_raw !== 4'hF) fail($sformatf("prdone_raw %b", prdone_raw));  // invented model: 1 when not in a session
    end
    enable = 1'b0;
    repeat (3) @(posedge clk);
    if (prdone !== 4'h0) fail("prdone must clear when disabled");
  endtask

  // one readback: variant v = 0 count 1 skip 0 idle, 1 count 2 skip 0 idle,
  // 2 count 1 skip 2 idle (data moves to word 1), 3 count 1 skip 0 inside an
  // enabled PR session (no FIFO words) with 3 extra header NOPs
  function automatic logic [31:0] xfw(input logic [1:0] ord, input logic [31:0] w);
    logic [31:0] r;
    case (ord)
      2'd0: for (int i = 0; i < 32; i++) r[i] = w[31 - i];
      2'd1: for (int b = 0; b < 4; b++)
              for (int i = 0; i < 8; i++) r[8*b + i] = w[8*b + 7 - i];
      2'd2: r = {w[7:0], w[15:8], w[23:16], w[31:24]};
      default: r = w;
    endcase
    return r;
  endfunction

  task automatic rb_case(input logic [1:0] o, input int v);
    int cnt, skip, at, tmo;
    logic [31:0] want, got;
    cnt  = (v == 1) ? 2 : 1;
    skip = (v == 2) ? 2 : 0;
    order = o; cont = 1'b0; force_nop = 1'b0;
    enable = (v == 3);
    repeat (4) @(posedge clk);
    rb_reg = 5'h0C; rb_cnt = cnt[7:0]; rb_skip = skip[5:0]; rb_nop = (v == 3) ? 4'd3 : 4'd0;
    @(posedge clk); #1;
    rb_req = ~rb_req;
    tmo = 0;
    while (rb_ack !== {4{rb_req}} && tmo < 400) begin @(posedge clk); #1; tmo++; end
    if (tmo >= 400) fail($sformatf("rb order %0d v %0d: no ack", o, v));
    if (rb_busy !== 4'h0) fail($sformatf("rb order %0d v %0d: busy after ack", o, v));
    for (int k = 0; k < 4; k++) begin
      for (int i = 0; i < 8; i++) begin
        got = rb_data[k][32*i +: 32];
        at  = 3 - skip;                         // MODEL_RD_LAT = 3
        if (k == o) want = (i >= at && i < at + cnt) ? xfw(k[1:0], 32'h04B3_5093) : 32'h0000_00DB;
        else        want = 32'h0000_009B;
        if (got !== want)
          fail($sformatf("rb order %0d v %0d model %0d word %0d: %08x want %08x", o, v, k, i, got, want));
      end
      if (prerror[k] !== 1'b0) fail($sformatf("rb order %0d v %0d model %0d prerror", o, v, k));
      if (v != 3 && prdone[k] !== 1'b0) fail($sformatf("rb order %0d v %0d model %0d prdone without a session", o, v, k));
    end
    if (g[0].dut.synced !== 1'b0 || g[1].dut.synced !== 1'b0 || g[2].dut.synced !== 1'b0 || g[3].dut.synced !== 1'b0)
      fail($sformatf("rb order %0d v %0d: a model is still synced after the DESYNC tail", o, v));
    enable = 1'b0;
    repeat (3) @(posedge clk);
  endtask

  initial begin
    synced_seen = '0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    // exact pin words for BE 0xAA995566
    enable = 1'b1;
    for (int o = 0; o < 4; o++) begin
      logic [31:0] want;
      order = o[1:0]; cont = 1'b0; force_nop = 1'b0;
      case (o)
        0: want = 32'h66AA_9955;   // bitrev32
        1: want = 32'h5599_AA66;   // per-byte bit reversal, byte order kept
        2: want = 32'h6655_99AA;   // byte reversal only
        default: want = 32'hAA99_5566;
      endcase
      @(posedge clk); #1; data = 32'hAA99_5566; valid = 1'b1;
      @(posedge clk); #1; valid = 1'b0;
      if (g[0].dut.i_q !== want || g[0].dut.csib_q !== 1'b0)
        fail($sformatf("order %0d: pin word %08x csib %0b want %08x", o, g[0].dut.i_q, g[0].dut.csib_q, want));
      @(posedge clk); #1;
      @(posedge clk); #1;
      if (g[0].dut.csib_q !== 1'b1) fail("CSIB must idle high without valid and !cont");
    end
    enable = 1'b0;
    repeat (3) @(posedge clk);

    for (int o = 0; o < 4; o++)
      for (int m = 0; m < 4; m++)
        run_stream(o[1:0], m);

    // NOP word transform when idle in forced mode (order 1: BE 20000000 -> 04000000)
    order = 2'd1; cont = 1'b0; force_nop = 1'b1; enable = 1'b1;
    repeat (4) @(posedge clk); #1;
    if (g[0].dut.i_q !== 32'h0400_0000 || g[0].dut.csib_q !== 1'b0)
      fail($sformatf("force idle word %08x csib %0b", g[0].dut.i_q, g[0].dut.csib_q));
    enable = 1'b0;
    repeat (3) @(posedge clk); #1;
    if (g[0].dut.csib_q !== 1'b1) fail("CSIB must be high when disabled even with force");

    // ---- AVAIL forced low in model 0 during an order-0 session
    force g[0].dut.model_avail = 1'b0;
    order = 2'd0; cont = 1'b0; force_nop = 1'b0;
    enable = 1'b1;
    repeat (4) @(posedge clk);
    if (avail_raw !== 4'b1110) fail($sformatf("avail_raw %b with model 0 forced low", avail_raw));
    #1; data = 32'hFFFF_FFFF; valid = 1'b1;
    @(posedge clk); #1; valid = 1'b0;
    repeat (3) @(posedge clk);
    if (avail_lost !== 4'b0001) fail($sformatf("avail_lost %b, want only model 0", avail_lost));
    release g[0].dut.model_avail; g[0].dut.model_avail = 1'b1;   // a released variable keeps the forced value
    enable = 1'b0;
    repeat (3) @(posedge clk);
    enable = 1'b1;
    repeat (3) @(posedge clk);
    if (avail_lost !== 4'b0000) fail("avail_lost must clear when enable rises");
    enable = 1'b0;
    repeat (3) @(posedge clk);

    // ---- readback sequencer (IDCODE = 0x04B35093 in the model)
    for (int o = 0; o < 4; o++)
      for (int v = 0; v < 4; v++)
        rb_case(o[1:0], v);
    if (rdwrb_viol != 0) fail($sformatf("RDWRB changed while CSIB was low %0d times", rdwrb_viol));
    if (rd_cycles == 0) fail("no read cycles seen");
    $display("rb: %0d read cycles on model 0, %0d RDWRB violations", rd_cycles, rdwrb_viol);

    if (errors == 0) $display("NG_PRIM_ICAP_TB_PASS");
    else             $display("NG_PRIM_ICAP_TB_FAIL %0d errors", errors);
    $finish;
  end

  initial begin
    #20_000_000;
    fail("watchdog");
    $finish;
  end
endmodule
