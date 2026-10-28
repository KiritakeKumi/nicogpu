// tb_dfx_iso.sv -- directed test of the DFX isolation blocks (xsim / any SV simulator).
//   iso_s: normal read, forced decouple in the middle of a read burst, local
//          write answer, reconnect after idle.
//   iso_m: decouple in the middle of a write burst (WSTRB=0 completion),
//          response dropping, drained.
// Prints TB_DFX_ISO_PASS on success.
`timescale 1ns/1ps
module tb_dfx_iso;
  logic clk = 0, rst_n = 0;
  always #2 clk = ~clk;
  int errors = 0;
  task automatic check(input bit c, input string msg);
    if (!c) begin errors++; $display("FAIL: %s @%0t", msg, $time); end
  endtask

  // ------------------------------------------------------------ iso_s (RM slave)
  logic s_dec;
  logic s_idle, s_err;
  logic [8:0] s_rd, s_wr;
  logic h_awv, h_awr, h_wv, h_wr, h_wl, h_bv, h_br, h_arv, h_arr, h_rv, h_rr, h_rl;
  logic [3:0] h_awid, h_arid, h_bid, h_rid;
  logic [7:0] h_arlen;
  logic [1:0] h_bresp, h_rresp;
  logic [63:0] h_rdata;
  logic r_awv, r_awr, r_wv, r_wr, r_wl, r_bv, r_br, r_arv, r_arr, r_rv, r_rr, r_rl;
  logic [3:0] r_bid, r_rid;
  logic [1:0] r_bresp, r_rresp;
  logic [63:0] r_rdata;
  logic [15:0] r_aw_pl, r_ar_pl;
  logic [71:0] r_w_pl;

  ng_dfx_iso_s #(.ID_W(4), .DATA_W(64), .AW_PL_W(16), .W_PL_W(72), .AR_PL_W(16),
                 .MAX_RD(4), .MAX_WR(4)) dut_s (
    .clk(clk), .rst_n(rst_n), .dec(s_dec), .idle(s_idle), .err_pulse(s_err), .rd_out(s_rd), .wr_out(s_wr),
    .sh_aw_valid(h_awv), .sh_aw_ready(h_awr), .sh_aw_pl(16'h0), .sh_aw_id(h_awid),
    .sh_w_valid(h_wv), .sh_w_ready(h_wr), .sh_w_pl(72'h0), .sh_w_last(h_wl),
    .sh_b_valid(h_bv), .sh_b_ready(h_br), .sh_b_id(h_bid), .sh_b_resp(h_bresp),
    .sh_ar_valid(h_arv), .sh_ar_ready(h_arr), .sh_ar_pl(16'h0), .sh_ar_id(h_arid), .sh_ar_len(h_arlen),
    .sh_r_valid(h_rv), .sh_r_ready(h_rr), .sh_r_id(h_rid), .sh_r_data(h_rdata), .sh_r_resp(h_rresp), .sh_r_last(h_rl),
    .rm_aw_valid(r_awv), .rm_aw_ready(r_awr), .rm_aw_pl(r_aw_pl),
    .rm_w_valid(r_wv), .rm_w_ready(r_wr), .rm_w_pl(r_w_pl), .rm_w_last(r_wl),
    .rm_b_valid(r_bv), .rm_b_ready(r_br), .rm_b_id(r_bid), .rm_b_resp(r_bresp),
    .rm_ar_valid(r_arv), .rm_ar_ready(r_arr), .rm_ar_pl(r_ar_pl),
    .rm_r_valid(r_rv), .rm_r_ready(r_rr), .rm_r_id(r_rid), .rm_r_data(r_rdata), .rm_r_resp(r_rresp), .rm_r_last(r_rl));

  // ------------------------------------------------------------ iso_m (RM master)
  logic m_dec, m_drained;
  logic [8:0] m_rd, m_wr;
  logic a_awv, a_awr, a_wv, a_wr, a_bv, a_br, a_arv, a_arr, a_rv, a_rr, a_rl;
  logic [7:0] a_awlen;
  logic [15:0] a_w_pl;
  logic x_awv, x_awr, x_wv, x_wr, x_wl, x_bv, x_br, x_arv, x_arr, x_rv, x_rr, x_rl;
  logic [15:0] x_w_pl;
  logic [7:0] x_aw_pl, x_ar_pl;

  ng_dfx_iso_m #(.AW_PL_W(8), .W_PL_W(16), .B_PL_W(2), .AR_PL_W(8), .R_PL_W(8),
                 .MAX_RD(4), .MAX_WR(4)) dut_m (
    .clk(clk), .rst_n(rst_n), .dec(m_dec), .drained(m_drained), .rd_out(m_rd), .wr_out(m_wr),
    .rm_aw_valid(a_awv), .rm_aw_ready(a_awr), .rm_aw_pl(8'h0), .rm_aw_len(a_awlen),
    .rm_w_valid(a_wv), .rm_w_ready(a_wr), .rm_w_pl(a_w_pl),
    .rm_b_valid(a_bv), .rm_b_ready(a_br), .rm_b_pl(),
    .rm_ar_valid(a_arv), .rm_ar_ready(a_arr), .rm_ar_pl(8'h0),
    .rm_r_valid(a_rv), .rm_r_ready(a_rr), .rm_r_pl(), .rm_r_last(a_rl),
    .sh_aw_valid(x_awv), .sh_aw_ready(x_awr), .sh_aw_pl(x_aw_pl),
    .sh_w_valid(x_wv), .sh_w_ready(x_wr), .sh_w_pl(x_w_pl), .sh_w_last(x_wl),
    .sh_b_valid(x_bv), .sh_b_ready(x_br), .sh_b_pl(2'b00),
    .sh_ar_valid(x_arv), .sh_ar_ready(x_arr), .sh_ar_pl(x_ar_pl),
    .sh_r_valid(x_rv), .sh_r_ready(x_rr), .sh_r_pl(8'h0), .sh_r_last(x_rl));

  // shell side of iso_m: always ready; count W beats
  int w_beats = 0, w_zero = 0, w_last = 0;
  assign x_awr = 1'b1;
  assign x_wr  = 1'b1;
  assign x_arr = 1'b1;
  always @(posedge clk) if (x_wv && x_wr) begin
    w_beats++;
    if (x_w_pl == 16'h0) w_zero++;
    if (x_wl) w_last++;
  end

  task automatic tick(input int n = 1);
    repeat (n) @(posedge clk);
    #0.5;
  endtask

  initial begin
    // idle defaults
    {h_awv, h_wv, h_wl, h_arv} = '0;
    h_br = 1; h_rr = 1; h_awid = 0; h_arid = 0; h_arlen = 0;
    {r_awr, r_wr, r_arr} = 3'b111;
    {r_bv, r_rv, r_rl} = '0; r_bid = 0; r_rid = 0; r_bresp = 0; r_rresp = 0; r_rdata = 0;
    {a_awv, a_wv, a_arv} = '0; a_br = 1; a_rr = 1; a_awlen = 0; a_w_pl = 0;
    {x_bv, x_rv, x_rl} = '0;
    s_dec = 0; m_dec = 0;
    tick(3); rst_n = 1; tick(8);
    check(!dut_s.iso && !dut_m.iso, "both blocks reconnect after reset");

    // ---------------- A: normal read, 4 beats from the RM
    h_arv = 1; h_arid = 4'h5; h_arlen = 3; tick(); h_arv = 0;
    check(s_rd == 1, "A: read tracked");
    for (int i = 0; i < 4; i++) begin
      r_rv = 1; r_rid = 4'h5; r_rdata = 64'h100 + i; r_rl = (i == 3);
      #0.1 check(h_rv && h_rdata == 64'h100 + i && h_rresp == 2'b00 && h_rid == 4'h5 && h_rl == (i == 3), $sformatf("A: beat %0d", i));
      tick();
    end
    r_rv = 0; r_rl = 0;
    check(s_rd == 0 && s_idle, "A: idle after read");

    // ---------------- B: RM hangs after one beat, forced decouple
    h_arv = 1; h_arid = 4'h7; h_arlen = 3; tick(); h_arv = 0;
    r_rv = 1; r_rid = 4'h7; r_rdata = 64'hAA; tick(); r_rv = 0;
    s_dec = 1; tick();
    begin
      int got = 0;
      for (int i = 0; i < 10 && got < 3; i++) begin
        if (h_rv) begin
          check(h_rresp == 2'b10 && h_rdata == '1 && h_rid == 4'h7, "B: local beat is SLVERR/all-ones");
          got++;
          if (got == 3) check(h_rl, "B: RLAST on the 4th beat");
        end
        tick();
      end
      check(got == 3, "B: remaining 3 beats answered locally");
    end
    check(s_rd == 0, "B: queue empty");

    // ---------------- C: write while decoupled -> local SLVERR
    h_awv = 1; h_awid = 4'h3; tick(); h_awv = 0;
    h_wv = 1; h_wl = 0; tick(); h_wl = 1; tick(); h_wv = 0; h_wl = 0;
    begin
      int seen = 0;
      for (int i = 0; i < 5; i++) begin
        if (h_bv) begin check(h_bresp == 2'b10 && h_bid == 4'h3, "C: B SLVERR with the right ID"); seen++; end
        tick();
      end
      check(seen == 1, "C: exactly one B");
    end
    check(r_awv == 0 && r_wv == 0, "C: nothing reached the RM");

    // ---------------- D: reconnect only after idle, then traffic reaches the RM
    s_dec = 0; tick(4);
    check(!dut_s.iso, "D: reconnected");
    h_arv = 1; h_arid = 4'h1; h_arlen = 0;
    #0.1 check(r_arv, "D: AR forwarded to the RM");
    tick(); h_arv = 0;
    r_rv = 1; r_rid = 4'h1; r_rdata = 64'h55; r_rl = 1;
    #0.1 check(h_rv && h_rdata == 64'h55 && h_rresp == 2'b00, "D: RM data returned");
    tick(); r_rv = 0; r_rl = 0;

    // ---------------- E: RM decoupled in the middle of a 4-beat write burst
    a_awv = 1; a_awlen = 3; tick(); a_awv = 0;
    a_wv = 1; a_w_pl = 16'hBEEF; tick(); a_wv = 0;
    check(w_beats == 1 && w_zero == 0, "E: first beat from the RM");
    m_dec = 1; tick(6);
    check(w_beats == 4 && w_zero == 3 && w_last == 1, "E: burst completed with 3 zero beats and one WLAST");
    check(!m_drained, "E: not drained before B");
    x_bv = 1;
    #0.1 check(x_br && !a_bv, "E: B accepted by the shell side, hidden from the RM");
    tick(); x_bv = 0; tick();
    check(m_drained, "E: drained after B");

    // ---------------- F: read response while decoupled is dropped
    m_dec = 0; tick(2);
    a_arv = 1; tick(); a_arv = 0;
    check(m_rd == 1, "F: read outstanding");
    m_dec = 1; tick();
    x_rv = 1; x_rl = 1;
    #0.1 check(x_rr && !a_rv, "F: R dropped while decoupled");
    tick(); x_rv = 0; x_rl = 0; tick();
    check(m_drained, "F: drained");
    a_awv = 1;
    #0.1 check(!x_awv && !a_awr, "F: new RM request blocked while decoupled");
    a_awv = 0;

    if (errors == 0) $display("TB_DFX_ISO_PASS");
    else $display("TB_DFX_ISO_FAIL errors=%0d", errors);
    $finish;
  end
endmodule
