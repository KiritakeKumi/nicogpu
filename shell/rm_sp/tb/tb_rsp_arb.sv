// tb_rsp_arb.sv -- unit test of the clk_l2 arbiters of the SP RM (ng_rsp_mmuarb, ng_rsp_oparb).
// Two requesters with random think times and a resource with random accept / response latency.  Checks:
//   * every response goes to the requester that owns the transaction (request carries the owner id + a sequence
//     number, the model echoes it back), nothing lost or duplicated, the request fields are the owner's;
//   * fairness: while a requester is waiting, the other one is granted at most once (round-robin), so a requester
//     that keeps asking (CP polling WAIT_MEM) cannot starve the other one;
//   * a saturated phase (both requesters always asking) alternates the grants exactly.
// One process: at each negedge the TB drives all inputs (blocking), waits 1 ns for the combinational outputs, and
// evaluates the handshakes that the coming posedge will perform (Verilator --timing NBA-in-task rule).
// Pass: NG_RSP_ARB_TB_PASS.
`timescale 1ns / 1ps
module tb_rsp_arb;
  logic clk = 0, rst_n = 0;
  always #2 clk = ~clk;
  int errors = 0, checks = 0;
  function automatic void check(input bit c, input string s);
    checks++;
    if (!c) begin errors++; if (errors < 20) $display("FAIL @%0t: %s", $time, s); end
  endfunction

  // ---------------------------------------------------------------- MMU arbiter
  logic [1:0]  m_rv, m_rr, m_pv, m_pr;
  logic [3:0]  m_vm [2];
  logic [63:0] m_va [2];
  logic [1:0]  m_ac [2];
  logic        o_rv, o_rr, o_at, o_pv, o_pr;
  logic [3:0]  o_vm;
  logic [63:0] o_va;
  logic [1:0]  o_ac, m_dbg;
  logic [31:0] m_gnt [2];
  ng_rsp_mmuarb u_m (
    .clk(clk), .rst_n(rst_n), .req_valid(m_rv), .req_ready(m_rr), .req_vmid(m_vm), .req_va(m_va), .req_acc(m_ac),
    .req_atomic(2'b10), .resp_valid(m_pv), .resp_ready(m_pr),
    .m_req_valid(o_rv), .m_req_ready(o_rr), .m_req_vmid(o_vm), .m_req_va(o_va), .m_req_acc(o_ac),
    .m_req_atomic(o_at), .m_resp_valid(o_pv), .m_resp_ready(o_pr), .dbg_st(m_dbg), .grants(m_gnt));

  // ---------------------------------------------------------------- op arbiter
  logic [1:0]  p_v, p_r, p_rs, p_wr;
  logic [46:0] p_a [2];
  logic [7:0]  p_s [2];
  logic [63:0] p_d [2];
  logic        e_v, e_r, e_wr, e_rs;
  logic [46:0] e_a;
  logic [7:0]  e_s;
  logic [63:0] e_d;
  logic [1:0]  p_dbg;
  logic [31:0] p_gnt [2];
  ng_rsp_oparb #(.AW(47)) u_o (
    .clk(clk), .rst_n(rst_n), .op_valid(p_v), .op_ready(p_r), .op_wr(p_wr), .op_addr(p_a), .op_wstrb(p_s),
    .op_wdata(p_d), .rs_valid(p_rs), .e_valid(e_v), .e_ready(e_r), .e_wr(e_wr), .e_addr(e_a), .e_wstrb(e_s),
    .e_wdata(e_d), .e_rs_valid(e_rs), .dbg_st(p_dbg), .grants(p_gnt));

  // resource models
  int          mm_st, mm_lat, oe_st, oe_lat;
  logic [63:0] mm_va;
  logic [46:0] oe_a;
  // requesters [arb][req]: arb 0 = MMU arbiter, arb 1 = op arbiter
  logic [1:0]  s_m_rv, s_m_rr, s_m_pv, s_p_v, s_p_r, s_p_rs;
  logic        s_o_rv, s_o_rr, s_o_at, s_o_pr, s_e_v, s_e_r, s_e_wr;
  logic [63:0] s_o_va, s_e_d;
  logic [3:0]  s_o_vm;
  logic [1:0]  s_o_ac;
  logic [46:0] s_e_a;
  logic [7:0]  s_e_s;
  int r_seq [2][2], r_exp [2][2], r_wait [2][2], r_think [2][2], n_done [2][2], sat0 [2][2];
  bit r_out [2][2];

  initial begin
    bit sat;
    mm_st = 0; oe_st = 0; mm_lat = 0; oe_lat = 0; mm_va = 0; oe_a = 0;
    for (int a = 0; a < 2; a++) for (int r = 0; r < 2; r++) begin
      r_seq[a][r] = 0; r_exp[a][r] = 0; r_out[a][r] = 0; r_wait[a][r] = 0; r_think[a][r] = 0; n_done[a][r] = 0;
    end
    m_rv = 0; m_pr = 2'b11; p_v = 0; p_wr = 0; o_rr = 0; o_pv = 0; e_r = 0; e_rs = 0;
    for (int r = 0; r < 2; r++) begin
      m_vm[r] = 4'(r + 1); m_va[r] = 0; m_ac[r] = 2'(r); p_a[r] = 0; p_s[r] = 8'(r + 1); p_d[r] = 64'(r);
    end
    repeat (5) @(negedge clk);
    rst_n = 1;
    @(negedge clk);
    for (int cyc = 0; cyc < 60000; cyc++) begin
      sat = (cyc >= 30000);
      if (cyc == 30000) for (int a = 0; a < 2; a++) for (int r = 0; r < 2; r++) sat0[a][r] = n_done[a][r];
      // ---- drive: resource outputs from their state
      o_pv = (mm_st == 2);
      o_rr = (mm_st == 0) && ($urandom_range(0, 2) == 0);
      e_rs = (oe_st == 1) && (oe_lat == 0);
      e_r  = (oe_st == 0) && ($urandom_range(0, 1) == 0);
      // ---- drive: requesters start a new transaction
      for (int r = 0; r < 2; r++) begin
        if (!r_out[0][r] && !m_rv[r]) begin
          if (sat || r_think[0][r] == 0) begin
            m_rv[r] = 1; m_va[r] = {32'(r), 32'(r_seq[0][r])}; r_wait[0][r] = 0;
          end else r_think[0][r]--;
        end
        if (!r_out[1][r] && !p_v[r]) begin
          if (sat || r_think[1][r] == 0) begin
            p_v[r] = 1; p_a[r] = {15'h0, 16'(r), 16'(r_seq[1][r])}; p_wr[r] = r_seq[1][r][0]; r_wait[1][r] = 0;
          end else r_think[1][r]--;
        end
      end
      #1;
      // ---- snapshot what the coming posedge sees, apply the consequences after it
      s_m_rv = m_rv; s_m_rr = m_rr; s_m_pv = m_pv; s_o_rv = o_rv; s_o_rr = o_rr; s_o_va = o_va; s_o_vm = o_vm;
      s_o_ac = o_ac; s_o_at = o_at; s_o_pr = o_pr; s_p_v = p_v; s_p_r = p_r; s_p_rs = p_rs; s_e_v = e_v; s_e_r = e_r;
      s_e_a = e_a; s_e_s = e_s; s_e_d = e_d; s_e_wr = e_wr;
      @(posedge clk);
      #0.5;
      // ---- evaluate the handshakes of the coming posedge
      if ($countones(s_m_pv) > 1 || $countones(s_p_rs) > 1) check(0, "two responses in one cycle");
      if ($countones(s_m_rr) > 1 || $countones(s_p_r) > 1) check(0, "two grants in one cycle");
      // MMU resource
      if (mm_st == 0 && s_o_rv && s_o_rr) begin
        check(s_o_vm == m_vm[s_o_va[32]] && s_o_ac == m_ac[s_o_va[32]] && s_o_at == s_o_va[32], "mmu arb: request fields of the owner");
        mm_va = s_o_va; mm_lat = $urandom_range(0, 6); mm_st = 1;
      end else if (mm_st == 1) begin
        if (mm_lat > 0) mm_lat--; else mm_st = 2;
      end else if (mm_st == 2 && s_o_pr) mm_st = 0;
      // op engine
      if (oe_st == 0 && s_e_v && s_e_r) begin
        check(s_e_s == p_s[s_e_a[16]] && s_e_d == p_d[s_e_a[16]], "op arb: request fields of the owner");
        oe_a = s_e_a; oe_lat = $urandom_range(0, 8); oe_st = 1;
      end else if (oe_st == 1) begin
        if (oe_lat > 0) oe_lat--; else oe_st = 0;
      end
      for (int r = 0; r < 2; r++) begin
        if (s_m_rv[r] && s_m_rr[r]) begin
          check(s_o_va == m_va[r], "mmu arb: va of the owner");
          check(r_wait[0][r] <= 1, $sformatf("mmu arb: requester %0d waited %0d grants of the other", r, r_wait[0][r]));
          m_rv[r] = 0; r_out[0][r] = 1; r_exp[0][r] = r_seq[0][r]; r_seq[0][r]++;
          if (s_m_rv[1 - r]) r_wait[0][1 - r]++;
        end
        if (s_m_pv[r]) begin
          check(r_out[0][r] && mm_va == {32'(r), 32'(r_exp[0][r])}, $sformatf("mmu arb: response to requester %0d", r));
          r_out[0][r] = 0; n_done[0][r]++; r_think[0][r] = $urandom_range(0, 12);
        end
        if (s_p_v[r] && s_p_r[r]) begin
          check(s_e_a == p_a[r] && s_e_wr == p_wr[r], "op arb: request fields of the owner");
          check(r_wait[1][r] <= 1, $sformatf("op arb: requester %0d waited %0d grants of the other", r, r_wait[1][r]));
          p_v[r] = 0; r_out[1][r] = 1; r_exp[1][r] = r_seq[1][r]; r_seq[1][r]++;
          if (s_p_v[1 - r]) r_wait[1][1 - r]++;
        end
        if (s_p_rs[r]) begin
          check(r_out[1][r] && oe_a == {15'h0, 16'(r), 16'(r_exp[1][r])}, $sformatf("op arb: response to requester %0d", r));
          r_out[1][r] = 0; n_done[1][r]++; r_think[1][r] = $urandom_range(0, 12);
        end
      end
      @(negedge clk);
    end
    for (int a = 0; a < 2; a++) begin
      int d0, d1;
      d0 = n_done[a][0] - sat0[a][0]; d1 = n_done[a][1] - sat0[a][1];
      $display("arb %0d (%s): random phase %0d / %0d, saturated phase %0d / %0d transactions (CP / SP)", a,
               a == 0 ? "mmu" : "op", sat0[a][0], sat0[a][1], d0, d1);
      check(sat0[a][0] > 100 && sat0[a][1] > 100, "random phase: both requesters served");
      check(d0 > 100 && (d0 - d1 <= 1) && (d1 - d0 <= 1), "saturated phase: grants alternate");
    end
    begin
      int busy;
      busy = n_done[0][0] + n_done[0][1] + r_out[0][0] + r_out[0][1];
      check(m_gnt[0] + m_gnt[1] >= 32'(busy) && m_gnt[0] + m_gnt[1] <= 32'(busy + 1), "mmu grant counters");
    end
    $display("tb_rsp_arb: %0d checks, %0d errors", checks, errors);
    if (errors == 0) $display("NG_RSP_ARB_TB_PASS"); else $display("NG_RSP_ARB_TB_FAIL");
    $finish;
  end
endmodule
