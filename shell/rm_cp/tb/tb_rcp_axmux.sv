// tb_rcp_axmux.sv -- unit test of ng_rcp_axmux (2 masters -> 1 ID-less AXI slave).
//
// tb_axmux_core: two random masters (reads and writes, 1..64 beats, several transactions outstanding, random
// valid/ready stalls) -> ng_rcp_axmux -> slave model with a byte memory, random AR/AW/W ready, random response
// latency, strictly in-order responses and at most DEPTH outstanding per direction, injected SLVERR responses.
// Address map of the test: bit 40 = master id, bit 41 = read (read-only region, data = hash(address)) so a mis-routed
// beat or response is always detectable.  Checks: per master own R data / rresp / rlast in order, own B responses,
// W beats (data, strobes, wlast) at the slave arrive in AW order from the right master and are never sent before the
// AW, the final memory equals the masters' shadow, valid/payload hold on every channel, idle == nothing outstanding.
// Top: five instantiations (DW=64 DEPTH=16, DW=128 DEPTH=16, DW=64 DEPTH=4 -> exercises the FIFO-full path, and the
// REG_A variants DW=64 DEPTH=16 / DW=128 DEPTH=4: slave limit DEPTH, mux FIFOs DEPTH+2, idle == nothing at the slave).
// Seed: read from ./seed.txt (written by the runner; xsim.bat cannot pass +seed=N), default 1.  Pass token NG_RCP_AXMUX_TB_PASS.

package axtb_pkg;
  int errs = 0;
  task automatic fail(input string s);
    errs++;
    if (errs <= 20) $display("TB_FAIL @%0t: %s", $time, s);
  endtask
endpackage

// valid must stay high with a stable payload until the handshake
module axmux_hold #(parameter int W = 1, parameter string NAME = "") (
  input logic clk, input logic rst_n, input logic v, input logic r, input logic [W-1:0] p
);
  logic pv, pr;
  logic [W-1:0] pp;
  always @(posedge clk) begin
    if (!rst_n) begin
      pv <= 1'b0; pr <= 1'b1;
    end else begin
      if (v === 1'bx || r === 1'bx) axtb_pkg::fail({NAME, ": X on valid/ready"});
      if (pv && !pr) begin
        if (v !== 1'b1) axtb_pkg::fail({NAME, ": valid dropped before ready"});
        else if (p !== pp) axtb_pkg::fail({NAME, ": payload changed while stalled"});
      end
      pv <= v; pr <= r; pp <= p;
    end
  end
endmodule

module tb_axmux_core #(
  parameter int DW    = 64,
  parameter int AW    = 47,
  parameter int DEPTH = 16,
  parameter bit REG_A = 1'b0,
  parameter int NTXN  = 300,
  parameter int unsigned SEED = 1
) (
  input  logic clk,
  input  logic rst_n,
  output logic fin
);
  localparam int B    = DW / 8;
  localparam int MAXO = 6;           // per master outstanding reads / writes
  localparam int KMID = 40, KRD = 41;

  // ----------------------------------------------------------
  int unsigned rs;
  function automatic int unsigned rnd();
    rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5;
    return rs;
  endfunction
  function automatic int rn(input int n);
    return int'(rnd() % unsigned'(n));
  endfunction
  function automatic logic [63:0] mix(input logic [63:0] xin);
    logic [63:0] x;
    x = xin;
    x ^= x >> 33; x *= 64'hff51afd7ed558ccd; x ^= x >> 33; x *= 64'hc4ceb9fe1a85ec53; x ^= x >> 33;
    return x;
  endfunction
  function automatic logic [DW-1:0] hsh(input logic [63:0] a, input logic [63:0] salt);
    logic [127:0] r;
    r = {mix(a * 64'd31 + salt + 64'd1), mix(a * 64'd31 + salt)};
    return r[DW-1:0];
  endfunction
  function automatic logic [1:0] errf(input logic [63:0] a, input logic [63:0] salt);
    return (mix(a ^ salt) % 8 == 0) ? 2'b10 : 2'b00;
  endfunction
  function automatic logic [B-1:0] strbf(input int m, input int seq, input int beat);
    logic [127:0] r;
    r = {mix(64'(m) * 64'd100003 + 64'(seq) * 64'd1009 + 64'(beat) + 64'h77),
         mix(64'(m) * 64'd100003 + 64'(seq) * 64'd1009 + 64'(beat))};
    // sometimes all ones, sometimes sparse
    return (r[3:0] < 4'd5) ? {B{1'b1}} : r[8 +: B];
  endfunction
  function automatic logic [DW-1:0] wdf(input int m, input int seq, input int beat);
    return hsh(64'(m) * 64'd1000003 + 64'(seq) * 64'd2003 + 64'(beat), 64'h5555);
  endfunction

  // traffic shape, derived from the seed (1 = rare ... 100 = every cycle)
  int unsigned ps;
  int p_iss, p_w, p_r, p_b, p_arr, p_awr, p_wr, p_sr, p_sb, lat_max;
  function automatic int pick3(input int a, input int b2, input int c);
    int k;
    k = rn(3);
    return (k == 0) ? a : (k == 1) ? b2 : c;
  endfunction
  initial begin
    ps = 1;
    begin
      int fd, rc;
      fd = $fopen("seed.txt", "r");
      if (fd != 0) begin rc = $fscanf(fd, "%d", ps); $fclose(fd); end
    end
    rs = (SEED + ps * 7919) * 32'h9e3779b1 + 32'd12345 + DW + DEPTH;
    if (rs == 0) rs = 1;
    repeat (8) void'(rnd());
    p_iss = pick3(20, 60, 100); p_w = pick3(30, 70, 100); p_r = pick3(30, 70, 100); p_b = pick3(30, 70, 100);
    p_arr = pick3(30, 70, 100); p_awr = pick3(30, 70, 100); p_wr = pick3(30, 70, 100);
    p_sr = pick3(40, 75, 100); p_sb = pick3(40, 75, 100);
    lat_max = pick3(0, 4, 24);
    $display("core DW=%0d DEPTH=%0d seed=%0d: p_iss=%0d p_w=%0d p_r=%0d p_b=%0d arr=%0d awr=%0d wr=%0d sr=%0d sb=%0d lat=%0d",
             DW, DEPTH, SEED, p_iss, p_w, p_r, p_b, p_arr, p_awr, p_wr, p_sr, p_sb, lat_max);
  end

  // ----------------------------------------------------------
  logic [7:0] shadow [longint unsigned];
  logic [7:0] mem    [longint unsigned];

  // ------------------------------------------------------------ DUT signals
  logic            s_awvalid [2]; logic            s_awready [2]; logic [AW-1:0]   s_awaddr [2]; logic [7:0] s_awlen [2];
  logic            s_wvalid  [2]; logic            s_wready  [2]; logic [DW-1:0]   s_wdata  [2]; logic [B-1:0] s_wstrb [2];
  logic            s_wlast   [2];
  logic            s_bvalid  [2]; logic            s_bready  [2]; logic [1:0]      s_bresp  [2];
  logic            s_arvalid [2]; logic            s_arready [2]; logic [AW-1:0]   s_araddr [2]; logic [7:0] s_arlen [2];
  logic            s_rvalid  [2]; logic            s_rready  [2]; logic [DW-1:0]   s_rdata  [2]; logic [1:0] s_rresp [2];
  logic            s_rlast   [2];

  logic            m_awvalid, m_awready, m_wvalid, m_wready, m_wlast, m_bvalid, m_bready;
  logic            m_arvalid, m_arready, m_rvalid, m_rready, m_rlast;
  logic [AW-1:0]   m_awaddr, m_araddr;
  logic [7:0]      m_awlen, m_arlen;
  logic [DW-1:0]   m_wdata, m_rdata;
  logic [B-1:0]    m_wstrb;
  logic [1:0]      m_bresp, m_rresp;
  logic            idle;

  ng_rcp_axmux #(.DW(DW), .AW(AW), .DEPTH(DEPTH + (REG_A ? 2 : 0)), .REG_A(REG_A)) dut (
    .clk, .rst_n, .idle,
    .s0_awvalid(s_awvalid[0]), .s0_awready(s_awready[0]), .s0_awaddr(s_awaddr[0]), .s0_awlen(s_awlen[0]),
    .s0_wvalid(s_wvalid[0]), .s0_wready(s_wready[0]), .s0_wdata(s_wdata[0]), .s0_wstrb(s_wstrb[0]), .s0_wlast(s_wlast[0]),
    .s0_bvalid(s_bvalid[0]), .s0_bready(s_bready[0]), .s0_bresp(s_bresp[0]),
    .s0_arvalid(s_arvalid[0]), .s0_arready(s_arready[0]), .s0_araddr(s_araddr[0]), .s0_arlen(s_arlen[0]),
    .s0_rvalid(s_rvalid[0]), .s0_rready(s_rready[0]), .s0_rdata(s_rdata[0]), .s0_rresp(s_rresp[0]), .s0_rlast(s_rlast[0]),
    .s1_awvalid(s_awvalid[1]), .s1_awready(s_awready[1]), .s1_awaddr(s_awaddr[1]), .s1_awlen(s_awlen[1]),
    .s1_wvalid(s_wvalid[1]), .s1_wready(s_wready[1]), .s1_wdata(s_wdata[1]), .s1_wstrb(s_wstrb[1]), .s1_wlast(s_wlast[1]),
    .s1_bvalid(s_bvalid[1]), .s1_bready(s_bready[1]), .s1_bresp(s_bresp[1]),
    .s1_arvalid(s_arvalid[1]), .s1_arready(s_arready[1]), .s1_araddr(s_araddr[1]), .s1_arlen(s_arlen[1]),
    .s1_rvalid(s_rvalid[1]), .s1_rready(s_rready[1]), .s1_rdata(s_rdata[1]), .s1_rresp(s_rresp[1]), .s1_rlast(s_rlast[1]),
    .m_awvalid, .m_awready, .m_awaddr, .m_awlen, .m_wvalid, .m_wready, .m_wdata, .m_wstrb, .m_wlast,
    .m_bvalid, .m_bready, .m_bresp, .m_arvalid, .m_arready, .m_araddr, .m_arlen,
    .m_rvalid, .m_rready, .m_rdata, .m_rresp, .m_rlast);

  // ------------------------------------------------------------ protocol monitors
  for (genvar k = 0; k < 2; k++) begin : gh
    axmux_hold #(AW + 8, "master AW") h_aw (clk, rst_n, s_awvalid[k], s_awready[k], {s_awaddr[k], s_awlen[k]});
    axmux_hold #(AW + 8, "master AR") h_ar (clk, rst_n, s_arvalid[k], s_arready[k], {s_araddr[k], s_arlen[k]});
    axmux_hold #(DW + B + 1, "master W") h_w (clk, rst_n, s_wvalid[k], s_wready[k], {s_wdata[k], s_wstrb[k], s_wlast[k]});
    axmux_hold #(2, "mux B out") h_b (clk, rst_n, s_bvalid[k], s_bready[k], s_bresp[k]);
    axmux_hold #(DW + 3, "mux R out") h_r (clk, rst_n, s_rvalid[k], s_rready[k], {s_rdata[k], s_rresp[k], s_rlast[k]});
  end
  axmux_hold #(AW + 8, "mux AW out") h_maw (clk, rst_n, m_awvalid, m_awready, {m_awaddr, m_awlen});
  axmux_hold #(AW + 8, "mux AR out") h_mar (clk, rst_n, m_arvalid, m_arready, {m_araddr, m_arlen});
  axmux_hold #(DW + B + 1, "mux W out") h_mw (clk, rst_n, m_wvalid, m_wready, {m_wdata, m_wstrb, m_wlast});
  axmux_hold #(2, "slave B") h_mb (clk, rst_n, m_bvalid, m_bready, m_bresp);
  axmux_hold #(DW + 3, "slave R") h_mr (clk, rst_n, m_rvalid, m_rready, {m_rdata, m_rresp, m_rlast});

  // ----------------------------------------------------------
  logic mdone [2];
  int   nw_done [2], nr_done [2];

  for (genvar i = 0; i < 2; i++) begin : gm
    typedef struct { logic [AW-1:0] addr; int len; } rd_t;
    typedef struct { logic [AW-1:0] addr; int len; int seq; } wr_t;
    rd_t         rdq [$];
    wr_t         wq  [$];
    logic [1:0]  bq  [$];
    int rd_out, wr_out, issued, wseq, w_beat, r_beat;
    wr_t wt;
    rd_t rt;
    logic [AW-1:0] a;
    int len, rw, bt;
    logic [63:0] ba;
    logic [B-1:0] sb;
    logic [DW-1:0] sd;

    always @(posedge clk) begin
      if (!rst_n) begin
        s_awvalid[i] <= 1'b0; s_wvalid[i] <= 1'b0; s_arvalid[i] <= 1'b0; s_bready[i] <= 1'b0; s_rready[i] <= 1'b0;
        s_awaddr[i] <= '0; s_awlen[i] <= '0; s_wdata[i] <= '0; s_wstrb[i] <= '0; s_wlast[i] <= 1'b0;
        s_araddr[i] <= '0; s_arlen[i] <= '0;
        rd_out = 0; wr_out = 0; issued = 0; wseq = 0; w_beat = 0; r_beat = 0;
        nw_done[i] = 0; nr_done[i] = 0; mdone[i] <= 1'b0;
        rdq.delete(); wq.delete(); bq.delete();
      end else begin
        // ---- issue read / write address (transaction is queued when presented)
        if (!(s_arvalid[i] && !s_arready[i])) s_arvalid[i] <= 1'b0;
        if (!(s_awvalid[i] && !s_awready[i])) s_awvalid[i] <= 1'b0;
        rw = rn(2);
        len = 1 + rn(64);
        if (rn(4) == 0) len = 1 + rn(4);
        a = (AW'(i) << KMID) | (AW'(rn(4096)) * B);
        if (issued < NTXN && rn(100) < p_iss) begin
          if (rw == 0 && rd_out < MAXO && !(s_arvalid[i] && !s_arready[i])) begin
            a |= AW'(1) << KRD;
            s_arvalid[i] <= 1'b1; s_araddr[i] <= a; s_arlen[i] <= 8'(len - 1);
            rt.addr = a; rt.len = len; rdq.push_back(rt);
            rd_out++; issued++;
          end else if (rw == 1 && wr_out < MAXO && !(s_awvalid[i] && !s_awready[i])) begin
            s_awvalid[i] <= 1'b1; s_awaddr[i] <= a; s_awlen[i] <= 8'(len - 1);
            wt.addr = a; wt.len = len; wt.seq = wseq; wq.push_back(wt);
            bq.push_back(errf(64'(a), 64'hB0B0));
            for (int b2 = 0; b2 < len; b2++) begin
              sb = strbf(i, wseq, b2);
              sd = wdf(i, wseq, b2);
              for (int k = 0; k < B; k++)
                if (sb[k]) shadow[longint'(a) + b2 * B + k] = sd[8*k +: 8];
            end
            wseq++; wr_out++; issued++;
          end
        end

        // ---- W data (may run ahead of the AW handshake)
        if (s_wvalid[i] && s_wready[i]) begin
          if (w_beat == wq[0].len - 1) begin wq.pop_front(); w_beat = 0; end else w_beat++;
        end
        if (!(s_wvalid[i] && !s_wready[i])) begin
          s_wvalid[i] <= 1'b0;
          if (wq.size() > 0 && rn(100) < p_w) begin
            s_wvalid[i] <= 1'b1;
            s_wdata[i]  <= wdf(i, wq[0].seq, w_beat);
            s_wstrb[i]  <= strbf(i, wq[0].seq, w_beat);
            s_wlast[i]  <= (w_beat == wq[0].len - 1);
          end
        end

        // --
        s_bready[i] <= (rn(100) < p_b);
        if (s_bvalid[i] && s_bready[i]) begin
          if (bq.size() == 0) axtb_pkg::fail($sformatf("master %0d: unexpected B", i));
          else begin
            if (s_bresp[i] !== bq[0]) axtb_pkg::fail($sformatf("master %0d: bresp %0d exp %0d", i, s_bresp[i], bq[0]));
            void'(bq.pop_front());
          end
          wr_out--; nw_done[i]++;
        end

        // --
        s_rready[i] <= (rn(100) < p_r);
        if (s_rvalid[i] && s_rready[i]) begin
          if (rdq.size() == 0) axtb_pkg::fail($sformatf("master %0d: unexpected R beat", i));
          else begin
            ba = 64'(rdq[0].addr) + 64'(r_beat * B);
            if (s_rdata[i] !== hsh(ba, 64'hAAAA)) axtb_pkg::fail($sformatf("master %0d: rdata mismatch addr %h", i, ba));
            if (s_rresp[i] !== errf(64'(rdq[0].addr), 64'hCCCC))
              axtb_pkg::fail($sformatf("master %0d: rresp %0d at %h", i, s_rresp[i], ba));
            if (s_rlast[i] !== (r_beat == rdq[0].len - 1))
              axtb_pkg::fail($sformatf("master %0d: rlast=%b at beat %0d of %0d", i, s_rlast[i], r_beat, rdq[0].len));
            if (r_beat == rdq[0].len - 1) begin void'(rdq.pop_front()); r_beat = 0; rd_out--; nr_done[i]++; end
            else r_beat++;
          end
        end

        mdone[i] <= (issued == NTXN) && (rd_out == 0) && (wr_out == 0);
      end
    end
  end

  // ------------------------------------------------------------ slave: AR / R
  typedef struct { logic [AW-1:0] addr; int len; logic [1:0] err; } sr_t;
  sr_t arq [$];
  int  ar_cnt, rbeat, rlat;
  bit  r_active;
  logic [63:0] sba;

  always @(posedge clk) begin
    if (!rst_n) begin
      m_arready <= 1'b0; m_rvalid <= 1'b0; m_rdata <= '0; m_rresp <= '0; m_rlast <= 1'b0;
      ar_cnt = 0; rbeat = 0; rlat = 0; r_active = 0; arq.delete();
    end else begin
      if (m_arvalid && m_arready) begin
        sr_t e;
        e.addr = m_araddr; e.len = int'(m_arlen) + 1; e.err = errf(64'(m_araddr), 64'hCCCC);
        if (m_araddr[KRD] !== 1'b1) axtb_pkg::fail("slave: AR outside the read region");
        arq.push_back(e); ar_cnt++;
      end
      if (m_rvalid && m_rready) begin
        if (m_rlast) begin void'(arq.pop_front()); ar_cnt--; r_active = 0; end else rbeat++;
      end
      if (!(m_rvalid && !m_rready)) begin
        m_rvalid <= 1'b0;
        if (!r_active && arq.size() > 0) begin r_active = 1; rbeat = 0; rlat = rn(lat_max + 1); end
        if (r_active) begin
          if (rlat > 0) rlat--;
          else if (rn(100) < p_sr) begin
            sba = 64'(arq[0].addr) + 64'(rbeat * B);
            m_rvalid <= 1'b1; m_rdata <= hsh(sba, 64'hAAAA); m_rresp <= arq[0].err;
            m_rlast <= (rbeat == arq[0].len - 1);
          end
        end
      end
      m_arready <= (rn(100) < p_arr) && (ar_cnt < DEPTH);
    end
  end

  // ------------------------------------------------------------ slave: AW / W / B
  typedef struct { logic [AW-1:0] addr; int len; int m; int seq; logic [1:0] err; } sw_t;
  sw_t awq [$];       // accepted AW, W burst not complete
  sw_t bqs [$];       // W complete, B outstanding
  int  aw_cnt, sw_beat, blat;
  int  wseqc [2];
  bit  b_active;

  always @(posedge clk) begin
    if (!rst_n) begin
      m_awready <= 1'b0; m_wready <= 1'b0; m_bvalid <= 1'b0; m_bresp <= '0;
      aw_cnt = 0; sw_beat = 0; blat = 0; b_active = 0; wseqc[0] = 0; wseqc[1] = 0; awq.delete(); bqs.delete();
    end else begin
      if (m_awvalid && m_awready) begin
        sw_t e;
        e.addr = m_awaddr; e.len = int'(m_awlen) + 1; e.m = int'(m_awaddr[KMID]);
        e.seq = wseqc[e.m]; wseqc[e.m] = wseqc[e.m] + 1;
        e.err = errf(64'(m_awaddr), 64'hB0B0);
        if (m_awaddr[KRD] !== 1'b0) axtb_pkg::fail("slave: AW inside the read region");
        awq.push_back(e); aw_cnt++;
      end
      if (m_wvalid && m_wready) begin
        if (awq.size() == 0) axtb_pkg::fail("slave: W beat before its AW was accepted");
        else begin
          sw_t e;
          e = awq[0];
          if (m_wdata !== wdf(e.m, e.seq, sw_beat))
            axtb_pkg::fail($sformatf("slave: W data mismatch (master %0d seq %0d beat %0d)", e.m, e.seq, sw_beat));
          if (m_wstrb !== strbf(e.m, e.seq, sw_beat))
            axtb_pkg::fail($sformatf("slave: W strobe mismatch (master %0d seq %0d beat %0d)", e.m, e.seq, sw_beat));
          if (m_wlast !== (sw_beat == e.len - 1))
            axtb_pkg::fail($sformatf("slave: wlast=%b at beat %0d of %0d", m_wlast, sw_beat, e.len));
          for (int k = 0; k < B; k++)
            if (m_wstrb[k]) mem[longint'(e.addr) + sw_beat * B + k] = m_wdata[8*k +: 8];
          if (sw_beat == e.len - 1) begin bqs.push_back(e); void'(awq.pop_front()); sw_beat = 0; end
          else sw_beat++;
        end
      end
      if (m_bvalid && m_bready) begin void'(bqs.pop_front()); aw_cnt--; b_active = 0; end
      if (!(m_bvalid && !m_bready)) begin
        m_bvalid <= 1'b0;
        if (!b_active && bqs.size() > 0) begin b_active = 1; blat = rn(lat_max + 1); end
        if (b_active) begin
          if (blat > 0) blat--;
          else if (rn(100) < p_sb) begin m_bvalid <= 1'b1; m_bresp <= bqs[0].err; end
        end
      end
      m_wready  <= (rn(100) < p_wr);
      m_awready <= (rn(100) < p_awr) && (aw_cnt < DEPTH);
    end
  end

  // ------------------------------------------------------------ idle must equal "nothing outstanding"
  always @(negedge clk) begin
    if (rst_n && (idle !== ((ar_cnt == 0) && (aw_cnt == 0))))
      axtb_pkg::fail($sformatf("idle=%b but ar_cnt=%0d aw_cnt=%0d", idle, ar_cnt, aw_cnt));
    if (rst_n && (ar_cnt > DEPTH || aw_cnt > DEPTH)) axtb_pkg::fail("slave outstanding limit exceeded");
  end

  // ------------------------------------------------------------ end of test checks
  int quiet;
  int nerr_mem;
  initial begin
    fin = 1'b0;
    quiet = 0;
    wait (rst_n === 1'b1);
    forever begin
      @(posedge clk);
      if (mdone[0] && mdone[1]) quiet++; else quiet = 0;
      if (quiet == 30) begin
        nerr_mem = 0;
        if (idle !== 1'b1) axtb_pkg::fail("idle not 1 at the end");
        if (shadow.size() != mem.size()) begin
          axtb_pkg::fail($sformatf("memory has %0d bytes, shadow %0d", mem.size(), shadow.size()));
        end
        foreach (shadow[k]) begin
          if (!mem.exists(k)) begin nerr_mem++; if (nerr_mem < 5) axtb_pkg::fail($sformatf("byte %h never written", k)); end
          else if (mem[k] !== shadow[k]) begin nerr_mem++; if (nerr_mem < 5) axtb_pkg::fail($sformatf("byte %h: %h exp %h", k, mem[k], shadow[k])); end
        end
        $display("core DW=%0d DEPTH=%0d REG_A=%0d: reads done %0d/%0d, writes done %0d/%0d, %0d bytes checked",
                 DW, DEPTH, REG_A, nr_done[0], nr_done[1], nw_done[0], nw_done[1], shadow.size());
        fin = 1'b1;
        break;
      end
    end
  end
endmodule

module tb_rcp_axmux;
  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic fin_a, fin_b, fin_c, fin_d, fin_e;

  always #2 clk = ~clk;

  tb_axmux_core #(.DW(64),  .DEPTH(16), .NTXN(300), .SEED(1)) u_a (.clk, .rst_n, .fin(fin_a));
  tb_axmux_core #(.DW(128), .DEPTH(16), .NTXN(300), .SEED(2)) u_b (.clk, .rst_n, .fin(fin_b));
  tb_axmux_core #(.DW(64),  .DEPTH(4),  .NTXN(300), .SEED(3)) u_c (.clk, .rst_n, .fin(fin_c));
  tb_axmux_core #(.DW(64),  .DEPTH(16), .REG_A(1'b1), .NTXN(300), .SEED(4)) u_d (.clk, .rst_n, .fin(fin_d));
  tb_axmux_core #(.DW(128), .DEPTH(4),  .REG_A(1'b1), .NTXN(300), .SEED(5)) u_e (.clk, .rst_n, .fin(fin_e));

  initial begin
    repeat (6) @(posedge clk);
    rst_n <= 1'b1;
    fork
      begin wait (fin_a && fin_b && fin_c && fin_d && fin_e); end
      begin repeat (3000000) @(posedge clk); axtb_pkg::fail("timeout"); end
    join_any
    repeat (5) @(posedge clk);
    if (axtb_pkg::errs == 0) $display("NG_RCP_AXMUX_TB_PASS");
    else $display("NG_RCP_AXMUX_TB_FAILED errs=%0d", axtb_pkg::errs);
    $finish;
  end
endmodule
