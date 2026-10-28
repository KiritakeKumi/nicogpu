// tb_bar_router.sv -- directed + soak testbench for ng_bar_router (P0-11).
//
// Driver: a simple AXI4 master that pushes transactions into queues; slave
// models on m0/m1/m2 with memory semantics, programmable response delay and
// a pause/release control (for the clock-stop orphan cases).  Checks:
//   D1  basic write/read on each BAR (data integrity via golden memories)
//   D10 BAR2 AxADDR alignment (P0-12 ruling): AxSIZE=3 -> AxADDR[2:0] cleared
//       on m2 (read and write); AxSIZE<3 and BAR0/BAR1 pass through untouched
//   D2  ordering fence: BAR0 write/read stall while a BAR2 write's B is
//       held off; released after B; BAR2 pipelining itself is NOT fenced
//   D3  BAR1 is never fenced
//   D4  unmapped window -> SLVERR (write B and single-beat read R)
//   D5  clock stop while idle: reads -> all-ones OKAY, writes dropped OKAY
//   D6  clock stop with an in-flight forwarded read: locally answered
//       immediately, late beats swallowed, fence_stuck pulses, resume only
//       after drain
//   D7  same with an in-flight forwarded write (B held off)
//   S1  soak: random targets, random read/write, random m-side delays and
//       random clock stops (also with in-flight traffic), golden compare,
//       watchdog on deadlock; every window honored
//   S2  pipelined soak: AW/AR issued back to back (up to 16 per direction,
//       unique IDs) to BAR0 (slow) / BAR2 (fast) / BAR1 / unmapped through
//       pipelined m-side models, random s-side back-pressure, clock drops
//       between writes; both ring queues wrap many times with completions
//       out of issue order.  Checks exactly-once B/R by ID, data/resp,
//       RLAST, no R interleave, m-side fence contract, full drain and
//       fence_stuck == 0 (own hang watchdog)
//   D8  partial-W stop: forwarded BAR0/BAR2 bursts (len 3/7/15), l2_clk_ok
//       drops after k = 0, 1, n-1, random of n W beats: m side sees exactly
//       n beats (k real, rest WSTRB=0), one OKAY B to the host, next writes
//       intact and paired with their own AW, counters drain
//   D9  mid-R stop: m0/m2-owned bursts stopped after k beats reached the
//       host: n beats, k real + all-ones OKAY, one RLAST, no second
//       response, queued reads complete, counters drain
//   all phases: AXI source rules (VALID held, payload stable until READY) on
//   the s-side B/R outputs (+noaxistab demotes them to a count)
//   D11 read-response order: reads with the same ARID (and mixed IDs) to slow
//       BAR0 / fast BAR1 / BAR2 must return in request order, own ID and data
// Plusargs: s2mask, nostop, noaxistab, nod8 (debug), s2n<N>, d11n<N>
// Pass token: NG_BAR_ROUTER_TB_PASS
`timescale 1ns / 1ps

module tb_bar_router;
  localparam int CLKP = 4;   // 250 MHz

  logic clk = 0, rst_n = 0, l2_clk_ok = 1;
  always #(CLKP/2.0) clk = ~clk;

  // ---- DUT pins -------------------------------------------------------------
  logic [3:0]  s_awid;   logic [31:0] s_awaddr; logic [7:0]  s_awlen;
  logic [2:0]  s_awsize; logic [1:0]  s_awburst; logic [3:0] s_awcache;
  logic [2:0]  s_awprot; logic        s_awvalid, s_awready;
  logic [63:0] s_wdata;  logic [7:0]  s_wstrb;   logic       s_wlast, s_wvalid, s_wready;
  logic [3:0]  s_bid;    logic [1:0]  s_bresp;   logic       s_bvalid, s_bready;
  logic [3:0]  s_arid;   logic [31:0] s_araddr;  logic [7:0] s_arlen;
  logic [2:0]  s_arsize; logic [1:0]  s_arburst; logic [3:0] s_arcache;
  logic [2:0]  s_arprot; logic        s_arvalid, s_arready;
  logic [3:0]  s_rid;    logic [63:0] s_rdata;   logic [1:0] s_rresp;
  logic        s_rlast, s_rvalid, s_rready;
  logic        fence_stuck;

  // m ports (arrays make the three slave models one generate block)
  localparam int NP = 3;   // 0=bar0 1=bar1 2=bar2
  logic [3:0]  m_awid  [NP];  logic [31:0] m_awaddr [NP];
  logic [7:0]  m_awlen [NP];  logic [2:0]  m_awsize [NP];
  logic [1:0]  m_awburst[NP]; logic [3:0]  m_awcache[NP];
  logic [2:0]  m_awprot[NP];  logic        m_awvalid[NP], m_awready[NP];
  logic [63:0] m_wdata [NP];  logic [7:0]  m_wstrb [NP];
  logic        m_wlast [NP], m_wvalid [NP], m_wready [NP];
  logic [3:0]  m_bid  [NP];   logic [1:0]  m_bresp [NP];
  logic        m_bvalid[NP], m_bready[NP];
  logic [3:0]  m_arid  [NP];  logic [31:0] m_araddr [NP];
  logic [7:0]  m_arlen [NP];  logic [2:0]  m_arsize [NP];
  logic [1:0]  m_arburst[NP]; logic [3:0]  m_arcache[NP];
  logic [2:0]  m_arprot[NP];  logic        m_arvalid[NP], m_arready[NP];
  logic [3:0]  m_rid   [NP];  logic [63:0] m_rdata  [NP];
  logic [1:0]  m_rresp [NP];  logic        m_rlast [NP], m_rvalid[NP], m_rready[NP];

  ng_bar_router dut (
    .clk_sh(clk), .rst_sh_n(rst_n), .l2_clk_ok(l2_clk_ok),
    .fence_stuck(fence_stuck),
    .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen), .s_awsize(s_awsize),
    .s_awburst(s_awburst), .s_awcache(s_awcache), .s_awprot(s_awprot),
    .s_awvalid(s_awvalid), .s_awready(s_awready),
    .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
    .s_wvalid(s_wvalid), .s_wready(s_wready),
    .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
    .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen), .s_arsize(s_arsize),
    .s_arburst(s_arburst), .s_arcache(s_arcache), .s_arprot(s_arprot),
    .s_arvalid(s_arvalid), .s_arready(s_arready),
    .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp),
    .s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),
    .m0_awid(m_awid[0]), .m0_awaddr(m_awaddr[0][19:0]), .m0_awlen(m_awlen[0]),
    .m0_awsize(m_awsize[0]), .m0_awburst(m_awburst[0]), .m0_awcache(m_awcache[0]),
    .m0_awprot(m_awprot[0]), .m0_awvalid(m_awvalid[0]), .m0_awready(m_awready[0]),
    .m0_wdata(m_wdata[0]), .m0_wstrb(m_wstrb[0]), .m0_wlast(m_wlast[0]),
    .m0_wvalid(m_wvalid[0]), .m0_wready(m_wready[0]),
    .m0_bid(m_bid[0]), .m0_bresp(m_bresp[0]), .m0_bvalid(m_bvalid[0]), .m0_bready(m_bready[0]),
    .m0_arid(m_arid[0]), .m0_araddr(m_araddr[0][19:0]), .m0_arlen(m_arlen[0]),
    .m0_arsize(m_arsize[0]), .m0_arburst(m_arburst[0]), .m0_arcache(m_arcache[0]),
    .m0_arprot(m_arprot[0]), .m0_arvalid(m_arvalid[0]), .m0_arready(m_arready[0]),
    .m0_rid(m_rid[0]), .m0_rdata(m_rdata[0]), .m0_rresp(m_rresp[0]),
    .m0_rlast(m_rlast[0]), .m0_rvalid(m_rvalid[0]), .m0_rready(m_rready[0]),
    .m1_awid(m_awid[1]), .m1_awaddr(m_awaddr[1][15:0]), .m1_awlen(m_awlen[1]),
    .m1_awsize(m_awsize[1]), .m1_awburst(m_awburst[1]), .m1_awcache(m_awcache[1]),
    .m1_awprot(m_awprot[1]), .m1_awvalid(m_awvalid[1]), .m1_awready(m_awready[1]),
    .m1_wdata(m_wdata[1]), .m1_wstrb(m_wstrb[1]), .m1_wlast(m_wlast[1]),
    .m1_wvalid(m_wvalid[1]), .m1_wready(m_wready[1]),
    .m1_bid(m_bid[1]), .m1_bresp(m_bresp[1]), .m1_bvalid(m_bvalid[1]), .m1_bready(m_bready[1]),
    .m1_arid(m_arid[1]), .m1_araddr(m_araddr[1][15:0]), .m1_arlen(m_arlen[1]),
    .m1_arsize(m_arsize[1]), .m1_arburst(m_arburst[1]), .m1_arcache(m_arcache[1]),
    .m1_arprot(m_arprot[1]), .m1_arvalid(m_arvalid[1]), .m1_arready(m_arready[1]),
    .m1_rid(m_rid[1]), .m1_rdata(m_rdata[1]), .m1_rresp(m_rresp[1]),
    .m1_rlast(m_rlast[1]), .m1_rvalid(m_rvalid[1]), .m1_rready(m_rready[1]),
    .m2_awid(m_awid[2]), .m2_awaddr(m_awaddr[2][27:0]), .m2_awlen(m_awlen[2]),
    .m2_awsize(m_awsize[2]), .m2_awburst(m_awburst[2]), .m2_awcache(m_awcache[2]),
    .m2_awprot(m_awprot[2]), .m2_awvalid(m_awvalid[2]), .m2_awready(m_awready[2]),
    .m2_wdata(m_wdata[2]), .m2_wstrb(m_wstrb[2]), .m2_wlast(m_wlast[2]),
    .m2_wvalid(m_wvalid[2]), .m2_wready(m_wready[2]),
    .m2_bid(m_bid[2]), .m2_bresp(m_bresp[2]), .m2_bvalid(m_bvalid[2]), .m2_bready(m_bready[2]),
    .m2_arid(m_arid[2]), .m2_araddr(m_araddr[2][27:0]), .m2_arlen(m_arlen[2]),
    .m2_arsize(m_arsize[2]), .m2_arburst(m_arburst[2]), .m2_arcache(m_arcache[2]),
    .m2_arprot(m_arprot[2]), .m2_arvalid(m_arvalid[2]), .m2_arready(m_arready[2]),
    .m2_rid(m_rid[2]), .m2_rdata(m_rdata[2]), .m2_rresp(m_rresp[2]),
    .m2_rlast(m_rlast[2]), .m2_rvalid(m_rvalid[2]), .m2_rready(m_rready[2])
  );

  int errors = 0;
  task automatic fail(input string what);
    $display("TB_FAIL t=%0t %s", $time, what);
    errors++;
  endtask

  // ---- slave models ----------------------------------------------------------
  // one per port: memory of 64-bit words indexed by addr>>3, delay counter,
  // pause flag.  Reads return {port, addr[15:0], beat}.
  logic [63:0] mem [NP][0:65535];
  int          bdelay [NP] = '{0, 0, 0};
  int          rdelay [NP] = '{0, 0, 0};
  logic        paused [NP] = '{0, 0, 0};
  // read burst bookkeeping per port
  int          rlen [NP]; logic [31:0] raddr [NP]; int rbeat [NP]; int rwait [NP];
  logic        ractive [NP];
  // write beat bookkeeping
  int          wbeats [NP]; logic [31:0] waddr [NP]; int wwait [NP];
  logic        bpend [NP]; logic [31:0] baddr [NP];

  // S2 pipelined slave mode: every port keeps in-order queues of accepted
  // AW / completed writes (B pending) / AR, echoes the real AXI ID, and applies
  // a per-port response latency (m0 slow, m1/m2 fast).
  bit s2_mode = 0;
  longint cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;
  typedef struct { logic [3:0] id; logic [15:0] a16; int len; int beat; longint t;
                   logic [7:0] strb [16]; logic [63:0] data [16]; } s2_txn_t;
  // directed mode (D8/D9): all m-side readies 1, fixed short latencies, no R
  // bubbles; s2_rcap[p] caps the beats presented for the front read burst of
  // port p; every completed m-side write burst is logged in s2_wlog[p]
  bit s2_dir = 0;
  int s2_rcap [NP] = '{9999, 9999, 9999};
  s2_txn_t s2_wlog [NP][$];
  s2_txn_t s2_awq [NP][$];
  s2_txn_t s2_bq  [NP][$];
  s2_txn_t s2_arq [NP][$];
  function automatic int s2_bdly(input int p);
    if (s2_dir) return 4;
    return (p == 0) ? $urandom_range(20, 200) : $urandom_range(0, 3);
  endfunction
  function automatic int s2_rdly(input int p);
    if (s2_dir) return 2;
    return (p == 0) ? $urandom_range(20, 200) : $urandom_range(0, 3);
  endfunction

  for (genvar p = 0; p < NP; p++) begin : g_slv
    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        m_awready[p] <= 0; m_wready[p] <= 0; m_bvalid[p] <= 0; m_arready[p] <= 0;
        m_rvalid[p] <= 0; ractive[p] <= 0; bpend[p] <= 0; wwait[p] <= 0; rwait[p] <= 0;
      end else if (s2_mode) begin : s2_slv
        s2_txn_t t;
        m_awready[p] <= s2_dir || ($urandom_range(0, 3) != 0);
        m_wready[p]  <= s2_dir || ($urandom_range(0, 3) != 0);
        m_arready[p] <= s2_dir || ($urandom_range(0, 3) != 0);
        // AW / W (W beats belong to AWs in order)
        if (m_awvalid[p] && m_awready[p]) begin
          t.id = m_awid[p]; t.a16 = m_awaddr[p][15:0]; t.len = m_awlen[p]; t.beat = 0; t.t = 0;
          s2_awq[p].push_back(t);
        end
        if (m_wvalid[p] && m_wready[p]) begin
          if (s2_awq[p].size() == 0) fail($sformatf("S2 m%0d W beat without AW", p));
          else begin
            if (m_wlast[p] !== (s2_awq[p][0].beat == s2_awq[p][0].len))
              fail($sformatf("S2 m%0d WLAST mismatch (AW id=%0d len=%0d beat=%0d wlast=%0d)", p,
                             s2_awq[p][0].id, s2_awq[p][0].len, s2_awq[p][0].beat, m_wlast[p]));
            if (s2_awq[p][0].beat < 16) begin
              s2_awq[p][0].strb[s2_awq[p][0].beat] = m_wstrb[p];
              s2_awq[p][0].data[s2_awq[p][0].beat] = m_wdata[p];
            end
            if (m_wlast[p]) begin
              t = s2_awq[p].pop_front(); t.t = cyc + s2_bdly(p); s2_bq[p].push_back(t);
              if (s2_dir) s2_wlog[p].push_back(t);
            end else s2_awq[p][0].beat++;
          end
        end
        // B, in order
        if (m_bvalid[p] && m_bready[p]) m_bvalid[p] <= 0;
        if ((!m_bvalid[p] || m_bready[p]) && s2_bq[p].size() != 0 && s2_bq[p][0].t <= cyc) begin
          t = s2_bq[p].pop_front();
          m_bvalid[p] <= 1; m_bid[p] <= t.id; m_bresp[p] <= 2'b00;
        end
        // AR / R, in order, beats may have bubbles
        if (m_arvalid[p] && m_arready[p]) begin
          t.id = m_arid[p]; t.a16 = m_araddr[p][15:0]; t.len = m_arlen[p]; t.beat = 0;
          t.t = cyc + s2_rdly(p);
          s2_arq[p].push_back(t);
        end
        if (m_rvalid[p] && m_rready[p]) begin
          m_rvalid[p] <= 0;
          if (s2_arq[p][0].beat == s2_arq[p][0].len) void'(s2_arq[p].pop_front());
          else s2_arq[p][0].beat++;
        end
        if ((!m_rvalid[p] || m_rready[p]) && s2_arq[p].size() != 0 && s2_arq[p][0].t <= cyc &&
            s2_arq[p][0].beat < s2_rcap[p] && (s2_dir || $urandom_range(0, 7) != 0)) begin
          m_rvalid[p] <= 1; m_rid[p] <= s2_arq[p][0].id; m_rresp[p] <= 2'b00;
          m_rdata[p]  <= {8'hE0 + p, 24'h0, s2_arq[p][0].a16, 16'(s2_arq[p][0].beat)};
          m_rlast[p]  <= (s2_arq[p][0].beat == s2_arq[p][0].len);
        end
      end else begin
        // simple ready patterns with random gaps
        m_awready[p] <= ($urandom_range(0, 3) != 0);
        m_wready[p]  <= ($urandom_range(0, 3) != 0);
        m_arready[p] <= ($urandom_range(0, 3) != 0);

        // write channel
        if (m_awvalid[p] && m_awready[p]) begin
          waddr[p]  <= m_awaddr[p];
          wbeats[p] <= m_awlen[p] + 1;
        end
        if (m_wvalid[p] && m_wready[p]) begin
          if (wbeats[p] > 0) begin
            mem[p][m_wdata[p][63:3] % 65536] <= m_wdata[p];  // simplified: last write wins
            wbeats[p] <= wbeats[p] - 1;
            if (wbeats[p] == 1) begin
              bpend[p] <= 1; baddr[p] <= m_awaddr[p];
              wwait[p] <= bdelay[p];
            end
          end
        end
        if (bpend[p]) begin
          if (wwait[p] > 0) wwait[p] <= wwait[p] - 1;
          else if (!paused[p] && !m_bvalid[p]) begin
            m_bvalid[p] <= 1; m_bresp[p] <= 2'b00; m_bid[p] <= 4'h0 + p;
          end
        end
        if (m_bvalid[p] && m_bready[p]) begin
          m_bvalid[p] <= 0; bpend[p] <= 0;
        end

        // read channel
        if (m_arvalid[p] && m_arready[p]) begin
          raddr[p] <= m_araddr[p]; rlen[p] <= m_arlen[p]; rbeat[p] <= 0;
          ractive[p] <= 1; rwait[p] <= rdelay[p];
        end
        if (ractive[p] && !m_rvalid[p]) begin
          if (rwait[p] > 0) rwait[p] <= rwait[p] - 1;
          else if (!paused[p]) begin
            m_rvalid[p] <= 1;
            m_rdata[p]  <= {8'hE0 + p, 24'h0, raddr[p][15:0], 16'h0 + 16'(rbeat[p])};
            m_rlast[p]  <= (rbeat[p] == rlen[p]);
            m_rresp[p]  <= 2'b00; m_rid[p] <= 4'h0 + p;
          end
        end
        if (m_rvalid[p] && m_rready[p]) begin
          m_rvalid[p] <= 0;
          if (rbeat[p] == rlen[p]) ractive[p] <= 0;
          else begin rbeat[p] <= rbeat[p] + 1; rwait[p] <= rdelay[p]; end
        end
      end
    end
  end

  // ---- master driver tasks ----------------------------------------------------
  task automatic mwr(input int p, input logic [31:0] addr, input logic [63:0] data);
    // single-beat write to window p (p=3 -> unmapped)
    logic [31:0] a;
    a = (p == 0) ? (addr & 32'h000FFFFF)
      : (p == 1) ? (32'h0010_0000 | (addr & 32'h0000FFFF))
      : (p == 2) ? (32'h1000_0000 | (addr & 32'h0FFFFFFF))
      : addr;
    @(posedge clk);
    s_awid <= $urandom_range(0, 15); s_awaddr <= a; s_awlen <= 0; s_awsize <= 3;
    s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    s_wdata <= data; s_wstrb <= 8'hFF; s_wlast <= 1; s_wvalid <= 1;
    do @(posedge clk); while (!s_wready);
    s_wvalid <= 0;
    do @(posedge clk); while (!s_bvalid);
    if (p != 3 && s_bresp != 2'b00) fail($sformatf("write window %0d resp %b", p, s_bresp));
    if (p == 3 && s_bresp != 2'b10) fail("BAD window write not SLVERR");
    do @(posedge clk); while (s_bvalid);
  endtask

  // single-beat read from window p
  task automatic mrd(input int p, input logic [31:0] addr, output logic [63:0] data);
    logic [31:0] a;
    a = (p == 0) ? (addr & 32'h000FFFFF)
      : (p == 1) ? (32'h0010_0000 | (addr & 32'h0000FFFF))
      : (p == 2) ? (32'h1000_0000 | (addr & 32'h0FFFFFFF))
      : addr;
    @(posedge clk);
    s_arid <= $urandom_range(0, 15); s_araddr <= a; s_arlen <= 0; s_arsize <= 3;
    s_arburst <= 2'b01; s_arcache <= 0; s_arprot <= 0; s_arvalid <= 1;
    do @(posedge clk); while (!s_arready);
    s_arvalid <= 0;
    do @(posedge clk); while (!s_rvalid);
    data = s_rdata;
    if (p != 3 && s_rresp != 2'b00) fail($sformatf("read window %0d resp %b", p, s_rresp));
    if (p == 3 && (s_rresp != 2'b10 || s_rdata !== 64'hFFFF_FFFF_FFFF_FFFF))
      fail("BAD window read not SLVERR/all-ones");
    if (!s_rlast) fail("single read missing RLAST");
    @(posedge clk);
    do @(posedge clk); while (s_rvalid);
  endtask

  // burst read (len+1 beats)
  task automatic mrdb(input int p, input logic [31:0] addr, input int len, output logic [63:0] d0);
    logic [31:0] a;
    logic [63:0] d; int beats;
    a = (p == 2) ? (32'h1000_0000 | (addr & 32'h0FFFFFFF)) : addr;
    @(posedge clk);
    s_arid <= $urandom_range(0, 15); s_araddr <= a; s_arlen <= len[7:0]; s_arsize <= 3;
    s_arburst <= 2'b01; s_arcache <= 0; s_arprot <= 0; s_arvalid <= 1;
    do @(posedge clk); while (!s_arready);
    s_arvalid <= 0;
    beats = len + 1;
    while (beats > 0) begin
      do @(posedge clk); while (!s_rvalid);
      d = s_rdata;
      if (beats == len + 1) d0 = d;
      if (s_rlast != (beats == 1)) fail("burst read RLAST mismatch");
      @(posedge clk);
      beats--;
    end
    do @(posedge clk); while (s_rvalid);
  endtask

  // non-blocking write start (for fence checks): returns after AW accepted
  task automatic wr_aw(input int p, input logic [31:0] addr);
    logic [31:0] a;
    a = (p == 0) ? (addr & 32'h000FFFFF)
      : (p == 1) ? (32'h0010_0000 | (addr & 32'h0000FFFF))
      : (p == 2) ? (32'h1000_0000 | (addr & 32'h0FFFFFFF)) : addr;
    @(posedge clk);
    s_awid <= $urandom_range(0, 15); s_awaddr <= a; s_awlen <= 0; s_awsize <= 3;
    s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;   // legal master: deassert at the handshake edge
  endtask

  task automatic wr_w(input logic [63:0] data);
    @(posedge clk);
    s_wdata <= data; s_wstrb <= 8'hFF; s_wlast <= 1; s_wvalid <= 1;
    do @(posedge clk); while (!s_wready);
    s_wvalid <= 0;
  endtask

  // D10 helpers: raw-address single-beat transfer with explicit AxSIZE; the
  // address seen by the m-side slave is captured on the AW/AR handshake.
  logic [31:0] cap_aw [NP], cap_ar [NP];
  localparam logic [31:0] CAPM [NP] = '{32'h000F_FFFF, 32'h0000_FFFF, 32'h0FFF_FFFF};   // m0/m1/m2 address widths
  for (genvar p = 0; p < NP; p++) begin : g_cap
    always @(posedge clk) begin
      if (m_awvalid[p] && m_awready[p]) cap_aw[p] <= m_awaddr[p] & CAPM[p];
      if (m_arvalid[p] && m_arready[p]) cap_ar[p] <= m_araddr[p] & CAPM[p];
    end
  end

  task automatic d10_wr(input int p, input logic [31:0] a, input logic [2:0] sz, input logic [31:0] want);
    @(posedge clk);
    s_awid <= 4'h5; s_awaddr <= a; s_awlen <= 0; s_awsize <= sz;
    s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    s_wdata <= 64'hD10D_D10D_D10D_D10D; s_wstrb <= 8'hFF; s_wlast <= 1; s_wvalid <= 1;
    do @(posedge clk); while (!s_wready);
    s_wvalid <= 0;
    do @(posedge clk); while (!s_bvalid);
    if (s_bresp != 2'b00) fail($sformatf("D10 write %h size %0d resp %b", a, sz, s_bresp));
    do @(posedge clk); while (s_bvalid);
    if (cap_aw[p] !== want)
      fail($sformatf("D10 m%0d AW addr %h size %0d: got %h want %h", p, a, sz, cap_aw[p], want));
  endtask

  task automatic d10_rd(input int p, input logic [31:0] a, input logic [2:0] sz, input logic [31:0] want);
    @(posedge clk);
    s_arid <= 4'h6; s_araddr <= a; s_arlen <= 0; s_arsize <= sz;
    s_arburst <= 2'b01; s_arcache <= 0; s_arprot <= 0; s_arvalid <= 1;
    do @(posedge clk); while (!s_arready);
    s_arvalid <= 0;
    do @(posedge clk); while (!s_rvalid);
    if (s_rresp != 2'b00) fail($sformatf("D10 read %h size %0d resp %b", a, sz, s_rresp));
    @(posedge clk);
    do @(posedge clk); while (s_rvalid);
    if (cap_ar[p] !== want)
      fail($sformatf("D10 m%0d AR addr %h size %0d: got %h want %h", p, a, sz, cap_ar[p], want));
  endtask

  task automatic drain_b();
    do @(posedge clk); while (!s_bvalid);
    @(posedge clk);
    do @(posedge clk); while (s_bvalid);
  endtask

  // ---- S2 pipelined soak: master, scoreboard, monitors ------------------------
  // Writes and reads are issued from two independent processes without
  // waiting for B/R.  Every outstanding transaction per direction owns a
  // distinct AXI ID (16 IDs, ID_W = 4), so each B/R is matched exactly by ID.
  // The router promises no cross-target response order (BAR1 and local
  // answers pass BAR0/BAR2, BAR0 and BAR2 complete independently), so with
  // unique IDs the per-ID ordering check reduces to "exactly one response,
  // R beats of one burst not interleaved".  m-side monitors check the fence
  // contract of the router header (BAR0 write alone; BAR2 write only with no
  // BAR0 write open and <= W_OSTD deep; BAR0/BAR2 reads only with no write
  // open and <= R_OSTD deep per port).
  // Clock drops: l2_clk_ok falls only between writes (never while a write's
  // W stream is open).  A BAR0/BAR2 read may return all-ones OKAY iff it was
  // accepted with the clock down or a drop happened while it was outstanding.
  typedef struct { bit v; int tgt; int len; logic [15:0] a16; int epoch; bit dead; int beat; } s2_exp_t;
  s2_exp_t s2_ew [16];
  s2_exp_t s2_er [16];
  int     s2_epoch = 0;
  bit     s2_dropping = 0, s2_stops = 1;
  // +s2mask steers around two router defects S2 found (see s2_run): no
  // BAR1 writes (m1 B lost when an m0/m2 B wins the s B mux in the same
  // cycle) and single-beat reads with s_rready = 1 (a clock stop while
  // m0/m2 owns an R burst double-answers it and retires a wrong entry)
  bit     s2_mask = 0;
  bit     d11_mode = 0;
  int     s2_nw_iss = 0, s2_nw_done = 0, s2_nr_iss = 0, s2_nr_done = 0;
  int     s2_nr_ones = 0, s2_ndrops = 0;
  int     s2_tgt_w [4] = '{0, 0, 0, 0};
  int     s2_tgt_r [4] = '{0, 0, 0, 0};
  longint s2_last_prog = 0;
  int     s2_wwrap = 0, s2_rwrap = 0;      // cycles with wrapped occupancy
  int     s2_max_wocc = 0, s2_max_rocc = 0;
  int     s2_wpops = 0, s2_rpops = 0;
  int     s2_rcur = -1;                    // ID owning the s R channel
  int     s2_ow0 = 0, s2_ow2 = 0, s2_or0 = 0, s2_or2 = 0;   // m-side open counts
  int     s2_max_ow2 = 0;

  always @(negedge l2_clk_ok) s2_epoch++;

  function automatic logic [31:0] s2_addr(input int tgt, input logic [15:0] a16);
    case (tgt)
      0: return {12'h000, 4'($urandom_range(0, 15)), a16};
      1: return {16'h0010, a16};
      2: return {4'h1, 12'($urandom()), a16};
      default: return ($urandom_range(0, 1)) ? {16'h0011, a16}
                                             : {4'($urandom_range(2, 15)), 12'($urandom()), a16};
    endcase
  endfunction
  function automatic int s2_pick(input int w0, input int w1, input int w2);
    int r; r = $urandom_range(0, 99);
    return (r < w0) ? 0 : (r < w0 + w1) ? 1 : (r < w0 + w1 + w2) ? 2 : 3;
  endfunction

  task automatic s2_writes(input int n);
    bit wbadep = 0;
    for (int k = 0; k < n; k++) begin
      int tgt, id, len; logic [15:0] a16; int tries;
      // idle gaps now and then, so the read side sees write-free windows
      if ($urandom_range(0, 15) == 0) repeat ($urandom_range(20, 300)) @(posedge clk);
      // clock drop only here: no write owns the W stream at this edge
      if (s2_stops && !s2_dropping && $urandom_range(0, 59) == 0) begin
        s2_dropping = 1; s2_ndrops++;
        fork
          begin
            l2_clk_ok <= 0;
            repeat ($urandom_range(1, 40)) @(posedge clk);
            l2_clk_ok <= 1;
            repeat ($urandom_range(5, 200)) @(posedge clk);
            s2_dropping = 0;
          end
        join_none
      end
      while (1) begin
        id = $urandom_range(0, 15);
        for (tries = 0; tries < 16 && s2_ew[id].v; tries++) id = (id + 1) % 16;
        if (!s2_ew[id].v) break;
        @(posedge clk);
      end
      // episodes of 100 writes: normal mix, or unmapped-heavy so locally
      // answered writes pile up in the ring behind a slow BAR0 head
      if (k % 100 == 0) wbadep = ($urandom_range(0, 2) == 0);
      tgt = wbadep ? s2_pick(15, 0, 5) : s2_pick(25, s2_mask ? 0 : 10, 40);
      len = $urandom_range(0, 1);
      a16 = 16'($urandom_range(0, 8191) * 8);
      s2_ew[id] = '{v:1, tgt:tgt, len:len, a16:a16, epoch:s2_epoch, dead:0, beat:0};
      s2_tgt_w[tgt]++; s2_nw_iss++;
      s_awid <= id; s_awaddr <= s2_addr(tgt, a16); s_awlen <= len; s_awsize <= 3;
      s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
      do @(posedge clk); while (!s_awready);
      s_awvalid <= 0;
      for (int b = 0; b <= len; b++) begin
        s_wdata <= {16'h5752, 12'h0, 4'(id), 16'(k), 16'(b)}; s_wstrb <= 8'hFF;
        s_wlast <= (b == len); s_wvalid <= 1;
        do @(posedge clk); while (!s_wready);
      end
      s_wvalid <= 0;
    end
  endtask

  task automatic s2_reads(input int n);
    for (int k = 0; k < n; k++) begin
      int tgt, id, len; logic [15:0] a16; int tries;
      if ($urandom_range(0, 31) == 0) repeat ($urandom_range(10, 100)) @(posedge clk);
      while (1) begin
        id = $urandom_range(0, 15);
        for (tries = 0; tries < 16 && s2_er[id].v; tries++) id = (id + 1) % 16;
        if (!s2_er[id].v) break;
        @(posedge clk);
      end
      tgt = s2_pick(35, 10, 35); len = s2_mask ? 0 : $urandom_range(0, 3);
      a16 = 16'($urandom_range(0, 8191) * 8);
      s2_er[id] = '{v:1, tgt:tgt, len:len, a16:a16, epoch:s2_epoch, dead:0, beat:0};
      s2_tgt_r[tgt]++; s2_nr_iss++;
      s_arid <= id; s_araddr <= s2_addr(tgt, a16); s_arlen <= len; s_arsize <= 3;
      s_arburst <= 2'b01; s_arcache <= 0; s_arprot <= 0; s_arvalid <= 1;
      do @(posedge clk); while (!s_arready);
      // accept edge: clock state and drop epoch as the router saw them
      s2_er[id].dead = !l2_clk_ok; s2_er[id].epoch = s2_epoch;
      s_arvalid <= 0;
    end
  endtask

  // random s-side back-pressure during S2 (R always ready in directed mode)
  always @(posedge clk) if (s2_mode) begin
    s_bready <= ($urandom_range(0, 4) != 0);
    s_rready <= s2_mask || s2_dir || ($urandom_range(0, 4) != 0);
  end

  // s-side R beat log (directed phases)
  typedef struct { logic [3:0] id; logic [63:0] data; logic [1:0] resp; logic last; } s2_rbeat_t;
  s2_rbeat_t s2_rlog [$];
  always @(posedge clk) if (s2_dir && s_rvalid && s_rready)
    s2_rlog.push_back('{id: s_rid, data: s_rdata, resp: s_rresp, last: s_rlast});

  // AXI source rules on the router's s-side B/R outputs (all phases): once
  // VALID is up it stays up, with a stable payload, until READY
  logic       pb_v = 0, pr_v = 0;
  logic [3:0] pb_id, pr_id; logic [1:0] pb_resp, pr_resp; logic [63:0] pr_data; logic pr_last;
  int         n_bstab = 0, n_rstab = 0;
  always @(posedge clk) if (rst_n) begin
    if (pb_v && !(s_bvalid && s_bid === pb_id && s_bresp === pb_resp)) begin
      n_bstab++;
      if ($test$plusargs("noaxistab")) ;
      else if (n_bstab <= 5) fail($sformatf("AXI B stability: pending bid=%0d bresp=%b -> bvalid=%0d bid=%0d bresp=%b l2=%0d",
                                       pb_id, pb_resp, s_bvalid, s_bid, s_bresp, l2_clk_ok));
      else errors++;
    end
    if (pr_v && !(s_rvalid && s_rid === pr_id && s_rresp === pr_resp && s_rdata === pr_data && s_rlast === pr_last)) begin
      n_rstab++;
      if ($test$plusargs("noaxistab")) ;
      else if (n_rstab <= 5) fail($sformatf("AXI R stability: pending rid=%0d d=%h -> rvalid=%0d rid=%0d d=%h l2=%0d",
                                       pr_id, pr_data, s_rvalid, s_rid, s_rdata, l2_clk_ok));
      else errors++;
    end
    pb_v <= s_bvalid && !s_bready; pb_id <= s_bid; pb_resp <= s_bresp;
    pr_v <= s_rvalid && !s_rready; pr_id <= s_rid; pr_resp <= s_rresp; pr_data <= s_rdata; pr_last <= s_rlast;
  end

  // B / R scoreboard
  always @(posedge clk) if (s2_mode && !d11_mode) begin : s2_sb
    s2_exp_t e;
    if (s_bvalid && s_bready) begin
      s2_last_prog = cyc;
      if (!s2_ew[s_bid].v) fail($sformatf("S2 B with no outstanding write, bid=%0d", s_bid));
      else begin
        if ((s2_ew[s_bid].tgt == 3) ? (s_bresp !== 2'b10) : (s_bresp !== 2'b00))
          fail($sformatf("S2 B bid=%0d tgt=%0d bresp=%b", s_bid, s2_ew[s_bid].tgt, s_bresp));
        s2_ew[s_bid].v = 0; s2_nw_done++;
      end
    end
    if (s_rvalid && s_rready) begin
      s2_last_prog = cyc;
      if (s2_rcur >= 0 && s2_rcur != s_rid)
        fail($sformatf("S2 R interleave: rid=%0d inside burst of %0d", s_rid, s2_rcur));
      if (!s2_er[s_rid].v) fail($sformatf("S2 R with no outstanding read, rid=%0d", s_rid));
      else begin
        e = s2_er[s_rid];
        if (e.tgt == 3) begin
          if (s_rresp !== 2'b10 || s_rdata !== '1)
            fail($sformatf("S2 BAD read rid=%0d resp=%b data=%h", s_rid, s_rresp, s_rdata));
        end else if (s_rresp !== 2'b00) begin
          fail($sformatf("S2 read rid=%0d tgt=%0d rresp=%b", s_rid, e.tgt, s_rresp));
        end else if (s_rdata !== {8'hE0 + 8'(e.tgt), 24'h0, e.a16, 16'(e.beat)}) begin
          if (s_rdata === '1 && e.tgt != 1 && (e.dead || e.epoch != s2_epoch)) s2_nr_ones++;
          else fail($sformatf("S2 read rid=%0d tgt=%0d beat=%0d data=%h (dead=%0d ep %0d/%0d)",
                              s_rid, e.tgt, e.beat, s_rdata, e.dead, e.epoch, s2_epoch));
        end
        if (s_rlast !== (e.beat == e.len))
          fail($sformatf("S2 RLAST rid=%0d beat=%0d len=%0d", s_rid, e.beat, e.len));
        if (s_rlast) begin s2_er[s_rid].v = 0; s2_nr_done++; s2_rcur = -1; end
        else begin s2_er[s_rid].beat++; s2_rcur = s_rid; end
      end
    end
  end

  // m-side fence contract + queue wrap statistics
  always @(posedge clk) if (s2_mode) begin : s2_mon
    int wocc, rocc;
    if (m_awvalid[0] && m_awready[0] && (s2_ow0 != 0 || s2_ow2 != 0))
      fail($sformatf("S2 fence: BAR0 AW forwarded with ow0=%0d ow2=%0d", s2_ow0, s2_ow2));
    if (m_awvalid[2] && m_awready[2] && (s2_ow0 != 0 || s2_ow2 >= 32))
      fail($sformatf("S2 fence: BAR2 AW forwarded with ow0=%0d ow2=%0d", s2_ow0, s2_ow2));
    if (m_arvalid[0] && m_arready[0] && (s2_ow0 != 0 || s2_ow2 != 0 || s2_or0 >= 8))
      fail($sformatf("S2 fence: BAR0 AR forwarded ow0=%0d ow2=%0d or0=%0d", s2_ow0, s2_ow2, s2_or0));
    if (m_arvalid[2] && m_arready[2] && (s2_ow0 != 0 || s2_ow2 != 0 || s2_or2 >= 8))
      fail($sformatf("S2 fence: BAR2 AR forwarded ow0=%0d ow2=%0d or2=%0d", s2_ow0, s2_ow2, s2_or2));
    s2_ow0 += int'(m_awvalid[0] && m_awready[0]) - int'(m_bvalid[0] && m_bready[0]);
    s2_ow2 += int'(m_awvalid[2] && m_awready[2]) - int'(m_bvalid[2] && m_bready[2]);
    s2_or0 += int'(m_arvalid[0] && m_arready[0]) - int'(m_rvalid[0] && m_rready[0] && m_rlast[0]);
    s2_or2 += int'(m_arvalid[2] && m_arready[2]) - int'(m_rvalid[2] && m_rready[2] && m_rlast[2]);
    wocc = (dut.wq_wp >= dut.wq_rp) ? dut.wq_wp - dut.wq_rp : dut.wq_wp + 36 - dut.wq_rp;
    rocc = (dut.rq_wp >= dut.rq_rp) ? dut.rq_wp - dut.rq_rp : dut.rq_wp + 18 - dut.rq_rp;
    if (wocc > s2_max_wocc) s2_max_wocc = wocc;
    if (s2_ow2 > s2_max_ow2) s2_max_ow2 = s2_ow2;
    if (rocc > s2_max_rocc) s2_max_rocc = rocc;
    if (dut.wq_wp < dut.wq_rp && dut.wq_wp != 0) s2_wwrap++;
    if (dut.rq_wp < dut.rq_rp && dut.rq_wp != 0) s2_rwrap++;
    if (dut.wq_rp != dut.wq_wp && dut.wq_rsp[dut.wq_rp]) s2_wpops++;
    if (dut.rq_rp != dut.rq_wp && dut.rq_rsp[dut.rq_rp]) s2_rpops++;
  end

  // m-to-s pass-through: every m-side B / R beat the router consumes while
  // not in orphan-swallow mode must appear on the s side in the same cycle
  always @(posedge clk) if (s2_mode) begin : s2_pt
    for (int q = 0; q < NP; q++) begin
      if (m_bvalid[q] && m_bready[q] && (q == 1 || !dut.b_swallow) &&
          !(s_bvalid && s_bready && s_bid === m_bid[q]))
        fail($sformatf("S2 m%0d B id=%0d consumed but not delivered (s_bvalid=%0d s_bid=%0d)",
                       q, m_bid[q], s_bvalid, s_bid));
      if (m_rvalid[q] && m_rready[q] && (q == 1 || !dut.r_swallow) &&
          !(s_rvalid && s_rready && s_rid === m_rid[q] && s_rdata === m_rdata[q]))
        fail($sformatf("S2 m%0d R id=%0d consumed but not delivered", q, m_rid[q]));
    end
  end

  // S2 hang watchdog: no B/R for 20000 cycles while anything is outstanding
  always @(posedge clk) if (s2_mode && (s2_nw_iss != s2_nw_done || s2_nr_iss != s2_nr_done)
                            && cyc - s2_last_prog > 20000) begin
    $display("TB_FAIL t=%0t S2 hang: writes %0d/%0d reads %0d/%0d wq rp=%0d wp=%0d rq rp=%0d wp=%0d",
             $time, s2_nw_done, s2_nw_iss, s2_nr_done, s2_nr_iss,
             dut.wq_rp, dut.wq_wp, dut.rq_rp, dut.rq_wp);
    $display("NG_BAR_ROUTER_TB_FAIL S2 hang");
    $finish;
  end

  task automatic s2_run(input int n);
    longint c0;
    if ($test$plusargs("nostop")) s2_stops = 0;
    if ($test$plusargs("s2mask")) s2_mask = 1;
    $display("S2 start: n=%0d stops=%0d mask=%0d", n, s2_stops, s2_mask);
    @(posedge clk);
    s2_mode = 1; s2_last_prog = cyc; c0 = cyc;
    s2_max_wocc = 0; s2_max_rocc = 0; s2_max_ow2 = 0;
    fork
      s2_writes(n);
      s2_reads(n);
    join
    while (s2_nw_iss != s2_nw_done || s2_nr_iss != s2_nr_done) @(posedge clk);
    wait (!s2_dropping);
    repeat (300) @(posedge clk);
    if (fence_stuck) fail("S2 left fence_stuck");
    if (dut.post_stop) fail("S2 left post_stop");
    if (s2_ow0 || s2_ow2 || s2_or0 || s2_or2)
      fail($sformatf("S2 m side not drained ow0=%0d ow2=%0d or0=%0d or2=%0d", s2_ow0, s2_ow2, s2_or0, s2_or2));
    for (int q = 0; q < NP; q++)
      if (s2_awq[q].size() || s2_bq[q].size() || s2_arq[q].size() || m_bvalid[q] || m_rvalid[q])
        fail($sformatf("S2 m%0d slave queues not empty", q));
    $display("S2 stats: %0d cycles, writes %0d (bar0 %0d bar1 %0d bar2 %0d bad %0d), reads %0d (bar0 %0d bar1 %0d bar2 %0d bad %0d)",
             cyc - c0, s2_nw_done, s2_tgt_w[0], s2_tgt_w[1], s2_tgt_w[2], s2_tgt_w[3],
             s2_nr_done, s2_tgt_r[0], s2_tgt_r[1], s2_tgt_r[2], s2_tgt_r[3]);
    $display("S2 stats: wq pops %0d (max occ %0d, wrapped-occupancy cycles %0d), rq pops %0d (max occ %0d, wrapped %0d), clock drops %0d, all-ones reads %0d",
             s2_wpops, s2_max_wocc, s2_wwrap, s2_rpops, s2_max_rocc, s2_rwrap, s2_ndrops, s2_nr_ones);
    // pipelining: the write ring must hold more than head + W owner, and
    // BAR2 writes must overlap at the m side (only checked on the full mix)
    $display("S2 pipelining: max write-ring occupancy %0d, max BAR2 writes open at m2 %0d",
             s2_max_wocc, s2_max_ow2);
    if (!s2_mask && (s2_max_wocc <= 2 || s2_max_ow2 <= 1))
      fail("S2 writes did not pipeline (ring occupancy <= 2 or <= 1 BAR2 write open)");
    @(posedge clk);
    s2_mode = 0; s_bready <= 1; s_rready <= 1;
  endtask

  // ---- D11 read-response ordering (BOARD BUG: BAR0 read returned BAR1 data) ----
  // The XDMA bridge pairs R beats with outstanding reads by ID and order, so
  // responses to reads with the same ARID must come back in acceptance order
  // even when targets differ (slow BAR0 vs fast BAR1/BAR2 slaves, S2 pipelined
  // models: m0 20..200 cycles, m1/m2 0..3).  Every R beat is checked against
  // the oldest outstanding read of its own RID: tag, address and beat.
  typedef struct { int tgt; int len; logic [15:0] a16; int beat; } d11_exp_t;
  d11_exp_t d11_q [16][$];
  int  d11_iss = 0, d11_done = 0, d11_err = 0;
  int  d11_rcur = -1;
  always @(posedge clk) if (d11_mode && s_rvalid && s_rready) begin : d11_chk
    d11_exp_t e;
    if (d11_rcur >= 0 && d11_rcur != int'(s_rid)) begin
      fail($sformatf("D11 R interleave rid=%0d inside burst of %0d", s_rid, d11_rcur)); d11_err++;
    end
    if (d11_q[s_rid].size() == 0) begin
      fail($sformatf("D11 R with no outstanding read rid=%0d", s_rid)); d11_err++;
    end else begin
      e = d11_q[s_rid][0];
      if (s_rresp !== 2'b00 || s_rdata !== {8'hE0 + 8'(e.tgt), 24'h0, e.a16, 16'(e.beat)}) begin
        if (d11_err < 8)
          fail($sformatf("D11 rid=%0d expects tgt=%0d a16=%h beat=%0d, got data=%h resp=%b",
                         s_rid, e.tgt, e.a16, e.beat, s_rdata, s_rresp));
        d11_err++;
      end
      if (s_rlast !== (e.beat == e.len)) begin fail($sformatf("D11 RLAST rid=%0d", s_rid)); d11_err++; end
      if (e.beat == e.len) begin void'(d11_q[s_rid].pop_front()); d11_done++; d11_rcur = -1; end
      else begin d11_q[s_rid][0].beat++; d11_rcur = s_rid; end
    end
  end

  // mode: 0 same ID, 1 two IDs, 2 unique cycling IDs, 3 random IDs
  // pat: 0 BAR0,BAR1  1 BAR1,BAR0  2 BAR0,BAR1,BAR2,BAR1  3 random (BAR1 heavy)
  task automatic d11_phase(input int n, input int mode, input int pat);
    int tgt, id, len; logic [15:0] a16;
    for (int k = 0; k < n; k++) begin
      case (pat)
        0: tgt = k % 2;
        1: tgt = 1 - (k % 2);
        2: begin case (k % 4) 0: tgt = 0; 1: tgt = 1; 2: tgt = 2; default: tgt = 1; endcase end
        default: tgt = $urandom_range(0, 9) < 4 ? 1 : (($urandom_range(0, 1)) ? 0 : 2);
      endcase
      case (mode)
        0: id = 7;
        1: id = $urandom_range(1, 2);
        2: id = k % 16;
        default: id = $urandom_range(0, 15);
      endcase
      len = (pat == 3) ? $urandom_range(0, 3) : 0;
      a16 = 16'($urandom_range(0, 8191) * 8);
      if ($urandom_range(0, 15) == 0) repeat ($urandom_range(1, 40)) @(posedge clk);
      d11_q[id].push_back('{tgt:tgt, len:len, a16:a16, beat:0});
      d11_iss++;
      s_arid <= id; s_araddr <= s2_addr(tgt, a16); s_arlen <= len; s_arsize <= 3;
      s_arburst <= 2'b01; s_arcache <= 0; s_arprot <= 0; s_arvalid <= 1;
      do @(posedge clk); while (!s_arready);
      s_arvalid <= 0;
    end
  endtask

  task automatic d11_run(input int n);
    int c;
    @(posedge clk);
    d11_mode = 1; s2_mode = 1; s2_dir = 0; s2_last_prog = cyc;
    for (int m = 0; m < 4; m++) for (int pt = 0; pt < 4; pt++) begin
      d11_phase(n, m, pt);
      c = 0;
      while (d11_done != d11_iss && c < 200000) begin @(posedge clk); c++; end
      if (d11_done != d11_iss) begin
        fail($sformatf("D11 hang mode=%0d pat=%0d done %0d/%0d", m, pt, d11_done, d11_iss));
        d11_iss = d11_done;
        for (int q = 0; q < 16; q++) d11_q[q].delete();
      end
    end
    repeat (300) @(posedge clk);
    if (fence_stuck) fail("D11 left fence_stuck");
    s2_mode = 0; d11_mode = 0; s_bready <= 1; s_rready <= 1;
    $display("D11 stats: %0d reads, %0d mismatches", d11_done, d11_err);
  endtask

  // ---- D8 / D9 directed clock-stop cases (pipelined models, directed mode) ---
  // one full forwarded write through the scoreboard, then check its m-side log
  task automatic d8_clean_write(input int p, input logic [3:0] id, input int len);
    int c;
    s2_ew[id] = '{v:1, tgt:p, len:len, a16:16'h0080, epoch:s2_epoch, dead:0, beat:0};
    s2_nw_iss++;
    s_awid <= id; s_awaddr <= s2_addr(p, 16'h0080); s_awlen <= len; s_awsize <= 3;
    s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    for (int b = 0; b <= len; b++) begin
      s_wdata <= {16'hC1EA, 8'(p), 4'h0, id, 16'(len), 16'(b)}; s_wstrb <= 8'hFF;
      s_wlast <= (b == len); s_wvalid <= 1;
      do @(posedge clk); while (!s_wready);
    end
    s_wvalid <= 0;
    c = 0;
    while (s2_ew[id].v && c < 2000) begin @(posedge clk); c++; end
    if (s2_ew[id].v) fail($sformatf("D8 clean write m%0d id=%0d: no B", p, id));
    c = 0;
    while (s2_wlog[p].size() == 0 && c < 100) begin @(posedge clk); c++; end
    if (s2_wlog[p].size() != 1) fail($sformatf("D8 clean write m%0d: %0d m-side bursts logged", p, s2_wlog[p].size()));
    else begin
      s2_txn_t t; t = s2_wlog[p].pop_front();
      if (t.id !== id || t.len != len) fail($sformatf("D8 clean write m%0d: logged id=%0d len=%0d", p, t.id, t.len));
      for (int b = 0; b <= len; b++)
        if (t.strb[b] !== 8'hFF || t.data[b] !== {16'hC1EA, 8'(p), 4'h0, id, 16'(len), 16'(b)})
          fail($sformatf("D8 clean write m%0d beat %0d strb=%h data=%h", p, b, t.strb[b], t.data[b]));
    end
  endtask

  // forwarded write to BAR p (0/2), len+1 beats, clock drops after k beats
  task automatic d8_case(input int p, input int len, input int k);
    logic [3:0] id; int c; int e0;
    e0 = errors;
    id = 4'($urandom_range(0, 15));
    s2_wlog[p].delete();
    s2_ew[id] = '{v:1, tgt:p, len:len, a16:16'h0100, epoch:s2_epoch, dead:0, beat:0};
    s2_nw_iss++;
    @(posedge clk);
    s_awid <= id; s_awaddr <= s2_addr(p, 16'h0100); s_awlen <= len; s_awsize <= 3;
    s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    for (int b = 0; b < k; b++) begin
      s_wdata <= {16'hD8D8, 8'(p), 8'(k), 16'(len), 16'(b)}; s_wstrb <= 8'hFF;
      s_wlast <= (b == len); s_wvalid <= 1;
      do @(posedge clk); while (!s_wready);
    end
    s_wvalid <= 0;
    l2_clk_ok <= 0;                   // k beats reached the m side, the rest has not
    repeat (4) @(posedge clk);
    for (int b = k; b <= len; b++) begin   // host completes its burst (absorbed)
      s_wdata <= {16'hDEAD, 8'(p), 8'(k), 16'(len), 16'(b)}; s_wstrb <= 8'hFF;
      s_wlast <= (b == len); s_wvalid <= 1;
      do @(posedge clk); while (!s_wready);
    end
    s_wvalid <= 0;
    repeat (10) @(posedge clk);
    l2_clk_ok <= 1;
    c = 0;
    while ((s2_ew[id].v || dut.post_stop) && c < 3000) begin @(posedge clk); c++; end
    if (s2_ew[id].v) fail($sformatf("D8 m%0d len=%0d k=%0d: host got no B", p, len, k));
    if (dut.post_stop) fail($sformatf("D8 m%0d len=%0d k=%0d: post_stop stuck", p, len, k));
    // the stopped write's m-side burst: exactly len+1 beats, k real, rest WSTRB=0
    if (s2_wlog[p].size() != 1) fail($sformatf("D8 m%0d len=%0d k=%0d: %0d m-side bursts completed (want 1)",
                                               p, len, k, s2_wlog[p].size()));
    else begin
      s2_txn_t t; t = s2_wlog[p].pop_front();
      if (t.id !== id || t.len != len) fail($sformatf("D8 m%0d k=%0d: logged id=%0d len=%0d", p, k, t.id, t.len));
      for (int b = 0; b <= len; b++) begin
        if (b < k && (t.strb[b] !== 8'hFF || t.data[b] !== {16'hD8D8, 8'(p), 8'(k), 16'(len), 16'(b)}))
          fail($sformatf("D8 m%0d len=%0d k=%0d beat %0d: real beat strb=%h data=%h", p, len, k, b, t.strb[b], t.data[b]));
        if (b >= k && t.strb[b] !== 8'h00)
          fail($sformatf("D8 m%0d len=%0d k=%0d beat %0d: pad beat strb=%h", p, len, k, b, t.strb[b]));
      end
    end
    // the next forwarded writes (same port, then the other) arrive intact
    d8_clean_write(p, id ^ 4'h5, $urandom_range(1, 7));
    d8_clean_write(2 - p, id ^ 4'hA, $urandom_range(1, 7));
    repeat (20) @(posedge clk);
    if (fence_stuck || dut.w0_cnt || dut.w2_cnt || s2_ow0 || s2_ow2)
      fail($sformatf("D8 m%0d len=%0d k=%0d: not drained (fence_stuck=%0d w0=%0d w2=%0d)",
                     p, len, k, fence_stuck, dut.w0_cnt, dut.w2_cnt));
    if (errors != e0) $display("D8 case m%0d len=%0d k=%0d FAILED", p, len, k);
  endtask

  // read to BAR p (0/2), len+1 beats, clock drops after k beats reached s;
  // a second read on p, one on the other BAR and one unmapped read queue up
  task automatic d9_case(input int p, input int len, input int k);
    logic [3:0] id; int c, e0, nb; logic [3:0] ids [4]; int tg [4]; int ln [4];
    e0 = errors;
    id = 4'($urandom_range(0, 15));
    ids = '{id, id ^ 4'h3, id ^ 4'h6, id ^ 4'h9};
    tg  = '{p, p, 2 - p, 3};
    ln  = '{len, 1, 2, 1};
    s2_rlog.delete();
    s2_rcap[p] = k;                   // m side presents only k beats of the burst
    for (int j = 0; j < 4; j++) begin
      s2_er[ids[j]] = '{v:1, tgt:tg[j], len:ln[j], a16:16'(16'h0200 + 8 * j), epoch:s2_epoch, dead:0, beat:0};
      s2_nr_iss++;
      @(posedge clk);
      s_arid <= ids[j]; s_araddr <= s2_addr(tg[j], 16'(16'h0200 + 8 * j)); s_arlen <= ln[j];
      s_arsize <= 3; s_arburst <= 2'b01; s_arcache <= 0; s_arprot <= 0; s_arvalid <= 1;
      do @(posedge clk); while (!s_arready);
      s2_er[ids[j]].epoch = s2_epoch; s2_er[ids[j]].dead = !l2_clk_ok;
      s_arvalid <= 0;
    end
    // wait until k beats of the burst reached the host and the m side stalls
    c = 0;
    do begin
      @(posedge clk); c++;
      nb = 0; foreach (s2_rlog[j]) if (s2_rlog[j].id == id) nb++;
    end while (nb < k && c < 2000);
    repeat (5) @(posedge clk);
    if (!(dut.r_owner_v && dut.r_owner == (p == 0 ? 2'd0 : 2'd1)))
      fail($sformatf("D9 m%0d len=%0d k=%0d: burst not owned by m%0d at the stop", p, len, k, p));
    l2_clk_ok <= 0;
    repeat (10) @(posedge clk);
    s2_rcap[p] = 9999;                // late m-side beats now flow (swallowed)
    repeat (20) @(posedge clk);
    l2_clk_ok <= 1;
    c = 0;
    while ((s2_er[ids[0]].v || s2_er[ids[1]].v || s2_er[ids[2]].v || s2_er[ids[3]].v || dut.post_stop)
           && c < 3000) begin @(posedge clk); c++; end
    repeat (50) @(posedge clk);       // a duplicate response would show up here
    for (int j = 0; j < 4; j++) if (s2_er[ids[j]].v) fail($sformatf("D9 m%0d k=%0d: read %0d incomplete", p, k, j));
    if (dut.post_stop) fail($sformatf("D9 m%0d len=%0d k=%0d: post_stop stuck", p, len, k));
    // strict per-beat check of the stopped burst
    nb = 0;
    foreach (s2_rlog[j]) if (s2_rlog[j].id == id) begin
      logic [63:0] exp;
      exp = (nb < k) ? {8'hE0 + 8'(p), 24'h0, 16'h0200, 16'(nb)} : '1;
      if (s2_rlog[j].data !== exp || s2_rlog[j].resp !== 2'b00 || s2_rlog[j].last !== (nb == len))
        fail($sformatf("D9 m%0d len=%0d k=%0d beat %0d: data=%h resp=%b last=%0d (want %h)",
                       p, len, k, nb, s2_rlog[j].data, s2_rlog[j].resp, s2_rlog[j].last, exp));
      nb++;
    end
    if (nb != len + 1) fail($sformatf("D9 m%0d len=%0d k=%0d: host got %0d beats (want %0d)", p, len, k, nb, len + 1));
    if (fence_stuck || dut.r0_cnt || dut.r2_cnt || s2_or0 || s2_or2 || s2_arq[0].size() || s2_arq[2].size())
      fail($sformatf("D9 m%0d len=%0d k=%0d: not drained (fence_stuck=%0d r0=%0d r2=%0d)",
                     p, len, k, fence_stuck, dut.r0_cnt, dut.r2_cnt));
    // a fresh read afterwards returns real data
    s2_rlog.delete();
    s2_er[id] = '{v:1, tgt:p, len:3, a16:16'h0300, epoch:s2_epoch, dead:0, beat:0};
    s2_nr_iss++;
    @(posedge clk);
    s_arid <= id; s_araddr <= s2_addr(p, 16'h0300); s_arlen <= 3; s_arvalid <= 1;
    do @(posedge clk); while (!s_arready);
    s2_er[id].epoch = s2_epoch;
    s_arvalid <= 0;
    c = 0; while (s2_er[id].v && c < 500) begin @(posedge clk); c++; end
    nb = 0;
    foreach (s2_rlog[j]) if (s2_rlog[j].id == id) begin
      if (s2_rlog[j].data !== {8'hE0 + 8'(p), 24'h0, 16'h0300, 16'(nb)})
        fail($sformatf("D9 m%0d k=%0d: follow-up read beat %0d data=%h", p, k, nb, s2_rlog[j].data));
      nb++;
    end
    if (nb != 4) fail($sformatf("D9 m%0d k=%0d: follow-up read got %0d beats", p, k, nb));
    if (errors != e0) $display("D9 case m%0d len=%0d k=%0d FAILED", p, len, k);
  endtask

  // ---- main -------------------------------------------------------------------
  logic [63:0] d;
  int i, p, e10;
  int n_soak = 2000;
  int n_s2;
  int d8_p [2] = '{0, 2};
  int d8_len [3] = '{3, 7, 15};

  initial begin
    s_awvalid = 0; s_wvalid = 0; s_arvalid = 0; s_bready = 1; s_rready = 1;
    s_awid = 0; s_awaddr = 0; s_awlen = 0; s_awsize = 0; s_awburst = 0; s_awcache = 0; s_awprot = 0;
    s_wdata = 0; s_wstrb = 0; s_wlast = 0;
    s_arid = 0; s_araddr = 0; s_arlen = 0; s_arsize = 0; s_arburst = 0; s_arcache = 0; s_arprot = 0;
    repeat (5) @(posedge clk);
    rst_n <= 1;
    repeat (10) @(posedge clk);

    // D1 basic (slave reads return the synthetic pattern {port,addr,beat})
    mwr(0, 32'h0000_1000, 64'h1111_0000_0000_1000);
    mwr(1, 32'h0000_0004, 64'h2222_0000_0000_0004);
    mwr(2, 32'h0001_2340, 64'h3333_0000_0001_2340);
    mrd(0, 32'h0000_1000, d); if (d !== {8'hE0, 24'h0, 16'h1000, 16'h0}) fail("D1 bar0 data");
    mrd(1, 32'h0000_0004, d); if (d !== {8'hE1, 24'h0, 16'h0004, 16'h0}) fail("D1 bar1 data");
    mrd(2, 32'h0001_2340, d); if (d !== {8'hE2, 24'h0, 16'h2340, 16'h0}) fail("D1 bar2 data");
    if (errors == 0) $display("D1_PASS");

    // D10 BAR2 AxADDR alignment (bridge-window base 0x1000_0000; m2 address is window-relative)
    e10 = errors;
    for (int o = 0; o < 8; o++) begin
      d10_wr(2, 32'h1000_4100 + o, 3'd3, 32'h0000_4100);   // full width: cleared to 8B
      d10_rd(2, 32'h1000_5200 + o, 3'd3, 32'h0000_5200);
    end
    d10_wr(2, 32'h1000_4104, 3'd2, 32'h0000_4104);         // narrow: untouched
    d10_rd(2, 32'h1000_5206, 3'd1, 32'h0000_5206);
    d10_wr(2, 32'h1000_4107, 3'd0, 32'h0000_4107);
    d10_wr(0, 32'h0000_1004, 3'd3, 32'h0000_1004);         // BAR0/BAR1 never touched
    d10_rd(0, 32'h0000_1004, 3'd3, 32'h0000_1004);
    d10_wr(1, 32'h0010_0004, 3'd3, 32'h0000_0004);
    d10_rd(1, 32'h0010_0004, 3'd3, 32'h0000_0004);
    if (errors == e10) $display("D10_PASS");

    // D2 fence: bar2 write with B held, then offer a bar0 AW and verify it
    // is not accepted until the bar2 B flows (sequential, single driver).
    paused[2] <= 1;
    wr_aw(2, 32'h0000_0100);
    wr_w(64'hA000);                   // AW+W delivered, B held by paused[2]
    // offer the bar0 write
    s_awid <= 4'h3; s_awaddr <= 32'h0000_0200; s_awlen <= 0; s_awsize <= 3;
    s_awburst <= 2'b01; s_awcache <= 0; s_awprot <= 0; s_awvalid <= 1;
    repeat (8) @(posedge clk);
    if (s_awready) fail("D2 bar0 AW accepted while bar2 B pending");
    else $display("D2_FENCE_OK");
    paused[2] <= 0;                    // bar2 B flows; fence must open
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    @(posedge clk);
    s_wdata <= 64'hB000; s_wstrb <= 8'hFF; s_wlast <= 1; s_wvalid <= 1;
    do @(posedge clk); while (!s_wready);
    s_wvalid <= 0;
    drain_b();                        // bar2 B then bar0 B
    if (errors == 0) $display("D2_PASS");

    // D3 BAR1 during bar2 pending
    paused[2] <= 1;
    fork
      begin
        wr_aw(2, 32'h0000_0300);
        wr_w(64'hC000);
        drain_b();
      end
      begin
        repeat (4) @(posedge clk);
        mrd(1, 32'h0000_0010, d);   // must complete while bar2 B held
        paused[2] <= 0;
      end
    join
    if (errors == 0) $display("D3_PASS");

    // D4 unmapped
    mwr(3, 32'h0011_0000, 64'hD);   // gap between BAR1 end and BAR2 base
    mrd(3, 32'h0011_0000, d);
    if (errors == 0) $display("D4_PASS");

    // D5 stop while idle
    l2_clk_ok <= 0;
    repeat (5) @(posedge clk);
    mrd(2, 32'h0000_0000, d);
    if (d !== 64'hFFFF_FFFF_FFFF_FFFF) fail("D5 stop read not all-ones");
    mwr(2, 32'h0000_0000, 64'hE);   // dropped, OKAY
    mrd(0, 32'h0000_0000, d);
    if (d !== 64'hFFFF_FFFF_FFFF_FFFF) fail("D5 stop bar0 read not all-ones");
    l2_clk_ok <= 1;
    repeat (20) @(posedge clk);     // post_stop clears immediately (no orphans)
    if (fence_stuck) fail("D5 fence_stuck with no orphans");
    mrd(2, 32'h0000_0000, d);
    if (errors == 0) $display("D5_PASS");

    // D6 stop with in-flight forwarded read (bar2 responder paused)
    paused[2] <= 1; rdelay[2] = 0;
    fork
      begin : d6_read
        mrd(2, 32'h0000_1234, d);
      end
      begin
        wait (m_arvalid[2] === 1'b1);
        @(posedge clk);
        l2_clk_ok <= 0;               // clock dies after AR was forwarded
        // m2 responder is paused -> no beats arrive while dead
        repeat (20) @(posedge clk);
        paused[2] <= 0;               // late beats now flow and are swallowed
        repeat (20) @(posedge clk);
        l2_clk_ok <= 1;
        repeat (30) @(posedge clk);
        if (fence_stuck) fail("D6 fence_stuck not cleared after drain");
      end
    join
    mrd(2, 32'h0000_1234, d);
    if (errors == 0) $display("D6_PASS");

    // D7 stop with in-flight forwarded write (B held off)
    paused[2] <= 1;
    fork
      begin
        wr_aw(2, 32'h0000_0500);
        wr_w(64'hF000);
        drain_b();                    // answered locally OKAY while dead
      end
      begin
        wait (m_awvalid[2] === 1'b1 || m_wvalid[2] === 1'b1);
        @(posedge clk);
        l2_clk_ok <= 0;
        repeat (15) @(posedge clk);
        paused[2] <= 0;               // late B arrives, swallowed
        repeat (15) @(posedge clk);
        l2_clk_ok <= 1;
        repeat (30) @(posedge clk);
      end
    join
    if (errors == 0) $display("D7_PASS");

    // S1 soak
    for (i = 0; i < n_soak; i++) begin
      p = $urandom_range(0, 4);      // 0..2 windows, 3 = unmapped, 4 = stop toggle
      bdelay[0] = $urandom_range(0, 3);
      bdelay[1] = $urandom_range(0, 3);
      bdelay[2] = $urandom_range(0, 3);
      rdelay[0] = $urandom_range(0, 3);
      rdelay[2] = $urandom_range(0, 3);
      if (p == 4 && !($test$plusargs("nostop"))) begin
        if (l2_clk_ok) begin
          l2_clk_ok <= 0;
          repeat ($urandom_range(1, 6)) @(posedge clk);
          l2_clk_ok <= 1;
        end
      end else if ($urandom_range(0, 1)) begin
        // p==3 targets the unmapped gap above BAR1 (0x0011_0000..0x0011_FFFF)
        mwr(p, (p == 3) ? (32'h0011_0000 | $urandom_range(0, 32'h0000FFC0))
                        : $urandom_range(0, 32'h0007FFC0), $urandom());
      end else begin
        mrd(p, (p == 3) ? (32'h0011_0000 | $urandom_range(0, 32'h0000FFC0))
                        : $urandom_range(0, 32'h0007FFC0), d);
        if (p <= 2 && l2_clk_ok) begin
          if (d[63:56] !== 8'hE0 + p) fail($sformatf("soak read port tag %02x", d[63:56]));
        end
      end
    end
    // let everything settle and confirm the fence clears
    repeat (200) @(posedge clk);
    if (fence_stuck) fail("soak left fence_stuck");
    mrd(2, 32'h0000_0000, d);
    if (errors == 0) $display("SOAK_PASS");

    // D8 / D9: directed partial-W stop and mid-R-burst stop
    @(posedge clk);
    s2_mode = 1; s2_dir = 1; s2_last_prog = cyc;
    repeat (5) @(posedge clk);
    begin
      int e0; e0 = errors;
      if (!$test$plusargs("nod8")) foreach (d8_p[a]) foreach (d8_len[b]) begin
        d8_case(d8_p[a], d8_len[b], 0);
        d8_case(d8_p[a], d8_len[b], 1);
        d8_case(d8_p[a], d8_len[b], d8_len[b]);          // n-1 of n beats
        d8_case(d8_p[a], d8_len[b], $urandom_range(2, d8_len[b] - 1));
      end
      if ($test$plusargs("nod8")) begin $display("D8_SKIPPED"); errors++; end
      else if (errors == e0) $display("D8_PASS");
      e0 = errors;
      foreach (d8_p[a]) foreach (d8_len[b]) begin
        d9_case(d8_p[a], d8_len[b], 1);
        d9_case(d8_p[a], d8_len[b], d8_len[b]);          // all but the last beat
        d9_case(d8_p[a], d8_len[b], $urandom_range(2, d8_len[b] - 1));
      end
      if (errors == e0) $display("D9_PASS");
    end
    s2_dir = 0; s2_mode = 0; s_bready <= 1; s_rready <= 1;
    repeat (20) @(posedge clk);

    // S2 pipelined soak (queues wrap, out-of-order completion, clock drops)
    if (!$value$plusargs("s2n%d", n_s2)) n_s2 = 4000;
    s2_run(n_s2);
    if (errors == 0) $display("S2_PASS");

    // D11 read-response ordering by ID (slow BAR0 vs fast BAR1/BAR2)
    begin
      int e0; e0 = errors;
      d11_run($value$plusargs("d11n%d", n_s2) ? n_s2 : 150);
      if (errors == e0) $display("D11_PASS");
    end

    $display("AXI stability violations on s side: B %0d, R %0d%s", n_bstab, n_rstab,
             $test$plusargs("noaxistab") ? " (demoted by +noaxistab)" : "");
    if (errors == 0) $display("NG_BAR_ROUTER_TB_PASS");
    else $display("NG_BAR_ROUTER_TB_FAIL errors=%0d", errors);
    $finish;
  end

  // watchdog
  initial begin
    #40_000_000;
    $display("NG_BAR_ROUTER_TB_FAIL watchdog");
    $finish;
  end
endmodule
