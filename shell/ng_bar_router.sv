// ng_bar_router.sv -- XDMA m_axib (single AXI4 64-bit master, 32-bit bridge
// address space) to the three shell BAR paths (P0-11).
//
// The XDMA AXI Bridge translates each enabled PCIe BAR into a window of the
// m_axib address space through its pciebar2axibar_n entries (the build sets
// BAR0 -> 0x0000_0000/1 MiB, BAR1 -> 0x0010_0000/64 KiB, BAR2 ->
// 0x1000_0000/256 MiB).  This router decodes those windows and enforces the
// PCIe ordering rules of DFX_BOUNDARY.md section 1.5 plus the clock-stop
// behavior of sections 2.4 and 5.3:
//
//   * posted-write ordering across BAR0/BAR2: a BAR0 write waits until every
//     earlier BAR2 write has its B; a BAR2 write waits until every earlier
//     BAR0 write has its B.  BAR2-to-BAR2 writes may pipeline W_OSTD deep,
//     because the whole downstream path (clock converter, then the boundary
//     iso block that re-serializes by arrival order) preserves write order.
//   * reads on BAR0/BAR2 wait until all earlier writes have their B (read
//     completions must not pass posted writes).
//   * BAR1 (shell registers, pure clk_sh logic) is exempt from the write
//     fence (DFX_BOUNDARY.md 1.5) and is forwarded at once.
//   * read responses return strictly in the order the ARs were accepted (all
//     three windows plus local answers share one ring, the oldest entry owns
//     the s R channel).  AXI only demands order per ARID, but the XDMA bridge
//     pairs R data with its outstanding reads by ID/order and the targets
//     differ in latency: a fast BAR1 response that overtook an older slow
//     BAR0 read with the same ARID was delivered as that read's data (board
//     bug BAR0 read returning BAR1 data).  A BAR1 response
//     nothing older pending still passes with the same latency as before.
//   * while l2_clk_ok = 0 (MMCM retune/unlock): BAR0/BAR2 requests are
//     answered locally -- reads return all-ones OKAY, writes are absorbed
//     and acknowledged OKAY ("read all ones, drop writes, no error", the
//     unified isolation semantics of section 5.3).  Transactions already
//     forwarded to the m side are completed locally and their late
//     responses are swallowed (orphan handling).  After l2_clk_ok returns,
//     forwarding stays blocked (requests back-pressure, not dropped) until
//     every orphan response has been swallowed; fence_stuck reports that.
//   * unmapped bridge addresses answer SLVERR.
//
// AW/AR/W are passed to the decoded m port combinationally on the accept
// cycle (the m-side ready feeds the s-side ready chain, one gate of logic),
// so the fence decision is exact: nothing is buffered between the decision
// and the forwarded request.  Consequence: m-side AWVALID/ARVALID/WVALID
// depend on the m-side READY, so every m port must face a slave whose
// READY does not wait for VALID (true for ng_axi_cc and ng_axi64_to_lite32,
// the only slaves in the shell).  Because the bridge interleaves nothing on the
// W channel, the oldest outstanding write owns the W stream: while it is
// open, no new AW is accepted (this is what makes per-write local
// completion and the BAR1 pass-through unambiguous).
module ng_bar_router #(
  parameter int ID_W = 4,
  parameter int A0_W = 20,                       // BAR0 1 MiB
  parameter int A1_W = 16,                       // BAR1 64 KiB
  parameter int A2_W = 28,                       // BAR2 256 MiB
  parameter logic [31:0] BAR1_BASE = 32'h0010_0000,
  parameter logic [31:0] BAR2_BASE = 32'h1000_0000,
  parameter int W_OSTD = 32,                     // BAR2 write pipelining
  parameter int R_OSTD = 8                       // per-direction read depth
) (
  input  logic clk_sh,
  input  logic rst_sh_n,
  input  logic l2_clk_ok,
  // -------- subordinate side: XDMA m_axib (AXI4, 64-bit, 32-bit address) ----
  input  logic [ID_W-1:0] s_awid,
  input  logic [31:0]      s_awaddr,
  input  logic [7:0]       s_awlen,
  input  logic [2:0]       s_awsize,
  input  logic [1:0]       s_awburst,
  input  logic [3:0]       s_awcache,
  input  logic [2:0]       s_awprot,
  input  logic             s_awvalid,
  output logic             s_awready,
  input  logic [63:0]      s_wdata,
  input  logic [7:0]       s_wstrb,
  input  logic             s_wlast,
  input  logic             s_wvalid,
  output logic             s_wready,
  output logic [ID_W-1:0] s_bid,
  output logic [1:0]       s_bresp,
  output logic             s_bvalid,
  input  logic             s_bready,
  input  logic [ID_W-1:0] s_arid,
  input  logic [31:0]      s_araddr,
  input  logic [7:0]       s_arlen,
  input  logic [2:0]       s_arsize,
  input  logic [1:0]       s_arburst,
  input  logic [3:0]       s_arcache,
  input  logic [2:0]       s_arprot,
  input  logic             s_arvalid,
  output logic             s_arready,
  output logic [ID_W-1:0] s_rid,
  output logic [63:0]      s_rdata,
  output logic [1:0]       s_rresp,
  output logic             s_rlast,
  output logic             s_rvalid,
  input  logic             s_rready,
  // -------- manager side 0: BAR0 window -> RM control path ------------------
  output logic [ID_W-1:0] m0_awid,
  output logic [A0_W-1:0] m0_awaddr,
  output logic [7:0]      m0_awlen,
  output logic [2:0]      m0_awsize,
  output logic [1:0]      m0_awburst,
  output logic [3:0]      m0_awcache,
  output logic [2:0]      m0_awprot,
  output logic            m0_awvalid,
  input  logic            m0_awready,
  output logic [63:0]     m0_wdata,
  output logic [7:0]      m0_wstrb,
  output logic            m0_wlast,
  output logic            m0_wvalid,
  input  logic            m0_wready,
  input  logic [ID_W-1:0] m0_bid,
  input  logic [1:0]      m0_bresp,
  input  logic            m0_bvalid,
  output logic            m0_bready,
  output logic [ID_W-1:0] m0_arid,
  output logic [A0_W-1:0] m0_araddr,
  output logic [7:0]      m0_arlen,
  output logic [2:0]      m0_arsize,
  output logic [1:0]      m0_arburst,
  output logic [3:0]      m0_arcache,
  output logic [2:0]      m0_arprot,
  output logic            m0_arvalid,
  input  logic            m0_arready,
  input  logic [ID_W-1:0] m0_rid,
  input  logic [63:0]     m0_rdata,
  input  logic [1:0]      m0_rresp,
  input  logic            m0_rlast,
  input  logic            m0_rvalid,
  output logic            m0_rready,
  // -------- manager side 1: BAR1 window -> shell registers (pass-through) ---
  output logic [ID_W-1:0] m1_awid,
  output logic [A1_W-1:0] m1_awaddr,
  output logic [7:0]      m1_awlen,
  output logic [2:0]      m1_awsize,
  output logic [1:0]      m1_awburst,
  output logic [3:0]      m1_awcache,
  output logic [2:0]      m1_awprot,
  output logic            m1_awvalid,
  input  logic            m1_awready,
  output logic [63:0]     m1_wdata,
  output logic [7:0]      m1_wstrb,
  output logic            m1_wlast,
  output logic            m1_wvalid,
  input  logic            m1_wready,
  input  logic [ID_W-1:0] m1_bid,
  input  logic [1:0]       m1_bresp,
  input  logic            m1_bvalid,
  output logic            m1_bready,
  output logic [ID_W-1:0] m1_arid,
  output logic [A1_W-1:0] m1_araddr,
  output logic [7:0]      m1_arlen,
  output logic [2:0]      m1_arsize,
  output logic [1:0]      m1_arburst,
  output logic [3:0]      m1_arcache,
  output logic [2:0]      m1_arprot,
  output logic            m1_arvalid,
  input  logic            m1_arready,
  input  logic [ID_W-1:0] m1_rid,
  input  logic [63:0]     m1_rdata,
  input  logic [1:0]      m1_rresp,
  input  logic            m1_rlast,
  input  logic            m1_rvalid,
  output logic            m1_rready,
  // -------- manager side 2: BAR2 window -> RM memory window -----------------
  output logic [ID_W-1:0] m2_awid,
  output logic [A2_W-1:0] m2_awaddr,
  output logic [7:0]      m2_awlen,
  output logic [2:0]      m2_awsize,
  output logic [1:0]      m2_awburst,
  output logic [3:0]      m2_awcache,
  output logic [2:0]      m2_awprot,
  output logic            m2_awvalid,
  input  logic            m2_awready,
  output logic [63:0]     m2_wdata,
  output logic [7:0]      m2_wstrb,
  output logic            m2_wlast,
  output logic            m2_wvalid,
  input  logic            m2_wready,
  input  logic [ID_W-1:0] m2_bid,
  input  logic [1:0]      m2_bresp,
  input  logic            m2_bvalid,
  output logic            m2_bready,
  output logic [ID_W-1:0] m2_arid,
  output logic [A2_W-1:0] m2_araddr,
  output logic [7:0]      m2_arlen,
  output logic [2:0]      m2_arsize,
  output logic [1:0]      m2_arburst,
  output logic [3:0]      m2_arcache,
  output logic [2:0]      m2_arprot,
  output logic            m2_arvalid,
  input  logic            m2_arready,
  input  logic [ID_W-1:0] m2_rid,
  input  logic [63:0]     m2_rdata,
  input  logic [1:0]      m2_rresp,
  input  logic            m2_rlast,
  input  logic            m2_rvalid,
  output logic            m2_rready,
  // -------- status ----------------------------------------------------------
  output logic fence_stuck   // forwarded transactions were answered locally
);

  // ------------------------------------------------------------- constants ---
  typedef enum logic [1:0] { T_BAR0 = 2'd0, T_BAR2 = 2'd1, T_BAD = 2'd2, T_BAR1 = 2'd3 } tgt_t;
  localparam int WQ = W_OSTD + 4;
  localparam int RQ = 2 * R_OSTD + 2;

  function automatic tgt_t decode(input logic [31:0] a);
    if (a >= BAR2_BASE)      decode = (a - BAR2_BASE < (32'd1 << A2_W)) ? T_BAR2 : T_BAD;
    else if (a >= BAR1_BASE) decode = (a - BAR1_BASE < (32'd1 << A1_W)) ? T_BAR1 : T_BAD;
    else                     decode = (a < (32'd1 << A0_W)) ? T_BAR0 : T_BAD;
  endfunction

  function automatic logic [6:0] qnext(input logic [6:0] p, input int DEPTH);
    qnext = (p == DEPTH - 1) ? 7'd0 : p + 7'd1;
  endfunction

  // ring index p+i modulo DEPTH (p < DEPTH, i < DEPTH; the depths are not
  // powers of two, so a plain 7-bit add runs past the array end)
  function automatic logic [6:0] qadd(input logic [6:0] p, input int i, input int DEPTH);
    logic [7:0] s;
    s = {1'b0, p} + 8'(i);
    qadd = (s >= 8'(DEPTH)) ? 7'(s - 8'(DEPTH)) : s[6:0];
  endfunction

  // -------------------------------------------------- queues (BAR0/BAR2/BAD) -
  // one write entry per accepted AW (BAR1 writes are not queued).  wq_rsp is
  // set when the response was delivered (m side) or generated (local); the
  // head pops once responded.
  logic [1:0]      wq_tgt [0:WQ-1];
  logic [ID_W-1:0] wq_id  [0:WQ-1];
  logic            wq_fwd [0:WQ-1];
  logic            wq_wdn [0:WQ-1];
  logic            wq_rsp [0:WQ-1];
  logic [6:0]      wq_wp, wq_rp;
  logic [6:0]      wq_n;          // occupancy 0..WQ (pointers alone cannot tell full from empty)
  logic [6:0]      whead;

  logic [1:0]      rq_tgt [0:RQ-1];
  logic [ID_W-1:0] rq_id  [0:RQ-1];
  logic [7:0]      rq_len [0:RQ-1];
  logic            rq_fwd [0:RQ-1];
  logic            rq_rsp [0:RQ-1];
  logic [6:0]      rq_wp, rq_rp;
  logic [6:0]      rq_n;          // occupancy 0..RQ

  // m-side obligations (fence counters): writes counted at AW accept, reads
  // at AR accept; both decremented when the response is delivered or, for
  // orphans, swallowed.
  logic [5:0] w2_cnt, w0_cnt, r0_cnt, r2_cnt;

  logic post_stop;      // freeze after a clock stop until everything drains
  logic l2_ok_q;

  // one-cycle hold after an address handshake: keeps a master that holds
  // AWVALID/ARVALID past the accept edge from double-issuing (AXI requires
  // deassert-or-new-transaction, but this is free to guarantee)
  logic aw_hold, ar_hold;

  assign fence_stuck = post_stop && ((w2_cnt | w0_cnt | r0_cnt | r2_cnt) != '0);

  // An AW is accepted only while no write owns the W stream, so every entry
  // but the newest already has its W data: the newest entry is the W owner.
  // (Using the ring head here serialized all writes behind the oldest B.)
  logic whead_v, whead_open;
  assign whead_v   = (wq_n != 7'd0);
  assign whead     = (wq_wp == 7'd0) ? 7'(WQ - 1) : (wq_wp - 7'd1);
  assign whead_open = whead_v && !wq_wdn[whead];

  // BAR1 W stream state (the write that owns W is a BAR1 one)
  logic aw_open;       // a BAR1 write is outstanding and owns W
  logic [1:0] w_owner; // 0 = queue head, 1 = BAR1

  // ------------------------------------------------------------- AW decode ---
  tgt_t aw_tgt, ar_tgt;
  assign aw_tgt = decode(s_awaddr);
  assign ar_tgt = decode(s_araddr);

  logic aw_local;
  // BAD is always answered locally; BAR0 / BAR2 only while the L2 clock is down (BAR1 never)
  assign aw_local = (aw_tgt == T_BAD) || ((aw_tgt == T_BAR0 || aw_tgt == T_BAR2) && !l2_clk_ok);
  logic aw_fence;
  always_comb begin
    unique case (aw_tgt)
      T_BAR0: aw_fence = (w2_cnt != 0) || (w0_cnt != 0);
      T_BAR2: aw_fence = (w0_cnt != 0) || (w2_cnt == W_OSTD);
      default: aw_fence = 1'b0;
    endcase
  end
  // a new AW can only be accepted while no write owns the W stream
  logic w_stream_free;
  assign w_stream_free = !whead_open && !aw_open;

  logic aw_fwd;
  assign aw_fwd = !aw_local && !aw_fence && !post_stop;

  assign s_awready = aw_hold ? 1'b0 :
                     (aw_tgt == T_BAR1)
                   ? (m1_awready && w_stream_free)
                   : ((wq_n != 7'(WQ)) && w_stream_free &&
                      (aw_local || (aw_fwd && ((aw_tgt == T_BAR0) ? m0_awready : m2_awready))));

  assign m1_awid    = s_awid;
  assign m1_awaddr  = s_awaddr[A1_W-1:0];
  assign m1_awlen   = s_awlen;
  assign m1_awsize  = s_awsize;
  assign m1_awburst = s_awburst;
  assign m1_awcache = s_awcache;
  assign m1_awprot  = s_awprot;
  assign m1_awvalid = s_awvalid && s_awready && (aw_tgt == T_BAR1);

  logic aw_m0, aw_m2;
  assign aw_m0 = s_awvalid && s_awready && (aw_tgt == T_BAR0) && aw_fwd;
  assign aw_m2 = s_awvalid && s_awready && (aw_tgt == T_BAR2) && aw_fwd;
  assign m0_awid    = s_awid;
  assign m0_awaddr  = s_awaddr[A0_W-1:0];
  assign m0_awlen   = s_awlen;
  assign m0_awsize  = s_awsize;
  assign m0_awburst = s_awburst;
  assign m0_awcache = s_awcache;
  assign m0_awprot  = s_awprot;
  assign m0_awvalid = aw_m0;
  assign m2_awid    = s_awid;
  // P0-12 ruling: XDMA gives dword AxADDR; full-width (AxSIZE=3) transfers reach the RM
  // 8-byte aligned (DFX_BOUNDARY section 3); narrow transfers pass through.
  assign m2_awaddr  = {s_awaddr[A2_W-1:3], (s_awsize == 3'd3) ? 3'b000 : s_awaddr[2:0]};
  assign m2_awlen   = s_awlen;
  assign m2_awsize  = s_awsize;
  assign m2_awburst = s_awburst;
  assign m2_awcache = s_awcache;
  assign m2_awprot  = s_awprot;
  assign m2_awvalid = aw_m2;

  // --------------------------------------------------------------- W path ----
  assign s_wready = aw_open ? m1_wready
                   : whead_open ? (wq_fwd[whead]
                       ? ((wq_tgt[whead] == T_BAR0) ? m0_wready : m2_wready)
                       : 1'b1)
                   : 1'b0;

  logic w_m0, w_m2;
  assign w_m0 = s_wvalid && s_wready && !aw_open && whead_open &&
                wq_fwd[whead] && (wq_tgt[whead] == T_BAR0);
  assign w_m2 = s_wvalid && s_wready && !aw_open && whead_open &&
                wq_fwd[whead] && (wq_tgt[whead] == T_BAR2);
  // W pad engine: a clock stop that catches a forwarded write before its
  // last W beat leaves an AW with partial data in the m-side path.  The
  // host's remaining beats are absorbed locally (the entry is converted),
  // and the m-side burst is completed with WSTRB=0 beats so the downstream
  // never pairs the next write's data with this AW (DFX_BOUNDARY 5.3, same
  // rule as the iso blocks).  Its B then drains as an orphan.  No new write
  // is forwarded while post_stop holds, so the pad owns the m-side W.
  logic       pad_v, pad_m2;
  logic [8:0] pad_rem;
  logic [7:0] w_len_q;    // AWLEN of the W owner
  logic [8:0] w_sent;     // W beats of the W owner forwarded so far
  logic pad_m0_v, pad_m2_v, pad_hs;
  assign pad_m0_v = pad_v && !pad_m2;
  assign pad_m2_v = pad_v &&  pad_m2;
  assign pad_hs   = (pad_m0_v && m0_wready) || (pad_m2_v && m2_wready);

  assign m0_wdata  = pad_v ? 64'h0 : s_wdata;
  assign m0_wstrb  = pad_v ? 8'h0  : s_wstrb;
  assign m0_wlast  = pad_v ? (pad_rem == 9'd1) : s_wlast;
  assign m0_wvalid = w_m0 || pad_m0_v;
  assign m2_wdata  = pad_v ? 64'h0 : s_wdata;
  assign m2_wstrb  = pad_v ? 8'h0  : s_wstrb;
  assign m2_wlast  = pad_v ? (pad_rem == 9'd1) : s_wlast;
  assign m2_wvalid = w_m2 || pad_m2_v;
  assign m1_wdata  = s_wdata;
  assign m1_wstrb  = s_wstrb;
  assign m1_wlast  = s_wlast;
  assign m1_wvalid = s_wvalid && s_wready && aw_open;

  // --------------------------------------------------------------- AR path ---
  logic ar_local, ar_fence;
  // BAD is always answered locally; BAR0 / BAR2 only while the L2 clock is down (BAR1 never)
  assign ar_local = (ar_tgt == T_BAD) || ((ar_tgt == T_BAR0 || ar_tgt == T_BAR2) && !l2_clk_ok);
  always_comb begin
    unique case (ar_tgt)
      T_BAR0: ar_fence = (w2_cnt != 0) || (w0_cnt != 0) || (r0_cnt == R_OSTD);
      T_BAR2: ar_fence = (w2_cnt != 0) || (w0_cnt != 0) || (r2_cnt == R_OSTD);
      default: ar_fence = 1'b0;
    endcase
  end
  logic ar_fwd;
  assign ar_fwd = !ar_local && !ar_fence && !post_stop;

  assign s_arready = ar_hold ? 1'b0 :
                     (ar_tgt == T_BAR1) ? (m1_arready && (rq_n != 7'(RQ)))
                   : (rq_n != 7'(RQ)) &&
                     (ar_local || (ar_fwd && ((ar_tgt == T_BAR0) ? m0_arready : m2_arready)));

  assign m1_arid    = s_arid;
  assign m1_araddr  = s_araddr[A1_W-1:0];
  assign m1_arlen   = s_arlen;
  assign m1_arsize  = s_arsize;
  assign m1_arburst = s_arburst;
  assign m1_arcache = s_arcache;
  assign m1_arprot  = s_arprot;
  assign m1_arvalid = s_arvalid && s_arready && (ar_tgt == T_BAR1);

  logic ar_m0, ar_m2;
  assign ar_m0 = s_arvalid && s_arready && (ar_tgt == T_BAR0) && ar_fwd;
  assign ar_m2 = s_arvalid && s_arready && (ar_tgt == T_BAR2) && ar_fwd;
  assign m0_arid    = s_arid;
  assign m0_araddr  = s_araddr[A0_W-1:0];
  assign m0_arlen   = s_arlen;
  assign m0_arsize  = s_arsize;
  assign m0_arburst = s_arburst;
  assign m0_arcache = s_arcache;
  assign m0_arprot  = s_arprot;
  assign m0_arvalid = ar_m0;
  assign m2_arid    = s_arid;
  assign m2_araddr  = {s_araddr[A2_W-1:3], (s_arsize == 3'd3) ? 3'b000 : s_araddr[2:0]};
  assign m2_arlen   = s_arlen;
  assign m2_arsize  = s_arsize;
  assign m2_arburst = s_arburst;
  assign m2_arcache = s_arcache;
  assign m2_arprot  = s_arprot;
  assign m2_arvalid = ar_m2;

  // -------------------------------------------------- queue oldest searches --
  // Per-entry flags are evaluated at constant indices; the oldest match is
  // found by rotating the match vector so the read pointer lands on bit 0
  // and taking the lowest set bit.  (Loops that index the arrays with a
  // computed rp+i are both wrong past the ring end and pathological for
  // Vivado's elaborator.)
  logic [WQ-1:0] wq_live;
  logic [RQ-1:0] rq_live;
  always_comb begin
    for (int j = 0; j < WQ; j++)
      wq_live[j] = ((j >= int'(wq_rp)) ? (j - int'(wq_rp)) : (j + WQ - int'(wq_rp))) < int'(wq_n);
    for (int j = 0; j < RQ; j++)
      rq_live[j] = ((j >= int'(rq_rp)) ? (j - int'(rq_rp)) : (j + RQ - int'(rq_rp))) < int'(rq_n);
  end

  // oldest set bit of m (bit j = ring entry j) counted from rp: {valid, index}
  function automatic logic [7:0] oldest(input logic [63:0] m, input logic [6:0] rp, input int DEPTH);
    logic [127:0] r;
    logic         v;
    int           k;
    r = ({64'h0, m} | ({64'h0, m} << DEPTH)) >> rp;
    v = 1'b0; k = 0;
    for (int i = 63; i >= 0; i--)
      if (i < DEPTH && r[i]) begin v = 1'b1; k = i; end
    oldest = {v, qadd(rp, k, DEPTH)};
  endfunction

  logic [WQ-1:0] wm_b0, wm_b2, wm_bl;
  logic [RQ-1:0] rm_u, rm_l;
  always_comb begin
    for (int j = 0; j < WQ; j++) begin
      wm_b0[j] = wq_live[j] && wq_wdn[j] && !wq_rsp[j] &&  wq_fwd[j] && (wq_tgt[j] == T_BAR0);
      wm_b2[j] = wq_live[j] && wq_wdn[j] && !wq_rsp[j] &&  wq_fwd[j] && (wq_tgt[j] == T_BAR2);
      wm_bl[j] = wq_live[j] && wq_wdn[j] && !wq_rsp[j] && !wq_fwd[j];
    end
    for (int j = 0; j < RQ; j++) begin
      rm_u[j]  = rq_live[j] && !rq_rsp[j];                  // unanswered, oldest owns R
      rm_l[j]  = rq_live[j] && !rq_rsp[j] && !rq_fwd[j];    // locally answered (BAR1 is always fwd)
    end
  end

  logic [6:0] wb0, wb2, wbl;
  logic       wb0_v, wb2_v, wbl_v;
  assign {wb0_v, wb0} = oldest(64'(wm_b0), wq_rp, WQ);
  assign {wb2_v, wb2} = oldest(64'(wm_b2), wq_rp, WQ);
  assign {wbl_v, wbl} = oldest(64'(wm_bl), wq_rp, WQ);

  logic [6:0] rh;        // oldest unanswered read: the only one that may own R
  logic       rh_v;
  logic       rl_v;      // some local answer is still pending
  assign {rh_v, rh} = oldest(64'(rm_u), rq_rp, RQ);
  assign rl_v = |rm_l;
  logic [1:0] rh_tgt;
  logic       rh_fwd;
  assign rh_tgt = rq_tgt[rh];
  assign rh_fwd = rq_fwd[rh];

  // --------------------------------------------------------------- B path ----
  // m-side B: deliver to s (normal) or swallow (orphan).  Priority m0 > m2.
  logic b_swallow;   // orphan mode: m-side B is dropped, counts decrement
  assign b_swallow = post_stop || !l2_clk_ok;

  // s-side B source, priority m0 > m2 > m1 > local (m0/m2 only outside
  // orphan mode).  Once BVALID is up, the source -- and for queue sources
  // the entry -- is latched until the handshake: AXI forbids a pending B
  // from changing or being withdrawn, whether by a higher-priority arrival
  // or by a clock stop.  A held m0/m2 B is still delivered after a stop; its
  // entry was converted to local at the stop, and this completion retires it.
  localparam logic [1:0] BS_M0 = 2'd0, BS_M2 = 2'd1, BS_M1 = 2'd2, BS_LOC = 2'd3;
  logic       b_hold;          // a B was pending (not taken) last cycle
  logic [1:0] b_src_q;
  logic [6:0] b_idx_q;
  logic [1:0] b_src;           // effective source this cycle
  logic       b_any;           // a B is presented to s
  logic [6:0] b_idx;           // queue entry it retires (m0/m2/local)
  always_comb begin
    b_any = 1'b1; b_src = BS_LOC; b_idx = wbl;
    if (b_hold) begin
      b_src = b_src_q; b_idx = b_idx_q;
    end else if (!b_swallow && m0_bvalid && wb0_v) begin
      b_src = BS_M0; b_idx = wb0;
    end else if (!b_swallow && m2_bvalid && wb2_v) begin
      b_src = BS_M2; b_idx = wb2;
    end else if (m1_bvalid) begin
      b_src = BS_M1;
    end else if (!wbl_v) begin
      b_any = 1'b0;
    end
  end

  logic b_from_m0, b_from_m2, b_from_m1, b_from_loc;
  assign b_from_m0  = b_any && (b_src == BS_M0);
  assign b_from_m2  = b_any && (b_src == BS_M2);
  assign b_from_m1  = b_any && (b_src == BS_M1);
  assign b_from_loc = b_any && (b_src == BS_LOC);

  // m0/m2 B that is not being delivered is swallowed in orphan mode
  assign m0_bready = b_from_m0 ? s_bready : b_swallow;
  assign m2_bready = b_from_m2 ? s_bready : b_swallow;
  assign m1_bready = b_from_m1 && s_bready;

  assign s_bvalid = b_any;
  assign s_bid    = b_from_m0 ? m0_bid
                  : b_from_m2 ? m2_bid
                  : b_from_m1 ? m1_bid
                  : wq_id[b_idx];
  assign s_bresp  = b_from_m0 ? m0_bresp
                  : b_from_m2 ? m2_bresp
                  : b_from_m1 ? m1_bresp
                  : ((wq_tgt[b_idx] == T_BAD) ? 2'b10 : 2'b00);  // BAD=SLVERR, stop=OKAY

  // --------------------------------------------------------------- R path ----
  // one source owns the s R channel until RLAST: 0=m0 1=m2 2=local 3=m1.
  logic [1:0] r_owner;
  logic       r_owner_v;
  logic [6:0] r_idx;     // queue entry of the owned burst
  logic [7:0] rl_beat;   // beats of the owned burst already delivered
  logic       r_conv;    // stop seen during an m0/m2 burst, convert at a beat boundary

  logic r_swallow;
  assign r_swallow = post_stop || !l2_clk_ok;

  logic own_m0, own_m2, own_m1, own_loc;
  assign own_m0  = r_owner_v && (r_owner == 2'd0);
  assign own_m2  = r_owner_v && (r_owner == 2'd1);
  assign own_m1  = r_owner_v && (r_owner == 2'd3);
  assign own_loc = r_owner_v && (r_owner == 2'd2);

  assign m0_rready = own_m0 ? s_rready : (r_swallow ? 1'b1 : 1'b0);
  assign m2_rready = own_m2 ? s_rready : (r_swallow ? 1'b1 : 1'b0);
  assign m1_rready = own_m1 ? s_rready : 1'b0;

  assign s_rvalid = own_m0  ? m0_rvalid
                  : own_m2  ? m2_rvalid
                  : own_m1  ? m1_rvalid
                  : own_loc;
  assign s_rid    = own_m0 ? m0_rid : own_m2 ? m2_rid : own_m1 ? m1_rid : rq_id[r_idx];
  assign s_rdata  = own_m0 ? m0_rdata : own_m2 ? m2_rdata : own_m1 ? m1_rdata
                  : 64'hFFFF_FFFF_FFFF_FFFF;
  assign s_rresp  = own_m0 ? m0_rresp : own_m2 ? m2_rresp : own_m1 ? m1_rresp
                  : ((rq_tgt[r_idx] == T_BAD) ? 2'b10 : 2'b00);
  assign s_rlast  = own_m0 ? m0_rlast : own_m2 ? m2_rlast : own_m1 ? m1_rlast
                  : (rl_beat == rq_len[r_idx]);

  // ------------------------------------------------------- update terms ---
  logic stop_edge;                       // l2_clk_ok just fell
  assign stop_edge = l2_ok_q && !l2_clk_ok;

  logic w_push, r_push, w_pop, r_pop;
  assign w_push = s_awvalid && s_awready && (aw_tgt != T_BAR1);
  assign r_push = s_arvalid && s_arready;      // every read takes a ring slot (BAR1 too)
  assign w_pop  = (wq_n != 7'd0) && wq_rsp[wq_rp];
  assign r_pop  = (rq_n != 7'd0) && rq_rsp[rq_rp];

  // response completions that retire a queue entry
  logic r0_done, r2_done, rl_done;
  logic b_retire;   // the s-side B handshake retires queue entry b_idx
  assign b_retire = s_bvalid && s_bready && !b_from_m1;
  assign r0_done = own_m0 && m0_rvalid && s_rready && m0_rlast;
  assign r2_done = own_m2 && m2_rvalid && s_rready && m2_rlast;
  logic r1_done;
  assign r1_done = own_m1 && m1_rvalid && s_rready && m1_rlast;
  assign rl_done = own_loc && s_rready && (rl_beat == rq_len[r_idx]);
  logic r_done, r_beat_hs;
  assign r_done    = r0_done || r2_done || r1_done || rl_done;
  assign r_beat_hs = s_rvalid && s_rready && (own_m0 || own_m2 || own_loc);

  // W owner progress (pad bookkeeping)
  logic w_fwd_hs;
  assign w_fwd_hs = w_m0 || w_m2;   // w_m* already include the m-side ready

  // Obligation counters: one merged update per cycle (an accept and a
  // completion in the same cycle must both count), floored at zero.
  function automatic logic [5:0] cnt_upd(input logic [5:0] c, input logic inc, input int dec);
    int n;
    n = int'(c) + int'(inc) - dec;
    cnt_upd = (n < 0) ? 6'd0 : 6'(n);
  endfunction

  logic [5:0] w0_nx, w2_nx, r0_nx, r2_nx;
  // (a write caught mid-W by a stop keeps its obligation: the pad engine
  // completes it, so its B does come back and is swallowed)
  assign w0_nx = cnt_upd(w0_cnt, aw_m0, int'(m0_bvalid && m0_bready));
  assign w2_nx = cnt_upd(w2_cnt, aw_m2, int'(m2_bvalid && m2_bready));
  // an m-side read burst ends on its RLAST handshake, whether delivered
  // (owner) or swallowed (orphan)
  assign r0_nx = cnt_upd(r0_cnt, ar_m0, int'(m0_rvalid && m0_rready && m0_rlast));
  assign r2_nx = cnt_upd(r2_cnt, ar_m2, int'(m2_rvalid && m2_rready && m2_rlast));

  // ------------------------------------------------------------ sequential ---
  always_ff @(posedge clk_sh or negedge rst_sh_n) begin
    if (!rst_sh_n) begin
      aw_hold <= 1'b0; ar_hold <= 1'b0;
      wq_wp <= 7'd0; wq_rp <= 7'd0; wq_n <= 7'd0;
      rq_wp <= 7'd0; rq_rp <= 7'd0; rq_n <= 7'd0;
      w2_cnt <= 6'd0; w0_cnt <= 6'd0; r0_cnt <= 6'd0; r2_cnt <= 6'd0;
      post_stop <= 1'b0; l2_ok_q <= 1'b0;
      r_owner <= 2'd0; r_owner_v <= 1'b0; rl_beat <= 8'd0; r_idx <= 7'd0; r_conv <= 1'b0;
      b_hold <= 1'b0; b_src_q <= BS_LOC; b_idx_q <= 7'd0;
      aw_open <= 1'b0;
      pad_v <= 1'b0; pad_m2 <= 1'b0; pad_rem <= 9'd0;
      w_len_q <= 8'd0; w_sent <= 9'd0;
    end else begin
      // -------- handshake holds
      aw_hold <= s_awvalid && s_awready;
      ar_hold <= s_arvalid && s_arready;

      // -------- stop detection
      l2_ok_q <= l2_clk_ok;
      if (stop_edge) post_stop <= 1'b1;
      else if (post_stop && l2_clk_ok && w2_cnt == 0 && w0_cnt == 0 &&
               r0_cnt == 0 && r2_cnt == 0 && !wbl_v && !rl_v && !r_owner_v && !pad_v)
        post_stop <= 1'b0;

      // -------- W owner bookkeeping and the stop pad
      if (w_push) begin
        w_len_q <= s_awlen;
        w_sent  <= 9'd0;
      end else if (w_fwd_hs) begin
        w_sent  <= w_sent + 9'd1;
      end
      if (stop_edge && whead_open && wq_fwd[whead] &&
          !(w_fwd_hs && s_wlast)) begin
        pad_v   <= 1'b1;
        pad_m2  <= (wq_tgt[whead] == T_BAR2);
        pad_rem <= {1'b0, w_len_q} + 9'd1 - w_sent - 9'(w_fwd_hs);
      end else if (pad_hs) begin
        pad_rem <= pad_rem - 9'd1;
        if (pad_rem == 9'd1) pad_v <= 1'b0;
      end

      w0_cnt <= w0_nx; w2_cnt <= w2_nx;
      r0_cnt <= r0_nx; r2_cnt <= r2_nx;

      // -------- BAR1 W stream ownership
      if (s_awvalid && s_awready && aw_tgt == T_BAR1) aw_open <= 1'b1;
      if (aw_open && m1_wvalid && m1_wready && m1_wlast) aw_open <= 1'b0;

      // -------- ring pointers
      if (w_push) wq_wp <= qnext(wq_wp, WQ);
      if (w_pop)  wq_rp <= qnext(wq_rp, WQ);
      wq_n <= wq_n + 7'(w_push) - 7'(w_pop);
      if (r_push) rq_wp <= qnext(rq_wp, RQ);
      if (r_pop)  rq_rp <= qnext(rq_rp, RQ);
      rq_n <= rq_n + 7'(r_push) - 7'(r_pop);

      // -------- R ownership
      if (!r_owner_v) begin
        rl_beat <= 8'd0;
        // only the oldest unanswered read may take R (in-order responses)
        if (rh_v) begin
          if (rh_tgt == T_BAR1) begin
            if (m1_rvalid) begin r_owner <= 2'd3; r_owner_v <= 1'b1; r_idx <= rh; end
          end else if (!rh_fwd) begin
            r_owner <= 2'd2; r_owner_v <= 1'b1; r_idx <= rh;
          end else if (!r_swallow) begin
            if (rh_tgt == T_BAR0 && m0_rvalid) begin r_owner <= 2'd0; r_owner_v <= 1'b1; r_idx <= rh; end
            if (rh_tgt == T_BAR2 && m2_rvalid) begin r_owner <= 2'd1; r_owner_v <= 1'b1; r_idx <= rh; end
          end
        end
      end else if (r_done) begin
        r_owner_v <= 1'b0;
        r_conv    <= 1'b0;
      end else begin
        if (r_beat_hs) rl_beat <= rl_beat + 8'd1;
        // a stop in the middle of an m0/m2 burst: the rest of the burst is
        // answered locally (all ones, OKAY) and the m-side remainder is
        // swallowed with the other orphans.  A beat already presented to s
        // must stay unchanged until taken (AXI), so switch at a beat boundary.
        if ((stop_edge || r_conv) && (own_m0 || own_m2)) begin
          if (s_rvalid && !s_rready) r_conv <= 1'b1;
          else begin r_owner <= 2'd2; r_conv <= 1'b0; end
        end
      end

      // -------- s-side B hold (see the B path)
      b_hold  <= s_bvalid && !s_bready;
      b_src_q <= b_src;
      b_idx_q <= b_idx;
    end
  end

  // ------------------------------------------------------- queue entries ---
  // Every array write is at a constant index (no reset: an entry's fields
  // are written when it is pushed, and only live entries are ever read).
  always_ff @(posedge clk_sh) begin
    for (int j = 0; j < WQ; j++) begin
      // conversion to local at a clock stop (an entry pushed this cycle is
      // decided by aw_fwd, which already sees the stop)
      if (stop_edge && wq_live[j]) wq_fwd[j] <= 1'b0;
      if (w_push && 7'(j) == wq_wp) begin
        wq_tgt[j] <= aw_tgt;
        wq_id [j] <= s_awid;
        wq_fwd[j] <= aw_fwd;   // 0 when aw_local (BAD or dead clock)
        wq_wdn[j] <= 1'b0;
        wq_rsp[j] <= 1'b0;
      end
      if (!aw_open && whead_open && s_wvalid && s_wready && s_wlast && 7'(j) == whead)
        wq_wdn[j] <= 1'b1;
      if (b_retire && 7'(j) == b_idx)
        wq_rsp[j] <= 1'b1;
    end
    for (int j = 0; j < RQ; j++) begin
      if (stop_edge && rq_live[j] && rq_tgt[j] != T_BAR1) rq_fwd[j] <= 1'b0;
      if (r_push && 7'(j) == rq_wp) begin
        rq_tgt[j] <= ar_tgt;
        rq_id [j] <= s_arid;
        rq_len[j] <= s_arlen;
        rq_fwd[j] <= ar_fwd || (ar_tgt == T_BAR1);   // BAR1 never converts to local
        rq_rsp[j] <= 1'b0;
      end
      if (r_done && 7'(j) == r_idx)
        rq_rsp[j] <= 1'b1;
    end
  end

endmodule
