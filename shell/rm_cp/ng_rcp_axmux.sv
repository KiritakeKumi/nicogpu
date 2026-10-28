// ng_rcp_axmux.sv -- 2-master -> 1-slave AXI4 multiplexer for the ID-less RM-internal AXI (section 2.1).
//
// The RM-internal AXI carries no ID signals: every transaction is ID 0, so the slave returns R bursts and B
// responses strictly in issue order.  The mux therefore remembers, per channel, which master each accepted
// address belongs to and routes the responses (and fetches the W data) by that history:
//   AR  round-robin between s0/s1; the grant is locked from the cycle m_arvalid is presented until m_arready
//       (AXI: a presented valid must not change).  On the AR handshake the master index goes into the R-order FIFO.
//       R beats go to the master at the FIFO head; the entry is popped on the beat with rlast.
//   AW  same arbitration.  On the AW handshake the index is pushed into the W-order FIFO and the B-order FIFO.
//       W beats are only taken from the master at the W-FIFO head (popped at wlast); B goes to the B-FIFO head.
//   No AR/AW is presented (m_*valid stays low) while the corresponding FIFO is full, so DEPTH must be >= the
//   outstanding limits of the slave port behind the mux (16 for ng_rmt_port as used by the CP RM).
//   The address and data paths are purely combinational muxes (no slice); the only state is the grant lock, the
//   round-robin bit and the three order FIFOs.  m_*valid depends on the masters' valids and on FIFO-full (state),
//   never on m_*ready of the same channel.  Master-side ready is m_*valid && m_*ready for the granted master.
//   idle = no outstanding transaction (R/W/B order FIFOs empty), so no W burst is open either.
//
// Ports: s0_*/s1_* are the master-facing slave ports, m_* is the slave-facing master port with the same signal set
// as ng_rmt_port's internal side (s_*), so m_* connects straight to it.
//
// REG_A (default 0 = the behaviour): AW and AR leave through a full-throughput register slice (all
// outputs and the slice's ready come from flops).  Without it the ready chain  port -> engine mux -> this mux -> master  is
// combinational through every mux level; with it the chain ends in the slice.  The order FIFOs are pushed when the
// slice accepts an address, so DEPTH must be >= the slave's outstanding limit + 2 (the two slice entries), and `idle`
// then counts only transactions that have left the slice (an address still waiting in the slice has not reached the
// slave, so a quiesce that holds it off must not keep idle low).
//
// no write-before-read ordering between the two masters, no QoS / priority, one round-robin
// bit per address channel.  See

module ng_rcp_axmux #(
  parameter int DW    = 64,
  parameter int AW    = 47,
  parameter int DEPTH = 16,
  parameter bit REG_A = 1'b0
) (
  input  logic            clk,
  input  logic            rst_n,
  output logic            idle,
  // master 0
  input  logic            s0_awvalid,
  output logic            s0_awready,
  input  logic [AW-1:0]   s0_awaddr,
  input  logic [7:0]      s0_awlen,
  input  logic            s0_wvalid,
  output logic            s0_wready,
  input  logic [DW-1:0]   s0_wdata,
  input  logic [DW/8-1:0] s0_wstrb,
  input  logic            s0_wlast,
  output logic            s0_bvalid,
  input  logic            s0_bready,
  output logic [1:0]      s0_bresp,
  input  logic            s0_arvalid,
  output logic            s0_arready,
  input  logic [AW-1:0]   s0_araddr,
  input  logic [7:0]      s0_arlen,
  output logic            s0_rvalid,
  input  logic            s0_rready,
  output logic [DW-1:0]   s0_rdata,
  output logic [1:0]      s0_rresp,
  output logic            s0_rlast,
  // master 1
  input  logic            s1_awvalid,
  output logic            s1_awready,
  input  logic [AW-1:0]   s1_awaddr,
  input  logic [7:0]      s1_awlen,
  input  logic            s1_wvalid,
  output logic            s1_wready,
  input  logic [DW-1:0]   s1_wdata,
  input  logic [DW/8-1:0] s1_wstrb,
  input  logic            s1_wlast,
  output logic            s1_bvalid,
  input  logic            s1_bready,
  output logic [1:0]      s1_bresp,
  input  logic            s1_arvalid,
  output logic            s1_arready,
  input  logic [AW-1:0]   s1_araddr,
  input  logic [7:0]      s1_arlen,
  output logic            s1_rvalid,
  input  logic            s1_rready,
  output logic [DW-1:0]   s1_rdata,
  output logic [1:0]      s1_rresp,
  output logic            s1_rlast,
  // slave side
  output logic            m_awvalid,
  input  logic            m_awready,
  output logic [AW-1:0]   m_awaddr,
  output logic [7:0]      m_awlen,
  output logic            m_wvalid,
  input  logic            m_wready,
  output logic [DW-1:0]   m_wdata,
  output logic [DW/8-1:0] m_wstrb,
  output logic            m_wlast,
  input  logic            m_bvalid,
  output logic            m_bready,
  input  logic [1:0]      m_bresp,
  output logic            m_arvalid,
  input  logic            m_arready,
  output logic [AW-1:0]   m_araddr,
  output logic [7:0]      m_arlen,
  input  logic            m_rvalid,
  output logic            m_rready,
  input  logic [DW-1:0]   m_rdata,
  input  logic [1:0]      m_rresp,
  input  logic            m_rlast
);
  // ------------------------------------------------------------------ order FIFOs
  localparam int CW = $clog2(DEPTH + 1);
  logic rf_push, rf_pop, rf_head, rf_empty, rf_full;
  logic wf_push, wf_pop, wf_head, wf_empty, wf_full;
  logic bf_push, bf_pop, bf_head, bf_empty, bf_full;
  logic [CW-1:0] rf_cnt, wf_cnt, bf_cnt;
  logic ar_sel, aw_sel;            // granted master of the AR / AW channel

  ng_prim_fifo_sync #(.W(1), .DEPTH(DEPTH)) u_rf (.clk, .rst_n, .push(rf_push), .din(ar_sel), .pop(rf_pop),
                                                .dout(rf_head), .empty(rf_empty), .full(rf_full), .count(rf_cnt));
  ng_prim_fifo_sync #(.W(1), .DEPTH(DEPTH)) u_wf (.clk, .rst_n, .push(wf_push), .din(aw_sel), .pop(wf_pop),
                                                .dout(wf_head), .empty(wf_empty), .full(wf_full), .count(wf_cnt));
  ng_prim_fifo_sync #(.W(1), .DEPTH(DEPTH)) u_bf (.clk, .rst_n, .push(bf_push), .din(aw_sel), .pop(bf_pop),
                                                .dout(bf_head), .empty(bf_empty), .full(bf_full), .count(bf_cnt));

  // AR / AW as presented by the arbiters (i_*) and the slave-facing port (m_*): wires, or through a register slice
  logic          wok;                   // W of the head burst may go: its AW has left the register slice
  logic          i_awvalid, i_awready, i_arvalid, i_arready;
  logic [AW-1:0] i_awaddr, i_araddr;
  logic [7:0]    i_awlen, i_arlen;
  if (REG_A) begin : g_ra
    logic [1:0] aw_occ, ar_occ;
    ng_prim_skid #(.W(AW + 8)) u_aw (.clk, .rst_n, .s_valid(i_awvalid), .s_ready(i_awready),
                                   .s_data({i_awaddr, i_awlen}), .m_valid(m_awvalid), .m_ready(m_awready),
                                   .m_data({m_awaddr, m_awlen}), .occ(aw_occ));
    ng_prim_skid #(.W(AW + 8)) u_ar (.clk, .rst_n, .s_valid(i_arvalid), .s_ready(i_arready),
                                   .s_data({i_araddr, i_arlen}), .m_valid(m_arvalid), .m_ready(m_arready),
                                   .m_data({m_araddr, m_arlen}), .occ(ar_occ));
    assign idle = (rf_cnt == CW'(ar_occ)) && (wf_cnt == CW'(aw_occ)) && (bf_cnt == CW'(aw_occ));
    // W only after the AW went out of the slice (the W FIFO is pushed when the slice accepts): credits = AWs sent and
    // not yet followed by their W burst; AWs leave in order, so a credit means the head burst's AW has gone
    logic [CW:0] wcred;
    always_ff @(posedge clk) begin
      if (!rst_n) wcred <= '0;
      else        wcred <= wcred + (CW+1)'(m_awvalid && m_awready) - (CW+1)'(wf_pop);
    end
    assign wok = (wcred != '0);
  end else begin : g_nr
    assign m_awvalid = i_awvalid;  assign m_awaddr = i_awaddr;  assign m_awlen = i_awlen;  assign i_awready = m_awready;
    assign m_arvalid = i_arvalid;  assign m_araddr = i_araddr;  assign m_arlen = i_arlen;  assign i_arready = m_arready;
    assign idle = rf_empty && wf_empty && bf_empty;
    assign wok  = 1'b1;
  end

  // ------------------------------------------------------------------ AR arbitration (lock + round robin)
  logic ar_lock, ar_lock_sel, ar_last;     // lock: m_arvalid presented, not yet accepted
  logic ar_any, ar_pick, ar_go;
  assign ar_any  = s0_arvalid || s1_arvalid;
  // both valid -> the one not granted last; otherwise whoever is valid
  assign ar_pick = (s0_arvalid && s1_arvalid) ? !ar_last : s1_arvalid;
  assign ar_sel  = ar_lock ? ar_lock_sel : ar_pick;

  assign i_arvalid  = ar_any && !rf_full;
  assign i_araddr   = ar_sel ? s1_araddr : s0_araddr;
  assign i_arlen    = ar_sel ? s1_arlen  : s0_arlen;
  assign ar_go      = i_arvalid && i_arready;
  assign s0_arready = ar_go && !ar_sel;
  assign s1_arready = ar_go &&  ar_sel;
  assign rf_push    = ar_go;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ar_lock <= 1'b0; ar_lock_sel <= 1'b0; ar_last <= 1'b1;   // first grant goes to master 0
    end else if (ar_go) begin
      ar_lock <= 1'b0; ar_last <= ar_sel;
    end else if (i_arvalid) begin
      ar_lock <= 1'b1; ar_lock_sel <= ar_sel;
    end
  end

  // ------------------------------------------------------------------ AW arbitration
  logic aw_lock, aw_lock_sel, aw_last;
  logic aw_any, aw_pick, aw_go;
  assign aw_any  = s0_awvalid || s1_awvalid;
  assign aw_pick = (s0_awvalid && s1_awvalid) ? !aw_last : s1_awvalid;
  assign aw_sel  = aw_lock ? aw_lock_sel : aw_pick;

  assign i_awvalid  = aw_any && !wf_full && !bf_full;
  assign i_awaddr   = aw_sel ? s1_awaddr : s0_awaddr;
  assign i_awlen    = aw_sel ? s1_awlen  : s0_awlen;
  assign aw_go      = i_awvalid && i_awready;
  assign s0_awready = aw_go && !aw_sel;
  assign s1_awready = aw_go &&  aw_sel;
  assign wf_push    = aw_go;
  assign bf_push    = aw_go;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      aw_lock <= 1'b0; aw_lock_sel <= 1'b0; aw_last <= 1'b1;
    end else if (aw_go) begin
      aw_lock <= 1'b0; aw_last <= aw_sel;
    end else if (i_awvalid) begin
      aw_lock <= 1'b1; aw_lock_sel <= aw_sel;
    end
  end

  // ------------------------------------------------------------------ W: only from the master at the W-FIFO head
  assign m_wvalid  = !wf_empty && wok && (wf_head ? s1_wvalid : s0_wvalid);
  assign m_wdata   = wf_head ? s1_wdata : s0_wdata;
  assign m_wstrb   = wf_head ? s1_wstrb : s0_wstrb;
  assign m_wlast   = wf_head ? s1_wlast : s0_wlast;
  assign s0_wready = !wf_empty && wok && !wf_head && m_wready;
  assign s1_wready = !wf_empty && wok &&  wf_head && m_wready;
  assign wf_pop    = m_wvalid && m_wready && m_wlast;

  // ------------------------------------------------------------------ B: routed by the B-FIFO head
  assign s0_bvalid = m_bvalid && !bf_empty && !bf_head;
  assign s1_bvalid = m_bvalid && !bf_empty &&  bf_head;
  assign s0_bresp  = m_bresp;
  assign s1_bresp  = m_bresp;
  assign m_bready  = !bf_empty && (bf_head ? s1_bready : s0_bready);
  assign bf_pop    = m_bvalid && m_bready;

  // ------------------------------------------------------------------ R: routed by the R-FIFO head
  assign s0_rvalid = m_rvalid && !rf_empty && !rf_head;
  assign s1_rvalid = m_rvalid && !rf_empty &&  rf_head;
  assign s0_rdata  = m_rdata;
  assign s1_rdata  = m_rdata;
  assign s0_rresp  = m_rresp;
  assign s1_rresp  = m_rresp;
  assign s0_rlast  = m_rlast;
  assign s1_rlast  = m_rlast;
  assign m_rready  = !rf_empty && (rf_head ? s1_rready : s0_rready);
  assign rf_pop    = m_rvalid && m_rready && m_rlast;
endmodule
