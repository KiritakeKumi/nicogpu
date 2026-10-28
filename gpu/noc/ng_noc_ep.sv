// ng_noc_ep.sv -- NoC endpoint adapter: converts the router credit
// link to plain valid/ready streams for agents (L2 slice, TB drivers,
// future LSU / CP / copy engine front-ends).
//
//   TX (agent -> NoC): small FIFO; a flit is injected when the per-VC credit
//     shadow for its class is non-zero (credits = router local input FIFO
//     depth, returned when the router forwards the flit).
//   RX (NoC -> agent): FIFO of depth RX_DEPTH; the router reserved
//     LOC_CREDITS slots per VC, so total occupancy never exceeds
//     NUM_VC * LOC_CREDITS -- set RX_DEPTH = NUM_VC * LOC_CREDITS.
//     A credit pulse is returned to the router when the agent pops a flit.
module ng_noc_ep #(
  parameter int NUM_VC = 2,
  parameter int INJ_CREDITS = 4,          // = router BUF_DEPTH
  parameter int RX_DEPTH = 2 * 4,         // = NUM_VC * LOC_CREDITS
  parameter int TXQ_DEPTH = 4             // agent -> NoC staging FIFO
) (
  input  logic clk,
  input  logic rst_n,
  // agent side: transmit stream in, receive stream out
  input  logic                       ti_valid,
  output logic                       ti_ready,
  input  ng_noc_pkg::flit_t          ti_flit,
  output logic                       to_valid,
  input  logic                       to_ready,
  output ng_noc_pkg::flit_t          to_flit,
  // NoC side
  output logic                       inj_valid,
  output ng_noc_pkg::flit_t          inj_flit,
  input  logic [NUM_VC-1:0]          inj_credit,   // pulses from the router
  input  logic                       recv_valid,
  input  ng_noc_pkg::flit_t          recv_flit,
  output logic [NUM_VC-1:0]          recv_credit   // pulses to the router
);

  import ng_noc_pkg::*;

  localparam int CR_W = $clog2(INJ_CREDITS + 1) + 1;
  localparam int TQ_LB = clog2_1(TXQ_DEPTH);
  localparam int TQ_CNT = TQ_LB + 1;
  localparam int RX_LB = clog2_1(RX_DEPTH);
  localparam int RX_CNT = RX_LB + 1;

  // FIFO pointers wrap at the depth through ng_noc_pkg::ptr_inc (the plain `+ 1` was only right for
  // power-of-two depths; TXQ_DEPTH = 1 indexed txq[1] and a depth of 3 / 6 / ... ran past the array,
  //

  // ---------------- TX ------------------------------------------------------
  flit_t             txq [TXQ_DEPTH];
  logic [TQ_LB-1:0]  tx_rp, tx_wp;
  logic [TQ_CNT-1:0] tx_cnt;
  logic [CR_W-1:0]   cred_tx [NUM_VC];
  logic              tx_fire;
  logic [1:0]        head_cls;

  // X-safe: class of the head flit, 0 when the queue is empty
  assign head_cls = (tx_cnt != '0) ? txq[tx_rp].cls : 2'd0;
  assign tx_fire   = (tx_cnt != '0) && (cred_tx[head_cls] != '0);
  assign inj_valid = tx_fire;
  assign inj_flit  = (tx_cnt != '0) ? txq[tx_rp] : '0;
  assign ti_ready  = (tx_cnt < TQ_CNT'(TXQ_DEPTH));

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      tx_rp  <= '0;
      tx_wp  <= '0;
      tx_cnt <= '0;
      for (int v = 0; v < NUM_VC; v++) cred_tx[v] <= CR_W'(INJ_CREDITS);
    end else begin
      if (ti_valid && ti_ready) begin
        txq[tx_wp] <= ti_flit;
        tx_wp <= TQ_LB'(ptr_inc(tx_wp, TXQ_DEPTH));
      end
      if (tx_fire) tx_rp <= TQ_LB'(ptr_inc(tx_rp, TXQ_DEPTH));
      tx_cnt <= tx_cnt + TQ_CNT'(ti_valid && ti_ready ? 1 : 0)
                       - TQ_CNT'(tx_fire ? 1 : 0);
      // credit shadow: consume on fire, return on router pulse (a cycle can
      // do both to the same VC -- keep them in one assignment)
      for (int v = 0; v < NUM_VC; v++) begin
        cred_tx[v] <= cred_tx[v] + CR_W'(inj_credit[v]) - CR_W'(tx_fire && (head_cls == 2'(v)));
      end
    end
  end

  // ---------------- RX ------------------------------------------------------
  flit_t             rxq [RX_DEPTH];
  logic [RX_LB-1:0]  rx_rp, rx_wp;
  logic [RX_CNT-1:0] rx_cnt;
  logic [1:0]        rx_head_cls;

  assign to_valid = (rx_cnt != '0);
  assign to_flit  = (rx_cnt != '0) ? rxq[rx_rp] : '0;
  assign rx_head_cls = (rx_cnt != '0) ? rxq[rx_rp].cls : 2'd0;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rx_rp  <= '0;
      rx_wp  <= '0;
      rx_cnt <= '0;
      recv_credit <= '0;
    end else begin
      recv_credit <= '0;
      if (recv_valid) begin  // space guaranteed by router credit accounting
        rxq[rx_wp] <= recv_flit;
        rx_wp <= RX_LB'(ptr_inc(rx_wp, RX_DEPTH));
      end
      if (to_valid && to_ready) begin
        rx_rp <= RX_LB'(ptr_inc(rx_rp, RX_DEPTH));
        recv_credit[rx_head_cls] <= 1'b1;
      end
      rx_cnt <= rx_cnt + RX_CNT'(recv_valid ? 1 : 0)
                       - RX_CNT'((to_valid && to_ready) ? 1 : 0);
    end
  end

  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (rst_n) begin
      if (recv_valid && (rx_cnt == RX_CNT'(RX_DEPTH)))
        $error("ng_noc_ep: RX overflow (credit protocol violated)");
      if (ti_valid && ti_flit.cls >= NUM_VC)
        $error("ng_noc_ep: flit class out of range");
    end
  end
  // synthesis translate_on
endmodule
