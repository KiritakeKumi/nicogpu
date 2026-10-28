// ng_dfx_iso_s.sv -- static-side isolation for a port on which the RM is the
// slave (s_axil_ctrl, s_axi_bar2).  DFX_BOUNDARY.md
//
// Every host request accepted by the shell is recorded in order.  Boundary
// rule: the RM answers R and B strictly in request order, so the head of each
// queue is always the transaction being answered.
// dec = 0: transparent; RID/BID/RLAST towards the host are regenerated from the
//          queues so a misbehaving RM cannot corrupt the PCIe bridge.
// dec = 1: nothing reaches the RM; outstanding and new host requests are
//          answered locally (ERR_RESP, read data ERR_DATA), including the tail
//          of a read burst the RM had started, and W beats of accepted AWs are
//          sunk.  The host therefore never sees a completion timeout.
// Isolation takes effect in the cycle after dec rises.  When dec falls the
// block first stops accepting new requests (hold), waits until every locally
// answered transaction has completed (idle) and only then reconnects the RM,
// so no request can straddle the switch.
module ng_dfx_iso_s #(
  parameter int              ID_W     = 4,
  parameter int              DATA_W   = 64,
  parameter int              AW_PL_W  = 64,
  parameter int              W_PL_W   = 72,
  parameter int              AR_PL_W  = 64,
  parameter int              MAX_RD   = 8,
  parameter int              MAX_WR   = 8,
  parameter logic [1:0]      ERR_RESP = 2'b10,
  parameter logic [DATA_W-1:0] ERR_DATA = '1
) (
  input  logic               clk,
  input  logic               rst_n,
  input  logic               dec,        // decouple request
  output logic               idle,
  output logic               err_pulse,
  output logic [8:0]         rd_out,
  output logic [8:0]         wr_out,
  // shell side (host bridge is the master)
  input  logic               sh_aw_valid,
  output logic               sh_aw_ready,
  input  logic [AW_PL_W-1:0] sh_aw_pl,
  input  logic [ID_W-1:0]    sh_aw_id,
  input  logic               sh_w_valid,
  output logic               sh_w_ready,
  input  logic [W_PL_W-1:0]  sh_w_pl,
  input  logic               sh_w_last,
  output logic               sh_b_valid,
  input  logic               sh_b_ready,
  output logic [ID_W-1:0]    sh_b_id,
  output logic [1:0]         sh_b_resp,
  input  logic               sh_ar_valid,
  output logic               sh_ar_ready,
  input  logic [AR_PL_W-1:0] sh_ar_pl,
  input  logic [ID_W-1:0]    sh_ar_id,
  input  logic [7:0]         sh_ar_len,
  output logic               sh_r_valid,
  input  logic               sh_r_ready,
  output logic [ID_W-1:0]    sh_r_id,
  output logic [DATA_W-1:0]  sh_r_data,
  output logic [1:0]         sh_r_resp,
  output logic               sh_r_last,
  // RM side (RM is the slave)
  output logic               rm_aw_valid,
  input  logic               rm_aw_ready,
  output logic [AW_PL_W-1:0] rm_aw_pl,
  output logic               rm_w_valid,
  input  logic               rm_w_ready,
  output logic [W_PL_W-1:0]  rm_w_pl,
  output logic               rm_w_last,
  input  logic               rm_b_valid,
  output logic               rm_b_ready,
  input  logic [ID_W-1:0]    rm_b_id,
  input  logic [1:0]         rm_b_resp,
  output logic               rm_ar_valid,
  input  logic               rm_ar_ready,
  output logic [AR_PL_W-1:0] rm_ar_pl,
  input  logic               rm_r_valid,
  output logic               rm_r_ready,
  input  logic [ID_W-1:0]    rm_r_id,
  input  logic [DATA_W-1:0]  rm_r_data,
  input  logic [1:0]         rm_r_resp,
  input  logic               rm_r_last
);
  logic iso, hold;   // effective isolation, reconnect in progress
  // --------------------------------------------------------------
  logic                   rq_empty, rq_full, rq_pop;
  logic [ID_W+7:0]        rq_head;
  logic [$clog2(MAX_RD):0] rq_cnt;
  logic [7:0]             rbeat;
  logic                   ar_hs, r_hs;

  assign rm_ar_valid = sh_ar_valid && !rq_full && !iso && !hold;
  assign sh_ar_ready = !rq_full && !hold && (iso ? 1'b1 : rm_ar_ready);
  assign rm_ar_pl    = sh_ar_pl;
  assign ar_hs       = sh_ar_valid && sh_ar_ready;

  ng_prim_fifo_sync #(.W(ID_W + 8), .DEPTH(MAX_RD), .GATED(1'b1)) u_rq (
    .clk(clk), .rst_n(rst_n), .push(ar_hs), .din({sh_ar_id, sh_ar_len}), .pop(rq_pop),
    .dout(rq_head), .empty(rq_empty), .full(rq_full), .count(rq_cnt)
  );

  // RM responses with nothing outstanding (or while decoupled) are dropped.
  assign rm_r_ready = iso || rq_empty || sh_r_ready;
  assign sh_r_valid = !rq_empty && (iso || rm_r_valid);
  assign sh_r_id    = rq_head[ID_W+7:8];
  assign sh_r_last  = (rbeat == rq_head[7:0]);
  assign sh_r_data  = iso ? ERR_DATA : rm_r_data;
  assign sh_r_resp  = iso ? ERR_RESP : rm_r_resp;
  assign r_hs       = sh_r_valid && sh_r_ready;
  assign rq_pop     = r_hs && sh_r_last;

  // --------------------------------------------------------------
  logic                   wq_empty, wq_full, wq_pop;
  logic [ID_W-1:0]        wq_head;
  logic [$clog2(MAX_WR):0] wq_cnt;
  logic [8:0]             wdone;      // W bursts fully transferred, not yet answered
  logic [8:0]             wowed;      // accepted AWs whose W burst has not completed
  logic                   w_in_burst;
  logic                   aw_hs, w_hs, b_hs;

  assign rm_aw_valid = sh_aw_valid && !wq_full && !iso && !hold;
  assign sh_aw_ready = !wq_full && !hold && (iso ? 1'b1 : rm_aw_ready);
  assign rm_aw_pl    = sh_aw_pl;
  assign aw_hs       = sh_aw_valid && sh_aw_ready;

  ng_prim_fifo_sync #(.W(ID_W), .DEPTH(MAX_WR), .GATED(1'b1)) u_wq (
    .clk(clk), .rst_n(rst_n), .push(aw_hs), .din(sh_aw_id), .pop(wq_pop),
    .dout(wq_head), .empty(wq_empty), .full(wq_full), .count(wq_cnt)
  );

  // While isolated, W beats are only sunk for bursts whose AW was accepted
  // (a slave may wait for AW before accepting W), so no beat is lost if the
  // host sends W ahead of AW around a reconnect.
  assign rm_w_valid = sh_w_valid && !iso;
  assign sh_w_ready = iso ? ((wowed != '0) || w_in_burst) : rm_w_ready;
  assign rm_w_pl    = sh_w_pl;
  assign rm_w_last  = sh_w_last;
  assign w_hs       = sh_w_valid && sh_w_ready;

  assign rm_b_ready = iso || wq_empty || (sh_b_ready && (wdone != '0));
  assign sh_b_valid = !wq_empty && (wdone != '0) && (iso || rm_b_valid);
  assign sh_b_id    = wq_head;
  assign sh_b_resp  = iso ? ERR_RESP : rm_b_resp;
  assign b_hs       = sh_b_valid && sh_b_ready;
  assign wq_pop     = b_hs;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      iso        <= 1'b1;
      hold       <= 1'b0;
      rbeat      <= '0;
      wdone      <= '0;
      wowed      <= '0;
      w_in_burst <= 1'b0;
      err_pulse  <= 1'b0;
    end else begin
      if (dec) begin
        iso  <= 1'b1;
        hold <= 1'b0;
      end else if (iso) begin
        if (!hold)     hold <= 1'b1;
        else if (idle) begin
          iso  <= 1'b0;
          hold <= 1'b0;
        end
      end
      if (r_hs) rbeat <= sh_r_last ? 8'd0 : rbeat + 8'd1;
      wdone <= wdone + 9'(w_hs && sh_w_last) - 9'(b_hs);
      wowed <= wowed + 9'(aw_hs) - 9'(w_hs && sh_w_last);
      if (w_hs) w_in_burst <= !sh_w_last;
      err_pulse <= iso && ((r_hs && sh_r_last) || b_hs);
    end
  end

  assign rd_out = 9'(rq_cnt);
  assign wr_out = 9'(wq_cnt);
  assign idle   = rq_empty && wq_empty && (wdone == '0) && !w_in_burst;
endmodule
