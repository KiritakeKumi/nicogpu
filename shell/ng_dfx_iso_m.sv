// ng_dfx_iso_m.sv -- static-side isolation for an AXI4 port on which the RM is
// the master (m_axi_c1..c4, m_axi_host).  DFX_BOUNDARY.md
//
// dec = 0: transparent, but enforces the boundary rules the shell relies on:
//          at most MAX_RD / MAX_WR outstanding transactions, W never ahead of
//          its AW, WLAST regenerated from AWLEN.
// dec = 1: RM requests are blocked (RM sees ready = 0), R/B responses from the
//          shell are accepted and dropped, and any write burst whose AW was
//          already issued is completed with WSTRB = 0 beats, so the MIG /
//          PCIe bridge never sees a torn burst.
// drained = nothing in flight (the precondition for a clean reset / PR).
// Isolation is released only once drained, so a response that belongs to a
// transaction issued before the decouple can never reach the (reset) RM.
module ng_dfx_iso_m #(
  parameter int AW_PL_W = 64,
  parameter int W_PL_W  = 72,   // {wdata, wstrb}; all-zero payload = no byte written
  parameter int B_PL_W  = 8,
  parameter int AR_PL_W = 64,
  parameter int R_PL_W  = 72,
  parameter int MAX_RD  = 32,
  parameter int MAX_WR  = 32
) (
  input  logic               clk,
  input  logic               rst_n,
  input  logic               dec,        // decouple request
  output logic               drained,
  output logic [8:0]         rd_out,
  output logic [8:0]         wr_out,
  // RM side (RM is the master)
  input  logic               rm_aw_valid,
  output logic               rm_aw_ready,
  input  logic [AW_PL_W-1:0] rm_aw_pl,
  input  logic [7:0]         rm_aw_len,
  input  logic               rm_w_valid,
  output logic               rm_w_ready,
  input  logic [W_PL_W-1:0]  rm_w_pl,
  output logic               rm_b_valid,
  input  logic               rm_b_ready,
  output logic [B_PL_W-1:0]  rm_b_pl,
  input  logic               rm_ar_valid,
  output logic               rm_ar_ready,
  input  logic [AR_PL_W-1:0] rm_ar_pl,
  output logic               rm_r_valid,
  input  logic               rm_r_ready,
  output logic [R_PL_W-1:0]  rm_r_pl,
  output logic               rm_r_last,
  // shell side (towards the MIG / PCIe bridge)
  output logic               sh_aw_valid,
  input  logic               sh_aw_ready,
  output logic [AW_PL_W-1:0] sh_aw_pl,
  output logic               sh_w_valid,
  input  logic               sh_w_ready,
  output logic [W_PL_W-1:0]  sh_w_pl,
  output logic               sh_w_last,
  input  logic               sh_b_valid,
  output logic               sh_b_ready,
  input  logic [B_PL_W-1:0]  sh_b_pl,
  output logic               sh_ar_valid,
  input  logic               sh_ar_ready,
  output logic [AR_PL_W-1:0] sh_ar_pl,
  input  logic               sh_r_valid,
  output logic               sh_r_ready,
  input  logic [R_PL_W-1:0]  sh_r_pl,
  input  logic               sh_r_last
);
  logic iso;
  always_ff @(posedge clk) begin
    if (!rst_n) iso <= 1'b1;
    else        iso <= dec || (iso && !drained);
  end

  // --------------------------------------------------------------
  logic rd_room;
  assign rd_room     = (rd_out < 9'(MAX_RD));
  assign sh_ar_valid = rm_ar_valid && !iso && rd_room;
  assign rm_ar_ready = sh_ar_ready && !iso && rd_room;
  assign sh_ar_pl    = rm_ar_pl;

  assign rm_r_valid  = sh_r_valid && !iso;
  assign sh_r_ready  = iso ? 1'b1 : rm_r_ready;
  assign rm_r_pl     = sh_r_pl;
  assign rm_r_last   = sh_r_last;

  logic ar_hs, r_done;
  assign ar_hs  = sh_ar_valid && sh_ar_ready;
  assign r_done = sh_r_valid && sh_r_ready && sh_r_last;

  // --------------------------------------------------------------
  logic       wl_empty, wl_full, wl_pop;
  logic [7:0] wl_len, wbeat;
  logic [$clog2(MAX_WR):0] wl_cnt;

  assign sh_aw_valid = rm_aw_valid && !iso && !wl_full && (wr_out < 9'(MAX_WR));
  assign rm_aw_ready = sh_aw_ready && !iso && !wl_full && (wr_out < 9'(MAX_WR));
  assign sh_aw_pl    = rm_aw_pl;

  logic aw_hs, w_hs, b_hs;
  assign aw_hs = sh_aw_valid && sh_aw_ready;

  ng_prim_fifo_sync #(.W(8), .DEPTH(MAX_WR), .GATED(1'b1)) u_wlen (
    .clk(clk), .rst_n(rst_n), .push(aw_hs), .din(rm_aw_len), .pop(wl_pop),
    .dout(wl_len), .empty(wl_empty), .full(wl_full), .count(wl_cnt)
  );

  // W only flows for a burst whose AW has already been accepted.
  assign sh_w_valid = !wl_empty && (iso ? 1'b1 : rm_w_valid);
  assign rm_w_ready = !wl_empty && !iso && sh_w_ready;
  assign sh_w_pl    = iso ? '0 : rm_w_pl;
  assign sh_w_last  = (wbeat == wl_len);
  assign w_hs       = sh_w_valid && sh_w_ready;
  assign wl_pop     = w_hs && sh_w_last;

  assign rm_b_valid = sh_b_valid && !iso;
  assign sh_b_ready = iso ? 1'b1 : rm_b_ready;
  assign rm_b_pl    = sh_b_pl;
  assign b_hs       = sh_b_valid && sh_b_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rd_out <= '0;
      wr_out <= '0;
      wbeat  <= '0;
    end else begin
      rd_out <= rd_out + 9'(ar_hs) - 9'(r_done);
      wr_out <= wr_out + 9'(aw_hs) - 9'(b_hs);
      if (w_hs) wbeat <= sh_w_last ? 8'd0 : wbeat + 8'd1;
    end
  end

  assign drained = (rd_out == '0) && (wr_out == '0) && wl_empty;
endmodule
