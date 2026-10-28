// ng_rmt_bar2.sv -- SLR1 side of the BAR2 window.
//
// BAR2 offset [27:26] = channel n, [25:0] = offset in the 64 MiB slice
// WIN_SEL[n]; channel byte address = {WIN_SEL[n], off[25:3], 3'b000} (AxADDR[2:0]
// only steps narrow beats, WSTRB decides the bytes).  One write burst and one read burst at a time
// (independent paths), each answered in arrival order.  Every 64-bit beat
// becomes one link item to channel n (<= 16 beats in flight per path); the
// write B is sent after every beat's channel B is back (SLVERR if any failed);
// read beats carry their own channel RRESP.  AxSIZE < 3 (narrow) is served as
// plain AXI INCR: beat address = first address, then aligned to the size and
// stepped by 1<<AxSIZE; each beat is one 64-bit channel item at the word
// containing that address (write lanes/strobes come from the master, a read
// returns the whole word and the master picks its lanes).  AxSIZE > 3 cannot
// occur on a 64-bit port: SLVERR without touching the channel (W beats
// consumed; reads return all-ones).
// All s_axi_bar2 outputs come from skid flip-flops.
module ng_rmt_bar2 (
  input  logic         clk,
  input  logic         rst_n,
  // s_axi_bar2
  input  logic [3:0]   s_awid,
  input  logic [27:0]  s_awaddr,
  input  logic [7:0]   s_awlen,
  input  logic [2:0]   s_awsize,
  input  logic         s_awvalid,
  output logic         s_awready,
  input  logic [63:0]  s_wdata,
  input  logic [7:0]   s_wstrb,
  input  logic         s_wlast,
  input  logic         s_wvalid,
  output logic         s_wready,
  output logic [3:0]   s_bid,
  output logic [1:0]   s_bresp,
  output logic         s_bvalid,
  input  logic         s_bready,
  input  logic [3:0]   s_arid,
  input  logic [27:0]  s_araddr,
  input  logic [7:0]   s_arlen,
  input  logic [2:0]   s_arsize,
  input  logic         s_arvalid,
  output logic         s_arready,
  output logic [3:0]   s_rid,
  output logic [63:0]  s_rdata,
  output logic [1:0]   s_rresp,
  output logic         s_rlast,
  output logic         s_rvalid,
  input  logic         s_rready,
  // configuration / status
  input  logic [6:0]   win_sel [4],
  output logic [3:0]   ch_busy,
  // link requests (payload shared, valid one-hot per channel)
  output logic [3:0]   wq_valid,
  input  logic [3:0]   wq_ready,
  output logic [106:0] wq_data,
  output logic [3:0]   rq_valid,
  input  logic [3:0]   rq_ready,
  output logic [106:0] rq_data,
  // link responses (BW / BR kinds only reach here)
  input  logic [3:0]   bw_valid,
  input  logic [3:0]   bw_err,
  input  logic [3:0]   br_valid,
  input  logic [65:0]  br_data [4]   // {resp, data64}
);
  localparam logic [1:0] K_BW = 2'd0, K_BR = 2'd1;

  // ---------------------------------------------------------------- input skids
  logic        aw_v, aw_r, w_v, w_r, ar_v, ar_r;
  logic [3:0]  aw_id, ar_id;
  logic [27:0] aw_a, ar_a;
  logic [7:0]  aw_len, ar_len;
  logic [2:0]  aw_size, ar_size;
  logic [63:0] w_d;
  logic [7:0]  w_s;
  logic        w_l;

  ng_rmt_islice #(.W(43)) u_aw (
    .clk(clk), .rst_n(rst_n),
    .s_valid(s_awvalid), .s_ready(s_awready), .s_data({s_awid, s_awaddr, s_awlen, s_awsize}),
    .m_valid(aw_v), .m_ready(aw_r), .m_data({aw_id, aw_a, aw_len, aw_size}));
  ng_rmt_islice #(.W(73)) u_w (
    .clk(clk), .rst_n(rst_n),
    .s_valid(s_wvalid), .s_ready(s_wready), .s_data({s_wdata, s_wstrb, s_wlast}),
    .m_valid(w_v), .m_ready(w_r), .m_data({w_d, w_s, w_l}));
  ng_rmt_islice #(.W(43)) u_ar (
    .clk(clk), .rst_n(rst_n),
    .s_valid(s_arvalid), .s_ready(s_arready), .s_data({s_arid, s_araddr, s_arlen, s_arsize}),
    .m_valid(ar_v), .m_ready(ar_r), .m_data({ar_id, ar_a, ar_len, ar_size}));

  // ---------------------------------------------------------------- output skids
  logic       b_v, b_r;
  logic [3:0] b_id;
  logic [1:0] b_resp;
  ng_prim_skid #(.W(6), .ATTR(1'b1)) u_b (
    .clk(clk), .rst_n(rst_n),
    .s_valid(b_v), .s_ready(b_r), .s_data({b_id, b_resp}),
    .m_valid(s_bvalid), .m_ready(s_bready), .m_data({s_bid, s_bresp}), .occ());

  logic        ro_v, ro_r, ro_last;
  logic [3:0]  ro_id;
  logic [63:0] ro_d;
  logic [1:0]  ro_resp;
  ng_prim_skid #(.W(71), .ATTR(1'b1)) u_r (
    .clk(clk), .rst_n(rst_n),
    .s_valid(ro_v), .s_ready(ro_r), .s_data({ro_id, ro_d, ro_resp, ro_last}),
    .m_valid(s_rvalid), .m_ready(s_rready), .m_data({s_rid, s_rdata, s_rresp, s_rlast}), .occ());

  // channel byte address of the 64-bit word holding beat address off
  function automatic logic [32:0] chaddr(input logic [6:0] win, input logic [25:0] off);
    return {win, off[25:3], 3'b000};
  endfunction

  // next INCR beat address: align to the transfer size, then step by it
  function automatic logic [25:0] beat_next(input logic [25:0] a, input logic [2:0] sz);
    logic [25:0] step;
    step = 26'd1 << sz;
    return (a & ~(step - 26'd1)) + step;
  endfunction

  // ================================================================ write path
  typedef enum logic [1:0] {WI, WD, WW} wst_t;
  wst_t        ws;
  logic [1:0]  wch;
  logic [25:0] woff;
  logic [2:0]  wsz;
  logic [3:0]  wid;
  logic        wbad, werr;
  logic [8:0]  wiss, wrcv;
  logic [6:0]  wwin;

  wire w_room = (wiss - wrcv) < 9'd16;
  wire w_fwd  = (ws == WD) && w_v && !wbad && w_room;
  assign wq_valid = w_fwd ? (4'b0001 << wch) : 4'b0000;
  assign wq_data  = {K_BW, chaddr(wwin, woff), w_s, w_d};
  wire w_link_rdy = wq_ready[wch];
  assign w_r   = (ws == WD) && (wbad || (w_room && w_link_rdy));
  assign aw_r  = (ws == WI);
  assign b_v   = (ws == WW) && (wiss == wrcv);
  assign b_id  = wid;
  assign b_resp = (werr || wbad) ? 2'b10 : 2'b00;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ws <= WI; wiss <= '0; wrcv <= '0; werr <= 1'b0; wbad <= 1'b0; wch <= '0;
    end else begin
      if (|bw_valid) begin
        wrcv <= wrcv + 1'b1;
        if (|(bw_valid & bw_err)) werr <= 1'b1;
      end
      unique case (ws)
        WI: if (aw_v) begin
          ws   <= WD;
          wch  <= aw_a[27:26];
          woff <= aw_a[25:0];
          wsz  <= aw_size;
          wid  <= aw_id;
          wbad <= (aw_size > 3'd3);
          wwin <= win_sel[aw_a[27:26]];
          wiss <= '0;
          wrcv <= '0;
          werr <= 1'b0;
        end
        WD: if (w_v && w_r) begin
          if (!wbad) begin
            wiss <= wiss + 1'b1;
            woff <= beat_next(woff, wsz);
          end
          if (w_l) ws <= WW;
        end
        default: if (b_v && b_r) ws <= WI;
      endcase
    end
  end

  // ================================================================ read path
  typedef enum logic [0:0] {RI, RS} rst_t;
  rst_t        rs;
  logic [1:0]  rch;
  logic [25:0] roff;
  logic [2:0]  rsz;
  logic [3:0]  rid;
  logic        rbad;
  logic [7:0]  rlen;
  logic [8:0]  riss, rsent;
  logic [6:0]  rwin;
  logic        rf_empty;
  logic [65:0] rf_dout;

  wire r_room = (riss - rsent) < 9'd16;
  wire r_fwd  = (rs == RS) && !rbad && (riss <= 9'(rlen)) && r_room;
  assign rq_valid = r_fwd ? (4'b0001 << rch) : 4'b0000;
  assign rq_data  = {K_BR, chaddr(rwin, roff), 8'h00, 64'h0};
  wire r_iss = r_fwd && rq_ready[rch];
  assign ar_r = (rs == RI);

  assign ro_v    = (rs == RS) && (rbad || !rf_empty);
  assign ro_id   = rid;
  assign ro_d    = rbad ? '1 : rf_dout[63:0];
  assign ro_resp = rbad ? 2'b10 : rf_dout[65:64];
  assign ro_last = (rsent == 9'(rlen));
  wire ro_hs = ro_v && ro_r;

  logic        br_hit;
  logic [65:0] br_sel;
  always_comb begin
    br_hit = |br_valid;
    br_sel = br_data[rch];
  end

  ng_prim_fifo_sync #(.W(66), .DEPTH(16), .REG_FLAGS(1'b1)) u_rf (
    .clk(clk), .rst_n(rst_n), .push(br_hit), .din(br_sel), .pop(ro_hs && !rbad),
    .dout(rf_dout), .empty(rf_empty), .full(), .count());

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rs <= RI; riss <= '0; rsent <= '0; rbad <= 1'b0; rch <= '0;
    end else begin
      unique case (rs)
        RI: if (ar_v) begin
          rs    <= RS;
          rch   <= ar_a[27:26];
          roff  <= ar_a[25:0];
          rsz   <= ar_size;
          rid   <= ar_id;
          rlen  <= ar_len;
          rbad  <= (ar_size > 3'd3);
          rwin  <= win_sel[ar_a[27:26]];
          riss  <= '0;
          rsent <= '0;
        end
        default: begin
          if (r_iss) begin
            riss <= riss + 1'b1;
            roff <= beat_next(roff, rsz);
          end
          if (ro_hs) begin
            rsent <= rsent + 1'b1;
            if (ro_last) rs <= RI;
          end
        end
      endcase
    end
  end

  assign ch_busy = ((ws != WI) ? (4'b0001 << wch) : 4'b0000) |
                   ((rs != RI) ? (4'b0001 << rch) : 4'b0000);
endmodule
