// ng_rmt_dmach.sv -- channel side of the DMA path.
//
// A block is 512 B.  Write: 64 link items (64-bit each, block address on every
// item) are packed LSB-first into W beats; the AW(s) of the block are issued
// when its first item arrives (C1 1x8 beats, C2/C4 1x16, C3 2x16).  When all
// B's of the block are back, one "done" response carries the OR of the
// errors.  Read: one link item requests a block; the R beats are unpacked
// into 64 responses of 64 bits (each with its beat's RRESP).
module ng_rmt_dmach #(
  parameter int DW = 512,
  parameter int AW = 33
) (
  input  logic          clk,
  input  logic          rst_n,
  output logic          busy,
  input  logic          i_valid,
  output logic          i_ready,
  input  logic          i_rd,
  input  logic [AW-1:0] i_addr,
  input  logic [63:0]   i_data,
  output logic          o_valid,
  input  logic          o_ready,
  output logic [66:0]   o_data,      // {is_rd, resp, data64}
  output logic          m_awvalid,
  input  logic          m_awready,
  output logic [AW-1:0] m_awaddr,
  output logic [7:0]    m_awlen,
  output logic          m_wvalid,
  input  logic          m_wready,
  output logic [DW-1:0] m_wdata,
  output logic          m_wlast,
  input  logic          m_bvalid,
  output logic          m_bready,
  input  logic [1:0]    m_bresp,
  output logic          m_arvalid,
  input  logic          m_arready,
  output logic [AW-1:0] m_araddr,
  output logic [7:0]    m_arlen,
  input  logic          m_rvalid,
  output logic          m_rready,
  input  logic [DW-1:0] m_rdata,
  input  logic [1:0]    m_rresp
);
  localparam int L     = DW / 64;
  localparam int BPB   = DW / 8;
  localparam int BEATS = 512 / BPB;
  localparam int BLEN  = (BEATS > 16) ? 16 : BEATS;
  localparam int NB    = BEATS / BLEN;
  localparam int SW    = (L > 1) ? $clog2(L) : 1;

  logic          rhv, rv_out, r_act;

  // --------------------------------------------------------------
  logic          w_act;             // block in progress (first item seen .. done sent)
  logic [5:0]    w_items;           // items accepted in this block
  logic [SW-1:0] w_sub;
  logic [DW-1:0] pk;
  logic          wv;
  logic [DW-1:0] wd;
  logic [4:0]    wbeat;
  logic [1:0]    aw_left, b_left;
  logic [AW-1:0] aw_addr;
  logic          b_err, dv;

  wire item_w = i_valid && !i_rd;
  wire item_r = i_valid && i_rd;
  wire w_can  = !dv && !(w_act && w_items == 6'd0) && (!wv || (w_sub != SW'(L - 1)));
  assign i_ready = i_rd ? !r_act : w_can;

  wire w_acc = item_w && w_can;
  wire [DW-1:0] pk_next = (L > 1) ? {i_data, pk[DW-1:64]} : DW'(i_data);

  assign m_awvalid = (aw_left != 2'd0);
  assign m_awaddr  = aw_addr;
  assign m_awlen   = 8'(BLEN - 1);
  assign m_wvalid  = wv;
  assign m_wdata   = wd;
  assign m_wlast   = (wbeat == 5'(BLEN - 1));
  assign m_bready  = 1'b1;

  wire aw_hs = m_awvalid && m_awready;
  wire w_hs  = m_wvalid && m_wready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      w_act <= 1'b0; w_items <= '0; w_sub <= '0; wv <= 1'b0; wbeat <= '0;
      aw_left <= '0; b_left <= '0; b_err <= 1'b0; dv <= 1'b0;
    end else begin
      if (w_acc) begin
        w_items <= w_items + 1'b1;
        w_sub   <= (w_sub == SW'(L - 1)) ? '0 : w_sub + 1'b1;
        if (!w_act) begin
          w_act   <= 1'b1;
          aw_left <= 2'(NB);
          b_left  <= 2'(NB);
          b_err   <= 1'b0;
          aw_addr <= {i_addr[AW-1:9], 9'h0};
        end
      end
      if (aw_hs) begin
        aw_left <= aw_left - 1'b1;
        aw_addr <= aw_addr + AW'(BLEN * BPB);
      end
      if (w_acc && w_sub == SW'(L - 1)) wv <= 1'b1;
      else if (w_hs)                    wv <= 1'b0;
      if (w_hs) wbeat <= m_wlast ? 5'd0 : wbeat + 5'd1;
      if (m_bvalid) begin
        b_left <= b_left - 1'b1;
        if (m_bresp != 2'b00) b_err <= 1'b1;
        if (b_left == 2'd1) dv <= 1'b1;
      end
      if (dv && o_ready && !rv_out) begin
        dv      <= 1'b0;
        w_act   <= 1'b0;
        w_items <= '0;
      end
    end
  end
  always_ff @(posedge clk) begin
    if (w_acc) pk <= pk_next;
    if (w_acc && w_sub == SW'(L - 1)) wd <= pk_next;
  end

  // --------------------------------------------------------------
  logic [1:0]    ar_left;
  logic [AW-1:0] ar_addr;
  logic [DW-1:0] rh;
  logic [1:0]    rhr;
  logic [SW-1:0] r_sub;
  logic [6:0]    r_items;           // responses still to send

  assign m_arvalid = (ar_left != 2'd0);
  assign m_araddr  = ar_addr;
  assign m_arlen   = 8'(BLEN - 1);
  assign m_rready  = !rhv;
  wire ar_hs = m_arvalid && m_arready;
  wire r_hs  = m_rvalid && m_rready;
  assign rv_out = rhv;
  wire o_hs_r = rhv && o_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      r_act <= 1'b0; ar_left <= '0; rhv <= 1'b0; r_sub <= '0; r_items <= '0;
    end else begin
      if (item_r && !r_act) begin
        r_act   <= 1'b1;
        ar_left <= 2'(NB);
        ar_addr <= {i_addr[AW-1:9], 9'h0};
        r_items <= 7'd64;
      end
      if (ar_hs) begin
        ar_left <= ar_left - 1'b1;
        ar_addr <= ar_addr + AW'(BLEN * BPB);
      end
      if (r_hs) begin
        rhv   <= 1'b1;
        r_sub <= '0;
      end else if (o_hs_r) begin
        r_sub   <= r_sub + 1'b1;
        r_items <= r_items - 1'b1;
        if (r_sub == SW'(L - 1)) rhv <= 1'b0;
        if (r_items == 7'd1) r_act <= 1'b0;
      end
    end
  end
  always_ff @(posedge clk) begin
    if (r_hs) begin
      rh  <= m_rdata;
      rhr <= m_rresp;
    end else if (o_hs_r && L > 1) begin
      rh <= rh >> 64;
    end
  end

  // --------------------------------------------------------------
  assign o_valid = rhv || dv;
  assign o_data  = rhv ? {1'b1, rhr, rh[63:0]} : {1'b0, (b_err ? 2'b10 : 2'b00), 64'h0};

  assign busy = w_act || r_act || (aw_left != 0) || (ar_left != 0);
endmodule
