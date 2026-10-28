// ng_rmt_bar2ch.sv -- channel side of the BAR2 path.
//
// Each link item is one 64-bit BAR2 beat already translated to a channel
// byte address ({WIN_SEL, off[25:3], 3'b0}).  A write beat becomes one
// single-beat full-width write: data replicated to every 64-bit lane, WSTRB =
// the beat's strobes shifted to the addressed lane (no wide mux).  A read beat
// becomes one single-beat read; the addressed lane is picked from R.  B and R
// results go back as link responses in issue order (ID 0 keeps order).
module ng_rmt_bar2ch #(
  parameter int DW = 512,
  parameter int AW = 33
) (
  input  logic            clk,
  input  logic            rst_n,
  output logic            busy,
  // link items
  input  logic            i_valid,
  output logic            i_ready,
  input  logic            i_rd,
  input  logic [AW-1:0]   i_addr,
  input  logic [63:0]     i_data,
  input  logic [7:0]      i_strb,
  // responses
  output logic            bo_valid,
  input  logic            bo_ready,
  output logic [1:0]      bo_resp,
  output logic            ro_valid,
  input  logic            ro_ready,
  output logic [65:0]     ro_data,     // {resp, data64}
  // master
  output logic            m_awvalid,
  input  logic            m_awready,
  output logic [AW-1:0]   m_awaddr,
  output logic            m_wvalid,
  input  logic            m_wready,
  output logic [DW-1:0]   m_wdata,
  output logic [DW/8-1:0] m_wstrb,
  input  logic            m_bvalid,
  output logic            m_bready,
  input  logic [1:0]      m_bresp,
  output logic            m_arvalid,
  input  logic            m_arready,
  output logic [AW-1:0]   m_araddr,
  input  logic            m_rvalid,
  output logic            m_rready,
  input  logic [DW-1:0]   m_rdata,
  input  logic [1:0]      m_rresp
);
  localparam int L  = DW / 64;
  localparam int LG = $clog2(DW / 8);
  localparam int LW = LG - 3;          // lane index width (>= 1 here)

  logic          hv, hk, aw_sent, w_sent;
  logic [AW-1:0] haddr;
  logic [63:0]   hdata;
  logic [7:0]    hstrb;
  logic [4:0]    wr_out, rd_out;
  logic [LW-1:0] lane_h, lane_q;
  logic          lf_empty, lf_full;

  assign lane_h  = haddr[LG-1:3];
  assign i_ready = !hv;

  assign m_awvalid = hv && !hk && !aw_sent && (wr_out < 5'd16);
  assign m_awaddr  = {haddr[AW-1:LG], LG'(0)};
  assign m_wvalid  = hv && !hk && !w_sent;
  assign m_wdata   = {L{hdata}};
  assign m_wstrb   = (DW/8)'(hstrb) << (8 * lane_h);
  assign m_arvalid = hv && hk && !lf_full;
  assign m_araddr  = {haddr[AW-1:LG], LG'(0)};

  wire aw_hs = m_awvalid && m_awready;
  wire w_hs  = m_wvalid && m_wready;
  wire ar_hs = m_arvalid && m_arready;
  wire wr_fin = hv && !hk && (aw_sent || aw_hs) && (w_sent || w_hs);

  assign bo_valid = m_bvalid;
  assign bo_resp  = m_bresp;
  assign m_bready = bo_ready;
  wire b_hs = m_bvalid && m_bready;

  logic ro_sready;
  assign m_rready = ro_sready && !lf_empty;
  wire r_hs = m_rvalid && m_rready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      hv <= 1'b0; aw_sent <= 1'b0; w_sent <= 1'b0;
      wr_out <= '0; rd_out <= '0;
    end else begin
      if (i_valid && i_ready) begin
        hv <= 1'b1;
        aw_sent <= 1'b0;
        w_sent  <= 1'b0;
      end else if (wr_fin || ar_hs) begin
        hv <= 1'b0;
      end else begin
        if (aw_hs) aw_sent <= 1'b1;
        if (w_hs)  w_sent  <= 1'b1;
      end
      wr_out <= wr_out + 5'(aw_hs) - 5'(b_hs);
      rd_out <= rd_out + 5'(ar_hs) - 5'(r_hs);
    end
  end
  always_ff @(posedge clk) begin
    if (i_valid && i_ready) begin
      hk    <= i_rd;
      haddr <= i_addr;
      hdata <= i_data;
      hstrb <= i_strb;
    end
  end

  assign busy = hv || (wr_out != '0) || (rd_out != '0);

  ng_prim_fifo_sync #(.W(LW), .DEPTH(16), .REG_FLAGS(1'b1)) u_lane (
    .clk(clk), .rst_n(rst_n), .push(ar_hs), .din(lane_h), .pop(r_hs),
    .dout(lane_q), .empty(lf_empty), .full(lf_full), .count());

  ng_prim_skid #(.W(66), .ATTR(1'b1)) u_ro (
    .clk(clk), .rst_n(rst_n),
    .s_valid(m_rvalid && !lf_empty), .s_ready(ro_sready),
    .s_data({m_rresp, m_rdata[64*lane_q +: 64]}),
    .m_valid(ro_valid), .m_ready(ro_ready), .m_data(ro_data), .occ());
endmodule
