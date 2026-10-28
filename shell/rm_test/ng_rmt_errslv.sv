// ng_rmt_errslv.sv -- local error responder for a requester that does not own
// its channel: the request never reaches the channel port.
// Writes: AW accepted, W beats absorbed up to WLAST, one SLVERR B.
// Reads: AR accepted, ARLEN+1 beats of all-ones data with SLVERR and RLAST.
// One write and one read at a time, answered in order.
module ng_rmt_errslv #(
  parameter int DW = 64
) (
  input  logic          clk,
  input  logic          rst_n,
  output logic          busy,
  input  logic          s_awvalid,
  output logic          s_awready,
  input  logic          s_wvalid,
  output logic          s_wready,
  input  logic          s_wlast,
  output logic          s_bvalid,
  input  logic          s_bready,
  output logic [1:0]    s_bresp,
  input  logic          s_arvalid,
  output logic          s_arready,
  input  logic [7:0]    s_arlen,
  output logic          s_rvalid,
  input  logic          s_rready,
  output logic [DW-1:0] s_rdata,
  output logic [1:0]    s_rresp,
  output logic          s_rlast
);
  typedef enum logic [1:0] {W_IDLE, W_DATA, W_RESP} ws_t;
  ws_t        ws;
  logic       r_act;
  logic [7:0] r_left;

  assign s_awready = (ws == W_IDLE);
  assign s_wready  = (ws == W_DATA);
  assign s_bvalid  = (ws == W_RESP);
  assign s_bresp   = 2'b10;
  assign s_arready = !r_act;
  assign s_rvalid  = r_act;
  assign s_rdata   = '1;
  assign s_rresp   = 2'b10;
  assign s_rlast   = (r_left == 8'd0);
  assign busy      = (ws != W_IDLE) || r_act;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ws     <= W_IDLE;
      r_act  <= 1'b0;
      r_left <= '0;
    end else begin
      unique case (ws)
        W_IDLE:  if (s_awvalid) ws <= W_DATA;
        W_DATA:  if (s_wvalid && s_wlast) ws <= W_RESP;
        default: if (s_bready) ws <= W_IDLE;
      endcase
      if (!r_act) begin
        if (s_arvalid) begin
          r_act  <= 1'b1;
          r_left <= s_arlen;
        end
      end else if (s_rready) begin
        if (r_left == 8'd0) r_act <= 1'b0;
        else                r_left <= r_left - 8'd1;
      end
    end
  end
endmodule
