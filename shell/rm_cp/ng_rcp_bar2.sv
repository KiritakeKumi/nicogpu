// ng_rcp_bar2.sv -- BAR2 adapter of the CP RM: C3 window, nothing else.
//
// BAR2 offset o (28 bits).  With C3_WIN_CTRL.ENABLE and window size S = 2^SIZE_LOG2, offsets
// o >= 256 MiB - S go to C3 at C3_WIN_BASE + (o - (256 MiB - S)); everything else would be VRAM, which this RM does
// not have: reads return SLVERR with data 0, writes are consumed and answered SLVERR (no memory access).
// Bursts are executed beat by beat: every beat is one 64-bit operation on the C3 engine (requester 0), the next beat
// starts when the previous one is complete, B / the R beats follow the completions in order.  INCR only (FIXED /
// WRAP are served as INCR), AxSIZE <= 3 (narrow beats keep their lane strobes); AxSIZE > 3 is consumed and answered
// SLVERR without memory access.  One burst at a time; AW is served before AR when both are pending.
// Reads of a window that moves are ordered with writes by construction (a single outstanding operation).
module ng_rcp_bar2 #(
  parameter int AW = 31
) (
  input  logic          clk,
  input  logic          rst_n,
  // window
  input  logic          win_en,
  input  logic [4:0]    win_log2,
  input  logic [30:0]   win_base,
  // s_axi_bar2
  input  logic [3:0]    s_awid,
  input  logic [27:0]   s_awaddr,
  input  logic [7:0]    s_awlen,
  input  logic [2:0]    s_awsize,
  input  logic          s_awvalid,
  output logic          s_awready,
  input  logic [63:0]   s_wdata,
  input  logic [7:0]    s_wstrb,
  input  logic          s_wlast,
  input  logic          s_wvalid,
  output logic          s_wready,
  output logic [3:0]    s_bid,
  output logic [1:0]    s_bresp,
  output logic          s_bvalid,
  input  logic          s_bready,
  input  logic [3:0]    s_arid,
  input  logic [27:0]   s_araddr,
  input  logic [7:0]    s_arlen,
  input  logic [2:0]    s_arsize,
  input  logic          s_arvalid,
  output logic          s_arready,
  output logic [3:0]    s_rid,
  output logic [63:0]   s_rdata,
  output logic [1:0]    s_rresp,
  output logic          s_rlast,
  output logic          s_rvalid,
  input  logic          s_rready,
  // C3 engine requester 0
  output logic          op_valid,
  input  logic          op_ready,
  output logic          op_wr,
  output logic [AW-1:0] op_addr,
  output logic [7:0]    op_wstrb,
  output logic [63:0]   op_wdata,
  input  logic          rs_valid,
  input  logic          rs_err,
  input  logic [63:0]   rs_rdata,
  output logic          busy
);
  typedef enum logic [3:0] {S_IDLE, S_WGET, S_WOP, S_WRS, S_B, S_ROP, S_RRS, S_R} st_t;
  st_t st;

  logic        aw_pend, ar_pend;
  logic [3:0]  aw_id, ar_id;
  logic [27:0] aw_addr, ar_addr;
  logic [7:0]  aw_len, ar_len;
  logic [2:0]  aw_size, ar_size;
  logic [27:0] cur;          // address of the current beat
  logic [7:0]  left;         // beats still to do after the current one
  logic [2:0]  size;
  logic [3:0]  id;
  logic        err;          // sticky for the burst
  logic        is_wr;
  logic        b_hs_q, r_hs_q;

  assign busy = (st != S_IDLE);

  // window decode of the current beat
  wire [31:0] span  = 32'h1 << win_log2;
  wire        in_win = win_en && ((cur | 28'(span - 32'h1)) == 28'hFFF_FFFF);
  wire [30:0] c3off  = win_base | ({3'b000, cur} & (span[30:0] - 31'h1));
  wire        ok_sz  = (size <= 3'd3);

  function automatic logic [27:0] next_addr(input logic [27:0] a, input logic [2:0] sz);
    logic [27:0] m;
    m = (28'h1 << sz) - 28'h1;
    return (a & ~m) + (28'h1 << sz);
  endfunction

  assign op_wr   = is_wr;
  assign op_addr = {c3off[AW-1:3], 3'b000};

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; aw_pend <= 1'b0; ar_pend <= 1'b0; s_awready <= 1'b1; s_arready <= 1'b1; s_wready <= 1'b0;
      s_bvalid <= 1'b0; s_rvalid <= 1'b0; s_rlast <= 1'b0; s_bresp <= 2'b00; s_rresp <= 2'b00;
      op_valid <= 1'b0; err <= 1'b0; is_wr <= 1'b0; b_hs_q <= 1'b0; r_hs_q <= 1'b0;
      s_bid <= '0; s_rid <= '0; s_rdata <= '0;
      cur <= '0; left <= '0; size <= '0; id <= '0; op_wstrb <= '0; op_wdata <= '0;
    end else begin
      b_hs_q <= s_bvalid && s_bready;
      r_hs_q <= s_rvalid && s_rready;
      if (s_bvalid && s_bready) s_bvalid <= 1'b0;
      if (s_rvalid && s_rready) s_rvalid <= 1'b0;
      if (s_awvalid && s_awready) begin
        aw_pend <= 1'b1; aw_id <= s_awid; aw_addr <= s_awaddr; aw_len <= s_awlen; aw_size <= s_awsize;
        s_awready <= 1'b0;
      end
      if (s_arvalid && s_arready) begin
        ar_pend <= 1'b1; ar_id <= s_arid; ar_addr <= s_araddr; ar_len <= s_arlen; ar_size <= s_arsize;
        s_arready <= 1'b0;
      end

      unique case (st)
        S_IDLE: begin
          if (aw_pend) begin
            cur <= aw_addr; left <= aw_len; size <= aw_size; id <= aw_id; err <= 1'b0; is_wr <= 1'b1;
            s_wready <= 1'b1;
            st <= S_WGET;
          end else if (ar_pend) begin
            cur <= ar_addr; left <= ar_len; size <= ar_size; id <= ar_id; err <= 1'b0; is_wr <= 1'b0;
            st <= S_ROP;
          end
        end
        // ---- write burst: take a W beat, run it, repeat
        S_WGET: if (s_wvalid && s_wready) begin
          s_wready <= 1'b0;
          op_wstrb <= s_wstrb; op_wdata <= s_wdata;
          if (in_win && ok_sz) begin
            op_valid <= 1'b1;
            st <= S_WOP;
          end else begin
            err <= 1'b1;
            if (left == 8'd0) begin
              s_bvalid <= 1'b1; s_bid <= id; s_bresp <= 2'b10; st <= S_B;
            end else begin
              left <= left - 8'd1; cur <= next_addr(cur, size); s_wready <= 1'b1;
            end
          end
        end
        S_WOP: if (op_valid && op_ready) begin
          op_valid <= 1'b0;
          st <= S_WRS;
        end
        S_WRS: if (rs_valid) begin
          if (rs_err) err <= 1'b1;
          if (left == 8'd0) begin
            s_bvalid <= 1'b1; s_bid <= id; s_bresp <= (err || rs_err) ? 2'b10 : 2'b00; st <= S_B;
          end else begin
            left <= left - 8'd1; cur <= next_addr(cur, size); s_wready <= 1'b1; st <= S_WGET;
          end
        end
        S_B: if (b_hs_q) begin
          aw_pend <= 1'b0; s_awready <= 1'b1; st <= S_IDLE;
        end
        // ---- read burst: one operation per beat
        S_ROP: begin
          if (in_win && ok_sz) begin
            op_valid <= 1'b1;
            if (op_valid && op_ready) begin op_valid <= 1'b0; st <= S_RRS; end
          end else begin
            s_rvalid <= 1'b1; s_rid <= id; s_rdata <= 64'h0; s_rresp <= 2'b10; s_rlast <= (left == 8'd0);
            st <= S_R;
          end
        end
        S_RRS: if (rs_valid) begin
          s_rvalid <= 1'b1; s_rid <= id; s_rdata <= rs_rdata; s_rresp <= rs_err ? 2'b10 : 2'b00;
          s_rlast <= (left == 8'd0);
          st <= S_R;
        end
        default: if (r_hs_q) begin    // S_R
          if (left == 8'd0) begin
            ar_pend <= 1'b0; s_arready <= 1'b1; st <= S_IDLE;
          end else begin
            left <= left - 8'd1; cur <= next_addr(cur, size); st <= S_ROP;
          end
        end
      endcase
    end
  end
endmodule
