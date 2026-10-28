// ng_axi64_to_lite32.sv -- AXI4 (64-bit) subordinate to AXI4-Lite (32-bit)
// manager, burst-decomposing, single outstanding transaction (P0-11).
//
// Sits between the BAR router (clk_sh side, XDMA m_axib traffic) and the
// 32-bit register slaves: the RM control port s_axil_ctrl and the shell
// register block BAR1.  Host MMIO is single 32-bit accesses, but the XDMA
// bridge may present posted writes as short bursts; this module absorbs any
// INCR burst and decomposes it into single-beat Lite accesses:
//   * a write beat with both strobe halves set becomes two Lite writes
//     (low word then high word); narrow beats (one half strobed) become one
//   * a read burst becomes one Lite read per beat, zero-extended into the
//     64-bit lane position selected by the beat address (AXI narrow rule)
//   * the start address may be unaligned: the XDMA bridge presents a TLP's
//     dword address (e.g. 0x..C for a 2-DW write crossing an 8-byte boundary).
//     Beats after the first are aligned to the transfer size (AXI A3.4.1), and
//     the strobes alone pick the lanes: lane 0 is the dword at the 8-byte-
//     aligned beat base, lane 1 the one above it.  (P0-12 board finding
//     2026-09-29: stepping the unaligned start by +8 sent beat 1 of every such
//     write 4 bytes too high.)
// One AW/AR is processed at a time; the clock converter in front absorbs
// PCIe-side flow control.  Errors: any Lite error response makes the whole
// transaction respond SLVERR; WLAST not matching AWLEN also gives SLVERR.
module ng_axi64_to_lite32 #(
  parameter int AW_W = 20   // address width, both sides, word aligned
) (
  input  logic clk,
  input  logic rst_n,
  // subordinate: AXI4 64-bit full
  input  logic [3:0]      s_awid,
  input  logic [AW_W-1:0] s_awaddr,
  input  logic [7:0]      s_awlen,
  input  logic [2:0]      s_awsize,
  input  logic            s_awvalid,
  output logic            s_awready,
  input  logic [63:0]     s_wdata,
  input  logic [7:0]      s_wstrb,
  input  logic            s_wlast,
  input  logic            s_wvalid,
  output logic            s_wready,
  output logic [3:0]      s_bid,
  output logic [1:0]      s_bresp,
  output logic            s_bvalid,
  input  logic            s_bready,
  input  logic [3:0]      s_arid,
  input  logic [AW_W-1:0] s_araddr,
  input  logic [7:0]      s_arlen,
  input  logic [2:0]      s_arsize,
  input  logic            s_arvalid,
  output logic            s_arready,
  output logic [3:0]      s_rid,
  output logic [63:0]     s_rdata,
  output logic [1:0]      s_rresp,
  output logic            s_rlast,
  output logic            s_rvalid,
  input  logic            s_rready,
  // manager: AXI4-Lite 32-bit
  output logic [AW_W-1:0] m_awaddr,
  output logic [2:0]      m_awprot,
  output logic            m_awvalid,
  input  logic            m_awready,
  output logic [31:0]     m_wdata,
  output logic [3:0]      m_wstrb,
  output logic            m_wvalid,
  input  logic            m_wready,
  input  logic [1:0]      m_bresp,
  input  logic            m_bvalid,
  output logic            m_bready,
  output logic [AW_W-1:0] m_araddr,
  output logic [2:0]      m_arprot,
  output logic            m_arvalid,
  input  logic            m_arready,
  input  logic [31:0]     m_rdata,
  input  logic [1:0]      m_rresp,
  input  logic            m_rvalid,
  output logic            m_rready
);

  // ------------------------------------------------------------------ write --
  // IDLE -> BEAT (accept a W beat) -> ISSUE_LO/SKIP -> WAIT_LO -> ISSUE_HI ->
  // WAIT_HI -> END (B response or next beat)
  enum logic [2:0] { W_IDLE, W_BEAT, W_ISSUE_LO, W_WAIT_LO, W_ISSUE_HI, W_WAIT_HI, W_END } wst;
  logic [3:0]      w_id;
  logic [AW_W-1:0] w_addr;
  logic [7:0]      w_len;
  logic [2:0]      w_size;
  logic [7:0]      w_beat;
  logic            w_err;
  logic            w_last_seen;
  logic            hi_pend;
  logic            aw_done, w_done;   // AW / W already accepted this issue
  logic            issue, aw_hs, w_hs;
  logic [31:0]     lo_data,  hi_data;
  logic [3:0]      lo_strb,  hi_strb;
  logic [AW_W-1:0] lo_addr,  hi_addr;

  // next beat address: align to the transfer size, then step (AXI INCR)
  function automatic logic [AW_W-1:0] next_beat(input logic [AW_W-1:0] a, input logic [2:0] size);
    logic [AW_W-1:0] step;
    step = AW_W'(1) << size;
    return (a & ~(step - AW_W'(1))) + step;
  endfunction

  assign s_awready = (wst == W_IDLE);
  assign s_wready  = (wst == W_BEAT);
  assign m_bready  = 1'b1;
  assign m_awprot  = 3'b010;

  // AW and W are independent channels: a lite slave may take one before the
  // other, so each valid drops once its own handshake has happened
  assign issue     = (wst == W_ISSUE_LO) || (wst == W_ISSUE_HI);
  assign m_awvalid = issue && !aw_done;
  assign m_awaddr  = (wst == W_ISSUE_HI) ? hi_addr : lo_addr;
  assign m_wvalid  = issue && !w_done;
  assign aw_hs     = m_awvalid && m_awready;
  assign w_hs      = m_wvalid && m_wready;
  assign m_wdata   = (wst == W_ISSUE_HI) ? hi_data : lo_data;
  assign m_wstrb   = (wst == W_ISSUE_HI) ? hi_strb : lo_strb;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wst <= W_IDLE; w_beat <= 8'd0; w_err <= 1'b0; w_last_seen <= 1'b0;
      hi_pend <= 1'b0; s_bvalid <= 1'b0;
      aw_done <= 1'b0; w_done <= 1'b0;
      // payload registers too: an async-reset block that leaves some of its
      // registers unreset synthesizes them with reset as a hold enable
      w_id <= '0; w_addr <= '0; w_len <= 8'd0; w_size <= 3'd0;
      lo_data <= 32'd0; lo_strb <= 4'd0; lo_addr <= '0;
      hi_data <= 32'd0; hi_strb <= 4'd0; hi_addr <= '0;
      s_bid <= '0; s_bresp <= 2'b00;
    end else begin
      if (s_bvalid && s_bready) s_bvalid <= 1'b0;
      if (aw_hs) aw_done <= 1'b1;
      if (w_hs)  w_done  <= 1'b1;
      if (m_bvalid && m_bready && m_bresp != 2'b00) w_err <= 1'b1;
      unique case (wst)
        W_IDLE: begin
          w_beat <= 8'd0; w_err <= 1'b0; w_last_seen <= 1'b0;
          if (s_awvalid && s_awready) begin
            w_id   <= s_awid;
            w_addr <= s_awaddr;
            w_len  <= s_awlen;
            w_size <= s_awsize;
            wst    <= W_BEAT;
          end
        end
        W_BEAT: begin
          if (s_wvalid && s_wready) begin
            // lanes come from the strobes; addresses from the beat's 8-byte
            // base (lite slaves decode [AW_W-1:2], so byte offsets never mattered)
            lo_data  <= s_wdata[31:0];
            lo_strb  <= s_wstrb[3:0];
            lo_addr  <= {w_addr[AW_W-1:3], 3'b000};
            hi_pend  <= |s_wstrb[7:4];
            hi_data  <= s_wdata[63:32];
            hi_strb  <= s_wstrb[7:4];
            hi_addr  <= {w_addr[AW_W-1:3], 3'b100};
            w_beat   <= w_beat + 8'd1;
            w_addr   <= next_beat(w_addr, w_size);
            if (s_wlast) begin
              w_last_seen <= 1'b1;
              if (w_beat != w_len) w_err <= 1'b1;  // short or early WLAST
            end
            wst <= |s_wstrb[3:0] ? W_ISSUE_LO : (|s_wstrb[7:4] ? W_ISSUE_HI : W_END);
          end
        end
        W_ISSUE_LO: if ((aw_done || aw_hs) && (w_done || w_hs)) begin
          aw_done <= 1'b0; w_done <= 1'b0;
          wst     <= W_WAIT_LO;
        end
        W_WAIT_LO: if (m_bvalid && m_bready) begin
          wst <= hi_pend ? W_ISSUE_HI : W_END;
        end
        W_ISSUE_HI: if ((aw_done || aw_hs) && (w_done || w_hs)) begin
          hi_pend <= 1'b0;
          aw_done <= 1'b0; w_done <= 1'b0;
          wst     <= W_WAIT_HI;
        end
        W_WAIT_HI: if (m_bvalid && m_bready) begin
          wst <= W_END;
        end
        W_END: begin
          if (w_last_seen) begin
            s_bvalid <= 1'b1;
            s_bid    <= w_id;
            s_bresp  <= w_err ? 2'b10 : 2'b00;
            wst      <= W_IDLE;
          end else begin
            wst <= W_BEAT;
          end
        end
        default: ;
      endcase
    end
  end

  // ------------------------------------------------------------------- read --
  enum logic [1:0] { R_IDLE, R_ISSUE, R_DATA } rst_;
  logic [3:0]      r_id;
  logic [AW_W-1:0] r_addr;
  logic [7:0]      r_len;
  logic [2:0]      r_size;
  logic [7:0]      r_beat;

  assign s_arready = (rst_ == R_IDLE);
  assign m_arvalid = (rst_ == R_ISSUE);
  assign m_araddr  = r_addr;
  assign m_arprot  = 3'b010;
  assign m_rready  = (rst_ == R_DATA);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rst_ <= R_IDLE; r_beat <= 8'd0; s_rvalid <= 1'b0; s_rlast <= 1'b0;
      r_id <= '0; r_addr <= '0; r_len <= 8'd0; r_size <= 3'd0;
      s_rid <= '0; s_rdata <= 64'd0; s_rresp <= 2'b00;
    end else begin
      if (s_rvalid && s_rready) s_rvalid <= 1'b0;
      unique case (rst_)
        R_IDLE: begin
          r_beat <= 8'd0;
          if (s_arvalid && s_arready) begin
            r_id   <= s_arid;
            r_addr <= s_araddr;
            r_len  <= s_arlen;
            r_size <= s_arsize;
            rst_   <= R_ISSUE;
          end
        end
        R_ISSUE: if (m_arvalid && m_arready) rst_ <= R_DATA;
        R_DATA: begin
          if (m_rvalid && m_rready) begin
            s_rvalid <= 1'b1;
            s_rid    <= r_id;
            s_rdata  <= r_addr[2] ? {m_rdata, 32'h0} : {32'h0, m_rdata};
            s_rresp  <= m_rresp;
            s_rlast  <= (r_beat == r_len);
            if (r_beat == r_len) begin
              rst_ <= R_IDLE;
            end else begin
              r_beat <= r_beat + 8'd1;
              r_addr <= next_beat(r_addr, r_size);
              rst_   <= R_ISSUE;
            end
          end
        end
        default: ;
      endcase
    end
  end

endmodule
