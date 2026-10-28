// ng_rmt_dma.sv -- host loop-back DMA engine of the test RM.
//
// Register word index (BAR0 0x8600 + 4*idx):
//   0 CTRL(W: b0 START, b1 CLEAR; RW: b2 IRQ_EN, b3 STOP_ON_ERR)
//   1 STATUS(b0 BUSY RO; b1 DONE, b2 CFG_ERR, b3 AXI_ERR W1C)
//   2 SRC_SEL 3 DST_SEL (0 host, 1..4 C1..C4)  4 SRC_LO 5 SRC_HI 6 DST_LO
//   7 DST_HI (15 bits: host IOVA[46:32]; channel uses bit 0)  8 LEN
//   9 CYCLES 10 AXI_ERRCNT (blocks with an error response) 11 BLOCKS_DONE
// One 512 B block at a time: read the source into a 64x64 buffer (host: one
// 64-beat AR; channel: one link read request), then write the destination
// (host: one 64-beat AW/W; channel: 64 link items), wait for the B, next block.
module ng_rmt_dma (
  input  logic          clk,
  input  logic          rst_n,
  // registers
  input  logic          rb_wr,
  input  logic [3:0]    rb_idx,
  input  logic [3:0]    rb_wstrb,
  input  logic [31:0]   rb_wdata,
  output logic [31:0]   rb_rdata,
  input  logic [1:0]    owner [4],
  output logic          busy,
  output logic [1:0]    ch,
  output logic          irq_pulse,
  // link requests to channel `ch`
  output logic [3:0]    dq_valid,
  input  logic [3:0]    dq_ready,
  output logic [106:0]  dq_data,
  // link responses (DW done / DR data), always accepted
  input  logic [3:0]    dw_valid,
  input  logic [3:0]    dw_err,
  input  logic [3:0]    dr_valid,
  input  logic [65:0]   dr_data [4],
  // host port (internal side of ng_rmt_port)
  output logic          h_awvalid,
  input  logic          h_awready,
  output logic [46:0]   h_awaddr,
  output logic          h_wvalid,
  input  logic          h_wready,
  output logic [63:0]   h_wdata,
  output logic          h_wlast,
  input  logic          h_bvalid,
  output logic          h_bready,
  input  logic [1:0]    h_bresp,
  output logic          h_arvalid,
  input  logic          h_arready,
  output logic [46:0]   h_araddr,
  input  logic          h_rvalid,
  output logic          h_rready,
  input  logic [63:0]   h_rdata,
  input  logic [1:0]    h_rresp
);
  localparam logic [1:0] K_DW = 2'd2, K_DR = 2'd3;

  function automatic logic [31:0] wmask(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
    logic [31:0] r;
    r = old;
    for (int i = 0; i < 4; i++) if (s[i]) r[8*i +: 8] = d[8*i +: 8];
    return r;
  endfunction

  logic        irq_en, stop_err, st_done, st_cfg, st_axi;
  logic [2:0]  src_sel, dst_sel;
  logic [31:0] src_lo, dst_lo, len, cycles, axierrcnt, blocks;
  logic [14:0] src_hi, dst_hi;

  typedef enum logic [2:0] {S_IDLE, S_RD, S_WR, S_BLK, S_FIN} st_t;
  st_t st;
  assign busy = (st != S_IDLE);

  logic        s_host;          // source is the host (else destination is)
  logic [46:0] s_cur, d_cur;
  logic [22:0] left;            // blocks left (LEN >> 9)
  logic        addr_sent, err;
  logic [6:0]  n_in, n_out;     // beats received into / sent from the buffer
  logic        b_wait;

  // Per-block watchdog.  A block is at most 64 beats on each side, so anything
  // past a few hundred thousand clk_l2 cycles means a handshake was lost.  The
  // first on-board run of this RM (build28, 2026-09-29) showed what happens
  // without it: the first host->channel transfer completed with an AXI error,
  // the next one sat in S_RD waiting for an R beat that never arrived, BUSY
  // stayed set, and CTRL=CLEAR could not clear it because CLEAR did not touch
  // the FSM.  The stuck engine also kept the port's outstanding counters
  // non-zero, so the RM could not be quiesced -- which blocked isolate and
  // therefore PR reconfiguration as well.  Abort the block instead of wedging.
  localparam logic [19:0] TO_LIMIT = 20'hFFFFF;
  logic [19:0] to_cnt;
  wire  abort = busy && (to_cnt == TO_LIMIT);

  logic [63:0] buf_mem [64];
  logic [63:0] buf_rd;
  assign buf_rd = buf_mem[n_out[5:0]];

  // ---------------------------------------------------------------- data movement
  wire rd_phase = (st == S_RD);
  wire wr_phase = (st == S_WR);

  assign h_arvalid = rd_phase && s_host && !addr_sent;
  assign h_araddr  = s_cur;
  assign h_rready  = 1'b1;
  wire h_r_hs = h_rvalid && rd_phase && s_host;

  assign h_awvalid = wr_phase && !s_host && !addr_sent;
  assign h_awaddr  = d_cur;
  assign h_wvalid  = wr_phase && !s_host && (n_out != 7'd64);
  assign h_wdata   = buf_rd;
  assign h_wlast   = (n_out == 7'd63);
  assign h_bready  = 1'b1;
  wire h_w_hs = h_wvalid && h_wready;

  // channel side
  wire c_rd_req = rd_phase && !s_host && !addr_sent;
  wire c_wr_req = wr_phase && s_host && (n_out != 7'd64);
  assign dq_valid = (c_rd_req || c_wr_req) ? (4'b0001 << ch) : 4'b0000;
  assign dq_data  = c_rd_req ? {K_DR, s_cur[32:0], 8'h00, 64'h0}
                             : {K_DW, d_cur[32:0], 8'hFF, buf_rd};
  wire c_hs = (c_rd_req || c_wr_req) && dq_ready[ch];
  wire c_r_in = dr_valid[ch] && rd_phase && !s_host;
  wire [65:0] c_r_d = dr_data[ch];

  wire in_beat = h_r_hs || c_r_in;
  wire [63:0] in_data = s_host ? h_rdata : c_r_d[63:0];
  wire [1:0]  in_resp = s_host ? h_rresp : c_r_d[65:64];

  always_ff @(posedge clk) if (in_beat) buf_mem[n_in[5:0]] <= in_data;

  // ---------------------------------------------------------------- registers + FSM
  wire rb_ctrl  = rb_wr && (rb_idx == 4'd0) && rb_wstrb[0];
  wire do_start = rb_ctrl && rb_wdata[0] && (st == S_IDLE);
  wire do_clear = rb_ctrl && rb_wdata[1];
  wire cfg_wr   = rb_wr && (st == S_IDLE);

  logic [2:0] chsel;
  logic       cfg_ok;
  always_comb begin
    chsel  = (src_sel == 3'd0) ? dst_sel : src_sel;
    cfg_ok = ((src_sel == 3'd0) != (dst_sel == 3'd0)) &&
             (chsel >= 3'd1) && (chsel <= 3'd4) &&
             (owner[2'(chsel - 3'd1)] == 2'd2) &&
             (len != 0) && (len[8:0] == 9'h0) &&
             (src_lo[8:0] == 9'h0) && (dst_lo[8:0] == 9'h0);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      irq_en <= 1'b0; stop_err <= 1'b0; st_done <= 1'b0; st_cfg <= 1'b0; st_axi <= 1'b0;
      src_sel <= '0; dst_sel <= '0; src_lo <= '0; dst_lo <= '0; src_hi <= '0; dst_hi <= '0;
      len <= '0; cycles <= '0; axierrcnt <= '0; blocks <= '0;
      st <= S_IDLE; irq_pulse <= 1'b0; ch <= '0; s_host <= 1'b0;
      left <= '0; addr_sent <= 1'b0; err <= 1'b0; n_in <= '0; n_out <= '0; b_wait <= 1'b0;
    end else begin
      irq_pulse <= 1'b0;
      if (busy) cycles <= cycles + 1'b1;
      // watchdog runs per block: cleared at S_BLK and whenever the engine
      // leaves the busy state
      if (!busy || do_clear || do_start || (st == S_BLK)) to_cnt <= '0;
      else                                               to_cnt <= (to_cnt == TO_LIMIT) ? '0 : (to_cnt + 1'b1);

      if (rb_ctrl) begin
        irq_en   <= rb_wdata[2];
        stop_err <= rb_wdata[3];
      end
      if (rb_wr && rb_idx == 4'd1 && rb_wstrb[0]) begin
        if (rb_wdata[1]) st_done <= 1'b0;
        if (rb_wdata[2]) st_cfg  <= 1'b0;
        if (rb_wdata[3]) st_axi  <= 1'b0;
      end
      if (cfg_wr) begin
        unique case (rb_idx)
          4'd2: if (rb_wstrb[0]) src_sel <= rb_wdata[2:0];
          4'd3: if (rb_wstrb[0]) dst_sel <= rb_wdata[2:0];
          4'd4: src_lo <= wmask(src_lo, rb_wdata, rb_wstrb);
          4'd5: src_hi <= 15'(wmask(32'(src_hi), rb_wdata, rb_wstrb));
          4'd6: dst_lo <= wmask(dst_lo, rb_wdata, rb_wstrb);
          4'd7: dst_hi <= 15'(wmask(32'(dst_hi), rb_wdata, rb_wstrb));
          4'd8: len    <= wmask(len, rb_wdata, rb_wstrb);
          default: ;
        endcase
      end

      // data counters
      if (in_beat) begin
        n_in <= n_in + 1'b1;
        if (in_resp != 2'b00) err <= 1'b1;
      end
      if (h_w_hs || (c_hs && c_wr_req)) n_out <= n_out + 1'b1;
      if ((h_arvalid && h_arready) || (h_awvalid && h_awready) || (c_hs && c_rd_req)) addr_sent <= 1'b1;
      if (h_bvalid && b_wait) begin
        b_wait <= 1'b0;
        if (h_bresp != 2'b00) err <= 1'b1;
      end
      if (dw_valid[ch] && b_wait) begin
        b_wait <= 1'b0;
        if (dw_err[ch]) err <= 1'b1;
      end

      unique case (st)
        S_IDLE: if (do_start) begin
          if (!cfg_ok) begin
            st_cfg <= 1'b1;
          end else begin
            st        <= S_RD;
            st_done   <= 1'b0;
            cycles    <= '0;
            blocks    <= '0;
            ch        <= 2'(chsel - 3'd1);
            s_host    <= (src_sel == 3'd0);
            s_cur     <= {src_hi, src_lo};
            d_cur     <= {dst_hi, dst_lo};
            left      <= len[31:9];
            addr_sent <= 1'b0;
            err       <= 1'b0;
            n_in      <= '0;
            n_out     <= '0;
          end
        end
        S_RD: if (abort) begin
          st        <= S_FIN;
          st_axi    <= 1'b1;
          axierrcnt <= axierrcnt + 1'b1;
        end else if (n_in == 7'd64) begin
          st        <= S_WR;
          addr_sent <= 1'b0;
          b_wait    <= 1'b1;
        end
        S_WR: if (abort) begin
          st        <= S_FIN;
          st_axi    <= 1'b1;
          axierrcnt <= axierrcnt + 1'b1;
        end else if (n_out == 7'd64 && !b_wait && (addr_sent || s_host)) begin
          st <= S_BLK;
        end
        S_BLK: begin
          blocks    <= blocks + 1'b1;
          if (err) begin
            st_axi    <= 1'b1;
            axierrcnt <= axierrcnt + 1'b1;
          end
          left      <= left - 1'b1;
          s_cur     <= s_cur + 47'd512;
          d_cur     <= d_cur + 47'd512;
          addr_sent <= 1'b0;
          err       <= 1'b0;
          n_in      <= '0;
          n_out     <= '0;
          st        <= ((left == 23'd1) || (err && stop_err)) ? S_FIN : S_RD;
        end
        default: begin
          st        <= S_IDLE;
          st_done   <= 1'b1;
          irq_pulse <= irq_en;
        end
      endcase

      if (do_clear) begin
        st_done   <= 1'b0;
        cycles    <= '0;
        axierrcnt <= '0;
        blocks    <= '0;
        // CLEAR must also abort a block in flight: without this a lost
        // handshake leaves the FSM parked in S_RD/S_WR and BUSY set for ever
        // (see the watchdog note above).
        st        <= S_IDLE;
        err       <= 1'b0;
        addr_sent <= 1'b0;
        b_wait    <= 1'b0;
        n_in      <= '0;
        n_out     <= '0;
        left      <= '0;
      end
    end
  end

  always_comb begin
    unique case (rb_idx)
      4'd0:  rb_rdata = {28'h0, stop_err, irq_en, 2'b00};
      4'd1:  rb_rdata = {28'h0, st_axi, st_cfg, st_done, busy};
      4'd2:  rb_rdata = 32'(src_sel);
      4'd3:  rb_rdata = 32'(dst_sel);
      4'd4:  rb_rdata = src_lo;
      4'd5:  rb_rdata = 32'(src_hi);
      4'd6:  rb_rdata = dst_lo;
      4'd7:  rb_rdata = 32'(dst_hi);
      4'd8:  rb_rdata = len;
      4'd9:  rb_rdata = cycles;
      4'd10: rb_rdata = axierrcnt;
      4'd11: rb_rdata = blocks;
      default: rb_rdata = 32'h0;
    endcase
  end
endmodule
