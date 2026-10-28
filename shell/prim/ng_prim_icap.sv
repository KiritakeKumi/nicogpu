// ng_prim_icap.sv -- configuration-port wrapper.
//
// Interface: one 32-bit configuration word per valid&ready, in bitstream word
// order (the value of each big-endian word of the .bin file).  prdone/prerror
// report the result of the last partial load.
// FPGA mapping (P0-11): ICAPE3 of the master SLR (SLR1, the IS_MASTER die
// whose ICAP programs all three SLRs -- Vivado places it there automatically
// on SSI parts).  CSIB is driven from valid&&enable; the clock is clk_ts at
// 99.9 MHz (ICAPE3/CLK minimum period 4.875 ns in the timing report, so not
// at the limit).  PRDONE/PRERROR are latched to levels that
// hold until the port is disabled, so the 2-flop synchronizer downstream
// cannot miss them (a bare pulse would be lost, which is why the 2026-09-27
// board run hung in PR_STATUS.BUSY with data flowing into nowhere).
//
// PR diagnostics (PR_OPT knobs, all-zero = behaviour of the P0-11 shell):
//   order[1:0]  pin-side word format applied to the BE file word w:
//                 0 = bitrev32(w)               (R1, the P0-11 choice; board
//                     2026-10-14: the engine never syncs with it)
//                 1 = per-byte bit reversal, byte order kept (R0, "bitswapped";
//                     board 2026-10-14: O 9B -> DB, the engine syncs)
//                 2 = byte reversal only, no bit reversal
//                 3 = identity
//   csib_cont   keep CSIB low through the PR window: cycles without a FIFO
//               word feed a NOP (BE 0x20000000, transformed like data) --
//               but only between packets (see the packet tracker; inside a
//               write packet CSIB is released as before, a NOP there would be
//               its payload).  csib_force: NOP in every idle cycle, even
//               inside a packet (gapless streams only; corrupts otherwise)
//   O capture   ICAPE3.O is registered and logged (clk domain): last non-zero
//               word + valid sticky, the last 4 *distinct consecutive* words
//               (change log), a saturating change counter, and  the
//               number of data words strobed into the ICAP.  Cleared when
//               enable rises.  A snapshot handshake (toggle req/ack) copies
//               them tear-free to the clk_sh side (obs, 201 bits).  The decode
//               of O (e.g. O[7:4] sync state per UG570) is NOT done here: raw
//               words only, to be confirmed on the board.
//   prdone_raw / prerror_raw / avail_raw   registered raw ICAPE3 outputs (live).
//
// additions (static build b18):
//   avail_lost   sticky since enable rose: a data word sat on the pins while
//                AVAIL was low (the ICAP may not have taken it)
//   prdone_fell  sticky since enable rose: raw PRDONE was seen low (PRDONE is
//                1 when idle -- board 2026-10-13 E0 -- so only a fall and a
//                rise mean the engine finished something)
//   readback     a self-contained sequencer (rb_req toggle; parameters
//   (RB)         quasi-static): writes  FFFFFFFF, AA995566, NOP,
//                type-1 read header (0x28000000 | reg<<13 | count), 2+rb_nop
//                NOPs; CSIB high; RDWRB=1; CSIB low for rb_skip+8 read cycles
//                capturing ICAPE3.O of read cycles rb_skip..rb_skip+7 into
//                rb_data (raw pin words: decode with the same ORDER transform);
//                CSIB high; RDWRB=0; then DESYNC (30008001 0000000D) + 32 NOPs.
//                RDWRB only changes while CSIB is high (one idle cycle on each
//                side).  All words go through the current ORDER transform.  It
//                runs with or without a PR session; the caller (ng_shell_ctrl)
//                only starts it when the PR FIFO is empty, and the FIFO is not
//                popped while it runs (rb_busy).  rb_data is stable from the
//                rb_ack toggle until the next request.
//
// The pin pipeline (CSIB, RDWRB, I) is registered once for timing: the
// primitive sees the word one clk after valid.
//
// Behavioral model (simulation): it decodes the pin word with the *model's*
// expected order MODEL_ORDER (parameter, the order "the silicon" wants), flags
// an error if a word arrives while !enable, tracks "synced" after the sync
// word 0xAA995566 (cleared by DESYNC / idle), and reports prdone when it sees
// the DESYNC command word (0x0000000D right after 0x30008001) during a PR
// session.  Reads: a type-1 read header while synced arms N words; from read
// cycle MODEL_RD_LAT on, O shows xf(MODEL_ORDER, value).  ALL model values
// below (O status words, read latency, register contents, PRDONE idle level,
// AVAIL) are INVENTED, not from silicon.
module ng_prim_icap #(
  parameter logic [1:0] MODEL_ORDER  = 2'd0,  // simulation only: order the model's "silicon" expects
  parameter int         MODEL_RD_LAT = 3      // simulation only: read cycles before the first data word
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         enable,
  input  logic         valid,
  output logic         ready,
  input  logic [31:0]  data,
  // diagnostics knobs (already synchronized into clk, quasi-static)
  input  logic [1:0]   order,
  input  logic         csib_cont,
  input  logic         csib_force,
  output logic         prdone,
  output logic         prerror,
  output logic         prdone_raw,
  output logic         prerror_raw,
  output logic         avail_raw,
  output logic         avail_lost,
  output logic         prdone_fell,
  // O observation snapshot (toggle handshake, see header)
  input  logic         snap_req,     // already synchronized into clk
  output logic         snap_ack,     // toggle, clk domain
  output logic [200:0] obs,          // {icap_words, chg[7:0], vld, h3, h2, h1, h0, nz}, stable between req and next req
  // live values for the JTAG black box (may tear; debug only)
  output logic [31:0]  icap_words,
  output logic [31:0]  o_live,
  // readback sequencer (see header)
  input  logic         rb_req,       // toggle, already synchronized into clk
  input  logic [4:0]   rb_reg,       // quasi-static while a request is pending
  input  logic [7:0]   rb_cnt,       // read header word count (0 = 1)
  input  logic [5:0]   rb_skip,
  input  logic [3:0]   rb_nop,
  output logic         rb_ack,       // toggle, clk domain
  output logic         rb_busy,      // level, clk domain: the sequencer owns the pins
  output logic [255:0] rb_data       // {d7, .., d0}
);
  localparam logic [31:0] NOP_BE = 32'h2000_0000;

  function automatic logic [31:0] xf(input logic [1:0] ord, input logic [31:0] w);
    logic [31:0] r;
    case (ord)
      2'd0: for (int i = 0; i < 32; i++) r[i] = w[31 - i];
      2'd1: for (int b = 0; b < 4; b++)
              for (int i = 0; i < 8; i++) r[8*b + i] = w[8*b + 7 - i];
      2'd2: r = {w[7:0], w[15:8], w[23:16], w[31:24]};
      default: r = w;
    endcase
    return r;
  endfunction

  assign ready = 1'b1;

  // ---- packet tracker (BE words): after the sync word, pk_rem = payload words
  // still owed by the current write packet (type-1 count[10:0] / type-2
  // count[26:0], opcode write).  A NOP injected inside a packet would be
  // taken as its payload and corrupt the load, so csib_cont only fills idle
  // cycles with NOP between packets; csib_force fills everywhere (only safe
  // for a stream delivered without gaps).
  logic        pk_sync;
  logic [26:0] pk_rem;
  always_ff @(posedge clk) begin
    if (!rst_n || !enable) begin
      pk_sync <= 1'b0;
      pk_rem  <= '0;
    end else if (valid) begin
      if (!pk_sync) begin
        if (data == 32'hAA99_5566) pk_sync <= 1'b1;
      end else if (pk_rem != '0) begin
        pk_rem <= pk_rem - 27'd1;
      end else if (data[28:27] == 2'b10) begin
        if (data[31:29] == 3'b001)      pk_rem <= {16'h0, data[10:0]};
        else if (data[31:29] == 3'b010) pk_rem <= data[26:0];
      end
    end
  end

  // ---- readback sequencer state (drives the pins while rb_st != RB_IDLE)
  typedef enum logic [3:0] {
    RB_IDLE, RB_WR, RB_SW_R0, RB_SW_R1, RB_RD, RB_SW_W0, RB_SW_W1, RB_TAIL, RB_DONE
  } rb_st_t;
  localparam int RB_TAIL_N = 34;          // DESYNC (2 words) + 32 NOPs
  rb_st_t      rb_st;
  logic [5:0]  rb_i;                      // word index inside RB_WR / RB_TAIL
  logic [6:0]  rb_rc;                     // read cycles issued
  logic [4:0]  rb_reg_l;
  logic [7:0]  rb_cnt_l;
  logic [5:0]  rb_skip_l;
  logic [3:0]  rb_nop_l;
  logic        rb_drive;
  logic        rb_csib_n, rb_rdwrb_n;
  logic [31:0] rb_word_n;
  logic [5:0]  rb_wr_last;
  assign rb_drive   = (rb_st != RB_IDLE);
  assign rb_busy    = rb_drive;
  assign rb_wr_last = 6'd5 + {2'b00, rb_nop_l};   // FFFFFFFF, SYNC, NOP, header, (2 + rb_nop) NOPs

  always_comb begin
    rb_csib_n  = 1'b1;
    rb_rdwrb_n = 1'b0;
    rb_word_n  = NOP_BE;
    case (rb_st)
      RB_WR: begin
        rb_csib_n = 1'b0;
        case (rb_i)
          6'd0:    rb_word_n = 32'hFFFF_FFFF;
          6'd1:    rb_word_n = 32'hAA99_5566;
          6'd3:    rb_word_n = {3'b001, 2'b01, 9'h0, rb_reg_l, 2'b00, 3'b000, (rb_cnt_l == 8'd0) ? 8'd1 : rb_cnt_l};
          default: rb_word_n = NOP_BE;
        endcase
      end
      RB_SW_R1, RB_SW_W0: rb_rdwrb_n = 1'b1;
      RB_RD: begin rb_csib_n = 1'b0; rb_rdwrb_n = 1'b1; end
      RB_TAIL: begin
        rb_csib_n = 1'b0;
        case (rb_i)
          6'd0:    rb_word_n = 32'h3000_8001;
          6'd1:    rb_word_n = 32'h0000_000D;
          default: rb_word_n = NOP_BE;
        endcase
      end
      default: ;
    endcase
  end

  // ---- registered pin pipeline
  logic        csib_q, rdwrb_q, dstb_q, rbpin_q;
  logic [31:0] i_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      csib_q  <= 1'b1;
      rdwrb_q <= 1'b0;
      dstb_q  <= 1'b0;
      rbpin_q <= 1'b0;
      i_q     <= '0;
    end else if (rb_drive) begin
      csib_q  <= rb_csib_n;
      rdwrb_q <= rb_rdwrb_n;
      dstb_q  <= 1'b0;
      rbpin_q <= 1'b1;
      i_q     <= xf(order, rb_word_n);
    end else begin
      csib_q  <= ~(enable && (valid || csib_force || (csib_cont && pk_rem == '0)));
      rdwrb_q <= 1'b0;
      dstb_q  <= enable && valid;
      rbpin_q <= 1'b0;
      i_q     <= xf(order, valid ? data : NOP_BE);
    end
  end

  logic        prdone_p, prerror_p, avail_p;
  logic [31:0] o_pin;

  // ---- ICAPE3 O observation (common to FPGA and simulation)
  logic [31:0] o_q, o_nz, h0, h1, h2, h3, wcnt;
  logic [7:0]  chg;
  logic        o_vld, en_q;
  logic [200:0] live;
  assign live       = {wcnt, chg, o_vld, h3, h2, h1, h0, o_nz};
  assign icap_words = wcnt;
  assign o_live     = o_q;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      o_q <= '0; o_nz <= '0; h0 <= '0; h1 <= '0; h2 <= '0; h3 <= '0; wcnt <= '0;
      chg <= '0; o_vld <= 1'b0; en_q <= 1'b0;
      avail_raw <= 1'b0; avail_lost <= 1'b0; prdone_fell <= 1'b0;
    end else begin
      en_q      <= enable;
      o_q       <= o_pin;
      avail_raw <= avail_p;
      if (enable && !en_q) begin
        o_nz <= '0; h0 <= '0; h1 <= '0; h2 <= '0; h3 <= '0; chg <= '0; o_vld <= 1'b0;
        wcnt <= {31'h0, valid && !rb_drive};               // a word may already pop in the first enabled cycle
        avail_lost <= 1'b0; prdone_fell <= 1'b0;
      end else if (enable) begin
        if (o_q != 32'h0) begin o_nz <= o_q; o_vld <= 1'b1; end
        if (o_q != h0) begin
          h0 <= o_q; h1 <= h0; h2 <= h1; h3 <= h2;
          if (chg != 8'hFF) chg <= chg + 8'd1;
        end
        if (valid && !rb_drive) wcnt <= wcnt + 32'd1;     // popped now, strobed next cycle
        if (dstb_q && !avail_p) avail_lost <= 1'b1;
        if (en_q && !prdone_raw) prdone_fell <= 1'b1;
      end
    end
  end

  // ---- snapshot: copy live -> obs when the request toggle differs from ack
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      snap_ack <= 1'b0;
      obs      <= '0;
    end else if (snap_req != snap_ack) begin
      obs      <= live;
      snap_ack <= snap_req;
    end
  end

  // ---- readback sequencer + capture.  The capture is aligned with o_q:
  // rbrd_q marks "o_q holds O of a read cycle" (rdwrb_q && !csib_q one clk
  // earlier); rb_k counts those cycles; words rb_skip..rb_skip+7 are kept.
  logic       rbrd_q;
  logic [6:0] rb_k;
  logic [6:0] rb_rd_last;
  assign rb_rd_last = {1'b0, rb_skip_l} + 7'd7;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rb_st <= RB_IDLE; rb_i <= '0; rb_rc <= '0; rb_ack <= 1'b0;
      rb_reg_l <= '0; rb_cnt_l <= '0; rb_skip_l <= '0; rb_nop_l <= '0;
      rbrd_q <= 1'b0; rb_k <= '0; rb_data <= '0;
    end else begin
      rbrd_q <= rbpin_q && rdwrb_q && !csib_q;
      if (rbrd_q) begin
        if (rb_k >= {1'b0, rb_skip_l} && rb_k <= rb_rd_last)
          rb_data[32*(rb_k - {1'b0, rb_skip_l}) +: 32] <= o_q;
        rb_k <= rb_k + 7'd1;
      end
      case (rb_st)
        RB_IDLE: if (rb_req != rb_ack) begin
          rb_reg_l <= rb_reg; rb_cnt_l <= rb_cnt; rb_skip_l <= rb_skip; rb_nop_l <= rb_nop;
          rb_i <= '0; rb_k <= '0; rb_data <= '0;
          rb_st <= RB_WR;
        end
        RB_WR: begin
          rb_i <= rb_i + 6'd1;
          if (rb_i == rb_wr_last) rb_st <= RB_SW_R0;
        end
        RB_SW_R0: rb_st <= RB_SW_R1;
        RB_SW_R1: begin rb_rc <= '0; rb_st <= RB_RD; end
        RB_RD: begin
          rb_rc <= rb_rc + 7'd1;
          if (rb_rc == rb_rd_last) rb_st <= RB_SW_W0;
        end
        RB_SW_W0: rb_st <= RB_SW_W1;
        RB_SW_W1: begin rb_i <= '0; rb_st <= RB_TAIL; end
        RB_TAIL: begin
          rb_i <= rb_i + 6'd1;
          if (rb_i == 6'(RB_TAIL_N - 1)) rb_st <= RB_DONE;
        end
        RB_DONE: begin rb_ack <= ~rb_ack; rb_st <= RB_IDLE; end
        default: rb_st <= RB_IDLE;
      endcase
    end
  end

`ifdef SYNTHESIS
  ICAPE3 u_icape3 (
    .CLK(clk), .CSIB(csib_q), .RDWRB(rdwrb_q),
    .I(i_q), .O(o_pin), .AVAIL(avail_p),
    .PRDONE(prdone_p), .PRERROR(prerror_p)
  );
  always_ff @(posedge clk) begin
    prdone_raw  <= prdone_p;
    prerror_raw <= prerror_p;
  end
  // latch the one-cycle PRDONE/PRERROR pulses into levels; prdone clears
  // when the port is disabled (next START), prerror holds until reset --
  // both match the behavioral model so ng_shell_ctrl sees identical timing.
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      prdone  <= 1'b0;
      prerror <= 1'b0;
    end else if (!enable) begin
      prdone  <= 1'b0;
      if (valid) prerror <= 1'b1;   // word dropped while disabled = error (as the model)
    end else begin
      if (prdone_p)  prdone  <= 1'b1;
      if (prerror_p) prerror <= 1'b1;
    end
  end
`else
  localparam logic [31:0] DESYNC    = 32'h0000_000D;
  localparam logic [31:0] SYNCW     = 32'hAA99_5566;
  localparam logic [31:0] O_UNSYNC  = 32'h0000_009B;   // INVENTED model value
  localparam logic [31:0] O_SYNCED  = 32'h0000_00DB;   // INVENTED model value
  logic model_avail = 1'b1;                            // INVENTED: testbenches may force it low
  logic prev_cmd;      // previous word was "type-1 write CMD register"
  logic synced;
  logic [4:0]  rd_reg;
  logic [10:0] rd_left;
  logic [6:0]  rd_cyc;
  logic [31:0] be;     // pin word decoded with the model's expected order (all xf are involutions)
  logic        rd_now;
  function automatic logic [31:0] model_reg(input logic [4:0] a);
    case (a)
      5'h07:   return 32'h1090_79FC;   // STAT  (INVENTED for the model; same pattern as the board's JTAG STAT)
      5'h0C:   return 32'h04B3_5093;   // IDCODE
      5'h16:   return 32'h0000_0001;   // BOOTSTS
      default: return {16'hA5A5, 11'h0, a};
    endcase
  endfunction
  assign be     = xf(MODEL_ORDER, i_q);
  assign rd_now = !csib_q && rdwrb_q && rd_cyc >= 7'(MODEL_RD_LAT) && rd_left != '0;
  assign o_pin  = rd_now ? xf(MODEL_ORDER, model_reg(rd_reg)) : (synced ? O_SYNCED : O_UNSYNC);
  assign avail_p     = model_avail;
  assign prerror_raw = 1'b0;
  assign prdone_raw  = !synced;    // INVENTED: idle level 1, low while a session runs

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      prdone <= 1'b0; prerror <= 1'b0; prev_cmd <= 1'b0; synced <= 1'b0;
      rd_reg <= '0; rd_left <= '0; rd_cyc <= '0;
    end else begin
      if (!enable) begin
        prdone <= 1'b0;
        if (valid) prerror <= 1'b1;
      end
      // read cycles: count consecutive ones, consume armed words
      if (!csib_q && rdwrb_q) begin
        rd_cyc <= rd_cyc + 7'd1;
        if (rd_now) rd_left <= rd_left - 11'd1;
      end else begin
        rd_cyc <= '0;
      end
      if (!enable && !rbpin_q) begin
        prev_cmd <= 1'b0;
        synced   <= 1'b0;
        rd_left  <= '0;
      end else if (!csib_q && !rdwrb_q) begin
        prev_cmd <= (be == 32'h3000_8001);
        if (be == SYNCW) synced <= 1'b1;
        if (prev_cmd && be == DESYNC) begin
          if (enable) prdone <= 1'b1;
          synced  <= 1'b0;
          rd_left <= '0;
        end
        if (synced && be[31:27] == 5'b00101) begin          // type-1 read header
          rd_reg  <= be[17:13];
          rd_left <= be[10:0];
        end
      end
    end
  end
`endif
endmodule
