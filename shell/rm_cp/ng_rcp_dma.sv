// ng_rcp_dma.sv -- bulk Copy engine on clk_l2: one COPY / FILL command from the CP (one clock-domain
// handshake) becomes AXI bursts on the host (64-bit) and C3 (128-bit) masters
//
//   reader   plans the source word stream [src>>3, (src+size-1)>>3] in bursts that stay inside one 4 KiB page
//            (host <= 64 beats, C3 <= 16 beats); a burst is issued only when the FIFO can take all its words
//            (space reserved at issue), so R beats are never back-pressured by the FIFO.  Responses are taken
//            strictly in plan order (`ri_*` order FIFO): only the port of the head entry gets rready.  A C3 beat
//            yields up to two words (two cycles); a sparse page pushes zero words without an access.
//   FIFO     2^FIFO_LOG2 x 64 bit (BRAM), first-word-fall-through.
//   align    output word j covers dst bytes [(dst & ~7) + 8j, +8); its source bytes start at S = src - (dst & 7) + 8j,
//            so out_j = {w[q+1], w[q]} >> 8 * (S & 7) with q = S >> 3 (words outside the stream read as 0).  The
//            first / last output words carry partial strobes.  FILL: out_j = {fill, fill}.
//   writer   plans the destination in bursts the same way (`wi_*` order FIFO, one entry per AW or per sparse
//            segment), assembles W beats from the aligned words (C3: two words per beat), counts B responses.
//   xlate    one 4 KiB page per lookup, reader and writer alternate on the MMU client port.  VMID 0 is decoded
//            locally: SYS -> C3 offset, PA[47] -> host, else an error.  Translated: fault -> error (the MMU latched
//            it), sparse -> zeros / dropped writes, SYS -> C3, PA[47] -> host, else BAD_PHYS on ef_* and an error.
//   end      after the last B: one flush read of the last host address written (a read completion cannot pass the
//            posted writes before it; same reason as ng_rcp_hosteng), then `done`.
//   stop     an error or `abort`: no new AR / AW / lookup; R of issued reads drained and dropped, W of issued
//            AWs completed with zero strobes, B collected, no flush; then `done` (err = 1 unless it was an abort).
// Overlapping source and destination are undefined; destination bytes written before an error
// are unspecified.
module ng_rcp_dma
  import ng_mmu_pkg::*;
#(
  parameter int FIFO_LOG2 = 9
) (
  input  logic          clk,
  input  logic          rst_n,
  // command
  input  logic          start,
  input  logic          op_fill,
  input  logic [3:0]    vmid,
  input  logic [47:0]   src,
  input  logic [47:0]   dst,
  input  logic [63:0]   size,
  input  logic [31:0]   fill,
  input  logic          abort,
  output logic          busy,
  output logic          done,
  output logic          err,
  // MMU translation client
  output logic          t_req_valid,
  input  logic          t_req_ready,
  output logic [3:0]    t_req_vmid,
  output logic [63:0]   t_req_va,
  output logic [1:0]    t_req_acc,
  input  logic          t_resp_valid,
  output logic          t_resp_ready,
  input  logic [2:0]    t_resp_fault,
  input  logic          t_resp_sparse,
  input  logic [47:0]   t_resp_pa,
  input  logic [1:0]    t_resp_level,
  input  logic [63:0]   t_resp_pte,
  // BAD_PHYS report
  output logic          ef_valid,
  output logic [3:0]    ef_vmid,
  output logic [63:0]   ef_va,
  output logic [1:0]    ef_acc,
  output logic [2:0]    ef_code,
  output logic [1:0]    ef_level,
  output logic [63:0]   ef_pte,
  // host master (64-bit)
  output logic          h_awvalid,
  input  logic          h_awready,
  output logic [46:0]   h_awaddr,
  output logic [7:0]    h_awlen,
  output logic          h_wvalid,
  input  logic          h_wready,
  output logic [63:0]   h_wdata,
  output logic [7:0]    h_wstrb,
  output logic          h_wlast,
  input  logic          h_bvalid,
  output logic          h_bready,
  input  logic [1:0]    h_bresp,
  output logic          h_arvalid,
  input  logic          h_arready,
  output logic [46:0]   h_araddr,
  output logic [7:0]    h_arlen,
  input  logic          h_rvalid,
  output logic          h_rready,
  input  logic [63:0]   h_rdata,
  input  logic [1:0]    h_rresp,
  input  logic          h_rlast,
  // C3 master (128-bit)
  output logic          c_awvalid,
  input  logic          c_awready,
  output logic [30:0]   c_awaddr,
  output logic [7:0]    c_awlen,
  output logic          c_wvalid,
  input  logic          c_wready,
  output logic [127:0]  c_wdata,
  output logic [15:0]   c_wstrb,
  output logic          c_wlast,
  input  logic          c_bvalid,
  output logic          c_bready,
  input  logic [1:0]    c_bresp,
  output logic          c_arvalid,
  input  logic          c_arready,
  output logic [30:0]   c_araddr,
  output logic [7:0]    c_arlen,
  input  logic          c_rvalid,
  output logic          c_rready,
  input  logic [127:0]  c_rdata,
  input  logic [1:0]    c_rresp,
  input  logic          c_rlast
);
  localparam int FD = 1 << FIFO_LOG2;
  localparam int CW = FIFO_LOG2 + 1;          // FIFO occupancy counter width

  // page kinds (translation cache) / order-FIFO entry kinds
  localparam logic [1:0] K_HOST = 2'd0, K_C3 = 2'd1, K_ZERO = 2'd2;   // K_ZERO: sparse (read 0 / write dropped)

  typedef enum logic [2:0] {S_IDLE, S_RUN, S_FAR, S_FR, S_DONE} st_t;
  st_t st;

  // ----------------------------------------------------------------
  logic        fill_q;
  logic [3:0]  vmid_q;
  logic [31:0] fillv_q;
  logic [47:0] dst_q;
  logic [2:0]  r_sh;            // (S & 7), constant over the command
  logic        stop, err_q, abort_q;

  assign busy = (st != S_IDLE);
  assign stop = err_q || abort_q || abort;

  // word counts at start (end address computed in 64 bits)
  wire [63:0] s_end = {16'h0, src} + size - 64'd1;
  wire [63:0] d_end = {16'h0, dst} + size - 64'd1;
  wire [60:0] s_words = s_end[63:3] - {16'h0, src[47:3]} + 61'd1;
  wire [60:0] d_words = d_end[63:3] - {16'h0, dst[47:3]} + 61'd1;

  // ------------------------------------------------------------------ translation cache + MMU client
  logic        rt_v, wt_v;               // reader / writer page cached
  logic [35:0] rt_pg, wt_pg;             // VA page [47:12]
  logic [1:0]  rt_k, wt_k;
  logic [34:0] rt_b, wt_b;               // host PA[46:12] or C3 offset [30:12] (low 19 bits)
  logic        x_busy, x_sent, x_who;    // lookup open, request accepted, 0 = reader / 1 = writer
  logic        x_last;                   // last client served (alternation)
  logic [35:0] x_pg;
  logic        r_need, w_need;           // client waits for its page (set by the planners below)
  logic [35:0] r_need_pg, w_need_pg;

  assign t_req_valid  = x_busy && !x_sent;
  assign t_req_vmid   = vmid_q;
  assign t_req_va     = {16'h0, x_pg, 12'h000};
  assign t_req_acc    = x_who ? ACC_W : ACC_R;
  assign t_resp_ready = x_busy && x_sent;

  assign ef_vmid = vmid_q;
  assign ef_code = F_BAD_PHYS;

  // VMID 0: physical decode of a page
  function automatic logic [1:0] phys_kind(input logic [47:0] a, output logic ok);
    ok = 1'b1;
    if (a[47:31] == SYS_PA_HI) return K_C3;
    if (a[47])                 return K_HOST;
    ok = 1'b0;
    return K_HOST;
  endfunction

  // ------------------------------------------------------------------ reader plan
  logic [47:0] ra;               // next source word address (8-aligned)
  logic [60:0] rleft;            // source words not yet planned
  logic [CW:0] rsv;              // FIFO words reserved by planned, not yet pushed reads
  logic [CW-1:0] f_cnt;          // FIFO occupancy

  // reader order FIFO: one entry per planned burst / zero segment
  localparam int RI = 16;
  logic [1:0]  ri_k   [RI];
  logic [9:0]  ri_nw  [RI];
  logic        ri_sk  [RI];      // C3: drop the low word of the first beat
  logic [3:0]  ri_wp, ri_rp;
  logic [4:0]  ri_n;
  wire         ri_full  = (ri_n == 5'(RI));
  wire         ri_empty = (ri_n == 5'd0);

  // burst size: min(words left, words to the 4 KiB page end, port cap); computed on capped values
  wire [9:0]  r_topg  = 10'd512 - {1'b0, ra[11:3]};
  wire [9:0]  r_left  = (rleft > 61'd512) ? 10'd512 : rleft[9:0];
  // (sparse source segments <= 64 words too: a blocked reader then implies >= 448 buffered words, enough for any
  // write burst the writer waits for, see w_data_ok)
  wire [9:0]  r_cap   = (rt_k == K_C3) ? (10'd32 - {9'h0, ra[3]}) : 10'd64;
  wire [9:0]  r_nw0   = (r_left < r_topg) ? r_left : r_topg;
  wire [9:0]  r_nw_c  = (r_nw0 < r_cap) ? r_nw0 : r_cap;
  // burst size registered (timing: 61-bit count -> caps -> mins): valid one cycle after a burst or a page update
  logic [9:0] r_nw;
  logic       r_nw_ok;
  wire [9:0]  r_lastw = {1'b0, ra[11:3]} + r_nw - 10'd1;           // last word index in the page
  wire [7:0]  r_beats = 8'((r_lastw >> 1) - {2'b0, ra[11:4]});     // C3 beats - 1
  wire        r_page_ok = rt_v && (rt_pg == ra[47:12]);
  wire        r_room  = ({1'b0, f_cnt} + rsv + (CW+1)'(r_nw)) <= (CW+1)'(FD);
  wire        r_go    = (st == S_RUN) && !stop && !fill_q && (rleft != 0) && r_page_ok && r_nw_ok && !ri_full &&
                        r_room &&
                        ((rt_k == K_HOST) ? !h_arvalid : (rt_k == K_C3) ? !c_arvalid : 1'b1);

  // ------------------------------------------------------------------ writer plan
  logic [47:0] wa;               // next destination word address
  logic [60:0] wleft;
  localparam int WI = 16;
  logic [1:0]  wi_k   [WI];
  logic [9:0]  wi_nw  [WI];
  logic        wi_hi  [WI];      // C3: first word goes to the upper half of the first beat
  logic [7:0]  wi_bt  [WI];      // beats - 1 (C3), words - 1 (host)
  logic [3:0]  wi_wp, wi_rp;
  logic [4:0]  wi_n;
  wire         wi_full  = (wi_n == 5'(WI));
  wire         wi_empty = (wi_n == 5'd0);

  wire [9:0]  w_topg  = 10'd512 - {1'b0, wa[11:3]};
  wire [9:0]  w_left  = (wleft > 61'd512) ? 10'd512 : wleft[9:0];
  wire [9:0]  w_cap   = (wt_k == K_HOST) ? 10'd64 : (wt_k == K_C3) ? (10'd32 - {9'h0, wa[3]}) : 10'd512;
  wire [9:0]  w_nw0   = (w_left < w_topg) ? w_left : w_topg;
  wire [9:0]  w_nw_c  = (w_nw0 < w_cap) ? w_nw0 : w_cap;
  logic [9:0] w_nw;
  logic       w_nw_ok;
  wire [9:0]  w_lastw = {1'b0, wa[11:3]} + w_nw - 10'd1;
  wire [7:0]  w_beats = 8'((w_lastw >> 1) - {2'b0, wa[11:4]});
  wire        w_page_ok = wt_v && (wt_pg == wa[47:12]);
  // An AW goes out only when the data for its whole burst (and for every planned burst before it) is buffered or
  // the source has been read completely: a W burst that waits for read data can block that read (board, b7: the
  // host path held the first 4-byte copy's read behind its open write, both outstanding forever).
  logic        ow_v;             // aligned output word valid (aligner below)
  logic [13:0] w_pend;           // output words planned on the writer side and not yet taken from the aligner
  wire  [13:0] w_avail = 14'(f_cnt) + 14'(ow_v);
  wire         w_data_ok = fill_q || (wt_k == K_ZERO) || ((rleft == 0) && ri_empty) ||
                           (w_avail >= w_pend + 14'(w_nw) + 14'd1);   // dropped words need no AW
  wire        w_go    = (st == S_RUN) && !stop && (wleft != 0) && w_page_ok && w_nw_ok && !wi_full && w_data_ok &&
                        ((wt_k == K_HOST) ? !h_awvalid : (wt_k == K_C3) ? !c_awvalid : 1'b1);

  assign r_need    = (st == S_RUN) && !stop && !fill_q && (rleft != 0) && !r_page_ok;
  assign r_need_pg = ra[47:12];
  assign w_need    = (st == S_RUN) && !stop && (wleft != 0) && !w_page_ok;
  assign w_need_pg = wa[47:12];

  // ------------------------------------------------------------------ FIFO (FWFT, registered BRAM read)
  logic [63:0]  fm [FD];
  logic [FIFO_LOG2-1:0] f_wp, f_rp;
  logic [CW-1:0] f_mem;          // words in the array (not in the output register)
  logic          f_dv;
  logic [63:0]   f_dout;
  logic          f_push, f_pop;
  logic [63:0]   f_din;
  assign f_cnt = f_mem + CW'(f_dv);

  always_ff @(posedge clk) if (f_push) fm[f_wp] <= f_din;
  always_ff @(posedge clk) begin
    if (!rst_n || st == S_IDLE) begin
      f_wp <= '0; f_rp <= '0; f_mem <= '0; f_dv <= 1'b0;
    end else begin
      logic take;
      take = (!f_dv || f_pop) && (f_mem != '0);
      if (take) begin f_dout <= fm[f_rp]; f_rp <= f_rp + 1'b1; end
      if (take) f_dv <= 1'b1; else if (f_pop) f_dv <= 1'b0;
      f_mem <= f_mem + CW'(f_push) - CW'(take);
      if (f_push) f_wp <= f_wp + 1'b1;
    end
  end

  // ------------------------------------------------------------------ R side: push in plan order
  logic [9:0] rp_cnt;            // words pushed for the head entry
  logic       rp_first;          // head C3 entry: next beat is its first
  logic       rp_hi;             // C3: low word of the current beat done, upper next
  wire  [1:0] rh_k  = ri_k[ri_rp];
  wire  [9:0] rh_nw = ri_nw[ri_rp];
  wire        rh_sk = ri_sk[ri_rp];
  logic       r_pop;             // head entry finished
  logic       r_bad;             // RRESP error seen

  // flush read owns the host R channel in S_FR
  always_comb begin
    h_rready = 1'b0; c_rready = 1'b0; f_push = 1'b0; f_din = h_rdata; r_pop = 1'b0; r_bad = 1'b0;
    if (st == S_FR) begin
      h_rready = 1'b1;
    end else if (!ri_empty) begin
      unique case (rh_k)
        K_HOST: begin
          h_rready = 1'b1;
          if (h_rvalid) begin
            f_push = !stop;
            r_bad  = (h_rresp != 2'b00);
            r_pop  = h_rlast;
          end
        end
        K_C3: begin
          if (stop) begin
            c_rready = 1'b1;
            r_pop = c_rvalid && c_rlast;
          end else if (c_rvalid) begin
            r_bad = (c_rresp != 2'b00);
            if (!rp_hi) begin
              // low word: pushed unless it precedes the stream (first beat, rh_sk) or the entry is complete
              f_push = !(rp_first && rh_sk) && (rp_cnt != rh_nw);
              f_din  = c_rdata[63:0];
              // the upper word is needed if words remain after this one
              if ((rp_cnt + 10'(f_push)) == rh_nw) begin c_rready = 1'b1; r_pop = c_rlast; end
            end else begin
              f_push = 1'b1;
              f_din  = c_rdata[127:64];
              c_rready = 1'b1;
              r_pop = c_rlast;
            end
          end
        end
        default: begin             // K_ZERO
          f_push = !stop && (rp_cnt != rh_nw);
          f_din  = 64'h0;
          r_pop  = stop || ((rp_cnt + 10'd1) >= rh_nw);
        end
      endcase
    end
  end

  // ----------------------------------------------------------------
  logic [60:0] o_left;           // output words still to produce
  logic [60:0] s_left;           // stream words still to pop
  logic [63:0] lo;
  logic        lo_ok, o_first;
  logic [63:0] ow;
  logic [7:0]  ow_s;
  logic        ow_take;
  logic [2:0]  d_lo, d_hi;       // dst & 7, (dst + size - 1) & 7

  wire        o_room = !ow_v || ow_take;
  wire        hi_from_fifo = (s_left != 0);
  wire [127:0] pair = {(hi_from_fifo ? f_dout : 64'h0), lo};
  wire [63:0]  shifted = 64'(pair >> {r_sh, 3'b000});
  wire [7:0]   s_first = 8'hff << d_lo;
  wire [7:0]   s_last  = 8'hff >> (3'd7 - d_hi);
  wire         o_last  = (o_left == 61'd1);
  wire [7:0]   o_strb  = (o_first ? s_first : 8'hff) & (o_last ? s_last : 8'hff);
  wire         a_need_lo = !fill_q && !lo_ok;
  wire         a_emit  = (st == S_RUN) && !stop && (o_left != 0) && o_room && !a_need_lo &&
                         (fill_q || !hi_from_fifo || f_dv);
  assign f_pop = (st == S_RUN) && !stop && !fill_q &&
                 ((a_need_lo && f_dv && (o_left != 0)) || (a_emit && hi_from_fifo));

  // ------------------------------------------------------------------ W side
  logic [9:0] wp_cnt;            // words consumed for the head entry
  logic [7:0] wp_bt;             // beats sent for the head entry
  logic       cb_v;              // C3 beat register full
  logic [127:0] cb_d;
  logic [15:0]  cb_s;
  wire  [1:0] wh_k  = wi_k[wi_rp];
  wire  [9:0] wh_nw = wi_nw[wi_rp];
  wire  [7:0] wh_bt = wi_bt[wi_rp];
  wire        cb_pos = wi_hi[wi_rp] ^ wp_cnt[0];   // C3: the entry's next word goes to the upper half
  logic       w_pop;

  always_comb begin
    h_wvalid = 1'b0; h_wdata = ow; h_wstrb = ow_s; h_wlast = 1'b0;
    c_wvalid = 1'b0; c_wdata = cb_d; c_wstrb = cb_s; c_wlast = (wp_bt == wh_bt);
    ow_take = 1'b0; w_pop = 1'b0;
    if (!wi_empty) begin
      unique case (wh_k)
        K_HOST: begin
          h_wlast = (wp_bt == wh_bt);
          // a presented beat keeps its payload until accepted (AXI), also across `stop`: an aligned word that is
          // already up goes out as is, after that only zero-strobe beats
          h_wvalid = 1'b1;
          if (ow_v) ow_take = h_wready;
          else if (stop) h_wstrb = 8'h00;
          else h_wvalid = 1'b0;
          w_pop = h_wvalid && h_wready && h_wlast;
        end
        K_C3: begin
          if (stop) begin
            c_wvalid = 1'b1;
            if (!cb_v) c_wstrb = 16'h0;       // a full beat register still goes out unchanged
          end else begin
            c_wvalid = cb_v;
            // fill the beat register: one aligned word per cycle
            ow_take = ow_v && !cb_v && (wp_cnt != wh_nw);
          end
          w_pop = c_wvalid && c_wready && c_wlast;
        end
        default: begin             // K_ZERO: dropped words
          ow_take = !stop && ow_v && (wp_cnt != wh_nw);
          w_pop   = stop || (ow_take && (wp_cnt + 10'd1 == wh_nw)) || (wp_cnt == wh_nw);
        end
      endcase
    end
  end

  // ------------------------------------------------------------------ B / bookkeeping
  logic [4:0] h_wout, c_wout;    // AWs without B
  logic       host_wr;           // a host write was issued
  logic [46:0] last_h;           // its address
  assign h_bready = 1'b1;
  assign c_bready = 1'b1;

  wire r_idle = ri_empty && !h_arvalid && !c_arvalid;
  wire w_idle = wi_empty && !h_awvalid && !c_awvalid && (h_wout == 5'd0) && (c_wout == 5'd0);
  wire x_idle = !x_busy;
  wire all_planned = (fill_q || rleft == 0) && (wleft == 0);

  // ----------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; done <= 1'b0; err <= 1'b0; err_q <= 1'b0; abort_q <= 1'b0;
      r_nw <= '0; r_nw_ok <= 1'b0; w_nw <= '0; w_nw_ok <= 1'b0; w_pend <= '0;
      fill_q <= 1'b0; vmid_q <= '0; fillv_q <= '0; dst_q <= '0; r_sh <= '0; d_lo <= '0; d_hi <= '0;
      rt_v <= 1'b0; wt_v <= 1'b0; rt_pg <= '0; wt_pg <= '0; rt_k <= '0; wt_k <= '0; rt_b <= '0; wt_b <= '0;
      x_busy <= 1'b0; x_sent <= 1'b0; x_who <= 1'b0; x_last <= 1'b0; x_pg <= '0;
      ef_valid <= 1'b0; ef_va <= '0; ef_acc <= '0; ef_level <= '0; ef_pte <= '0;
      ra <= '0; rleft <= '0; rsv <= '0; wa <= '0; wleft <= '0;
      ri_wp <= '0; ri_rp <= '0; ri_n <= '0; wi_wp <= '0; wi_rp <= '0; wi_n <= '0;
      rp_cnt <= '0; rp_first <= 1'b1; rp_hi <= 1'b0;
      o_left <= '0; s_left <= '0; lo <= '0; lo_ok <= 1'b0; o_first <= 1'b1; ow <= '0; ow_s <= '0; ow_v <= 1'b0;
      wp_cnt <= '0; wp_bt <= '0; cb_v <= 1'b0; cb_d <= '0; cb_s <= '0;
      h_wout <= '0; c_wout <= '0; host_wr <= 1'b0; last_h <= '0;
      h_arvalid <= 1'b0; h_araddr <= '0; h_arlen <= '0; c_arvalid <= 1'b0; c_araddr <= '0; c_arlen <= '0;
      h_awvalid <= 1'b0; h_awaddr <= '0; h_awlen <= '0; c_awvalid <= 1'b0; c_awaddr <= '0; c_awlen <= '0;
    end else begin
      done <= 1'b0;
      ef_valid <= 1'b0;
      if (st != S_IDLE && abort) abort_q <= 1'b1;
      r_nw <= r_nw_c; r_nw_ok <= (st == S_RUN) && !r_go && r_page_ok;
      w_nw <= w_nw_c; w_nw_ok <= (st == S_RUN) && !w_go && w_page_ok;

      // ---- address channel handshakes
      if (h_arvalid && h_arready) h_arvalid <= 1'b0;
      if (c_arvalid && c_arready) c_arvalid <= 1'b0;
      if (h_awvalid && h_awready) h_awvalid <= 1'b0;
      if (c_awvalid && c_awready) c_awvalid <= 1'b0;
      h_wout <= h_wout + 5'(h_awvalid && h_awready) - 5'(h_bvalid);
      c_wout <= c_wout + 5'(c_awvalid && c_awready) - 5'(c_bvalid);
      if ((h_bvalid && h_bresp != 2'b00) || (c_bvalid && c_bresp != 2'b00)) err_q <= 1'b1;

      unique case (st)
        S_IDLE: if (start) begin
          fill_q <= op_fill; vmid_q <= vmid; fillv_q <= fill; dst_q <= dst;
          r_sh <= src[2:0] - dst[2:0];
          d_lo <= dst[2:0]; d_hi <= d_end[2:0];
          err_q <= 1'b0; abort_q <= abort;
          rt_v <= 1'b0; wt_v <= 1'b0; x_last <= 1'b0;
          ra <= {src[47:3], 3'b000}; rleft <= op_fill ? 61'd0 : s_words;
          wa <= {dst[47:3], 3'b000}; wleft <= d_words;
          o_left <= d_words; s_left <= op_fill ? 61'd0 : s_words;
          // q0 = -1 (src & 7 < dst & 7): the word before the stream is the implicit zero
          lo <= '0; lo_ok <= op_fill || (src[2:0] < dst[2:0]); o_first <= 1'b1; ow_v <= 1'b0;
          rp_cnt <= '0; rp_first <= 1'b1; rp_hi <= 1'b0;
          wp_cnt <= '0; wp_bt <= '0; cb_v <= 1'b0; cb_s <= '0;
          host_wr <= 1'b0;
          st <= (size == 64'd0) ? S_DONE : S_RUN;
        end

        S_RUN: begin
          if (stop ? (r_idle && w_idle && x_idle)
                   : (all_planned && r_idle && w_idle && x_idle && !ow_v && (o_left == 0)))
            st <= (!stop && host_wr) ? S_FAR : S_DONE;
        end

        S_FAR: begin
          if (!h_arvalid) begin
            h_arvalid <= 1'b1; h_araddr <= {last_h[46:3], 3'b000}; h_arlen <= 8'd0;
            st <= S_FR;
          end
        end

        S_FR: if (h_rvalid && h_rlast) begin
          if (h_rresp != 2'b00) err_q <= 1'b1;
          st <= S_DONE;
        end

        default: begin             // S_DONE
          done <= 1'b1;
          err  <= err_q && !abort_q;
          st   <= S_IDLE;
        end
      endcase

      // ---- translation: start a lookup (reader / writer alternate), take the response
      if (!x_busy) begin
        if (r_need && (!w_need || x_last)) begin
          x_who <= 1'b0; x_pg <= r_need_pg; x_last <= 1'b0;
          if (vmid_q == 4'd0) begin
            logic ok; logic [1:0] k;
            k = phys_kind({r_need_pg, 12'h000}, ok);
            if (!ok) err_q <= 1'b1;
            else begin rt_v <= 1'b1; rt_pg <= r_need_pg; rt_k <= k; rt_b <= r_need_pg[34:0]; end
          end else begin
            x_busy <= 1'b1; x_sent <= 1'b0;
          end
        end else if (w_need) begin
          x_who <= 1'b1; x_pg <= w_need_pg; x_last <= 1'b1;
          if (vmid_q == 4'd0) begin
            logic ok; logic [1:0] k;
            k = phys_kind({w_need_pg, 12'h000}, ok);
            if (!ok) err_q <= 1'b1;
            else begin wt_v <= 1'b1; wt_pg <= w_need_pg; wt_k <= k; wt_b <= w_need_pg[34:0]; end
          end else begin
            x_busy <= 1'b1; x_sent <= 1'b0;
          end
        end
      end else if (!x_sent) begin
        if (t_req_ready) x_sent <= 1'b1;
      end else if (t_resp_valid) begin
        logic [1:0] k;
        logic       ok;
        ok = 1'b1;
        k  = K_ZERO;
        x_busy <= 1'b0;
        if (t_resp_fault != F_NONE) begin
          ok = 1'b0; err_q <= 1'b1;                       // latched in the MMU fault registers
        end else if (t_resp_sparse) begin
          k = K_ZERO;
        end else if (t_resp_pa[47:31] == SYS_PA_HI) begin
          k = K_C3;
        end else if (t_resp_pa[47]) begin
          k = K_HOST;
        end else begin                                     // VRAM / hole: not reachable in this RM
          ok = 1'b0; err_q <= 1'b1;
          ef_valid <= 1'b1; ef_va <= {16'h0, x_pg, 12'h000}; ef_acc <= x_who ? ACC_W : ACC_R;
          ef_level <= t_resp_level; ef_pte <= t_resp_pte;
        end
        if (ok) begin
          if (!x_who) begin rt_v <= 1'b1; rt_pg <= x_pg; rt_k <= k; rt_b <= t_resp_pa[46:12]; end
          else        begin wt_v <= 1'b1; wt_pg <= x_pg; wt_k <= k; wt_b <= t_resp_pa[46:12]; end
        end
      end

      // ---- reader plan
      if (r_go) begin
        ri_k[ri_wp] <= rt_k; ri_nw[ri_wp] <= r_nw; ri_sk[ri_wp] <= ra[3];
        ri_wp <= ri_wp + 1'b1;
        if (rt_k == K_HOST) begin
          h_arvalid <= 1'b1; h_araddr <= {rt_b, ra[11:3], 3'b000}; h_arlen <= 8'(r_nw - 10'd1);
        end else if (rt_k == K_C3) begin
          c_arvalid <= 1'b1; c_araddr <= {rt_b[18:0], ra[11:4], 4'h0}; c_arlen <= r_beats;
        end
        ra <= ra + {35'h0, r_nw, 3'b000};
        rleft <= rleft - 61'(r_nw);
      end
      ri_n <= ri_n + 5'(r_go) - 5'(r_pop);
      // (a stopped command leaves reservations that were never pushed: cleared between commands)
      if (st == S_IDLE) rsv <= '0;
      else rsv <= rsv + (r_go ? (CW+1)'(r_nw) : '0) - (CW+1)'(f_push);

      // ---- R side state
      if (r_bad) err_q <= 1'b1;
      if (r_pop) begin
        ri_rp <= ri_rp + 1'b1; rp_cnt <= '0; rp_first <= 1'b1; rp_hi <= 1'b0;
      end else if (!ri_empty) begin
        if (f_push) rp_cnt <= rp_cnt + 10'd1;
        if (rh_k == K_C3 && c_rvalid && !stop) begin
          if (!rp_hi) begin
            if (!((rp_cnt + 10'(f_push)) == rh_nw)) rp_hi <= 1'b1;   // upper word follows
            else rp_first <= 1'b0;
          end else begin
            rp_hi <= 1'b0; rp_first <= 1'b0;
          end
        end
      end

      // --
      if (ow_take) ow_v <= 1'b0;
      if (st == S_RUN && !stop && !fill_q && a_need_lo && f_dv && (o_left != 0)) begin
        lo <= f_dout; lo_ok <= 1'b1; s_left <= s_left - 61'd1;
      end else if (a_emit) begin
        ow   <= fill_q ? {fillv_q, fillv_q} : shifted;
        ow_s <= o_strb;
        ow_v <= 1'b1;
        o_first <= 1'b0;
        o_left <= o_left - 61'd1;
        if (!fill_q) begin
          lo <= hi_from_fifo ? f_dout : 64'h0;
          if (hi_from_fifo) s_left <= s_left - 61'd1;
        end
      end

      // ---- writer plan
      if (w_go) begin
        wi_k[wi_wp] <= wt_k; wi_nw[wi_wp] <= w_nw; wi_hi[wi_wp] <= wa[3];
        wi_bt[wi_wp] <= (wt_k == K_C3) ? w_beats : 8'(w_nw - 10'd1);
        wi_wp <= wi_wp + 1'b1;
        if (wt_k == K_HOST) begin
          h_awvalid <= 1'b1; h_awaddr <= {wt_b, wa[11:3], 3'b000}; h_awlen <= 8'(w_nw - 10'd1);
          host_wr <= 1'b1; last_h <= {wt_b, wa[11:3], 3'b000};
        end else if (wt_k == K_C3) begin
          c_awvalid <= 1'b1; c_awaddr <= {wt_b[18:0], wa[11:4], 4'h0}; c_awlen <= w_beats;
        end
        wa <= wa + {35'h0, w_nw, 3'b000};
        wleft <= wleft - 61'(w_nw);
      end
      wi_n <= wi_n + 5'(w_go) - 5'(w_pop);
      if (st == S_IDLE) w_pend <= '0;
      else w_pend <= w_pend + (w_go ? 14'(w_nw) : 14'd0) - 14'(ow_take);

      // ---- W side state
      if (w_pop) begin
        wi_rp <= wi_rp + 1'b1; wp_cnt <= '0; wp_bt <= '0;
        cb_v <= 1'b0; cb_s <= '0;
      end else if (!wi_empty) begin
        if (ow_take) wp_cnt <= wp_cnt + 10'd1;
        unique case (wh_k)
          K_HOST: if (h_wvalid && h_wready) wp_bt <= wp_bt + 8'd1;
          K_C3: begin
            if (c_wvalid && c_wready) begin
              wp_bt <= wp_bt + 8'd1; cb_v <= 1'b0; cb_s <= '0;
            end
            if (ow_take) begin
              if (!cb_pos) begin cb_d[63:0] <= ow; cb_s[7:0] <= ow_s; end
              else begin cb_d[127:64] <= ow; cb_s[15:8] <= ow_s; end
              // beat complete after the upper word or after the entry's last word
              if (cb_pos || (wp_cnt + 10'd1 == wh_nw)) cb_v <= 1'b1;
            end
          end
          default: ;
        endcase
      end
    end
  end
endmodule
