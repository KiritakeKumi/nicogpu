// ng_rsp_bridge.sv -- clock-domain bridges of the M1 SP RM: the SP (clk_ts) talks to the memory side of the
// CP RM (clk_l2).  Constraints: rtl/shell/rm_sp/ng_rm_sp.xdc.
//
//   ng_rsp_fab_ts / ng_rsp_fab_l2   SP fabric port (ng_lsu_pkg::mreq_t line request: icache fill, LSU uc line read /
//        byte-enabled line write / single atomic / cache maintenance).:
//        NOUT request slots; the slot number is the tag.  The request crosses as a 124-bit head through an
//        asynchronous FIFO (Gray pointers) plus the write data / atomic operands in a slot-indexed LUTRAM (wbuf,
//        written by clk_ts BEFORE the head is pushed); the answer comes back as {slot, err} through a second
//        asynchronous FIFO, the read line / old value stay in slot-indexed LUTRAMs (rbuf / rold, written by clk_l2
//        BEFORE the completion is pushed) until the requester frees the slot.  Ordering rules (ng_rsp_fab_ts):
//          same 128 B line, either side a write -> the later request waits until the earlier one has completed
//          (RAW / WAR / WAW; read-read never waits; orphan slots count);
//          M_AMO / M_INV_* / M_WB_L2 are fences: accepted only with no slot allocated, nothing else is accepted
//          until the fence has completed.
//        ng_rsp_fab_l2 executes the heads in FIFO order, several in flight per target (one ID-less AXI port each,
//        in-order R / B; across host / C3 completions come back out of order and carry the slot):
//          M_RD   one read burst of the 128 B line (host 16 x 8 B, C3 8 x 16 B) into rbuf[slot]
//          M_WR   host: a sequence of "runs" -- consecutive whole qwords as one burst, a partially enabled qword as a
//                 1-beat burst with its byte strobes (WR_RUNS = 0: the rule, i.e. only one
//                 contiguous run of whole qwords as a burst, else the per-qword operation path); C3: one burst over
//                 [first .. last enabled 16 B beat], any strobes.  Completed after the B of the last burst.
// every host write beat has a CONTIGUOUS byte strobe -- the XDMA AXI->PCIe bridge writes a
//                 single beat as the byte range [first .. last enabled byte] (measured on the board), so
//                 a qword whose strobes have holes is sent as one 1-beat write per byte run (burst path and
//                 operation path alike; at most 4 for 0x55 / 0xAA).  C3 (MIG) takes any strobe and is not split.
//          M_AMO  read-modify-write of the containing qword through the 64-bit operation path (oparb requester 1),
//                 old value (zero-extended for 32-bit) into rold[slot]; FP ops / misaligned -> ERR_BAD_PA.
// atomic only with respect to other SP accesses (it is a fence); the CP, BAR2 and the
//                 host can interleave between the read and the write.
//          M_INV_L1 / M_INV_L2 / M_WB_L2   completed at once (this RM has no L2; uc data never sits in the L1)
//        Target: uc && PA in SYS (2 GiB at 0x10_0000_0000) -> C3 engine (C3 offset = PA - SYS_BASE),
//                uc && PA in the host aperture (PA[47]) -> host engine (address PA[46:0]),
//                anything else (VRAM: no L2 / VRAM channel in this RM, or a hole) -> err = ERR_BAD_PA (1), no access.
//        A bus error (RRESP / BRESP != OKAY, operation error) -> err = ERR_DDR (3) (the rest of the request still runs;
//        a failed read beat reads as 0).
//   ng_rsp_mport1 adapter: the serial mreq_t / mrsp_t port of ng_sp_top (MEM_OUT = 1, one request
//        flight, the 1024-bit line in the response) onto the slot port of ng_rsp_fab_ts.
//   ng_rsp_mmu_ts / ng_rsp_mmu_l2   SP MMU client port (ng_sp_top mmu_*) -> ng_rsp_mmuarb requester 1 -> ng_mmu; toggle
//        handshake (bundle in flops, toggle through two ASYNC_REG flops, the bundle is quasi-static).
//   ng_rsp_flt_ts / ng_rsp_flt_l2   SP external fault report (LSU BAD_PHYS_ADDR: ext_flt_*) -> one clk_l2 pulse for
//        ng_mmu ext_flt_* (code BAD_PHYS_ADDR, level 0, pte 0).  A second report before the first one crossed
//        (impossible in practice: the LSU reports only from its trap state and the SP halts) is dropped.

// ================================================================================================================
// ng_rsp_fab_ts -- clk_ts half of the SP fabric port.
//
// Request port (the slot is the tag; NOUT = 1 is the one-at-a-time behaviour):
//   mem_req_valid / mem_req_ready / mem_req    the requester holds valid and the request until ready.  ready needs a
//        free slot, no same-line conflict, no fence pending (fence: alloc == 0 and nothing accepted until it
//        completed).  An M_WR is copied into wbuf over 4 clocks (one 32 B pair per clock) before ready rises; the
//        other kinds are accepted in the clock they are offered.  mem_req_slot = the slot the request will take.
//   mem_rsp_valid / _slot / _err / _old    one-clock pulse per completed request (not back-pressured);
//        _old (AMO old value) is valid in the pulse clock.  Orphan slots never answer.
//   mem_rd_slot / mem_rd_data / mem_free    asynchronous read of the line of an answered M_RD slot (rbuf); the slot
//        stays allocated until mem_free with mem_rd_slot (only M_RD slots wait for it; the others are freed by the
//        answer).  rdata is undefined for err = ERR_BAD_PA.
//   sp_rst    SP datapath reset (core_reset | wd_abort): every allocated slot not yet answered becomes an orphan
//        (its answer is reclaimed, not delivered); answered M_RD slots waiting for mem_free are freed.  No request is
//        accepted while it is high.  The FIFOs and the slots are reset by rst_n only.
//   out_cnt / fab_busy    allocated slots (orphans included) / out_cnt != 0 -> ng_cp_port (abort).
// CDC: head FIFO (this side writes hmem + the Gray write pointer x_hgray; clk_l2 reads hmem[x_hra]), write data RAM
// wbe / wbo (even / odd 16 B beat of each 32 B pair, {be[15:0], data[127:0]}, address {slot, pair}, read by clk_l2 at
// x_wra), completion FIFO and rbuf / rold in ng_rsp_fab_l2 (read here at y_cra / y_rra / y_ora).
// AMO operands go to wbe[{slot, 0}] = {16'h0, amo_cmp, amo_a}.
module ng_rsp_fab_ts
  import ng_lsu_pkg::*;
#(
  parameter int  NOUT = 8,                                // request slots: 1, 2, 4, 8 or 16
  localparam int SW   = (NOUT > 1) ? $clog2(NOUT) : 1,    // slot (tag) width
  localparam int NSP  = 1 << SW,                          // RAM / FIFO depth in slots (>= 2)
  localparam int HW   = SW + 124                          // head FIFO entry {fhead_t, slot}
) (
  input  logic          clk,              // clk_ts
  input  logic          rst_n,
  input  logic          sp_rst,
  input  logic          mem_req_valid,
  output logic          mem_req_ready,
  input  mreq_t         mem_req,
  output logic [SW-1:0] mem_req_slot,
  output logic          mem_rsp_valid,
  output logic [SW-1:0] mem_rsp_slot,
  output logic [1:0]    mem_rsp_err,
  output logic          mem_rsp_rd,       // the answered slot was an M_RD (its line waits in rbuf for mem_free)
  output logic [63:0]   mem_rsp_old,
  input  logic [SW-1:0] mem_rd_slot,
  output logic [1023:0] mem_rd_data,
  input  logic          mem_free,
  output logic [SW:0]   out_cnt,
  output logic          fab_busy,
  // to / from clk_l2 (ng_rsp_fab_l2)
  output logic [SW:0]   x_hgray,
  input  logic [SW-1:0] x_hra,
  output logic [HW-1:0] x_hd,
  input  logic [SW+1:0] x_wra,
  output logic [143:0]  x_wde,
  output logic [143:0]  x_wdo,
  input  logic [SW:0]   y_cgray,
  output logic [SW-1:0] y_cra,
  input  logic [SW+1:0] y_cd,
  output logic [SW-1:0] y_rra,
  input  logic [1023:0] y_rdh,
  input  logic [1023:0] y_rdc,
  output logic [SW-1:0] y_ora,
  input  logic [63:0]   y_old,
  // debug
  output logic [31:0]   n_req,
  output logic [31:0]   n_err,
  output logic [47:0]   last_pa,
  output logic [2:0]    last_kind,
  output logic [1:0]    last_err,
  // latency statistics: clk_ts cycles from the accept (mem_req_valid && mem_req_ready) to the response pulse
  // (mem_rsp_valid), per M_RD / M_WR request (I$ fills are M_RD; AMO / cache maintenance / orphans are not counted).
  // Free-running from rst_n like n_req; lat_*_sum wraps at 2^48 (a single latency is modulo 2^32).
  output logic [47:0]   rd_lat_sum,
  output logic [31:0]   rd_lat_n,
  output logic [31:0]   rd_lat_max,
  output logic [47:0]   wr_lat_sum,
  output logic [31:0]   wr_lat_n,
  output logic [31:0]   wr_lat_max,
  // slot statistics (free-running from rst_n)
  output logic [SW:0]   max_out,          // largest out_cnt seen
  output logic [31:0]   n_orphan,         // answers reclaimed as orphans (not delivered)
  output logic [31:0]   n_cstall,         // clocks a request waited for a same-line conflict
  output logic [31:0]   n_sstall,         // clocks a request waited for a free slot
  output logic [31:0]   n_fence,          // fence requests accepted (AMO, cache maintenance)
  output logic [31:0]   n_fstall          // clocks a request waited for a fence (pending fence, or a fence for alloc == 0)
);
  typedef struct packed {
    logic [15:0]   hq;        // host write: qwords whose byte strobes are not one contiguous run (split)
    logic          fb;        // host write that the rule sends to the operation path (partial qword / holey)
    logic [15:0]   en;        // write: last beat of each burst (host: qword, C3: 16 B beat)
    logic [15:0]   st;        // write: first beat of each burst
    logic [15:0]   wm;        // write: qwords with any byte enabled
    logic          amo64;
    logic [4:0]    amo_op;
    logic          imm;       // complete at once with err 0 (cache maintenance, write with be == 0)
    logic          bad;       // ERR_BAD_PA, no access
    logic [47:0]   pa;
    logic [2:0]    kind;
  } fhead_t;

  function automatic logic multi_run(input logic [15:0] m);     // more than one run of set bits
    logic seen, prev;
    multi_run = 1'b0; seen = 1'b0; prev = 1'b0;
    for (int i = 0; i < 16; i++) begin
      if (m[i] && !prev) begin
        if (seen) multi_run = 1'b1;
        seen = 1'b1;
      end
      prev = m[i];
    end
  endfunction
  // the byte strobes of one qword are not a single contiguous run (0 and full / contiguous masks are not)
  function automatic logic holey8(input logic [7:0] s);
    logic seen, prev;
    holey8 = 1'b0; seen = 1'b0; prev = 1'b0;
    for (int i = 0; i < 8; i++) begin
      if (s[i] && !prev) begin
        if (seen) holey8 = 1'b1;
        seen = 1'b1;
      end
      prev = s[i];
    end
  endfunction

  // ---------------------------------------------------------------- request classification (clk_ts, from mem_req)
  wire [47:0] pa      = mem_req.pa;
  wire        k_rd    = (mem_req.kind == M_RD);
  wire        k_wr    = (mem_req.kind == M_WR);
  wire        k_amo   = (mem_req.kind == M_AMO);
  wire        k_maint = (mem_req.kind == M_INV_L1) || (mem_req.kind == M_INV_L2) || (mem_req.kind == M_WB_L2);
  wire        k_fence = k_amo || k_maint;
  wire        in_sys  = (pa[47:31] == 17'h00020);
  wire        in_host = pa[47];
  wire        amo_ok  = (mem_req.amo_op <= 5'd12) && (mem_req.amo64 ? (pa[2:0] == 3'b000) && (mem_req.amo_op <= 5'd10)
                                                                    : (pa[1:0] == 2'b00));
  wire        c_bad   = !k_maint && (!mem_req.uc || !(in_sys || in_host) || (k_amo && !amo_ok) || !(k_rd || k_wr || k_amo));

  logic [15:0] wmq, fq, ffq, sth, enh, stc, enc;
  logic [7:0]  cb;
  logic [2:0]  clo, chi;
  fhead_t      hn;
  always_comb begin
    for (int q = 0; q < 16; q++) begin
      wmq[q] = |mem_req.be[8*q +: 8];
      fq[q]  = &mem_req.be[8*q +: 8];
    end
    ffq = wmq & fq;
    // host runs: a whole qword continues the run of a whole previous qword; a partial qword is a run of its own
    for (int q = 0; q < 16; q++) begin
      sth[q] = wmq[q] && (!fq[q] || (q == 0)  || !ffq[(q + 15) % 16]);
      enh[q] = wmq[q] && (!fq[q] || (q == 15) || !ffq[(q + 1) % 16]);
    end
    // C3: one burst over [first .. last enabled 16 B beat]
    for (int b = 0; b < 8; b++) cb[b] = wmq[2*b] | wmq[2*b+1];
    clo = '0; chi = '0;
    for (int b = 7; b >= 0; b--) if (cb[b]) clo = 3'(b);
    for (int b = 0; b < 8; b++)  if (cb[b]) chi = 3'(b);
    stc = (cb != 8'h0) ? (16'h1 << clo) : 16'h0;
    enc = (cb != 8'h0) ? (16'h1 << chi) : 16'h0;
    hn        = '0;
    hn.kind   = mem_req.kind;
    hn.pa     = pa;
    hn.bad    = c_bad;
    hn.imm    = k_maint || (!c_bad && k_wr && (wmq == 16'h0));
    hn.amo_op = mem_req.amo_op;
    hn.amo64  = mem_req.amo64;
    hn.wm     = wmq;
    hn.st     = in_host ? sth : stc;
    hn.en     = in_host ? enh : enc;
    hn.fb     = in_host && (((wmq & ~fq) != 16'h0) || multi_run(wmq));
    for (int q = 0; q < 16; q++) hn.hq[q] = in_host && holey8(mem_req.be[8*q +: 8]);
  end

  // --------------------------------------------------------------
  logic [NOUT-1:0] alloc, orphan, rdw, s_rd, s_wr, s_amo, s_fence, s_host;
  logic [40:0]     s_line [NOUT];
  logic [31:0]     s_t0   [NOUT];
  logic [31:0]     tnow;
  logic [NOUT-1:0] lhit;
  logic [SW-1:0]   fslot;
  always_comb begin
    for (int i = 0; i < NOUT; i++) lhit[i] = alloc[i] && (s_line[i] == pa[47:7]) && (s_wr[i] || k_wr);
    fslot = '0;
    for (int i = NOUT - 1; i >= 0; i--) if (!alloc[i]) fslot = SW'(i);
  end
  wire any_free = (alloc != {NOUT{1'b1}});
  wire conflict = (lhit != '0);
  wire bar_act  = ((alloc & s_fence) != '0);
  wire f_wait   = bar_act || (k_fence && (alloc != '0));
  wire can      = mem_req_valid && !sp_rst && !f_wait && any_free && !conflict;

  // accept: M_WR stages its 4 data pairs into wbuf first (ready in the 4th clock), the rest is accepted at once
  logic          stg;
  logic [1:0]    pair;
  logic [SW-1:0] stg_slot;
  assign mem_req_ready = stg ? ((pair == 2'd3) && !sp_rst) : (can && !k_wr);
  assign mem_req_slot  = stg ? stg_slot : fslot;
  wire   hs            = mem_req_valid && mem_req_ready;

  // ---------------------------------------------------------------- write data RAM (LUTRAM, read by clk_l2)
  logic [143:0] wbe [NSP * 4];
  logic [143:0] wbo [NSP * 4];
  wire  [1:0]   wpair = stg ? pair : 2'd0;
  wire          wb_we = stg ? !sp_rst : (can && (k_wr || k_amo));
  wire  [SW+1:0] wb_a = {mem_req_slot, wpair};
  logic [143:0] wbe_d, wbo_d;
  always_comb begin
    wbe_d = {mem_req.be[32*wpair +: 16], mem_req.wdata[256*wpair +: 128]};
    wbo_d = {mem_req.be[32*wpair + 16 +: 16], mem_req.wdata[256*wpair + 128 +: 128]};
    if (k_amo) wbe_d = {16'h0, mem_req.amo_cmp, mem_req.amo_a};
  end
  always_ff @(posedge clk) begin
    if (wb_we) begin
      wbe[wb_a] <= wbe_d;
      wbo[wb_a] <= wbo_d;
    end
  end
  assign x_wde = wbe[x_wra];
  assign x_wdo = wbo[x_wra];

  // ---------------------------------------------------------------- head FIFO (write side)
  logic [HW-1:0] hmem [NSP];
  logic [SW:0]   hwb;
  always_ff @(posedge clk) if (hs) hmem[hwb[SW-1:0]] <= {hn, mem_req_slot};
  assign x_hd = hmem[x_hra];

  // ---------------------------------------------------------------- completion FIFO (read side)
  (* ASYNC_REG = "TRUE" *) logic [SW:0] cg1, cg2;
  logic [SW:0]   crb;
  wire  [SW:0]   crg   = crb ^ (crb >> 1);
  wire           c_pop = (crg != cg2);
  wire  [SW-1:0] c_slot = y_cd[SW+1:2];
  wire  [1:0]    c_err  = y_cd[1:0];
  assign y_cra = crb[SW-1:0];
  wire           c_drop = orphan[c_slot] || sp_rst;          // reclaim, do not deliver
  wire  [31:0]   lat    = tnow - s_t0[c_slot];

  assign y_rra       = mem_rd_slot;
  assign mem_rd_data = s_host[mem_rd_slot] ? y_rdh : y_rdc;
  assign y_ora       = mem_rsp_slot;
  assign mem_rsp_old = y_old;
  assign fab_busy    = (alloc != '0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      stg <= 1'b0; pair <= '0; stg_slot <= '0;
      hwb <= '0; x_hgray <= '0; cg1 <= '0; cg2 <= '0; crb <= '0;
      alloc <= '0; orphan <= '0; rdw <= '0; s_rd <= '0; s_wr <= '0; s_amo <= '0; s_fence <= '0; s_host <= '0;
      for (int i = 0; i < NOUT; i++) begin s_line[i] <= '0; s_t0[i] <= '0; end
      tnow <= '0; out_cnt <= '0;
      mem_rsp_valid <= 1'b0; mem_rsp_slot <= '0; mem_rsp_err <= '0; mem_rsp_rd <= 1'b0;
      n_req <= '0; n_err <= '0; last_pa <= '0; last_kind <= '0; last_err <= '0;
      rd_lat_sum <= '0; rd_lat_n <= '0; rd_lat_max <= '0; wr_lat_sum <= '0; wr_lat_n <= '0; wr_lat_max <= '0;
      max_out <= '0; n_orphan <= '0; n_cstall <= '0; n_sstall <= '0; n_fence <= '0; n_fstall <= '0;
    end else begin
      tnow <= tnow + 32'd1;
      cg1 <= y_cgray; cg2 <= cg1;

      // --
      if (stg) begin
        if (sp_rst || !mem_req_valid) stg <= 1'b0;            // requester reset: restart from the first pair
        else if (pair == 2'd3)        stg <= 1'b0;            // handshake this clock
        else                          pair <= pair + 2'd1;
      end else if (can && k_wr) begin
        stg <= 1'b1; pair <= 2'd1; stg_slot <= fslot;         // pair 0 written this clock
      end
      if (!stg && mem_req_valid && !sp_rst && !can) begin
        if (f_wait)         n_fstall <= n_fstall + 32'd1;
        else if (!any_free) n_sstall <= n_sstall + 32'd1;
        else if (conflict)  n_cstall <= n_cstall + 32'd1;
      end
      if (hs) begin
        hwb     <= hwb + 1'b1;
        x_hgray <= (hwb + 1'b1) ^ ((hwb + 1'b1) >> 1);
        n_req   <= n_req + 32'd1;
        last_pa <= pa; last_kind <= mem_req.kind;
        if (k_fence) n_fence <= n_fence + 32'd1;
      end

      // --
      mem_rsp_valid <= 1'b0;
      if (c_pop) begin
        crb <= crb + 1'b1;
        if (c_drop) n_orphan <= n_orphan + 32'd1;
        if (!c_drop) begin
          mem_rsp_valid <= 1'b1; mem_rsp_slot <= c_slot; mem_rsp_err <= c_err; mem_rsp_rd <= s_rd[c_slot];
          last_err <= c_err;
          if (c_err != 2'd0) n_err <= n_err + 32'd1;
          if (s_rd[c_slot]) begin
            rd_lat_sum <= rd_lat_sum + 48'(lat); rd_lat_n <= rd_lat_n + 32'd1;
            if (lat > rd_lat_max) rd_lat_max <= lat;
          end
          if (s_wr[c_slot]) begin
            wr_lat_sum <= wr_lat_sum + 48'(lat); wr_lat_n <= wr_lat_n + 32'd1;
            if (lat > wr_lat_max) wr_lat_max <= lat;
          end
        end
      end

      // ---- slot state
      for (int i = 0; i < NOUT; i++) begin
        logic set_a, pop_i, pop_free, pop_rdw, free_i, rst_free;
        set_a    = hs && (mem_req_slot == SW'(i));
        pop_i    = c_pop && (c_slot == SW'(i));
        pop_free = pop_i && (c_drop || !s_rd[i]);
        pop_rdw  = pop_i && !c_drop && s_rd[i];
        free_i   = mem_free && (mem_rd_slot == SW'(i)) && rdw[i];
        rst_free = sp_rst && rdw[i];
        alloc[i]  <= set_a || (alloc[i] && !(pop_free || free_i || rst_free));
        rdw[i]    <= pop_rdw || (rdw[i] && !(free_i || rst_free));
        orphan[i] <= (orphan[i] && !pop_i) || (sp_rst && alloc[i] && !rdw[i] && !pop_i);
        if (set_a) begin
          s_rd[i] <= k_rd; s_wr[i] <= k_wr; s_amo[i] <= k_amo; s_fence[i] <= k_fence; s_host[i] <= in_host;
          s_line[i] <= pa[47:7]; s_t0[i] <= tnow;
        end
      end
      out_cnt <= (SW + 1)'($countones(alloc));
      if (out_cnt > max_out) max_out <= out_cnt;
    end
  end

`ifndef SYNTHESIS
  // the slot count bounds both FIFOs: a head is pushed only into a free slot, a completion only for an allocated one
  always @(posedge clk) if (rst_n) begin
    if (hs && alloc[mem_req_slot]) $display("ERROR ng_rsp_fab_ts: accepted into an allocated slot %0d", mem_req_slot);
    if (c_pop && !alloc[c_slot])   $display("ERROR ng_rsp_fab_ts: completion for a free slot %0d", c_slot);
  end
`endif
endmodule

// ================================================================================================================
// ng_rsp_fab_l2 -- clk_l2 half of the SP fabric port.
//
// Dispatcher (one head at a time, in FIFO order; a head is only popped with quiesce = 0, so heads still in the FIFO are
// not "busy" for the RM quiesce, as in):
//   bad / imm          completion at once (ERR_BAD_PA / 0), no bus access
//   M_RD (BURST)       one AR; the R beats land in rbuf[slot] through a register stage per target; the completion
//                      is set with the rbuf write of the last beat
//   M_WR (BURST)       the runs of the head (st / en flags computed in clk_ts): per run one AW + its W beats from
//                      wbuf; the B-order FIFO entry of the last run carries `last` -> completion with that B
//                      (B in order per target, so every earlier B of the slot has arrived)
// a run that is a holey host qword (hd.hq, computed in clk_ts) reads its strobes
//                      wbuf (D_WS0, the fetch is idle there) into sb_rem, then issues one AW (same qword, len 0) + one
//                      W beat (strobe & f_m) per byte run (D_WS1 -> D_WRB -> D_WS1 ...); `last` on the final one
//   M_AMO, BURST = 0, WR_RUNS = 0 host fallback   wait until nothing of this port is outstanding, then the
//                      64-bit operation path (oparb); completion when done. a holey host qword is one
//                      operation per byte run (D_OPS between D_OPL and D_OP); atomics write 0x0F / 0xF0 / 0xFF
//   HOST_WR_FLUSH host write   serial: wait until idle, the runs, wait until all their B arrived, one 1-beat flush
//                      read of the first run's address; its R completes the slot (semantics, for bisecting)
// Response channels are always ready (rready = bready = 1): the destination of every R / B was fixed when its
// address was issued (R / B order FIFOs of {slot, flag}), so they can never be blocked -- the deadlock-freedom
// argument   A write address is only issued with its data available (wbuf) and the W beat
// pipeline never holds beats of both targets (a write to the other target waits until it drained), so a stalled
// target cannot hold up the other one's W channel.
// Completions: per-slot done / error bits, pushed one at a time (at most every second clock, which keeps the Gray
// pointer step >= 8 ns apart for the 6 ns datapath bound on the synchroniser) into the completion FIFO after the data
// (rbuf / rold) was written.
// Timing (250 MHz): every AXI output comes from a flop; the wbuf read (address = registered {slot, pair}) feeds one
// 4:1 select into a W stage register, then the W output register; the R beat goes through one register stage before
// the rbuf write (write enable = decode of the registered beat index); the run iterator uses two clocks per run (two
// independent 16-bit priority encoders, then the AW fields); the atomic ALU is split into two clocks; the byte-run
// split  takes the strobes into a register first (lowest run = s & ~(s + (s & -s)), 8 bits, from a flop).
module ng_rsp_fab_l2
  import ng_lsu_pkg::*;
#(
  parameter int  NOUT          = 8,
  parameter bit  BURST         = 1'b1,      // 0: every line as 64-bit operations, for bisecting
  parameter bit  HOST_WR_FLUSH = 1'b0,      // 1: host writes serial with a flush read before the completion
  parameter bit  WR_RUNS       = 1'b1,      // 0: host partial / holey writes take the operation path (rule)
  localparam int SW  = (NOUT > 1) ? $clog2(NOUT) : 1,
  localparam int NSP = 1 << SW,
  localparam int HW  = SW + 124
) (
  input  logic          clk,              // clk_l2
  input  logic          rst_n,
  input  logic          quiesce,
  // to / from clk_ts (ng_rsp_fab_ts)
  input  logic [SW:0]   x_hgray,          // asynchronous (Gray)
  output logic [SW-1:0] x_hra,
  input  logic [HW-1:0] x_hd,
  output logic [SW+1:0] x_wra,
  input  logic [143:0]  x_wde,
  input  logic [143:0]  x_wdo,
  output logic [SW:0]   y_cgray,
  input  logic [SW-1:0] y_cra,            // clk_ts read addresses (asynchronous LUTRAM reads)
  output logic [SW+1:0] y_cd,
  input  logic [SW-1:0] y_rra,
  output logic [1023:0] y_rdh,
  output logic [1023:0] y_rdc,
  input  logic [SW-1:0] y_ora,
  output logic [63:0]   y_old,
  // 64-bit operation (to ng_rsp_oparb requester 1 of the C3 engine / the host engine)
  output logic          c_valid,
  input  logic          c_ready,
  input  logic          c_rs_valid,
  input  logic          c_rs_err,
  input  logic [63:0]   c_rs_rdata,
  output logic          h_valid,
  input  logic          h_ready,
  input  logic          h_rs_valid,
  input  logic          h_rs_err,
  input  logic [63:0]   h_rs_rdata,
  output logic          op_wr,
  output logic [46:0]   op_addr,          // C3 offset (C3) or host address (host)
  output logic [7:0]    op_wstrb,
  output logic [63:0]   op_wdata,
  // burst master, host (ID-less AXI, 64-bit beats; to ng_rcp_axmux s1 in front of ng_rcp_hosteng b_*)
  output logic          bh_awvalid,
  input  logic          bh_awready,
  output logic [46:0]   bh_awaddr,
  output logic [7:0]    bh_awlen,
  output logic          bh_wvalid,
  input  logic          bh_wready,
  output logic [63:0]   bh_wdata,
  output logic [7:0]    bh_wstrb,
  output logic          bh_wlast,
  input  logic          bh_bvalid,
  output logic          bh_bready,
  input  logic [1:0]    bh_bresp,
  output logic          bh_arvalid,
  input  logic          bh_arready,
  output logic [46:0]   bh_araddr,
  output logic [7:0]    bh_arlen,
  input  logic          bh_rvalid,
  output logic          bh_rready,
  input  logic [63:0]   bh_rdata,
  input  logic [1:0]    bh_rresp,
  input  logic          bh_rlast,
  // burst master, C3 (128-bit beats, 31-bit offset; to ng_rcp_axmux s1 in front of ng_rcp_c3eng b_*)
  output logic          bc_awvalid,
  input  logic          bc_awready,
  output logic [30:0]   bc_awaddr,
  output logic [7:0]    bc_awlen,
  output logic          bc_wvalid,
  input  logic          bc_wready,
  output logic [127:0]  bc_wdata,
  output logic [15:0]   bc_wstrb,
  output logic          bc_wlast,
  input  logic          bc_bvalid,
  output logic          bc_bready,
  input  logic [1:0]    bc_bresp,
  output logic          bc_arvalid,
  input  logic          bc_arready,
  output logic [30:0]   bc_araddr,
  output logic [7:0]    bc_arlen,
  input  logic          bc_rvalid,
  output logic          bc_rready,
  input  logic [127:0]  bc_rdata,
  input  logic [1:0]    bc_rresp,
  input  logic          bc_rlast,
  // debug
  output logic [2:0]    dbg_st,
  output logic [31:0]   n_ops,
  output logic [31:0]   n_bad,
  output logic [47:0]   bad_pa,
  output logic [31:0]   n_burst,          // bursts started (read, write run, flush read)
  output logic [31:0]   n_wfb             // line writes sent to the operation path by the rule (WR_RUNS = 0)
);
  localparam logic [1:0] ERR_BAD_PA = 2'd1, ERR_DDR = 2'd3;

  typedef struct packed {      // = ng_rsp_fab_ts fhead_t
    logic [15:0]   hq;
    logic          fb;
    logic [15:0]   en;
    logic [15:0]   st;
    logic [15:0]   wm;
    logic          amo64;
    logic [4:0]    amo_op;
    logic          imm;
    logic          bad;
    logic [47:0]   pa;
    logic [2:0]    kind;
  } fhead_t;

  function automatic logic [3:0] first1(input logic [15:0] m);
    first1 = '0;
    for (int i = 15; i >= 0; i--) if (m[i]) first1 = 4'(i);
  endfunction
  // the lowest run of set bits (adding the lowest set bit clears exactly that run)
  function automatic logic [7:0] lowrun(input logic [7:0] s);
    lowrun = s & ~(s + (s & (~s + 8'd1)));
  endfunction

  // ---------------------------------------------------------------- head FIFO (read side)
  (* ASYNC_REG = "TRUE" *) logic [SW:0] hg1, hg2;
  logic [SW:0]   hrb;
  wire  [SW:0]   hrg  = hrb ^ (hrb >> 1);
  wire           h_ne = (hrg != hg2);
  assign x_hra = hrb[SW-1:0];
  fhead_t        hd;
  logic [SW-1:0] h_slot;
  wire           h_host = hd.pa[47];
  wire  [46:0]   h_ta   = h_host ? hd.pa[46:0] : {16'h0, hd.pa[30:0]};   // target address
  wire           h_rd   = (hd.kind == M_RD);
  wire           h_wr   = (hd.kind == M_WR);
  wire           h_amo  = (hd.kind == M_AMO);

  typedef enum logic [4:0] {D_IDLE, D_DEC, D_RD, D_WR0, D_WR1, D_WRB, D_FW0, D_FW1, D_FLR, D_OPW, D_OPL, D_OP, D_WAIT,
                            D_CALC1, D_CALC2, D_WS0, D_WS1, D_OPS} ds_t;
  ds_t ds;
  always_comb begin
    unique case (ds)
      D_IDLE:                     dbg_st = 3'd0;
      D_OPW, D_OPL, D_OPS, D_OP:  dbg_st = 3'd1;
      D_WAIT:                     dbg_st = 3'd2;
      D_CALC1, D_CALC2:           dbg_st = 3'd3;
      D_DEC:                      dbg_st = 3'd4;
      D_WR0, D_WR1, D_FW0, D_FW1, D_WS0, D_WS1: dbg_st = 3'd5;
      D_RD, D_FLR:                dbg_st = 3'd6;
      default:                    dbg_st = 3'd7;     // D_WRB
    endcase
  end

  // ---------------------------------------------------------------- burst address channels
  logic         ar_v, aw_v, a_host;
  logic [46:0]  b_addr;
  logic [7:0]   b_len;
  assign bh_arvalid = ar_v && a_host;    assign bc_arvalid = ar_v && !a_host;
  assign bh_araddr  = b_addr;            assign bc_araddr  = b_addr[30:0];
  assign bh_arlen   = b_len;             assign bc_arlen   = b_len;
  assign bh_awvalid = aw_v && a_host;    assign bc_awvalid = aw_v && !a_host;
  assign bh_awaddr  = b_addr;            assign bc_awaddr  = b_addr[30:0];
  assign bh_awlen   = b_len;             assign bc_awlen   = b_len;
  wire ar_hs = ar_v && (a_host ? bh_arready : bc_arready);
  wire aw_hs = aw_v && (a_host ? bh_awready : bc_awready);
  assign bh_rready = 1'b1;  assign bc_rready = 1'b1;
  assign bh_bready = 1'b1;  assign bc_bready = 1'b1;

  // ---------------------------------------------------------------- R / B order FIFOs ({flag, slot})
  logic          ar_fl;                   // the pending AR is a flush read
  logic          aw_last;                 // the pending AW is the last run of its slot
  logic [SW:0]   rfh_q, rfc_q, bfh_q, bfc_q;
  logic          rfh_e, rfc_e, bfh_e, bfc_e, rfh_f, rfc_f, bfh_f, bfc_f;
  ng_prim_fifo_sync #(.W(SW + 1), .DEPTH(NSP), .REG_FLAGS(1'b1)) u_rfh (.clk(clk), .rst_n(rst_n), .push(ar_hs && a_host),
    .din({ar_fl, h_slot}), .pop(bh_rvalid && bh_rlast), .dout(rfh_q), .empty(rfh_e), .full(rfh_f), .count());
  ng_prim_fifo_sync #(.W(SW + 1), .DEPTH(NSP), .REG_FLAGS(1'b1)) u_rfc (.clk(clk), .rst_n(rst_n), .push(ar_hs && !a_host),
    .din({ar_fl, h_slot}), .pop(bc_rvalid && bc_rlast), .dout(rfc_q), .empty(rfc_e), .full(rfc_f), .count());
  ng_prim_fifo_sync #(.W(SW + 1), .DEPTH(16), .REG_FLAGS(1'b1)) u_bfh (.clk(clk), .rst_n(rst_n), .push(aw_hs && a_host),
    .din({aw_last, h_slot}), .pop(bh_bvalid), .dout(bfh_q), .empty(bfh_e), .full(bfh_f), .count());
  ng_prim_fifo_sync #(.W(SW + 1), .DEPTH(16), .REG_FLAGS(1'b1)) u_bfc (.clk(clk), .rst_n(rst_n), .push(aw_hs && !a_host),
    .din({aw_last, h_slot}), .pop(bc_bvalid), .dout(bfc_q), .empty(bfc_e), .full(bfc_f), .count());

  // ---------------------------------------------------------------- W beat pipeline (fetch -> stage A -> W register)
  // f_i: qword index (C3 beats step by 2), the wbuf address is {f_slot, f_i[3:2]} straight from the flops; the
  // operation path uses the same registers for its write data / atomic operands.
  logic          f_act, f_c3;
  logic [SW-1:0] f_slot;
  logic [3:0]    f_i, f_e;
  logic [7:0]    f_m;                     // host strobe mask of the fetch (byte run; 0xFF otherwise)
  logic          a_v, a_h, a_l;
  logic [127:0]  a_d;
  logic [15:0]   a_s;
  logic          wv, w_h, wl;
  logic [127:0]  wd;
  logic [15:0]   ws;
  assign x_wra = {f_slot, f_i[3:2]};
  wire  [143:0] wsel  = f_i[1] ? x_wdo : x_wde;                       // the 16 B beat
  wire  [63:0]  qsel  = f_i[0] ? wsel[127:64]  : wsel[63:0];          // the qword of it
  wire  [7:0]   qstb  = f_i[0] ? wsel[143:136] : wsel[135:128];
  wire  [127:0] nb_d  = f_c3 ? wsel[127:0]   : {64'h0, qsel};
  wire  [15:0]  nb_s  = f_c3 ? wsel[143:128] : {8'h0, qstb & f_m};
  wire          w_hs  = wv && (w_h ? bh_wready : bc_wready);
  wire          w_adv = !wv || w_hs;
  wire          a_take = a_v && w_adv;
  wire          w_empty = !f_act && !a_v && !wv;
  assign bh_wvalid = wv && w_h;          assign bc_wvalid = wv && !w_h;
  assign bh_wdata  = wd[63:0];           assign bc_wdata  = wd;
  assign bh_wstrb  = ws[7:0];            assign bc_wstrb  = ws;
  assign bh_wlast  = wl;                 assign bc_wlast  = wl;

  // ---------------------------------------------------------------- R beat register stage (per target)
  logic          rqh_v, rqh_fl, rqh_e, rqh_l;
  logic [SW-1:0] rqh_slot;
  logic [3:0]    rqh_q, rih;
  logic [63:0]   rqh_d;
  logic          rqc_v, rqc_fl, rqc_e, rqc_l, rqc_w2;
  logic [SW-1:0] rqc_slot;
  logic [3:0]    rqc_q;
  logic [2:0]    ric;
  logic [127:0]  rqc_d;

  logic [15:0]   wm_rem;                  // operation path: qwords still to write
  logic          amo_wr;                  // operation path: the atomic's write
  // byte-run split of a holey host qword (burst path D_WS0 / D_WS1, operation path D_OPS): strobes not sent yet
  logic [7:0]    sb_rem;
  wire  [7:0]    sb_run = lowrun(sb_rem);
  wire  [7:0]    sb_nxt = sb_rem & ~sb_run;
  // operation path response (selected by the target of the head)
  wire        rsv = h_host ? h_rs_valid : c_rs_valid;
  wire        rse = h_host ? h_rs_err   : c_rs_err;
  wire [63:0] rsd = h_host ? h_rs_rdata : c_rs_rdata;
  wire        op_rs_rd = (ds == D_WAIT) && rsv && h_rd;

  // ---------------------------------------------------------------- read-back RAMs (LUTRAM, read by clk_ts)
  // rbuf per target, 16 qword RAMs each (host R: qword i; C3 R: qwords 2i, 2i+1; operation path: qword q)
  for (genvar k = 0; k < 16; k++) begin : g_rbh
    logic [63:0] m [NSP];
    wire we = rqh_v && !rqh_fl && (rqh_q == 4'(k));
    always_ff @(posedge clk) if (we) m[rqh_slot] <= rqh_e ? 64'h0 : rqh_d;
    assign y_rdh[64*k +: 64] = m[y_rra];
  end
  for (genvar k = 0; k < 16; k++) begin : g_rbc
    logic [63:0] m [NSP];
    wire we = rqc_v && !rqc_fl && (rqc_w2 ? (rqc_q[3:1] == 3'(k / 2)) : (rqc_q == 4'(k)));
    wire [63:0] d = (rqc_w2 && (k % 2 == 1)) ? rqc_d[127:64] : rqc_d[63:0];
    always_ff @(posedge clk) if (we) m[rqc_slot] <= rqc_e ? 64'h0 : d;
    assign y_rdc[64*k +: 64] = m[y_rra];
  end
  logic [63:0] rold [NSP];
  wire         rold_we = (ds == D_WAIT) && rsv && h_amo && !amo_wr;
  always_ff @(posedge clk) if (rold_we) rold[h_slot] <= hd.amo64 ? rsd : {32'h0, hd.pa[2] ? rsd[63:32] : rsd[31:0]};
  assign y_old = rold[y_ora];

  // ---------------------------------------------------------------- per-slot completion state + completion FIFO
  logic [NOUT-1:0] cdone, serr, cbad, cd_set, se_set, cd_clr;
  logic [SW-1:0]   pp;
  logic            pp_v, cpush_q;
  logic [SW+1:0]   cmem [NSP];
  logic [SW:0]     cwb;
  always_comb begin
    pp = '0;
    for (int i = NOUT - 1; i >= 0; i--) if (cdone[i]) pp = SW'(i);
  end
  assign pp_v = (cdone != '0);
  wire push = pp_v && !cpush_q;
  wire [15:0] wm_nxt = wm_rem & ~(16'h1 << f_i);      // registered in D_OPL (wm_nq / op_nq / op_lq) for D_WAIT
  logic [15:0] wm_nq;
  logic [3:0]  op_nq;
  logic        op_lq;
  always_comb begin
    cd_set = '0; se_set = '0; cd_clr = '0;
    if (rqh_v && rqh_e) se_set[rqh_slot] = 1'b1;
    if (rqh_v && rqh_l) cd_set[rqh_slot] = 1'b1;
    if (rqc_v && rqc_e) se_set[rqc_slot] = 1'b1;
    if (rqc_v && rqc_l) cd_set[rqc_slot] = 1'b1;
    if (bh_bvalid) begin
      if (bh_bresp != 2'b00) se_set[bfh_q[SW-1:0]] = 1'b1;
      if (bfh_q[SW])         cd_set[bfh_q[SW-1:0]] = 1'b1;
    end
    if (bc_bvalid) begin
      if (bc_bresp != 2'b00) se_set[bfc_q[SW-1:0]] = 1'b1;
      if (bfc_q[SW])         cd_set[bfc_q[SW-1:0]] = 1'b1;
    end
    if ((ds == D_DEC) && (hd.bad || hd.imm)) cd_set[h_slot] = 1'b1;
    if ((ds == D_WAIT) && rsv && !h_rd) begin                 // operation path: write qword / atomic
      if (rse) se_set[h_slot] = 1'b1;
      if (h_wr ? (op_lq && (sb_rem == 8'h0)) : (amo_wr || rse)) cd_set[h_slot] = 1'b1;
    end
    if (push) cd_clr[pp] = 1'b1;
  end
  always_ff @(posedge clk) if (push) cmem[cwb[SW-1:0]] <= {pp, cbad[pp] ? ERR_BAD_PA : serr[pp] ? ERR_DDR : 2'd0};
  assign y_cd = cmem[y_cra];

  // ---------------------------------------------------------------- atomic ALU (ng_l2_amo_alu ops 0..12), 2 clocks
  logic [63:0] amo_q, amo_r, amo_a, amo_c;
  logic [63:0] c1_sum, c1_dif, c1_inc, c1_dec, c1_A, c1_B;
  logic        c1_lts, c1_gts, c1_ltu, c1_gtu, c1_eqc, c1_az;
  wire  [31:0] a32 = hd.pa[2] ? amo_q[63:32] : amo_q[31:0];
  wire  [63:0] opA  = hd.amo64 ? amo_q : {32'h0, a32};
  wire  [63:0] opB  = hd.amo64 ? amo_a : {32'h0, amo_a[31:0]};
  wire  [63:0] opAs = hd.amo64 ? amo_q : {{32{a32[31]}}, a32};
  wire  [63:0] opBs = hd.amo64 ? amo_a : {{32{amo_a[31]}}, amo_a[31:0]};
  logic [63:0] c2_res;
  always_comb begin
    unique case (hd.amo_op)
      5'd0:    c2_res = c1_sum;                              // ADD
      5'd1:    c2_res = c1_dif;                              // SUB
      5'd2:    c2_res = c1_A & c1_B;                         // AND
      5'd3:    c2_res = c1_A | c1_B;                         // OR
      5'd4:    c2_res = c1_A ^ c1_B;                         // XOR
      5'd5:    c2_res = c1_lts ? c1_A : c1_B;                // MIN_S
      5'd6:    c2_res = c1_gts ? c1_A : c1_B;                // MAX_S
      5'd7:    c2_res = c1_ltu ? c1_A : c1_B;                // MIN_U
      5'd8:    c2_res = c1_gtu ? c1_A : c1_B;                // MAX_U
      5'd9:    c2_res = c1_B;                                // XCHG
      5'd10:   c2_res = c1_eqc ? c1_B : c1_A;                // CAS
      5'd11:   c2_res = c1_ltu ? c1_inc : 64'h0;             // INC (32-bit, CUDA wrap: a >= b -> 0)
      5'd12:   c2_res = (c1_az || c1_gtu) ? c1_B : c1_dec;   // DEC (32-bit, CUDA wrap)
      default: c2_res = c1_A;
    endcase
  end

  // ---------------------------------------------------------------- operation path outputs
  logic [63:0] op_wd_q;
  logic [7:0]  op_ws_q;
  assign c_valid  = (ds == D_OP) && !h_host;
  assign h_valid  = (ds == D_OP) && h_host;
  assign op_wr    = h_wr || (h_amo && amo_wr);
  assign op_addr  = h_amo ? {h_ta[46:3], 3'b000} : {h_ta[46:7], f_i, 3'b000};
  assign op_wstrb = h_amo ? (hd.amo64 ? 8'hFF : hd.pa[2] ? 8'hF0 : 8'h0F) : op_ws_q;
  assign op_wdata = h_amo ? amo_r : op_wd_q;

  // everything this port has issued has completed (also: no R beat / W beat in a register stage)
  wire out_empty = rfh_e && rfc_e && bfh_e && bfc_e && !rqh_v && !rqc_v && !ar_v && !aw_v && w_empty;

  // ---------------------------------------------------------------- the run iterator / dispatcher
  logic [15:0] st_rem, en_rem;
  logic [3:0]  p_q, e_q;
  logic        first_run, fmode;
  logic [46:0] fl_addr;
  wire         to_op = h_amo || !BURST || (h_wr && !WR_RUNS && h_host && hd.fb);
  wire         bf_full_t = h_host ? bfh_f : bfc_f;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ds <= D_IDLE; hg1 <= '0; hg2 <= '0; hrb <= '0; hd <= '0; h_slot <= '0;
      ar_v <= 1'b0; aw_v <= 1'b0; a_host <= 1'b0; b_addr <= '0; b_len <= '0; ar_fl <= 1'b0; aw_last <= 1'b0;
      f_act <= 1'b0; f_c3 <= 1'b0; f_slot <= '0; f_i <= '0; f_e <= '0; f_m <= 8'hFF; sb_rem <= '0;
      a_v <= 1'b0; a_h <= 1'b0; a_l <= 1'b0; a_d <= '0; a_s <= '0; wv <= 1'b0; w_h <= 1'b0; wl <= 1'b0; wd <= '0; ws <= '0;
      rqh_v <= 1'b0; rqh_fl <= 1'b0; rqh_e <= 1'b0; rqh_l <= 1'b0; rqh_slot <= '0; rqh_q <= '0; rqh_d <= '0; rih <= '0;
      rqc_v <= 1'b0; rqc_fl <= 1'b0; rqc_e <= 1'b0; rqc_l <= 1'b0; rqc_w2 <= 1'b0; rqc_slot <= '0; rqc_q <= '0;
      rqc_d <= '0; ric <= '0;
      cdone <= '0; serr <= '0; cbad <= '0; cpush_q <= 1'b0; cwb <= '0; y_cgray <= '0;
      st_rem <= '0; en_rem <= '0; p_q <= '0; e_q <= '0; first_run <= 1'b0; fmode <= 1'b0;
      fl_addr <= '0; wm_rem <= '0; amo_wr <= 1'b0; amo_q <= '0; amo_r <= '0; amo_a <= '0; amo_c <= '0;
      c1_sum <= '0; c1_dif <= '0; c1_inc <= '0; c1_dec <= '0; c1_A <= '0; c1_B <= '0;
      c1_lts <= 1'b0; c1_gts <= 1'b0; c1_ltu <= 1'b0; c1_gtu <= 1'b0; c1_eqc <= 1'b0; c1_az <= 1'b0;
      op_wd_q <= '0; op_ws_q <= '0; wm_nq <= '0; op_nq <= '0; op_lq <= 1'b0;
      n_ops <= '0; n_bad <= '0; bad_pa <= '0; n_burst <= '0; n_wfb <= '0;
    end else begin
      hg1 <= x_hgray; hg2 <= hg1;

      // ---- completion state, completion FIFO push (at most every second clock)
      cdone   <= (cdone & ~cd_clr) | cd_set;
      serr    <= serr | se_set;
      cpush_q <= push;
      if (push) begin
        cwb     <= cwb + 1'b1;
        y_cgray <= (cwb + 1'b1) ^ ((cwb + 1'b1) >> 1);
      end

      // ---- address handshakes
      if (ar_hs) ar_v <= 1'b0;
      if (aw_hs) aw_v <= 1'b0;

      // ---- W beat pipeline
      if (w_adv) begin
        wv <= a_v;
        if (a_v) begin w_h <= a_h; wd <= a_d; ws <= a_s; wl <= a_l; end
      end
      if (f_act && (!a_v || a_take)) begin
        a_v <= 1'b1; a_h <= !f_c3; a_d <= nb_d; a_s <= nb_s; a_l <= (f_i == f_e);
        f_i <= f_i + (f_c3 ? 4'd2 : 4'd1);
        if (f_i == f_e) f_act <= 1'b0;
      end else if (a_take) begin
        a_v <= 1'b0;
      end

      // ---- R beats (rready = 1) and operation-path read data -> register stage
      rqh_v <= 1'b0;
      if (bh_rvalid) begin
        rqh_v <= 1'b1; rqh_fl <= rfh_q[SW]; rqh_slot <= rfh_q[SW-1:0]; rqh_q <= rih; rqh_d <= bh_rdata;
        rqh_e <= (bh_rresp != 2'b00); rqh_l <= bh_rlast;
        rih <= bh_rlast ? 4'd0 : rih + 4'd1;
      end else if (op_rs_rd && h_host) begin
        rqh_v <= 1'b1; rqh_fl <= 1'b0; rqh_slot <= h_slot; rqh_q <= f_i; rqh_d <= rsd; rqh_e <= rse;
        rqh_l <= (f_i == 4'd15);
      end
      rqc_v <= 1'b0;
      if (bc_rvalid) begin
        rqc_v <= 1'b1; rqc_fl <= rfc_q[SW]; rqc_slot <= rfc_q[SW-1:0]; rqc_q <= {ric, 1'b0}; rqc_w2 <= 1'b1;
        rqc_d <= bc_rdata; rqc_e <= (bc_rresp != 2'b00); rqc_l <= bc_rlast;
        ric <= bc_rlast ? 3'd0 : ric + 3'd1;
      end else if (op_rs_rd && !h_host) begin
        rqc_v <= 1'b1; rqc_fl <= 1'b0; rqc_slot <= h_slot; rqc_q <= f_i; rqc_w2 <= 1'b0; rqc_d <= {64'h0, rsd};
        rqc_e <= rse; rqc_l <= (f_i == 4'd15);
      end

      // --
      unique case (ds)
        D_IDLE: if (h_ne && !quiesce) begin
          hd     <= fhead_t'(x_hd[HW-1:SW]);
          h_slot <= x_hd[SW-1:0];
          hrb    <= hrb + 1'b1;
          ds     <= D_DEC;
        end
        D_DEC: begin
          cbad[h_slot] <= hd.bad;
          serr[h_slot] <= 1'b0;
          amo_wr       <= 1'b0;
          if (hd.bad || hd.imm) begin
            if (hd.bad) begin n_bad <= n_bad + 32'd1; bad_pa <= hd.pa; end
            ds <= D_IDLE;
          end else if (to_op) begin
            if (h_wr && BURST) n_wfb <= n_wfb + 32'd1;
            ds <= D_OPW;
          end else if (h_rd) begin                              // one read burst of the whole line
            ar_v <= 1'b1; a_host <= h_host; ar_fl <= 1'b0;
            b_addr <= {h_ta[46:7], 7'd0}; b_len <= h_host ? 8'd15 : 8'd7;
            n_burst <= n_burst + 32'd1;
            ds <= D_RD;
          end else begin                                        // M_WR: the runs
            st_rem <= hd.st; en_rem <= hd.en; first_run <= 1'b1;
            fmode  <= HOST_WR_FLUSH && h_host;
            ds     <= (HOST_WR_FLUSH && h_host) ? D_FW0 : D_WR0;
          end
        end
        D_RD, D_FLR: if (ar_hs) ds <= D_IDLE;

        // ---- write runs: select (two independent encoders: runs alternate start, end), then AW + the W fetch
        D_FW0: if (out_empty) ds <= D_WR0;
        D_WR0: begin
          p_q      <= first1(st_rem);
          e_q      <= first1(en_rem);
          st_rem   <= st_rem & (st_rem - 16'd1);
          en_rem   <= en_rem & (en_rem - 16'd1);
          ds       <= D_WR1;
        end
        D_WR1: if (hd.hq[p_q]) begin                            // holey host qword (1-beat run): strobes
          f_slot <= h_slot; f_i <= p_q;                         // the fetch is idle here (D_WRB waited for it)
          ds     <= D_WS0;
        end else if (!bf_full_t && (w_empty || (f_c3 == !h_host))) begin
          aw_v    <= 1'b1; a_host <= h_host; aw_last <= (st_rem == 16'h0) && !fmode;   // st_rem: this run cleared
          b_addr  <= h_host ? {h_ta[46:7], p_q, 3'b000} : {h_ta[46:7], p_q[2:0], 4'b0000};
          b_len   <= 8'(e_q - p_q);
          f_act   <= 1'b1; f_slot <= h_slot; f_c3 <= !h_host; f_m <= 8'hFF;
          f_i     <= h_host ? p_q : {p_q[2:0], 1'b0};
          f_e     <= h_host ? e_q : {e_q[2:0], 1'b0};
          if (first_run) fl_addr <= h_host ? {h_ta[46:7], p_q, 3'b000} : {h_ta[46:7], p_q[2:0], 4'b0000};
          first_run <= 1'b0;
          n_burst <= n_burst + 32'd1;
          ds      <= D_WRB;
        end
        D_WS0: begin                                            // wbuf[{slot, f_i[3:2]}] is valid: the qword's strobes
          sb_rem <= qstb;
          ds     <= D_WS1;
        end
        D_WS1: if (!bf_full_t && (w_empty || !f_c3)) begin      // one 1-beat write per byte run (host only)
          aw_v    <= 1'b1; a_host <= 1'b1; aw_last <= (st_rem == 16'h0) && !fmode && (sb_nxt == 8'h0);
          b_addr  <= {h_ta[46:7], p_q, 3'b000};
          b_len   <= 8'd0;
          f_act   <= 1'b1; f_slot <= h_slot; f_c3 <= 1'b0; f_m <= sb_run;
          f_i     <= p_q;
          f_e     <= p_q;
          sb_rem  <= sb_nxt;
          if (first_run) fl_addr <= {h_ta[46:7], p_q, 3'b000};
          first_run <= 1'b0;
          n_burst <= n_burst + 32'd1;
          ds      <= D_WRB;
        end
        D_WRB: if (!aw_v && !f_act)
          ds <= (sb_rem != 8'h0) ? D_WS1 : (st_rem != 16'h0) ? D_WR0 : (fmode ? D_FW1 : D_IDLE);
        D_FW1: if (out_empty) begin                             // every B of the runs arrived: the flush read
          ar_v <= 1'b1; a_host <= 1'b1; ar_fl <= 1'b1; b_addr <= fl_addr; b_len <= 8'd0;
          n_burst <= n_burst + 32'd1;
          ds <= D_FLR;
        end

        // ---- 64-bit operation path (/ code): nothing else of this port in flight
        D_OPW: if (out_empty) begin
          f_slot <= h_slot;
          f_i    <= h_wr ? first1(hd.wm) : 4'd0;
          wm_rem <= hd.wm;
          ds     <= D_OPL;
        end
        D_OPL: begin                                            // wbuf[{slot, f_i[3:2]}] is valid: data / operands
          op_wd_q <= qsel; op_ws_q <= qstb;
          wm_nq <= wm_nxt; op_nq <= first1(wm_nxt); op_lq <= (wm_nxt == 16'h0);
          if (h_amo && !amo_wr) begin amo_a <= x_wde[63:0]; amo_c <= x_wde[127:64]; end
          if (h_wr && hd.hq[f_i]) begin                         // holey host qword, one operation per run
            sb_rem <= qstb;
            ds     <= D_OPS;
          end else begin
            ds     <= D_OP;
          end
        end
        D_OPS: begin
          op_ws_q <= sb_run;
          sb_rem  <= sb_nxt;
          ds      <= D_OP;
        end
        D_OP: if (h_host ? h_ready : c_ready) begin
          n_ops <= n_ops + 32'd1;
          ds <= D_WAIT;
        end
        D_WAIT: if (rsv) begin
          if (h_rd) begin
            f_i <= f_i + 4'd1;
            ds  <= (f_i == 4'd15) ? D_IDLE : D_OP;
          end else if (h_wr) begin
            if (sb_rem != 8'h0) begin                           // next byte run of the same qword
              ds <= D_OPS;
            end else begin
              wm_rem <= wm_nq;
              f_i    <= op_nq;
              ds     <= op_lq ? D_IDLE : D_OPL;
            end
          end else if (!amo_wr) begin                           // atomic: old value read
            amo_q <= rsd;
            ds <= rse ? D_IDLE : D_CALC1;                       // a failed read: no write
          end else begin
            ds <= D_IDLE;
          end
        end
        D_CALC1: begin
          c1_sum <= opA + opB;  c1_dif <= opA - opB;  c1_inc <= opA + 64'd1;  c1_dec <= opA - 64'd1;
          c1_A   <= opA;        c1_B   <= opB;
          c1_lts <= ($signed(opAs) < $signed(opBs));  c1_gts <= ($signed(opAs) > $signed(opBs));
          c1_ltu <= (opA < opB);                      c1_gtu <= (opA > opB);
          c1_eqc <= hd.amo64 ? (amo_q == amo_c) : (a32 == amo_c[31:0]);
          c1_az  <= (a32 == 32'h0);
          ds <= D_CALC2;
        end
        D_CALC2: begin
          amo_r  <= hd.amo64 ? c2_res : (hd.pa[2] ? {c2_res[31:0], 32'h0} : {32'h0, c2_res[31:0]});
          amo_wr <= 1'b1;
          ds <= D_OP;
        end
        default: ds <= D_IDLE;
      endcase
    end
  end

`ifndef SYNTHESIS
  always @(posedge clk) if (rst_n) begin
    if (bh_rvalid && rfh_e) $display("ERROR ng_rsp_fab_l2: host R beat without an outstanding read");
    if (bc_rvalid && rfc_e) $display("ERROR ng_rsp_fab_l2: C3 R beat without an outstanding read");
    if (bh_bvalid && bfh_e) $display("ERROR ng_rsp_fab_l2: host B without an outstanding write");
    if (bc_bvalid && bfc_e) $display("ERROR ng_rsp_fab_l2: C3 B without an outstanding write");
    if ((ar_hs && (a_host ? rfh_f : rfc_f)) || (aw_hs && (a_host ? bfh_f : bfc_f)))
      $display("ERROR ng_rsp_fab_l2: order FIFO overflow");
    if (bh_rvalid && op_rs_rd && h_host) $display("ERROR ng_rsp_fab_l2: R beat during the operation path");
    if (bc_rvalid && op_rs_rd && !h_host) $display("ERROR ng_rsp_fab_l2: R beat during the operation path");
    // the XDMA bridge needs a contiguous byte strobe per host write beat
    if (bh_wvalid && ((bh_wstrb & ~lowrun(bh_wstrb)) != 8'h0))
      $display("ERROR ng_rsp_fab_l2: host W beat with a holey strobe %02h", bh_wstrb);
    if (h_valid && op_wr && ((op_wstrb & ~lowrun(op_wstrb)) != 8'h0))
      $display("ERROR ng_rsp_fab_l2: host write operation with a holey strobe %02h", op_wstrb);
  end
`endif
endmodule

// ================================================================================================================
// ng_rsp_mport1 -- compatibility "mport" for MEM_OUT = 1: the serial fabric port of ng_sp_top (one request
// in flight, mrsp_t with the 1024-bit line) onto the slot port of ng_rsp_fab_ts.  The request passes straight
// through; the answer pulse reads the line of its slot (rbuf) into the mrsp_t register in the same clock and frees
// the slot (mem_free is ignored by fab_ts for non-M_RD slots), the SP sees mem_rsp_valid one clock later (+10 ns).
// rdata reads 0 for a non-M_RD answer and for ERR_BAD_PA, old reads 0 except for an atomic (as).
// SP reset: ng_sp_top's fabric arbiter (fa_st) is reset by rst_n only -- after a datapath reset it stays in A_WAIT
// (fa_drop set) until "the" answer arrives.  fab_ts never delivers the answer of a request in flight at sp_rst (orphan),
// so the adapter answers it itself: a request accepted and not yet answered when sp_rst is seen gets one dummy answer
// (err 0, data 0) in the first clock after sp_rst, which the SP drops (fa_drop was set during the reset, sp_rst covers
// its registered crst_n).  The SP's next request then waits in fab_ts behind the orphan if it is on the same line.
module ng_rsp_mport1
  import ng_lsu_pkg::*;
#(
  parameter int SW = 3
) (
  input  logic          clk,              // clk_ts
  input  logic          rst_n,
  input  logic          sp_rst,           // = ng_rsp_fab_ts sp_rst
  // ng_sp_top fabric port
  input  logic          sp_req_valid,
  output logic          sp_req_ready,
  input  mreq_t         sp_req,
  output logic          sp_rsp_valid,
  output mrsp_t         sp_rsp,
  // ng_rsp_fab_ts request port
  output logic          f_req_valid,
  input  logic          f_req_ready,
  output mreq_t         f_req,
  input  logic          f_rsp_valid,
  input  logic [SW-1:0] f_rsp_slot,
  input  logic [1:0]    f_rsp_err,
  input  logic          f_rsp_rd,
  input  logic [63:0]   f_rsp_old,
  output logic [SW-1:0] f_rd_slot,
  input  logic [1023:0] f_rd_data,
  output logic          f_free
);
  logic is_amo;           // the request in flight is an atomic (the SP holds it until the answer)
  logic pend;             // a request was accepted and not yet answered
  logic fake;             // pend was orphaned by sp_rst: dummy answer after the reset
  assign f_req_valid  = sp_req_valid;
  assign sp_req_ready = f_req_ready;
  assign f_req        = sp_req;
  assign f_rd_slot    = f_rsp_slot;
  assign f_free       = f_rsp_valid;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sp_rsp_valid <= 1'b0; sp_rsp <= '0; is_amo <= 1'b0; pend <= 1'b0; fake <= 1'b0;
    end else begin
      sp_rsp_valid <= f_rsp_valid || (fake && !sp_rst);
      if (sp_req_valid && f_req_ready) begin
        is_amo <= (sp_req.kind == M_AMO);
        pend   <= 1'b1;
      end else if (f_rsp_valid) begin
        pend <= 1'b0;
      end else if (sp_rst && pend) begin
        pend <= 1'b0; fake <= 1'b1;
      end
      if (fake && !sp_rst) fake <= 1'b0;
      if (f_rsp_valid) begin
        sp_rsp.rdata <= (f_rsp_rd && (f_rsp_err != 2'd1)) ? f_rd_data : '0;
        sp_rsp.old   <= is_amo ? f_rsp_old : 64'h0;
        sp_rsp.err   <= f_rsp_err;
      end else if (fake && !sp_rst) begin
        sp_rsp <= '0;
      end
    end
  end
endmodule

// ================================================================================================================
module ng_rsp_mmu_ts (
  input  logic        clk,                // clk_ts
  input  logic        rst_n,
  // SP MMU client (ng_sp_top mmu_*)
  input  logic        req_valid,
  output logic        req_ready,
  input  logic [3:0]  req_vmid,
  input  logic [63:0] req_va,
  input  logic [1:0]  req_acc,
  input  logic        req_atomic,
  output logic        resp_valid,
  input  logic        resp_ready,
  output logic [2:0]  resp_fault,
  output logic        resp_sparse,
  output logic [47:0] resp_pa,
  output logic [63:0] resp_pte,
  // to / from clk_l2
  output logic        req_tog_o,
  output logic [3:0]  x_vmid,
  output logic [63:0] x_va,
  output logic [1:0]  x_acc,
  output logic        x_atomic,
  input  logic        ack_tog_i,
  input  logic [2:0]  y_fault,
  input  logic        y_sparse,
  input  logic [47:0] y_pa,
  input  logic [63:0] y_pte,
  // debug
  output logic [1:0]  dbg_st,
  output logic [31:0] n_req,
  output logic [31:0] n_flt,
  output logic [63:0] last_va,
  output logic [2:0]  last_fault
);
  (* ASYNC_REG = "TRUE" *) logic a1, a2;
  typedef enum logic [1:0] {S_IDLE, S_WAIT, S_RESP} st_t;
  st_t st;
  assign dbg_st     = st;
  assign req_ready  = (st == S_IDLE);
  assign resp_valid = (st == S_RESP);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; a1 <= 1'b0; a2 <= 1'b0; req_tog_o <= 1'b0;
      x_vmid <= '0; x_va <= '0; x_acc <= '0; x_atomic <= 1'b0;
      resp_fault <= '0; resp_sparse <= 1'b0; resp_pa <= '0; resp_pte <= '0;
      n_req <= '0; n_flt <= '0; last_va <= '0; last_fault <= '0;
    end else begin
      a1 <= ack_tog_i; a2 <= a1;
      unique case (st)
        S_IDLE: if (req_valid) begin
          x_vmid <= req_vmid; x_va <= req_va; x_acc <= req_acc; x_atomic <= req_atomic;
          req_tog_o <= ~req_tog_o;
          n_req <= n_req + 32'd1; last_va <= req_va;
          st <= S_WAIT;
        end
        S_WAIT: if (a2 == req_tog_o) begin
          resp_fault <= y_fault; resp_sparse <= y_sparse; resp_pa <= y_pa; resp_pte <= y_pte;
          last_fault <= y_fault;
          if (y_fault != 3'd0) n_flt <= n_flt + 32'd1;
          st <= S_RESP;
        end
        default: if (resp_ready) st <= S_IDLE;
      endcase
    end
  end
endmodule

module ng_rsp_mmu_l2 (
  input  logic        clk,                // clk_l2
  input  logic        rst_n,
  input  logic        req_tog_i,          // asynchronous
  input  logic [3:0]  x_vmid,
  input  logic [63:0] x_va,
  input  logic [1:0]  x_acc,
  input  logic        x_atomic,
  output logic        ack_tog_o,
  output logic [2:0]  y_fault,
  output logic        y_sparse,
  output logic [47:0] y_pa,
  output logic [63:0] y_pte,
  // translation client (ng_rsp_mmuarb requester 1)
  output logic        t_req_valid,
  input  logic        t_req_ready,
  output logic [3:0]  t_req_vmid,
  output logic [63:0] t_req_va,
  output logic [1:0]  t_req_acc,
  output logic        t_req_atomic,
  input  logic        t_resp_valid,
  output logic        t_resp_ready,
  input  logic [2:0]  t_resp_fault,
  input  logic        t_resp_sparse,
  input  logic [47:0] t_resp_pa,
  input  logic [63:0] t_resp_pte,
  output logic [1:0]  dbg_st
);
  (* ASYNC_REG = "TRUE" *) logic r1, r2;
  typedef enum logic [1:0] {S_IDLE, S_REQ, S_RESP} st_t;
  st_t st;
  assign dbg_st       = st;
  assign t_req_valid  = (st == S_REQ);
  assign t_req_vmid   = x_vmid;
  assign t_req_va     = x_va;
  assign t_req_acc    = x_acc;
  assign t_req_atomic = x_atomic;
  assign t_resp_ready = (st == S_RESP);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; r1 <= 1'b0; r2 <= 1'b0; ack_tog_o <= 1'b0;
      y_fault <= '0; y_sparse <= 1'b0; y_pa <= '0; y_pte <= '0;
    end else begin
      r1 <= req_tog_i; r2 <= r1;
      unique case (st)
        S_IDLE: if (r2 != ack_tog_o) st <= S_REQ;
        S_REQ:  if (t_req_ready) st <= S_RESP;
        default: if (t_resp_valid) begin
          y_fault <= t_resp_fault; y_sparse <= t_resp_sparse; y_pa <= t_resp_pa; y_pte <= t_resp_pte;
          ack_tog_o <= r2;
          st <= S_IDLE;
        end
      endcase
    end
  end
endmodule

// ================================================================================================================
module ng_rsp_flt_ts (
  input  logic        clk,                // clk_ts
  input  logic        rst_n,
  input  logic        flt_valid,          // ng_sp_top ext_flt_* (single-cycle report)
  input  logic [3:0]  flt_vmid,
  input  logic [63:0] flt_va,
  input  logic [1:0]  flt_acc,
  output logic        tog_o,
  output logic [3:0]  x_vmid,
  output logic [63:0] x_va,
  output logic [1:0]  x_acc,
  input  logic        ack_tog_i,
  // debug
  output logic        last_valid,
  output logic [63:0] last_va,
  output logic [3:0]  last_vmid,
  output logic [1:0]  last_acc
);
  (* ASYNC_REG = "TRUE" *) logic a1, a2;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      a1 <= 1'b0; a2 <= 1'b0; tog_o <= 1'b0; x_vmid <= '0; x_va <= '0; x_acc <= '0;
      last_valid <= 1'b0; last_va <= '0; last_vmid <= '0; last_acc <= '0;
    end else begin
      a1 <= ack_tog_i; a2 <= a1;
      if (flt_valid) begin
        last_valid <= 1'b1; last_va <= flt_va; last_vmid <= flt_vmid; last_acc <= flt_acc;
        if (a2 == tog_o) begin              // previous report crossed: send this one
          x_vmid <= flt_vmid; x_va <= flt_va; x_acc <= flt_acc;
          tog_o <= ~tog_o;
        end
      end
    end
  end
endmodule

module ng_rsp_flt_l2 (
  input  logic        clk,                // clk_l2
  input  logic        rst_n,
  input  logic        tog_i,              // asynchronous
  output logic        ack_tog_o,
  output logic        pend                // a report is waiting (x_* quasi-static); cleared by take
  ,input logic        take
);
  (* ASYNC_REG = "TRUE" *) logic r1, r2;
  assign pend = (r2 != ack_tog_o);
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      r1 <= 1'b0; r2 <= 1'b0; ack_tog_o <= 1'b0;
    end else begin
      r1 <= tog_i; r2 <= r1;
      if (pend && take) ack_tog_o <= r2;
    end
  end
endmodule
