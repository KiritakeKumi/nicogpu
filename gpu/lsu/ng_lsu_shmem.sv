// ng_lsu_shmem.sv -- banked shared-memory array with bank-conflict serialisation.
//
// 32 banks x 32 bit (design doc: 64 KiB max per workgroup in the half-core's
// URAM).  Dword d lives in bank d % 32, row d / 32.  The LSU presents one
// "round" per request pulse for ONE GROUP of LPB lanes (lanes grp*LPB .. grp*LPB+LPB-1; the LSU
// owns the group bookkeeping): a pending-lane mask, per-lane byte address and access size
// (1..16 B, byte-precise, unaligned allowed -- ISA has no alignment requirement), store data.
// The unit serves the maximal PREFIX of pending lanes of the group (ascending lane order) whose
// dword footprints do not collide:
//   * loads: two lanes reading the SAME dword share the bank access
//             (broadcast); different dwords in one bank collide;
//   * stores: any two lanes touching the same bank collide (the later lane
//             must win byte-for-byte, so overlapping stores are ordered by
//             never sharing a round).
// A prefix (not an arbitrary subset) is served so that store ordering across
// lanes stays exactly the model's ascending-lane order for every overlap
// pattern, including multi-dword and unaligned accesses; groups are served in ascending order
// by the LSU, so the order is global.
// rsp_sel = lanes of the group served, rsp_rdata = 16 B per lane (loads; bytes beyond the access
// size are 0).  The LSU re-presents the remaining lanes until pend is empty, so (LPB = 8) a stride-1
// b32 access of 32 lanes takes 4 rounds, a stride-32 pattern 32 rounds.
//
// LPB (lanes per beat, default 8; 32 = the former single-round 32-lane behaviour) scales the
// scan, the write crossbar and the read crossbar.
//
// Pipeline (request pulse in cycle 0; one round at a time -- the LSU waits for the response):
//   0      request inputs captured
//   1      footprint decode, bank-conflict scan -> sel / per-bank row / nmax = most dwords of a served
//          lane (registered); store data window shift (registered)
//   2..    bank RAM access (read-first, registered RAM output).  Stores: one write pass per dword index
//          j < nmax (pass j writes dword j of every served lane into bank (b0 + j) mod 32; the dword
//          window of a lane is 1..5 dwords, the usual case is 1 pass).  Loads: one RAM cycle.
//   then   loads only: nmax extraction cycles (cycle j takes dword j of every served lane out of the
//          held bank outputs: one 32:1 mux per lane instead of five)
//   last   byte align (shift by the byte offset, zero beyond the access size) -> registered response
// Per-round latency is therefore 5 + nmax cycles (load) / 4 + nmax (store) instead of 4 cycles of the
// one-pass version; in exchange the write and read crossbars are 1/5 of the size.

// ng_lsu_shmem_bank -- one shared-memory bank: single-port, read-first, byte-write RAM (zero-initialised).
module ng_lsu_shmem_bank #(
  parameter int DEPTH = 512
) (
  input  logic                     clk,
  input  logic                     en,
  input  logic [$clog2(DEPTH)-1:0] addr,
  input  logic [3:0]               we,
  input  logic [31:0]              wdata,
  output logic [31:0]              rdata
);
  (* ram_style = "block" *) logic [31:0] m [DEPTH];
  always @(posedge clk) begin
    if (en) begin
      rdata <= m[addr];
      for (int i = 0; i < 4; i++)
        if (we[i]) m[addr][8*i +: 8] <= wdata[8*i +: 8];
    end
  end
  initial begin
    for (int r = 0; r < DEPTH; r++) m[r] = 32'd0;
  end
endmodule

// ng_lsu_shmem_scan -- which pending lanes of the group are served in this round, and the row each
// bank is addressed with.
//
// Lane l covers the dwords d0[l] .. d0[l]+nd[l]-1 (nd 1..5) = the banks (b0 + j) mod 32.  Because the
// served lanes are a PREFIX of the pending lanes, lane l is served iff it and every earlier pending
// lane are conflict-free against all EARLIER pending lanes, i.e. a purely pairwise relation:
//   delta = d0[l] - d0[k] (k < l); two dwords u of k, v of l fall into one bank iff u - v = 32 m;
//   with u = d0[k] + a, v = d0[l] + b (a < nd[k], b < nd[l]) that needs delta = (a - b) + 32 m, so only
//   delta mod 32 in [0, nd[k]-1] (t = delta mod 32) or in [33 - nd[l], 31] (t = delta mod 32 - 32) can
//   collide, and the two lanes read the SAME dword iff delta == t, i.e. delta[..:5] == 0 (resp. -1).
//   store: any bank overlap conflicts; load: only a bank overlap on a different dword.
// The per-bank row = row of the FIRST pending lane touching the bank (served lanes are earlier than the
// unserved ones, so this is the served lane's row whenever the bank is used).
module ng_lsu_shmem_scan #(
  parameter int NB  = 32,
  parameter int AW  = 16,
  parameter int LPB = 8
) (
  input  logic                         wr,
  input  logic [LPB-1:0]               pend,
  input  logic [AW-1:0]                addr [LPB],
  input  logic [4:0]                   nb,          // bytes - 1
  output logic [LPB-1:0]               sel,
  output logic [AW-2-$clog2(NB)-1:0]   brow [NB]
);
  localparam int BKW = $clog2(NB);
  localparam int DW  = AW - 2;
  localparam int RW  = DW - BKW;

  logic [DW-1:0]  d0 [LPB];
  logic [BKW-1:0] b0 [LPB];
  logic [RW-1:0]  rh0 [LPB];
  logic [RW-1:0]  rh1 [LPB];
  logic [2:0]     nd [LPB];
  always_comb begin
    for (int l = 0; l < LPB; l++) begin
      d0[l]  = addr[l][AW-1:2];
      b0[l]  = addr[l][2 +: BKW];
      rh0[l] = addr[l][AW-1:2+BKW];
      rh1[l] = addr[l][AW-1:2+BKW] + RW'(1);
      nd[l]  = 3'((int'(addr[l][1:0]) + int'(nb) + 1 + 3) / 4);
    end
  end

  // pairwise conflicts of lane l against earlier pending lanes
  logic [LPB-1:0] bad;                      // pending lane that conflicts with an earlier pending lane
  always_comb begin
    logic signed [DW:0] delta;
    logic [BKW-1:0]     dlo;
    logic [DW-BKW:0]    dhi;
    logic               ov_lo, ov_hi, c;
    for (int l = 0; l < LPB; l++) begin
      c = 1'b0;
      for (int k = 0; k < l; k++) begin
        delta = $signed({1'b0, d0[l]}) - $signed({1'b0, d0[k]});
        dlo   = delta[BKW-1:0];
        dhi   = delta[DW:BKW];
        ov_lo = (int'(dlo) < int'(nd[k]));                           // t = dlo      (0..4)
        ov_hi = (int'(dlo) + int'(nd[l]) >= 33) ;                    // t = dlo - 32 (-4..-1)
        if (pend[k]) begin
          if (wr) c = c | ov_lo | ov_hi;
          else    c = c | (ov_lo & (dhi != '0)) | (ov_hi & (dhi != '1));
        end
      end
      bad[l] = pend[l] & c;
    end
  end
  always_comb begin
    logic run;
    run = 1'b0;
    for (int l = 0; l < LPB; l++) begin
      run = run | bad[l];
      sel[l] = pend[l] & ~run;
    end
  end

  // row each bank is addressed with: row of the first pending lane that touches it
  for (genvar b = 0; b < NB; b++) begin : g_row
    logic [RW-1:0] fr;
    always_comb begin
      logic [BKW-1:0] jj;
      logic           t, run;
      logic [RW-1:0]  rr;
      run = 1'b0;
      fr  = '0;
      for (int l = 0; l < LPB; l++) begin
        jj = BKW'(b) - b0[l];
        t  = pend[l] & (jj < BKW'(nd[l]));
        rr = (BKW'(b) < b0[l]) ? rh1[l] : rh0[l];
        fr = fr | (rr & {RW{t & ~run}});
        run = run | t;
      end
    end
    assign brow[b] = fr;
  end
endmodule

// ng_lsu_shmem_wx -- write crossbar of ONE dword pass: dword `pj` of every served lane goes to bank
// (b0 + pj) mod 32.  Served store lanes never share a bank, so at most one lane writes a bank: the lane is
// encoded per bank and a plain 8:1 mux picks its dword.
module ng_lsu_shmem_wx #(
  parameter int NB  = 32,
  parameter int LPB = 8
) (
  input  logic                   wr,
  input  logic [2:0]             pj,
  input  logic [LPB-1:0]         sel,
  input  logic [$clog2(NB)-1:0]  b0 [LPB],
  input  logic [2:0]             nd [LPB],
  input  logic [159:0]           wwin [LPB],
  input  logic [19:0]            wbm  [LPB],
  output logic [NB-1:0]          we,
  output logic [3:0]             wbe  [NB],
  output logic [31:0]            wdat [NB]
);
  localparam int BKW = $clog2(NB);
  localparam int LW  = ng_noc_pkg::clog2_1(LPB);
  logic [31:0]     wd [LPB];       // dword pj of each lane
  logic [3:0]      wm [LPB];       // its byte mask
  logic [BKW-1:0]  tb [LPB];       // target bank
  logic [LPB-1:0]  act;
  always_comb begin
    for (int l = 0; l < LPB; l++) begin
      wd[l]  = '0;
      wm[l]  = '0;
      for (int j = 0; j < 5; j++)
        if (int'(pj) == j) begin wd[l] = wwin[l][32*j +: 32]; wm[l] = wbm[l][4*j +: 4]; end
      tb[l]  = b0[l] + BKW'(pj);
      act[l] = wr & sel[l] & (int'(pj) < int'(nd[l]));
    end
  end
  always_comb begin
    logic [LW-1:0] idx;
    for (int b = 0; b < NB; b++) begin
      idx = '0;
      we[b] = 1'b0;
      for (int l = 0; l < LPB; l++)
        if (act[l] && tb[l] == BKW'(b)) begin idx = LW'(l); we[b] = 1'b1; end
      wdat[b] = wd[idx];
      wbe[b]  = we[b] ? wm[idx] : 4'd0;
    end
  end
endmodule

// ng_lsu_shmem_rx -- read crossbar of ONE dword pass: lane l takes dword xj of its footprint, i.e. the output of
// bank (b0 + xj) mod 32 (zero for unserved lanes).
module ng_lsu_shmem_rx #(
  parameter int NB  = 32,
  parameter int LPB = 8
) (
  input  logic [31:0]            bq [NB],
  input  logic [LPB-1:0]         sel,
  input  logic [$clog2(NB)-1:0]  b0 [LPB],
  input  logic [2:0]             xj,
  output logic [31:0]            dw [LPB]
);
  localparam int BKW = $clog2(NB);
  always_comb begin
    for (int l = 0; l < LPB; l++)
      dw[l] = sel[l] ? bq[BKW'(b0[l] + BKW'(xj))] : 32'd0;
  end
endmodule

module ng_lsu_shmem #(
  parameter int NB    = 32,
  parameter int BYTES = 65536,
  parameter int AW    = 16,         // byte address width (log2 BYTES)
  parameter int LPB   = 8           // lanes per beat (round)
) (
  input  logic                clk,
  input  logic                rst_n,

  input  logic                req_valid,     // one-cycle pulse (not while a round is in flight)
  input  logic                req_wr,
  input  logic [(LPB < 32 ? $clog2(32 / LPB) : 1)-1:0] req_grp,   // group index (echoed in rsp_grp)
  input  logic [LPB-1:0]      req_pend,
  input  logic [AW-1:0]       req_addr [LPB],
  input  logic [4:0]          req_nb,        // access bytes - 1 (0..15)
  input  logic [127:0]        req_wdata [LPB],

  output logic                rsp_valid,
  output logic [(LPB < 32 ? $clog2(32 / LPB) : 1)-1:0] rsp_grp,
  output logic [LPB-1:0]      rsp_sel,
  output logic [127:0]        rsp_rdata [LPB],

  // scrub: clr_en zeroes row clr_row (one dword row = 128 B) of every bank in this clock; only
  // while no round is in flight (the work distributor drives it between dispatches).
  input  logic                clr_en,
  input  logic [AW-2-$clog2(NB)-1:0] clr_row
);
  localparam int DEPTH = BYTES / 4 / NB;
  localparam int BKW   = $clog2(NB);
  localparam int DW    = AW - 2;           // dword address width
  localparam int RW    = DW - BKW;         // row address width
  localparam int GW    = (LPB < 32) ? $clog2(32 / LPB) : 1;

`ifndef NG_SHMEM_BANK_RAM
`ifndef SYNTHESIS
  logic [31:0] mem [NB][DEPTH];
`endif
`endif

  // ---------------------------------------------------------------- stage 0: capture
  logic              c_v, c_wr;
  logic [GW-1:0]     c_grp;
  logic [LPB-1:0]    c_pend;
  logic [AW-1:0]     c_addr [LPB];
  logic [4:0]        c_nb;
  logic [127:0]      c_wdata [LPB];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      c_v <= 1'b0; c_wr <= 1'b0; c_grp <= '0; c_pend <= '0; c_nb <= '0;
      for (int l = 0; l < LPB; l++) begin c_addr[l] <= '0; c_wdata[l] <= '0; end
    end else begin
      c_v <= req_valid;
      if (req_valid) begin
        c_wr <= req_wr; c_grp <= req_grp; c_pend <= req_pend; c_nb <= req_nb;
        for (int l = 0; l < LPB; l++) begin c_addr[l] <= req_addr[l]; c_wdata[l] <= req_wdata[l]; end
      end
    end
  end

  // ---------------------------------------------------------------- stage 1: decode + conflict scan
  logic [BKW-1:0] b0 [LPB];
  logic [1:0]     bo [LPB];
  logic [2:0]     nd [LPB];
  logic [LPB-1:0] sel;
  logic [RW-1:0]  brow [NB];               // bank row
  logic [159:0]   wwin [LPB];              // store data shifted to the byte offset in a 5-dword window
  logic [19:0]    wbm  [LPB];              // byte mask of the window
  logic [2:0]     nmax;                    // most dwords of a served lane

  always_comb begin
    for (int l = 0; l < LPB; l++) begin
      b0[l]  = c_addr[l][2 +: BKW];
      bo[l]  = c_addr[l][1:0];
      nd[l]  = 3'((int'(c_addr[l][1:0]) + int'(c_nb) + 1 + 3) / 4);
      wwin[l] = 160'(c_wdata[l]) << (8 * int'(c_addr[l][1:0]));
      wbm[l]  = 20'((32'd1 << (int'(c_nb) + 1)) - 32'd1) << int'(c_addr[l][1:0]);
    end
    nmax = 3'd1;
    for (int l = 0; l < LPB; l++)
      if (sel[l] && nd[l] > nmax) nmax = nd[l];
  end

  ng_lsu_shmem_scan #(.NB(NB), .AW(AW), .LPB(LPB)) u_scan (
    .wr(c_wr), .pend(c_pend), .addr(c_addr), .nb(c_nb), .sel(sel), .brow(brow));

  logic              s_wr;
  logic [GW-1:0]     s_grp;
  logic [LPB-1:0]    s_sel;
  logic [4:0]        s_nb;
  logic [2:0]        s_nmax;
  logic [RW-1:0]     s_brow [NB];
  logic [BKW-1:0]    s_b0 [LPB];
  logic [1:0]        s_bo [LPB];
  logic [2:0]        s_nd [LPB];
  logic [159:0]      s_wwin [LPB];
  logic [19:0]       s_wbm [LPB];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_wr <= 1'b0; s_grp <= '0; s_sel <= '0; s_nb <= '0; s_nmax <= 3'd1;
      for (int b = 0; b < NB; b++) s_brow[b] <= '0;
      for (int l = 0; l < LPB; l++) begin
        s_b0[l] <= '0; s_bo[l] <= '0; s_nd[l] <= '0; s_wwin[l] <= '0; s_wbm[l] <= '0;
      end
    end else begin
      if (c_v) begin
        s_wr <= c_wr; s_grp <= c_grp; s_sel <= sel; s_nb <= c_nb; s_nmax <= nmax;
        for (int b = 0; b < NB; b++) s_brow[b] <= brow[b];
        for (int l = 0; l < LPB; l++) begin
          s_b0[l] <= b0[l]; s_bo[l] <= bo[l]; s_nd[l] <= nd[l]; s_wwin[l] <= wwin[l]; s_wbm[l] <= wbm[l];
        end
      end
    end
  end

  // --------------------------------------------------------------
  // ph2: RAM cycle(s) (stores: one per dword pass pj), ph3: load extraction (one per dword xj),
  // ph4: byte align + response.  s_* stay stable for the whole round.
  logic       ph2, ph3, ph4;
  logic [2:0] pj;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ph2 <= 1'b0; ph3 <= 1'b0; ph4 <= 1'b0; pj <= '0;
    end else begin
      ph4 <= 1'b0;
      if (c_v) begin
        ph2 <= 1'b1; pj <= '0;
      end else if (ph2) begin
        if (s_wr) begin
          if (int'(pj) + 1 >= int'(s_nmax)) begin ph2 <= 1'b0; ph4 <= 1'b1; end
          else pj <= pj + 3'd1;
        end else begin
          ph2 <= 1'b0; ph3 <= 1'b1; pj <= '0;
        end
      end else if (ph3) begin
        if (int'(pj) + 1 >= int'(s_nmax)) begin ph3 <= 1'b0; ph4 <= 1'b1; end
        else pj <= pj + 3'd1;
      end
    end
  end

  // ---------------------------------------------------------------- stage 2: write crossbar + RAM
  logic [NB-1:0]   we;
  logic [3:0]      wbe  [NB];
  logic [31:0]     wdat [NB];
  ng_lsu_shmem_wx #(.NB(NB), .LPB(LPB)) u_wx (
    .wr(s_wr), .pj(pj), .sel(s_sel), .b0(s_b0), .nd(s_nd), .wwin(s_wwin), .wbm(s_wbm),
    .we(we), .wbe(wbe), .wdat(wdat));

  logic [31:0]    bq [NB];                 // bank read data (registered inside the RAM, held)

  // One single-port, read-first, byte-write RAM per bank (inferred block RAM; 512 x 32 each).
`ifdef SYNTHESIS
 `define NG_SHMEM_BANK_RAM
`endif
`ifdef NG_SHMEM_BANK_RAM
  // Synthesis structure: one small RAM module per bank (a single 3-D `mem` array with 32 per-bank
  // write blocks makes Vivado's RAM inference run for tens of minutes).
  for (genvar gb = 0; gb < NB; gb++) begin : g_bank
    ng_lsu_shmem_bank #(.DEPTH(DEPTH)) u_bank (
      .clk(clk), .en(ph2 | clr_en), .addr(clr_en ? clr_row : s_brow[gb]),
      .we(clr_en ? 4'hf : (we[gb] ? wbe[gb] : 4'b0)),
      .wdata(clr_en ? 32'd0 : wdat[gb]), .rdata(bq[gb]));
  end
`else
  // Simulation (default): behaviourally identical array twin; keeps the testbench back door
  // `u_shmem.mem[bank][row]` (tb_lsu_vec) working.  NG_SHMEM_BANK_RAM selects the RAM-module path
  // in simulation as well (done for tb_lsu_shmem / tb_lsu_dir addendum).
  for (genvar gb = 0; gb < NB; gb++) begin : g_bank
    always @(posedge clk) begin
      if (clr_en) begin
        mem[gb][clr_row] <= 32'd0;
      end else if (ph2) begin
        bq[gb] <= mem[gb][s_brow[gb]];
        if (we[gb]) begin
          for (int i = 0; i < 4; i++)
            if (wbe[gb][i]) mem[gb][s_brow[gb]][8*i +: 8] <= wdat[gb][8*i +: 8];
        end
      end
    end
  end
  initial begin
    for (int b = 0; b < NB; b++) for (int r = 0; r < DEPTH; r++) mem[b][r] = 32'd0;
  end
`endif

  // ---------------------------------------------------------------- stage 3: read extraction + align
  logic [31:0] rxw [LPB];
  ng_lsu_shmem_rx #(.NB(NB), .LPB(LPB)) u_rx (.bq(bq), .sel(s_sel), .b0(s_b0), .xj(pj), .dw(rxw));

  logic [31:0] rw [LPB][5];                 // dword window of each lane
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int l = 0; l < LPB; l++) for (int j = 0; j < 5; j++) rw[l][j] <= '0;
    end else if (c_v) begin
      for (int l = 0; l < LPB; l++) for (int j = 0; j < 5; j++) rw[l][j] <= '0;
    end else if (ph3) begin
      for (int l = 0; l < LPB; l++)
        for (int j = 0; j < 5; j++)
          if (int'(pj) == j) rw[l][j] <= rxw[l];
    end
  end

  logic [127:0] rd_al [LPB];
  always_comb begin
    logic [159:0] wflat;
    logic [159:0] sh;
    logic [127:0] r;
    for (int l = 0; l < LPB; l++) begin
      for (int j = 0; j < 5; j++) wflat[32*j +: 32] = rw[l][j];
      sh = wflat >> (8 * int'(s_bo[l]));
      r  = sh[127:0];
      for (int q = 0; q < 16; q++)
        if (q > int'(s_nb)) r[8*q +: 8] = 8'd0;
      rd_al[l] = r;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rsp_valid <= 1'b0;
      rsp_grp   <= '0;
      rsp_sel   <= '0;
      for (int l = 0; l < LPB; l++) rsp_rdata[l] <= '0;
    end else begin
      rsp_valid <= ph4;
      if (ph4) begin
        rsp_grp <= s_grp;
        rsp_sel <= s_sel;
        for (int l = 0; l < LPB; l++) rsp_rdata[l] <= rd_al[l];
      end
    end
  end

endmodule
