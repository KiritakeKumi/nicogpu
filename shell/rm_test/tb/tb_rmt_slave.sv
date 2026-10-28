// tb_rmt_slave.sv -- behavioral AXI4 slave memory for the test RM TB.
//
// * sparse memory of full-width beats (unwritten beats read as 0)
// * random AWREADY/WREADY/ARREADY, random B/R latency and R/B gaps, responses
//   strictly in order (all IDs are 0)
// * SLVERR injection for bursts whose start address is in [err_lo, err_hi)
//   (erroneous writes do not modify memory; erroneous reads return 0xBADBAD00)
// * protocol assertions on the master (RM) side: INCR only, AxSIZE = full
//   width, AxLEN <= MAXLEN, address aligned, no 4 KiB crossing, AxCACHE=0011,
//   AxPROT=010, AxID=0, W never before its AW (strict: AW accepted in an
//   earlier cycle), WLAST exactly on the last beat, outstanding <= MAXWR /
//   MAXRD, VALID held and payload stable until READY
// * freeze=1 (set by the TB after rm_quiesce_ack): any AW/AR handshake fails
// * XDMA_STRB=1 (the host port behind the XDMA AXI->PCIe bridge, measured on the board): the enabled bytes
//   a write burst must form ONE contiguous byte range, else FAIL (strb_fail, default 1; n_holey counts the beats);
//   a beat with holes is written like the bridge does it: the whole range [first .. last enabled byte] of the beat
//   takes wdata (holes included)
module tb_rmt_slave #(
  parameter int    DW     = 512,
  parameter int    AW     = 33,
  parameter int    IW     = 6,
  parameter int    MAXLEN = 15,
  parameter int    MAXWR  = 32,
  parameter int    MAXRD  = 64,
  parameter string NAME   = "c1",
  parameter bit    XDMA_STRB = 1'b0
) (
  input  logic            clk,
  input  logic            rst_n,
  input  logic [IW-1:0]   awid,
  input  logic [AW-1:0]   awaddr,
  input  logic [7:0]      awlen,
  input  logic [2:0]      awsize,
  input  logic [1:0]      awburst,
  input  logic [3:0]      awcache,
  input  logic [2:0]      awprot,
  input  logic [3:0]      awqos,
  input  logic            awvalid,
  output logic            awready,
  input  logic [DW-1:0]   wdata,
  input  logic [DW/8-1:0] wstrb,
  input  logic            wlast,
  input  logic            wvalid,
  output logic            wready,
  output logic [IW-1:0]   bid,
  output logic [1:0]      bresp,
  output logic            bvalid,
  input  logic            bready,
  input  logic [IW-1:0]   arid,
  input  logic [AW-1:0]   araddr,
  input  logic [7:0]      arlen,
  input  logic [2:0]      arsize,
  input  logic [1:0]      arburst,
  input  logic [3:0]      arcache,
  input  logic [2:0]      arprot,
  input  logic [3:0]      arqos,
  input  logic            arvalid,
  output logic            arready,
  output logic [IW-1:0]   rid,
  output logic [DW-1:0]   rdata,
  output logic [1:0]      rresp,
  output logic            rlast,
  output logic            rvalid,
  input  logic            rready
);
  localparam int BPB = DW / 8;
  localparam int SZ  = $clog2(BPB);
  typedef logic [DW-1:0] beat_t;

  beat_t mem [longint unsigned];

  int      ready_pct = 75;
  int      gap_pct   = 15;
  int      lat_min   = 2;
  int      lat_max   = 30;
  bit      err_en    = 1'b0;
  longint unsigned err_lo = 0, err_hi = 0;
  bit      freeze    = 1'b0;
  bit      strb_fail = 1'b1;     // XDMA_STRB: a holey burst is a FAIL (0: only written the XDMA way)
  int      n_holey   = 0;        // XDMA_STRB: write beats that broke the contiguous-range rule
  int      wst       = 0;        // XDMA_STRB: byte-range state of the current burst (0 none yet, 1 in it, 2 after it)
  int      errors    = 0;
  int      n_aw = 0, n_ar = 0, n_wb = 0, n_b = 0, n_rb = 0;
  int      wr_out = 0, rd_out = 0, max_wr_out = 0, max_rd_out = 0;
  int      aw_len_hist [256];
  int      ar_len_hist [256];

  typedef struct { longint unsigned addr; int len; bit err; } bst_t;
  typedef struct { longint t; bit err; } bq_t;
  typedef struct { longint t; longint unsigned addr; int len; bit err; } rq_t;
  bst_t awq [$];
  bq_t  bq [$];
  rq_t  rq [$];
  int   wbeat = 0, rbeat = 0;
  longint cyc = 0;

  logic                          aw_wait, w_wait, ar_wait;
  logic [AW+8+3+2+4+3+IW-1:0]    aw_hold, ar_hold;
  logic [DW+DW/8:0]              w_hold;

  function automatic beat_t peek(longint unsigned b);
    return mem.exists(b) ? mem[b] : '0;
  endfunction
  function automatic void poke(longint unsigned b, beat_t d);
    mem[b] = d;
  endfunction
  function automatic bit in_err(longint unsigned a);
    return err_en && (a >= err_lo) && (a < err_hi);
  endfunction
  function automatic void fail(string s);
    errors++;
    if (errors <= 20) $display("FAIL [%s] t=%0t %s", NAME, $time, s);
  endfunction
  function automatic void chk_ax(string ch, logic [IW-1:0] id, longint unsigned a, int len,
                                 logic [2:0] size, logic [1:0] burst, logic [3:0] cache, logic [2:0] prot);
    if (burst != 2'b01) fail($sformatf("%s burst %0d not INCR", ch, burst));
    if (size != 3'(SZ)) fail($sformatf("%s size %0d != %0d", ch, size, SZ));
    if (len > MAXLEN)   fail($sformatf("%s len %0d > %0d", ch, len, MAXLEN));
    if ((a % BPB) != 0) fail($sformatf("%s addr %h unaligned", ch, a));
    if ((a % 4096) + (len + 1) * BPB > 4096) fail($sformatf("%s addr %h len %0d crosses 4 KiB", ch, a, len));
    if (cache != 4'b0011) fail($sformatf("%s cache %b", ch, cache));
    if (prot != 3'b010) fail($sformatf("%s prot %b", ch, prot));
    if (id != '0) fail($sformatf("%s id %0d", ch, id));
    if (freeze) fail($sformatf("%s handshake at %h after rm_quiesce_ack", ch, a));
  endfunction

  function automatic int lat();
    return lat_min + $urandom_range(0, lat_max - lat_min);
  endfunction

  always @(posedge clk) begin
    if (!rst_n) begin
      awready <= 1'b0; wready <= 1'b0; arready <= 1'b0;
      bvalid <= 1'b0; rvalid <= 1'b0; bid <= '0; rid <= '0;
      awq.delete(); bq.delete(); rq.delete();
      wbeat = 0; rbeat = 0; wr_out = 0; rd_out = 0;
      aw_wait = 1'b0; w_wait = 1'b0; ar_wait = 1'b0;
    end else begin
      cyc++;
      // ---- VALID held / payload stable until READY
      if (aw_wait) begin
        if (!awvalid) fail("AWVALID dropped before AWREADY");
        else if ({awid, awaddr, awlen, awsize, awburst, awcache, awprot} != aw_hold) fail("AW payload changed while waiting");
      end
      if (w_wait) begin
        if (!wvalid) fail("WVALID dropped before WREADY");
        else if ({wdata, wstrb, wlast} != w_hold) fail("W payload changed while waiting");
      end
      if (ar_wait) begin
        if (!arvalid) fail("ARVALID dropped before ARREADY");
        else if ({arid, araddr, arlen, arsize, arburst, arcache, arprot} != ar_hold) fail("AR payload changed while waiting");
      end
      aw_wait = awvalid && !awready;
      w_wait  = wvalid && !wready;
      ar_wait = arvalid && !arready;
      aw_hold = {awid, awaddr, awlen, awsize, awburst, awcache, awprot};
      w_hold  = {wdata, wstrb, wlast};
      ar_hold = {arid, araddr, arlen, arsize, arburst, arcache, arprot};

      // ---- W (only against AWs accepted in earlier cycles)
      if (wvalid && wready) begin
        n_wb++;
        if (awq.size() == 0) begin
          fail("W beat before its AW was accepted");
        end else begin
          bst_t a;
          longint unsigned b;
          logic [BPB-1:0] ws;
          a = awq[0];
          b = (a.addr >> SZ) + wbeat;
          ws = wstrb;
          if (XDMA_STRB) begin
            bit hole;
            int lo, hi;
            if (wbeat == 0) wst = 0;
            hole = 1'b0; lo = -1; hi = -1;
            for (int i = 0; i < BPB; i++) begin
              if (ws[i]) begin
                if (wst == 2) hole = 1'b1;
                wst = 1;
                if (lo < 0) lo = i;
                hi = i;
              end else if (wst == 1) begin
                wst = 2;
              end
            end
            if (hole) begin
              n_holey++;
              if (strb_fail) fail($sformatf("holey write strobe %h (beat %0d of the burst at %h), XDMA fills it",
                                            wstrb, wbeat, a.addr));
            end
            if (lo >= 0) for (int i = lo; i <= hi; i++) ws[i] = 1'b1;
          end
          if (!a.err) begin
            beat_t m;
            m = peek(b);
            for (int i = 0; i < BPB; i++) if (ws[i]) m[8*i +: 8] = wdata[8*i +: 8];
            mem[b] = m;
          end
          if (wlast != (wbeat == a.len)) fail($sformatf("WLAST=%0d on beat %0d of len %0d", wlast, wbeat, a.len));
          if (wbeat == a.len) begin
            bq.push_back('{cyc + lat(), a.err});
            void'(awq.pop_front());
            wbeat = 0;
          end else begin
            wbeat++;
          end
        end
      end
      // --
      if (awvalid && awready) begin
        chk_ax("AW", awid, awaddr, awlen, awsize, awburst, awcache, awprot);
        awq.push_back('{awaddr, awlen, in_err(awaddr)});
        n_aw++;
        wr_out++;
        aw_len_hist[awlen]++;
        if (wr_out > max_wr_out) max_wr_out = wr_out;
        if (wr_out > MAXWR) fail($sformatf("write outstanding %0d > %0d", wr_out, MAXWR));
      end
      // --
      if (arvalid && arready) begin
        chk_ax("AR", arid, araddr, arlen, arsize, arburst, arcache, arprot);
        rq.push_back('{cyc + lat(), araddr, arlen, in_err(araddr)});
        n_ar++;
        rd_out++;
        ar_len_hist[arlen]++;
        if (rd_out > max_rd_out) max_rd_out = rd_out;
        if (rd_out > MAXRD) fail($sformatf("read outstanding %0d > %0d", rd_out, MAXRD));
      end
      // --
      if (bvalid && bready) begin
        wr_out--;
        n_b++;
      end
      if (!bvalid || bready) begin
        if (bq.size() != 0 && bq[0].t <= cyc && $urandom_range(0, 99) >= gap_pct) begin
          bvalid <= 1'b1;
          bresp  <= bq[0].err ? 2'b10 : 2'b00;
          void'(bq.pop_front());
        end else begin
          bvalid <= 1'b0;
        end
      end
      // --
      if (rvalid && rready) begin
        n_rb++;
        if (rlast) rd_out--;
      end
      if (!rvalid || rready) begin
        if (rq.size() != 0 && rq[0].t <= cyc && $urandom_range(0, 99) >= gap_pct) begin
          rvalid <= 1'b1;
          rdata  <= rq[0].err ? {(DW/32){32'hBADBAD00}} : peek((rq[0].addr >> SZ) + rbeat);
          rresp  <= rq[0].err ? 2'b10 : 2'b00;
          rlast  <= (rbeat == rq[0].len);
          if (rbeat == rq[0].len) begin
            void'(rq.pop_front());
            rbeat = 0;
          end else begin
            rbeat++;
          end
        end else begin
          rvalid <= 1'b0;
        end
      end
      // --
      awready <= ($urandom_range(0, 99) < ready_pct);
      wready  <= ($urandom_range(0, 99) < ready_pct);
      arready <= ($urandom_range(0, 99) < ready_pct);
    end
  end

  function automatic bit idle();
    return (wr_out == 0) && (rd_out == 0) && (awq.size() == 0);
  endfunction
endmodule
