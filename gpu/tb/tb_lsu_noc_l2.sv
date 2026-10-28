// tb_lsu_noc_l2.sv -- integration testbench:
//   ng_l1d (small, to force evictions) -> ng_lsu_noc_adapter -> ng_noc_ep ->
//   ng_noc_mesh (3x2, node 3 unused) -> 4 x (ng_noc_ep + ng_l2_slice (small
//   SETS/WAYS) + tb_l2_axi_mem_model), plus a TB uc bypass memory.
//
// The requestor is driven in two ways, switchable between operations:
//   direct: the TB drives the adapter's upstream port itself (every request
//            reaches the adapter; also M_INV_L1, which L1D would absorb);
//   via L1D: the TB drives ng_l1d (L1 hits never reach the adapter; checks L1
//            fill / write-through / invalidate coherence on top of the L2).
// A byte-level shadow memory (VRAM keyed by PA, uc keyed by PA) is the
// reference for every read data and AMO old value; the end state is checked
// after a wb.l2 by comparing every shadow byte with the DDR models.
//
// Fault-injection / coverage helpers (all in the TB, DUT untouched):
//   * ti gate: random stalls on the adapter->endpoint request stream (checks
//     the adapter holds a stalled flit stable and loses nothing);
//   * response shuffle: NoC responses are buffered and handed to the adapter in
//     random order (tag-based reassembly of the 16 read chunks);
//   * per-slice request / DDR-read / DDR-write counters (routing, hit / miss,
//     eviction write-back), natural out-of-order maintenance responses.
// Seeds: xsim -sv_seed / Verilator +SEED=.  Expect NG_LSU_NOC_L2_TB_PASS.
`timescale 1ns/1ps
module tb_lsu_noc_l2;
  import ng_noc_pkg::*;
  import ng_l2_addrmap::*;
  import ng_lsu_pkg::mreq_t, ng_lsu_pkg::mrsp_t;
  import ng_lsu_pkg::M_RD, ng_lsu_pkg::M_WR, ng_lsu_pkg::M_AMO, ng_lsu_pkg::M_INV_L1,
         ng_lsu_pkg::M_INV_L2, ng_lsu_pkg::M_WB_L2;

  localparam int GW = 3, GH = 2, NN = GW * GH;
  localparam int BUF_DEPTH = 4, LOC_CREDITS = 4;
  localparam int AD_NODE = 0;
  localparam int SLICE_NODE [4] = '{1, 2, 4, 5};
  localparam int SL_SETS = 4, SL_WAYS = 2;       // tiny slices: constant eviction
  localparam int OPS_PER_PHASE = 300;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  // ---- mesh ----------------------------------------------------------------
  logic [NN-1:0]        ep_inj_valid;
  flit_t [NN-1:0]       ep_inj_flit;
  logic [NN-1:0][1:0]   ep_inj_credit;
  logic [NN-1:0]        ep_recv_valid;
  flit_t [NN-1:0]       ep_recv_flit;
  logic [NN-1:0][1:0]   ep_recv_credit;

  ng_noc_mesh #(
    .GW(GW), .GH(GH), .NUM_VC(2),
    .BUF_DEPTH(BUF_DEPTH), .LOC_CREDITS(LOC_CREDITS)
  ) u_mesh (
    .clk(clk), .rst_n(rst_n),
    .ep_inj_valid(ep_inj_valid), .ep_inj_flit(ep_inj_flit),
    .ep_inj_credit(ep_inj_credit),
    .ep_recv_valid(ep_recv_valid), .ep_recv_flit(ep_recv_flit),
    .ep_recv_credit(ep_recv_credit)
  );

  // node 3 has no agent
  assign ep_inj_valid[3]   = 1'b0;
  assign ep_inj_flit[3]    = '0;
  assign ep_recv_credit[3] = '0;

  // ---- L2 slices (+ endpoint + DDR model) -----------------------------------
  logic  sl_req_v [4];
  logic  sl_req_r [4];
  flit_t sl_req_f [4];
  logic  sl_resp_v [4];
  logic  sl_resp_r [4];
  flit_t sl_resp_f [4];
  int    ar_cnt [4];      // DDR read bursts (L2 fills)
  int    aw_cnt [4];      // DDR write bursts (write-backs)
  int    sreq_cnt [4];    // flits accepted by each slice

  for (genvar s = 0; s < 4; s++) begin : g_s
    localparam int NODE = SLICE_NODE[s];
    localparam int AW = (s < 2) ? 33 : 32;          // C1 halves: 512b, C2/C4: 256b
    localparam int DW = (s < 2) ? 512 : 256;

    ng_noc_ep #(
      .NUM_VC(2), .INJ_CREDITS(BUF_DEPTH), .RX_DEPTH(2 * LOC_CREDITS)
    ) u_ep (
      .clk(clk), .rst_n(rst_n),
      .ti_valid(sl_resp_v[s]), .ti_ready(sl_resp_r[s]), .ti_flit(sl_resp_f[s]),
      .to_valid(sl_req_v[s]),  .to_ready(sl_req_r[s]),  .to_flit(sl_req_f[s]),
      .inj_valid(ep_inj_valid[NODE]), .inj_flit(ep_inj_flit[NODE]),
      .inj_credit(ep_inj_credit[NODE]),
      .recv_valid(ep_recv_valid[NODE]), .recv_flit(ep_recv_flit[NODE]),
      .recv_credit(ep_recv_credit[NODE])
    );

    logic              arv, arr, rv, rr, rl, awv, awr, wv, wr, wl, bv, br;
    logic [AW-1:0]     araddr, awaddr;
    logic [7:0]        arlen, awlen;
    logic [2:0]        arsize, awsize;
    logic [1:0]        arburst, awburst, rresp, bresp;
    logic [5:0]        arid, rid, awid, bid;
    logic [DW-1:0]     rdata, wdata;
    logic [DW/8-1:0]   wstrb;

    ng_l2_slice #(.SETS(SL_SETS), .WAYS(SL_WAYS), .AXI_AW(AW), .AXI_DW(DW),
                  .AXI_IW(6), .SLICE_ID(NODE)) u_sl (
      .clk(clk), .rst_n(rst_n),
      .s_req_valid(sl_req_v[s]), .s_req_ready(sl_req_r[s]), .s_req_flit(sl_req_f[s]),
      .m_resp_valid(sl_resp_v[s]), .m_resp_ready(sl_resp_r[s]),
      .m_resp_flit(sl_resp_f[s]),
      .arvalid(arv), .arready(arr), .araddr(araddr), .arlen(arlen),
      .arsize(arsize), .arburst(arburst), .arid(arid),
      .rvalid(rv), .rready(rr), .rdata(rdata), .rlast(rl),
      .rid(rid), .rresp(rresp),
      .awvalid(awv), .awready(awr), .awaddr(awaddr), .awlen(awlen),
      .awsize(awsize), .awburst(awburst), .awid(awid),
      .wvalid(wv), .wready(wr), .wdata(wdata), .wstrb(wstrb), .wlast(wl),
      .bvalid(bv), .bready(br), .bid(bid), .bresp(bresp)
    );
    tb_l2_axi_mem_model #(.AW(AW), .DW(DW), .IW(6)) u_m (
      .clk(clk), .rst_n(rst_n),
      .arvalid(arv), .arready(arr), .araddr(araddr), .arlen(arlen),
      .arsize(arsize), .arburst(arburst), .arid(arid),
      .rvalid(rv), .rready(rr), .rdata(rdata), .rlast(rl),
      .rid(rid), .rresp(rresp),
      .awvalid(awv), .awready(awr), .awaddr(awaddr), .awlen(awlen),
      .awsize(awsize), .awburst(awburst), .awid(awid),
      .wvalid(wv), .wready(wr), .wdata(wdata), .wstrb(wstrb), .wlast(wl),
      .bvalid(bv), .bready(br), .bid(bid), .bresp(bresp)
    );

    always_ff @(posedge clk) begin
      if (!rst_n) begin
        ar_cnt[s] <= 0;
        aw_cnt[s] <= 0;
        sreq_cnt[s] <= 0;
      end else begin
        if (arv && arr) ar_cnt[s] <= ar_cnt[s] + 1;
        if (awv && awr) aw_cnt[s] <= aw_cnt[s] + 1;
        if (sl_req_v[s] && sl_req_r[s]) sreq_cnt[s] <= sreq_cnt[s] + 1;
      end
    end
  end

  function automatic logic [7:0] peek_ddr(input logic [47:0] pa);
    ch_addr_s ca;
    ca = l2_ch_of(pa);
    case (l2_slice_idx(pa))
      0:       return g_s[0].u_m.peek(ca.off[32:0]);
      1:       return g_s[1].u_m.peek(ca.off[32:0]);
      2:       return g_s[2].u_m.peek(ca.off[31:0]);
      default: return g_s[3].u_m.peek(ca.off[31:0]);
    endcase
  endfunction

  // ---- requestor node: adapter + endpoint ------------------------------------
  logic  ad_req_valid, ad_req_ready, ad_rsp_valid;
  mreq_t ad_req;
  mrsp_t ad_rsp;
  logic  ad_ti_valid, ad_ti_ready, ad_to_valid, ad_to_ready;
  flit_t ad_ti_flit, ad_to_flit;
  logic  byp_req_valid, byp_req_ready, byp_rsp_valid;
  mreq_t byp_req;
  mrsp_t byp_rsp;

  logic  ti_gate = 1'b1;                 // TB backpressure on the request stream
  logic  ep_ti_valid, ep_ti_ready;
  logic  ep_to_valid, ep_to_ready;
  flit_t ep_to_flit;
  assign ep_ti_valid = ad_ti_valid & ti_gate;
  assign ad_ti_ready = ep_ti_ready & ti_gate;
  assign ep_to_ready = 1'b1;             // the shuffle shim sinks everything at once

  ng_noc_ep #(
    .NUM_VC(2), .INJ_CREDITS(BUF_DEPTH), .RX_DEPTH(2 * LOC_CREDITS), .TXQ_DEPTH(2)
  ) u_ad_ep (
    .clk(clk), .rst_n(rst_n),
    .ti_valid(ep_ti_valid), .ti_ready(ep_ti_ready), .ti_flit(ad_ti_flit),
    .to_valid(ep_to_valid), .to_ready(ep_to_ready), .to_flit(ep_to_flit),
    .inj_valid(ep_inj_valid[AD_NODE]), .inj_flit(ep_inj_flit[AD_NODE]),
    .inj_credit(ep_inj_credit[AD_NODE]),
    .recv_valid(ep_recv_valid[AD_NODE]), .recv_flit(ep_recv_flit[AD_NODE]),
    .recv_credit(ep_recv_credit[AD_NODE])
  );

  ng_lsu_noc_adapter #(
    .SRC_ID(6'(AD_NODE)),
    .SLICE_NODES({6'(SLICE_NODE[3]), 6'(SLICE_NODE[2]), 6'(SLICE_NODE[1]), 6'(SLICE_NODE[0])}),
    .TAG_HI(4'h0)
  ) u_ad (
    .clk(clk), .rst_n(rst_n),
    .req_valid(ad_req_valid), .req_ready(ad_req_ready), .req(ad_req),
    .rsp_valid(ad_rsp_valid), .rsp(ad_rsp),
    .ti_valid(ad_ti_valid), .ti_ready(ad_ti_ready), .ti_flit(ad_ti_flit),
    .to_valid(ad_to_valid), .to_ready(ad_to_ready), .to_flit(ad_to_flit),
    .byp_req_valid(byp_req_valid), .byp_req_ready(byp_req_ready), .byp_req(byp_req),
    .byp_rsp_valid(byp_rsp_valid), .byp_rsp(byp_rsp)
  );

  // ---- L1D + direct/L1D upstream mux -------------------------------------------
  logic  use_l1;                          // 1: TB drives L1D, 0: TB drives the adapter
  logic  l1_req_valid, l1_req_ready, l1_rsp_valid;
  mreq_t l1_req;
  mrsp_t l1_rsp;
  logic  d_req_valid;
  mreq_t d_req;
  logic  l1_mem_req_valid, l1_mem_req_ready, l1_mem_rsp_valid;
  mreq_t l1_mem_req;
  mrsp_t l1_mem_rsp;
  logic [31:0] st_hit, st_miss, st_fill, st_inv;

  ng_l1d #(.SETS(4), .WAYS(2)) u_l1 (
    .clk(clk), .rst_n(rst_n), .ext_inv(1'b0),
    .req_valid(l1_req_valid), .req_ready(l1_req_ready), .req(l1_req),
    .rsp_valid(l1_rsp_valid), .rsp(l1_rsp),
    .mem_req_valid(l1_mem_req_valid), .mem_req_ready(l1_mem_req_ready),
    .mem_req(l1_mem_req), .mem_rsp_valid(l1_mem_rsp_valid), .mem_rsp(l1_mem_rsp),
    .stat_hit(st_hit), .stat_miss(st_miss), .stat_fill(st_fill), .stat_inv(st_inv)
  );

  assign ad_req_valid     = use_l1 ? l1_mem_req_valid : d_req_valid;
  assign ad_req           = use_l1 ? l1_mem_req : d_req;
  assign l1_mem_req_ready = ad_req_ready & use_l1;
  assign l1_mem_rsp_valid = ad_rsp_valid & use_l1;
  assign l1_mem_rsp       = ad_rsp;

  logic  d_req_ready;
  assign d_req_ready = ad_req_ready;
  logic  t_req_ready, t_rsp_valid;
  mrsp_t t_rsp;
  assign t_req_ready = use_l1 ? l1_req_ready : d_req_ready;
  assign t_rsp_valid = use_l1 ? l1_rsp_valid : ad_rsp_valid;
  assign t_rsp       = use_l1 ? l1_rsp : ad_rsp;

  // ---- reference memories ------------------------------------------------------
  // shadow: the architectural truth (VRAM and uc keyed by PA, disjoint ranges);
  // ucmem: the bypass-port device model (separate storage, same semantics).
  logic [7:0] shadow [logic [47:0]];
  logic [7:0] ucmem  [logic [47:0]];

  function automatic logic [7:0] hash48(input logic [47:0] a);
    logic [63:0] p;
    p = a * 64'h9E3779B97F4A7C15;
    return p[55:48];
  endfunction

  function automatic logic [7:0] defbyte(input logic [47:0] pa);
    if (pa < L2_VRAM_BYTES) return hash48(l2_ch_of(pa).off);   // = DDR model default
    return hash48(pa ^ 48'h5A5A_A5A5_1234);
  endfunction

  function automatic logic [7:0] gb(input bit dev, input logic [47:0] pa);
    if (dev) return ucmem.exists(pa) ? ucmem[pa] : defbyte(pa);
    return shadow.exists(pa) ? shadow[pa] : defbyte(pa);
  endfunction

  function automatic void sb(input bit dev, input logic [47:0] pa, input logic [7:0] v);
    if (dev) ucmem[pa] = v;
    else     shadow[pa] = v;
  endfunction

  // little-endian value of nb bytes at pa in the shadow
  function automatic logic [63:0] val_at(input logic [47:0] pa, input int nb);
    logic [63:0] v;
    v = '0;
    for (int i = 0; i < nb; i++) v[8*i +: 8] = gb(1'b0, pa + 48'(i));
    return v;
  endfunction

  function automatic logic [63:0] ref_alu(input logic [4:0] o, input logic w64,
                                          input logic [63:0] av, input logic [63:0] bv,
                                          input logic [63:0] cv);
    logic [31:0] a32, b32, r32;
    a32 = av[31:0];
    b32 = bv[31:0];
    r32 = a32;
    if (w64) begin
      case (o)
        AMO_ADD:   return av + bv;
        AMO_SUB:   return av - bv;
        AMO_AND:   return av & bv;
        AMO_OR:    return av | bv;
        AMO_XOR:   return av ^ bv;
        AMO_MIN_S: return ($signed(av) < $signed(bv)) ? av : bv;
        AMO_MAX_S: return ($signed(av) > $signed(bv)) ? av : bv;
        AMO_MIN_U: return (av < bv) ? av : bv;
        AMO_MAX_U: return (av > bv) ? av : bv;
        AMO_XCHG:  return bv;
        AMO_CAS:   return (av == cv) ? bv : av;
        default:   return av;
      endcase
    end else begin
      case (o)
        AMO_ADD:   r32 = a32 + b32;
        AMO_SUB:   r32 = a32 - b32;
        AMO_AND:   r32 = a32 & b32;
        AMO_OR:    r32 = a32 | b32;
        AMO_XOR:   r32 = a32 ^ b32;
        AMO_MIN_S: r32 = ($signed(a32) < $signed(b32)) ? a32 : b32;
        AMO_MAX_S: r32 = ($signed(a32) > $signed(b32)) ? a32 : b32;
        AMO_MIN_U: r32 = (a32 < b32) ? a32 : b32;
        AMO_MAX_U: r32 = (a32 > b32) ? a32 : b32;
        AMO_XCHG:  r32 = b32;
        AMO_CAS:   r32 = (a32 == cv[31:0]) ? b32 : a32;
        AMO_INC:   r32 = (a32 >= b32) ? 32'd0 : a32 + 32'd1;
        AMO_DEC:   r32 = (a32 == 32'd0 || a32 > b32) ? b32 : a32 - 32'd1;
        default:   r32 = a32;
      endcase
      return {32'd0, r32};
    end
  endfunction

  // executes one request on memory `dev` (0: shadow, 1: ucmem); returns the response
  function automatic mrsp_t exec_on(input bit dev, input mreq_t r);
    mrsp_t o;
    logic [63:0] oldv, newv;
    int nb;
    o = '0;
    case (r.kind)
      M_RD: begin
        for (int i = 0; i < 128; i++) o.rdata[8*i +: 8] = gb(dev, r.pa + 48'(i));
      end
      M_WR: begin
        for (int i = 0; i < 128; i++) if (r.be[i]) sb(dev, r.pa + 48'(i), r.wdata[8*i +: 8]);
      end
      M_AMO: begin
        nb = r.amo64 ? 8 : 4;
        oldv = '0;
        for (int i = 0; i < nb; i++) oldv[8*i +: 8] = gb(dev, r.pa + 48'(i));
        newv = ref_alu(r.amo_op, r.amo64, oldv, r.amo_a, r.amo_cmp);
        for (int i = 0; i < nb; i++) sb(dev, r.pa + 48'(i), newv[8*i +: 8]);
        o.old = r.amo_rtn ? oldv : 64'd0;
      end
      default: ;
    endcase
    return o;
  endfunction

  // ---- uc bypass device ---------------------------------------------------------
  // ready toggles randomly while idle; the response pulse comes 1..4 cycles after
  // the handshake.  Drives on negedge with blocking assigns.
  mreq_t  bp_cap;
  mrsp_t  bp_rsp_q;
  bit     bp_hs, bp_busy;
  int     bp_delay;
  int     byp_cnt;
  initial begin
    byp_req_ready = 1'b0;
    byp_rsp_valid = 1'b0;
    byp_rsp       = '0;
    bp_hs = 1'b0; bp_busy = 1'b0; bp_delay = 0; byp_cnt = 0;
    forever begin
      @(negedge clk);
      if (!rst_n) begin
        byp_req_ready = 1'b0;
        byp_rsp_valid = 1'b0;
        bp_hs = 1'b0; bp_busy = 1'b0;
      end else begin
        byp_rsp_valid = 1'b0;
        if (bp_hs) begin                       // a handshake happened at the last posedge
          bp_rsp_q = exec_on(1'b1, bp_cap);
          bp_delay = $urandom_range(0, 3);
          bp_busy  = 1'b1;
          bp_hs    = 1'b0;
          byp_cnt++;
        end
        if (bp_busy) begin
          byp_req_ready = 1'b0;
          if (bp_delay == 0) begin
            byp_rsp_valid = 1'b1;
            byp_rsp       = bp_rsp_q;
            bp_busy       = 1'b0;
          end else begin
            bp_delay--;
          end
        end else begin
          byp_req_ready = ($urandom_range(0, 2) != 0);
        end
        bp_hs  = byp_req_valid && byp_req_ready;   // takes effect at the next posedge
        bp_cap = byp_req;
      end
    end
  end

  // ---- request-stream backpressure gate ------------------------------------------
  int bp_mode;       // 0: none, 1: ~50% ready, 2: ~20% ready
  initial begin
    forever begin
      @(negedge clk);
      case (bp_mode)
        1:       ti_gate = ($urandom_range(0, 1) == 1);
        2:       ti_gate = ($urandom_range(0, 4) == 0);
        default: ti_gate = 1'b1;
      endcase
    end
  end

  // ---- response shuffle shim --------------------------------------------------------
  flit_t sh_q [$];
  bit    shuf_en;
  int    sh_age, sh_target;
  bit    sh_rel;
  always @(posedge clk) begin
    if (rst_n && ep_to_valid && ep_to_ready) sh_q.push_back(ep_to_flit);
  end
  initial begin
    ad_to_valid = 1'b0;
    ad_to_flit  = '0;
    sh_age = 0; sh_rel = 1'b0; sh_target = 3;
    forever begin
      @(negedge clk);
      ad_to_valid = 1'b0;
      if (sh_q.size() == 0) begin
        sh_rel = 1'b0;
        sh_age = 0;
      end else if (!shuf_en) begin
        ad_to_flit  = sh_q.pop_front();
        ad_to_valid = 1'b1;
      end else begin
        int idx;
        if (!sh_rel) begin
          sh_age++;
          if (sh_q.size() >= sh_target || sh_age >= 12) sh_rel = 1'b1;
        end
        if (sh_rel && ($urandom_range(0, 3) != 0)) begin
          idx = $urandom_range(0, sh_q.size() - 1);
          ad_to_flit = sh_q[idx];
          sh_q.delete(idx);
          ad_to_valid = 1'b1;
        end
        if (sh_rel && sh_q.size() == 0) begin
          sh_rel = 1'b0;
          sh_age = 0;
          sh_target = $urandom_range(2, 6);
        end
      end
    end
  end

  // ---- protocol monitors ------------------------------------------------------------
  int     errors, checks;
  int     bp_cycles;           // adapter request flit stalled by ti_ready
  int     ooo_cnt;             // responses whose tag is lower than the previous one
  int     last_tag;
  int     n_ti_flits;
  bit     prev_stall;
  flit_t  prev_flit;
  always @(posedge clk) begin
    if (rst_n) begin
      if (prev_stall && (!ad_ti_valid || ad_ti_flit !== prev_flit)) begin
        errors++;
        $display("TB_FAIL lsu_noc_l2: adapter dropped / changed a stalled request flit");
      end
      prev_stall = ad_ti_valid && !ad_ti_ready;
      prev_flit  = ad_ti_flit;
      if (prev_stall) bp_cycles++;
      if (ad_ti_valid && ad_ti_ready) n_ti_flits++;
      if (ad_req_valid && ad_req_ready) last_tag = -1;
      if (ad_to_valid) begin
        if (last_tag >= 0 && int'(ad_to_flit.tag[3:0]) < last_tag) ooo_cnt++;
        last_tag = int'(ad_to_flit.tag[3:0]);
      end
    end
  end

  task automatic fail(input string msg);
    errors++;
    if (errors <= 30) $display("TB_FAIL lsu_noc_l2: %s", msg);
  endtask

  // ---- request builders ---------------------------------------------------------------
  function automatic mreq_t base_req(input logic [2:0] kind, input logic [47:0] pa);
    mreq_t r;
    r = '0;
    r.kind = kind;
    r.pa   = pa;
    r.uc   = !ng_lsu_pkg::pa_is_vram(pa);
    return r;
  endfunction

  function automatic logic [1023:0] rnd1024();
    logic [1023:0] d;
    for (int i = 0; i < 32; i++) d[32*i +: 32] = $urandom;
    return d;
  endfunction

  function automatic logic [127:0] rnd_be(input int style);
    logic [127:0] b;
    b = '0;
    case (style)
      0: b = '1;
      1: b = '0;
      2: for (int i = 0; i < 128; i++) b[i] = ($urandom_range(0, 3) != 0);
      3: for (int i = 0; i < 128; i++) b[i] = ($urandom_range(0, 31) == 0);
      4: for (int k = 0; k < 16; k++) if ($urandom_range(0, 1) == 1) b[8*k +: 8] = 8'hFF;
      5: b[$urandom_range(0, 127)] = 1'b1;
      6: begin b[8*$urandom_range(0, 15) +: 8] = 8'hFF; end
      default: for (int i = 0; i < 128; i++) b[i] = $urandom_range(0, 1);
    endcase
    return b;
  endfunction

  function automatic mreq_t mk_rd(input logic [47:0] pa);
    return base_req(M_RD, pa);
  endfunction
  function automatic mreq_t mk_wr(input logic [47:0] pa, input logic [127:0] be,
                                  input logic [1023:0] d);
    mreq_t r;
    r = base_req(M_WR, pa);
    r.be = be;
    r.wdata = d;
    return r;
  endfunction
  function automatic mreq_t mk_amo(input logic [47:0] pa, input logic [4:0] o,
                                   input logic w64, input logic rtn,
                                   input logic [63:0] a, input logic [63:0] c);
    mreq_t r;
    r = base_req(M_AMO, pa);
    r.amo_op = o; r.amo64 = w64; r.amo_rtn = rtn; r.amo_a = a; r.amo_cmp = c;
    return r;
  endfunction

  // ---- one operation: reference, issue, wait, compare -----------------------------------
  localparam int LIM = 60000;

  task automatic issue(input mreq_t r, output mrsp_t o);
    int n;
    @(negedge clk);
    if (use_l1) begin
      l1_req = r;
      l1_req_valid = 1'b1;
    end else begin
      d_req = r;
      d_req_valid = 1'b1;
    end
    n = 0;
    while (!t_req_ready) begin
      @(negedge clk);
      if (++n > LIM) begin fail("timeout waiting for req_ready"); $finish; end
    end
    @(negedge clk);
    l1_req_valid = 1'b0;
    d_req_valid  = 1'b0;
    n = 0;
    while (!t_rsp_valid) begin
      @(negedge clk);
      if (++n > LIM) begin
        fail($sformatf("timeout waiting for response (kind %0d pa %h)", r.kind, r.pa));
        $display("NG_LSU_NOC_L2_TB_FAIL: deadlock");
        $finish;
      end
    end
    o = t_rsp;
  endtask

  int n_op [8];   // per kind counters (index = kind, 6 = uc, 7 = zero-be write)

  task automatic op(input mreq_t r);
    mrsp_t exp_r, got;
    bit is_uc;
    is_uc = r.uc && (r.kind == M_RD || r.kind == M_WR || r.kind == M_AMO);
    exp_r = exec_on(1'b0, r);
    issue(r, got);
    checks++;
    n_op[r.kind]++;
    if (is_uc) n_op[6]++;
    if (r.kind == M_WR && r.be == '0) n_op[7]++;
    if (got.err !== 2'd0)
      fail($sformatf("kind %0d pa %h err=%0d", r.kind, r.pa, got.err));
    if (r.kind == M_RD && got.rdata !== exp_r.rdata) begin
      for (int i = 0; i < 128; i++)
        if (got.rdata[8*i +: 8] !== exp_r.rdata[8*i +: 8]) begin
          fail($sformatf("RD pa %h byte %0d got %h want %h (l1=%0d uc=%0d)", r.pa, i,
                         got.rdata[8*i +: 8], exp_r.rdata[8*i +: 8], use_l1, r.uc));
          break;
        end
    end
    if (r.kind == M_AMO && got.old !== exp_r.old)
      fail($sformatf("AMO op %0d w64 %0d rtn %0d pa %h old got %h want %h", r.amo_op,
                     r.amo64, r.amo_rtn, r.pa, got.old, exp_r.old));
  endtask

  // error-path operation: no reference update, only the err code is checked
  task automatic op_err(input mreq_t r, input logic [1:0] want);
    mrsp_t got;
    issue(r, got);
    checks++;
    if (got.err !== want)
      fail($sformatf("err op kind %0d pa %h: err got %0d want %0d", r.kind, r.pa, got.err, want));
  endtask

  // ---- address pools ------------------------------------------------------------------------
  localparam int HOT_N = 36, UC_N = 8;
  logic [47:0] hot [HOT_N];
  logic [47:0] uchot [UC_N];
  longint fresh_ctr;

  task automatic init_pools();
    for (int i = 0; i < HOT_N; i++) begin
      logic [47:0] g;
      g = 48'($urandom_range(0, 15));
      if (i >= 28 && i < 32)      hot[i] = 48'h0000_0002_0000_0000 + (g << 12);
      else if (i >= 32)           hot[i] = 48'h0000_0003_FFFF_0000 + (g << 12);
      else                        hot[i] = g << 12;
      hot[i] = hot[i] | (48'($urandom_range(0, 31)) << 7);
    end
    for (int i = 0; i < UC_N; i++) begin
      uchot[i] = ((i < 4) ? 48'h0010_0000_0000 : 48'h8000_0000_0000)
                 + (48'($urandom_range(0, 3)) << 12) + (48'($urandom_range(0, 3)) << 7);
    end
    fresh_ctr = 0;
  endtask

  // a never-touched line of a given slice (far VRAM window 0x1_0000_0000+)
  function automatic logic [47:0] fresh_line(input int slice, input int line);
    int slot;
    slot = 0;
    for (int sl = 0; sl < 4; sl++) if (l2_slice_idx(48'(sl) << 12) == slice) slot = sl;
    return 48'h0000_0001_0000_0000 + ((48'(fresh_ctr) * 4 + 48'(slot)) << 12) +
           (48'(line) << 7);
  endfunction

  // ---- random request ---------------------------------------------------------------------------
  task automatic gen_req(output mreq_t r);
    int roll, sub, style;
    logic [47:0] pa, base;
    logic [4:0] o;
    logic w64;
    int sz, off;
    logic [63:0] av, cv, old;
    roll = $urandom_range(0, 99);
    if (roll >= 80) begin                           // uc
      base = uchot[$urandom_range(0, UC_N - 1)];
      sub = $urandom_range(0, 2);
      roll = (sub == 0) ? 0 : (sub == 1) ? 30 : 60;
    end else begin
      // half of the traffic on 6 lines so that the tiny L1D actually hits
      base = hot[($urandom_range(0, 1) == 1) ? $urandom_range(0, 5) : $urandom_range(0, HOT_N - 1)];
    end
    if (roll < 22) begin
      r = mk_rd(base);
    end else if (roll < 52) begin
      style = ($urandom_range(0, 11) == 0) ? 1 : $urandom_range(0, 7);
      r = mk_wr(base, rnd_be(style), rnd1024());
    end else if (roll < 72) begin
      o   = 5'($urandom_range(0, 12));
      w64 = (o == AMO_INC || o == AMO_DEC) ? 1'b0 : ($urandom_range(0, 1) == 1);
      sz  = w64 ? 8 : 4;
      off = ($urandom_range(0, 4) == 0) ? $urandom_range(0, 128 - sz)
                                        : (sz * $urandom_range(0, 128 / sz - 1));
      pa  = base + 48'(off);
      av  = ($urandom_range(0, 1) == 1) ? 64'($urandom_range(0, 40)) : {$urandom, $urandom};
      old = val_at(pa, sz);
      cv  = ($urandom_range(0, 1) == 1) ? old : {$urandom, $urandom};
      r = mk_amo(pa, o, w64, ($urandom_range(0, 9) < 7), av, cv);
    end else if (roll < 75) begin
      r = base_req(M_INV_L2, '0);
    end else if (roll < 78) begin
      r = base_req(M_WB_L2, '0);
    end else begin
      r = base_req(M_INV_L1, '0);
    end
    r.pol     = ($urandom_range(0, 9) < 6) ? 2'd0 : 2'($urandom_range(1, 3));
    r.noalloc = ($urandom_range(0, 9) == 0);
  endtask

  task automatic gap();
    repeat ($urandom_range(0, 3)) @(negedge clk);
  endtask

  task automatic set_mode(input bit via_l1, input int bp, input bit shuf);
    @(negedge clk);
    use_l1  = via_l1;
    bp_mode = bp;
    shuf_en = shuf;
    if (via_l1) op(base_req(M_INV_L1, '0));   // direct phases leave stale lines behind
  endtask

  task automatic random_phase(input bit via_l1, input int bp, input bit shuf, input int n);
    mreq_t r;
    set_mode(via_l1, bp, shuf);
    for (int i = 0; i < n; i++) begin
      gen_req(r);
      // L1D phases: a third of the requests are plain reads of the tight pool (L1 hits)
      if (via_l1 && $urandom_range(0, 2) == 0) r = mk_rd(hot[$urandom_range(0, 5)]);
      op(r);
      gap();
    end
  endtask

  // ---- counters snapshot helpers -----------------------------------------------------------------
  int s_sreq [4], s_ar [4], s_aw [4];
  task automatic snap();
    for (int t = 0; t < 4; t++) begin
      s_sreq[t] = sreq_cnt[t]; s_ar[t] = ar_cnt[t]; s_aw[t] = aw_cnt[t];
    end
  endtask
  // the in-flight DDR / flit counters settle a cycle or two after the response
  task automatic settle();
    repeat (6) @(negedge clk);
  endtask
  task automatic expect_delta(input string what, input int slice, input int nreq, input int nar);
    settle();
    for (int t = 0; t < 4; t++) begin
      checks++;
      if (sreq_cnt[t] - s_sreq[t] != ((t == slice) ? nreq : 0))
        fail($sformatf("%s: slice %0d got %0d flits want %0d", what, t,
                       sreq_cnt[t] - s_sreq[t], (t == slice) ? nreq : 0));
      if (nar >= 0 && (ar_cnt[t] - s_ar[t] != ((t == slice) ? nar : 0)))
        fail($sformatf("%s: slice %0d got %0d DDR reads want %0d", what, t,
                       ar_cnt[t] - s_ar[t], (t == slice) ? nar : 0));
    end
  endtask
  function automatic int total_sreq();
    return sreq_cnt[0] + sreq_cnt[1] + sreq_cnt[2] + sreq_cnt[3];
  endfunction

  // ---- directed tests ------------------------------------------------------------------------------
  task automatic directed();
    mreq_t r;
    logic [1023:0] d1, d2;
    logic [47:0] pa;
    int t0, ooo0, cnt0;
    set_mode(1'b0, 0, 1'b0);

    // D1: write with be == 0 completes without any flit
    $display("TB: D1 zero-be write");
    snap();
    r = mk_wr(fresh_line(0, 3), 128'd0, rnd1024());
    t0 = n_ti_flits;
    op(r);
    checks++;
    if (n_ti_flits != t0) fail("D1: be==0 write sent flits");
    expect_delta("D1", 0, 0, 0);
    // ... and through L1D
    set_mode(1'b1, 0, 1'b0);
    t0 = n_ti_flits;
    op(mk_wr(fresh_line(1, 3), 128'd0, rnd1024()));
    checks++;
    if (n_ti_flits != t0) fail("D1: be==0 write via L1D sent flits");
    set_mode(1'b0, 0, 1'b0);

    // D2: reads reach their slice, 16 chunks, one DDR fill then hits
    $display("TB: D2 per-slice reads");
    for (int s = 0; s < 4; s++) begin
      pa = fresh_line(s, 5);
      fresh_ctr++;
      snap();
      op(mk_rd(pa));
      expect_delta($sformatf("D2 rd1 slice %0d", s), s, 16, 1);
      snap();
      op(mk_rd(pa));
      expect_delta($sformatf("D2 rd2 slice %0d (hit)", s), s, 16, 0);
      // flit counts of the other kinds
      snap();
      op(mk_wr(pa, ({127'd0, 1'b1} | (128'd1 << 24) | (128'd1 << 127)), rnd1024()));
      expect_delta($sformatf("D2 wr 3 chunks slice %0d", s), s, 3, 0);
      snap();
      op(mk_amo(pa + 48'd8, AMO_ADD, 1'b0, 1'b1, 64'd5, '0));
      expect_delta($sformatf("D2 amo slice %0d", s), s, 1, 0);
      snap();
      op(mk_wr(pa, '1, rnd1024()));
      expect_delta($sformatf("D2 wr 16 chunks slice %0d", s), s, 16, 0);
    end

    // D3: INV_L2 keeps the data (write-back first), WB_L2 keeps lines valid
    $display("TB: D3 inv/wb preserve data");
    for (int s = 0; s < 4; s++) begin
      pa = fresh_line(s, 7);
      fresh_ctr++;
      d1 = rnd1024();
      op(mk_wr(pa, '1, d1));
      checks++;
      begin
        bit differs;
        differs = 1'b0;
        for (int i = 0; i < 128; i++) if (peek_ddr(pa + 48'(i)) !== d1[8*i +: 8]) differs = 1'b1;
        if (!differs) fail($sformatf("D3 slice %0d: dirty line already in DDR before inv", s));
      end
      op(base_req(M_INV_L2, '0));
      checks++;
      for (int i = 0; i < 128; i++)
        if (peek_ddr(pa + 48'(i)) !== d1[8*i +: 8]) begin
          fail($sformatf("D3 slice %0d: INV_L2 did not write back byte %0d", s, i));
          break;
        end
      snap();
      op(mk_rd(pa));                       // data still right, refilled from DDR
      expect_delta($sformatf("D3 slice %0d post-inv rd", s), s, 16, 1);
      d2 = rnd1024();
      op(mk_wr(pa, rnd_be(2), d2));
      op(base_req(M_WB_L2, '0));
      snap();
      op(mk_rd(pa));                       // WB keeps the line valid -> hit
      expect_delta($sformatf("D3 slice %0d post-wb rd", s), s, 16, 0);
      checks++;
      for (int i = 0; i < 128; i++)
        if (peek_ddr(pa + 48'(i)) !== gb(1'b0, pa + 48'(i))) begin
          fail($sformatf("D3 slice %0d: WB_L2 DDR byte %0d != shadow", s, i));
          break;
        end
    end

    // D4: maintenance fan-out reaches every slice; responses return out of order
    $display("TB: D4 maintenance fan-out / reorder");
    op(base_req(M_WB_L2, '0));
    pa = fresh_line(0, 0);
    fresh_ctr++;
    for (int i = 0; i < 4; i++) op(mk_wr(pa + 48'(i << 7), '1, rnd1024()));   // slice 0 dirty
    ooo0 = ooo_cnt;
    snap();
    op(base_req(M_WB_L2, '0));
    settle();
    for (int t = 0; t < 4; t++) begin
      checks++;
      if (sreq_cnt[t] - s_sreq[t] != 1)
        fail($sformatf("D4: slice %0d got %0d maintenance flits", t, sreq_cnt[t] - s_sreq[t]));
    end
    checks++;
    if (ooo_cnt == ooo0) fail("D4: slice 0 (dirty) did not answer after the clean slices");
    else $display("TB: D4 out-of-order maintenance responses observed: %0d", ooo_cnt - ooo0);
    for (int i = 0; i < 4 * 128; i++) begin
      checks++;
      if (peek_ddr(pa + 48'(i)) !== gb(1'b0, pa + 48'(i))) begin
        fail($sformatf("D4: DDR byte %0d != shadow after WB_L2", i));
        break;
      end
    end
    // same with INV_L2 and through the response shuffle
    for (int i = 0; i < 4; i++) op(mk_wr(pa + 48'(i << 7), rnd_be(2), rnd1024()));
    set_mode(1'b0, 0, 1'b1);
    ooo0 = ooo_cnt;
    op(base_req(M_INV_L2, '0));
    for (int i = 0; i < 4; i++) op(mk_rd(pa + 48'(i << 7)));
    op(mk_rd(hot[0]));
    op(mk_rd(hot[1]));
    set_mode(1'b0, 0, 1'b0);

    // D5: eviction write-back (6 lines of one set of slice 0, 2 ways)
    $display("TB: D5 eviction");
    snap();
    for (int i = 0; i < 6; i++) begin
      pa = fresh_line(0, 2);
      fresh_ctr++;
      op(mk_wr(pa, rnd_be(2), rnd1024()));
    end
    settle();
    checks++;
    if (aw_cnt[0] - s_aw[0] < 3)
      fail($sformatf("D5: only %0d write-backs in slice 0", aw_cnt[0] - s_aw[0]));

    // D6: AMO sweep, all ops x sizes x slices x rtn
    $display("TB: D6 AMO sweep");
    for (int o = 0; o <= 12; o++) begin
      for (int w = 0; w < 2; w++) begin
        if (w == 1 && (o == AMO_INC || o == AMO_DEC)) continue;
        for (int s = 0; s < 4; s++) begin
          for (int rtn = 0; rtn < 2; rtn++) begin
            pa = fresh_line(s, 9) + 48'(w ? 8 : 4) * 48'(o);
            if (rtn == 0) fresh_ctr++;   // keep the same address for rtn=1 (reads back)
            op(mk_wr(pa & ~48'h7F, rnd_be(2), rnd1024()));
            op(mk_amo(pa, 5'(o), w[0], rtn[0], {$urandom, $urandom},
                      ($urandom_range(0, 1) == 1) ? val_at(pa, w ? 8 : 4) : {$urandom, $urandom}));
            op(mk_rd(pa & ~48'h7F));
          end
        end
      end
    end

    // D7: error paths (slice err comes back through the adapter)
    $display("TB: D7 errors");
    op(mk_rd(48'h0000_0003_FFFF_FF80));              // last line of VRAM is legal
    r = base_req(M_RD, 48'h0000_0004_0000_0000);
    r.uc = 1'b0;                                    // not VRAM, but forced into the NoC
    op_err(r, ERR_BAD_PA);
    r = mk_amo(fresh_line(1, 1) + 48'd124, AMO_ADD, 1'b1, 1'b1, 64'd1, '0);
    op_err(r, ERR_CROSS);
    r = mk_amo(fresh_line(2, 1), AMO_INC, 1'b1, 1'b1, 64'd1, '0);
    op_err(r, ERR_BAD_PA);
    r = mk_amo(fresh_line(3, 1), AMO_NONE, 1'b0, 1'b1, 64'd1, '0);
    op_err(r, ERR_BAD_PA);

    // D8: uc requests bypass the NoC; M_INV_L1 at the adapter is just acked
    $display("TB: D8 uc bypass / inv.l1 ack");
    snap();
    cnt0 = byp_cnt;
    t0 = n_ti_flits;
    for (int i = 0; i < UC_N; i++) begin
      d1 = rnd1024();
      op(mk_wr(uchot[i], rnd_be(2), d1));
      op(mk_rd(uchot[i]));
      op(mk_amo(uchot[i] + 48'd16, AMO_ADD, i[0], 1'b1, {$urandom, $urandom}, '0));
      op(mk_amo(uchot[i] + 48'd32, AMO_CAS, i[1], 1'b1, 64'd7, val_at(uchot[i] + 48'd32, i[1] ? 8 : 4)));
      op(mk_rd(uchot[i]));
    end
    checks++;
    if (n_ti_flits != t0 || total_sreq() != s_sreq[0] + s_sreq[1] + s_sreq[2] + s_sreq[3])
      fail("D8: uc request produced NoC traffic");
    checks++;
    if (byp_cnt - cnt0 != 5 * UC_N) fail($sformatf("D8: %0d bypass requests", byp_cnt - cnt0));
    t0 = n_ti_flits;
    op(base_req(M_INV_L1, '0));
    checks++;
    if (n_ti_flits != t0) fail("D8: INV_L1 produced flits");

    // D9: heavy backpressure on the request stream, dense 16-chunk traffic
    $display("TB: D9 backpressure");
    set_mode(1'b0, 2, 1'b0);
    for (int s = 0; s < 4; s++) begin
      pa = fresh_line(s, 11);
      fresh_ctr++;
      op(mk_wr(pa, '1, rnd1024()));
      op(mk_rd(pa));
      op(base_req(M_WB_L2, '0));
    end
    set_mode(1'b0, 1, 1'b1);
    for (int s = 0; s < 4; s++) begin
      pa = fresh_line(s, 12);
      fresh_ctr++;
      op(mk_wr(pa, '1, rnd1024()));
      op(mk_rd(pa));
    end
    set_mode(1'b0, 0, 1'b0);
  endtask

  // ---- final check -------------------------------------------------------------------------------------
  task automatic final_check();
    int nb, bad;
    set_mode(1'b0, 0, 1'b0);
    op(base_req(M_WB_L2, '0));
    nb = 0; bad = 0;
    foreach (shadow[k]) begin
      logic [7:0] got;
      nb++;
      if (k < L2_VRAM_BYTES) got = peek_ddr(k);
      else                   got = gb(1'b1, k);
      if (got !== shadow[k]) begin
        bad++;
        if (bad <= 10) fail($sformatf("final: pa %h mem %h shadow %h", k, got, shadow[k]));
      end
    end
    checks += nb;
    $display("TB: final DDR/uc mirror check: %0d bytes, %0d mismatches", nb, bad);
  endtask

  // ---- watchdog -------------------------------------------------------------------------------------------
  initial begin
    #2_000_000_000;
    $display("NG_LSU_NOC_L2_TB_FAIL: absolute watchdog timeout");
    $finish;
  end

  // ---- main -----------------------------------------------------------------------------------------------
  initial begin
    int seed;
    errors = 0; checks = 0; bp_cycles = 0; ooo_cnt = 0; last_tag = -1; n_ti_flits = 0;
    prev_stall = 1'b0;
    bp_mode = 0; shuf_en = 1'b0; use_l1 = 1'b0;
    l1_req_valid = 1'b0; l1_req = '0; d_req_valid = 1'b0; d_req = '0;
    for (int i = 0; i < 8; i++) n_op[i] = 0;
    if ($value$plusargs("SEED=%d", seed)) begin int dummy; dummy = $urandom(seed); end

    rst_n = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);
    init_pools();

    directed();

    $display("TB: random phases (%0d ops each)", OPS_PER_PHASE);
    random_phase(1'b0, 0, 1'b0, OPS_PER_PHASE);
    $display("TB: phase direct/plain done (%0d checks)", checks);
    random_phase(1'b0, 1, 1'b1, OPS_PER_PHASE);
    $display("TB: phase direct/gate+shuffle done (%0d checks)", checks);
    random_phase(1'b1, 0, 1'b0, OPS_PER_PHASE);
    $display("TB: phase L1D/plain done (%0d checks)", checks);
    random_phase(1'b1, 2, 1'b1, OPS_PER_PHASE);
    $display("TB: phase L1D/heavy gate+shuffle done (%0d checks)", checks);
    random_phase(1'b0, 1, 1'b0, OPS_PER_PHASE);

    final_check();

    $display("TB: ops rd=%0d wr=%0d amo=%0d inv1=%0d inv2=%0d wb2=%0d uc=%0d zero_be=%0d",
             n_op[0], n_op[1], n_op[2], n_op[3], n_op[4], n_op[5], n_op[6], n_op[7]);
    $display("TB: flits=%0d bp_stall_cycles=%0d ooo=%0d L1 hit/miss/fill/inv=%0d/%0d/%0d/%0d",
             n_ti_flits, bp_cycles, ooo_cnt, st_hit, st_miss, st_fill, st_inv);
    $display("TB: DDR reads %0d/%0d/%0d/%0d writes %0d/%0d/%0d/%0d", ar_cnt[0], ar_cnt[1],
             ar_cnt[2], ar_cnt[3], aw_cnt[0], aw_cnt[1], aw_cnt[2], aw_cnt[3]);
    checks++;
    if (bp_cycles == 0) fail("no ti_ready backpressure observed");
    checks++;
    if (st_hit == 0 || st_fill == 0) fail("L1D never hit / filled");

    if (errors == 0)
      $display("NG_LSU_NOC_L2_TB_PASS: %0d checks, 0 errors", checks);
    else
      $display("NG_LSU_NOC_L2_TB_FAIL: %0d errors of %0d checks", errors, checks);
    $finish;
  end
endmodule
