// tb_noc_l2map.sv -- integration testbench: 3x2 NoC mesh connecting two
// requestor agents to four L2 slices (slice 0/1 -> C1 halves, 512b AXI;
// slice 2 -> C2 and slice 3 -> C4, 256b AXI), each with its own DDR memory
// model.  Checks, end to end:
//   * golden interleave routing: PA -> slice node (
//     vectors from rtl/gpu/tb/golden/), including the DDR address each slice
//     drives (channel offset);
//   * pipelined random RD/WR/AMO soak from both agents (disjoint windows)
//     against a reference model;
//   * cross-agent atomicity: shared-address AMO ADD from both agents
//     serializes at one slice (rtn-value multiset invariant);
//   * deadlock watchdog.
// Expect NG_NOC_L2MAP_TB_PASS.
`timescale 1ns/1ps
module tb_noc_l2map;
  import ng_noc_pkg::*;
  import ng_l2_addrmap::*;

  localparam int GW = 3, GH = 2, NN = GW * GH;
  localparam int BUF_DEPTH = 4, LOC_CREDITS = 4;
  localparam int MAXOUT = 4;

  localparam int AGENT_NODE [2] = '{0, 3};
  localparam int SLICE_NODE [4] = '{1, 2, 4, 5};

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

  // ---- slice-side streams + endpoint adapters ------------------------------
  logic  sl_req_v [4];
  logic  sl_req_r [4];
  flit_t sl_req_f [4];
  logic  sl_resp_v [4];
  logic  sl_resp_r [4];
  flit_t sl_resp_f [4];

  for (genvar s = 0; s < 4; s++) begin : g_sep
    localparam int NODE = SLICE_NODE[s];
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
  end

  // ---- DDR read monitors per slice -----------------------------------------
  int          ar_cnt [4];
  logic [47:0] last_ar [4];

  // ---- slice 0 (C1 even, node 1): 512b --------------------------------------
  logic              s0_arv, s0_arr, s0_rv, s0_rr, s0_rl;
  logic [32:0]       s0_araddr, s0_awaddr;
  logic [7:0]        s0_arlen, s0_awlen;
  logic [2:0]        s0_arsize, s0_awsize;
  logic [1:0]        s0_arburst, s0_awburst;
  logic [5:0]        s0_arid, s0_rid, s0_awid, s0_bid;
  logic [511:0]      s0_rdata, s0_wdata;
  logic [1:0]        s0_rresp, s0_bresp;
  logic              s0_awv, s0_awr, s0_wv, s0_wr, s0_wl, s0_bv, s0_br;
  logic [63:0]       s0_wstrb;

  ng_l2_slice #(.SETS(16), .WAYS(4), .AXI_AW(33), .AXI_DW(512), .AXI_IW(6),
                .SLICE_ID(SLICE_NODE[0])) u_s0 (
    .clk(clk), .rst_n(rst_n),
    .s_req_valid(sl_req_v[0]), .s_req_ready(sl_req_r[0]), .s_req_flit(sl_req_f[0]),
    .m_resp_valid(sl_resp_v[0]), .m_resp_ready(sl_resp_r[0]),
    .m_resp_flit(sl_resp_f[0]),
    .arvalid(s0_arv), .arready(s0_arr), .araddr(s0_araddr), .arlen(s0_arlen),
    .arsize(s0_arsize), .arburst(s0_arburst), .arid(s0_arid),
    .rvalid(s0_rv), .rready(s0_rr), .rdata(s0_rdata), .rlast(s0_rl),
    .rid(s0_rid), .rresp(s0_rresp),
    .awvalid(s0_awv), .awready(s0_awr), .awaddr(s0_awaddr), .awlen(s0_awlen),
    .awsize(s0_awsize), .awburst(s0_awburst), .awid(s0_awid),
    .wvalid(s0_wv), .wready(s0_wr), .wdata(s0_wdata), .wstrb(s0_wstrb),
    .wlast(s0_wl),
    .bvalid(s0_bv), .bready(s0_br), .bid(s0_bid), .bresp(s0_bresp)
  );
  tb_l2_axi_mem_model #(.AW(33), .DW(512), .IW(6)) u_m0 (
    .clk(clk), .rst_n(rst_n),
    .arvalid(s0_arv), .arready(s0_arr), .araddr(s0_araddr), .arlen(s0_arlen),
    .arsize(s0_arsize), .arburst(s0_arburst), .arid(s0_arid),
    .rvalid(s0_rv), .rready(s0_rr), .rdata(s0_rdata), .rlast(s0_rl),
    .rid(s0_rid), .rresp(s0_rresp),
    .awvalid(s0_awv), .awready(s0_awr), .awaddr(s0_awaddr), .awlen(s0_awlen),
    .awsize(s0_awsize), .awburst(s0_awburst), .awid(s0_awid),
    .wvalid(s0_wv), .wready(s0_wr), .wdata(s0_wdata), .wstrb(s0_wstrb),
    .wlast(s0_wl),
    .bvalid(s0_bv), .bready(s0_br), .bid(s0_bid), .bresp(s0_bresp)
  );
  always_ff @(posedge clk)
    if (rst_n && s0_arv && s0_arr) begin
      ar_cnt[0] <= ar_cnt[0] + 1;
      last_ar[0] <= s0_araddr;
    end

  // ---- slice 1 (C1 odd, node 2): 512b ---------------------------------------
  logic              s1_arv, s1_arr, s1_rv, s1_rr, s1_rl;
  logic [32:0]       s1_araddr, s1_awaddr;
  logic [7:0]        s1_arlen, s1_awlen;
  logic [2:0]        s1_arsize, s1_awsize;
  logic [1:0]        s1_arburst, s1_awburst;
  logic [5:0]        s1_arid, s1_rid, s1_awid, s1_bid;
  logic [511:0]      s1_rdata, s1_wdata;
  logic [1:0]        s1_rresp, s1_bresp;
  logic              s1_awv, s1_awr, s1_wv, s1_wr, s1_wl, s1_bv, s1_br;
  logic [63:0]       s1_wstrb;

  ng_l2_slice #(.SETS(16), .WAYS(4), .AXI_AW(33), .AXI_DW(512), .AXI_IW(6),
                .SLICE_ID(SLICE_NODE[1])) u_s1 (
    .clk(clk), .rst_n(rst_n),
    .s_req_valid(sl_req_v[1]), .s_req_ready(sl_req_r[1]), .s_req_flit(sl_req_f[1]),
    .m_resp_valid(sl_resp_v[1]), .m_resp_ready(sl_resp_r[1]),
    .m_resp_flit(sl_resp_f[1]),
    .arvalid(s1_arv), .arready(s1_arr), .araddr(s1_araddr), .arlen(s1_arlen),
    .arsize(s1_arsize), .arburst(s1_arburst), .arid(s1_arid),
    .rvalid(s1_rv), .rready(s1_rr), .rdata(s1_rdata), .rlast(s1_rl),
    .rid(s1_rid), .rresp(s1_rresp),
    .awvalid(s1_awv), .awready(s1_awr), .awaddr(s1_awaddr), .awlen(s1_awlen),
    .awsize(s1_awsize), .awburst(s1_awburst), .awid(s1_awid),
    .wvalid(s1_wv), .wready(s1_wr), .wdata(s1_wdata), .wstrb(s1_wstrb),
    .wlast(s1_wl),
    .bvalid(s1_bv), .bready(s1_br), .bid(s1_bid), .bresp(s1_bresp)
  );
  tb_l2_axi_mem_model #(.AW(33), .DW(512), .IW(6)) u_m1 (
    .clk(clk), .rst_n(rst_n),
    .arvalid(s1_arv), .arready(s1_arr), .araddr(s1_araddr), .arlen(s1_arlen),
    .arsize(s1_arsize), .arburst(s1_arburst), .arid(s1_arid),
    .rvalid(s1_rv), .rready(s1_rr), .rdata(s1_rdata), .rlast(s1_rl),
    .rid(s1_rid), .rresp(s1_rresp),
    .awvalid(s1_awv), .awready(s1_awr), .awaddr(s1_awaddr), .awlen(s1_awlen),
    .awsize(s1_awsize), .awburst(s1_awburst), .awid(s1_awid),
    .wvalid(s1_wv), .wready(s1_wr), .wdata(s1_wdata), .wstrb(s1_wstrb),
    .wlast(s1_wl),
    .bvalid(s1_bv), .bready(s1_br), .bid(s1_bid), .bresp(s1_bresp)
  );
  always_ff @(posedge clk)
    if (rst_n && s1_arv && s1_arr) begin
      ar_cnt[1] <= ar_cnt[1] + 1;
      last_ar[1] <= s1_araddr;
    end

  // ---- slice 2 (C2, node 4): 256b --------------------------------------------
  logic              s2_arv, s2_arr, s2_rv, s2_rr, s2_rl;
  logic [31:0]       s2_araddr, s2_awaddr;
  logic [7:0]        s2_arlen, s2_awlen;
  logic [2:0]        s2_arsize, s2_awsize;
  logic [1:0]        s2_arburst, s2_awburst;
  logic [5:0]        s2_arid, s2_rid, s2_awid, s2_bid;
  logic [255:0]      s2_rdata, s2_wdata;
  logic [1:0]        s2_rresp, s2_bresp;
  logic              s2_awv, s2_awr, s2_wv, s2_wr, s2_wl, s2_bv, s2_br;
  logic [31:0]       s2_wstrb;

  ng_l2_slice #(.SETS(16), .WAYS(4), .AXI_AW(32), .AXI_DW(256), .AXI_IW(6),
                .SLICE_ID(SLICE_NODE[2])) u_s2 (
    .clk(clk), .rst_n(rst_n),
    .s_req_valid(sl_req_v[2]), .s_req_ready(sl_req_r[2]), .s_req_flit(sl_req_f[2]),
    .m_resp_valid(sl_resp_v[2]), .m_resp_ready(sl_resp_r[2]),
    .m_resp_flit(sl_resp_f[2]),
    .arvalid(s2_arv), .arready(s2_arr), .araddr(s2_araddr), .arlen(s2_arlen),
    .arsize(s2_arsize), .arburst(s2_arburst), .arid(s2_arid),
    .rvalid(s2_rv), .rready(s2_rr), .rdata(s2_rdata), .rlast(s2_rl),
    .rid(s2_rid), .rresp(s2_rresp),
    .awvalid(s2_awv), .awready(s2_awr), .awaddr(s2_awaddr), .awlen(s2_awlen),
    .awsize(s2_awsize), .awburst(s2_awburst), .awid(s2_awid),
    .wvalid(s2_wv), .wready(s2_wr), .wdata(s2_wdata), .wstrb(s2_wstrb),
    .wlast(s2_wl),
    .bvalid(s2_bv), .bready(s2_br), .bid(s2_bid), .bresp(s2_bresp)
  );
  tb_l2_axi_mem_model #(.AW(32), .DW(256), .IW(6)) u_m2 (
    .clk(clk), .rst_n(rst_n),
    .arvalid(s2_arv), .arready(s2_arr), .araddr(s2_araddr), .arlen(s2_arlen),
    .arsize(s2_arsize), .arburst(s2_arburst), .arid(s2_arid),
    .rvalid(s2_rv), .rready(s2_rr), .rdata(s2_rdata), .rlast(s2_rl),
    .rid(s2_rid), .rresp(s2_rresp),
    .awvalid(s2_awv), .awready(s2_awr), .awaddr(s2_awaddr), .awlen(s2_awlen),
    .awsize(s2_awsize), .awburst(s2_awburst), .awid(s2_awid),
    .wvalid(s2_wv), .wready(s2_wr), .wdata(s2_wdata), .wstrb(s2_wstrb),
    .wlast(s2_wl),
    .bvalid(s2_bv), .bready(s2_br), .bid(s2_bid), .bresp(s2_bresp)
  );
  always_ff @(posedge clk)
    if (rst_n && s2_arv && s2_arr) begin
      ar_cnt[2] <= ar_cnt[2] + 1;
      last_ar[2] <= s2_araddr;
    end

  // ---- slice 3 (C4, node 5): 256b --------------------------------------------
  logic              s3_arv, s3_arr, s3_rv, s3_rr, s3_rl;
  logic [31:0]       s3_araddr, s3_awaddr;
  logic [7:0]        s3_arlen, s3_awlen;
  logic [2:0]        s3_arsize, s3_awsize;
  logic [1:0]        s3_arburst, s3_awburst;
  logic [5:0]        s3_arid, s3_rid, s3_awid, s3_bid;
  logic [255:0]      s3_rdata, s3_wdata;
  logic [1:0]        s3_rresp, s3_bresp;
  logic              s3_awv, s3_awr, s3_wv, s3_wr, s3_wl, s3_bv, s3_br;
  logic [31:0]       s3_wstrb;

  ng_l2_slice #(.SETS(16), .WAYS(4), .AXI_AW(32), .AXI_DW(256), .AXI_IW(6),
                .SLICE_ID(SLICE_NODE[3])) u_s3 (
    .clk(clk), .rst_n(rst_n),
    .s_req_valid(sl_req_v[3]), .s_req_ready(sl_req_r[3]), .s_req_flit(sl_req_f[3]),
    .m_resp_valid(sl_resp_v[3]), .m_resp_ready(sl_resp_r[3]),
    .m_resp_flit(sl_resp_f[3]),
    .arvalid(s3_arv), .arready(s3_arr), .araddr(s3_araddr), .arlen(s3_arlen),
    .arsize(s3_arsize), .arburst(s3_arburst), .arid(s3_arid),
    .rvalid(s3_rv), .rready(s3_rr), .rdata(s3_rdata), .rlast(s3_rl),
    .rid(s3_rid), .rresp(s3_rresp),
    .awvalid(s3_awv), .awready(s3_awr), .awaddr(s3_awaddr), .awlen(s3_awlen),
    .awsize(s3_awsize), .awburst(s3_awburst), .awid(s3_awid),
    .wvalid(s3_wv), .wready(s3_wr), .wdata(s3_wdata), .wstrb(s3_wstrb),
    .wlast(s3_wl),
    .bvalid(s3_bv), .bready(s3_br), .bid(s3_bid), .bresp(s3_bresp)
  );
  tb_l2_axi_mem_model #(.AW(32), .DW(256), .IW(6)) u_m3 (
    .clk(clk), .rst_n(rst_n),
    .arvalid(s3_arv), .arready(s3_arr), .araddr(s3_araddr), .arlen(s3_arlen),
    .arsize(s3_arsize), .arburst(s3_arburst), .arid(s3_arid),
    .rvalid(s3_rv), .rready(s3_rr), .rdata(s3_rdata), .rlast(s3_rl),
    .rid(s3_rid), .rresp(s3_rresp),
    .awvalid(s3_awv), .awready(s3_awr), .awaddr(s3_awaddr), .awlen(s3_awlen),
    .awsize(s3_awsize), .awburst(s3_awburst), .awid(s3_awid),
    .wvalid(s3_wv), .wready(s3_wr), .wdata(s3_wdata), .wstrb(s3_wstrb),
    .wlast(s3_wl),
    .bvalid(s3_bv), .bready(s3_br), .bid(s3_bid), .bresp(s3_bresp)
  );
  always_ff @(posedge clk)
    if (rst_n && s3_arv && s3_arr) begin
      ar_cnt[3] <= ar_cnt[3] + 1;
      last_ar[3] <= s3_araddr;
    end

  // ---- requestor agents ------------------------------------------------------
  logic  ti_v [2];
  logic  ti_r [2];
  flit_t ti_f [2];
  logic  to_v [2];
  logic  to_r [2];
  flit_t to_f [2];

  flit_t          sendq [2][$];
  logic [63:0]    junkv;
  logic [63:0]    expd  [2][int];
  int             mode  [2][int];   // 1: exact data check, 2: sum rtn, 0: none
  int             outst [2];
  longint         amo_sum [2];
  logic [5:0]     last_src [2];
  int             delivered;
  int             errors, checks;
  int             active;

  for (genvar ag = 0; ag < 2; ag++) begin : g_ag
    localparam int NODE = AGENT_NODE[ag];
    logic  inj_v;
    flit_t inj_f;
    ng_noc_ep #(
      .NUM_VC(2), .INJ_CREDITS(BUF_DEPTH), .RX_DEPTH(2 * LOC_CREDITS)
    ) u_ep (
      .clk(clk), .rst_n(rst_n),
      .ti_valid(ti_v[ag]), .ti_ready(ti_r[ag]), .ti_flit(ti_f[ag]),
      .to_valid(to_v[ag]), .to_ready(to_r[ag]), .to_flit(to_f[ag]),
      .inj_valid(inj_v), .inj_flit(inj_f), .inj_credit(ep_inj_credit[NODE]),
      .recv_valid(ep_recv_valid[NODE]), .recv_flit(ep_recv_flit[NODE]),
      .recv_credit(ep_recv_credit[NODE])
    );
    assign ep_inj_valid[NODE] = inj_v;
    assign ep_inj_flit[NODE]  = inj_f;

    // sender
    initial begin
      ti_v[ag] = 1'b0;
      forever begin
        @(negedge clk);
        if (sendq[ag].size() > 0 && outst[ag] < MAXOUT) begin
          automatic flit_t f;
          f = sendq[ag].pop_front();
          outst[ag]++;
          ti_v[ag] = 1'b1;
          ti_f[ag] = f;
          while (!ti_r[ag]) @(negedge clk);
          @(negedge clk);
          ti_v[ag] = 1'b0;
        end
      end
    end

    // receiver
    initial begin
      to_r[ag] = 1'b1;
      forever begin
        @(negedge clk);
        if (to_v[ag]) begin
          automatic flit_t f;
          f = to_f[ag];
          last_src[ag] = f.src;
          checks++;
          if (f.err != ERR_NONE) begin
            errors++;
            $display("TB_FAIL l2map: agent %0d resp err=%0d tag=%0d", ag, f.err, f.tag);
          end else if (mode[ag].exists(int'(f.tag))) begin
            if (mode[ag][int'(f.tag)] == 1) begin
              if (f.data !== expd[ag][int'(f.tag)]) begin
                errors++;
                $display("TB_FAIL l2map: agent %0d tag=%0d data=%h want=%h",
                         ag, f.tag, f.data, expd[ag][int'(f.tag)]);
              end
            end else if (mode[ag][int'(f.tag)] == 2) begin
              amo_sum[ag] += longint'(f.data);
            end
            mode[ag].delete(int'(f.tag));
            expd[ag].delete(int'(f.tag));
          end
          outst[ag]--;
          delivered++;
        end
      end
    end
  end

  // ---- reference model (VRAM PA keyed, matches the per-channel DDR models) --
  logic [7:0] refmem [logic [47:0]];

  function automatic logic [7:0] hash48(input logic [47:0] a);
    logic [63:0] p;
    p = a * 64'h9E3779B97F4A7C15;
    return p[55:48];
  endfunction

  function automatic logic [7:0] refbyte(input logic [47:0] pa);
    return refmem.exists(pa) ? refmem[pa] : hash48(l2_ch_of(pa).off);
  endfunction

  function automatic logic [63:0] ref_alu(input logic [4:0] o, input logic w64,
                                          input logic [63:0] av,
                                          input logic [63:0] bv,
                                          input logic [63:0] cv);
    logic [31:0] a32, b32, r32;
    a32 = av[31:0];
    b32 = bv[31:0];
    if (w64) begin
      unique case (o)
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
      unique case (o)
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

  function automatic logic [63:0] ref_apply(input flit_t f);
    logic [63:0] old, newv;
    int nb;
    nb = 1 << f.size;
    old = '0;
    for (int b = 0; b < 8; b++)
      if (b < nb) old[8*b +: 8] = refbyte(f.addr + 48'(b));
    if (f.kind == KIND_WR) begin
      for (int b = 0; b < 8; b++)
        if (f.be[b]) refmem[f.addr + 48'(b)] = f.data[8*b +: 8];
      return 64'd0;
    end else if (f.kind == KIND_AMO) begin
      newv = ref_alu(f.op, f.size == SZ_8, old, f.data, f.cmp);
      for (int b = 0; b < 8; b++)
        if (b < nb) refmem[f.addr + 48'(b)] = newv[8*b +: 8];
      return old;
    end else begin
      return old;
    end
  endfunction

  function automatic flit_t mk_req(input logic [47:0] pa, input int ag,
                                   input logic [1:0] kindx, input logic [4:0] op,
                                   input logic [1:0] sz, input logic rtn,
                                   input logic [63:0] d, input logic [63:0] cmp,
                                   input logic [7:0] be, input logic [7:0] tag);
    ch_addr_s ca;
    int sl;
    ca = l2_ch_of(pa);
    sl = l2_slice_idx(pa);
    return '{cls: CLS_REQ, kind: kindx, op: op, size: sz, rtn: rtn, addr: pa,
             src: 6'(AGENT_NODE[ag]), dst: 6'(SLICE_NODE[sl]), tag: tag,
             data: d, cmp: cmp, be: be, default: '0};
  endfunction

  // golden vectors
  localparam int GMAX = 2048;
  logic [47:0] g_off   [GMAX];
  logic [47:0] g_choff [GMAX];
  logic [7:0]  g_slice [GMAX];
  logic [31:0] g_cnt_arr [1];
  logic [31:0] g_cnt;
  string        gdir;

  // deadlock watchdog
  int      idle_cycles;
  longint  last_delivered;
  initial begin
    idle_cycles = 0;
    last_delivered = 0;
    forever begin
      @(posedge clk);
      if (active) begin
        if (delivered == last_delivered) idle_cycles++;
        else begin
          idle_cycles    = 0;
          last_delivered = delivered;
        end
        if (idle_cycles > 20000) begin
          $display("NG_NOC_L2MAP_TB_FAIL: deadlock watchdog (delivered=%0d)",
                   delivered);
          $finish;
        end
      end
    end
  end

  // ---- main -----------------------------------------------------------------
  initial begin
    errors = 0; checks = 0; delivered = 0; active = 0;
    if (!$value$plusargs("GOLDEN_DIR=%s", gdir)) gdir = "golden";
    $readmemh({gdir, "/l2_map_vram_off.hex"}, g_off);
    $readmemh({gdir, "/l2_map_choff.hex"}, g_choff);
    $readmemh({gdir, "/l2_map_slice.hex"}, g_slice);
    $readmemh({gdir, "/l2_map_count.hex"}, g_cnt_arr);
    g_cnt = g_cnt_arr[0];

    rst_n = 1'b0;
    for (int s = 0; s < 4; s++) ar_cnt[s] = 0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);
    active = 1;

    // ============ phase 1: golden mapping, one outstanding ================
    $display("TB: golden vectors = %0d", g_cnt);
    for (int i = 0; i < int'(g_cnt); i++) begin
      int sl, cnt0;
      logic [63:0] want;
      flit_t f;
      if (32'(g_off[i][6:0]) + 8 > 128) continue;  // cross-line: requestor split
      sl = l2_slice_idx(g_off[i]);
      checks++;
      if (sl != int'(g_slice[i])) begin
        errors++;
        $display("TB_FAIL l2map: slice fn off=%h got %0d want %0d",
                 g_off[i], sl, g_slice[i]);
      end
      cnt0 = ar_cnt[sl];
      f = mk_req(g_off[i], 0, KIND_RD, AMO_NONE, SZ_8, 1'b0, '0, '0, 8'h00,
                 8'(i % 256));
      want = ref_apply(f);
      expd[0][i % 256] = want;
      mode[0][i % 256] = 1;
      sendq[0].push_back(f);
      // drain: queue empty AND response seen (outst counts injection->resp);
      // the sender pops on a later negedge, so also wait a cycle for it to
      // register the request before sampling outst.
      @(negedge clk);
      wait (sendq[0].size() == 0 && outst[0] == 0);
      checks++;
      if (last_src[0] != 6'(SLICE_NODE[sl])) begin
        errors++;
        $display("TB_FAIL l2map: off=%h served by node %0d want %0d",
                 g_off[i], last_src[0], SLICE_NODE[sl]);
      end
      if (ar_cnt[sl] > cnt0) begin
        if (last_ar[sl] != (g_choff[i] & ~48'h7F)) begin
          errors++;
          $display("TB_FAIL l2map: off=%h choff=%h ddr araddr=%h",
                   g_off[i], g_choff[i], last_ar[sl]);
        end
      end
    end
    $display("TB: phase 1 golden mapping done (%0d checks)", checks);

    // ============ phase 2: two-agent pipelined soak ========================
    for (int sd = 0; sd < 2; sd++) begin
      // agent 0 window: granules 1..2 (C2/C1odd); agent 1: granules 48..49
      for (int n = 0; n < 160; n++) begin
        for (int ag = 0; ag < 2; ag++) begin
          logic [47:0] pa;
          logic [1:0] sz;
          logic [63:0] d;
          logic [4:0] o;
          logic [7:0] be;
          flit_t f;
          pa = (ag == 0 ? 48'h0001_0000 : 48'h0003_0000) + 48'($urandom % 8192);
          if (pa[6:0] > 48'd120) pa = pa & ~48'h7F;
          d  = 64'($urandom);
          case ($urandom_range(0, 9))
            0, 1, 2, 3: begin
              f = mk_req(pa, ag, KIND_RD, AMO_NONE, SZ_8, 1'b0, '0, '0, 8'h00,
                         8'(n));
              expd[ag][n] = ref_apply(f);
              mode[ag][n] = 1;
            end
            4, 5, 6: begin
              be = ($urandom_range(0, 3) == 0) ? 8'h0F : 8'hFF;
              f = mk_req(pa, ag, KIND_WR, AMO_NONE, SZ_8, 1'b0, d, '0, be,
                         8'(n));
              junkv = ref_apply(f);
              mode[ag][n] = 0;
            end
            default: begin
              o  = 5'($urandom_range(0, 12));
              sz = ((o == AMO_INC) || (o == AMO_DEC)) ? SZ_4 :
                   (($urandom_range(0, 1) == 0) ? SZ_4 : SZ_8);
              f = mk_req(pa, ag, KIND_AMO, o, sz, 1'b1, d, 64'($urandom),
                         8'h00, 8'(n));
              expd[ag][n] = ref_apply(f);
              mode[ag][n] = 1;
            end
          endcase
          sendq[ag].push_back(f);
        end
      end
      @(negedge clk);
      wait (outst[0] == 0 && outst[1] == 0 && sendq[0].size() == 0 &&
            sendq[1].size() == 0);
      $display("TB: phase 2 seed %0d done (%0d checks)", sd, checks);
    end

    // ============ phase 3: cross-agent atomicity ===========================
    begin
      logic [47:0] spa;
      flit_t f;
      longint want_sum;
      spa = 48'h0006_0000;  // slot 2 -> C1 odd -> slice 1
      // initialise the shared counter
      f = mk_req(spa, 0, KIND_WR, AMO_NONE, SZ_4, 1'b0, 64'd1000, '0, 8'h0F,
                 8'hF0);
      junkv = ref_apply(f);
      mode[0][8'hF0] = 0;
      sendq[0].push_back(f);
      @(negedge clk);
      wait (sendq[0].size() == 0 && outst[0] == 0);
      // 128 AMO ADD rtn from each agent (pipelined)
      for (int n = 0; n < 128; n++) begin
        for (int ag = 0; ag < 2; ag++) begin
          f = mk_req(spa, ag, KIND_AMO, AMO_ADD, SZ_4, 1'b1, 64'd1, '0, 8'h00,
                     8'(n));
          mode[ag][n] = 2;
          sendq[ag].push_back(f);
        end
      end
      @(negedge clk);
      wait (outst[0] == 0 && outst[1] == 0 && sendq[0].size() == 0 &&
            sendq[1].size() == 0);
      // final value
      f = mk_req(spa, 0, KIND_RD, AMO_NONE, SZ_4, 1'b0, '0, '0, 8'h00, 8'hF1);
      expd[0][8'hF1] = 64'd1256;
      mode[0][8'hF1] = 1;
      sendq[0].push_back(f);
      @(negedge clk);
      wait (sendq[0].size() == 0 && outst[0] == 0);
      want_sum = 0;
      for (int v = 0; v < 256; v++) want_sum += v;
      checks += 2;
      if ((amo_sum[0] + amo_sum[1]) != (1000 * 256 + want_sum)) begin
        errors++;
        $display("TB_FAIL l2map: rtn sum=%0d want=%0d",
                 amo_sum[0] + amo_sum[1], 1000 * 256 + want_sum);
      end
    end
    active = 0;

    if (errors == 0)
      $display("NG_NOC_L2MAP_TB_PASS: %0d checks, 0 errors", checks);
    else
      $display("NG_NOC_L2MAP_TB_FAIL: %0d errors of %0d checks", errors, checks);
    $finish;
  end

  initial begin
    #100_000_000;
    $display("NG_NOC_L2MAP_TB_FAIL: absolute watchdog timeout");
    $finish;
  end
endmodule
