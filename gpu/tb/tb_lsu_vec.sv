// tb_lsu_vec.sv -- LSU + L1D + shared memory vs functional-model vectors.
//
// Consumes the flat text made by sim/lsu/vec2txt_lsu.py from the `arch_seq`
// vectors of sim/lsu/dump_lsu.cpp (model-driven: lsu_semantics.cpp through the
// sem_harness).  Per vector: load the snapshot into ng_sp_regfile, load the
// initial memory blocks into the behavioural fabric model behind the L1 (a
// byte-addressed associative array that also executes the AMOs with the real
// L2 AMO ALU), zero the shared window, then issue the instruction sequence one
// by one to ng_lsu_top and wait for done / trap.  At the end ALL vregs / uregs
// are compared with "snapshot + expected diff", all memory bytes ever touched
// (initial blocks, anything the DUT wrote) are compared with the expected image,
// and traps (index, cause, info) must match.  waitcnt accounting
// (cnt_inc / cnt_dec id) is checked per instruction.
//
// Stimulus is driven and outputs are sampled on the falling clock edge (the
// race lesson of under Verilator 5).  Runs under xsim and Verilator 5.
// Plusargs:  +VEC=<file> (default lsu.txt)  +MAXFAIL=<n>  +LAT (print latency)
// Config (tb_lsu_cfg.svh from run_tb_lsu.sh): NG_TB_NL, NG_TB_L1SETS,
// NG_TB_L1WAYS, NG_TB_MAXLAT (random fabric/MMU latency bound), NG_TB_SHBASE; NG_TB_LSUK (LSU contexts,
// default 1 = ng_lsu), NG_TB_MEMOUT (uc channel slots), NG_TB_HOSTMAP (1: the MMU stub maps every VRAM VA into the
// host aperture at +0xC000_0000_0000 -- PA[47] set, so the requests take the uc path, i.e. ng_lsu_kn's slot channel;
// the fabric models undo the offset, so the memory image is the model's).  The uc channel answers through a tagged
// multi-outstanding responder (random latency, two targets out of order, same-line conflicts, slots held to u_free).
`timescale 1ns / 1ps

`include "tb_lsu_cfg.svh"

module tb_lsu_vec;
  import ng_lsu_pkg::*;
  parameter int NL      = `NG_TB_NL;
  parameter int L1SETS  = `NG_TB_L1SETS;
  parameter int L1WAYS  = `NG_TB_L1WAYS;
  parameter int MAXLAT  = `NG_TB_MAXLAT;
  parameter int SHRAND  = `NG_TB_SHRAND;     // 1: random shared window base per vector

`ifndef NG_TB_SHLPB
 `define NG_TB_SHLPB 8
`endif
`ifndef NG_TB_L1LPB
 `define NG_TB_L1LPB 4
`endif
`ifndef NG_TB_LSUK
 `define NG_TB_LSUK 1
`endif
`ifndef NG_TB_MEMOUT
 `define NG_TB_MEMOUT 8
`endif
`ifndef NG_TB_HOSTMAP
 `define NG_TB_HOSTMAP 0
`endif
  localparam int LSUK    = `NG_TB_LSUK;
  localparam int MEMOUT  = `NG_TB_MEMOUT;
  localparam bit HOSTMAP = `NG_TB_HOSTMAP;
  localparam int USW     = ng_noc_pkg::clog2_1(MEMOUT);
  localparam longint unsigned HOFF = 64'h0000_c000_0000_0000;
  localparam int SHL = `NG_TB_SHLPB;       // shared-memory lanes per round
  localparam int L1L = `NG_TB_L1LPB;       // global-path lanes per beat

  localparam int NBEAT = 32 / NL;
  localparam int BTW   = (NL < 32) ? $clog2(32 / NL) : 1;
  localparam int SW    = 4;
  localparam int DW    = NL * 32;
  localparam int SLOT  = 3;
  localparam int SH_AW = 16;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // ------------------------------------------------------------ DUT signals
  logic              iss_valid = 1'b0, iss_ready;
  logic [31:0]       iss_w0 = '0, iss_w1 = '0, iss_exec = '0;
  logic [SW-1:0]     iss_slot = SW'(SLOT);
  logic [7:0]        iss_nregs = 8'd128;
  logic [15:0]       iss_tag = '0;
  ctx_t              iss_ctx = '0;
  logic              done_valid, trap_valid, busy;
  logic [15:0]       done_tag;
  logic [3:0]        trap_cause;
  logic [31:0]       trap_info;
  logic              cnt_inc_valid, cnt_dec_valid;
  logic [SW-1:0]     cnt_inc_slot, cnt_dec_slot;
  logic [2:0]        cnt_inc_id, cnt_dec_id;

  logic              l_rd_valid, l_wr_valid, l_ur_valid, l_uw_valid;
  logic [SW-1:0]     l_rd_slot, l_wr_slot, l_ur_slot, l_uw_slot;
  logic [6:0]        l_rd_reg, l_wr_reg;
  logic [BTW-1:0]    l_rd_beat, l_wr_beat;
  logic [1:0]        l_rd_hsel, l_wr_hsel;
  logic [NL-1:0]     l_wr_lanes;
  logic [DW-1:0]     l_wr_data;
  logic [5:0]        l_ur_reg, l_uw_reg;
  logic [31:0]       l_uw_data;

  logic              mmu_req_valid, mmu_resp_ready;
  logic              mmu_req_ready = 1'b0, mmu_resp_valid = 1'b0;
  logic [3:0]        mmu_req_vmid;
  logic [63:0]       mmu_req_va;
  logic [1:0]        mmu_req_acc;
  logic              mmu_req_atomic;
  logic [7:0]        mmu_req_tag;
  logic [2:0]        mmu_resp_fault = '0;
  logic              mmu_resp_sparse = 1'b0;
  logic [47:0]       mmu_resp_pa = '0;
  logic [63:0]       mmu_resp_pte = '0;
  logic              ext_flt_valid;
  logic [3:0]        ext_flt_vmid;
  logic [63:0]       ext_flt_va;
  logic [1:0]        ext_flt_acc;

  logic              mem_req_valid, mem_rsp_valid = 1'b0;
  logic              mem_req_ready = 1'b0;
  mreq_t             mem_req;
  mrsp_t             mem_rsp = '0;
  logic [31:0]       st_l1_rd, st_l1_wr, st_l1_amo, st_sh_rounds, st_hit, st_miss, st_fill, st_inv;
  logic              u_req_valid, u_req_ready, u_rsp_valid, u_rd_req, u_rd_gnt, u_free;
  mreq_t             u_req;
  logic [USW-1:0]    u_req_slot, u_rsp_slot, u_rd_slot;
  logic [1:0]        u_rsp_err;
  logic [1023:0]     u_rd_data;

  // ------------------------------------------------------------ register file + port mux
  logic              dbg = 1'b1;
  logic              t_rd_valid = 1'b0, t_wr_valid = 1'b0, t_ur_valid = 1'b0, t_uw_valid = 1'b0;
  logic [6:0]        t_rd_reg = '0, t_wr_reg = '0;
  logic [BTW-1:0]    t_rd_beat = '0, t_wr_beat = '0;
  logic [DW-1:0]     t_wr_data = '0;
  logic [5:0]        t_ur_reg = '0, t_uw_reg = '0;
  logic [31:0]       t_uw_data = '0;

  logic [2:0]        rf_rd_valid, rf_rd_ready, rf_rdv_valid, rf_rdv_err;
  logic [SW-1:0]     rf_rd_slot [3];
  logic [6:0]        rf_rd_reg  [3];
  logic [BTW-1:0]    rf_rd_beat [3];
  logic [1:0]        rf_rd_hsel [3];
  logic [DW-1:0]     rf_rdv_data [3];
  logic              rf_perf;
  logic [1:0]        rf_wr_valid, rf_wr_ready, rf_wr_err;
  logic [SW-1:0]     rf_wr_slot [2];
  logic [6:0]        rf_wr_reg  [2];
  logic [BTW-1:0]    rf_wr_beat [2];
  logic [NL-1:0]     rf_wr_lanes [2];
  logic [1:0]        rf_wr_hsel [2];
  logic [DW-1:0]     rf_wr_data [2];
  logic [2:0]        rf_ur_valid, rf_urd_valid;
  logic [SW-1:0]     rf_ur_slot [3];
  logic [5:0]        rf_ur_reg  [3];
  logic [31:0]       rf_urd_data [3];
  logic [1:0]        rf_uw_valid;
  logic [SW-1:0]     rf_uw_slot [2];
  logic [5:0]        rf_uw_reg  [2];
  logic [31:0]       rf_uw_data [2];
  logic              alloc_valid = 1'b0;
  logic [SW-1:0]     alloc_slot = '0;
  logic [9:0]        alloc_base = '0;
  logic [7:0]        alloc_nregs = '0;
  logic              alloc_err;

  ng_sp_regfile #(.NG_RF_NL(NL)) u_rf (
    .clk(clk), .rst_n(rst_n),
    .alloc_valid(alloc_valid), .alloc_slot(alloc_slot), .alloc_base(alloc_base),
    .alloc_nregs(alloc_nregs), .alloc_err(alloc_err),
    .rd_valid(rf_rd_valid), .rd_ready(rf_rd_ready), .rd_slot(rf_rd_slot), .rd_reg(rf_rd_reg),
    .rd_beat(rf_rd_beat), .rd_hsel(rf_rd_hsel), .rdv_valid(rf_rdv_valid), .rdv_err(rf_rdv_err),
    .rdv_data(rf_rdv_data), .perf_rd_conflict(rf_perf),
    .wr_valid(rf_wr_valid), .wr_ready(rf_wr_ready), .wr_slot(rf_wr_slot), .wr_reg(rf_wr_reg),
    .wr_beat(rf_wr_beat), .wr_lanes(rf_wr_lanes), .wr_hsel(rf_wr_hsel), .wr_data(rf_wr_data),
    .wr_err(rf_wr_err),
    .ur_valid(rf_ur_valid), .ur_slot(rf_ur_slot), .ur_reg(rf_ur_reg),
    .urd_valid(rf_urd_valid), .urd_data(rf_urd_data),
    .uw_valid(rf_uw_valid), .uw_slot(rf_uw_slot), .uw_reg(rf_uw_reg), .uw_data(rf_uw_data)
  );

  always_comb begin
    for (int k = 0; k < 3; k++) begin
      rf_rd_valid[k] = 1'b0; rf_rd_slot[k] = SW'(SLOT); rf_rd_reg[k] = '0; rf_rd_beat[k] = '0; rf_rd_hsel[k] = 2'd0;
    end
    if (dbg) begin
      rf_rd_valid[0] = t_rd_valid; rf_rd_reg[0] = t_rd_reg; rf_rd_beat[0] = t_rd_beat;
    end else begin
      rf_rd_valid[0] = l_rd_valid; rf_rd_slot[0] = l_rd_slot; rf_rd_reg[0] = l_rd_reg;
      rf_rd_beat[0] = l_rd_beat; rf_rd_hsel[0] = l_rd_hsel;
    end
    rf_wr_valid = 2'b00;
    for (int k = 0; k < 2; k++) begin
      rf_wr_slot[k] = SW'(SLOT); rf_wr_reg[k] = '0; rf_wr_beat[k] = '0; rf_wr_lanes[k] = '0;
      rf_wr_hsel[k] = 2'b11; rf_wr_data[k] = '0;
    end
    if (dbg) begin
      rf_wr_valid[0] = t_wr_valid; rf_wr_reg[0] = t_wr_reg; rf_wr_beat[0] = t_wr_beat; rf_wr_lanes[0] = '1;
      rf_wr_data[0] = t_wr_data;
    end else begin
      rf_wr_valid[1] = l_wr_valid; rf_wr_slot[1] = l_wr_slot; rf_wr_reg[1] = l_wr_reg; rf_wr_beat[1] = l_wr_beat;
      rf_wr_lanes[1] = l_wr_lanes; rf_wr_hsel[1] = l_wr_hsel; rf_wr_data[1] = l_wr_data;   // async-writeback port
    end
    rf_ur_valid = 3'b000;
    for (int k = 0; k < 3; k++) begin rf_ur_slot[k] = SW'(SLOT); rf_ur_reg[k] = '0; end
    if (dbg) begin
      rf_ur_valid[0] = t_ur_valid; rf_ur_reg[0] = t_ur_reg;
    end else begin
      rf_ur_valid[1] = l_ur_valid; rf_ur_slot[1] = l_ur_slot; rf_ur_reg[1] = l_ur_reg;
    end
    rf_uw_valid = 2'b00;
    for (int k = 0; k < 2; k++) begin rf_uw_slot[k] = SW'(SLOT); rf_uw_reg[k] = '0; rf_uw_data[k] = '0; end
    if (dbg) begin
      rf_uw_valid[0] = t_uw_valid; rf_uw_reg[0] = t_uw_reg; rf_uw_data[0] = t_uw_data;
    end else begin
      rf_uw_valid[1] = l_uw_valid; rf_uw_slot[1] = l_uw_slot; rf_uw_reg[1] = l_uw_reg; rf_uw_data[1] = l_uw_data;
    end
  end

  // ----------------------------------------------------------
  ng_lsu_top #(.NL(NL), .SLOTW(SW), .SH_AW(SH_AW), .SH_LPB(SHL), .L1_LPB(L1L), .L1_SETS(L1SETS), .L1_WAYS(L1WAYS),
               .LSU_K(LSUK), .MEM_OUT(MEMOUT)) dut (
    .clk(clk), .rst_n(rst_n), .inv_l1_ext(1'b0), .sh_clr_en(1'b0), .sh_clr_row('0),
    .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1), .iss_exec(iss_exec),
    .iss_slot(iss_slot), .iss_nregs(iss_nregs), .iss_tag(iss_tag), .iss_ctx(iss_ctx),
    .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid), .trap_cause(trap_cause),
    .trap_info(trap_info), .busy(busy),
    .cnt_inc_valid(cnt_inc_valid), .cnt_inc_slot(cnt_inc_slot), .cnt_inc_id(cnt_inc_id),
    .cnt_dec_valid(cnt_dec_valid), .cnt_dec_slot(cnt_dec_slot), .cnt_dec_id(cnt_dec_id),
    .rd_valid(l_rd_valid), .rd_ready(rf_rd_ready[1'b0]), .rd_slot(l_rd_slot), .rd_reg(l_rd_reg),
    .rd_beat(l_rd_beat), .rd_hsel(l_rd_hsel), .rdv_valid(rf_rdv_valid[0]), .rdv_err(rf_rdv_err[0]),
    .rdv_data(rf_rdv_data[0]),
    .wr_valid(l_wr_valid), .wr_ready(rf_wr_ready[1]), .wr_slot(l_wr_slot), .wr_reg(l_wr_reg), .wr_beat(l_wr_beat),
    .wr_lanes(l_wr_lanes), .wr_hsel(l_wr_hsel), .wr_data(l_wr_data),
    .ur_valid(l_ur_valid), .ur_slot(l_ur_slot), .ur_reg(l_ur_reg), .urd_valid(rf_urd_valid[1]),
    .urd_data(rf_urd_data[1]),
    .uw_valid(l_uw_valid), .uw_slot(l_uw_slot), .uw_reg(l_uw_reg), .uw_data(l_uw_data),
    .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
    .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic),
    .mmu_req_tag(mmu_req_tag), .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready),
    .mmu_resp_fault(mmu_resp_fault), .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa),
    .mmu_resp_pte(mmu_resp_pte),
    .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
    .mem_req_valid(mem_req_valid), .mem_req_ready(mem_req_ready), .mem_req(mem_req),
    .mem_rsp_valid(mem_rsp_valid), .mem_rsp(mem_rsp),
    .u_req_valid(u_req_valid), .u_req_ready(u_req_ready), .u_req(u_req), .u_req_slot(u_req_slot),
    .u_rsp_valid(u_rsp_valid), .u_rsp_slot(u_rsp_slot), .u_rsp_err(u_rsp_err),
    .u_rd_req(u_rd_req), .u_rd_slot(u_rd_slot), .u_rd_gnt(u_rd_gnt), .u_rd_data(u_rd_data), .u_free(u_free),
    .stat_l1_rd(st_l1_rd), .stat_l1_wr(st_l1_wr), .stat_l1_amo(st_l1_amo), .stat_sh_rounds(st_sh_rounds),
    .stat_hit(st_hit), .stat_miss(st_miss), .stat_fill(st_fill), .stat_inv(st_inv)
  );

  // ------------------------------------------------------------ random source
  logic [31:0] lfsr = 32'h2468ace1;
  function automatic int rnd(input int n);   // 0 .. n-1, advances lfsr (xorshift32)
    lfsr = lfsr ^ (lfsr << 13);
    lfsr = lfsr ^ (lfsr >> 17);
    lfsr = lfsr ^ (lfsr << 5);
    return (n <= 1) ? 0 : int'(lfsr % n);
  endfunction

  // ------------------------------------------------------------ MMU stub (VMID 0 bypass: PA = VA[47:0]; VA[63:48] != 0 -> NON_CANONICAL (fault 2), as ng_mmu / the model)
  typedef enum int {M_IDLE, M_ACC, M_WAITL, M_RESP} mst_t;
  mst_t  mst = M_IDLE;
  int    m_lat;
  logic [63:0] m_va;
  int    mmu_reqs = 0;
  always @(negedge clk) begin
    mmu_req_ready <= 1'b0;
    mmu_resp_valid <= 1'b0;
    case (mst)
      M_IDLE: if (mmu_req_valid) begin
        m_va = mmu_req_va;
        if ($test$plusargs("DBG")) $display("DBG MMU va=%h acc=%0d at=%b st=%0d", mmu_req_va, mmu_req_acc, mmu_req_atomic, dut.g_core.u_lsu.u_core.st);
        mmu_req_ready <= 1'b1;
        mst <= M_ACC;
      end
      M_ACC: begin
        mmu_reqs++;
        m_lat = rnd(MAXLAT + 1);
        mst <= (m_lat == 0) ? M_RESP : M_WAITL;
        if (m_lat == 0) begin
          mmu_resp_pa <= (|m_va[63:48]) ? 48'd0 : hmap(m_va[47:0]); mmu_resp_fault <= (|m_va[63:48]) ? 3'd2 : 3'd0; mmu_resp_sparse <= 1'b0; mmu_resp_pte <= 64'd0;
          mmu_resp_valid <= 1'b1;
          mst <= M_IDLE;
        end
      end
      M_WAITL: begin
        m_lat = m_lat - 1;
        if (m_lat <= 0) begin
          mmu_resp_pa <= (|m_va[63:48]) ? 48'd0 : hmap(m_va[47:0]); mmu_resp_fault <= (|m_va[63:48]) ? 3'd2 : 3'd0; mmu_resp_sparse <= 1'b0; mmu_resp_pte <= 64'd0;
          mmu_resp_valid <= 1'b1;
          mst <= M_IDLE;
        end
      end
      default: mst <= M_IDLE;
    endcase
  end

  // ------------------------------------------------------------ fabric model behind the L1
  logic [7:0] mem [longint unsigned];
  logic [7:0] emem [longint unsigned];
  int  n_mem_rd = 0, n_mem_wr = 0, n_mem_amo = 0, n_mem_inv2 = 0, n_mem_wb2 = 0, n_mem_uc = 0;

  function automatic logic [7:0] mrd(input longint unsigned a);
    return mem.exists(a) ? mem[a] : 8'd0;
  endfunction
  // HOSTMAP: VRAM VA -> host PA (+HOFF) in the MMU stub, undone by the fabric models (ma)
  function automatic logic [47:0] hmap(input logic [47:0] a);
    return (HOSTMAP && a < 48'h0004_0000_0000) ? a + 48'(HOFF) : a;
  endfunction
  function automatic longint unsigned ma(input logic [47:0] pa);
    return (HOSTMAP && pa[47:46] == 2'b11) ? longint'(pa) - HOFF : longint'(pa);
  endfunction

  logic [4:0]  amo_op_s;
  logic        amo64_s;
  logic [63:0] amo_a_s, amo_b_s, amo_cmp_s, amo_r_s;
  logic        amo_hit_s;
  ng_l2_amo_alu u_alu (.op(amo_op_s), .is64(amo64_s), .a(amo_a_s), .b(amo_b_s), .cmp(amo_cmp_s), .r(amo_r_s),
                       .cas_hit(amo_hit_s));

  // ---------------------------------------------------------------- uc channel responder (slot protocol)
  // Behavioural ng_rsp_fab_ts  for the LSU's uc channel: MEMOUT slots (lowest free),
  // same line with either side a write waits (answered read slots count), an M_WR is accepted after 4 clocks of
  // valid, random 1/8 ready stalls, latency u_lo..u_hi (+400 in the SLOW region), two "targets" (PA[12]) in order
  // each, out of order across, one answer per clock, ERR region -> err = 1 (no access), M_RD slots held until
  // u_free (data in us_line), u_rd_gnt refused 1/4 of the clocks (an I$ / L1D answer using the read port in the SP),
  // !rst_n = sp_rst: answered reads freed, requests in flight orphaned (answer not delivered).
  localparam int UNS = 1 << USW;
  bit [UNS-1:0]  us_al = '0, us_ans = '0, us_orph = '0;
  mreq_t         us_rq   [UNS];
  longint        us_due  [UNS];
  logic [1:0]    us_er   [UNS];
  logic [1023:0] us_line [UNS];
  longint        ucyc = 0;
  longint        utl [2] = '{0, 0};
  int            u_lo = 0, u_hi = 2 * MAXLAT + 2;
  bit            u_stall_q = 1'b0, u_gnt_q = 1'b1;
  int            u_wstg = 0;
  longint unsigned U_ERR_LO  = 64'h0000_8000_00e0_0000, U_ERR_HI  = 64'h0000_8000_00e0_1000;
  longint unsigned U_SLOW_LO = 64'h0000_8000_00f0_0000, U_SLOW_HI = 64'h0000_8000_00f1_0000;
  int            un_rd = 0, un_wr = 0, un_err = 0, un_orph = 0, un_max = 0, un_bad = 0;
  logic [USW-1:0] u_fsl;
  logic          u_any, u_conf, u_can;
  initial for (int i = 0; i < UNS; i++) begin us_rq[i] = '0; us_due[i] = 0; us_er[i] = '0; us_line[i] = '0; end
  always_comb begin
    u_any = 1'b0; u_fsl = '0; u_conf = 1'b0;
    for (int i = UNS - 1; i >= 0; i--) if (!us_al[i] && i < MEMOUT) begin u_any = 1'b1; u_fsl = USW'(i); end
    for (int i = 0; i < UNS; i++)
      if (us_al[i] && (us_rq[i].pa[47:7] == u_req.pa[47:7]) && (us_rq[i].kind == M_WR || u_req.kind == M_WR)) u_conf = 1'b1;
    u_can       = u_req_valid && rst_n && u_any && !u_conf && !u_stall_q;
    u_req_ready = u_can && (u_req.kind != M_WR || u_wstg >= 3);
    u_req_slot  = u_fsl;
    u_rd_gnt    = u_gnt_q;
    u_rd_data   = us_line[u_rd_slot];
  end
  always @(posedge clk) begin
    int bi;
    longint bd;
    ucyc <= ucyc + 1;
    u_rsp_valid <= 1'b0;
    u_stall_q <= ($urandom_range(0, 7) == 0);
    u_gnt_q   <= ($urandom_range(0, 3) != 0);
    if (u_can && u_req.kind == M_WR) u_wstg <= u_req_ready ? 0 : u_wstg + 1; else u_wstg <= 0;
    if (u_req_valid && u_req_ready) begin
      int lat, t;
      longint unsigned a;
      a = longint'(u_req.pa);
      lat = (u_hi > u_lo) ? $urandom_range(u_lo, u_hi) : u_lo;
      if (a >= U_SLOW_LO && a < U_SLOW_HI) lat = lat + 400;
      t = u_req.pa[12] ? 1 : 0;
      if (ucyc + longint'(lat) <= utl[t]) utl[t] = utl[t] + 1; else utl[t] = ucyc + longint'(lat);
      us_due[u_fsl] <= utl[t];
      us_al[u_fsl] <= 1'b1; us_ans[u_fsl] <= 1'b0; us_orph[u_fsl] <= 1'b0; us_rq[u_fsl] <= u_req;
      us_er[u_fsl] <= (a >= U_ERR_LO && a < U_ERR_HI) ? 2'd1 : 2'd0;
    end
    bi = -1; bd = 0;
    for (int i = 0; i < UNS; i++)
      if (us_al[i] && !us_ans[i] && us_due[i] <= ucyc && (bi < 0 || us_due[i] < bd)) begin bi = i; bd = us_due[i]; end
    if (bi >= 0) begin
      logic [1023:0] line;
      line = '0;
      if (us_er[bi] == 2'd0) begin
        if (us_rq[bi].kind == M_RD) begin
          un_rd++;
          for (int b = 0; b < 128; b++) line[8 * b +: 8] = mrd(ma({us_rq[bi].pa[47:7], 7'd0}) + b);
        end else if (us_rq[bi].kind == M_WR) begin
          un_wr++;
          for (int b = 0; b < 128; b++)
            if (us_rq[bi].be[b]) mem[ma({us_rq[bi].pa[47:7], 7'd0}) + b] = us_rq[bi].wdata[8 * b +: 8];
        end
      end else un_err++;
      if (us_orph[bi] || !rst_n) begin
        us_al[bi] <= 1'b0; un_orph++;
      end else begin
        u_rsp_valid <= 1'b1; u_rsp_slot <= USW'(bi); u_rsp_err <= us_er[bi];
        if (us_rq[bi].kind == M_RD) begin us_ans[bi] <= 1'b1; us_line[bi] <= line; end
        else us_al[bi] <= 1'b0;
      end
    end
    if (u_free) begin
      if (!u_rd_gnt) begin un_bad++; $display("FAIL: uc responder: u_free without u_rd_gnt"); end
      if (us_al[u_rd_slot] && us_ans[u_rd_slot]) begin us_al[u_rd_slot] <= 1'b0; us_ans[u_rd_slot] <= 1'b0; end
      else begin un_bad++; $display("FAIL: uc responder: u_free of slot %0d that holds no answered read", u_rd_slot); end
    end
    if (!rst_n)
      for (int i = 0; i < UNS; i++) begin
        if (us_al[i] && us_ans[i]) begin us_al[i] <= 1'b0; us_ans[i] <= 1'b0; end
        else if (us_al[i] && i != bi) us_orph[i] <= 1'b1;
      end
    if ($countones(us_al) > un_max) un_max <= $countones(us_al);
  end


  typedef enum int {F_IDLE, F_ACC, F_LAT} fst_t;
  fst_t  fst = F_IDLE;
  mreq_t f_rq;
  mrsp_t f_rs;
  int    f_lat;
  always @(negedge clk) begin
    mem_req_ready <= 1'b0;
    mem_rsp_valid <= 1'b0;
    case (fst)
      F_IDLE: if (mem_req_valid) begin
        f_rq = mem_req;
        mem_req_ready <= 1'b1;
        fst <= F_ACC;
      end
      F_ACC: begin
        // handshake happened at the last rising edge: execute the request now
        f_rs = '0;
        case (f_rq.kind)
          M_RD: begin
            n_mem_rd++;
            if (f_rq.uc) n_mem_uc++;
            for (int b = 0; b < 128; b++) f_rs.rdata[8*b +: 8] = mrd(ma(f_rq.pa) + b);
          end
          M_WR: begin
            n_mem_wr++;
            if ($test$plusargs("DBG")) $display("DBG WR pa=%h be=%h", f_rq.pa, f_rq.be);
            if (f_rq.uc) n_mem_uc++;
            for (int b = 0; b < 128; b++) if (f_rq.be[b]) mem[ma(f_rq.pa) + b] = f_rq.wdata[8*b +: 8];
          end
          M_AMO: begin
            logic [63:0] o;
            int nb;
            n_mem_amo++;
            if (f_rq.uc) n_mem_uc++;
            if ($test$plusargs("DBG")) $display("DBG AMO pa=%h op=%0d w64=%b a=%h cmp=%h", f_rq.pa, f_rq.amo_op, f_rq.amo64, f_rq.amo_a, f_rq.amo_cmp);
            nb = f_rq.amo64 ? 8 : 4;
            o = '0;
            for (int b = 0; b < nb; b++) o[8*b +: 8] = mrd(ma(f_rq.pa) + b);
            amo_op_s = f_rq.amo_op; amo64_s = f_rq.amo64; amo_a_s = o; amo_b_s = f_rq.amo_a; amo_cmp_s = f_rq.amo_cmp;
            #1;
            for (int b = 0; b < nb; b++) mem[ma(f_rq.pa) + b] = amo_r_s[8*b +: 8];
            f_rs.old = o;
          end
          M_INV_L2: n_mem_inv2++;
          M_WB_L2:  n_mem_wb2++;
          default: ;
        endcase
        f_lat = rnd(MAXLAT + 1);
        if (f_lat == 0) begin
          mem_rsp <= f_rs; mem_rsp_valid <= 1'b1; fst <= F_IDLE;
        end else fst <= F_LAT;
      end
      F_LAT: begin
        f_lat = f_lat - 1;
        if (f_lat <= 0) begin
          mem_rsp <= f_rs; mem_rsp_valid <= 1'b1; fst <= F_IDLE;
        end
      end
      default: fst <= F_IDLE;
    endcase
  end

  always @(negedge clk) if ($test$plusargs("DBG2") && dut.g_core.u_lsu.u_core.st == 6) begin
    $display("DBG2 PINIT ub=%h imm=%h rs1v0=%h ua0=%h ua1=%h rs1=%0d is_at=%b sp=%0d um=%b w0=%h w1=%h",
             dut.g_core.u_lsu.u_core.ub, dut.g_core.u_lsu.u_core.f_imm, dut.g_core.u_lsu.u_core.rs1v[0], dut.g_core.u_lsu.u_core.ua[0], dut.g_core.u_lsu.u_core.ua[1], dut.g_core.u_lsu.u_core.f_rs1,
             dut.g_core.u_lsu.u_core.is_at, dut.g_core.u_lsu.u_core.dq.space, dut.g_core.u_lsu.u_core.dq.um, dut.g_core.u_lsu.u_core.w0_q, dut.g_core.u_lsu.u_core.w1_q);
  end

  always @(negedge clk) if ($test$plusargs("DBG4") && !dbg) begin
    if (rf_rd_valid[0] || rf_rdv_valid[0])
      $display("DBG4 t=%0t rd_valid=%b ready=%b reg=%0d beat=%0d slot=%0d | rdv_valid=%b err=%b d0=%h st=%0d vj=%0d vph=%b",
               $time, rf_rd_valid[0], rf_rd_ready[0], rf_rd_reg[0], rf_rd_beat[0], rf_rd_slot[0], rf_rdv_valid[0],
               rf_rdv_err[0], rf_rdv_data[0][31:0], dut.g_core.u_lsu.u_core.st, dut.g_core.u_lsu.u_core.vj, dut.g_core.u_lsu.u_core.vph);
  end

  // ------------------------------------------------------------ debug access tasks
  task automatic write_vreg(input int r, input logic [31:0] vals [32]);
    for (int b = 0; b < NBEAT; b++) begin
      logic [DW-1:0] d;
      for (int i = 0; i < NL; i++) d[i*32 +: 32] = vals[b*NL + i];
      t_wr_valid = 1'b1; t_wr_reg = 7'(r); t_wr_beat = BTW'(b); t_wr_data = d;
      @(negedge clk);
      while (!rf_wr_ready[0]) @(negedge clk);
    end
    t_wr_valid = 1'b0;
  endtask

  task automatic read_vreg(input int r, output logic [31:0] vals [32]);
    for (int b = 0; b < NBEAT; b++) begin
      t_rd_valid = 1'b1; t_rd_reg = 7'(r); t_rd_beat = BTW'(b);
      @(negedge clk);
      while (!rf_rd_ready[0]) @(negedge clk);
      for (int i = 0; i < NL; i++) vals[b*NL + i] = rf_rdv_data[0][i*32 +: 32];
      t_rd_valid = 1'b0;
    end
  endtask

  task automatic write_ureg(input int u, input logic [31:0] v);
    t_uw_valid = 1'b1; t_uw_reg = 6'(u); t_uw_data = v;
    @(negedge clk);
    t_uw_valid = 1'b0;
  endtask

  task automatic read_ureg(input int u, output logic [31:0] v);
    t_ur_valid = 1'b1; t_ur_reg = 6'(u);
    @(negedge clk);
    t_ur_valid = 1'b0;
    @(negedge clk);
    v = rf_urd_data[0];
  endtask

  // ------------------------------------------------------------ vector reader
  int fd;
  int nvec;
  logic [8*256-1:0] vecfile;
  int maxfail;

  logic [31:0] in_v [16][32];
  logic [31:0] e_v  [128][32];
  logic [31:0] in_u [64];
  logic [31:0] e_u  [64];
  logic [31:0] g_v  [32];
  logic [31:0] g_u;

  function automatic logic [31:0] rd_hex();
    logic [31:0] x;
    int rc;
    rc = $fscanf(fd, "%h", x);
    if (rc != 1) begin
      $display("TB_ERROR: vector file truncated");
      $finish;
    end
    return x;
  endfunction

  function automatic logic [7:0] rd_byte();
    logic [31:0] x;
    x = rd_hex();
    return x[7:0];
  endfunction

  int n_pass = 0, n_fail = 0, shown = 0;
  int n_insn_run = 0, n_trap_ok = 0;
  int cnt_bad = 0;
  int cov_shared_rounds = 0, cov_sh_conflict = 0;
  int insn_cov [string];

  // memory block loader: returns after filling `mem` (initial) and `emem`
  task automatic load_blocks(input bit to_mem, input bit to_emem);
    int nb;
    nb = int'(rd_hex());
    for (int i = 0; i < nb; i++) begin
      logic [31:0] ahi, alo;
      int nbytes;
      longint unsigned a;
      ahi = rd_hex(); alo = rd_hex(); nbytes = int'(rd_hex());
      a = {ahi, alo};
      for (int j = 0; j < nbytes; j++) begin
        logic [7:0] bv;
        bv = rd_byte();
        if (to_mem) mem[a + j] = bv;
        if (to_emem) emem[a + j] = bv;
      end
    end
  endtask

  initial begin
    int rc;
    if (!$value$plusargs("VEC=%s", vecfile)) vecfile = "lsu.txt";
    if (!$value$plusargs("MAXFAIL=%d", maxfail)) maxfail = 10;
    fd = $fopen(vecfile, "r");
    if (fd == 0) begin
      $display("TB_ERROR: cannot open %s", vecfile);
      $finish;
    end
    nvec = int'(rd_hex());
    repeat (5) @(negedge clk);
    rst_n = 1'b1;
    repeat (3) @(negedge clk);

    for (int vi = 0; vi < nvec; vi++) begin
      logic [8*32-1:0] id_s;
      int    nregs, ninsn;
      logic [31:0] ex, shared_b, scr_lo, scr_hi, scr_bytes, vmid;
      string iname [64];
      int    icnt [64];
      int    inw [64];
      logic [31:0] iw0 [64];
      logic [31:0] iw1 [64];
      int    irb [64];
      int    nu, nwv, nwu;
      int    tr_f, tr_idx;
      logic [31:0] tr_cause, tr_info;
      bit    bad;
      int    nbad;
      int    ran;
      bit    got_trap, got_done;
      int    waitc;
      int    sh_base;

      rc = $fscanf(fd, "%s", id_s);
      nregs = int'(rd_hex()); ex = rd_hex(); shared_b = rd_hex(); scr_lo = rd_hex(); scr_hi = rd_hex();
      scr_bytes = rd_hex(); vmid = rd_hex(); ninsn = int'(rd_hex());
      for (int k = 0; k < ninsn; k++) begin
        string nm;
        rc = $fscanf(fd, "%s", nm);
        iname[k] = nm;
        icnt[k] = int'(rd_hex());
        inw[k] = int'(rd_hex());
        iw0[k] = rd_hex();
        iw1[k] = rd_hex();
        irb[k] = int'(rd_hex());
      end
      for (int r = 0; r < 16; r++) for (int l = 0; l < 32; l++) in_v[r][l] = rd_hex();
      for (int r = 0; r < 128; r++) for (int l = 0; l < 32; l++) e_v[r][l] = (r < 16) ? in_v[r][l] : 32'd0;
      for (int u = 0; u < 64; u++) in_u[u] = 32'd0;
      nu = int'(rd_hex());
      for (int k = 0; k < nu; k++) begin
        int ui;
        ui = int'(rd_hex());
        in_u[ui] = rd_hex();
      end
      for (int u = 0; u < 64; u++) e_u[u] = in_u[u];
      mem.delete(); emem.delete();
      load_blocks(1'b1, 1'b1);
      tr_f = int'(rd_hex()); tr_cause = rd_hex(); tr_idx = int'(rd_hex()); tr_info = rd_hex();
      nwv = int'(rd_hex());
      for (int k = 0; k < nwv; k++) begin
        int rr;
        rr = int'(rd_hex());
        for (int l = 0; l < 32; l++) e_v[rr][l] = rd_hex();
      end
      nwu = int'(rd_hex());
      for (int k = 0; k < nwu; k++) begin
        int ui;
        ui = int'(rd_hex());
        e_u[ui] = rd_hex();
      end
      // expected memory image = initial + diff runs
      begin
        int nmd;
        nmd = int'(rd_hex());
        for (int i = 0; i < nmd; i++) begin
          logic [31:0] ahi, alo;
          int nbytes;
          longint unsigned a;
          ahi = rd_hex(); alo = rd_hex(); nbytes = int'(rd_hex());
          a = {ahi, alo};
          for (int j = 0; j < nbytes; j++) emem[a + j] = rd_byte();
        end
      end

      // ---- load the snapshot (dbg owns the ports)
      dbg = 1'b1;
      alloc_valid = 1'b1; alloc_slot = SW'(SLOT); alloc_base = 10'd0; alloc_nregs = 8'(nregs);
      @(negedge clk);
      alloc_valid = 1'b0;
      @(negedge clk);
      for (int r = 0; r < nregs; r++) begin
        if (r >= 16) for (int l = 0; l < 32; l++) g_v[l] = 32'd0;
        else for (int l = 0; l < 32; l++) g_v[l] = in_v[r][l];
        write_vreg(r, g_v);
      end
      for (int u = 0; u < 64; u++) write_ureg(u, in_u[u]);
      if ($test$plusargs("DBG3")) begin
        read_vreg(nregs - 1, g_v);
        $display("DBG3 %0s v%0d lane0..3 = %h %h %h %h", id_s, nregs - 1, g_v[0], g_v[1], g_v[2], g_v[3]);
      end

      // shared window: random base inside the array, zeroed
      sh_base = SHRAND ? (rnd(((1 << SH_AW) - int'(shared_b)) / 4 + 1) * 4) : 0;
      if (sh_base + int'(shared_b) > (1 << SH_AW)) sh_base = 0;
      for (int a = 0; a < int'(shared_b); a += 4) begin
        int dw;
        dw = (sh_base + a) / 4;
        dut.g_core.u_lsu.u_core.u_shmem.mem[dw % 32][dw / 32] = 32'd0;
      end

      // ---- run the sequence
      iss_ctx = '0;
      iss_ctx.vmid = vmid[3:0];
      iss_ctx.sh_base = 16'(sh_base);
      iss_ctx.sh_size = shared_b;
      iss_ctx.scr_base = {scr_hi[15:0], scr_lo};
      iss_ctx.scr_bytes = scr_bytes;
      iss_ctx.gslot = 32'd0;
      bad = 1'b0; nbad = 0; ran = 0; got_trap = 1'b0;
      @(negedge clk);
      dbg = 1'b0;
      @(negedge clk);
      // fresh "kernel launch": invalidate the L1 (the testbench replaced the memory behind it)
      begin
        waitc = 0;
        while (!iss_ready && waitc < 100) begin @(negedge clk); waitc++; end
        iss_w0 = 32'h000004ce; iss_w1 = 32'd0; iss_exec = 32'd0; iss_tag = 16'hffff; iss_valid = 1'b1;   // cache.inv.l1
        @(negedge clk);
        iss_valid = 1'b0;
        waitc = 0;
        while (!done_valid && !trap_valid && waitc < 1000) begin @(negedge clk); waitc++; end
        repeat (2) @(negedge clk);
      end
      for (int k = 0; k < ninsn && !got_trap && !bad; k++) begin
        // wait for the unit to be idle
        waitc = 0;
        while (!iss_ready && waitc < 100) begin @(negedge clk); waitc++; end
        iss_w0 = iw0[k]; iss_w1 = iw1[k]; iss_exec = ex; iss_nregs = 8'(nregs); iss_tag = 16'(k); iss_valid = 1'b1;
        #1;
        if (!cnt_inc_valid || cnt_inc_slot !== SW'(SLOT) || int'(cnt_inc_id) != icnt[k]) begin
          cnt_bad++;
          if (cnt_bad <= 5) $display("FAIL %0s insn %0d %0s: cnt_inc valid=%b slot=%0d id=%0d expected id %0d",
                                     id_s, k, iname[k], cnt_inc_valid, cnt_inc_slot, cnt_inc_id, icnt[k]);
          bad = 1'b1;
        end
        @(negedge clk);
        iss_valid = 1'b0;
        got_trap = 1'b0; got_done = 1'b0; waitc = 0;
        while (!got_trap && !got_done && waitc < 200000) begin
          if (trap_valid) got_trap = 1'b1;
          if (done_valid) got_done = 1'b1;
          if (!got_trap && !got_done) begin @(negedge clk); waitc++; end
        end
        ran++;
        n_insn_run++;
        if ($test$plusargs("LAT") && got_done) $display("LAT %0s %0d", iname[k], waitc);
        if (!got_trap && !got_done) begin
          $display("FAIL %0s insn %0d %0s: timeout (no done/trap) state=%0d", id_s, k, iname[k], dut.g_core.u_lsu.u_core.st);
          bad = 1'b1;
        end else begin
          // the counter decrement accompanies done / trap with the same id
          if (!cnt_dec_valid || cnt_dec_slot !== SW'(SLOT) || int'(cnt_dec_id) != icnt[k]) begin
            cnt_bad++;
            if (cnt_bad <= 5) $display("FAIL %0s insn %0d %0s: cnt_dec valid=%b slot=%0d id=%0d expected id %0d",
                                       id_s, k, iname[k], cnt_dec_valid, cnt_dec_slot, cnt_dec_id, icnt[k]);
            bad = 1'b1;
          end
          if (got_trap) begin
            if (tr_f == 0 || tr_idx != k || trap_cause !== tr_cause[3:0] || trap_info !== tr_info) begin
              $display("FAIL %0s insn %0d %0s: trap cause=%0d info=%08x; expected %s idx=%0d cause=%0d info=%08x",
                       id_s, k, iname[k], trap_cause, trap_info, tr_f != 0 ? "trap" : "none", tr_idx, tr_cause, tr_info);
              bad = 1'b1;
            end else n_trap_ok++;
          end
        end
        repeat (2) @(negedge clk);
      end
      if (!bad && !got_trap && tr_f != 0) begin
        $display("FAIL %0s: expected trap at insn %0d (cause %0d) but none occurred", id_s, tr_idx, tr_cause);
        bad = 1'b1;
      end
      for (int k = 0; k < ninsn; k++) if (!bad) insn_cov[iname[k]] = insn_cov.exists(iname[k]) ? insn_cov[iname[k]] + 1 : 1;
      cov_shared_rounds = int'(st_sh_rounds);

      // ---- read back everything and compare
      dbg = 1'b1;
      @(negedge clk);
      for (int r = 0; r < nregs; r++) begin
        read_vreg(r, g_v);
        for (int l = 0; l < 32; l++) begin
          if (g_v[l] !== e_v[r][l]) begin
            if (nbad < 4 && shown < maxfail)
              $display("FAIL %0s: v%0d lane %0d expected %08x got %08x", id_s, r, l, e_v[r][l], g_v[l]);
            nbad++; bad = 1'b1;
          end
        end
      end
      for (int u = 0; u < 64; u++) begin
        read_ureg(u, g_u);
        if (g_u !== e_u[u]) begin
          if (nbad < 4 && shown < maxfail)
            $display("FAIL %0s: u%0d expected %08x got %08x", id_s, u, e_u[u], g_u);
          nbad++; bad = 1'b1;
        end
      end
      // memory image
      foreach (mem[a]) begin
        logic [7:0] ev;
        ev = emem.exists(a) ? emem[a] : 8'd0;
        if (mem[a] !== ev) begin
          if (nbad < 6 && shown < maxfail) $display("FAIL %0s: mem[%0h] expected %02x got %02x", id_s, a, ev, mem[a]);
          nbad++; bad = 1'b1;
        end
      end
      foreach (emem[a]) begin
        if (!mem.exists(a) && emem[a] !== 8'd0) begin
          if (nbad < 6 && shown < maxfail) $display("FAIL %0s: mem[%0h] expected %02x got (untouched)", id_s, a, emem[a]);
          nbad++; bad = 1'b1;
        end
      end
      if (bad) begin
        n_fail++; shown++;
        if (shown <= maxfail) $display("     (id=%0s ran=%0d/%0d exec=%08x nregs=%0d shared=%0d)", id_s, ran, ninsn, ex, nregs, shared_b);
      end else n_pass++;
    end

    begin
      int ok_names;
      ok_names = insn_cov.num();
      $display("NG_LSU_VEC NL=%0d L1=%0dx%0d MAXLAT=%0d vectors pass=%0d fail=%0d of %0d; insns run=%0d traps ok=%0d; distinct insns seen=%0d",
               NL, L1SETS, L1WAYS, MAXLAT, n_pass, n_fail, nvec, n_insn_run, n_trap_ok, ok_names);
      $display("cov: l1 rd=%0d wr=%0d amo=%0d hit=%0d miss=%0d fill=%0d inv=%0d | fabric rd=%0d wr=%0d amo=%0d uc=%0d inv2=%0d wb2=%0d | sh_rounds=%0d | mmu reqs=%0d",
               st_l1_rd, st_l1_wr, st_l1_amo, st_hit, st_miss, st_fill, st_inv, n_mem_rd, n_mem_wr, n_mem_amo, n_mem_uc,
               n_mem_inv2, n_mem_wb2, st_sh_rounds, mmu_reqs);
      $display("cov (LSU_K=%0d MEMOUT=%0d HOSTMAP=%0d): uc channel rd=%0d wr=%0d err=%0d orphans=%0d max_out=%0d protocol_errors=%0d ring_max=%0d iof_max=%0d kn_err=%0d",
               LSUK, MEMOUT, HOSTMAP, un_rd, un_wr, un_err, un_orph, un_max, un_bad, dut.sim_ring_max, dut.sim_iof_max, dut.sim_err);
      if (n_fail == 0 && n_pass == nvec && un_bad == 0 && dut.sim_err == 0) $display("NG_LSU_VEC_ALL_OK");
      else $display("NG_LSU_VEC_FAILED");
    end
    $finish;
  end

  // global watchdog
  initial begin
    repeat (8) #1000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
