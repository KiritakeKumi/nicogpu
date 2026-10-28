// tb_lsu_dir.sv -- directed LSU / coalescer / shared-memory / fault-precision tests.
//
// Hand-built M / UM / SYS format instruction words (no model vectors): checks
// what the vector comparison cannot pin down by itself --
//   * coalescing: number of L1 line requests per access pattern;
//   * L1 behaviour seen from the instruction level (hit on repeat, cpol l2 reads
//     fresh data, stale data until cache.inv.l1);
//   * store ordering (last lane wins), atomic serialisation (rtn = running prefix);
//   * precise MemFault: a faulting lane in the middle of a store leaves memory
//     untouched, trap info = faulting lane's VA, BAD_PHYS reported on ext_flt_*;
//   * shared-memory bank-conflict round counts at instruction level;
//   * waitcnt accounting ids, cache.inv.l1 / inv.l2 / wb.l2 forwarding;
//   * (NG_TB_LSUK >= 2, ng_lsu_kn): the uc channel against a tagged multi-outstanding responder (slot protocol
//     of ng_rsp_fab_ts: random latency, two targets in order each / out of order across, same-line conflicts, read
//     slots held until u_free, error region, sp_rst = !rst_n orphans) -- section M9: back-to-back instructions of
//     several warp slots, done / cnt_dec in acceptance order, cross-slot RAW through the line conflict, late error
//     (trap after the older ones, no register write, younger ones complete), precise trap behind an in-flight
//     instruction, ring-full stall, synchronous class drains, uld / sliced scratch on the uc path, reset in flight.
// The MMU stub faults VA in [0xdead0000, 0xdead0000+0x10000) (NOT_PRESENT) and
// VA[63:48] != 0 (NON_CANONICAL, like ng_mmu in VMID 0 bypass / the model); it is
// the identity elsewhere.
`timescale 1ns / 1ps

module tb_lsu_dir;
  import ng_lsu_pkg::*;
  parameter int NL = 16;
`ifdef NG_TB_SHLPB
  localparam int SHL = `NG_TB_SHLPB;     // shared-memory lanes per round
`else
  localparam int SHL = 8;
`endif
`ifdef NG_TB_L1LPB
  localparam int L1L = `NG_TB_L1LPB;     // global-path lanes per beat
`else
  localparam int L1L = 4;
`endif
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = (NL < 32) ? $clog2(32 / NL) : 1;
  localparam int SW    = 4;
  localparam int DW    = NL * 32;
  localparam int SLOT  = 2;
`ifdef NG_TB_LSUK
  localparam int LSUK  = `NG_TB_LSUK;      // LSU contexts (1 = ng_lsu)
`else
  localparam int LSUK  = 1;
`endif
`ifdef NG_TB_MEMOUT
  localparam int MEMOUT = `NG_TB_MEMOUT;   // uc channel slots
`else
  localparam int MEMOUT = 8;
`endif
  localparam int USW   = ng_noc_pkg::clog2_1(MEMOUT);

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  logic              iss_valid = 1'b0, iss_ready;
  logic [SW-1:0]     iss_slot_v = SW'(SLOT);   // slot of the instruction being issued (M9: several slots)
  logic [15:0]       iss_tag_v = 16'd0;
  logic [SW-1:0]     tb_slot = SW'(SLOT);      // slot of the TB's own register-file accesses (dbg mode)
  logic [31:0]       iss_w0 = '0, iss_w1 = '0, iss_exec = '0;
  logic [7:0]        iss_nregs = 8'd128;
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
  logic              mem_req_valid, mem_rsp_valid = 1'b0, mem_req_ready = 1'b0;
  mreq_t             mem_req;
  mrsp_t             mem_rsp = '0;
  logic [31:0]       st_l1_rd, st_l1_wr, st_l1_amo, st_sh_rounds, st_hit, st_miss, st_fill, st_inv;
  // uc channel
  logic              u_req_valid, u_req_ready, u_rsp_valid, u_rd_req, u_rd_gnt, u_free;
  mreq_t             u_req;
  logic [USW-1:0]    u_req_slot, u_rsp_slot, u_rd_slot;
  logic [1:0]        u_rsp_err;
  logic [1023:0]     u_rd_data;

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
      rf_rd_valid[k] = 1'b0; rf_rd_slot[k] = tb_slot; rf_rd_reg[k] = '0; rf_rd_beat[k] = '0; rf_rd_hsel[k] = 2'd0;
    end
    if (dbg) begin rf_rd_valid[0] = t_rd_valid; rf_rd_reg[0] = t_rd_reg; rf_rd_beat[0] = t_rd_beat; end
    else begin
      rf_rd_valid[0] = l_rd_valid; rf_rd_slot[0] = l_rd_slot; rf_rd_reg[0] = l_rd_reg; rf_rd_beat[0] = l_rd_beat;
      rf_rd_hsel[0] = l_rd_hsel;
    end
    rf_wr_valid = 2'b00;
    for (int k = 0; k < 2; k++) begin
      rf_wr_slot[k] = tb_slot; rf_wr_reg[k] = '0; rf_wr_beat[k] = '0; rf_wr_lanes[k] = '0; rf_wr_hsel[k] = 2'b11;
      rf_wr_data[k] = '0;
    end
    if (dbg) begin
      rf_wr_valid[0] = t_wr_valid; rf_wr_reg[0] = t_wr_reg; rf_wr_beat[0] = t_wr_beat; rf_wr_lanes[0] = '1;
      rf_wr_data[0] = t_wr_data;
    end else begin
      rf_wr_valid[1] = l_wr_valid; rf_wr_slot[1] = l_wr_slot; rf_wr_reg[1] = l_wr_reg; rf_wr_beat[1] = l_wr_beat;
      rf_wr_lanes[1] = l_wr_lanes; rf_wr_hsel[1] = l_wr_hsel; rf_wr_data[1] = l_wr_data;
    end
    rf_ur_valid = 3'b000;
    for (int k = 0; k < 3; k++) begin rf_ur_slot[k] = tb_slot; rf_ur_reg[k] = '0; end
    if (dbg) begin rf_ur_valid[0] = t_ur_valid; rf_ur_reg[0] = t_ur_reg; end
    else begin rf_ur_valid[1] = l_ur_valid; rf_ur_slot[1] = l_ur_slot; rf_ur_reg[1] = l_ur_reg; end
    rf_uw_valid = 2'b00;
    for (int k = 0; k < 2; k++) begin rf_uw_slot[k] = tb_slot; rf_uw_reg[k] = '0; rf_uw_data[k] = '0; end
    if (dbg) begin rf_uw_valid[0] = t_uw_valid; rf_uw_reg[0] = t_uw_reg; rf_uw_data[0] = t_uw_data; end
    else begin rf_uw_valid[1] = l_uw_valid; rf_uw_slot[1] = l_uw_slot; rf_uw_reg[1] = l_uw_reg; rf_uw_data[1] = l_uw_data; end
  end

  ng_lsu_top #(.NL(NL), .SLOTW(SW), .SH_AW(16), .SH_LPB(SHL), .L1_LPB(L1L), .L1_SETS(16), .L1_WAYS(2),
               .LSU_K(LSUK), .MEM_OUT(MEMOUT)) dut (
    .clk(clk), .rst_n(rst_n), .inv_l1_ext(1'b0), .sh_clr_en(1'b0), .sh_clr_row('0),
    .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1), .iss_exec(iss_exec),
    .iss_slot(iss_slot_v), .iss_nregs(iss_nregs), .iss_tag(iss_tag_v), .iss_ctx(iss_ctx),
    .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid), .trap_cause(trap_cause),
    .trap_info(trap_info), .busy(busy),
    .cnt_inc_valid(cnt_inc_valid), .cnt_inc_slot(cnt_inc_slot), .cnt_inc_id(cnt_inc_id),
    .cnt_dec_valid(cnt_dec_valid), .cnt_dec_slot(cnt_dec_slot), .cnt_dec_id(cnt_dec_id),
    .rd_valid(l_rd_valid), .rd_ready(rf_rd_ready[0]), .rd_slot(l_rd_slot), .rd_reg(l_rd_reg), .rd_beat(l_rd_beat),
    .rd_hsel(l_rd_hsel), .rdv_valid(rf_rdv_valid[0]), .rdv_err(rf_rdv_err[0]), .rdv_data(rf_rdv_data[0]),
    .wr_valid(l_wr_valid), .wr_ready(rf_wr_ready[1]), .wr_slot(l_wr_slot), .wr_reg(l_wr_reg), .wr_beat(l_wr_beat),
    .wr_lanes(l_wr_lanes), .wr_hsel(l_wr_hsel), .wr_data(l_wr_data),
    .ur_valid(l_ur_valid), .ur_slot(l_ur_slot), .ur_reg(l_ur_reg), .urd_valid(rf_urd_valid[1]), .urd_data(rf_urd_data[1]),
    .uw_valid(l_uw_valid), .uw_slot(l_uw_slot), .uw_reg(l_uw_reg), .uw_data(l_uw_data),
    .mmu_req_valid(mmu_req_valid), .mmu_req_ready(mmu_req_ready), .mmu_req_vmid(mmu_req_vmid),
    .mmu_req_va(mmu_req_va), .mmu_req_acc(mmu_req_acc), .mmu_req_atomic(mmu_req_atomic), .mmu_req_tag(mmu_req_tag),
    .mmu_resp_valid(mmu_resp_valid), .mmu_resp_ready(mmu_resp_ready), .mmu_resp_fault(mmu_resp_fault),
    .mmu_resp_sparse(mmu_resp_sparse), .mmu_resp_pa(mmu_resp_pa), .mmu_resp_pte(mmu_resp_pte),
    .ext_flt_valid(ext_flt_valid), .ext_flt_vmid(ext_flt_vmid), .ext_flt_va(ext_flt_va), .ext_flt_acc(ext_flt_acc),
    .mem_req_valid(mem_req_valid), .mem_req_ready(mem_req_ready), .mem_req(mem_req),
    .mem_rsp_valid(mem_rsp_valid), .mem_rsp(mem_rsp),
    .u_req_valid(u_req_valid), .u_req_ready(u_req_ready), .u_req(u_req), .u_req_slot(u_req_slot),
    .u_rsp_valid(u_rsp_valid), .u_rsp_slot(u_rsp_slot), .u_rsp_err(u_rsp_err),
    .u_rd_req(u_rd_req), .u_rd_slot(u_rd_slot), .u_rd_gnt(u_rd_gnt), .u_rd_data(u_rd_data), .u_free(u_free),
    .stat_l1_rd(st_l1_rd), .stat_l1_wr(st_l1_wr), .stat_l1_amo(st_l1_amo), .stat_sh_rounds(st_sh_rounds),
    .stat_hit(st_hit), .stat_miss(st_miss), .stat_fill(st_fill), .stat_inv(st_inv)
  );

  // ---------------------------------------------------------------- MMU stub with a fault window
  typedef enum int {M_IDLE, M_ACC} mst_t;
  mst_t mst = M_IDLE;
  logic [63:0] m_va;
  int mmu_reqs = 0;
  logic [47:0] remap_off = 48'h0010_0000;
  always @(negedge clk) begin
    mmu_req_ready <= 1'b0;
    mmu_resp_valid <= 1'b0;
    case (mst)
      M_IDLE: if (mmu_req_valid) begin m_va = mmu_req_va; mmu_req_ready <= 1'b1; mst <= M_ACC; end
      M_ACC: begin
        mmu_reqs++;
        mmu_resp_pa <= m_va[47:0];
        mmu_resp_fault <= (|m_va[63:48]) ? 3'd2 : (m_va >= 64'hdead0000 && m_va < 64'hdeae0000) ? 3'd1 : 3'd0;
        // test windows: [0x00800000, +64 KiB) is remapped by remap_off, [0x00b00000, +4 KiB) is sparse
        if (m_va >= 64'h0080_0000 && m_va < 64'h0081_0000) mmu_resp_pa <= m_va[47:0] + remap_off;
        mmu_resp_sparse <= (m_va >= 64'h00b0_0000 && m_va < 64'h00b0_1000); mmu_resp_pte <= 64'd0;
        mmu_resp_valid <= 1'b1;
        mst <= M_IDLE;
      end
      default: mst <= M_IDLE;
    endcase
  end

  // --------------------------------------------------------------
  logic [7:0] mem [longint unsigned];
  int n_rd = 0, n_wr = 0, n_amo = 0, n_inv2 = 0, n_wb2 = 0;
  logic [4:0]  amo_op_s;  logic amo64_s;
  logic [63:0] amo_a_s, amo_b_s, amo_cmp_s, amo_r_s;  logic amo_hit_s;
  ng_l2_amo_alu u_alu (.op(amo_op_s), .is64(amo64_s), .a(amo_a_s), .b(amo_b_s), .cmp(amo_cmp_s), .r(amo_r_s), .cas_hit(amo_hit_s));
  function automatic logic [7:0] mrd(input longint unsigned a); return mem.exists(a) ? mem[a] : 8'd0; endfunction
  function automatic logic [31:0] mrd32(input longint unsigned a);
    return {mrd(a+3), mrd(a+2), mrd(a+1), mrd(a)};
  endfunction
  task automatic mwr32(input longint unsigned a, input logic [31:0] v);
    for (int b = 0; b < 4; b++) mem[a + b] = v[8*b +: 8];
  endtask
  typedef enum int {F_IDLE, F_ACC} fst_t;
  fst_t fst = F_IDLE;
  logic inj_err = 1'b0;     // fabric error injection: every response carries err = 1 (ERR_*)
  mreq_t f_rq;  mrsp_t f_rs;
  always @(negedge clk) begin
    mem_req_ready <= 1'b0;
    mem_rsp_valid <= 1'b0;
    case (fst)
      F_IDLE: if (mem_req_valid) begin f_rq = mem_req; mem_req_ready <= 1'b1; fst <= F_ACC; end
      F_ACC: begin
        f_rs = '0;
        case (f_rq.kind)
          M_RD: begin n_rd++; for (int b = 0; b < 128; b++) f_rs.rdata[8*b +: 8] = mrd(longint'(f_rq.pa) + b); end
          M_WR: begin n_wr++; for (int b = 0; b < 128; b++) if (f_rq.be[b]) mem[longint'(f_rq.pa) + b] = f_rq.wdata[8*b +: 8]; end
          M_AMO: begin
            logic [63:0] o; int nb;
            n_amo++;
            nb = f_rq.amo64 ? 8 : 4; o = '0;
            for (int b = 0; b < nb; b++) o[8*b +: 8] = mrd(longint'(f_rq.pa) + b);
            amo_op_s = f_rq.amo_op; amo64_s = f_rq.amo64; amo_a_s = o; amo_b_s = f_rq.amo_a; amo_cmp_s = f_rq.amo_cmp;
            #1;
            for (int b = 0; b < nb; b++) mem[longint'(f_rq.pa) + b] = amo_r_s[8*b +: 8];
            f_rs.old = o;
          end
          M_INV_L2: n_inv2++;
          M_WB_L2: n_wb2++;
          default: ;
        endcase
        if (inj_err) f_rs.err = 2'd1;
        mem_rsp <= f_rs; mem_rsp_valid <= 1'b1; fst <= F_IDLE;
      end
      default: fst <= F_IDLE;
    endcase
  end

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
  int            u_lo = 0, u_hi = 6;
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
          for (int b = 0; b < 128; b++) line[8 * b +: 8] = mrd(longint'({us_rq[bi].pa[47:7], 7'd0}) + b);
        end else if (us_rq[bi].kind == M_WR) begin
          un_wr++;
          for (int b = 0; b < 128; b++)
            if (us_rq[bi].be[b]) mem[longint'({us_rq[bi].pa[47:7], 7'd0}) + b] = us_rq[bi].wdata[8 * b +: 8];
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

  // M9 event log: completions in order ({trap, cnt id, slot}), done tags, the ext_flt VA of traps
  int          ev_q [$];
  int          tag_q [$];
  int          acc_q [$];
  logic [63:0] flt_q [$];
  always @(posedge clk) if (rst_n) begin
    if (cnt_dec_valid) ev_q.push_back(int'({8'(trap_valid), 8'(cnt_dec_id), 8'(cnt_dec_slot)}));
    if (done_valid) tag_q.push_back(int'(done_tag));
    if (trap_valid && ext_flt_valid) flt_q.push_back(ext_flt_va);
  end

  // --------------------------------------------------------------
  int errors = 0, checks = 0;
  task automatic check(input bit ok, input string what);
    checks++;
    if (!ok) begin errors++; $display("FAIL: %s", what); end
  endtask

  task automatic set_vreg(input int r, input logic [31:0] vals [32]);
    for (int b = 0; b < NBEAT; b++) begin
      logic [DW-1:0] d;
      for (int i = 0; i < NL; i++) d[i*32 +: 32] = vals[b*NL + i];
      t_wr_valid = 1'b1; t_wr_reg = 7'(r); t_wr_beat = BTW'(b); t_wr_data = d;
      @(negedge clk);
      while (!rf_wr_ready[0]) @(negedge clk);
    end
    t_wr_valid = 1'b0;
  endtask
  task automatic get_vreg(input int r, output logic [31:0] vals [32]);
    for (int b = 0; b < NBEAT; b++) begin
      t_rd_valid = 1'b1; t_rd_reg = 7'(r); t_rd_beat = BTW'(b);
      @(negedge clk);
      while (!rf_rd_ready[0]) @(negedge clk);
      for (int i = 0; i < NL; i++) vals[b*NL + i] = rf_rdv_data[0][i*32 +: 32];
      t_rd_valid = 1'b0;
    end
  endtask
  task automatic set_ureg(input int u, input logic [31:0] v);
    t_uw_valid = 1'b1; t_uw_reg = 6'(u); t_uw_data = v;
    @(negedge clk);
    t_uw_valid = 1'b0;
  endtask
  task automatic get_ureg(input int u, output logic [31:0] v);
    t_ur_valid = 1'b1; t_ur_reg = 6'(u);
    @(negedge clk);
    t_ur_valid = 1'b0;
    @(negedge clk);
    v = rf_urd_data[0];
  endtask

  // vector register pair of per-lane 64-bit pointers
  task automatic set_ptr(input int r, input longint unsigned base, input int stride);
    logic [31:0] lo [32];
    logic [31:0] hi [32];
    for (int l = 0; l < 32; l++) begin
      lo[l] = 32'(base + longint'(stride) * l);
      hi[l] = 32'((base + longint'(stride) * l) >> 32);
    end
    set_vreg(r, lo); set_vreg(r + 1, hi);
  endtask
  task automatic set_lin(input int r, input logic [31:0] base, input int stride);
    logic [31:0] v [32];
    for (int l = 0; l < 32; l++) v[l] = base + 32'(stride * l);
    set_vreg(r, v);
  endtask

  // instruction words
  function automatic logic [63:0] mk_m(input logic [6:0] opc, input int rd, input int rs1, input int rs2, input int f5,
                                       input int sp, input int cpol, input int imm);
    logic [31:0] w0, w1;
    w0 = {f5[4:0], 6'(rs2), 7'(rs1), 7'(rd), opc};
    w1 = {imm[23:0], 2'd0, 3'(cpol), 3'(sp)};
    return {w1, w0};
  endfunction
  function automatic logic [63:0] mk_um(input int f7, input int rd, input int rs1, input int rs2, input int cpol, input int imm);
    logic [31:0] w0, w1;
    w0 = {f7[6:0], 6'(rs2), 6'(rs1), 6'(rd), 7'h25};
    w1 = {5'd0, 3'(cpol), imm[23:0]};
    return {w1, w0};
  endfunction
  localparam logic [6:0] OP_LD = 7'h15, OP_ST = 7'h19, OP_AT = 7'h1d, OP_ATR = 7'h21;

  // run one instruction; returns whether it trapped
  logic last_trap;
  logic [3:0]  last_cause;
  logic [31:0] last_info;
  int  last_cycles;
  bit  saw_ext_flt;
  logic [63:0] saw_ext_va;
  task automatic run(input logic [63:0] w, input logic [31:0] exec, input int nregs);
    int guard;
    dbg = 1'b0;
    @(negedge clk);
    iss_w0 = w[31:0]; iss_w1 = w[63:32]; iss_exec = exec; iss_nregs = 8'(nregs); iss_valid = 1'b1;
    #1;
    check(cnt_inc_valid && cnt_inc_slot == SW'(SLOT), "cnt_inc pulse at accept");
    @(negedge clk);
    iss_valid = 1'b0;
    guard = 0; last_trap = 0; saw_ext_flt = 0;
    while (!done_valid && !trap_valid && guard < 100000) begin @(negedge clk); guard++; end
    last_cycles = guard;
    check(guard < 100000, "instruction completes");
    last_trap = trap_valid; last_cause = trap_cause; last_info = trap_info;
    check(cnt_dec_valid && cnt_dec_slot == SW'(SLOT), "cnt_dec pulse with done/trap");
    if (ext_flt_valid) begin saw_ext_flt = 1; saw_ext_va = ext_flt_va; end
    repeat (2) @(negedge clk);
    dbg = 1'b1;
    @(negedge clk);
  endtask

  int d_rd, d_wr, d_amo, d_hit, d_rounds;
  localparam int NGS = 32 / SHL;                   // shared-memory groups (rounds of a conflict-free 32-lane access)
  logic [31:0] vv [32];
  logic [31:0] uu;
  logic [31:0] zero32 [32];

  task automatic snap();
    d_rd = st_l1_rd; d_wr = st_l1_wr; d_amo = st_l1_amo; d_hit = st_hit; d_rounds = st_sh_rounds;
  endtask

  longint unsigned A0 = 64'h0040_0000;

  // ---------------------------------------------------------------- M9: cross-instruction concurrency (LSU_K >= 2)
  localparam longint unsigned HB = 64'h0000_8000_0040_0000;      // host aperture (PA[47]): the uc channel
  int          ex [$];                                            // expected completions {trap, cnt id, slot}
  logic [31:0] tinfo_q [$];
  always @(posedge clk) if (rst_n && trap_valid) tinfo_q.push_back(trap_info);

  function automatic int evc(input bit trap, input int id, input int slot);
    return int'({8'(trap), 8'(id), 8'(slot)});
  endfunction
  task automatic m9_alloc();
    for (int s = 3; s < 10; s++) begin
      alloc_valid = 1'b1; alloc_slot = SW'(s); alloc_base = 10'((s - 2) * 128); alloc_nregs = 8'd128;
      @(negedge clk);
    end
    alloc_valid = 1'b0;
    @(negedge clk);
  endtask
  task automatic m9_issue(input logic [63:0] w, input int slot, input int tag, input int id, input bit trap,
                          input logic [31:0] exec);
    @(negedge clk);
    iss_w0 = w[31:0]; iss_w1 = w[63:32]; iss_exec = exec; iss_nregs = 8'd128; iss_slot_v = SW'(slot);
    iss_tag_v = 16'(tag); iss_valid = 1'b1;
    #1;
    while (!iss_ready) begin @(negedge clk); #1; end
    acc_q.push_back(tag);
    if (!trap) ex.push_back(evc(1'b0, id, slot)); else ex.push_back(evc(1'b1, id, slot));
    @(negedge clk);
    iss_valid = 1'b0; iss_slot_v = SW'(SLOT); iss_tag_v = 16'd0;
  endtask
  task automatic m9_start();
    ev_q.delete(); tag_q.delete(); acc_q.delete(); ex.delete(); flt_q.delete(); tinfo_q.delete();
    dbg = 1'b0;
  endtask
  // wait for every expected completion, then compare the completion order with the acceptance order
  task automatic m9_end(input string what);
    int g;
    g = 0;
    while (ev_q.size() < ex.size() && g < 400000) begin @(negedge clk); g++; end
    check(ev_q.size() == ex.size(), $sformatf("%s: %0d of %0d completions", what, ev_q.size(), ex.size()));
    repeat (4) @(negedge clk);
    check(!busy, {what, ": LSU idle afterwards"});
    for (int i = 0; i < ex.size() && i < ev_q.size(); i++)
      check(ev_q[i] == ex[i], $sformatf("%s: completion %0d = %06h (trap/id/slot), expected %06h (acceptance order)",
                                        what, i, ev_q[i], ex[i]));
    for (int i = 0, j = 0; i < acc_q.size(); i++) begin
      if (ex[i][16]) continue;                       // traps carry no tag
      check(j < tag_q.size() && tag_q[j] == acc_q[i], $sformatf("%s: done tag %0d in acceptance order", what, j));
      j++;
    end
    dbg = 1'b1;
    @(negedge clk);
  endtask

  task automatic m9_tests();
    logic [31:0] v [32];
    logic [31:0] w [32];
    int tg, nl, s1;
    $display("NOTE M9 (LSU_K=%0d MEMOUT=%0d)", LSUK, MEMOUT);
    m9_alloc();
    u_lo = 0; u_hi = 6;
    // ---- M9a: 8 slots, stores, then loads of the next slot's store region (cross-slot RAW on the same lines,
    //      through the line conflict), then 2-line loads; completion order = acceptance order
    for (int s = 2; s < 10; s++) begin
      tb_slot = SW'(s);
      set_ptr(2, HB + 64'h10000 + 64'(s) * 4096, 4);
      set_ptr(4, HB + 64'h10000 + 64'((s - 2 + 1) % 8 + 2) * 4096, 4);
      set_ptr(0, HB + 64'(s) * 4096, 4);
      set_lin(8, 32'h5000_0000 + 32'(s) * 256, 1);
      for (int l = 0; l < 64; l++) mwr32(HB + 64'(s) * 4096 + 4 * l, 32'h1100_0000 + 32'(s) * 256 + l);
    end
    tb_slot = SW'(SLOT);
    m9_start();
    tg = 16'h100;
    for (int s = 2; s < 10; s++) m9_issue(mk_m(OP_ST, 8, 2, 0, 2, 0, 0, 0), s, tg++, 1, 0, 32'hffffffff);
    for (int s = 2; s < 10; s++) m9_issue(mk_m(OP_LD, 9, 4, 0, 4, 0, 0, 0), s, tg++, 0, 0, 32'hffffffff);
    for (int s = 2; s < 10; s++) m9_issue(mk_m(OP_LD, 10, 0, 0, 4, 0, 0, 64), s, tg++, 0, 0, 32'hffffffff);
    m9_end("M9a");
    for (int s = 2; s < 10; s++) begin
      s1 = (s - 2 + 1) % 8 + 2;
      tb_slot = SW'(s);
      get_vreg(9, v);
      for (int l = 0; l < 32; l++) check(v[l] === 32'h5000_0000 + 32'(s1) * 256 + l, "M9a load after the other slot's store (RAW)");
      get_vreg(10, v);
      for (int l = 0; l < 32; l++) check(v[l] === 32'h1100_0000 + 32'(s) * 256 + 16 + l, "M9a 2-line load data");
      for (int l = 0; l < 32; l++) check(mrd32(HB + 64'h10000 + 64'(s) * 4096 + 4 * l) === 32'h5000_0000 + 32'(s) * 256 + l, "M9a store memory");
    end
    check(un_max >= 2, "M9a several uc requests in flight");
    // ---- M9b: late errors (fabric error answers) behind a slow load; younger instructions complete
    for (int s = 2; s < 6; s++) begin
      tb_slot = SW'(s);
      set_lin(11, 32'hbad0_0000 + 32'(s) * 256, 1);
    end
    for (int l = 0; l < 32; l++) mwr32(U_SLOW_LO + 4 * l, 32'h2200_0000 + l);
    tb_slot = 2; set_ptr(0, U_SLOW_LO, 4);
    tb_slot = 3; set_ptr(0, U_ERR_LO, 4);
    tb_slot = 4; set_ptr(0, HB + 4 * 4096, 4);
    tb_slot = 5; set_ptr(2, U_ERR_LO - 64, 4);
    tb_slot = SW'(SLOT);
    begin
      int le0;
      le0 = dut.sim_late_err;
      m9_start();
      m9_issue(mk_m(OP_LD, 11, 0, 0, 4, 0, 0, 0), 2, 16'h200, 0, 0, 32'hffffffff);
      m9_issue(mk_m(OP_LD, 11, 0, 0, 4, 0, 0, 0), 3, 16'h201, 0, 1, 32'hffffffff);
      m9_issue(mk_m(OP_LD, 11, 0, 0, 4, 0, 0, 0), 4, 16'h202, 0, 0, 32'hffffffff);
      m9_issue(mk_m(OP_ST, 8, 2, 0, 2, 0, 0, 0), 5, 16'h203, 1, 1, 32'hffffffff);
      m9_end("M9b");
      check(dut.sim_late_err - le0 == 2, "M9b two late-error traps");
      check(flt_q.size() == 2 && flt_q[0] == U_ERR_LO && flt_q[1] == U_ERR_LO,
            "M9b ext_flt_va = VA of the first lane of the first erroring group");
      check(tinfo_q.size() == 2 && tinfo_q[0] == 32'(U_ERR_LO) && tinfo_q[1] == 32'(U_ERR_LO), "M9b trap info");
    end
    tb_slot = 2; get_vreg(11, v);
    for (int l = 0; l < 32; l++) check(v[l] === 32'h2200_0000 + l, "M9b slow load data");
    tb_slot = 3; get_vreg(11, v);
    for (int l = 0; l < 32; l++) check(v[l] === 32'hbad0_0000 + 3 * 256 + l, "M9b late error: no register write");
    tb_slot = 4; get_vreg(11, v);
    for (int l = 0; l < 32; l++) check(v[l] === 32'h1100_0000 + 4 * 256 + l, "M9b younger load completes");
    // ---- M9c: precise (PROBE) trap of an instruction accepted behind an in-flight one: reported after its done
    tb_slot = 2; set_ptr(0, U_SLOW_LO + 64'h400, 4);
    tb_slot = 3; set_ptr(0, 64'hdead0000, 4);
    tb_slot = SW'(SLOT);
    begin
      int tw0;
      tw0 = dut.sim_twait;
      m9_start();
      m9_issue(mk_m(OP_LD, 12, 0, 0, 4, 0, 0, 0), 2, 16'h300, 0, 0, 32'hffffffff);
      m9_issue(mk_m(OP_LD, 12, 0, 0, 4, 0, 0, 0), 3, 16'h301, 0, 1, 32'hffffffff);
      m9_end("M9c");
      check(tinfo_q.size() == 1 && tinfo_q[0] == 32'hdead0000, "M9c precise trap info");
      check(dut.sim_twait > tw0, "M9c the FE trap waited for the older instruction (S_TWAIT)");
    end
    // ---- M9d: ring full -- min(K + 2, 8) slow loads back to back
    nl = (LSUK + 2 < 8) ? LSUK + 2 : 8;
    for (int s = 2; s < 2 + nl; s++) begin
      tb_slot = SW'(s);
      set_ptr(0, U_SLOW_LO + 64'h800 + 64'(s) * 128, 4);
      for (int l = 0; l < 32; l++) mwr32(U_SLOW_LO + 64'h800 + 64'(s) * 128 + 4 * l, 32'h3300_0000 + 32'(s) * 256 + l);
    end
    tb_slot = SW'(SLOT);
    begin
      int ra0;
      ra0 = dut.sim_ralloc;
      m9_start();
      for (int s = 2; s < 2 + nl; s++) m9_issue(mk_m(OP_LD, 12, 0, 0, 4, 0, 0, 0), s, 16'h400 + s, 0, 0, 32'hffffffff);
      m9_end("M9d");
      check(dut.sim_ralloc > ra0, "M9d the FE waited for a ring entry (S_RALLOC)");
      check(dut.sim_ring_max == LSUK, "M9d the ring was full");
    end
    for (int s = 2; s < 2 + nl; s++) begin
      tb_slot = SW'(s); get_vreg(12, v);
      for (int l = 0; l < 32; l++) check(v[l] === 32'h3300_0000 + 32'(s) * 256 + l, "M9d data");
    end
    // ---- M9e: a synchronous instruction (ld.shared) waits until the ring has drained
    tb_slot = 2; set_ptr(0, U_SLOW_LO + 64'h1000, 4);
    tb_slot = 3; set_lin(6, 32'd0, 4);
    tb_slot = SW'(SLOT);
    begin
      int sy0;
      sy0 = dut.sim_sync;
      m9_start();
      m9_issue(mk_m(OP_LD, 13, 0, 0, 4, 0, 0, 0), 2, 16'h500, 0, 0, 32'hffffffff);
      m9_issue(mk_m(OP_LD, 13, 6, 0, 4, 3, 0, 0), 3, 16'h501, 3, 0, 32'hffffffff);
      m9_end("M9e");
      check(dut.sim_sync > sy0, "M9e ld.shared waited for the drain (S_SYNC)");
    end
    // ---- M9f: uld (asynchronous) on the uc channel behind a slow load
    tb_slot = 4;
    set_ureg(2, 32'(HB + 4 * 4096)); set_ureg(3, 32'((HB + 4 * 4096) >> 32));
    tb_slot = SW'(SLOT);
    m9_start();
    m9_issue(mk_m(OP_LD, 13, 0, 0, 4, 0, 0, 0), 2, 16'h600, 0, 0, 32'hffffffff);
    m9_issue(mk_um(4, 16, 2, 0, 2, 4), 4, 16'h601, 4, 0, 32'd0);
    m9_end("M9f");
    tb_slot = 4;
    for (int k = 0; k < 16; k++) begin
      logic [31:0] uv;
      get_ureg(16 + k, uv);
      check(uv === mrd32(HB + 4 * 4096 + 4 + 4 * k), "M9f uld.b512 data");
    end
    tb_slot = SW'(SLOT);
    // ---- M9g: sliced scratch (b64 at offset 8: two dword columns -> two ring entries) on the uc channel
    begin
      ctx_t c0;
      c0 = iss_ctx;
      iss_ctx.scr_base = 48'h8000_00d0_0000; iss_ctx.scr_bytes = 32'd64; iss_ctx.gslot = 32'd0;
      tb_slot = 5;
      set_lin(6, 32'd8, 0);
      set_lin(14, 32'h7700_0000, 1);
      set_lin(15, 32'h7800_0000, 1);
      tb_slot = 6;
      set_lin(6, 32'd8, 0);
      tb_slot = SW'(SLOT);
      m9_start();
      m9_issue(mk_m(OP_LD, 13, 0, 0, 4, 0, 0, 0), 2, 16'h700, 0, 0, 32'hffffffff);
      m9_issue(mk_m(OP_ST, 14, 6, 0, 3, 4, 0, 0), 5, 16'h701, 1, 0, 32'hffffffff);   // st.scratch.b64
      m9_issue(mk_m(OP_LD, 16, 6, 0, 5, 4, 0, 0), 6, 16'h702, 0, 0, 32'hffffffff);   // ld.scratch.b64 (slot 6: other R)
      m9_end("M9g");
      for (int l = 0; l < 32; l++) begin
        check(mrd32(64'h8000_00d0_0000 + 256 + 4 * l) === 32'h7700_0000 + l, "M9g scratch dword column 2");
        check(mrd32(64'h8000_00d0_0000 + 384 + 4 * l) === 32'h7800_0000 + l, "M9g scratch dword column 3");
      end
      // the load ran with gslot 0 too (one context for all slots in this TB): same region
      tb_slot = 6; get_vreg(16, v); get_vreg(17, w);
      for (int l = 0; l < 32; l++) check(v[l] === 32'h7700_0000 + l && w[l] === 32'h7800_0000 + l, "M9g scratch load");
      tb_slot = SW'(SLOT);
      iss_ctx = c0;
    end
    // ---- M9h: reset with requests in flight (some answered, some not): nothing completes, no slot leaks, the next
    //      instruction is correct
    u_lo = 20; u_hi = 60;
    for (int s = 2; s < 6; s++) begin
      tb_slot = SW'(s);
      set_ptr(0, ((s % 2) != 0 ? U_SLOW_LO + 64'h2000 : HB + 64'h20000) + 64'(s) * 256, 4);
    end
    tb_slot = SW'(SLOT);
    begin
      int r0, n0, g;
      m9_start();
      r0 = un_rd;
      for (int s = 2; s < 6; s++) m9_issue(mk_m(OP_LD, 12, 0, 0, 4, 0, 0, 0), s, 16'h800 + s, 0, 0, 32'hffffffff);
      g = 0;
      while (un_rd - r0 < 2 && g < 10000) begin @(negedge clk); g++; end
      n0 = ev_q.size();
      check(busy, "M9h LSU busy when it is reset");
      rst_n = 1'b0;
      repeat (6) @(negedge clk);
      rst_n = 1'b1;
      repeat (4) @(negedge clk);
      check(!busy, "M9h idle after the reset");
      alloc_valid = 1'b1; alloc_slot = SW'(SLOT); alloc_base = 10'd0; alloc_nregs = 8'd128;
      @(negedge clk);
      alloc_valid = 1'b0;
      m9_alloc();
      g = 0;
      while (us_al != '0 && g < 2000) begin @(negedge clk); g++; end
      check(us_al == '0, "M9h every slot free after the orphans came back (no leak)");
      check(ev_q.size() == n0, "M9h nothing completed after the reset");
      dbg = 1'b1;
      tb_slot = 2;
      set_ptr(0, HB + 2 * 4096, 4);
      set_lin(12, 32'd0, 0);
      tb_slot = SW'(SLOT);
      m9_start();
      m9_issue(mk_m(OP_LD, 12, 0, 0, 4, 0, 0, 0), 2, 16'h900, 0, 0, 32'hffffffff);
      m9_end("M9h after the reset");
      tb_slot = 2; get_vreg(12, v);
      for (int l = 0; l < 32; l++) check(v[l] === 32'h1100_0000 + 2 * 256 + l, "M9h load after the reset");
      tb_slot = SW'(SLOT);
    end
    u_lo = 0; u_hi = 6;
    check(un_bad == 0, "M9 uc responder protocol errors");
    check(dut.sim_err == 0, "M9 ng_lsu_kn invariant violations");
    $display("NOTE M9 done: uc rd=%0d wr=%0d err=%0d orphans=%0d max_out=%0d ring_max=%0d iof_max=%0d late_err=%0d",
             un_rd, un_wr, un_err, un_orph, un_max, dut.sim_ring_max, dut.sim_iof_max, dut.sim_late_err);
  endtask

  initial begin
    for (int l = 0; l < 32; l++) zero32[l] = 32'd0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (3) @(negedge clk);
    alloc_valid = 1'b1; alloc_slot = SW'(SLOT); alloc_base = 10'd0; alloc_nregs = 8'd128;
    @(negedge clk);
    alloc_valid = 1'b0;
    @(negedge clk);
    for (int r = 0; r < 128; r++) set_vreg(r, zero32);
    for (int u = 0; u < 64; u++) set_ureg(u, 32'd0);
    iss_ctx = '0;
    iss_ctx.sh_base = 16'd0; iss_ctx.sh_size = 32'd8192;
    for (int i = 0; i < 4096; i++) mem[A0 + i] = 8'(i * 7 + 3);

    // ---------------- coalescing: ld.global.b32 (rd=8, rs1=v0:v1, imm 0, def)
    set_ptr(0, A0, 4);                        // contiguous, line aligned (0x400000)
    snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);
    check(!last_trap, "T1 no trap");
    check(st_l1_rd - d_rd == 1, "T1 contiguous 32 x b32 -> 1 line request");
    get_vreg(8, vv);
    for (int l = 0; l < 32; l++) check(vv[l] === mrd32(A0 + 4 * l), "T1 data");
    set_ptr(0, A0 + 64, 4);                   // 64 B offset: straddles two lines
    snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);   // cpol l2: bypass L1
    check(st_l1_rd - d_rd == 2, "T2 offset-64 contiguous -> 2 line requests");
    set_ptr(0, A0, 128);                      // one line per lane
    snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
    check(st_l1_rd - d_rd == 32, "T3 stride-128 -> 32 line requests");
    set_ptr(0, A0 + 8, 0);                    // broadcast
    snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
    check(st_l1_rd - d_rd == 1, "T4 broadcast -> 1 line request");
    get_vreg(8, vv);
    for (int l = 0; l < 32; l++) check(vv[l] === mrd32(A0 + 8), "T4 broadcast data");
    set_ptr(0, A0 + 124, 0);                  // b64 at line end - 4: crosses into the next line
    snap(); run(mk_m(OP_LD, 8, 0, 0, 5, 0, 2, 0), 32'h1, 128);
    check(st_l1_rd - d_rd == 2, "T5 b64 across a line boundary -> 2 line requests");
    get_vreg(8, vv);
    check(vv[0] === mrd32(A0 + 124), "T5 data lo");
    snap(); run(mk_m(OP_LD, 8, 0, 0, 5, 0, 2, 0), 32'h0, 128);
    check(!last_trap && st_l1_rd - d_rd == 0, "T6 EXEC=0 -> no line request");

    // ---------------- stores: coalesced write, last lane wins
    set_ptr(0, A0 + 512, 4);
    set_lin(8, 32'h1000, 1);
    snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
    check(st_l1_wr - d_wr == 1, "T7 contiguous store -> 1 line write");
    for (int l = 0; l < 32; l++) check(mrd32(A0 + 512 + 4 * l) === 32'h1000 + l, "T7 memory");
    set_ptr(0, A0 + 768, 0);                  // all lanes to the same address
    snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
    check(mrd32(A0 + 768) === 32'h1000 + 31, "T8 same-address store: highest active lane wins");
    snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'h0000_00f0, 128);
    check(mrd32(A0 + 768) === 32'h1000 + 7, "T8b same-address store with EXEC=0xf0: lane 7 wins");

    // ---------------- atomics: serial in lane order
    mwr32(A0 + 1024, 32'd100);
    set_ptr(0, A0 + 1024, 0);
    set_lin(8, 32'd1, 1);                     // lane l adds l+1
    snap(); run(mk_m(OP_ATR, 8, 0, 0, 0, 0, 0, 0), 32'hffffffff, 128);   // atom.global.add.u32.rtn
    check(st_l1_amo - d_amo == 32, "T9 one atomic per active lane");
    get_vreg(8, vv);
    begin
      int sum;
      sum = 100;
      for (int l = 0; l < 32; l++) begin
        check(vv[l] === 32'(sum), "T9 returned running prefix");
        sum += l + 1;
      end
      check(mrd32(A0 + 1024) === 32'(sum), "T9 final value");
    end

    // ---------------- L1 behaviour at instruction level
    set_ptr(0, A0 + 2048, 4);
    run(mk_m(OP_LD, 8, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);      // def: allocate
    snap(); run(mk_m(OP_LD, 9, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);
    check(st_hit - d_hit == 1 && st_l1_rd - d_rd == 1, "T10 repeat load hits in L1");
    mwr32(A0 + 2048, 32'hfeedf00d);                                // another agent writes behind the L1
    run(mk_m(OP_LD, 10, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);
    get_vreg(10, vv);
    check(vv[0] !== 32'hfeedf00d, "T11 L1 returns STALE data until invalidated (non-coherent)");
    run(mk_m(OP_LD, 11, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);     // cpol l2: fresh
    get_vreg(11, vv);
    check(vv[0] === 32'hfeedf00d, "T12 cpol l2 load sees the fresh data");
    begin
      int inv0;
      inv0 = st_inv;
      run({32'd0, 32'h000004ce}, 32'd0, 128);                      // cache.inv.l1
      check(st_inv - inv0 == 1, "T13 cache.inv.l1 reaches the L1");
      check(cnt_dec_id === 3'd0 || 1'b1, "T13 id");
    end
    run(mk_m(OP_LD, 12, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);
    get_vreg(12, vv);
    check(vv[0] === 32'hfeedf00d, "T13 after cache.inv.l1 the load is fresh");
    begin
      int i2, w2;
      i2 = n_inv2; w2 = n_wb2;
      run({32'd0, 32'h0000054e}, 32'd0, 128);                      // cache.inv.l2
      run({32'd0, 32'h000005ce}, 32'd0, 128);                      // cache.wb.l2
      check(n_inv2 - i2 == 1 && n_wb2 - w2 == 1, "T14 cache.inv.l2 / cache.wb.l2 forwarded to the fabric");
    end

    // ---------------- precise MemFault
    for (int l = 0; l < 32; l++) mwr32(A0 + 3072 + 4 * l, 32'hdddd0000 + l);
    begin
      logic [31:0] lo [32];
      logic [31:0] hi [32];
      for (int l = 0; l < 32; l++) begin
        lo[l] = 32'(A0 + 3072 + 4 * l); hi[l] = 32'd0;
      end
      lo[5] = 32'hdead1230; hi[5] = 32'd0;                         // faulting lane
      lo[9] = 32'hdead4560; hi[9] = 32'd0;
      set_vreg(0, lo); set_vreg(1, hi);
    end
    set_lin(8, 32'h5a5a0000, 1);
    snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
    check(last_trap && last_cause == 4'd5, "T15 store with faulting lanes traps MemFault");
    check(last_info === 32'hdead1230, "T15 trap info = VA of the FIRST faulting lane");
    check(st_l1_wr == d_wr, "T15 no store was committed to the L1/L2 port (precise)");
    for (int l = 0; l < 32; l++) check(mrd32(A0 + 3072 + 4 * l) === 32'hdddd0000 + l, "T15 memory untouched");
    // same fault, EXEC masks the faulting lanes out: the store goes through
    snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff & ~((32'd1 << 5) | (32'd1 << 9)), 128);
    check(!last_trap, "T16 faulting lanes masked by EXEC: no trap");
    check(mrd32(A0 + 3072 + 4 * 4) === 32'h5a5a0004 && mrd32(A0 + 3072 + 4 * 5) === 32'hdddd0005, "T16 memory");
    // BAD_PHYS: PA outside VRAM/SYS/HOST
    set_ptr(0, 64'h20_0000_0100, 0);
    run(mk_m(OP_LD, 8, 0, 0, 4, 0, 0, 0), 32'h1, 128);
    check(last_trap && last_cause == 4'd5 && last_info === 32'h0000_0100, "T17 bad physical address -> MemFault, info = VA low 32");

    // ---------------- fabric error response (model counterpart since test_fabric_error,
    //                  PhysMem::add_fabric_error; the model is precise, the RTL may already have written earlier line groups)
    begin
      logic [31:0] sent [32];
      for (int l = 0; l < 32; l++) sent[l] = 32'hbad00000 + l;
      set_ptr(0, A0 + 3584, 4);
      set_vreg(8, sent);
      inj_err = 1'b1;
      snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);
      check(last_trap && last_cause == 4'd5 && last_info === 32'(A0 + 3584), "T17a fabric err on load -> MemFault, info = VA of first lane");
      check(saw_ext_flt, "T17a BAD_PHYS reported through ext_flt");
      get_vreg(8, vv);
      for (int l = 0; l < 32; l++) check(vv[l] === sent[l], "T17a destination not written on a faulting load");
      snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      check(last_trap && last_cause == 4'd5 && last_info === 32'(A0 + 3584), "T17b fabric err on store -> MemFault");
      snap(); run(mk_m(OP_ATR, 8, 0, 0, 0, 0, 0, 0), 32'h4, 128);
      check(last_trap && last_cause == 4'd5 && last_info === 32'(A0 + 3584 + 8), "T17c fabric err on atomic -> MemFault, info = lane VA");
      get_vreg(8, vv);
      check(vv[2] === sent[2], "T17c atomic return register not written");
      inj_err = 1'b0;
      snap(); run(mk_m(OP_LD, 9, 0, 0, 4, 0, 0, 0), 32'hffffffff, 128);
      check(!last_trap, "T17d no trap once the fabric is healthy again");
    end

    // NON_CANONICAL (VA[63:48] != 0, VMID 0 bypass): vector global load and uniform load both trap MemFault with
    // info = VA low 32; the access must not be truncated to VA[47:0] (CI regression: lsu-01778)
    set_ptr(0, 64'hffff_ffff_ffd9_5020, 0);
    snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 0, 0), 32'h1, 128);
    check(last_trap && last_cause == 4'd5 && last_info === 32'hffd9_5020, "T18 non-canonical VA (vector ld) -> MemFault, info = VA low 32");
    set_ureg(2, 32'hffd9_5021); set_ureg(3, 32'hffff_ffff);
    snap(); run(mk_um(4, 16, 2, 0, 2, 0), 32'd0, 128);
    check(last_trap && last_cause == 4'd5 && last_info === 32'hffd9_5021, "T19 non-canonical VA (uld.b512) -> MemFault, info = VA low 32");
    check(st_l1_rd == d_rd, "T19 no fabric read was issued");

    // ---------------- shared memory round counts
    set_lin(4, 32'd0, 4);                     // v4 = 4*lane: conflict free
    snap(); run(mk_m(OP_LD, 8, 4, 0, 4, 3, 0, 0), 32'hffffffff, 128);   // ld.shared.b32
    check(!last_trap && st_sh_rounds - d_rounds == NGS, "T20 ld.shared stride-1 dword: one round per lane group");
    set_lin(4, 32'd0, 8);
    snap(); run(mk_m(OP_LD, 8, 4, 0, 4, 3, 0, 0), 32'hffffffff, 128);
    check(st_sh_rounds - d_rounds == NGS * ((SHL * 2 + 31) / 32), "T21 ld.shared stride-2 dwords: 2-way conflict only across a whole 32-lane group");
    set_lin(4, 32'd0, 128);
    snap(); run(mk_m(OP_LD, 8, 4, 0, 4, 3, 0, 0), 32'hffffffff, 128);   // 32 * 128 = 4096 < 8192
    check(st_sh_rounds - d_rounds == 32, "T22 ld.shared stride-32 dwords: 32 rounds (same bank)");
    set_lin(4, 32'd16, 0);
    snap(); run(mk_m(OP_LD, 8, 4, 0, 4, 3, 0, 0), 32'hffffffff, 128);
    check(st_sh_rounds - d_rounds == NGS, "T23 ld.shared broadcast: one round per lane group");
    set_lin(4, 32'd0, 16);
    snap(); run(mk_m(OP_LD, 8, 4, 0, 7, 3, 0, 0), 32'hffffffff, 128);   // ld.shared.b128, 16 B stride
    check(st_sh_rounds - d_rounds == 32 / (SHL < 8 ? SHL : 8), "T24 ld.shared.b128 16 B stride: 8 lanes per round");
    // shared store then load round trip, different lanes / unaligned
    set_lin(4, 32'd1, 5);                     // odd byte offsets
    set_lin(8, 32'hc0de0000, 1);
    run(mk_m(OP_ST, 8, 4, 0, 2, 3, 0, 0), 32'hffffffff, 128);          // st.shared.b32
    run(mk_m(OP_LD, 9, 4, 0, 4, 3, 0, 0), 32'hffffffff, 128);
    get_vreg(9, vv);
    begin
      // later lanes overwrite overlapping bytes of earlier lanes (stride 5, 4-byte stores overlap by 0 -> no overlap)
      for (int l = 0; l < 32; l++) check(vv[l] === 32'hc0de0000 + l, "T25 unaligned shared round trip");
    end
    // out of bounds shared (size 8192)
    set_lin(4, 32'd8192 - 8, 4);
    run(mk_m(OP_LD, 8, 4, 0, 4, 3, 0, 0), 32'hffffffff, 128);
    check(last_trap && last_cause == 4'd11, "T26 shared out of bounds traps SharedBounds");
    check(last_info === 32'(8192 - 8 + 8), "T26 info = offset of the first offending lane (lane 2)");

    // ---------------- uld coalescing
    for (int u = 0; u < 64; u++) set_ureg(u, 32'd0);
    set_ureg(2, 32'(A0)); set_ureg(3, 32'd0);
    snap(); run(mk_um(4, 16, 2, 0, 2, 0), 32'd0, 128);                 // uld.b512 u16.., [u2:u3 + 0], l2
    check(!last_trap && st_l1_rd - d_rd == 1, "T30 uld.b512 aligned 64 B: 1 line request");
    get_ureg(16, uu);
    check(uu === mrd32(A0), "T30 data");
    snap(); run(mk_um(4, 16, 2, 0, 2, 100), 32'd0, 128);               // offset 100: crosses a line
    check(st_l1_rd - d_rd == 2, "T31 uld.b512 at offset 100: 2 line requests");
    get_ureg(31, uu);
    check(uu === mrd32(A0 + 100 + 60), "T31 last dword");


    // ---------------- translation memo (pass 0 answers are reused by pass 1)
    // (a) MMU requests: one per line group (was two); the fabric still sees one request per group
    begin
      int m0;
      set_ptr(0, A0, 128);
      m0 = mmu_reqs; snap(); run(mk_m(OP_LD, 8, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
      check(!last_trap && st_l1_rd - d_rd == 32 && mmu_reqs - m0 == 32, "M1 stride-128 load: 32 lines -> 32 MMU requests (one per group)");
      set_lin(8, 32'h6100, 1);
      m0 = mmu_reqs; snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      check(!last_trap && st_l1_wr - d_wr == 32 && mmu_reqs - m0 == 32, "M2 stride-128 store: 32 lines -> 32 MMU requests");
      for (int l = 0; l < 32; l++) check(mrd32(A0 + 128 * l) === 32'h6100 + l, "M2 memory");
      // more groups than memo entries: b64 lanes crossing a line boundary -> 33 distinct lines; beyond MEMO_N the
      // commit pass falls back to the MMU, results must stay correct
      set_ptr(0, A0 + 32'h20000 + 124, 128);
      for (int l = 0; l < 32; l++) begin mwr32(A0 + 32'h20000 + 124 + 128 * l, 32'hc0000000 + l); mwr32(A0 + 32'h20000 + 128 + 128 * l, 32'hd0000000 + l); end
      m0 = mmu_reqs; snap(); run(mk_m(OP_LD, 8, 0, 0, 5, 0, 2, 0), 32'hffffffff, 128);
      check(!last_trap && st_l1_rd - d_rd == 33, "M3 b64 line-crossing lanes: 33 line requests");
      get_vreg(8, vv);
      check(vv[0] === 32'hc0000000 && vv[31] === 32'hc000001f, "M3 data lo");
      check(mmu_reqs - m0 == 33 + (33 - 32), "M3 groups beyond MEMO_N=32 are translated again in the commit pass");
    end
    // (b) fault on a later line group: precise, nothing committed
    begin
      logic [31:0] lo [32];
      logic [31:0] hi [32];
      for (int l = 0; l < 32; l++) begin lo[l] = 32'(A0 + 128 * l); hi[l] = 32'd0; end
      lo[20] = 32'hdead0100; lo[27] = 32'hdead0200;
      set_vreg(0, lo); set_vreg(1, hi);
      set_lin(8, 32'h7200, 1);
      snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      check(last_trap && last_cause == 4'd5 && last_info === 32'hdead0100, "M4 fault on group 20 -> MemFault of lane 20");
      check(st_l1_wr == d_wr, "M4 no store committed although 19 groups translated fine before the fault");
      for (int l = 0; l < 32; l++) if (l != 20 && l != 27) check(mrd32(A0 + 128 * l) === 32'h6100 + l, "M4 memory untouched");
      // the same registers with the faulting lanes masked: a clean instruction right after the trap uses fresh entries
      snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff & ~((32'd1 << 20) | (32'd1 << 27)), 128);
      check(!last_trap && st_l1_wr - d_wr == 30, "M5 after the trap: 30 line writes");
      for (int l = 0; l < 32; l++)
        check(mrd32(A0 + 128 * l) === ((l == 20 || l == 27) ? 32'h6100 + l : 32'h7200 + l), "M5 memory");
    end
    // (c) page remap between two instructions with the same VAs: the second one must use the new mapping
    begin
      set_ptr(0, 64'h0080_0000, 128);
      set_lin(8, 32'h8100, 1);
      remap_off = 48'h0010_0000;
      run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      for (int l = 0; l < 32; l++) check(mrd32(64'h0090_0000 + 128 * l) === 32'h8100 + l, "M6 first mapping");
      set_lin(8, 32'h8200, 1);
      remap_off = 48'h0020_0000;
      run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      for (int l = 0; l < 32; l++) begin
        check(mrd32(64'h00a0_0000 + 128 * l) === 32'h8200 + l, "M6 second mapping gets the new data");
        check(mrd32(64'h0090_0000 + 128 * l) === 32'h8100 + l, "M6 old mapping untouched");
      end
      run(mk_m(OP_LD, 9, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
      get_vreg(9, vv);
      for (int l = 0; l < 32; l++) check(vv[l] === 32'h8200 + l, "M6 load through the second mapping");
      remap_off = 48'h0010_0000;
      run(mk_m(OP_LD, 9, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
      get_vreg(9, vv);
      for (int l = 0; l < 32; l++) check(vv[l] === 32'h8100 + l, "M6 load back through the first mapping");
    end
    // (d) sparse pages: the memoised sparse bit suppresses the fabric request, neighbours are unaffected
    begin
      logic [31:0] lo [32];
      logic [31:0] hi [32];
      for (int l = 0; l < 32; l++) begin lo[l] = 32'(A0 + 4096 + 128 * l); hi[l] = 32'd0; end
      for (int l = 4; l < 8; l++) lo[l] = 32'h00b0_0000 + 128 * (l - 4);
      set_vreg(0, lo); set_vreg(1, hi);
      set_lin(8, 32'h9300, 1);
      snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      check(!last_trap && st_l1_wr - d_wr == 28, "M7 store with 4 sparse lines: 28 line writes");
      for (int l = 0; l < 32; l++)
        if (l >= 4 && l < 8) check(mrd32(64'h00b0_0000 + 128 * (l - 4)) === 32'd0, "M7 sparse line not written");
        else check(mrd32(A0 + 4096 + 128 * l) === 32'h9300 + l, "M7 memory");
      snap(); run(mk_m(OP_LD, 9, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
      check(!last_trap && st_l1_rd - d_rd == 28, "M7 load with 4 sparse lines: 28 line reads");
      get_vreg(9, vv);
      for (int l = 0; l < 32; l++) if (l < 4 || l >= 8) check(vv[l] === 32'h9300 + l, "M7 load data");
    end
    // (e) abort (core reset) in the middle of an instruction, in the probe pass and in the commit pass; the next
    // instruction has other groups and must not see stale memo contents
    for (int ph = 0; ph < 2; ph++) begin
      int m0, w0, g;
      logic [63:0] w;
      set_ptr(0, A0 + 8192, 128);
      set_lin(8, 32'hab00, 1);
      w = mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0);
      dbg = 1'b0;
      @(negedge clk);
      m0 = mmu_reqs; w0 = n_wr;
      iss_w0 = w[31:0]; iss_w1 = w[63:32]; iss_exec = 32'hffffffff; iss_nregs = 8'd128; iss_valid = 1'b1;
      @(negedge clk);
      iss_valid = 1'b0;
      g = 0;
      while (!(ph == 0 ? (mmu_reqs - m0 >= 12) : (n_wr - w0 >= 3)) && g < 100000) begin @(negedge clk); g++; end
      check(g < 100000 && !done_valid && !trap_valid, "M8 instruction is still running when it is aborted");
      rst_n = 1'b0;
      repeat (6) @(negedge clk);
      rst_n = 1'b1;
      repeat (4) @(negedge clk);
      dbg = 1'b1;
      alloc_valid = 1'b1; alloc_slot = SW'(SLOT); alloc_base = 10'd0; alloc_nregs = 8'd128;
      @(negedge clk);
      alloc_valid = 1'b0;
      @(negedge clk);
      check(!busy, "M8 idle after the reset");
      // different groups: 2 lines (offset 64 contiguous) with other VAs
      set_ptr(0, A0 + 12288 + 64, 4);
      set_lin(8, 32'hcd00, 1);
      m0 = mmu_reqs; snap(); run(mk_m(OP_ST, 8, 0, 0, 2, 0, 0, 0), 32'hffffffff, 128);
      check(!last_trap && st_l1_wr - d_wr == 2 && mmu_reqs - m0 == 2, "M8 post-abort store: 2 lines, 2 MMU requests");
      if (st_l1_wr - d_wr != 2 || mmu_reqs - m0 != 2) $display("M8 ph %0d: wr %0d mmu %0d trap %0b", ph, st_l1_wr - d_wr, mmu_reqs - m0, last_trap);
      for (int l = 0; l < 32; l++) check(mrd32(A0 + 12288 + 64 + 4 * l) === 32'hcd00 + l, "M8 post-abort memory");
      run(mk_m(OP_LD, 9, 0, 0, 4, 0, 2, 0), 32'hffffffff, 128);
      get_vreg(9, vv);
      for (int l = 0; l < 32; l++) check(vv[l] === 32'hcd00 + l, "M8 post-abort load");
    end

    if (LSUK > 1) m9_tests();
    $display("NG_LSU_DIR checks=%0d errors=%0d | mmu reqs=%0d fabric rd=%0d wr=%0d amo=%0d | LSU_K=%0d MEMOUT=%0d uc rd=%0d wr=%0d err=%0d orphans=%0d max_out=%0d",
             checks, errors, mmu_reqs, n_rd, n_wr, n_amo, LSUK, MEMOUT, un_rd, un_wr, un_err, un_orph, un_max);
    if (errors == 0) $display("NG_LSU_DIR_TB_PASS");
    else $display("NG_LSU_DIR_TB_FAIL");
    $finish;
  end

  initial begin
    repeat (8) #1000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
