// ng_gpu_rm_cp.sv -- CP reconfigurable module (rm_kind 0x8002): the CP subsystem (Ibex + hardware boot loader
// + Copy engine + register block + GPU MMU) behind the DFX boundary, without L2 / cores.
//
// Same port list as every RM (ng_gpu_rm_ports.svh); a different RM variant of module ng_gpu_rm.  Design and
// tentative decisions:
//   clk_l2   boundary side: s_axil_ctrl registers (ng_rcp_regs), BAR2 C3 window (ng_rcp_bar2), the C3 and host
//            AXI masters (ng_rcp_c3eng / ng_rcp_hosteng), quiesce / ready, the GPU MMU (ng_mmu: BAR0 0x0200..0x033F,
//            CP VMID != 0 translation in ng_rcp_memif_l2, page-table reads on C3 engine requester 2 via ng_rcp_mmurd)
//   clk_ts   CP domain (99.942 MHz, plan A): ng_cp_top; register bridge ng_rcp_cpbus, memory port ng_rcp_memif_ts
// What this RM can do: everything up to the command processor -- firmware boot from C3, ring consumption, FENCE /
// TIMESTAMP / COPY / FILL / WRITE_DATA / WAIT_MEM on physical (VMID 0) SYS and HOST addresses, user IBs (INDIRECT with
// VMID 1..15: CP accesses are translated by ng_mmu and must land in SYS or HOST), fence + RPTR write-back to host
// memory, IRQ (IRQ_STATUS bit 2 = MMU fault).  What it cannot: VRAM (a translation to VRAM is BAD_PHYS_ADDR),
// DISPATCH (the work distributor port completes immediately), VRAM through BAR2.
// MMU: rm_has_mmu in the driver only needs RM_ID and BAR2_INFO (already provided); the context / invalidate /
// fault registers are the real ng_mmu register file.
module ng_gpu_rm
  import ng_dfx_pkg::*;
  import ng_cp_pkg::*;
#(
  parameter logic [31:0] BUILD_ID = 32'h0000_0000,
  parameter logic [15:0] RM_REV   = 16'h0001
) (
`include "ng_gpu_rm_ports.svh"
);
  localparam logic [15:0] RM_KIND_CP = 16'h8002;
  localparam logic [3:0]  MEM_MASK   = 4'b0100;       // C3 only
  localparam int          QAGE       = 20;            // > the round trip of the quiesce / idle paths

  wire clk = clk_l2;
  (* keep = "true", shreg_extract = "no" *) logic rst_n;
  always_ff @(posedge clk) rst_n <= rst_l2_n;

  assign rm_info = {12'h000, MEM_MASK, RM_REV, BUILD_ID, RM_KIND_CP, NG_DFX_IFACE_VERSION, NG_RM_INFO_MAGIC};

  // ------------------------------------------------------------------ BAR0 registers + CP bridge
  logic        win_en;
  logic [4:0]  win_log2;
  logic [30:0] win_base;
  logic        cp_req, cp_wr, cp_ack_ts;
  logic [12:0] cp_addr;
  logic [31:0] cp_wdata, cp_rdata;
  (* ASYNC_REG = "TRUE" *) logic ack_s1, ack_s2;
  logic        csr_wr;
  logic [11:0] csr_waddr, csr_raddr;
  logic [31:0] csr_wdata, csr_rdata;
  always_ff @(posedge clk) begin ack_s1 <= cp_ack_ts; ack_s2 <= ack_s1; end

  logic [31:0] l2dbg_none [16];       // no debug page in this RM (ng_rcp_regs DBG_PAGE = 0)
  assign l2dbg_none = '{default: 32'h0};
  ng_rcp_regs #(.RM_REV(RM_REV)) u_regs (
    .clk(clk), .rst_n(rst_n),
    .s_awaddr(s_axil_ctrl_awaddr), .s_awvalid(s_axil_ctrl_awvalid), .s_awready(s_axil_ctrl_awready),
    .s_wdata(s_axil_ctrl_wdata), .s_wstrb(s_axil_ctrl_wstrb), .s_wvalid(s_axil_ctrl_wvalid),
    .s_wready(s_axil_ctrl_wready), .s_bvalid(s_axil_ctrl_bvalid), .s_bready(s_axil_ctrl_bready),
    .s_araddr(s_axil_ctrl_araddr), .s_arvalid(s_axil_ctrl_arvalid), .s_arready(s_axil_ctrl_arready),
    .s_rdata(s_axil_ctrl_rdata), .s_rvalid(s_axil_ctrl_rvalid), .s_rready(s_axil_ctrl_rready),
    .win_en(win_en), .win_log2(win_log2), .win_base(win_base),
    .cp_req(cp_req), .cp_wr(cp_wr), .cp_addr(cp_addr), .cp_wdata(cp_wdata), .cp_ack(ack_s2), .cp_rdata(cp_rdata),
    .csr_wr(csr_wr), .csr_waddr(csr_waddr), .csr_wdata(csr_wdata), .csr_raddr(csr_raddr), .csr_rdata(csr_rdata),
    .l2dbg(l2dbg_none));
  assign s_axil_ctrl_bresp = 2'b00;
  assign s_axil_ctrl_rresp = 2'b00;

  // ------------------------------------------------------------------ quiesce / ready
  logic        q_in;
  logic [5:0]  q_age;
  logic [3:0]  rdy_cnt;
  logic        c3_idle, host_idle;
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rm_ready <= 1'b0; rm_quiesce_ack <= 1'b0; q_in <= 1'b0; q_age <= '0; rdy_cnt <= '0;
    end else begin
      if (rdy_cnt != 4'd10) rdy_cnt <= rdy_cnt + 1'b1;
      rm_ready <= (rdy_cnt == 4'd10);
      q_in     <= shell_quiesce_req;
      if (!q_in)                  q_age <= '0;
      else if (q_age != 6'(QAGE)) q_age <= q_age + 1'b1;
      rm_quiesce_ack <= q_in && (q_age == 6'(QAGE)) && c3_idle && host_idle;
    end
  end

  // ------------------------------------------------------------------ CP (clk_ts)
  logic        host_wr, host_rd;
  logic [12:0] host_addr;
  logic [31:0] host_wdata, host_rdata;
  logic        m_req_valid, m_req_ready, m_rsp_valid, m_rsp_fault;
  mem_req_t    m_req;
  logic [63:0] m_rsp_data;
  logic        wd_start, wd_done, cp_irq, core_reset, cpu_quiet, mmu_irq_ts;
  wd_req_t     wd_req;
  logic        dbg_valid;
  logic [31:0] dbg_code;

  ng_cp_top #(.BOOT_LOADER(1'b1), .IBEX_IMAGE(""), .TICK_CYCLES(99_000)) u_cp (   // 99.94 MHz: tick just under 1 ms
    .clk(clk_ts), .rst_n(rst_ts_n),
    .host_wr(host_wr), .host_rd(host_rd), .host_addr(host_addr), .host_wdata(host_wdata), .host_rdata(host_rdata),
    .irq_ext_i({29'h0, mmu_irq_ts, 2'b00}), .rm_irq_o(cp_irq),
    .m_req_valid(m_req_valid), .m_req_ready(m_req_ready), .m_req(m_req),
    .m_rsp_valid(m_rsp_valid), .m_rsp_fault(m_rsp_fault), .m_rsp_data(m_rsp_data),
    .cb_valid(), .cb_fill(), .cb_vmid(), .cb_src(), .cb_dst(), .cb_size(), .cb_fillv(), .cb_abort(),
    .cb_done(1'b0), .cb_fault(1'b0),
    .wd_start(wd_start), .wd_req(wd_req), .wd_done(wd_done), .wd_fault(1'b0), .wd_instr(64'h0), .sp_idle_i(1'b1), .wd_abort_o(), .fab_busy_i(1'b0), .scrub_set_i(16'h0), .sp_inv_o(),
    .dbg_valid_o(dbg_valid), .dbg_code_o(dbg_code), .cpu_quiet_o(cpu_quiet), .core_reset_o(core_reset));

  // no shader cores: a DISPATCH is acknowledged as done at once
  always_ff @(posedge clk_ts or negedge rst_ts_n) begin
    if (!rst_ts_n) wd_done <= 1'b0;
    else           wd_done <= wd_start;
  end

  logic cpb_ack;
  ng_rcp_cpbus u_cpbus (
    .clk(clk_ts), .rst_n(rst_ts_n),
    .req_tog_i(cp_req), .wr_i(cp_wr), .addr_i(cp_addr), .wdata_i(cp_wdata),
    .ack_tog_o(cpb_ack), .rdata_o(cp_rdata),
    .host_wr(host_wr), .host_rd(host_rd), .host_addr(host_addr), .host_wdata(host_wdata), .host_rdata(host_rdata));
  assign cp_ack_ts = cpb_ack;

  // CP interrupt level -> clk_l2
  (* ASYNC_REG = "TRUE" *) logic irq_s1, irq_s2;
  always_ff @(posedge clk) begin irq_s1 <= cp_irq; irq_s2 <= irq_s1; end
  assign rm_irq = {3'b000, irq_s2};

  // ------------------------------------------------------------------ CP memory port -> C3 / host
  logic        x_req, x_host, x_wr, x_ack, y_err;
  logic [3:0]  x_vmid;
  logic [47:0] x_addr;
  logic [46:0] o_addr;
  logic [7:0]  x_wstrb;
  logic [63:0] x_wdata, y_rdata;
  logic        ml_busy;
  logic        c1_valid, c1_ready, c1_rs_valid, c1_rs_err;
  logic [63:0] c1_rs_rdata;
  logic        h_valid, h_ready, h_rs_valid, h_rs_err;
  logic [63:0] h_rs_rdata;

  // ------------------------------------------------------------------ GPU MMU (clk_l2)
  logic        t_req_valid, t_req_ready, t_resp_valid, t_resp_ready, t_resp_sparse;
  logic [3:0]  t_req_vmid;
  logic [63:0] t_req_va, t_resp_pte;
  logic [1:0]  t_req_acc, t_resp_level;
  logic [2:0]  t_resp_fault, t_resp_size;
  logic [47:0] t_resp_pa;
  logic [7:0]  t_resp_tag;
  logic        ef_valid;
  logic [3:0]  ef_vmid;
  logic [63:0] ef_va, ef_pte;
  logic [1:0]  ef_acc, ef_level;
  logic [2:0]  ef_code;
  logic        mm_rd_valid, mm_rd_ready, mm_rsp_valid, mm_rsp_err, mmu_irq;
  logic [47:0] mm_rd_addr;
  logic [63:0] mm_rsp_data;
  logic [31:0] mm_walks, mm_hits;
  logic        c2_valid, c2_ready, c2_rs_valid;
  logic [30:0] c2_addr;

  ng_rcp_memif_ts u_mts (
    .clk(clk_ts), .rst_n(rst_ts_n),
    .m_req_valid(m_req_valid), .m_req_ready(m_req_ready), .m_req(m_req),
    .m_rsp_valid(m_rsp_valid), .m_rsp_fault(m_rsp_fault), .m_rsp_data(m_rsp_data),
    .b_valid(1'b0), .b_fill(1'b0), .b_vmid(4'h0), .b_src(48'h0), .b_dst(48'h0), .b_size(64'h0), .b_fillv(32'h0),
    .b_abort(1'b0), .b_done(), .b_fault(),
    .x_bulk(), .x_fill(), .x_src(), .x_size(), .x_fillv(), .x_abort(),
    .req_tog_o(x_req), .x_vmid(x_vmid), .x_host(x_host), .x_wr(x_wr), .x_addr(x_addr), .x_wstrb(x_wstrb), .x_wdata(x_wdata),
    .ack_tog_i(x_ack), .y_err(y_err), .y_rdata(y_rdata));

  ng_rcp_memif_l2 u_ml2 (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in),
    .req_tog_i(x_req), .x_vmid(x_vmid), .x_host(x_host), .x_wr(x_wr), .x_addr(x_addr), .x_wstrb(x_wstrb),
    .x_wdata(x_wdata), .x_bulk(1'b0), .x_fill(1'b0), .x_src(48'h0), .x_size(64'h0),
    .x_fillv(32'h0), .x_abort(1'b0),
    .ack_tog_o(x_ack), .y_err(y_err), .y_rdata(y_rdata), .busy(ml_busy), .o_addr(o_addr),
    .c_valid(c1_valid), .c_ready(c1_ready), .c_rs_valid(c1_rs_valid), .c_rs_err(c1_rs_err), .c_rs_rdata(c1_rs_rdata),
    .h_valid(h_valid), .h_ready(h_ready), .h_rs_valid(h_rs_valid), .h_rs_err(h_rs_err), .h_rs_rdata(h_rs_rdata),
    .t_req_valid(t_req_valid), .t_req_ready(t_req_ready), .t_req_vmid(t_req_vmid), .t_req_va(t_req_va),
    .t_req_acc(t_req_acc), .t_resp_valid(t_resp_valid), .t_resp_ready(t_resp_ready), .t_resp_fault(t_resp_fault),
    .t_resp_sparse(t_resp_sparse), .t_resp_pa(t_resp_pa), .t_resp_level(t_resp_level), .t_resp_pte(t_resp_pte),
    .ef_valid(ef_valid), .ef_vmid(ef_vmid), .ef_va(ef_va), .ef_acc(ef_acc), .ef_code(ef_code), .ef_level(ef_level),
    .ef_pte(ef_pte),
    .d_h_awvalid(), .d_h_awready(1'b0), .d_h_awaddr(), .d_h_awlen(), .d_h_wvalid(), .d_h_wready(1'b0), .d_h_wdata(),
    .d_h_wstrb(), .d_h_wlast(), .d_h_bvalid(1'b0), .d_h_bready(), .d_h_bresp(2'b00), .d_h_arvalid(),
    .d_h_arready(1'b0), .d_h_araddr(), .d_h_arlen(), .d_h_rvalid(1'b0), .d_h_rready(), .d_h_rdata('0),
    .d_h_rresp(2'b00), .d_h_rlast(1'b0), .d_c_awvalid(), .d_c_awready(1'b0), .d_c_awaddr(), .d_c_awlen(),
    .d_c_wvalid(), .d_c_wready(1'b0), .d_c_wdata(), .d_c_wstrb(), .d_c_wlast(), .d_c_bvalid(1'b0), .d_c_bready(),
    .d_c_bresp(2'b00), .d_c_arvalid(), .d_c_arready(1'b0), .d_c_araddr(), .d_c_arlen(), .d_c_rvalid(1'b0),
    .d_c_rready(), .d_c_rdata('0), .d_c_rresp(2'b00), .d_c_rlast(1'b0));

  // ------------------------------------------------------------------ BAR2 (C3 window) + C3 engine
  localparam int C3_AW = M_AXI_C3_ADDR_W;
  localparam int C3_DW = M_AXI_C3_DATA_W;
  logic        b2_valid, b2_ready, b2_wr, b2_rs_valid, b2_rs_err, b2_busy;
  logic [C3_AW-1:0] b2_addr;
  logic [7:0]  b2_wstrb;
  logic [63:0] b2_wdata, b2_rs_rdata;
  logic [2:0]  e_valid, e_ready, e_rs_valid;
  logic        e_rs_err;
  logic [63:0] e_rs_rdata;

  ng_rcp_bar2 #(.AW(C3_AW)) u_bar2 (
    .clk(clk), .rst_n(rst_n), .win_en(win_en), .win_log2(win_log2), .win_base(win_base),
    .s_awid(s_axi_bar2_awid), .s_awaddr(s_axi_bar2_awaddr), .s_awlen(s_axi_bar2_awlen), .s_awsize(s_axi_bar2_awsize),
    .s_awvalid(s_axi_bar2_awvalid), .s_awready(s_axi_bar2_awready),
    .s_wdata(s_axi_bar2_wdata), .s_wstrb(s_axi_bar2_wstrb), .s_wlast(s_axi_bar2_wlast),
    .s_wvalid(s_axi_bar2_wvalid), .s_wready(s_axi_bar2_wready),
    .s_bid(s_axi_bar2_bid), .s_bresp(s_axi_bar2_bresp), .s_bvalid(s_axi_bar2_bvalid), .s_bready(s_axi_bar2_bready),
    .s_arid(s_axi_bar2_arid), .s_araddr(s_axi_bar2_araddr), .s_arlen(s_axi_bar2_arlen), .s_arsize(s_axi_bar2_arsize),
    .s_arvalid(s_axi_bar2_arvalid), .s_arready(s_axi_bar2_arready),
    .s_rid(s_axi_bar2_rid), .s_rdata(s_axi_bar2_rdata), .s_rresp(s_axi_bar2_rresp), .s_rlast(s_axi_bar2_rlast),
    .s_rvalid(s_axi_bar2_rvalid), .s_rready(s_axi_bar2_rready),
    .op_valid(b2_valid), .op_ready(b2_ready), .op_wr(b2_wr), .op_addr(b2_addr), .op_wstrb(b2_wstrb),
    .op_wdata(b2_wdata), .rs_valid(b2_rs_valid), .rs_err(b2_rs_err), .rs_rdata(b2_rs_rdata), .busy(b2_busy));

  // requester 1 = the CP; its address comes from the clk_ts side (quasi-static while the handshake is open)
  assign b2_rs_valid = e_rs_valid[0];
  assign c1_rs_valid = e_rs_valid[1];
  assign b2_rs_err = e_rs_err;   assign b2_rs_rdata = e_rs_rdata;
  assign c1_rs_err = e_rs_err;   assign c1_rs_rdata = e_rs_rdata;
  assign c1_ready  = e_ready[1];
  assign b2_ready  = e_ready[0];
  assign c2_ready  = e_ready[2];
  assign c2_rs_valid = e_rs_valid[2];
  assign e_valid   = {c2_valid, c1_valid, b2_valid};

  ng_rcp_c3eng #(.DW(C3_DW), .AW(C3_AW)) u_c3 (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in), .mem_ready(shell_mem_ready[2]), .idle(c3_idle),
    .op_valid(e_valid), .op_ready(e_ready), .op_wr({x_wr, b2_wr}),
    .op_addr0(b2_addr), .op_addr1(o_addr[C3_AW-1:0]), .op_addr2(c2_addr[C3_AW-1:0]),
    .op_wstrb0(b2_wstrb), .op_wstrb1(x_wstrb), .op_wdata0(b2_wdata), .op_wdata1(x_wdata),
    .rs_valid(e_rs_valid), .rs_err(e_rs_err), .rs_rdata(e_rs_rdata),
    .m_awaddr(m_axi_c3_awaddr), .m_awlen(m_axi_c3_awlen), .m_awvalid(m_axi_c3_awvalid), .m_awready(m_axi_c3_awready),
    .m_wdata(m_axi_c3_wdata), .m_wstrb(m_axi_c3_wstrb), .m_wlast(m_axi_c3_wlast), .m_wvalid(m_axi_c3_wvalid),
    .m_wready(m_axi_c3_wready), .m_bresp(m_axi_c3_bresp), .m_bvalid(m_axi_c3_bvalid), .m_bready(m_axi_c3_bready),
    .m_araddr(m_axi_c3_araddr), .m_arlen(m_axi_c3_arlen), .m_arvalid(m_axi_c3_arvalid), .m_arready(m_axi_c3_arready),
    .m_rdata(m_axi_c3_rdata), .m_rresp(m_axi_c3_rresp), .m_rlast(m_axi_c3_rlast), .m_rvalid(m_axi_c3_rvalid),
    .m_rready(m_axi_c3_rready),
    .b_awvalid('0), .b_awready(), .b_awaddr('0), .b_awlen('0), .b_wvalid('0), .b_wready(), .b_wdata('0),
    .b_wstrb('0), .b_wlast('0), .b_bvalid(), .b_bready('0), .b_bresp(), .b_arvalid('0), .b_arready(), .b_araddr('0),
    .b_arlen('0), .b_rvalid(), .b_rready('0), .b_rdata(), .b_rresp(), .b_rlast());

  ng_mmu u_mmu (
    .clk(clk), .rst_n(rst_n),
    .csr_wr(csr_wr), .csr_waddr(csr_waddr), .csr_wdata(csr_wdata), .csr_raddr(csr_raddr), .csr_rdata(csr_rdata),
    .irq_fault(mmu_irq),
    .req_valid(t_req_valid), .req_ready(t_req_ready), .req_vmid(t_req_vmid), .req_va(t_req_va), .req_acc(t_req_acc),
    .req_atomic(1'b0), .req_tag(8'h00),
    .resp_valid(t_resp_valid), .resp_ready(t_resp_ready), .resp_fault(t_resp_fault), .resp_sparse(t_resp_sparse),
    .resp_pa(t_resp_pa), .resp_size(t_resp_size), .resp_level(t_resp_level), .resp_pte(t_resp_pte),
    .resp_tag(t_resp_tag),
    .ext_flt_valid(ef_valid), .ext_flt_vmid(ef_vmid), .ext_flt_va(ef_va), .ext_flt_acc(ef_acc),
    .ext_flt_code(ef_code), .ext_flt_level(ef_level), .ext_flt_pte(ef_pte),
    .rd_valid(mm_rd_valid), .rd_ready(mm_rd_ready), .rd_addr(mm_rd_addr),
    .rsp_valid(mm_rsp_valid), .rsp_data(mm_rsp_data), .rsp_err(mm_rsp_err),
    .stat_walks(mm_walks), .stat_tlb_hits(mm_hits));

  // page-table reads -> C3 engine requester 2
  ng_rcp_mmurd u_mmurd (
    .clk(clk), .rst_n(rst_n),
    .rd_valid(mm_rd_valid), .rd_ready(mm_rd_ready), .rd_addr(mm_rd_addr),
    .rsp_valid(mm_rsp_valid), .rsp_data(mm_rsp_data), .rsp_err(mm_rsp_err),
    .op_valid(c2_valid), .op_ready(c2_ready), .op_addr(c2_addr),
    .rs_valid(c2_rs_valid), .rs_err(e_rs_err), .rs_rdata(e_rs_rdata));

  // MMU fault interrupt -> CP register block (clk_ts), IRQ_STATUS bit 2
  ng_rcp_irq_l2ts u_mmu_irq (
    .clk_l2(clk), .rst_l2_n(rst_n), .pulse_l2(mmu_irq), .clk_ts(clk_ts), .rst_ts_n(rst_ts_n), .pulse_ts(mmu_irq_ts));

  assign m_axi_c3_awid = '0; assign m_axi_c3_awsize = 3'($clog2(C3_DW / 8)); assign m_axi_c3_awburst = 2'b01;
  assign m_axi_c3_awcache = 4'b0011; assign m_axi_c3_awprot = 3'b010; assign m_axi_c3_awqos = '0;
  assign m_axi_c3_arid = '0; assign m_axi_c3_arsize = 3'($clog2(C3_DW / 8)); assign m_axi_c3_arburst = 2'b01;
  assign m_axi_c3_arcache = 4'b0011; assign m_axi_c3_arprot = 3'b010; assign m_axi_c3_arqos = '0;

  // ------------------------------------------------------------------ host engine
  ng_rcp_hosteng u_host (
    .clk(clk), .rst_n(rst_n), .quiesce(q_in), .idle(host_idle),
    .op_valid(h_valid), .op_ready(h_ready), .op_wr(x_wr), .op_addr(o_addr), .op_wstrb(x_wstrb), .op_wdata(x_wdata),
    .rs_valid(h_rs_valid), .rs_err(h_rs_err), .rs_rdata(h_rs_rdata),
    .m_awaddr(m_axi_host_awaddr), .m_awlen(m_axi_host_awlen), .m_awvalid(m_axi_host_awvalid),
    .m_awready(m_axi_host_awready), .m_wdata(m_axi_host_wdata), .m_wstrb(m_axi_host_wstrb),
    .m_wlast(m_axi_host_wlast), .m_wvalid(m_axi_host_wvalid), .m_wready(m_axi_host_wready),
    .m_bresp(m_axi_host_bresp), .m_bvalid(m_axi_host_bvalid), .m_bready(m_axi_host_bready),
    .m_araddr(m_axi_host_araddr), .m_arlen(m_axi_host_arlen), .m_arvalid(m_axi_host_arvalid),
    .m_arready(m_axi_host_arready), .m_rdata(m_axi_host_rdata), .m_rresp(m_axi_host_rresp),
    .m_rlast(m_axi_host_rlast), .m_rvalid(m_axi_host_rvalid), .m_rready(m_axi_host_rready),
    .b_awvalid('0), .b_awready(), .b_awaddr('0), .b_awlen('0), .b_wvalid('0), .b_wready(), .b_wdata('0),
    .b_wstrb('0), .b_wlast('0), .b_bvalid(), .b_bready('0), .b_bresp(), .b_arvalid('0), .b_arready(), .b_araddr('0),
    .b_arlen('0), .b_rvalid(), .b_rready('0), .b_rdata(), .b_rresp(), .b_rlast());
  assign m_axi_host_awid = '0; assign m_axi_host_awsize = 3'd3; assign m_axi_host_awburst = 2'b01;
  assign m_axi_host_awcache = 4'b0011; assign m_axi_host_awprot = 3'b010; assign m_axi_host_awqos = '0;
  assign m_axi_host_arid = '0; assign m_axi_host_arsize = 3'd3; assign m_axi_host_arburst = 2'b01;
  assign m_axi_host_arcache = 4'b0011; assign m_axi_host_arprot = 3'b010; assign m_axi_host_arqos = '0;

  // ------------------------------------------------------------------ unused channels C1, C2, C4: never issue
  `define NG_RCP_TIE(p) \
    assign m_axi_``p``_awid = '0; assign m_axi_``p``_awaddr = '0; assign m_axi_``p``_awlen = '0; \
    assign m_axi_``p``_awsize = '0; assign m_axi_``p``_awburst = 2'b01; assign m_axi_``p``_awcache = 4'b0011; \
    assign m_axi_``p``_awprot = 3'b010; assign m_axi_``p``_awqos = '0; assign m_axi_``p``_awvalid = 1'b0; \
    assign m_axi_``p``_wdata = '0; assign m_axi_``p``_wstrb = '0; assign m_axi_``p``_wlast = 1'b0; \
    assign m_axi_``p``_wvalid = 1'b0; assign m_axi_``p``_bready = 1'b1; \
    assign m_axi_``p``_arid = '0; assign m_axi_``p``_araddr = '0; assign m_axi_``p``_arlen = '0; \
    assign m_axi_``p``_arsize = '0; assign m_axi_``p``_arburst = 2'b01; assign m_axi_``p``_arcache = 4'b0011; \
    assign m_axi_``p``_arprot = 3'b010; assign m_axi_``p``_arqos = '0; assign m_axi_``p``_arvalid = 1'b0; \
    assign m_axi_``p``_rready = 1'b1;
  `NG_RCP_TIE(c1)
  `NG_RCP_TIE(c2)
  `NG_RCP_TIE(c4)
  `undef NG_RCP_TIE

  // ----------------------------------------------------------------
  assign dbg_hub_tdo = 1'b0;
  logic [31:0] obs_sr;
  always_ff @(posedge dbg_obs_tck) begin
    if (dbg_obs_sel && dbg_obs_capture) obs_sr <= NG_RM_INFO_MAGIC;
    else if (dbg_obs_sel && dbg_obs_shift) obs_sr <= {dbg_obs_tdi, obs_sr[31:1]};
  end
  always_ff @(negedge dbg_obs_tck) dbg_obs_tdo <= obs_sr[0];

  logic unused;
  assign unused = &{1'b0, t_resp_size, t_resp_tag, mm_walks, mm_hits, ml_busy, b2_busy, dbg_valid, dbg_code, cpu_quiet, core_reset, wd_req,
                    s_axil_ctrl_awprot, s_axil_ctrl_arprot, s_axi_bar2_awburst, s_axi_bar2_awcache,
                    s_axi_bar2_awprot, s_axi_bar2_awqos, s_axi_bar2_arburst, s_axi_bar2_arcache,
                    s_axi_bar2_arprot, s_axi_bar2_arqos, clk_core, clk_core2x, rst_core_n, rst_ts_n,
                    m_axi_c1_awready, m_axi_c1_wready, m_axi_c1_bid, m_axi_c1_bresp, m_axi_c1_bvalid,
                    m_axi_c1_arready, m_axi_c1_rid, m_axi_c1_rdata, m_axi_c1_rresp, m_axi_c1_rlast, m_axi_c1_rvalid,
                    m_axi_c2_awready, m_axi_c2_wready, m_axi_c2_bid, m_axi_c2_bresp, m_axi_c2_bvalid,
                    m_axi_c2_arready, m_axi_c2_rid, m_axi_c2_rdata, m_axi_c2_rresp, m_axi_c2_rlast, m_axi_c2_rvalid,
                    m_axi_c4_awready, m_axi_c4_wready, m_axi_c4_bid, m_axi_c4_bresp, m_axi_c4_bvalid,
                    m_axi_c4_arready, m_axi_c4_rid, m_axi_c4_rdata, m_axi_c4_rresp, m_axi_c4_rlast, m_axi_c4_rvalid,
                    m_axi_c3_bid, m_axi_c3_rid, m_axi_host_bid, m_axi_host_rid, shell_mem_ready[3], shell_mem_ready[1:0],
                    dbg_hub_drck, dbg_hub_tck, dbg_hub_tms, dbg_hub_tdi, dbg_hub_sel, dbg_hub_shift,
                    dbg_hub_capture, dbg_hub_update, dbg_hub_reset, dbg_hub_runtest, dbg_hub_bscanid_en,
                    dbg_obs_update};
endmodule
