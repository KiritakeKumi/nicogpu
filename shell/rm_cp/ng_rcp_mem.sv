// ng_rcp_mem.sv -- memory side of the CP RM (MMU integration): CP memory port -> MMU / C3 / host, BAR2,
// CP and the MMU walker sharing the C3 port.
//
//   ng_rcp_memif_ts   clk_ts: takes the CP request (ng_cp_pkg::mem_req_t, one outstanding).  VMID 0 (physical): decodes
//                     the address (SYS -> C3, HOST -> host memory, everything else -> fault) exactly as before.
//                     VMID != 0: the full 48-bit CP address is a VA and is passed on untouched together with the VMID
//                     (only an address that is not 8-byte aligned faults here).  Hands the request to the clk_l2 side
//                     with a toggle handshake; the data bundle (vmid, host, wr, addr, wstrb, wdata) is quasi-static.
//   ng_rcp_memif_l2   clk_l2: VMID != 0 first goes to ng_mmu (translation client port t_*), the response (pa / fault /
//                     sparse) is then executed like a physical request: SYS -> C3 engine (requester 1), HOST -> host
//                     engine, anything else -> BAD_PHYS_ADDR reported through the MMU's ext_flt_* port and a faulting
//                     response.  A translation fault is already latched by the MMU; the CP gets m_rsp_fault + data 0.
//                     A sparse-null hit answers a read with 0 and drops a write (no error).
//   ng_rcp_c3eng      one 64-bit operation at a time on the m_axi_c3 port (128-bit beats, single beat, lane select),
//                     round-robin between requester 0 (BAR2 adapter), 1 (CP) and 2 (MMU page-table reads, read only)
//   ng_rcp_hosteng    one 64-bit operation at a time on the m_axi_host port
// bulk path: a whole COPY / FILL from the CP Copy engine (ng_cp_copy BULK) crosses with the same toggle
//                     handshake (x_bulk + quasi-static command, x_abort as a separately synchronised level);
//                     ng_rcp_memif_l2 runs it on ng_rcp_dma, which owns the MMU client port and the fault report
//                     while it runs and issues AXI bursts on its own host / C3 master ports (d_h_* / d_c_*, muxed
//                     into the engines' ports by ng_rcp_axmux).
// Operation interface (clk_l2): op_valid/op_ready, op_wr, op_addr (byte address, 8-byte aligned), op_wstrb,
// op_wdata; rs_valid is a single-cycle pulse with rs_err / rs_rdata.  All boundary-facing AXI signals go through
// ng_rmt_port (registered boundary stage with quiesce), as in the test RM.
//
// one CP request and one translation at a time (no hit-under-miss, no prefetch); the MMU's
// misaligned-access fault (MISALIGNED) is not generated: the CP issues naturally aligned 64-bit lanes, an unaligned
// request still faults in ng_rcp_memif_ts without a fault-record entry.  A request that is held off by quiesce before
// the translation starts, or a translation whose table read is held off, simply resumes after quiesce is released.

module ng_rcp_memif_ts
  import ng_cp_pkg::*;
(
  input  logic        clk,            // clk_ts
  input  logic        rst_n,
  // CP side
  input  logic        m_req_valid,
  output logic        m_req_ready,
  input  mem_req_t    m_req,
  output logic        m_rsp_valid,
  output logic        m_rsp_fault,
  output logic [63:0] m_rsp_data,
  // CP Copy engine, whole commands (BULK; tie b_valid to 0 without it)
  input  logic        b_valid,
  input  logic        b_fill,
  input  logic [3:0]  b_vmid,
  input  logic [47:0] b_src,
  input  logic [47:0] b_dst,
  input  logic [63:0] b_size,
  input  logic [31:0] b_fillv,
  input  logic        b_abort,
  output logic        b_done,
  output logic        b_fault,
  // to the clk_l2 side (quasi-static while the handshake is open)
  output logic        req_tog_o,
  output logic        x_bulk,
  output logic        x_fill,
  output logic [47:0] x_src,
  output logic [63:0] x_size,
  output logic [31:0] x_fillv,
  output logic        x_abort,        // level (b_abort registered), synchronised on the clk_l2 side
  output logic [3:0]  x_vmid,
  output logic        x_host,
  output logic        x_wr,
  output logic [47:0] x_addr,         // VMID 0: [46:0] = C3 offset / host address, [47] = 0; VMID != 0: the VA
  output logic [7:0]  x_wstrb,
  output logic [63:0] x_wdata,
  input  logic        ack_tog_i,      // asynchronous
  input  logic        y_err,
  input  logic [63:0] y_rdata
);
  localparam logic [47:0] SYS_BASE = 48'h0010_0000_0000;
  localparam logic [47:0] SYS_END  = 48'h0010_8000_0000;     // 2 GiB
  localparam logic [47:0] HOST_BASE = 48'h8000_0000_0000;

  (* ASYNC_REG = "TRUE" *) logic a1, a2;
  typedef enum logic [1:0] {S_IDLE, S_WAIT, S_FAULT} st_t;
  st_t st;

  // a bulk command and a single request never compete: the CP port issues no MEM_* access while the Copy engine
  // is busy, and in BULK mode the Copy engine itself uses only b_*
  assign m_req_ready = (st == S_IDLE) && !b_valid;

  wire        to_sys  = (m_req.addr >= SYS_BASE) && (m_req.addr < SYS_END);
  wire        to_host = (m_req.addr >= HOST_BASE);
  wire        virt    = (m_req.vmid != 4'd0);
  wire        ok      = (virt || to_sys || to_host) && (m_req.addr[2:0] == 3'b000);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; a1 <= 1'b0; a2 <= 1'b0; req_tog_o <= 1'b0;
      x_vmid <= '0; x_host <= 1'b0; x_wr <= 1'b0; x_addr <= '0; x_wstrb <= '0; x_wdata <= '0;
      x_bulk <= 1'b0; x_fill <= 1'b0; x_src <= '0; x_size <= '0; x_fillv <= '0; x_abort <= 1'b0;
      m_rsp_valid <= 1'b0; m_rsp_fault <= 1'b0; m_rsp_data <= '0; b_done <= 1'b0; b_fault <= 1'b0;
    end else begin
      a1 <= ack_tog_i; a2 <= a1;
      m_rsp_valid <= 1'b0;
      b_done <= 1'b0;
      x_abort <= b_abort;
      unique case (st)
        S_IDLE: if (b_valid && !b_done) begin                // b_valid falls the cycle after b_done
          x_bulk <= 1'b1; x_fill <= b_fill; x_vmid <= b_vmid; x_src <= b_src; x_addr <= b_dst; x_size <= b_size;
          x_fillv <= b_fillv;
          req_tog_o <= ~req_tog_o;
          st <= S_WAIT;
        end else if (m_req_valid) begin
          x_bulk <= 1'b0;
          if (ok) begin
            x_vmid  <= m_req.vmid;
            x_host  <= to_host && !virt;
            x_wr    <= m_req.write;
            x_addr  <= virt ? m_req.addr : to_host ? {1'b0, m_req.addr[46:0]} : 48'(m_req.addr - SYS_BASE);
            x_wstrb <= m_req.wstrb;
            x_wdata <= m_req.wdata;
            req_tog_o <= ~req_tog_o;
            st <= S_WAIT;
          end else begin
            st <= S_FAULT;
          end
        end
        S_WAIT: if (a2 == req_tog_o) begin
          if (x_bulk) begin b_done <= 1'b1; b_fault <= y_err; end
          else begin m_rsp_valid <= 1'b1; m_rsp_fault <= y_err; m_rsp_data <= y_rdata; end
          st <= S_IDLE;
        end
        default: begin       // S_FAULT: unmapped physical address / misaligned
          m_rsp_valid <= 1'b1; m_rsp_fault <= 1'b1; m_rsp_data <= '0;
          st <= S_IDLE;
        end
      endcase
    end
  end
endmodule

module ng_rcp_memif_l2
  import ng_mmu_pkg::*;
(
  input  logic        clk,            // clk_l2
  input  logic        rst_n,
  input  logic        quiesce,        // no new operation while 1
  input  logic        req_tog_i,      // asynchronous
  input  logic [3:0]  x_vmid,
  input  logic        x_host,
  input  logic        x_wr,
  input  logic [47:0] x_addr,
  input  logic [7:0]  x_wstrb,
  input  logic [63:0] x_wdata,
  input  logic        x_bulk,
  input  logic        x_fill,
  input  logic [47:0] x_src,
  input  logic [63:0] x_size,
  input  logic [31:0] x_fillv,
  input  logic        x_abort,        // asynchronous level
  output logic        ack_tog_o,
  output logic        y_err,
  output logic [63:0] y_rdata,
  output logic        busy,
  // address of the physical operation (valid from the cycle c_valid / h_valid rises): C3 offset or host address
  output logic [46:0] o_addr,
  // C3 engine requester 1
  output logic        c_valid,
  input  logic        c_ready,
  input  logic        c_rs_valid,
  input  logic        c_rs_err,
  input  logic [63:0] c_rs_rdata,
  // host engine
  output logic        h_valid,
  input  logic        h_ready,
  input  logic        h_rs_valid,
  input  logic        h_rs_err,
  input  logic [63:0] h_rs_rdata,
  // ng_mmu translation client port (clk_l2) and its external fault report
  output logic        t_req_valid,
  input  logic        t_req_ready,
  output logic [3:0]  t_req_vmid,
  output logic [63:0] t_req_va,
  output logic [1:0]  t_req_acc,
  input  logic        t_resp_valid,
  output logic        t_resp_ready,
  input  logic [2:0]  t_resp_fault,
  input  logic        t_resp_sparse,
  input  logic [47:0] t_resp_pa,
  input  logic [1:0]  t_resp_level,
  input  logic [63:0] t_resp_pte,
  output logic        ef_valid,
  output logic [3:0]  ef_vmid,
  output logic [63:0] ef_va,
  output logic [1:0]  ef_acc,
  output logic [2:0]  ef_code,
  output logic [1:0]  ef_level,
  output logic [63:0] ef_pte,
  // ng_rcp_dma burst masters (to the second port of ng_rcp_hosteng / ng_rcp_c3eng)
  output logic          d_h_awvalid,
  input  logic          d_h_awready,
  output logic [46:0]   d_h_awaddr,
  output logic [7:0]    d_h_awlen,
  output logic          d_h_wvalid,
  input  logic          d_h_wready,
  output logic [63:0]   d_h_wdata,
  output logic [7:0]    d_h_wstrb,
  output logic          d_h_wlast,
  input  logic          d_h_bvalid,
  output logic          d_h_bready,
  input  logic [1:0]    d_h_bresp,
  output logic          d_h_arvalid,
  input  logic          d_h_arready,
  output logic [46:0]   d_h_araddr,
  output logic [7:0]    d_h_arlen,
  input  logic          d_h_rvalid,
  output logic          d_h_rready,
  input  logic [63:0]   d_h_rdata,
  input  logic [1:0]    d_h_rresp,
  input  logic          d_h_rlast,
  output logic          d_c_awvalid,
  input  logic          d_c_awready,
  output logic [30:0]   d_c_awaddr,
  output logic [7:0]    d_c_awlen,
  output logic          d_c_wvalid,
  input  logic          d_c_wready,
  output logic [127:0]   d_c_wdata,
  output logic [15:0]    d_c_wstrb,
  output logic          d_c_wlast,
  input  logic          d_c_bvalid,
  output logic          d_c_bready,
  input  logic [1:0]    d_c_bresp,
  output logic          d_c_arvalid,
  input  logic          d_c_arready,
  output logic [30:0]   d_c_araddr,
  output logic [7:0]    d_c_arlen,
  input  logic          d_c_rvalid,
  output logic          d_c_rready,
  input  logic [127:0]   d_c_rdata,
  input  logic [1:0]    d_c_rresp,
  input  logic          d_c_rlast
);
  (* ASYNC_REG = "TRUE" *) logic r1, r2;
  (* ASYNC_REG = "TRUE" *) logic ab1, ab2;
  logic r3, to_host;
  logic [3:0]  q_vmid;
  logic [47:0] q_va;
  logic        q_wr;
  logic [1:0]  ef_level_q;
  logic [63:0] ef_pte_q;
  typedef enum logic [2:0] {S_IDLE, S_XREQ, S_XRESP, S_ISSUE, S_WAIT, S_BULK} st_t;
  st_t st;

  logic        d_start, d_done, d_err, d_busy;
  logic        dt_req_valid, dt_resp_ready, def_valid;
  logic [3:0]  dt_req_vmid, def_vmid;
  logic [63:0] dt_req_va, def_va, def_pte;
  logic [1:0]  dt_req_acc, def_acc, def_level;
  logic [2:0]  def_code;
  logic        ef_valid_q;
  wire         bulk = (st == S_BULK);

  assign busy    = (st != S_IDLE);
  assign c_valid = (st == S_ISSUE) && !to_host;
  assign h_valid = (st == S_ISSUE) && to_host;

  // MMU client: acc = write ? ACC_W: ACC_R, no atomics, tag unused (one outstanding); the DMA's while S_BULK
  assign t_req_valid  = bulk ? dt_req_valid : (st == S_XREQ);
  assign t_req_vmid   = bulk ? dt_req_vmid : q_vmid;
  assign t_req_va     = bulk ? dt_req_va : {16'h0, q_va};
  assign t_req_acc    = bulk ? dt_req_acc : (q_wr ? ACC_W : ACC_R);
  assign t_resp_ready = bulk ? dt_resp_ready : (st == S_XRESP);

  // BAD_PHYS report: one registered pulse (ef_valid) with the stored fields; the DMA's while S_BULK
  assign ef_valid = bulk ? def_valid : ef_valid_q;
  assign ef_vmid  = bulk ? def_vmid : q_vmid;
  assign ef_va    = bulk ? def_va : {16'h0, q_va};
  assign ef_acc   = bulk ? def_acc : (q_wr ? ACC_W : ACC_R);
  assign ef_code  = bulk ? def_code : F_BAD_PHYS;
  assign ef_level = bulk ? def_level : ef_level_q;
  assign ef_pte   = bulk ? def_pte : ef_pte_q;

  ng_rcp_dma u_dma (
    .clk(clk), .rst_n(rst_n),
    .start(d_start), .op_fill(x_fill), .vmid(x_vmid), .src(x_src), .dst(x_addr), .size(x_size), .fill(x_fillv),
    .abort(ab2), .busy(d_busy), .done(d_done), .err(d_err),
    .t_req_valid(dt_req_valid), .t_req_ready(bulk && t_req_ready), .t_req_vmid(dt_req_vmid), .t_req_va(dt_req_va),
    .t_req_acc(dt_req_acc), .t_resp_valid(bulk && t_resp_valid), .t_resp_ready(dt_resp_ready),
    .t_resp_fault(t_resp_fault), .t_resp_sparse(t_resp_sparse), .t_resp_pa(t_resp_pa), .t_resp_level(t_resp_level),
    .t_resp_pte(t_resp_pte),
    .ef_valid(def_valid), .ef_vmid(def_vmid), .ef_va(def_va), .ef_acc(def_acc), .ef_code(def_code),
    .ef_level(def_level), .ef_pte(def_pte),
    .h_awvalid(d_h_awvalid), .h_awready(d_h_awready), .h_awaddr(d_h_awaddr), .h_awlen(d_h_awlen),
    .h_wvalid(d_h_wvalid), .h_wready(d_h_wready), .h_wdata(d_h_wdata), .h_wstrb(d_h_wstrb), .h_wlast(d_h_wlast),
    .h_bvalid(d_h_bvalid), .h_bready(d_h_bready), .h_bresp(d_h_bresp), .h_arvalid(d_h_arvalid),
    .h_arready(d_h_arready), .h_araddr(d_h_araddr), .h_arlen(d_h_arlen), .h_rvalid(d_h_rvalid),
    .h_rready(d_h_rready), .h_rdata(d_h_rdata), .h_rresp(d_h_rresp), .h_rlast(d_h_rlast),
    .c_awvalid(d_c_awvalid), .c_awready(d_c_awready), .c_awaddr(d_c_awaddr), .c_awlen(d_c_awlen),
    .c_wvalid(d_c_wvalid), .c_wready(d_c_wready), .c_wdata(d_c_wdata), .c_wstrb(d_c_wstrb), .c_wlast(d_c_wlast),
    .c_bvalid(d_c_bvalid), .c_bready(d_c_bready), .c_bresp(d_c_bresp), .c_arvalid(d_c_arvalid),
    .c_arready(d_c_arready), .c_araddr(d_c_araddr), .c_arlen(d_c_arlen), .c_rvalid(d_c_rvalid),
    .c_rready(d_c_rready), .c_rdata(d_c_rdata), .c_rresp(d_c_rresp), .c_rlast(d_c_rlast));

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; r1 <= 1'b0; r2 <= 1'b0; r3 <= 1'b0; ack_tog_o <= 1'b0; to_host <= 1'b0;
      y_err <= 1'b0; y_rdata <= '0; o_addr <= '0;
      q_vmid <= '0; q_va <= '0; q_wr <= 1'b0; ef_valid_q <= 1'b0; ef_level_q <= '0; ef_pte_q <= '0;
      ab1 <= 1'b0; ab2 <= 1'b0; d_start <= 1'b0;
    end else begin
      r1 <= req_tog_i; r2 <= r1; r3 <= r2;
      ab1 <= x_abort; ab2 <= ab1;
      ef_valid_q <= 1'b0;
      d_start <= 1'b0;
      unique case (st)
        S_IDLE: if ((r2 != ack_tog_o) && !quiesce) begin    // a request is open (req != ack)
          if (x_bulk) begin                                 // whole COPY / FILL: the DMA (bursts stall in quiesce)
            d_start <= 1'b1;
            st <= S_BULK;
          end else if (x_vmid == 4'd0) begin                // physical: as before
            to_host <= x_host;
            o_addr  <= x_addr[46:0];
            st <= S_ISSUE;
          end else begin                                    // virtual: translate first
            q_vmid <= x_vmid;
            q_va   <= x_addr;
            q_wr   <= x_wr;
            st <= S_XREQ;
          end
        end
        S_XREQ: if (t_req_ready) st <= S_XRESP;
        S_XRESP: if (t_resp_valid) begin
          if (t_resp_fault != F_NONE) begin                 // already latched in the MMU fault registers
            y_err <= 1'b1; y_rdata <= '0; ack_tog_o <= r2; st <= S_IDLE;
          end else if (t_resp_sparse) begin                 // sparse-null: read 0, write dropped, no error
            y_err <= 1'b0; y_rdata <= '0; ack_tog_o <= r2; st <= S_IDLE;
          end else if (t_resp_pa[47:31] == SYS_PA_HI) begin // SYS (2 GiB at 0x10_0000_0000) -> C3
            to_host <= 1'b0; o_addr <= {16'h0, t_resp_pa[30:0]}; st <= S_ISSUE;
          end else if (t_resp_pa[47]) begin                 // host aperture
            to_host <= 1'b1; o_addr <= t_resp_pa[46:0]; st <= S_ISSUE;
          end else begin                                    // VRAM / hole: not reachable in this RM
            ef_valid_q <= 1'b1; ef_level_q <= t_resp_level; ef_pte_q <= t_resp_pte;
            y_err <= 1'b1; y_rdata <= '0; ack_tog_o <= r2; st <= S_IDLE;
          end
        end
        S_ISSUE: if (to_host ? h_ready : c_ready) st <= S_WAIT;
        S_BULK: if (d_done) begin
          y_err <= d_err; y_rdata <= '0; ack_tog_o <= r2; st <= S_IDLE;
        end
        default: begin
          if (to_host ? h_rs_valid : c_rs_valid) begin
            y_err   <= to_host ? h_rs_err : c_rs_err;
            y_rdata <= to_host ? h_rs_rdata : c_rs_rdata;
            ack_tog_o <= r2;
            st <= S_IDLE;
          end
        end
      endcase
    end
  end

endmodule

// ----------------------------------------------------------------------------------------------------------------
module ng_rcp_c3eng #(
  parameter int DW = 128,
  parameter int AW = 31
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          quiesce,
  input  logic          mem_ready,
  output logic          idle,
  // requester 0 (BAR2), 1 (CP) and 2 (MMU page-table reads: always a 64-bit read): op interface
  input  logic [2:0]    op_valid,
  output logic [2:0]    op_ready,
  input  logic [1:0]    op_wr,              // requesters 0 and 1 (requester 2 never writes)
  input  logic [AW-1:0] op_addr0,
  input  logic [AW-1:0] op_addr1,
  input  logic [AW-1:0] op_addr2,
  input  logic [7:0]    op_wstrb0,
  input  logic [7:0]    op_wstrb1,
  input  logic [63:0]   op_wdata0,
  input  logic [63:0]   op_wdata1,
  output logic [2:0]    rs_valid,
  output logic          rs_err,
  output logic [63:0]   rs_rdata,
  // m_axi_c3 (boundary stage inside)
  output logic [AW-1:0]   m_awaddr,
  output logic [7:0]      m_awlen,
  output logic            m_awvalid,
  input  logic            m_awready,
  output logic [DW-1:0]   m_wdata,
  output logic [DW/8-1:0] m_wstrb,
  output logic            m_wlast,
  output logic            m_wvalid,
  input  logic            m_wready,
  input  logic [1:0]      m_bresp,
  input  logic            m_bvalid,
  output logic            m_bready,
  output logic [AW-1:0]   m_araddr,
  output logic [7:0]      m_arlen,
  output logic            m_arvalid,
  input  logic            m_arready,
  input  logic [DW-1:0]   m_rdata,
  input  logic [1:0]      m_rresp,
  input  logic            m_rlast,
  input  logic            m_rvalid,
  output logic            m_rready,
  // second master: ng_rcp_dma bursts, muxed with the engine's own operations (ng_rcp_axmux)
  input  logic             b_awvalid,
  output logic             b_awready,
  input  logic [AW-1:0]    b_awaddr,
  input  logic [7:0]       b_awlen,
  input  logic             b_wvalid,
  output logic             b_wready,
  input  logic [DW-1:0]    b_wdata,
  input  logic [DW/8-1:0]  b_wstrb,
  input  logic             b_wlast,
  output logic             b_bvalid,
  input  logic             b_bready,
  output logic [1:0]       b_bresp,
  input  logic             b_arvalid,
  output logic             b_arready,
  input  logic [AW-1:0]    b_araddr,
  input  logic [7:0]       b_arlen,
  output logic             b_rvalid,
  input  logic             b_rready,
  output logic [DW-1:0]    b_rdata,
  output logic [1:0]       b_rresp,
  output logic             b_rlast
);
  localparam int LB = $clog2(DW / 8);          // beat address bits (4)
  typedef enum logic [2:0] {S_IDLE, S_ISSUE, S_BWAIT, S_RWAIT} st_t;
  st_t st;
  logic [1:0]  who, last_who;
  logic        wr, aw_done, w_done, ar_done;
  logic [AW-1:0] addr;
  logic [7:0]  wstrb;
  logic [63:0] wdata;

  logic p_awvalid, p_awready, p_wvalid, p_wready, p_arvalid, p_arready;
  logic p_bvalid, p_bready, p_rvalid, p_rready, p_rlast;
  logic [1:0] p_bresp, p_rresp;
  logic [DW-1:0] p_rdata;
  logic port_idle;

  wire [AW-1:0] a_al = {addr[AW-1:LB], {LB{1'b0}}};
  wire          hi   = addr[3];
  wire [DW-1:0] wd_beat = hi ? {wdata, 64'h0} : {64'h0, wdata};
  wire [DW/8-1:0] ws_beat = hi ? {wstrb, 8'h00} : {8'h00, wstrb};

  // an operation that has not reached the port yet (held off by quiesce) does not count: it simply waits
  assign idle = port_idle && ((st == S_IDLE) || (st == S_ISSUE && !aw_done && !w_done && !ar_done));
  // round-robin: the requester after the one served last has the highest priority
  logic [2:0] rq, gnt;
  assign rq = op_valid & {3{(st == S_IDLE) && !quiesce && mem_ready}};
  always_comb begin
    gnt = 3'b000;
    unique case (last_who)
      2'd0:    gnt = rq[1] ? 3'b010 : rq[2] ? 3'b100 : rq[0] ? 3'b001 : 3'b000;
      2'd1:    gnt = rq[2] ? 3'b100 : rq[0] ? 3'b001 : rq[1] ? 3'b010 : 3'b000;
      default: gnt = rq[0] ? 3'b001 : rq[1] ? 3'b010 : rq[2] ? 3'b100 : 3'b000;
    endcase
  end
  assign op_ready = gnt;

  assign p_awvalid = (st == S_ISSUE) && wr && !aw_done;
  assign p_wvalid  = (st == S_ISSUE) && wr && !w_done;
  assign p_arvalid = (st == S_ISSUE) && !wr && !ar_done;
  assign p_bready  = (st == S_BWAIT);
  assign p_rready  = (st == S_RWAIT);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; who <= 2'd0; last_who <= 2'd0; wr <= 1'b0; aw_done <= 1'b0; w_done <= 1'b0; ar_done <= 1'b0;
      rs_valid <= '0; rs_err <= 1'b0; rs_rdata <= '0; addr <= '0; wstrb <= '0; wdata <= '0;
    end else begin
      rs_valid <= '0;
      unique case (st)
        S_IDLE: begin
          aw_done <= 1'b0; w_done <= 1'b0; ar_done <= 1'b0;
          if (|gnt) begin
            who      <= gnt[2] ? 2'd2 : gnt[1] ? 2'd1 : 2'd0;
            last_who <= gnt[2] ? 2'd2 : gnt[1] ? 2'd1 : 2'd0;
            wr    <= gnt[2] ? 1'b0 : gnt[1] ? op_wr[1] : op_wr[0];
            addr  <= gnt[2] ? op_addr2 : gnt[1] ? op_addr1 : op_addr0;
            wstrb <= gnt[2] ? 8'h00 : gnt[1] ? op_wstrb1 : op_wstrb0;
            wdata <= gnt[2] ? 64'h0 : gnt[1] ? op_wdata1 : op_wdata0;
            st <= S_ISSUE;
          end
        end
        S_ISSUE: begin
          if (p_awvalid && p_awready) aw_done <= 1'b1;
          if (p_wvalid && p_wready)   w_done  <= 1'b1;
          if (p_arvalid && p_arready) ar_done <= 1'b1;
          if (wr ? ((aw_done || (p_awvalid && p_awready)) && (w_done || (p_wvalid && p_wready)))
                 : (ar_done || (p_arvalid && p_arready)))
            st <= wr ? S_BWAIT : S_RWAIT;
        end
        S_BWAIT: if (p_bvalid) begin
          rs_valid[who] <= 1'b1; rs_err <= (p_bresp != 2'b00); rs_rdata <= '0;
          st <= S_IDLE;
        end
        default: if (p_rvalid && p_rlast) begin       // S_RWAIT: single beat
          rs_valid[who] <= 1'b1; rs_err <= (p_rresp != 2'b00);
          rs_rdata <= hi ? p_rdata[127:64] : p_rdata[63:0];
          st <= S_IDLE;
        end
      endcase
    end
  end

  logic x_awvalid, x_awready, x_wvalid, x_wready, x_wlast, x_bvalid, x_bready, x_arvalid, x_arready;
  logic x_rvalid, x_rready, x_rlast;
  logic [AW-1:0] x_awaddr, x_araddr;
  logic [7:0] x_awlen, x_arlen;
  logic [DW-1:0] x_wdata, x_rdata;
  logic [DW/8-1:0] x_wstrb;
  logic [1:0] x_bresp, x_rresp;
  logic mux_idle;
  ng_rcp_axmux #(.DW(DW), .AW(AW), .DEPTH(16)) u_mux (
    .clk(clk), .rst_n(rst_n), .idle(mux_idle),
    .s0_awvalid(p_awvalid), .s0_awready(p_awready), .s0_awaddr(a_al), .s0_awlen(8'd0),
    .s0_wvalid(p_wvalid), .s0_wready(p_wready), .s0_wdata(wd_beat), .s0_wstrb(ws_beat), .s0_wlast(1'b1),
    .s0_bvalid(p_bvalid), .s0_bready(p_bready), .s0_bresp(p_bresp),
    .s0_arvalid(p_arvalid), .s0_arready(p_arready), .s0_araddr(a_al), .s0_arlen(8'd0),
    .s0_rvalid(p_rvalid), .s0_rready(p_rready), .s0_rdata(p_rdata), .s0_rresp(p_rresp), .s0_rlast(p_rlast),
    .s1_awvalid(b_awvalid), .s1_awready(b_awready), .s1_awaddr(b_awaddr), .s1_awlen(b_awlen), .s1_wvalid(b_wvalid),
    .s1_wready(b_wready), .s1_wdata(b_wdata), .s1_wstrb(b_wstrb), .s1_wlast(b_wlast), .s1_bvalid(b_bvalid),
    .s1_bready(b_bready), .s1_bresp(b_bresp), .s1_arvalid(b_arvalid), .s1_arready(b_arready), .s1_araddr(b_araddr),
    .s1_arlen(b_arlen), .s1_rvalid(b_rvalid), .s1_rready(b_rready), .s1_rdata(b_rdata), .s1_rresp(b_rresp),
    .s1_rlast(b_rlast),
    .m_awvalid(x_awvalid), .m_awready(x_awready), .m_awaddr(x_awaddr), .m_awlen(x_awlen), .m_wvalid(x_wvalid),
    .m_wready(x_wready), .m_wdata(x_wdata), .m_wstrb(x_wstrb), .m_wlast(x_wlast), .m_bvalid(x_bvalid),
    .m_bready(x_bready), .m_bresp(x_bresp), .m_arvalid(x_arvalid), .m_arready(x_arready), .m_araddr(x_araddr),
    .m_arlen(x_arlen), .m_rvalid(x_rvalid), .m_rready(x_rready), .m_rdata(x_rdata), .m_rresp(x_rresp),
    .m_rlast(x_rlast));

  ng_rmt_port #(.DW(DW), .AW(AW), .MAX_WR(16), .MAX_RD(16)) u_port (
    .clk(clk), .rst_n(rst_n), .quiesce(quiesce), .idle(port_idle),
    .s_awvalid(x_awvalid), .s_awready(x_awready), .s_awaddr(x_awaddr), .s_awlen(x_awlen), .s_wvalid(x_wvalid),
    .s_wready(x_wready), .s_wdata(x_wdata), .s_wstrb(x_wstrb), .s_wlast(x_wlast), .s_bvalid(x_bvalid),
    .s_bready(x_bready), .s_bresp(x_bresp), .s_arvalid(x_arvalid), .s_arready(x_arready), .s_araddr(x_araddr),
    .s_arlen(x_arlen), .s_rvalid(x_rvalid), .s_rready(x_rready), .s_rdata(x_rdata), .s_rresp(x_rresp),
    .s_rlast(x_rlast),
    .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid), .m_wready(m_wready),
    .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready));
endmodule

// ----------------------------------------------------------------------------------------------------------------
module ng_rcp_hosteng (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          quiesce,
  output logic          idle,
  input  logic          op_valid,
  output logic          op_ready,
  input  logic          op_wr,
  input  logic [46:0]   op_addr,
  input  logic [7:0]    op_wstrb,
  input  logic [63:0]   op_wdata,
  output logic          rs_valid,
  output logic          rs_err,
  output logic [63:0]   rs_rdata,
  output logic [46:0]   m_awaddr,
  output logic [7:0]    m_awlen,
  output logic          m_awvalid,
  input  logic          m_awready,
  output logic [63:0]   m_wdata,
  output logic [7:0]    m_wstrb,
  output logic          m_wlast,
  output logic          m_wvalid,
  input  logic          m_wready,
  input  logic [1:0]    m_bresp,
  input  logic          m_bvalid,
  output logic          m_bready,
  output logic [46:0]   m_araddr,
  output logic [7:0]    m_arlen,
  output logic          m_arvalid,
  input  logic          m_arready,
  input  logic [63:0]   m_rdata,
  input  logic [1:0]    m_rresp,
  input  logic          m_rlast,
  input  logic          m_rvalid,
  output logic          m_rready,
  // second master: ng_rcp_dma bursts, muxed with the engine's own operations (ng_rcp_axmux)
  input  logic             b_awvalid,
  output logic             b_awready,
  input  logic [47-1:0]    b_awaddr,
  input  logic [7:0]       b_awlen,
  input  logic             b_wvalid,
  output logic             b_wready,
  input  logic [64-1:0]    b_wdata,
  input  logic [64/8-1:0]  b_wstrb,
  input  logic             b_wlast,
  output logic             b_bvalid,
  input  logic             b_bready,
  output logic [1:0]       b_bresp,
  input  logic             b_arvalid,
  output logic             b_arready,
  input  logic [47-1:0]    b_araddr,
  input  logic [7:0]       b_arlen,
  output logic             b_rvalid,
  input  logic             b_rready,
  output logic [64-1:0]    b_rdata,
  output logic [1:0]       b_rresp,
  output logic             b_rlast
);
  typedef enum logic [2:0] {S_IDLE, S_ISSUE, S_BWAIT, S_RWAIT} st_t;
  st_t st;
  logic wr, aw_done, w_done, ar_done;
  logic flushing, werr;       // write flush: a read of the same address after the B response
  logic [46:0] addr;
  logic [7:0]  wstrb;
  logic [63:0] wdata;
  logic p_awvalid, p_awready, p_wvalid, p_wready, p_arvalid, p_arready;
  logic p_bvalid, p_bready, p_rvalid, p_rready, p_rlast;
  logic [1:0] p_bresp, p_rresp;
  logic [63:0] p_rdata;
  logic port_idle;

  // A request that the port has not taken yet does not count as busy (it resumes after quiesce is released, or goes away with
  // the RM).  That includes the flush read behind a write (flushing, aw_done / w_done still set from the write): without
  // the `flushing` term a quiesce that arrives between the B response and the acceptance of the flush read was never
  // acknowledged.
  assign idle     = port_idle && ((st == S_IDLE) ||
                                  (st == S_ISSUE && ((!aw_done && !w_done && !ar_done) || (flushing && !ar_done))));
  assign op_ready = (st == S_IDLE) && !quiesce;
  assign p_awvalid = (st == S_ISSUE) && wr && !aw_done;
  assign p_wvalid  = (st == S_ISSUE) && wr && !w_done;
  assign p_arvalid = (st == S_ISSUE) && !wr && !ar_done;
  assign p_bready  = (st == S_BWAIT);
  assign p_rready  = (st == S_RWAIT);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; wr <= 1'b0; aw_done <= 1'b0; w_done <= 1'b0; ar_done <= 1'b0; flushing <= 1'b0; werr <= 1'b0;
      rs_valid <= 1'b0; rs_err <= 1'b0; rs_rdata <= '0; addr <= '0; wstrb <= '0; wdata <= '0;
    end else begin
      rs_valid <= 1'b0;
      unique case (st)
        S_IDLE: begin
          aw_done <= 1'b0; w_done <= 1'b0; ar_done <= 1'b0;
          if (op_valid && op_ready) begin
            wr <= op_wr; addr <= op_addr; wstrb <= op_wstrb; wdata <= op_wdata;
            st <= S_ISSUE;
          end
        end
        S_ISSUE: begin
          if (p_awvalid && p_awready) aw_done <= 1'b1;
          if (p_wvalid && p_wready)   w_done  <= 1'b1;
          if (p_arvalid && p_arready) ar_done <= 1'b1;
          if (wr ? ((aw_done || (p_awvalid && p_awready)) && (w_done || (p_wvalid && p_wready)))
                 : (ar_done || (p_arvalid && p_arready)))
            st <= wr ? S_BWAIT : S_RWAIT;
        end
        // The B response of a posted PCIe write only says the bridge took it, while the MSI the CP raises next takes
        // another, shorter path to the host (board: 'fence MSI lost or late', the ISR read the old fence value).  So a
        // write is acknowledged to the CP after a read of the same address came back: a read request cannot pass
        // earlier posted writes, so when its completion returns the data is in host memory and the FENCE write /
        // IRQ_SET that follows cannot become visible before it.
        S_BWAIT: if (p_bvalid) begin
          werr <= (p_bresp != 2'b00);
          flushing <= 1'b1; wr <= 1'b0; ar_done <= 1'b0;
          st <= S_ISSUE;
        end
        default: if (p_rvalid && p_rlast) begin
          rs_valid <= 1'b1; rs_err <= (p_rresp != 2'b00) || (flushing && werr);
          rs_rdata <= flushing ? '0 : p_rdata;
          flushing <= 1'b0;
          st <= S_IDLE;
        end
      endcase
    end
  end

  logic x_awvalid, x_awready, x_wvalid, x_wready, x_wlast, x_bvalid, x_bready, x_arvalid, x_arready;
  logic x_rvalid, x_rready, x_rlast;
  logic [47-1:0] x_awaddr, x_araddr;
  logic [7:0] x_awlen, x_arlen;
  logic [64-1:0] x_wdata, x_rdata;
  logic [64/8-1:0] x_wstrb;
  logic [1:0] x_bresp, x_rresp;
  logic mux_idle;
  ng_rcp_axmux #(.DW(64), .AW(47), .DEPTH(16)) u_mux (
    .clk(clk), .rst_n(rst_n), .idle(mux_idle),
    .s0_awvalid(p_awvalid), .s0_awready(p_awready), .s0_awaddr(addr), .s0_awlen(8'd0),
    .s0_wvalid(p_wvalid), .s0_wready(p_wready), .s0_wdata(wdata), .s0_wstrb(wstrb), .s0_wlast(1'b1),
    .s0_bvalid(p_bvalid), .s0_bready(p_bready), .s0_bresp(p_bresp),
    .s0_arvalid(p_arvalid), .s0_arready(p_arready), .s0_araddr(addr), .s0_arlen(8'd0),
    .s0_rvalid(p_rvalid), .s0_rready(p_rready), .s0_rdata(p_rdata), .s0_rresp(p_rresp), .s0_rlast(p_rlast),
    .s1_awvalid(b_awvalid), .s1_awready(b_awready), .s1_awaddr(b_awaddr), .s1_awlen(b_awlen), .s1_wvalid(b_wvalid),
    .s1_wready(b_wready), .s1_wdata(b_wdata), .s1_wstrb(b_wstrb), .s1_wlast(b_wlast), .s1_bvalid(b_bvalid),
    .s1_bready(b_bready), .s1_bresp(b_bresp), .s1_arvalid(b_arvalid), .s1_arready(b_arready), .s1_araddr(b_araddr),
    .s1_arlen(b_arlen), .s1_rvalid(b_rvalid), .s1_rready(b_rready), .s1_rdata(b_rdata), .s1_rresp(b_rresp),
    .s1_rlast(b_rlast),
    .m_awvalid(x_awvalid), .m_awready(x_awready), .m_awaddr(x_awaddr), .m_awlen(x_awlen), .m_wvalid(x_wvalid),
    .m_wready(x_wready), .m_wdata(x_wdata), .m_wstrb(x_wstrb), .m_wlast(x_wlast), .m_bvalid(x_bvalid),
    .m_bready(x_bready), .m_bresp(x_bresp), .m_arvalid(x_arvalid), .m_arready(x_arready), .m_araddr(x_araddr),
    .m_arlen(x_arlen), .m_rvalid(x_rvalid), .m_rready(x_rready), .m_rdata(x_rdata), .m_rresp(x_rresp),
    .m_rlast(x_rlast));

  ng_rmt_port #(.DW(64), .AW(47), .MAX_WR(8), .MAX_RD(8)) u_port (
    .clk(clk), .rst_n(rst_n), .quiesce(quiesce), .idle(port_idle),
    .s_awvalid(x_awvalid), .s_awready(x_awready), .s_awaddr(x_awaddr), .s_awlen(x_awlen), .s_wvalid(x_wvalid),
    .s_wready(x_wready), .s_wdata(x_wdata), .s_wstrb(x_wstrb), .s_wlast(x_wlast), .s_bvalid(x_bvalid),
    .s_bready(x_bready), .s_bresp(x_bresp), .s_arvalid(x_arvalid), .s_arready(x_arready), .s_araddr(x_araddr),
    .s_arlen(x_arlen), .s_rvalid(x_rvalid), .s_rready(x_rready), .s_rdata(x_rdata), .s_rresp(x_rresp),
    .s_rlast(x_rlast),
    .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wlast(m_wlast), .m_wvalid(m_wvalid), .m_wready(m_wready),
    .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rlast(m_rlast), .m_rvalid(m_rvalid), .m_rready(m_rready));
endmodule
