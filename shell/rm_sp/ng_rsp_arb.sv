// ng_rsp_arb.sv -- clk_l2 arbiters of the M1 SP RM: two requesters (0 = CP, 1 = SP) share one resource.
//
//   ng_rsp_mmuarb   the ng_mmu translation client port (valid/ready request, valid/ready response, one translation in
//                   flight -- ng_mmu's own contract).  A grant is held from the request handshake until the response
//                   handshake, then the other requester has priority (round-robin): a requester that is waiting is
//                   served after at most one translation of the other one, so CP polling (WAIT_MEM on a VA) cannot
//                   starve the SP and vice versa.
//   ng_rsp_oparb    a 64-bit operation port (op_valid/op_ready, rs_valid pulse) of ng_rcp_c3eng (requester 1) or
//                   ng_rcp_hosteng.  Same rule: the grant is held until the owner's rs_valid, then round-robin.
// Both register the owner in a first cycle (A_IDLE -> A_REQ) so the muxed request fields come from a flop select.
// Counters (grants per requester) feed the RM debug page.
module ng_rsp_mmuarb (
  input  logic        clk,
  input  logic        rst_n,
  // requester 0 / 1
  input  logic [1:0]  req_valid,
  output logic [1:0]  req_ready,
  input  logic [3:0]  req_vmid   [2],
  input  logic [63:0] req_va     [2],
  input  logic [1:0]  req_acc    [2],
  input  logic [1:0]  req_atomic,
  output logic [1:0]  resp_valid,
  input  logic [1:0]  resp_ready,
  // ng_mmu client port
  output logic        m_req_valid,
  input  logic        m_req_ready,
  output logic [3:0]  m_req_vmid,
  output logic [63:0] m_req_va,
  output logic [1:0]  m_req_acc,
  output logic        m_req_atomic,
  input  logic        m_resp_valid,
  output logic        m_resp_ready,
  // debug
  output logic [1:0]  dbg_st,             // {busy, owner}
  output logic [31:0] grants [2]
);
  typedef enum logic [1:0] {A_IDLE, A_REQ, A_RSP} st_t;
  st_t  st;
  logic own, last;

  // round-robin pick: the requester that was not served last wins a tie
  wire pick = (req_valid == 2'b11) ? ~last : req_valid[1];

  assign m_req_valid  = (st == A_REQ) && req_valid[own];
  assign m_req_vmid   = req_vmid[own];
  assign m_req_va     = req_va[own];
  assign m_req_acc    = req_acc[own];
  assign m_req_atomic = req_atomic[own];
  assign req_ready    = (st == A_REQ) ? (2'(m_req_ready) << own) : 2'b00;
  assign resp_valid   = (st == A_RSP) ? (2'(m_resp_valid) << own) : 2'b00;
  assign m_resp_ready = (st == A_RSP) && resp_ready[own];
  assign dbg_st       = {st != A_IDLE, own};

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= A_IDLE; own <= 1'b0; last <= 1'b1; grants[0] <= '0; grants[1] <= '0;
    end else begin
      unique case (st)
        A_IDLE: if (|req_valid) begin
          own <= pick; st <= A_REQ;
          grants[pick] <= grants[pick] + 32'd1;
        end
        A_REQ: if (m_req_valid && m_req_ready) st <= A_RSP;
        default: if (m_resp_valid && m_resp_ready) begin last <= own; st <= A_IDLE; end
      endcase
    end
  end
endmodule

module ng_rsp_oparb #(
  parameter int AW = 47
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [1:0]    op_valid,
  output logic [1:0]    op_ready,
  input  logic [1:0]    op_wr,
  input  logic [AW-1:0] op_addr  [2],
  input  logic [7:0]    op_wstrb [2],
  input  logic [63:0]   op_wdata [2],
  output logic [1:0]    rs_valid,
  // engine side
  output logic          e_valid,
  input  logic          e_ready,
  output logic          e_wr,
  output logic [AW-1:0] e_addr,
  output logic [7:0]    e_wstrb,
  output logic [63:0]   e_wdata,
  input  logic          e_rs_valid,
  // debug
  output logic [1:0]    dbg_st,
  output logic [31:0]   grants [2]
);
  typedef enum logic [1:0] {A_IDLE, A_REQ, A_RSP} st_t;
  st_t  st;
  logic own, last;
  wire  pick = (op_valid == 2'b11) ? ~last : op_valid[1];

  assign e_valid  = (st == A_REQ) && op_valid[own];
  assign e_wr     = op_wr[own];
  assign e_addr   = op_addr[own];
  assign e_wstrb  = op_wstrb[own];
  assign e_wdata  = op_wdata[own];
  assign op_ready = (st == A_REQ) ? (2'(e_ready) << own) : 2'b00;
  assign rs_valid = (st == A_RSP) ? (2'(e_rs_valid) << own) : 2'b00;
  assign dbg_st   = {st != A_IDLE, own};

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= A_IDLE; own <= 1'b0; last <= 1'b1; grants[0] <= '0; grants[1] <= '0;
    end else begin
      unique case (st)
        A_IDLE: if (|op_valid) begin
          own <= pick; st <= A_REQ;
          grants[pick] <= grants[pick] + 32'd1;
        end
        A_REQ: if (e_valid && e_ready) st <= A_RSP;
        default: if (e_rs_valid) begin last <= own; st <= A_IDLE; end
      endcase
    end
  end
endmodule
