// ng_core_arb.sv -- ng_core arbiters (round robin, one transaction in flight per shared resource).
//
//   ng_core_mmuarb  N translation clients (the SPs' mmu_* ports) -> one ng_mmu client port.  The grant is held from
//                   the request handshake to the response handshake (ng_mmu has one translation in flight), then the
//                   next requester after the owner wins (round robin).  Same contract as rm_sp's ng_rsp_mmuarb
//                   generalised to N; the owner is registered first (A_IDLE -> A_REQ) so the request mux is driven
//                   from a flop select.
//   ng_core_ucarb   N uc bypass ports (ng_lsu_noc_adapter byp_*: mreq_t / mrsp_t, level request, response pulse >= 1
//                   cycle after the handshake) -> one uc port (SYS / HOST path: on the board the C3 / host engines).
//   ng_core_fltarb  N ext_flt pulses (the LSUs' BAD_PHYS_ADDR reports) -> one ng_mmu ext_flt port: one pending entry
//                   per requester, lowest index first; a report arriving while the requester's entry is still
//                   pending is dropped (the MMU latch keeps only the first fault + MULTI anyway).
// Requesters are expected to hold valid until ready (ng_sp_top's MMU / fabric arbiters only see rst_n, so an SP
// core_reset never withdraws a request in flight).
// ng_core_rrpick -- round robin: the first valid requester after `last` (`last` itself when none is valid)
module ng_core_rrpick #(
  parameter int N  = 4,
  parameter int OW = ng_noc_pkg::clog2_1(N)
) (
  input  logic [N-1:0]  valid,
  input  logic [OW-1:0] last,
  output logic [OW-1:0] pick
);
  always_comb begin
    pick = last;
    for (int k = N; k >= 1; k--) begin
      int c;
      c = (int'(last) + k) % N;
      if (valid[c]) pick = OW'(c);
    end
  end
endmodule

module ng_core_mmuarb #(
  parameter int N = 4
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [N-1:0]  req_valid,
  output logic [N-1:0]  req_ready,
  input  logic [3:0]    req_vmid   [N],
  input  logic [63:0]   req_va     [N],
  input  logic [1:0]    req_acc    [N],
  input  logic [N-1:0]  req_atomic,
  input  logic [7:0]    req_tag    [N],
  output logic [N-1:0]  resp_valid,
  input  logic [N-1:0]  resp_ready,
  // ng_mmu client port
  output logic          m_req_valid,
  input  logic          m_req_ready,
  output logic [3:0]    m_req_vmid,
  output logic [63:0]   m_req_va,
  output logic [1:0]    m_req_acc,
  output logic          m_req_atomic,
  output logic [7:0]    m_req_tag,
  input  logic          m_resp_valid,
  output logic          m_resp_ready,
  output logic          busy,
  output logic [31:0]   grants [N]
);
  localparam int OW = ng_noc_pkg::clog2_1(N);
  typedef enum logic [1:0] {A_IDLE, A_REQ, A_RSP} st_t;
  st_t          st;
  logic [OW-1:0] own, last, pick;

  // round robin: first valid requester after `last`
  ng_core_rrpick #(.N(N), .OW(OW)) u_pick (.valid(req_valid), .last(last), .pick(pick));

  assign busy         = (st != A_IDLE);
  assign m_req_valid  = (st == A_REQ) && req_valid[own];
  assign m_req_vmid   = req_vmid[own];
  assign m_req_va     = req_va[own];
  assign m_req_acc    = req_acc[own];
  assign m_req_atomic = req_atomic[own];
  assign m_req_tag    = req_tag[own];
  always_comb begin
    req_ready  = '0;
    resp_valid = '0;
    if (st == A_REQ) req_ready[own]  = m_req_ready;
    if (st == A_RSP) resp_valid[own] = m_resp_valid;
  end
  assign m_resp_ready = (st == A_RSP) && resp_ready[own];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= A_IDLE; own <= '0; last <= OW'(N - 1);
      for (int i = 0; i < N; i++) grants[i] <= '0;
    end else begin
      case (st)
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

module ng_core_ucarb
  import ng_lsu_pkg::mreq_t;
  import ng_lsu_pkg::mrsp_t;
#(
  parameter int N = 4
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [N-1:0]  req_valid,
  output logic [N-1:0]  req_ready,
  input  mreq_t         req       [N],
  output logic [N-1:0]  rsp_valid,
  output mrsp_t         rsp,
  // shared uc port
  output logic          m_req_valid,
  input  logic          m_req_ready,
  output mreq_t         m_req,
  input  logic          m_rsp_valid,
  input  mrsp_t         m_rsp,
  output logic          busy,
  output logic [31:0]   grants [N]
);
  localparam int OW = ng_noc_pkg::clog2_1(N);
  typedef enum logic [1:0] {A_IDLE, A_REQ, A_RSP} st_t;
  st_t           st;
  logic [OW-1:0] own, last, pick;

  ng_core_rrpick #(.N(N), .OW(OW)) u_pick (.valid(req_valid), .last(last), .pick(pick));

  assign busy        = (st != A_IDLE);
  assign m_req_valid = (st == A_REQ) && req_valid[own];
  assign m_req       = req[own];
  assign rsp         = m_rsp;
  always_comb begin
    req_ready = '0;
    rsp_valid = '0;
    if (st == A_REQ) req_ready[own] = m_req_ready;
    if (st == A_RSP) rsp_valid[own] = m_rsp_valid;
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= A_IDLE; own <= '0; last <= OW'(N - 1);
      for (int i = 0; i < N; i++) grants[i] <= '0;
    end else begin
      case (st)
        A_IDLE: if (|req_valid) begin
          own <= pick; st <= A_REQ;
          grants[pick] <= grants[pick] + 32'd1;
        end
        A_REQ: if (m_req_valid && m_req_ready) st <= A_RSP;
        default: if (m_rsp_valid) begin last <= own; st <= A_IDLE; end
      endcase
    end
  end
endmodule

module ng_core_fltarb #(
  parameter int N = 4
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [N-1:0]  in_valid,
  input  logic [3:0]    in_vmid [N],
  input  logic [63:0]   in_va   [N],
  input  logic [1:0]    in_acc  [N],
  output logic          out_valid,
  output logic [3:0]    out_vmid,
  output logic [63:0]   out_va,
  output logic [1:0]    out_acc,
  output logic [31:0]   dropped
);
  logic [N-1:0] pend;
  logic [3:0]   p_vmid [N];
  logic [63:0]  p_va   [N];
  logic [1:0]   p_acc  [N];
  logic         has;
  int           sel;
  always_comb begin
    has = |pend;
    sel = int'(ng_bitops_pkg::lowest_set(64'(pend)));
  end
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      pend <= '0; out_valid <= 1'b0; out_vmid <= '0; out_va <= '0; out_acc <= '0; dropped <= '0;
      for (int i = 0; i < N; i++) begin p_vmid[i] <= '0; p_va[i] <= '0; p_acc[i] <= '0; end
    end else begin
      out_valid <= has;
      if (has) begin
        out_vmid <= p_vmid[sel]; out_va <= p_va[sel]; out_acc <= p_acc[sel];
      end
      for (int i = 0; i < N; i++) begin
        if (has && sel == i) pend[i] <= 1'b0;
        if (in_valid[i]) begin
          if (!pend[i] || (has && sel == i)) begin
            pend[i] <= 1'b1; p_vmid[i] <= in_vmid[i]; p_va[i] <= in_va[i]; p_acc[i] <= in_acc[i];
          end else begin
            dropped <= dropped + 32'd1;
          end
        end
      end
    end
  end
endmodule
