// ng_lsu_noc_adapter.sv -- L1D line-granular memory port -> NoC flits
// -> L2 slices (and the uc bypass port), plus fan-out of the cache maintenance
// operations.  Fulfils the interface proposal
// tentative (3).
//
// Upstream: the `mem_*` port of ng_l1d (level valid/ready request, one
//             outstanding; a one-cycle `rsp_valid` pulse that always comes
//             >= 2 cycles after the request handshake).
// Downstream: the `ti_*` / `to_*` streams of ng_noc_ep (single-flit messages,
//             CLS_REQ out, CLS_RESP in) plus a second mreq/mrsp port `byp_*`
//             for uc (non-VRAM, SYS/HOST) requests that never enter the NoC.
//
// Translation (slice = ng_l2_addrmap::l2_slice_idx(pa), node = SLICE_NODES[slice]):
//   M_RD   16 x KIND_RD  SZ_8 flits (line base + 8k), tag = {TAG_HI, k}; the
//          responses may arrive in any order and are re-assembled by tag.
//   M_WR   one KIND_WR SZ_8 flit per 8 B chunk whose be byte is non-zero
//          (data/be = the matching 64 / 8 bits of wdata/be); all chunk acks must
//          be back before L1D is answered (write-through completes on L2 ack).
//          An all-zero be completes without any flit.
//   M_AMO  one KIND_AMO flit (op = amo_op, SZ_4/SZ_8 by amo64, rtn, data = amo_a,
//          cmp = amo_cmp); the response data goes to mrsp.old.
//   M_INV_L2 / M_WB_L2  one KIND_MAINT flit (MAINT_INV_L2 / MAINT_WB_L2) to
//          EVERY slice (tag = slice index); answered when all four slices ack.
//   M_INV_L1  is absorbed by ng_l1d and never reaches this port; if it does it is
//          acknowledged immediately (defensive).
//   uc     M_RD/M_WR/M_AMO with mreq.uc go to the bypass port unchanged.
//
// mrsp.err = the error of the lowest-numbered chunk (RD/WR) / slice (maint)
// whose response carried err != 0 (deterministic regardless of arrival order).
//
// one request in flight, flits are sent
// serially; to_ready is tied high (the response side never stalls the NoC; the
// only state it needs is the 1024-bit assembly buffer).  Throughput is not a
// goal of v1; a multi-outstanding / chunk-pipelined version is a later
// optimisation behind the same ports.
module ng_lsu_noc_adapter
  import ng_noc_pkg::*;
  import ng_l2_addrmap::l2_slice_idx;
  import ng_bitops_pkg::lowest_set;
  import ng_lsu_pkg::mreq_t, ng_lsu_pkg::mrsp_t, ng_lsu_pkg::LINE_BITS;
  import ng_lsu_pkg::M_RD, ng_lsu_pkg::M_WR, ng_lsu_pkg::M_AMO, ng_lsu_pkg::M_INV_L1,
         ng_lsu_pkg::M_INV_L2, ng_lsu_pkg::M_WB_L2;
#(
  parameter logic [5:0]  SRC_ID      = 6'd0,                      // this node's NoC id
  parameter logic [23:0] SLICE_NODES = {6'd5, 6'd4, 6'd2, 6'd1},  // slice i -> node [6i+:6]
  parameter logic [3:0]  TAG_HI      = 4'h0                       // tag[7:4] of every flit
) (
  input  logic     clk,
  input  logic     rst_n,

  // ---- upstream: ng_l1d mem port ----
  input  logic     req_valid,
  output logic     req_ready,
  input  mreq_t    req,
  output logic     rsp_valid,       // one-cycle pulse
  output mrsp_t    rsp,

  // ---- downstream: ng_noc_ep agent streams ----
  output logic     ti_valid,
  input  logic     ti_ready,
  output flit_t    ti_flit,
  input  logic     to_valid,
  output logic     to_ready,
  input  flit_t    to_flit,

  // ---- downstream: uc bypass (SYS / HOST path of the integration layer) ----
  output logic     byp_req_valid,
  input  logic     byp_req_ready,
  output mreq_t    byp_req,
  input  logic     byp_rsp_valid,   // pulse, >= 1 cycle after the request handshake
  input  mrsp_t    byp_rsp
);

  typedef enum logic [2:0] {S_IDLE, S_NOC, S_BYP_REQ, S_BYP_WAIT, S_RSP} st_t;
  st_t st;

  mreq_t          rq;
  logic [15:0]    nzmask;        // chunks / slices that get a flit
  logic [15:0]    pend_mask;     // chunks / slices whose flit is still to be sent
  logic [15:0]    got;           // responses seen, per chunk / slice
  logic [LINE_BITS-1:0]  rbuf;
  logic [63:0]    old_q;
  logic [1:0]     err_q;
  logic           err_vld;
  logic [3:0]     err_idx;
  mrsp_t          rs;

  // ---------------- helpers ------------------------------------------------
  function automatic logic [15:0] chunk_mask(input mreq_t r);
    logic [15:0] m;
    m = '0;
    case (r.kind)
      M_RD:               m = 16'hFFFF;
      M_WR:               for (int k = 0; k < 16; k++) m[k] = |r.be[8*k +: 8];
      M_AMO:              m = 16'h0001;
      M_INV_L2, M_WB_L2:  m = 16'h000F;          // one flit per slice
      default:            m = 16'h0000;          // M_INV_L1: nothing to send
    endcase
    return m;
  endfunction

  function automatic logic is_bypass(input mreq_t r);
    return r.uc && (r.kind == M_RD || r.kind == M_WR || r.kind == M_AMO);
  endfunction

  function automatic logic [5:0] node_of(input int slice);
    return SLICE_NODES[6*slice +: 6];
  endfunction

  function automatic flit_t mk_flit(input mreq_t r, input logic [3:0] k);
    flit_t f;
    f     = '0;
    f.cls = CLS_REQ;
    f.src = SRC_ID;
    f.tag = {TAG_HI, k};
    f.op  = AMO_NONE;
    f.dst = node_of(l2_slice_idx(r.pa));         // RD / WR / AMO: the line's slice
    case (r.kind)
      M_RD: begin
        f.kind = KIND_RD;
        f.size = SZ_8;
        f.addr = r.pa + 48'({k, 3'b000});
      end
      M_WR: begin
        f.kind = KIND_WR;
        f.size = SZ_8;
        f.addr = r.pa + 48'({k, 3'b000});
        f.data = r.wdata[64*int'(k) +: 64];
        f.be   = r.be[8*int'(k) +: 8];
      end
      M_AMO: begin
        f.kind = KIND_AMO;
        f.op   = r.amo_op;
        f.size = r.amo64 ? SZ_8 : SZ_4;
        f.rtn  = r.amo_rtn;
        f.addr = r.pa;
        f.data = r.amo_a;
        f.cmp  = r.amo_cmp;
      end
      default: begin                              // M_INV_L2 / M_WB_L2: k = slice
        f.kind = KIND_MAINT;
        f.op   = (r.kind == M_INV_L2) ? MAINT_INV_L2 : MAINT_WB_L2;
        f.dst  = node_of(int'(k[1:0]));
      end
    endcase
    return f;
  endfunction

  // ---------------- next flit to send: the lowest pending chunk / slice ----
  wire       has_nxt = |pend_mask;
  wire [3:0] nxt     = 4'(lowest_set({48'd0, pend_mask}));

  assign ti_valid = (st == S_NOC) && has_nxt;
  assign ti_flit  = mk_flit(rq, nxt);
  assign to_ready = 1'b1;                      // responses are always sunk

  assign req_ready     = (st == S_IDLE);
  assign rsp_valid     = (st == S_RSP);
  assign rsp           = rs;
  assign byp_req_valid = (st == S_BYP_REQ);
  assign byp_req       = rq;

  // ---------------- FSM ----------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st      <= S_IDLE;
      rq      <= '0;
      nzmask  <= '0;
      pend_mask <= '0;
      got     <= '0;
      rbuf    <= '0;
      old_q   <= '0;
      err_q   <= '0;
      err_vld <= 1'b0;
      err_idx <= '0;
      rs      <= '0;
    end else begin
      case (st)
        S_IDLE: begin
          if (req_valid) begin
            rq      <= req;
            nzmask  <= chunk_mask(req);
            pend_mask <= chunk_mask(req);
            got     <= '0;
            rbuf    <= '0;
            old_q   <= '0;
            err_q   <= '0;
            err_vld <= 1'b0;
            err_idx <= '0;
            st      <= is_bypass(req) ? S_BYP_REQ : S_NOC;
          end
        end

        S_NOC: begin
          if (ti_valid && ti_ready) pend_mask[nxt] <= 1'b0;
          if (to_valid) begin                  // response: any order
            got[to_flit.tag[3:0]] <= 1'b1;
            if (rq.kind == M_RD)  rbuf[64*int'(to_flit.tag[3:0]) +: 64] <= to_flit.data;
            if (rq.kind == M_AMO) old_q <= to_flit.data;
            if (to_flit.err != ERR_NONE &&
                (!err_vld || to_flit.tag[3:0] < err_idx)) begin
              err_vld <= 1'b1;
              err_idx <= to_flit.tag[3:0];
              err_q   <= to_flit.err;
            end
          end
          if (!has_nxt && (got == nzmask)) begin
            rs.rdata <= rbuf;
            rs.old   <= old_q;
            rs.err   <= err_q;
            st       <= S_RSP;
          end
        end

        S_BYP_REQ: begin
          if (byp_req_ready) st <= S_BYP_WAIT;
        end

        S_BYP_WAIT: begin
          if (byp_rsp_valid) begin
            rs <= byp_rsp;
            st <= S_RSP;
          end
        end

        S_RSP: st <= S_IDLE;

        default: st <= S_IDLE;
      endcase
    end
  end

  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (rst_n && to_valid) begin
      if (st != S_NOC)
        $error("ng_lsu_noc_adapter: stray NoC response outside a NoC request");
      else begin
        if (to_flit.cls != CLS_RESP || to_flit.tag[7:4] != TAG_HI)
          $error("ng_lsu_noc_adapter: response with wrong class / tag high bits");
        if (!nzmask[to_flit.tag[3:0]] || got[to_flit.tag[3:0]])
          $error("ng_lsu_noc_adapter: unexpected / duplicate response tag %0d",
                 to_flit.tag[3:0]);
      end
    end
  end
  // synthesis translate_on
endmodule
