// ng_noc_router.sv -- NoC v1 router: 5-port (Local/East/West/North/South)
// wormhole router for a GW x GH 2D mesh, credit-based flow control, one
// virtual channel per message class (NUM_VC=2: VC0=requests, VC1=responses).
//
// Deadlock freedom (argument in):
//   * within a VC, routing is dimension-order (X then Y), so the channel
//     dependency graph of that VC is acyclic;
//   * the two VCs have independent credit pools and independent input FIFOs,
//     and no message ever changes class in flight, so no cyclic wait can form
//     across classes (protocol-level separation);
//   * hop-by-hop credits are exact: a flit is only sent when the receiver has
//     a reserved slot, giving end-to-end backpressure without loss.
//
// Link contract (all in the same clock domain, clk_l2):
//   forward: valid + flit (receiver must accept whenever valid -- space is
//             guaranteed by credit accounting);
//   backward: credit-return pulse, one per freed buffer slot.
// Every message is a single flit (ng_noc_pkg), so there is no
// head/body/tail distinction; FIFO order per (input,VC) preserves per-pair
// ordering.
module ng_noc_router #(
  parameter int X = 0,           // mesh coordinates of this router
  parameter int Y = 0,
  parameter int GW = 2,          // mesh width  (nodes per row)
  parameter int GH = 2,          // mesh height (rows)
  parameter int NUM_VC = 2,
  parameter int BUF_DEPTH = 4,   // input FIFO depth per (port, VC)
  parameter int OUT_CREDITS = BUF_DEPTH,  // credits granted to neighbour FIFOs
  parameter int LOC_CREDITS = BUF_DEPTH   // credits granted to local endpoint
) (
  input  logic                       clk,
  input  logic                       rst_n,
  // in links, index: 0=Local, 1=East, 2=West, 3=North, 4=South
  input  logic [4:0]                 in_valid,
  input  ng_noc_pkg::flit_t [4:0]    in_flit,
  output logic [4:0][NUM_VC-1:0]     in_credit_ret,
  // out links
  output logic [4:0]                 out_valid,
  output ng_noc_pkg::flit_t [4:0]    out_flit,
  input  logic [4:0][NUM_VC-1:0]     out_credit_ret
);

  import ng_noc_pkg::*;

  localparam int P_L = 0, P_E = 1, P_W = 2, P_N = 3, P_S = 4;
  localparam logic [2:0] PRT_L = 3'd0, PRT_E = 3'd1, PRT_W = 3'd2, PRT_N = 3'd3,
                         PRT_S = 3'd4;
  localparam int BUF_LB = clog2_1(BUF_DEPTH);
  localparam int CNT_W = BUF_LB + 1;
  localparam int CR_W = $clog2(((OUT_CREDITS > LOC_CREDITS) ? OUT_CREDITS
                                                             : LOC_CREDITS) + 1) + 1;
  localparam int NC = 5 * NUM_VC;  // flattened (port,vc) candidate count
  localparam int RRW = clog2_1(NC);

  // ---------------- routing: dimension order X then Y ----------------------
  function automatic logic [2:0] route(input logic [5:0] dst);
    int dx, dy;
    dx = dst % GW;
    dy = dst / GW;
    if (dx > X) return PRT_E;
    if (dx < X) return PRT_W;
    if (dy > Y) return PRT_N;
    if (dy < Y) return PRT_S;
    return PRT_L;
  endfunction

  // ---------------- input FIFOs --------------------------------------------
  flit_t                     fifo [5][NUM_VC][BUF_DEPTH];
  logic [BUF_LB-1:0]         rp [5][NUM_VC];
  logic [BUF_LB-1:0]         wp [5][NUM_VC];
  logic [CNT_W-1:0]          cnt [5][NUM_VC];

  // ---------------- output credit shadows ----------------------------------
  logic [CR_W-1:0]           cred [5][NUM_VC];

  // ---------------- arbitration --------------------------------------------
  logic [2:0]                tgt [5][NUM_VC];   // requested out port
  logic [4:0][NUM_VC-1:0]    grant;
  logic [RRW-1:0]            rr [5];            // round-robin pointer per out (0 .. NC-1)
  int                        widx [5];          // winner index per out

  always_comb begin
    int p, vc, o, k, idx, start;
    grant = '0;
    for (int a = 0; a < 5; a++)
      for (int b = 0; b < NUM_VC; b++) widx[a] = -1;
    for (int a = 0; a < 5; a++) begin
      for (int b = 0; b < NUM_VC; b++) begin
        // X-safe: only route entries that actually exist
        if (cnt[a][b] != '0 && cnt[a][b] !== 'x)
          tgt[a][b] = route(fifo[a][b][rp[a][b]].dst);
        else
          tgt[a][b] = '0;
      end
    end
    for (o = 0; o < 5; o++) begin
      start = rr[o];
      for (k = 0; k < NC; k++) begin
        idx = (start + k) % NC;
        p  = idx / NUM_VC;
        vc = idx % NUM_VC;
        if (cnt[p][vc] != '0 && cnt[p][vc] !== 'x &&
            tgt[p][vc] == 3'(o) && cred[o][vc] != '0) begin
          grant[p][vc] = 1'b1;
          widx[o] = idx;
          break;  // one grant per out port
        end
      end
    end
  end

  // ---------------- datapath / control --------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      for (int a = 0; a < 5; a++) begin
        for (int b = 0; b < NUM_VC; b++) begin
          rp[a][b]  <= '0;
          wp[a][b]  <= '0;
          cnt[a][b] <= '0;
          cred[a][b] <= (a == P_L) ? CR_W'(LOC_CREDITS) : CR_W'(OUT_CREDITS);
        end
      end
      rr            <= '{default: '0};
      out_valid     <= '0;
      in_credit_ret <= '0;
    end else begin
      in_credit_ret <= '0;
      out_valid     <= '0;

      // input side: accept (credit guarantees space), pop on grant.
      // X-safe: only push when the sender actually asserts valid.
      for (int p = 0; p < 5; p++) begin
        int vcin;
        vcin = (in_valid[p] === 1'b1) ? int'(in_flit[p].cls) : -1;
        for (int vc = 0; vc < NUM_VC; vc++) begin
          bit push, pop;
          push = (vcin == vc);
          pop  = grant[p][vc];
          if (push) begin
            fifo[p][vc][wp[p][vc]] <= in_flit[p];
            wp[p][vc] <= BUF_LB'(ptr_inc(wp[p][vc], BUF_DEPTH));   // wrap at BUF_DEPTH
          end
          if (pop) begin
            rp[p][vc] <= BUF_LB'(ptr_inc(rp[p][vc], BUF_DEPTH));
            in_credit_ret[p][vc] <= 1'b1;
          end
          cnt[p][vc] <= cnt[p][vc] + CNT_W'(push ? 1 : 0) - CNT_W'(pop ? 1 : 0);
        end
      end

      // grants: flit moves to the output register
      for (int p = 0; p < 5; p++) begin
        for (int vc = 0; vc < NUM_VC; vc++) begin
          if (grant[p][vc]) begin
            out_flit[tgt[p][vc]]  <= fifo[p][vc][rp[p][vc]];
            out_valid[tgt[p][vc]] <= 1'b1;
          end
        end
      end

      // output credit shadows: consume on grant, return on downstream pulse
      // (both can hit the same (port,vc) in one cycle -- one assignment)
      for (int o = 0; o < 5; o++) begin
        for (int vc = 0; vc < NUM_VC; vc++) begin
          bit take, give;
          take = 1'b0;
          for (int p = 0; p < 5; p++)
            if (grant[p][vc] && (tgt[p][vc] == 3'(o))) take = 1'b1;
          give = out_credit_ret[o][vc];
          cred[o][vc] <= cred[o][vc] + CR_W'(give) - CR_W'(take);
        end
      end

      // round-robin pointers advance past the winner
      for (int o = 0; o < 5; o++) begin
        if (widx[o] >= 0) rr[o] <= RRW'((widx[o] + 1) % NC);
      end
    end
  end

  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (rst_n) begin
      for (int p = 0; p < 5; p++) begin
        if (in_valid[p] && (in_flit[p].cls >= NUM_VC))
          $error("ng_noc_router: flit class out of range");
      end
      for (int p = 0; p < 5; p++) begin
        for (int vc = 0; vc < NUM_VC; vc++) begin
          if (cnt[p][vc] > CNT_W'(BUF_DEPTH))
            $error("ng_noc_router: input FIFO overflow");
        end
      end
    end
  end
  // synthesis translate_on
endmodule
