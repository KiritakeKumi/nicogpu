// ng_noc_mesh.sv -- NoC v1 mesh: GW x GH grid of ng_noc_router.
// Node id = y * GW + x.  Endpoints (cores, L2 slices, CP, copy engine)
// attach at every node through the same credit-based link contract the
// routers use between each other; ng_noc_ep converts that link to plain
// valid/ready streams on the agent side.
//
// Injection (endpoint -> NoC) and receive (NoC -> endpoint) are separate
// arrays per node; VC0 carries requests, VC1 responses (see ng_noc_pkg).
module ng_noc_mesh #(
  parameter int GW = 2,
  parameter int GH = 2,
  parameter int NUM_VC = 2,
  parameter int BUF_DEPTH = 4,   // router input FIFO depth (also = injection
                                 // credits announced to endpoints)
  parameter int LOC_CREDITS = 4  // per-VC receive budget each endpoint grants
) (
  input  logic                            clk,
  input  logic                            rst_n,
  // endpoint -> NoC (injection) per node
  input  logic [GW*GH-1:0]                ep_inj_valid,
  input  ng_noc_pkg::flit_t [GW*GH-1:0]   ep_inj_flit,
  output logic [GW*GH-1:0][NUM_VC-1:0]    ep_inj_credit,   // pulse per freed slot
  // NoC -> endpoint per node
  output logic [GW*GH-1:0]                ep_recv_valid,
  output ng_noc_pkg::flit_t [GW*GH-1:0]   ep_recv_flit,
  input  logic [GW*GH-1:0][NUM_VC-1:0]    ep_recv_credit  // pulse per freed slot
);

  import ng_noc_pkg::*;

  localparam int NN = GW * GH;
  localparam logic [2:0] P_L = 3'd0, P_E = 3'd1, P_W = 3'd2, P_N = 3'd3,
                         P_S = 3'd4;

  // per-router link wires: lin_* = router input side, lout_* = output side
  logic [NN-1:0][4:0]                 lin_v, lout_v;
  flit_t [NN-1:0][4:0]                lin_f, lout_f;
  logic [NN-1:0][4:0][NUM_VC-1:0]     lin_cr, lout_cr;

  for (genvar gy = 0; gy < GH; gy++) begin : g_row
    for (genvar gx = 0; gx < GW; gx++) begin : g_col
      localparam int N = gy * GW + gx;
      ng_noc_router #(
        .X          (gx),
        .Y          (gy),
        .GW         (GW),
        .GH         (GH),
        .NUM_VC     (NUM_VC),
        .BUF_DEPTH  (BUF_DEPTH),
        .OUT_CREDITS(BUF_DEPTH),
        .LOC_CREDITS(LOC_CREDITS)
      ) u_r (
        .clk            (clk),
        .rst_n          (rst_n),
        .in_valid       (lin_v[N]),
        .in_flit        (lin_f[N]),
        .in_credit_ret  (lin_cr[N]),
        .out_valid      (lout_v[N]),
        .out_flit       (lout_f[N]),
        .out_credit_ret (lout_cr[N])
      );

      // local endpoint link
      assign lin_v[N][P_L] = ep_inj_valid[N];
      assign lin_f[N][P_L] = ep_inj_flit[N];
      assign ep_inj_credit[N] = lin_cr[N][P_L];
      assign ep_recv_valid[N] = lout_v[N][P_L];
      assign ep_recv_flit[N]  = lout_f[N][P_L];
      assign lout_cr[N][P_L]  = ep_recv_credit[N];

      // East link: my P_E out -> neighbour's P_W in
      if (gx + 1 < GW) begin : g_e
        localparam int M = gy * GW + gx + 1;
        assign lin_v[M][P_W] = lout_v[N][P_E];
        assign lin_f[M][P_W] = lout_f[N][P_E];
        assign lout_cr[N][P_E] = lin_cr[M][P_W];
      end else begin : g_e_edge
        assign lout_cr[N][P_E] = '0;  // nothing routes off-grid
      end

      // West link
      if (gx > 0) begin : g_w
        localparam int M = gy * GW + gx - 1;
        assign lin_v[M][P_E] = lout_v[N][P_W];
        assign lin_f[M][P_E] = lout_f[N][P_W];
        assign lout_cr[N][P_W] = lin_cr[M][P_E];
      end else begin : g_w_edge
        assign lout_cr[N][P_W] = '0;
      end

      // North link (node id +GW): my P_N out -> neighbour's P_S in
      if (gy + 1 < GH) begin : g_n
        localparam int M = (gy + 1) * GW + gx;
        assign lin_v[M][P_S] = lout_v[N][P_N];
        assign lin_f[M][P_S] = lout_f[N][P_N];
        assign lout_cr[N][P_N] = lin_cr[M][P_S];
      end else begin : g_n_edge
        assign lout_cr[N][P_N] = '0;
      end

      // South link
      if (gy > 0) begin : g_s
        localparam int M = (gy - 1) * GW + gx;
        assign lin_v[M][P_N] = lout_v[N][P_S];
        assign lin_f[M][P_N] = lout_f[N][P_S];
        assign lout_cr[N][P_S] = lin_cr[M][P_N];
      end else begin : g_s_edge
        assign lout_cr[N][P_S] = '0;
      end
    end
  end
endmodule
