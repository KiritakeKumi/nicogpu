// ng_rmt_prim.sv -- small building blocks of the test RM (rm_kind 0x8001).
//
//   ng_rmt_pipe  N chained skids (N = 0: wires); SLR-crossing pipeline
//   ng_rmt_dly   N plain register stages for quasi-static / pulse sideband
//   ng_rmt_arb3  3:1 round-robin stream merge with a registered output
//   ng_rmt_islice  input-side register slice for boundary-driven channels
// The skid and the FIFO used here are the shared rtl/shell/prim/ng_prim_skid.sv (ATTR = 1: the SLR-pipeline /
// boundary-input coding) and rtl/shell/prim/ng_prim_fifo_sync.sv (REG_FLAGS = 1); every RM build that
// reads this file must read those two as well.
//
// SLR pipeline registers carry keep + shreg_extract="no": without them
// synthesis turns a pipe whose ready is tied high into an SRL (one site, no
// SLR crossing help) and merges the identical per-channel copies of shared
// buses (one chain fanning out to three SLRs).
//
// Control flops are reset; data flops are not (DFX_BOUNDARY).
module ng_rmt_pipe #(
  parameter int W = 32,
  parameter int N = 2
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         s_valid,
  output logic         s_ready,
  input  logic [W-1:0] s_data,
  output logic         m_valid,
  input  logic         m_ready,
  output logic [W-1:0] m_data
);
  if (N == 0) begin : g_wire
    assign m_valid = s_valid;
    assign s_ready = m_ready;
    assign m_data  = s_data;
  end else begin : g_pipe
    logic         v [N+1];
    logic         r [N+1];
    logic [W-1:0] d [N+1];
    assign v[0]    = s_valid;
    assign s_ready = r[0];
    assign d[0]    = s_data;
    for (genvar i = 0; i < N; i++) begin : g_st
      ng_prim_skid #(.W(W), .ATTR(1'b1)) u_skid (
        .clk(clk), .rst_n(rst_n),
        .s_valid(v[i]), .s_ready(r[i]), .s_data(d[i]),
        .m_valid(v[i+1]), .m_ready(r[i+1]), .m_data(d[i+1]), .occ());
    end
    assign m_valid = v[N];
    assign r[N]    = m_ready;
    assign m_data  = d[N];
  end
endmodule

// Plain delay line. RST=1: all stages reset to 0 (use for valid/pulse/level bits).
// KEEP=1: SLR pipeline stages (kept, never SRL); KEEP=0: local delay, SRL allowed.
module ng_rmt_dly #(
  parameter int W    = 1,
  parameter int N    = 2,
  parameter bit RST  = 1'b1,
  parameter bit KEEP = 1'b1
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [W-1:0] d,
  output logic [W-1:0] q
);
  if (N == 0) begin : g_wire
    assign q = d;
  end else if (KEEP) begin : g_keep
    (* keep = "true", shreg_extract = "no" *) logic [W-1:0] s [N];
    always_ff @(posedge clk) begin
      if (RST && !rst_n) begin
        for (int i = 0; i < N; i++) s[i] <= '0;
      end else begin
        s[0] <= d;
        for (int i = 1; i < N; i++) s[i] <= s[i-1];
      end
    end
    assign q = s[N-1];
  end else begin : g_dly
    logic [W-1:0] s [N];
    always_ff @(posedge clk) begin
      if (RST && !rst_n) begin
        for (int i = 0; i < N; i++) s[i] <= '0;
      end else begin
        s[0] <= d;
        for (int i = 1; i < N; i++) s[i] <= s[i-1];
      end
    end
    assign q = s[N-1];
  end
endmodule

// 3:1 round-robin merge of valid/ready streams into a skid register.
module ng_rmt_arb3 #(
  parameter int W = 32
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [2:0]   s_valid,
  output logic [2:0]   s_ready,
  input  logic [W-1:0] s_data0,
  input  logic [W-1:0] s_data1,
  input  logic [W-1:0] s_data2,
  output logic         m_valid,
  input  logic         m_ready,
  output logic [W-1:0] m_data
);
  logic [1:0]   last;   // last granted
  logic [1:0]   sel;
  logic         any;
  logic         k_ready;
  logic [W-1:0] k_data;

  always_comb begin
    any = |s_valid;
    unique case (last)
      2'd0:    sel = s_valid[1] ? 2'd1 : s_valid[2] ? 2'd2 : 2'd0;
      2'd1:    sel = s_valid[2] ? 2'd2 : s_valid[0] ? 2'd0 : 2'd1;
      default: sel = s_valid[0] ? 2'd0 : s_valid[1] ? 2'd1 : 2'd2;
    endcase
    k_data  = (sel == 2'd0) ? s_data0 : (sel == 2'd1) ? s_data1 : s_data2;
    s_ready = 3'b000;
    s_ready[sel] = k_ready;
  end

  always_ff @(posedge clk) begin
    if (!rst_n)                last <= 2'd2;
    else if (any && k_ready)   last <= sel;
  end

  ng_prim_skid #(.W(W), .ATTR(1'b1)) u_out (
    .clk(clk), .rst_n(rst_n),
    .s_valid(any), .s_ready(k_ready), .s_data(k_data),
    .m_valid(m_valid), .m_ready(m_ready), .m_data(m_data), .occ());
endmodule

// Input-side register slice for valid/ready channels driven by boundary
// inputs (DFX_BOUNDARY: at most one LUT between an input pin and the
// first flip-flop).  s_valid is only ANDed with the registered s_ready; data
// is captured unconditionally; a small FIFO behind the capture stage absorbs
// the one-cycle ready latency (s_ready is computed from flip-flops and the
// internal pop only, assuming the worst case that the current cycle accepts).
module ng_rmt_islice #(
  parameter int W     = 32,
  parameter int DEPTH = 4
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         s_valid,
  output logic         s_ready,
  input  logic [W-1:0] s_data,
  output logic         m_valid,
  input  logic         m_ready,
  output logic [W-1:0] m_data
);
  localparam int CW = $clog2(DEPTH + 1);
  logic          v1;
  logic [W-1:0]  d1;
  logic [CW-1:0] occ;       // accepted and not yet popped
  logic          f_empty;
  wire           pop = m_valid && m_ready;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      v1      <= 1'b0;
      occ     <= '0;
      s_ready <= 1'b0;
    end else begin
      v1      <= s_valid && s_ready;
      occ     <= occ + CW'(s_valid && s_ready) - CW'(pop);
      s_ready <= (occ + CW'(s_ready) - CW'(pop)) < CW'(DEPTH);
    end
  end
  always_ff @(posedge clk) d1 <= s_data;

  ng_prim_fifo_sync #(.W(W), .DEPTH(DEPTH), .REG_FLAGS(1'b1)) u_f (
    .clk(clk), .rst_n(rst_n), .push(v1), .din(d1), .pop(pop),
    .dout(m_data), .empty(f_empty), .full(), .count());
  assign m_valid = !f_empty;
endmodule
