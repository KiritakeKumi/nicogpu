// ng_prim_skid.sv -- full-throughput register slice (skid buffer) for one valid/ready channel, shared by the static
// shell (boundary channels, clock-converter read sides) and the reconfigurable modules.  s_ready, m_valid and m_data
// all come straight from flip-flops (usable on boundary pins, DFX_BOUNDARY.md 7); control flops are reset, data flops
// are not (DFX_BOUNDARY 8); no FPGA init values.  occ = entries held (m_valid + the skid register, 0..2).
//
//   W     data width
//   ATTR  0: plain coding.  1: the SLR-pipeline / boundary-input coding of the test RM: the load condition
//         (m_ready || !m_valid, a boundary input when the skid drives a master port) and the reset terms are forced
//         through one LUT to the CE / synchronous set-reset pins (direct_enable / direct_reset).  Both codings behave
//         identically; SystemVerilog cannot make an attribute conditional on a parameter, hence two generate arms.
//         The ports and the skid register carry shreg_extract = "no" in both arms (not conditional either): without it
//         synthesis turns a pipe whose ready is tied high into an SRL (one site, no SLR crossing help) and merges
//         identical per-channel copies of shared buses.  The attribute is inert on the other instances (no flop chain).
//
// Replaces ng_axi_skid (ATTR = 0), ng_rmt_skid (ATTR = 1) and ng_rcp_axmux_skid (ATTR = 0).
module ng_prim_skid #(
  parameter int W    = 32,
  parameter bit ATTR = 1'b0
) (
  input  logic         clk,
  input  logic         rst_n,
  input  logic         s_valid,
  (* shreg_extract = "no" *) output logic s_ready,
  input  logic [W-1:0] s_data,
  (* shreg_extract = "no" *) output logic m_valid,
  input  logic         m_ready,
  (* shreg_extract = "no" *) output logic [W-1:0] m_data,
  output logic [1:0]   occ
);
  (* shreg_extract = "no" *) logic         buf_v;
  (* shreg_extract = "no" *) logic [W-1:0] buf_d;
  wire          acc = s_valid && s_ready;   // s_ready == !buf_v
  wire          ld  = m_ready || !m_valid;

  assign occ = {1'b0, m_valid} + {1'b0, buf_v};

  if (ATTR) begin : g_attr
    // m_ready must reach every flop through one LUT: the load condition goes straight to CE (data, m_valid) or to
    // the synchronous set/reset (buf_v, s_ready).
    (* direct_enable = "yes" *) wire ld_ce  = m_ready || !m_valid;
    (* direct_reset = "yes" *)  wire ld_rst = m_ready || !m_valid || !rst_n;
    (* direct_reset = "yes" *)  wire v_rst  = !rst_n;

    always_ff @(posedge clk) begin
      if (v_rst)      m_valid <= 1'b0;
      else if (ld_ce) m_valid <= buf_v || acc;
    end
    always_ff @(posedge clk) begin
      if (ld_rst)   buf_v <= 1'b0;
      else if (acc) buf_v <= 1'b1;
    end
    always_ff @(posedge clk) begin
      if (ld_rst)   s_ready <= 1'b1;
      else if (acc) s_ready <= 1'b0;
    end
    always_ff @(posedge clk) begin
      if (ld_ce) m_data <= buf_v ? buf_d : s_data;
    end
    always_ff @(posedge clk) begin
      if (acc) buf_d <= s_data;   // acc implies buf_v == 0: harmless when m_data loads too
    end
  end else begin : g_plain
    always_ff @(posedge clk) begin
      if (!rst_n) begin
        m_valid <= 1'b0; buf_v <= 1'b0; s_ready <= 1'b1;
      end else if (ld) begin
        m_valid <= buf_v || acc; buf_v <= 1'b0; s_ready <= 1'b1;
      end else if (acc) begin
        buf_v <= 1'b1; s_ready <= 1'b0;
      end
    end
    always_ff @(posedge clk) begin
      if (ld) m_data <= buf_v ? buf_d : s_data;
      if (acc) buf_d <= s_data;                  // acc implies buf_v == 0: harmless when m_data loads too
    end
  end
endmodule
