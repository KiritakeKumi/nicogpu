// ng_shell_static_top.sv -- skeleton of the static region around the DFX
// boundary (DFX_BOUNDARY.md).  What is here:
//   * RM clock buffers (gated, clk_core = clk_core2x / 2) and RM resets
//   * static reset/synchronizers into clk_l2
//   * ng_shell_ctrl (BAR1 registers, life cycle, PR/ICAP path, MSI arming)
//   * ng_dfx_boundary (generated: register slices, isolation, the RP instance)
// What P0-11 adds around it: SI5381 init, MMCM (ng_prim_mmcm), XDMA in AXI
// Bridge mode, the BAR0/BAR1/BAR2 router with the clk_sh -> clk_l2 clock
// converters, 4x MIG with clk_l2 -> ui_clk converters, BSCAN, flash boot.
// The sh_* ports below are exactly the shell side of the boundary; ports on
// clk_l2 are synchronous to clk_l2_s (same MMCM output as the RM's clk_l2).
module ng_shell_static_top
  import ng_dfx_pkg::*;
#(
  parameter logic [63:0] SHELL_UID = 64'h0,
  parameter logic [15:0] SHELL_REV   // no default: ng_board_top is the single place that sets the revision
) (
  // PCIe bridge clock domain (XDMA axi_aclk / axi_aresetn)
  input  logic        clk_sh,
  input  logic        rst_sh_n,
  // MMCM outputs (static region) and status
  input  logic        mmcm_clk_core2x,   // CLKOUT0, 399.767 MHz
  input  logic        mmcm_clk_l2,       // CLKOUT1, 249.854 MHz
  input  logic        mmcm_clk_ts,       // CLKOUT2,  99.942 MHz
  input  logic        mmcm_locked,
  output logic        mmcm_reset,
  // MMCM DRP (clk_sh domain, DCLK = clk_sh), driven by CLK_DRP_* in the ctrl block
  output logic        mmcm_drp_en,
  output logic        mmcm_drp_we,
  output logic [6:0]  mmcm_drp_addr,
  output logic [15:0] mmcm_drp_di,
  input  logic [15:0] mmcm_drp_do,
  input  logic        mmcm_drp_rdy,
  // BAR router health
  input  logic        fence_stuck,
  input  logic        eos,               // STARTUPE3 EOS, async (synchronized in the ctrl block)
  // board status
  input  logic        si_done,
  input  logic        si_err,
  input  logic [3:0]  mem_calib,
  input  logic        link_up,
  // static clk_l2 for the shell-side CDCs of the boundary ports
  output logic        clk_l2_s,
  output logic        rst_l2_s_n,
  // BAR1 (shell registers), clk_sh domain
  input  logic [15:0] bar1_awaddr,
  input  logic        bar1_awvalid,
  output logic        bar1_awready,
  input  logic [31:0] bar1_wdata,
  input  logic [3:0]  bar1_wstrb,
  input  logic        bar1_wvalid,
  output logic        bar1_wready,
  output logic [1:0]  bar1_bresp,
  output logic        bar1_bvalid,
  input  logic        bar1_bready,
  input  logic [15:0] bar1_araddr,
  input  logic        bar1_arvalid,
  output logic        bar1_arready,
  output logic [31:0] bar1_rdata,
  output logic [1:0]  bar1_rresp,
  output logic        bar1_rvalid,
  input  logic        bar1_rready,
  // MSI (XDMA usr_irq_req / usr_irq_ack), clk_sh domain
  output logic [1:0]  msi_req,
  input  logic [1:0]  msi_ack,
  // PR/ICAP black box -> BSCAN USER1 (ng_bscan_status), debug only
  output logic [159:0] pr_dbg,
`include "ng_dfx_shell_ports.svh"
);
  // ----------------------------------------------------------------
  logic en_core_s, en_l2_s, en_ts_s, rm_clk_en;
  logic clk_core, clk_core2x, clk_l2, clk_ts;

  // static copies (never gated): boundary logic, synchronizers.  The static
  // l2 copy crosses synchronously into the RM's gated clk_l2 at the
  // boundary, so it uses the same buffer type as clk_l2 (BUFGCE).
  ng_prim_clkdiv #(.DIV(1)) u_bufg_l2s (.clk_i(mmcm_clk_l2), .en(1'b1), .clk_o(clk_l2_s));

  ng_prim_sync u_en_core (.clk(mmcm_clk_core2x), .rst_n(mmcm_locked), .d(rm_clk_en), .q(en_core_s));
  ng_prim_sync u_en_l2   (.clk(mmcm_clk_l2),     .rst_n(mmcm_locked), .d(rm_clk_en), .q(en_l2_s));
  ng_prim_sync u_en_ts   (.clk(mmcm_clk_ts),     .rst_n(mmcm_locked), .d(rm_clk_en), .q(en_ts_s));
  ng_prim_clkdiv #(.DIV(1), .DIV_BUF(1'b1)) u_bufg_core2x (.clk_i(mmcm_clk_core2x), .en(en_core_s), .clk_o(clk_core2x));
  ng_prim_clkdiv #(.DIV(2)) u_bufg_core   (.clk_i(mmcm_clk_core2x), .en(en_core_s), .clk_o(clk_core));
  ng_prim_clkdiv #(.DIV(1)) u_bufg_l2     (.clk_i(mmcm_clk_l2),     .en(en_l2_s),   .clk_o(clk_l2));
  ng_prim_clkdiv #(.DIV(1)) u_bufg_ts     (.clk_i(mmcm_clk_ts),     .en(en_ts_s),   .clk_o(clk_ts));

  // ----------------------------------------------------------------
  logic rm_reset, rst_ts_s_n;
  logic rst_core_n, rst_l2_n, rst_ts_n;
  ng_prim_rst_sync #(.FANOUT_STAGE(1'b1)) u_rst_l2s (.clk(clk_l2_s),    .arst_n(rst_sh_n && mmcm_locked), .rst_n(rst_l2_s_n));
  ng_prim_rst_sync u_rst_tss (.clk(mmcm_clk_ts), .arst_n(rst_sh_n && mmcm_locked), .rst_n(rst_ts_s_n));
  // RM resets: asynchronous assertion, synchronous deassertion in each RM domain.
  ng_prim_rst_sync u_rst_core (.clk(clk_core), .arst_n(rst_l2_s_n && !rm_reset), .rst_n(rst_core_n));
  ng_prim_rst_sync u_rst_l2   (.clk(clk_l2),   .arst_n(rst_l2_s_n && !rm_reset), .rst_n(rst_l2_n));
  ng_prim_rst_sync u_rst_ts   (.clk(clk_ts),   .arst_n(rst_l2_s_n && !rm_reset), .rst_n(rst_ts_n));

  // ------------------------------------------------------------------ control <-> clk_l2
  logic quiesce_req, decouple, info_sample_sh;   // clk_sh
  logic dec, info_sample, sh_quiesce_req;          // clk_l2_s (boundary names)
  logic [3:0] sh_mem_ready;
  ng_prim_sync #(.W(3)) u_ctl2l2 (.clk(clk_l2_s), .rst_n(rst_l2_s_n),
    .d({quiesce_req, decouple, info_sample_sh}), .q({sh_quiesce_req, dec, info_sample}));
  ng_prim_sync #(.W(4)) u_cal2l2 (.clk(clk_l2_s), .rst_n(rst_l2_s_n), .d(mem_calib), .q(sh_mem_ready));

  logic iso_idle, m_drained, err_pulse, err_toggle;
  logic [7:0] m_rd_out, m_wr_out, s_rd_out, s_wr_out;
  logic [3:0] rm_irq_q;
  logic rm_quiesce_ack_q, rm_ready_q;
  logic [127:0] rm_info_q;
  always_ff @(posedge clk_l2_s) begin
    if (!rst_l2_s_n) err_toggle <= 1'b0;
    else if (err_pulse) err_toggle <= ~err_toggle;
  end

  // ------------------------------------------------------------------ clock meters
  // CLK_FREQ_*: edge counts of the ungated MMCM outputs over a 1 ms window of
  // clk_sh (125 MHz -> 125,000 cycles), so the count is the frequency in kHz.
  // clk_core is exactly clk_core2x / 2 (ng_prim_clkdiv), the ctrl block halves.
  localparam int CLK_SH_KHZ = 125000;
  logic [31:0] freq_core2x, freq_l2, freq_ts;
  ng_clk_meas #(.REF_TICKS(CLK_SH_KHZ), .REF_FREQ_KHZ(CLK_SH_KHZ), .CNT_W(20)) u_meas_core2x (
    .clk_meas(mmcm_clk_core2x), .clk_ref(clk_sh), .rst_n(rst_sh_n), .freq_khz(freq_core2x));
  ng_clk_meas #(.REF_TICKS(CLK_SH_KHZ), .REF_FREQ_KHZ(CLK_SH_KHZ), .CNT_W(20)) u_meas_l2 (
    .clk_meas(mmcm_clk_l2),     .clk_ref(clk_sh), .rst_n(rst_sh_n), .freq_khz(freq_l2));
  ng_clk_meas #(.REF_TICKS(CLK_SH_KHZ), .REF_FREQ_KHZ(CLK_SH_KHZ), .CNT_W(20)) u_meas_ts (
    .clk_meas(mmcm_clk_ts),     .clk_ref(clk_sh), .rst_n(rst_sh_n), .freq_khz(freq_ts));

  // ------------------------------------------------------------------ shell control (BAR1)
  ng_shell_ctrl #(.SHELL_UID(SHELL_UID), .SHELL_REV(SHELL_REV)) u_ctrl (
    .clk_sh(clk_sh), .rst_sh_n(rst_sh_n),
    .s_awaddr(bar1_awaddr), .s_awvalid(bar1_awvalid), .s_awready(bar1_awready),
    .s_wdata(bar1_wdata), .s_wstrb(bar1_wstrb), .s_wvalid(bar1_wvalid), .s_wready(bar1_wready),
    .s_bresp(bar1_bresp), .s_bvalid(bar1_bvalid), .s_bready(bar1_bready),
    .s_araddr(bar1_araddr), .s_arvalid(bar1_arvalid), .s_arready(bar1_arready),
    .s_rdata(bar1_rdata), .s_rresp(bar1_rresp), .s_rvalid(bar1_rvalid), .s_rready(bar1_rready),
    .si_done(si_done), .si_err(si_err), .mmcm_locked(mmcm_locked), .mem_calib(mem_calib), .link_up(link_up),
    .fence_stuck(fence_stuck), .eos(eos), .freq_core2x(freq_core2x), .freq_l2(freq_l2), .freq_ts(freq_ts),
    .drp_en(mmcm_drp_en), .drp_we(mmcm_drp_we), .drp_a(mmcm_drp_addr), .drp_di(mmcm_drp_di),
    .drp_do(mmcm_drp_do), .drp_rdy(mmcm_drp_rdy),
    .quiesce_req(quiesce_req), .decouple(decouple), .rm_reset(rm_reset), .info_sample(info_sample_sh),
    .rm_clk_en(rm_clk_en), .mmcm_reset(mmcm_reset),
    .l2_rm_quiesce_ack(rm_quiesce_ack_q), .l2_m_drained(m_drained), .l2_iso_idle(iso_idle),
    .l2_rm_ready(rm_ready_q), .l2_rm_irq0(rm_irq_q[0]), .l2_err_pulse_toggle(err_toggle),
    .l2_outstanding({s_wr_out, s_rd_out, m_wr_out, m_rd_out}), .l2_rm_info(rm_info_q),
    .msi_req(msi_req), .msi_ack(msi_ack),
    .clk_icap(mmcm_clk_ts), .rst_icap_n(rst_ts_s_n), .pr_dbg(pr_dbg)
  );

  // ------------------------------------------------------------------ boundary + RP
  logic sh_rst_n;
  assign sh_rst_n = rst_l2_s_n;

  ng_dfx_boundary u_boundary (.*);
endmodule
