// ng_bscan_status.sv -- BSCAN USER1 shell status observation chain (P0-11).
//
// Replaces the old per-design observation chains (ddr_siclk_status etc.):
// one 128-bit snapshot register, captured on CAPTURE and shifted out on
// SHIFT; new bits shift in as zeros.  Read from OpenOCD with
//   jtag newtap ... ; then irscan USER1; drscan 128 bits
// Bit map (LSB first):
//    [0]     si_done            [1]     si_err
//    [2]     mmcm_locked        [3]     fence_stuck
//    [4]     link_up            [7:5]   ltssm_state
//    [11:8]  mem_calib C1..C4   [12]    msi_enable
//   [15:13]  msi_vector_width  [31:16]  shell revision
//   [95:32]  shell UID         [127:96] reserved, reads as zero
//   [287:128] PR/ICAP black box (pr_dbg, layout in ng_shell_ctrl.sv):
//            ICAP words, live O, PR_WORDS, clk_sh / clk_icap heartbeats,
//            PR/ICAP flags, marker 0x5A in [287:280].  Appended after the
//            original 128 bits, so a 128-bit scan still reads the old layout
//            unchanged (LSB shifts out first); scan 288 bits for the box.
//            Sampled raw on CAPTURE (no synchronizer): values may tear while
//            they change, which is fine for a post-mortem read.
// SHELL_REV has no default: the board top is the only place that says which revision this build is.
// Without BSCANE2 (RTL elaboration, simulation) the module has no behaviour: the ports stay unused.
module ng_bscan_status #(
  parameter logic [15:0] SHELL_REV,
  parameter logic [63:0] SHELL_UID = 64'h0
) (
  input  logic       si_done,
  input  logic       si_err,
  input  logic       mmcm_locked,
  input  logic       fence_stuck,
  input  logic       link_up,
  input  logic [5:0] ltssm_state,
  input  logic [3:0] mem_calib,
  input  logic       msi_enable,
  input  logic [2:0] msi_vector_width,
  input  logic [159:0] pr_dbg
);
`ifdef SYNTHESIS
  logic capture, shift, sel, tdi, tck, tdo, tap_reset;
  BSCANE2 u_bscan (
    .CAPTURE (capture),
    .DRCK    (),
    .RESET   (tap_reset),
    .RUNTEST (),
    .SEL     (sel),
    .SHIFT   (shift),
    .TCK     (tck),
    .TDI     (tdi),
    .TMS     (),
    .UPDATE  (),
    .TDO     (tdo)
  );

  logic [287:0] sr;
  logic [127:0] base;
  // the original concatenation is 131 bits wide (
  // 5.1); its top 3 bits are zeros and were always cut by the 128-bit assign
  assign base = 128'({32'h0, SHELL_UID, SHELL_REV, msi_vector_width, msi_enable,
                      mem_calib, ltssm_state, link_up, fence_stuck,
                      mmcm_locked, si_err, si_done});
  always_ff @(posedge tck) begin
    if (tap_reset)       sr <= '0;  // BSCANE2 RESET is active high (Test-Logic-Reset)
    else if (capture)    sr <= {pr_dbg, base};
    else if (shift && sel) sr <= {tdi, sr[287:1]};
  end
  assign tdo = sr[0];
`endif
endmodule
