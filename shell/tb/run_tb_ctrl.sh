#!/usr/bin/env bash
# P0-11: run the ng_shell_ctrl testbench (W1C/MSI-storm, release interlock,
# PR smoke) with Vivado xsim.
#   rtl/shell/tb/run_tb_ctrl.sh [vivado_bin_dir] [work_dir]
# Defaults: Vivado from NICOGPU_VIVADO / C:/AMDDesignTools/2026.1/Vivado/bin (tb_lib.sh) and build/tb_ctrl (relative
# to the caller).  Expect NG_CTRL_TB_PASS.  Use a fresh work_dir each time
# (xsim.dir locking).
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/tb_ctrl}
SRC=$TB_SHELL
mkdir -p "$WORK"
cd "$WORK"
xsim_run tb_shell_ctrl NG_CTRL_TB_PASS "$SRC/generated/ng_dfx_pkg.sv" "$SRC/prim/ng_prim_sync.sv" \
  "$SRC/prim/ng_prim_fifo_async.sv" "$SRC/prim/ng_prim_icap.sv" \
  "$SRC/ng_shell_ctrl.sv" "$SRC/tb/tb_shell_ctrl.sv"
