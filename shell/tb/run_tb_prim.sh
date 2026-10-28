#!/usr/bin/env bash
# P0-11: run the ng_prim_mmcm / ng_clk_meas testbench with Vivado xsim.
#   rtl/shell/tb/run_tb_prim.sh [vivado_bin_dir] [work_dir]
# Defaults: Vivado from NICOGPU_VIVADO / C:/AMDDesignTools/2026.1/Vivado/bin (tb_lib.sh) and build/prim_mmcm/tb
# (relative to the caller).  Expect NG_PRIM_MMCM_TB_PASS.  Use a fresh
# work_dir each time (xsim.dir locking).
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/prim_mmcm/tb}
SRC=$TB_SHELL
mkdir -p "$WORK"
cd "$WORK"
xsim_run tb_prim_mmcm NG_PRIM_MMCM_TB_PASS "$SRC/prim/ng_prim_sync.sv" "$SRC/prim/ng_prim_mmcm.sv" "$SRC/prim/ng_clk_meas.sv" \
  "$SRC/tb/tb_prim_mmcm.sv"
