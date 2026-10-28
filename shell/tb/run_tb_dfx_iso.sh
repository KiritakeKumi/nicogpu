#!/usr/bin/env bash
# P0-10: run the DFX isolation testbench with Vivado xsim (was syn/dfx/run_tb.sh).
#   rtl/shell/tb/run_tb_dfx_iso.sh [vivado_bin_dir] [work_dir]
# Defaults: Vivado from NICOGPU_VIVADO / C:/AMDDesignTools/2026.1/Vivado/bin (tb_lib.sh) and build/tb_dfx_iso.
# Expect TB_DFX_ISO_PASS.  Use a fresh work_dir each time (xsim.dir locking).
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/tb_dfx_iso}
SRC=$TB_SHELL
mkdir -p "$WORK"
cd "$WORK"
xsim_run tb_dfx_iso TB_DFX_ISO_PASS "$SRC/prim/ng_prim_fifo_sync.sv" "$SRC/ng_dfx_iso_s.sv" "$SRC/ng_dfx_iso_m.sv" "$SRC/tb/tb_dfx_iso.sv"
