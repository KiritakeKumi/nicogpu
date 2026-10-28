#!/usr/bin/env bash
# P0-11: run the ng_axi64_to_lite32 testbench with Vivado xsim.
#   rtl/shell/tb/run_tb_lite.sh [vivado_bin_dir] [work_dir]
# Defaults: Vivado from NICOGPU_VIVADO / C:/AMDDesignTools/2026.1/Vivado/bin (tb_lib.sh) and build/tb_lite (relative
# to the caller).  Expect NG_LITE_TB_PASS.  Use a fresh work_dir each time
# (xsim.dir locking).
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/tb_lite}
SRC=$TB_SHELL
mkdir -p "$WORK"
cd "$WORK"
xsim_run tb_axi64_to_lite32 NG_LITE_TB_PASS "$SRC/ng_axi64_to_lite32.sv" "$SRC/tb/tb_axi64_to_lite32.sv"
