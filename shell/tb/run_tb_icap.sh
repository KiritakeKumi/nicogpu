#!/usr/bin/env bash
# PR diagnostics: run the ng_prim_icap model testbench (word-order selection,
# continuous CSIB, ICAPE3.O capture/snapshot) with Vivado xsim.
#   rtl/shell/tb/run_tb_icap.sh [vivado_bin_dir] [work_dir]
# Expect NG_PRIM_ICAP_TB_PASS.  Use a fresh work_dir each time.
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/tb_icap}
SRC=$TB_SHELL
mkdir -p "$WORK"
cd "$WORK"
xsim_run tb_prim_icap NG_PRIM_ICAP_TB_PASS "$SRC/prim/ng_prim_icap.sv" "$SRC/tb/tb_prim_icap.sv"
