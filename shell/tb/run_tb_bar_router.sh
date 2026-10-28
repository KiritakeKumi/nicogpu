#!/usr/bin/env bash
# P0-11: run the BAR router testbench with Vivado xsim.
#   rtl/shell/tb/run_tb_bar_router.sh [vivado_bin_dir] [work_dir]
# Expect NG_BAR_ROUTER_TB_PASS.  Use a fresh work_dir each time.
# Env: ROUTER=<path> tests another router copy (e.g. a pre-fix one);
#      XSIM_ARGS="-testplusarg s2n8000 -sv_seed 7" passes plusargs
#      (s2mask, nostop, noaxistab, nod8: see tb_bar_router.sv).
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/tb_router}
SRC=$TB_SHELL
RTR=${ROUTER:-$SRC/ng_bar_router.sv}
case $RTR in /*|?:*) ;; *) RTR=$(pwd)/$RTR ;; esac
mkdir -p "$WORK"
cd "$WORK"
xsim_run tb_bar_router NG_BAR_ROUTER_TB_PASS "$RTR" "$SRC/tb/tb_bar_router.sv"
