#!/usr/bin/env bash
# unit test of ng_rcp_axmux (2 masters -> 1 ID-less AXI slave) with xsim.
#   rtl/shell/rm_cp/tb/run_tb_rcp_axmux.sh [vivado_bin_dir] [work_dir] ["seed ..."]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, $TMPDIR/ng_tb_rcp_axmux, seeds "1 2 3".
# The seed changes the traffic shape (issue / stall probabilities, latencies) and all random choices.
# Instantiates DW=64 DEPTH=16, DW=128 DEPTH=16, DW=64 DEPTH=4 (FIFO-full path) and the REG_A variants.
# SIM=verilator runs it with Verilator instead of xsim.  xelab -O0 mirrors the other
# run_tb_*.sh.  Expect NG_RCP_AXMUX_TB_PASS for every seed.
# SIM=verilator env: VERILATOR, VL_MAKEFLAGS, NJOBS as in rtl/shell/rm_sp/tb/run_tb_rsp_arb.sh.
set -eu
set -o pipefail
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-${TMPDIR:-/tmp}/ng_tb_rcp_axmux}
SEEDS=${3:-"1 2 3"}
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
RM=$ROOT/rtl/shell/rm_cp
PRIM=$ROOT/rtl/shell/prim
mkdir -p "$WORK"
cd "$WORK"
if [ "${SIM:-xsim}" = verilator ]; then
  # Verilator path for machines without Vivado (D6): SIM=verilator run_tb_rcp_axmux.sh x <work_dir> ["seed ..."]
  rm -rf obj
  ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} ${NJOBS:+-j "$NJOBS"} --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD -Wno-INITIALDLY     --top-module tb_rcp_axmux --Mdir obj -o sim "$RM/ng_rcp_axmux.sv" "$PRIM/ng_prim_skid.sv" "$PRIM/ng_prim_fifo_sync.sv" "$RM/tb/tb_rcp_axmux.sv" > vl_build.out 2>&1     || { grep -E "%Error" vl_build.out | head -20; tail -20 vl_build.out; exit 1; }
  for SEED in $SEEDS; do
    echo "$SEED" > seed.txt
    ./obj/sim > "vl_run_$SEED.out" 2>&1 || true
    echo "== axmux TB seed $SEED =="
    grep -E "TB_FAIL|core DW|NG_RCP_AXMUX|ERROR|FATAL" "vl_run_$SEED.out" | head -40
    grep -q NG_RCP_AXMUX_TB_PASS "vl_run_$SEED.out"
  done
  exit 0
fi
"$BIN/xvlog.bat" -sv "$RM/ng_rcp_axmux.sv" "$PRIM/ng_prim_skid.sv" "$PRIM/ng_prim_fifo_sync.sv" "$RM/tb/tb_rcp_axmux.sv" > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_rcp_axmux -s sim_axmux > xelab.out 2>&1 || { cat xelab.out; exit 1; }
for SEED in $SEEDS; do
  echo "$SEED" > seed.txt
  "$BIN/xsim.bat" sim_axmux -R > "xsim_$SEED.out" 2>&1 || true
  echo "== axmux TB seed $SEED =="
  grep -E "TB_FAIL|core DW|NG_RCP_AXMUX|ERROR|FATAL" "xsim_$SEED.out" | head -40
  grep -q NG_RCP_AXMUX_TB_PASS "xsim_$SEED.out"
done
