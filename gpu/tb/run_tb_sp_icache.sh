#!/usr/bin/env bash
# unit test of the L0 I$ (ng_sp_icache) against a reference model (rtl/gpu/tb/tb_sp_icache.sv).
#   rtl/gpu/tb/run_tb_sp_icache.sh [LINES:WAYS ...]      (default: 8:0 32:0 32:32 32:1 16:2 64:4)
# Env: SIM=xsim|verilator (default xsim; Verilator: VERILATOR / VL_MAKEFLAGS as run_tb_sp_top.sh, on this Windows
#      host source the environment of run_tb_sp_top_vl.sh), WORK (default build/gpu/tb_sp_icache[_vl]), VIVADO_BIN,
#      NG_SEED (default 1), NG_FETCHES (default 60000).
# Prints NG_SP_ICACHE_TB_PASS per configuration and NG_SP_ICACHE_TB_ALL_OK at the end.
set -eu
SIM=${SIM:-xsim}
BIN=${VIVADO_BIN:-C:/AMDDesignTools/2026.1/Vivado/bin}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
RTL=$ROOT/rtl/gpu
if [ "$SIM" = verilator ]; then WORK=${WORK:-$ROOT/build/gpu/tb_sp_icache_vl}; else WORK=${WORK:-$ROOT/build/gpu/tb_sp_icache}; fi
case $WORK in /*|?:*) ;; *) WORK=$ROOT/$WORK ;; esac
CFGS=${*:-"8:0 32:0 32:32 32:1 16:2 64:4"}
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/lsu/ng_lsu_pkg.sv $RTL/sp/ng_sp_icache.sv $RTL/tb/tb_sp_icache.sv"
for c in $CFGS; do
  L=${c%%:*}
  W=${c##*:}
  D=$WORK/c${L}_${W}
  mkdir -p "$D"
  cd "$D"
  {
    echo "\`define NG_TB_IC_LINES $L"
    echo "\`define NG_TB_IC_WAYS $W"
    echo "\`define NG_TB_SEED ${NG_SEED:-1}"
    echo "\`define NG_TB_FETCHES ${NG_FETCHES:-60000}"
  } > tb_sp_icache_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf obj
    ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} --binary --timing -Wno-fatal -Wno-lint -Wno-style \
      -Wno-INITIALDLY -Wno-TIMESCALEMOD +define+NG_SP_ASSERT --top-module tb_sp_icache -I. -Mdir obj $SRCS \
      > vl_build.out 2>&1 || { grep -E "%Error" vl_build.out | head -40; tail -20 vl_build.out; exit 1; }
    ./obj/Vtb_sp_icache > sim.out 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -d NG_SP_ASSERT -i . $SRCS > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
    "$BIN/xelab.bat" -debug off -O1 tb_sp_icache -s tb_sp_icache > xelab.out 2>&1 \
      || { grep -E "ERROR" xelab.out | head -40; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" tb_sp_icache > sim.out 2>&1 || true
  fi
  grep -E "TB_ERROR|NOTE|NG_SP_ICACHE|Fatal|FATAL" sim.out | head -30 || true
  grep -q "NG_SP_ICACHE_TB_PASS" sim.out || { echo "--- $D/sim.out (tail) ---"; tail -20 sim.out; exit 1; }
done
echo "NG_SP_ICACHE_TB_ALL_OK ($SIM $CFGS)"
