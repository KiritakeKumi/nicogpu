#!/usr/bin/env bash
# SFU arithmetic blocks (fma / divsqrt / pack functions / lane) vs SoftFloat + model kernels.
#   rtl/gpu/tb/run_tb_sfu_prim.sh <prim.txt> [work_dir] [vivado_bin_dir]
#   prim.txt: made by sim/sfu dump_prim (build/sfu_dump: `dump_prim prim.txt --n N --seed S`)
# Env: SIM=xsim|verilator (default xsim);  XSIM_ARGS / VL_ARGS passed to the simulator.
# Expect NG_SFU_PRIM_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_sfu_prim.sh <prim.txt> [work_dir] [vivado_bin]}
WORK=${2:-build/gpu/tb_sfu_prim}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
BIN=${3:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" prim.txt
SRCS="$RTL/sfu/ng_sfu_fp_pkg.sv $RTL/sfu/ng_sfu_fma.sv $RTL/sfu/ng_sfu_divsqrt.sv $RTL/sfu/ng_sfu_lane.sv \
$RTL/tb/tb_sfu_prim.sv"
if [ "$SIM" = verilator ]; then
  rm -rf obj
  verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style --top-module tb_sfu_prim -Mdir obj $SRCS > vl.build 2>&1 \
    || { tail -40 vl.build; exit 1; }
  ./obj/Vtb_sfu_prim +VEC=prim.txt ${VL_ARGS:-} > sim.out 2>&1 || true
else
  "$BIN/xvlog.bat" -sv $SRCS > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
  "$BIN/xelab.bat" -debug off tb_sfu_prim -s tb_sfu_prim_s > xelab.out 2>&1 || { cat xelab.out; exit 1; }
  printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} tb_sfu_prim_s > sim.out 2>&1 || true
fi
grep -E "FAIL|NG_SFU_PRIM|TB_ERROR|^op " sim.out | head -80 || true
grep -q "NG_SFU_PRIM_ALL_OK" sim.out
