#!/usr/bin/env bash
# L1D -> ng_lsu_noc_adapter -> NoC mesh -> 4 L2 slices -> DDR models
# (tb_lsu_noc_l2), random + directed, shadow-memory reference.
#   rtl/gpu/tb/run_tb_lsu_noc_l2.sh [work_dir] [vivado_bin_dir] ["seed ..."]
# Defaults: build/gpu/tb_lsu_noc_l2, C:/AMDDesignTools/2026.1/Vivado/bin, seeds "1 2 3".
# Env: SIM=xsim|verilator (default xsim).  Verilator on Windows/Git Bash needs
#   VERILATOR=verilator.exe VL_MAKEFLAGS="CXX=g++ CC=gcc LINK=g++" plus the PATH of
#   sim/verilator/env.bat (see run_tb_lsu_unit.sh); unset on Linux/CI.
# xelab -O0 works around an xsim 2026.1 optimizer crash on the L2 designs.
# Expect NG_LSU_NOC_L2_TB_PASS per seed and finally NG_LSU_NOC_L2_ALL_OK.
set -eu
WORK=${1:-build/gpu/tb_lsu_noc_l2}
BIN=${2:-C:/AMDDesignTools/2026.1/Vivado/bin}
SEEDS=${3:-"1 2 3"}
SIM=${SIM:-xsim}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"

TOP=tb_lsu_noc_l2
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/l2/ng_l2_addrmap.sv $RTL/lsu/ng_lsu_pkg.sv \
  $RTL/noc/ng_noc_router.sv $RTL/noc/ng_noc_mesh.sv $RTL/noc/ng_noc_ep.sv \
  $RTL/l2/ng_l2_amo_alu.sv $RTL/l2/ng_l2_slice.sv $RTL/prim/ng_prim_l2ram.sv \
  $RTL/lsu/ng_l1d.sv $RTL/lsu/ng_lsu_noc_adapter.sv \
  $RTL/tb/tb_l2_axi_mem_model.sv $RTL/tb/$TOP.sv"

if [ "$SIM" = verilator ]; then
  rm -rf "obj_$TOP"
  # shellcheck disable=SC2086
  ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 4096 --unroll-stmts 4000000 -Wno-TIMESCALEMOD --top-module "$TOP" \
    -Mdir "obj_$TOP" $SRCS > "vl_$TOP.build" 2>&1 || { tail -40 "vl_$TOP.build"; exit 1; }
else
  # shellcheck disable=SC2086
  "$BIN/xvlog.bat" -sv $SRCS > "xvlog_$TOP.out" 2>&1 || { cat "xvlog_$TOP.out"; exit 1; }
  "$BIN/xelab.bat" -debug off -O0 "$TOP" -s "${TOP}_s" > "xelab_$TOP.out" 2>&1 \
    || { cat "xelab_$TOP.out"; exit 1; }
fi

for seed in $SEEDS; do
  out="sim_${TOP}_${seed}.out"
  if [ "$SIM" = verilator ]; then
    "./obj_$TOP/V$TOP" +SEED="$seed" > "$out" 2>&1 || true
  else
    "$BIN/xsim.bat" "${TOP}_s" -runall -sv_seed "$seed" > "$out" 2>&1 || true
  fi
  grep -E "TB_FAIL|NG_LSU_NOC|TB: (ops|flits|DDR|final|D4)|Error|\\\$error" "$out" | head -40 || true
  grep -q NG_LSU_NOC_L2_TB_PASS "$out" || { echo "--- $out (tail) ---"; tail -30 "$out"; exit 1; }
done
echo "NG_LSU_NOC_L2_ALL_OK"
