#!/usr/bin/env bash
# FP32 atomics + L2 maintenance testbenches (xsim).
#   rtl/gpu/tb/run_tb_l2_p13b.sh [vivado_bin_dir] [work_dir] [sv_seed]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, build/tb_l2_p13b, seed 1.
# The FP32 reference vectors are regenerated per seed by
# sim/l2/gen_fatomic_vec.py (exact-rational reference of the model semantics).
# Expect NG_L2_FATOMIC_TB_PASS and NG_L2_MAINT_TB_PASS.
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-build/tb_l2_p13b}
SEED=${3:-1}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
SRC=$ROOT/rtl/gpu
PY=${PYTHON:-python}

mkdir -p "$WORK"
"$PY" "$ROOT/sim/l2/gen_fatomic_vec.py" --selftest
"$PY" "$ROOT/sim/l2/gen_fatomic_vec.py" "$WORK/fa_vec.hex" --seed "$SEED" --rand "${FA_RAND:-20000}"
cd "$WORK"

COMMON="$SRC/noc/ng_noc_pkg.sv $SRC/l2/ng_l2_addrmap.sv $SRC/l2/ng_l2_amo_alu.sv \
  $SRC/l2/ng_l2_slice.sv $SRC/prim/ng_prim_l2ram.sv $SRC/tb/tb_l2_axi_mem_model.sv"

run_one() {  # name top pass-token
  # shellcheck disable=SC2086
  "$BIN/xvlog.bat" -sv $COMMON "$SRC/tb/$1.sv" > "xvlog_$1.out" 2>&1 \
    || { cat "xvlog_$1.out"; exit 1; }
  "$BIN/xelab.bat" -debug off -O0 "$1" -s "sim_$1" > "xelab_$1.out" 2>&1 \
    || { cat "xelab_$1.out"; exit 1; }
  "$BIN/xsim.bat" "sim_$1" -runall -sv_seed "$SEED" > "xsim_$1.out" 2>&1 || true
  grep -E "TB_FAIL|PASS|vectors" "xsim_$1.out" | head -40
  grep -q "$2" "xsim_$1.out"
}
run_one tb_l2_fatomic NG_L2_FATOMIC_TB_PASS
run_one tb_l2_maint NG_L2_MAINT_TB_PASS
