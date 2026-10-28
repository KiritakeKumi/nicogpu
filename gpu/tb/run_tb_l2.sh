#!/usr/bin/env bash
# run the L2 testbenches (AMO ALU vectors + slice cache) with xsim.
#   rtl/gpu/tb/run_tb_l2.sh [vivado_bin_dir] [work_dir] [sv_seed]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, build/tb_l2, seed 1.
# Regenerates the golden interleave vectors if missing and copies them into
# the work dir (xsim.bat mangles -testplusarg, so the TB default "golden"
# relative path is used).
# xelab -O0 works around an xsim 2026.1 optimizer crash (EXCEPTION_ACCESS_
# VIOLATION) on this design.
# Rerun with different SEED values for multi-seed soak (the TBs also vary
# traffic internally per round).
# Expect NG_L2_AMO_TB_PASS and NG_L2_TB_PASS.
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-build/tb_l2}
SEED=${3:-1}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
SRC=$ROOT/rtl/gpu
GOLDEN=$SRC/tb/golden

if [ ! -f "$GOLDEN/l2_map_count.hex" ]; then
  (cd "$ROOT" && python rtl/gpu/tb/gen_golden/gen_interleave_golden.py)
fi

mkdir -p "$WORK/golden"
cp "$GOLDEN"/*.hex "$WORK/golden/"
cd "$WORK"

# ---- 1) AMO ALU ------------------------------------------------------------
"$BIN/xvlog.bat" -sv \
  "$SRC/noc/ng_noc_pkg.sv" "$SRC/l2/ng_l2_amo_alu.sv" "$SRC/tb/tb_l2_amo_alu.sv" \
  > xvlog_amo.out 2>&1 || { cat xvlog_amo.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_l2_amo_alu -s sim_amo > xelab_amo.out 2>&1 \
  || { cat xelab_amo.out; exit 1; }
"$BIN/xsim.bat" sim_amo -runall -sv_seed "$SEED" > xsim_amo.out 2>&1 || true
grep -E "TB_FAIL|PASS" xsim_amo.out
grep -q NG_L2_AMO_TB_PASS xsim_amo.out

# ---- 2) L2 slice -------------------------------------------------------------
"$BIN/xvlog.bat" -sv \
  "$SRC/noc/ng_noc_pkg.sv" \
  "$SRC/l2/ng_l2_addrmap.sv" "$SRC/l2/ng_l2_amo_alu.sv" "$SRC/l2/ng_l2_slice.sv" \
  "$SRC/prim/ng_prim_l2ram.sv" \
  "$SRC/tb/tb_l2_axi_mem_model.sv" "$SRC/tb/tb_l2_slice.sv" \
  > xvlog_slice.out 2>&1 || { cat xvlog_slice.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_l2_slice -s sim_slice > xelab_slice.out 2>&1 \
  || { cat xelab_slice.out; exit 1; }
"$BIN/xsim.bat" sim_slice -runall -sv_seed "$SEED" > xsim_slice.out 2>&1 || true
grep -E "TB_FAIL|PASS" xsim_slice.out
grep -q NG_L2_TB_PASS xsim_slice.out
