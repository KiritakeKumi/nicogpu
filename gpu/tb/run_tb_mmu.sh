#!/usr/bin/env bash
# run the MMU testbenches with xsim.
#   rtl/gpu/tb/run_tb_mmu.sh [vivado_bin_dir] [work_dir] ["seed ..."]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, $TMPDIR/ng_tb_mmu, seed 1.
# Each seed in the (quoted, space separated) list runs sequentially (1 xsim job at a time).
# `seed` selects both the page-table vector set generated from the functional
# model (gen_mmu_vectors.cpp) and the TB's -sv_seed (latencies / backpressure).
# The generator is built with Vivado's bundled MinGW c++ (C++17) against
#/{mmu,pt_builder,memory,interleave}.cpp; set CXX to override.
# xelab -O0 mirrors run_tb_l2.sh (xsim 2026.1 optimizer crash workaround).
# Expect NG_MMU_DECODE_TB_PASS and NG_MMU_TB_PASS.
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-${TMPDIR:-/tmp}/ng_tb_mmu}
SEEDS=${3:-1}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
SRC=$ROOT/rtl/gpu
CXX=${CXX:-$(dirname "$BIN")/tps/mingw/10.0.0/win64.o/nt/bin/c++}
[ -x "$CXX" ] || [ -x "$CXX.exe" ] || CXX=c++

mkdir -p "$WORK"
cd "$WORK"

# ---- generator ----------------------------------------------------------------
if [ ! -x ./gen_mmu_vectors.exe ] || [ "$SRC/tb/gen_golden/gen_mmu_vectors.cpp" -nt ./gen_mmu_vectors.exe ]; then
  "$CXX" -std=c++17 -O2 -I"$ROOT/model/include"     "$SRC/tb/gen_golden/gen_mmu_vectors.cpp"     "$ROOT/model/src/mmu.cpp" "$ROOT/model/src/pt_builder.cpp"     "$ROOT/model/src/memory.cpp" "$ROOT/model/src/interleave.cpp"     -o gen_mmu_vectors.exe
fi
python "$SRC/tb/gen_golden/gen_mmu_decode_vec.py" mmu_decode.hex

PKG=$SRC/mmu/ng_mmu_pkg.sv
# ---- 1) entry decode vs ptegen reference --------------------------------------
"$BIN/xvlog.bat" -sv "$PKG" "$SRC/tb/tb_mmu_decode.sv" > xvlog_dec.out 2>&1 \
  || { cat xvlog_dec.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_mmu_decode -s sim_dec > xelab_dec.out 2>&1 \
  || { cat xelab_dec.out; exit 1; }
"$BIN/xsim.bat" sim_dec -runall > xsim_dec.out 2>&1 || true
grep -E "TB_FAIL|PASS" xsim_dec.out
grep -q NG_MMU_DECODE_TB_PASS xsim_dec.out

# ---- 2) MMU top ----------------------------------------------------------------
"$BIN/xvlog.bat" -sv "$PKG" \
  "$SRC/mmu/ng_mmu_tlb.sv" "$SRC/mmu/ng_mmu_walker.sv" "$SRC/mmu/ng_mmu_regs.sv" \
  "$SRC/mmu/ng_mmu.sv" "$SRC/tb/tb_mmu.sv" > xvlog_mmu.out 2>&1 \
  || { cat xvlog_mmu.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_mmu -s sim_mmu > xelab_mmu.out 2>&1 \
  || { cat xelab_mmu.out; exit 1; }
for SEED in $SEEDS; do
  ./gen_mmu_vectors.exe "$SEED" mmu_vec.hex
  "$BIN/xsim.bat" sim_mmu -runall -sv_seed "$SEED" > "xsim_mmu_$SEED.out" 2>&1 || true
  echo "== seed $SEED =="
  grep -E "tb_mmu knobs|replay|directed|TB_FAIL|PASS" "xsim_mmu_$SEED.out"
  grep -q NG_MMU_TB_PASS "xsim_mmu_$SEED.out"
done
