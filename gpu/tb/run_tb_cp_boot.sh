#!/usr/bin/env bash
# run the CP boot loader unit test (ng_cp_boot) with xsim.
#   rtl/gpu/tb/run_tb_cp_boot.sh [vivado_bin_dir] [work_dir] ["seed ..."]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, $TMPDIR/ng_tb_cp_boot, seed 1.  Needs python (tools/ngcf).
# The seed changes the generated images' contents and the memory model's latency / back-pressure.
# xelab -O0 mirrors the other run_tb_*.sh.  Expect NG_CP_BOOT_TB_PASS.
set -eu
set -o pipefail
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-${TMPDIR:-/tmp}/ng_tb_cp_boot}
SEEDS=${3:-1}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
RTL=$ROOT/rtl/gpu
PY=${PYTHON:-python}
mkdir -p "$WORK"
cd "$WORK"
for SEED in $SEEDS; do
  rm -rf cases_cur
  "$PY" "$ROOT/tools/ngcf/gen_boot_cases.py" cases_cur "$SEED"
  echo "$SEED $((1 + SEED % 4)) $((100 - (SEED % 3) * 30))" > cases_cur/params.txt
  "$BIN/xvlog.bat" -sv "$RTL/cp/ng_cp_pkg.sv" "$RTL/cp/ng_cp_boot.sv" "$RTL/tb/tb_cp_boot.sv" > xvlog.out 2>&1 \
    || { cat xvlog.out; exit 1; }
  "$BIN/xelab.bat" -debug off -O0 tb_cp_boot -s "sim_$SEED" > xelab.out 2>&1 || { cat xelab.out; exit 1; }
  "$BIN/xsim.bat" "sim_$SEED" -runall > "xsim_$SEED.out" 2>&1 || true
  echo "== boot TB seed $SEED =="
  grep -E "TB_FAIL|done|PASS|FAIL" "xsim_$SEED.out" | head -40
  grep -q NG_CP_BOOT_TB_PASS "xsim_$SEED.out"
done
