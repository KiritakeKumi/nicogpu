#!/usr/bin/env bash
# run the ng_rcp_dma testbench with xsim.
#   rtl/shell/rm_cp/tb/run_tb_rcp_dma.sh [vivado_bin_dir] [work_dir] ["seed ..."]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, $TMPDIR/ng_tb_rcp_dma, seed 1.  Env: DMA_SRC overrides the DUT file
# (default rtl/shell/rm_cp/ng_rcp_dma.sv; rtl/shell/rm_cp/tb/ng_rcp_dma_stub.sv only proves the tb elaborates), ITERS = random loop length.
# Expect NG_RCP_DMA_TB_PASS for every seed.
set -eu
set -o pipefail
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-${TMPDIR:-/tmp}/ng_tb_rcp_dma}
SEEDS=${3:-1}
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
DMA_SRC=${DMA_SRC:-$ROOT/rtl/shell/rm_cp/ng_rcp_dma.sv}
ITERS=${ITERS:-150}
mkdir -p "$WORK"
cd "$WORK"
"$BIN/xvlog.bat" -sv "$ROOT/rtl/gpu/mmu/ng_mmu_pkg.sv" "$DMA_SRC" "$ROOT/rtl/shell/rm_cp/tb/tb_rcp_dma.sv" > xvlog.out 2>&1 \
  || { cat xvlog.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_rcp_dma -s sim_rcp_dma > xelab.out 2>&1 || { cat xelab.out; exit 1; }
RC=0
for SEED in $SEEDS; do
  echo "$SEED $ITERS" > params.txt
  "$BIN/xsim.bat" sim_rcp_dma -runall > "xsim_$SEED.out" 2>&1 || true
  echo "== rcp_dma TB seed $SEED =="
  grep -E "^TEST|TB_FAIL|SUMMARY|NG_RCP_DMA_TB" "xsim_$SEED.out" | head -80
  grep -q NG_RCP_DMA_TB_PASS "xsim_$SEED.out" || RC=1
done
exit $RC
