#!/usr/bin/env bash
# run the NoC testbenches (mesh all-to-all soak + L2 integration /
# golden interleave mapping) with xsim.
#   rtl/gpu/tb/run_tb_noc.sh [vivado_bin_dir] [work_dir] [sv_seed]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, build/tb_noc, seed 1.
# Regenerates the golden interleave vectors if missing and copies them into
# the work dir (xsim.bat mangles -testplusarg, so the TB default "golden"
# relative path is used).
# xelab -O0 works around an xsim 2026.1 optimizer crash (EXCEPTION_ACCESS_
# VIOLATION) on this design.
# Rerun with different SEED values for multi-seed soak (the TBs also vary
# traffic internally per round).
# Expect NG_NOC_TB_PASS (twice: default depths, then the depths) and NG_NOC_L2MAP_TB_PASS.
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-build/tb_noc}
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

# ---- 1) mesh all-to-all soak -------------------------------------------------
"$BIN/xvlog.bat" -sv \
  "$SRC/noc/ng_noc_pkg.sv" "$SRC/noc/ng_noc_router.sv" "$SRC/noc/ng_noc_mesh.sv" \
  "$SRC/noc/ng_noc_ep.sv" "$SRC/tb/tb_noc_mesh.sv" \
  > xvlog_mesh.out 2>&1 || { cat xvlog_mesh.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_noc_mesh -s sim_mesh > xelab_mesh.out 2>&1 \
  || { cat xelab_mesh.out; exit 1; }
"$BIN/xsim.bat" sim_mesh -runall -sv_seed "$SEED" > xsim_mesh.out 2>&1 || true
grep -E "TB_FAIL|PASS" xsim_mesh.out
grep -q NG_NOC_TB_PASS xsim_mesh.out

# ---- 1b) the same soak with router FIFO depth 3, receive credits 3 (RX FIFO 6) and TXQ_DEPTH 1 ----------
# (non-power-of-two FIFOs and the single-entry TX queue; before their pointers ran past the arrays)
# (the depths come from a define file compiled first: xvlog.bat mangles '=' in -d values)
printf '`define NG_NOC_TB_BUF 3\n`define NG_NOC_TB_LOC 3\n`define NG_NOC_TB_TXQ 1\n' > noc_depths3.svh
"$BIN/xvlog.bat" -sv noc_depths3.svh \
  "$SRC/noc/ng_noc_pkg.sv" "$SRC/noc/ng_noc_router.sv" "$SRC/noc/ng_noc_mesh.sv" \
  "$SRC/noc/ng_noc_ep.sv" "$SRC/tb/tb_noc_mesh.sv" \
  > xvlog_mesh3.out 2>&1 || { cat xvlog_mesh3.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_noc_mesh -s sim_mesh3 > xelab_mesh3.out 2>&1 \
  || { cat xelab_mesh3.out; exit 1; }
"$BIN/xsim.bat" sim_mesh3 -runall -sv_seed "$SEED" > xsim_mesh3.out 2>&1 || true
grep -E "TB_FAIL|PASS" xsim_mesh3.out
grep -q NG_NOC_TB_PASS xsim_mesh3.out

# ---- 2) NoC + 4x L2 slice integration ----------------------------------------
"$BIN/xvlog.bat" -sv \
  "$SRC/noc/ng_noc_pkg.sv" "$SRC/noc/ng_noc_router.sv" "$SRC/noc/ng_noc_mesh.sv" \
  "$SRC/noc/ng_noc_ep.sv" \
  "$SRC/l2/ng_l2_addrmap.sv" "$SRC/l2/ng_l2_amo_alu.sv" "$SRC/l2/ng_l2_slice.sv" \
  "$SRC/prim/ng_prim_l2ram.sv" \
  "$SRC/tb/tb_l2_axi_mem_model.sv" "$SRC/tb/tb_noc_l2map.sv" \
  > xvlog_l2map.out 2>&1 || { cat xvlog_l2map.out; exit 1; }
"$BIN/xelab.bat" -debug off -O0 tb_noc_l2map -s sim_l2map > xelab_l2map.out 2>&1 \
  || { cat xelab_l2map.out; exit 1; }
"$BIN/xsim.bat" sim_l2map -runall -sv_seed "$SEED" > xsim_l2map.out 2>&1 || true
grep -E "TB_FAIL|PASS" xsim_l2map.out
grep -q NG_NOC_L2MAP_TB_PASS xsim_l2map.out
