#!/usr/bin/env bash
# the L2 / NoC / MMU unit testbenches in one run (the CI entry `l2noc` of .github/workflows/ci-rtl-heavy.yml;
# Verilator by default -- the per-unit xsim scripts run_tb_l2.sh / run_tb_l2_p13b.sh / run_tb_noc.sh / run_tb_mmu.sh /
# run_tb_lsu_noc_l2.sh stay the local reference).
#
#   rtl/gpu/tb/run_tb_l2noc.sh [work_dir] ["seed ..."]        (default build/gpu/tb_l2noc, seeds "1")
#
# Benches (expect token):
#   tb_l2_amo_alu   AMO ALU vectors                                          NG_L2_AMO_TB_PASS
#   tb_l2_slice     slice: hit / miss / evict / AMO / errors / golden map     NG_L2_TB_PASS
#   tb_l2_fatomic   FP32 atomics vs sim/l2/gen_fatomic_vec.py (per seed)      NG_L2_FATOMIC_TB_PASS
#   tb_l2_maint     inv.l2 / wb.l2                                           NG_L2_MAINT_TB_PASS
#   tb_noc_mesh     3x3 all-to-all soak, default depths and 3 / 3 / 1        NG_NOC_TB_PASS (x2)
#   tb_noc_l2map    3x2 mesh + 4 slices, golden map, linearisation           NG_NOC_L2MAP_TB_PASS
#   tb_lsu_noc_l2   L1D -> adapter -> NoC -> slices (run_tb_lsu_noc_l2.sh)  NG_LSU_NOC_L2_TB_PASS
#   tb_mmu_decode   PTE decode vs gen_mmu_decode_vec.py                      NG_MMU_DECODE_TB_PASS
#   tb_mmu          MMU top vs model vectors (gen_mmu_vectors.cpp, per seed) NG_MMU_TB_PASS
# Env: SIM=verilator|xsim, VERILATOR, VL_MAKEFLAGS, NJOBS (Verilator -j), VIVADO_BIN, PYTHON (default python3 / python),
#      CXX (model vector generator, default c++), ONLY="tb_... ..." (subset).
# Prints NG_L2NOC_ALL_OK at the end.
set -eu
SIM=${SIM:-verilator}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
WORK=${1:-$ROOT/build/gpu/tb_l2noc}
SEEDS=${2:-1}
BIN=${VIVADO_BIN:-C:/AMDDesignTools/2026.1/Vivado/bin}
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
SRC=$ROOT/rtl/gpu
if command -v cygpath >/dev/null 2>&1; then SRC=$(cygpath -m "$SRC"); fi
if [ -z "${PYTHON:-}" ]; then
  if command -v python3 >/dev/null 2>&1 && python3 -c "import sys" >/dev/null 2>&1; then PYTHON=python3; else PYTHON=python; fi
fi
CXX=${CXX:-c++}
mkdir -p "$WORK"
cd "$WORK"
[ -f "$SRC/tb/golden/l2_map_count.hex" ] || (cd "$ROOT" && "$PYTHON" rtl/gpu/tb/gen_golden/gen_interleave_golden.py)
mkdir -p golden
cp "$SRC"/tb/golden/*.hex golden/

L2="$SRC/noc/ng_noc_pkg.sv $SRC/l2/ng_l2_addrmap.sv $SRC/l2/ng_l2_amo_alu.sv $SRC/l2/ng_l2_slice.sv \
$SRC/prim/ng_prim_l2ram.sv $SRC/tb/tb_l2_axi_mem_model.sv"
NOC="$SRC/noc/ng_noc_pkg.sv $SRC/noc/ng_noc_router.sv $SRC/noc/ng_noc_mesh.sv $SRC/noc/ng_noc_ep.sv"
MMU="$SRC/mmu/ng_mmu_pkg.sv $SRC/mmu/ng_mmu_tlb.sv $SRC/mmu/ng_mmu_walker.sv $SRC/mmu/ng_mmu_regs.sv $SRC/mmu/ng_mmu.sv"

want() { [ -z "${ONLY:-}" ] || case " $ONLY " in *" $1 "*) return 0 ;; *) return 1 ;; esac; }

# build <name> <top> <sources...>: compile into obj_<name> (Verilator) / snapshot sim_<name> (xsim)
build() {
  local name=$1 top=$2; shift 2
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$name"
    # shellcheck disable=SC2086
    ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} ${NJOBS:+-j "$NJOBS"} --binary --timing \
      -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY -Wno-TIMESCALEMOD --unroll-count 4096 --unroll-stmts 4000000 \
      --top-module "$top" -I. -Mdir "obj_$name" "$@" > "vl_$name.build" 2>&1 \
      || { grep -E "%Error" "vl_$name.build" | head -30; tail -20 "vl_$name.build"; return 1; }
  else
    "$BIN/xvlog.bat" -sv "$@" > "xvlog_$name.out" 2>&1 || { grep ERROR "xvlog_$name.out" | head -30; return 1; }
    "$BIN/xelab.bat" -debug off -O0 "$top" -s "sim_$name" > "xelab_$name.out" 2>&1 \
      || { grep ERROR "xelab_$name.out" | head -30; return 1; }
  fi
}
# run <name> <top> <seed> <token>
run() {
  local name=$1 top=$2 seed=$3 tok=$4 out="sim_$1_$3.out"
  if [ "$SIM" = verilator ]; then
    "./obj_$name/V$top" "+verilator+seed+$seed" "+SEED=$seed" > "$out" 2>&1 || true
  else
    "$BIN/xsim.bat" "sim_$name" -runall -sv_seed "$seed" > "$out" 2>&1 || true
  fi
  grep -E "TB_FAIL|_PASS" "$out" | head -10 || true
  grep -q "$tok" "$out" || { echo "--- $out (tail) ---"; tail -30 "$out"; return 1; }
}

if want tb_l2_amo_alu; then
  echo "== tb_l2_amo_alu"
  build amo tb_l2_amo_alu "$SRC/noc/ng_noc_pkg.sv" "$SRC/l2/ng_l2_amo_alu.sv" "$SRC/tb/tb_l2_amo_alu.sv"
  for s in $SEEDS; do run amo tb_l2_amo_alu "$s" NG_L2_AMO_TB_PASS; done
fi
if want tb_l2_slice; then
  echo "== tb_l2_slice"
  # shellcheck disable=SC2086
  build slice tb_l2_slice $L2 "$SRC/tb/tb_l2_slice.sv"
  for s in $SEEDS; do run slice tb_l2_slice "$s" NG_L2_TB_PASS; done
fi
if want tb_l2_fatomic; then
  echo "== tb_l2_fatomic"
  "$PYTHON" "$ROOT/sim/l2/gen_fatomic_vec.py" --selftest > fa_selftest.out
  # shellcheck disable=SC2086
  build fatomic tb_l2_fatomic $L2 "$SRC/tb/tb_l2_fatomic.sv"
  for s in $SEEDS; do
    "$PYTHON" "$ROOT/sim/l2/gen_fatomic_vec.py" fa_vec.hex --seed "$s" --rand "${FA_RAND:-20000}" > /dev/null
    run fatomic tb_l2_fatomic "$s" NG_L2_FATOMIC_TB_PASS
  done
fi
if want tb_l2_maint; then
  echo "== tb_l2_maint"
  # shellcheck disable=SC2086
  build maint tb_l2_maint $L2 "$SRC/tb/tb_l2_maint.sv"
  for s in $SEEDS; do run maint tb_l2_maint "$s" NG_L2_MAINT_TB_PASS; done
fi
if want tb_noc_mesh; then
  echo "== tb_noc_mesh (default depths)"
  # shellcheck disable=SC2086
  build mesh tb_noc_mesh $NOC "$SRC/tb/tb_noc_mesh.sv"
  for s in $SEEDS; do run mesh tb_noc_mesh "$s" NG_NOC_TB_PASS; done
  echo "== tb_noc_mesh (router FIFO 3, receive credits 3, TXQ_DEPTH 1)"
  printf '`define NG_NOC_TB_BUF 3\n`define NG_NOC_TB_LOC 3\n`define NG_NOC_TB_TXQ 1\n' > noc_depths3.svh
  # shellcheck disable=SC2086
  build mesh3 tb_noc_mesh noc_depths3.svh $NOC "$SRC/tb/tb_noc_mesh.sv"
  for s in $SEEDS; do run mesh3 tb_noc_mesh "$s" NG_NOC_TB_PASS; done
fi
if want tb_noc_l2map; then
  echo "== tb_noc_l2map"
  # shellcheck disable=SC2086
  build l2map tb_noc_l2map $NOC $L2 "$SRC/tb/tb_noc_l2map.sv"
  for s in $SEEDS; do run l2map tb_noc_l2map "$s" NG_NOC_L2MAP_TB_PASS; done
fi
if want tb_lsu_noc_l2; then
  echo "== tb_lsu_noc_l2"
  SIM=$SIM bash "$SRC/tb/run_tb_lsu_noc_l2.sh" "$WORK/lsu_noc_l2" "$BIN" "$SEEDS" | grep -E "PASS|ALL_OK|TB_FAIL"
fi
if want tb_mmu_decode || want tb_mmu; then
  echo "== tb_mmu_decode / tb_mmu"
  "$PYTHON" "$SRC/tb/gen_golden/gen_mmu_decode_vec.py" mmu_decode.hex > /dev/null
  build mmudec tb_mmu_decode "$SRC/mmu/ng_mmu_pkg.sv" "$SRC/tb/tb_mmu_decode.sv"
  run mmudec tb_mmu_decode 1 NG_MMU_DECODE_TB_PASS
  "$CXX" -std=c++17 -O2 -I"$ROOT/model/include" "$SRC/tb/gen_golden/gen_mmu_vectors.cpp" "$ROOT/model/src/mmu.cpp" \
    "$ROOT/model/src/pt_builder.cpp" "$ROOT/model/src/memory.cpp" "$ROOT/model/src/interleave.cpp" -o gen_mmu_vectors
  # shellcheck disable=SC2086
  build mmu tb_mmu $MMU "$SRC/tb/tb_mmu.sv"
  for s in $SEEDS; do
    ./gen_mmu_vectors "$s" mmu_vec.hex
    run mmu tb_mmu "$s" NG_MMU_TB_PASS
  done
fi
echo "NG_L2NOC_ALL_OK ($SIM, seeds $SEEDS)"
