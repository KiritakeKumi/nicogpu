#!/usr/bin/env bash
# the multi-SP core (ng_core: NSP x ng_sp_top + ng_core_wd + per-SP L1D -> NoC -> 4 L2 slices -> DDR models)
# against dispatch bundles exported with the model as one core of NSP SPs (tools/dispatch_bundle/mk_core_bundles.sh).
#
#   rtl/gpu/tb/run_tb_core.sh <bundle_dir...>
#
# Relative bundle paths are taken from the repo root.  Env:
#   SIM=xsim|verilator     simulator (default xsim)
#   WORK=<dir>             scratch directory (default build/gpu/tb_core[_vl])
#   VIVADO_BIN=<dir>       xsim tools (default C:/AMDDesignTools/2026.1/Vivado/bin)
#   NG_MODE=run|reset      run: one dispatch, compare; reset: core_reset at each NG_RESET_AT commit count of a fresh
#                          start, then the checked run (as run_tb_sp_top.sh)
#   NG_RESET_AT="n ..."    commit counts for the aborts (default "40")
#   NG_CFG="NSP:MAXLAT:SFU:L2SETS:L2WAYS"   SPs (must match the bundles' NG_CORE_NSP), random MMU / uc / DDR latency
#                          bound, ng_sp_top EN_SFU, L2 slice sets / ways (default "4:3:1:16:2": 4 KiB slices, so the
#                          bundles' working sets evict constantly)
#   NG_LINT=1              only compile / elaborate
#   VERILATOR / VL_MAKEFLAGS / NJOBS   as run_tb_sp_top.sh
# Output: one NG_CORE_PASS <bundle> line per bundle, NG_CORE_TB_ALL_OK at the end.
# xsim: xelab -O1 (NG_XELAB_O; the L2-only benches use -O0 for an xsim 2026.1 optimizer crash at the default level).
set -eu
. "$(dirname "$0")/../../shell/tb/tb_lib.sh"
SIM=${SIM:-xsim}
BIN=$(vivado_bin)
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
RTL=$ROOT/rtl/gpu
if [ "$SIM" = verilator ]; then WORK=${WORK:-$ROOT/build/gpu/tb_core_vl}; else WORK=${WORK:-$ROOT/build/gpu/tb_core}; fi
MODE=${NG_MODE:-run}
CFG=${NG_CFG:-4:3:1:16:2}
RESET_AT=${NG_RESET_AT:-40}
case $WORK in /*|?:*) ;; *) WORK=$ROOT/$WORK ;; esac
mkdir -p "$WORK"

IFS=: read -r NSPV MAXLAT ENSFU L2S L2W <<EOF
$CFG
EOF

# the SP sources: rtl/gpu/sp/sp_files.f (noc_pkg ... sp_top) without ng_lsu_kn (the core runs LSU_K = 1 only); ng_l2_addrmap and
# the CP package / port / copy first (ng_sp_top imports ng_cp_pkg)
SRCS="$RTL/l2/ng_l2_addrmap.sv $RTL/cp/ng_cp_pkg.sv $RTL/cp/ng_cp_port.sv $RTL/cp/ng_cp_copy.sv \
$(flist "$RTL/sp/sp_files.f" | grep -v '/ng_lsu_kn\.sv$' | tr '\n' ' ') \
$RTL/noc/ng_noc_router.sv $RTL/noc/ng_noc_mesh.sv $RTL/noc/ng_noc_ep.sv \
$RTL/l2/ng_l2_slice.sv $RTL/prim/ng_prim_l2ram.sv $RTL/lsu/ng_lsu_noc_adapter.sv \
$RTL/core/ng_core_arb.sv $RTL/core/ng_core_wd.sv $RTL/core/ng_core.sv \
$RTL/tb/tb_l2_axi_mem_model.sv $RTL/tb/tb_core.sv"

cd "$WORK"
{
  echo "\`define NG_TB_NSP $NSPV"
  echo "\`define NG_TB_MAXLAT $MAXLAT"
  echo "\`define NG_TB_EN_SFU $ENSFU"
  echo "\`define NG_TB_L2_SETS $L2S"
  echo "\`define NG_TB_L2_WAYS $L2W"
} > tb_core_cfg.svh

if [ "$SIM" = vlint ]; then
  # shellcheck disable=SC2086
  ${VERILATOR:-verilator} --lint-only --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY -Wno-TIMESCALEMOD \
    --unroll-count 4096 --unroll-stmts 4000000 +define+NG_SP_ASSERT --top-module tb_core -I. $SRCS > vl_lint.out 2>&1 \
    || { grep -E "%Error" vl_lint.out | head -40; exit 1; }
  grep -E "%Warning-(WIDTH|UNDRIVEN|MULTIDRIVEN|PINMISSING|SELRANGE|IMPLICIT)" vl_lint.out | grep -E "core|tb_core" | head -40 || true
  echo "NG_CORE_VLINT_OK ($CFG)"
  exit 0
elif [ "$SIM" = verilator ]; then
  rm -rf obj_core
  # shellcheck disable=SC2086
  ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} ${NJOBS:+-j "$NJOBS"} --binary --timing -Wno-fatal -Wno-lint \
    -Wno-style -Wno-INITIALDLY -Wno-TIMESCALEMOD --unroll-count 4096 --unroll-stmts 4000000 \
    +define+NG_SP_ASSERT --top-module tb_core -I. -Mdir obj_core $SRCS > vl_build.out 2>&1 \
    || { grep -E "%Error" vl_build.out | head -40; tail -20 vl_build.out; exit 1; }
else
  # shellcheck disable=SC2086
  "$BIN/xvlog.bat" -sv -d NG_SP_ASSERT -i . $SRCS > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
  "$BIN/xelab.bat" -debug off -O${NG_XELAB_O:-1} tb_core -s tb_core > xelab.out 2>&1 \
    || { grep -E "ERROR" xelab.out | head -40; exit 1; }
fi
if [ "${NG_LINT:-0}" = 1 ]; then
  echo "NG_CORE_LINT_OK ($SIM $CFG)"
  exit 0
fi

[ $# -ge 1 ] || { echo "usage: run_tb_core.sh <bundle_dir...>"; exit 2; }
set -- $(for b in "$@"; do case $b in /*|?:*) echo "$b" ;; *) echo "$ROOT/$b" ;; esac; done)
for b in "$@"; do
  [ -f "$b/meta.hex" ] || { echo "not a bundle: $b"; exit 2; }
  bw=$b
  if command -v cygpath >/dev/null 2>&1; then bw=$(cygpath -m "$b"); fi
  name=$(basename "$b")
  n=0; pts=""
  for r in $RESET_AT; do n=$((n + 1)); pts="$pts $r"; done
  printf '%s\n%s\n%s%s\n' "$bw" "$MODE" "$n" "$pts" > tb_core_run.txt
  if [ "$SIM" = verilator ]; then
    ./obj_core/Vtb_core > "sim_$name.out" 2>&1 || true
  else
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" tb_core > "sim_$name.out" 2>&1 || true
  fi
  grep -E "TB_ERROR|MISMATCH|NG_CORE|NOTE|FAIL|Error|error" "sim_$name.out" | head -40 || true
  grep -q "NG_CORE_PASS" "sim_$name.out" || { echo "--- sim_$name.out (tail) ---"; tail -30 "sim_$name.out"; exit 1; }
done
echo "NG_CORE_TB_ALL_OK ($SIM $MODE $CFG)"
