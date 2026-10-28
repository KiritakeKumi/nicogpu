#!/usr/bin/env bash
# SP top level (ng_sp_top + ng_wd + ng_sp_icache + units) against
# dispatch bundles exported from the functional model (tools/dispatch_bundle).
#
#   rtl/gpu/tb/run_tb_sp_top.sh <bundle_dir...>
#
#   bundle_dir: directories written by tools/dispatch_bundle (bundle_run.exe);
#                tools/dispatch_bundle/mkbundles.sh builds the kernel set into
#                build/gpu/bundles/<name>.  Relative paths are taken from the repo root.
# Env:
#   SIM=xsim|verilator     simulator (default xsim)
#   WORK=<dir>             scratch directory (default build/gpu/tb_sp_top[_vl])
#   VIVADO_BIN=<dir>       xsim tools (default C:/AMDDesignTools/2026.1/Vivado/bin)
#   NG_MODE=run|reset|scrub run: one dispatch, compare with the bundle;
#                          reset: core_reset at each NG_RESET_AT point of a fresh
#                          start (abort / sp_idle checks), then a full checked run
#                          scrub (bundle k_scrub only, implies NG_SCRUB=1): after the checked run, residue planted
#                          the register file / shared array / L1D is probed by dispatches without / with an owner change
#                          (ng_cp_port scrub -> ng_wd) and after CACHE(INV_L1) packets (P_SP_INV), then a scrubbing
#                          dispatch is checked against the model
#   NG_SCRUB=0|1           ng_cp_port SCRUB (generate wd_req.scrub from owner changes; default 0 = previous, cycle for cycle)
#   NG_RESET_AT="n ..."    commit counts for the aborts (default "40")
#   NG_CFG="NL:MAXLAT:EN"  register-file lanes, random MMU/fabric latency bound,
#                          1 = instantiate SFU+XLANE+FP64 (default "16:3:0")
#   NG_CLEAR=0|1           ng_sp_top CLEAR_VREGS (default 0: v1..v127 undefined at warp start)
#   NG_BDEPTH=<n>          ng_sp_top VALU_BDEPTH (VALU back-half contexts, default 3; 1 =)
#   NG_BRND=0|1            ng_sp_top BR_NODRAIN (pure branches beside a busy VALU, default 1; 0 =)
#   NG_IC=<n>              ng_sp_top IC_LINES (L0 I$ lines, default 32 = KiB 4-way; 8 = the I$)
#   NG_F16=0|1             ng_sp_top EN_VALU_F16CVT (VALU fcvt.f16 / fcvt.f32.f16 / pkcvt.f16, default 1)
#   NG_LSU_K=<n>           ng_sp_top LSU_K (1 = ng_lsu, default; 2 / 4 / 8 = ng_lsu_kn ring entries)
#   NG_MEMOUT=<n>          ng_sp_top MEM_OUT (1 = serial fabric stub, default; 2..16 = tagged multi-outstanding
#                          fabric model with the slot protocol of ng_rsp_fab_ts)
#   NG_FLAT="lo:hi"        MEMOUT > 1: fabric latency range in clocks (default 0:MAXLAT; e.g. 200:380 ~ the 2.9 us
#                          board average); a 2 % tail adds 4 * hi + 50
#   NG_ORPHAN=0|1          MEMOUT > 1: 1 (default) = sp_rst = !crst_n orphans requests in flight (as rm_sp),
#                          0 = their answers are delivered after a reset (exercises the mport drop path)
#   NG_TIMEOUT=<n>         clocks to wait for wd_done (default 4000000; raise it for large NG_FLAT latencies)
#   NG_REUSE_BUILD=1       skip the compile when WORK already holds a successful build (the caller guarantees the same
#                          configuration; arch_seq runs many bundles per build)
#   NG_KEEPGOING=1         a failing bundle does not stop the run (exit 1 at the end, NG_SP_TOP_TB_FAILED)
#   NG_LINT=1              only compile/elaborate
#   VERILATOR / VL_MAKEFLAGS  as in run_tb_lsu.sh (Windows: verilator.exe, CXX=g++ ...)
#   NJOBS=<n>              Verilator -j (verilate + C++ build jobs; unset = Verilator default)
# Output: one NG_SP_TOP_PASS <bundle> line per bundle, NG_SP_TOP_TB_ALL_OK at the end.
# Notes: compile-time knobs go through tb_sp_top_cfg.svh and run-time ones
#   through tb_sp_top_run.txt (xsim.bat mangles '=' in arguments, so there are
#   no plusargs / -d values on the command line).  One build serves all bundles.
set -eu
. "$(dirname "$0")/../../shell/tb/tb_lib.sh"
SIM=${SIM:-xsim}
BIN=$(vivado_bin)
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
RTL=$ROOT/rtl/gpu
if [ "$SIM" = verilator ]; then WORK=${WORK:-$ROOT/build/gpu/tb_sp_top_vl}; else WORK=${WORK:-$ROOT/build/gpu/tb_sp_top}; fi
MODE=${NG_MODE:-run}
CFG=${NG_CFG:-16:3:0}
RESET_AT=${NG_RESET_AT:-40}
case $WORK in /*|?:*) ;; *) WORK=$ROOT/$WORK ;; esac
mkdir -p "$WORK"

IFS=: read -r NLV MAXLAT ENX <<EOF
$CFG
EOF
NG_FLAT=${NG_FLAT:-0:$MAXLAT}

# the SP sources: rtl/gpu/sp/sp_files.f (noc_pkg ... sp_top); the CP package / port / copy (ng_sp_top imports ng_cp_pkg)
SRCS="$RTL/cp/ng_cp_pkg.sv $RTL/cp/ng_cp_port.sv $RTL/cp/ng_cp_copy.sv $(flist "$RTL/sp/sp_files.f" | tr '\n' ' ') $RTL/tb/tb_sp_top.sv"

SCRUBV=${NG_SCRUB:-0}
[ "$MODE" = scrub ] && SCRUBV=1
cd "$WORK"
{
  echo "\`define NG_TB_NL $NLV"
  echo "\`define NG_TB_MAXLAT $MAXLAT"
  echo "\`define NG_TB_EN_EXTRA $ENX"
  echo "\`define NG_TB_CLEAR ${NG_CLEAR:-0}"
  echo "\`define NG_TB_BDEPTH ${NG_BDEPTH:-3}"
  echo "\`define NG_TB_BRND ${NG_BRND:-1}"
  echo "\`define NG_TB_IC_LINES ${NG_IC:-32}"
  echo "\`define NG_TB_F16CVT ${NG_F16:-1}"
  echo "\`define NG_TB_LSU_K ${NG_LSU_K:-1}"
  echo "\`define NG_TB_MEMOUT ${NG_MEMOUT:-1}"
  echo "\`define NG_TB_FLAT_LO ${NG_FLAT%%:*}"
  echo "\`define NG_TB_FLAT_HI ${NG_FLAT##*:}"
  echo "\`define NG_TB_ORPHAN ${NG_ORPHAN:-1}"
  echo "\`define NG_TB_SCRUB ${SCRUBV}"
  echo "\`define NG_TB_TIMEOUT ${NG_TIMEOUT:-4000000}"
} > tb_sp_top_cfg.svh

if [ "${NG_REUSE_BUILD:-0}" = 1 ] && [ -e build_sp_top.ok ]; then
  :   # same compile-time configuration as the build that wrote build_sp_top.ok (the caller's WORK dir is per configuration)
elif [ "$SIM" = verilator ]; then
  rm -rf obj_sp_top build_sp_top.ok
  ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} ${NJOBS:+-j "$NJOBS"} --binary --timing -Wno-fatal -Wno-lint \
    -Wno-style -Wno-INITIALDLY -Wno-TIMESCALEMOD --unroll-count 4096 --unroll-stmts 4000000 \
    +define+NG_SP_ASSERT --top-module tb_sp_top -I. -Mdir obj_sp_top $SRCS > vl_build.out 2>&1 \
    || { grep -E "%Error" vl_build.out | head -40; tail -20 vl_build.out; exit 1; }
else
  "$BIN/xvlog.bat" -sv -d NG_SP_ASSERT -i . $SRCS > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
  "$BIN/xelab.bat" -debug off -O1 tb_sp_top -s tb_sp_top > xelab.out 2>&1 \
    || { grep -E "ERROR" xelab.out | head -40; exit 1; }
fi
[ "${NG_LINT:-0}" = 1 ] || touch build_sp_top.ok
if [ "${NG_LINT:-0}" = 1 ]; then
  echo "NG_SP_TOP_LINT_OK ($SIM $CFG)"
  exit 0
fi

[ $# -ge 1 ] || { echo "usage: run_tb_sp_top.sh <bundle_dir...>"; exit 2; }
set -- $(for b in "$@"; do case $b in /*|?:*) echo "$b" ;; *) echo "$ROOT/$b" ;; esac; done)
nfail=0
for b in "$@"; do
  [ -f "$b/meta.hex" ] || { echo "not a bundle: $b"; exit 2; }
  bw=$b
  if command -v cygpath >/dev/null 2>&1; then bw=$(cygpath -m "$b"); fi
  name=$(basename "$b")
  n=0; pts=""
  for r in $RESET_AT; do n=$((n + 1)); pts="$pts $r"; done
  printf '%s\n%s\n%s%s\n' "$bw" "$MODE" "$n" "$pts" > tb_sp_top_run.txt
  if [ "$SIM" = verilator ]; then
    ./obj_sp_top/Vtb_sp_top > "sim_$name.out" 2>&1 || true
  else
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" tb_sp_top > "sim_$name.out" 2>&1 || true
  fi
  grep -E "TB_ERROR|MISMATCH|NG_SP_TOP|NOTE|FAIL" "sim_$name.out" | head -40 || true
  if ! grep -q "NG_SP_TOP_PASS" "sim_$name.out"; then
    echo "--- sim_$name.out (tail) ---"; tail -30 "sim_$name.out"
    [ "${NG_KEEPGOING:-0}" = 1 ] || exit 1
    nfail=$((nfail + 1))
  fi
done
[ "$nfail" = 0 ] || { echo "NG_SP_TOP_TB_FAILED ($nfail bundles)"; exit 1; }
echo "NG_SP_TOP_TB_ALL_OK ($SIM $MODE $CFG)"
