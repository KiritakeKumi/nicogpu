#!/usr/bin/env bash
# week-1 acceptance suite for ng_sp_top (one command):
#   1. build the bundle exporter and export every kernel bundle
#      (tools/dispatch_bundle/mkbundles.sh -> build/gpu/bundles/<name>), plus the ngcomp memory-model
#      bundles (tools/dispatch_bundle/mk_p135_bundles.py -> build/gpu/bundles_p135/p135_mm_*);
#   2. xsim, NL=16, latency 0..3, no optional units: every week-1 bundle (+ the shared/atomic bundles);
#   3. xsim, reset mode: repeated core_reset aborts (fabric / MMU in flight), then full run;
#   4. xsim, NL=32, latency 0..8, SFU+XLANE+FP64 attached: every bundle + k_units
#      + k_sfui (SFU int div/rem edge table) + k_sfuf (SFU f32/f64 rcp rsq sqrt fdiv exp2 log2 sin cos) + k_xlane ;
#   4a. xsim, NL=16, latency 0..3, SFU+XLANE+FP64 attached (the rm_sp NL): k_xlane + k_units + a unit-mix subset;
#   4b. xsim, NL=16, CLEAR_VREGS=1: every week-1 bundle (kernels must not depend on initial v values);
#   4c. xsim, NL=16, BR_NODRAIN=0 + VALU_BDEPTH=1 (the fallback parameters);
#   4d. xsim, NL=16, IC_LINES=8 (the KiB fully associative I$; steps 2..4c use the default 32):
#       every week-1 bundle + the large-loop kernels k_bigloop (3.4 KB body) / k_bigloop5k (5.6 KB);
#   4e. xsim, NL=16, EN_VALU_F16CVT=0 (off; steps 2..4d run with it on and include k_f16cvt): a VALU subset;
#   4f. NG_MODE=scrub with k_scrub (LSU_K=1 / 4 / 2, NL=16 / 32): residue visible without an owner change, gone after one;
#   5. (SUITE_VL=1) the same as 2 + 3 with Verilator (run_tb_sp_top_vl.sh).
#   6. LSU_K=2 / 4 with MEM_OUT=8 (tagged multi-outstanding fabric model): run + reset (one reset leg with the
#      answers of orphaned requests delivered, the mport drop path), LSU_K=2 on the serial port (MEM_OUT=1);
#      SUITE_LAT=1 adds board-like fabric latency (200..380 clk_ts) legs for LSU_K=2 / 4.
#   rtl/gpu/tb/run_tb_sp_top_suite.sh
# Prints NG_SP_TOP_SUITE_OK at the end.
set -eu -o pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
TB=$ROOT/rtl/gpu/tb
B=$ROOT/build/gpu/bundles
bash "$ROOT/tools/dispatch_bundle/mkbundles.sh" "$B" > "$ROOT/build/gpu/mkbundles.log" 2>&1 \
  || { tail -20 "$ROOT/build/gpu/mkbundles.log"; exit 1; }
# ngcomp memory-model kernels (device / system scope fences -> cache.wb.l2 / cache.inv.l2 / cache.inv.l1)
B135=$ROOT/build/gpu/bundles_p135
# python3 from PATH; override with NG_PY310
PY=${NG_PY310:-python3}
"$PY" "$ROOT/tools/dispatch_bundle/mk_p135_bundles.py" "$B135" > "$ROOT/build/gpu/mk_p135.log" 2>&1 \
  || { tail -20 "$ROOT/build/gpu/mk_p135.log"; exit 1; }
W1="$B/k_vadd $B/k_vadd_big $B/k_ids $B/k_intops $B/k_trap_explicit $B/k_trap_lsu $B/k_loop"
# shared memory / barrier / atomics (follow-up): per-WG shared windows, shared atomics, global atomics on
# GTT (uncached path) + VRAM buffers, shared bounds trap
SH="$B/k_shared $B/k_shared_tr $B/k_shared_red $B/k_shared_atom $B/k_shared_512 $B/k_shared_odd $B/k_shared_big $B/k_trap_shared $B/k_gatom"
# f32 VALU (EN_VALU_FP): arithmetic mix, compare/select/convert/round, MODE + fflags CSR plumbing, FpException trap
FP="$B/k_fsaxpy $B/k_fsel $B/k_fmode $B/k_trap_fpexc"
FP="$FP $B/k_f16cvt"   # VALU binary16 <-> binary32 conversions (EN_VALU_F16CVT, default on in these runs)
W1="$W1 $SH $FP $B/k_mw $B/k_mw2 $B/k_mw3 $B/k_trap_mwfp $B/k_trap_mwrr $B/k_trap_mwchk"   # k_mw: cross-warp VALU/LSU concurrency (TB asserts LSU accepted under a fixed op)
W1="$W1 $B/k_memord"   # ERRATA v0.1.5 draft: same-warp overlapping ld/st/atom without waitcnt (VRAM, GTT, shared)
W1="$W1 $B/k_brnd $B/k_trap_brur"   # pure branches beside a busy VALU; branch uniform read yielding to a stuck VALU CHK
W1="$W1 $B/k_cacheop"  # cache.inv.l1 / cache.inv.l2 / cache.wb.l2 (TB checks every one reached the L1D / fabric)
W1="$W1 $B/k_scratch $B/k_scratch_gtt"   # dword-interleaved scratch (ERRATA v0.1.6), VRAM + GTT scratch buffer
W1="$W1 $B/k_stream $B/k_stream_rd $B/k_stream_v"   # ngv_membw-like streaming (copy / read), one line per memory instruction, GTT / VRAM
W1="$W1 $B135/p135_mm_dev_mp $B135/p135_mm_dev_acqrel_wg $B135/p135_mm_sys_mp $B135/p135_mm_shared_wg $B135/p135_mm_dedup $B135/p135_mm_subgroup_only"
BIG="$B/k_bigloop $B/k_bigloop5k"   # loop bodies of 27 / 45 I$ lines (run in steps 2 and 4d)
RST="$B/k_vadd_big $B/k_intops $B/k_loop $B/k_shared_atom $B/k_gatom $B/k_fsaxpy $B/k_cacheop"
PTS="1 3 17 29 40 55 90 120 200"

echo "== xsim 16:3:0 run";   NG_CFG=16:3:0 bash "$TB/run_tb_sp_top.sh" $W1 $BIG | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 reset"; NG_CFG=16:3:0 NG_MODE=reset NG_RESET_AT="$PTS" bash "$TB/run_tb_sp_top.sh" $RST \
  | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 32:8:1 run";   NG_CFG=32:8:1 WORK=$ROOT/build/gpu/tb_sp_top_c2 bash "$TB/run_tb_sp_top.sh" $W1 "$B/k_units" "$B/k_xlane" "$B/k_sfui" "$B/k_sfuf" \
  | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:1 run (XLANE on, NL=16 = the rm_sp shape)"
NG_CFG=16:3:1 WORK=$ROOT/build/gpu/tb_sp_top_xl16 bash "$TB/run_tb_sp_top.sh" "$B/k_xlane" "$B/k_units" $B/k_vadd $B/k_intops $B/k_fsaxpy $B/k_fmode $B/k_mw $B/k_brnd $B/k_shared_red \
  | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 run, CLEAR_VREGS=1"
NG_CLEAR=1 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_clr bash "$TB/run_tb_sp_top.sh" $W1 | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 run, BR_NODRAIN=0 VALU_BDEPTH=1 ( fallback)"
NG_BRND=0 NG_BDEPTH=1 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_fb bash "$TB/run_tb_sp_top.sh" $W1 | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 run, IC_LINES=8 ( I\$)"
NG_IC=8 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_ic8 bash "$TB/run_tb_sp_top.sh" $W1 $BIG | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 run, EN_VALU_F16CVT=0 ( off: the previous VALU)"
F16OFF="$B/k_vadd $B/k_intops $B/k_loop $B/k_fsaxpy $B/k_fsel $B/k_fmode $B/k_trap_fpexc $B/k_mw $B/k_mw3 $B/k_brnd"
NG_F16=0 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_f16off bash "$TB/run_tb_sp_top.sh" $F16OFF | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
# LSU_K >= 2 (ng_lsu_kn) on the tagged multi-outstanding fabric model
# (MEM_OUT = 8, the slot protocol of ng_rsp_fab_ts) and on the serial stub (MEM_OUT = 1, 1-slot adapter); the legs above
# keep LSU_K = 1 / MEM_OUT = 1 (cycle for cycle the pre-26c3 SP, the bisection reference)
echo "== xsim 16:3:0 run, LSU_K=2 MEM_OUT=8"
NG_LSU_K=2 NG_MEMOUT=8 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_k2 bash "$TB/run_tb_sp_top.sh" $W1 $BIG \
  | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 reset, LSU_K=2 MEM_OUT=8, answers of orphaned requests delivered (mport drop path)"
NG_LSU_K=2 NG_MEMOUT=8 NG_ORPHAN=0 NG_FLAT=0:40 NG_CFG=16:3:0 NG_MODE=reset NG_RESET_AT="$PTS" WORK=$ROOT/build/gpu/tb_sp_top_k2 \
  bash "$TB/run_tb_sp_top.sh" $RST | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 32:8:1 run, LSU_K=4 MEM_OUT=8"
NG_LSU_K=4 NG_MEMOUT=8 NG_CFG=32:8:1 WORK=$ROOT/build/gpu/tb_sp_top_k4 bash "$TB/run_tb_sp_top.sh" $W1 "$B/k_units" "$B/k_sfui" "$B/k_sfuf" \
  | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 reset, LSU_K=4 MEM_OUT=8 (orphans as rm_sp)"
NG_LSU_K=4 NG_MEMOUT=8 NG_FLAT=0:40 NG_CFG=16:3:0 NG_MODE=reset NG_RESET_AT="$PTS" WORK=$ROOT/build/gpu/tb_sp_top_k4r \
  bash "$TB/run_tb_sp_top.sh" $RST | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
echo "== xsim 16:3:0 run, LSU_K=2 MEM_OUT=1 (serial port, 1-slot adapter)"
NG_LSU_K=2 NG_MEMOUT=1 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_k2s bash "$TB/run_tb_sp_top.sh" $W1 | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
# owner-change scrub (ng_cp_port SCRUB=1): run + residue / scrub / CACHE-invalidate / model phases
echo "== xsim 16:3:0 scrub, LSU_K=1"
NG_MODE=scrub NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_scr1 bash "$TB/run_tb_sp_top.sh" $B/k_scrub | grep -E "NG_SP_TOP_(PASS|FAIL|TB)|NOTE S5b"
echo "== xsim 16:3:0 scrub, LSU_K=4 MEM_OUT=8"
NG_MODE=scrub NG_LSU_K=4 NG_MEMOUT=8 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_scr4 bash "$TB/run_tb_sp_top.sh" $B/k_scrub | grep -E "NG_SP_TOP_(PASS|FAIL|TB)|NOTE S5b"
echo "== xsim 32:8:1 scrub, LSU_K=2 MEM_OUT=8"
NG_MODE=scrub NG_LSU_K=2 NG_MEMOUT=8 NG_CFG=32:8:1 WORK=$ROOT/build/gpu/tb_sp_top_scr2 bash "$TB/run_tb_sp_top.sh" $B/k_scrub | grep -E "NG_SP_TOP_(PASS|FAIL|TB)|NOTE S5b"
if [ "${SUITE_LAT:-0}" = 1 ]; then
  # board-like fabric latency (200..380 clk_ts ~ the 2.9 us b13 average); slow (atomics are serial: k_gatom excluded)
  LAT="$B/k_stream $B/k_stream_rd $B/k_stream_v $B/k_vadd_big $B/k_memord $B/k_cacheop $B/k_scratch $B/k_scratch_gtt $B/k_mw $B/k_mw2 $B/k_mw3 $B/k_trap_lsu"
  for k in 2 4; do
    echo "== xsim 16:3:0 run, LSU_K=$k MEM_OUT=8, fabric latency 200..380"
    NG_LSU_K=$k NG_MEMOUT=8 NG_FLAT=200:380 NG_TIMEOUT=60000000 NG_CFG=16:3:0 WORK=$ROOT/build/gpu/tb_sp_top_lat$k \
      bash "$TB/run_tb_sp_top.sh" $LAT | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
  done
fi
if [ "${SUITE_VL:-0}" = 1 ]; then
  echo "== verilator 16:3:0 run";   NG_CFG=16:3:0 bash "$TB/run_tb_sp_top_vl.sh" $W1 | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
  echo "== verilator 16:3:0 reset"; NG_CFG=16:3:0 NG_MODE=reset NG_RESET_AT="$PTS" bash "$TB/run_tb_sp_top_vl.sh" $RST \
    | grep -E "NG_SP_TOP_(PASS|FAIL|TB)"
fi
echo "NG_SP_TOP_SUITE_OK"
