#!/usr/bin/env bash
# arch_seq -- random legal instruction streams, ng_sp_top vs the functional model, per instruction.
#
#   rtl/gpu/tb/run_tb_arch_seq.sh [work_dir]
#
# tools/archseq/archseq.py generates one dispatch bundle per seed (a grid of workgroups, workgroup k runs random
# program k, built from ngasm with legality constraints: waitcnt/hazards, shared-memory init, aligned atomics, ...),
# the model's strict-waitcnt checker and a poisoned run (undefined registers / shared words) prove the programs legal,
# bundle_run.exe exports pre/post memory + the per-warp commit trace, and tb_sp_top (run_tb_sp_top.sh) replays them on
# ng_sp_top and compares every commit (pc, instruction word, exec) and every memory byte.  Architectural state is made
# observable by stores of v8..v39 / u52..u59 / exec / fflags / mode snapshots to per-workgroup OUT slots, so the memory
# comparison covers the register state at the snapshot points as well.
# Env:
#   SIM=verilator|xsim       simulator (default verilator; xsim on the Windows dev box, serialised by build/locked.sh)
#   ARCH_SEQ_SEEDS="a-b"     campaign seeds (default 1-28: 28 bundles x 8 programs = 224 programs, 16:3:1 and 16:3:0)
#   ARCH_SEQ_NPROG=<n>       programs per bundle (default 8)
#   ARCH_SEQ_PROBE="a-b"     extra bundles in probe mode (destination registers dumped after every instruction;
#                            default 101-104; empty = none)
#   ARCH_SEQ_ENV="K=V ..."   extra run_tb_sp_top.sh environment (e.g. "NG_LSU_K=4 NG_MEMOUT=8")
#   NG_PY310                 python >= 3.10 with PyYAML (default python3)
#   NG_BUNDLE_RUN            bundle_run.exe (default build/gpu/dispatch_bundle/bundle_run.exe, built if missing;
#                            CI: the one the `bundles` job puts into the artifact as $BUNDLES/bundle_run.exe)
#   NJOBS                    Verilator -j
# Output: NG_ARCH_SEQ_PASS (archseq.py) and NG_ARCH_SEQ_TB_PASS at the end.
set -eu
set -o pipefail
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
WORK=${1:-$ROOT/build/gpu/tb_arch_seq}
case $WORK in /*|?:*) ;; *) WORK=$ROOT/$WORK ;; esac
mkdir -p "$WORK"
SIM=${SIM:-verilator}
PY=${NG_PY310:-python3}
SEEDS=${ARCH_SEQ_SEEDS:-1-28}
NPROG=${ARCH_SEQ_NPROG:-8}
PROBE=${ARCH_SEQ_PROBE-101-104}
export SIM

if [ -z "${NG_BUNDLE_RUN:-}" ]; then
  NG_BUNDLE_RUN=$ROOT/build/gpu/dispatch_bundle/bundle_run.exe
  if [ -n "${BUNDLES:-}" ] && [ -f "$BUNDLES/bundle_run.exe" ]; then
    NG_BUNDLE_RUN=$BUNDLES/bundle_run.exe
    chmod +x "$NG_BUNDLE_RUN" 2>/dev/null || true
  fi
fi
if [ ! -f "$NG_BUNDLE_RUN" ]; then
  echo "== building the bundle exporter"
  NG_CC=${NG_CC:-c++} NG_PY=$PY bash "$ROOT/tools/dispatch_bundle/build.sh" "$(dirname "$NG_BUNDLE_RUN")"
fi
export NG_BUNDLE_RUN

# model-only self test first: generator + strict waitcnt checker + poisoned run (fast, no simulator)
echo "== archseq selftest (model only)"
"$PY" "$ROOT/tools/archseq/archseq.py" selftest --seeds 1-4 --nprog 4 --out "$WORK/selftest"

extra=()
for kv in ${ARCH_SEQ_ENV:-}; do extra+=(--env "$kv"); done
wk=()
[ "$SIM" = verilator ] && wk=(--sim verilator)
echo "== arch_seq check bundles: seeds $SEEDS x $NPROG programs ($SIM)"
"$PY" "$ROOT/tools/archseq/archseq.py" run --seeds "$SEEDS" --nprog "$NPROG" --out "$WORK/check" --work "$WORK/tb" \
  --nolock "${wk[@]}" "${extra[@]}" | tee "$WORK/check.log"
grep -q NG_ARCH_SEQ_PASS "$WORK/check.log"
if [ -n "$PROBE" ]; then
  echo "== arch_seq probe bundles (every destination register after every instruction): seeds $PROBE"
  "$PY" "$ROOT/tools/archseq/archseq.py" run --seeds "$PROBE" --nprog 4 --mode probe --out "$WORK/probe" \
    --work "$WORK/tb" --nolock "${wk[@]}" "${extra[@]}" | tee "$WORK/probe.log"
  grep -q NG_ARCH_SEQ_PASS "$WORK/probe.log"
fi
echo "NG_ARCH_SEQ_TB_PASS"
