#!/usr/bin/env bash
# run the SP-scheduler testbenches with Vivado xsim.
#   rtl/gpu/tb/run_tb_sp_sched.sh [vivado_bin_dir] [work_dir] [test]
#   test = basic | waitcnt | barrier | trap | soak | chain | brwait | trace | all (default all)
# Expect NG_SP_SCHED_<TB>_TB_PASS lines for each selected test.
#   Use a fresh work_dir each time.
# Env: SOAK_SEEDS="1 2 3 4 5" overrides the soak seed list;
#      TRACE_SCEN=<dir> runs one trace scenario (default: generates with the
#      model runner when available, else the bundled static scenarios).
# Note: xsim.bat on Windows cannot forward valued plusargs, so soak seeds are
#   passed by regenerating a tb_sp_sched_seed.svh and recompiling per seed
#   (cheap: one xvlog + one xelab per seed).
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-build/gpu/tb}
TEST=${3:-all}
SRC=$(cd "$(dirname "$0")/../.." && pwd); SRC=$SRC/gpu
RTL=$SRC/sp_sched
PRIM=$SRC/prim
TBD=$SRC/tb
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"

xv ()  { "$BIN/xvlog.bat" -sv -i . -i "$TBD" "$@"; }
run1 () {  # run1 <name> <tb-module> <extra xvlog args...>
  local name=$1 mod=$2; shift 2
  xv "$RTL/ng_sp_sched_pkg.sv" "$RTL/ng_sp_predec.sv" "$RTL/ng_sp_sched.sv" \
     "$PRIM/ng_sp_prim_rr_arb.sv" "$TBD/tb_sp_sched_${name}.sv" "$@" > xvlog.out 2>&1 \
    || { cat xvlog.out; exit 1; }
  "$BIN/xelab.bat" -debug off "$mod" -s "tb_${name}" > xelab.out 2>&1 \
    || { cat xelab.out; exit 1; }
  printf 'run -all\nquit\n' | "$BIN/xsim.bat" "tb_${name}" > "xsim_${name}.out" 2>&1 || true
  grep -E "PASS|FAIL" "xsim_${name}.out" || true
  grep -q "NG_SP_SCHED_$(echo "$name" | tr 'a-z' 'A-Z')_TB_PASS" "xsim_${name}.out"
}

# ---------------- directed + soak ----------------
if [ "$TEST" = all ] || [ "$TEST" = basic ];  then run1 basic  tb_sp_sched_basic;  fi
if [ "$TEST" = all ] || [ "$TEST" = waitcnt ]; then run1 waitcnt tb_sp_sched_waitcnt; fi
if [ "$TEST" = all ] || [ "$TEST" = barrier ]; then run1 barrier tb_sp_sched_barrier; fi
if [ "$TEST" = all ] || [ "$TEST" = trap ];    then run1 trap    tb_sp_sched_trap;    fi
if [ "$TEST" = all ] || [ "$TEST" = chain ];   then run1 chain   tb_sp_sched_chain;   fi
if [ "$TEST" = all ] || [ "$TEST" = brwait ];  then run1 brwait  tb_sp_sched_brwait;  fi
if [ "$TEST" = all ] || [ "$TEST" = soak ]; then
  for s in ${SOAK_SEEDS:-1 2 3 4 5}; do
    printf '`define NG_SOAK_SEED %s\n`define NG_SOAK_ITERS 40\n' "$s" > tb_sp_sched_seed.svh
    run1 soak tb_sp_sched_soak -d NG_SOAK_SEED_FILE
  done
  rm -f tb_sp_sched_seed.svh
fi

# ---------------- trace vs functional model ----------------
if [ "$TEST" = all ] || [ "$TEST" = trace ]; then
  GEN=$SRC/tb/model_trace/sp_trace_gen.exe
  if [ ! -f "$GEN" ]; then
    GEN=${NG_MODEL_TRACE_EXE:-$(cd "$SRC/../.." && pwd)/build/gpu/model_trace_gen/sp_trace_gen.exe}
  fi
  if [ ! -f "$GEN" ]; then
    # build it from the worktree (isagen + model sources; softfloat from
    # source unless NG_SF_LIB points at a prebuilt libnicogpu_softfloat.a)
    NG_SF_LIB=${NG_SF_LIB:-"$(cd "$SRC/../.." && pwd)/build/gcc/libnicogpu_softfloat.a"} \
      bash "$SRC/tb/model_trace/build_model_trace.sh" || { echo "model build failed"; exit 1; }
    GEN=$(cd "$SRC/../.." && pwd)/build/gpu/model_trace_gen/sp_trace_gen.exe
  fi
  if command -v python >/dev/null 2>&1; then
    python "$TBD/model_trace/run_trace_check.py" --bin "$BIN" --work "$WORK" \
      --gen "$GEN" || { echo "trace check failed"; exit 1; }
  else
    echo "python not found: model trace check skipped" >&2
    exit 1
  fi
fi

echo "NG_SP_SCHED_TB_ALL_OK ($TEST)"
