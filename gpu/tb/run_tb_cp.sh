#!/usr/bin/env bash
# run the CP subsystem testbench (rtl/gpu/cp) with xsim.
#   rtl/gpu/tb/run_tb_cp.sh [vivado_bin_dir] [work_dir] ["seed ..."] [model_seeds] [gen_cp_vectors exe] ["gen args, e.g. --only copy_fill"]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin, $TMPDIR/ng_tb_cp, TB seed 1, 20 random streams.
# Steps:
#   1. gen_cp_vectors (functional model, fw/cp/difftest/CMakeLists.txt) writes
#      cp_vec.hex.  Built with cmake+ninja in $WORK/model unless GEN_CP points
#      at a prebuilt executable.  On Windows put mingw64 (gcc, g++, ninja,
#      cmake) on PATH first, e.g. C:/msys64/mingw64/bin.
#   2. xsc compiles the real CP firmware (fw/cp/src, -DNGFW_BUS_HOOK) together
#      with the coroutine layer rtl/gpu/cp/ng_cp_cpu_dpi.c into the DPI library.
#   3. xvlog/xelab/xsim run tb_cp (one run per TB seed; the seed changes memory
#      latency / back-pressure only, the vector stream is fixed).
# xelab -O0 mirrors the other run_tb_*.sh (xsim 2026.1 optimizer workaround).
# Expect NG_CP_TB_PASS.  No Vivado synthesis/implementation is run.
set -eu
set -o pipefail
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-${TMPDIR:-/tmp}/ng_tb_cp}
SEEDS=${3:-1}
MSEEDS=${4:-20}
GEN_CP=${5:-${GEN_CP:-}}
GEN_ARGS=${6:-}
# Windows: the generator is a mingw64 binary and needs its runtime DLLs on PATH.
if [ -d /c/msys64/mingw64/bin ]; then PATH=/c/msys64/mingw64/bin:$PATH; fi
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
if command -v cygpath >/dev/null 2>&1; then ROOT=$(cygpath -m "$ROOT"); fi
RTL=$ROOT/rtl/gpu
FW=$ROOT/fw/cp

if [ -n "${GEN_CP:-}" ]; then
  case "$GEN_CP" in /*|?:*) ;; *) GEN_CP=$(pwd)/$GEN_CP ;; esac
fi
mkdir -p "$WORK"
cd "$WORK"

# ---- 1) model-generated vectors ------------------------------------------------
if [ -z "${GEN_CP:-}" ]; then
  [ -f model/build.ninja ] || cmake -S "$ROOT/model" -B model -G Ninja -DCMAKE_BUILD_TYPE=Release > cmake.out 2>&1
  ninja -C model gen_cp_vectors > ninja.out 2>&1 || { tail -30 ninja.out; exit 1; }
  GEN_CP=$(find model -name 'gen_cp_vectors*' -type f \( -name '*.exe' -o -perm -u+x \) | head -1)
fi
# shellcheck disable=SC2086
"$GEN_CP" cp_vec.hex --seeds "$MSEEDS" $GEN_ARGS | tail -4

# ---- 2) DPI library: real firmware + coroutine layer ---------------------------
rm -rf xsim.dir
"$BIN/xsc.bat" "$RTL/cp/ng_cp_cpu_dpi.c" \
  "$FW/src/ngfw_core.c" "$FW/src/ngfw_pkt.c" "$FW/src/ngfw_util.c" \
  "$FW/src/ngfw_plat_ibex.c" "$FW/src/ngfw_ibex_main.c" \
  --gcc_compile_options -DNGFW_BUS_HOOK \
  --gcc_compile_options "-I$FW/include" --gcc_compile_options "-I$FW/src" > xsc.out 2>&1 \
  || { cat xsc.out; exit 1; }

# ---- 3) RTL + TB ------------------------------------------------------------------
# One compile + elaboration per TB seed: the knobs reach the TB through a generated
# include file (xelab.bat mangles "-generic_top NAME=VALUE" on Windows).
for SEED in $SEEDS; do
  # LAT_MAX / RDY_PCT vary with the seed: seed 3 = zero-wait-ish memory, others slower / more back-pressure
  LAT=$((1 + SEED % 4)); RDY=$((100 - (SEED % 3) * 35))
  printf '`define TB_SEED %s
`define TB_LAT_MAX %s
`define TB_RDY_PCT %s
' "$SEED" "$LAT" "$RDY" > tb_cp_cfg.svh
  "$BIN/xvlog.bat" -sv -d TB_CFG -i . "$RTL/cp/ng_cp_pkg.sv" "$RTL/cp/ng_cp_copy.sv" "$RTL/cp/ng_cp_regs.sv"     "$RTL/cp/ng_cp_port.sv" "$RTL/cp/ng_cp_cpu_stub.sv" "$RTL/cp/ng_cp_top.sv"     "$RTL/tb/tb_cp.sv" > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
  "$BIN/xelab.bat" -debug off -O0 tb_cp -sv_lib dpi -s sim_cp > "xelab_$SEED.out" 2>&1     || { cat "xelab_$SEED.out"; exit 1; }
  "$BIN/xsim.bat" sim_cp -runall > "xsim_cp_$SEED.out" 2>&1 || true
  echo "== TB seed $SEED =="
  grep -E "tb_cp|\[vec\] [0-9]|t_.* done|TB_FAIL|PASS|FAIL" "xsim_cp_$SEED.out" | head -60
  grep -q NG_CP_TB_PASS "xsim_cp_$SEED.out"
done
