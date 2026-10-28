#!/usr/bin/env bash
# run tb_cp under Verilator (--binary --timing), with either CPU socket:
#   MODE=stub  behavioural DPI stub running the real firmware (host build, ucontext coroutines)
#   MODE=ibex  real lowRISC Ibex running the riscv-linked firmware image (no DPI)
#   rtl/gpu/tb/run_tb_cp_verilator.sh [work_dir] ["seed ..."] [model_seeds] ["gen args"]
# Env: MODE (stub|ibex, default stub), IBEX (checkout, default <repo>/third_party/ibex),
#      GEN_CP (prebuilt gen_cp_vectors; else cmake+ninja in $WORK/model), RISCV (tool prefix,
#      default riscv64-unknown-elf-), VOPTS (extra verilator args), NJOBS,
#      BOOT=1 (MODE=ibex): load the firmware through the hardware boot loader ng_cp_boot from an NGCF image
#      (tools/ngcf) written to C3 by the TB, instead of the $readmemh IMEM preload.
# Verilator >= 5.032.  Linux only (ucontext).  Expect NG_CP_TB_PASS per seed.
set -eu
set -o pipefail
WORK=${1:-${TMPDIR:-/tmp}/ng_tb_cp_vl}
SEEDS=${2:-1}
MSEEDS=${3:-20}
GEN_ARGS=${4:-}
MODE=${MODE:-stub}
RISCV=${RISCV:-riscv64-unknown-elf-}
NJOBS=${NJOBS:-8}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
IBEX=${IBEX:-$ROOT/third_party/ibex}
RTL=$ROOT/rtl/gpu
FW=$ROOT/fw/cp
SRCS="src/ngfw_core.c src/ngfw_pkt.c src/ngfw_util.c src/ngfw_plat_ibex.c src/ngfw_ibex_main.c"
mkdir -p "$WORK"
cd "$WORK"

# ---- 1) model-generated vectors ------------------------------------------------
GEN_CP=${GEN_CP:-}
if [ -z "$GEN_CP" ]; then
  [ -f model/build.ninja ] || cmake -S "$ROOT/model" -B model -G Ninja -DCMAKE_BUILD_TYPE=Release > cmake.out 2>&1
  ninja -C model gen_cp_vectors > ninja.out 2>&1 || { tail -30 ninja.out; exit 1; }
  GEN_CP=$(find model -name 'gen_cp_vectors*' -type f -perm -u+x | head -1)
fi
# shellcheck disable=SC2086
"$GEN_CP" cp_vec.hex --seeds "$MSEEDS" $GEN_ARGS | tail -4

DEFS=()
IBEXF=()
LDF=()
FILES="$RTL/cp/ng_cp_pkg.sv $RTL/cp/ng_cp_copy.sv $RTL/cp/ng_cp_regs.sv $RTL/cp/ng_cp_port.sv"
if [ "$MODE" = ibex ]; then
  # ---- 2a) real firmware image -> IMEM hex, quiet-flag address --------------------
  mkdir -p rv
  M="-march=rv32imc_zicsr -mabi=ilp32"
  (cd "$FW" && for f in $SRCS; do
     ${RISCV}gcc $M -mcmodel=medany -std=c99 -Wall -Wextra -Werror -pedantic -ffreestanding -nostdlib \
       -ffunction-sections -fdata-sections -DNGFW_IBEX -Os -Iinclude -c "$f" -o "$WORK/rv/$(basename "$f" .c).o"
   done)
  ${RISCV}gcc $M -nostdlib -Wl,--gc-sections -Wl,-Map=rv/fw.map -T "$FW/ibex/link.ld" "$FW/ibex/crt0.S" rv/ngfw_*.o -lgcc -o rv/fw.elf
  ${RISCV}size rv/fw.elf
  ${RISCV}objcopy -O binary -j .text rv/fw.elf rv/fw.bin
  python3 "$FW/tools/bin2hex.py" rv/fw.bin fw_imem.hex --words 16384
  if [ "${BOOT:-0}" = 1 ]; then
    BSS=$(( 0x$(${RISCV}nm rv/fw.elf | awk '$3=="__bss_end"{print $1}') - 0x$(${RISCV}nm rv/fw.elf | awk '$3=="__bss_start"{print $1}') ))
    python3 "$ROOT/tools/ngcf/ngcf_pack.py" pack --text rv/fw.bin --bss "$BSS" --entry 0x80 --hex fw_ngcf.hex -o rv/fw.ngcf
  fi
  qa=$(${RISCV}nm rv/fw.elf | awk '$3=="ngfw_ibex_quiet"{print $1}')
  QW=$(( (0x$qa - 0x100000) / 4 ))
  echo "ngfw_ibex_quiet @0x$qa -> DMEM word $QW"
  # ---- 2b) Ibex file list ---------------------------------------------------------
  bash "$RTL/cp/ibex/gen_prim_shim.sh" "$IBEX" shim
  export IBEX SHIM=$WORK/shim
  envsubst '$IBEX $SHIM' < "$RTL/cp/ibex/ibex_files.f" | grep -v '^#' > ibex.f
  P=$IBEX/vendor/lowrisc_ip
  DEFS=(-DNG_CP_IBEX "-DIBEX_QUIET_WORD=$QW" '-DIBEX_IMAGE="fw_imem.hex"'
        "+incdir+$IBEX/rtl" "+incdir+$P/ip/prim/rtl" "+incdir+$P/dv/sv/dv_utils")
  IBEXF=("$IBEX/lint/verilator_waiver.vlt" -f ibex.f)
  FILES="$FILES $RTL/cp/ng_cp_ibex_wrap.sv $RTL/cp/ng_cp_boot.sv"
  if [ "${BOOT:-0}" = 1 ]; then DEFS+=(-DTB_BOOT_LOADER); fi
else
  # ---- 2) host-built firmware + coroutine layer -----------------------------------
  mkdir -p host
  for f in $SRCS; do
    gcc -std=gnu99 -O1 -g -DNGFW_BUS_HOOK -I"$FW/include" -I"$FW/src" -c "$FW/$f" -o "host/$(basename "$f" .c).o"
  done
  gcc -std=gnu99 -O1 -g -DNGFW_BUS_HOOK -I"$FW/include" -I"$FW/src" -c "$RTL/cp/ng_cp_cpu_dpi.c" -o host/ng_cp_cpu_dpi.o
  LDF=(-LDFLAGS "$WORK/host/*.o")
  FILES="$FILES $RTL/cp/ng_cp_cpu_stub.sv"
fi

# ---- 3) verilate + run, one build per TB seed ----------------------------------------
for SEED in $SEEDS; do
  LAT=$((1 + SEED % 4)); RDY=$((100 - (SEED % 3) * 35))
  rm -rf "obj_$SEED"
  # shellcheck disable=SC2086
  verilator --binary --timing -j "$NJOBS" -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
    --top-module tb_cp --Mdir "obj_$SEED" -o "sim_$SEED" \
    -GSEED=$SEED -GLAT_MAX=$LAT -GRDY_PCT=$RDY "${DEFS[@]}" ${VOPTS:-} "${LDF[@]}" \
    "${IBEXF[@]}" $FILES "$RTL/cp/ng_cp_top.sv" "$RTL/tb/tb_cp.sv" > "vl_build_$SEED.out" 2>&1 \
    || { tail -40 "vl_build_$SEED.out"; exit 1; }
  "./obj_$SEED/sim_$SEED" > "vl_run_$SEED.out" 2>&1 || true
  echo "== TB seed $SEED ($MODE, verilator) =="
  grep -E "tb_cp|\[vec\] [0-9]|t_.* done|TB_FAIL|PASS|FAIL|Error|%" "vl_run_$SEED.out" | head -60
  grep -q NG_CP_TB_PASS "vl_run_$SEED.out"
done
