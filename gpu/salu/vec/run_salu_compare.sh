#!/bin/sh
# run_salu_compare.sh -- SALU/branch/misc unit (ng_salu) vs functional model, end to end.
#
# usage: rtl/gpu/salu/vec/run_salu_compare.sh <nicogpu_vecexport> <dump_salu_rand> [work_dir]
#   <nicogpu_vecexport>   stock exporter  (cmake -S tools/vecexport/dump ...)
#   <dump_salu_rand>   random exporter (cmake -S rtl/gpu/salu/vec ...)
#   work_dir           default build/salu_cmp
# Env: SIM=xsim|verilator (default xsim)  SALU_RAND_PER (default 120)  SALU_RAND_SEED (default 1)
#      SALU_RUNS (passed to run_tb_salu.sh: "0 1" = without / with register-file back-pressure)
#      RF_SRC  register file source (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Steps: export int + misc vectors (stock set: 107 instructions) -> flat text
#        -> random-operand vectors from the same instruction words (107 x PER, default 12840)
#        -> testbench run over both sets.  On Windows put the MinGW runtime dir
#        (C:/msys64/mingw64/bin) on PATH so the exporter binaries start.
set -eu
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
DUMP=${1:?stock exporter binary}
RAND=${2:?random exporter binary}
WORK=${3:-build/salu_cmp}
PY=${PYTHON:-python3}
export SIM=${SIM:-xsim}
RF=${RF_SRC:-$ROOT/rtl/gpu/regfile/ng_sp_regfile.sv}
V2T="$ROOT/rtl/gpu/salu/vec/vec2txt_salu.py"
mkdir -p "$WORK"
$PY "$ROOT/tools/vecexport/vecexport.py" export --unit int --dumper "$DUMP" --out-dir "$WORK" --quiet
$PY "$ROOT/tools/vecexport/vecexport.py" export --unit misc --dumper "$DUMP" --out-dir "$WORK" --quiet
$PY "$V2T" "$WORK/stock.txt" "$WORK/vectors_int.jsonl" "$WORK/vectors_misc.jsonl" --words "$WORK/words.txt"
"$RAND" --words "$WORK/words.txt" --out "$WORK/rand.jsonl" --per "${SALU_RAND_PER:-120}" --seed "${SALU_RAND_SEED:-1}"
$PY "$V2T" "$WORK/rand.txt" "$WORK/rand.jsonl"
bash "$ROOT/rtl/gpu/tb/run_tb_salu.sh" "$WORK/stock.txt" "$WORK/tb_stock" "$RF"
bash "$ROOT/rtl/gpu/tb/run_tb_salu.sh" "$WORK/rand.txt" "$WORK/tb_rand" "$RF"
echo "SALU_COMPARE_OK"
