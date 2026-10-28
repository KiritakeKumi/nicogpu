#!/usr/bin/env bash
# unit / directed testbenches of the LSU block (xsim; no model vectors needed).
#   rtl/gpu/tb/run_tb_lsu_unit.sh [work_dir] [vivado_bin_dir] ["seed ..."]
#
#   tb_lsu_shmem: banked shared memory: conflict rule vs reference, round counts, data
#   tb_lsu_l1: L1D write-through / non-coherent semantics, cpol, inv / wb (shadow model)
#   tb_lsu_dir: coalescing counts, store order, atomics, precise MemFault, shared rounds,
#                  waitcnt accounting, uld (hand-built instruction words)
# Seeds (default "1 2 3") drive the random parts of shmem / l1 (xsim -sv_seed / Verilator +SEED).
# LSU_K=<n> (default 1 = ng_lsu) / MEMOUT=<n> (default 8) build tb_lsu_dir with ng_lsu_kn and its uc channel
#   responder; LSU_K >= 2 adds section M9 (cross-instruction concurrency, late errors, reset in flight).
# Expect NG_LSU_SHMEM_TB_PASS / NG_LSU_L1_TB_PASS / NG_LSU_DIR_TB_PASS and finally
# NG_LSU_UNIT_ALL_OK.  Env SIM=verilator uses verilator --binary instead (needs Verilator 5).
# Windows/Git Bash: VERILATOR=verilator.exe VL_MAKEFLAGS="CXX=g++ CC=gcc LINK=g++" plus the PATH of
# sim/verilator/env.bat (the shipped verilated.mk hardcodes /usr/bin/g++); unset on Linux/CI.
set -eu
WORK=${1:-build/gpu/tb_lsu_unit}
BIN=${2:-C:/AMDDesignTools/2026.1/Vivado/bin}
SEEDS=${3:-"1 2 3"}
SIM=${SIM:-xsim}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"

run_tb() {   # <top> <pass-token> <seed> <sources...>
  top=$1; tok=$2; seed=$3; shift 3
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_${top}"
    ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 4096 --unroll-stmts 4000000 -Wno-TIMESCALEMOD --top-module "$top" \
      -Mdir "obj_${top}" "$@" > "vl_${top}.build" 2>&1 || { tail -40 "vl_${top}.build"; exit 1; }
    "./obj_${top}/V${top}" +SEED="$seed" > "sim_${top}_${seed}.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv "$@" > "xvlog_${top}.out" 2>&1 || { cat "xvlog_${top}.out"; exit 1; }
    "$BIN/xelab.bat" -debug off -O0 "$top" -s "${top}_s" > "xelab_${top}.out" 2>&1 || { cat "xelab_${top}.out"; exit 1; }
    "$BIN/xsim.bat" ${top}_s -R -sv_seed "$seed" > "sim_${top}_${seed}.out" 2>&1 || true
  fi
  grep -E "FAIL|NG_LSU|TB_ERROR" "sim_${top}_${seed}.out" | head -20 || true
  grep -q "$tok" "sim_${top}_${seed}.out" || { echo "--- sim_${top}_${seed}.out (head) ---"; head -30 "sim_${top}_${seed}.out"; exit 1; }
}

PKG="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/l2/ng_l2_amo_alu.sv $RTL/lsu/ng_lsu_pkg.sv"
# lanes per beat under test (env SH_LPB / L1_LPB, default 8 / 4); passed to the TBs as macros through a
# first source file (xsim.bat mangles '=' in -d arguments)
{
  echo "\`define NG_TB_LPB ${SH_LPB:-8}"
  echo "\`define NG_TB_SHLPB ${SH_LPB:-8}"
  echo "\`define NG_TB_L1LPB ${L1_LPB:-4}"
  echo "\`define NG_TB_LSUK ${LSU_K:-1}"
  echo "\`define NG_TB_MEMOUT ${MEMOUT:-8}"
} > lsu_unit_cfg.sv
CFG="$WORK/lsu_unit_cfg.sv"
for s in $SEEDS; do
  run_tb tb_lsu_shmem NG_LSU_SHMEM_TB_PASS "$s" "$CFG" "$RTL/noc/ng_noc_pkg.sv" "$RTL/lsu/ng_lsu_shmem.sv" "$RTL/tb/tb_lsu_shmem.sv"
  run_tb tb_lsu_l1 NG_LSU_L1_TB_PASS "$s" $PKG "$RTL/lsu/ng_l1d.sv" "$RTL/tb/tb_lsu_l1.sv"
done
run_tb tb_lsu_dir NG_LSU_DIR_TB_PASS 1 "$CFG" $PKG "$RTL/lsu/ng_lsu_dec.sv" "$RTL/lsu/ng_lsu_fatomic.sv" \
  "$RTL/lsu/ng_lsu_shmem.sv" "$RTL/lsu/ng_l1d.sv" "$RTL/lsu/ng_lsu.sv" "$RTL/lsu/ng_lsu_kn.sv" "$RTL/lsu/ng_lsu_top.sv" \
  "$RTL/regfile/ng_sp_regfile.sv" "$RTL/tb/tb_lsu_dir.sv"
echo "NG_LSU_UNIT_ALL_OK"
