#!/usr/bin/env bash
# SALU / branch / misc-sync-CSR unit (ng_salu) vs functional-model vector comparison.
#   rtl/gpu/tb/run_tb_salu.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 rtl/gpu/salu/vec/vec2txt_salu.py vectors.txt in.jsonl...`
#   work_dir: scratch directory (default build/gpu/tb_salu); use a fresh one
#   regfile_src: ng_sp_regfile.sv (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Env: SIM=xsim|verilator (default xsim)
#      SALU_RUNS="0 1"   0 = no register-file back-pressure, 1 = random ur_ready/uw_ready stalls (+STALL)
#      XSIM_ARGS / VL_ARGS  extra simulator arguments, e.g. '-testplusarg LAT' / '+LAT'
# Expect one NG_SALU_VEC_ALL_OK per run and a final NG_SALU_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_salu.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin]}
WORK=${2:-build/gpu/tb_salu}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
RF=${3:-$RTL/regfile/ng_sp_regfile.sv}
BIN=${4:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" salu.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/salu/ng_salu_pkg.sv $RTL/salu/ng_salu_dec.sv $RTL/salu/ng_salu.sv $RF $RTL/tb/tb_salu_vec.sv"
if [ "$SIM" = verilator ]; then
  rm -rf obj
  verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 256 --top-module tb_salu_vec \
    -I. -Mdir obj $SRCS > vl.build 2>&1 || { tail -40 vl.build; exit 1; }
else
  "$BIN/xvlog.bat" -sv -i . $SRCS > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
  "$BIN/xelab.bat" -debug off tb_salu_vec -s tb_salu > xelab.out 2>&1 || { cat xelab.out; exit 1; }
fi
for run in ${SALU_RUNS:-0 1}; do
  args=""
  [ "$run" = 1 ] && args="STALL"
  if [ "$SIM" = verilator ]; then
    a=""; [ -n "$args" ] && a="+$args"
    "./obj/Vtb_salu_vec" +VEC=salu.txt $a ${VL_ARGS:-} > "sim_$run.out" 2>&1 || true
  else
    xa=""; [ -n "$args" ] && xa="-testplusarg $args"
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" $xa ${XSIM_ARGS:-} tb_salu > "sim_$run.out" 2>&1 || true
  fi
  grep -E "FAIL|DIAG|NG_SALU_VEC|TB_ERROR|cov:|LAT |\(w0=" "sim_$run.out" | head -60 || true
  grep -q "NG_SALU_VEC_ALL_OK" "sim_$run.out" || { echo "--- sim_$run.out (head) ---"; head -40 "sim_$run.out"; exit 1; }
done
echo "NG_SALU_VEC_TB_ALL_OK"
