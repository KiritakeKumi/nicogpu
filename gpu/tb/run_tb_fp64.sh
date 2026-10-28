#!/usr/bin/env bash
# FP64 unit vectors vs functional-model comparison.
#   rtl/gpu/tb/run_tb_fp64.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 sim/sfu/vec2txt_sfu.py --unit fp64 vectors.jsonl vectors.txt`
#   work_dir: scratch directory (default build/gpu/tb_fp64); use a fresh one
#   regfile_src: ng_sp_regfile.sv (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Env: SIM=xsim|verilator (default xsim)
#      FP64_CFGS="16:2:1:0 32:4:1:1"   NL:NFL:WP:BG per run
#        NL  lanes per beat (register-file NG_RF_NL)
#        NFL FP64 physical lanes (1,2,4,8,16,32)
#        WP  register-file write port used by the FP64 unit (the other one carries a background writer when BG=1)
#        BG  1 = background writer on the other write port (exercises wr_ready stalls)
#      XSIM_ARGS='-testplusarg LAT' / VL_ARGS='+LAT'  print per-vector latency (LAT <insn> <cycles> <active lanes>)
# Expect one NG_FP64_VEC_ALL_OK per configuration and a final NG_FP64_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_fp64.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin]}
WORK=${2:-build/gpu/tb_fp64}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
RF=${3:-$RTL/regfile/ng_sp_regfile.sv}
BIN=${4:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" fp64.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/sfu/ng_sfu_fp_pkg.sv $RTL/fp64/ng_fp64_pkg.sv $RTL/fp64/ng_fp64_fn_pkg.sv $RTL/fp64/ng_fp64_dec.sv $RTL/fp64/ng_fp64_lane.sv $RTL/fp64/ng_fp64.sv $RF $RTL/tb/tb_fp64_vec.sv"
for cfg in ${FP64_CFGS:-16:2:1:0 32:4:1:1}; do
  nl=$(echo "$cfg" | cut -d: -f1)
  nsl=$(echo "$cfg" | cut -d: -f2)
  wp=$(echo "$cfg" | cut -d: -f3)
  bg=$(echo "$cfg" | cut -d: -f4)
  tag="${nl}_${nsl}_${wp}_${bg}"
  {
    echo "\`define NG_TB_NL $nl"
    echo "\`define NG_TB_NFL $nsl"
    echo "\`define NG_TB_VWP $wp"
    echo "\`define NG_TB_BG $bg"
  } > tb_fp64_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$tag"
    verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 256 --top-module tb_fp64_vec \
      -I. -Mdir "obj_$tag" $SRCS > "vl_$tag.build" 2>&1 || { tail -40 "vl_$tag.build"; exit 1; }
    "./obj_$tag/Vtb_fp64_vec" +VEC=fp64.txt ${VL_ARGS:-} > "sim_$tag.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -i . $SRCS > "xvlog_$tag.out" 2>&1 || { cat "xvlog_$tag.out"; exit 1; }
    "$BIN/xelab.bat" -debug off tb_fp64_vec -s "tb_fp64_$tag" > "xelab_$tag.out" 2>&1 \
      || { cat "xelab_$tag.out"; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} "tb_fp64_$tag" > "sim_$tag.out" 2>&1 || true
  fi
  grep -E "FAIL|NG_FP64_VEC|TB_ERROR|cov:|LAT |\(w0=" "sim_$tag.out" | head -60 || true
  grep -q "NG_FP64_VEC_ALL_OK" "sim_$tag.out"
done
echo "NG_FP64_VEC_TB_ALL_OK"
