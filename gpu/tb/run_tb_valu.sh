#!/usr/bin/env bash
# integer VALU vs functional-model vector comparison.
#   rtl/gpu/tb/run_tb_valu.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 sim/valu/vec2txt.py vectors_int.jsonl vectors.txt`
#   work_dir: scratch directory (default build/gpu/tb_valu); use a fresh one
#   regfile_src: ng_sp_regfile.sv (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Env: SIM=xsim|verilator (default xsim)
#      VALU_CFGS="16:0:0 32:0:0 16:1:1 32:1:1"   NL:VWP:BG per run
#        NL  lanes per beat (register-file NG_RF_NL)
#        VWP register-file write port used by the VALU (1 = loses bank conflicts)
#        BG  1 = background writer on the other write port (exercises wr_ready stalls)
#      VALU_DUAL=0|1|2  1: every vector next to a filler instruction of another warp ;
#                       2: next to two fillers in random order, mostly back to back
#      VALU_BDEPTH=<n>  ng_valu BDEPTH (back-half contexts, default 3; 1 =)
#      XSIM_ARGS='-testplusarg LAT' / VL_ARGS='+LAT'  print per-vector latency (LAT <insn> <cycles> <active lanes>)
# Expect one NG_VALU_VEC_ALL_OK per configuration and a final NG_VALU_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_valu.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin]}
WORK=${2:-build/gpu/tb_valu}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
RF=${3:-$RTL/regfile/ng_sp_regfile.sv}
BIN=${4:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" valu.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/valu/ng_valu_pkg.sv $RTL/valu/ng_valu_fp_pkg.sv $RTL/valu/ng_valu_dec.sv $RTL/valu/ng_valu_mul.sv \
$RTL/valu/ng_valu_lane.sv $RTL/valu/ng_valu_fp_side.sv $RTL/valu/ng_valu_fp_lane.sv $RTL/valu/ng_valu_fp.sv \
$RTL/valu/ng_valu.sv $RF $RTL/tb/tb_valu_vec.sv"
for cfg in ${VALU_CFGS:-16:0:0 32:0:0 16:1:1 32:1:1}; do
  nl=${cfg%%:*}; rest=${cfg#*:}; vwp=${rest%%:*}; bg=${rest#*:}
  tag="${nl}_${vwp}_${bg}"
  {
    echo "\`define NG_TB_NL $nl"
    echo "\`define NG_TB_VWP $vwp"
    echo "\`define NG_TB_BG $bg"
    echo "\`define NG_TB_DUAL ${VALU_DUAL:-0}"
    echo "\`define NG_TB_BDEPTH ${VALU_BDEPTH:-3}"
  } > tb_valu_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$tag"
    verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 256 --top-module tb_valu_vec \
      -I. -Mdir "obj_$tag" $SRCS > "vl_$tag.build" 2>&1 || { tail -40 "vl_$tag.build"; exit 1; }
    "./obj_$tag/Vtb_valu_vec" +VEC=valu.txt ${VL_ARGS:-} > "sim_$tag.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -i . $SRCS > "xvlog_$tag.out" 2>&1 || { cat "xvlog_$tag.out"; exit 1; }
    "$BIN/xelab.bat" -debug off tb_valu_vec -s "tb_valu_$tag" > "xelab_$tag.out" 2>&1 \
      || { cat "xelab_$tag.out"; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} "tb_valu_$tag" > "sim_$tag.out" 2>&1 || true
  fi
  grep -E "FAIL|DIAG|NG_VALU_VEC|TB_ERROR|cov:|LAT |\(w0=" "sim_$tag.out" | head -60 || true
  grep -q "NG_VALU_VEC_ALL_OK" "sim_$tag.out" || { echo "--- sim_$tag.out (head) ---"; head -40 "sim_$tag.out"; exit 1; }
done
echo "NG_VALU_VEC_TB_ALL_OK"
