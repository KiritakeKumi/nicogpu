#!/usr/bin/env bash
# cross-lane unit (ng_xlane) vs functional-model vector comparison.
#   rtl/gpu/tb/run_tb_xlane.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 sim/xlane/vec2txt.py vectors_xlane.jsonl vectors.txt`
#   work_dir: scratch directory (default build/gpu/tb_xlane); use a fresh one
#   regfile_src: ng_sp_regfile.sv (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Env: SIM=xsim|verilator (default xsim)
#      XLANE_CFGS="16:0:0 32:0:0 16:1:1 32:1:1"   NL:VWP:BG per run
#        NL  lanes per beat (register-file NG_RF_NL)
#        VWP register-file write port used by the unit (1 = loses bank conflicts)
#        BG  1 = background writer on the other write port (exercises wr_ready stalls)
#      XSIM_ARGS='-testplusarg LAT' / VL_ARGS='+LAT'  print per-vector latency (LAT <insn> <cycles> <active lanes>)
# Expect one NG_XLANE_VEC_ALL_OK per configuration and a final NG_XLANE_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_xlane.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin]}
WORK=${2:-build/gpu/tb_xlane}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
RF=${3:-$RTL/regfile/ng_sp_regfile.sv}
BIN=${4:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" xlane.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/xlane/ng_xlane_pkg.sv $RTL/xlane/ng_xlane_dec.sv $RTL/xlane/ng_xlane_fsub.sv \
$RTL/xlane/ng_xlane.sv $RF $RTL/tb/tb_xlane_vec.sv"
for cfg in ${XLANE_CFGS:-16:0:0 32:0:0 16:1:1 32:1:1}; do
  nl=${cfg%%:*}; rest=${cfg#*:}; vwp=${rest%%:*}; bg=${rest#*:}
  tag="${nl}_${vwp}_${bg}"
  {
    echo "\`define NG_TB_NL $nl"
    echo "\`define NG_TB_VWP $vwp"
    echo "\`define NG_TB_BG $bg"
  } > tb_xlane_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$tag"
    verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 256 --top-module tb_xlane_vec \
      -I. -Mdir "obj_$tag" $SRCS > "vl_$tag.build" 2>&1 || { tail -40 "vl_$tag.build"; exit 1; }
    "./obj_$tag/Vtb_xlane_vec" +VEC=xlane.txt ${VL_ARGS:-} > "sim_$tag.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -i . $SRCS > "xvlog_$tag.out" 2>&1 || { cat "xvlog_$tag.out"; exit 1; }
    "$BIN/xelab.bat" -debug off tb_xlane_vec -s "tb_xlane_$tag" > "xelab_$tag.out" 2>&1 \
      || { cat "xelab_$tag.out"; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} "tb_xlane_$tag" > "sim_$tag.out" 2>&1 || true
  fi
  grep -E "FAIL|NG_XLANE_VEC|TB_ERROR|cov:|LAT |\(w0=" "sim_$tag.out" | head -60 || true
  grep -q "NG_XLANE_VEC_ALL_OK" "sim_$tag.out"
done
echo "NG_XLANE_VEC_TB_ALL_OK"
