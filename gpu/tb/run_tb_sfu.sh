#!/usr/bin/env bash
# SFU instruction vectors vs functional-model comparison.
#   rtl/gpu/tb/run_tb_sfu.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 sim/sfu/vec2txt_sfu.py vectors.jsonl vectors.txt`
#   work_dir: scratch directory (default build/gpu/tb_sfu); use a fresh one
#   regfile_src: ng_sp_regfile.sv (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Env: SIM=xsim|verilator (default xsim)
#      SFU_CFGS="16:4:1:0 32:8:1:1 32:1:1:1"   NL:NSL:WP:BG per run (NSL=1 is the M1 build)
#        NL  lanes per beat (register-file NG_RF_NL)
#        NSL SFU physical lanes (1,2,4,8,16,32)
#        WP  register-file write port used by the SFU (the other one carries a background writer when BG=1)
#        BG  1 = background writer on the other write port (exercises wr_ready stalls)
#      XSIM_ARGS='-testplusarg LAT' / VL_ARGS='+LAT'  print per-vector latency (LAT <insn> <cycles> <active lanes>)
# Expect one NG_SFU_VEC_ALL_OK per configuration and a final NG_SFU_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_sfu.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin]}
WORK=${2:-build/gpu/tb_sfu}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
RF=${3:-$RTL/regfile/ng_sp_regfile.sv}
BIN=${4:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" sfu.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/sfu/ng_sfu_fp_pkg.sv $RTL/sfu/ng_sfu_pkg.sv $RTL/sfu/ng_sfu_dec.sv $RTL/sfu/ng_sfu_fma.sv \
$RTL/sfu/ng_sfu_divsqrt.sv $RTL/sfu/ng_sfu_lane.sv $RTL/sfu/ng_sfu.sv $RF $RTL/tb/tb_sfu_vec.sv"
for cfg in ${SFU_CFGS:-16:4:1:0 32:8:1:1 32:1:1:1}; do
  nl=$(echo "$cfg" | cut -d: -f1)
  nsl=$(echo "$cfg" | cut -d: -f2)
  wp=$(echo "$cfg" | cut -d: -f3)
  bg=$(echo "$cfg" | cut -d: -f4)
  tag="${nl}_${nsl}_${wp}_${bg}"
  {
    echo "\`define NG_TB_NL $nl"
    echo "\`define NG_TB_NSL $nsl"
    echo "\`define NG_TB_VWP $wp"
    echo "\`define NG_TB_BG $bg"
  } > tb_sfu_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$tag"
    verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 256 --top-module tb_sfu_vec \
      -I. -Mdir "obj_$tag" $SRCS > "vl_$tag.build" 2>&1 || { tail -40 "vl_$tag.build"; exit 1; }
    "./obj_$tag/Vtb_sfu_vec" +VEC=sfu.txt ${VL_ARGS:-} > "sim_$tag.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -i . $SRCS > "xvlog_$tag.out" 2>&1 || { cat "xvlog_$tag.out"; exit 1; }
    "$BIN/xelab.bat" -debug off tb_sfu_vec -s "tb_sfu_$tag" > "xelab_$tag.out" 2>&1 \
      || { cat "xelab_$tag.out"; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} "tb_sfu_$tag" > "sim_$tag.out" 2>&1 || true
  fi
  grep -E "FAIL|NG_SFU_VEC|TB_ERROR|cov:|LAT |\(w0=" "sim_$tag.out" | head -60 || true
  grep -q "NG_SFU_VEC_ALL_OK" "sim_$tag.out"
done
echo "NG_SFU_VEC_TB_ALL_OK"
