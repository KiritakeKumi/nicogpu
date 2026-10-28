#!/usr/bin/env bash
# f32 VALU (ng_valu EN_FP=1) vs functional-model vector comparison.
#   rtl/gpu/tb/run_tb_valu_fp.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 sim/sfu/vec2txt_sfu.py --unit valu vectors.jsonl vectors.txt`
#                 (MODE / fflags token stream; vectors from rtl/gpu/valu/vec/dump_valu_fp)
#   work_dir: scratch directory (default build/gpu/tb_valu_fp); use a fresh one
#   regfile_src: ng_sp_regfile.sv (default rtl/gpu/regfile/ng_sp_regfile.sv)
# Env: SIM=xsim|verilator (default xsim)
#      VALU_FP_CFGS="16:4:0:0 32:4:1:1"   NL:NFL:VWP:BG per run
#        NL  lanes per beat (register-file NG_RF_NL)
#        NFL f32 physical lanes (divides NL)
#        VWP register-file write port used by the VALU (1 = loses bank conflicts)
#        BG  1 = background writer on the other write port (exercises wr_ready stalls)
#      VALU_DUAL=0|1|2  1: every vector next to a filler instruction of another warp ;
#                       2: next to two fillers in random order, mostly back to back
#      VALU_BDEPTH=<n>  ng_valu BDEPTH (back-half contexts, default 3; 1 =)
#      VALU_F16=0|1     ng_valu EN_F16CVT (binary16 conversions, default 1; with 0 leave the f16 names out
#                       of the vectors -- they are illegal instructions there)
#      F16_SWEEP_DUMP=<dump_valu_fp exe>  (with VALU_F16 != 0) also generate the deterministic binary16 sweeps
#                       `dump_valu_fp --f16-sweep N` for each N in F16_SWEEPS (default "4" = flag isolation: every lane of a
#                       vector holds the same value, so a single wrong UF/NX is not masked by the per-vector OR; 1 = all
#                       65536 f16 patterns, 2 = f32 -> f16 boundaries) and run them through the same configurations
#      XSIM_ARGS='-testplusarg LAT' / VL_ARGS='+LAT'  print per-vector latency
# Expect one NG_VALU_FP_VEC_ALL_OK per configuration and a final NG_VALU_FP_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_valu_fp.sh <vectors.txt> [work_dir] [regfile_src] [vivado_bin]}
WORK=${2:-build/gpu/tb_valu_fp}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
RF=${3:-$RTL/regfile/ng_sp_regfile.sv}
BIN=${4:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
if [ -n "${F16_SWEEP_DUMP:-}" ] && [ "${VALU_F16:-1}" != 0 ] && [ -z "${F16_SWEEP_CHILD:-}" ]; then
  SELF=$(cd "$(dirname "$0")" && pwd)/$(basename "$0")
  SW_ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
  SW_WORK=${2:-build/gpu/tb_valu_fp}
  mkdir -p "$SW_WORK"
  for n in ${F16_SWEEPS:-4}; do
    "$F16_SWEEP_DUMP" --f16-sweep "$n" --out "$SW_WORK/f16sw$n.jsonl" > /dev/null
    python3 "$SW_ROOT/sim/sfu/vec2txt_sfu.py" --unit valu "$SW_WORK/f16sw$n.jsonl" "$SW_WORK/f16sw$n.txt" > /dev/null
    F16_SWEEP_CHILD=1 bash "$SELF" "$SW_WORK/f16sw$n.txt" "$SW_WORK/sw$n" "${3:-}" "${4:-}" || exit 1
  done
fi
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" valu_fp.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/valu/ng_valu_pkg.sv $RTL/valu/ng_valu_fp_pkg.sv $RTL/valu/ng_valu_dec.sv $RTL/valu/ng_valu_mul.sv \
$RTL/valu/ng_valu_lane.sv $RTL/valu/ng_valu_fp_side.sv $RTL/valu/ng_valu_fp_lane.sv $RTL/valu/ng_valu_fp.sv \
$RTL/valu/ng_valu.sv $RF $RTL/tb/tb_valu_fp_vec.sv"
for cfg in ${VALU_FP_CFGS:-16:4:0:0 32:4:1:1}; do
  nl=$(echo "$cfg" | cut -d: -f1)
  nfl=$(echo "$cfg" | cut -d: -f2)
  vwp=$(echo "$cfg" | cut -d: -f3)
  bg=$(echo "$cfg" | cut -d: -f4)
  tag="${nl}_${nfl}_${vwp}_${bg}"
  {
    echo "\`define NG_TB_NL $nl"
    echo "\`define NG_TB_NFL $nfl"
    echo "\`define NG_TB_VWP $vwp"
    echo "\`define NG_TB_BG $bg"
    echo "\`define NG_TB_DUAL ${VALU_DUAL:-0}"
    echo "\`define NG_TB_BDEPTH ${VALU_BDEPTH:-3}"
    echo "\`define NG_TB_F16 ${VALU_F16:-1}"
  } > tb_valu_fp_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$tag"
    verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 256 --top-module tb_valu_fp_vec \
      -I. -Mdir "obj_$tag" $SRCS > "vl_$tag.build" 2>&1 || { tail -40 "vl_$tag.build"; exit 1; }
    "./obj_$tag/Vtb_valu_fp_vec" +VEC=valu_fp.txt ${VL_ARGS:-} > "sim_$tag.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -i . $SRCS > "xvlog_$tag.out" 2>&1 || { cat "xvlog_$tag.out"; exit 1; }
    "$BIN/xelab.bat" -debug off tb_valu_fp_vec -s "tb_valu_fp_$tag" > "xelab_$tag.out" 2>&1 \
      || { cat "xelab_$tag.out"; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} "tb_valu_fp_$tag" > "sim_$tag.out" 2>&1 || true
  fi
  grep -E "FAIL|NG_VALU_FP_VEC|TB_ERROR|cov:|LAT |\(w0=" "sim_$tag.out" | head -60 || true
  grep -q "NG_VALU_FP_VEC_ALL_OK" "sim_$tag.out" || { echo "--- sim_$tag.out (head) ---"; head -40 "sim_$tag.out"; exit 1; }
done
echo "NG_VALU_FP_VEC_TB_ALL_OK"
