#!/usr/bin/env bash
# LSU + L1D + shared memory vs functional-model vectors.
#   rtl/gpu/tb/run_tb_lsu.sh <vectors.txt> [work_dir] [vivado_bin_dir]
#
#   vectors.txt: flat text made by `python3 sim/lsu/vec2txt_lsu.py vectors_lsu.jsonl vectors.txt`
#                 (sim/lsu/run_lsu_compare.sh does the whole export + convert + run chain)
#   work_dir: scratch directory (default build/gpu/tb_lsu); use a fresh one
# Env: SIM=xsim|verilator (default xsim)
#      LSU_CFGS="16:4:2:0:0 32:16:4:3:1 ..."  NL:L1SETS:L1WAYS:MAXLAT:SHRAND per run
#        NL      register-file lanes per beat (NG_RF_NL)
#        L1SETS/WAYS  L1D geometry (small values force evictions)
#        MAXLAT  random fabric / MMU response latency bound (0 = fastest)
#        SHRAND  1 = random base of the shared window inside the 64 KiB array
#      SH_LPB / L1_LPB   lanes per beat of the shared-memory unit / of the global-path scatter+merge
#                        (default 8 / 4; 32 = single wide round, the former behaviour)
#      XSIM_ARGS='-testplusarg LAT' / VL_ARGS='+LAT'  print per-instruction latency
#      LSU_K=<n> (default 1 = ng_lsu) / MEMOUT=<n> (default 8): ng_lsu_kn and its uc channel slots;
#      HOSTMAP=1: VRAM addresses are mapped into the host aperture (uc path, i.e. the slot channel for LSU_K >= 2)
# Expect one NG_LSU_VEC_ALL_OK per configuration and a final NG_LSU_VEC_TB_ALL_OK.
set -eu
VEC=${1:?usage: run_tb_lsu.sh <vectors.txt> [work_dir] [vivado_bin]}
WORK=${2:-build/gpu/tb_lsu}
BIN=${3:-C:/AMDDesignTools/2026.1/Vivado/bin}
SIM=${SIM:-xsim}
RTL=$(cd "$(dirname "$0")/../.." && pwd)/gpu
case $VEC in /*|?:*) ;; *) VEC=$(pwd)/$VEC ;; esac
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
cp "$VEC" lsu.txt
SRCS="$RTL/noc/ng_noc_pkg.sv $RTL/prim/ng_bitops_pkg.sv $RTL/l2/ng_l2_amo_alu.sv \
$RTL/lsu/ng_lsu_pkg.sv $RTL/lsu/ng_lsu_dec.sv $RTL/lsu/ng_lsu_fatomic.sv $RTL/lsu/ng_lsu_shmem.sv \
$RTL/lsu/ng_l1d.sv $RTL/lsu/ng_lsu.sv $RTL/lsu/ng_lsu_kn.sv $RTL/lsu/ng_lsu_top.sv $RTL/regfile/ng_sp_regfile.sv \
$RTL/tb/tb_lsu_vec.sv"
for cfg in ${LSU_CFGS:-16:4:2:0:0 32:256:4:3:1 16:2:1:5:1}; do
  IFS=: read -r nl sets ways lat shr <<EOF
$cfg
EOF
  tag="${nl}_${sets}_${ways}_${lat}_${shr}"
  {
    echo "\`define NG_TB_NL $nl"
    echo "\`define NG_TB_L1SETS $sets"
    echo "\`define NG_TB_L1WAYS $ways"
    echo "\`define NG_TB_MAXLAT $lat"
    echo "\`define NG_TB_SHRAND $shr"
    echo "\`define NG_TB_SHLPB ${SH_LPB:-8}"
    echo "\`define NG_TB_L1LPB ${L1_LPB:-4}"
    echo "\`define NG_TB_LSUK ${LSU_K:-1}"
    echo "\`define NG_TB_MEMOUT ${MEMOUT:-8}"
    echo "\`define NG_TB_HOSTMAP ${HOSTMAP:-0}"
  } > tb_lsu_cfg.svh
  if [ "$SIM" = verilator ]; then
    rm -rf "obj_$tag"
    ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-INITIALDLY --unroll-count 4096 --unroll-stmts 4000000 -Wno-TIMESCALEMOD --top-module tb_lsu_vec \
      -I. -Mdir "obj_$tag" $SRCS > "vl_$tag.build" 2>&1 || { tail -40 "vl_$tag.build"; exit 1; }
    "./obj_$tag/Vtb_lsu_vec" +VEC=lsu.txt ${VL_ARGS:-} > "sim_$tag.out" 2>&1 || true
  else
    "$BIN/xvlog.bat" -sv -i . $SRCS > "xvlog_$tag.out" 2>&1 || { cat "xvlog_$tag.out"; exit 1; }
    "$BIN/xelab.bat" -debug off -O0 tb_lsu_vec -s "tb_lsu_$tag" > "xelab_$tag.out" 2>&1 \
      || { cat "xelab_$tag.out"; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" ${XSIM_ARGS:-} "tb_lsu_$tag" > "sim_$tag.out" 2>&1 || true
  fi
  grep -E "FAIL|NOTE|NG_LSU_VEC|TB_ERROR|cov:|LAT |\(id=" "sim_$tag.out" | head -60 || true
  grep -q "NG_LSU_VEC_ALL_OK" "sim_$tag.out" || { echo "--- sim_$tag.out (head) ---"; head -40 "sim_$tag.out"; exit 1; }
done
echo "NG_LSU_VEC_TB_ALL_OK"
