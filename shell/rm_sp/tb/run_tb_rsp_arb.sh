#!/usr/bin/env bash
# unit test of the SP RM arbiters (ng_rsp_mmuarb / ng_rsp_oparb), Verilator --timing.
#   rtl/shell/rm_sp/tb/run_tb_rsp_arb.sh [work_dir]          Expect NG_RSP_ARB_TB_PASS.
# Linux: verilator on PATH.  Windows (Git Bash): set VERILATOR_ROOT / PATH to the Windows Verilator package
# (see rtl/gpu/tb/run_tb_sp_top_vl.sh) and VL_MAKEFLAGS="CXX=g++ CC=gcc LINK=g++".
# NJOBS=<n>: Verilator -j (unset = Verilator default).
# SIM=xsim: Vivado xvlog / xelab / xsim.bat through cmd instead (VIVADO_BIN, default C:/AMDDesignTools/2026.1/Vivado/bin).
set -eu
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
WORK=${1:-${TMPDIR:-/tmp}/ng_tb_rsp_arb}
mkdir -p "$WORK"
cd "$WORK"
S=$ROOT/rtl/shell/rm_sp
if command -v cygpath >/dev/null 2>&1; then S=$(cygpath -m "$S"); fi
if [ "${SIM:-verilator}" = xsim ]; then
  BIN=${VIVADO_BIN:-C:/AMDDesignTools/2026.1/Vivado/bin}
  rm -rf xsim.dir
  cmd //c "$BIN/xvlog.bat" -sv "$S/ng_rsp_arb.sv" "$S/tb/tb_rsp_arb.sv" > xvlog.out 2>&1 || { grep ERROR xvlog.out | head -20; exit 1; }
  cmd //c "$BIN/xelab.bat" -debug off -O1 tb_rsp_arb -s tb_rsp_arb > xelab.out 2>&1 || { grep ERROR xelab.out | head -20; exit 1; }
  printf 'run -all\nquit\n' | cmd //c "$BIN/xsim.bat" tb_rsp_arb > vl_run.out 2>&1 || true
  grep -E "FAIL|PASS|arb |tb_rsp_arb" vl_run.out | head -40
  grep -q NG_RSP_ARB_TB_PASS vl_run.out
  exit $?
fi
rm -rf obj
${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} ${NJOBS:+-j "$NJOBS"} --binary --timing -Wno-fatal -Wno-lint -Wno-style \
  --top-module tb_rsp_arb --Mdir obj -o sim "$S/ng_rsp_arb.sv" "$S/tb/tb_rsp_arb.sv" > vl_build.out 2>&1 \
  || { grep -E "%Error" vl_build.out | head -20; tail -20 vl_build.out; exit 1; }
./obj/sim > vl_run.out 2>&1 || true
grep -E "FAIL|PASS|arb |tb_rsp_arb" vl_run.out | head -40
grep -q NG_RSP_ARB_TB_PASS vl_run.out
