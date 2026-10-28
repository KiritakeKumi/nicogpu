#!/usr/bin/env bash
# system test of the CP RM (rtl/shell/rm_cp) with the real Ibex firmware, Verilator --timing.
#   rtl/shell/rm_cp/tb/run_tb_rm_cp.sh [work_dir]
# Env: IBEX (checkout, default <repo>/third_party/ibex), RISCV (tool prefix, default riscv64-unknown-elf-),
#      VOPTS (extra verilator args), NJOBS.  Expect NG_RM_CP_TB_PASS.
#      SIM=verilator|xsim (default verilator; Verilator path: Linux only).  SIM=xsim drives Vivado xvlog / xelab /
#      xsim.bat through cmd (Git Bash on Windows, NICOGPU_VIVADO or VIVADO_BIN, default C:/AMDDesignTools/2026.1/Vivado/bin),
#      the same way as rtl/shell/rm_sp/tb/run_tb_rm_sp.sh (argument file, ibex_if_stage.sv without its DPI exports).
#      FW_NGCF=<file>: a prebuilt NGCF firmware image instead of building fw/cp (rm_fw_build in tb_lib.sh).
# The firmware build, Ibex file list, port declarations and xvlog argument file are shared with run_tb_rm_sp.sh
# (rtl/shell/tb/tb_lib.sh); the testbench tasks the two TBs have in common are in tb_rm_cp_common.svh.
set -eu
set -o pipefail
. "$(dirname "$0")/../../tb/tb_lib.sh"
WORK=${1:-${TMPDIR:-/tmp}/ng_tb_rm_cp}
RISCV=${RISCV:-riscv64-unknown-elf-}
NJOBS=${NJOBS:-8}
ROOT=$TB_ROOT
IBEX=${IBEX:-$ROOT/third_party/ibex}
RTL=$ROOT/rtl
SIM=${SIM:-verilator}
mkdir -p "$WORK"
cd "$WORK"

rm_fw_build                                   # firmware -> NGCF image (fw_ngcf.hex)
rm_ibex_flist                                 # Ibex file list + prim shim (ibex.f, shim/)
rm_port_decls > tb_rm_cp_decls.svh            # port declarations of the DUT as TB signals

# ---- verilate + run
export NGRTL=$RTL
FILES="$RTL/shell/generated/ng_dfx_pkg.sv $(flist "$RTL/gpu/cp/cp_mmu_files.f" | tr '\n' ' ')
 $RTL/shell/prim/ng_prim_skid.sv $RTL/shell/prim/ng_prim_fifo_sync.sv $RTL/shell/rm_test/ng_rmt_prim.sv $RTL/shell/rm_test/ng_rmt_port.sv
 $RTL/shell/rm_cp/ng_rcp_regs.sv $RTL/shell/rm_cp/ng_rcp_cpbus.sv $RTL/shell/rm_cp/ng_rcp_axmux.sv $RTL/shell/rm_cp/ng_rcp_dma.sv $RTL/shell/rm_cp/ng_rcp_mem.sv
 $RTL/shell/rm_cp/ng_rcp_mmu.sv $RTL/shell/rm_cp/ng_rcp_bar2.sv $RTL/shell/rm_cp/ng_gpu_rm_cp.sv
 $RTL/shell/rm_test/tb/tb_rmt_slave.sv $RTL/shell/rm_cp/tb/tb_rm_cp.sv"
if [ "$SIM" = xsim ]; then
  BIN=$(vivado_bin)
  rm_xsim_prep
  rm_xv_f "-d NG_CP_IBEX"
  cmd //c "$BIN/xvlog.bat" -f xv.f > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
  cmd //c "$BIN/xelab.bat" -debug off -O1 tb_rm_cp -s tb_rm_cp > xelab.out 2>&1 || { grep -E "ERROR" xelab.out | head -40; exit 1; }
  printf 'run -all\nquit\n' | cmd //c "$BIN/xsim.bat" tb_rm_cp > vl_run.out 2>&1 || true
  grep -E "^TEST|FAIL|PASS|tb_rm_cp:" vl_run.out | head -80
  grep -q NG_RM_CP_TB_PASS vl_run.out
  exit $?
fi
rm -rf obj
# shellcheck disable=SC2086
verilator --binary --timing -j "$NJOBS" -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
  --top-module tb_rm_cp --Mdir obj -o sim -DNG_CP_IBEX "+incdir+$RTL/shell/generated" "+incdir+$RTL/shell/rm_cp/tb" "+incdir+$WORK" \
  "+incdir+$IBEX/rtl" "+incdir+$P/ip/prim/rtl" "+incdir+$P/dv/sv/dv_utils" ${VOPTS:-} \
  "$IBEX/lint/verilator_waiver.vlt" -f ibex.f $FILES > vl_build.out 2>&1 || { tail -50 vl_build.out; exit 1; }
./obj/sim > vl_run.out 2>&1 || true
grep -E "^TEST|FAIL|PASS|tb_rm_cp:" vl_run.out | head -80
grep -q NG_RM_CP_TB_PASS vl_run.out
