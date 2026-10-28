#!/usr/bin/env bash
# system test of the M1 SP RM (rtl/shell/rm_sp = rm_cp + one SP) with the real Ibex firmware, Verilator --timing.
#   rtl/shell/rm_sp/tb/run_tb_rm_sp.sh [work_dir]
# Env: IBEX (checkout, default <repo>/third_party/ibex), RISCV (tool prefix, default riscv64-unknown-elf-),
#      BUNDLES (dispatch bundles, default <repo>/build/gpu/bundles: k_vadd k_ids k_intops k_loop k_vadd_big
#      k_trap_explicit k_trap_lsu k_cacheop  k_scratch  k_f16cvt  k_xlane  from tools/dispatch_bundle/mkbundles.sh + k_atom, k_spin
#      rtl/shell/rm_sp/tb/kernels 8), VOPTS (extra verilator args), NJOBS.
#      SIM=verilator|xsim (default verilator).  SIM=xsim drives Vivado xvlog/xelab/xsim.bat (NICOGPU_VIVADO or VIVADO_BIN,
#      default C:/AMDDesignTools/2026.1/Vivado/bin; only '-d NAME' defines without '=' because xsim.bat mangles '=').
#      FW_NGCF=<file>: use a prebuilt NGCF firmware image instead of building fw/cp with the RISC-V toolchain
#      (tools/ngcf/ngcf_pack.py hex re-packs it and requires a byte-identical result); BUNDLES may point to a
#      local copy of the bundle directories.
#      TB_PARAMS="NAME=value ..." (xsim): parameter overrides of tb_rm_sp through a wrapper top (xsim.bat cannot take
#      '='), e.g. TB_PARAMS="SP_NOUT=1" or "SP_MEM_OUT=1" ; the Verilator path takes VOPTS=-GNAME=value.
# Verilator path: Linux only.  Expect NG_RM_SP_TB_PASS.
# The firmware build, Ibex file list, port declarations and xvlog argument file are shared with run_tb_rm_cp.sh
# (rtl/shell/tb/tb_lib.sh); the testbench tasks the two TBs have in common are in rm_cp/tb/tb_rm_cp_common.svh.
set -eu
set -o pipefail
. "$(dirname "$0")/../../tb/tb_lib.sh"
WORK=${1:-${TMPDIR:-/tmp}/ng_tb_rm_sp}
RISCV=${RISCV:-riscv64-unknown-elf-}
NJOBS=${NJOBS:-8}
ROOT=$TB_ROOT
IBEX=${IBEX:-$ROOT/third_party/ibex}
BUNDLES=${BUNDLES:-$ROOT/build/gpu/bundles}
RTL=$ROOT/rtl
SIM=${SIM:-verilator}
mkdir -p "$WORK/bundles"
cd "$WORK"

# ---- bundles (the TB reads bundles/<name>/*.hex)
for b in k_vadd k_ids k_intops k_loop k_atom k_spin k_vadd_big k_trap_explicit k_trap_lsu k_cacheop k_scratch k_f16cvt k_xlane; do
  [ -f "$BUNDLES/$b/meta.hex" ] || { echo "missing bundle $BUNDLES/$b ()"; exit 2; }
  rm -rf "bundles/$b"; cp -r "$BUNDLES/$b" "bundles/$b"
done

rm_fw_build                                   # firmware -> NGCF image (fw_ngcf.hex)
rm_ibex_flist                                 # Ibex file list + prim shim (ibex.f, shim/)
rm_port_decls > tb_rm_sp_decls.svh            # port declarations of the DUT as TB signals

# ---- verilate + run
export NGRTL=$RTL
FILES="$RTL/shell/generated/ng_dfx_pkg.sv $(flist "$RTL/gpu/cp/cp_mmu_files.f" | tr '\n' ' ')
 $RTL/shell/prim/ng_prim_skid.sv $RTL/shell/prim/ng_prim_fifo_sync.sv $RTL/shell/rm_test/ng_rmt_prim.sv $RTL/shell/rm_test/ng_rmt_port.sv
 $(flist "$RTL/gpu/sp/sp_files.f" | tr '\n' ' ')
 $RTL/shell/rm_cp/ng_rcp_cpbus.sv $RTL/shell/rm_cp/ng_rcp_axmux.sv $RTL/shell/rm_cp/ng_rcp_dma.sv $RTL/shell/rm_cp/ng_rcp_mem.sv $RTL/shell/rm_cp/ng_rcp_mmu.sv
 $RTL/shell/rm_cp/ng_rcp_bar2.sv
 $RTL/shell/rm_cp/ng_rcp_regs.sv $RTL/shell/rm_sp/ng_rsp_arb.sv $RTL/shell/rm_sp/ng_rsp_bridge.sv
 $RTL/shell/rm_sp/ng_rsp_dbg.sv $RTL/shell/rm_sp/ng_gpu_rm_sp.sv
 $RTL/shell/rm_test/tb/tb_rmt_slave.sv $RTL/shell/rm_sp/tb/tb_rm_sp.sv"
if [ "$SIM" = xsim ]; then
  BIN=$(vivado_bin)
  # one argument file: the command line of 100+ sources is longer than cmd.exe allows
  rm_xsim_prep
  if [ -n "${TB_PARAMS:-}" ]; then rm_xv_f "-d NG_SP_ASSERT -d NG_CP_IBEX" tb_rm_sp_cfg.sv; else rm_xv_f "-d NG_SP_ASSERT -d NG_CP_IBEX"; fi
  TOP=tb_rm_sp
  if [ -n "${TB_PARAMS:-}" ]; then
    ov=$(for kv in $TB_PARAMS; do printf '.%s(%s), ' "${kv%%=*}" "${kv#*=}"; done)
    printf 'module tb_rm_sp_cfg; tb_rm_sp #(%s) u; endmodule\n' "${ov%, }" > tb_rm_sp_cfg.sv
    TOP=tb_rm_sp_cfg
  fi
  cmd //c "$BIN/xvlog.bat" -f xv.f > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
  cmd //c "$BIN/xelab.bat" -debug off -O1 $TOP -s tb_rm_sp > xelab.out 2>&1 || { grep -E "ERROR" xelab.out | head -40; exit 1; }
  printf 'run -all\nquit\n' | cmd //c "$BIN/xsim.bat" tb_rm_sp > vl_run.out 2>&1 || true
  grep -E "^TEST|FAIL|PASS|MISMATCH|tb_rm_sp:|  (debug|concurrent|bundle|mmu|abort)|MMU/FAB" vl_run.out | head -150
  grep -q NG_RM_SP_TB_PASS vl_run.out
  exit $?
fi
rm -rf obj
# shellcheck disable=SC2086
verilator --binary --timing -j "$NJOBS" -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD -Wno-INITIALDLY \
  --unroll-count 4096 --unroll-stmts 4000000 +define+NG_SP_ASSERT \
  --top-module tb_rm_sp --Mdir obj -o sim -DNG_CP_IBEX "+incdir+$RTL/shell/generated" "+incdir+$RTL/shell/rm_cp/tb" "+incdir+$WORK" \
  "+incdir+$IBEX/rtl" "+incdir+$P/ip/prim/rtl" "+incdir+$P/dv/sv/dv_utils" ${VOPTS:-} \
  "$IBEX/lint/verilator_waiver.vlt" -f ibex.f $FILES > vl_build.out 2>&1 || { grep -E "%Error" vl_build.out | head -40; tail -30 vl_build.out; exit 1; }
./obj/sim > vl_run.out 2>&1 || true
grep -E "^TEST|FAIL|PASS|MISMATCH|tb_rm_sp:|  (debug|concurrent|bundle|mmu)" vl_run.out | head -150
grep -q NG_RM_SP_TB_PASS vl_run.out
