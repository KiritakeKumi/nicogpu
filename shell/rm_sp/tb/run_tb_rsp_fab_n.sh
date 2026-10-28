#!/usr/bin/env bash
# unit test of the SP fabric port with several requests in flight -- ng_rsp_fab_ts (clk_ts) + ng_rsp_fab_l2
# (clk_l2, non-integer clock ratio) with the real engines, ng_rcp_axmux, ng_rsp_oparb, two random burst masters (DMA
# stand-ins) and tb_rmt_slave;
#   rtl/shell/rm_sp/tb/run_tb_rsp_fab_n.sh [work_dir]          Expect NG_RSP_FAB_N_TB_PASS for every configuration.
# Env: CONFIGS  space-separated NOUT,BURST,HOST_WR_FLUSH,WR_RUNS tuples (default: NOUT 1 2 4 8 with 1,0,1, then
#               8 with HOST_WR_FLUSH, 8 with BURST=0, 8 with WR_RUNS=0)
#      NREQ (default 3000), SEED (default 1)
#      SIM=verilator|xsim (default verilator).  xsim: Vivado xvlog / xelab / xsim.bat through cmd (Git Bash on
#      Windows; VIVADO_BIN, default C:/AMDDesignTools/2026.1/Vivado/bin); the parameters go through one small wrapper
#      top per configuration (no '=' on the xsim.bat command line).  VERILATOR, NJOBS for the Verilator path.
set -eu
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
WORK=${1:-${TMPDIR:-/tmp}/ng_tb_rsp_fab_n}
mkdir -p "$WORK"
cd "$WORK"
R=$ROOT/rtl
if command -v cygpath >/dev/null 2>&1; then R=$(cygpath -m "$R"); fi
FILES="$R/shell/generated/ng_dfx_pkg.sv $R/gpu/cp/ng_cp_pkg.sv $R/gpu/mmu/ng_mmu_pkg.sv $R/gpu/lsu/ng_lsu_pkg.sv $R/shell/prim/ng_prim_skid.sv $R/shell/prim/ng_prim_fifo_sync.sv $R/shell/rm_test/ng_rmt_prim.sv $R/shell/rm_test/ng_rmt_port.sv
 $R/shell/rm_cp/ng_rcp_axmux.sv $R/shell/rm_cp/ng_rcp_dma.sv $R/shell/rm_cp/ng_rcp_mem.sv $R/shell/rm_sp/ng_rsp_arb.sv
 $R/shell/rm_sp/ng_rsp_bridge.sv $R/shell/rm_test/tb/tb_rmt_slave.sv $R/shell/rm_sp/tb/tb_rsp_fab.sv
 $R/shell/rm_sp/tb/tb_rsp_fab_n.sv"
CONFIGS=${CONFIGS:-"1,1,0,1 2,1,0,1 4,1,0,1 8,1,0,1 8,1,1,1 8,0,0,1 8,1,0,0"}
NREQ=${NREQ:-3000}
SEED=${SEED:-1}
SIM=${SIM:-verilator}
rc=0
if [ "$SIM" = xsim ]; then
  BIN=${VIVADO_BIN:-C:/AMDDesignTools/2026.1/Vivado/bin}
  rm -rf xsim.dir
  {
    echo "-sv"
    for f in $FILES; do echo "$f"; done
    for cfg in $CONFIGS; do
      IFS=, read -r n b f w <<< "$cfg"
      echo "cfg_$n$b$f$w.sv"
      printf 'module tb_cfg_%s; tb_rsp_fab_n #(.NOUT(%s), .BURST(%s), .HOST_WR_FLUSH(%s), .WR_RUNS(%s), .NREQ(%s), .SEED(%s)) u; endmodule\n' \
        "$n$b$f$w" "$n" "$b" "$f" "$w" "$NREQ" "$SEED" > "cfg_$n$b$f$w.sv"
    done
  } > xv.f
  cmd //c "$BIN/xvlog.bat" -f xv.f > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
  for cfg in $CONFIGS; do
    IFS=, read -r n b f w <<< "$cfg"
    t=$n$b$f$w
    echo "== NOUT=$n BURST=$b HOST_WR_FLUSH=$f WR_RUNS=$w"
    cmd //c "$BIN/xelab.bat" -debug off -O1 "tb_cfg_$t" -s "tb_cfg_$t" > "xelab_$t.out" 2>&1 \
      || { grep -E "ERROR" "xelab_$t.out" | head -40; exit 1; }
    printf 'run -all\nquit\n' | cmd //c "$BIN/xsim.bat" "tb_cfg_$t" > "run_$t.out" 2>&1 || true
    grep -E "FAIL|PASS|TEST|tb_rsp_fab_n:|line reads|accepted|DUT:|dma bursts|mismatch|ERROR" "run_$t.out" | head -60
    grep -q NG_RSP_FAB_N_TB_PASS "run_$t.out" || rc=1
  done
  exit $rc
fi
for cfg in $CONFIGS; do
  IFS=, read -r n b f w <<< "$cfg"
  t=$n$b$f$w
  echo "== NOUT=$n BURST=$b HOST_WR_FLUSH=$f WR_RUNS=$w"
  rm -rf "obj_$t"
  # shellcheck disable=SC2086
  ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} --binary --timing -j "${NJOBS:-8}" -Wno-fatal -Wno-lint -Wno-style \
    -Wno-TIMESCALEMOD -Wno-INITIALDLY \
    -GNOUT=$n -GBURST=$b -GHOST_WR_FLUSH=$f -GWR_RUNS=$w -GNREQ=$NREQ -GSEED=$SEED \
    --top-module tb_rsp_fab_n --Mdir "obj_$t" -o sim $FILES > "vl_build_$t.out" 2>&1 \
    || { grep -E "%Error" "vl_build_$t.out" | head -20; tail -20 "vl_build_$t.out"; exit 1; }
  "./obj_$t/sim" > "vl_run_$t.out" 2>&1 || true
  grep -E "FAIL|PASS|TEST|tb_rsp_fab_n:|line reads|accepted|DUT:|dma bursts|mismatch|ERROR" "vl_run_$t.out" | head -60
  grep -q NG_RSP_FAB_N_TB_PASS "vl_run_$t.out" || rc=1
done
exit $rc
