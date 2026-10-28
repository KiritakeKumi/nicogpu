#!/usr/bin/env bash
# unit test of the SP fabric port memory side (line request -> AXI bursts / 64-bit operations) with the real
# engines, the ng_rcp_axmux in front of their burst ports, a random burst master (DMA stand-in) and tb_rmt_slave.
# the TB drives ng_rsp_fab_ts + ng_rsp_fab_l2 (one clock, one request at a time); the multi-request test
# two clocks is run_tb_rsp_fab_n.sh.
#   rtl/shell/rm_sp/tb/run_tb_rsp_fab.sh [work_dir]          Expect NG_RSP_FAB_TB_PASS for every configuration.
# Env: CONFIGS  BURST,HOST_WR_FLUSH[,NOUT[,WR_RUNS]] tuples (default "1,0 1,1 0,0 1,0,1,0"; NOUT default 8, WR_RUNS
#               default 1; "1,0,1,0" = the configuration: one slot host write rule)
#      NRAND (default 400), EXTRA_G (more verilator -G options)
#      SIM=verilator|xsim (default verilator).  xsim: Vivado xvlog / xelab / xsim.bat through cmd (Git Bash on
#      Windows; VIVADO_BIN, default C:/AMDDesignTools/2026.1/Vivado/bin), parameters through a wrapper top per
#      configuration.  VERILATOR, NJOBS for the Verilator path.
set -eu
ROOT=$(cd "$(dirname "$0")/../../../.." && pwd)
WORK=${1:-${TMPDIR:-/tmp}/ng_tb_rsp_fab}
mkdir -p "$WORK"
cd "$WORK"
R=$ROOT/rtl
if command -v cygpath >/dev/null 2>&1; then R=$(cygpath -m "$R"); fi
FILES="$R/shell/generated/ng_dfx_pkg.sv $R/gpu/cp/ng_cp_pkg.sv $R/gpu/mmu/ng_mmu_pkg.sv $R/gpu/lsu/ng_lsu_pkg.sv $R/shell/prim/ng_prim_skid.sv $R/shell/prim/ng_prim_fifo_sync.sv $R/shell/rm_test/ng_rmt_prim.sv $R/shell/rm_test/ng_rmt_port.sv
 $R/shell/rm_cp/ng_rcp_axmux.sv $R/shell/rm_cp/ng_rcp_dma.sv $R/shell/rm_cp/ng_rcp_mem.sv $R/shell/rm_sp/ng_rsp_arb.sv
 $R/shell/rm_sp/ng_rsp_bridge.sv $R/shell/rm_test/tb/tb_rmt_slave.sv $R/shell/rm_sp/tb/tb_rsp_fab.sv"
CONFIGS=${CONFIGS:-"1,0 1,1 0,0 1,0,1,0"}
NRAND=${NRAND:-400}
SIM=${SIM:-verilator}
cfg4() {   # BURST,FLUSH[,NOUT[,WR_RUNS]] -> "b f n w"
  IFS=, read -r b f n w <<< "$1"
  echo "$b ${f:-0} ${n:-8} ${w:-1}"
}
rc=0
if [ "$SIM" = xsim ]; then
  BIN=${VIVADO_BIN:-C:/AMDDesignTools/2026.1/Vivado/bin}
  rm -rf xsim.dir
  {
    echo "-sv"
    for f in $FILES; do echo "$f"; done
    for cfg in $CONFIGS; do
      read -r b f n w <<< "$(cfg4 "$cfg")"
      echo "cfg_$b$f$n$w.sv"
      printf 'module tb_cfg_%s; tb_rsp_fab #(.BURST(%s), .HOST_WR_FLUSH(%s), .NOUT(%s), .WR_RUNS(%s), .NRAND(%s)) u; endmodule\n' \
        "$b$f$n$w" "$b" "$f" "$n" "$w" "$NRAND" > "cfg_$b$f$n$w.sv"
    done
  } > xv.f
  cmd //c "$BIN/xvlog.bat" -f xv.f > xvlog.out 2>&1 || { grep -E "ERROR" xvlog.out | head -40; exit 1; }
  for cfg in $CONFIGS; do
    read -r b f n w <<< "$(cfg4 "$cfg")"
    t=$b$f$n$w
    echo "== BURST=$b HOST_WR_FLUSH=$f NOUT=$n WR_RUNS=$w"
    cmd //c "$BIN/xelab.bat" -debug off -O1 "tb_cfg_$t" -s "tb_cfg_$t" > "xelab_$t.out" 2>&1 \
      || { grep -E "ERROR" "xelab_$t.out" | head -40; exit 1; }
    printf 'run -all\nquit\n' | cmd //c "$BIN/xsim.bat" "tb_cfg_$t" > "run_$t.out" 2>&1 || true
    grep -E "FAIL|PASS|TEST|tb_rsp_fab|cycles/line|dma bursts|slots:|mismatch|ERROR" "run_$t.out" | head -60
    grep -q NG_RSP_FAB_TB_PASS "run_$t.out" || rc=1
  done
  exit $rc
fi
for cfg in $CONFIGS; do
  read -r b f n w <<< "$(cfg4 "$cfg")"
  t=$b$f$n$w
  echo "== BURST=$b HOST_WR_FLUSH=$f NOUT=$n WR_RUNS=$w"
  rm -rf "obj_$t"
  # shellcheck disable=SC2086
  ${VERILATOR:-verilator} ${VL_MAKEFLAGS:+-MAKEFLAGS "$VL_MAKEFLAGS"} --binary --timing -j "${NJOBS:-8}" -Wno-fatal -Wno-lint -Wno-style \
    -Wno-TIMESCALEMOD -Wno-INITIALDLY \
    -GBURST=$b -GHOST_WR_FLUSH=$f -GNOUT=$n -GWR_RUNS=$w -GNRAND=$NRAND ${EXTRA_G:-} \
    --top-module tb_rsp_fab --Mdir "obj_$t" -o sim $FILES > "vl_build_$t.out" 2>&1 \
    || { grep -E "%Error" "vl_build_$t.out" | head -20; tail -20 "vl_build_$t.out"; exit 1; }
  "./obj_$t/sim" > "vl_run_$t.out" 2>&1 || true
  grep -E "FAIL|PASS|TEST|tb_rsp_fab|cycles/line|dma bursts|slots:|mismatch|ERROR" "vl_run_$t.out" | head -60
  grep -q NG_RSP_FAB_TB_PASS "vl_run_$t.out" || rc=1
done
exit $rc
