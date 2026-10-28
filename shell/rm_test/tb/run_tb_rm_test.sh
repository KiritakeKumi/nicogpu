#!/usr/bin/env bash
# run the test-RM unit testbench with Vivado xsim.
#   rtl/shell/rm_test/tb/run_tb_rm_test.sh [vivado_bin_dir] [work_dir]
# Defaults: C:/AMDDesignTools/2026.1/Vivado/bin and build/tb_rmtest/run
# (relative to the caller; build/ is not tracked).  Expect NG_RM_TEST_TB_PASS.
# Use a fresh work_dir per concurrent run (xsim.dir locking).
# Env: XSIM_ARGS="-sv_seed 7 -testplusarg quick" passes seed / plusargs.
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-build/tb_rmtest/run}
SRC=$(cd "$(dirname "$0")/../../.." && pwd)/shell
mkdir -p "$WORK"
cd "$WORK"
"$BIN/xvlog.bat" -sv -i "$SRC/generated" "$SRC/generated/ng_dfx_pkg.sv" "$SRC"/rm_test/*.sv \
  "$SRC/prim/ng_prim_skid.sv" "$SRC/prim/ng_prim_fifo_sync.sv" \
  "$SRC/rm_test/tb/tb_rmt_slave.sv" "$SRC/rm_test/tb/tb_rm_test.sv" > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
"$BIN/xelab.bat" -debug off tb_rm_test -s tb > xelab.out 2>&1 || { cat xelab.out; exit 1; }
"$BIN/xsim.bat" tb -runall ${XSIM_ARGS:-} > xsim.out 2>&1 || true
grep -E "FAIL|PASS|ERROR|^TEST|^INFO|^SUMMARY" xsim.out | head -120
grep -q NG_RM_TEST_TB_PASS xsim.out
