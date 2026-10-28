#!/usr/bin/env bash
# run the register-file testbench with Vivado xsim over seeds and lane
# configurations.
#   rtl/gpu/tb/run_tb_regfile.sh [vivado_bin_dir] [work_dir]
# Env: RF_SEEDS="1 2 3 4 5"  RF_NLS="16 32"  RF_CYCLES=12000
# Expect one NG_RF_TB_PASS line per (seed, NL).  Use a fresh work_dir each time.
# (xsim.bat cannot forward valued plusargs, so the config is regenerated into
# tb_regfile_cfg.svh and recompiled per run.)
set -eu
BIN=${1:-C:/AMDDesignTools/2026.1/Vivado/bin}
WORK=${2:-build/gpu/tb_rf}
SRC=$(cd "$(dirname "$0")/../.." && pwd); SRC=$SRC/gpu
case $WORK in /*|?:*) ;; *) WORK=$(pwd)/$WORK ;; esac
mkdir -p "$WORK"
cd "$WORK"
for nl in ${RF_NLS:-16 32}; do
  for s in ${RF_SEEDS:-1 2 3 4 5}; do
    printf '`define NG_TB_SEED %s\n`define NG_TB_NL %s\n`define NG_TB_CYCLES %s\n' \
      "$s" "$nl" "${RF_CYCLES:-12000}" > tb_regfile_cfg.svh
    "$BIN/xvlog.bat" -sv -i . -d NG_TB_CFG_FILE "$SRC/noc/ng_noc_pkg.sv" "$SRC/regfile/ng_sp_regfile.sv" \
      "$SRC/tb/tb_regfile.sv" > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
    "$BIN/xelab.bat" -debug off tb_regfile -s tb_rf > xelab.out 2>&1 || { cat xelab.out; exit 1; }
    printf 'run -all\nquit\n' | "$BIN/xsim.bat" tb_rf > "xsim_${nl}_${s}.out" 2>&1 || true
    grep -E "FAIL|PASS|cov:" "xsim_${nl}_${s}.out" || true
    grep -q "NG_RF_TB_PASS" "xsim_${nl}_${s}.out"
  done
done
echo "NG_RF_TB_ALL_OK"
