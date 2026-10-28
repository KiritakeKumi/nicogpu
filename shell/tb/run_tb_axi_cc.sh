#!/usr/bin/env bash
# P0-11: run the ng_axi_cc async clock-converter testbench with Vivado xsim.
#   rtl/shell/tb/run_tb_axi_cc.sh [vivado_bin_dir] [work_dir]
# Runs several width / depth / clock-ratio configs; expect one
# NG_AXI_CC_TB_PASS per config.  Use a fresh work_dir each time.
# NG_AXI_CC_DUT=<file> overrides the DUT source (used for bug-injection runs).
set -eu
. "$(dirname "$0")/tb_lib.sh"
BIN=${1:-$(vivado_bin)}
WORK=${2:-build/tb_axi_cc}
SRC=$TB_SHELL
DUT=${NG_AXI_CC_DUT:-$SRC/ng_axi_cc.sv}
mkdir -p "$WORK"
cd "$WORK"
# name  seed  parameter overrides (xelab.bat mangles -generic_top "A=B", so
# each config is a tiny generated wrapper module instead)
CONFIGS=(
  "c1_64b_fast2slow  11 .ADDR_W(32),.DATA_W(64),.DEPTH(4),.S_PER(4000),.M_PER(7519)"
  "c2_64b_slow2fast  22 .ADDR_W(33),.DATA_W(64),.DEPTH(8),.S_PER(7519),.M_PER(3333)"
  "c3_512b_slow2fast 33 .ADDR_W(33),.DATA_W(512),.DEPTH(4),.S_PER(4000),.M_PER(3333)"
  "c4_64b_neareq     44 .ADDR_W(32),.DATA_W(64),.DEPTH(16),.S_PER(4000),.M_PER(4093)"
  "c5_64b_syncrd     55 .ADDR_W(32),.DATA_W(64),.DEPTH(4),.S_PER(7519),.M_PER(3333),.SYNC_RD(1)"
  "c6_256b_fast2slow 66 .ADDR_W(32),.DATA_W(256),.DEPTH(16),.S_PER(3333),.M_PER(4000)"
)
: > cfg.sv
for c in "${CONFIGS[@]}"; do
  set -- $c
  echo "module $1; tb_axi_cc #($3) tb; endmodule" >> cfg.sv
done
xsim_compile "$SRC/prim/ng_prim_sync.sv" "$SRC/prim/ng_prim_fifo_async.sv" "$DUT" "$SRC/prim/ng_prim_skid.sv" "$SRC/tb/tb_axi_cc.sv" cfg.sv
npass=0
for c in "${CONFIGS[@]}"; do
  set -- $c
  name=$1; seed=$2
  "$BIN/xelab.bat" -debug off "$name" -s "sim_$name" > "xelab_$name.out" 2>&1 \
    || { cat "xelab_$name.out"; exit 1; }
  t0=$(date +%s)
  "$BIN/xsim.bat" "sim_$name" -runall -sv_seed "$seed" > "xsim_$name.out" 2>&1 || true
  echo "== $name (seed $seed, $(( $(date +%s) - t0 )) s)"
  grep -E "FAIL|PASS|ERROR|Fatal|phase|backpressure" "xsim_$name.out" || true
  grep -q NG_AXI_CC_TB_PASS "xsim_$name.out" && npass=$((npass + 1))
done
echo "configs passed: $npass/${#CONFIGS[@]}"
[ "$npass" -eq "${#CONFIGS[@]}" ]
