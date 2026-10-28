#!/usr/bin/env bash
# tb_lib.sh -- shared helpers of the testbench runners (rtl/shell/tb/run_tb_*.sh, rtl/shell/rm_{cp,sp}/tb/run_tb_rm_*.sh,
# rtl/gpu/tb/run_tb_{core,sp_top}.sh).  Source it, do not execute it:
# . "$(dirname "$0")/tb_lib.sh"
# Sets TB_SHELL (rtl/shell), TB_RTL (rtl) and TB_ROOT (repo root).

TB_LIB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TB_SHELL=$(cd "$TB_LIB_DIR/.." && pwd)
TB_RTL=$(cd "$TB_LIB_DIR/../.." && pwd)
TB_ROOT=$(cd "$TB_LIB_DIR/../../.." && pwd)

# vivado_bin: the Vivado bin directory.  NICOGPU_VIVADO (the vivado.bat path of syn/run_vivado.sh) wins, then VIVADO_BIN
# (the directory), then the 2026.1 default.
vivado_bin() {
  if [ -n "${NICOGPU_VIVADO:-}" ]; then dirname "$NICOGPU_VIVADO"
  elif [ -n "${VIVADO_BIN:-}" ]; then echo "$VIVADO_BIN"
  else echo "C:/AMDDesignTools/2026.1/Vivado/bin"; fi
}

# mp <path>: the path in the form xvlog / xelab / xsim read from argument files (mixed C:/... on Git Bash / MSYS).
mp() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else echo "$1"; fi; }

# ---- xsim (BIN = vivado_bin; run inside the work directory) -------------------------------------------------------

# xsim_compile <xvlog files / args...>: xvlog -sv, the log in xvlog.out (dumped, then exit 1, on failure).
xsim_compile() {
  "$BIN/xvlog.bat" -sv "$@" > xvlog.out 2>&1 || { cat xvlog.out; exit 1; }
}

# xsim_run <top> <pass token> <xvlog files...>: compile, elaborate <top> (snapshot tb), run to the end (XSIM_ARGS = extra
# xsim arguments, e.g. "-testplusarg x -sv_seed 7"), print the FAIL / PASS / ERROR lines; the exit status is that of
# "the pass token is in xsim.out".
xsim_run() {
  local top=$1 token=$2
  shift 2
  xsim_compile "$@"
  "$BIN/xelab.bat" -debug off "$top" -s tb > xelab.out 2>&1 || { cat xelab.out; exit 1; }
  "$BIN/xsim.bat" tb -runall ${XSIM_ARGS:-} > xsim.out 2>&1 || true
  grep -E "FAIL|PASS|ERROR" xsim.out
  grep -q "$token" xsim.out
}

# ---- file lists ------------------------------------------------------------------------------------------------------

# flist <file.f>: the paths of a filelist (rtl/gpu/sp/sp_files.f, rtl/gpu/cp/cp_mmu_files.f, ...), one per line, comments
# dropped.  $NGRTL (the rtl/ directory, default TB_RTL) is expanded like $IBEX / $SHIM in ibex_files.f.  (tr: the MinGW
# envsubst writes CRLF, which would end up in the paths.)
flist() { NGRTL=${NGRTL:-$TB_RTL} envsubst '$NGRTL' < "$1" | tr -d '\r' | grep -v '^#' | grep -v '^$'; }

# ---- CP RM system tests (rm_cp / rm_sp): Ibex firmware, Ibex file list, DUT port declarations --------------------------
# They expect ROOT (repo root), RTL (= $ROOT/rtl), IBEX, SIM, RISCV and the current directory = the work directory.

# rm_fw_build: build fw/cp with the RISC-V toolchain and pack it to fw_ngcf.hex (+ rv/fw.ngcf).  FW_NGCF=<file>: use a
# prebuilt NGCF image instead (tools/ngcf/ngcf_pack.py hex re-packs it and requires a byte-identical result).
rm_fw_build() {
  local fw=$ROOT/fw/cp w=$PWD f m="-march=rv32imc_zicsr -mabi=ilp32"
  local srcs="src/ngfw_core.c src/ngfw_pkt.c src/ngfw_util.c src/ngfw_plat_ibex.c src/ngfw_ibex_main.c"
  mkdir -p rv
  if [ -n "${FW_NGCF:-}" ]; then
    python3 "$ROOT/tools/ngcf/ngcf_pack.py" hex "$FW_NGCF" fw_ngcf.hex
    return
  fi
  (cd "$fw" && for f in $srcs; do
     ${RISCV}gcc $m -mcmodel=medany -std=c99 -Wall -Wextra -Werror -pedantic -ffreestanding -nostdlib \
       -ffunction-sections -fdata-sections -DNGFW_IBEX -Os -Iinclude -c "$f" -o "$w/rv/$(basename "$f" .c).o"
   done)
  # --build-id=none: a toolchain that emits a build-id note by default (e.g. the Vitis riscv64-unknown-elf 13.4 on Windows)
  # would place it at address 0 and push the vector table to 0x24
  ${RISCV}gcc $m -nostdlib -Wl,--gc-sections -Wl,--build-id=none -T "$fw/ibex/link.ld" "$fw/ibex/crt0.S" rv/ngfw_*.o \
    -lgcc -o rv/fw.elf
  ${RISCV}objcopy -O binary -j .text rv/fw.elf rv/fw.bin
  local bss=$(( 0x$(${RISCV}nm rv/fw.elf | awk '$3=="__bss_end"{print $1}') - 0x$(${RISCV}nm rv/fw.elf | awk '$3=="__bss_start"{print $1}') ))
  python3 "$ROOT/tools/ngcf/ngcf_pack.py" pack --text rv/fw.bin --bss "$bss" --entry 0x80 --hex fw_ngcf.hex -o rv/fw.ngcf
}

# rm_ibex_flist: the Ibex prim shim (shim/) and ibex.f (ibex_files.f with $IBEX / $SHIM expanded).  Under SIM=xsim on
# Windows IBEX and SHIM are mixed-form paths.  Sets/exports IBEX, SHIM and P (= $IBEX/vendor/lowrisc_ip).
rm_ibex_flist() {
  bash "$RTL/gpu/cp/ibex/gen_prim_shim.sh" "$IBEX" shim
  if [ "$SIM" = xsim ] && command -v cygpath >/dev/null 2>&1; then IBEX=$(cygpath -m "$IBEX"); SHIM=$(cygpath -m "$PWD")/shim
  else SHIM=$PWD/shim; fi
  export IBEX SHIM
  envsubst '$IBEX $SHIM' < "$RTL/gpu/cp/ibex/ibex_files.f" | grep -v '^#' > ibex.f
  P=$IBEX/vendor/lowrisc_ip
}

# rm_port_decls: <rtl/shell/generated/ng_gpu_rm_ports.svh> as TB signal declarations on stdout.
rm_port_decls() {
  python3 - "$RTL/shell/generated/ng_gpu_rm_ports.svh" <<'PYEOF'
import re, sys
for line in open(sys.argv[1]):
    m = re.match(r'\s*(input|output)\s+logic\s*(\[[^\]]+\])?\s*(\w+)\s*,?\s*$', line)
    if m:
        print("  logic %s %s;" % (m.group(2) or "", m.group(3)))
PYEOF
}

# rm_xsim_prep: xsim cannot compile the sim-only `export "DPI-C"` of ibex_if_stage inside its generate scope (invalid C
# name "gen_prefetch_buffer.simutil_*"): use a copy of that one file without the two export lines (ibex.f points at it).
rm_xsim_prep() {
  rm -rf xsim.dir
  sed '/export "DPI-C"/d' "$IBEX/rtl/ibex_if_stage.sv" > "$PWD/ibex_if_stage.sv"
  sed -i "s#^.*/rtl/ibex_if_stage\.sv\$#$(mp "$PWD")/ibex_if_stage.sv#" ibex.f
}

# rm_xv_f <"-d NAME ..." defines> <extra last lines...>: the xvlog argument file xv.f (one file: the command line of 100+
# sources is longer than cmd.exe allows) with the defines, include dirs, ibex.f and $FILES (a list of paths, global).
rm_xv_f() {
  local defs=$1 i f
  shift
  {
    echo "-sv $defs"
    for i in "$RTL/shell/generated" "$RTL/shell/rm_cp/tb" "$PWD" "$IBEX/rtl" "$P/ip/prim/rtl" "$P/dv/sv/dv_utils"; do echo "-i $(mp "$i")"; done
    echo "-f ibex.f"
    for f in $FILES; do mp "$f"; done
    for f in "$@"; do echo "$f"; done
  } > xv.f
}
