#!/usr/bin/env bash
# run_tb_sp_top.sh with Verilator 5 on this Windows host (the official
# Windows package); on Linux/CI just use
# `SIM=verilator rtl/gpu/tb/run_tb_sp_top.sh ...` with verilator on PATH.
#   rtl/gpu/tb/run_tb_sp_top_vl.sh <bundle_dir...>      (same env knobs as run_tb_sp_top.sh)
set -eu
VLW=${NG_VERILATOR_WIN:-D:/verilator-windows-windows_amd64-5.052}
if [ -z "${VERILATOR:-}" ] && [ -d "$VLW" ]; then
  VLU=/$(echo "$VLW" | sed 's#^\([A-Za-z]\):#\L\1#')
  export VERILATOR_ROOT=$VLW
  export PATH="$VLU/w64devkit/bin:$VLU/bin:$PATH"
  export VERILATOR=verilator.exe
  export VL_MAKEFLAGS=${VL_MAKEFLAGS:-"CXX=g++ CC=gcc LINK=g++"}
fi
SIM=verilator exec bash "$(dirname "$0")/run_tb_sp_top.sh" "$@"
