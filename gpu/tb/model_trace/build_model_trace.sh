#!/usr/bin/env bash
# build_model_trace.sh -- build the model-side trace generator.
#
#   rtl/gpu/tb/model_trace/build_model_trace.sh [repo_root] [out_dir]
#
# Compiles the functional model from the current worktree (read-only usage;
# nothing under model/ is modified) plus the trace generator
# (sp_trace_gen.cpp) into <out_dir>/sp_trace_gen.exe with MSYS2 mingw-w64
# g++ (C:/msys64/mingw64).  The ISA decode table is regenerated with
# (needs python3 + PyYAML at build time
# only).  Env:
#   NG_CC     compiler        (default C:/msys64/mingw64/bin/c++.exe)
#   NG_PY     python          (default python)
#   NG_SF_LIB prebuilt libnicogpu_softfloat.a; when unset, SoftFloat is
#             compiled  (slow first build).
set -eu
ROOT=${1:-$(cd "$(dirname "$0")/../../../.." && pwd)}
OUT=${2:-$ROOT/build/gpu/model_trace_gen}
CC=${NG_CC:-C:/msys64/mingw64/bin/c++.exe}
PY=${NG_PY:-python}
mkdir -p "$OUT"

# PATH must contain the mingw runtime DLLs for cc1plus and the result
case ":$PATH:" in
  */c/msys64/mingw64/bin:*) ;;
  *) export PATH="/c/msys64/mingw64/bin:$PATH" ;;
esac

# 1. ISA decode table + descriptor header (generated, not committed)
if [ ! -f "$OUT/gen/nicogpu_isa.h" ]; then
  mkdir -p "$OUT/gen"
  "$PY" "$ROOT/model/gen/nicogpu_isagen_decode.py" "$ROOT/spec/isa/nicogpu_isa.yaml" \
    --out-dir "$OUT/gen" --name nicogpu_isa
fi
if [ ! -f "$OUT/gen/nicogpu_desc.h" ]; then
  mkdir -p "$OUT/gen"
  "$PY" "$ROOT/tools/descgen/descgen.py" gen --out-dir "$OUT/gen" \
    --yaml "$ROOT/spec/desc/nicogpu_desc.yaml" \
    --vk-formats "$ROOT/spec/desc/vk_formats.yaml" \
    --isa "$ROOT/spec/isa/nicogpu_isa.yaml"
fi

if [ ! -f "$OUT/gen/nicogpu_gfx.h" ]; then
  mkdir -p "$OUT/gen"
  "$PY" "$ROOT/tools/gfxgen/gfxgen.py" gen --out-dir "$OUT/gen" \
    --yaml "$ROOT/spec/gfx/nicogpu_gfx.yaml" \
    --cmd-yaml "$ROOT/spec/cmd/nicogpu_cmd.yaml"
fi

# 2. SoftFloat (prebuilt library or from source)
SF_LIB=${NG_SF_LIB:-}
if [ -z "$SF_LIB" ]; then
  SF_LIB="$OUT/libnicogpu_softfloat.a"
  if [ ! -f "$SF_LIB" ]; then
    ( cd "$ROOT/model/extern/softfloat" && ls source/*.c source/RISCV/*.c ) \
      | while read -r f; do
        "$CC" -c -w -DSOFTFLOAT_FAST_INT64 -DSOFTFLOAT_ROUND_ODD \
          -I "$ROOT/model/extern/softfloat/include" \
          -I "$ROOT/model/extern/softfloat/source/RISCV" \
          "$ROOT/model/extern/softfloat/$f" -o "$OUT/$(basename "$f" .c).o"
      done
    ar rcs "$SF_LIB" "$OUT"/*.o
    rm -f "$OUT"/*.o
  fi
fi

# 3. Model + generator
"$CC" -std=c++17 -O1 -ffp-contract=off \
  -I "$ROOT/model/include" -I "$OUT/gen" \
  -I "$ROOT/model/extern/softfloat/include" \
  -DSOFTFLOAT_FAST_INT64 -DSOFTFLOAT_ROUND_ODD \
  "$ROOT/rtl/gpu/tb/model_trace/sp_trace_gen.cpp" \
  "$OUT/gen/nicogpu_isa.cpp" \
  "$ROOT/model/src/sp.cpp" "$ROOT/model/src/isa.cpp" \
  "$ROOT/model/src/memory.cpp" "$ROOT/model/src/mmu.cpp" \
  "$ROOT/model/src/interleave.cpp" "$ROOT/model/src/desc.cpp" \
  "$ROOT/model/src/texfmt.cpp" "$ROOT/model/src/bcdecode.cpp" \
  "$ROOT/model/src/gfx/gfx_state.cpp" "$ROOT/model/src/gfx/raster.cpp" "$ROOT/model/src/gfx/zs.cpp" \
  "$ROOT/model/src/gfx/tile.cpp" \
  "$ROOT/model/src/sem/main_semantics.cpp" "$ROOT/model/src/sem/int_semantics.cpp" \
  "$ROOT/model/src/sem/fp_semantics.cpp" "$ROOT/model/src/sem/fp64_sfu_semantics.cpp" \
  "$ROOT/model/src/sem/xlane_semantics.cpp" "$ROOT/model/src/sem/lsu_semantics.cpp" \
  "$ROOT/model/src/sem/misc_semantics.cpp" "$ROOT/model/src/sem/sample_semantics.cpp" \
  "$ROOT/model/src/sem/tex_semantics.cpp" "$ROOT/model/src/sem/gfx_semantics.cpp" \
  "$SF_LIB" \
  -o "$OUT/sp_trace_gen.exe"
echo "BUILD OK: $OUT/sp_trace_gen.exe"
