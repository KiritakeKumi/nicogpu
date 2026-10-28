#!/usr/bin/env python3
"""Generate ng_fp64_pkg.sv and ng_fp64_dec.sv

The decode table (match/mask, operand fields and kinds) comes straight from
the ISA YAML so it cannot drift from the functional model's decoder; only the
name -> (lane op, MODE scope, DAZ/FTZ use, compare predicate) mapping below is
hand written (it mirrors).  Usage (repo
root):

    python3 rtl/gpu/fp64/gen_fp64_dec.py [--check-index vectors_fp64_sfu.isa_index.json]

The generated files are committed; CI re-generates them and diffs.
"""
import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

# lane operation codes (ng_fp64_lane LOP_*)
LOP = ["ADD", "SUB", "MUL", "FMA", "MIN", "MAX", "SGNJ", "SGNJN", "SGNJX", "LDEXP", "FCLASS", "FLOOR", "CEIL",
       "TRUNC", "RNDNE", "FRACT", "CVT_F64_F32", "CVT_F32_F64", "CVT_I32_F64", "CVT_U32_F64", "CVT_F64_I32",
       "CVT_F64_U32", "FREXP_M", "FREXP_E", "CMP", "CMP_CLASS"]
# MODE scope: which MODE fields the instruction consults (FpScope in the model)
SC_NONE, SC_F64, SC_F32, SC_CMP = 0, 1, 2, 3
# compare predicates (fp_util.h pred:): LT=1 EQ=2 GT=4 UN=8 SIG=16
LT, EQ, GT, UN, SIG = 1, 2, 4, 8, 16
PRED = {"eq": EQ, "ne": LT | GT, "lt": LT | SIG, "le": LT | EQ | SIG, "gt": GT | SIG, "ge": GT | EQ | SIG,
        "o": LT | EQ | GT, "u": UN, "equ": UN | EQ, "neu": UN | LT | GT, "ltu": UN | LT, "leu": UN | LT | EQ,
        "gtu": UN | GT, "geu": UN | GT | EQ}

# name -> (lop, scope, dazk, ftzk)   dazk: 0 none, 1 DAZ64 source, 2 DAZ32 source   ftzk: 0 none, 1 FTZ64, 2 FTZ32
TABLE = {
    "fadd.f64": ("ADD", SC_F64, 1, 1), "fsub.f64": ("SUB", SC_F64, 1, 1), "fmul.f64": ("MUL", SC_F64, 1, 1),
    "ffma.f64": ("FMA", SC_F64, 1, 1),
    "fmin.f64": ("MIN", SC_F64, 1, 0), "fmax.f64": ("MAX", SC_F64, 1, 0),
    "fsgnj.f64": ("SGNJ", SC_NONE, 0, 0), "fsgnjn.f64": ("SGNJN", SC_NONE, 0, 0), "fsgnjx.f64": ("SGNJX", SC_NONE, 0, 0),
    "fldexp.f64": ("LDEXP", SC_F64, 1, 1),
    "fclass.f64": ("FCLASS", SC_NONE, 0, 0),
    "ffloor.f64": ("FLOOR", SC_F64, 1, 1), "fceil.f64": ("CEIL", SC_F64, 1, 1), "ftrunc.f64": ("TRUNC", SC_F64, 1, 1),
    "frndne.f64": ("RNDNE", SC_F64, 1, 1), "ffract.f64": ("FRACT", SC_F64, 1, 1),
    "fcvt.f64.f32": ("CVT_F64_F32", SC_F64, 2, 0),
    "fcvt.f32.f64": ("CVT_F32_F64", SC_F32, 1, 2), "fcvt.f32.f64.m": ("CVT_F32_F64", SC_F32, 1, 2),
    "fcvt.i32.f64": ("CVT_I32_F64", SC_F64, 1, 0), "fcvt.u32.f64": ("CVT_U32_F64", SC_F64, 1, 0),
    "fcvt.f64.i32": ("CVT_F64_I32", SC_NONE, 0, 0), "fcvt.f64.u32": ("CVT_F64_U32", SC_NONE, 0, 0),
    "frexp.mant.f64": ("FREXP_M", SC_F64, 1, 0), "frexp.exp.f64": ("FREXP_E", SC_F64, 1, 0),
    "cmp.class.f64": ("CMP_CLASS", SC_NONE, 0, 0),
}
KIND = {"vreg": "K_VREG", "vpair": "K_VPAIR", "ureg": "K_UREG"}
# YAML types the rs1 of fclass.f64 as `vreg`, the model (and `pair64`, `d = fclass(s1_64)`) reads a pair
KIND_OVERRIDE = {("fclass.f64", "rs1"): "K_VPAIR"}


def cfg_of(name):
    if name in TABLE:
        return TABLE[name] + (0,)
    if name.startswith("cmp.") and name.endswith(".f64"):
        return ("CMP", SC_CMP, 0, 0, PRED[name[4:-4]])
    raise KeyError(name)


def mask_of(fields):
    m = 0
    for hi, lo in fields:
        m |= ((1 << (hi - lo + 1)) - 1) << lo
    return m


def fxn(fld, name, n):
    hi, lo = fld[name]
    wd = hi - lo + 1
    if wd == n:
        return "w[%d:%d]" % (hi, lo)
    return "{%d'd0, w[%d:%d]}" % (n - wd, hi, lo)


PKG_TAIL = """
  // MODE scope (which MODE fields the instruction consults; the model's FpScope)
  localparam logic [1:0] SC_NONE = 2'd0;   // no MODE use, never IllegalInsn on MODE, no flags
  localparam logic [1:0] SC_F64  = 2'd1;   // rm64 / DAZ64 / FTZ64, FP trap enables
  localparam logic [1:0] SC_F32  = 2'd2;   // rm32 / FTZ32 (fcvt.f32.f64): destination width
  localparam logic [1:0] SC_CMP  = 2'd3;   // fixed RNE, flags + trap enables only

  // operand kinds
  localparam logic [1:0] K_NONE  = 2'd0;
  localparam logic [1:0] K_VREG  = 2'd1;   // one 32-bit vector register
  localparam logic [1:0] K_VPAIR = 2'd2;   // even-aligned register pair (binary64)
  localparam logic [1:0] K_UREG  = 2'd3;   // uniform register (compare result mask)

  // trap causes (model csr.h TrapCause)
  localparam logic [3:0] CAUSE_ILLEGAL  = 4'd1;
  localparam logic [3:0] CAUSE_FPEXC    = 4'd4;
  localparam logic [3:0] CAUSE_REGRANGE = 4'd10;

  typedef struct packed {
    logic        hit;       // recognised FP64 encoding
    logic [4:0]  lop;       // LOP_*
    logic [4:0]  pred;      // compare predicate (LT=1 EQ=2 GT=4 UN=8 SIG=16)
    logic [1:0]  scope;     // SC_*
    logic [1:0]  dazk;      // 0 none, 1 DAZ64 flushes the source, 2 DAZ32 flushes an f32 source
    logic [1:0]  ftzk;      // 0 none, 1 FTZ64 flushes the result, 2 FTZ32 flushes the f32 result
    logic        has_rm;    // explicit rm operand (ffma.f64, fcvt.f32.f64.m)
    logic [2:0]  rm;
    logic [2:0]  neg;       // ffma source modifiers
    logic [2:0]  abs;
    logic [1:0]  a_kind;    // rs1
    logic [1:0]  b_kind;    // rs2
    logic [1:0]  c_kind;    // rs3
    logic [1:0]  d_kind;    // rd
    logic [6:0]  rd;
    logic [6:0]  rs1;
    logic [6:0]  rs2;
    logic [6:0]  rs3;
  } dec_t;

endpackage
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--isa", default=os.path.join(ROOT, "spec", "isa", "nicogpu_isa.yaml"))
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--check-index", help="isa_index.json from vecexport: compare match/mask")
    a = ap.parse_args()
    y = yaml.safe_load(open(a.isa, encoding="utf-8"))
    fmts = y["formats"]

    entries = []
    for ins in y["instructions"]:
        if ins["unit"] != "fp64":
            continue
        cfg = cfg_of(ins["name"])
        fm = fmts[ins["format"]]
        fld = fm["fields"]
        opf = {o["field"]: o for o in ins["operands"]}
        mask = mask_of([fld[k] for k in fld if k not in opf])
        match = 0
        for k, v in ins["fixed"].items():
            match |= v << fld[k][1]
        entries.append((ins, cfg, fld, fm["width"], opf, mask, match))

    if a.check_index:
        idx = {i["name"]: i for i in json.load(open(a.check_index))["insns"]}
        bad = 0
        for ins, _c, _f, _w, _o, mask, match in entries:
            e = idx.get(ins["name"])
            if e is None or int(e["mask"], 16) != mask or int(e["match"], 16) != match:
                print("index mismatch:", ins["name"], hex(mask), hex(match), e)
                bad += 1
        have = {q[0]["name"] for q in entries}
        miss = [n for n, e in idx.items() if e["unit"] == "fp64" and n not in have]
        print("index check: %d entries, %d mismatches, fp64 names in index without entry: %d" % (len(entries), bad, len(miss)))
        if bad or miss:
            sys.exit(1)

    with open(os.path.join(a.out, "ng_fp64_pkg.sv"), "w", newline="\n") as f:
        f.write("// GENERATED by gen_fp64_dec.py from -- do not edit.\n")
        f.write("// ng_fp64_pkg: lane op codes and decoded-instruction record of the FP64 unit .\n")
        f.write("package ng_fp64_pkg;\n\n  // lane operation (ng_fp64_lane op input)\n")
        for i, n in enumerate(LOP):
            f.write("  localparam logic [4:0] LOP_%s = 5'd%d;\n" % (n, i))
        f.write(PKG_TAIL)

    by_op = {}
    for e in entries:
        by_op.setdefault(e[6] & 0x7F, []).append(e)
    lines = []
    for opc in sorted(by_op):
        lines.append("      7'd%d: begin" % opc)
        for ins, cfg, fld, width, opf, mask, match in by_op[opc]:
            lop, scope, dazk, ftzk, pred = cfg
            wmask = (1 << width) - 1
            cond = "(w & 64'h%016x) == 64'h%016x" % (mask & wmask, match & wmask)
            s = ["        // %s" % ins["name"], "        if (%s) begin" % cond]
            s.append("          d.hit = 1'b1; d.lop = LOP_%s; d.pred = 5'd%d; d.scope = 2'd%d; d.dazk = 2'd%d; d.ftzk = 2'd%d;"
                     % (lop, pred, scope, dazk, ftzk))
            kinds = {"rd": "K_NONE", "rs1": "K_NONE", "rs2": "K_NONE", "rs3": "K_NONE"}
            for fname, o in opf.items():
                if fname in kinds:
                    kinds[fname] = KIND_OVERRIDE.get((ins["name"], fname), KIND[o["kind"]])
                    s.append("          d.%s = %s;" % (fname, fxn(fld, fname, 7)))
                elif fname == "rm":
                    s.append("          d.has_rm = 1'b1; d.rm = %s;" % fxn(fld, "rm", 3))
                elif fname in ("neg", "abs"):
                    s.append("          d.%s = %s;" % (fname, fxn(fld, fname, 3)))
                else:
                    raise ValueError((ins["name"], fname))
            s.append("          d.d_kind = %s; d.a_kind = %s; d.b_kind = %s; d.c_kind = %s;"
                     % (kinds["rd"], kinds["rs1"], kinds["rs2"], kinds["rs3"]))
            s.append("        end")
            lines.extend(s)
        lines.append("      end")

    with open(os.path.join(a.out, "ng_fp64_dec.sv"), "w", newline="\n") as f:
        f.write("// GENERATED by gen_fp64_dec.py from -- do not edit.\n")
        f.write("// ng_fp64_dec: combinational decoder of the %d FP64 instructions .\n" % len(entries))
        f.write("// Match/mask = opcode + fixed fields + every unused field must be zero, i.e. the\n")
        f.write("// model decoder's rule (checked against vecexport's isa_index by --check-index).\n")
        f.write("module ng_fp64_dec\n  import ng_fp64_pkg::*;\n(\n")
        f.write("  input  logic [31:0] w0,\n  input  logic [31:0] w1,\n  output dec_t        d\n);\n")
        f.write("  logic [63:0] w;\n  assign w = {w1, w0};\n")
        f.write("  always_comb begin\n    d = '0;\n    case (w[6:0])\n")
        f.write("\n".join(lines))
        f.write("\n      default: ;\n    endcase\n  end\nendmodule\n")
    print("generated %d entries" % len(entries))


if __name__ == "__main__":
    main()
