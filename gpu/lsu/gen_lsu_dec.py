#!/usr/bin/env python3
"""Generate ng_lsu_dec.sv (decoder of the 302 `unit: lsu` instructions) from


Match/mask come straight from the YAML (opcode + fixed fields + every unused
field must be zero, the model decoder's rule); only the name -> attribute
mapping below is hand written.  Usage (repo root):

    python3 rtl/gpu/lsu/gen_lsu_dec.py [--check-index vectors_lsu.isa_index.json]

The generated file is committed; CI does not need PyYAML for the RTL runs.
"""
import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

SPACES = {"global": 0, "ubase": 1, "buf": 2, "shared": 3, "scratch": 4}
CNT = {"vm": 0, "vst": 1, "tex": 2, "ls": 3, "sm": 4}
# AMO op codes: ng_noc_pkg::AMO_* for 0..12, ng_lsu_pkg AMO_F* for 13..15
AMO = {"add": 0, "sub": 1, "and": 2, "or": 3, "xor": 4, "min.i": 5, "max.i": 6, "min.u": 7, "max.u": 8,
       "xchg": 9, "cmpxchg": 10, "inc": 11, "dec": 12, "fadd": 13, "fmin": 14, "fmax": 15}
LDN = {"u8": (1, 0), "i8": (1, 1), "u16": (2, 0), "i16": (2, 1), "b32": (4, 0), "b64": (8, 0), "b96": (12, 0),
       "b128": (16, 0)}
STN = {"b8": 1, "b16": 2, "b32": 4, "b64": 8, "b96": 12, "b128": 16}


def mask_of(fields):
    m = 0
    for hi, lo in fields:
        m |= ((1 << (hi - lo + 1)) - 1) << lo
    return m


def attrs(name, latency):
    """name -> dict of dec_t fields."""
    p = name.split(".")
    a = {"cnt": CNT[latency.split(":")[1]]}
    if p[0] == "cache":
        a.update(kind="K_CACHE", cache_op={"inv.l1": 0, "inv.l2": 1, "wb.l2": 2}[".".join(p[1:])])
        return a
    if p[0] == "uld":
        rest = p[1:]
        uk = 0
        if rest[0] in ("x", "buf", "bufx"):
            uk = {"x": 1, "buf": 2, "bufx": 3}[rest[0]]
            rest = rest[1:]
        bits = int(rest[0][1:])
        a.update(kind="K_ULD", um=1, ukind=uk, nbytes=bits // 8, ndst=0, has_cpol=1)
        return a
    sp = SPACES[p[1]]
    a.update(space=sp, has_cpol=0 if sp in (3, 4) else 1)
    if p[0] == "ld":
        n, sx = LDN[p[2]]
        a.update(kind="K_LD", nbytes=n, sext=sx, ndst=max(1, n // 4))
    elif p[0] == "st":
        n = STN[p[2]]
        a.update(kind="K_ST", nbytes=n, ndata=max(1, n // 4))
    elif p[0] == "atom":
        rtn = p[-1] == "rtn"
        q = p[2:-1] if rtn else p[2:]
        op = q[0]
        ty = q[1]
        if op in ("min", "max"):
            op = op + "." + ("i" if ty.startswith("i") else "u")
        w64 = ty.endswith("64")
        kdw = 2 if w64 else 1
        a.update(kind="K_ATOM", amo=AMO[op], w64=int(w64), rtn=int(rtn), nbytes=8 if w64 else 4,
                 ndata=2 * kdw if q[0] == "cmpxchg" else kdw, ndst=kdw if rtn else 0)
    else:
        raise KeyError(name)
    return a


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
        if ins["unit"] != "lsu":
            continue
        at = attrs(ins["name"], ins["latency"])
        fm = fmts[ins["format"]]
        fld = fm["fields"]
        opf = {o["field"] for o in ins["operands"]}
        mask = mask_of([fld[k] for k in fld if k not in opf])
        match = 0
        for k, v in ins["fixed"].items():
            match |= v << fld[k][1]
        entries.append((ins, at, fm["width"], mask, match))

    if a.check_index:
        idx = {i["name"]: i for i in json.load(open(a.check_index))["insns"]}
        bad = 0
        for ins, _a, _w, mask, match in entries:
            e = idx.get(ins["name"])
            if e is None or int(e["mask"], 16) != mask or int(e["match"], 16) != match:
                print("index mismatch:", ins["name"], hex(mask), hex(match), e)
                bad += 1
        have = {q[0]["name"] for q in entries}
        miss = [n for n, e in idx.items() if e["unit"] == "lsu" and n not in have]
        print("index check: %d entries, %d mismatches, lsu names in index without entry: %d" % (len(entries), bad, len(miss)))
        if bad:
            sys.exit(1)

    by_op = {}
    for e in entries:
        by_op.setdefault(e[4] & 0x7F, []).append(e)
    lines = []
    for opc in sorted(by_op):
        lines.append("      7'd%d: begin" % opc)
        for ins, at, width, mask, match in by_op[opc]:
            wmask = (1 << width) - 1
            lines.append("        // %s" % ins["name"])
            lines.append("        if ((w & 64'h%016x) == 64'h%016x) begin" % (mask & wmask, match & wmask))
            s = ["d.hit = 1'b1;"]
            for k in ("kind", "space", "ukind", "nbytes", "sext", "amo", "w64", "rtn", "cnt", "cache_op", "ndata", "ndst",
                      "has_cpol", "um"):
                if k not in at:
                    continue
                v = at[k]
                if k == "kind":
                    s.append("d.kind = %s;" % v)
                elif k == "space":
                    s.append("d.space = 3'd%d;" % v)
                else:
                    wd = {"ukind": 2, "nbytes": 7, "amo": 5, "cnt": 3, "cache_op": 2, "ndata": 3, "ndst": 3}.get(k, 1)
                    s.append("d.%s = %d'd%d;" % (k, wd, v))
            lines.append("          " + " ".join(s))
            lines.append("        end")
        lines.append("      end")

    with open(os.path.join(a.out, "ng_lsu_dec.sv"), "w", newline="\n") as f:
        f.write("// GENERATED by gen_lsu_dec.py from -- do not edit.\n")
        f.write("// ng_lsu_dec: combinational decoder of the %d LSU instructions .\n" % len(entries))
        f.write("// Match/mask = opcode + fixed fields + every unused field must be zero (the model\n")
        f.write("// decoder's rule; checked against vecexport's isa_index by --check-index).\n")
        f.write("// Operand fields are extracted by ng_lsu (formats M / UM / SYS are fixed).\n")
        f.write("module ng_lsu_dec\n  import ng_lsu_pkg::*;\n(\n")
        f.write("  input  logic [31:0] w0,\n  input  logic [31:0] w1,\n  output dec_t        d\n);\n")
        f.write("  logic [63:0] w;\n  assign w = {w1, w0};\n")
        f.write("  always_comb begin\n    d = '0;\n    case (w[6:0])\n")
        f.write("\n".join(lines))
        f.write("\n      default: ;\n    endcase\n  end\nendmodule\n")
    print("generated %d entries" % len(entries))


if __name__ == "__main__":
    main()
