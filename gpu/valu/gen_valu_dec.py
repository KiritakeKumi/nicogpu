#!/usr/bin/env python3
"""Generate ng_valu_pkg.sv and ng_valu_dec.sv

The decoder covers the 150 integer-VALU instructions  plus the f32 VALU
subset (parameter EN_FP of ng_valu_dec; micro-op numbers are read
ng_valu_fp_pkg.sv).

The decode table (match/mask, operand routing) comes straight from the ISA
YAML so it cannot drift from the functional model's decoder; only the
name -> micro-op mapping below is hand written.  Usage (repo root):

    python3 rtl/gpu/valu/gen_valu_dec.py [--check-index vectors_int.isa_index.json]

The generated files are committed; CI does not need PyYAML.
"""
import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

# micro-op list (order = encoding)
UOPS = """ADD SUB SLL SRL SRA SLT SLTU XOR OR AND ANDN ORN XNOR MIN MAX MINU MAXU
MUL MULH MULHSU MULHU MAC ROL ROR CZEQ CZNE ADDSATI ADDSATU SUBSATI SUBSATU BFM
MULWU MULW ADD64 SUB64 ADDSX64 ADDZX64 SLL64 SRL64 SRA64 MBCNT CNDMASK MOV NOT
CLZ CTZ CPOP BREV REV8 SEXTB SEXTH ZEXTB ZEXTH LANEID NEG ABS MOV64 CMP CMP64
SEL BFEU BFEI BFI PERM MAD MAD64 ALIGNBIT MED3I MED3U DP4AI DP4AU SEL64""".split

FOPS = {}
CMP_FN = {"eq": 0, "ne": 1, "lt": 2, "ge": 3, "gt": 4}

SIMPLE = {
    "add": "ADD", "sub": "SUB", "sll": "SLL", "srl": "SRL", "sra": "SRA",
    "slt": "SLT", "sltu": "SLTU", "xor": "XOR", "or": "OR", "and": "AND",
    "andn": "ANDN", "orn": "ORN", "xnor": "XNOR", "min": "MIN", "max": "MAX",
    "minu": "MINU", "maxu": "MAXU", "mul": "MUL", "mac": "MAC", "rol": "ROL",
    "ror": "ROR", "czero.eqz": "CZEQ", "czero.nez": "CZNE",
    "addsat.i32": "ADDSATI", "addsat.u32": "ADDSATU", "subsat.i32": "SUBSATI",
    "subsat.u32": "SUBSATU", "bfm": "BFM", "add64": "ADD64", "sub64": "SUB64",
    "addsx64": "ADDSX64", "addzx64": "ADDZX64", "sll64": "SLL64",
    "srl64": "SRL64", "sra64": "SRA64", "mbcnt": "MBCNT", "cndmask": "CNDMASK",
    "mov": "MOV", "movi": "MOV", "movrel.s": "MOV", "movrel.d": "MOV",
    "not": "NOT", "clz": "CLZ", "ctz": "CTZ", "cpop": "CPOP", "brev": "BREV",
    "rev8": "REV8", "sext.b": "SEXTB", "sext.h": "SEXTH", "zext.b": "ZEXTB",
    "zext.h": "ZEXTH", "laneid": "LANEID", "neg": "NEG", "abs": "ABS",
    "mov64": "MOV64", "sel": "SEL", "bfe.u32": "BFEU", "bfe.i32": "BFEI",
    "bfi": "BFI", "perm.b32": "PERM", "mad.u32": "MAD", "alignbit": "ALIGNBIT",
    "med3.i32": "MED3I", "med3.u32": "MED3U", "dp4a.i32": "DP4AI",
    "dp4a.u32": "DP4AU", "sel64": "SEL64",
}
REV = {"subr": "SUB", "sllr": "SLL", "srlr": "SRL", "srar": "SRA"}
MUL = {"mulh": ("MULH", 1, 1), "mulhsu": ("MULHSU", 1, 0), "mulhu": ("MULHU", 0, 0),
       "mulw": ("MULW", 1, 1), "mulwu": ("MULWU", 0, 0),
       "mad64.u32": ("MAD64", 0, 0), "mad64.i32": ("MAD64", 1, 1)}


def uop_of(name):
    """name -> (uop, dict(swap, sa, sb, cmpfn, cmps)); KeyError = not this unit."""
    base = name
    for suf in (".vu", ".vi", ".vl"):
        if name.endswith(suf):
            base = name[: -len(suf)]
    x = {}
    if name == "mov64.vu":
        return "MOV64", x
    if base.startswith("cmp."):
        p = base.split(".")  # cmp fn type
        if p[2] not in ("i32", "u32", "i64", "u64"):
            raise KeyError(name)  # cmp.*.f32 etc. belong to the fp unit
        x["cmpfn"] = CMP_FN[p[1]]
        x["cmps"] = 1 if p[2].startswith("i") else 0
        return ("CMP64" if p[2].endswith("64") else "CMP"), x
    if base in SIMPLE:
        return SIMPLE[base], x
    if base in REV:
        x["swap"] = 1
        return REV[base], x
    if base in MUL:
        u, sa, sb = MUL[base]
        x.update(sa=sa, sb=sb)
        return u, x
    raise KeyError(name)


# ---- f32 VALU subset  --------------------------------------------------
# name (suffix .vu/.vl/.m stripped) -> FO_ name
FP_SIMPLE = {
    "fadd.f32": "ADD", "fsub.f32": "SUB", "fsubr.f32": "SUBR", "fmul.f32": "MUL",
    "fmin.f32": "MIN", "fmax.f32": "MAX", "fsgnj.f32": "SGNJ", "fsgnjn.f32": "SGNJN",
    "fsgnjx.f32": "SGNJX", "fmac.f32": "FMA", "ffma.f32": "FMA", "fldexp.f32": "LDEXP",
    "fmed3.f32": "MED3", "fclass.f32": "CLASS", "ffloor.f32": "FLOOR", "fceil.f32": "CEIL",
    "ftrunc.f32": "TRUNC", "frndne.f32": "RNDNE", "ffract.f32": "FRACT", "fsat.f32": "SAT",
    "fcvt.i32.f32": "CVTFI", "fcvt.u32.f32": "CVTFU", "fcvt.f32.i32": "CVTIF",
    "fcvt.f32.u32": "CVTUF", "frexp.mant.f32": "FREXPM", "frexp.exp.f32": "FREXPE",
}
# predicate bits: LT=1 EQ=2 GT=4 UN=8 SIG=16
FP_PRED = {"eq": 2, "ne": 5, "lt": 17, "le": 19, "gt": 20, "ge": 22, "o": 7, "u": 8,
           "equ": 10, "neu": 13, "ltu": 9, "leu": 11, "gtu": 12, "geu": 14}
FP_PRED_V = {"feq.f32": 2, "flt.f32": 17, "fle.f32": 19}
# binary16 <-> binary32 conversions (ng_valu_dec EN_F16CVT); suffix .m stripped.  The .lo/.hi
# f32 -> f16 forms keep the other half of rd: rd is read as well (d_read, like fmac).
FP_F16CVT = {
    "fcvt.f32.f16.lo": ("H2FL", 0), "fcvt.f32.f16.hi": ("H2FH", 0),
    "fcvt.f16.f32.lo": ("F2HL", 1), "fcvt.f16.f32.hi": ("F2HH", 1),
    "pkcvt.f16.f32": ("PKF2H", 0),
}


def fp_of(name):
    """name -> (FO name, dict(fpred, fsub, d_read, f16)); KeyError = not in the f32 subset."""
    base, sfx = name, ""
    for suf in (".vu", ".vl", ".m"):
        if name.endswith(suf):
            base, sfx = name[: -len(suf)], suf
    x = {}
    if base in FP_F16CVT and sfx in ("", ".m"):
        fo, dr = FP_F16CVT[base]
        x["f16"] = 1
        if dr:
            x["d_read"] = 1
        return fo, x
    if sfx == ".m":
        m = {"fadd.f32": "ADD", "fmul.f32": "MUL", "fcvt.f32.i32": "CVTIF", "fcvt.f32.u32": "CVTUF",
             "fcvt.i32.f32": "CVTFIM", "fcvt.u32.f32": "CVTFUM"}
        if base not in m:
            raise KeyError(name)
        return m[base], x
    if base in ("feq.f32", "flt.f32", "fle.f32"):
        return {"feq.f32": "FEQ", "flt.f32": "FLT", "fle.f32": "FLE"}[base], dict(fpred=FP_PRED_V[base])
    if base.startswith("cmp."):
        p = base.split(".")
        if len(p) != 3 or p[2] != "f32":
            raise KeyError(name)
        if p[1] == "class":
            return "CMPCLS", x
        return "CMP", dict(fpred=FP_PRED[p[1]])
    if base.startswith("fcvt.f32.ubyte"):
        return "UBYTE", dict(fsub=int(base[-1]))
    if base in FP_SIMPLE:
        if base == "fmac.f32":
            x["d_read"] = 1
        return FP_SIMPLE[base], x
    raise KeyError(name)


def load_fops():
    import re
    txt = open(os.path.join(HERE, "ng_valu_fp_pkg.sv"), encoding="utf-8").read()
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"FO_(\w+)\s*=\s*6'd(\d+)", txt)}


def mask_of(fields):
    m = 0
    for hi, lo in fields:
        m |= ((1 << (hi - lo + 1)) - 1) << lo
    return m


def fx7(fld, name):
    """7-bit zero-extended field expression."""
    hi, lo = fld[name]
    wd = hi - lo + 1
    if wd == 7:
        return "w[%d:%d]" % (hi, lo)
    return "{%d'd0, w[%d:%d]}" % (7 - wd, hi, lo)


def fx3(fld, name):
    hi, lo = fld[name]
    assert hi - lo + 1 == 3
    return "w[%d:%d]" % (hi, lo)


def immx(fld, kind):
    hi, lo = fld["imm"]
    wd = hi - lo + 1
    if kind in ("simm8", "simm18"):
        return "{{%d{w[%d]}}, w[%d:%d]}" % (32 - wd, hi, hi, lo)
    if kind == "imm32":
        return "w[%d:%d]" % (hi, lo)
    raise ValueError(kind)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--isa", default=os.path.join(ROOT, "spec", "isa", "nicogpu_isa.yaml"))
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--check-index", help="isa_index.json from vecexport: compare match/mask")
    a = ap.parse_args()
    FOPS.update(load_fops())
    y = yaml.safe_load(open(a.isa, encoding="utf-8"))
    fmts = y["formats"]

    entries = []
    for ins in y["instructions"]:
        if ins["unit"] != "valu":
            continue
        try:
            uop, x = uop_of(ins["name"])
            x["fop"] = None
        except KeyError:
            try:
                fo, x = fp_of(ins["name"])
            except KeyError:
                continue  # f16 / pack / dot2 / other valu ops: not this unit
            uop = "MOV"
            x["fop"] = fo
        if ins["format"] in ("H", "HC") or any(o["kind"] == "vreg16" for o in ins["operands"]):
            continue  # 16-bit half-register ops (fp16/int16): later task
        fm = fmts[ins["format"]]
        fld = fm["fields"]
        opf = {o["field"]: o for o in ins["operands"]}
        mask = mask_of([fld[k] for k in fld if k not in opf])
        match = 0
        for k, v in ins["fixed"].items():
            match |= v << fld[k][1]
        entries.append((ins, uop, x, fld, fm["width"], opf, mask, match))

    if a.check_index:
        idx = {i["name"]: i for i in json.load(open(a.check_index))["insns"]}
        bad = 0
        for ins, _u, _x, _f, _w, _o, mask, match in entries:
            e = idx.get(ins["name"])
            if e is None or int(e["mask"], 16) != mask or int(e["match"], 16) != match:
                print("index mismatch:", ins["name"], hex(mask), hex(match), e)
                bad += 1
        have = {q[0]["name"] for q in entries}
        miss = [n for n, e in idx.items() if e["unit"] == "valu" and n not in have]
        print("index check: %d entries, %d mismatches, valu names in index without entry: %d" % (len(entries), bad, len(miss)))
        if bad:
            sys.exit(1)

    # ----------------------------------------------------------
    with open(os.path.join(a.out, "ng_valu_pkg.sv"), "w", newline="\n") as f:
        f.write("// GENERATED by gen_valu_dec.py from -- do not edit.\n")
        f.write("// ng_valu_pkg: micro-ops and decoded-instruction record of the VALU ( int f32).\n")
        f.write("package ng_valu_pkg;\n\n  // micro-op codes (lane ALU function)\n")
        for i, u in enumerate(UOPS):
            f.write("  localparam logic [6:0] U_%s = 7'd%d;\n" % (u, i))
        f.write(
            """
  // operand source classes
  localparam logic [2:0] S_NONE  = 3'd0;
  localparam logic [2:0] S_VREG  = 3'd1;  // one vector register
  localparam logic [2:0] S_VPAIR = 3'd2;  // even-aligned vector register pair
  localparam logic [2:0] S_UREG  = 3'd3;  // uniform register, broadcast to all lanes
  localparam logic [2:0] S_UPAIR = 3'd4;  // uniform register pair, broadcast
  localparam logic [2:0] S_IMM   = 3'd5;  // instruction immediate, broadcast
  // destination kinds
  localparam logic [1:0] D_NONE  = 2'd0;
  localparam logic [1:0] D_VREG  = 2'd1;
  localparam logic [1:0] D_VPAIR = 2'd2;
  localparam logic [1:0] D_UREG  = 2'd3;  // uniform lane mask (cmp)

  // cmp predicate codes (dec_t.cmpfn)
  localparam logic [2:0] CMP_EQ = 3'd0, CMP_NE = 3'd1, CMP_LT = 3'd2, CMP_GE = 3'd3, CMP_GT = 3'd4;

  // trap causes (model csr.h TrapCause)
  localparam logic [3:0] CAUSE_ILLEGAL  = 4'd1;
  localparam logic [3:0] CAUSE_REGRANGE = 4'd10;
  localparam logic [3:0] CAUSE_FPEXC    = 4'd4;

  typedef struct packed {
    logic        hit;      // recognised VALU encoding
    logic [6:0]  uop;
    logic [2:0]  a_src;    // rs1 role
    logic [2:0]  b_src;    // rs2 / immediate role
    logic [2:0]  c_src;    // rs3 role (S_NONE / S_VREG / S_VPAIR)
    logic [1:0]  d_kind;
    logic        d_read;   // destination is also a source (mac)
    logic        swap;     // swap a and b (subr, sllr, ...)
    logic        rel_s;    // movrel.s: A register = rs1 + u
    logic        rel_d;    // movrel.d: destination register = rd + u
    logic        sa;       // multiplier: a is signed
    logic        sb;       // multiplier: b is signed
    logic        cmps;     // cmp: signed compare
    logic [2:0]  cmpfn;    // cmp predicate
    logic [6:0]  rd;
    logic [6:0]  rs1;
    logic [6:0]  rs2;
    logic [6:0]  rs3;
    logic [31:0] imm;      // sign-extended / literal immediate
    // ---- f32 VALU subset  ----
    logic        fp;       // floating-point instruction (d.fop valid)
    logic [5:0]  fop;      // ng_valu_fp_pkg FO_*
    logic [4:0]  fpred;    // compare predicate bits
    logic [1:0]  fsub;     // fcvt.f32.ubyteN index
    logic        has_rm;   // rm operand present
    logic [2:0]  rm;       // rm operand (7 = dynamic, 5/6 reserved)
    logic [2:0]  mneg;     // X-format source negate
    logic [2:0]  mabs;     // X-format source abs
    logic        msat;     // X-format result saturate
  } dec_t;

endpackage
"""
        )

    # ----------------------------------------------------------
    lines = []
    by_op = {}
    for e in entries:
        by_op.setdefault(e[7] & 0x7F, []).append(e)
    for opc in sorted(by_op):
        lines.append("      7'd%d: begin" % opc)
        for ins, uop, x, fld, width, opf, mask, match in by_op[opc]:
            wmask = (1 << width) - 1
            cond = "(w & 64'h%016x) == 64'h%016x" % (mask & wmask, match & wmask)
            if x.get("fop"):
                cond = ("EN_FP && EN_F16CVT && " if x.get("f16") else "EN_FP && ") + cond
            s = ["        // %s" % ins["name"], "        if (%s) begin" % cond]
            s.append("          d.hit = 1'b1; d.uop = U_%s;" % uop)
            if x.get("fop"):
                s.append("          d.fp = 1'b1; d.fop = 6'd%d;  // FO_%s" % (FOPS[x["fop"]], x["fop"]))
            a_src, b_src, c_src, d_kind = "S_NONE", "S_NONE", "S_NONE", "D_NONE"
            has_rs1 = "rs1" in opf
            for fname, o in opf.items():
                kind = o["kind"]
                if fname == "rd":
                    if o["role"] == "dst":
                        d_kind = {"vreg": "D_VREG", "vpair": "D_VPAIR", "ureg": "D_UREG"}[kind]
                    s.append("          d.rd = %s;" % fx7(fld, "rd"))
                elif fname == "rs1":
                    a_src = {"vreg": "S_VREG", "vpair": "S_VPAIR", "ureg": "S_UREG", "upair": "S_UPAIR"}[kind]
                    s.append("          d.rs1 = %s;" % fx7(fld, "rs1"))
                elif fname == "rs2":
                    b_src = {"vreg": "S_VREG", "vpair": "S_VPAIR", "ureg": "S_UREG"}[kind]
                    s.append("          d.rs2 = %s;" % fx7(fld, "rs2"))
                elif fname == "rs3":
                    c_src = {"vreg": "S_VREG", "vpair": "S_VPAIR"}[kind]
                    s.append("          d.rs3 = %s;" % fx7(fld, "rs3"))
                elif fname == "rm":
                    s.append("          d.has_rm = 1'b1; d.rm = %s;" % fx3(fld, "rm"))
                elif fname == "neg":
                    s.append("          d.mneg = %s;" % fx3(fld, "neg"))
                elif fname == "abs":
                    s.append("          d.mabs = %s;" % fx3(fld, "abs"))
                elif fname == "sat":
                    s.append("          d.msat = w[%d];" % fld["sat"][0])
                elif fname == "imm":
                    s.append("          d.imm = %s;" % immx(fld, kind))
                    if has_rs1:
                        b_src = "S_IMM"
                    else:
                        a_src = "S_IMM"
                else:
                    raise ValueError((ins["name"], fname))
            rel_s = ins["name"] == "movrel.s"
            rel_d = ins["name"] == "movrel.d"
            if rel_d:
                d_kind = "D_VREG"
            s.append("          d.a_src = %s; d.b_src = %s; d.c_src = %s; d.d_kind = %s;" % (a_src, b_src, c_src, d_kind))
            fl = []
            if ins["name"] in ("mac", "mac.vu") or x.get("d_read"):
                fl.append("d.d_read = 1'b1;")
            if x.get("fpred"):
                fl.append("d.fpred = 5'd%d;" % x["fpred"])
            if x.get("fsub"):
                fl.append("d.fsub = 2'd%d;" % x["fsub"])
            if x.get("swap"):
                fl.append("d.swap = 1'b1;")
            if rel_s:
                fl.append("d.rel_s = 1'b1;")
            if rel_d:
                fl.append("d.rel_d = 1'b1;")
            if x.get("sa"):
                fl.append("d.sa = 1'b1;")
            if x.get("sb"):
                fl.append("d.sb = 1'b1;")
            if x.get("cmps"):
                fl.append("d.cmps = 1'b1;")
            if "cmpfn" in x:
                fl.append("d.cmpfn = 3'd%d;" % x["cmpfn"])
            if fl:
                s.append("          " + " ".join(fl))
            s.append("        end")
            lines.extend(s)
        lines.append("      end")

    with open(os.path.join(a.out, "ng_valu_dec.sv"), "w", newline="\n") as f:
        nf16 = sum(1 for e in entries if e[2].get("f16"))
        f.write("// GENERATED by gen_valu_dec.py from -- do not edit.\n")
        f.write("// ng_valu_dec: combinational decoder of the %d VALU instructions (150 integer the rest f32 via EN_FP;\n" % len(entries))
        f.write("// %d of those are the binary16 <-> binary32 conversions of which also need EN_F16CVT).\n" % nf16)
        f.write("// Match/mask = opcode + fixed fields + every unused field must be zero, i.e. the\n")
        f.write("// model decoder's rule (checked against vecexport's isa_index by --check-index).\n")
        f.write("module ng_valu_dec\n  import ng_valu_pkg::*;\n#(\n  parameter bit EN_FP = 1'b0,   // 1: also decode the f32 VALU subset \n"
                "  parameter bit EN_F16CVT = 1'b0  // 1 (with EN_FP): also the binary16 conversions \n) (\n")
        f.write("  input  logic [31:0] w0,\n  input  logic [31:0] w1,\n  output dec_t        d\n);\n")
        f.write("  logic [63:0] w;\n  assign w = {w1, w0};\n")
        f.write("  always_comb begin\n    d = '0;\n    case (w[6:0])\n")
        f.write("\n".join(lines))
        f.write("\n      default: ;\n    endcase\n  end\nendmodule\n")
    print("generated %d entries" % len(entries))


if __name__ == "__main__":
    main()
