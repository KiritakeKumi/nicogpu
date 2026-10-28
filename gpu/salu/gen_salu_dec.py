#!/usr/bin/env python3
"""Generate ng_salu_pkg.sv and ng_salu_dec.sv

Decoder of the 107 scalar/uniform-side instructions: SALU (81), branch
(10), sync (sleep barrier icache.inv waitcnt) and misc (nop endpgm trap brk
setprio csrr csrw csrs csrc csrwi csrsi csrci).  Match/mask come straight from
the ISA YAML (opcode + fixed fields + every unused field must be zero, i.e. the
model decoder's rule); only the name -> micro-op mapping below is hand written.
Usage (repo root):

    python3 rtl/gpu/salu/gen_salu_dec.py [--check-index vectors_int.isa_index.json]

The generated files are committed; CI does not need PyYAML.
"""
import argparse
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

UNITS = ("salu", "branch", "sync", "misc")

# micro-op list (order = encoding)
OPS = """ADD SUB SLL SLT SLTU XOR SRL SRA OR AND MIN MAX MINU MAXU ANDN ORN XNOR
MUL MULH MULHU MULHSU CZEQ CZNE SEQ SNE SGE SGEU BFM BFEU BFEI ROL ROR
ADD64 SUB64 ADDSX64 ADDZX64
MOV NOT CLZ CTZ CPOP BREV SEXTB SEXTH MOV64 WQM MOVI UPC UPCA
EXEC_GET EXEC_SET SVE_AND SVE_OR SVE_XOR SVE_ANDN1 SVE_ANDN2 SVE_MOV EXEC_WQM
JMP BEXECZ BEXECNZ JAL BZ BNZ BNONE BANY JR JALR
NOP ENDPGM TRAP BRK SETPRIO SLEEP BARRIER ICACHE WAITCNT
CSRR CSRW CSRS CSRC CSRWI CSRSI CSRCI""".split

# name -> micro-op; operand-2 immediate forms are detected from the YAML operands
MAP = {
    "uadd": "ADD", "usub": "SUB", "usll": "SLL", "uslt": "SLT", "usltu": "SLTU",
    "uxor": "XOR", "usrl": "SRL", "usra": "SRA", "uor": "OR", "uand": "AND",
    "umin": "MIN", "umax": "MAX", "uminu": "MINU", "umaxu": "MAXU",
    "uandn": "ANDN", "uorn": "ORN", "uxnor": "XNOR", "umul": "MUL",
    "umulh": "MULH", "umulhu": "MULHU", "umulhsu": "MULHSU",
    "uczero.eqz": "CZEQ", "uczero.nez": "CZNE", "useq": "SEQ", "usne": "SNE",
    "usge": "SGE", "usgeu": "SGEU", "ubfm": "BFM", "ubfe.u32": "BFEU",
    "ubfe.i32": "BFEI", "uadd64": "ADD64", "usub64": "SUB64",
    "uaddsx64": "ADDSX64", "uaddzx64": "ADDZX64", "urol": "ROL", "uror": "ROR",
    "uaddi": "ADD", "uandi": "AND", "uori": "OR", "uxori": "XOR", "uslli": "SLL",
    "usrli": "SRL", "usrai": "SRA", "uslti": "SLT", "usltiu": "SLTU",
    "useqi": "SEQ", "usnei": "SNE", "umuli": "MUL", "uadd64i": "ADD64",
    "umov": "MOV", "unot": "NOT", "uclz": "CLZ", "uctz": "CTZ", "ucpop": "CPOP",
    "ubrev": "BREV", "usext.b": "SEXTB", "usext.h": "SEXTH", "umov64": "MOV64",
    "uwqm": "WQM", "exec.get": "EXEC_GET", "exec.set": "EXEC_SET",
    "saveexec.and": "SVE_AND", "saveexec.or": "SVE_OR", "saveexec.xor": "SVE_XOR",
    "saveexec.andn1": "SVE_ANDN1", "saveexec.andn2": "SVE_ANDN2",
    "saveexec.mov": "SVE_MOV", "exec.wqm": "EXEC_WQM",
    "umovi": "MOVI", "upc": "UPC", "upca": "UPCA",
    "umov.l": "MOVI", "uadd.l": "ADD", "uand.l": "AND", "uor.l": "OR",
    "uxor.l": "XOR", "umul.l": "MUL", "useq.l": "SEQ", "uslt.l": "SLT",
    "usltu.l": "SLTU", "uadd64.l": "ADD64",
    "jmp": "JMP", "bexecz": "BEXECZ", "bexecnz": "BEXECNZ", "jal": "JAL",
    "bz": "BZ", "bnz": "BNZ", "bnone": "BNONE", "bany": "BANY", "jr": "JR",
    "jalr": "JALR",
    "nop": "NOP", "endpgm": "ENDPGM", "trap": "TRAP", "brk": "BRK",
    "setprio": "SETPRIO", "sleep": "SLEEP", "barrier": "BARRIER",
    "icache.inv": "ICACHE", "waitcnt": "WAITCNT",
    "csrr": "CSRR", "csrw": "CSRW", "csrs": "CSRS", "csrc": "CSRC",
    "csrwi": "CSRWI", "csrsi": "CSRSI", "csrci": "CSRCI",
}
BRANCH_OPS = "JMP BEXECZ BEXECNZ JAL BZ BNZ BNONE BANY JR JALR".split()


def mask_of(fields):
    m = 0
    for hi, lo in fields:
        m |= ((1 << (hi - lo + 1)) - 1) << lo
    return m


def zext(fld, name, width):
    hi, lo = fld[name]
    wd = hi - lo + 1
    if wd == width:
        return "w[%d:%d]" % (hi, lo)
    return "{%d'd0, w[%d:%d]}" % (width - wd, hi, lo)


def imm_expr(fld, kind, scale):
    hi, lo = fld["imm"]
    wd = hi - lo + 1
    if kind == "imm32":
        return "w[%d:%d]" % (hi, lo)
    if kind == "uimm20":
        return "{%d'd0, w[%d:%d]}" % (32 - wd, hi, lo)
    # signed (simm10 / simm17 / brNN / jofs10), scaled by 4 for br*/jofs10
    if scale:
        return "{{%d{w[%d]}}, w[%d:%d], 2'b00}" % (32 - wd - 2, hi, hi, lo)
    return "{{%d{w[%d]}}, w[%d:%d]}" % (32 - wd, hi, hi, lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--isa", default=os.path.join(ROOT, "spec", "isa", "nicogpu_isa.yaml"))
    ap.add_argument("--out", default=HERE)
    ap.add_argument("--check-index", action="append", help="isa_index.json from vecexport: compare match/mask")
    a = ap.parse_args()
    y = yaml.safe_load(open(a.isa, encoding="utf-8"))
    fmts = y["formats"]
    okinds = y["operand_kinds"]

    entries = []
    for ins in y["instructions"]:
        if ins["unit"] not in UNITS:
            continue
        if ins["name"] not in MAP:
            sys.exit("unmapped instruction: " + ins["name"])
        fm = fmts[ins["format"]]
        fld = fm["fields"]
        opf = {o["field"]: o for o in ins["operands"]}
        mask = mask_of([fld[k] for k in fld if k not in opf])
        match = 0
        for k, v in ins["fixed"].items():
            match |= v << fld[k][1]
        entries.append((ins, MAP[ins["name"]], fld, fm["width"], opf, mask, match))
    names = {e[0]["name"] for e in entries}
    for n in MAP:
        if n not in names:
            sys.exit("mapped but not in YAML: " + n)

    for ci in a.check_index or []:
        idx = {i["name"]: i for i in json.load(open(ci))["insns"]}
        bad = 0
        for ins, _o, _f, _w, _op, mask, match in entries:
            e = idx.get(ins["name"])
            if e is None or int(e["mask"], 16) != mask or int(e["match"], 16) != match:
                print("index mismatch:", ins["name"], hex(mask), hex(match), e)
                bad += 1
        print("index check %s: %d entries, %d mismatches" % (ci, len(entries), bad))
        if bad:
            sys.exit(1)

    # ----------------------------------------------------------------
    P = []
    P.append("// GENERATED by gen_salu_dec.py from -- do not edit.")
    P.append("// ng_salu_pkg: micro-ops, CSR numbers, trap causes and the decoded record of the")
    P.append("// scalar/uniform execution unit (SALU + branch + sync/misc/CSR).")
    P.append("package ng_salu_pkg;")
    P.append("")
    P.append("  // micro-op codes")
    for i, o in enumerate(OPS):
        P.append("  localparam logic [6:0] O_%s = 7'd%d;" % (o, i))
    P.append("")
    P.append("  // trap causes (model csr.h TrapCause)")
    P.append("  localparam logic [3:0] CAUSE_ILLEGAL   = 4'd1;")
    P.append("  localparam logic [3:0] CAUSE_EXPLICIT  = 4'd2;")
    P.append("  localparam logic [3:0] CAUSE_BRK       = 4'd3;")
    P.append("  localparam logic [3:0] CAUSE_ILLMODE   = 4'd8;")
    P.append("  localparam logic [3:0] CAUSE_ILLCSR    = 4'd9;")
    P.append("")
    P.append("  // CSR numbers (operand_kinds.csr of the ISA YAML)")
    for k, v in okinds["csr"]["values"].items():
        P.append("  localparam logic [9:0] CSR_%s = 10'd%d;" % (k.upper(), v))
    P.append("")
    P.append("  // sync/misc event kinds (ev_kind)")
    ev = ["NONE", "NOP", "ENDPGM", "BARRIER", "WAITCNT", "SLEEP", "ICACHE", "SETPRIO"]
    for i, e in enumerate(ev):
        P.append("  localparam logic [2:0] EV_%s = 3'd%d;" % (e, i))
    P.append("")
    P.append("  typedef struct packed {")
    P.append("    logic        hit;       // recognised encoding of this unit")
    P.append("    logic [6:0]  op;")
    P.append("    logic [5:0]  rd;")
    P.append("    logic [5:0]  rs1;       // also the 6-bit immediate of csr*i")
    P.append("    logic [5:0]  rs2;")
    P.append("    logic        rs1_rd;    // reads u[rs1]")
    P.append("    logic        rs2_rd;    // reads u[rs2]")
    P.append("    logic        rd_wr;     // writes u[rd]")
    P.append("    logic        rd_pair;   // rd / rs1 / rs2 are even-aligned 64-bit pairs")
    P.append("    logic        rs1_pair;")
    P.append("    logic        rs2_pair;")
    P.append("    logic        b_imm;     // second ALU operand is the immediate")
    P.append("    logic        is_br;     // branch class (reports through br_*)")
    P.append("    logic [31:0] imm;       // sign-extended / scaled / literal immediate")
    P.append("    logic [9:0]  csr;")
    P.append("  } dec_t;")
    P.append("")
    P.append("endpackage")
    open(os.path.join(a.out, "ng_salu_pkg.sv"), "w", newline="\n").write("\n".join(P) + "\n")

    # ----------------------------------------------------------------
    D = []
    D.append("// GENERATED by gen_salu_dec.py from -- do not edit.")
    D.append("// ng_salu_dec: combinational decoder of the %d SALU/branch/sync/misc/CSR instructions ." % len(entries))
    D.append("// Match/mask = opcode + fixed fields + every unused field must be zero, i.e. the")
    D.append("// model decoder's rule (checked against vecexport's isa_index by --check-index).")
    D.append("module ng_salu_dec")
    D.append("  import ng_salu_pkg::*;")
    D.append("(")
    D.append("  input  logic [31:0] w0,")
    D.append("  input  logic [31:0] w1,")
    D.append("  output dec_t        d")
    D.append(");")
    D.append("  logic [63:0] w;")
    D.append("  assign w = {w1, w0};")
    D.append("  always_comb begin")
    D.append("    d = '0;")
    opcs = sorted({e[6] & 0x7f for e in entries})
    D.append("    case (w[6:0])")
    for opc in opcs:
        D.append("      7'd%d: begin" % opc)
        for ins, op, fld, width, opf, mask, match in entries:
            if (match & 0x7f) != opc:
                continue
            mk = mask if width == 64 else mask & 0xFFFFFFFF
            D.append("        // %s" % ins["name"])
            D.append("        if ((w & 64'h%016x) == 64'h%016x) begin" % (mk, match))
            D.append("          d.hit = 1'b1; d.op = O_%s;" % op)
            if "rd" in opf:
                D.append("          d.rd = %s;" % zext(fld, "rd", 6))
            if "rs1" in opf:
                D.append("          d.rs1 = %s;" % zext(fld, "rs1", 6))
            if "rs2" in opf:
                D.append("          d.rs2 = %s;" % zext(fld, "rs2", 6))
            flags = []
            if "rs1" in opf and opf["rs1"]["kind"] in ("ureg", "upair"):
                flags.append("d.rs1_rd = 1'b1;")
            if "rs2" in opf and opf["rs2"]["kind"] in ("ureg", "upair"):
                flags.append("d.rs2_rd = 1'b1;")
            if "rd" in opf and opf["rd"].get("role") == "dst":
                flags.append("d.rd_wr = 1'b1;")
            if "rd" in opf and opf["rd"]["kind"] == "upair":
                flags.append("d.rd_pair = 1'b1;")
            if "rs1" in opf and opf["rs1"]["kind"] == "upair":
                flags.append("d.rs1_pair = 1'b1;")
            if "rs2" in opf and opf["rs2"]["kind"] == "upair":
                flags.append("d.rs2_pair = 1'b1;")
            if "imm" in opf and ins["unit"] == "salu":
                flags.append("d.b_imm = 1'b1;")
            if op in BRANCH_OPS:
                flags.append("d.is_br = 1'b1;")
            if flags:
                D.append("          " + " ".join(flags))
            if "imm" in opf:
                k = opf["imm"]["kind"]
                sc = okinds[k].get("scale", 1) not in (None, 1)
                D.append("          d.imm = %s;" % imm_expr(fld, k, sc))
            if "csr" in opf:
                D.append("          d.csr = %s;" % zext(fld, "csr", 10))
            D.append("        end")
        D.append("      end")
    D.append("      default: ;")
    D.append("    endcase")
    D.append("  end")
    D.append("endmodule")
    open(os.path.join(a.out, "ng_salu_dec.sv"), "w", newline="\n").write("\n".join(D) + "\n")
    print("generated %d entries" % len(entries))


if __name__ == "__main__":
    main()
