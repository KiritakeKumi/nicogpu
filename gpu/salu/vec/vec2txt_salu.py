#!/usr/bin/env python3
"""Convert nicogpu-vec/0.1 JSONL into the flat hex text read by rtl/gpu/tb/tb_salu_vec.sv.

Two producers are understood:
  * the stock exporter (tools/vecexport, units int and misc): arch_insn records whose
    insn.unit_table is salu / branch / sync / misc.  The stock vectors have no PC,
    no CSR snapshot and no branch outcome; the converter fills the defaults of the
    stock harness (pc 0, all CSRs 0, code_base 0x100000, next_pc taken from the
    branch semantics is NOT available -> branch next_pc checking is off for them).
  * dump_salu_rand (rtl/gpu/salu/vec): carries input.pc, input.csrs, ext{}, and
    expected.final_exec / final_csrs / next_pc.

    python3 vec2txt_salu.py out.txt in1.jsonl [in2.jsonl ...] [--words words.txt]

Token stream (whitespace separated; numbers hex, ids/names words):
    <nvec> <ninsn>
    per vector:
      <id> <name> <insn_idx>
      <nwords> <w0> <w1> <pc> <exec> <slot> <sp>
      <code_lo> <code_hi> <scratch_lo> <scratch_hi> <scratch_size>
      <mode> <fflags> <trapctl> <trapbase> <trapcause> <trappc> <trapinfo>   (input CSRs)
      <nu> (<idx> <val>)*nu                      input uregs (non-zero only)
      <trap> <cause> <info>                      expected trap (0 0 0 if none)
      <nwu> (<idx> <val>)*nwu                    expected ureg diff
      <exec_after>
      <mode> <fflags> <trapctl> <trapbase> <trapcause> <trappc> <trapinfo>   (final CSRs)
      <chk_br> <next_pc> <csr_mask>              branch outcome check flag + expected next PC; bit k of csr_mask = compare final CSR k
"""
import argparse
import json
import sys

UNITS = ("salu", "branch", "sync", "misc")
CSRN = ("mode", "fflags", "trapctl", "trapbase", "trapcause", "trappc", "trapinfo")


def h(v):
    return "%x" % (int(v, 16) if isinstance(v, str) else v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("jsonl", nargs="+")
    ap.add_argument("--words", help="also write '<name> <w0> <w1>' per distinct instruction (input of dump_salu_rand)")
    a = ap.parse_args()

    recs = []
    names = {}
    allwords = {}
    for fn in a.jsonl:
        with open(fn, encoding="utf-8") as f:
            for line in f:
                r = json.loads(line)
                if r.get("kind") != "arch_insn":
                    continue
                ins = r["insn"]
                if ins["unit_table"] not in UNITS and ins["name"] != "illegal":
                    continue
                if ins["name"] not in allwords:
                    ws0 = ins["words"]
                    allwords[ins["name"]] = "%s %s %s" % (ins["name"], ws0[0], ws0[1] if len(ws0) > 1 else "0x0")
                if ins["unit_table"] == "branch" and r["expected"]["trap"] and "ext" not in r:
                    # stock jr/jalr vectors: the model's trap is the FETCH of the target
                    # (empty code heap), not an effect of the instruction -> not comparable
                    continue
                recs.append(r)
                names.setdefault(ins["name"], len(names))

    out = ["%x %x" % (len(recs), len(names))]
    for r in recs:
        ins, inp, exp = r["insn"], r["input"], r["expected"]
        ext = r.get("ext") or {}
        words = ins["words"]
        w0 = int(words[0], 16)
        w1 = int(words[1], 16) if len(words) > 1 else 0
        out.append("%s %s %x" % (r["id"], ins["name"].replace(" ", "_"), names[ins["name"]]))
        cb = int(ext.get("code_base", "0x100000"), 16)
        sv = int(ext.get("scratch_va", "0x0"), 16)
        out.append("%x %x %x %x %x %x %x" % (len(words), w0, w1, inp.get("pc", 0), int(inp["exec"], 16),
                                             ext.get("slot", 0), ext.get("sp", 0)))
        out.append("%x %x %x %x %x" % (cb & 0xffffffff, cb >> 32, sv & 0xffffffff, sv >> 32,
                                       int(ext.get("scratch_bytes", "0x0"), 16)))
        ic = inp.get("csrs") or {}
        out.append(" ".join(h(ic.get(k, "0x0")) for k in CSRN))
        us = sorted(inp["uregs"].items(), key=lambda kv: int(kv[0]))
        out.append("%x" % len(us) + "".join(" %x %s" % (int(k), h(v)) for k, v in us))
        t = exp["trap"]
        out.append("1 %x %s" % (t["cause"], h(t["info"])) if t else "0 0 0")
        wu = (exp["writes"].get("uregs") or {}) if exp["writes"] else {}
        out.append("%x" % len(wu) + "".join(" %x %s" % (int(k), h(wu[k])) for k in sorted(wu, key=int)))
        # exec / CSR after
        if "final_exec" in exp:
            ex_after = h(exp["final_exec"])
        else:
            wex = (exp["writes"] or {}).get("exec")
            ex_after = h(wex) if wex else h(inp["exec"])
        out.append(ex_after)
        if "final_csrs" in exp:
            fc = exp["final_csrs"]
            out.append(" ".join(h(fc[k]) for k in CSRN))
        else:
            wc = (exp["writes"] or {}).get("csrs") or {}
            # stock vectors: csrs keys 0=mode 1=fflags 2=trapctl; trap CSRs after a trap are not exported
            vals = [h(ic.get(k, "0x0")) for k in CSRN]
            for key, idx in (("0", 0), ("1", 1), ("2", 2)):
                if key in wc:
                    vals[idx] = h(wc[key])
            if t:
                vals[4] = "%x" % t["cause"]
                vals[5] = "%x" % t["pc"]
                vals[6] = h(t["info"])
            out.append(" ".join(vals))
        np_ = exp.get("next_pc")
        # csr check mask: bit k = compare final CSR k.  The stock exporter only reports the
        # mode/fflags/trapctl diff (and the trap CSRs on a trap): trapbase/trapcause/trappc/
        # trapinfo writes by csrw/csrs/csrc are invisible there.
        if "final_csrs" in exp:
            cmask = 0x7f
        else:
            cmask = 0x07 | (0x78 if t else 0)
        out.append("1 %x %x" % (np_, cmask) if np_ is not None else "0 0 %x" % cmask)

    with open(a.out, "w", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    sys.stderr.write("vec2txt_salu: %d vectors, %d distinct instructions\n" % (len(recs), len(names)))
    if a.words:
        with open(a.words, "w", newline="\n") as f:
            f.write("\n".join(allwords.values()) + "\n")
    with open(a.out + ".names", "w", newline="\n") as f:
        for n in names:
            f.write(n + "\n")


if __name__ == "__main__":
    main()
