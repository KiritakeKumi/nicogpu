#!/usr/bin/env python3
""" dump ptegen.py's decode vectors (3124 entries x levels) as a $readmemh
file for tb_mmu_decode.sv.  ptegen.decode_entry is the Python reference of
 rules D1..D15 (cross-checked against the C++ model by
ptegen itself).

Record = 3 x 64-bit words: entry | level | kind<<8 | rule<<16 | size_code<<24 | base_pfn
Usage: gen_mmu_decode_vec.py <out.hex>
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "ptegen"))
import ptegen  # noqa: E402

KIND = {"not_present": 0, "bad_pte": 1, "sparse_null": 2, "table": 3, "leaf": 4}
SIZE = {0: 0, 4 << 10: 0, 64 << 10: 1, 2 << 20: 2, 1 << 30: 3, 512 << 30: 4}


def main():
    out = sys.argv[1]
    s = ptegen.Spec()
    cases = ptegen.vector_cases(s)
    with open(out, "w", newline="\n") as f:
        for e, level in cases:
            kind, rule, size, base = ptegen.decode_entry(s, e, level)
            w1 = level | (KIND[kind] << 8) | (rule << 16) | (SIZE[size] << 24)
            f.write(f"{e:016x} {w1:016x} {base >> 12:016x}\n")
    print(f"gen_mmu_decode_vec: {len(cases)} vectors -> {out}")


if __name__ == "__main__":
    main()
