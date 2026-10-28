#!/usr/bin/env python3
"""gen_scenarios.py -- trace-comparison scenario generator.

Writes <out>/<name>/{scen.cfg,code.hex} for the scheduler-visible instruction
subset (misc/sync/branch, the classes ng_sp_predec decodes):
  nop, sleep, setprio, barrier, waitcnt, endpgm, jmp, bz, bnz, bnone, bany,
  bexecz, bexecnz, trap, brk, plus (optionally) one out-of-set opcode issued
  to the pipe -- model-side traces use real ISA names, so trace scenarios
  stay within the subset above.

Encoding reference: (frozen v0.1).
  SYS   w0 = imm[31:12] f5[11:7] 0x4e    (nop=0 endpgm=1 trap=2 brk=3
                                          sleep=4 setprio=5 barrier=6 icache=12)
  WAIT  w0 = sm[31:30] sfu[29:27] ls[26:23] vst[22:18] tex[17:13] vm[12:7] 0x50
  J     w0 = imm[31:10] f3[9:7] 0x46     (jmp=0 bexecz=1 bexecnz=2; br22 <<2)
  B     w0 = imm[31:15] f2[14:13] rs1[12:7] 0x4a  (bz=0 bnz=1 bnone=2 bany=3;
                                          br17 <<2)
Python >= 3.8 stdlib only.
"""
from __future__ import annotations

import argparse
from pathlib import Path

OP_SYS = 0x4E
OP_WAIT = 0x50
OP_J = 0x46
OP_B = 0x4A


def w_sys(f5: int, imm20: int = 0) -> int:
    assert 0 <= imm20 < (1 << 20)
    return (imm20 << 12) | (f5 << 7) | OP_SYS


def w_waitcnt(vm: int, vst: int, tex: int, ls: int, sm: int, sfu: int) -> int:
    return ((sm & 3) << 30) | ((sfu & 7) << 27) | ((ls & 15) << 23) | \
           ((vst & 31) << 18) | ((tex & 31) << 13) | ((vm & 63) << 7) | OP_WAIT


def w_jmp(off: int) -> int:
    assert off % 4 == 0
    v = (off >> 2) & ((1 << 22) - 1)
    return (v << 10) | OP_J


def w_bop(f2: int, rs1: int, off: int) -> int:
    assert off % 4 == 0
    v = (off >> 2) & ((1 << 17) - 1)
    return (v << 15) | (f2 << 13) | ((rs1 & 0x7F) << 7) | OP_B


def w_bexecz(off: int) -> int:
    v = (off >> 2) & ((1 << 22) - 1)
    return (v << 10) | (1 << 7) | OP_J


def w_bexecnz(off: int) -> int:
    v = (off >> 2) & ((1 << 22) - 1)
    return (v << 10) | (2 << 7) | OP_J


class Prog:
    """A little assembler: words are (name, encoded) in emission order."""

    def __init__(self) -> None:
        self.words: list[tuple[str, int]] = []

    def emit(self, name: str, w: int) -> None:
        self.words.append((name, w))

    def label_pc(self, base: int, index: int) -> int:
        return base + 4 * index

    def finish(self) -> list[int]:
        return [w for _, w in self.words]


def write_scenario(out: Path, base: int, words: list[int],
                   launches: list[tuple[int, int, int]], end_kind: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "code.hex", "w", encoding="ascii", newline="\n") as f:
        for w in words:
            f.write("%08x\n" % w)
    with open(out / "scen.cfg", "w", encoding="ascii", newline="\n") as f:
        f.write("base %x\n" % base)
        f.write("words %d\n" % len(words))
        for wg, nwarps, pc in launches:
            f.write("launch %d %d %x\n" % (wg, nwarps, pc))
        f.write("end %s\n" % end_kind)


# --------------------------------------------------------------------------
# Scenarios.  Launch order is the TB's tb_launch order; the model places
# warps the same way (lowest free slot), so slot numbering matches.
BASE = 0x1000


def scen_simple(out: Path) -> None:
    """One WG, two warps: nop x2, endpgm -- interleaving + termination."""
    p = Prog()
    for _ in range(2):
        p.emit("nop", w_sys(0))
    p.emit("endpgm", w_sys(1))
    write_scenario(out, BASE, p.finish(), [(0, 2, BASE)], "DONE")


def scen_barrier(out: Path) -> None:
    """Two warps rendezvous at a barrier; warp 0 arrives first (bz u29)."""
    p = Prog()
    p.emit("bz", w_bop(0, 29, 16))      # u29 != 0 for warp 1 -> fallthrough
    p.emit("nop", w_sys(0))             # warp 1 walks these
    p.emit("nop", w_sys(0))
    p.emit("barrier", w_sys(6))         # both meet here (word 3)
    p.emit("endpgm", w_sys(1))
    # warp 0 (u29 == 0) jumps +16 -> word 4? off=16 lands at word 4.  Make the
    # jump land exactly on the barrier: bz +12 -> word 3.
    p.words[0] = ("bz", w_bop(0, 29, 12))
    write_scenario(out, BASE, p.finish(), [(0, 2, BASE)], "DONE")


def scen_waitcnt_loop(out: Path) -> None:
    """waitcnt + forward jumps; three warps of one WG interleave."""
    p = Prog()
    p.emit("nop", w_sys(0))
    p.emit("waitcnt", w_waitcnt(63, 31, 31, 15, 3, 7))  # no-op thresholds
    p.emit("sleep", w_sys(4, 3))
    p.emit("setprio", w_sys(5, 2))
    p.emit("jmp", w_jmp(8))             # -> word 6
    p.emit("nop", w_sys(0))             # skipped
    p.emit("barrier", w_sys(6))         # all three meet at word 6
    p.emit("bexecnz", w_bexecnz(8))     # taken (EXEC full) -> word 10
    p.emit("nop", w_sys(0))             # skipped
    p.emit("bexecz", w_bexecz(8))       # not taken (EXEC full)
    p.emit("endpgm", w_sys(1))
    write_scenario(out, BASE, p.finish(), [(0, 3, BASE)], "DONE")


def scen_trap_imm(out: Path) -> None:
    """Two warps; warp 0 traps on `trap 0x15`, warp 1 finishes."""
    p = Prog()
    p.emit("bz", w_bop(0, 29, 8))       # warp 0 -> word 2; warp 1 falls through
    p.emit("endpgm", w_sys(1))          # warp 1 end (word 1)
    p.emit("trap", w_sys(2, 0x15))      # warp 0 traps (word 2)
    p.emit("nop", w_sys(0))             # never reached
    write_scenario(out, BASE, p.finish(), [(0, 2, BASE)], "TRAP")


def scen_two_wgs(out: Path) -> None:
    """Two workgroups with independent barriers, different entry PCs."""
    # WG0 @BASE: barrier round trip (3 warps)
    p = Prog()
    p.emit("nop", w_sys(0))
    p.emit("barrier", w_sys(6))
    p.emit("endpgm", w_sys(1))
    # WG1 @BASE+0x20: waitcnt variants + bnz u29 split (2 warps)
    q = Prog()
    q.emit("bnz", w_bop(1, 29, 12))     # warp 0 (u29=0): NOT taken (bnz, u29==0)
    q.emit("waitcnt", w_waitcnt(0, 31, 31, 15, 3, 7))  # passes (nothing outstanding)
    q.emit("bnone", w_bop(2, 26, 12))   # u26 != 0 -> (u&EXEC)!=0 -> not taken
    q.emit("brk", w_sys(3))             # word 3: warp 0 traps on brk
    q.emit("endpgm", w_sys(1))          # word 4: warp 1 (u29=1) jumps here
    words = p.finish() + [0] * 5 + q.finish()  # pad to 0x20
    write_scenario(out, BASE, words, [(0, 3, BASE), (1, 2, BASE + 0x20)], "TRAP")


def scen_branch_mix(out: Path) -> None:
    """bany/bnone/bz/bnz on u26/u29 with a mixed DONE/TRAP outcome.

    Warp 0 of WG0 (u29 == 0, u26 == 0) walks into the brk; the other warps
    rendezvous at the barrier and finish.  The model aborts the dispatch at
    the brk (trap policy v0.1), so the trace ends there.
    """
    p = Prog()
    p.emit("bany", w_bop(3, 26, 8))     # u26 != 0 for wg>0 -> taken
    p.emit("bnone", w_bop(2, 29, 8))    # u29 != 0 for warps 1+ -> not taken
    p.emit("bnz", w_bop(1, 29, 4))      # taken for warp 1+ -> skips word 3
    p.emit("brk", w_sys(3))             # only warp 0 of WG0 reaches this
    p.emit("barrier", w_sys(6))         # word 4: rendezvous
    p.emit("endpgm", w_sys(1))
    write_scenario(out, BASE, p.finish(), [(0, 3, BASE), (1, 2, BASE)], "TRAP")


SCENARIOS = {
    "simple": scen_simple,
    "barrier": scen_barrier,
    "waitcnt_loop": scen_waitcnt_loop,
    "trap_imm": scen_trap_imm,
    "two_wgs": scen_two_wgs,
    "branch_mix": scen_branch_mix,
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--only", help="comma-separated scenario names")
    args = ap.parse_args()
    out = Path(args.out_dir)
    names = args.only.split(",") if args.only else list(SCENARIOS)
    for n in names:
        if n not in SCENARIOS:
            print("unknown scenario %s" % n)
            return 1
        SCENARIOS[n](out / n)
        print("wrote %s" % (out / n))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
