#!/usr/bin/env python3
"""Generate golden L2 interleave vectors for the testbenches.

Source of truth: (golden definition).  This script is
a faithful Python port of the C++ functions (line references below); there is
no C++ compiler on the Windows box, so the port is kept line-for-line
reviewable instead.  If interleave.cpp ever changes, update this port and
re-run:

    python rtl/gpu/tb/gen_golden/gen_interleave_golden.py

Outputs (rtl/gpu/tb/golden/):
    l2_map_vram_off.hex   48-bit VRAM offset (= PA, VRAM base is 0)
    l2_map_slice.hex      L2 slice index 0..3 (interleave.cpp:50-53)
    l2_map_ch.hex         DDR channel 0=C1, 1=C2, 3=C4
    l2_map_choff.hex      48-bit byte offset inside that channel
    l2_map_count.hex      number of vectors (single entry)

The testbenches tb_l2_slice.sv and tb_noc_l2map.sv read these files and check
the RTL (ng_l2_addrmap + ng_l2_slice AXI addresses + NoC slice routing)
against them.
"""
import os

# / interleave.h constants
GRAN = 4 * 1024                 # kGranule
VRAM = 16 * (1 << 30)           # kVramSize, 16 GiB
C1 = 8 * (1 << 30)              # kC1Size
C2 = 4 * (1 << 30)              # kC2Size
C4 = 4 * (1 << 30)              # kC4Size
SLOT2SLICE = [0, 2, 1, 3]       # interleave.cpp:51


def vram_interleave(off):
    """interleave.cpp:17-26 -- slot = granule&3: 0->C1, 1->C2, 2->C1, 3->C4."""
    g = off // GRAN
    in_ = off % GRAN
    group = g >> 2
    slot = g & 3
    if slot == 0:
        return (0, ((group << 1) | 0) * GRAN + in_)
    if slot == 1:
        return (1, group * GRAN + in_)
    if slot == 2:
        return (0, ((group << 1) | 1) * GRAN + in_)
    return (3, group * GRAN + in_)


def vram_deinterleave(ch, choff):
    """interleave.cpp:28-48 -- inverse; returns None when out of range."""
    cg = choff // GRAN
    in_ = choff % GRAN
    if ch == 0:  # C1
        if choff >= C1:
            return None
        g = ((cg >> 1) << 2) | (2 if (cg & 1) else 0)
    elif ch == 1:  # C2
        if choff >= C2:
            return None
        g = (cg << 2) | 1
    elif ch == 3:  # C4
        if choff >= C4:
            return None
        g = (cg << 2) | 3
    else:
        return None
    return g * GRAN + in_


def l2_slice(off):
    """interleave.cpp:50-53."""
    return SLOT2SLICE[(off // GRAN) & 3]


def selfcheck():
    """Consistency checks of the ported functions themselves."""
    # spot-check the slot pattern C1/C2/C1/C4 over the first 8 granules
    for g, want in enumerate([0, 1, 0, 3, 0, 1, 0, 3]):
        ch, _ = vram_interleave(g * GRAN)
        assert ch == want, (g, ch, want)
    # interleave/deinterleave round trip over a spread of offsets
    for off in [0, 1, GRAN - 1, GRAN, 2 * GRAN + 5, 3 * GRAN - 1,
                (1 << 32) + 12345, VRAM // 2 + 777, VRAM - 1,
                VRAM - GRAN - 3, 0x3FFFFD * GRAN, 0x3FFFFE * GRAN + 9,
                0x3FFFFF * GRAN + 4095]:
        ch, choff = vram_interleave(off)
        back = vram_deinterleave(ch, choff)
        assert back == off, (hex(off), ch, hex(choff), back)
    # slice 0/1 -> C1 (even/odd channel granule), 2 -> C2, 3 -> C4 (SS6.1)
    for g in range(0, 64):
        off = g * GRAN
        ch, choff = vram_interleave(off)
        s = l2_slice(off)
        assert s in (0, 1, 2, 3)
        if s == 0:
            assert ch == 0 and (choff // GRAN) % 2 == 0
        elif s == 1:
            assert ch == 0 and (choff // GRAN) % 2 == 1
        elif s == 2:
            assert ch == 1
        else:
            assert ch == 3


def gen_offsets():
    """Vector set: granule-boundary sweep + channel-capacity edges + random."""
    offs = []

    # 1) boundary sweep around the first 64 granules
    for g in range(0, 64):
        for d in (-8, -1, 0, 1, 7, 8, GRAN // 2, GRAN - 8, GRAN - 1):
            off = g * GRAN + d
            if 0 <= off < VRAM:
                offs.append(off)

    # 2) channel capacity edges: last granules of C2/C4/C1-odd/C1-even and
    #    the very end of VRAM (16 GiB - 1)
    edge_granules = [
        (1 << 22) - 1,               # g max, slot 3 -> C4 last
        (1 << 22) - 2,               # slot 2 -> C1 odd last
        (1 << 22) - 3,               # slot 1 -> C2 last
        (1 << 22) - 4,               # slot 0 -> C1 even last
        (1 << 20) - 1,               # group boundary -1
        (1 << 20),                   # group boundary
    ]
    for g in edge_granules:
        for d in (-8, -1, 0, 1, GRAN // 2, GRAN - 1):
            off = g * GRAN + d
            if 0 <= off < VRAM:
                offs.append(off)

    # 3) random spread (LCG, deterministic, stdlib only)
    state = 0x12345678
    for _ in range(512):
        state = (state * 1664525 + 1013904223) & 0xFFFFFFFF
        offs.append((state * (1 << 32) + state) % VRAM)

    # de-duplicate, keep order
    seen = set()
    out = []
    for off in offs:
        if off not in seen:
            seen.add(off)
            out.append(off)
    return out


def main():
    selfcheck()
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.normpath(os.path.join(here, "..", "golden"))
    os.makedirs(outdir, exist_ok=True)

    offs = gen_offsets()
    with open(os.path.join(outdir, "l2_map_vram_off.hex"), "w") as f_off, \
         open(os.path.join(outdir, "l2_map_slice.hex"), "w") as f_sl, \
         open(os.path.join(outdir, "l2_map_ch.hex"), "w") as f_ch, \
         open(os.path.join(outdir, "l2_map_choff.hex"), "w") as f_co, \
         open(os.path.join(outdir, "l2_map_count.hex"), "w") as f_ct:
        for off in offs:
            ch, choff = vram_interleave(off)
            f_off.write("%012x\n" % off)
            f_sl.write("%x\n" % l2_slice(off))
            f_ch.write("%x\n" % ch)
            f_co.write("%012x\n" % choff)
        f_ct.write("%08x\n" % len(offs))
    print("gen_interleave_golden: wrote %d vectors to %s" % (len(offs), outdir))


if __name__ == "__main__":
    main()
