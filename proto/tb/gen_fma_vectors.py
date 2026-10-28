#!/usr/bin/env python3
"""Generate test vectors for fma32_lane: exact a*b+c rounded once (RNE) to binary32.
Output: fma_vec.hex lines "aaaaaaaa bbbbbbbb cccccccc rrrrrrrr" (NaN -> 7fc00000)."""
import random, struct, sys
from fractions import Fraction

def dec(u):
    s = -1 if u >> 31 else 1
    e = (u >> 23) & 0xFF
    f = u & 0x7FFFFF
    if e == 0xFF:
        return ('nan' if f else ('inf', s))
    if e == 0:
        return Fraction(s * f, 1 << 149)
    return Fraction(s * ((1 << 23) | f), 1) * (Fraction(2) ** (e - 150))

def enc(x, zsign):
    if x == 0:
        return zsign << 31
    s = 1 if x < 0 else 0
    m = -x if x < 0 else x
    # find e with 2^e <= m < 2^(e+1)
    e = m.numerator.bit_length() - m.denominator.bit_length()
    if Fraction(2) ** e > m: e -= 1
    if Fraction(2) ** (e + 1) <= m: e += 1
    q = Fraction(2) ** (max(e, -126) - 23)
    t = m / q
    n = t.numerator // t.denominator
    r = t - n
    if r > Fraction(1, 2) or (r == Fraction(1, 2) and (n & 1)):
        n += 1
    v = n * q
    if v >= Fraction(2) ** 128:
        return (s << 31) | 0x7F800000
    if v < Fraction(2) ** -126:
        return (s << 31) | n
    e2 = v.numerator.bit_length() - v.denominator.bit_length()
    if Fraction(2) ** e2 > v: e2 -= 1
    mant = int(v / Fraction(2) ** (e2 - 23)) - (1 << 23)
    return (s << 31) | ((e2 + 127) << 23) | mant

def fma(a, b, c):
    A, B, C = dec(a), dec(b), dec(c)
    def isnan(x): return x == 'nan'
    def isinf(x): return isinstance(x, tuple)
    if isnan(A) or isnan(B) or isnan(C): return 0x7FC00000
    sp = ((a ^ b) >> 31) & 1
    if isinf(A) or isinf(B):
        if (not isinf(A) and A == 0) or (not isinf(B) and B == 0): return 0x7FC00000
        if isinf(C) and C[1] != (-1 if sp else 1): return 0x7FC00000
        return (sp << 31) | 0x7F800000
    if isinf(C): return ((c >> 31) << 31) | 0x7F800000
    x = A * B + C
    sc = (c >> 31) & 1
    zs = sp & sc
    return enc(x, zs)

def rnd_f32(rng):
    k = rng.random()
    if k < 0.05: return rng.choice([0, 0x80000000, 0x7F800000, 0xFF800000, 0x7FC00000, 0x7F800001, 0x00000001, 0x807FFFFF, 0x00800000, 0x7F7FFFFF])
    if k < 0.20:  # subnormal
        return (rng.getrandbits(1) << 31) | rng.getrandbits(23)
    if k < 0.30:  # tiny normal
        return (rng.getrandbits(1) << 31) | (rng.randint(1, 30) << 23) | rng.getrandbits(23)
    if k < 0.38:  # huge
        return (rng.getrandbits(1) << 31) | (rng.randint(220, 254) << 23) | rng.getrandbits(23)
    return (rng.getrandbits(1) << 31) | (rng.randint(100, 154) << 23) | rng.getrandbits(23)

def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    rng = random.Random(1)
    with open(sys.argv[2] if len(sys.argv) > 2 else 'fma_vec.hex', 'w') as f:
        for i in range(n):
            a, b = rnd_f32(rng), rnd_f32(rng)
            if rng.random() < 0.3:
                # near-cancellation: c ~= -(a*b)
                A, B = dec(a), dec(b)
                if not isinstance(A, (tuple, str)) and not isinstance(B, (tuple, str)):
                    c = enc(-(A * B), 0) ^ (rng.getrandbits(2) if rng.random() < 0.7 else 0)
                else:
                    c = rnd_f32(rng)
            else:
                c = rnd_f32(rng)
            f.write(f"{a:08x} {b:08x} {c:08x} {fma(a, b, c):08x}\n")

if __name__ == '__main__':
    main()
