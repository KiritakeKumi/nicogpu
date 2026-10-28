// dump_valu_fp.cpp -- randomised-operand vector exporter for the f32 VALU subset.
//
// Same model path  and output format (sim/FORMAT.md,
// nicogpu-vec/0.1 arch_insn + the MODE / fflags extension of the FP64 / SFU exporters) as
// sim/fp64/dump_rand.cpp, for the f32 instructions named in a names file.  Per
// instruction it emits N variants with
//   * binary32 operand generators aimed at the FMA datapath: specials (zeros, inf,
//     qNaN/sNaN with payloads, max/min normal, subnormals), exact few-bit fractions
//     (ties), near-1 values, wide exponent ranges, extremes; correlated operand sets that
//     force cancellation (c = -(a*b) +- ulps, b = -a +- ulps) and product exponents steered
//     onto the overflow / underflow / subnormal edges; per-lane injection of NaN / inf /
//     zero combinations,
//   * instruction-specific operands: i32/u32 data for fcvt.f32.i32/u32, byte data for
//     fcvt.f32.ubyteN, float ranges around the integer limits for fcvt.i32/u32.f32,
//     fractions / halves for the round-to-integer ops, exponent counts for fldexp, class
//     masks for cmp.class,
//   * random MODE (rm32 incl. reserved 5..7, DAZ/FTZ, FP trap enables) and preloaded
//     fflags, random explicit rm (0..7) and neg/abs/sat modifiers of the X forms,
//   * random EXEC, re-drawn operand registers (aliasing allowed, out-of-allocation
//     indices), num_vregs in {16, 17, 24, 128}, occasional encoding-bit flips.
// FORMAT extensions (additive): input.csrs {"0": MODE, "1": fflags}, expected.writes.csrs["1"].
//
// usage: dump_valu_fp --names names.txt --out vectors.jsonl [--per N] [--seed S] [--clean] [--f16-sweep 1..7]
//   --clean: arithmetic-focused set -- all registers inside a 128-register allocation, no
//            encoding-bit flips, no reserved rm, FP trap enables off (so every lane's flags and
//            results are visible instead of ending in a trap)
//   --f16-sweep (may replace --names): deterministic binary16 conversion coverage --
//            1: fcvt.f32.f16.lo/hi over all 65536 binary16 patterns (+ the exponent-0 blocks with DAZ16),
//            2: fcvt.f16.f32.lo/hi(.m) / pkcvt.f16.f32(.m) over every exponent around the binary16 range x
//               rounding-boundary mantissas x every rounding mode (MODE.rm16 / rm operand) x FTZ16 / DAZ32,
//            3: both;  4: flag isolation (every lane of a vector holds the same value: tininess-after-rounding edge, exact subnormals under FTZ16)
//   names.txt: one instruction name per line (fp_names.txt: f32 subset; fp16_names.txt: conversions)
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

extern "C" {
#include "softfloat.h"
}
#include "sem_harness.h"
#include "nicogpu_isa.h"

using namespace nicogpu;
using semtest::Harness;
using semtest::Options;
using semtest::Result;

namespace {

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x5a17ull) {}
  uint32_t next() {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return static_cast<uint32_t>((s * 0x2545F4914F6CDD1Dull) >> 32);
  }
  uint32_t below(uint32_t n) { return next() % n; }
};

constexpr u32 kFrac = 0x7fffffu;
u32 mk(bool sign, int e, u32 frac) { return (static_cast<u32>(sign) << 31) | (static_cast<u32>(e) << 23) | (frac & kFrac); }

// few significant mantissa bits: exact binary fractions, ties, integers
u32 few_frac(Rng& r) {
  const unsigned nb = 1 + r.below(6);
  u32 fr = 0;
  for (unsigned i = 0; i < nb; ++i) fr |= 1u << (22 - r.below(10));
  return fr;
}

// binary32 operand.  mode: 0 generic, 1 float -> integer conversion range, 2 round-to-integer
u32 genf(Rng& r, int mode) {
  const bool sign = r.below(2);
  const uint32_t k = r.below(100);
  if (mode == 1) {  // |x| from 2^-3 to 2^34
    const int e = 127 - 3 + static_cast<int>(r.below(38));
    const u32 fr = r.below(3) == 0 ? few_frac(r) : r.next();
    u32 v = mk(sign, e, fr);
    if (r.below(4) == 0) {  // make it an integer
      const int sh = 23 - (e - 127);
      if (sh > 0 && sh < 32) v &= ~((1u << sh) - 1);
    }
    if (k < 8) {
      static const u32 ed[] = {0x4effffffu, 0x4f000000u, 0xcf000000u, 0x4f7fffffu, 0x4f800000u, 0xbf7fffffu, 0x3f7fffffu,
                               0x3f800000u, 0x4eff0000u, 0xcf000001u, 0xbf000000u, 0x3f000000u, 0x4b7fffffu, 0x4b800000u,
                               0x7f800000u, 0xff800000u, 0x7fc00000u, 0x80000000u, 0x4effffc0u, 0x4f7fffc0u};
      return ed[r.below(sizeof ed / sizeof ed[0])] ^ (r.below(6) == 0 ? 0x80000000u : 0u);
    }
    if (k < 12) return mk(sign, r.below(2) ? 0 : 255, r.next());
    return v;
  }
  if (mode == 2) {
    if (k < 25) {
      static const u32 rp[] = {0x3f000000u, 0xbf000000u, 0x3e800000u, 0x3fc00000u, 0x40200000u, 0xbfc00000u, 0xc0200000u,
                               0x3f7fffffu, 0xbf7fffffu, 0x4b7ffffeu, 0x4b000000u, 0x4b000001u, 0x80000000u, 0x00000000u,
                               0x3f800001u, 0xbf800001u, 0x40000000u, 0x40400000u, 0x40a00000u, 0x00000001u, 0x007fffffu,
                               0x3effffffu, 0x3f000001u};
      return rp[r.below(sizeof rp / sizeof rp[0])] ^ (r.below(5) == 0 ? 0x80000000u : 0u);
    }
    if (k < 70) {
      const int e = 127 - 3 + static_cast<int>(r.below(29));
      return mk(sign, e, r.below(2) ? few_frac(r) : r.next());
    }
  }
  if (k < 12) {  // specials
    switch (r.below(14)) {
      case 0: return mk(sign, 0, 0);
      case 1: return mk(sign, 255, 0);
      case 2: return mk(sign, 255, 1u << 22);                          // qNaN
      case 3: return mk(sign, 255, 1 + (r.next() & (kFrac >> 1)));     // sNaN
      case 4: return mk(sign, 255, (1u << 22) | (r.next() & kFrac));
      case 5: return mk(sign, 254, kFrac);                             // max finite
      case 6: return mk(sign, 1, 0);                                   // min normal
      case 7: return mk(sign, 0, kFrac);                               // max subnormal
      case 8: return mk(sign, 0, 1);                                   // min subnormal
      case 9: return mk(sign, 127, 0);                                 // 1.0
      case 10: return mk(sign, 127 + static_cast<int>(r.below(21)) - 10, 0);  // 2^k
      case 11: return mk(sign, 126, 0);                                // 0.5
      case 12: return mk(sign, 126, kFrac);                            // just below 1
      default: return mk(sign, 127, 1);                                // 1 + ulp
    }
  }
  if (k < 24) return mk(sign, 0, r.next());                                          // subnormal / zero
  if (k < 38) return mk(sign, 127 + static_cast<int>(r.below(9)) - 4, few_frac(r));  // exact fractions
  if (k < 62) return mk(sign, 127 + static_cast<int>(r.below(7)) - 3, r.next());     // near 1
  if (k < 74) return mk(sign, 1 + static_cast<int>(r.below(254)), r.next());         // any finite normal
  if (k < 84) {                                                                      // near the extremes
    const int e = r.below(2) ? 1 + static_cast<int>(r.below(6)) : 253 - static_cast<int>(r.below(6));
    return mk(sign, e, r.below(3) ? r.next() : (r.below(2) ? kFrac : 0));
  }
  if (k < 92) return mk(sign, 127 + static_cast<int>(r.below(60)) - 30, r.next());   // wide but sane
  return r.next();
}

u32 neg32(u32 x) { return x ^ 0x80000000u; }

u32 mul_rne(u32 a, u32 b) {
  softfloat_roundingMode = softfloat_round_near_even;
  float32_t x, y;
  x.v = a;
  y.v = b;
  const u32 z = f32_mul(x, y).v;
  softfloat_exceptionFlags = 0;
  return z;
}

u32 ulps(Rng& r, u32 x) {
  switch (r.below(6)) {
    case 0: return x;
    case 1: return x + 1;
    case 2: return x - 1;
    case 3: return x + 2;
    case 4: return x - 2;
    default: return x + r.below(5) - 2;
  }
}

// steer the biased exponents of (a, b) so that the product lands on an interesting exponent
void steer(Rng& r, u32& a, u32& b) {
  static const int targets[] = {254 + 127 - 1, 254 + 127, 254 + 127 + 1, 127 + 1, 127, 127 - 1, 127 - 10,
                                127 - 22, 127 - 23, 127 - 24, 127 - 25, 127 - 26, 127 - 40, 127 - 126, 127 - 149};
  const int t = targets[r.below(sizeof targets / sizeof targets[0])] + static_cast<int>(r.below(3)) - 1;
  const int ea = 1 + static_cast<int>(r.below(253));
  int eb = t - ea;
  if (eb < 1) eb = 1 + static_cast<int>(r.below(4));
  if (eb > 254) eb = 254 - static_cast<int>(r.below(4));
  a = mk(a >> 31, ea, r.below(3) ? r.next() : few_frac(r));
  b = mk(b >> 31, eb, r.below(3) ? r.next() : few_frac(r));
}

u32 gen_int(Rng& r) {
  static const u32 corner[] = {0u, 1u, 0xffffffffu, 0x80000000u, 0x7fffffffu, 2u, 3u, 0xfffffffeu, 0xffffu, 100u,
                               0x80000001u, 0x00ffffffu, 0x01000000u, 0x01000001u, 0xfffffffu, 0x7fffff80u, 0x7fffffc0u,
                               0xffffff80u, 0xffffffc0u, 0x80000040u, 0x02000001u};
  switch (r.below(6)) {
    case 0: return corner[r.below(sizeof corner / sizeof corner[0])];
    case 1: return r.below(70);
    case 2: return 0u - r.below(70);
    case 3: return r.next() >> r.below(32);
    default: return r.next();
  }
}

// fldexp count: small, around the exponent range edges, huge
u32 gen_exp(Rng& r) {
  switch (r.below(8)) {
    case 0: return 0u;
    case 1: return r.below(8) - 4u;
    case 2: return static_cast<u32>(static_cast<int>(r.below(700)) - 350);
    case 3: return static_cast<u32>(static_cast<int>(r.below(60)) - 30);
    case 4: return static_cast<u32>(static_cast<int>(r.below(10)) + 120);
    case 5: return static_cast<u32>(-149 - static_cast<int>(r.below(6)) + 3);
    case 6: return gen_int(r);
    default: return static_cast<u32>(static_cast<int>(r.below(2400)) - 1200);
  }
}

u32 gen_class(Rng& r) {
  switch (r.below(5)) {
    case 0: return 0x3ffu;
    case 1: return 1u << r.below(10);
    case 2: return r.next() & 0x3ffu;
    case 3: return 0x300u;
    default: return r.next();
  }
}

u32 rand_exec(Rng& r) {
  switch (r.below(9)) {
    case 0: return 0x55555555u;
    case 1: case 2: return 0xffffffffu;
    case 3: return 0u;
    case 4: return 1u << r.below(32);
    case 5: return 0xffff0000u;
    case 6: return 0x0000ffffu;
    case 7: return 0xaaaaaaaau;
    default: return r.next();
  }
}
u32 rand_mode(Rng& r) {
  u32 m = r.below(5);
  if (r.below(25) == 0) m = 5 + r.below(3);                   // reserved rm32 5..7 (rare)
  m |= (r.below(5) << 3) | (r.below(5) << 6);                 // rm16 / rm64 (unused by f32)
  for (unsigned b = 9; b <= 14; ++b)
    if (r.below(3) == 0) m |= 1u << b;
  // FP trap enables turn most vectors into FpException traps (flags are frequent): keep them rarer
  if (r.below(8) == 0) m |= (r.below(2) ? (1u << r.below(5)) : r.below(32)) << 16;
  return m;
}

// ---- binary16 <-> binary32 conversion operands ---------------------------------------
// one binary16 value: specials, subnormals (DAZ16), anything
u32 gen_h(Rng& r) {
  const u32 s = r.below(2) ? 0x8000u : 0u;
  const uint32_t k = r.below(100);
  if (k < 20) {
    static const u32 sp[] = {0x0000u, 0x7c00u, 0x7e00u, 0x7c01u, 0x7d55u, 0x7fffu, 0x7bffu, 0x0400u,
                             0x03ffu, 0x0001u, 0x3c00u, 0x3bffu, 0x0200u, 0x7e01u, 0x0401u, 0x3c01u};
    return sp[r.below(sizeof sp / sizeof sp[0])] | s;
  }
  if (k < 40) return (r.next() & 0x3ffu) | s;   // subnormal / zero
  return r.next() & 0xffffu;
}
u32 gen_h2(Rng& r) { return gen_h(r) | (gen_h(r) << 16); }
// binary32 aimed at the binary16 range: rounding patterns at the 13 dropped bits, the overflow and
// underflow edges, f32 specials and subnormals (DAZ32)
u32 gen_f2h(Rng& r) {
  const bool sign = r.below(2);
  const uint32_t k = r.below(100);
  static const u32 low13[] = {0x0000u, 0x0001u, 0x0fffu, 0x1000u, 0x1001u, 0x1fffu, 0x0800u, 0x17ffu};
  if (k < 10) {
    static const u32 sp[] = {0x00000000u, 0x7f800000u, 0x7fc00000u, 0x7f800001u, 0x7fa00000u, 0x7fc00123u,
                             0x00000001u, 0x007fffffu, 0x00400000u, 0x7f7fffffu, 0x00800000u, 0x3f800000u};
    return sp[r.below(sizeof sp / sizeof sp[0])] ^ (sign ? 0x80000000u : 0u);
  }
  if (k < 25) {
    static const u32 ed[] = {0x477fe000u, 0x477ff000u, 0x477fefffu, 0x477ff001u, 0x477fffffu, 0x47800000u,
                             0x38800000u, 0x387fe000u, 0x387ff000u, 0x387fffffu, 0x38800001u, 0x33800000u,
                             0x33000000u, 0x33000001u, 0x32ffffffu, 0x33c00000u, 0x33bfffffu, 0x32800000u,
                             0x387fc000u, 0x38000000u, 0x37800000u, 0x477fd000u, 0x46fff000u, 0x3f801000u};
    return ed[r.below(sizeof ed / sizeof ed[0])] ^ (sign ? 0x80000000u : 0u);
  }
  if (k < 80) {   // biased f32 exponent 99..143: f16 subnormal / normal / overflow
    const int e = 99 + static_cast<int>(r.below(45));
    u32 fr = r.next() & kFrac;
    if (r.below(3)) fr = (fr & ~0x1fffu) | low13[r.below(8)];
    if (r.below(8) == 0) fr |= 0x7fe000u;   // carry into the exponent
    return mk(sign, e, fr);
  }
  if (k < 90) return mk(sign, 1 + static_cast<int>(r.below(254)), r.next());
  return r.next();
}
// rm16 for the binary16 results: 0..4, 7 (= follow MODE.rm32), rarely the reserved 5 / 6
u32 rand_rm16(Rng& r, bool clean) {
  const uint32_t k = r.below(16);
  if (k < 3) return 7;
  if (!clean && k == 3) return 5 + r.below(2);
  return r.below(5);
}

std::string hex32(u32 v) {
  char b[16];
  std::snprintf(b, sizeof b, "0x%08x", v);
  return b;
}
std::string id_of(unsigned insn, unsigned k) {
  char b[32];
  std::snprintf(b, sizeof b, "rnd-%04u-%03u", insn, k);
  return b;
}
std::string esc(const std::string& s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

constexpr unsigned kPre = 16;

bool starts(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }
bool is_f16cvt(const std::string& n) {
  return starts(n, "fcvt.f32.f16") || starts(n, "fcvt.f16.f32") || starts(n, "pkcvt.f16.f32");
}

// data class of source operand idx (0 = rs1, 1 = rs2, 2 = rs3) of instruction `n`: 0 float, 3 int, 4 exp count,
// 5 class mask, 6 any bits, 1/2 float modes for conversions / round-to-integer
int src_class(const std::string& n, int idx) {
  if (starts(n, "fcvt.f32.f16")) return 8;                                   // binary16 halves
  if (starts(n, "fcvt.f16.f32") || starts(n, "pkcvt.f16.f32")) return 9;   // f32 aimed at the f16 range
  if (starts(n, "fcvt.f32.i32") || starts(n, "fcvt.f32.u32")) return 3;
  if (starts(n, "fcvt.f32.ubyte")) return 6;
  if (starts(n, "fcvt.i32.f32") || starts(n, "fcvt.u32.f32")) return 1;
  if (n == "ffloor.f32" || n == "fceil.f32" || n == "ftrunc.f32" || n == "frndne.f32" || n == "ffract.f32") return 2;
  if (starts(n, "fldexp") && idx == 1) return 4;
  if (starts(n, "cmp.class") && idx == 1) return 5;
  if (n == "fclass.f32") return 6;
  if (starts(n, "fsgnj")) return 7;
  return 0;
}
u32 gen_class_val(Rng& r, int cls) {
  switch (cls) {
    case 8: return gen_h2(r);
    case 9: return gen_f2h(r);
    case 1: return genf(r, 1);
    case 2: return genf(r, 2);
    case 3: return gen_int(r);
    case 4: return gen_exp(r);
    case 5: return gen_class(r);
    case 6: return r.below(3) == 0 ? genf(r, 0) : r.next();
    default: return genf(r, 0);
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string names_path, out_path;
  unsigned per = 24;
  uint64_t seed = 1;
  bool clean = false;
  int f16_sweep = 0;   // 1 = binary16 -> binary32 sweep, 2 = binary32 -> binary16 sweep, 3 = both
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", a.c_str()); std::exit(2); }
      return argv[++i];
    };
    if (a == "--names") names_path = next();
    else if (a == "--out") out_path = next();
    else if (a == "--per") per = static_cast<unsigned>(std::atoi(next().c_str()));
    else if (a == "--seed") seed = static_cast<uint64_t>(std::atoll(next().c_str()));
    else if (a == "--clean") clean = true;
    else if (a == "--f16-sweep") f16_sweep = std::atoi(next().c_str());
    else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  if ((names_path.empty() && !f16_sweep) || out_path.empty()) {
    std::fprintf(stderr, "usage: dump_valu_fp (--names names.txt | --f16-sweep 1..7) --out vectors.jsonl [--per N] [--seed S]\n");
    return 2;
  }
  const isa::IsaTable& t = nicogpu_isa_table();
  const int endpgm = t.find("endpgm");
  u64 endpgm_word = 0;
  unsigned endpgm_len = 0;
  std::string err;
  if (endpgm < 0 || !isa::encode(t, endpgm, nullptr, 0, &endpgm_word, &endpgm_len, &err)) {
    std::fprintf(stderr, "endpgm: %s\n", err.c_str());
    return 2;
  }

  std::vector<std::string> names;
  std::set<std::string> known;
  {
    std::ifstream kf(names_path);
    if (!kf && !f16_sweep) { std::fprintf(stderr, "cannot open %s\n", names_path.c_str()); return 2; }
    std::string kl;
    while (std::getline(kf, kl)) {
      std::istringstream ks(kl);
      std::string kn;
      if (ks >> kn) { names.push_back(kn); known.insert(kn); }
    }
  }
  std::FILE* f = std::fopen(out_path.c_str(), "wb");
  if (!f) { std::fprintf(stderr, "cannot write %s\n", out_path.c_str()); return 2; }
  std::fprintf(f, "{\"kind\":\"file_header\",\"format\":\"nicogpu-vec/0.1\",\"owner_unit\":\"valu_fp\",\"isa\":\"%s\","
                  "\"code_base\":\"0x0000000000100000\",\"data_base\":\"0x0000000000400000\",\"num_vregs\":128,"
                  "\"threads\":32,\"lanes\":32,\"exec_init\":\"0x55555555\",\"generator\":\"nicogpu_valu_fp_rand\"}\n",
               t.isa_version);

  static const u32 kNregs[] = {16, 17, 24, 128, 128, 128};
  int emitted = 0, traps = 0, skipped = 0, fptraps = 0;
  unsigned insn_no = 0;
  auto emit_one = [&](const std::string& id, const std::string& vname, const std::string& disasm, unsigned width_bits,
                      const std::vector<u32>& words, const std::vector<u32>& iv, const std::vector<u32>& iu, u32 exec,
                      u32 mode, u32 ff0, u32 nv, const Options& o) -> bool {
    Harness h;
    std::vector<u32> code = words;
    code.push_back(static_cast<u32>(endpgm_word));
    if (endpgm_len == 8) code.push_back(static_cast<u32>(endpgm_word >> 32));
    const Result r = h.run(code, [&](sp::Warp& w) {
      w.exec = exec;
      w.csr.mode = mode;
      w.csr.fflags = ff0;
      for (unsigned reg = 0; reg < kPre; ++reg)
        for (unsigned l = 0; l < 32; ++l) w.v[reg * 32 + l] = iv[reg * 32 + l];
      for (unsigned reg = 0; reg < 64; ++reg) w.u[reg] = iu[reg];
    }, o);
    if (r.ok && r.w(0).state != sp::WarpState::Done) {
      std::fprintf(stderr, "%s: warp did not retire\n", vname.c_str());
      return false;
    }
    const u32 ff1 = r.w(0).csr.fflags;

    // -
    std::fprintf(f, "{\"kind\":\"arch_insn\",\"id\":\"%s\",\"owner_unit\":\"valu_fp\",", id.c_str());
    std::fprintf(f, "\"insn\":{\"name\":\"%s\",\"disasm\":\"%s\",\"width_bits\":%u,\"unit_table\":\"valu\",\"words\":[",
                 vname.c_str(), esc(disasm).c_str(), width_bits);
    for (size_t i = 0; i < words.size(); ++i) std::fprintf(f, "%s\"%s\"", i ? "," : "", hex32(words[i]).c_str());
    std::fprintf(f, "]},\"input\":{\"pc\":0,\"exec\":\"%s\",\"num_vregs\":%u,\"csrs\":{\"0\":\"%s\",\"1\":\"%s\"},\"vregs\":[",
                 hex32(exec).c_str(), nv, hex32(mode).c_str(), hex32(ff0).c_str());
    for (unsigned reg = 0; reg < kPre; ++reg) {
      std::fprintf(f, "%s[", reg ? "," : "");
      for (unsigned l = 0; l < 32; ++l) std::fprintf(f, "%s\"%s\"", l ? "," : "", hex32(iv[reg * 32 + l]).c_str());
      std::fprintf(f, "]");
    }
    std::fprintf(f, "],\"uregs\":{");
    bool first = true;
    for (unsigned reg = 0; reg < 64; ++reg) {
      if (iu[reg] == 0) continue;
      std::fprintf(f, "%s\"%u\":\"%s\"", first ? "" : ",", reg, hex32(iu[reg]).c_str());
      first = false;
    }
    std::fprintf(f, "},\"memory\":[]},\"expected\":{\"trap\":");
    if (!r.ok) {
      ++traps;
      const sp::Warp& w = r.w(0);
      if (w.csr.trapcause == 4) ++fptraps;
      std::fprintf(f, "{\"cause\":%u,\"pc\":%u,\"info\":\"%s\",\"message\":\"%s\"},\"writes\":{", w.csr.trapcause,
                   w.csr.trappc, hex32(w.csr.trapinfo).c_str(), esc(r.error).c_str());
      if (ff1 != ff0) std::fprintf(f, "\"csrs\":{\"1\":\"%s\"}", hex32(ff1).c_str());
      std::fprintf(f, "}");
    } else {
      std::fprintf(f, "null,\"writes\":{\"vregs\":{");
      first = true;
      for (unsigned reg = 0; reg < nv; ++reg) {
        bool same = true;
        for (unsigned l = 0; l < 32; ++l) {
          const u32 before = reg < kPre ? iv[reg * 32 + l] : 0u;
          if (r.w(0).v[reg * 32 + l] != before) { same = false; break; }
        }
        if (same) continue;
        std::fprintf(f, "%s\"%u\":[", first ? "" : ",", reg);
        first = false;
        for (unsigned l = 0; l < 32; ++l)
          std::fprintf(f, "%s\"%s\"", l ? "," : "", hex32(r.w(0).v[reg * 32 + l]).c_str());
        std::fprintf(f, "]");
      }
      std::fprintf(f, "},\"uregs\":{");
      first = true;
      for (unsigned reg = 0; reg < 64; ++reg) {
        if (r.w(0).u[reg] == iu[reg]) continue;
        std::fprintf(f, "%s\"%u\":\"%s\"", first ? "" : ",", reg, hex32(r.w(0).u[reg]).c_str());
        first = false;
      }
      std::fprintf(f, "},\"exec\":%s,\"csrs\":{", r.w(0).exec != exec ? ("\"" + hex32(r.w(0).exec) + "\"").c_str() : "null");
      if (ff1 != ff0) std::fprintf(f, "\"1\":\"%s\"", hex32(ff1).c_str());
      std::fprintf(f, "}}");
    }
    std::fprintf(f, "}}\n");
    ++emitted;
    return true;
  };
  for (const std::string& name : names) {
    const int bidx = t.find(name.c_str());
    if (bidx < 0) { std::fprintf(stderr, "unknown instruction %s\n", name.c_str()); return 2; }
    const isa::InsnDesc& d = t.insns[bidx];
    Rng rng(seed * 1000003ull + insn_no);
    ++insn_no;

    for (unsigned k = 0; k < per; ++k) {
      Options o;
      o.num_vregs = clean ? 128 : kNregs[rng.below(6)];
      const u32 nv = o.num_vregs;
      // -
      i64 ops[isa::kMaxOperands];
      for (unsigned oi = 0; oi < d.nops; ++oi) {
        const char* kind = d.ops[oi].kind;
        i64 v = 0;
        if (std::strcmp(kind, "vreg") == 0) {
          v = rng.below(16);
          if (nv < 128 && rng.below(6) == 0) v = nv + rng.below(2);
        } else if (std::strcmp(kind, "ureg") == 0) {
          v = rng.below(64);
        } else if (std::strcmp(kind, "rm") == 0) {
          v = (!clean && rng.below(10) == 0) ? 5 + rng.below(3) : (rng.below(4) == 0 ? 7 : rng.below(5));
        } else if (std::strcmp(kind, "mod3") == 0) {
          v = rng.below(3) == 0 ? rng.below(8) : 0;
        } else if (std::strcmp(kind, "uimm1") == 0) {
          v = rng.below(4) == 0 ? 1 : 0;
        } else if (std::strcmp(kind, "imm32") == 0) {
          v = static_cast<i64>(static_cast<i32>(gen_class_val(rng, src_class(name, 1))));
        }
        ops[oi] = v;
      }
      u64 word = 0;
      unsigned elen = 0;
      if (!isa::encode(t, bidx, ops, d.nops, &word, &elen, &err)) { ++skipped; continue; }
      std::string vname = d.name;
      const isa::InsnDesc* vd = &d;
      if (!clean && rng.below(12) == 0) {
        word ^= u64{1} << (2 + rng.below(elen * 8 - 2));
        isa::DecodedInsn md;
        if (isa::decode(t, word, elen, &md)) {
          const std::string mn = t.insns[md.index].name;
          if (!known.count(mn)) { ++skipped; continue; }
          vname = mn;
          vd = &t.insns[md.index];
        }
      }
      std::vector<u32> words{static_cast<u32>(word)};
      if (elen == 8) words.push_back(static_cast<u32>(word >> 32));
      isa::DecodedInsn dec;
      std::string disasm = "illegal";
      const bool decoded = isa::decode(t, word, elen, &dec);
      if (decoded) disasm = isa::disassemble(dec);

      // --- state: random float registers, then format-aware source operands
      std::vector<u32> iv(kPre * 32), iu(64);
      for (auto& x : iv) x = genf(rng, 0);
      for (auto& x : iu) x = genf(rng, 0);
      if (decoded) {
        const bool fma_like = vname == "ffma.f32" || starts(vname, "fmul.f32") || starts(vname, "fmac.f32");
        const bool add_like = starts(vname, "fadd.f32") || starts(vname, "fsub.f32");
        i64 pr[3] = {-1, -1, -1};   // rs1, rs2, rs3 registers (or -1)
        i64 acc = -1;
        for (unsigned oi = 0; oi < vd->nops; ++oi) {
          const char* kind = vd->ops[oi].kind;
          const char* field = vd->ops[oi].field;
          const i64 r = dec.op[oi];
          if (std::strcmp(field, "rd") == 0 && starts(vname, "fmac.f32") && std::strcmp(kind, "vreg") == 0 && r < static_cast<i64>(kPre)) {
            acc = r;
            for (unsigned l = 0; l < 32; ++l) iv[r * 32 + l] = genf(rng, 0);
          }
          if (vd->ops[oi].role != isa::Role::Src) continue;
          const int idx = std::strcmp(field, "rs1") == 0 ? 0 : std::strcmp(field, "rs2") == 0 ? 1 : 2;
          const int cls = src_class(vname, idx);
          if (std::strcmp(kind, "vreg") == 0 && r >= 0 && r < static_cast<i64>(kPre)) {
            pr[idx] = r;
            for (unsigned l = 0; l < 32; ++l) iv[r * 32 + l] = gen_class_val(rng, cls);
          } else if (std::strcmp(kind, "ureg") == 0 && r >= 0 && r < 64) {
            iu[r] = gen_class_val(rng, cls);
          }
        }
        (void)acc;
        const bool distinct = pr[0] >= 0 && pr[1] >= 0 && pr[0] != pr[1];
        // special-value combinations injected lane by lane
        const unsigned inj = rng.below(4);
        const int cls0 = src_class(vname, 0);
        if (inj != 0 && (cls0 == 0 || cls0 == 1 || cls0 == 2 || cls0 == 7)) {
          static const u32 pool[] = {0x7fc00000u, 0xffc00000u, 0x7f800001u, 0xffa00000u, 0x7f800000u, 0xff800000u,
                                     0x00000000u, 0x80000000u, 0x3f800000u, 0xbf800000u, 0x00000001u, 0x7f7fffffu,
                                     0x00800000u, 0x007fffffu, 0x7fc00123u};
          static const u32 rpool[] = {0xbf000000u, 0x3f000000u, 0xbe800000u, 0x3fc00000u, 0x40200000u, 0xbfc00000u,
                                      0xc0200000u, 0x3f7fffffu, 0xbf7fffffu, 0x4b7ffffeu, 0x4effffffu, 0xcf000000u,
                                      0x4f7fffffu, 0x4b000000u, 0x80000000u, 0x33800000u, 0x3f800001u, 0xbf800001u};
          auto put = [&](i64 reg, unsigned l, u32 x) { iv[reg * 32 + l] = x; };
          if (inj == 1) {
            for (unsigned n = 1 + rng.below(2); n > 0; --n) {
              const unsigned l = rng.below(32);
              for (int ix = 0; ix < 3; ++ix) {
                if (pr[ix] < 0) continue;
                if (src_class(vname, ix) >= 3 && src_class(vname, ix) != 7) continue;
                if ((cls0 == 1 || cls0 == 2) && ix == 0) put(pr[ix], l, rpool[rng.below(static_cast<unsigned>(sizeof rpool / sizeof rpool[0]))]);
                else put(pr[ix], l, pool[rng.below(8u)]);
              }
            }
          } else {
            for (int ix = 0; ix < 3; ++ix) {
              if (pr[ix] < 0) continue;
              if (src_class(vname, ix) >= 3 && src_class(vname, ix) != 7) continue;
              for (unsigned l = 0; l < 32; ++l)
                if (rng.below(4) < (inj == 3 ? 3u : 1u))
                  put(pr[ix], l, pool[rng.below(inj == 3 ? 8u : static_cast<unsigned>(sizeof pool / sizeof pool[0]))]);
            }
          }
        }
        auto get = [&](i64 reg, unsigned l) { return iv[reg * 32 + l]; };
        auto put2 = [&](i64 reg, unsigned l, u32 x) { iv[reg * 32 + l] = x; };
        if (distinct && fma_like && pr[1] >= 0 && src_class(vname, 1) == 0 && rng.below(2) == 0) {
          for (unsigned l = 0; l < 32; ++l) {
            if (rng.below(3) == 0) {
              u32 a = get(pr[0], l), b = get(pr[1], l);
              steer(rng, a, b);
              put2(pr[0], l, a);
              put2(pr[1], l, b);
            }
          }
        }
        if (distinct && vname == "ffma.f32" && pr[2] >= 0 && pr[2] != pr[0] && pr[2] != pr[1] && rng.below(2) == 0) {
          for (unsigned l = 0; l < 32; ++l)
            if (rng.below(2) == 0) put2(pr[2], l, ulps(rng, neg32(mul_rne(get(pr[0], l), get(pr[1], l)))));
        }
        // directed rounding-boundary triples for the fused multiply-add (exact ties, tininess after
        // rounding, overflow by rounding, exact cancellation, subnormal ties); signs flipped together
        if (distinct && vname == "ffma.f32" && pr[2] >= 0 && pr[2] != pr[0] && pr[2] != pr[1] && rng.below(2) == 0) {
          static const u32 et[][3] = {
              {0x0dc00000u, 0x26800000u, 0x007fffffu},  // 1.5*2^-100 * 2^-50 + (2^-126 - 2^-149): sum = 2^-126 - 2^-151
              {0x0dc00000u, 0x26800000u, 0x00000000u},  // 1.5 * 2^-150
              {0x0d800000u, 0x26800000u, 0x00000000u},  // exactly half the smallest subnormal
              {0x0d800000u, 0x26800000u, 0x80000000u},
              {0x0d800000u, 0x26800000u, 0x00000001u},  // 1.5 * 2^-149 tie
              {0x7f7fffffu, 0x3f800000u, 0x73000000u},  // max finite + half ulp: tie -> overflow
              {0x7f7fffffu, 0x3f800000u, 0x72ffffffu},
              {0x7f7fffffu, 0x3f800000u, 0x73000001u},
              {0x40400000u, 0x40a00000u, 0xc1700000u},  // 3*5 - 15 = 0
              {0x00000003u, 0x3f000000u, 0x00000000u},  // 1.5 * min subnormal
              {0x3fffffffu, 0x3f800000u, 0x33000000u},  // (2 - 2^-23) + 2^-25: carry into the exponent
              {0x3fffffffu, 0x3f800000u, 0x33800000u},
              {0x00800000u, 0x3f800000u, 0x80000001u},  // min normal - min subnormal
              {0x00800000u, 0x3f000000u, 0x00000000u},  // min normal / 2 (exact subnormal)
              {0x00ffffffu, 0x3f000000u, 0x00000000u},
              {0x3f800000u, 0x3f800000u, 0xb3800000u},  // 1 - 2^-24
              {0x7f000000u, 0x7f000000u, 0xff800000u},  // overflowing product + (-inf)
              {0x7f7fffffu, 0x40000000u, 0x00000000u},  // 2 * max finite -> overflow
          };
          for (unsigned l = 0; l < 32; ++l)
            if (rng.below(3) == 0) {
              const u32* t3 = et[rng.below(sizeof et / sizeof et[0])];
              const u32 sg = rng.below(2) ? 0x80000000u : 0u;
              put2(pr[0], l, t3[0] ^ sg);
              put2(pr[1], l, t3[1]);
              put2(pr[2], l, t3[2] ^ sg);
            }
        }
        if (distinct && add_like && starts(vname, "f") && vd->nops >= 3 && std::strcmp(vd->ops[2].kind, "vreg") == 0 && rng.below(2) == 0) {
          for (unsigned l = 0; l < 32; ++l)
            if (rng.below(2) == 0) {
              const u32 a = get(pr[0], l);
              put2(pr[1], l, ulps(rng, rng.below(2) ? neg32(a) : a));
            }
        }
        if (distinct && (vname == "fmed3.f32" || starts(vname, "fmin.f32") || starts(vname, "fmax.f32") ||
                         starts(vname, "feq") || starts(vname, "flt") || starts(vname, "fle") || starts(vname, "cmp.")) &&
            src_class(vname, 1) == 0 && rng.below(3) == 0) {
          for (unsigned l = 0; l < 32; ++l)
            if (rng.below(3) == 0) put2(pr[1], l, rng.below(2) ? get(pr[0], l) : ulps(rng, get(pr[0], l)));
        }
      }
      const u32 exec = rand_exec(rng);
      u32 mode = rand_mode(rng);
      if (clean) { mode &= ~(0x1fu << 16); mode &= ~7u; mode |= rng.below(5); }
      if (is_f16cvt(vname)) mode = (mode & ~(7u << 3)) | (rand_rm16(rng, clean) << 3);   // rm16 matters
      const u32 ff0 = rng.below(2) ? 0u : rng.below(32);

      if (!emit_one(id_of(insn_no - 1, k), vname, disasm, vd->width_bits, words, iv, iu, exec, mode, ff0, nv, o)) return 1;
    }
  }  // ---- binary16 conversion sweeps (deterministic coverage, full EXEC, 128 registers, no trap enables)
  if (f16_sweep) {
    Rng rng(seed * 7919ull + 39);
    unsigned sno = 0;
    // encode `nm` with rd = v2, rs1 = v1, rs2 = v3 and the given rm operand
    auto sweep_one = [&](const char* nm, i64 rmop, std::vector<u32>& iv, u32 mode) -> bool {
      const int bi = t.find(nm);
      if (bi < 0) { std::fprintf(stderr, "unknown instruction %s\n", nm); return false; }
      const isa::InsnDesc& sd = t.insns[bi];
      i64 ops[isa::kMaxOperands];
      for (unsigned oi = 0; oi < sd.nops; ++oi) {
        const char* fld = sd.ops[oi].field;
        ops[oi] = std::strcmp(fld, "rd") == 0 ? 2 : std::strcmp(fld, "rs1") == 0 ? 1 : std::strcmp(fld, "rs2") == 0 ? 3 : rmop;
      }
      u64 word = 0;
      unsigned elen = 0;
      if (!isa::encode(t, bi, ops, sd.nops, &word, &elen, &err)) { std::fprintf(stderr, "%s: %s\n", nm, err.c_str()); return false; }
      std::vector<u32> words{static_cast<u32>(word)};
      if (elen == 8) words.push_back(static_cast<u32>(word >> 32));
      isa::DecodedInsn dec;
      const std::string disasm = isa::decode(t, word, elen, &dec) ? isa::disassemble(dec) : "illegal";
      std::vector<u32> iu(64);
      for (auto& x : iu) x = genf(rng, 0);
      Options o;
      o.num_vregs = 128;
      char id[48];
      std::snprintf(id, sizeof id, "f16sw-%05u", sno++);
      return emit_one(id, nm, disasm, sd.width_bits, words, iv, iu, 0xffffffffu, mode, 0u, 128u, o);
    };
    auto fill = [&](std::vector<u32>& iv) {
      iv.assign(kPre * 32, 0);
      for (auto& x : iv) x = genf(rng, 0);
    };
    // irrelevant MODE bits are random: DAZ32 / FTZ32 (f16 -> f32), rm32 0..4, rm64
    auto noise = [&]() -> u32 { return rng.below(5) | (rng.below(5) << 6) | (rng.below(2) << 9) | (rng.below(2) << 10); };
    if (f16_sweep & 1) {
      // binary16 -> binary32: all 65536 patterns in the selected half (the other half random), DAZ16 off; the
      // blocks holding zeros / subnormals (exponent 0) once more with DAZ16 on.  FTZ16 / rm16 do not matter.
      for (const char* nm : {"fcvt.f32.f16.lo", "fcvt.f32.f16.hi"}) {
        const bool hi = std::strcmp(nm, "fcvt.f32.f16.hi") == 0;
        for (int pass = 0; pass < 2; ++pass)
          for (u32 blk = 0; blk < 2048; ++blk) {
            if (pass == 1 && ((blk * 32) & 0x7c00u) != 0) continue;
            std::vector<u32> iv;
            fill(iv);
            for (unsigned l = 0; l < 32; ++l) {
              const u32 p = blk * 32 + l, other = gen_h(rng);
              iv[1 * 32 + l] = hi ? (p << 16) | other : (other << 16) | p;
            }
            u32 mode = noise() | (rand_rm16(rng, true) << 3) | (rng.below(2) << 12);
            if (pass) mode |= 1u << 11;
            if (!sweep_one(nm, 0, iv, mode)) return 1;
          }
      }
    }
    if (f16_sweep & 2) {
      // binary32 -> binary16: every exponent around the binary16 range (+ f32 subnormals and the top end), 16
      // mantissa patterns aimed at the 13 dropped bits (ties, sticky, carry into the exponent) x 2 signs per vector,
      // each rounding mode through MODE.rm16 (plain forms) or the rm operand (.m, also rm = dyn); FTZ16 and DAZ32
      // variants around the subnormal edges.
      static const u32 pats[16] = {0x000000u, 0x000001u, 0x000fffu, 0x001000u, 0x001001u, 0x001fffu, 0x002000u,
                                   0x003000u, 0x7fe000u, 0x7ff000u, 0x7fefffu, 0x7ff001u, 0x7fffffu, 0x401000u,
                                   0x000000u, 0x000000u};   // 14, 15: random (one with a tie at bit 12)
      std::vector<int> exps;
      for (int e = 0; e <= 3; ++e) exps.push_back(e);
      for (int e = 95; e <= 145; ++e) exps.push_back(e);
      for (int e = 251; e <= 255; ++e) exps.push_back(e);
      for (const char* nm : {"fcvt.f16.f32.lo", "fcvt.f16.f32.hi", "pkcvt.f16.f32", "fcvt.f16.f32.lo.m",
                             "fcvt.f16.f32.hi.m", "pkcvt.f16.f32.m"}) {
        const bool dotm = std::strlen(nm) > 2 && std::strcmp(nm + std::strlen(nm) - 2, ".m") == 0;
        for (unsigned rmi = 0; rmi < (dotm ? 6u : 5u); ++rmi) {
          for (int e : exps) {
            const bool edge = e <= 3 || (e >= 96 && e <= 115);
            for (int var = 0; var < (edge ? 3 : 1); ++var) {
              std::vector<u32> iv;
              fill(iv);
              for (unsigned l = 0; l < 32; ++l) {
                u32 fr = pats[l & 15];
                if ((l & 15) == 14) fr = rng.next() & kFrac;
                if ((l & 15) == 15) fr = ((rng.next() & kFrac) & ~0x1fffu) | 0x1000u;
                iv[1 * 32 + l] = mk(l >= 16, e, fr);
                u32 fr2 = pats[(l + 5) & 15];
                if (((l + 5) & 15) >= 14) fr2 = rng.next() & kFrac;
                iv[3 * 32 + l] = mk(l < 16, e, fr2);
              }
              u32 mode = noise();
              i64 rmop = 0;
              if (dotm) {
                rmop = rmi < 5 ? static_cast<i64>(rmi) : 7;
                mode |= rand_rm16(rng, true) << 3;   // only read by rm = dyn
              } else {
                mode |= rmi << 3;
              }
              mode &= ~((1u << 9) | (1u << 12));
              if (var == 1) mode |= 1u << 12;        // FTZ16
              if (var == 2) mode |= 1u << 9;         // DAZ32
              mode |= rng.below(2) << 11;            // DAZ16: does not apply to f32 sources
              if (!sweep_one(nm, rmop, iv, mode)) return 1;
            }
          }
        }
      }
    }
    if (f16_sweep & 4) {
      // binary32 -> binary16, flag isolation: fflags of a vector are the OR over all lanes, so the mixed-lane sweep
      // above cannot see a single wrong UF / NX.  Here every lane of a vector holds the SAME value (both sources),
      // aimed at the tininess-after-rounding edge (exponent 112 carrying into the normal range) and at exact
      // binary16 subnormals under FTZ16 (UF|NX is the only inexactness), plus exact normals (no flags).
      struct Cs { int e; u32 fr; };
      std::vector<Cs> cs;
      for (int e = 100; e <= 115; ++e) cs.push_back({e, 0u});
      for (u32 fr : {0x7fe000u, 0x7ff000u, 0x7fefffu, 0x7fffffu, 0x7fd000u, 0x7fcfffu})
        for (int e : {111, 112, 113}) cs.push_back({e, fr});
      cs.push_back({120, 0u});
      cs.push_back({127, 0x400000u});
      for (const char* nm : {"fcvt.f16.f32.lo", "fcvt.f16.f32.hi", "pkcvt.f16.f32", "fcvt.f16.f32.lo.m",
                             "fcvt.f16.f32.hi.m", "pkcvt.f16.f32.m"}) {
        const bool dotm = std::strlen(nm) > 2 && std::strcmp(nm + std::strlen(nm) - 2, ".m") == 0;
        for (unsigned rmi = 0; rmi < (dotm ? 6u : 5u); ++rmi)
          for (const Cs& c : cs)
            for (int var = 0; var < 3; ++var)
              for (int neg = 0; neg < 2; ++neg) {
                std::vector<u32> iv;
                fill(iv);
                for (unsigned l = 0; l < 32; ++l) {
                  iv[1 * 32 + l] = mk(neg != 0, c.e, c.fr);
                  iv[3 * 32 + l] = mk(neg != 0, c.e, c.fr);
                }
                u32 mode = noise();
                i64 rmop = 0;
                if (dotm) {
                  rmop = rmi < 5 ? static_cast<i64>(rmi) : 7;
                  mode |= rand_rm16(rng, true) << 3;
                } else {
                  mode |= rmi << 3;
                }
                mode &= ~((1u << 9) | (1u << 12));
                if (var == 1) mode |= 1u << 12;        // FTZ16
                if (var == 2) mode |= 1u << 9;         // DAZ32
                mode |= rng.below(2) << 11;
                if (!sweep_one(nm, rmop, iv, mode)) return 1;
              }
      }
    }
  }

  std::fclose(f);
  std::printf("dump_valu_fp: vectors=%d traps=%d (fp exception traps=%d) encode_skipped=%d out=%s\n", emitted, traps,
              fptraps, skipped, out_path.c_str());
  return 0;
}
