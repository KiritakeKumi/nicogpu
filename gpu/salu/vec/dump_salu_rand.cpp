// dump_salu_rand.cpp -- randomised-operand vector exporter for the scalar/uniform
// execution unit ng_salu: SALU + branch + sync/misc/CSR instructions.
//
// Same model path as the stock exporter (tools/vecexport) and sim/valu/dump_rand.cpp
// (model Core with the formal ISA table, VMID bypass), but
//   * per instruction word of the input list, N variants with re-drawn operands
//     (aliasing, misaligned pairs, corner-biased uniform registers, EXEC, CSR numbers,
//     immediates; one variant in 12 flips a single encoding bit);
//   * randomised per-warp state: all 64 uniform registers, EXEC, the CSR block
//     (mode fflags trapctl trapbase trapcause trappc trapinfo), scratch/code launch
//     parameters, and the PC of the instruction (code offset);
//   * the instruction sits in a window of `endpgm` filler words, so a taken branch
//     lands on an endpgm and the model's NEXT PC is observable: next_pc is taken from
//     the second committed instruction, or (target outside the window) from the trap
//     PC of the failed fetch.  Branch-class vectors compare it with br_taken/br_target.
//   * the final EXEC and CSR block are exported (expected.final_*), because csr*,
//     exec.*/saveexec.* and trap latching change state other than u-registers.
// Output: JSON lines, format nicogpu-vec/0.1 arch_insn plus the "ext" / extra "expected"
// keys read by rtl/gpu/salu/vec/vec2txt_salu.py.
//
// usage: dump_salu_rand --words words.txt --out vectors.jsonl [--per N] [--seed S]
//   words.txt: one instruction per line  "<name> <w0-hex> <w1-hex>"
// Deterministic for a given (words, per, seed).
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "sem_harness.h"
#include "nicogpu_isa.h"

using namespace nicogpu;

namespace {

struct Rng {
  uint64_t s;
  explicit Rng(uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x7654321ull) {}
  uint32_t next() {  // xorshift64*
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return static_cast<uint32_t>((s * 0x2545F4914F6CDD1Dull) >> 32);
  }
  uint32_t below(uint32_t n) { return next() % n; }
};

u32 rand_word(Rng& r) {
  static const u32 corner[] = {0u, 1u, 0xffffffffu, 0x80000000u, 0x7fffffffu, 0xffffu, 0xffu, 0x80u, 0x7fu,
                               31u, 32u, 33u, 63u, 64u, 0x80000001u, 0xfffffffeu, 0x0d0c0703u, 0x00ff00ffu,
                               0x001f0003u, 0x00200005u, 0x00210000u, 0x003f001fu, 0x00010000u, 0x55555555u,
                               0xaaaaaaaau, 0x0000ffffu, 0xffff0000u, 0x00000100u};
  switch (r.below(8)) {
    case 0: case 1: case 2: return corner[r.below(sizeof corner / sizeof corner[0])];
    case 3: return r.below(70);
    case 4: return 0u - r.below(70);
    default: return r.next();
  }
}
u32 rand_exec(Rng& r) {
  switch (r.below(9)) {
    case 0: return 0x55555555u;
    case 1: return 0xffffffffu;
    case 2: return 0u;
    case 3: return 1u << r.below(32);
    case 4: return 0xffff0000u;
    case 5: return 0x0000ffffu;
    case 6: return 0xaaaaaaaau;
    default: return r.next();
  }
}
u32 rand_mode(Rng& r) {  // legal MODE values only (csr.h mode:: layout)
  u32 m = 0;
  if (r.below(3) == 0) return 0;
  m |= r.below(5);               // rm32 0..4
  m |= (r.below(2) ? 7u : r.below(5)) << 3;  // rm16
  m |= (r.below(2) ? 7u : r.below(5)) << 6;  // rm64
  m |= (r.next() & 0x3fu) << 9;  // daz/ftz flags
  m |= (r.next() & 0x1fu) << 16; // trap enables
  return m;
}
const u32 kCsrs[] = {0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x22, 0x23,
                     0x30, 0x31, 0x32, 0x33, 0x34, 0x40, 0x41, 0x42, 0x43};

std::string hex32(u32 v) {
  char b[16];
  std::snprintf(b, sizeof b, "0x%08x", v);
  return b;
}
std::string hex64(u64 v) {
  char b[24];
  std::snprintf(b, sizeof b, "0x%016llx", static_cast<unsigned long long>(v));
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

constexpr u64 kWinBase = semtest::kCodeBase;  // physical window base
constexpr unsigned kWinBytes = 0x400;
constexpr unsigned kTestOff = 0x200;           // instruction under test inside the window

struct Sink final : sp::TraceSink {
  std::vector<u64> pcs;
  sp::Warp final_w;
  bool have = false, trapped = false;
  u64 trap_pc = 0;
  void on_commit(const sp::CommitRecord& rec) override {
    pcs.push_back(rec.pc);
    if (rec.warp->state == sp::WarpState::Done) { final_w = *rec.warp; have = true; }
  }
  void on_trap(const sp::Warp& w, const std::string&, const isa::DecodedInsn*) override {
    final_w = w; have = true; trapped = true; trap_pc = w.csr.trappc;
  }
};

struct Init {
  u32 exec;
  std::vector<u32> u;
  sp::WarpCsrs csr;
};

}  // namespace

int main(int argc, char** argv) {
  std::string words_path, out_path;
  unsigned per = 120;
  uint64_t seed = 1;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) { std::fprintf(stderr, "missing value for %s\n", a.c_str()); std::exit(2); }
      return argv[++i];
    };
    if (a == "--words") words_path = next();
    else if (a == "--out") out_path = next();
    else if (a == "--per") per = static_cast<unsigned>(std::atoi(next().c_str()));
    else if (a == "--seed") seed = static_cast<uint64_t>(std::atoll(next().c_str()));
    else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
  }
  if (words_path.empty() || out_path.empty()) {
    std::fprintf(stderr, "usage: dump_salu_rand --words words.txt --out vectors.jsonl [--per N] [--seed S]\n");
    return 2;
  }
  const isa::IsaTable& t = nicogpu_isa_table();
  const int endpgm = t.find("endpgm");
  u64 endpgm_word = 0;
  unsigned endpgm_len = 0;
  std::string err;
  if (endpgm < 0 || !isa::encode(t, endpgm, nullptr, 0, &endpgm_word, &endpgm_len, &err) || endpgm_len != 4) {
    std::fprintf(stderr, "endpgm: %s\n", err.c_str());
    return 2;
  }

  std::ifstream wf(words_path);
  if (!wf) { std::fprintf(stderr, "cannot open %s\n", words_path.c_str()); return 2; }
  std::set<std::string> known;
  {
    std::ifstream kf(words_path);
    std::string kl;
    while (std::getline(kf, kl)) {
      std::istringstream ks(kl);
      std::string kn;
      if (ks >> kn) known.insert(kn);
    }
  }
  std::FILE* f = std::fopen(out_path.c_str(), "wb");
  if (!f) { std::fprintf(stderr, "cannot write %s\n", out_path.c_str()); return 2; }
  std::fprintf(f, "{\"kind\":\"file_header\",\"format\":\"nicogpu-vec/0.1\",\"owner_unit\":\"salu\",\"isa\":\"%s\","
                  "\"generator\":\"nicogpu_salu_rand\"}\n", t.isa_version);

  int emitted = 0, traps = 0, skipped = 0;
  std::string line;
  unsigned insn_no = 0;
  while (std::getline(wf, line)) {
    if (line.empty()) continue;
    std::istringstream ls(line);
    std::string name, sw0, sw1;
    ls >> name >> sw0 >> sw1;
    const u64 w0 = std::strtoul(sw0.c_str(), nullptr, 16);
    const u64 w1 = sw1.empty() ? 0 : std::strtoul(sw1.c_str(), nullptr, 16);
    const unsigned len = (w0 & 3u) == 1u ? 8 : 4;
    isa::DecodedInsn base;
    if (!isa::decode(t, len == 8 ? ((w1 << 32) | w0) : w0, len, &base)) {
      std::fprintf(stderr, "%s: word does not decode\n", name.c_str());
      return 2;
    }
    const isa::InsnDesc& d = t.insns[base.index];
    Rng rng(seed * 1000003ull + insn_no);
    ++insn_no;

    for (unsigned k = 0; k < per; ++k) {
      // ---- PC / code base
      u32 pc_off;
      if (rng.below(10) < 7) pc_off = 4 * rng.below(0x800);
      else pc_off = rng.next() & ~3u;
      const u64 code_va = kWinBase + kTestOff;
      const u64 code_base = code_va - pc_off;

      // --
      i64 ops[isa::kMaxOperands];
      int rs1_idx = -1, rd_idx = -1;
      std::vector<unsigned> pair_ops;
      for (unsigned oi = 0; oi < d.nops; ++oi) {
        const char* kind = d.ops[oi].kind;
        const char* field = d.ops[oi].field;
        i64 v = base.op[oi];
        if (std::strcmp(field, "rs1") == 0) rs1_idx = static_cast<int>(oi);
        if (std::strcmp(field, "rd") == 0) rd_idx = static_cast<int>(oi);
        if (std::strcmp(kind, "ureg") == 0) {
          v = rng.below(5) == 0 ? rng.below(64) : rng.below(16);
        } else if (std::strcmp(kind, "upair") == 0) {
          v = 2 * (rng.below(5) == 0 ? rng.below(32) : rng.below(8));
          pair_ops.push_back(oi);
        } else if (std::strcmp(kind, "simm10") == 0) {
          v = static_cast<i64>(rng.below(1024)) - 512;
          if (rng.below(5) == 0) v = (rng.below(2) ? -512 : 511);
        } else if (std::strcmp(kind, "simm17") == 0) {
          v = static_cast<i64>(rng.below(1u << 17)) - (1 << 16);
        } else if (std::strncmp(kind, "br", 2) == 0) {
          const unsigned bits = d.ops[oi].bits;
          if (rng.below(8) < 6) {  // small, inside the window, never 0
            do { v = 4 * (static_cast<i64>(rng.below(0xf0)) - 0x78); } while (v == 0);
          } else if (rng.below(3) == 0) {
            do { v = 4 * (static_cast<i64>(rng.below(1u << bits)) - (1ll << (bits - 1))); } while (v == 0);
          } else {
            do { v = 4 * (static_cast<i64>(rng.below(0x100)) - 0x80); } while (v == 0);
          }
        } else if (std::strcmp(kind, "jofs10") == 0) {
          v = 4 * (static_cast<i64>(rng.below(1024)) - 512);
        } else if (std::strcmp(kind, "imm32") == 0) {
          v = static_cast<i64>(rand_word(rng));
        } else if (std::strcmp(kind, "uimm20") == 0) {
          v = rng.below(3) == 0 ? rng.below(8) : (rng.next() & 0xfffff);
        } else if (std::strcmp(kind, "uimm6") == 0) {
          v = rng.below(64);
        } else if (std::strcmp(kind, "csr") == 0) {
          v = rng.below(8) == 0 ? rng.below(1024) : kCsrs[rng.below(sizeof kCsrs / sizeof kCsrs[0])];
        } else if (std::strncmp(kind, "cnt_", 4) == 0) {
          v = rng.next() & ((1u << d.ops[oi].bits) - 1);
        }
        ops[oi] = v;
      }
      // aliasing: rd == rs1 now and then
      if (rd_idx >= 0 && rs1_idx >= 0 && rng.below(6) == 0) ops[rd_idx] = ops[rs1_idx];

      u64 word = 0;
      unsigned elen = 0;
      if (!isa::encode(t, base.index, ops, d.nops, &word, &elen, &err)) { ++skipped; continue; }
      // deliberate odd pair registers (IllegalInsn)
      if (!pair_ops.empty() && rng.below(10) == 0) {
        const unsigned oi = pair_ops[rng.below(static_cast<uint32_t>(pair_ops.size()))];
        word ^= u64{1} << d.ops[oi].ranges[d.ops[oi].nranges - 1].lsb;
      }
      // one variant in 12 flips a single encoding bit (bits >= 2 keep the length class)
      std::string vname = d.name;
      unsigned vwidth = d.width_bits;
      std::string vunit = "branch";
      bool is_br = d.unit == isa::Unit::Branch;
      if (rng.below(12) == 0) {
        word ^= u64{1} << (2 + rng.below(elen * 8 - 2));
      }
      std::vector<u32> words{static_cast<u32>(word)};
      if (elen == 8) words.push_back(static_cast<u32>(word >> 32));
      isa::DecodedInsn dec;
      bool decoded = isa::decode(t, word, elen, &dec);
      std::string disasm = "illegal";
      if (decoded) {
        disasm = isa::disassemble(dec);
        const isa::InsnDesc& md = t.insns[dec.index];
        vname = md.name;
        vwidth = md.width_bits;
        is_br = md.unit == isa::Unit::Branch;
        if (!known.count(vname)) { ++skipped; continue; }
      } else {
        vname = "illegal";
        is_br = false;
      }

      // --
      Init in;
      in.u.resize(64);
      for (auto& x : in.u) x = rand_word(rng);
      in.exec = rand_exec(rng);
      in.csr.mode = rand_mode(rng);
      in.csr.fflags = rng.next() & 0x1f;
      in.csr.trapctl = rng.below(2) ? rng.next() : 0;
      in.csr.trapbase = rng.below(2) ? rng.next() : 0;
      in.csr.trapcause = rng.below(2) ? rng.below(12) : 0;
      in.csr.trappc = rng.below(2) ? rng.next() & ~3u : 0;
      in.csr.trapinfo = rng.below(2) ? rng.next() : 0;
      const u64 scratch_va = rng.below(2) ? (static_cast<u64>(rng.next()) << 32 | rng.next()) & ~0xfffull : 0;
      const u32 scratch_bytes = rng.below(2) ? (rng.next() & 0xffff0) : 0;

      // branch targets: steer jr/jalr into the window most of the time, never onto the instruction itself
      u32 target_off = 0;
      bool br_indirect = false;
      if (decoded && is_br) {
        const std::string mn = vname;
        if (mn == "jr" || mn == "jalr") {
          br_indirect = true;
          const int ri = mn == "jr" ? 0 : 1;
          const i64 imm = dec.op[ri + 1];
          const u32 rs1 = static_cast<u32>(dec.op[ri]);
          if (rng.below(10) < 8) {
            u32 delta = 4 * rng.below(0x100) - 0x200 + 4;
            target_off = pc_off + delta;
          } else {
            target_off = rng.next() & ~3u;
          }
          if (target_off == pc_off) target_off += 4;
          in.u[rs1] = target_off - static_cast<u32>(imm) + rng.below(4);
          if (mn == "jalr" && static_cast<u32>(dec.op[0]) == rs1) {
            // rd == rs1: the value is read before the link is written; nothing to adjust
          }
        } else {
          const i64 off = dec.op[mn == "jmp" || mn == "bexecz" || mn == "bexecnz" ? 0 : (mn == "jal" ? 1 : 1)];
          if (off == 0) { ++skipped; continue; }
        }
      }
      (void)br_indirect;

      // ---- run the model
      mem::PhysMem mem;
      mmu::Mmu mmu(mem);
      std::vector<u32> code(kWinBytes / 4, static_cast<u32>(endpgm_word));
      for (size_t i = 0; i < words.size(); ++i) code[kTestOff / 4 + i] = words[i];
      if (mem.write(kWinBase, code.data(), code.size() * 4) != Fault::None) { std::fprintf(stderr, "mem write\n"); return 2; }
      std::vector<std::unique_ptr<sp::Core>> cores;
      sp::CoreConfig cfg;
      cores.push_back(std::make_unique<sp::Core>(0, t, mmu, cfg));
      Sink sink;
      cores[0]->set_trace(&sink);
      cores[0]->set_warp_init([&](sp::Warp& w) {
        w.exec = in.exec;
        for (unsigned reg = 0; reg < 64; ++reg) w.u[reg] = in.u[reg];
        w.csr = in.csr;
      });
      sp::DispatchInfo di;
      di.vmid = mmu::kVmidBypass;
      di.code_base = code_base;
      di.code_va = code_va;
      di.grid[0] = 1; di.wg[0] = 32; di.wg[1] = 1; di.wg[2] = 1;
      di.num_vregs = 128;
      di.scratch_va = scratch_va;
      di.scratch_bytes = scratch_bytes;
      const sp::DispatchStatus st = sp::run_dispatch(cores, di);
      if (!sink.have) { std::fprintf(stderr, "%s: no final state\n", name.c_str()); return 1; }
      const sp::Warp& w = sink.final_w;

      // test instruction trapped itself <=> it never committed
      const bool test_trapped = sink.pcs.empty();
      bool fake_trap = false;  // trap of the NEXT fetch (not an effect of the instruction)
      u32 next_pc = 0;
      if (!test_trapped) {
        if (sink.pcs.size() >= 2) next_pc = static_cast<u32>(sink.pcs[1] - code_base);
        else if (sink.trapped) { next_pc = static_cast<u32>(sink.trap_pc); fake_trap = true; }
        else next_pc = pc_off + elen;  // endpgm under test
      }
      sp::WarpCsrs fin = w.csr;
      if (fake_trap) { fin.trapcause = in.csr.trapcause; fin.trappc = in.csr.trappc; fin.trapinfo = in.csr.trapinfo; }
      const bool expect_trap = test_trapped;

      // --
      std::fprintf(f, "{\"kind\":\"arch_insn\",\"id\":\"rnd-%04u-%03u\",\"owner_unit\":\"salu\",", insn_no - 1, k);
      std::fprintf(f, "\"insn\":{\"name\":\"%s\",\"disasm\":\"%s\",\"width_bits\":%u,\"unit_table\":\"%s\",\"words\":[",
                   vname.c_str(), esc(disasm).c_str(), vwidth, is_br ? "branch" : "salu");
      for (size_t i = 0; i < words.size(); ++i) std::fprintf(f, "%s\"%s\"", i ? "," : "", hex32(words[i]).c_str());
      std::fprintf(f, "]},\"input\":{\"pc\":%u,\"exec\":\"%s\",\"uregs\":{", pc_off, hex32(in.exec).c_str());
      bool first = true;
      for (unsigned reg = 0; reg < 64; ++reg) {
        if (in.u[reg] == 0) continue;
        std::fprintf(f, "%s\"%u\":\"%s\"", first ? "" : ",", reg, hex32(in.u[reg]).c_str());
        first = false;
      }
      std::fprintf(f, "},\"csrs\":{\"mode\":\"%s\",\"fflags\":\"%s\",\"trapctl\":\"%s\",\"trapbase\":\"%s\","
                      "\"trapcause\":\"%s\",\"trappc\":\"%s\",\"trapinfo\":\"%s\"}},",
                   hex32(in.csr.mode).c_str(), hex32(in.csr.fflags).c_str(), hex32(in.csr.trapctl).c_str(),
                   hex32(in.csr.trapbase).c_str(), hex32(in.csr.trapcause).c_str(), hex32(in.csr.trappc).c_str(),
                   hex32(in.csr.trapinfo).c_str());
      std::fprintf(f, "\"ext\":{\"slot\":%u,\"sp\":%u,\"code_base\":\"%s\",\"scratch_va\":\"%s\",\"scratch_bytes\":\"%s\"},",
                   w.slot, w.sp, hex64(code_base).c_str(), hex64(scratch_va).c_str(), hex32(scratch_bytes).c_str());
      std::fprintf(f, "\"expected\":{\"trap\":");
      if (expect_trap) {
        ++traps;
        std::fprintf(f, "{\"cause\":%u,\"pc\":%u,\"info\":\"%s\",\"message\":\"%s\"},\"writes\":{}", w.csr.trapcause,
                     w.csr.trappc, hex32(w.csr.trapinfo).c_str(), esc(st.error).c_str());
      } else {
        std::fprintf(f, "null,\"writes\":{\"uregs\":{");
        first = true;
        for (unsigned reg = 0; reg < 64; ++reg) {
          if (w.u[reg] == in.u[reg]) continue;
          std::fprintf(f, "%s\"%u\":\"%s\"", first ? "" : ",", reg, hex32(w.u[reg]).c_str());
          first = false;
        }
        std::fprintf(f, "}}");
      }
      std::fprintf(f, ",\"final_exec\":\"%s\",\"final_csrs\":{\"mode\":\"%s\",\"fflags\":\"%s\",\"trapctl\":\"%s\","
                      "\"trapbase\":\"%s\",\"trapcause\":\"%s\",\"trappc\":\"%s\",\"trapinfo\":\"%s\"},",
                   hex32(w.exec).c_str(), hex32(fin.mode).c_str(), hex32(fin.fflags).c_str(), hex32(fin.trapctl).c_str(),
                   hex32(fin.trapbase).c_str(), hex32(fin.trapcause).c_str(), hex32(fin.trappc).c_str(),
                   hex32(fin.trapinfo).c_str());
      std::fprintf(f, "\"next_pc\":%s}}\n", (is_br && !expect_trap) ? std::to_string(next_pc).c_str() : "null");
      ++emitted;
    }
  }
  std::fclose(f);
  std::printf("dump_salu_rand: vectors=%d traps=%d encode_skipped=%d out=%s\n", emitted, traps, skipped,
              out_path.c_str());
  return 0;
}
