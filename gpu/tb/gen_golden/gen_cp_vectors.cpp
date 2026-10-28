// golden vector generator for the CP RTL testbench (rtl/gpu/tb/tb_cp.sv).
//
// Runs directed scenarios and random command streams on the functional model's
// cp::Cp (the reference v0.2) and records, as a flat word stream,
//   * every host-visible event the testbench must replay on the RTL
//     (BAR0 register writes, doorbells, host memory pokes), and
//   * after every event group, what the RTL must show once it has settled:
//     CP register block reads, IRQ_STATUS (MMU_FAULT bit masked: the MMU is
//     not part of the CP block), the functional timestamp, and FNV-1a digests
//     of the memory the command streams can write.
// The stream format (6 words per event) is documented in tb_cp.sv.  Scenarios
// and random streams are the differential test's, minus everything that
// executes a shader (the RTL testbench has no SP; DISPATCH is covered by a
// directed work-distributor protocol test in tb_cp.sv instead).
//
// Usage: gen_cp_vectors <out.hex> [--seeds N] [--seed0 S] [--only scenario]
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "test_util.h"
#include "nicogpu/device.h"
#include "nicogpu/pt_builder.h"
#include "nicogpu_isa_sample.h"

using namespace nicogpu;

namespace {

const isa::IsaTable& T = nicogpu_isa_sample_table();

u32 lo32(u64 v) { return static_cast<u32>(v); }
u32 hi32(u64 v) { return static_cast<u32>(v >> 32); }

// Memory layout shared by every case (VMID 1 maps kVa..kVa+16 MiB to VRAM).
constexpr unsigned kVmid = 1;
constexpr u64 kVa = 0x1'0000'0000ull;
constexpr u64 kPa = 0x1000'0000ull;
constexpr u64 kWinSize = 16 * MiB;
constexpr u32 kRingLog2 = 10;
constexpr u32 kRingDw = 1u << kRingLog2;

constexpr u64 kIb = kVa + 0x1000;      // command buffers
constexpr u64 kIb2 = kVa + 0x2000;
constexpr u64 kCode = kVa + 0x10000;
constexpr u64 kData = kVa + 0x20000;   // 0x1000 bytes of fixed pseudo-random words
constexpr u64 kFlag = kVa + 0x21000;   // known words for WAIT_MEM / predicate
constexpr u64 kArgs = kVa + 0x22000;
constexpr u64 kDst = kVa + 0x28000;    // scribble area
constexpr u64 kOut = kVa + 0x30000;
constexpr u64 kBig = kVa + 0x40000;    // 64 KiB source for the long COPY cases
constexpr u64 kBigDst = kVa + 0x60000;
constexpr u64 kUnmapped = kVa + kWinSize + 0x10000;

constexpr u32 kFlagWord0 = 0x12345678u;

// ---- event stream ---------------------------------------------------------------
enum : u32 {
  E_END = 0,
  E_HW = 1,      // host BAR0 write:           off, val
  E_HR = 2,      // host BAR0 read + compare:  off, expect, mask
  E_MEMW = 3,    // host memory poke:          pa_lo, pa_hi, val
  E_CHK = 4,     // memory digest compare:     pa_lo, pa_hi, nwords, hash
  E_SETTLE = 5,  // wait until the CP is quiet
  E_RESET = 6,   // reset DUT + memories (start of a scenario)
  E_NOTE = 7,    // scenario id (printed)
  E_HRNZ = 8,    // host BAR0 read, (value != 0) must equal `expect`
};

std::vector<u32> g_stream;
unsigned g_events = 0;

void emit(u32 op, u32 a = 0, u32 b = 0, u32 c = 0, u32 d = 0, u32 e = 0) {
  const u32 w[6] = {op, a, b, c, d, e};
  g_stream.insert(g_stream.end(), w, w + 6);
  ++g_events;
}

// ---------------------------------------------------------------------------
struct Rng {
  u64 s;
  explicit Rng(u64 seed) : s(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) {
    if (!s) s = 1;
    for (int i = 0; i < 4; ++i) next();
  }
  u64 next() {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return s * 0x2545F4914F6CDD1Dull;
  }
  u32 below(u32 n) { return static_cast<u32>((next() >> 33) % n); }
  bool chance(u32 pct) { return below(100) < pct; }
};

u32 fnv_words(const std::vector<u32>& w) {
  u32 h = 2166136261u;
  for (u32 x : w) h = (h ^ x) * 16777619u;
  return h;
}

// ---------------------------------------------------------------------------
// The model side of the rig; mirrors every host-visible action into the stream.
struct Side {
  Device dev;
  u64 next_sys = mem::kSysBase;
  u64 ring = 0, wb_slot = 0;
  std::unique_ptr<mmu::PageTableBuilder> pt;

  Side() : dev(T) {
    ring = sys_page(4u << kRingLog2);
    wb_slot = sys_page(4096);
    pt = std::make_unique<mmu::PageTableBuilder>(dev.phys(), [this] { return sys_page(4096); }, [](u64) {});
    REQUIRE(pt->set(kVa, kWinSize, mmu::PageTableBuilder::Kind::Map, kPa, mmu::PTE_R | mmu::PTE_W | mmu::PTE_X));
    const u32 ctx = regs::MMU_CTX_BASE + regs::MMU_CTX_STRIDE * kVmid;
    dev.mmio_write32(ctx + regs::MMU_CTX_ROOT_LO, lo32(pt->root()));
    dev.mmio_write32(ctx + regs::MMU_CTX_ROOT_HI, hi32(pt->root()));
    dev.mmio_write32(ctx + regs::MMU_CTX_CTRL, 1);
    dev.mmio_write32(regs::IRQ_ENABLE, 0x1f);
    emit(E_HW, regs::IRQ_ENABLE, 0x1f);
  }
  u64 sys_page(u64 bytes) {
    const u64 pa = next_sys;
    next_sys += (bytes + 4095) & ~4095ull;
    return pa;
  }
  u32 r(u32 off) { return dev.mmio_read32(off); }
  void w(u32 off, u32 v) {
    dev.mmio_write32(off, v);
    emit(E_HW, off, v);
  }
  void doorbell(u32 wptr) {
    dev.mmio_write32(regs::DOORBELL0, wptr);
    emit(E_HW, regs::DOORBELL0, wptr);
  }
  void poke_pa(u64 pa, u32 v) {
    dev.phys().write32(pa, v);
    emit(E_MEMW, lo32(pa), hi32(pa), v);
  }
  void put(u64 va, const std::vector<u32>& dws) {
    for (size_t i = 0; i < dws.size(); ++i) poke_pa(kPa + (va - kVa) + 4 * i, dws[i]);
  }
  void ring_write(u32 index, u32 value) { poke_pa(ring + 4ull * (index & (kRingDw - 1)), value); }
  u32 digest(u64 pa, u32 nwords) {
    std::vector<u32> w(nwords);
    dev.phys().read(pa, w.data(), 4ull * nwords);
    return fnv_words(w);
  }
};

// Scenario-level coverage tally.
struct Coverage {
  u64 checks = 0, err_seen[16] = {}, wraps = 0, waits = 0, ticks = 0;
} g_cov;

// ---------------------------------------------------------------------------
struct Pair {
  Side m;
  std::string label;
  u32 wptr = 0;
  u64 inflight_ib = 0;
  bool had_err = false;
  std::vector<std::pair<u64, u32>> extra;  // additional VA windows (words) digested at every check

  explicit Pair(const std::string& l) : label(l) {
    emit(E_RESET);
    Rng rd(0xda7a);
    std::vector<u32> data(1024);
    for (auto& d : data) d = 0xA5A50000u | rd.below(0x10000);
    const std::vector<u32> flags = {kFlagWord0, 0x9abcdef0u, 0, 0xffffffffu, 7, 0};
    m.put(kData, data);
    m.put(kFlag, flags);
    m.w(regs::CP_RING_BASE_LO, lo32(m.ring));
    m.w(regs::CP_RING_BASE_HI, hi32(m.ring));
    m.w(regs::CP_RING_SIZE_LOG2, kRingLog2);
    check("after ring setup");
  }

  void both_w(u32 off, u32 v, const char* tag) {
    m.w(off, v);
    check(tag);
  }
  void boot(bool enable = true) {
    both_w(regs::CP_CTRL, 0, "release CORE_RESET");
    if (enable) both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "ENABLE");
  }
  void enable_rptr_wb() {
    m.w(regs::CP_RPTR_WB_LO, lo32(m.wb_slot) | regs::CP_RPTR_WB_ENABLE);
    m.w(regs::CP_RPTR_WB_HI, hi32(m.wb_slot));
    check("RPTR write-back configured");
  }
  void recover() { both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE | regs::CP_CTRL_RESET, "RESET"); }
  // Host memory pokes are not observable events; the next doorbell/register
  // write is the check point (a blocked WAIT_MEM would otherwise complete in
  // the RTL, which polls, before the model, which re-polls on events only).
  void put(u64 va, const std::vector<u32>& dws, const char* = "host put") { m.put(va, dws); }
  void ring_put(const std::vector<u32>& dws, bool doorbell = true, size_t split = 0) {
    size_t i = 0;
    auto put_n = [&](size_t n) {
      for (; n && i < dws.size(); --n, ++i) {
        m.ring_write(wptr, dws[i]);
        ++wptr;
      }
    };
    if (split && split < dws.size()) {
      put_n(split);
      m.doorbell(wptr);
      check("ring half packet");
    }
    put_n(dws.size());
    if (doorbell) {
      m.doorbell(wptr);
      check("ring doorbell");
    }
  }
  void ring_doorbell(const char* tag = "doorbell") {
    m.doorbell(wptr);
    check(tag);
  }
  u32 reg(u32 off) { return m.r(off); }
  // 64 KiB of pseudo-random source data for the long COPY cases.
  void load_big() {
    Rng rd(0xb16);
    std::vector<u32> big(16384);
    for (auto& d : big) d = 0x5A000000u | rd.below(0x1000000);
    m.put(kBig, big);
  }
  void run_ib(u64 va, const std::vector<u32>& ib, const char* = "INDIRECT") {
    const bool waiting = (m.r(regs::CP_STATUS) & regs::CP_STATUS_WAITING) != 0;
    if (waiting && va == inflight_ib) va = (va == kIb) ? kIb2 : kIb;
    if (!waiting) inflight_ib = va;
    put(va, ib);
    ring_put(indirect(va, static_cast<u32>(ib.size())));
  }
  static std::vector<u32> indirect(u64 va, u32 size, u32 vmid = kVmid) {
    return {cp::header(cp::OP_INDIRECT, 4), lo32(va), hi32(va), size, vmid};
  }

  // What the RTL must show once settled.
  void check(const char*) {
    ++g_cov.checks;
    emit(E_SETTLE);
    static const u32 kRegs[] = {
        regs::CP_RING_BASE_LO, regs::CP_RING_BASE_HI, regs::CP_RING_SIZE_LOG2, regs::CP_RING_RPTR,
        regs::CP_RING_WPTR,    regs::CP_CTRL,         regs::CP_STATUS,         regs::CP_ERROR,
        regs::CP_ERROR_RPTR,   regs::CP_ERROR_INFO,   regs::CP_ERROR_IB_OFFSET, regs::CP_CMD_VERSION,
        regs::CP_FW_STATUS,    regs::CP_FW_BASE_LO,   regs::CP_FW_BASE_HI,     regs::CP_FW_SIZE,
        regs::CP_RPTR_WB_LO,   regs::CP_RPTR_WB_HI,
    };
    for (u32 off : kRegs) emit(E_HR, off, m.r(off), 0xffffffffu);
    // CP_FW_VERSION is implementation defined: only "reported once READY".
    emit(E_HRNZ, regs::CP_FW_VERSION, m.r(regs::CP_FW_VERSION) != 0);
    // IRQ_STATUS without MMU_FAULT (the MMU is outside the CP block).
    emit(E_HR, regs::IRQ_STATUS, m.r(regs::IRQ_STATUS), ~regs::IRQ_MMU_FAULT);
    emit(E_HR, regs::TIMESTAMP_LO, lo32(m.dev.cp().ticks()), 0xffffffffu);
    // Memory: scribble area through VMID 1 and as plain physical memory (ring
    // packets use physical addresses), the RPTR write-back slot.
    emit(E_CHK, lo32(kPa + (kDst - kVa)), hi32(kPa + (kDst - kVa)), 4096, m.digest(kPa + (kDst - kVa), 4096));
    emit(E_CHK, lo32(kDst), hi32(kDst), 4096, m.digest(kDst, 4096));
    for (const auto& e : extra) {
      emit(E_CHK, lo32(kPa + (e.first - kVa)), hi32(kPa + (e.first - kVa)), e.second,
           m.digest(kPa + (e.first - kVa), e.second));
    }
    emit(E_CHK, lo32(m.wb_slot), hi32(m.wb_slot), 1, m.digest(m.wb_slot, 1));
    const u32 errc = m.r(regs::CP_ERROR);
    if (errc && !had_err) ++g_cov.err_seen[errc & 15];
    had_err = errc != 0;
    if (m.r(regs::CP_STATUS) & regs::CP_STATUS_WAITING) ++g_cov.waits;
  }
};


// Packet builders.
using Pk = std::vector<u32>;
Pk operator+(Pk a, const Pk& b) {
  a.insert(a.end(), b.begin(), b.end());
  return a;
}
Pk nop(u32 n = 0) {
  Pk p = {cp::header(cp::OP_NOP, n)};
  p.resize(1 + n, 0xdeadbeef);
  return p;
}
Pk write_data(u64 va, std::vector<u32> data) {
  Pk p = {cp::header(cp::OP_WRITE_DATA, 2 + static_cast<u32>(data.size())), lo32(va), hi32(va)};
  return p + data;
}
Pk fence(u64 va, u64 v, u32 flags) {
  return {cp::header(cp::OP_FENCE, 5), lo32(va), hi32(va), lo32(v), hi32(v), flags};
}
Pk timestamp(u64 va) { return {cp::header(cp::OP_TIMESTAMP, 2), lo32(va), hi32(va)}; }
Pk copy(u64 src, u64 dst, u64 size, u32 mode = cp::COPY_MODE_LINEAR) {
  return {cp::header(cp::OP_COPY, 7), lo32(src), hi32(src), lo32(dst), hi32(dst), lo32(size), hi32(size), mode};
}
Pk fill(u64 dst, u64 size, u32 v, bool pred = false) {
  return {cp::header(cp::OP_FILL, 5, pred), lo32(dst), hi32(dst), lo32(size), hi32(size), v};
}
Pk cache(u32 flags) { return {cp::header(cp::OP_CACHE, 1), flags}; }
Pk set_shader(u64 code, u32 vregs, u32 wgx, u32 shared = 0) {
  return {cp::header(cp::OP_SET_SHADER, 11), lo32(code), hi32(code), 0, vregs, shared, 0, 0, 0, wgx, 1, 1};
}
Pk user_regs(u32 start, std::vector<u32> v) {
  Pk p = {cp::header(cp::OP_SET_USER_REGS, 1 + static_cast<u32>(v.size())), start};
  return p + v;
}
Pk dispatch(u32 gx, u32 gy = 1, u32 gz = 1, bool pred = false) {
  return {cp::header(cp::OP_DISPATCH, 3, pred), gx, gy, gz};
}
Pk dispatch_indirect(u64 args, bool pred = false) {
  return {cp::header(cp::OP_DISPATCH_INDIRECT, 2, pred), lo32(args), hi32(args)};
}
Pk wait_mem(u64 va, u64 ref, u64 mask, u32 func, bool b64) {
  return {cp::header(cp::OP_WAIT_MEM, 7), lo32(va), hi32(va), lo32(ref), hi32(ref), lo32(mask), hi32(mask),
          func | (b64 ? cp::WAIT_BIT64 : 0u)};
}
Pk query(u64 va, u32 kind) { return {cp::header(cp::OP_QUERY, 3), lo32(va), hi32(va), kind}; }
Pk set_predicate(u64 va, u32 ctrl) { return {cp::header(cp::OP_SET_PREDICATE, 3), lo32(va), hi32(va), ctrl}; }

// The common "compute prologue": shader + user regs (base = kOut, push u10).
Pk compute_prologue(u32 wgx = 8, u32 push = 1000) {
  return set_shader(kCode, 16, wgx) + user_regs(0, {lo32(kOut), hi32(kOut)}) + user_regs(10, {push});
}

// ---------------------------------------------------------------------------

// Directed scenarios.  Each builds a fresh Pair, so a failure names the case.
using Scenario = void (*)(Pair&);
struct Named {
  const char* name;
  Scenario fn;
};

void sc_handshake(Pair& p) {
  p.both_w(regs::CP_FW_BASE_LO, 0x1000, "FW_BASE_LO");
  p.both_w(regs::CP_FW_SIZE, 0x2000, "FW_SIZE");
  p.boot(false);
  CHECK(p.reg(regs::CP_FW_STATUS) & regs::CP_FW_STATUS_READY);
  CHECK(p.reg(regs::CP_STATUS) & regs::CP_STATUS_FW_READY);
  p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "ENABLE");
  p.ring_put(nop());
  // (CORE_RESET re-assertion is an RTL-only test in tb_cp.sv: the hardware
  // register block clears WPTR then, the model keeps it --
}

void sc_ring_packets(Pair& p) {
  p.boot();
  p.enable_rptr_wb();
  p.ring_put(nop(3) + nop());
  p.ring_put(fence(kDst, 0xcafe, 0));
  p.ring_put(fence(kDst + 8, 0x1122334455667788ull, cp::FENCE_FLAG_64BIT));
  p.ring_put(fence(kDst + 16, 7, cp::FENCE_FLAG_IRQ | cp::FENCE_FLAG_L2_WB));
  p.ring_put(timestamp(kDst + 24));
  p.ring_put(write_data(kDst + 32, {1, 2, 3, 4, 5}));
  p.ring_put(fill(kDst + 64, 64, 0xabababab));
  p.ring_put(copy(kData, kDst + 128, 256));
  p.ring_put(copy(kData + 3, kDst + 1024 + 1, 301));  // unaligned linear copy
  p.ring_put(cache(0x3f));
  p.ring_put(wait_mem(kFlag, kFlagWord0, 0xffffffff, cp::WAIT_EQ, false));
}

void sc_predicate(Pair& p) {
  p.boot();
  p.run_ib(kIb, compute_prologue() + set_predicate(kFlag, cp::PRED_ENABLE) +
                    fill(kDst, 4, 0x11, true) + set_predicate(kFlag + 8, cp::PRED_ENABLE) +
                    fill(kDst + 4, 4, 0x22, true) +
                    set_predicate(kFlag + 8, cp::PRED_ENABLE | cp::PRED_INVERT) + fill(kDst + 8, 4, 0x33, true) +
                    set_predicate(0, 0) + fill(kDst + 12, 4, 0x44, true));
}

void sc_wait_block(Pair& p) {
  p.boot();
  // Blocks (flag word 5 is 0, GE 5 false), the CP stays WAITING.
  p.run_ib(kIb, wait_mem(kFlag + 20, 5, 0xffffffff, cp::WAIT_GE, false) + fill(kDst, 4, 0x77));
  CHECK(p.reg(regs::CP_STATUS) & regs::CP_STATUS_WAITING);
  p.ring_doorbell("re-poll while blocked");
  p.put(kFlag + 20, {9}, "satisfy wait");
  p.ring_doorbell("re-poll after release");
  CHECK(!(p.reg(regs::CP_STATUS) & regs::CP_STATUS_WAITING));
  // 64-bit wait with a mask and each function.
  for (u32 fn = 0; fn <= cp::WAIT_LE; ++fn) {
    p.run_ib(kIb, wait_mem(kFlag, 0x12345600, 0xffffff00, fn, true) + fill(kDst + 16 + 4 * fn, 4, fn));
    if (p.reg(regs::CP_STATUS) & regs::CP_STATUS_WAITING) {
      // Initially v == ref: EQ/GE/LE pass, NE/GT/LT block.  Release by moving v.
      p.put(kFlag, {fn == cp::WAIT_LT ? 0x12345500u : 0x12345700u, 0}, "flip flag to release");
      p.ring_doorbell("release");
      p.put(kFlag, {kFlagWord0, 0x9abcdef0u}, "restore flag");
    }
  }
}

void sc_errors_ib(Pair& p) {
  // Every IB error family; recover with RESET between cases.
  const std::vector<Pk> bad_ibs = {
      Pk{0x000000ffu},                                                              // unknown opcode 0xff
      Pk{cp::header(cp::OP_NOP, 0) | (1u << 9)},                                    // reserved header bit
      Pk{cp::header(cp::OP_INDIRECT, 4), 0, 0, 0, 1},                               // INDIRECT in IB
      dispatch(1),                                                                  // no SET_SHADER
      dispatch_indirect(kArgs),                                                     // no SET_SHADER
      Pk{cp::header(cp::OP_FENCE, 4), 0, 0, 0, 0},                                  // COUNT too small
      Pk{cp::header(cp::OP_NOP, 0, true)},                                          // PRED on NOP
      Pk{cp::header(cp::OP_DRAW, 3), 0, 0, 0},                                      // reserved opcode, bad count
      Pk{cp::header(cp::OP_DRAW, 4), 0, 0, 0, 0},                                   // reserved opcode (UNSUPPORTED)
      Pk{cp::header(cp::OP_DRAW_INDIRECT, 6, true), 0, 0, 0, 0, 0, 0},
      copy(kData, kDst, 16, cp::COPY_MODE_LINEAR_TO_T64K),                          // tiled COPY
      copy(kData, kDst, 16, 3),                                                     // reserved mode
      copy(kData, kDst, 16, 0x10),                                                  // reserved flags
      fence(kDst + 4, 1, cp::FENCE_FLAG_64BIT),                                     // misaligned 64-bit
      fence(kDst, 1, cp::FENCE_FLAG_IRQ),                                           // IRQ flag not allowed in IB
      fence(kDst, 1, 0x80),                                                         // reserved flags
      timestamp(kDst + 4),                                                          // misaligned
      fence(kUnmapped, 1, 0),                                                       // MEM_FAULT
      write_data(kUnmapped, {1, 2}),
      write_data(kDst + 2, {1, 2}),
      fill(kDst, 6, 0), fill(kDst + 2, 8, 0), fill(kUnmapped, 16, 0),
      copy(kUnmapped, kDst, 16), copy(kData, kUnmapped, 16),
      cache(0x40), cache(0xffffffffu),
      wait_mem(kFlag, 0, 0, 6, false), wait_mem(kFlag + 4, 0, 0, 0, true),
      wait_mem(kFlag, 1, 0, cp::WAIT_EQ, false) + Pk{},                             // satisfied (mask 0 -> 0 vs 1: blocks)
      Pk{cp::header(cp::OP_WAIT_MEM, 7), lo32(kFlag), hi32(kFlag), 0, 1, 0, 0, 0},  // REF_HI set on 32-bit wait
      query(kDst, 2), query(kDst, 3), query(kDst, 0x10), query(kDst + 4, 0), query(kUnmapped, 1),
      set_predicate(kFlag, 4), set_predicate(kFlag + 2, cp::PRED_ENABLE), set_predicate(kFlag, 2),
      set_predicate(1, 0), set_predicate(kUnmapped, cp::PRED_ENABLE),
      set_shader(kCode + 1, 16, 8), set_shader(kCode, 0, 8), set_shader(kCode, 129, 8),
      set_shader(kCode, 16, 0), set_shader(kCode, 16, 2048), set_shader(kCode, 16, 8, 0x20000),
      set_shader(kCode, 256, 8),
      user_regs(0x20, {1}), user_regs(25, {1, 2}), user_regs(0, {}),
      compute_prologue() + dispatch_indirect(kArgs + 2), compute_prologue() + dispatch_indirect(kUnmapped),
      Pk{cp::header(cp::OP_NOP, 4), 1, 2},                                          // packet runs past IB end
      write_data(kDst, {1}) + Pk{cp::header(cp::OP_WRITE_DATA, 3), lo32(kDst)},      // truncated tail
  };
  p.boot();
  unsigned idx = 0;
  for (const Pk& ib : bad_ibs) {
    if (ib.empty()) continue;
    const std::string tag = "bad IB #" + std::to_string(idx++);
    p.label = tag;
    p.put(kIb, ib, "bad IB put");
    p.ring_put(Pair::indirect(kIb, static_cast<u32>(ib.size())));
    p.recover();
  }
}

void sc_errors_ring(Pair& p) {
  const std::vector<Pk> bad_ring = {
      Pk{0x000000ffu},
      Pk{cp::header(cp::OP_NOP, 0) | (1u << 12)},
      dispatch(1), dispatch_indirect(kArgs), set_shader(kCode, 16, 8), user_regs(0, {1}),
      set_predicate(0, 0), query(kDst, 0),
      Pair::indirect(kIb, 0, 0),                          // VMID 0
      Pair::indirect(kIb, 4, 16),                         // VMID reserved bits
      Pk{cp::header(cp::OP_INDIRECT, 4), 1, 0, 4, kVmid}, // misaligned IB VA
      Pk{cp::header(cp::OP_INDIRECT, 4), 0, 0x10000, 4, kVmid},  // VA beyond 48 bits
      Pair::indirect(kIb, (1u << 20) + 1),                // too large
      Pair::indirect(kUnmapped, 4),                       // fetch fault
      Pair::indirect(kIb, 8, 5),                          // VMID without a root: fetch fault
      Pk{cp::header(cp::OP_INDIRECT, 3), 0, 0, 0},
      fence(kUnmapped, 1, 0) , fence(kDst, 1, 0x100),
      Pk{cp::header(cp::OP_DRAW, 4), 0, 0, 0, 0},
      Pk{cp::header(cp::OP_FILL, 5, true), lo32(kDst), hi32(kDst), 4, 0, 1},  // PRED allowed on FILL
      Pk{cp::header(cp::OP_FENCE, 5, true), lo32(kDst), hi32(kDst), 1, 0, 0}, // PRED not allowed on FENCE
  };
  p.boot();
  // Nested INDIRECT: an IB that itself holds an INDIRECT.
  p.put(kIb2, {cp::header(cp::OP_NOP, 0)}, "nested target");
  p.put(kIb, Pair::indirect(kIb2, 1), "nested IB put");
  p.ring_put(Pair::indirect(kIb, 5));
  p.recover();
  unsigned idx = 0;
  for (const Pk& pk : bad_ring) {
    p.label = "bad ring #" + std::to_string(idx++);
    p.ring_put(pk);
    p.recover();
  }
}

void sc_ring_config(Pair& p) {
  // Ring size out of range, unaligned base, WPTR ahead by more than the ring.
  p.both_w(regs::CP_CTRL, 0, "boot");
  for (u32 lg : {0u, 9u, 21u, 31u}) {
    p.both_w(regs::CP_RING_SIZE_LOG2, lg, "bad size");
    p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "ENABLE (bad ring size)");
    p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE | regs::CP_CTRL_RESET, "RESET");
    p.both_w(regs::CP_CTRL, 0, "disable");
  }
  p.both_w(regs::CP_RING_SIZE_LOG2, kRingLog2, "restore size");
  p.both_w(regs::CP_RING_BASE_LO, lo32(p.m.ring) + 0x100, "misaligned base");
  p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "ENABLE (bad ring base)");
  p.recover();
  p.both_w(regs::CP_RING_BASE_LO, lo32(p.m.ring), "restore base");
  p.wptr = kRingDw + 8;  // beyond the ring size
  p.ring_doorbell("wptr too far ahead");
  p.recover();
}

void sc_halt_req(Pair& p) {
  p.boot();
  p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE | regs::CP_CTRL_HALT_REQ, "HALT_REQ");
  p.ring_put(fill(kDst, 16, 1) + fill(kDst + 16, 16, 2));
  CHECK(p.reg(regs::CP_STATUS) & regs::CP_STATUS_STOPPED);
  p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "resume");
  CHECK(!(p.reg(regs::CP_STATUS) & regs::CP_STATUS_STOPPED));
  // HALT_REQ only takes effect at command-buffer boundaries: a blocked IB
  // keeps running (and keeps waiting) until released.
  p.run_ib(kIb, wait_mem(kFlag + 20, 5, 0xffffffff, cp::WAIT_GE, false) + fill(kDst + 32, 4, 9));
  p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE | regs::CP_CTRL_HALT_REQ, "HALT_REQ during IB");
  p.put(kFlag + 20, {6}, "release");
  p.ring_doorbell("after release");
  p.ring_put(fill(kDst + 36, 4, 3));
  p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "resume 2");
}

void sc_ring_wrap(Pair& p) {
  p.boot();
  p.enable_rptr_wb();
  // 3000 dwords of packets through a 1024-dword ring, packets straddling the
  // wrap point, half-written packets, and an INDIRECT in the middle.
  u32 n = 0;
  for (int i = 0; i < 200; ++i) {
    Pk pk = write_data(kDst + 8 * (i % 100), {static_cast<u32>(i), static_cast<u32>(i * 3), 7, 8});
    p.ring_put(pk, true, (i % 3) == 0 ? 3 : 0);
    n += static_cast<u32>(pk.size());
    if (i % 17 == 5) p.run_ib(kIb, fill(kDst + 2048, 32, i) + timestamp(kDst + 2080));
  }
  CHECK(n > kRingDw);
}

void sc_ib_fetch_fault(Pair& p) {
  p.boot();
  // IB entirely unmapped: the implicit C-INV is issued before the fetch ,
  // so the cache counters are compared strictly.  (A mapping boundary inside the IB is a
  // documented streaming-vs-prefetch difference, and not run here.)
  p.ring_put(Pair::indirect(kUnmapped, 16));
  p.recover();
  p.ring_put(Pair::indirect(kUnmapped + 0x100000, 0));  // empty IB at an unmapped address: no fetch
  p.ring_put(Pair::indirect(kIb, 0));
}

void sc_mmu_fault_irq(Pair& p) {
  p.boot();
  p.both_w(regs::IRQ_STATUS, 0xffffffffu, "IRQ clear");
  p.run_ib(kIb, fill(kUnmapped + 0x40, 32, 1));
  p.recover();
  p.run_ib(kIb, write_data(kUnmapped, {1}));  // second fault: record stays on the first
}

// CP_RING_SIZE_LOG2 is stored full-width by the firmware: values
// >= 32 read back unchanged and are rejected like the model does.
void sc_ring_size_wide(Pair& p) {
  p.both_w(regs::CP_CTRL, 0, "boot");
  for (u32 lg : {32u, 33u, 42u}) {
    p.both_w(regs::CP_RING_SIZE_LOG2, lg, "wide size");
    p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "ENABLE (wide ring size)");
    p.recover();
    p.both_w(regs::CP_CTRL, 0, "disable");
  }
}

// ---- Copy engine scenarios (new in) --------------------------------------
u64 pa_of(u64 va) { return kPa + (va - kVa); }

void sc_copy_fill(Pair& p) {
  p.extra.push_back({kBigDst, 32768});
  p.load_big();
  p.boot();
  p.enable_rptr_wb();
  // Ring COPY: physical addresses, byte granular, every size class and
  // misalignment combination around the 8-byte lane and the 4 KiB granule.
  const u64 sizes[] = {0,  1,   2,   3,   4,    5,    7,    8,    9,    15,   16,   17,   31,
                       33, 63,  64,  65,  127,  255,  256,  257,  1000, 4095, 4096, 4097, 5000};
  u32 i = 0;
  for (u64 sz : sizes) {
    const u64 so = (i * 3) % 8, dof = (i * 5) % 8;
    p.ring_put(copy(pa_of(kBig) + so + 0x1000 * (i % 5), pa_of(kBigDst) + dof + 0x1800 * (i % 7), sz));
    ++i;
  }
  p.ring_put(copy(pa_of(kBig), pa_of(kBigDst) + 0x8000, 16384));   // multi-granule
  p.ring_put(copy(pa_of(kBig) + 5, pa_of(kBigDst) + 0xc003, 32749));
  // Ring FILL.
  const u64 fsizes[] = {0, 4, 8, 12, 16, 20, 64, 100, 1024, 4096, 4100};
  i = 0;
  for (u64 sz : fsizes) {
    p.ring_put(fill(pa_of(kBigDst) + 0xd000 + 4 * (i % 2) + 0x2000 * (i % 3), sz, 0xC0DE0000u + i));
    ++i;
  }
  // The same through an IB (VMID 1 virtual addresses) with predication.
  Pk ib = set_predicate(kFlag, cp::PRED_ENABLE);
  ib = ib + copy(kBig + 3, kBigDst + 0x11, 777) + copy(kBig + 0x1fff, kBigDst + 0x1ffd, 33) +
       fill(kBigDst + 0x3000, 256, 0x1234abcd, true) +
       set_predicate(kFlag + 8, cp::PRED_ENABLE) + fill(kBigDst + 0x3200, 64, 0xdead0000u, true) +
       set_predicate(0, 0) + fill(kBigDst + 0x3300, 64, 0xbeef0001u) + copy(kBig, kBigDst + 0x4000, 65536 - 64) +
       timestamp(kDst) + fence(kDst + 8, 0x77, 0);
  p.run_ib(kIb, ib);
}

void sc_copy_faults(Pair& p) {
  p.extra.push_back({kBigDst, 32768});
  p.load_big();
  p.boot();
  const std::vector<Pk> bad_ibs = {
      copy(kUnmapped, kDst, 16),          copy(kData, kUnmapped, 100),   copy(kUnmapped, kUnmapped + 64, 3),
      fill(kUnmapped, 64, 0xabcdu),       fill(kUnmapped + 0x8000, 4096, 1),
      copy(kUnmapped, kBigDst, 70000),    copy(kData, kUnmapped + 4, 1),
  };
  unsigned idx = 0;
  for (const Pk& ib : bad_ibs) {
    p.label = "bad copy IB #" + std::to_string(idx++);
    p.put(kIb, ib);
    p.ring_put(Pair::indirect(kIb, static_cast<u32>(ib.size())));
    p.recover();
  }
  // Ring: physical addresses outside every region (PA >= 16 GiB below SYS).
  const u64 kBadPa = 0x8'0000'0000ull;
  const std::vector<Pk> bad_ring = {
      copy(kBadPa, pa_of(kBigDst), 64), copy(pa_of(kBig), kBadPa, 64), fill(kBadPa, 128, 5),
      copy(pa_of(kBig), pa_of(kBigDst), 64),  // good one after recovery
  };
  idx = 0;
  for (const Pk& pk : bad_ring) {
    p.label = "bad copy ring #" + std::to_string(idx++);
    p.ring_put(pk);
    p.recover();
  }
}


const Named kScenarios[] = {
    {"ring_size_wide", sc_ring_size_wide},
    {"handshake", sc_handshake},
    {"ring_packets", sc_ring_packets},
    {"copy_fill", sc_copy_fill},
    {"copy_faults", sc_copy_faults},
    {"predicate", sc_predicate},
    {"wait_block", sc_wait_block},
    {"errors_ib", sc_errors_ib},
    {"errors_ring", sc_errors_ring},
    {"ring_config", sc_ring_config},
    {"halt_req", sc_halt_req},
    {"ring_wrap", sc_ring_wrap},
    {"ib_fetch_fault", sc_ib_fetch_fault},
    {"mmu_fault_irq", sc_mmu_fault_irq},
};

// ---------------------------------------------------------------------------

// Random streams.  Every packet is generated well-formed and may then be
// mutated into an error case.  Memory side effects of *failing* packets are
// only compared where both implementations agree by construction: mutations
// either leave addresses valid, misalign them, or move them wholly outside
// the mapping (never partially), and sizes are bounded (COPY/FILL <= 1 KiB),
// so the chunk-size difference between model (64 KiB) and firmware (256 B)
// is unobservable.  DISPATCH grid dimensions are never mutated and DISPATCH /
// DISPATCH_INDIRECT opcodes are never mutation targets (runaway dispatches).
u64 rand_dst(Rng& r, u32 align = 8) { return kDst + (r.below(0x3000) & ~(align - 1)); }

Pk gen_packet(Rng& r, bool in_ib) {
  const u32 k0 = r.below(in_ib ? 28 : 11);
  u32 k = k0 >= 22 ? (k0 % 2 ? 6u : 5u) : k0;  // IB: copy/fill-heavy tail (no shader execution)
  if (k == 14 || k == 15) k = 17;
  switch (k) {
    case 0: return nop(r.below(4));
    case 1: return fence(rand_dst(r, 4) & ~7ull, 0xA5A50000u | r.below(0x10000), r.chance(30) ? cp::FENCE_FLAG_L2_WB : 0);
    case 2: return fence(rand_dst(r, 8), (static_cast<u64>(r.below(0xffff)) << 32) | r.below(0xffff),
                         cp::FENCE_FLAG_64BIT | (!in_ib && r.chance(50) ? cp::FENCE_FLAG_IRQ : 0));
    case 3: return timestamp(rand_dst(r, 8));
    case 4: return write_data(rand_dst(r, 4), Pk(1 + r.below(8), 0xA5A50000u | r.below(0x10000)));
    case 5: return fill(rand_dst(r, 4) & ~3ull, 4 * r.below(256), 0xA5A50000u | r.below(0x10000), in_ib && r.chance(30));
    case 6: {
      const u64 sz = r.below(1024);
      return copy(kData + r.below(0x800), kDst + 0x2000 + r.below(0x1000), sz);  // src/dst disjoint
    }
    case 7: return cache(r.below(0x40));
    case 8: return wait_mem(kFlag, kFlagWord0, 0xffffffff, cp::WAIT_EQ, false);
    case 9: return wait_mem(kFlag + 4, 0x9abcdef0u, 0xffffffffu, r.chance(50) ? cp::WAIT_GE : cp::WAIT_LE, false);
    case 10: return Pair::indirect(kIb2, 0);  // ring only; empty nested IB
    case 11: return compute_prologue(1 + r.below(64), r.below(0x10000));
    case 12: return set_shader(kCode, 1 + r.below(128), 1 + r.below(64));
    case 13: return user_regs(r.below(26), Pk(1, r.below(1000)));
    case 14: return dispatch(r.below(4), 1 + r.below(2), 1, in_ib && r.chance(40));
    case 15: return dispatch_indirect(kArgs + 12 * r.below(4), in_ib && r.chance(40));
    case 16: return set_predicate(kFlag + 4 * r.below(6), r.chance(20) ? 0u : (cp::PRED_ENABLE | (r.chance(50) ? cp::PRED_INVERT : 0)));
    case 17: return query(rand_dst(r, 8), r.chance(50) ? cp::QUERY_OCCLUSION : cp::QUERY_PIPELINE_STATS);
    case 18: return fill(rand_dst(r, 4) & ~3ull, 16, r.below(100), true);
    case 19: return nop(r.below(2));
    case 20: return user_regs(0, {lo32(kOut), hi32(kOut)});
    default: return set_predicate(0, 0);
  }
}

void mutate(Rng& r, Pk& p) {
  const u32 hdr = p[0];
  const u8 op = static_cast<u8>(hdr & 0xff);
  switch (r.below(7)) {
    case 0: p[0] |= 1u << (9 + r.below(7)); break;                      // reserved header bit
    case 1: p[0] = (hdr & 0xffffu) | (((hdr >> 16) + (r.chance(50) ? 1u : 0xffffu)) << 16); break;  // COUNT +-1
    case 2:
      if (op != cp::OP_DISPATCH && op != cp::OP_DISPATCH_INDIRECT) p[0] |= cp::HDR_PRED;
      break;
    case 3: {
      if (op == cp::OP_DISPATCH || op == cp::OP_DISPATCH_INDIRECT) break;
      u32 nop2 = r.below(0x20);
      if (nop2 == cp::OP_DISPATCH || nop2 == cp::OP_DISPATCH_INDIRECT) nop2 = 0x1f;
      p[0] = (hdr & ~0xffu) | nop2;
      break;
    }
    case 4:
      if (p.size() > 1 && op != cp::OP_DISPATCH) p[1] |= 1;               // misalign first payload dword (address)
      break;
    case 5:
      if (p.size() > 2 && op != cp::OP_DISPATCH && op != cp::OP_COPY && op != cp::OP_INDIRECT) p[2] = 2;  // wholly unmapped
      break;
    default:
      if (p.size() > 1 && op != cp::OP_DISPATCH && op != cp::OP_FILL && op != cp::OP_COPY && op != cp::OP_INDIRECT &&
          op != cp::OP_WRITE_DATA)
        p.back() ^= 1u << r.below(32);
      break;
  }
}


void run_random(u64 seed) {
  Rng r(seed);
  Pair p("random seed " + std::to_string(seed));
  emit(E_NOTE, 1000 + static_cast<u32>(seed));
  p.boot();
  if (r.chance(50)) p.enable_rptr_wb();
  const u32 rounds = r.chance(15) ? 90 + r.below(60) : 3 + r.below(8);  // long runs wrap the ring
  for (u32 round = 0; round < rounds; ++round) {
    p.label = "random seed " + std::to_string(seed) + " round " + std::to_string(round);
    const u32 act = r.below(10);
    const u32 mut_rate = r.chance(35) ? 25 : 0;  // most rounds are clean so streams get deep
    if (act == 0) p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE | regs::CP_CTRL_HALT_REQ, "HALT_REQ");
    if (act <= 5) {  // ring-level packets
      Pk ring;
      for (u32 i = 1 + r.below(5); i; --i) {
        Pk pk = gen_packet(r, false);
        if (r.chance(mut_rate)) mutate(r, pk);
        ring = ring + pk;
      }
      p.ring_put(ring, true, r.chance(20) ? 1 + r.below(4) : 0);
    } else {  // one command buffer
      Pk ib = r.chance(85) ? compute_prologue(1 + r.below(64), r.below(1000)) : Pk{};
      for (u32 i = 1 + r.below(10); i; --i) {
        Pk pk = gen_packet(r, true);
        if (r.chance(mut_rate)) mutate(r, pk);
        ib = ib + pk;
      }
      p.run_ib(r.chance(50) ? kIb : kIb2, ib);
    }
    if (p.m.dev.cp().halted()) p.recover();
    if (p.m.dev.cp().stopped()) p.both_w(regs::CP_CTRL, regs::CP_CTRL_ENABLE, "resume");
    if (r.chance(10)) p.recover();
  }
  p.check("final");
  g_cov.ticks += p.m.dev.cp().ticks();
  g_cov.wraps += p.wptr / kRingDw;
  if (p.m.dev.cp().dispatches() != 0) {
    std::fprintf(stderr, "seed %llu executed a dispatch (generator bug)\n", (unsigned long long)seed);
    ++g_failures;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const char* out = nullptr;
  unsigned seeds = 20;
  u64 seed0 = 1;
  const char* only = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--only") && i + 1 < argc) {
      only = argv[++i];
      continue;
    }
    if (!std::strcmp(argv[i], "--seeds") && i + 1 < argc) seeds = static_cast<unsigned>(std::atoi(argv[++i]));
    else if (!std::strcmp(argv[i], "--seed0") && i + 1 < argc) seed0 = static_cast<u64>(std::atoll(argv[++i]));
    else if (argv[i][0] != '-' && !out) out = argv[i];
    else {
      std::fprintf(stderr, "usage: gen_cp_vectors <out.hex> [--seeds N] [--seed0 S]\n");
      return 2;
    }
  }
  if (!out) {
    std::fprintf(stderr, "usage: gen_cp_vectors <out.hex> [--seeds N] [--seed0 S]\n");
    return 2;
  }
  u32 id = 0;
  for (const auto& s : kScenarios) {
    if (only && std::strcmp(only, s.name)) continue;
    Pair p(s.name);
    emit(E_NOTE, id++);
    s.fn(p);
    p.check("scenario end");
    if (p.m.dev.cp().dispatches() != 0) {
      std::fprintf(stderr, "scenario %s executed a dispatch (generator bug)\n", s.name);
      ++g_failures;
    }
    g_cov.ticks += p.m.dev.cp().ticks();
    std::printf("  scenario %-16s id %u\n", s.name, id - 1);
  }
  for (unsigned i = 0; i < seeds; ++i) run_random(seed0 + i);
  emit(E_END);
  std::FILE* f = std::fopen(out, "w");
  if (!f) {
    std::perror(out);
    return 1;
  }
  for (u32 w : g_stream) std::fprintf(f, "%08x\n", w);
  std::fclose(f);
  std::printf("  %u scenarios + %u random seeds: %u events, %llu checks, %llu ring wraps, %llu blocked checks, "
              "errors latched:",
              static_cast<unsigned>(sizeof kScenarios / sizeof kScenarios[0]), seeds, g_events,
              (unsigned long long)g_cov.checks, (unsigned long long)g_cov.wraps, (unsigned long long)g_cov.waits);
  for (int e = 1; e < 16; ++e)
    if (g_cov.err_seen[e]) std::printf(" %d:%llu", e, (unsigned long long)g_cov.err_seen[e]);
  std::printf("\n");
  return test_result("gen_cp_vectors");
}
