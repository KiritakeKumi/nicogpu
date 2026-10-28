// gen_mmu_vectors.cpp -- MMU TB vector generator.
//
// Drives the *functional model* (+ pt_builder.cpp) and dumps
// a replay file for rtl/gpu/tb/tb_mmu.sv: page-table memory images, context /
// invalidate operations and translation queries with the model's answers.
//   gen_mmu_vectors <seed> <out.hex>
// Build: see run_tb_mmu.sh (Vivado's bundled MinGW c++, C++17).
//
// File format: $readmemh, 4 x 64-bit words per record:
//   w0 = {op[63:56], vmid[55:48], acc|atomic<<4 [47:40], exp_fault[39:32],
//         flags[31:24] (bit0 sparse), exp_level[23:16], exp_size[15:8], en[7:0]}
//   w1 = va | table address | invalidate mask | root PA
//   w2 = exp_pa | memory data
//   w3 = exp_pte
// ops: 0 END, 1 CLEAR_MEM, 2 MEM(w1=addr, w2=data), 3 ROOT(vmid, w1=root, en),
//      4 INV(w1=mask), 5 QUERY, 6 PHASE(w1=phase number, informational)
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "nicogpu/mmu.h"
#include "nicogpu/pt_builder.h"

using namespace nicogpu;
using namespace nicogpu::mmu;
using mem::kSysBase;

namespace {

std::mt19937_64 rng;
u64 rnd(u64 n) { return n ? rng() % n : 0; }
bool chance(unsigned pct) { return rnd(100) < pct; }

FILE* out;
unsigned n_rec = 0;
void rec(u64 op, u64 vmid, u64 acc, u64 f, u64 flags, u64 lvl, u64 sz, u64 en, u64 w1, u64 w2, u64 w3) {
  const u64 w0 = (op << 56) | (vmid << 48) | (acc << 40) | (f << 32) | (flags << 24) | (lvl << 16) | (sz << 8) | en;
  std::fprintf(out, "%016llx %016llx %016llx %016llx\n", (unsigned long long)w0, (unsigned long long)w1,
               (unsigned long long)w2, (unsigned long long)w3);
  ++n_rec;
}

struct Alloc {
  u64 next = kSysBase + 0x1000;
  std::vector<u64> free_list;
  std::set<u64> live;
  mem::PhysMem* pm = nullptr;
  u64 alloc() {
    u64 p;
    if (!free_list.empty()) {
      p = free_list.back();
      free_list.pop_back();
    } else {
      p = next;
      next += 4096;
    }
    live.insert(p);
    for (unsigned i = 0; i < 512; ++i) pm->write64(p + 8 * i, 0);
    return p;
  }
  void release(u64 p) {
    live.erase(p);
    free_list.push_back(p);
  }
};

unsigned size_code(u64 sz) {
  switch (sz) {
    case 4 * KiB: return 0;
    case 64 * KiB: return 1;
    case 2 * MiB: return 2;
    case 1 * GiB: return 3;
    case 512 * GiB: return 4;
  }
  std::fprintf(stderr, "bad page size %llu\n", (unsigned long long)sz);
  std::exit(2);
}

struct Stats {
  unsigned q = 0, by_fault[8] = {}, sparse = 0, by_size[5] = {}, atomic = 0, bypass = 0;
} st;

// fault-free (vmid, va) of this / the previous phase
std::vector<std::pair<unsigned, u64>> last_ok, prev_ok;

struct Exp {
  Fault fault = Fault::None;
  bool sparse = false;
  u64 pa = 0, pte = 0, size = 4 * KiB;
  unsigned level = 0;
};

mem::PhysMem pm;
Mmu* mmu_p;

Exp expect(unsigned vmid, u64 va, Access acc, bool atomic) {
  Exp e;
  Translation t = mmu_p->translate(vmid, va, atomic ? Access::Write : acc);
  if (vmid != 0 && t.fault == Fault::None && atomic && !t.sparse_null && !(t.pte & PTE_R)) t.fault = Fault::Permission;
  e.fault = t.fault;
  e.sparse = t.sparse_null;
  e.pa = t.pa;
  e.pte = t.pte;
  e.size = t.page_size ? t.page_size : 4 * KiB;
  e.level = t.level;
  if (t.fault != Fault::None && !atomic) {
    // Cross-check the model's fault latch against what the TB will expect.
    mmu_p->clear_fault();
    u8 b;
    if (acc == Access::Write) mmu_p->write(vmid, va, &b, 1);
    else mmu_p->read(vmid, va, &b, 1, acc);
    const FaultRecord& r = mmu_p->fault_record();
    if (!r.valid || r.vmid != vmid || r.va != va || r.access != acc || r.fault != t.fault || r.level != t.level ||
        r.pte != t.pte) {
      std::fprintf(stderr, "fault record mismatch vmid %u va %llx\n", vmid, (unsigned long long)va);
      std::exit(3);
    }
    mmu_p->clear_fault();
  }
  return e;
}

void query(unsigned vmid, u64 va, Access acc, bool atomic) {
  const Exp e = expect(vmid, va, acc, atomic);
  ++st.q;
  ++st.by_fault[static_cast<unsigned>(e.fault)];
  if (atomic) ++st.atomic;
  if (vmid == 0) ++st.bypass;
  if (e.fault == Fault::None) {
    if (vmid != 0) last_ok.push_back({vmid, va});
    if (e.sparse) ++st.sparse;
    ++st.by_size[size_code(e.size)];
  }
  rec(5, vmid, static_cast<u64>(acc) | (atomic ? 0x10 : 0), static_cast<u64>(e.fault), e.sparse ? 1 : 0, e.level,
      size_code(e.size), 0, va, e.pa, e.pte);
}

Access rand_acc(bool* atomic) {
  const unsigned r = static_cast<unsigned>(rnd(100));
  *atomic = false;
  if (r < 50) return Access::Read;
  if (r < 75) return Access::Write;
  if (r < 88) {
    *atomic = true;
    return Access::Write;
  }
  return Access::Exec;
}

u64 rand_pa(u64 align) {
  u64 pa;
  const unsigned r = static_cast<unsigned>(rnd(100));
  if (r < 60) pa = rnd(16 * GiB);
  else if (r < 85) pa = mem::kHostBase + rnd(1ull << 40);
  else pa = rng() & 0x0000'ffff'ffff'ffffull;
  pa &= ~(align - 1);
  return pa & 0x0000'ffff'ffff'f000ull;
}

u64 rand_flags() {
  u64 f = 0;
  if (chance(85)) f |= PTE_R;
  if (chance(60)) f |= PTE_W;
  if (chance(25)) f |= PTE_X;
  const unsigned m = static_cast<unsigned>(rnd(3));
  return f | mtype_bits(static_cast<MType>(m));
}

// Anchors: different L3 / L2 / L1 slots so that splits land in fresh tables.
const u64 kAnchors[] = {0x1'0000'0000ull, 0x1'4000'0000ull, 0x40'0000'0000ull, 0x7fff'8000'0000ull,
                        0x2000'0000'0000ull};

struct VmCfg {
  unsigned id = 0;
  PageTableBuilder* pt = nullptr;
  std::vector<std::pair<u64, u64>> touched;  // (va, size)
  std::vector<std::pair<u64, u64>> mapped;   // Map ops only (later ops may overwrite)
};

void random_op(VmCfg& vm) {
  const u64 anchor = kAnchors[rnd(5)];
  const unsigned ar = static_cast<unsigned>(rnd(100));
  const u64 align = ar < 45 ? 4 * KiB : ar < 75 ? 64 * KiB : 2 * MiB;
  const u64 window = 256 * MiB;
  u64 va = anchor + (rnd(window / align) * align);
  u64 size;
  const unsigned sr = static_cast<unsigned>(rnd(100));
  PageTableBuilder::Kind kind;
  const unsigned kr = static_cast<unsigned>(rnd(100));
  kind = kr < 55 ? PageTableBuilder::Kind::Map : kr < 78 ? PageTableBuilder::Kind::SparseNull
                                                         : PageTableBuilder::Kind::Unmap;
  if (sr < 35) size = 4 * KiB * (1 + rnd(40));
  else if (sr < 65) size = 64 * KiB * (1 + rnd(4));
  else if (sr < 92) size = 2 * MiB * (1 + rnd(3));
  else if (kind == PageTableBuilder::Kind::SparseNull && sr < 97) {
    size = 1 * GiB;
    va = anchor & ~(1 * GiB - 1);
  } else if (kind == PageTableBuilder::Kind::SparseNull) {
    size = 512 * GiB;
    va = (100 + rnd(3)) * 512 * GiB;
  } else size = 4 * KiB;
  if (align == 2 * MiB && size < 2 * MiB) size = 2 * MiB;
  if (chance(40)) {  // edit right where the previous phase's translations were cached
    std::vector<u64> mine;
    for (auto& pr : prev_ok)
      if (pr.first == vm.id) mine.push_back(pr.second);
    if (!mine.empty()) {
      va = mine[rnd(mine.size())] & ~(4 * KiB - 1);
      size = 4 * KiB * (1 + rnd(3));
      if (chance(30)) { va &= ~(64 * KiB - 1); size = 64 * KiB; }
      if (chance(10)) { va &= ~(2 * MiB - 1); size = 2 * MiB; }
    }
  }
  const u64 pa = rand_pa(size >= 2 * MiB ? 2 * MiB : size >= 64 * KiB ? 64 * KiB : 4 * KiB);
  if (kind == PageTableBuilder::Kind::Map && pa + size >= (1ull << 48)) return;
  if (!vm.pt->set(va, size, kind, pa, rand_flags())) {
    std::fprintf(stderr, "builder.set failed\n");
    std::exit(4);
  }
  vm.touched.push_back({va, size});
  if (kind == PageTableBuilder::Kind::Map) vm.mapped.push_back({va, size});
}

// ---- fuzz tables (hand built, includes malformed entries) -------------------
struct Fuzz {
  Alloc* al;
  std::map<u64, unsigned> level_of;  // table -> level
  std::map<u64, std::vector<unsigned>> slots;
  u64 root = 0;

  u64 new_table(unsigned level) {
    u64 p = al->alloc();
    level_of[p] = level;
    return p;
  }
  u64 pick_table(unsigned level) {
    std::vector<u64> v;
    for (auto& kv : level_of)
      if (kv.second == level) v.push_back(kv.first);
    return v.empty() ? 0 : v[rnd(v.size())];
  }
  u64 sys_pa_any() { return kSysBase + 4096 * rnd(1u << 19); }

  u64 rand_entry(unsigned level) {
    const unsigned r = static_cast<unsigned>(rnd(100));
    const u64 sw = chance(30) ? (rng() & 0xff00'0000'0000'0000ull) : 0;
    u64 e = 0;
    if (level > 0) {
      if (r < 40) {  // PDE to (possibly shared) child
        u64 child = chance(70) ? new_table(level - 1) : pick_table(level - 1);
        if (!child) child = new_table(level - 1);
        e = make_pde(child);
      } else if (r < 46) e = make_pde(sys_pa_any());               // PDE to empty SYS page
      else if (r < 50) e = make_pde(rnd(16 * GiB) & ~4095ull);     // PDE outside SYS
      else if (r < 54) e = make_pde(new_table(level - 1)) | (1ull << (2 + rnd(5)));  // PDE with attr bits
      else if (r < 58) e = make_pde(new_table(level - 1)) | PTE_F64K;
      else if (r < 62) e = make_pde(new_table(level - 1)) | (1ull << (9 + rnd(3)));  // reserved bit
      else if (r < 65) e = make_pde(new_table(level - 1)) | (1ull << (48 + rnd(8)));
      else if (r < 72) e = make_sparse_null();
      else if (r < 75) e = PTE_V | PTE_SPARSE;
      else if (r < 78) e = PTE_SPARSE | (1ull << (1 + rnd(7)));
      else if (r < 80) e = PTE_SPARSE | (rng() & PTE_PA_MASK);
      else if (r < 82) e = rng() & ~(PTE_V | PTE_SPARSE);          // not present, junk bits
      else if (level == 1 && r < 90) {                             // 2M leaf
        e = make_pte(rand_pa(2 * MiB), rand_flags());
        if (chance(10)) e |= PTE_F64K;
        if (chance(10)) e |= 0x1000ull << rnd(9);                  // misaligned PA
        if (chance(8)) e = (e & ~PTE_MTYPE_MASK) | mtype_bits(MTYPE_RSVD);
      } else if (level >= 2 && r < 90) e = make_pte(rand_pa(4 * KiB), rand_flags());  // D9
      else e = rng();                                              // garbage
    } else {
      if (r < 40) e = make_pte(rand_pa(4 * KiB), rand_flags());
      else if (r < 48) e = 0;
      else if (r < 55) e = make_sparse_null();
      else if (r < 58) e = make_pte(rand_pa(4 * KiB), rand_flags(), false, false);  // LEAF=0 (D8)
      else if (r < 61) e = make_pte(rand_pa(4 * KiB), rand_flags()) | (1ull << (9 + rnd(3)));
      else if (r < 64) e = (make_pte(rand_pa(4 * KiB), rand_flags()) & ~PTE_MTYPE_MASK) | mtype_bits(MTYPE_RSVD);
      else if (r < 68) e = make_pte(rand_pa(64 * KiB), rand_flags(), true) | (0x1000ull << rnd(4));  // D13
      else if (r < 72) e = PTE_V | PTE_SPARSE;
      else if (r < 76) e = rng() & ~(PTE_V | PTE_SPARSE);
      else if (r < 80) e = PTE_SPARSE | (1ull << (1 + rnd(7)));
      else if (r < 84) e = make_pte(rand_pa(4 * KiB), rand_flags()) | (1ull << (48 + rnd(8)));
      else e = rng() & ~PTE_F64K;
      // F64K on a stray entry could make an inconsistent group that the
      // hardware may resolve either way; only the group filler below sets F64K
      // on valid entries.
      if ((e & PTE_V) && !(e & PTE_SPARSE) && (e & PTE_F64K) && ((e & PTE_PA_MASK) % (64 * KiB) == 0) &&
          !(e & PTE_RSVD_MASK))
        e &= ~PTE_F64K;
    }
    return e | sw;
  }

  void fill(u64 table, unsigned level) {
    const unsigned n = level >= 2 ? 5 + static_cast<unsigned>(rnd(5)) : level == 1 ? 8 + static_cast<unsigned>(rnd(8))
                                                                   : 12 + static_cast<unsigned>(rnd(24));
    for (unsigned i = 0; i < n; ++i) {
      const unsigned idx = static_cast<unsigned>(rnd(512));
      pm.write64(table + 8ull * idx, rand_entry(level));
      slots[table].push_back(idx);
    }
    if (level == 0) {  // consistent 64K groups
      const unsigned g = 1 + static_cast<unsigned>(rnd(4));
      for (unsigned k = 0; k < g; ++k) {
        const unsigned base = 16 * static_cast<unsigned>(rnd(32));
        const u64 pte = make_pte(rand_pa(64 * KiB), rand_flags(), true) | (chance(30) ? (rng() & PTE_SW_MASK) : 0);
        for (unsigned i = 0; i < 16; ++i) pm.write64(table + 8ull * (base + i), pte);
        for (unsigned i = 0; i < 16; ++i) slots[table].push_back(base + i);
      }
    }
  }

  void build() {
    root = new_table(3);
    fill(root, 3);
    // children were created while filling; fill them breadth-first
    for (unsigned lvl = 2; lvl + 1 > 0; --lvl) {
      std::vector<u64> ts;
      for (auto& kv : level_of)
        if (kv.second == lvl && !slots.count(kv.first)) ts.push_back(kv.first);
      for (u64 t : ts) fill(t, lvl);
      if (lvl == 0) break;
    }
    // make sure every table has a slot list (shared-but-unfilled tables)
  }

  u64 rand_va() {
    u64 table = root;
    u64 va = 0;
    for (int level = 3; level >= 0; --level) {
      unsigned idx;
      auto it = slots.find(table);
      if (it != slots.end() && !it->second.empty() && chance(90)) idx = it->second[rnd(it->second.size())];
      else idx = static_cast<unsigned>(rnd(512));
      va |= static_cast<u64>(idx) << level_shift(static_cast<unsigned>(level));
      const u64 e = pm.read64(table + 8ull * idx);
      const DecodedEntry d = decode_entry(e, static_cast<unsigned>(level));
      if (d.kind != EntryKind::Table) {
        if (level > 0) va |= rng() & ((1ull << level_shift(static_cast<unsigned>(level))) - 1);
        else va |= rnd(4096);
        return va;
      }
      table = d.base;
    }
    return va | rnd(4096);
  }
};

// Emit only the words that changed since the previous dump (the TB keeps its
// memory across phases; first dump starts from a cleared memory).
std::map<u64, u64> shadow;
void dump_memory(Alloc& al) {
  std::map<u64, u64> now;
  for (u64 p : al.live)
    for (unsigned i = 0; i < 512; ++i) {
      const u64 v = pm.read64(p + 8 * i);
      if (v) now[p + 8 * i] = v;
    }
  if (shadow.empty()) rec(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  for (auto& kv : now) {
    auto it = shadow.find(kv.first);
    if (it == shadow.end() || it->second != kv.second) rec(2, 0, 0, 0, 0, 0, 0, 0, kv.first, kv.second, 0);
  }
  for (auto& kv : shadow)
    if (!now.count(kv.first)) rec(2, 0, 0, 0, 0, 0, 0, 0, kv.first, 0, 0);
  shadow = now;
}

u64 pick_va(const std::vector<VmCfg*>& all, VmCfg* own) {
  const unsigned r = static_cast<unsigned>(rnd(100));
  VmCfg* v = chance(25) ? all[rnd(all.size())] : own;
  if (r < 10 || v->touched.empty()) {
    const unsigned k = static_cast<unsigned>(rnd(100));
    if (k < 50) return kAnchors[rnd(5)] + rnd(256 * MiB);
    if (k < 75) return rng() & 0x0000'ffff'ffff'ffffull;
    return (100 + rnd(3)) * 512 * GiB + rnd(512 * GiB);
  }
  auto t = (chance(55) && !v->mapped.empty()) ? v->mapped[rnd(v->mapped.size())] : v->touched[rnd(v->touched.size())];
  u64 off = rnd(t.second);
  if (chance(20)) off = (t.second - 1) - rnd(std::min<u64>(t.second, 4096));      // tail
  if (chance(15) && t.first >= 8192) return t.first - 1 - rnd(4096);              // just before
  if (chance(10)) return t.first + t.second + rnd(4096);                          // just after
  return t.first + off;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) return 1;
  const u64 seed = std::strtoull(argv[1], nullptr, 0);
  rng.seed(seed * 0x9E3779B97F4A7C15ull + 12345);
  out = std::fopen(argv[2], "w");
  if (!out) return 1;

  Mmu mmu(pm);
  mmu_p = &mmu;
  Alloc al;
  al.pm = &pm;
  auto mk = [&](void) {
    return new PageTableBuilder(pm, [&]() { return al.alloc(); }, [&](u64 p) { al.release(p); });
  };

  // VMID map: 1,2,3 own builders; 5 enabled then disabled/re-enabled; 6 switches
  // between two roots; 10 shares VMID 1's root; 4 fuzz; 7 bad root; 8 has a root
  // but is never enabled; 9 unused; 11..15 light use (own small builder each).
  VmCfg vm[16];
  std::vector<PageTableBuilder*> builders;
  for (unsigned v : {1u, 2u, 3u, 5u, 11u, 12u, 13u}) {
    vm[v].pt = mk();
    builders.push_back(vm[v].pt);
  }
  VmCfg v6b;
  v6b.id = 6;
  for (unsigned v = 0; v < 16; ++v) vm[v].id = v;
  vm[6].pt = mk();
  v6b.pt = mk();
  builders.push_back(vm[6].pt);
  builders.push_back(v6b.pt);
  vm[10].pt = vm[1].pt;  // shares tables (and touched list is separate)
  Fuzz fz;
  fz.al = &al;
  fz.build();
  VmCfg* v6cur = &vm[6];

  std::vector<VmCfg*> all;
  for (unsigned v : {1u, 2u, 3u, 5u, 6u, 11u, 12u}) all.push_back(&vm[v]);
  all.push_back(&v6b);

  const unsigned phases = 6;
  const unsigned nq = 220;
  for (unsigned ph = 0; ph < phases; ++ph) {
    rec(6, 0, 0, 0, 0, 0, 0, 0, ph, 0, 0);
    prev_ok = last_ok;
    last_ok.clear();
    // table edits
    for (unsigned v : {1u, 2u, 3u, 5u, 11u, 12u, 13u}) {
      const unsigned n = 6 + static_cast<unsigned>(rnd(10));
      for (unsigned i = 0; i < n; ++i) random_op(vm[v]);
      vm[10].touched = vm[1].touched;
      vm[10].mapped = vm[1].mapped;
    }
    for (unsigned i = 0; i < 10; ++i) random_op(*v6cur);
    if (ph % 2 == 1) for (unsigned i = 0; i < 6; ++i) random_op(*v6cur);
    if (ph >= 2) {  // mutate fuzz tables a little between phases
      for (auto& kv : fz.level_of)
        if (chance(40)) {
          const unsigned idx = static_cast<unsigned>(rnd(512));
          pm.write64(kv.first + 8ull * idx, fz.rand_entry(kv.second));
          fz.slots[kv.first].push_back(idx);
        }
    }
    // Commit to "hardware": invalidate everything, then free retired tables.
    mmu.invalidate(0xFFFE);
    for (auto* b : builders) {
      std::string why;
      if (!b->verify(&why)) {
        std::fprintf(stderr, "builder verify: %s\n", why.c_str());
        return 5;
      }
      b->reclaim();
    }
    if (ph == 0) {
      // fresh fuzz fill may have allocated tables lazily; nothing else to do
    }
    dump_memory(al);

    // contexts
    const bool v5_en = !(ph == 2);                      // disabled in phase 2, back in 3
    mmu.set_root(1, vm[1].pt->root(), true);
    rec(3, 1, 0, 0, 0, 0, 0, 1, vm[1].pt->root(), 0, 0);
    mmu.set_root(2, vm[2].pt->root(), true);
    rec(3, 2, 0, 0, 0, 0, 0, 1, vm[2].pt->root(), 0, 0);
    mmu.set_root(3, vm[3].pt->root(), true);
    rec(3, 3, 0, 0, 0, 0, 0, 1, vm[3].pt->root(), 0, 0);
    mmu.set_root(4, fz.root, true);
    rec(3, 4, 0, 0, 0, 0, 0, 1, fz.root, 0, 0);
    mmu.set_root(5, vm[5].pt->root(), v5_en);
    rec(3, 5, 0, 0, 0, 0, 0, v5_en ? 1 : 0, vm[5].pt->root(), 0, 0);
    v6cur = (ph % 2 == 0) ? &vm[6] : &v6b;
    mmu.set_root(6, v6cur->pt->root(), true);
    rec(3, 6, 0, 0, 0, 0, 0, 1, v6cur->pt->root(), 0, 0);
    mmu.set_root(7, 0x0000'1234'5000ull, true);          // root outside SYS
    rec(3, 7, 0, 0, 0, 0, 0, 1, 0x0000'1234'5000ull, 0, 0);
    mmu.set_root(8, vm[1].pt->root(), false);            // root but disabled
    rec(3, 8, 0, 0, 0, 0, 0, 0, vm[1].pt->root(), 0, 0);
    mmu.set_root(10, vm[1].pt->root(), true);            // shares tables with VMID 1
    rec(3, 10, 0, 0, 0, 0, 0, 1, vm[1].pt->root(), 0, 0);
    for (unsigned v : {11u, 12u, 13u}) {
      mmu.set_root(v, vm[v].pt->root(), true);
      rec(3, v, 0, 0, 0, 0, 0, 1, vm[v].pt->root(), 0, 0);
    }
    mmu.invalidate(0xFFFE);
    rec(4, 0, 0, 0, 0, 0, 0, 0, 0xFFFE, 0, 0);

    // re-query what was cached before the edits: a missed flush shows up here
    for (size_t i = prev_ok.size() > 24 ? prev_ok.size() - 24 : 0; i < prev_ok.size(); ++i)
      query(prev_ok[i].first, prev_ok[i].second, Access::Read, false);

    // VMID 0 bypass at the edges of the decoded regions (VRAM 16 GiB, SYS 2 GiB, HOST >= 2^47): legal PAs pass
    // through, holes answer BAD_PHYS_ADDR (level 0, pte 0), VA >= 2^48 stays NON_CANONICAL.
    {
      static const u64 edges[] = {0x0,
                                  0x1,
                                  0x3'ffff'ffffull,
                                  0x4'0000'0000ull,
                                  0xf'ffff'f000ull,
                                  0x10'0000'0000ull - 1,
                                  0x10'0000'0000ull,
                                  0x10'7fff'ffffull,
                                  0x10'8000'0000ull,
                                  0x7fff'ffff'ffffull,
                                  0x8000'0000'0000ull,
                                  0xffff'ffff'ffffull,
                                  0x1'0000'0000'0000ull};
      for (u64 e : edges) {
        bool atomic_e;
        const Access acc_e = rand_acc(&atomic_e);
        query(0, e, acc_e, atomic_e);
      }
    }

    // queries
    for (unsigned q = 0; q < nq; ++q) {
      bool atomic;
      Access acc;
      const unsigned r = static_cast<unsigned>(rnd(100));
      unsigned vmid;
      u64 va;
      if (r < 40) {
        static const unsigned pool[] = {1, 2, 3, 5, 6, 10, 11, 12, 13};
        vmid = pool[rnd(9)];
        VmCfg* own = vmid == 6 ? v6cur : &vm[vmid];
        va = pick_va(all, own);
      } else if (r < 60) {
        vmid = 4;
        va = fz.rand_va();
      } else if (r < 66) {
        vmid = static_cast<unsigned>(rnd(16));  // includes 0 bypass, 7, 8, 9
        va = rng() & 0x0000'ffff'ffff'ffffull;
      } else if (r < 70) {
        static const unsigned pool[] = {1, 2, 3, 4, 6};
        vmid = pool[rnd(5)];
        va = pick_va(all, &vm[1]) | (1ull << (48 + rnd(16)));  // non-canonical
      } else {
        // locality: re-hit recent pages (TLB hits) of one VMID
        vmid = 1 + static_cast<unsigned>(rnd(3));
        va = pick_va(all, &vm[vmid]);
        for (unsigned k = 0; k < 4; ++k) {
          acc = rand_acc(&atomic);
          query(vmid, (va & ~4095ull) + rnd(4096), acc, atomic);
        }
        continue;
      }
      acc = rand_acc(&atomic);
      query(vmid, va, acc, atomic);
    }
    // one guaranteed hit-check pattern per phase: same page twice
    bool a2;
    Access ac = rand_acc(&a2);
    const u64 va = pick_va(all, &vm[2]);
    query(2, va, ac, a2);
    query(2, va ^ 8, ac, a2);
  }
  rec(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  std::fclose(out);

  std::printf("gen_mmu_vectors seed %llu: %u records, %u queries\n", (unsigned long long)seed, n_rec, st.q);
  std::printf("  faults: none %u not_present %u non_canon %u perm %u bad_pte %u bad_phys %u | sparse %u atomic %u bypass %u\n",
              st.by_fault[0], st.by_fault[1], st.by_fault[2], st.by_fault[3], st.by_fault[4], st.by_fault[5], st.sparse, st.atomic,
              st.bypass);
  std::printf("  sizes: 4K %u 64K %u 2M %u 1G %u 512G %u\n", st.by_size[0], st.by_size[1], st.by_size[2],
              st.by_size[3], st.by_size[4]);
  return 0;
}
