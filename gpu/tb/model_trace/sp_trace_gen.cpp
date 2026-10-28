// sp_trace_gen.cpp -- model-side trace generator.
//
// Drives the functional model  over a scenario directory
// (scen.cfg + code.hex, produced by gen_scenarios.py) and writes the model's
// warp commit/trap trace to model.trace using the Core's TraceSink hooks.
// The RTL TB (tb_sp_sched_trace.sv) writes the same format; the two files
// are diffed line by line.
//
// Trace line formats (identical to tb_sp_sched_trace.sv):
//   C <slot> <pc> <mnemonic>          one committed instruction
//   T <slot> <pc> <cause> <info>      the warp's trap (model halts there)
//
// The model runs with CoreConfig quantum=1 so its commit order is exactly
// the RTL's round-robin order.
//
// This file is RTL-side test tooling (rtl/gpu/tb/model_trace/); it only
// reads the frozen model headers and never modifies model sources.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "nicogpu/memory.h"
#include "nicogpu/mmu.h"
#include "nicogpu/sp.h"

#include "nicogpu_isa.h"  // generated: const isa::IsaTable& nicogpu_isa_table

namespace {

using namespace nicogpu;

std::string g_dir;
std::vector<std::string> g_lines;

class TracePrinter final : public sp::TraceSink {
 public:
  void on_commit(const sp::CommitRecord& r) override {
    char buf[128];
    std::snprintf(buf, sizeof buf, "C %02u %08llx %s", r.slot,
                  static_cast<unsigned long long>(r.pc), r.insn->desc->name);
    g_lines.emplace_back(buf);
  }
  // The core latches trapcause/trappc/trapinfo before calling on_trap; with
  // code_base = 0, trappc is the trapping instruction's absolute PC.
  void on_trap(const sp::Warp& w, const std::string& error, const isa::DecodedInsn* insn) override {
    (void)error;
    (void)insn;
    char buf[128];
    std::snprintf(buf, sizeof buf, "T %02u %08x %u %08x", w.slot, w.csr.trappc,
                  w.csr.trapcause, w.csr.trapinfo);
    g_lines.emplace_back(buf);
  }
};

struct Launch {
  unsigned wg = 0, nwarps = 0;
  u64 pc = 0;
};

int load_scenario(u64* base, unsigned* words, std::vector<Launch>* launches) {
  FILE* cfg = std::fopen((g_dir + "/scen.cfg").c_str(), "r");
  if (!cfg) {
    std::fprintf(stderr, "sp_trace_gen: cannot open %s/scen.cfg\n", g_dir.c_str());
    return 2;
  }
  char tok[64];
  while (std::fscanf(cfg, "%63s", tok) == 1) {
    if (!std::strcmp(tok, "base")) {
      unsigned long long v = 0;
      if (std::fscanf(cfg, "%llx", &v) != 1) return 2;
      *base = v;
    } else if (!std::strcmp(tok, "words")) {
      unsigned v = 0;
      if (std::fscanf(cfg, "%u", &v) != 1) return 2;
      *words = v;
    } else if (!std::strcmp(tok, "launch")) {
      Launch l;
      unsigned long long pc = 0;
      if (std::fscanf(cfg, "%u %u %llx", &l.wg, &l.nwarps, &pc) != 3) return 2;
      l.pc = pc;
      launches->push_back(l);
    } else if (!std::strcmp(tok, "end")) {
      char kind[16];
      if (std::fscanf(cfg, "%15s", kind) != 1) return 2;
    } else {
      std::fprintf(stderr, "sp_trace_gen: bad scen.cfg token %s\n", tok);
      return 2;
    }
  }
  std::fclose(cfg);
  return 0;
}

int run() {
  u64 base = 0x1000;
  unsigned words = 0;
  std::vector<Launch> launches;
  if (load_scenario(&base, &words, &launches)) return 2;

  // Load the code image into physical memory.  VMID 0 is the kernel bypass
  // (VA == PA, no translation), so the model fetches our words directly.
  mem::PhysMem pm;
  mmu::Mmu mmu(pm);
  FILE* hex = std::fopen((g_dir + "/code.hex").c_str(), "r");
  if (!hex) {
    std::fprintf(stderr, "sp_trace_gen: cannot open %s/code.hex\n", g_dir.c_str());
    return 2;
  }
  std::vector<u32> code;
  unsigned v;
  while (std::fscanf(hex, "%x", &v) == 1) code.push_back(static_cast<u32>(v));
  std::fclose(hex);
  if (code.size() < words) {
    std::fprintf(stderr, "sp_trace_gen: code.hex shorter than %u words\n", words);
    return 2;
  }
  for (unsigned i = 0; i < words; i++) {
    if (pm.write32(base + 4ull * i, code[i]) != Fault::None) return 2;
  }
  // Pre-populate one extra word so an 8-byte fetch at the last word reads a
  // defined second half (matches the RTL fetch model).
  pm.write32(base + 4ull * words, 0);

  const isa::IsaTable& t = nicogpu_isa_table();
  sp::CoreConfig cc;
  cc.num_sp = 1;               // one SP == one ng_sp_sched instance
  cc.quantum = 1;              // one insn per warp per scheduling round
  cc.warp_slots_per_sp = 16;
  sp::Core core(0, t, mmu, cc);
  TracePrinter tp;
  core.set_trace(&tp);

  // Launch every workgroup first (the RTL TB launches all at t = 0), then
  // step; step returns false once nothing is resident.
  for (const Launch& l : launches) {
    sp::DispatchInfo di;
    di.vmid = 0;
    di.code_va = l.pc;
    di.code_base = 0;          // PC offsets == absolute VA here
    di.num_vregs = 8;
    di.wg[0] = l.nwarps * 32;  // nwarps full warps
    di.grid[0] = 1;
    di.grid[1] = 1;
    di.grid[2] = 1;
    const u32 id[3] = {l.wg, 0, 0};
    core.launch(di, id);
  }
  sp::DispatchStatus st;
  while (core.step(&st)) {
    if (!st.ok) break;
  }

  FILE* out = std::fopen((g_dir + "/model.trace").c_str(), "w");
  if (!out) return 2;
  for (const std::string& l : g_lines) std::fprintf(out, "%s\n", l.c_str());
  std::fclose(out);
  if (!st.ok) std::fprintf(stderr, "sp_trace_gen: %s\n", st.error.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: sp_trace_gen <scenario-dir>\n");
    return 2;
  }
  g_dir = argv[1];
  return run();
}
