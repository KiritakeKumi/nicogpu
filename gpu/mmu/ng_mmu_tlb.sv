// ng_mmu_tlb.sv -- fully associative, VMID-tagged translation cache.
//
// One entry per translation at its natural size (4K / 64K / 2M leaf, or the
// span of a sparse-null entry: 4K / 2M / 1G / 512G), exactly the granularity
// of the functional model.  Faults are never cached
// (MMU.md 7.1); the caller simply does not fill them.  The raw PTE is kept
// so that a PERMISSION fault on a hit can report the entry (MMU_FAULT_PTE).
//
//   * lookup: combinational on (vmid, va).
//   * fill: one entry per cycle; victim = first invalid, else round-robin.
//               A fill whose vmid is being flushed in the same cycle is dropped
//               (an in-flight walk must not repopulate across an invalidate,
//               MMU.md 5.3).
//   * flush: per-VMID mask, clears every matching entry in one cycle
//               (flush wins over fill).
module ng_mmu_tlb
  import ng_mmu_pkg::*;
#(
  parameter int ENTRIES = 16
) (
  input  logic              clk,
  input  logic              rst_n,

  // lookup
  input  logic [3:0]        lk_vmid,
  input  logic [47:0]       lk_va,
  output logic              lk_hit,
  output logic [2:0]        lk_size,
  output logic              lk_sparse,
  output logic [35:0]       lk_pfn,      // base frame of the page (leaf) / 0
  output logic [63:0]       lk_pte,

  // fill
  input  logic              fill_valid,
  input  logic [3:0]        fill_vmid,
  input  logic [47:0]       fill_va,
  input  logic [2:0]        fill_size,
  input  logic              fill_sparse,
  input  logic [35:0]       fill_pfn,
  input  logic [63:0]       fill_pte,

  // invalidate (bit n = flush VMID n)
  input  logic [15:0]       flush_mask
);
  localparam int IW = (ENTRIES <= 1) ? 1 : $clog2(ENTRIES);

  logic [ENTRIES-1:0] e_valid;
  logic [3:0]         e_vmid   [ENTRIES];
  logic [2:0]         e_size   [ENTRIES];
  logic               e_sparse [ENTRIES];
  logic [35:0]        e_tag    [ENTRIES];   // VA[47:12] of the page base
  logic [35:0]        e_pfn    [ENTRIES];
  logic [63:0]        e_pte    [ENTRIES];

  logic [IW-1:0]      rr;

  // ---------------- lookup ---------------------------------------------------
  logic [ENTRIES-1:0] match;
  always_comb begin
    for (int i = 0; i < ENTRIES; i++)
      match[i] = e_valid[i] && (e_vmid[i] == lk_vmid) &&
                 (((e_tag[i] ^ lk_va[47:12]) & ~size_lowmask(e_size[i])) == 36'd0);
  end

  always_comb begin
    lk_hit    = |match;
    lk_size   = SZ_4K;
    lk_sparse = 1'b0;
    lk_pfn    = '0;
    lk_pte    = '0;
    for (int i = ENTRIES - 1; i >= 0; i--) begin   // lowest index wins
      if (match[i]) begin
        lk_size   = e_size[i];
        lk_sparse = e_sparse[i];
        lk_pfn    = e_pfn[i];
        lk_pte    = e_pte[i];
      end
    end
  end

  // ---------------- fill / flush ---------------------------------------------
  // Victim selection is registered (timing): vic_oh/have_free_q are computed from
  // e_valid/rr of the PREVIOUS cycle.  Fills are separated by a whole
  // lookup + walk (>= 4 cycles: S_WALK fill -> S_RESP -> S_LOOKUP -> S_WALK), so
  // by the next fill the registers reflect the post-fill state.  A flush that
  // frees an entry in the last cycle is merely not seen yet (sub-optimal victim,
  // translation still correct); an entry can only become valid through a fill,
  // so a stale "free" victim never overwrites a valid entry.
  logic [ENTRIES-1:0] free_oh;
  logic               any_free;
  always_comb begin
    free_oh  = '0;
    any_free = 1'b0;
    for (int i = ENTRIES - 1; i >= 0; i--) begin
      if (!e_valid[i]) begin
        free_oh  = ENTRIES'(1) << i;
        any_free = 1'b1;
      end
    end
  end

  logic [ENTRIES-1:0] vic_oh;
  logic               have_free_q;
  always_ff @(posedge clk) begin
    vic_oh      <= any_free ? free_oh : (ENTRIES'(1) << rr);
    have_free_q <= any_free;
  end

  wire fill_ok = fill_valid && !flush_mask[fill_vmid];

  (* max_fanout = 40 *) logic [ENTRIES-1:0] wen;   // one-hot per-entry write enable
  assign wen = vic_oh & {ENTRIES{fill_ok}};

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      e_valid <= '0;
      rr      <= '0;
    end else begin
      for (int i = 0; i < ENTRIES; i++)
        if (e_valid[i] && flush_mask[e_vmid[i]]) e_valid[i] <= 1'b0;
      // fill after flush: a fill never targets a flushed VMID, but its victim
      // slot may hold a flushed entry, so the fill assignment must come last.
      for (int i = 0; i < ENTRIES; i++)
        if (wen[i]) e_valid[i] <= 1'b1;
      if (fill_ok && !have_free_q) rr <= (rr == IW'(ENTRIES - 1)) ? '0 : rr + 1'b1;
    end
  end

  // Entry payload (no reset).  e_tag is stored unmasked: the lookup masks with
  // e_size and nothing else reads it.
  always_ff @(posedge clk) begin
    for (int i = 0; i < ENTRIES; i++)
      if (wen[i]) begin
        e_vmid[i]   <= fill_vmid;
        e_size[i]   <= fill_size;
        e_sparse[i] <= fill_sparse;
        e_tag[i]    <= fill_va[47:12];
        e_pfn[i]    <= fill_pfn;
        e_pte[i]    <= fill_pte;
      end
  end

`ifndef SYNTHESIS
  // fills must be >= 2 cycles apart (registered victim select)
  logic fill_q;
  always_ff @(posedge clk) begin
    fill_q <= fill_ok;
    if (rst_n && fill_q && fill_ok) $error("ng_mmu_tlb: back-to-back fills");
  end
`endif

endmodule : ng_mmu_tlb
