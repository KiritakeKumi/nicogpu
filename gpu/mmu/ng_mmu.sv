// ng_mmu.sv -- GPU MMU top: VMID contexts + TLB + page walker + fault latch.
//
// Behavioural contract: v0.1, golden model
// (Mmu::translate).  One translation client, one translation in flight
// (multi-client arbitration, hit-under-miss and a PDE cache are
// listed as gaps in).
//
// Client interface (valid/ready both ways, single outstanding):
//   req_*   vmid, 64-bit va, acc (ACC_R/W/X), atomic (with acc=ACC_W: needs R
//           and W, MMU.md 4.1), tag (echoed)
//   resp_*  fault (F_*), sparse (sparse-null hit: read 0 / drop write / atomic
//           returns 0; EXEC already turned into PERMISSION), pa, size (SZ_*),
//           level, pte (leaf / faulting entry, 0 for NON_CANONICAL and VMID 0 bypass; bypass VA[63:48]!=0 -> NON_CANONICAL;
//           bypass PA outside VRAM/SYS/HOST -> BAD_PHYS_ADDR level 0),
//           tag.  pa / size are meaningful only when fault == F_NONE and
//           sparse == 0 (size is also valid for sparse: the span).
// Timing: TLB hit = accept -> resp_valid in 2 cycles; miss adds the walk
// (<= 4 dependent table reads).
//
// Every non-NONE response is latched into the fault registers (first fault +
// MULTI).  ext_flt_* lets the requester report faults the MMU cannot
// (BAD_PHYS_ADDR after translation, MISALIGNED) through the same latch.
//
// Table read port: rd_* / rsp_* -- VMID-0 physical 64-bit read on the C3 (SYS)
// path, never through L1/L2.  rsp_valid comes at least one cycle after the
// rd_valid&&rd_ready handshake.
module ng_mmu
  import ng_mmu_pkg::*;
#(
  parameter int TLB_ENTRIES = 16
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- register bus (BAR0 offsets, see ng_mmu_regs) ----
  input  logic        csr_wr,
  input  logic [11:0] csr_waddr,
  input  logic [31:0] csr_wdata,
  input  logic [11:0] csr_raddr,
  output logic [31:0] csr_rdata,
  output logic        irq_fault,

  // ---- translation client ----
  input  logic        req_valid,
  output logic        req_ready,
  input  logic [3:0]  req_vmid,
  input  logic [63:0] req_va,
  input  logic [1:0]  req_acc,
  input  logic        req_atomic,
  input  logic [7:0]  req_tag,

  output logic        resp_valid,
  input  logic        resp_ready,
  output logic [2:0]  resp_fault,
  output logic        resp_sparse,
  output logic [47:0] resp_pa,
  output logic [2:0]  resp_size,
  output logic [1:0]  resp_level,
  output logic [63:0] resp_pte,
  output logic [7:0]  resp_tag,

  // ---- external fault report (BAD_PHYS_ADDR / MISALIGNED) ----
  input  logic        ext_flt_valid,
  input  logic [3:0]  ext_flt_vmid,
  input  logic [63:0] ext_flt_va,
  input  logic [1:0]  ext_flt_acc,
  input  logic [2:0]  ext_flt_code,
  input  logic [1:0]  ext_flt_level,
  input  logic [63:0] ext_flt_pte,

  // ---- page-table read port ----
  output logic        rd_valid,
  input  logic        rd_ready,
  output logic [47:0] rd_addr,
  input  logic        rsp_valid,
  input  logic [63:0] rsp_data,
  input  logic        rsp_err,

  // ---- statistics (test / debug) ----
  output logic [31:0] stat_walks,
  output logic [31:0] stat_tlb_hits
);
  typedef enum logic [1:0] {S_IDLE, S_LOOKUP, S_WALK, S_RESP} state_t;
  state_t st;

  // ---------------- registers ------------------------------------------------
  logic [15:0] ctx_enable;
  logic [35:0] ctx_root_pfn [NUM_VMID];
  logic [15:0] inv_pend;

  logic        flt_valid;
  logic [3:0]  flt_vmid;
  logic [63:0] flt_va, flt_pte;
  logic [1:0]  flt_acc, flt_level;
  logic [2:0]  flt_code;

  ng_mmu_regs u_regs (
    .clk, .rst_n,
    .csr_wr, .csr_waddr, .csr_wdata, .csr_raddr, .csr_rdata,
    .ctx_enable, .ctx_root_pfn, .inv_pend,
    .flt_valid, .flt_vmid, .flt_va, .flt_acc, .flt_code, .flt_level, .flt_pte,
    .irq_fault
  );

  // ---------------- request register -----------------------------------------
  logic [3:0]  q_vmid;
  logic [63:0] q_va;
  logic [1:0]  q_acc;
  logic        q_atomic;
  logic [7:0]  q_tag;

  // ---------------- TLB -------------------------------------------------------
  logic        tl_hit, tl_sparse;
  logic [2:0]  tl_size;
  logic [35:0] tl_pfn;
  logic [63:0] tl_pte;
  logic        fill_valid;

  // ---------------- walker ----------------------------------------------------
  logic        wk_start, wk_ready, wk_abort, wk_done;
  logic [2:0]  wk_fault;
  logic [1:0]  wk_level;
  logic [63:0] wk_pte;
  logic        wk_sparse;
  logic [2:0]  wk_size;
  logic [35:0] wk_pfn;

  ng_mmu_tlb #(.ENTRIES(TLB_ENTRIES)) u_tlb (
    .clk, .rst_n,
    .lk_vmid(q_vmid), .lk_va(q_va[47:0]),
    .lk_hit(tl_hit), .lk_size(tl_size), .lk_sparse(tl_sparse), .lk_pfn(tl_pfn), .lk_pte(tl_pte),
    .fill_valid, .fill_vmid(q_vmid), .fill_va(q_va[47:0]),
    .fill_size(wk_size), .fill_sparse(wk_sparse), .fill_pfn(wk_pfn), .fill_pte(wk_pte),
    .flush_mask(inv_pend)
  );

  ng_mmu_walker u_walker (
    .clk, .rst_n,
    .start(wk_start), .start_ready(wk_ready),
    .start_va(q_va[47:0]), .start_root_pfn(ctx_root_pfn[q_vmid]),
    .abort(wk_abort),
    .rd_valid, .rd_ready, .rd_addr, .rsp_valid, .rsp_data, .rsp_err,
    .done(wk_done), .res_fault(wk_fault), .res_level(wk_level), .res_pte(wk_pte),
    .res_sparse(wk_sparse), .res_size(wk_size), .res_pfn(wk_pfn)
  );

  // ---------------- translation result (comb) -------------------------------
  wire byp      = (q_vmid == 4'd0);
  wire noncanon = (|q_va[63:48]) || !ctx_enable[q_vmid];

  // source entry: TLB hit (LOOKUP) or walker result (WALK)
  logic        s_is_walk;
  logic [2:0]  s_fault;
  logic [1:0]  s_level;
  logic [63:0] s_pte;
  logic        s_sparse;
  logic [2:0]  s_size;
  logic [35:0] s_pfn;
  always_comb begin
    s_is_walk = (st == S_WALK);
    if (s_is_walk) begin
      s_fault = wk_fault; s_level = wk_level; s_pte = wk_pte;
      s_sparse = wk_sparse; s_size = wk_size; s_pfn = wk_pfn;
    end else begin
      s_fault = F_NONE; s_level = size_level(tl_size); s_pte = tl_pte;
      s_sparse = tl_sparse; s_size = tl_size; s_pfn = tl_pfn;
    end
  end

  // permission check + response fields
  logic [2:0]  o_fault;
  logic [1:0]  o_level;
  logic [63:0] o_pte;
  logic        o_sparse;
  logic [47:0] o_pa;
  logic [2:0]  o_size;
  logic        perm_ok;
  always_comb begin
    case (q_acc)
      ACC_R:   perm_ok = s_pte[2];
      ACC_W:   perm_ok = s_pte[3] && (!q_atomic || s_pte[2]);
      ACC_X:   perm_ok = s_pte[4];
      default: perm_ok = 1'b0;
    endcase
    o_fault  = s_fault;
    o_level  = s_level;
    o_pte    = s_pte;
    o_sparse = 1'b0;
    o_size   = s_size;
    o_pa     = '0;
    if (s_fault == F_NONE) begin
      if (s_sparse) begin
        o_sparse = 1'b1;
        if (q_acc != ACC_R && q_acc != ACC_W) o_fault = F_PERMISSION;   // EXEC (and reserved)
      end else begin
        o_pa = page_pa(s_pfn, q_va[47:0], s_size);
        if (!perm_ok) o_fault = F_PERMISSION;
      end
    end
  end

  // response registers
  logic [2:0]  r_fault;
  logic        r_sparse;
  logic [47:0] r_pa;
  logic [2:0]  r_size;
  logic [1:0]  r_level;
  logic [63:0] r_pte;
  logic        rpt;       // one-cycle fault report pulse (registered with the response)

  assign resp_valid  = (st == S_RESP);
  assign resp_fault  = r_fault;
  assign resp_sparse = r_sparse;
  assign resp_pa     = r_pa;
  assign resp_size   = r_size;
  assign resp_level  = r_level;
  assign resp_pte    = r_pte;
  assign resp_tag    = q_tag;

  assign req_ready = (st == S_IDLE) || (st == S_RESP && resp_ready);
  wire   accept    = req_valid && req_ready;

  // ---------------- control ---------------------------------------------------
  wire walk_inv = inv_pend[q_vmid];

  always_comb begin
    wk_start   = 1'b0;
    wk_abort   = 1'b0;
    fill_valid = 1'b0;
    if (st == S_LOOKUP && !byp && !noncanon && !tl_hit && wk_ready) wk_start = 1'b1;
    if (st == S_WALK) begin
      if (walk_inv) wk_abort = 1'b1;
      else if (wk_done && wk_fault == F_NONE) fill_valid = 1'b1;
    end
  end

  // response load: one comb decision, registered below
  logic        ld;
  logic [2:0]  ld_fault;
  logic        ld_sparse;
  logic [47:0] ld_pa;
  logic [2:0]  ld_size;
  logic [1:0]  ld_level;
  logic [63:0] ld_pte;
  logic        go_walk;
  always_comb begin
    ld = 1'b0; go_walk = 1'b0;
    ld_fault = o_fault; ld_sparse = o_sparse; ld_pa = o_pa;
    ld_size = o_size; ld_level = o_level; ld_pte = o_pte;
    if (st == S_LOOKUP) begin
      if (byp && (|q_va[63:48])) begin
        // bypass VA[63:48] != 0 -> NON_CANONICAL (48-bit PA bus, no truncation)
        ld = 1'b1; ld_fault = F_NON_CANONICAL; ld_sparse = 1'b0; ld_pa = 48'h0;
        ld_size = SZ_4K; ld_level = 2'd0; ld_pte = 64'h0;
      end else if (byp && !pa_decoded(q_va[47:0])) begin
        // bypass PA = VA must lie in VRAM / SYS / HOST; a hole is BAD_PHYS_ADDR (LEVEL 0, PTE 0)
        ld = 1'b1; ld_fault = F_BAD_PHYS; ld_sparse = 1'b0; ld_pa = 48'h0;
        ld_size = SZ_4K; ld_level = 2'd0; ld_pte = 64'h0;
      end else if (byp) begin
        ld = 1'b1; ld_fault = F_NONE; ld_sparse = 1'b0; ld_pa = q_va[47:0];
        ld_size = SZ_4K; ld_level = 2'd0; ld_pte = 64'h0;
      end else if (noncanon) begin
        ld = 1'b1; ld_fault = F_NON_CANONICAL; ld_sparse = 1'b0; ld_pa = 48'h0;
        ld_size = SZ_4K; ld_level = 2'd0; ld_pte = 64'h0;
      end else if (tl_hit) begin
        ld = 1'b1;
      end else if (wk_ready) begin
        go_walk = 1'b1;
      end
    end else if (st == S_WALK) begin
      if (!walk_inv && wk_done) ld = 1'b1;
    end
  end

  always_ff @(posedge clk) begin
    rpt <= 1'b0;
    if (!rst_n) begin
      st            <= S_IDLE;
      stat_walks    <= '0;
      stat_tlb_hits <= '0;
    end else begin
      if (st == S_LOOKUP && tl_hit && !byp && !noncanon) stat_tlb_hits <= stat_tlb_hits + 1;
      if (go_walk) stat_walks <= stat_walks + 1;
      if (ld) begin
        r_fault  <= ld_fault;
        r_sparse <= ld_sparse;
        r_pa     <= ld_pa;
        r_size   <= ld_size;
        r_level  <= ld_level;
        r_pte    <= ld_pte;
        rpt      <= (ld_fault != F_NONE);
        st       <= S_RESP;
      end else if (go_walk) begin
        st <= S_WALK;
      end else if (st == S_WALK && walk_inv) begin
        st <= S_LOOKUP;             // re-evaluate everything after the flush
      end else if (st == S_RESP && resp_ready) begin
        st <= S_IDLE;
      end
      if (accept) st <= S_LOOKUP;
    end
    if (accept) begin
      q_vmid   <= req_vmid;
      q_va     <= req_va;
      q_acc    <= req_acc;
      q_atomic <= req_atomic;
      q_tag    <= req_tag;
    end
  end

  // ---------------- fault latch feed ------------------------------------------
  always_comb begin
    if (rpt) begin
      flt_valid = 1'b1;
      flt_vmid  = q_vmid;
      flt_va    = q_va;
      flt_acc   = q_acc;       // atomics are ACC_W already
      flt_code  = r_fault;
      flt_level = r_level;
      flt_pte   = r_pte;
    end else begin
      flt_valid = ext_flt_valid;
      flt_vmid  = ext_flt_vmid;
      flt_va    = ext_flt_va;
      flt_acc   = ext_flt_acc;
      flt_code  = ext_flt_code;
      flt_level = ext_flt_level;
      flt_pte   = ext_flt_pte;
    end
  end

endmodule : ng_mmu
