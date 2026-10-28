// ng_sp_icache.sv -- SP L0 instruction cache (4 KiB set-associative default).
//
// LINES x 128 B lines in front of the scheduler's fetch port (ng_sp_sched if_*: one outstanding request,
// untagged response).  Two organisations, selected by WAYS (0 = auto):
//   * set-associative (default: LINES = 32, WAYS = 4 -> 8 sets x 4 ways, 4 KiB):
//     set = VA[7 +: log2(SETS)], tag = VA[48: 7 + log2(SETS)] (code VAs are < 2**49, see va0), valid bits
//     flip-flops (one-cycle invalidation), tags in one distributed RAM per way (read by set), line data in one
//     LINES x 1024 distributed RAM addressed {way, set} (whole-line write on a fill, asynchronous read of the hit
//     way's line, then the word mux), replacement tree pseudo-LRU per set (WAYS - 1 bits; updated on every lookup
//     hit and on every install; an invalid way is filled first).  WAYS = 1 is direct mapped.
//   * fully associative with a FIFO pointer (WAYS >= LINES; auto for LINES <= 8): the cache, register
//     arrays.  LINES = 8 (auto) is the I$ cycle for cycle (same state machine, same replacement).
// Both share the state machine below, so fetch timing does not depend on the organisation: hit 3 cycles (accept,
// lookup, respond); a 64-bit instruction adds one lookup cycle; a miss adds MMU + fabric + one lookup.
//
// A fetch returns {w1, w0}; w1 is only fetched when w0 encodes a 64-bit
// instruction (w0[1:0] == 2'b01), exactly like the functional model's
// run_warp: w0 is read first (fault info = VA of w0), then w1 (fault info =
// VA of w0 + 4), so a 64-bit instruction straddling a line or page boundary
// faults on the right word.  For a 32-bit instruction w1 is returned as 0.
//
// Misses use their own MMU client port (access = Exec) and their own
// line-granular fabric port (ng_lsu_pkg::mreq_t M_RD of one 128 B line; uc =
// PA outside VRAM) -- a dedicated fetch path, not the LSU (the LSU is a
// one-instruction-at-a-time pipe of hundreds of cycles and its L1D is a data
// cache).  ng_sp_top arbitrates both ports against the LSU.  Sparse-null code
// pages read as zeros (model: mmu.read of a sparse-null entry).
//
// Fetch errors after a successful translation -- the translated PA is outside every decoded region, or the fabric
// answers the line read with an error (rm_sp: no VRAM behind the PA) -- are reported on ext_flt_* (one cycle, in S_RSP,
// access = Exec, VA = the faulting word's VA) so that ng_sp_top can feed them to the MMU fault latch as BAD_PHYS_ADDR,
// exactly like the LSU's (ng_lsu ext_flt_*).  MMU-detected faults (resp_fault != 0) are not reported
// here: ng_mmu latched them itself when it answered the request.
//
// Invalidation: `inv` (dispatch start == implicit INV_I ,
// and the icache.inv instruction) clears every valid bit in one cycle.
//
// Counters (unchanged by): stat_hit counts w0 lookups that hit -- every fetch that is answered without a
// fault, including the lookup that follows a fill; stat_miss counts every miss (w0 or w1 lookup) = line fills
// attempted.  Fetches = stat_hit + (misses that faulted); fill ratio = stat_miss / stat_hit.
//
// line count / organisation, no
// prefetch, no MMU/TLB of its own (every miss translates).
module ng_sp_icache
  import ng_lsu_pkg::*;
#(
  parameter int LINES = 32,
  parameter int WAYS  = 0      // 0 = auto: LINES <= 8 -> LINES (fully associative FIFO), else 4
) (
  input  logic          clk,
  input  logic          rst_n,

  input  logic          inv,
  input  logic [47:0]   code_base,   // CODE_BASE of the running dispatch
  input  logic [3:0]    vmid,

  // ---- scheduler fetch port ----
  input  logic          req_valid,
  output logic          req_ready,
  input  logic [31:0]   req_pc,      // code offset
  output logic          rsp_valid,
  output logic          rsp_fault,
  output logic [63:0]   rsp_data,    // {w1, w0}
  output logic [31:0]   rsp_fault_info,  // model trapinfo: low 32 bits of the faulting word's VA

  // ---- MMU client (ng_mmu) ----
  output logic          mmu_req_valid,
  input  logic          mmu_req_ready,
  output logic [3:0]    mmu_req_vmid,
  output logic [63:0]   mmu_req_va,
  output logic [1:0]    mmu_req_acc,
  output logic          mmu_req_atomic,
  output logic [7:0]    mmu_req_tag,
  input  logic          mmu_resp_valid,
  output logic          mmu_resp_ready,
  input  logic [2:0]    mmu_resp_fault,
  input  logic          mmu_resp_sparse,
  input  logic [47:0]   mmu_resp_pa,

  // ---- external fault report (BAD_PHYS_ADDR after translation, see above); single-cycle pulse ----
  output logic          ext_flt_valid,
  output logic [3:0]    ext_flt_vmid,
  output logic [63:0]   ext_flt_va,
  output logic [1:0]    ext_flt_acc,

  // ---- fabric line port ----
  output logic          mem_req_valid,
  input  logic          mem_req_ready,
  output mreq_t         mem_req,
  input  logic          mem_rsp_valid,
  input  mrsp_t         mem_rsp,

  output logic          busy,
  output logic [31:0]   stat_hit,
  output logic [31:0]   stat_miss
);
  localparam int WE = (WAYS > 0) ? ((WAYS < LINES) ? WAYS : LINES) : ((LINES <= 8) ? LINES : 4);
  localparam bit FA = (WE >= LINES);

  typedef enum logic [2:0] {S_IDLE, S_LK, S_MMU, S_MMUW, S_MEM, S_MEMW, S_RSP} st_t;
  st_t st;

  logic [63:0]    va0;               // VA of w0 (< 2**49: 48-bit CODE_BASE + 32-bit offset)
  logic           ph;                // 0: looking up w0, 1: w1
  logic [31:0]    w0_q;
  logic [47:0]    fill_pa;           // line base PA of the pending fill
  logic [63:0]    rdata_q;
  logic           fault_q;
  logic           xflt_q;            // fault_q is a post-translation fault (BAD_PHYS_ADDR / fabric error): report on ext_flt_*
  logic [31:0]    finfo_q;

  wire  [63:0]    fva   = va0 + (ph ? 64'd4 : 64'd0);
  wire  [4:0]     fword = fva[6:2];

  // ---- storage interface (one of the two generate branches below) ----
  logic           hit;               // lookup of fva hits (combinational, valid in S_LK)
  logic [31:0]    hword;             // the word at fva on a hit
  logic           lk_hit;            // a lookup hit is being consumed this cycle (replacement state update)
  logic           ins;               // install the line of fva this cycle
  logic [LINE_BITS-1:0]  ins_data;

  assign lk_hit   = (st == S_LK) && hit;
  assign ins      = ((st == S_MMUW) && mmu_resp_valid && (mmu_resp_fault == 3'd0) && mmu_resp_sparse) ||
                    ((st == S_MEMW) && mem_rsp_valid && (mem_rsp.err == 2'd0));
  assign ins_data = (st == S_MEMW) ? mem_rsp.rdata : '0;      // sparse-null code pages read as zeros

  generate
    if (FA) begin : g_fa
      // ---------------------------------------------------------- fully associative, FIFO
      localparam int LW = ng_noc_pkg::clog2_1(LINES);
      logic [56:0]    tag_a  [LINES];   // VA[63:7]
      logic [LINE_BITS-1:0]  data_a [LINES];
      logic           val_a  [LINES];
      logic [LW-1:0]  vptr;
      wire  [56:0]    ftag = fva[63:7];

      always_comb begin
        hit   = 1'b0;
        hword = '0;
        for (int i = 0; i < LINES; i++)
          if (val_a[i] && tag_a[i] == ftag) begin
            hit   = 1'b1;
            hword = data_a[i][32*fword +: 32];
          end
      end

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          for (int i = 0; i < LINES; i++) begin val_a[i] <= 1'b0; tag_a[i] <= '0; end
          vptr <= '0;
        end else begin
          if (ins) begin
            val_a[vptr]  <= 1'b1;  tag_a[vptr] <= ftag;  data_a[vptr] <= ins_data;
            vptr <= (int'(vptr) == LINES - 1) ? '0 : vptr + 1'b1;
          end
          // invalidation wins over a same-cycle fill of the valid bits (a fill in
          // flight still installs its line afterwards: harmless, the code is the
          // same bytes the next miss would read)
          if (inv) for (int i = 0; i < LINES; i++) val_a[i] <= 1'b0;
        end
      end
    end else begin : g_sa
      // ---------------------------------------------------------- set associative, tree pseudo-LRU
      localparam int SETS = LINES / WE;
      localparam int SW   = ng_noc_pkg::clog2_1(SETS);
      localparam int WW   = ng_noc_pkg::clog2_1(WE);
      localparam int TW   = 42 - ((SETS > 1) ? SW : 0);       // VA[48: 7 + log2(SETS)]
      localparam int PW   = (WE > 1) ? WE - 1 : 1;             // tree bits per set (node n at bit n - 1)

      wire [SW-1:0]   fset = (SETS > 1) ? fva[7 +: SW] : '0;
      wire [TW-1:0]   ftag = fva[48 : 49 - TW];

      logic [WE-1:0]  val_q  [SETS];
      logic [PW-1:0]  plru_q [SETS];
      logic [TW-1:0]  rtag   [WE];                             // tag of each way of fset
      logic [WE-1:0]  wm;                                      // way match
      logic [WW-1:0]  hway, vway;
      logic [LINE_BITS-1:0]  data_m [LINES];                          // {way, set}: distributed RAM
      logic [LINE_BITS-1:0]  rline;

      for (genvar w = 0; w < WE; w++) begin : g_way
        logic [TW-1:0] tag_m [SETS];                           // distributed RAM, no reset (val_q qualifies)
        always_ff @(posedge clk) if (ins && vway == WW'(w)) tag_m[fset] <= ftag;
        assign rtag[w] = tag_m[fset];
        assign wm[w]   = val_q[fset][w] && (rtag[w] == ftag);
      end

      always_comb begin
        hway = '0;
        for (int w = 0; w < WE; w++) if (wm[w]) hway = WW'(w);
      end
      assign hit = |wm;

      localparam int LAW = $clog2(LINES);                     // {way, set} (SETS > 1 here; WAYS = 1: set only)
      wire [LAW-1:0]  raddr = LAW'({hway, fset});
      wire [LAW-1:0]  waddr = LAW'({vway, fset});
      always_ff @(posedge clk) if (ins) data_m[waddr] <= ins_data;
      assign rline = data_m[raddr];
      assign hword = rline[32*fword +: 32];

      // victim: the lowest invalid way, else the tree's pseudo-LRU way (bit 1 at a node: victim on the right)
      function automatic logic [WW-1:0] tree_victim(input logic [PW-1:0] t);
        int node, lo, half;
        node = 1; lo = 0;
        for (int l = 0; l < WW; l++) begin
          half = WE >> (l + 1);
          if (t[node - 1]) begin node = 2 * node + 1; lo = lo + half; end
          else node = 2 * node;
        end
        return WW'(lo);
      endfunction
      // point every node on the path to way w away from it
      function automatic logic [PW-1:0] tree_touch(input logic [PW-1:0] t, input logic [WW-1:0] w);
        int node, lo, half;
        logic [PW-1:0] r;
        r = t; node = 1; lo = 0;
        for (int l = 0; l < WW; l++) begin
          half = WE >> (l + 1);
          if (int'(w) < lo + half) begin r[node - 1] = 1'b1; node = 2 * node; end
          else begin r[node - 1] = 1'b0; node = 2 * node + 1; lo = lo + half; end
        end
        return r;
      endfunction

      always_comb begin
        vway = (WE > 1) ? tree_victim(plru_q[fset]) : '0;
        for (int w = WE - 1; w >= 0; w--) if (!val_q[fset][w]) vway = WW'(w);
      end

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          for (int s = 0; s < SETS; s++) begin val_q[s] <= '0; plru_q[s] <= '0; end
        end else begin
          if (WE > 1) begin
            if (lk_hit)   plru_q[fset] <= tree_touch(plru_q[fset], hway);
            else if (ins) plru_q[fset] <= tree_touch(plru_q[fset], vway);
          end
          if (ins) val_q[fset][vway] <= 1'b1;
          // invalidation wins over a same-cycle fill (see g_fa)
          if (inv) for (int s = 0; s < SETS; s++) val_q[s] <= '0;
        end
      end
    end
  endgenerate

  assign req_ready      = (st == S_IDLE);
  assign rsp_valid      = (st == S_RSP);
  assign rsp_fault      = fault_q;
  assign rsp_data       = rdata_q;
  assign rsp_fault_info = finfo_q;
  assign busy           = (st != S_IDLE);

  assign ext_flt_valid  = (st == S_RSP) && xflt_q;
  assign ext_flt_vmid   = vmid;
  assign ext_flt_va     = fva;
  assign ext_flt_acc    = 2'd2;        // Exec

  assign mmu_req_valid  = (st == S_MMU);
  assign mmu_req_vmid   = vmid;
  assign mmu_req_va     = fva;
  assign mmu_req_acc    = 2'd2;        // Exec
  assign mmu_req_atomic = 1'b0;
  assign mmu_req_tag    = 8'hc1;      // opaque (ng_sp_top routes by owner, not by tag)
  assign mmu_resp_ready = (st == S_MMUW);

  always_comb begin
    mem_req          = '0;
    mem_req.kind     = M_RD;
    mem_req.pa       = fill_pa;
    mem_req.uc       = !pa_is_vram(fill_pa);
    mem_req.pol      = 2'd0;
    mem_req.noalloc  = 1'b0;
  end
  assign mem_req_valid = (st == S_MEM);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE;
      va0 <= '0; ph <= 1'b0; w0_q <= '0; fill_pa <= '0;
      rdata_q <= '0; fault_q <= 1'b0; xflt_q <= 1'b0; finfo_q <= '0;
      stat_hit <= '0; stat_miss <= '0;
    end else begin
      case (st)
        S_IDLE: begin
          if (req_valid) begin
            va0     <= {16'd0, code_base} + {32'd0, req_pc};
            ph      <= 1'b0;
            fault_q <= 1'b0;
            xflt_q  <= 1'b0;
            finfo_q <= '0;
            st      <= S_LK;
          end
        end
        S_LK: begin
          if (hit) begin
            if (!ph) stat_hit <= stat_hit + 1;
            if (!ph && hword[1:0] == 2'b01) begin
              w0_q <= hword;
              ph   <= 1'b1;             // look up w1 next cycle
            end else begin
              rdata_q <= ph ? {hword, w0_q} : {32'd0, hword};
              st      <= S_RSP;
            end
          end else begin
            stat_miss <= stat_miss + 1;
            st        <= S_MMU;
          end
        end
        S_MMU: if (mmu_req_ready) st <= S_MMUW;
        S_MMUW: begin
          if (mmu_resp_valid) begin
            if (mmu_resp_fault != 3'd0) begin
              fault_q <= 1'b1; finfo_q <= fva[31:0]; st <= S_RSP;
            end else if (mmu_resp_sparse) begin
              st <= S_LK;                                         // installed as zeros (ins)
            end else if (!pa_valid({mmu_resp_pa[47:12], fva[11:0]})) begin
              fault_q <= 1'b1; xflt_q <= 1'b1; finfo_q <= fva[31:0]; st <= S_RSP;   // BAD_PHYS_ADDR
            end else begin
              fill_pa <= {mmu_resp_pa[47:12], fva[11:7], 7'd0};
              st      <= S_MEM;
            end
          end
        end
        S_MEM: if (mem_req_ready) st <= S_MEMW;
        S_MEMW: begin
          if (mem_rsp_valid) begin
            if (mem_rsp.err != 2'd0) begin
              fault_q <= 1'b1; xflt_q <= 1'b1; finfo_q <= fva[31:0]; st <= S_RSP;
            end else begin
              st <= S_LK;                                         // installed (ins)
            end
          end
        end
        S_RSP: st <= S_IDLE;
        default: st <= S_IDLE;
      endcase
    end
  end

`ifdef NG_SP_ASSERT
  initial begin
    if (!FA && (LINES % WE != 0 || (LINES / WE) & (LINES / WE - 1) || WE & (WE - 1)))
      $fatal(1, "ng_sp_icache: LINES / WAYS must be powers of two (LINES %0d WAYS %0d)", LINES, WE);
  end
`endif
endmodule
