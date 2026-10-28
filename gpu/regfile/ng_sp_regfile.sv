// ng_sp_regfile.sv -- SP register file, first cut.
//
// One SP's vector (vreg) and uniform (ureg) register files behind a
// port-arbitrated interface for the operand-read / writeback stages that
//..f will add.
//
// Vector file: 128 KiB per SP = VREGS (1024)
// warp-registers of 32 lanes x 32 bit, dynamically allocated to the 16 warp
// slots (per-slot base + size, num_vregs <= 128 per thread).  The L
// configuration has NL = 16 physical lanes, so one warp-register is two beats
// of 16 lanes; XL is NL = 32 (one beat).  Storage is NBANK (4) banks, each
// one read port + one write port (simple dual port, BRAM-mappable, byte
// write enable):
//     phys = base[slot] + reg;  bank = phys % NBANK;
//     entry = (phys / NBANK) * NBEAT + beat.
// Consecutive registers therefore fall into different banks; a warp-register
// is always inside one bank.
//
// Ports (all requests are level valid/ready; ready may depend on valid):
//   * NRP (3) vector read ports (rs1/rs2/rs3): one beat per request.  Bank
//     conflict = two requests to the same bank in one cycle; the lowest port
//     index wins, the others see ready=0 and must hold.  Read data returns
//     exactly one cycle after the grant (rdv_valid) and is write-first
//     bypassed: a write granted in the same cycle to the same bank+entry is
//     visible to the read.
//   * NWP (2) vector write ports: lane mask + half select (16-bit half
//     registers, `v.l`/`v.h`: only the selected half of each dword changes).
//     Same-bank writes in one cycle serialise (lowest port first).
//   * NURP (3) uniform read ports, NUWP (2) uniform write ports: 64 x 32 bit
//     per warp; no bank conflicts; same-cycle same-register writes resolve to
//     the highest port; reads are write-first bypassed; latency 1.
//   * range checks against the slot's allocation: a vector access with
//     reg >= num_vregs (or to an unallocated slot) is accepted, dropped, and
//     reported as an error one cycle later (the model raises TrapCause
//     RegRange; the pipe converts the error into a precise trap).
//
// storage is not cleared on allocation;
// one register per request (multi-register pair/b128 accesses are sequenced
// by the caller); half select = lo/hi (not an arbitrary bit mask); fixed
// port priority; uniform file = LVT + replicated block RAM.
module ng_sp_regfile #(
  parameter int NG_RF_NUM_WARPS = 16,    // warp slots per SP
  parameter int NG_RF_NL        = 16,    // physical lanes per beat (16 = L, 32 = XL)
  parameter int NG_RF_VREGS     = 1024,  // warp-registers per SP (128 KiB / 128 B)
  parameter int NG_RF_NBANK     = 4,     // vector banks (power of two)
  parameter int NG_RF_NUREG     = 64,    // uniform registers per warp (power of two)
  parameter int NG_RF_NRP       = 3,     // vector read ports
  parameter int NG_RF_NWP       = 2,     // vector write ports
  parameter int NG_RF_NURP      = 3,     // uniform read ports
  parameter int NG_RF_NUWP      = 2      // uniform write ports
) (
  input  logic clk,
  input  logic rst_n,

  // ---- allocation (work distributor / scheduler launch side) --------------
  // alloc_nregs = 0 releases the slot.  base + nregs must fit VREGS and
  // nregs <= 128, otherwise alloc_err pulses next cycle and nothing changes.
  // Do not access the slot in the cycle of its (re)allocation.
  input  logic                                   alloc_valid,
  input  logic [$clog2(NG_RF_NUM_WARPS)-1:0]     alloc_slot,
  input  logic [$clog2(NG_RF_VREGS)-1:0]         alloc_base,
  input  logic [7:0]                             alloc_nregs,
  output logic                                   alloc_err,

  // ---- vector read ports ---------------------------------------------------
  input  logic [NG_RF_NRP-1:0]                   rd_valid,
  output logic [NG_RF_NRP-1:0]                   rd_ready,
  input  logic [$clog2(NG_RF_NUM_WARPS)-1:0]     rd_slot [NG_RF_NRP],
  input  logic [6:0]                             rd_reg  [NG_RF_NRP],
  input  logic [(NG_RF_NL < 32 ? $clog2(32 / NG_RF_NL) : 1)-1:0] rd_beat [NG_RF_NRP],
  input  logic [1:0]                             rd_hsel [NG_RF_NRP],  // 0 full, 1 lo -> [15:0], 2 hi -> [15:0]
  output logic [NG_RF_NRP-1:0]                   rdv_valid,   // response, 1 cycle after grant
  output logic [NG_RF_NRP-1:0]                   rdv_err,     // range error (data = 0)
  output logic [NG_RF_NL*32-1:0]                 rdv_data [NG_RF_NRP],
  output logic                                   perf_rd_conflict,  // some request lost a bank

  // ---- vector write ports --------------------------------------------------
  input  logic [NG_RF_NWP-1:0]                   wr_valid,
  output logic [NG_RF_NWP-1:0]                   wr_ready,
  input  logic [$clog2(NG_RF_NUM_WARPS)-1:0]     wr_slot [NG_RF_NWP],
  input  logic [6:0]                             wr_reg  [NG_RF_NWP],
  input  logic [(NG_RF_NL < 32 ? $clog2(32 / NG_RF_NL) : 1)-1:0] wr_beat [NG_RF_NWP],
  input  logic [NG_RF_NL-1:0]                    wr_lanes [NG_RF_NWP],
  input  logic [1:0]                             wr_hsel [NG_RF_NWP],  // bit0 write low half, bit1 high half
  input  logic [NG_RF_NL*32-1:0]                 wr_data [NG_RF_NWP],
  output logic [NG_RF_NWP-1:0]                   wr_err,      // pulse, 1 cycle after a dropped write

  // ---- uniform ports -------------------------------------------------------
  input  logic [NG_RF_NURP-1:0]                  ur_valid,
  input  logic [$clog2(NG_RF_NUM_WARPS)-1:0]     ur_slot [NG_RF_NURP],
  input  logic [$clog2(NG_RF_NUREG)-1:0]         ur_reg  [NG_RF_NURP],
  output logic [NG_RF_NURP-1:0]                  urd_valid,
  output logic [31:0]                            urd_data [NG_RF_NURP],

  input  logic [NG_RF_NUWP-1:0]                  uw_valid,
  input  logic [$clog2(NG_RF_NUM_WARPS)-1:0]     uw_slot [NG_RF_NUWP],
  input  logic [$clog2(NG_RF_NUREG)-1:0]         uw_reg  [NG_RF_NUWP],
  input  logic [31:0]                            uw_data [NG_RF_NUWP]
);
  localparam int NW    = NG_RF_NUM_WARPS;
  localparam int SW    = $clog2(NW);
  localparam int NL    = NG_RF_NL;
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int NB    = NG_RF_NBANK;
  localparam int BKW   = $clog2(NB);
  localparam int PW    = $clog2(NG_RF_VREGS);
  localparam int DEPTH = (NG_RF_VREGS / NB) * NBEAT;
  localparam int AW    = $clog2(DEPTH);
  localparam int DW    = NL * 32;
  localparam int BEW   = NL * 4;
  localparam int NRP   = NG_RF_NRP;
  localparam int NWP   = NG_RF_NWP;
  localparam int URW   = $clog2(NG_RF_NUREG);

  // ---------------------------------------------------------------- allocation table
  logic [PW-1:0] a_base [NW];
  logic [7:0]    a_n    [NW];   // 0 = unallocated
  logic          alloc_err_r;
  assign alloc_err = alloc_err_r;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int s = 0; s < NW; s++) begin
        a_base[s] <= '0;
        a_n[s]    <= '0;
      end
      alloc_err_r <= 1'b0;
    end else begin
      alloc_err_r <= 1'b0;
      if (alloc_valid) begin
        if (alloc_nregs > 8'd128 || (int'(alloc_base) + int'(alloc_nregs) > NG_RF_VREGS)) begin
          alloc_err_r <= 1'b1;
        end else begin
          a_base[alloc_slot] <= alloc_base;
          a_n[alloc_slot]    <= alloc_nregs;
        end
      end
    end
  end

  // ---------------------------------------------------------------- address decode
  function automatic logic [AW-1:0] entry_of(input logic [PW-1:0] phys, input logic [BTW-1:0] beat);
    int e;
    e = int'(phys >> BKW) * NBEAT + ((NBEAT > 1) ? int'(beat) : 0);
    return AW'(e);
  endfunction

  logic [PW-1:0]  r_phys [NRP];
  logic [BKW-1:0] r_bank [NRP];
  logic [AW-1:0]  r_ent  [NRP];
  logic [NRP-1:0] r_bad, r_bankreq, r_gnt;
  logic [PW-1:0]  w_phys [NWP];
  logic [BKW-1:0] w_bank [NWP];
  logic [AW-1:0]  w_ent  [NWP];
  logic [NWP-1:0] w_bad, w_bankreq, w_gnt;
  logic [BEW-1:0] w_be   [NWP];

  always_comb begin
    for (int p = 0; p < NRP; p++) begin
      r_phys[p] = a_base[rd_slot[p]] + PW'(rd_reg[p]);
      r_bank[p] = r_phys[p][BKW-1:0];
      r_ent[p]  = entry_of(r_phys[p], rd_beat[p]);
      r_bad[p]  = rd_valid[p] && (int'(rd_reg[p]) >= int'(a_n[rd_slot[p]]));
      r_bankreq[p] = rd_valid[p] && !r_bad[p];
    end
    for (int p = 0; p < NWP; p++) begin
      w_phys[p] = a_base[wr_slot[p]] + PW'(wr_reg[p]);
      w_bank[p] = w_phys[p][BKW-1:0];
      w_ent[p]  = entry_of(w_phys[p], wr_beat[p]);
      w_bad[p]  = wr_valid[p] && (int'(wr_reg[p]) >= int'(a_n[wr_slot[p]]));
      w_bankreq[p] = wr_valid[p] && !w_bad[p];
      for (int l = 0; l < NL; l++)
        w_be[p][l*4 +: 4] = {4{wr_lanes[p][l]}} & {{2{wr_hsel[p][1]}}, {2{wr_hsel[p][0]}}};
    end
  end

  // Bank arbitration: the lowest-index requester of a bank wins.  A range
  // error never needs a bank and is always accepted.
  always_comb begin
    for (int p = 0; p < NRP; p++) begin
      r_gnt[p] = r_bankreq[p];
      for (int q = 0; q < p; q++)
        if (r_bankreq[q] && r_bank[q] == r_bank[p]) r_gnt[p] = 1'b0;
    end
    for (int p = 0; p < NWP; p++) begin
      w_gnt[p] = w_bankreq[p];
      for (int q = 0; q < p; q++)
        if (w_bankreq[q] && w_bank[q] == w_bank[p]) w_gnt[p] = 1'b0;
    end
    rd_ready = r_gnt | r_bad;
    wr_ready = w_gnt | w_bad;
    perf_rd_conflict = |(r_bankreq & ~r_gnt);
  end

  // --------------------------------------------------------------
  // Each bank is ONE simple-dual-port block RAM array (1 write + 1 read per
  // cycle; the bank arbiter above already guarantees at most one winner per
  // bank for reads and for writes), byte write enables, synchronous read with
  // enable.  The registered write-first bypass of the old implementation is
  // reproduced outside the RAM: when the read is granted in the same cycle as
  // a write to the same entry, the write data/byte enables are captured next
  // to the RAM output and merged on the way out (BRAM same-address collision
  // data is therefore never used).  Visible behaviour is cycle-identical.
  logic [DW-1:0] bank_rd [NB];  // bank read data (RAM output merged with bypass)

  for (genvar b = 0; b < NB; b++) begin : g_bank
    (* ram_style = "block" *) logic [DW-1:0] mem [DEPTH];

    // Winning requesters of this bank (at most one each).
    logic          r_hit, w_hit;
    logic [AW-1:0] r_a, w_a;
    logic [DW-1:0] w_d;
    logic [BEW-1:0] w_e;
    always_comb begin
      r_hit = 1'b0; r_a = '0;
      w_hit = 1'b0; w_a = '0; w_d = '0; w_e = '0;
      for (int p = NRP - 1; p >= 0; p--)
        if (r_gnt[p] && r_bank[p] == BKW'(b)) begin r_hit = 1'b1; r_a = r_ent[p]; end
      for (int p = NWP - 1; p >= 0; p--)
        if (w_gnt[p] && w_bank[p] == BKW'(b)) begin
          w_hit = 1'b1; w_a = w_ent[p]; w_d = wr_data[p]; w_e = w_be[p];
        end
    end

    always_ff @(posedge clk) begin
      if (w_hit)
        for (int i = 0; i < BEW; i++)
          if (w_e[i]) mem[w_a][i*8 +: 8] <= w_d[i*8 +: 8];
    end

    logic [DW-1:0] ram_q;
    always_ff @(posedge clk) begin
      if (r_hit) ram_q <= mem[r_a];
    end

    // Bypass capture (only meaningful together with a granted read).
    logic [DW-1:0]  by_d;
    logic [BEW-1:0] by_e = '0;
    always_ff @(posedge clk) begin
      if (r_hit) begin
        by_d <= w_d;
        by_e <= (w_hit && r_a == w_a) ? w_e : '0;
      end
    end

    always_comb begin
      bank_rd[b] = ram_q;
      for (int i = 0; i < BEW; i++)
        if (by_e[i]) bank_rd[b][i*8 +: 8] = by_d[i*8 +: 8];
    end
  end

  // ---------------------------------------------------------------- read response
  logic [NRP-1:0]  rq_ok, rq_err;
  logic [BKW-1:0]  rq_bank [NRP];
  logic [1:0]      rq_hsel [NRP];
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rq_ok  <= '0;
      rq_err <= '0;
      wr_err <= '0;
      for (int p = 0; p < NRP; p++) begin rq_bank[p] <= '0; rq_hsel[p] <= '0; end
    end else begin
      rq_ok  <= r_gnt;
      rq_err <= r_bad;
      wr_err <= w_bad;
      for (int p = 0; p < NRP; p++) begin
        rq_bank[p] <= r_bank[p];
        rq_hsel[p] <= rd_hsel[p];
      end
    end
  end

  always_comb begin
    for (int p = 0; p < NRP; p++) begin
      logic [DW-1:0] d;
      d = bank_rd[rq_bank[p]];
      rdv_data[p] = '0;
      if (rq_ok[p]) begin
        for (int l = 0; l < NL; l++) begin
          case (rq_hsel[p])
            2'd1:    rdv_data[p][l*32 +: 32] = {16'd0, d[l*32 +: 16]};
            2'd2:    rdv_data[p][l*32 +: 32] = {16'd0, d[l*32 + 16 +: 16]};
            default: rdv_data[p][l*32 +: 32] = d[l*32 +: 32];
          endcase
        end
      end
    end
    rdv_valid = rq_ok | rq_err;
    rdv_err   = rq_err;
  end

  // ---------------------------------------------------------------- uniform file
  // Block-RAM implementation with a live-value table (LVT): one 1024 x 32 RAM
  // per (write port, read port) pair, every write port writes its own row of
  // copies, a flop LVT remembers which write port wrote each register last
  // (same-cycle collision: highest port wins, as before) and the read selects
  // that port's copy.  A write in the same cycle as the read bypasses (highest
  // matching port); the bypass data and the LVT choice are registered next to
  // the RAM output.  Outputs hold their value while ur_valid is low.
  localparam int NURPL = NG_RF_NURP;
  localparam int NUWPL = NG_RF_NUWP;
  localparam int UAW   = SW + URW;
  localparam int LW    = ng_noc_pkg::clog2_1(NUWPL);

  logic [LW-1:0] lvt [1 << UAW];
  initial for (int i = 0; i < (1 << UAW); i++) lvt[i] = '0;
  always_ff @(posedge clk) begin
    for (int q = 0; q < NUWPL; q++)
      if (uw_valid[q]) lvt[{uw_slot[q], uw_reg[q]}] <= LW'(q);
  end

  logic [31:0] uq_flat [NUWPL * NURPL];
  for (genvar q = 0; q < NUWPL; q++) begin : g_uw
    for (genvar p = 0; p < NURPL; p++) begin : g_ur
      (* ram_style = "block" *) logic [31:0] um [1 << UAW];
      always_ff @(posedge clk) begin
        if (uw_valid[q]) um[{uw_slot[q], uw_reg[q]}] <= uw_data[q];
      end
      logic [31:0] uq;
      always_ff @(posedge clk) begin
        if (ur_valid[p]) uq <= um[{ur_slot[p], ur_reg[p]}];
      end
      assign uq_flat[q * NURPL + p] = uq;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      urd_valid <= '0;
    end else begin
      urd_valid <= ur_valid;
    end
  end

  logic [LW-1:0]  u_sel   [NURPL];
  logic           u_byhit [NURPL];
  logic [31:0]    u_byd   [NURPL];
  always_ff @(posedge clk) begin
    for (int p = 0; p < NURPL; p++) begin
      if (ur_valid[p]) begin
        u_sel[p]   <= lvt[{ur_slot[p], ur_reg[p]}];
        u_byhit[p] <= 1'b0;
        for (int q = 0; q < NUWPL; q++)
          if (uw_valid[q] && uw_slot[q] == ur_slot[p] && uw_reg[q] == ur_reg[p]) begin
            u_byhit[p] <= 1'b1;
            u_byd[p]   <= uw_data[q];
          end
      end
    end
  end

  always_comb begin
    for (int p = 0; p < NURPL; p++) begin
      urd_data[p] = uq_flat[0 * NURPL + p];
      for (int q = 1; q < NUWPL; q++)
        if (u_sel[p] == LW'(q)) urd_data[p] = uq_flat[q * NURPL + p];
      if (u_byhit[p]) urd_data[p] = u_byd[p];
    end
  end
endmodule
