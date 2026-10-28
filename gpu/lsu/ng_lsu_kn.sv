// ng_lsu_kn.sv -- SP load/store unit with cross-instruction concurrency (LSU_K >= 2).
//
// Same instruction semantics, ports and statistics as ng_lsu (LSU_K = 1, one instruction in flight); front end /
// K-entry ring / IOF / back end.  The implementation is ng_lsu_core K >= 2
// ng_lsu.sv (shared with ng_lsu, see the description there); this module adds the uc channel and stat_kn ports.
//
//
module ng_lsu_kn
  import ng_lsu_pkg::*;
#(
  parameter int NL      = 16,
  parameter int SLOTW   = 4,
  parameter int SH_AW   = 16,
  parameter int SH_LPB  = 8,
  parameter int L1_LPB  = 4,
  parameter int K       = 2,                          // ring entries (2, 4, 8)
  parameter int MEM_OUT = 8,                          // fabric slots of the uc channel (1, 2, 4, 8, 16)
  parameter int NQ      = (MEM_OUT < 1) ? 2 : 2 * MEM_OUT,  // IOF entries (power of two)
  localparam int MSW    = ng_noc_pkg::clog2_1(MEM_OUT)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- shared-memory scrub (ng_wd, between dispatches) ----
  input  logic              sh_clr_en,
  input  logic [SH_AW-7-1:0] sh_clr_row,

  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,
  input  logic [31:0]       iss_w1,
  input  logic [31:0]       iss_exec,
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,
  input  logic [15:0]       iss_tag,
  input  ctx_t              iss_ctx,

  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              busy,

  output logic              cnt_inc_valid,
  output logic [SLOTW-1:0]  cnt_inc_slot,
  output logic [2:0]        cnt_inc_id,
  output logic              cnt_dec_valid,
  output logic [SLOTW-1:0]  cnt_dec_slot,
  output logic [2:0]        cnt_dec_id,

  output logic                      rd_valid,
  input  logic                      rd_ready,
  output logic [SLOTW-1:0]          rd_slot,
  output logic [6:0]                rd_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat,
  output logic [1:0]                rd_hsel,
  input  logic                      rdv_valid,
  input  logic                      rdv_err,
  input  logic [NL*32-1:0]          rdv_data,

  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  output logic                      ur_valid,
  output logic [SLOTW-1:0]          ur_slot,
  output logic [5:0]                ur_reg,
  input  logic                      urd_valid,
  input  logic [31:0]               urd_data,

  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data,

  output logic                      mmu_req_valid,
  input  logic                      mmu_req_ready,
  output logic [3:0]                mmu_req_vmid,
  output logic [63:0]               mmu_req_va,
  output logic [1:0]                mmu_req_acc,
  output logic                      mmu_req_atomic,
  output logic [7:0]                mmu_req_tag,
  input  logic                      mmu_resp_valid,
  output logic                      mmu_resp_ready,
  input  logic [2:0]                mmu_resp_fault,
  input  logic                      mmu_resp_sparse,
  input  logic [47:0]               mmu_resp_pa,
  input  logic [63:0]               mmu_resp_pte,

  output logic                      ext_flt_valid,
  output logic [3:0]                ext_flt_vmid,
  output logic [63:0]               ext_flt_va,
  output logic [1:0]                ext_flt_acc,

  // ---- L1D request port (uc = 0 line requests, synchronous-class requests) ----
  output logic                      l1_req_valid,
  input  logic                      l1_req_ready,
  output mreq_t                     l1_req,
  input  logic                      l1_rsp_valid,
  input  mrsp_t                     l1_rsp,

  // ---- uc channel: slot protocol (ng_rsp_fab_ts, through ng_sp_top's mport) ----
  output logic                      u_req_valid,
  input  logic                      u_req_ready,
  output mreq_t                     u_req,
  input  logic [MSW-1:0]            u_req_slot,     // slot the request takes (valid with u_req_ready)
  input  logic                      u_rsp_valid,    // answer of one of OUR slots (one-clock pulse)
  input  logic [MSW-1:0]            u_rsp_slot,
  input  logic [1:0]                u_rsp_err,
  output logic                      u_rd_req,       // the BE wants to read the line of u_rd_slot this clock
  output logic [MSW-1:0]            u_rd_slot,
  input  logic                      u_rd_gnt,       // read port free this clock (not used by an I$ / L1D answer)
  input  logic [LINE_BITS-1:0]      u_rd_data,
  output logic                      u_free,         // free u_rd_slot (only with u_rd_gnt)

  output logic [31:0]               stat_l1_rd,
  output logic [31:0]               stat_l1_wr,
  output logic [31:0]               stat_l1_amo,
  output logic [31:0]               stat_sh_rounds,
  output logic [31:0]               stat_instr_mem,
  output logic [31:0]               stat_groups,
  // debug counters (RM debug page 0x980..; word layout: ng_lsu_core)
  output logic [319:0]              stat_kn
);
  if (K < 2 || (K & (K - 1)) != 0) begin : g_bad_k
    $error("ng_lsu_kn: K must be a power of two >= 2 (LSU_K = 1 is ng_lsu)");
  end

  ng_lsu_core #(.NL(NL), .SLOTW(SLOTW), .SH_AW(SH_AW), .SH_LPB(SH_LPB), .L1_LPB(L1_LPB), .K(K), .MEM_OUT(MEM_OUT),
                .NQ(NQ)) u_core (.*);
endmodule
