// ng_lsu_top_stub.sv -- OOC-synthesis stand-in for ng_lsu_top (same name, ports and parameters).
// All inputs are folded into one registered parity bit (so combinational paths INTO the LSU in the
// SP top are timed against a flop); every output comes straight from a register (paths OUT of the
// LSU start at a flop).  Not functional; only used by rtl/gpu/sp/ooc_sp_top.tcl.
module ng_lsu_top
  import ng_lsu_pkg::*;
#(
  parameter int NL    = 16,
  parameter int SLOTW = 4,
  parameter int SH_AW = 16,
  parameter int L1_SETS = 256,
  parameter int L1_WAYS = 4,
  parameter int LSU_K   = 1,          // (interface only)
  parameter int MEM_OUT = 1,
  localparam int MSW    = ng_noc_pkg::clog2_1(MEM_OUT)
) (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              inv_l1_ext,       // (interface only)
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
  output logic                      mem_req_valid,
  input  logic                      mem_req_ready,
  output mreq_t                     mem_req,
  input  logic                      mem_rsp_valid,
  input  mrsp_t                     mem_rsp,
  output logic                      u_req_valid,
  input  logic                      u_req_ready,
  output mreq_t                     u_req,
  input  logic [MSW-1:0]            u_req_slot,
  input  logic                      u_rsp_valid,
  input  logic [MSW-1:0]            u_rsp_slot,
  input  logic [1:0]                u_rsp_err,
  output logic                      u_rd_req,
  output logic [MSW-1:0]            u_rd_slot,
  input  logic                      u_rd_gnt,
  input  logic [LINE_BITS-1:0]      u_rd_data,
  output logic                      u_free,
  output logic [31:0]               stat_l1_rd,
  output logic [31:0]               stat_l1_wr,
  output logic [31:0]               stat_l1_amo,
  output logic [31:0]               stat_sh_rounds,
  output logic [31:0]               stat_instr_mem,
  output logic [31:0]               stat_groups,
  output logic [351:0]              stat_k,
  output logic [31:0]               stat_hit,
  output logic [31:0]               stat_miss,
  output logic [31:0]               stat_fill,
  output logic [31:0]               stat_inv
);
  localparam int BW = (NL < 32 ? $clog2(32 / NL) : 1);
  localparam int OUT_W = 1+1+16+1+4+32+1 + 1+SLOTW+3 + 1+SLOTW+3 + 1+SLOTW+7+BW+2
                       + 1+SLOTW+7+BW+NL+2+NL*32 + 1+SLOTW+6 + 1+SLOTW+6+32
                       + 1+4+64+2+8+1 + 1+1+4+64+2 + 1+$bits(mreq_t) + 320
                       + 1+$bits(mreq_t)+1+MSW+1 + 352;
  logic par_q;
  logic [OUT_W-1:0] out_q;
  logic par_d;
  assign par_d = ^{inv_l1_ext, sh_clr_en, sh_clr_row, iss_valid, iss_w0, iss_w1, iss_exec, iss_slot, iss_nregs, iss_tag, iss_ctx,
                   rd_ready, rdv_valid, rdv_err, rdv_data, wr_ready, urd_valid, urd_data,
                   mmu_req_ready, mmu_resp_valid, mmu_resp_fault, mmu_resp_sparse, mmu_resp_pa,
                   mmu_resp_pte, mem_req_ready, mem_rsp_valid, mem_rsp, rst_n,
                   u_req_ready, u_req_slot, u_rsp_valid, u_rsp_slot, u_rsp_err, u_rd_gnt, u_rd_data};
  always_ff @(posedge clk) begin
    par_q <= par_d;
    out_q <= {out_q[OUT_W-2:0], par_q} ^ {out_q[0], out_q[OUT_W-1:1]};
  end
  assign {iss_ready, done_valid, done_tag, trap_valid, trap_cause, trap_info, busy,
          cnt_inc_valid, cnt_inc_slot, cnt_inc_id, cnt_dec_valid, cnt_dec_slot, cnt_dec_id,
          rd_valid, rd_slot, rd_reg, rd_beat, rd_hsel,
          wr_valid, wr_slot, wr_reg, wr_beat, wr_lanes, wr_hsel, wr_data,
          ur_valid, ur_slot, ur_reg, uw_valid, uw_slot, uw_reg, uw_data,
          mmu_req_valid, mmu_req_vmid, mmu_req_va, mmu_req_acc, mmu_req_tag, mmu_req_atomic,
          mmu_resp_ready, ext_flt_valid, ext_flt_vmid, ext_flt_va, ext_flt_acc,
          mem_req_valid, mem_req,
          stat_l1_rd, stat_l1_wr, stat_l1_amo, stat_sh_rounds, stat_instr_mem, stat_groups,
          stat_hit, stat_miss, stat_fill, stat_inv,
          u_req_valid, u_req, u_rd_req, u_rd_slot, u_free, stat_k}
         = out_q;
endmodule
