// ng_cp_memadapt.sv -- INTERFACE SKELETON (not integrated, not simulated):
// adapter between the CP memory request port (ng_cp_pkg::mem_req_t, one
// outstanding) and the MMU (ng_mmu client port) / NoC->L2 (ng_noc_ep agent
// streams) / C3 SYS-HOST path.  Design: section "Memory adapter".
//
//   CP --m_req--> [ xlate ] --(vmid!=0)--> MMU req/resp (tag 0)        faults -> MMU latch
//                    |  \--(vmid==0 or after resp)--> [ route by PA ]
//                    |                                  |- VRAM  -> NoC req flit (KIND_RD/WR, SZ_8, be)
//                    |                                  |            -> L2 slice (ng_l2_addrmap node map)
//                    |                                  |- SYS/HOST (C3) -> c3_* port (physical, not via L2)
//                    |                                  `- else -> ext_flt BAD_PHYS_ADDR, m_rsp_fault
//                    `--> m_rsp (data, fault) after the last response flit
// The adapter is a placeholder: all outputs are tied off so the file lints
// and documents the port list that the integration task must implement.
module ng_cp_memadapt
  import ng_cp_pkg::*;
#(
  parameter logic [5:0] NODE_ID = 6'd0              // CP node on the NoC mesh (
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- CP side (ng_cp_top m_*) ----
  input  logic        m_req_valid,
  output logic        m_req_ready,
  input  mem_req_t    m_req,
  output logic        m_rsp_valid,
  output logic        m_rsp_fault,
  output logic [63:0] m_rsp_data,

  // ---- MMU client (ng_mmu req_*/resp_*), acc = ACC_R/ACC_W, atomic = 0 ----
  output logic        mmu_req_valid,
  input  logic        mmu_req_ready,
  output logic [3:0]  mmu_req_vmid,
  output logic [63:0] mmu_req_va,
  output logic [1:0]  mmu_req_acc,
  output logic [7:0]  mmu_req_tag,
  input  logic        mmu_resp_valid,
  output logic        mmu_resp_ready,
  input  logic [2:0]  mmu_resp_fault,
  input  logic        mmu_resp_sparse,       // read 0 / drop write, no memory access
  input  logic [47:0] mmu_resp_pa,
  input  logic [2:0]  mmu_resp_size,         // span of the translation (page-cross check)
  // fault the MMU cannot see (BAD_PHYS_ADDR after translation)
  output logic        mmu_ext_flt_valid,
  output logic [3:0]  mmu_ext_flt_vmid,
  output logic [63:0] mmu_ext_flt_va,
  output logic [1:0]  mmu_ext_flt_acc,
  output logic [2:0]  mmu_ext_flt_code,

  // ---- NoC agent streams (ng_noc_ep ti_*/to_*), VRAM through the L2 ----
  output logic                 noc_tx_valid,
  input  logic                 noc_tx_ready,
  output ng_noc_pkg::flit_t    noc_tx_flit,
  input  logic                 noc_rx_valid,
  output logic                 noc_rx_ready,
  input  ng_noc_pkg::flit_t    noc_rx_flit,

  // ---- C3 SYS/HOST physical port (same shape as the MMU table-read port, plus write) ----
  output logic        c3_valid,
  input  logic        c3_ready,
  output logic        c3_write,
  output logic [47:0] c3_addr,               // 8-byte aligned
  output logic [7:0]  c3_wstrb,
  output logic [63:0] c3_wdata,
  input  logic        c3_rsp_valid,
  input  logic [63:0] c3_rsp_data,
  input  logic        c3_rsp_err
);
  assign m_req_ready = 1'b0;
  assign m_rsp_valid = 1'b0;  assign m_rsp_fault = 1'b0;  assign m_rsp_data = '0;
  assign mmu_req_valid = 1'b0;  assign mmu_req_vmid = '0;  assign mmu_req_va = '0;
  assign mmu_req_acc = '0;  assign mmu_req_tag = '0;  assign mmu_resp_ready = 1'b0;
  assign mmu_ext_flt_valid = 1'b0;  assign mmu_ext_flt_vmid = '0;  assign mmu_ext_flt_va = '0;
  assign mmu_ext_flt_acc = '0;  assign mmu_ext_flt_code = '0;
  assign noc_tx_valid = 1'b0;  assign noc_tx_flit = '0;  assign noc_rx_ready = 1'b0;
  assign c3_valid = 1'b0;  assign c3_write = 1'b0;  assign c3_addr = '0;
  assign c3_wstrb = '0;  assign c3_wdata = '0;

  logic unused;
  assign unused = &{1'b0, clk, rst_n, m_req_valid, m_req, mmu_req_ready, mmu_resp_valid,
                    mmu_resp_fault, mmu_resp_sparse, mmu_resp_pa, mmu_resp_size, noc_tx_ready,
                    noc_rx_valid, noc_rx_flit, c3_ready, c3_rsp_valid, c3_rsp_data, c3_rsp_err,
                    NODE_ID};
endmodule
