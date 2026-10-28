// ng_mmu_regs.sv -- MMU context / invalidate / fault-latch registers.
//
// BAR0 map (section 4):
//   0x0200+0x10*n +0  MMU_CTX_ROOT_LO[n]   RW  shadow root PA[31:12]
//                 +4  MMU_CTX_ROOT_HI[n]   RW  shadow root PA[47:32]
//                 +8  MMU_CTX_CTRL[n]      RW  bit0 ENABLE; write latches root,
//                                              sets/clears ENABLE, invalidates n
//   0x0300  MMU_INVALIDATE   W: bit n flush VMID n (bit0 ignored);
//                            R: pending mask (0 = complete)
//   0x0310  MMU_FAULT_STATUS RO, W1C bit0 (clears VALID and MULTI)
//   0x0314/18 MMU_FAULT_VA_LO/HI   0x031C/20 MMU_FAULT_PTE_LO/HI
// VMID 0 slot (0x0200-0x020F): writes ignored, reads 0.
//
// Invalidate timing (MMU.md 5.3): a write sets the pending bit; the bit is
// visible on `inv_pend` for exactly one cycle, during which the TLB flushes
// and the top level aborts any walk of that VMID; it reads 0 from the next
// cycle on.  A walk that completes in the very cycle of the write may still
// fill the TLB, but that fill is flushed in the following cycle.
//
// Fault latch (MMU.md 6.2): first fault is kept, later ones only set MULTI;
// W1C of bit0 re-arms.  A fault arriving in the clearing cycle is latched as
// the new first fault.  `irq_fault` pulses when VALID rises.
module ng_mmu_regs
  import ng_mmu_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // register bus (32-bit, byte offset relative to BAR0, 12 bits used)
  input  logic        csr_wr,
  input  logic [11:0] csr_waddr,
  input  logic [31:0] csr_wdata,
  input  logic [11:0] csr_raddr,
  output logic [31:0] csr_rdata,      // combinational read

  // per-VMID context (active values)
  output logic [15:0]       ctx_enable,
  output logic [35:0]       ctx_root_pfn [NUM_VMID],

  // invalidate: 1-cycle pending pulse per VMID
  output logic [15:0]       inv_pend,

  // fault report (from the translator or an external client: BAD_PHYS / MISALIGNED)
  input  logic        flt_valid,
  input  logic [3:0]  flt_vmid,
  input  logic [63:0] flt_va,
  input  logic [1:0]  flt_acc,
  input  logic [2:0]  flt_code,
  input  logic [1:0]  flt_level,
  input  logic [63:0] flt_pte,
  output logic        irq_fault
);
  logic [35:0] shadow_pfn [NUM_VMID];

  // fault record
  logic        f_valid, f_multi;
  logic [3:0]  f_vmid;
  logic [63:0] f_va, f_pte;
  logic [1:0]  f_acc, f_level;
  logic [2:0]  f_code;

  wire w_ctx   = csr_waddr[11:8] == 4'h2;
  wire [3:0] w_n = csr_waddr[7:4];
  wire [1:0] w_r = csr_waddr[3:2];
  wire w_inv   = csr_wr && csr_waddr == 12'h300;
  wire w_fst   = csr_wr && csr_waddr == 12'h310;

  logic [15:0] ctrl_inv;   // invalidate requests raised by CTRL writes
  always_comb begin
    ctrl_inv = '0;
    if (csr_wr && w_ctx && w_r == 2'd2 && w_n != 4'd0) ctrl_inv[w_n] = 1'b1;
  end
  wire [15:0] inv_set = ctrl_inv | (w_inv ? {csr_wdata[15:1], 1'b0} : 16'h0);

  logic [15:0] pend;
  assign inv_pend = pend;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ctx_enable <= '0;
      pend       <= '0;
      for (int i = 0; i < NUM_VMID; i++) begin
        shadow_pfn[i]   <= '0;
        ctx_root_pfn[i] <= '0;
      end
    end else begin
      pend <= inv_set;     // visible for one cycle, then reads 0
      if (csr_wr && w_ctx && w_n != 4'd0) begin
        case (w_r)
          2'd0: shadow_pfn[w_n][19:0]  <= csr_wdata[31:12];
          2'd1: shadow_pfn[w_n][35:20] <= csr_wdata[15:0];
          2'd2: begin
            ctx_root_pfn[w_n] <= shadow_pfn[w_n];
            ctx_enable[w_n]   <= csr_wdata[0];
          end
          default: ;
        endcase
      end
    end
  end

  // ---------------- fault latch ---------------------------------------------
  wire clr = w_fst && csr_wdata[0];
  always_ff @(posedge clk) begin
    irq_fault <= 1'b0;
    if (!rst_n) begin
      f_valid <= 1'b0;
      f_multi <= 1'b0;
    end else begin
      if (clr) begin
        f_valid <= 1'b0;
        f_multi <= 1'b0;
      end
      if (flt_valid) begin
        if (!f_valid || clr) begin
          f_valid <= 1'b1;
          f_multi <= 1'b0;
          f_vmid  <= flt_vmid;
          f_va    <= flt_va;
          f_acc   <= flt_acc;
          f_code  <= flt_code;
          f_level <= flt_level;
          f_pte   <= flt_pte;
          irq_fault <= 1'b1;
        end else begin
          f_multi <= 1'b1;
        end
      end
    end
  end

  logic [31:0] status;
  always_comb begin
    status        = 32'h0;
    status[0]     = f_valid;
    status[1]     = f_multi;
    status[6:4]   = f_code;
    status[9:8]   = f_acc;
    status[13:12] = f_level;
    status[19:16] = f_vmid;
  end

  // ---------------- read mux --------------------------------------------------
  always_comb begin
    csr_rdata = 32'h0;
    if (csr_raddr[11:8] == 4'h2) begin
      if (csr_raddr[7:4] != 4'd0) begin
        case (csr_raddr[3:2])
          2'd0: csr_rdata = {shadow_pfn[csr_raddr[7:4]][19:0], 12'h0};
          2'd1: csr_rdata = {16'h0, shadow_pfn[csr_raddr[7:4]][35:20]};
          2'd2: csr_rdata = {31'h0, ctx_enable[csr_raddr[7:4]]};
          default: ;
        endcase
      end
    end else begin
      case (csr_raddr)
        12'h300: csr_rdata = {16'h0, pend};
        12'h310: csr_rdata = status;
        12'h314: csr_rdata = f_va[31:0];
        12'h318: csr_rdata = f_va[63:32];
        12'h31C: csr_rdata = f_pte[31:0];
        12'h320: csr_rdata = f_pte[63:32];
        default: ;
      endcase
    end
  end

endmodule : ng_mmu_regs
