// ng_rcp_mmu.sv -- glue between ng_mmu (rtl/gpu/mmu) and the CP RM.
//
//   ng_rcp_mmurd   clk_l2: MMU page-table read port (rd_*/rsp_*) -> C3 engine requester 2.  A table read is a VMID-0
//                  physical 64-bit read in the SYS aperture: rd_addr[47:31] must be 0x20 (SYS_BASE = 0x10_0000_0000,
//                  2 GiB) and 8-byte aligned, the C3 offset is rd_addr - SYS_BASE; anything else answers rsp_err
//                  (the walker turns that into BAD_PHYS) without touching the bus.  One read at a time; rsp_valid is
//                  a registered single-cycle pulse, always at least one cycle after the rd_valid && rd_ready
//                  handshake.  Quiesce is applied by the C3 engine (op_ready), the same rule as BAR2 / CP: a read
//                  that has not reached the boundary port yet just waits.
//   ng_rcp_irq_l2ts  MMU fault interrupt, clk_l2 -> clk_ts (ng_cp_top irq_ext_i[2] -> IRQ_STATUS bit 2).  The MMU's
//                  irq_fault is a one-cycle clk_l2 pulse, which clk_ts (slower) could miss, so it is turned into a
//                  level (a toggle flop), crossed with a 2-flop synchroniser, and edge-detected on the clk_ts side
//                  into a one-cycle pulse.  ng_cp_regs ORs the pulse into IRQ_STATUS (sticky, W1C), as for any
//                  other irq_ext_i source.  Two faults closer together than ~3 clk_ts cycles merge into one
//                  interrupt (the MMU fault record already folds them: first fault kept, MULTI).
module ng_rcp_mmurd (
  input  logic        clk,              // clk_l2
  input  logic        rst_n,
  // ng_mmu table read port
  input  logic        rd_valid,
  output logic        rd_ready,
  input  logic [47:0] rd_addr,
  output logic        rsp_valid,
  output logic [63:0] rsp_data,
  output logic        rsp_err,
  // C3 engine requester 2
  output logic        op_valid,
  input  logic        op_ready,
  output logic [30:0] op_addr,
  input  logic        rs_valid,
  input  logic        rs_err,
  input  logic [63:0] rs_rdata
);
  typedef enum logic [1:0] {S_IDLE, S_OP, S_WAIT, S_ERR} st_t;
  st_t st;

  wire in_sys = (rd_addr[47:31] == 17'h00020) && (rd_addr[2:0] == 3'b000);

  assign rd_ready = (st == S_IDLE);
  assign op_valid = (st == S_OP);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; rsp_valid <= 1'b0; rsp_data <= '0; rsp_err <= 1'b0; op_addr <= '0;
    end else begin
      rsp_valid <= 1'b0;
      unique case (st)
        S_IDLE: if (rd_valid) begin
          op_addr <= rd_addr[30:0];
          st <= in_sys ? S_OP : S_ERR;
        end
        S_OP: if (op_ready) st <= S_WAIT;
        S_WAIT: if (rs_valid) begin
          rsp_valid <= 1'b1; rsp_err <= rs_err; rsp_data <= rs_err ? 64'h0 : rs_rdata;
          st <= S_IDLE;
        end
        default: begin                       // S_ERR
          rsp_valid <= 1'b1; rsp_err <= 1'b1; rsp_data <= '0;
          st <= S_IDLE;
        end
      endcase
    end
  end
endmodule

module ng_rcp_irq_l2ts (
  input  logic clk_l2,
  input  logic rst_l2_n,
  input  logic pulse_l2,              // one-cycle pulse in clk_l2
  input  logic clk_ts,
  input  logic rst_ts_n,
  output logic pulse_ts               // one-cycle pulse in clk_ts
);
  logic tog;
  (* ASYNC_REG = "TRUE" *) logic s1, s2;
  logic s3;

  always_ff @(posedge clk_l2) begin
    if (!rst_l2_n)      tog <= 1'b0;
    else if (pulse_l2)  tog <= ~tog;
  end

  always_ff @(posedge clk_ts or negedge rst_ts_n) begin
    if (!rst_ts_n) begin
      s1 <= 1'b0; s2 <= 1'b0; s3 <= 1'b0; pulse_ts <= 1'b0;
    end else begin
      s1 <= tog; s2 <= s1; s3 <= s2;
      pulse_ts <= s2 ^ s3;
    end
  end
endmodule
