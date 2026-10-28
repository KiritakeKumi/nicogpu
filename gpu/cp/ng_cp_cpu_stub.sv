// ng_cp_cpu_stub.sv -- behavioural stand-in for the Ibex core (simulation only).
//
// Drives the same data-port protocol Ibex would (req/gnt/rvalid, 32-bit,
// one outstanding access) from the real CP firmware, which runs as a DPI
// coroutine on the simulation host (ng_cp_cpu_dpi.c).  The core's interrupt
// input only wakes `wfi`; the firmware polls EVENT after each wake-up.
// Not synthesisable.  See for the limits (no instruction
// timing, no IMEM/DMEM, no boot ROM / firmware image load).
module ng_cp_cpu_stub (
  input  logic        clk,
  input  logic        rst_n,          // low = core held in reset (CORE_RESET)
  input  logic        irq_i,

  output logic        d_req,
  input  logic        d_gnt,
  input  logic        d_rvalid,
  output logic        d_we,
  output logic [3:0]  d_be,
  output logic [31:0] d_addr,
  output logic [31:0] d_wdata,
  input  logic [31:0] d_rdata,
  input  logic        d_err,

  output logic        quiet_o         // firmware found nothing to do (settle detect)
);
  import "DPI-C" function void ng_cpu_reset();
  import "DPI-C" function int ng_cpu_step(input int irq, input int rsp_valid, input int rsp_data,
                                          output int op_we, output int op_addr, output int op_wdata,
                                          output int quiet);

  typedef enum logic [1:0] {S_RUN, S_REQ, S_RSP} st_t;
  st_t st;
  logic quiet_q;
  assign quiet_o = quiet_q & ~irq_i;
  assign d_be = 4'hf;

  int ov, we_i, addr_i, wdata_i, q_i;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ng_cpu_reset();
      st <= S_RUN; d_req <= 1'b0; d_we <= 1'b0; d_addr <= '0; d_wdata <= '0; quiet_q <= 1'b0;
    end else begin
      unique case (st)
        S_RUN: begin
          ov = ng_cpu_step(int'(irq_i), 0, 0, we_i, addr_i, wdata_i, q_i);
          quiet_q <= (q_i != 0);
          if (ov != 0) begin
            d_req <= 1'b1; d_we <= (we_i != 0); d_addr <= addr_i; d_wdata <= wdata_i;
            st <= S_REQ;
          end
        end
        S_REQ: if (d_gnt) begin
          d_req <= 1'b0; st <= S_RSP;
        end
        S_RSP: if (d_rvalid) begin
          ov = ng_cpu_step(int'(irq_i), 1, int'(d_rdata), we_i, addr_i, wdata_i, q_i);
          quiet_q <= (q_i != 0);
          if (ov != 0) begin
            d_req <= 1'b1; d_we <= (we_i != 0); d_addr <= addr_i; d_wdata <= wdata_i;
            st <= S_REQ;
          end else begin
            st <= S_RUN;
          end
        end
        default: st <= S_RUN;
      endcase
    end
  end
endmodule
