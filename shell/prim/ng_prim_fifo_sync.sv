// ng_prim_fifo_sync.sv -- the small synchronous FIFO shared by the static shell (boundary trackers) and the
// reconfigurable modules (rm_test / rm_cp / rm_sp).  Storage is a plain register array (LUTRAM / FFs on FPGA, no vendor
// primitive); dout is the head entry, read combinationally (valid when !empty); push and pop may coincide.
//
//   W, DEPTH   entry width / entries.  DEPTH need not be a power of two (the pointers wrap explicitly then; a power of
//              two uses the natural wrap and costs no compare).  count is $clog2(DEPTH + 1) bits wide.
//   REG_FLAGS  0: empty / full are combinational compares of count.  1: empty / full are flops loaded from the next
//              count (exact, no lag, and no compare after the counter on the output path).
//   GATED      1: push while full and pop while empty are ignored (the boundary trackers see raw handshakes).
//              0: the caller guarantees legal push / pop; an illegal one corrupts the FIFO, as in the instances that
//              use it (the gating terms are not built).  Simulation reports a violation with $error.
//
// Replaces ng_fifo_sync (GATED = 1), ng_rmt_fifo (REG_FLAGS = 1), ng_rsp_sfifo (REG_FLAGS = 1) and
// ng_rcp_axmux_fifo (W = 1, both 0).  Control flops are reset, the storage is not.
module ng_prim_fifo_sync #(
  parameter int W         = 8,
  parameter int DEPTH     = 8,
  parameter bit REG_FLAGS = 1'b0,
  parameter bit GATED     = 1'b0
) (
  input  logic                        clk,
  input  logic                        rst_n,
  input  logic                        push,
  input  logic [W-1:0]                din,
  input  logic                        pop,
  output logic [W-1:0]                dout,
  output logic                        empty,
  output logic                        full,
  output logic [$clog2(DEPTH + 1)-1:0] count
);
  localparam int PW   = (DEPTH > 1) ? $clog2(DEPTH) : 1;
  localparam int CW   = $clog2(DEPTH + 1);
  localparam bit POW2 = (DEPTH & (DEPTH - 1)) == 0;

  logic [W-1:0]  mem [DEPTH];
  logic [PW-1:0] wp, rp;
  logic [CW-1:0] cnt;

  wire push_ok = GATED ? (push && !full)  : push;
  wire pop_ok  = GATED ? (pop  && !empty) : pop;
  wire [CW-1:0] cnt_n = cnt + CW'(push_ok) - CW'(pop_ok);

  assign dout  = mem[rp];
  assign count = cnt;

  always_ff @(posedge clk) if (push_ok) mem[wp] <= din;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wp  <= '0;
      rp  <= '0;
      cnt <= '0;
    end else begin
      if (push_ok) wp <= (!POW2 && wp == PW'(DEPTH - 1)) ? '0 : wp + 1'b1;
      if (pop_ok)  rp <= (!POW2 && rp == PW'(DEPTH - 1)) ? '0 : rp + 1'b1;
      cnt <= cnt_n;
    end
  end

`ifndef SYNTHESIS
  // caller contract of the ungated flavour (simulation only; +ng_fifo_nocheck turns it off for stress benches)
  if (!GATED) begin : g_contract
    bit nocheck;
    initial nocheck = $test$plusargs("ng_fifo_nocheck");
    always @(posedge clk) begin
      if (rst_n && !nocheck) begin
        if (push && full && !pop) $error("%m: push while full");
        if (pop && empty)         $error("%m: pop while empty");
      end
    end
  end
`endif

  if (REG_FLAGS) begin : g_reg_flags
    logic empty_q, full_q;
    always_ff @(posedge clk) begin
      if (!rst_n) begin
        empty_q <= 1'b1;
        full_q  <= 1'b0;
      end else begin
        empty_q <= (cnt_n == '0);
        full_q  <= (cnt_n == CW'(DEPTH));
      end
    end
    assign empty = empty_q;
    assign full  = full_q;
  end else begin : g_comb_flags
    assign empty = (cnt == '0);
    assign full  = (cnt == CW'(DEPTH));
  end
endmodule
