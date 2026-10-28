// ng_prim_l2ram.sv -- local primitive: simple-port synchronous RAM
// one read port and one write port (read and write may target the same
// address in the same cycle; the read returns the OLD content, i.e.
// read-first).  Byte enables on the write port.
//
// Inference mapping (no vendor primitive is instantiated, DFX_BOUNDARY):
//   * data array of an L2 slice (LINE_BYTES wide, DEPTH=SETS): URAM288 in
//     depth-cascade pairs, 64 of 72 bits used per column (
//     has the URAM count arithmetic);
//   * tag array (a few dozen bits wide): BRAM36 or LUTRAM;
//   * in xsim it is a plain behavioural array.
// File/module name is on purpose: rtl/gpu/prim/ is shared
// with the parallel work; independent names avoid merge conflicts.
module ng_prim_l2ram #(
  parameter int D_W    = 1024,  // data width
  parameter int DEPTH = 64,     // entries (power of two)
  parameter int BE_W   = D_W/8  // write byte-enable granularity: 8 bits
) (
  input  logic              clk,
  input  logic              we,
  input  logic [BE_W-1:0]   wbe,
  input  logic [$clog2(DEPTH)-1:0] waddr,
  input  logic [D_W-1:0]    wdata,
  input  logic              re,
  input  logic [$clog2(DEPTH)-1:0] raddr,
  output logic [D_W-1:0]    rdata_q
);
  // the array is split into columns of at most 256 bits.  One 1024-bit x 1024-entry array (an L2 slice data
  // way at the full SETS = 1024) is 1,048,576 bits, above Vivado synthesis' 1,000,000-bit limit for one variable (the
  // same failure the L1D hit at L1_SETS = 256, syn/results/ooc-gpu-units-20261014.md); columns of 256 bits are
  // 262,144 bits each and map to URAM288 / BRAM columns anyway.  Behaviour is unchanged.
  localparam int COL_W = (D_W > 256 && (D_W % 256) == 0) ? 256 : D_W;
  localparam int NCOL  = D_W / COL_W;
  localparam int CBE   = COL_W / 8;          // byte enables per column (8-bit granularity, as before)

  for (genvar c = 0; c < NCOL; c++) begin : g_col
    logic [COL_W-1:0] mem [DEPTH];
    logic [COL_W-1:0] q;
    assign rdata_q[c * COL_W +: COL_W] = q;

    always_ff @(posedge clk) begin
      if (we) begin
        for (int b = 0; b < CBE; b++) begin
          if (c * CBE + b < BE_W && wbe[c * CBE + b]) mem[waddr][8*b +: 8] <= wdata[c * COL_W + 8*b +: 8];
        end
      end
    end

    always_ff @(posedge clk) begin
      if (re) q <= mem[raddr];
    end

    // synthesis translate_off
    initial begin
      for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    end
    // synthesis translate_on
  end
endmodule
