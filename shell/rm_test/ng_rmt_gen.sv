// ng_rmt_gen.sv -- BIST data pattern generator, fixed latency LAT.
//
//   PAT=0: lane_k = mix64({24'b0, beat[31:0], 8'(k)} ^ {SEED, ~SEED})
//   PAT=1: lane_k = {16'hA5A5 ^ SEED[15:0], beat[15:0], beat_byte_addr[31:0]}
//          (byte address of the beat = lane 0, identical for all lanes; same
//          reading as tools/p0test/rmtest_model.h)
// mix64 = splitmix64 finalizer, input used directly as z:
//   z = (z ^ z>>30) * 0xBF58476D1CE4E5B9; z = (z ^ z>>27) * 0x94D049BB133111EB; z ^ z>>31
// The multipliers are followed by MUL_ST register stages so synthesis can
// build pipelined DSP cascades.  No stall: the user must have room for every
// beat it injects (credit counting).
module ng_rmt_mix64 #(
  parameter int MUL_ST = 4
) (
  input  logic        clk,
  input  logic [63:0] x,
  output logic [63:0] y
);
  localparam logic [63:0] C1 = 64'hBF58_476D_1CE4_E5B9;
  localparam logic [63:0] C2 = 64'h94D0_49BB_1331_11EB;
  logic [63:0] r0, r1;
  logic [63:0] m1 [MUL_ST];
  logic [63:0] m2 [MUL_ST];

  always_ff @(posedge clk) begin
    r0    <= x ^ (x >> 30);
    m1[0] <= r0 * C1;
    for (int i = 1; i < MUL_ST; i++) m1[i] <= m1[i-1];
    r1    <= m1[MUL_ST-1] ^ (m1[MUL_ST-1] >> 27);
    m2[0] <= r1 * C2;
    for (int i = 1; i < MUL_ST; i++) m2[i] <= m2[i-1];
    y     <= m2[MUL_ST-1] ^ (m2[MUL_ST-1] >> 31);
  end
endmodule

module ng_rmt_gen #(
  parameter int L      = 8,     // 64-bit lanes per beat
  parameter int LG     = 6,     // log2(bytes per beat)
  parameter int BIW    = 27,    // beat index width
  parameter int MUL_ST = 4
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          pat,
  input  logic [31:0]   seed,
  input  logic          in_valid,
  input  logic [BIW-1:0] in_beat,
  output logic          out_valid,
  output logic [64*L-1:0] out_data
);
  localparam int LAT_MIX = 2 * MUL_ST + 3;
  localparam int LAT     = LAT_MIX + 1;

  logic [63:0] seed64;
  logic [31:0] beat32;
  assign seed64 = {seed, ~seed};
  assign beat32 = 32'(in_beat);

  logic [64*L-1:0] prbs;
  for (genvar k = 0; k < L; k++) begin : g_lane
    ng_rmt_mix64 #(.MUL_ST(MUL_ST)) u_mix (
      .clk(clk),
      .x({24'h0, beat32, 8'(k)} ^ seed64),
      .y(prbs[64*k +: 64]));
  end

  logic           v_d;
  logic [BIW-1:0] b_d;
  ng_rmt_dly #(.W(1), .N(LAT_MIX), .RST(1'b1), .KEEP(1'b0)) u_vd (.clk(clk), .rst_n(rst_n), .d(in_valid), .q(v_d));
  ng_rmt_dly #(.W(BIW), .N(LAT_MIX), .RST(1'b0), .KEEP(1'b0)) u_bd (.clk(clk), .rst_n(rst_n), .d(in_beat), .q(b_d));

  logic [63:0] modw;
  logic [31:0] baddr;
  assign baddr = 32'({b_d, LG'(0)});
  assign modw  = {16'hA5A5 ^ seed[15:0], 16'(b_d), baddr};

  always_ff @(posedge clk) begin
    if (!rst_n) out_valid <= 1'b0;
    else        out_valid <= v_d;
    out_data <= pat ? {L{modw}} : prbs;
  end
endmodule
