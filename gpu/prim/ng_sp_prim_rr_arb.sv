// ng_sp_prim_rr_arb.sv -- N-input round-robin arbiter (SP scheduler local
// primitive; self-contained so rtl/gpu does not depend on
// rtl/shell/prim).  Grants at most one request per cycle; the pointer rotates
// past the granted input, so issue order over always-ready inputs is a stable
// cyclic sweep 0,1,..,N-1,0,.. matching the functional model's deterministic
// round-robin (quantum=1).
module ng_sp_prim_rr_arb #(
  parameter int N = 16
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic [N-1:0]  req,
  output logic [N-1:0]  grant,   // one-hot, at most one bit
  output logic          valid
);
  localparam int AW = (N <= 1) ? 1 : $clog2(N);

  logic [AW-1:0] ptr;
  logic [N-1:0]  rotated;
  logic [AW-1:0] rot_idx;   // lowest set position in `rotated`

  // rotated[j] = req[(ptr + j) mod N]: the first candidate after the pointer
  // is bit 0, so a lowest-bit-first priority encode implements the sweep.
  always_comb begin
    for (int j = 0; j < N; j++)
      rotated[j] = (ptr + j < N) ? req[ptr + j] : req[ptr + j - N];
  end

  always_comb begin
    rot_idx = '0;
    for (int i = N - 1; i >= 0; i--)
      if (rotated[i]) rot_idx = AW'(i);  // descending scan: lowest set bit wins
  end

  always_comb begin
    grant = '0;
    if (|rotated) grant[(ptr + rot_idx) % N] = 1'b1;
  end
  assign valid = |req;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ptr <= '0;
    end else if (|grant) begin
      for (int i = 0; i < N; i++)
        if (grant[i]) ptr <= (i == N - 1) ? '0 : AW'(i + 1);
    end
  end
endmodule
