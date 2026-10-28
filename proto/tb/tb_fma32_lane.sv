// tb_fma32_lane.sv -- vector test for fma32_lane (vectors from gen_fma_vectors.py)
`timescale 1ns/1ps
module tb_fma32_lane;
  localparam N = 20000, LAT = 7;   // input reg + 6 stages
  logic clk = 0; always #2.5 clk = ~clk;
  logic [127:0] vec [0:N-1];
  logic [31:0] a, b, c, r; logic iv, ov; logic [4:0] ff;
  fma32_lane dut(.clk(clk), .in_valid(iv), .a(a), .b(b), .c(c), .out_valid(ov), .r(r), .fflags(ff));
  integer i, errs = 0, chk = 0;
  logic [31:0] exp_q [$];
  initial begin
    $readmemh("fma_vec.mem", vec);
    iv = 0;
    for (i = 0; i < N + LAT + 2; i++) begin
      @(negedge clk);
      if (i < N) begin a = vec[i][127:96]; b = vec[i][95:64]; c = vec[i][63:32]; iv = 1; exp_q.push_back(vec[i][31:0]); end
      else iv = 0;
    end
    #20;
    $display("FMA32 TEST: checked=%0d errors=%0d", chk, errs);
    $finish;
  end
  always @(negedge clk) if (ov) begin
    automatic logic [31:0] e = exp_q.pop_front();
    chk++;
    if (r !== e) begin
      errs++;
      if (errs < 20) $display("MISMATCH #%0d got %08h exp %08h", chk, r, e);
    end
  end
endmodule
