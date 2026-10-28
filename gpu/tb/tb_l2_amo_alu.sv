// tb_l2_amo_alu.sv -- directed vectors for the L2 atomic ALU.
// Every op x {32,64} bit x sign/edge cases, checked against a reference
// implementation of the semantics in (aligned
// atom.*).
// Expect NG_L2_AMO_TB_PASS.
`timescale 1ns/1ps
module tb_l2_amo_alu;
  import ng_noc_pkg::*;

  logic [4:0]  op;
  logic        is64;
  logic [63:0] a, b, cmp;
  logic [63:0] r;
  logic        cas_hit;

  ng_l2_amo_alu dut (.*);

  int errors;
  int checks;

  function automatic logic [63:0] ref_alu(input logic [4:0] o,
                                          input logic w64,
                                          input logic [63:0] av,
                                          input logic [63:0] bv,
                                          input logic [63:0] cv);
    logic [31:0] a32, b32, r32;
    a32 = av[31:0];
    b32 = bv[31:0];
    if (w64) begin
      unique case (o)
        AMO_ADD:   return av + bv;
        AMO_SUB:   return av - bv;
        AMO_AND:   return av & bv;
        AMO_OR:    return av | bv;
        AMO_XOR:   return av ^ bv;
        AMO_MIN_S: return ($signed(av) < $signed(bv)) ? av : bv;
        AMO_MAX_S: return ($signed(av) > $signed(bv)) ? av : bv;
        AMO_MIN_U: return (av < bv) ? av : bv;
        AMO_MAX_U: return (av > bv) ? av : bv;
        AMO_XCHG:  return bv;
        AMO_CAS:   return (av == cv) ? bv : av;
        default:   return av;
      endcase
    end else begin
      unique case (o)
        AMO_ADD:   r32 = a32 + b32;
        AMO_SUB:   r32 = a32 - b32;
        AMO_AND:   r32 = a32 & b32;
        AMO_OR:    r32 = a32 | b32;
        AMO_XOR:   r32 = a32 ^ b32;
        AMO_MIN_S: r32 = ($signed(a32) < $signed(b32)) ? a32 : b32;
        AMO_MAX_S: r32 = ($signed(a32) > $signed(b32)) ? a32 : b32;
        AMO_MIN_U: r32 = (a32 < b32) ? a32 : b32;
        AMO_MAX_U: r32 = (a32 > b32) ? a32 : b32;
        AMO_XCHG:  r32 = b32;
        AMO_CAS:   r32 = (a32 == cv[31:0]) ? b32 : a32;
        AMO_INC:   r32 = (a32 >= b32) ? 32'd0 : a32 + 32'd1;
        AMO_DEC:   r32 = (a32 == 32'd0 || a32 > b32) ? b32 : a32 - 32'd1;
        default:   r32 = a32;
      endcase
      return {32'd0, r32};
    end
  endfunction

  task automatic chk(input logic [4:0] o, input logic w64,
                     input logic [63:0] av, input logic [63:0] bv,
                     input logic [63:0] cv);
    logic [63:0] want;
    op = o; is64 = w64; a = av; b = bv; cmp = cv;
    #1;
    want = ref_alu(o, w64, av, bv, cv);
    checks++;
    if (r !== want) begin
      errors++;
      $display("TB_FAIL amo_alu: op=%0d %0d a=%h b=%h cmp=%h r=%h want=%h",
               o, w64, av, bv, cv, r, want);
    end
  endtask

  logic [63:0] vals32 [8];
  logic [63:0] vals64 [8];

  initial begin
    errors = 0;
    checks = 0;
    vals32 = '{64'd0, 64'd1, 64'h0000FFFF, 64'h7FFFFFFF, 64'h80000000,
               64'hFFFFFFFF, 64'h12345678, 64'h5A5A0000};
    vals64 = '{64'd0, 64'd1, 64'h0000FFFFFFFFFFFF, 64'h7FFFFFFFFFFFFFFF,
               64'h8000000000000000, 64'hFFFFFFFFFFFFFFFF,
               64'h0123456789ABCDEF, 64'hFEDCBA9876543210};

    for (int op_i = 0; op_i <= 12; op_i++) begin
      logic [4:0] o;
      o = 5'(op_i);
      // 32-bit ops (inc/dec only defined for 32-bit)
      for (int i = 0; i < 8; i++)
        for (int j = 0; j < 8; j++)
          chk(o, 1'b0, vals32[i], vals32[j], vals32[(i+j) % 8]);
      if (o != AMO_INC && o != AMO_DEC) begin
        for (int i = 0; i < 8; i++)
          for (int j = 0; j < 8; j++)
            chk(o, 1'b1, vals64[i], vals64[j], vals64[(i+j) % 8]);
      end
    end

    // directed edges: inc/dec wrap semantics (CUDA-style)
    chk(AMO_INC, 1'b0, 64'd5,  64'd5,  64'd0);  // a>=b -> 0
    chk(AMO_INC, 1'b0, 64'd4,  64'd5,  64'd0);  // a<b  -> 5
    chk(AMO_INC, 1'b0, 64'd0,  64'd5,  64'd0);
    chk(AMO_INC, 1'b0, 64'hFFFFFFFF, 64'd0, 64'd0);  // 0>=0 -> 0
    chk(AMO_DEC, 1'b0, 64'd0,  64'd5,  64'd0);  // a==0 -> b
    chk(AMO_DEC, 1'b0, 64'd6,  64'd5,  64'd0);  // a>b  -> b
    chk(AMO_DEC, 1'b0, 64'd3,  64'd5,  64'd0);  // else -> 2
    chk(AMO_DEC, 1'b0, 64'd5,  64'd5,  64'd0);  // a==b -> 4
    // CAS hit/miss explicit
    chk(AMO_CAS, 1'b0, 64'hDEADBEEF, 64'hCAFEF00D, 64'hDEADBEEF);  // hit
    chk(AMO_CAS, 1'b0, 64'hDEADBEEF, 64'hCAFEF00D, 64'h00000000);  // miss
    chk(AMO_CAS, 1'b1, 64'h1122334455667788, 64'hAABBCCDD, 64'h1122334455667788);
    chk(AMO_CAS, 1'b1, 64'h1122334455667788, 64'hAABBCCDD, 64'h1122334455667789);
    // signed vs unsigned boundary
    chk(AMO_MIN_S, 1'b0, 64'h80000000, 64'd1, 64'd0);   // signed: min is 0x80000000
    chk(AMO_MIN_U, 1'b0, 64'h80000000, 64'd1, 64'd0);   // unsigned: min is 1
    chk(AMO_MAX_S, 1'b0, 64'h7FFFFFFF, 64'hFFFFFFFF, 64'd0);
    chk(AMO_MAX_U, 1'b0, 64'h7FFFFFFF, 64'hFFFFFFFF, 64'd0);
    chk(AMO_MIN_S, 1'b1, 64'h8000000000000000, 64'd1, 64'd0);
    chk(AMO_MIN_U, 1'b1, 64'h8000000000000000, 64'd1, 64'd0);

    if (errors == 0)
      $display("NG_L2_AMO_TB_PASS: %0d checks, 0 errors", checks);
    else
      $display("NG_L2_AMO_TB_FAIL: %0d errors of %0d checks", errors, checks);
    $finish;
  end
endmodule
