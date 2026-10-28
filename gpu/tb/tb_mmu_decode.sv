// tb_mmu_decode.sv -- ng_mmu_pkg::decode_entry vs the ptegen.py
// reference decoder (3124 vectors, all 15 rules at every applicable level).
// Expect NG_MMU_DECODE_TB_PASS.  Vector file: "mmu_decode.hex" in the cwd.
`timescale 1ns/1ps
module tb_mmu_decode;
  import ng_mmu_pkg::*;
  localparam int MAXV = 4096;
  logic [63:0] mem [3*MAXV];
  int errors = 0, n = 0;

  initial begin
    for (int i = 0; i < 3*MAXV; i++) mem[i] = 64'hx;
    $readmemh("mmu_decode.hex", mem);
    for (int i = 0; i < MAXV; i++) begin
      logic [63:0] e, w1, pfn;
      dec_t d;
      logic [1:0] level;
      logic [2:0] ekind, esize;
      logic [3:0] erule;
      e = mem[3*i]; w1 = mem[3*i+1]; pfn = mem[3*i+2];
      if (^e === 1'bx) break;
      level = w1[1:0]; ekind = w1[10:8]; erule = w1[19:16]; esize = w1[26:24];
      d = decode_entry(e, level);
      n++;
      if (d.kind !== ekind || d.rule !== erule) begin
        errors++;
        $display("TB_FAIL decode e=%016h L%0d: kind %0d/%0d rule %0d/%0d", e, level, d.kind, ekind, d.rule, erule);
      end else if ((ekind == K_LEAF || ekind == K_SPARSE) && d.size !== esize) begin
        errors++;
        $display("TB_FAIL decode e=%016h L%0d: size %0d/%0d", e, level, d.size, esize);
      end else if ((ekind == K_LEAF || ekind == K_TABLE) && d.pfn !== pfn[35:0]) begin
        errors++;
        $display("TB_FAIL decode e=%016h L%0d: pfn %h/%h", e, level, d.pfn, pfn[35:0]);
      end
    end
    if (n < 3000) begin errors++; $display("TB_FAIL only %0d vectors read", n); end
    if (errors == 0) $display("NG_MMU_DECODE_TB_PASS: %0d vectors, 0 errors", n);
    else $display("NG_MMU_DECODE_TB_FAIL: %0d errors", errors);
    $finish;
  end
endmodule
