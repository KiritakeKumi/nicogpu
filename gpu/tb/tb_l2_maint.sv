// tb_l2_maint.sv -- L2 cache maintenance (inv.l2 / wb.l2) on one slice.
//   wb.l2: every dirty line reaches DDR, lines stay valid (no refill)
//           become clean (no AW on later evict / on a repeated wb.l2);
//   inv.l2: data-preserving -- dirty lines are written back, then every line is
//           invalidated (next access misses and refills), no data is lost;
//   bad op -> ERR_BAD_PA, addr/size of a maintenance flit are ignored,
//   and a random soak against a byte-accurate reference with maintenance
//   interleaved (DDR image == reference after each maintenance).
// Stimulus and sampling on negedge.  Expect NG_L2_MAINT_TB_PASS.
`timescale 1ns/1ps
module tb_l2_maint;
  import ng_noc_pkg::*;
  import ng_l2_addrmap::*;

  localparam int SETS = 16, WAYS = 4, LINE_BYTES = 128;
  localparam int AXI_AW = 33, AXI_DW = 512, AXI_IW = 6;
  localparam logic [1:0] K_MAINT = 2'd3;         // written out independently of ng_noc_pkg
  localparam logic [4:0] M_INV = 5'd0, M_WB = 5'd1;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  logic              s_req_valid = 1'b0;
  logic              s_req_ready;
  flit_t             s_req_flit;
  logic              m_resp_valid;
  logic              m_resp_ready = 1'b1;
  flit_t             m_resp_flit;
  logic              arvalid, arready, rvalid, rready, rlast;
  logic [AXI_AW-1:0] araddr;
  logic [7:0]        arlen;
  logic [2:0]        arsize;
  logic [1:0]        arburst;
  logic [AXI_IW-1:0] arid, rid;
  logic [AXI_DW-1:0] rdata;
  logic [1:0]        rresp;
  logic              awvalid, awready, wvalid, wready, wlast, bvalid, bready;
  logic [AXI_AW-1:0] awaddr;
  logic [7:0]        awlen;
  logic [2:0]        awsize;
  logic [1:0]        awburst;
  logic [AXI_IW-1:0] awid, bid;
  logic [AXI_DW-1:0] wdata;
  logic [AXI_DW/8-1:0] wstrb;
  logic [1:0]        bresp;

  ng_l2_slice #(
    .SETS(SETS), .WAYS(WAYS), .LINE_BYTES(LINE_BYTES),
    .AXI_AW(AXI_AW), .AXI_DW(AXI_DW), .AXI_IW(AXI_IW), .SLICE_ID(0)
  ) dut (
    .clk(clk), .rst_n(rst_n),
    .s_req_valid(s_req_valid), .s_req_ready(s_req_ready), .s_req_flit(s_req_flit),
    .m_resp_valid(m_resp_valid), .m_resp_ready(m_resp_ready),
    .m_resp_flit(m_resp_flit),
    .arvalid(arvalid), .arready(arready), .araddr(araddr), .arlen(arlen),
    .arsize(arsize), .arburst(arburst), .arid(arid),
    .rvalid(rvalid), .rready(rready), .rdata(rdata), .rlast(rlast),
    .rid(rid), .rresp(rresp),
    .awvalid(awvalid), .awready(awready), .awaddr(awaddr), .awlen(awlen),
    .awsize(awsize), .awburst(awburst), .awid(awid),
    .wvalid(wvalid), .wready(wready), .wdata(wdata), .wstrb(wstrb),
    .wlast(wlast),
    .bvalid(bvalid), .bready(bready), .bid(bid), .bresp(bresp)
  );

  tb_l2_axi_mem_model #(.AW(AXI_AW), .DW(AXI_DW), .IW(AXI_IW)) u_mem (
    .clk(clk), .rst_n(rst_n),
    .arvalid(arvalid), .arready(arready), .araddr(araddr), .arlen(arlen),
    .arsize(arsize), .arburst(arburst), .arid(arid),
    .rvalid(rvalid), .rready(rready), .rdata(rdata), .rlast(rlast),
    .rid(rid), .rresp(rresp),
    .awvalid(awvalid), .awready(awready), .awaddr(awaddr), .awlen(awlen),
    .awsize(awsize), .awburst(awburst), .awid(awid),
    .wvalid(wvalid), .wready(wready), .wdata(wdata), .wstrb(wstrb),
    .wlast(wlast),
    .bvalid(bvalid), .bready(bready), .bid(bid), .bresp(bresp)
  );

  int ar_cnt, aw_cnt;
  always_ff @(posedge clk) begin
    if (!rst_n) begin ar_cnt <= 0; aw_cnt <= 0; end
    else begin
      if (arvalid && arready) ar_cnt <= ar_cnt + 1;
      if (awvalid && awready) aw_cnt <= aw_cnt + 1;
    end
  end

  // ---- reference -----------------------------------------------------------
  logic [7:0] refmem [logic [47:0]];
  int errors, checks;

  function automatic logic [7:0] hash48(input logic [47:0] a);
    logic [63:0] p;
    p = a * 64'h9E3779B97F4A7C15;
    return p[55:48];
  endfunction
  function automatic logic [7:0] refbyte(input logic [47:0] pa);
    return refmem.exists(pa) ? refmem[pa] : hash48(l2_ch_of(pa).off);
  endfunction

  // ---- bus tasks -------------------------------------------------------------
  task automatic send_req(input flit_t f);
    s_req_valid = 1'b1;
    s_req_flit  = f;
    while (!s_req_ready) @(negedge clk);
    @(negedge clk);
    s_req_valid = 1'b0;
  endtask
  task automatic recv_resp(output flit_t rr);
    m_resp_ready = 1'b1;
    while (!m_resp_valid) @(negedge clk);
    rr = m_resp_flit;
    @(negedge clk);
  endtask
  task automatic xact(input flit_t f, output flit_t rr);
    send_req(f);
    recv_resp(rr);
  endtask

  function automatic flit_t mk(input logic [1:0] kind, input logic [47:0] pa,
                               input logic [4:0] o, input logic [1:0] sz,
                               input logic rtn, input logic [63:0] d,
                               input logic [7:0] be);
    return '{cls: CLS_REQ, kind: kind, op: o, size: sz, rtn: rtn, addr: pa,
             src: 6'd1, dst: 6'd0, tag: 8'hA5, data: d, be: be, default: '0};
  endfunction

  function automatic logic [47:0] line_pa(input int n);
    return (48'(n >> 5) << 14) | (48'(n & 31) << 7);
  endfunction

  task automatic st8(input logic [47:0] pa, input logic [63:0] d);
    flit_t rr;
    xact(mk(KIND_WR, pa, AMO_NONE, SZ_8, 1'b0, d, 8'hFF), rr);
    for (int b = 0; b < 8; b++) refmem[pa + 48'(b)] = d[8*b +: 8];
    checks++;
    if (rr.err != ERR_NONE) begin errors++; $display("TB_FAIL: st err"); end
  endtask

  task automatic ld8_chk(input logic [47:0] pa, input string why);
    flit_t rr;
    logic [63:0] want;
    xact(mk(KIND_RD, pa, AMO_NONE, SZ_8, 1'b0, '0, 8'h00), rr);
    for (int b = 0; b < 8; b++) want[8*b +: 8] = refbyte(pa + 48'(b));
    checks++;
    if (rr.err != ERR_NONE || rr.data !== want) begin
      errors++;
      $display("TB_FAIL: %s ld %h got %h want %h err=%0d", why, pa, rr.data, want, rr.err);
    end
  endtask

  task automatic maint(input logic [4:0] o, input logic [47:0] pa, output flit_t rr);
    xact(mk(K_MAINT, pa, o, SZ_8, 1'b0, 64'hDEAD, 8'hFF), rr);
  endtask

  // every byte the reference knows about must be in DDR (after a flush)
  task automatic check_ddr(input string why);
    logic [47:0] k;
    int n;
    n = 0;
    if (refmem.first(k)) begin
      do begin
        checks++;
        if (u_mem.peek(AXI_AW'(l2_ch_of(k).off)) !== refmem[k]) begin
          errors++;
          if (errors < 30)
            $display("TB_FAIL: %s DDR[%h]=%h want %h", why, k,
                     u_mem.peek(AXI_AW'(l2_ch_of(k).off)), refmem[k]);
        end
        n++;
      end while (refmem.next(k));
    end
  endtask

  // ---- main --------------------------------------------------------------------
  flit_t rr;
  int c0, w0;
  initial begin
    errors = 0;
    checks = 0;
    rst_n = 1'b0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ===== 0) maintenance on an empty cache completes, no DDR traffic ========
    c0 = ar_cnt; w0 = aw_cnt;
    maint(M_WB, 48'h0, rr);
    checks++;
    if (rr.err != ERR_NONE) begin errors++; $display("TB_FAIL: empty wb err=%0d", rr.err); end
    maint(M_INV, 48'h0, rr);
    checks++;
    if (rr.err != ERR_NONE) begin errors++; $display("TB_FAIL: empty inv err=%0d", rr.err); end
    checks++;
    if (ar_cnt != c0 || aw_cnt != w0) begin errors++; $display("TB_FAIL: empty cache DDR traffic"); end

    // ===== 1) wb.l2: flush dirty, keep valid, become clean ====================
    for (int i = 0; i < 24; i++) st8(line_pa(i) + 48'(8 * (i % 14)), {$urandom(), $urandom()});
    w0 = aw_cnt;
    // nothing written back yet: lines are dirty in L2 only
    checks++;
    if (w0 != 0) begin errors++; $display("TB_FAIL: premature DDR write (%0d)", w0); end
    maint(M_WB, 48'hFFFF_FFFF_FFFF, rr);       // addr ignored (even non-VRAM)
    checks++;
    if (rr.err != ERR_NONE) begin errors++; $display("TB_FAIL: wb err=%0d", rr.err); end
    checks++;
    if (aw_cnt - w0 != 24) begin errors++; $display("TB_FAIL: wb wrote %0d lines, want 24", aw_cnt - w0); end
    check_ddr("after wb");
    // lines remain valid: re-read hits, no AR
    c0 = ar_cnt;
    for (int i = 0; i < 24; i++) ld8_chk(line_pa(i) + 48'(8 * (i % 14)), "wb-hit");
    checks++;
    if (ar_cnt != c0) begin errors++; $display("TB_FAIL: wb dropped lines (%0d refills)", ar_cnt - c0); end
    // lines are clean: repeated wb and evicting traffic cause no AW
    w0 = aw_cnt;
    maint(M_WB, 48'h0, rr);
    for (int i = 100; i < 100 + 4 * SETS * WAYS; i++) ld8_chk(line_pa(i), "evict");
    checks++;
    if (aw_cnt != w0) begin errors++; $display("TB_FAIL: clean lines written back (%0d)", aw_cnt - w0); end

    // ===== 2) inv.l2: data-preserving, everything invalidated =================
    for (int i = 0; i < 20; i++) st8(line_pa(300 + i) + 48'(8 * (i % 14)), {$urandom(), $urandom()});
    for (int i = 0; i < 10; i++) ld8_chk(line_pa(400 + i), "pre-inv clean fill");
    w0 = aw_cnt;
    maint(M_INV, 48'h0, rr);
    checks++;
    if (rr.err != ERR_NONE) begin errors++; $display("TB_FAIL: inv err=%0d", rr.err); end
    checks++;
    if (aw_cnt - w0 < 1) begin errors++; $display("TB_FAIL: inv lost dirty data (no AW)"); end
    check_ddr("after inv");
    // every previously cached line now misses
    c0 = ar_cnt;
    for (int i = 0; i < 20; i++) ld8_chk(line_pa(300 + i) + 48'(8 * (i % 14)), "inv-miss");
    for (int i = 0; i < 10; i++) ld8_chk(line_pa(400 + i), "inv-miss clean");
    checks++;
    if (ar_cnt - c0 != 30) begin errors++; $display("TB_FAIL: inv left lines valid (%0d refills, want 30)", ar_cnt - c0); end

    // ===== 3) bad maintenance op ================================================
    maint(5'd2, 48'h0, rr);
    checks++;
    if (rr.err != ERR_BAD_PA) begin errors++; $display("TB_FAIL: bad maint op err=%0d", rr.err); end
    maint(5'd31, 48'h0, rr);
    checks++;
    if (rr.err != ERR_BAD_PA) begin errors++; $display("TB_FAIL: bad maint op 31 err=%0d", rr.err); end
    // slice still alive
    ld8_chk(line_pa(1), "post-bad");

    // ===== 4) soak: random traffic + maintenance, DDR image == reference =======
    for (int round = 0; round < 6; round++) begin
      for (int n = 0; n < 120; n++) begin
        logic [47:0] pa;
        int k;
        pa = line_pa($urandom_range(95)) | 48'($urandom_range(15) * 8);
        k = $urandom_range(9);
        if (k < 4) st8(pa, {$urandom(), $urandom()});
        else if (k < 8) ld8_chk(pa, "soak");
        else begin
          // 32-bit add atomic with reference
          logic [31:0] o32, y32;
          y32 = $urandom();
          for (int b = 0; b < 4; b++) o32[8*b +: 8] = refbyte(pa + 48'(b));
          xact(mk(KIND_AMO, pa, AMO_ADD, SZ_4, 1'b1, {32'd0, y32}, 8'h00), rr);
          checks++;
          if (rr.data !== {32'd0, o32}) begin errors++; $display("TB_FAIL: soak amo old"); end
          o32 = o32 + y32;
          for (int b = 0; b < 4; b++) refmem[pa + 48'(b)] = o32[8*b +: 8];
        end
        if ($urandom_range(29) == 0) begin
          maint($urandom_range(1) ? M_INV : M_WB, 48'($urandom()), rr);
          checks++;
          if (rr.err != ERR_NONE) begin errors++; $display("TB_FAIL: soak maint err=%0d", rr.err); end
          check_ddr("soak maint");
        end
      end
      maint(M_INV, 48'h0, rr);
      check_ddr("soak round end");
      for (int n = 0; n < 40; n++) ld8_chk(line_pa($urandom_range(95)) | 48'($urandom_range(15) * 8), "soak reread");
    end

    if (errors == 0)
      $display("NG_L2_MAINT_TB_PASS: %0d checks, 0 errors", checks);
    else
      $display("NG_L2_MAINT_TB_FAIL: %0d errors of %0d checks", errors, checks);
    $finish;
  end

  initial begin
    #100ms;
    $display("NG_L2_MAINT_TB_FAIL: timeout");
    $finish;
  end
endmodule
