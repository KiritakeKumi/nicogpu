// tb_axi64_to_lite32.sv -- testbench for the AXI4-64 to AXI4-Lite-32 bridge.
//
// Cases:
//   * single-beat 32-bit write (low half), (high half)
//   * full 64-bit write -> two lite writes (low then high)
//   * narrow read on both halves
//   * burst write of 4 beats -> per-beat lite writes in order
//   * burst read of 4 beats -> one lite read per beat, lane by address
//   * lite slave error propagation -> SLVERR on the full side
//   * unaligned-start write/read bursts: beats after the first aligned to the
//     transfer size (P0-12 board finding, 2026-09-29)
// Pass token: NG_LITE_TB_PASS
`timescale 1ns / 1ps

module tb_axi64_to_lite32;
  logic clk = 0, rst_n = 0;
  always #2 clk = ~clk;

  logic [3:0]  s_awid;    logic [19:0] s_awaddr; logic [7:0] s_awlen;
  logic [2:0]  s_awsize;  logic        s_awvalid, s_awready;
  logic [63:0] s_wdata;   logic [7:0]  s_wstrb;   logic s_wlast, s_wvalid, s_wready;
  logic [3:0]  s_bid;     logic [1:0]  s_bresp;   logic s_bvalid, s_bready;
  logic [3:0]  s_arid;    logic [19:0] s_araddr;  logic [7:0] s_arlen;
  logic [2:0]  s_arsize;  logic        s_arvalid, s_arready;
  logic [3:0]  s_rid;     logic [63:0] s_rdata;   logic [1:0] s_rresp;
  logic        s_rlast, s_rvalid, s_rready;

  logic [19:0] m_awaddr;  logic        m_awvalid, m_awready;
  logic [31:0] m_wdata;   logic [3:0]  m_wstrb;   logic m_wvalid, m_wready;
  logic [1:0]  m_bresp;   logic        m_bvalid, m_bready;
  logic [19:0] m_araddr;  logic        m_arvalid, m_arready;
  logic [31:0] m_rdata;   logic [1:0]  m_rresp;   logic m_rvalid, m_rready;

  // error injection
  logic err_mode = 0;

  ng_axi64_to_lite32 #(.AW_W(20)) dut (
    .clk(clk), .rst_n(rst_n),
    .s_awid(s_awid), .s_awaddr(s_awaddr), .s_awlen(s_awlen), .s_awsize(s_awsize),
    .s_awvalid(s_awvalid), .s_awready(s_awready),
    .s_wdata(s_wdata), .s_wstrb(s_wstrb), .s_wlast(s_wlast),
    .s_wvalid(s_wvalid), .s_wready(s_wready),
    .s_bid(s_bid), .s_bresp(s_bresp), .s_bvalid(s_bvalid), .s_bready(s_bready),
    .s_arid(s_arid), .s_araddr(s_araddr), .s_arlen(s_arlen), .s_arsize(s_arsize),
    .s_arvalid(s_arvalid), .s_arready(s_arready),
    .s_rid(s_rid), .s_rdata(s_rdata), .s_rresp(s_rresp),
    .s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),
    .m_awaddr(m_awaddr), .m_awprot(), .m_awvalid(m_awvalid), .m_awready(m_awready),
    .m_wdata(m_wdata), .m_wstrb(m_wstrb), .m_wvalid(m_wvalid), .m_wready(m_wready),
    .m_bresp(m_bresp), .m_bvalid(m_bvalid), .m_bready(m_bready),
    .m_araddr(m_araddr), .m_arprot(), .m_arvalid(m_arvalid), .m_arready(m_arready),
    .m_rdata(m_rdata), .m_rresp(m_rresp), .m_rvalid(m_rvalid), .m_rready(m_rready)
  );

  int errors = 0;
  task automatic fail(input string w);
    $display("TB_FAIL %s", w); errors++;
  endtask

  // ---- lite slave: memory + in-order responses with occasional wait states
  // AW and W are accepted independently (AXI-lite allows either order), each
  // latched until the pair is complete; the write lands when both are held.
  logic [31:0] lmem [0:1023];
  logic        sl_aw_h, sl_w_h;
  logic [19:0] sl_aw_a;
  logic [31:0] sl_w_d;
  int          sl_wr_n = 0;   // completed lite writes (catches duplicated AW/W)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m_awready <= 0; m_wready <= 0; m_bvalid <= 0;
      m_arready <= 0; m_rvalid <= 0;
      sl_aw_h <= 0; sl_w_h <= 0;
    end else begin
      m_awready <= !sl_aw_h && !(m_awvalid && m_awready) && ($urandom_range(0, 2) != 0);
      m_wready  <= !sl_w_h  && !(m_wvalid && m_wready)   && ($urandom_range(0, 2) != 0);
      m_arready <= ($urandom_range(0, 2) != 0);
      if (m_awvalid && m_awready) begin
        if (sl_aw_h) fail("second AW before the previous write completed");
        sl_aw_h <= 1; sl_aw_a <= m_awaddr;
      end
      if (m_wvalid && m_wready) begin
        if (sl_w_h) fail("second W before the previous write completed");
        sl_w_h <= 1; sl_w_d <= m_wdata;
      end
      if (sl_aw_h && sl_w_h && !m_bvalid) begin
        lmem[sl_aw_a[11:2]] <= sl_w_d;
        m_bvalid <= 1;
        sl_wr_n  <= sl_wr_n + 1;
        sl_aw_h <= 0; sl_w_h <= 0;
      end
      if (m_bvalid && m_bready) begin
        m_bvalid <= 0; m_bresp <= err_mode ? 2'b10 : 2'b00;
      end
      if (m_arvalid && m_arready && !m_rvalid) begin
        m_rvalid <= 1;
        m_rdata  <= {m_araddr[15:0], 16'hBEEF};
        m_rresp  <= err_mode ? 2'b10 : 2'b00;
      end
      if (m_rvalid && m_rready) m_rvalid <= 0;
    end
  end

  // ---- full-side driver
  task automatic fwr(input logic [19:0] addr, input logic [63:0] data,
                     input logic [7:0] strb, input int len);
    @(posedge clk);
    s_awid <= 4'h7; s_awaddr <= addr; s_awlen <= len[7:0]; s_awsize <= 3;
    s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    for (int b = 0; b <= len; b++) begin
      @(posedge clk);
      s_wdata <= data + 64'(b); s_wstrb <= strb; s_wlast <= (b == len); s_wvalid <= 1;
      do @(posedge clk); while (!s_wready);
      s_wvalid <= 0;
    end
    do @(posedge clk); while (!s_bvalid);
    if (s_bresp != 2'b00) fail("write resp");
    do @(posedge clk); while (s_bvalid);
  endtask

  task automatic frd(input logic [19:0] addr, input int len);
    @(posedge clk);
    s_arid <= 4'h5; s_araddr <= addr; s_arlen <= len[7:0]; s_arsize <= 3;
    s_arvalid <= 1;
    do @(posedge clk); while (!s_arready);
    s_arvalid <= 0;
    for (int b = 0; b <= len; b++) begin
      do @(posedge clk); while (!s_rvalid);
      if (!s_rlast && b == len) fail("read last missing");
      if (s_rresp != 2'b00) fail("read resp");
      @(posedge clk);
    end
    do @(posedge clk); while (s_rvalid);
  endtask

  // burst with a per-beat strobe; the start address may be unaligned (the
  // XDMA bridge presents the TLP's dword address, e.g. 0x..C for a 2-DW write
  // crossing an 8-byte boundary -- P0-12 board finding)
  task automatic fwr_s(input logic [19:0] addr, input logic [63:0] data,
                       input logic [7:0] strb [4], input int len);
    @(posedge clk);
    s_awid <= 4'h7; s_awaddr <= addr; s_awlen <= len[7:0]; s_awsize <= 3;
    s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    for (int b = 0; b <= len; b++) begin
      @(posedge clk);
      s_wdata <= data + {32'(b), 32'(b)}; s_wstrb <= strb[b]; s_wlast <= (b == len); s_wvalid <= 1;
      do @(posedge clk); while (!s_wready);
      s_wvalid <= 0;
    end
    do @(posedge clk); while (!s_bvalid);
    if (s_bresp != 2'b00) fail("write resp");
    do @(posedge clk); while (s_bvalid);
  endtask

  // burst read returning each beat's data (the lite stub answers
  // {araddr[15:0], 16'hBEEF}, so the data identifies the address read)
  logic [63:0] rd_beats [4];
  task automatic frd_d(input logic [19:0] addr, input int len);
    @(posedge clk);
    s_arid <= 4'h5; s_araddr <= addr; s_arlen <= len[7:0]; s_arsize <= 3;
    s_arvalid <= 1;
    do @(posedge clk); while (!s_arready);
    s_arvalid <= 0;
    for (int b = 0; b <= len; b++) begin
      do @(posedge clk); while (!s_rvalid);
      rd_beats[b] = s_rdata;
      @(posedge clk);
    end
    do @(posedge clk); while (s_rvalid);
  endtask

  // write expecting SLVERR (error injection)
  task automatic fwr_err(input logic [19:0] addr);
    @(posedge clk);
    s_awid <= 4'h7; s_awaddr <= addr; s_awlen <= 0; s_awsize <= 3;
    s_awvalid <= 1;
    do @(posedge clk); while (!s_awready);
    s_awvalid <= 0;
    @(posedge clk);
    s_wdata <= 64'hDEAD_BEEF_DEAD_BEEF; s_wstrb <= 8'hFF; s_wlast <= 1; s_wvalid <= 1;
    do @(posedge clk); while (!s_wready);
    s_wvalid <= 0;
    do @(posedge clk); while (!s_bvalid);
    if (s_bresp != 2'b10) fail("error not propagated as SLVERR");
    do @(posedge clk); while (s_bvalid);
  endtask

  logic [63:0] d;
  initial begin
    s_awvalid = 0; s_wvalid = 0; s_arvalid = 0; s_bready = 1; s_rready = 1;
    s_awid = 0; s_awaddr = 0; s_awlen = 0; s_awsize = 0;
    s_wdata = 0; s_wstrb = 0; s_wlast = 0;
    s_arid = 0; s_araddr = 0; s_arlen = 0; s_arsize = 0;
    repeat (3) @(posedge clk);
    rst_n <= 1;
    repeat (5) @(posedge clk);

    // addresses chosen so the lite slave's [11:2] index does not collide
    fwr(20'h0000_1000, 64'hAAAA_BBBB_CCCC_DDDD, 8'h0F, 0);   // low half  -> idx 0x000
    fwr(20'h0000_2004, 64'hAAAA_BBBB_CCCC_DDDD, 8'hF0, 0);   // high half -> idx 0x001
    fwr(20'h0000_3100, 64'h1111_2222_3333_4444, 8'hFF, 0);   // full 64   -> idx 0x040/0x041
    fwr(20'h0000_4400, 64'h0, 8'hFF, 3);                     // burst     -> idx 0x100..
    frd (20'h0000_1000, 0);                                  // narrow reads
    frd (20'h0000_2004, 0);
    frd (20'h0000_5100, 3);                                  // burst read
    if (lmem[12'h000] !== 32'hCCCC_DDDD) fail("low half not written");
    if (lmem[12'h001] !== 32'hAAAA_BBBB) fail("high half not written");
    if (lmem[12'h040] !== 32'h3333_4444) fail("full-64 low");
    if (lmem[12'h041] !== 32'h1111_2222) fail("full-64 high");
    if (lmem[12'h100] !== 32'h0) fail("burst beat0");         // data+b
    // unaligned-start INCR bursts: beats after the first are aligned to the
    // transfer size (AXI A3.4.1), i.e. 0x700C -> 0x7010, 0x7204 -> 0x7208 -> 0x7210
    fwr_s(20'h0000_700C, 64'h5555_6666_7777_8888, '{8'hF0, 8'h0F, 8'h00, 8'h00}, 1);
    if (lmem[12'h003] !== 32'h5555_6666) fail("unaligned burst beat0 (0x700C)");
    if (lmem[12'h004] !== 32'h7777_8889) fail("unaligned burst beat1 must land at 0x7010");
    fwr_s(20'h0000_7204, 64'h9999_AAAA_BBBB_CCCC, '{8'hF0, 8'hFF, 8'h0F, 8'h00}, 2);
    if (lmem[12'h081] !== 32'h9999_AAAA) fail("unaligned burst beat0 (0x7204)");
    if (lmem[12'h082] !== 32'hBBBB_CCCD) fail("unaligned burst beat1 low must land at 0x7208");
    if (lmem[12'h083] !== 32'h9999_AAAB) fail("unaligned burst beat1 high must land at 0x720C");
    if (lmem[12'h084] !== 32'hBBBB_CCCE) fail("unaligned burst beat2 must land at 0x7210");
    frd_d(20'h0000_5304, 2);
    if (rd_beats[0] !== {16'h5304, 16'hBEEF, 32'h0}) fail("unaligned read beat0 (0x5304, high lane)");
    if (rd_beats[1][31:0] !== {16'h5308, 16'hBEEF}) fail("unaligned read beat1 must read 0x5308");
    if (rd_beats[2][31:0] !== {16'h5310, 16'hBEEF}) fail("unaligned read beat2 must read 0x5310");
    // error propagation
    err_mode <= 1;
    @(posedge clk);
    fwr_err(20'h0000_6000);
    // 1 (low) + 1 (high) + 2 (full) + 8 (4-beat full burst)
    // + 2 (0x700C burst) + 4 (0x7204 burst) + 2 (error write)
    repeat (5) @(posedge clk);
    if (sl_wr_n != 20) fail($sformatf("lite write count %0d, expected 20", sl_wr_n));
    if (errors == 0) $display("NG_LITE_TB_PASS");
    else $display("NG_LITE_TB_FAIL errors=%0d", errors);
    $finish;
  end

  initial begin
    #1_000_000;
    $display("NG_LITE_TB_FAIL watchdog");
    $finish;
  end
endmodule
