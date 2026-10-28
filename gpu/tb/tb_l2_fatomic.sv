// tb_l2_fatomic.sv -- FP32 atomics: (1) the AMO ALU against the model-
// semantics reference vectors (sim/l2/gen_fatomic_vec.py: fadd/fmin/fmax,
// bit-exact incl. NaN/-0/subnormal/overflow), (2) a subset of the same vectors
// through the full slice (miss/fill/evict path, unaligned offsets, neighbour
// bytes untouched, rtn/non-rtn), (3) FP32 op error cases.
// Stimulus and sampling on negedge.
// Vectors: +FVEC=<file> (default fa_vec.hex), 128 bit per line
//   {op, x(old), y(operand), expected}.   Expect NG_L2_FATOMIC_TB_PASS.
`timescale 1ns/1ps
module tb_l2_fatomic;
  import ng_noc_pkg::*;
  import ng_l2_addrmap::*;

  localparam int SETS = 16, WAYS = 4, LINE_BYTES = 128;
  localparam int AXI_AW = 33, AXI_DW = 512, AXI_IW = 6;
  localparam int NVMAX = 70000;
  // op codes, written out here on purpose (independent of the RTL pkg)
  localparam logic [4:0] OP_FADD = 5'd13, OP_FMIN = 5'd14, OP_FMAX = 5'd15;

  logic clk = 1'b0;
  logic rst_n;
  always #5 clk = ~clk;

  // ---- ALU DUT ---------------------------------------------------------------
  logic [4:0]  op;
  logic        is64;
  logic [63:0] a, b, cmp;
  logic [63:0] r;
  logic        cas_hit;
  ng_l2_amo_alu u_alu (.*);

  // ---- slice DUT + DDR ---------------------------------------------------------
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

  int errors, checks;
  logic [127:0] vec [NVMAX];
  int nvec;

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

  function automatic flit_t mk(input logic [1:0] kind, input logic [47:0] pa,
                               input logic [4:0] o, input logic [1:0] sz,
                               input logic rtn, input logic [63:0] d,
                               input logic [7:0] be);
    return '{cls: CLS_REQ, kind: kind, op: o, size: sz, rtn: rtn, addr: pa,
             src: 6'd1, dst: 6'd0, tag: 8'h5A, data: d, be: be, default: '0};
  endfunction

  task automatic xact(input flit_t f, output flit_t rr);
    send_req(f);
    recv_resp(rr);
  endtask

  // line-granular PAs inside slice 0 granules (pa[13:12]==0), 200 lines so
  // that the 16x4 cache evicts continuously
  function automatic logic [47:0] line_pa(input int n);
    return (48'(n >> 5) << 14) | (48'(n & 31) << 7);
  endfunction

  initial begin : alu_part
    string fvec;
    errors = 0;
    checks = 0;
    op = '0; is64 = 1'b0; a = '0; b = '0; cmp = '0;
    if (!$value$plusargs("FVEC=%s", fvec)) fvec = "fa_vec.hex";
    for (int i = 0; i < NVMAX; i++) vec[i] = 'x;
    $readmemh(fvec, vec);
    nvec = 0;
    while (nvec < NVMAX && ^vec[nvec] !== 1'bx) nvec++;
    $display("TB: %0d FP32 atomic vectors", nvec);
    if (nvec == 0) begin
      errors++;
      $display("TB_FAIL: no vectors loaded from %s", fvec);
    end

    rst_n = 1'b0;
    repeat (4) @(negedge clk);

    // ---- (1) ALU, every vector -----------------------------------------------
    for (int i = 0; i < nvec; i++) begin
      op   = vec[i][127:96];
      a    = {32'd0, vec[i][95:64]};
      b    = {32'd0, vec[i][63:32]};
      is64 = 1'b0;
      cmp  = '0;
      @(negedge clk);
      checks++;
      if (r !== {32'd0, vec[i][31:0]}) begin
        errors++;
        if (errors < 40)
          $display("TB_FAIL alu op=%0d x=%h y=%h r=%h want=%h", op, a[31:0],
                   b[31:0], r[31:0], vec[i][31:0]);
      end
    end
    // FP ops must leave the 64-bit lane untouched-by-default (no FP64 atomics)
    for (int i = 0; i < 64; i++) begin
      op = OP_FADD + 5'(i % 3); a = 64'h0123456789ABCDEF; b = 64'h3F8000003F800000;
      is64 = 1'b1;
      @(negedge clk);
      checks++;
      if (r !== a) begin errors++; $display("TB_FAIL alu 64-bit FP op changed data"); end
    end
    is64 = 1'b0;

    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ---- (2) slice, subset of vectors -----------------------------------------
    begin
      int stride, nline, nsl;
      flit_t rr;
      stride = (nvec > 1500) ? nvec / 1500 : 1;
      nline = 0;
      nsl = 0;
      for (int i = $urandom_range(stride - 1); i < nvec; i += stride) begin
        logic [47:0] pa;
        logic [31:0] x, y, want;
        logic [63:0] sent;
        logic        rtn;
        logic [6:0]  off;
        x = vec[i][95:64];
        y = vec[i][63:32];
        want = vec[i][31:0];
        off = 7'($urandom_range(120));          // 8 B window inside the line
        pa = line_pa(nline % 200) | 48'(off);
        nline++;
        rtn = $urandom_range(1);
        sent = {$urandom(), x};
        // sentinel 8 B: {random hi word, x}
        xact(mk(KIND_WR, pa, AMO_NONE_c(), SZ_8, 1'b0, sent, 8'hFF), rr);
        xact(mk(KIND_AMO, pa, vec[i][100:96], SZ_4, rtn, {32'd0, y}, 8'h00), rr);
        checks++;
        if (rr.err != ERR_NONE) begin
          errors++; $display("TB_FAIL slice: err=%0d op=%0d", rr.err, vec[i][100:96]);
        end else if (rr.data !== (rtn ? {32'd0, x} : 64'd0)) begin
          errors++;
          $display("TB_FAIL slice: rtn data=%h want=%h", rr.data, rtn ? {32'd0, x} : 64'd0);
        end
        xact(mk(KIND_RD, pa, AMO_NONE_c(), SZ_8, 1'b0, '0, 8'h00), rr);
        checks++;
        if (rr.data !== {sent[63:32], want}) begin
          errors++;
          $display("TB_FAIL slice: op=%0d x=%h y=%h mem=%h want=%h",
                   vec[i][100:96], x, y, rr.data, {sent[63:32], want});
        end
        nsl++;
      end
      $display("TB: %0d vectors through the slice", nsl);
    end

    // ---- (3) FP op error cases -------------------------------------------------
    begin
      flit_t rr;
      logic [4:0] fops [3];
      fops = '{OP_FADD, OP_FMIN, OP_FMAX};
      for (int k = 0; k < 3; k++) begin
        xact(mk(KIND_AMO, 48'h0000_0000_1000, fops[k], SZ_8, 1'b1, 64'd1, 8'h00), rr);
        checks++;
        if (rr.err != ERR_BAD_PA) begin errors++; $display("TB_FAIL: FP64 op %0d not rejected", fops[k]); end
        xact(mk(KIND_AMO, 48'h0000_0000_1000, fops[k], SZ_2, 1'b1, 64'd1, 8'h00), rr);
        checks++;
        if (rr.err != ERR_BAD_PA) begin errors++; $display("TB_FAIL: FP16 op %0d not rejected", fops[k]); end
        xact(mk(KIND_AMO, 48'h0000_0000_107E, fops[k], SZ_4, 1'b1, 64'd1, 8'h00), rr);
        checks++;
        if (rr.err != ERR_CROSS) begin errors++; $display("TB_FAIL: cross-line FP op %0d not rejected", fops[k]); end
      end
      // codes 16..30 stay undefined
      xact(mk(KIND_AMO, 48'h0000_0000_1000, 5'd16, SZ_4, 1'b1, 64'd1, 8'h00), rr);
      checks++;
      if (rr.err != ERR_BAD_PA) begin errors++; $display("TB_FAIL: op 16 not rejected"); end
    end

    if (errors == 0)
      $display("NG_L2_FATOMIC_TB_PASS: %0d checks, 0 errors", checks);
    else
      $display("NG_L2_FATOMIC_TB_FAIL: %0d errors of %0d checks", errors, checks);
    $finish;
  end

  function automatic logic [4:0] AMO_NONE_c();
    return AMO_NONE;
  endfunction

  // watchdog
  initial begin
    #200ms;
    $display("NG_L2_FATOMIC_TB_FAIL: timeout");
    $finish;
  end
endmodule
