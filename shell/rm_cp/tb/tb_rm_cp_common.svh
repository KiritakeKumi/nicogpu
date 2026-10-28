// tb_rm_cp_common.svh -- testbench tasks shared by tb_rm_cp.sv and tb_rm_sp.sv (rtl/shell/rm_sp/tb), `include d inside
// the testbench module after the DUT and the c3 / host slave models: the C1/C2/C4 monitor, check, the BAR0 (lw / lr /
// cpw / cpr) and BAR2 (b2_write / b2_read) drivers, the C3 / host memory accessors, put_c3, wait_reg, the command ring
// helpers (ring_put / ring_flush / fence_*), boot_fw and the IB builders (ib_* / run_ib).  Needs the module's `errors`,
// `checks`, the DUT port signals (tb_rm_*_decls.svh) and u_c3 / u_host.

  // C1/C2/C4 must stay silent
  always @(posedge clk_l2) if (m_axi_c1_awvalid | m_axi_c1_arvalid | m_axi_c2_awvalid | m_axi_c2_arvalid |
                               m_axi_c4_awvalid | m_axi_c4_arvalid) begin errors++; $display("FAIL: unused channel active"); end

  task automatic check(input bit cond, input string what);
    checks++;
    if (!cond) begin errors++; if (errors < 30) $display("FAIL @%0t: %s", $time, what); end
  endtask

  // ---- bus drivers.  Verilator runs a non-blocking assignment in a task/initial as a blocking one, so the TB
  // never drives at a clock edge: it drives and samples at negedge, when everything the DUT's flops did at the
  // last posedge is stable.  `valid` is set at a negedge; if `ready` is 1 at that negedge the handshake happens at the
  // next posedge, otherwise the TB waits for a negedge with ready = 1 (ready of all our DUT channels is registered).
  `define HS(rdy) begin while (!(rdy)) @(negedge clk_l2); @(negedge clk_l2); end

  // ---- BAR0 (AXI4-Lite), serialised
  task automatic lw(input logic [19:0] a, input logic [31:0] d);
    @(negedge clk_l2);
    s_axil_ctrl_awaddr = a; s_axil_ctrl_awvalid = 1'b1; s_axil_ctrl_wdata = d; s_axil_ctrl_wstrb = 4'hF;
    s_axil_ctrl_wvalid = 1'b1; s_axil_ctrl_bready = 1'b1;
    fork
      begin `HS(s_axil_ctrl_awready) s_axil_ctrl_awvalid = 1'b0; end
      begin `HS(s_axil_ctrl_wready)  s_axil_ctrl_wvalid = 1'b0; end
    join
    while (!s_axil_ctrl_bvalid) @(negedge clk_l2);
    check(s_axil_ctrl_bresp == 2'b00, "axil bresp");
    @(negedge clk_l2);
    s_axil_ctrl_bready = 1'b0;
  endtask

  task automatic lr(input logic [19:0] a, output logic [31:0] d);
    @(negedge clk_l2);
    s_axil_ctrl_araddr = a; s_axil_ctrl_arvalid = 1'b1; s_axil_ctrl_rready = 1'b1;
    `HS(s_axil_ctrl_arready)
    s_axil_ctrl_arvalid = 1'b0;
    while (!s_axil_ctrl_rvalid) @(negedge clk_l2);
    d = s_axil_ctrl_rdata;
    check(s_axil_ctrl_rresp == 2'b00, "axil rresp");
    @(negedge clk_l2);
    s_axil_ctrl_rready = 1'b0;
  endtask

  task automatic cpw(input logic [12:0] off, input logic [31:0] d); lw({7'h0, off}, d); endtask
  task automatic cpr(input logic [12:0] off, output logic [31:0] d); lr({7'h0, off}, d); endtask

  // ---- BAR2: write n x 64-bit as one burst / read back
  task automatic b2_write(input logic [27:0] a, input logic [63:0] d [], input logic [7:0] strb = 8'hFF,
                          input logic [2:0] size = 3'd3, output logic [1:0] resp);
    int n = d.size();
    @(negedge clk_l2);
    s_axi_bar2_awid = 4'h5; s_axi_bar2_awaddr = a; s_axi_bar2_awlen = 8'(n - 1); s_axi_bar2_awsize = size;
    s_axi_bar2_awburst = 2'b01; s_axi_bar2_awvalid = 1'b1; s_axi_bar2_bready = 1'b1;
    `HS(s_axi_bar2_awready)
    s_axi_bar2_awvalid = 1'b0;
    for (int i = 0; i < n; i++) begin
      s_axi_bar2_wdata = d[i]; s_axi_bar2_wstrb = strb; s_axi_bar2_wlast = (i == n - 1); s_axi_bar2_wvalid = 1'b1;
      `HS(s_axi_bar2_wready)
      s_axi_bar2_wvalid = 1'b0;
    end
    while (!s_axi_bar2_bvalid) @(negedge clk_l2);
    resp = s_axi_bar2_bresp;
    check(s_axi_bar2_bid == 4'h5, "bar2 bid");
    @(negedge clk_l2);
    s_axi_bar2_bready = 1'b0;
  endtask

  task automatic b2_read(input logic [27:0] a, input int n, output logic [63:0] d [], output logic [1:0] resp,
                         input logic [2:0] size = 3'd3);
    d = new[n];
    resp = 2'b00;
    @(negedge clk_l2);
    s_axi_bar2_arid = 4'h9; s_axi_bar2_araddr = a; s_axi_bar2_arlen = 8'(n - 1); s_axi_bar2_arsize = size;
    s_axi_bar2_arburst = 2'b01; s_axi_bar2_arvalid = 1'b1; s_axi_bar2_rready = 1'b1;
    `HS(s_axi_bar2_arready)
    s_axi_bar2_arvalid = 1'b0;
    for (int i = 0; i < n; i++) begin
      while (!s_axi_bar2_rvalid) @(negedge clk_l2);
      d[i] = s_axi_bar2_rdata;
      if (s_axi_bar2_rresp != 2'b00) resp = s_axi_bar2_rresp;
      check(s_axi_bar2_rlast == (i == n - 1), "bar2 rlast");
      check(s_axi_bar2_rid == 4'h9, "bar2 rid");
      @(negedge clk_l2);
    end
    s_axi_bar2_rready = 1'b0;
  endtask

  // ---- memory helpers (direct access to the slave models)
  function automatic logic [63:0] c3_q(input longint unsigned off);
    logic [127:0] b = u_c3.peek(off >> 4);
    return off[3] ? b[127:64] : b[63:0];
  endfunction
  function automatic logic [63:0] host_q(input longint unsigned a);
    return u_host.peek(a >> 3);
  endfunction
  function automatic void c3_put32(input longint unsigned off, input logic [31:0] v);
    logic [127:0] b = u_c3.peek(off >> 4);
    b[32*off[3:2] +: 32] = v;
    u_c3.poke(off >> 4, b);
  endfunction

  localparam longint unsigned SYS_BASE  = 64'h10_0000_0000;
  localparam longint unsigned HOST_BASE = 64'h8000_0000_0000;
  localparam longint unsigned FW_OFF    = 64'h1000;
  localparam longint unsigned RING_OFF  = 64'h20000;
  localparam longint unsigned FENCE_HOST = 64'h1_0000;       // host-memory offset of the fence page
  localparam logic [27:0] WIN_LO = 28'hF00_0000;             // 256 MiB - 16 MiB

  logic [31:0] v;
  logic [31:0] fw_img [0:4095];
  int fw_bytes;

  // push a 32-bit word array into C3 through the BAR2 window (8 qwords per burst)
  task automatic put_c3(input longint unsigned off, input logic [31:0] w [], input int nw);
    logic [63:0] d [];
    logic [63:0] one [];
    logic [1:0] r;
    int i = 0;
    if (off[2] && nw > 0) begin               // odd first word: one 4-byte beat in the upper lane
      one = new[1]; one[0] = {w[0], 32'h0};
      b2_write(WIN_LO + 28'(off), one, 8'hF0, 3'd2, r);
      check(r == 2'b00, "put_c3 head bresp");
      i = 1;
    end
    while (nw - i >= 2) begin
      int n = ((nw - i) / 2 > 8) ? 8 : (nw - i) / 2;
      d = new[n];
      for (int k = 0; k < n; k++) d[k] = {w[i + 2 * k + 1], w[i + 2 * k]};
      b2_write(WIN_LO + 28'(off + 4 * i), d, 8'hFF, 3'd3, r);
      check(r == 2'b00, "put_c3 bresp");
      i += 2 * n;
    end
    if (i < nw) begin                          // odd last word: lower lane
      one = new[1]; one[0] = {32'h0, w[i]};
      b2_write(WIN_LO + 28'(off + 4 * i), one, 8'h0F, 3'd2, r);
      check(r == 2'b00, "put_c3 tail bresp");
    end
  endtask

  task automatic wait_reg(input logic [12:0] off, input logic [31:0] mask, input logic [31:0] val, input int max_us,
                          output bit ok);
    logic [31:0] r;
    ok = 0;
    for (int t = 0; t < max_us && !ok; t++) begin
      cpr(off, r);
      if ((r & mask) == val) ok = 1;
      else repeat (50) @(posedge clk_l2);
    end
  endtask

  // ring helper (words appended at ring_wp, written through the window)
  logic [31:0] ring_buf [0:255];
  int ring_wp = 0;
  task automatic ring_put(input logic [31:0] w); ring_buf[ring_wp % 256] = w; ring_wp++; endtask
  task automatic ring_flush(input int from, input int to_);
    logic [31:0] t [];
    t = new[to_ - from];
    for (int i = from; i < to_; i++) t[i - from] = ring_buf[i % 256];
    put_c3(RING_OFF + 4 * from, t, to_ - from);
  endtask
  task automatic fence_prep(input longint unsigned seq);
    int s = ring_wp;
    ring_put(32'h0005_0002);
    ring_put((HOST_BASE + FENCE_HOST) & 32'hFFFF_FFFF); ring_put(((HOST_BASE + FENCE_HOST) >> 32) & 32'hFFFF_FFFF);
    ring_put(seq[31:0]); ring_put(seq[63:32]);
    ring_put(32'h3);                                     // 64-bit + IRQ
    ring_flush(s, ring_wp);
  endtask
  task automatic fence_job(input longint unsigned seq);
    fence_prep(seq);
    cpw(13'h1000, ring_wp);
  endtask

  task automatic boot_fw(input longint unsigned base, input int bytes, output bit ok, output logic [31:0] st,
                         input int tries = 4000);
    cpw(13'h0114, 32'h8);                                // CORE_RESET
    cpw(13'h0140, (SYS_BASE + base) & 32'hFFFF_FFFF); cpw(13'h0144, (SYS_BASE + base) >> 32);
    cpw(13'h0148, bytes);
    cpw(13'h0114, 32'h0);                                // release: the boot loader runs
    wait_reg(13'h0138, 32'h0000_FF01, 32'h1, tries, ok); // READY with BOOT_ERR == 0
    cpr(13'h0138, st);
  endtask

  // ---- MMU test helpers: page-table qwords and IB contents go into C3 through the BAR2 window
  localparam longint unsigned PT_ROOT = 64'h30000, PT_L2 = 64'h31000, PT_L1 = 64'h32000, PT_L0 = 64'h33000;
  localparam longint unsigned IB_PG = 64'h34000, DATA_PG = 64'h35000, RO_PG = 64'h37000;
  localparam longint unsigned VA0 = 64'h1_0000_0000;            // user VA base (4 GiB): L2 index 4, L1 index 0
  localparam longint unsigned HPG1 = 64'h2_0000, HPG2 = 64'h3_0000;   // host pages (host-memory offsets)

  task automatic put_q(input longint unsigned off, input logic [63:0] q);
    logic [31:0] t [];
    t = new[2]; t[0] = q[31:0]; t[1] = q[63:32];
    put_c3(off, t, 2);
  endtask

  logic [31:0] ib_buf [0:63];
  int ib_n = 0;
  task automatic ib_fence(input longint unsigned va, input logic [63:0] val);
    ib_buf[ib_n] = 32'h0005_0002; ib_buf[ib_n + 1] = va[31:0]; ib_buf[ib_n + 2] = va[63:32];
    ib_buf[ib_n + 3] = val[31:0]; ib_buf[ib_n + 4] = val[63:32]; ib_buf[ib_n + 5] = 32'h1;   // 64-bit, no IRQ
    ib_n += 6;
  endtask
  task automatic ib_write2(input longint unsigned va, input logic [31:0] d0, input logic [31:0] d1);
    ib_buf[ib_n] = 32'h0004_000E; ib_buf[ib_n + 1] = va[31:0]; ib_buf[ib_n + 2] = va[63:32];
    ib_buf[ib_n + 3] = d0; ib_buf[ib_n + 4] = d1;
    ib_n += 5;
  endtask
  task automatic ib_waitmem0(input longint unsigned va);          // wait until (*(u32*)va & ~0) == 0
    ib_buf[ib_n] = 32'h0007_0009; ib_buf[ib_n + 1] = va[31:0]; ib_buf[ib_n + 2] = va[63:32];
    ib_buf[ib_n + 3] = 32'h0; ib_buf[ib_n + 4] = 32'h0; ib_buf[ib_n + 5] = 32'hFFFF_FFFF;
    ib_buf[ib_n + 6] = 32'h0; ib_buf[ib_n + 7] = 32'h0;            // MASK_HI, CTRL = EQ / 32 bit
    ib_n += 8;
  endtask
  task automatic ib_pad();
    if (ib_n[0]) begin ib_buf[ib_n] = 32'h0; ib_n++; end            // NOP
  endtask
  task automatic ib_commit();
    logic [31:0] t [];
    t = new[ib_n];
    for (int i = 0; i < ib_n; i++) t[i] = ib_buf[i];
    put_c3(IB_PG, t, ib_n);
  endtask
  task automatic run_ib_noerrwait(input int vmid, input longint unsigned va, input int ndw);
    int s = ring_wp;
    ring_put(32'h0004_0001); ring_put(va[31:0]); ring_put({16'h0, va[47:32]}); ring_put(ndw); ring_put(vmid);
    ring_flush(s, ring_wp);
    cpw(13'h1000, ring_wp);
  endtask
  task automatic run_ib(input int vmid, input longint unsigned va, input int ndw, output bit ok);
    run_ib_noerrwait(vmid, va, ndw);
    wait_reg(13'h010c, 32'hFFFF_FFFF, 32'(ring_wp), 4000, ok);
  endtask
