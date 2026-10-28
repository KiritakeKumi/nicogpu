// tb_cp_boot.sv -- unit test of the CP boot loader ng_cp_boot (boot ROM).
//   Vectors: tools/ngcf/gen_boot_cases.py cases_cur (+ params.txt: seed lat_max rdy_pct), see run_tb_cp_boot.sh.
// Memory model: 64-bit words (C3), random ready and 1..LAT_MAX cycle latency, optional faulting qword.  Checks per
// case: err code / ok, IMEM and DMEM contents (text, data, zeroed bss rounded up to words, nothing else written),
// boot address; plus a mid-load abort followed by a complete load, and a memory that never answers.
// Prints NG_CP_BOOT_TB_PASS.
`timescale 1ns/1ps
module tb_cp_boot;
  import ng_cp_pkg::*;
  int SEED = 1, LAT_MAX = 4, RDY_PCT = 70;     // +SEED= +LAT_MAX= +RDY_PCT=
  localparam int IW = 16384, DW = 8192, MQ = 32768;

  int errors = 0;
  logic clk = 0, rst_n = 0, run = 0;
  always #5 clk = ~clk;

  logic [63:0] fw_base;
  logic [31:0] fw_size;
  logic        m_req_valid, m_req_ready;
  mem_req_t    m_req;
  logic        m_rsp_valid, m_rsp_fault;
  logic [63:0] m_rsp_data;
  logic        ld_we, ld_dmem;
  logic [31:0] ld_addr, ld_data, boot_addr;
  logic        busy, ok, err_vld;
  logic [7:0]  err;

  ng_cp_boot #(.IMEM_WORDS(IW), .DMEM_WORDS(DW)) dut (
    .clk, .rst_n, .run_i(run), .fw_base_i(fw_base), .fw_size_i(fw_size),
    .m_req_valid_o(m_req_valid), .m_req_ready_i(m_req_ready), .m_req_o(m_req),
    .m_rsp_valid_i(m_rsp_valid), .m_rsp_fault_i(m_rsp_fault), .m_rsp_data_i(m_rsp_data),
    .ld_we_o(ld_we), .ld_dmem_o(ld_dmem), .ld_addr_o(ld_addr), .ld_data_o(ld_data),
    .boot_addr_o(boot_addr), .busy_o(busy), .ok_o(ok), .err_vld_o(err_vld), .err_o(err)
  );

  // ---- memory model --------------------------------------------------------------------
  logic [63:0] mem [MQ];
  int          fault_q = -1;           // -1 none, -2 never answers
  int          reqs;
  int unsigned rng = 1;
  function automatic int unsigned rnd();
    rng = rng ^ (rng << 13); rng = rng ^ (rng >> 17); rng = rng ^ (rng << 5);
    return rng;
  endfunction

  logic        busy_m;
  int          wait_c;
  logic [47:0] a_q;
  logic [7:0]  s_q;
  logic        rdy_r;
  always @(posedge clk) rdy_r <= ((rnd() % 100) < RDY_PCT);
  assign m_req_ready = !busy_m && rdy_r;

  always @(posedge clk) begin
    m_rsp_valid <= 1'b0; m_rsp_fault <= 1'b0; m_rsp_data <= 64'hx;
    if (!rst_n) busy_m <= 1'b0;
    else begin
      if (m_req_valid && m_req_ready) begin
        if (m_req.write || m_req.vmid != 4'd0 || m_req.addr[2:0] != 3'b0) begin
          $display("TB_FAIL: bad request write=%0d vmid=%0d addr=%0h", m_req.write, m_req.vmid, m_req.addr);
          errors++;
        end
        busy_m <= 1'b1; wait_c <= 1 + (rnd() % LAT_MAX); a_q <= m_req.addr; s_q <= m_req.wstrb; reqs++;
        if (m_req.wstrb != 8'hFF && m_req.wstrb != 8'h0F) begin
          $display("TB_FAIL: bad wstrb %h", m_req.wstrb); errors++;
        end
      end else if (busy_m) begin
        if (wait_c > 1) wait_c <= wait_c - 1;
        else if (fault_q != -2) begin
          busy_m <= 1'b0; m_rsp_valid <= 1'b1;
          if (fault_q >= 0 && (a_q >> 3) == 48'(fault_q)) m_rsp_fault <= 1'b1;
          else m_rsp_data <= (a_q >> 3) < MQ ? mem[a_q >> 3] : 64'hDEAD_DEAD_DEAD_DEAD;
        end
      end
    end
  end

  // ---- load-port capture -------------------------------------------------------------------
  logic [31:0] imem [IW];
  logic [31:0] dmem [DW];
  int          nwr_i, nwr_d;
  always @(posedge clk) if (ld_we) begin
    if (ld_dmem) begin
      if (ld_addr[1:0] != 0 || (ld_addr >> 2) >= DW) begin $display("TB_FAIL: bad DMEM ld addr %h", ld_addr); errors++; end
      else begin dmem[ld_addr >> 2] <= ld_data; nwr_d++; end
    end else begin
      if (ld_addr[1:0] != 0 || (ld_addr >> 2) >= IW) begin $display("TB_FAIL: bad IMEM ld addr %h", ld_addr); errors++; end
      else begin imem[ld_addr >> 2] <= ld_data; nwr_i++; end
    end
  end
  // the loader only writes while it owns the port, and never once it reported ok or an error
  always @(posedge clk) if (ld_we && (ok || err_vld)) begin $display("TB_FAIL: write after the end"); errors++; end
  logic err_seen; logic [7:0] err_code;
  always @(posedge clk) if (err_vld) begin err_seen <= 1'b1; err_code <= err; end
  always @(posedge clk) if (ok && err_seen) begin $display("TB_FAIL: ok and error"); errors++; end

  localparam logic [31:0] POISON = 32'hDEAD_BEEF;

  task automatic fill_mems();
    for (int i = 0; i < IW; i++) imem[i] = POISON;
    for (int i = 0; i < DW; i++) dmem[i] = POISON;
    nwr_i = 0; nwr_d = 0;
  endtask

  task automatic stop_and_clear();
    run = 1'b0;
    repeat (4) @(posedge clk);
    err_seen = 1'b0; err_code = 8'h0;
  endtask

  task automatic wait_end(input int max_cycles, output bit timeout);
    int c = 0;
    timeout = 0;
    while (!ok && !err_seen && c < max_cycles) begin @(posedge clk); c++; end
    timeout = !(ok || err_seen);
    repeat (3) @(posedge clk);
  endtask

  task automatic run_case(input int n, input longint base, input int size, input int exp_err, input int fq,
                          input int tw, input int dw, input int bw, input int boot, input string dir);
    string f;
    bit to;
    logic [31:0] exp_w [];
    stop_and_clear();
    fill_mems();
    for (int i = 0; i < MQ; i++) mem[i] = 64'hA5A5_A5A5_A5A5_A5A5;
    f = $sformatf("%s/mem_%0d.hex", dir, n);   $readmemh(f, mem);
    fault_q = fq; fw_base = base[63:0]; fw_size = size[31:0];
    @(posedge clk);
    run = 1'b1;
    wait_end(400000, to);
    if (to) begin $display("TB_FAIL case %0d: timeout", n); errors++; end
    else if (exp_err == 0) begin
      if (!ok || err_seen) begin $display("TB_FAIL case %0d: expected ok, err=%0d", n, err_code); errors++; end
      else begin
        logic [31:0] t [];
        logic [31:0] d [];
        t = new[tw > 0 ? tw : 1];
        d = new[dw > 0 ? dw : 1];
        f = $sformatf("%s/text_%0d.hex", dir, n);  $readmemh(f, t);
        f = $sformatf("%s/data_%0d.hex", dir, n);  $readmemh(f, d);
        for (int i = 0; i < IW; i++) begin
          logic [31:0] e = (i < tw) ? t[i] : POISON;
          if (imem[i] !== e) begin $display("TB_FAIL case %0d: IMEM[%0d]=%h expected %h", n, i, imem[i], e); errors++; if (errors > 20) return; end
        end
        for (int i = 0; i < DW; i++) begin
          logic [31:0] e = (i < dw) ? d[i] : (i < dw + bw) ? 32'h0 : POISON;
          if (dmem[i] !== e) begin $display("TB_FAIL case %0d: DMEM[%0d]=%h expected %h", n, i, dmem[i], e); errors++; if (errors > 20) return; end
        end
        if (boot_addr !== boot[31:0]) begin $display("TB_FAIL case %0d: boot_addr %h expected %h", n, boot_addr, boot); errors++; end
        if (busy) begin $display("TB_FAIL case %0d: busy after ok", n); errors++; end
      end
    end else begin
      if (ok || !err_seen || err_code != exp_err[7:0]) begin
        $display("TB_FAIL case %0d: expected err %0d, got ok=%0d err_seen=%0d code=%0d", n, exp_err, ok, err_seen, err_code);
        errors++;
      end
      if (busy) begin $display("TB_FAIL case %0d: busy after error", n); errors++; end
    end
  endtask

  int c_base [$], c_size [$], c_err [$], c_fq [$], c_tw [$], c_dw [$], c_bw [$], c_boot [$];

  initial begin
    string dir;
    int fd, n, base, size, exp_err, fq, tw, dw, bw, boot, ncases;
    bit to;
    // xsim.bat mangles "=" in plusargs: everything comes from ./cases_cur (written by run_tb_cp_boot.sh)
    dir = "cases_cur";
    fd = $fopen("cases_cur/params.txt", "r");
    if (fd == 0 || $fscanf(fd, "%d %d %d", SEED, LAT_MAX, RDY_PCT) != 3) begin
      $display("TB_FAIL: cases_cur/params.txt missing"); $finish;
    end
    $fclose(fd);
    rng = SEED * 32'h9e3779b1 + 1;
    fw_base = '0; fw_size = '0; m_rsp_valid = 0; m_rsp_fault = 0; m_rsp_data = '0; busy_m = 0;
    err_seen = 0; err_code = 0; reqs = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (3) @(posedge clk);
    fd = $fopen({dir, "/cases.txt"}, "r");
    ncases = 0;
    while ($fscanf(fd, "%d %d %d %d %d %d %d %d %d\n", n, base, size, exp_err, fq, tw, dw, bw, boot) == 9) begin
      c_base.push_back(base); c_size.push_back(size); c_err.push_back(exp_err); c_fq.push_back(fq);
      c_tw.push_back(tw); c_dw.push_back(dw); c_bw.push_back(bw); c_boot.push_back(boot);
      run_case(n, base, size, exp_err, fq, tw, dw, bw, boot, dir);
      ncases++;
      if (errors > 20) break;
    end
    $fclose(fd);
    $display("t_cases done: %0d cases, %0d memory requests", ncases, reqs);

    // CORE_RESET in the middle of a load, then a complete load of another image
    stop_and_clear(); fill_mems();
    for (int i = 0; i < MQ; i++) mem[i] = 64'hA5A5_A5A5_A5A5_A5A5;
    $readmemh({dir, "/mem_0.hex"}, mem);
    fault_q = -1; fw_base = c_base[0]; fw_size = c_size[0];
    @(posedge clk); run = 1'b1;
    repeat (150) @(posedge clk);
    if (ok || err_seen) begin $display("TB_FAIL abort: finished too early"); errors++; end
    run = 1'b0;
    repeat (20) @(posedge clk);
    if (busy || ok) begin $display("TB_FAIL abort: still busy / ok after CORE_RESET"); errors++; end
    run_case(1, c_base[1], c_size[1], c_err[1], c_fq[1], c_tw[1], c_dw[1], c_bw[1], c_boot[1], dir);
    $display("t_abort done");

    // a memory that never answers: BOOT_ERR 5 after the response timeout
    stop_and_clear(); fill_mems();
    for (int i = 0; i < MQ; i++) mem[i] = 64'hA5A5_A5A5_A5A5_A5A5;
    $readmemh({dir, "/mem_0.hex"}, mem);
    fault_q = -2; fw_base = c_base[0]; fw_size = c_size[0];
    @(posedge clk); run = 1'b1;
    wait_end(1300000, to);
    if (to || !err_seen || err_code != 8'd5 || ok) begin
      $display("TB_FAIL no-response: to=%0d err_seen=%0d code=%0d", to, err_seen, err_code); errors++;
    end
    // ... and the stale-response guard: the next boot waits until the late response (if any) is gone
    fault_q = -1; stop_and_clear();
    run_case(0, c_base[0], c_size[0], c_err[0], c_fq[0], c_tw[0], c_dw[0], c_bw[0], c_boot[0], dir);
    $display("t_noresp done");

    if (errors == 0) $display("NG_CP_BOOT_TB_PASS");
    else $display("NG_CP_BOOT_TB_FAIL (%0d errors)", errors);
    $finish;
  end
endmodule
