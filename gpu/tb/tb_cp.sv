// tb_cp.sv -- testbench for the CP subsystem (rtl/gpu/cp).
//
// DUT: ng_cp_top with the real CP firmware (fw/cp/src, built -DNGFW_BUS_HOOK)
// running as a DPI coroutine behind ng_cp_cpu_stub.  Part 1 replays
// "cp_vec.hex" (rtl/gpu/tb/gen_golden/gen_cp_vectors.cpp: the functional
// model's reference results), part 2 runs RTL-only directed tests the model
// cannot express (work-distributor port, CORE_RESET, RO-register writes, IRQ
// line, memory-port back-pressure, CP_CTRL.RESET / CORE_RESET during a long COPY).
//
// Vector stream: 6 words per event {op, a, b, c, d, e}
//   1 HW     a=BAR0 offset b=value                     host register write
//   2 HR     a=offset b=expected c=mask                host read, compare under mask
//   3 MEMW   a=pa_lo b=pa_hi c=value                   poke one dword of memory
//   4 CHK    a=pa_lo b=pa_hi c=nwords d=fnv1a32        digest compare (aligned PA)
//   5 SETTLE                                           wait until the CP is quiet
//   6 RESET                                            reset DUT + clear memory
//   7 NOTE   a=id                                      scenario banner
//   8 HRNZ   a=offset b=expect_nonzero                 host read, (v != 0) compare
//   0 END
// Memory model: sparse dword memory; VMID 0 = physical (model's PA regions),
// VMID 1 = VA kVa..+16 MiB -> PA 0x1000_0000.., every other VMID faults.
// Expect NG_CP_TB_PASS.
`timescale 1ns/1ps
// run_tb_cp.sh writes tb_cp_cfg.svh (`define TB_SEED / TB_LAT_MAX / TB_RDY_PCT) per seed;
// the defaults below apply when the file is absent from the include path.
`ifdef TB_CFG
`include "tb_cp_cfg.svh"
`endif
`ifndef TB_SEED
`define TB_SEED 1
`endif
`ifndef TB_LAT_MAX
`define TB_LAT_MAX 3
`endif
`ifndef TB_RDY_PCT
`define TB_RDY_PCT 75
`endif
`ifndef IBEX_QUIET_WORD
`define IBEX_QUIET_WORD 0
`endif
`ifndef IBEX_IMAGE
`define IBEX_IMAGE "fw_imem.hex"
`endif
// TB_BOOT_LOADER (with NG_CP_IBEX): the DUT loads the firmware with the hardware boot loader from an NGCF image the
// TB writes to C3 (fw_ngcf.hex: one 32-bit word per line, from tools/ngcf/ngcf_pack.py) instead of the IMEM preload.
`ifdef TB_BOOT_LOADER
`define TB_BOOT_PAR 1'b1
`else
`define TB_BOOT_PAR 1'b0
`endif
module tb_cp #(
  parameter int SEED = `TB_SEED,         // memory latency / back-pressure randomisation
  parameter int LAT_MAX = `TB_LAT_MAX,   // memory response latency 1..LAT_MAX cycles
  parameter int RDY_PCT = `TB_RDY_PCT    // request-ready probability
);
  import ng_cp_pkg::*;

  localparam int MAXW = 4_000_000;
  localparam longint unsigned KVA = 64'h1_0000_0000;
  localparam longint unsigned KPA = 64'h1000_0000;
  localparam longint unsigned VA_WIN = 64'h100_0000;           // 16 MiB
  localparam longint unsigned VRAM_SIZE = 64'h4_0000_0000;     // 16 GiB
  localparam longint unsigned SYS_BASE = 64'h10_0000_0000;
  localparam longint unsigned SYS_SIZE = 64'h8000_0000;        // 2 GiB
  localparam longint unsigned HOST_BASE = 64'h8000_0000_0000;

  logic clk = 1'b0, rst_n = 1'b0;
  always #5 clk = ~clk;

  // host bus
  logic        host_wr = 1'b0, host_rd = 1'b0;
  logic [12:0] host_addr = '0;
  logic [31:0] host_wdata = '0;
  wire  [31:0] host_rdata;
  logic [31:0] irq_ext = '0;
  wire         rm_irq;

  // memory port
  wire         m_req_valid;
  logic        m_req_ready;
  mem_req_t    m_req;
  logic        m_rsp_valid = 1'b0, m_rsp_fault = 1'b0;
  logic [63:0] m_rsp_data = '0;

  // work distributor
  wire         wd_start;
  wd_req_t     wd_req;
  logic        wd_done = 1'b0, wd_fault = 1'b0;
  logic [63:0] wd_instr = '0;
  logic        sp_idle = 1'b1;

  wire         dbg_valid, cpu_quiet, core_reset;
  logic [15:0] scrub_set = '0;      // MMU CTX_CTRL write pulses (driven by t_scrub)
  wire  [2:0]  sp_inv;
  wire  [31:0] dbg_code;

  ng_cp_top #(.IBEX_QUIET_WORD(`IBEX_QUIET_WORD), .IBEX_IMAGE(`IBEX_IMAGE), .BOOT_LOADER(`TB_BOOT_PAR), .SCRUB(1'b1)) dut (
    .clk, .rst_n,
    .host_wr, .host_rd, .host_addr, .host_wdata, .host_rdata,
    .irq_ext_i(irq_ext), .rm_irq_o(rm_irq),
    .m_req_valid, .m_req_ready, .m_req, .m_rsp_valid, .m_rsp_fault, .m_rsp_data,
    .wd_start, .wd_req, .wd_done, .wd_fault, .wd_instr, .sp_idle_i(sp_idle), .wd_abort_o(), .fab_busy_i(1'b0),
    .scrub_set_i(scrub_set), .sp_inv_o(sp_inv),
    .cb_valid(), .cb_fill(), .cb_vmid(), .cb_src(), .cb_dst(), .cb_size(), .cb_fillv(), .cb_abort(),
    .cb_done(1'b0), .cb_fault(1'b0),
    .dbg_valid_o(dbg_valid), .dbg_code_o(dbg_code), .cpu_quiet_o(cpu_quiet),
    .core_reset_o(core_reset)
  );

  // xorshift32 (xsim has no $urandom(seed))
  logic [31:0] rng_state = 32'h2545f491;
  function automatic int rnd(input int n);
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return int'(rng_state % n);
  endfunction

  int errors = 0;
  int checks = 0;
  int verbose = 0;
  int lat_max = LAT_MAX;
  int rdy_pct = RDY_PCT;

  task automatic fail(input string msg);
    errors++;
    if (errors <= 40) $display("TB_FAIL @%0t %s", $time, msg);
  endtask

  // ---------------------------------------------------------------------------
  // memory model
  logic [31:0] mem [longint unsigned];

  function automatic logic [31:0] mrd(input longint unsigned pa);
    longint unsigned k = pa >> 2;
    return mem.exists(k) ? mem[k] : 32'h0;
  endfunction
  task automatic mwr(input longint unsigned pa, input logic [31:0] v);
    mem[pa >> 2] = v;
  endtask

  function automatic bit pa_valid(input longint unsigned pa);
    return (pa < VRAM_SIZE) || (pa >= SYS_BASE && pa - SYS_BASE < SYS_SIZE) || (pa >= HOST_BASE);
  endfunction

  bit map_vmid2 = 1'b0;       // t_scrub: VMID 2 maps the same window as VMID 1

  // translate an 8-byte lane address; returns 1 on success
  function automatic bit xlate(input logic [3:0] vmid, input logic [47:0] a, output longint unsigned pa);
    pa = 0;
    if (vmid == 4'd0) begin
      pa = a;
      return pa_valid(pa);
    end else if ((vmid == 4'd1 || (map_vmid2 && vmid == 4'd2)) && a >= KVA[47:0] && (a - KVA[47:0]) < VA_WIN) begin
      pa = KPA + (a - KVA[47:0]);
      return pa_valid(pa);
    end
    return 1'b0;
  endfunction

  // one outstanding request, random latency / ready
  bit          pend = 1'b0;
  int          cnt = 0;
  mem_req_t    preq;
  int          accesses = 0, mem_faults = 0, max_wait = 0;

  always @(posedge clk) m_req_ready <= !pend && (rnd(100) < rdy_pct);

  always @(posedge clk) begin
    m_rsp_valid <= 1'b0;
    m_rsp_fault <= 1'b0;
    if (pend) begin
      if (cnt > 0) cnt <= cnt - 1;
      else begin
        longint unsigned pa;
        bit ok;
        ok = xlate(preq.vmid, preq.addr, pa);
        m_rsp_valid <= 1'b1;
        pend <= 1'b0;
        accesses++;
        if (!ok) begin
          m_rsp_fault <= 1'b1;
          m_rsp_data <= '0;
          mem_faults++;
        end else if (preq.write) begin
          for (int i = 0; i < 8; i++) begin
            if (preq.wstrb[i]) begin
              logic [31:0] w = mrd(pa + (i & 4));
              w[8*(i & 3) +: 8] = preq.wdata[8*i +: 8];
              mwr(pa + (i & 4), w);
            end
          end
        end else begin
          m_rsp_data <= {mrd(pa + 4), mrd(pa)};
        end
      end
    end else if (m_req_valid && m_req_ready) begin
      preq <= m_req;
      pend <= 1'b1;
      cnt <= rnd(lat_max);
    end
  end

  // ---------------------------------------------------------------------------
  // work distributor stub: records the request, completes after a delay
  wd_req_t wd_log [$];
  int      wd_delay = 6;
  bit      wd_inject_fault = 1'b0;
  int      wd_cnt = -1;
  always @(posedge clk) begin
    wd_done <= 1'b0;
    if (wd_start) begin
      wd_log.push_back(wd_req);
      sp_idle <= 1'b0;
      wd_cnt <= wd_delay;
      wd_instr <= 64'd1000 * wd_req.grid[0] * wd_req.grid[1] * wd_req.grid[2] + 7;
    end else if (wd_cnt > 0) begin
      wd_cnt <= wd_cnt - 1;
    end else if (wd_cnt == 0) begin
      wd_fault <= wd_inject_fault;
      wd_done <= 1'b1;
      sp_idle <= 1'b1;
      wd_cnt <= -1;
    end
  end

  // ---------------------------------------------------------------------------
  // host bus tasks (drive at posedge, sample at negedge)
  task automatic hw_raw(input logic [12:0] off, input logic [31:0] v);
    @(posedge clk);
    host_wr <= 1'b1; host_addr <= off; host_wdata <= v;
    @(posedge clk);
    host_wr <= 1'b0;
  endtask

`ifdef TB_BOOT_LOADER
  // The recorded vector stream writes arbitrary values to CP_FW_BASE/SIZE (register read-back scenarios) and later
  // releases CORE_RESET.  The real kernel always points them at the image first: when a write releases CORE_RESET,
  // put the real image location in place, let the loader run, then restore what the vector had written.
  localparam longint unsigned FW_PA_R = SYS_BASE + 64'h10_0000;
  int fw_bytes_r = 0;
  task automatic hw(input logic [12:0] off, input logic [31:0] v);
    if (off == R_CTRL && !v[3] && core_reset && fw_bytes_r != 0) begin
      logic [31:0] sv_lo = dut.u_regs.fw_base_lo, sv_hi = dut.u_regs.fw_base_hi, sv_sz = dut.u_regs.fw_size;
      int t = 0;
      hw_raw(R_FW_BASE_LO, FW_PA_R[31:0]); hw_raw(R_FW_BASE_HI, FW_PA_R[63:32]); hw_raw(R_FW_SIZE, fw_bytes_r);
      hw_raw(off, v);
      while (!dut.boot_ok && dut.u_regs.f_fw_status[15:8] == 8'h0 && t < 200_000) begin @(posedge clk); t = t + 1; end
      if (sv_lo != FW_PA_R[31:0]) hw_raw(R_FW_BASE_LO, sv_lo);
      if (sv_hi != FW_PA_R[63:32]) hw_raw(R_FW_BASE_HI, sv_hi);
      if (sv_sz != fw_bytes_r) hw_raw(R_FW_SIZE, sv_sz);
    end else hw_raw(off, v);
  endtask
`else
  task automatic hw(input logic [12:0] off, input logic [31:0] v);
    hw_raw(off, v);
  endtask
`endif

  task automatic hr(input logic [12:0] off, output logic [31:0] v);
    @(posedge clk);
    host_addr <= off; host_rd <= 1'b1;
    @(negedge clk);
    v = host_rdata;
    @(posedge clk);
    host_rd <= 1'b0;
  endtask

  task automatic settle();
    int t = 0, quiet_cnt = 0;
    repeat (16) @(posedge clk);
    while (quiet_cnt < 8) begin
      @(posedge clk);
`ifdef TB_BOOT_LOADER
      if (core_reset || (cpu_quiet && dut.ev_pend == 4'h0 && !dut.boot_busy && dut.boot_ok)) quiet_cnt++;
`else
      if (core_reset || (cpu_quiet && dut.ev_pend == 4'h0)) quiet_cnt++;
`endif
      else quiet_cnt = 0;
      if (++t > 600_000) begin
        fail("settle timeout");
        return;
      end
    end
  endtask

  task automatic do_reset();
    rst_n = 1'b0;
    mem.delete();
    wd_log.delete();
    repeat (4) @(posedge clk);
    rst_n = 1'b1;
`ifdef TB_BOOT_LOADER
    load_ngcf();
`endif
    repeat (2) @(posedge clk);
  endtask

  function automatic logic [31:0] fnv(input longint unsigned pa, input int n);
    logic [31:0] h = 32'd2166136261;
    for (int i = 0; i < n; i++) h = (h ^ mrd(pa + 4 * i)) * 32'd16777619;
    return h;
  endfunction

  // ---------------------------------------------------------------------------
  // part 1: replay the model-generated vector stream
  logic [31:0] ev [0:MAXW-1];

  task automatic replay();
    int i = 0, nev = 0;
    bit done = 0;
    int hr_n = 0, chk_n = 0;
    logic [31:0] v;
    while (!done) begin
      logic [31:0] op, a, b, c, d;
      if (i + 6 > MAXW) begin fail("vector stream overrun"); break; end
      op = ev[i]; a = ev[i+1]; b = ev[i+2]; c = ev[i+3]; d = ev[i+4];
      i += 6; nev++;
      if (^op === 1'bx) begin fail("vector stream: X opcode (stream too long for MAXW?)"); break; end
      case (op)
        0: done = 1;
        1: hw(a[12:0], b);
        2: begin
          hr(a[12:0], v);
          hr_n++;
          if (((v ^ b) & c) != 0)
            fail($sformatf("HR off=%03h got=%08h exp=%08h mask=%08h (event %0d)", a, v, b, c, nev));
        end
        3: mwr({b, a}, c);
        4: begin
          logic [31:0] h = fnv({b, a}, c);
          chk_n++;
          if (h !== d) fail($sformatf("CHK pa=%0h n=%0d hash=%08h exp=%08h (event %0d)", {b, a}, c, h, d, nev));
        end
        5: settle();
        6: do_reset();
        7: $display("  [vec] scenario/seed id %0d (event %0d, errors so far %0d)", a, nev, errors);
        8: begin
          hr(a[12:0], v);
          hr_n++;
          if ((v != 0) !== (b != 0)) fail($sformatf("HRNZ off=%03h got=%08h exp_nz=%0d (event %0d)", a, v, b, nev));
        end
        default: begin fail($sformatf("bad op %0d", op)); done = 1; end
      endcase
    end
    checks += hr_n + chk_n;
    $display("  [vec] %0d events, %0d register compares, %0d digest compares, %0d memory accesses (%0d faults)",
             nev, hr_n, chk_n, accesses, mem_faults);
  endtask

  // ---------------------------------------------------------------------------
  // part 2: RTL-only directed tests
  localparam longint unsigned RING_PA = SYS_BASE + 64'h1_0000;
  localparam longint unsigned WB_PA = SYS_BASE + 64'h2_0000;
  localparam longint unsigned DATA_PA = KPA;

  int ring_wp = 0;
  task automatic ring_push(input logic [31:0] w);
    mwr(RING_PA + 4 * (ring_wp & 1023), w);
    ring_wp++;
  endtask
  task automatic ring_go();
    hw(R_DOORBELL0, ring_wp);
    settle();
  endtask

  task automatic expect_reg(input string name, input logic [12:0] off, input logic [31:0] exp, input logic [31:0] mask = 32'hffffffff);
    logic [31:0] v;
    hr(off, v);
    checks++;
    if (((v ^ exp) & mask) != 0) fail($sformatf("%s: reg %03h = %08h, expected %08h (mask %08h)", name, off, v, exp, mask));
  endtask
  task automatic expect_mem(input string name, input longint unsigned pa, input logic [31:0] exp);
    checks++;
    if (mrd(pa) !== exp) fail($sformatf("%s: mem[%0h] = %08h, expected %08h", name, pa, mrd(pa), exp));
  endtask

`ifdef TB_BOOT_LOADER
  localparam longint unsigned FW_PA = SYS_BASE + 64'h10_0000;
  int fw_words = 0;
  task automatic load_ngcf();
    logic [31:0] w [4096];
    int n = 0, fd;
    for (int i = 0; i < 4096; i++) w[i] = 32'h0;
    $readmemh("fw_ngcf.hex", w);
    fd = $fopen("fw_ngcf.size", "r");        // image bytes (2-state simulators cannot tell an x word from a zero word)
    if (fd == 0 || $fscanf(fd, "%d", n) != 1) begin fail("fw_ngcf.size missing"); n = 0; end
    else $fclose(fd);
    n = n / 4;
    for (int i = 0; i < n; i++) mwr(FW_PA + 4 * i, w[i]);
    fw_words = n; fw_bytes_r = 4 * n;
  endtask
  task automatic point_at_ngcf();
    hw_raw(R_FW_BASE_LO, FW_PA[31:0]); hw_raw(R_FW_BASE_HI, FW_PA[63:32]); hw_raw(R_FW_SIZE, 4 * fw_words);
  endtask
`endif

  task automatic cp_boot();
    ring_wp = 0;
`ifdef TB_BOOT_LOADER
    point_at_ngcf();
`endif
    hw(R_RING_BASE_LO, RING_PA[31:0]);
    hw(R_RING_BASE_HI, RING_PA[63:32]);
    hw(R_RING_SIZE, 32'd10);
    hw(R_CTRL, 32'h0);                       // release CORE_RESET
    hw(R_CTRL, 32'h1);                       // ENABLE
    settle();
  endtask

  // dispatch protocol: SET_SHADER + SET_USER_REGS + DISPATCH through an IB
  task automatic t_wdist();
    longint unsigned ib_pa = KPA + 64'h1000;
    int n = 0;
    logic [31:0] ib [$];
    wd_req_t r;
    // IB: SET_SHADER (11), SET_USER_REGS (start 0: 0x11, 0x22), DISPATCH 3x2x1
    ib.push_back(32'h0006 | (11 << 16));
    ib.push_back(32'h0001_0000);        // CODE_BASE_LO = 0x1_0001_0000 low
    ib.push_back(32'h0000_0001);        // CODE_BASE_HI
    ib.push_back(32'h0);                // ENTRY
    ib.push_back(32'd16);               // VREGS
    ib.push_back(32'h100);              // SHARED
    ib.push_back(32'h0); ib.push_back(32'h0); ib.push_back(32'h0);   // scratch
    ib.push_back(32'd8); ib.push_back(32'd1); ib.push_back(32'd1);   // WG
    ib.push_back(32'h0007 | (3 << 16)); ib.push_back(32'd0); ib.push_back(32'h11); ib.push_back(32'h22);
    ib.push_back(32'h0005 | (3 << 16)); ib.push_back(32'd3); ib.push_back(32'd2); ib.push_back(32'd1);
    foreach (ib[k]) mwr(ib_pa + 4 * k, ib[k]);
    n = ib.size();
    // VMID 1 VA of the IB = KVA + 0x1000
    cp_boot();
    ring_push(32'h0001 | (4 << 16)); ring_push(32'h0000_1000); ring_push(32'h1); ring_push(n); ring_push(32'd1);
    ring_go();
    checks++;
    if (wd_log.size() != 1) begin
      fail($sformatf("wdist: %0d requests, expected 1", wd_log.size()));
    end else begin
      r = wd_log[0];
      if (r.vmid !== 4'd1) fail($sformatf("wdist vmid %0d", r.vmid));
      if (r.num_vregs !== 32'd16) fail($sformatf("wdist vregs %0d", r.num_vregs));
      if (r.shared_bytes !== 32'h100) fail($sformatf("wdist shared %0h", r.shared_bytes));
      if (r.code_va !== 64'h1_0001_0000) fail($sformatf("wdist code_va %0h", r.code_va));
      if (r.code_base !== 64'h1_0001_0000) fail($sformatf("wdist code_base %0h", r.code_base));
      if (r.grid[0] !== 3 || r.grid[1] !== 2 || r.grid[2] !== 1) fail("wdist grid");
      if (r.wg[0] !== 8 || r.wg[1] !== 1 || r.wg[2] !== 1) fail("wdist wg");
      if (r.uregs[0] !== 32'h11 || r.uregs[1] !== 32'h22 || r.uregs[2] !== 32'h0) fail("wdist uregs");
    end
    expect_reg("wdist rptr", R_RING_RPTR, 5);
    expect_reg("wdist err", R_ERROR, 0);
    // functional timestamp includes the retired instructions (6*1000+7)
    begin
      logic [31:0] ts;
      hr(R_TS_LO, ts);
      checks++;
      if (ts < 32'd6007 || ts > 32'd6020) fail($sformatf("wdist ticks %0d (expected 6007 + a few packets)", ts));
    end
    // shader fault path: CP_ERROR = SHADER_FAULT (4)
    wd_inject_fault = 1'b1;
    ring_push(32'h0001 | (4 << 16)); ring_push(32'h0000_1000); ring_push(32'h1); ring_push(n); ring_push(32'd1);
    ring_go();
    wd_inject_fault = 1'b0;
    expect_reg("wdist fault err", R_ERROR, 4);
    expect_reg("wdist fault status", R_STATUS, 32'h13, 32'h1f);   // IDLE|HALTED|FW_READY
    expect_reg("wdist fault irq", R_IRQ_STATUS, 32'h2 | 32'h8, 32'h0a);  // CP_ERROR | CP_FW
  endtask

  // scrub generation of ng_cp_port.  A dispatch carries wd_req.scrub = 1 exactly when its owner differs from the
  // previous dispatch's: first dispatch after reset / CORE_RESET, a different VMID, or an MMU CTX_CTRL write of the
  // VMID (scrub_set pulse) since; never for the same owner back to back.  CACHE packets reach sp_inv_o.
  int n_inv_l1 = 0, n_inv_k = 0, n_inv_i = 0;
  always @(posedge clk) begin
    if (sp_inv[0]) n_inv_l1++;
    if (sp_inv[1]) n_inv_k++;
    if (sp_inv[2]) n_inv_i++;
  end
  task automatic scrub_ib(input int vmid);        // one DISPATCH through an IB of the given VMID (same shape as t_wdist)
    longint unsigned ib_pa = KPA + 64'h1000;
    logic [31:0] ib [$];
    int n;
    ib.push_back(32'h0006 | (11 << 16));
    ib.push_back(32'h0001_0000); ib.push_back(32'h0000_0001); ib.push_back(32'h0); ib.push_back(32'd16);
    ib.push_back(32'h100); ib.push_back(32'h0); ib.push_back(32'h0); ib.push_back(32'h0);
    ib.push_back(32'd8); ib.push_back(32'd1); ib.push_back(32'd1);
    ib.push_back(32'h0005 | (3 << 16)); ib.push_back(32'd3); ib.push_back(32'd2); ib.push_back(32'd1);
    foreach (ib[k]) mwr(ib_pa + 4 * k, ib[k]);
    n = ib.size();
    ring_push(32'h0001 | (4 << 16)); ring_push(32'h0000_1000); ring_push(32'h1); ring_push(n); ring_push(32'(vmid));
    ring_go();
  endtask
  task automatic scrub_expect(input string what, input int vmid, input bit exp);
    int n0 = wd_log.size();
    scrub_ib(vmid);
    checks++;
    if (wd_log.size() != n0 + 1) fail($sformatf("scrub %s: %0d new requests, expected 1", what, wd_log.size() - n0));
    else if (wd_log[n0].scrub !== exp) fail($sformatf("scrub %s: wd_req.scrub %0b, expected %0b", what, wd_log[n0].scrub, exp));
    else if (wd_log[n0].vmid !== 4'(vmid)) fail($sformatf("scrub %s: vmid %0d", what, wd_log[n0].vmid));
  endtask
  task automatic scrub_pulse(input int vmid);     // MMU CTX_CTRL write of VMID `vmid` (ng_gpu_rm_sp: toggle -> clk_ts pulse)
    @(posedge clk); scrub_set[vmid] <= 1'b1;
    @(posedge clk); scrub_set[vmid] <= 1'b0;
    repeat (2) @(posedge clk);
  endtask
  task automatic t_scrub();
    int i0, k0, i1;
    map_vmid2 = 1'b1;
    cp_boot();
    scrub_expect("first dispatch after reset", 1, 1'b1);
    scrub_expect("same owner again", 1, 1'b0);
    scrub_expect("same owner, third", 1, 1'b0);
    scrub_pulse(1);
    scrub_expect("ctx write of the same VMID", 1, 1'b1);
    scrub_expect("ctx-write scrub consumed", 1, 1'b0);
    scrub_pulse(2);                              // another VMID's context changed: not this owner
    scrub_expect("ctx write of another VMID", 1, 1'b0);
    scrub_expect("VMID changed 1 -> 2", 2, 1'b1);   // (the pending flag of VMID 2 is consumed with it)
    scrub_expect("VMID 2 again", 2, 1'b0);
    scrub_expect("VMID changed 2 -> 1", 1, 1'b1);
    scrub_pulse(1); scrub_pulse(1);              // two writes before the next dispatch: one scrub
    scrub_expect("two ctx writes", 1, 1'b1);
    scrub_expect("after two ctx writes", 1, 1'b0);
    // CACHE packets reach the SP's invalidate port (ring-level packets, 4 words each: header + FLAGS)
    i0 = n_inv_l1; k0 = n_inv_k; i1 = n_inv_i;
    ring_push(32'h000a | (1 << 16)); ring_push(32'h02);                     // CACHE INV_L1
    ring_push(32'h000a | (1 << 16)); ring_push(32'h04);                     // CACHE INV_K
    ring_push(32'h000a | (1 << 16)); ring_push(32'h08);                     // CACHE INV_I
    ring_push(32'h000a | (1 << 16)); ring_push(32'h01);                     // CACHE WAIT_IDLE only: no invalidate
    ring_push(32'h000a | (1 << 16)); ring_push(32'h02 | 32'h04 | 32'h08);   // all three
    ring_go();
    checks++;
    if (n_inv_l1 - i0 != 2 || n_inv_k - k0 != 2 || n_inv_i - i1 != 2)
      fail($sformatf("CACHE packets: INV_L1 %0d INV_K %0d INV_I %0d pulses, expected 2 / 2 / 2", n_inv_l1 - i0, n_inv_k - k0, n_inv_i - i1));
    expect_reg("CACHE packets: error", R_ERROR, 0);
    // an INDIRECT's implicit invalidate (clause C-INV: L1 + K + I) reaches the SP as well
    i0 = n_inv_l1; k0 = n_inv_k; i1 = n_inv_i;
    scrub_expect("after the CACHE packets", 1, 1'b0);
    checks++;
    if (n_inv_l1 - i0 != 1 || n_inv_k - k0 != 1 || n_inv_i - i1 != 1)
      fail($sformatf("INDIRECT: implicit INV_L1 %0d INV_K %0d INV_I %0d pulses, expected 1 / 1 / 1", n_inv_l1 - i0, n_inv_k - k0, n_inv_i - i1));
    // CORE_RESET (firmware reload): the SP's RAMs keep their contents, so the next dispatch scrubs
    hw(R_CTRL, 32'h8);
    repeat (4) @(posedge clk);
    hw(R_CTRL, 32'h1);
    settle();
    ring_wp = 0;
    scrub_expect("first dispatch after CORE_RESET", 1, 1'b1);
    scrub_expect("then the same owner", 1, 1'b0);
    map_vmid2 = 1'b0;
  endtask

  // CORE_RESET: hardware clears firmware-owned registers, firmware reboots
  task automatic t_core_reset();
    cp_boot();
    // a FENCE with IRQ, 64-bit
    ring_push(32'h0002 | (5 << 16)); ring_push(DATA_PA[31:0] + 32'h100); ring_push(DATA_PA[63:32]);
    ring_push(32'hcafe0001); ring_push(32'h0); ring_push(32'h3);   // 64-bit | IRQ
    hw(R_IRQ_ENABLE, 32'h1f);
    ring_go();
    expect_mem("fence lo", DATA_PA + 64'h100, 32'hcafe0001);
    expect_reg("fence irq", R_IRQ_STATUS, 32'h1, 32'h1);
    checks++;
    if (!rm_irq) fail("rm_irq not asserted after fence IRQ with IRQ_ENABLE");
    hw(R_IRQ_STATUS, 32'h1);
    repeat (2) @(posedge clk);
    checks++;
    if (rm_irq && (dut.u_regs.irq_status & 32'h1f & dut.u_regs.irq_enable) != 0) begin
      // CP_FW (bit 3) is still pending: rm_irq may remain high until it is cleared too
    end
    hw(R_IRQ_STATUS, 32'hffffffff);
    repeat (2) @(posedge clk);
    checks++;
    if (rm_irq) fail("rm_irq still asserted after W1C of all bits");
    expect_reg("pre-reset rptr", R_RING_RPTR, 6);
    // assert CORE_RESET
    hw(R_CTRL, 32'h8);
    repeat (4) @(posedge clk);
    expect_reg("core-reset rptr", R_RING_RPTR, 0);
    expect_reg("core-reset wptr", R_RING_WPTR, 0);
    expect_reg("core-reset status", R_STATUS, 32'h1);
    expect_reg("core-reset fw_status", R_FW_STATUS, 0);
    expect_reg("core-reset cmd_version", R_CMD_VERSION, 0);
    expect_reg("core-reset fw_version", R_FW_VERSION, 0);
    expect_reg("core-reset ctrl", R_CTRL, 32'h8);
    expect_reg("core-reset ring base kept", R_RING_BASE_LO, RING_PA[31:0]);
    // doorbell while in reset is latched; no work is done
    ring_wp = 0;
    ring_push(32'h0002 | (5 << 16)); ring_push(DATA_PA[31:0] + 32'h200); ring_push(DATA_PA[63:32]);
    ring_push(32'hbeef0002); ring_push(32'h0); ring_push(32'h0);
    hw(R_CTRL, 32'h8);
    hw(R_DOORBELL0, ring_wp);
    repeat (50) @(posedge clk);
    expect_mem("no work in reset", DATA_PA + 64'h200, 32'h0);
    // release: handshake again, ring processed from 0
    hw(R_CTRL, 32'h1);
    settle();
    expect_reg("reboot status", R_STATUS, 32'h11);
    expect_reg("reboot fw_status", R_FW_STATUS, 1);
    expect_reg("reboot cmd_version", R_CMD_VERSION, 32'h0000_0003);   // cmd v0.3 (P2-00b; firmware header ngfw_cmd.h)
    expect_reg("reboot rptr", R_RING_RPTR, 6);
    expect_mem("work after reboot", DATA_PA + 64'h200, 32'hbeef0002);
    expect_reg("reboot irq fw", R_IRQ_STATUS, 32'h8, 32'h8);
  endtask

  // RO / undefined writes ignored, RW readback, RESET self-clears
  task automatic t_regs();
    logic [31:0] v;
    cp_boot();
    hw(R_RING_RPTR, 32'h1234); hw(R_STATUS, 32'hffff); hw(R_ERROR, 32'h5); hw(R_HEARTBEAT, 32'h0);
    hw(R_CMD_VERSION, 32'h77); hw(R_FW_VERSION, 32'h77); hw(R_FW_STATUS, 32'h77); hw(R_RING_WPTR, 32'h99);
    hw(13'h01f0, 32'h1234); hw(13'h0160, 32'h1);
    settle();
    expect_reg("ro rptr", R_RING_RPTR, 0);
    expect_reg("ro status", R_STATUS, 32'h11);
    expect_reg("ro error", R_ERROR, 0);
    expect_reg("ro wptr", R_RING_WPTR, 0);
    expect_reg("ro fw_status", R_FW_STATUS, 1);
    expect_reg("undef 1f0", 13'h01f0, 0);
    hr(R_HEARTBEAT, v);
    checks++;
    if (v == 0) fail("heartbeat did not advance");
    hw(R_FW_BASE_LO, 32'h1000); hw(R_FW_BASE_HI, 32'h5); hw(R_FW_SIZE, 32'h2000);
    hw(R_RPTR_WB_LO, 32'h0000_3007); hw(R_RPTR_WB_HI, 32'h6);
    hw(R_RING_SIZE, 32'h0000_000a);
    settle();
    expect_reg("fw_base_lo", R_FW_BASE_LO, 32'h1000);
    expect_reg("fw_base_hi", R_FW_BASE_HI, 5);
    expect_reg("fw_size", R_FW_SIZE, 32'h2000);
    expect_reg("rptr_wb_lo (bit1 masked)", R_RPTR_WB_LO, 32'h3005);
    expect_reg("rptr_wb_hi", R_RPTR_WB_HI, 6);
    expect_reg("ring size full width", R_RING_SIZE, 32'h0000_000a);
    hw(R_RING_SIZE, 32'd10);
    // RESET bit reads back 0
    hw(R_CTRL, 32'h3);
    settle();
    expect_reg("ctrl reset self-clears", R_CTRL, 32'h1);
    // timestamp: LO read latches HI, FREQ constant
    expect_reg("ts freq", R_TS_FREQ, 32'd100_000_000);
  endtask

  // slow / heavily back-pressured memory + the copy engine against a big copy
  task automatic t_backpressure();
    int save_lat = lat_max, save_rdy = rdy_pct;
    cp_boot();
    for (int i = 0; i < 64; i++) mwr(DATA_PA + 64'h4000 + 4 * i, 32'h9000_0000 + i);
    lat_max = 9; rdy_pct = 20;
    // COPY physical, unaligned, 203 bytes
    ring_push(32'h0004 | (7 << 16));
    ring_push(DATA_PA[31:0] + 32'h4003); ring_push(DATA_PA[63:32]);
    ring_push(DATA_PA[31:0] + 32'h6005); ring_push(DATA_PA[63:32]);
    ring_push(32'd203); ring_push(32'd0); ring_push(32'd0);
    ring_go();
    lat_max = save_lat; rdy_pct = save_rdy;
    expect_reg("bp error", R_ERROR, 0);
    begin
      logic [7:0] s [203];
      bit okc = 1;
      for (int i = 0; i < 203; i++) begin
        logic [31:0] ws = mrd(DATA_PA + 64'h4003 + i - ((64'h4003 + i) & 3));
        logic [31:0] wd = mrd(DATA_PA + 64'h6005 + i - ((64'h6005 + i) & 3));
        logic [7:0] bs = ws[8*((64'h4003 + i) & 3) +: 8];
        logic [7:0] bd = wd[8*((64'h6005 + i) & 3) +: 8];
        if (bs !== bd) okc = 0;
      end
      checks++;
      if (!okc) fail("backpressure copy mismatch");
    end
  endtask

  // CP_CTRL.RESET / CORE_RESET while a long COPY runs (board: a 4 MiB copy outlived the job timeout, the CP never
  // answered the reset and the engine kept the memory port).  The engine must stop early and go idle, the CP must work
  // afterwards.
  task automatic copy_start_long();
    mwr(DATA_PA + 64'h1_fffc, 32'h5a5a_0001);          // last source word: must not reach the destination
    ring_push(32'h0004 | (7 << 16));
    ring_push(DATA_PA[31:0] + 32'h1_0000); ring_push(DATA_PA[63:32]);
    ring_push(DATA_PA[31:0] + 32'h3_0000); ring_push(DATA_PA[63:32]);
    ring_push(32'h1_0000); ring_push(32'd0); ring_push(32'd0);       // 64 KiB
    hw(R_DOORBELL0, ring_wp);
    for (int t = 0; !dut.u_port.copy_busy; t++) begin
      if (t > 20_000) begin fail("long COPY did not start"); return; end
      @(posedge clk);
    end
    repeat (500) @(posedge clk);
  endtask
  task automatic copy_wait_stopped(input string what);
    int t = 0;
    while (dut.u_port.copy_busy && t < 5_000) begin @(posedge clk); t++; end
    checks++;
    if (dut.u_port.copy_busy) fail($sformatf("%s: Copy engine still busy 5000 cycles later", what));
    expect_mem({what, ": copy stopped early"}, DATA_PA + 64'h3_fffc, 32'h0);
  endtask
  task automatic fence_check(input string what, input logic [31:0] v);
    ring_push(32'h0002 | (5 << 16)); ring_push(DATA_PA[31:0] + 32'h300); ring_push(DATA_PA[63:32]);
    ring_push(v); ring_push(32'h0); ring_push(32'h0);
    ring_go();
    expect_mem({what, ": fence after the reset"}, DATA_PA + 64'h300, v);
  endtask
  task automatic t_copy_abort();
    cp_boot();
    copy_start_long();
    hw(R_CTRL, 32'h3);                       // ENABLE | RESET
    copy_wait_stopped("CP_CTRL.RESET");
    settle();
    expect_reg("CP_CTRL.RESET: error", R_ERROR, 0);
    expect_reg("CP_CTRL.RESET: rptr = wptr", R_RING_RPTR, ring_wp);
    fence_check("CP_CTRL.RESET", 32'hab_0001);
    copy_start_long();
    hw(R_CTRL, 32'h8);                       // CORE_RESET
    copy_wait_stopped("CORE_RESET");
    hw(R_CTRL, 32'h1);
    settle();
    expect_reg("CORE_RESET: fw_status", R_FW_STATUS, 1);
    ring_wp = 0;                             // CORE_RESET cleared RPTR/WPTR
    fence_check("CORE_RESET", 32'hab_0002);
  endtask

  // S5: CORE_RESET while a firmware MEM access is (a) still waiting for the memory port to take it, (b) out and
  // waiting for its response.  (a) must be cancelled (nothing reaches memory after the reset), (b) must complete
  // before the boot loader / the restarted firmware may use the port; the CP works afterwards in both cases.
  task automatic mem_fence_start(input logic [31:0] v);
    ring_push(32'h0002 | (5 << 16)); ring_push(DATA_PA[31:0] + 32'h400); ring_push(DATA_PA[63:32]);
    ring_push(v); ring_push(32'h0); ring_push(32'h0);
    hw(R_DOORBELL0, ring_wp);                // no settle: the firmware is busy with the packet
  endtask
  task automatic t_mem_core_reset();
    int save_lat = lat_max, save_rdy = rdy_pct;
    int t;
    // (a) the memory port never says ready: the first MEM access of the packet is stuck before issue
    cp_boot();
    rdy_pct = 0;
    mem_fence_start(32'h5ca1_0001);
    for (t = 0; !(dut.u_port.mem_busy && !dut.u_port.mem_issued); t++) begin
      if (t > 20_000) begin fail("MEM access never became pending"); break; end
      @(posedge clk);
    end
    repeat (20) @(posedge clk);
    hw(R_CTRL, 32'h8);                       // CORE_RESET
    repeat (8) @(posedge clk);
    checks++;
    if (dut.u_port.mem_busy) fail("CORE_RESET: a MEM access that never went out is still pending");
    checks++;
    if (dut.u_port.mem_fault) fail("CORE_RESET: MEM fault flag not cleared");
    rdy_pct = save_rdy;
    repeat (300) @(posedge clk);
    expect_mem("CORE_RESET: cancelled MEM access wrote nothing", DATA_PA + 64'h400, 32'h0);
    hw(R_CTRL, 32'h1);
    settle();
    expect_reg("MEM cancel: fw_status", R_FW_STATUS, 1);
    ring_wp = 0;
    fence_check("MEM cancel", 32'hab_0003);
    // (b) the access is out and its response is slow: CORE_RESET must wait for the response before the loader runs
    do_reset();
    cp_boot();
    lat_max = 200; rdy_pct = 100;
    mem_fence_start(32'h5ca1_0002);
    for (t = 0; !(dut.u_port.mem_busy && dut.u_port.mem_issued); t++) begin
      if (t > 20_000) begin fail("MEM access never went out"); break; end
      @(posedge clk);
    end
    hw(R_CTRL, 32'h8);                       // CORE_RESET with the response outstanding
    repeat (4) @(posedge clk);
    checks++;
    if (!dut.u_port.mem_busy && pend) fail("CORE_RESET: MEM response outstanding but the port forgot it");
    lat_max = save_lat;
    hw(R_CTRL, 32'h1);
    settle();
    expect_reg("MEM drain: fw_status", R_FW_STATUS, 1);
    ring_wp = 0;
    fence_check("MEM drain", 32'hab_0004);
    lat_max = save_lat; rdy_pct = save_rdy;
  endtask

  // ---------------------------------------------------------------------------
  initial begin
    rng_state = 32'h9e3779b9 ^ (SEED * 32'h85ebca6b);
    for (int i = 0; i < MAXW; i++) ev[i] = 32'hx;
    $readmemh("cp_vec.hex", ev);
    $display("tb_cp: seed %0d, mem latency 1..%0d, ready %0d%%", SEED, lat_max, rdy_pct);
    do_reset();
    replay();
    $display("-- directed (RTL-only) tests");
    do_reset(); t_regs();          $display("  t_regs done, errors %0d", errors);
    do_reset(); t_core_reset();    $display("  t_core_reset done, errors %0d", errors);
    do_reset(); t_wdist();         $display("  t_wdist done, errors %0d", errors);
    do_reset(); t_backpressure();  $display("  t_backpressure done, errors %0d", errors);
    do_reset(); t_copy_abort();    $display("  t_copy_abort done, errors %0d", errors);
    do_reset(); t_mem_core_reset(); $display("  t_mem_core_reset done, errors %0d", errors);
    do_reset(); t_scrub();         $display("  t_scrub done, errors %0d", errors);
    $display("tb_cp: %0d checks, %0d memory accesses, %0d errors", checks, accesses, errors);
    if (errors == 0) $display("NG_CP_TB_PASS");
    else $display("NG_CP_TB_FAIL");
    $finish;
  end

  initial begin
    #2_000_000_000;
    $display("TB_FAIL global timeout");
    $finish;
  end
endmodule
