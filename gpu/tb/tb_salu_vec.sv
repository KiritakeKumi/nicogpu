// tb_salu_vec.sv -- SALU / branch / misc-sync-CSR unit vs functional-model vectors.
//
// Consumes the flat text produced by rtl/gpu/salu/vec/vec2txt_salu.py (stock
// exports of the int and misc units: 81 SALU + 10 branch + 16 sync/misc/CSR
// instructions, plus the randomised dump_salu_rand vectors).  Per vector:
//   1. launch the warp slot (winit: EXEC, scratch/code parameters; CSRs <- 0);
//   2. load the CSR snapshot by executing csrw instructions on the DUT itself;
//   3. load the 64 uniform registers through the register-file debug port;
//   4. issue the one instruction, wait for exactly one done/trap pulse, compare
//      trap cause/info, branch outcome (br_*), sync event (ev_*);
//   5. read back all 64 uregs and compare with "snapshot + expected diff" (any write
//      the model does not make is caught), then read EXEC and the CSR block back with
//      csrr instructions and compare with the model's final state;
//   6. before each launch, check that EXEC of every other slot still holds the value
//      the model expects (per-slot state isolation).
// Runs under xsim and Verilator 5 (--binary --timing).  Plusargs:
//   +VEC=<file>     vector text (default salu.txt)
//   +MAXFAIL=<n>    stop printing after n failing vectors (default 10)
//   +STALL          random ur_ready/uw_ready back-pressure from the register-file arbiter
`timescale 1ns / 1ps

module tb_salu_vec;
  localparam int SW = 4;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // ------------------------------------------------------------ DUT side
  logic              iss_valid, iss_ready;
  logic [31:0]       iss_w0, iss_w1, iss_pc;
  logic [SW-1:0]     iss_slot;
  logic [15:0]       iss_tag;
  logic              done_valid, trap_valid, busy;
  logic [15:0]       done_tag;
  logic [3:0]        trap_cause;
  logic [31:0]       trap_info;
  logic              br_valid, br_taken;
  logic [SW-1:0]     br_slot;
  logic [31:0]       br_target;
  logic              ev_valid;
  logic [SW-1:0]     ev_slot;
  logic [2:0]        ev_kind;
  logic [19:0]       ev_imm;
  logic [24:0]       ev_wait;

  logic [1:0]        s_ur_valid;
  logic              s_ur_ready, s_uw_ready;
  logic [SW-1:0]     s_ur_slot [2];
  logic [5:0]        s_ur_reg  [2];
  logic [31:0]       s_urd_data [2];
  logic [1:0]        s_uw_valid;
  logic [SW-1:0]     s_uw_slot [2];
  logic [5:0]        s_uw_reg  [2];
  logic [31:0]       s_uw_data [2];

  logic              winit_valid = 1'b0;
  logic [SW-1:0]     winit_slot = '0;
  logic [31:0]       winit_exec = '0, winit_code_lo = '0, winit_code_hi = '0;
  logic [31:0]       winit_scratch_lo = '0, winit_scratch_hi = '0, winit_scratch_size = '0, winit_wgslot = '0;
  logic [31:0]       sp_id = '0, core_id = '0;
  logic [SW-1:0]     cur_slot;
  logic [63:0]       hw_clock = '0, hw_time = '0;
  logic [SW-1:0]     q_slot = '0;
  logic [31:0]       q_exec, q_mode, q_trapctl;
  logic [4:0]        q_fflags;
  logic              fflags_set_valid = 1'b0;
  logic [SW-1:0]     fflags_set_slot = '0;
  logic [4:0]        fflags_set_bits = '0;
  logic              xtrap_valid = 1'b0;
  logic [SW-1:0]     xtrap_slot = '0;
  logic [3:0]        xtrap_cause = '0;
  logic [31:0]       xtrap_pc = '0, xtrap_info = '0;

  // ------------------------------------------------------------ TB side (debug access)
  logic              dbg = 1'b1;       // 1: testbench owns the register-file ports
  logic [SW-1:0]     tb_slot = '0;
  logic              t_ur_valid = 1'b0;
  logic [5:0]        t_ur_reg = '0;
  logic              t_uw_valid = 1'b0;
  logic [5:0]        t_uw_reg = '0;
  logic [31:0]       t_uw_data = '0;

  // ------------------------------------------------------------ register file
  logic [2:0]        rf_ur_valid, rf_urd_valid;
  logic [SW-1:0]     rf_ur_slot [3];
  logic [5:0]        rf_ur_reg  [3];
  logic [31:0]       rf_urd_data [3];
  logic [1:0]        rf_uw_valid;
  logic [SW-1:0]     rf_uw_slot [2];
  logic [5:0]        rf_uw_reg  [2];
  logic [31:0]       rf_uw_data [2];

  logic [2:0]        rf_rd_valid = '0, rf_rd_ready, rf_rdv_valid, rf_rdv_err;
  logic [SW-1:0]     rf_rd_slot [3];
  logic [6:0]        rf_rd_reg  [3];
  logic [0:0]        rf_rd_beat [3];
  logic [1:0]        rf_rd_hsel [3];
  logic [511:0]      rf_rdv_data [3];
  logic              rf_perf;
  logic [1:0]        rf_wr_valid = '0, rf_wr_ready, rf_wr_err;
  logic [SW-1:0]     rf_wr_slot [2];
  logic [6:0]        rf_wr_reg  [2];
  logic [0:0]        rf_wr_beat [2];
  logic [15:0]       rf_wr_lanes [2];
  logic [1:0]        rf_wr_hsel [2];
  logic [511:0]      rf_wr_data [2];
  logic              alloc_err;

  initial begin
    for (int k = 0; k < 3; k++) begin rf_rd_slot[k] = '0; rf_rd_reg[k] = '0; rf_rd_beat[k] = '0; rf_rd_hsel[k] = '0; end
    for (int k = 0; k < 2; k++) begin
      rf_wr_slot[k] = '0; rf_wr_reg[k] = '0; rf_wr_beat[k] = '0; rf_wr_lanes[k] = '0; rf_wr_hsel[k] = '0; rf_wr_data[k] = '0;
    end
  end

  ng_sp_regfile #(.NG_RF_NL(16)) u_rf (
    .clk(clk), .rst_n(rst_n),
    .alloc_valid(1'b0), .alloc_slot('0), .alloc_base('0), .alloc_nregs('0), .alloc_err(alloc_err),
    .rd_valid(rf_rd_valid), .rd_ready(rf_rd_ready), .rd_slot(rf_rd_slot), .rd_reg(rf_rd_reg),
    .rd_beat(rf_rd_beat), .rd_hsel(rf_rd_hsel), .rdv_valid(rf_rdv_valid), .rdv_err(rf_rdv_err),
    .rdv_data(rf_rdv_data), .perf_rd_conflict(rf_perf),
    .wr_valid(rf_wr_valid), .wr_ready(rf_wr_ready), .wr_slot(rf_wr_slot), .wr_reg(rf_wr_reg),
    .wr_beat(rf_wr_beat), .wr_lanes(rf_wr_lanes), .wr_hsel(rf_wr_hsel), .wr_data(rf_wr_data),
    .wr_err(rf_wr_err),
    .ur_valid(rf_ur_valid), .ur_slot(rf_ur_slot), .ur_reg(rf_ur_reg),
    .urd_valid(rf_urd_valid), .urd_data(rf_urd_data),
    .uw_valid(rf_uw_valid), .uw_slot(rf_uw_slot), .uw_reg(rf_uw_reg), .uw_data(rf_uw_data)
  );

  // optional back-pressure (register-file arbiter model)
  logic        stall_en = 1'b0;
  logic [31:0] lfsr = 32'h1badcafe;
  always @(negedge clk) lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
  always_comb begin
    s_ur_ready = !(stall_en && !dbg && lfsr[3] && lfsr[9]);
    s_uw_ready = !(stall_en && !dbg && lfsr[5] && lfsr[11]);
  end

  // port multiplexing: dbg=1 -> TB, dbg=0 -> SALU (requests gated by the ready bits)
  always_comb begin
    rf_ur_valid = 3'b000;
    for (int k = 0; k < 3; k++) begin rf_ur_slot[k] = tb_slot; rf_ur_reg[k] = '0; end
    if (dbg) begin
      rf_ur_valid[0] = t_ur_valid;
      rf_ur_reg[0]   = t_ur_reg;
    end else begin
      rf_ur_valid[1:0] = s_ur_valid & {2{s_ur_ready}};
      for (int k = 0; k < 2; k++) begin rf_ur_slot[k] = s_ur_slot[k]; rf_ur_reg[k] = s_ur_reg[k]; end
    end
    rf_uw_valid = 2'b00;
    for (int k = 0; k < 2; k++) begin rf_uw_slot[k] = tb_slot; rf_uw_reg[k] = '0; rf_uw_data[k] = '0; end
    if (dbg) begin
      rf_uw_valid[0] = t_uw_valid; rf_uw_reg[0] = t_uw_reg; rf_uw_data[0] = t_uw_data;
    end else begin
      rf_uw_valid = s_uw_valid & {2{s_uw_ready}};
      for (int k = 0; k < 2; k++) begin
        rf_uw_slot[k] = s_uw_slot[k]; rf_uw_reg[k] = s_uw_reg[k]; rf_uw_data[k] = s_uw_data[k];
      end
    end
  end
  always_comb begin s_urd_data[0] = rf_urd_data[0]; s_urd_data[1] = rf_urd_data[1]; end

  // ----------------------------------------------------------
  ng_salu #(.SLOTW(SW)) dut (
    .clk(clk), .rst_n(rst_n),
    .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1), .iss_pc(iss_pc),
    .iss_slot(iss_slot), .iss_tag(iss_tag),
    .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid),
    .trap_cause(trap_cause), .trap_info(trap_info), .busy(busy),
    .br_valid(br_valid), .br_slot(br_slot), .br_taken(br_taken), .br_target(br_target),
    .ev_valid(ev_valid), .ev_slot(ev_slot), .ev_kind(ev_kind), .ev_imm(ev_imm), .ev_wait(ev_wait),
    .ur_valid(s_ur_valid), .ur_ready(s_ur_ready), .ur_slot(s_ur_slot), .ur_reg(s_ur_reg),
    .urd_valid(rf_urd_valid[1:0]), .urd_data(s_urd_data),
    .uw_valid(s_uw_valid), .uw_ready(s_uw_ready), .uw_slot(s_uw_slot), .uw_reg(s_uw_reg), .uw_data(s_uw_data),
    .winit_valid(winit_valid), .winit_slot(winit_slot), .winit_exec(winit_exec),
    .winit_code_lo(winit_code_lo), .winit_code_hi(winit_code_hi),
    .winit_scratch_lo(winit_scratch_lo), .winit_scratch_hi(winit_scratch_hi),
    .winit_scratch_size(winit_scratch_size), .winit_wgslot(winit_wgslot),
    .sp_id(sp_id), .core_id(core_id), .cur_slot(cur_slot), .hw_clock(hw_clock), .hw_time(hw_time),
    .q_slot(q_slot), .q_exec(q_exec), .q_mode(q_mode), .q_fflags(q_fflags), .q_trapctl(q_trapctl), .qc_slot('0), .qc_exec(),
    .fflags_set_valid(fflags_set_valid), .fflags_set_slot(fflags_set_slot), .fflags_set_bits(fflags_set_bits),
    .xtrap_valid(xtrap_valid), .xtrap_slot(xtrap_slot), .xtrap_cause(xtrap_cause), .xtrap_pc(xtrap_pc),
    .xtrap_info(xtrap_info)
  );

  // ------------------------------------------------------------ pulse monitors (negedge sampling)
  int n_done_pulse = 0, n_trap_pulse = 0, n_br_pulse = 0, n_ev_pulse = 0;
  logic        m_done, m_trap, m_br, m_ev;
  logic [15:0] m_tag;
  logic [3:0]  m_cause;
  logic [31:0] m_info;
  logic        m_taken;
  logic [31:0] m_target;
  logic [2:0]  m_ekind;
  logic [19:0] m_eimm;
  logic [24:0] m_ewait;
  logic [SW-1:0] m_brslot, m_evslot;
  always @(negedge clk) begin
    if (done_valid) begin n_done_pulse++; m_tag = done_tag; end
    if (trap_valid) begin n_trap_pulse++; m_cause = trap_cause; m_info = trap_info; end
    if (br_valid) begin n_br_pulse++; m_taken = br_taken; m_target = br_target; m_brslot = br_slot; end
    if (ev_valid) begin n_ev_pulse++; m_ekind = ev_kind; m_eimm = ev_imm; m_ewait = ev_wait; m_evslot = ev_slot; end
  end

  // ------------------------------------------------------------ debug access tasks
  // Stimulus is driven and outputs sampled on the falling edge (see tb_valu_vec.sv).
  task automatic write_ureg(input int u, input logic [31:0] v);
    t_uw_valid <= 1'b1; t_uw_reg <= 6'(u); t_uw_data <= v;
    @(negedge clk);
    t_uw_valid <= 1'b0;
  endtask

  task automatic read_ureg(input int u, output logic [31:0] v);
    t_ur_valid <= 1'b1; t_ur_reg <= 6'(u);
    @(negedge clk);
    t_ur_valid <= 1'b0;
    @(negedge clk);
    v = rf_urd_data[0];
  endtask

  // Run one instruction on the DUT; returns when a done/trap pulse was seen (or timeout).
  // Pulse counters are cleared at issue, so the caller can inspect n_*_pulse / m_* afterwards.
  task automatic run_insn(input logic [31:0] w0, input logic [31:0] w1, input logic [31:0] pc,
                          input logic [SW-1:0] slot, input logic [15:0] tag, output bit timeout);
    int waitc;
    dbg = 1'b0;
    @(negedge clk);
    while (!iss_ready) @(negedge clk);
    n_done_pulse = 0; n_trap_pulse = 0; n_br_pulse = 0; n_ev_pulse = 0;
    iss_w0 = w0; iss_w1 = w1; iss_pc = pc; iss_slot = slot; iss_tag = tag; iss_valid = 1'b1;
    @(negedge clk);
    iss_valid = 1'b0;
    waitc = 0;
    while (n_done_pulse == 0 && n_trap_pulse == 0 && waitc < 2000) begin
      @(negedge clk);
      waitc++;
    end
    timeout = (n_done_pulse == 0 && n_trap_pulse == 0);
    // let late/duplicate pulses show up, then hand the ports back
    repeat (3) @(negedge clk);
    dbg = 1'b1;
  endtask

  function automatic logic [31:0] csrw_word(input int csr, input int rs);
    return 32'h52 | (32'(rs) << 13) | (32'd1 << 19) | (32'(csr) << 22);
  endfunction
  function automatic logic [31:0] csrr_word(input int csr, input int rd);
    return 32'h52 | (32'(rd) << 7) | (32'(csr) << 22);
  endfunction

  // CSR numbers in the order used by vec2txt_salu.py: mode fflags trapctl trapbase trapcause trappc trapinfo
  localparam int CSRN [7] = '{1, 2, 3, 64, 65, 66, 67};

  // ------------------------------------------------------------ vector reader
  int fd;
  int nvec, ninsn;
  logic [8*256-1:0] vecfile;
  int maxfail;
  int shown;
  int n_pass, n_fail;
  int insn_pass [512];
  int insn_fail [512];
  logic [31:0] e_u [64];
  logic [31:0] in_csr [7];
  logic [31:0] fin_csr [7];
  logic [31:0] g_u;

  function automatic logic [31:0] rd_hex();
    logic [31:0] x;
    int rc;
    rc = $fscanf(fd, "%h", x);
    if (rc != 1) begin
      $display("TB_ERROR: vector file truncated");
      $finish;
    end
    return x;
  endfunction

  // per-slot expected EXEC after the last vector that used the slot
  logic        slot_live [16];
  logic [31:0] slot_exec [16];
  int n_iso_bad;

  // coverage
  int cov_stall_ur = 0, cov_stall_uw = 0, cov_trap = 0, cov_br = 0, cov_br_taken = 0, cov_ev = 0;
  always @(negedge clk) begin
    if (!dbg && s_ur_valid != 0 && !s_ur_ready) cov_stall_ur++;
    if (!dbg && s_uw_valid != 0 && !s_uw_ready) cov_stall_uw++;
  end

  initial begin
    int rc;
    if (!$value$plusargs("VEC=%s", vecfile)) vecfile = "salu.txt";
    if (!$value$plusargs("MAXFAIL=%d", maxfail)) maxfail = 10;
    stall_en = $test$plusargs("STALL");
    fd = $fopen(vecfile, "r");
    if (fd == 0) begin
      $display("TB_ERROR: cannot open %s", vecfile);
      $finish;
    end
    nvec  = int'(rd_hex());
    ninsn = int'(rd_hex());
    for (int i = 0; i < 512; i++) begin insn_pass[i] = 0; insn_fail[i] = 0; end
    for (int i = 0; i < 16; i++) begin slot_live[i] = 1'b0; slot_exec[i] = '0; end
    n_pass = 0; n_fail = 0; shown = 0; n_iso_bad = 0;

    iss_valid = 1'b0; iss_w0 = '0; iss_w1 = '0; iss_pc = '0; iss_slot = '0; iss_tag = '0;
    repeat (5) @(negedge clk);
    rst_n <= 1'b1;
    repeat (3) @(negedge clk);

    for (int vi = 0; vi < nvec; vi++) begin
      logic [8*32-1:0] id_s, name_s;
      int    idx, nwords, nu, tr_f, nwu, chk_br, slot_i, nbad;
      logic [31:0] w0, w1, pc, ex, ex_after, sp, tr_cause, tr_info, np, csr_mask;
      logic [31:0] cb_lo, cb_hi, sc_lo, sc_hi, sc_sz;
      logic [SW-1:0] slot;
      logic        bad, tmo;
      logic [2:0]  ekind_exp;
      bit          is_ev, is_br_insn;

      rc = $fscanf(fd, "%s %s %h", id_s, name_s, idx);
      nwords = int'(rd_hex());
      w0 = rd_hex(); w1 = rd_hex(); pc = rd_hex(); ex = rd_hex(); slot_i = int'(rd_hex()); sp = rd_hex();
      cb_lo = rd_hex(); cb_hi = rd_hex(); sc_lo = rd_hex(); sc_hi = rd_hex(); sc_sz = rd_hex();
      for (int k = 0; k < 7; k++) in_csr[k] = rd_hex();
      for (int u = 0; u < 64; u++) e_u[u] = 32'd0;
      nu = int'(rd_hex());
      for (int k = 0; k < nu; k++) begin
        int ui;
        ui = int'(rd_hex());
        e_u[ui] = rd_hex();
      end
      tr_f = int'(rd_hex()); tr_cause = rd_hex(); tr_info = rd_hex();
      // slot: rotate through all slots; csrr warpid vectors need the model's slot
      slot = SW'(vi % 16);
      if (w0 == csrr_word(16, w0[12:7])) slot = SW'(slot_i);

      // ---- per-slot isolation check of the previous vectors' EXEC
      for (int s = 0; s < 16; s++) begin
        if (slot_live[s] && SW'(s) != slot) begin
          q_slot = SW'(s);
          #1;
          if (q_exec !== slot_exec[s]) begin
            if (n_iso_bad < 5) $display("FAIL isolation before %0s: slot %0d EXEC %08x expected %08x", id_s, s, q_exec, slot_exec[s]);
            n_iso_bad++;
          end
        end
      end

      // ---- launch the slot
      dbg = 1'b1;
      tb_slot = slot;
      sp_id = sp;
      @(negedge clk);
      winit_valid = 1'b1; winit_slot = slot; winit_exec = ex;
      winit_code_lo = cb_lo; winit_code_hi = cb_hi; winit_scratch_lo = sc_lo; winit_scratch_hi = sc_hi;
      winit_scratch_size = sc_sz; winit_wgslot = 32'd0;
      @(negedge clk);
      winit_valid = 1'b0;

      // ---- CSR snapshot via csrw (u0 as the carrier)
      for (int k = 0; k < 7; k++) begin
        if (in_csr[k] != 32'd0) begin
          write_ureg(0, in_csr[k]);
          run_insn(csrw_word(CSRN[k], 0), 32'd0, 32'd0, slot, 16'hfff0, tmo);
          if (tmo || n_trap_pulse != 0) begin
            $display("TB_ERROR %0s: CSR preload %0d failed (tmo=%0d trap=%0d)", id_s, k, tmo, n_trap_pulse);
            $finish;
          end
        end
      end

      // --
      for (int u = 0; u < 64; u++) write_ureg(u, 32'd0);
      begin
        // input uregs are the first nu entries of the stream; reload them
        // (e_u still equals the input here)
        for (int u = 0; u < 64; u++) if (e_u[u] != 32'd0) write_ureg(u, e_u[u]);
      end

      // ---- expected post-state
      nwu = int'(rd_hex());
      for (int k = 0; k < nwu; k++) begin
        int ui;
        ui = int'(rd_hex());
        e_u[ui] = rd_hex();
      end
      ex_after = rd_hex();
      for (int k = 0; k < 7; k++) fin_csr[k] = rd_hex();
      chk_br = int'(rd_hex()); np = rd_hex(); csr_mask = rd_hex();

      // ---- run the instruction
      hw_clock = 64'd0; hw_time = 64'd0;
      run_insn(w0, w1, pc, slot, 16'(vi), tmo);
      if ($test$plusargs("LAT") && !tmo) $display("LAT %0s", name_s);
      bad = 1'b0; nbad = 0;
      is_br_insn = (name_s == "jmp") || (name_s == "bexecz") || (name_s == "bexecnz") || (name_s == "jal") ||
                   (name_s == "bz") || (name_s == "bnz") || (name_s == "bnone") || (name_s == "bany") ||
                   (name_s == "jr") || (name_s == "jalr");
      ekind_exp = 3'd0;
      if (name_s == "nop") ekind_exp = 3'd1;
      else if (name_s == "endpgm") ekind_exp = 3'd2;
      else if (name_s == "barrier") ekind_exp = 3'd3;
      else if (name_s == "waitcnt") ekind_exp = 3'd4;
      else if (name_s == "sleep") ekind_exp = 3'd5;
      else if (name_s == "icache.inv") ekind_exp = 3'd6;
      else if (name_s == "setprio") ekind_exp = 3'd7;
      is_ev = (ekind_exp != 3'd0);

      if (tmo) begin
        $display("FAIL %0s %0s: timeout (no done/trap) st=%0d", id_s, name_s, dut.st);
        bad = 1'b1;
      end else if (n_done_pulse + n_trap_pulse != 1) begin
        $display("FAIL %0s %0s: %0d done + %0d trap pulses (want exactly one)", id_s, name_s, n_done_pulse, n_trap_pulse);
        bad = 1'b1;
      end else if (tr_f != 0) begin
        cov_trap++;
        if (n_trap_pulse != 1 || m_cause !== tr_cause[3:0] || m_info !== tr_info) begin
          $display("FAIL %0s %0s: expected trap cause=%0d info=%08x, got traps=%0d cause=%0d info=%08x done=%0d",
                   id_s, name_s, tr_cause, tr_info, n_trap_pulse, m_cause, m_info, n_done_pulse);
          bad = 1'b1;
        end
        if (n_br_pulse != 0 || n_ev_pulse != 0) begin
          $display("FAIL %0s %0s: br/ev pulse on a trapping instruction", id_s, name_s);
          bad = 1'b1;
        end
      end else begin
        if (n_trap_pulse != 0) begin
          $display("FAIL %0s %0s: unexpected trap cause=%0d info=%08x", id_s, name_s, m_cause, m_info);
          bad = 1'b1;
        end else begin
          if (m_tag !== 16'(vi)) begin
            $display("FAIL %0s %0s: done_tag %04x expected %04x", id_s, name_s, m_tag, 16'(vi)); bad = 1'b1;
          end
          // branch outcome
          if (is_br_insn) begin
            cov_br++;
            if (n_br_pulse != 1) begin
              $display("FAIL %0s %0s: %0d br pulses", id_s, name_s, n_br_pulse); bad = 1'b1;
            end else begin
              if (m_taken) cov_br_taken++;
              if (chk_br != 0) begin
                logic [31:0] got_np;
                got_np = m_taken ? m_target : (pc + 32'd4);
                if (got_np !== np || m_brslot !== slot) begin
                  $display("FAIL %0s %0s: next pc %08x (taken=%0d target=%08x) expected %08x", id_s, name_s,
                           got_np, m_taken, m_target, np);
                  bad = 1'b1;
                end
              end
            end
          end else if (n_br_pulse != 0) begin
            $display("FAIL %0s %0s: br pulse on a non-branch", id_s, name_s); bad = 1'b1;
          end
          // sync / misc event
          if (is_ev) begin
            cov_ev++;
            if (n_ev_pulse != 1 || m_ekind !== ekind_exp || m_evslot !== slot) begin
              $display("FAIL %0s %0s: event pulses=%0d kind=%0d expected %0d", id_s, name_s, n_ev_pulse, m_ekind, ekind_exp);
              bad = 1'b1;
            end else if ((name_s == "sleep" || name_s == "setprio") && m_eimm !== w0[31:12]) begin
              $display("FAIL %0s %0s: ev_imm %05x expected %05x", id_s, name_s, m_eimm, w0[31:12]); bad = 1'b1;
            end else if (name_s == "waitcnt" && m_ewait !== w0[31:7]) begin
              $display("FAIL %0s %0s: ev_wait %07x expected %07x", id_s, name_s, m_ewait, w0[31:7]); bad = 1'b1;
            end
          end else if (n_ev_pulse != 0) begin
            $display("FAIL %0s %0s: unexpected event pulse", id_s, name_s); bad = 1'b1;
          end
        end
      end

      // ---- read back the uregs
      for (int u = 0; u < 64; u++) begin
        read_ureg(u, g_u);
        if (g_u !== e_u[u]) begin
          if (nbad < 4 && shown < maxfail)
            $display("FAIL %0s %0s: u%0d expected %08x got %08x", id_s, name_s, u, e_u[u], g_u);
          nbad++; bad = 1'b1;
        end
      end

      // ---- read back EXEC and the CSR block (csrr on the DUT)
      for (int k = 0; k < 8; k++) begin
        int cn;
        logic [31:0] exp_v;
        cn = (k == 0) ? 0 : CSRN[k-1];
        exp_v = (k == 0) ? ex_after : fin_csr[k-1];
        run_insn(csrr_word(cn, 1), 32'd0, 32'd0, slot, 16'hfff1, tmo);
        if (tmo || n_trap_pulse != 0) begin
          $display("FAIL %0s %0s: csrr %0d readback failed", id_s, name_s, cn); bad = 1'b1;
        end else begin
          read_ureg(1, g_u);
          if (g_u !== exp_v && (k == 0 || csr_mask[k-1])) begin
            if (nbad < 8 && shown < maxfail)
              $display("FAIL %0s %0s: csr %0d expected %08x got %08x", id_s, name_s, cn, exp_v, g_u);
            nbad++; bad = 1'b1;
          end
        end
      end
      // q_* mirrors of exec/mode/fflags/trapctl
      q_slot = slot;
      #1;
      if (q_exec !== ex_after || q_mode !== fin_csr[0] || q_fflags !== fin_csr[1][4:0] || q_trapctl !== fin_csr[2]) begin
        // (mode/fflags/trapctl are always checked: mask bits 0..2 are set for every producer)
        $display("FAIL %0s %0s: q_* mismatch exec=%08x/%08x mode=%08x/%08x fflags=%02x/%02x trapctl=%08x/%08x",
                 id_s, name_s, q_exec, ex_after, q_mode, fin_csr[0], q_fflags, fin_csr[1][4:0], q_trapctl, fin_csr[2]);
        bad = 1'b1;
      end
      slot_live[slot] = 1'b1; slot_exec[slot] = ex_after;

      if (bad) begin
        n_fail++; shown++; insn_fail[idx]++;
        if (shown <= maxfail) $display("     (w0=%08x w1=%08x pc=%08x exec=%08x slot=%0d)", w0, w1, pc, ex, slot);
      end else begin
        n_pass++; insn_pass[idx]++;
      end
    end

    begin
      int ok_insn, bad_insn;
      ok_insn = 0; bad_insn = 0;
      for (int i = 0; i < ninsn; i++) begin
        if (insn_fail[i] > 0) bad_insn++;
        else if (insn_pass[i] > 0) ok_insn++;
      end
      $display("NG_SALU_VEC stall=%0d vectors pass=%0d fail=%0d of %0d; instructions pass=%0d fail=%0d of %0d; isolation_bad=%0d",
               stall_en, n_pass, n_fail, nvec, ok_insn, bad_insn, ninsn, n_iso_bad);
      $display("cov: traps=%0d branches=%0d taken=%0d events=%0d ur_stall_cycles=%0d uw_stall_cycles=%0d",
               cov_trap, cov_br, cov_br_taken, cov_ev, cov_stall_ur, cov_stall_uw);
      if (n_fail == 0 && n_pass == nvec && n_iso_bad == 0) $display("NG_SALU_VEC_ALL_OK");
      else $display("NG_SALU_VEC_FAILED");
    end
    $finish;
  end

  // global watchdog
  initial begin
    #2000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
