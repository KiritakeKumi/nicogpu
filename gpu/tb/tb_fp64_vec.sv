// tb_fp64_vec.sv -- FP64 (ng_fp64) vs functional-model vectors.
//
// Consumes the flat text produced by sim/sfu/vec2txt_sfu.py --unit fp64 from nicogpu-vec/0.1
// `arch_insn` vectors (tools/vecexport fp64_sfu unit, or sim/fp64 random exporter).  For each vector: load the
// snapshot into the register file (ng_sp_regfile), issue the one
// instruction (with the vector's MODE) to ng_fp64, wait for done/trap, then read back ALL vregs and
// 64 uregs, and the accumulated fflags, and compare with "snapshot + expected write diff" (so any write the
// model does not make is caught, including masked lanes and out-of-diff
// registers).  Expected traps: cause + info must match and nothing may change.
//
// Runs under xsim (Vivado) and Verilator 5 (--binary --timing).  Plusargs:
//   +VEC=<file>   vector text (default fp64.txt)
//   +MAXFAIL=<n>  stop printing after n failing vectors (default 10)
// Parameter NL (lanes per beat) selects the L (16) or XL (32) configuration.
`timescale 1ns / 1ps

`include "tb_fp64_cfg.svh"   // written by run_tb_fp64.sh: `define NG_TB_NL / NG_TB_NFL / NG_TB_WP / NG_TB_BG

module tb_fp64_vec;
  parameter int NL = `NG_TB_NL;
  parameter int NFL = `NG_TB_NFL;   // FP64 physical lanes
  parameter int VWP = `NG_TB_VWP;   // register-file write port used by the FP64
  parameter bit BG = `NG_TB_BG;     // background writer on the other write port

  localparam int NBEAT = 32 / NL;
  localparam int BTW   = (NL < 32) ? $clog2(32 / NL) : 1;
  localparam int SW    = 4;
  localparam int DW    = NL * 32;
  localparam int SLOT  = 3;       // warp slot used for all vectors (non-zero on purpose)
  localparam int BGSLOT = 5;      // warp slot of the background writer (physical base 512)

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // ------------------------------------------------------------ DUT side (ng_fp64)
  logic              iss_valid, iss_ready;
  logic [31:0]       iss_w0, iss_w1, iss_exec, iss_mode;
  logic [SW-1:0]     iss_slot;
  logic [7:0]        iss_nregs;
  logic [15:0]       iss_tag;
  logic              done_valid, trap_valid, busy, fflags_valid;
  logic [4:0]        fflags_set;
  logic [15:0]       done_tag;
  logic [3:0]        trap_cause;
  logic [31:0]       trap_info;

  logic              v_rd_valid;
  logic [SW-1:0]     v_rd_slot;
  logic [6:0]        v_rd_reg;
  logic [BTW-1:0]    v_rd_beat;
  logic [1:0]        v_rd_hsel;
  logic              v_uw_valid;
  logic [SW-1:0]     v_uw_slot;
  logic [5:0]        v_uw_reg;
  logic [31:0]       v_uw_data;
  logic              v_wr_valid;
  logic [SW-1:0]     v_wr_slot;
  logic [6:0]        v_wr_reg;
  logic [BTW-1:0]    v_wr_beat;
  logic [NL-1:0]     v_wr_lanes;
  logic [1:0]        v_wr_hsel;
  logic [DW-1:0]     v_wr_data;

  // ------------------------------------------------------------ TB side (debug access)
  logic              dbg = 1'b1;       // 1: testbench owns the register-file ports
  logic              t_rd_valid = 1'b0;
  logic [6:0]        t_rd_reg = '0;
  logic [BTW-1:0]    t_rd_beat = '0;
  logic              t_wr_valid = 1'b0;
  logic [6:0]        t_wr_reg = '0;
  logic [BTW-1:0]    t_wr_beat = '0;
  logic [DW-1:0]     t_wr_data = '0;
  logic              t_ur_valid = 1'b0;
  logic [5:0]        t_ur_reg = '0;
  logic              t_uw_valid = 1'b0;
  logic [5:0]        t_uw_reg = '0;
  logic [31:0]       t_uw_data = '0;

  // ------------------------------------------------------------ background writer (BG)
  logic              bg_valid = 1'b0;
  logic [6:0]        bg_reg = '0;
  logic [BTW-1:0]    bg_beat = '0;
  logic [DW-1:0]     bg_data = '0;
  logic [31:0]       lfsr = 32'h1badcafe;
  always @(posedge clk) begin
    lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};
    bg_valid <= BG && !dbg && lfsr[3];
    bg_reg   <= lfsr[14:8];
    bg_beat  <= BTW'(lfsr[16]);
    bg_data  <= {(DW/32){lfsr}};
  end

  // ------------------------------------------------------------ register file
  logic [2:0]        rf_rd_valid, rf_rd_ready, rf_rdv_valid, rf_rdv_err;
  logic [SW-1:0]     rf_rd_slot [3];
  logic [6:0]        rf_rd_reg  [3];
  logic [BTW-1:0]    rf_rd_beat [3];
  logic [1:0]        rf_rd_hsel [3];
  logic [DW-1:0]     rf_rdv_data [3];
  logic              rf_perf;
  logic [1:0]        rf_wr_valid, rf_wr_ready, rf_wr_err;
  logic [SW-1:0]     rf_wr_slot [2];
  logic [6:0]        rf_wr_reg  [2];
  logic [BTW-1:0]    rf_wr_beat [2];
  logic [NL-1:0]     rf_wr_lanes [2];
  logic [1:0]        rf_wr_hsel [2];
  logic [DW-1:0]     rf_wr_data [2];
  logic [2:0]        rf_ur_valid, rf_urd_valid;
  logic [SW-1:0]     rf_ur_slot [3];
  logic [5:0]        rf_ur_reg  [3];
  logic [31:0]       rf_urd_data [3];
  logic [1:0]        rf_uw_valid;
  logic [SW-1:0]     rf_uw_slot [2];
  logic [5:0]        rf_uw_reg  [2];
  logic [31:0]       rf_uw_data [2];
  logic              alloc_valid = 1'b0;
  logic [SW-1:0]     alloc_slot = '0;
  logic [9:0]        alloc_base = '0;
  logic [7:0]        alloc_nregs = '0;
  logic              alloc_err;

  ng_sp_regfile #(.NG_RF_NL(NL)) u_rf (
    .clk(clk), .rst_n(rst_n),
    .alloc_valid(alloc_valid), .alloc_slot(alloc_slot), .alloc_base(alloc_base),
    .alloc_nregs(alloc_nregs), .alloc_err(alloc_err),
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

  // port multiplexing: dbg=1 -> TB, dbg=0 -> FP64
  always_comb begin
    for (int k = 0; k < 3; k++) begin
      rf_rd_valid[k] = 1'b0;
      rf_rd_slot[k]  = SW'(SLOT);
      rf_rd_reg[k]   = '0;
      rf_rd_beat[k]  = '0;
      rf_rd_hsel[k]  = 2'd0;
    end
    if (dbg) begin
      rf_rd_valid[0] = t_rd_valid;
      rf_rd_reg[0]   = t_rd_reg;
      rf_rd_beat[0]  = t_rd_beat;
    end else begin
      rf_rd_valid[0] = v_rd_valid;
      rf_rd_slot[0] = v_rd_slot; rf_rd_reg[0] = v_rd_reg;
      rf_rd_beat[0] = v_rd_beat; rf_rd_hsel[0] = v_rd_hsel;
    end
    rf_wr_valid    = 2'b00;
    for (int k = 0; k < 2; k++) begin
      rf_wr_slot[k] = SW'(SLOT); rf_wr_reg[k] = '0; rf_wr_beat[k] = '0; rf_wr_lanes[k] = '0;
      rf_wr_hsel[k] = 2'b11;     rf_wr_data[k] = '0;
    end
    if (dbg) begin
      rf_wr_valid[0] = t_wr_valid;
      rf_wr_reg[0]   = t_wr_reg;
      rf_wr_beat[0]  = t_wr_beat;
      rf_wr_lanes[0] = '1;
      rf_wr_data[0]  = t_wr_data;
    end else begin
      // the FP64 owns write port VWP; with BG the other port carries unrelated
      // traffic to warp slot BGSLOT (same banks) so the FP64 sees wr_ready=0.
      rf_wr_valid[VWP] = v_wr_valid;
      rf_wr_slot[VWP]  = v_wr_slot; rf_wr_reg[VWP] = v_wr_reg; rf_wr_beat[VWP] = v_wr_beat;
      rf_wr_lanes[VWP] = v_wr_lanes; rf_wr_hsel[VWP] = v_wr_hsel; rf_wr_data[VWP] = v_wr_data;
      if (BG) begin
        rf_wr_valid[1-VWP] = bg_valid;
        rf_wr_slot[1-VWP]  = SW'(BGSLOT); rf_wr_reg[1-VWP] = bg_reg; rf_wr_beat[1-VWP] = bg_beat;
        rf_wr_lanes[1-VWP] = '1; rf_wr_hsel[1-VWP] = 2'b11; rf_wr_data[1-VWP] = bg_data;
      end
    end
    rf_ur_valid   = 3'b000;
    for (int k = 0; k < 3; k++) begin rf_ur_slot[k] = SW'(SLOT); rf_ur_reg[k] = '0; end
    if (dbg) begin
      rf_ur_valid[0] = t_ur_valid;
      rf_ur_reg[0]   = t_ur_reg;
    end
    rf_uw_valid   = 2'b00;
    rf_uw_slot[0] = SW'(SLOT); rf_uw_reg[0] = '0; rf_uw_data[0] = '0;
    rf_uw_slot[1] = '0; rf_uw_reg[1] = '0; rf_uw_data[1] = '0;
    if (dbg) begin
      rf_uw_valid[0] = t_uw_valid; rf_uw_reg[0] = t_uw_reg; rf_uw_data[0] = t_uw_data;
    end else begin
      rf_uw_valid[0] = v_uw_valid; rf_uw_slot[0] = v_uw_slot; rf_uw_reg[0] = v_uw_reg; rf_uw_data[0] = v_uw_data;
    end
  end

  // ----------------------------------------------------------
  ng_fp64 #(.NL(NL), .SLOTW(SW), .NFL(NFL)) dut (
    .clk(clk), .rst_n(rst_n),
    .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1),
    .iss_exec(iss_exec), .iss_slot(iss_slot), .iss_nregs(iss_nregs), .iss_mode(iss_mode), .iss_tag(iss_tag),
    .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid),
    .trap_cause(trap_cause), .trap_info(trap_info), .fflags_valid(fflags_valid), .fflags_set(fflags_set),
    .busy(busy),
    .rd_valid(v_rd_valid), .rd_ready(rf_rd_ready[0]), .rd_slot(v_rd_slot), .rd_reg(v_rd_reg),
    .rd_beat(v_rd_beat), .rd_hsel(v_rd_hsel), .rdv_valid(rf_rdv_valid[0]), .rdv_err(rf_rdv_err[0]),
    .rdv_data(rf_rdv_data[0]),
    .wr_valid(v_wr_valid), .wr_ready(rf_wr_ready[VWP]), .wr_slot(v_wr_slot), .wr_reg(v_wr_reg),
    .wr_beat(v_wr_beat), .wr_lanes(v_wr_lanes), .wr_hsel(v_wr_hsel), .wr_data(v_wr_data),
    .uw_valid(v_uw_valid), .uw_slot(v_uw_slot), .uw_reg(v_uw_reg), .uw_data(v_uw_data)
  );

  // ------------------------------------------------------------ debug access tasks
  // All testbench stimulus is driven, and all DUT outputs are sampled, on the falling
  // edge.  Changing inputs at the rising edge races with the DUT's own sampling in
  // the Verilator 5 simulator (the testbench process and the clocked logic are not
  // ordered the way xsim orders them): lost handshakes and zero read data.
  task automatic write_vreg(input int r, input logic [31:0] vals [32]);
    for (int b = 0; b < NBEAT; b++) begin
      logic [DW-1:0] d;
      for (int i = 0; i < NL; i++) d[i*32 +: 32] = vals[b*NL + i];
      t_wr_valid <= 1'b1; t_wr_reg <= 7'(r); t_wr_beat <= BTW'(b); t_wr_data <= d;
      @(negedge clk);
      while (!rf_wr_ready[0]) @(negedge clk);
    end
    t_wr_valid <= 1'b0;
  endtask

  task automatic read_vreg(input int r, output logic [31:0] vals [32]);
    for (int b = 0; b < NBEAT; b++) begin
      t_rd_valid <= 1'b1; t_rd_reg <= 7'(r); t_rd_beat <= BTW'(b);
      @(negedge clk);
      while (!rf_rd_ready[0]) @(negedge clk);
      // granted at the rising edge before this falling edge; the response (registered
      // there) is stable now and until the next rising edge
      for (int i = 0; i < NL; i++) vals[b*NL + i] = rf_rdv_data[0][i*32 +: 32];
      t_rd_valid <= 1'b0;
    end
  endtask

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

  // ------------------------------------------------------------ vector reader
  int fd;
  int nvec, ninsn;
  logic [8*256-1:0] vecfile;
  int maxfail;

  logic [31:0] in_v  [16][32];
  logic [31:0] e_v   [128][32];
  logic [31:0] e_u   [64];
  logic [31:0] g_v   [32];
  logic [31:0] g_u;
  logic [31:0] zero_v [32];

  int insn_pass [512];
  int insn_fail [512];

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

  int n_pass, n_fail, n_skip;
  logic [31:0] ff_reg = '0;     // the warp's fflags CSR as the FP64's fflags pulses update it
  always @(posedge clk) if (fflags_valid) ff_reg <= ff_reg | {27'd0, fflags_set};

  // coverage counters (report only): write stalls seen by the FP64, read bank conflicts
  int cov_wr_stall = 0, cov_rd_conf = 0, cov_trap = 0;
  always @(posedge clk) begin
    if (!dbg && v_wr_valid && !rf_wr_ready[VWP]) cov_wr_stall++;
    if (!dbg && rf_perf) cov_rd_conf++;
    if (trap_valid) cov_trap++;
  end

  int shown;

  initial begin
    int rc;
    if (!$value$plusargs("VEC=%s", vecfile)) vecfile = "fp64.txt";
    if (!$value$plusargs("MAXFAIL=%d", maxfail)) maxfail = 10;
    fd = $fopen(vecfile, "r");
    if (fd == 0) begin
      $display("TB_ERROR: cannot open %s", vecfile);
      $finish;
    end
    nvec  = int'(rd_hex());
    ninsn = int'(rd_hex());
    for (int i = 0; i < 32; i++) zero_v[i] = 32'd0;
    for (int i = 0; i < 512; i++) begin insn_pass[i] = 0; insn_fail[i] = 0; end
    n_pass = 0; n_fail = 0; shown = 0;

    iss_valid = 1'b0; iss_w0 = '0; iss_w1 = '0; iss_exec = '0;
    iss_slot = SW'(SLOT); iss_nregs = 8'd128; iss_tag = 16'h0; iss_mode = '0;
    repeat (5) @(negedge clk);
    rst_n <= 1'b1;
    repeat (3) @(negedge clk);
    alloc_valid <= 1'b1; alloc_slot <= SW'(BGSLOT); alloc_base <= 10'd512; alloc_nregs <= 8'd128;
    @(negedge clk);
    alloc_valid <= 1'b0;
    @(negedge clk);

    for (int vi = 0; vi < nvec; vi++) begin
      logic [8*32-1:0] id_s, name_s;
      int    idx, nwords;
      logic [31:0] w0, w1, ex, mode_v, ff0, ffx;
      int    nregs;
      int    nu, tr_f, nwv, nwu;
      logic [31:0] tr_cause, tr_info;
      logic [31:0] tmp;
      logic        bad, got_trap, got_done;
      int          waitc, nbad;

      rc = $fscanf(fd, "%s %s %h", id_s, name_s, idx);
      nwords = int'(rd_hex());
      w0 = rd_hex(); w1 = rd_hex(); ex = rd_hex(); nregs = int'(rd_hex());
      mode_v = rd_hex(); ff0 = rd_hex();
      for (int r = 0; r < 16; r++) for (int l = 0; l < 32; l++) in_v[r][l] = rd_hex();
      for (int r = 0; r < 128; r++) for (int l = 0; l < 32; l++) e_v[r][l] = (r < 16) ? in_v[r][l] : 32'd0;
      for (int u = 0; u < 64; u++) e_u[u] = 32'd0;
      nu = int'(rd_hex());
      for (int k = 0; k < nu; k++) begin
        int ui;
        ui = int'(rd_hex());
        e_u[ui] = rd_hex();
      end
      tr_f = int'(rd_hex()); tr_cause = rd_hex(); tr_info = rd_hex();

      // ---- load the snapshot (dbg owns the ports); uregs first from e_u (== input so far)
      dbg = 1'b1;
      alloc_valid <= 1'b1; alloc_slot <= SW'(SLOT); alloc_base <= 10'd0; alloc_nregs <= 8'(nregs);
      @(negedge clk);
      alloc_valid <= 1'b0;
      @(negedge clk);
      for (int r = 0; r < nregs; r++) begin
        for (int l = 0; l < 32; l++) g_v[l] = e_v[r][l];
        write_vreg(r, g_v);
      end
      for (int u = 0; u < 64; u++) write_ureg(u, e_u[u]);

      // ---- expected post-state = snapshot + diff
      nwv = int'(rd_hex());
      for (int k = 0; k < nwv; k++) begin
        int rr;
        rr = int'(rd_hex());
        for (int l = 0; l < 32; l++) e_v[rr][l] = rd_hex();
      end
      nwu = int'(rd_hex());
      for (int k = 0; k < nwu; k++) begin
        int ui;
        ui = int'(rd_hex());
        e_u[ui] = rd_hex();
      end
      ffx = rd_hex();

      // ---- run the instruction
      // Issue / completion are driven / sampled on the falling edge (the unit is idle,
      // so it accepts at the one rising edge in between; the one-cycle done/trap pulse
      // is stable at every falling edge).  Sampling right after @(posedge clk) races
      // under Verilator and can miss the pulse.
      @(negedge clk);
      dbg = 1'b0;
      ff_reg = ff0;
      @(negedge clk);
      iss_w0 = w0; iss_w1 = w1; iss_exec = ex; iss_nregs = 8'(nregs); iss_mode = mode_v; iss_valid = 1'b1;
      @(negedge clk);
      iss_valid = 1'b0;
      got_trap = 1'b0; got_done = 1'b0; waitc = 0;
      while (!got_trap && !got_done && waitc < 200000) begin
        if (trap_valid) got_trap = 1'b1;
        if (done_valid) got_done = 1'b1;
        if (!got_trap && !got_done) begin
          @(negedge clk);
          waitc++;
        end
      end
      if ($test$plusargs("LAT") && got_done) $display("LAT %0s %0d %0d", name_s, waitc, $countones(ex));
      bad = 1'b0; nbad = 0;
      if (!got_trap && !got_done) begin
        $display("FAIL %0s %0s: timeout (no done/trap)", id_s, name_s);
        bad = 1'b1;
      end else if (tr_f != 0) begin
        if (!got_trap || trap_cause !== tr_cause[3:0] || trap_info !== tr_info) begin
          $display("FAIL %0s %0s: expected trap cause=%0d info=%08x, got trap=%0d cause=%0d info=%08x done=%0d",
                   id_s, name_s, tr_cause, tr_info, got_trap, trap_cause, trap_info, got_done);
          bad = 1'b1;
        end
      end else if (got_trap) begin
        $display("FAIL %0s %0s: unexpected trap cause=%0d info=%08x", id_s, name_s, trap_cause, trap_info);
        bad = 1'b1;
      end
      repeat (2) @(negedge clk);
      if (ff_reg !== ffx) begin
        $display("FAIL %0s %0s: fflags expected %02x got %02x (mode=%08x)", id_s, name_s, ffx, ff_reg, mode_v);
        bad = 1'b1;
      end

      // ---- read back everything and compare
      dbg = 1'b1;
      for (int r = 0; r < nregs; r++) begin
        read_vreg(r, g_v);
        for (int l = 0; l < 32; l++) begin
          if (g_v[l] !== e_v[r][l]) begin
            if (nbad < 4 && shown < maxfail)
              $display("FAIL %0s %0s: v%0d lane %0d expected %08x got %08x", id_s, name_s, r, l, e_v[r][l], g_v[l]);
            nbad++; bad = 1'b1;
          end
        end
      end
      for (int u = 0; u < 64; u++) begin
        read_ureg(u, g_u);
        if (g_u !== e_u[u]) begin
          if (nbad < 4 && shown < maxfail)
            $display("FAIL %0s %0s: u%0d expected %08x got %08x", id_s, name_s, u, e_u[u], g_u);
          nbad++; bad = 1'b1;
        end
      end
      if (bad) begin
        n_fail++; shown++; insn_fail[idx]++;
        if (shown <= maxfail) $display("     (w0=%08x w1=%08x exec=%08x)", w0, w1, ex);
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
      $display("NG_FP64_VEC NL=%0d NFL=%0d VWP=%0d BG=%0d vectors pass=%0d fail=%0d of %0d; instructions pass=%0d fail=%0d of %0d",
               NL, NFL, VWP, BG, n_pass, n_fail, nvec, ok_insn, bad_insn, ninsn);
      $display("cov: wr_stall_cycles=%0d rd_bank_conflict_cycles=%0d traps=%0d", cov_wr_stall, cov_rd_conf, cov_trap);
      if (n_fail == 0 && n_pass == nvec) $display("NG_FP64_VEC_ALL_OK");
      else $display("NG_FP64_VEC_FAILED");
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
