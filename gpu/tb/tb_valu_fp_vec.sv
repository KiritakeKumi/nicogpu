// tb_valu_fp_vec.sv -- f32 VALU (ng_valu EN_FP) vs functional-model vectors.
// Derived from tb_valu_vec.sv; the vector text additionally carries MODE / fflags
// (sim/sfu/vec2txt_sfu.py token stream): MODE is presented on iss_mode, the fflags the
// unit reports (fflags_valid/fflags_set) are OR-ed into a TB-side fflags register and
// compared with the expected final fflags (also on FpException traps).
//
// Consumes the flat text produced by sim/valu/vec2txt.py from nicogpu-vec/0.1
// `arch_insn` vectors (tools/vecexport, int unit).  For each vector: load the
// snapshot into the register file (ng_sp_regfile), issue the one
// instruction to ng_valu, wait for done/trap, then read back ALL 128 vregs and
// 64 uregs and compare with "snapshot + expected write diff" (so any write the
// model does not make is caught, including masked lanes and out-of-diff
// registers).  Expected traps: cause + info must match and nothing may change.
//
// Runs under xsim (Vivado) and Verilator 5 (--binary --timing).  Plusargs:
//   +VEC=<file>   vector text (default valu.txt)
//   +MAXFAIL=<n>  stop printing after n failing vectors (default 10)
// Parameter NL (lanes per beat) selects the L (16) or XL (32) configuration.
`timescale 1ns / 1ps

`include "tb_valu_fp_cfg.svh"
`ifndef NG_TB_DUAL
`define NG_TB_DUAL 0
`endif
`ifndef NG_TB_BDEPTH
`define NG_TB_BDEPTH 3
`endif   // written by run_tb_valu_fp.sh: `define NG_TB_NL <16|32>, NG_TB_NFL, NG_TB_VWP, NG_TB_BG
`ifndef NG_TB_F16
`define NG_TB_F16 1
`endif   // ng_valu EN_F16CVT (binary16 conversions; 0: they are illegal instructions)

module tb_valu_fp_vec;
  parameter int NL = `NG_TB_NL;
  parameter int NFL = `NG_TB_NFL;   // f32 physical lanes
  parameter int VWP = `NG_TB_VWP;   // register-file write port used by the VALU
  parameter bit BG = `NG_TB_BG;     // background writer on the other write port
  parameter int DUAL = `NG_TB_DUAL; // 1 = every vector runs next to a filler instruction of another warp slot;
                                    // 2 = next to two fillers (three warps in flight)
  parameter int BDEPTH = `NG_TB_BDEPTH; // ng_valu back-half contexts

  localparam int NBEAT = 32 / NL;
  localparam int BTW   = (NL < 32) ? $clog2(32 / NL) : 1;
  localparam int SW    = 4;
  localparam int DW    = NL * 32;
  localparam int SLOT  = 3;       // warp slot used for all vectors (non-zero on purpose)
  localparam int BGSLOT = 5;      // warp slot of the background writer (physical base 512)

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // ------------------------------------------------------------ DUT side (ng_valu)
  logic              iss_valid, iss_ready;
  logic [31:0]       iss_w0, iss_w1, iss_exec;
  logic [SW-1:0]     iss_slot;
  logic [7:0]        iss_nregs;
  logic [31:0]       iss_mode;
  logic              fflags_valid;
  logic [4:0]        fflags_set;
  logic [15:0]       iss_tag;
  logic              done_valid, trap_valid, busy;
  logic [15:0]       done_tag;
  logic [3:0]        trap_cause;
  logic [31:0]       trap_info;

  logic [2:0]        v_rd_valid;
  logic [SW-1:0]     v_rd_slot [3];
  logic [6:0]        v_rd_reg  [3];
  logic [BTW-1:0]    v_rd_beat [3];
  logic [1:0]        v_rd_hsel [3];
  logic              v_wr_valid;
  logic [SW-1:0]     v_wr_slot;
  logic [6:0]        v_wr_reg;
  logic [BTW-1:0]    v_wr_beat;
  logic [NL-1:0]     v_wr_lanes;
  logic [1:0]        v_wr_hsel;
  logic [DW-1:0]     v_wr_data;
  logic [1:0]        v_ur_valid;
  logic [SW-1:0]     v_ur_slot [2];
  logic [5:0]        v_ur_reg  [2];
  logic [31:0]       v_urd_data [2];
  logic              v_uw_valid;
  logic [SW-1:0]     v_uw_slot;
  logic [5:0]        v_uw_reg;
  logic [31:0]       v_uw_data;

  // ------------------------------------------------------------ TB side (debug access)
  logic              dbg = 1'b1;       // 1: testbench owns the register-file ports
  logic [SW-1:0]     dbg_slot = SW'(SLOT);   // warp slot addressed by the debug port
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

  // port multiplexing: dbg=1 -> TB, dbg=0 -> VALU
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
      rf_rd_slot[0]  = dbg_slot;
      rf_rd_reg[0]   = t_rd_reg;
      rf_rd_beat[0]  = t_rd_beat;
    end else begin
      rf_rd_valid = v_rd_valid;
      for (int k = 0; k < 3; k++) begin
        rf_rd_slot[k] = v_rd_slot[k]; rf_rd_reg[k] = v_rd_reg[k];
        rf_rd_beat[k] = v_rd_beat[k]; rf_rd_hsel[k] = v_rd_hsel[k];
      end
    end
    rf_wr_valid    = 2'b00;
    for (int k = 0; k < 2; k++) begin
      rf_wr_slot[k] = SW'(SLOT); rf_wr_reg[k] = '0; rf_wr_beat[k] = '0; rf_wr_lanes[k] = '0;
      rf_wr_hsel[k] = 2'b11;     rf_wr_data[k] = '0;
    end
    if (dbg) begin
      rf_wr_valid[0] = t_wr_valid;
      rf_wr_slot[0]  = dbg_slot;
      rf_wr_reg[0]   = t_wr_reg;
      rf_wr_beat[0]  = t_wr_beat;
      rf_wr_lanes[0] = '1;
      rf_wr_data[0]  = t_wr_data;
    end else begin
      // the VALU owns write port VWP; with BG the other port carries unrelated
      // traffic to warp slot BGSLOT (same banks) so the VALU sees wr_ready=0.
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
    for (int k = 0; k < 3; k++) begin rf_ur_slot[k] = dbg_slot; rf_ur_reg[k] = '0; end
    if (dbg) begin
      rf_ur_valid[0] = t_ur_valid;
      rf_ur_reg[0]   = t_ur_reg;
    end else begin
      rf_ur_valid[1:0] = v_ur_valid;
      for (int k = 0; k < 2; k++) begin rf_ur_slot[k] = v_ur_slot[k]; rf_ur_reg[k] = v_ur_reg[k]; end
    end
    rf_uw_valid   = 2'b00;
    rf_uw_slot[0] = dbg_slot; rf_uw_reg[0] = '0; rf_uw_data[0] = '0;
    rf_uw_slot[1] = '0; rf_uw_reg[1] = '0; rf_uw_data[1] = '0;
    if (dbg) begin
      rf_uw_valid[0] = t_uw_valid; rf_uw_reg[0] = t_uw_reg; rf_uw_data[0] = t_uw_data;
    end else begin
      rf_uw_valid[0] = v_uw_valid; rf_uw_slot[0] = v_uw_slot; rf_uw_reg[0] = v_uw_reg; rf_uw_data[0] = v_uw_data;
    end
  end

  always_comb begin v_urd_data[0] = rf_urd_data[0]; v_urd_data[1] = rf_urd_data[1]; end

  // ----------------------------------------------------------
  ng_valu #(.NL(NL), .SLOTW(SW), .EN_FP(1'b1), .NFL(NFL), .BDEPTH(BDEPTH), .EN_F16CVT(`NG_TB_F16)) dut (
    .clk(clk), .rst_n(rst_n),
    .iss_valid(iss_valid), .iss_ready(iss_ready), .iss_w0(iss_w0), .iss_w1(iss_w1),
    .iss_exec(iss_exec), .iss_slot(iss_slot), .iss_nregs(iss_nregs), .iss_mode(iss_mode), .iss_tag(iss_tag),
    .done_valid(done_valid), .done_tag(done_tag), .trap_valid(trap_valid),
    .trap_cause(trap_cause), .trap_info(trap_info), .fflags_valid(fflags_valid), .fflags_set(fflags_set), .busy(busy),
    .rd_valid(v_rd_valid), .rd_ready(rf_rd_ready), .rd_slot(v_rd_slot), .rd_reg(v_rd_reg),
    .rd_beat(v_rd_beat), .rd_hsel(v_rd_hsel), .rdv_valid(rf_rdv_valid), .rdv_err(rf_rdv_err),
    .rdv_data(rf_rdv_data),
    .wr_valid(v_wr_valid), .wr_ready(rf_wr_ready[VWP]), .wr_slot(v_wr_slot), .wr_reg(v_wr_reg),
    .wr_beat(v_wr_beat), .wr_lanes(v_wr_lanes), .wr_hsel(v_wr_hsel), .wr_data(v_wr_data),
    .ur_valid(v_ur_valid), .ur_slot(v_ur_slot), .ur_reg(v_ur_reg),
    .urd_valid(rf_urd_valid[1:0]), .urd_data(v_urd_data),
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


  // ------------------------------------------------------------ DUAL mode helpers
  // Filler A (slot FA_SLOT): add v10, v10, v11.  Filler B (slot FB_SLOT, DUAL=2, kind drawn per
  // vector): add64 v[12:13], v[10:11], v[12:13] (pair write, two W cycles),
  // cmp.lt.i32 u40, v12, v10 (uniform write), mov.vu v9, u40 or, in the f32 TB, fadd.f32 v14, v14, v15.
  // The fillers' slots are not the background writer's, so DUAL also runs with BG=1.
  localparam int FA_SLOT = 9;     // physical base 256
  localparam int FB_SLOT = 11;    // physical base 384
  localparam int NFB     = 4;     // filler-B kinds
  localparam logic [31:0] FILL_W0 = 32'h01628502;      // add v10, v10, v11
  localparam logic [31:0] FILLB_W0 [4] = '{32'h01828606,   // add64 v[12:13], v[10:11], v[12:13]
                                           32'h10a19426,   // cmp.lt.i32 u40, v12, v10
                                           32'h000a0490,   // mov.vu v9, u40 (no vector source: never waits for operand reads)
                                           32'h01e38714};  // fadd.f32 v14, v14, v15 (f32 TB only)
  logic [31:0] fil_a [32], fil_b [32];
  logic [31:0] fb_v [16][32];   // filler-B registers v9..v15 (expected)
  logic [31:0] fb_u40;
  int          pl_n = 0;
  int          pl_kind [$];
  logic [3:0]  pl_cause [$];
  logic [31:0] pl_info [$];
  // filler B's architectural effect (kind k)
  task automatic fb_apply(input int k);
    for (int l = 0; l < 32; l++) begin
      case (k)
        0: {fb_v[13][l], fb_v[12][l]} = {fb_v[13][l], fb_v[12][l]} + {fb_v[11][l], fb_v[10][l]};
        1: fb_u40[l] = ($signed(fb_v[12][l]) < $signed(fb_v[10][l]));
        2: fb_v[9][l] = fb_u40;
        default: fb_v[14][l] = $shortrealtobits($bitstoshortreal(fb_v[14][l]) + $bitstoshortreal(fb_v[15][l]));
      endcase
    end
  endtask
  always @(negedge clk) if (rst_n && DUAL != 0) begin
    if (done_valid) begin pl_kind.push_back(0); pl_cause.push_back(4'd0); pl_info.push_back(32'd0); end
    if (trap_valid) begin pl_kind.push_back(1); pl_cause.push_back(trap_cause); pl_info.push_back(trap_info); end
  end

  task automatic issue_insn(input logic [31:0] a0, input logic [31:0] a1, input logic [31:0] e,
                            input int slot, input int nr, input logic [31:0] md);
    while (!iss_ready) @(negedge clk);
    iss_w0 = a0; iss_w1 = a1; iss_exec = e; iss_slot = SW'(slot); iss_nregs = 8'(nr);
    iss_mode = md;
    iss_valid = 1'b1;
    @(negedge clk);
    iss_valid = 1'b0;
    iss_slot = SW'(SLOT);
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
  logic [31:0] ff_reg = '0;     // the warp's fflags CSR as the unit's fflags pulses update it
  // (DUAL: only the main vector's pulse, the d_mi-th of the vector, updates it; the fillers run in other warps)
  int ff_mi = 0;
  always @(posedge clk) if (fflags_valid && (DUAL == 0 || pl_kind.size() == ff_mi + 1)) ff_reg <= ff_reg | {27'd0, fflags_set};

  // coverage counters (report only): write stalls seen by the VALU, read bank conflicts
  int cov_wr_stall = 0, cov_rd_conf = 0, cov_trap = 0;
  always @(posedge clk) begin
    if (!dbg && v_wr_valid && !rf_wr_ready[VWP]) cov_wr_stall++;
    if (!dbg && rf_perf) cov_rd_conf++;
    if (trap_valid) cov_trap++;
  end

  int shown;
  logic [3:0]  mc;
  logic [31:0] mi;
  int          d_order, d_gap, d_mi, d_fi, d_fbk;
  logic        d_bad;

  initial begin
    int rc;
    if (!$value$plusargs("VEC=%s", vecfile)) vecfile = "valu_fp.txt";
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
    if (DUAL != 0) begin
      logic [31:0] seedv;
      alloc_valid <= 1'b1; alloc_slot <= SW'(FA_SLOT); alloc_base <= 10'd256; alloc_nregs <= 8'd128;
      @(negedge clk);
      alloc_slot <= SW'(FB_SLOT); alloc_base <= 10'd384;
      @(negedge clk);
      alloc_valid <= 1'b0;
      @(negedge clk);
      seedv = 32'h2545f491;
      dbg_slot = SW'(FA_SLOT);
      for (int l = 0; l < 32; l++) begin
        seedv = seedv * 32'd1664525 + 32'd1013904223; fil_a[l] = seedv;
        seedv = seedv * 32'd1664525 + 32'd1013904223; fil_b[l] = seedv;
      end
      write_vreg(10, fil_a);
      write_vreg(11, fil_b);
      dbg_slot = SW'(FB_SLOT);
      for (int r = 9; r < 16; r++) begin
        for (int l = 0; l < 32; l++) begin
          seedv = seedv * 32'd1664525 + 32'd1013904223;
          // v14/v15: small integers as binary32 (exact sums); the others random
          fb_v[r][l] = (r == 14) ? $shortrealtobits(shortreal'(l)) : (r == 15) ? $shortrealtobits(shortreal'(l + 1)) : seedv;
          g_v[l] = fb_v[r][l];
        end
        write_vreg(r, g_v);
      end
      fb_u40 = 32'd0;
      write_ureg(40, fb_u40);
      dbg_slot = SW'(SLOT);
    end

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
      // Issue and completion are driven / sampled on the falling edge: the DUT is
      // idle here (iss_ready), so it accepts at the one rising edge in between, and
      // the one-cycle done/trap pulse is stable at every falling edge.  (Sampling
      // them right after @(posedge clk) is a scheduling race: Verilator may show
      // the post-edge value and the pulse is missed.)
      @(negedge clk);
      dbg = 1'b0;
      ff_reg = ff0;
      @(negedge clk);
      if (DUAL != 0) begin
        // main vector + filler(s) of other slots; the order and the gaps between them are random
        // (DUAL=2: half of the gaps are zero, i.e. back to back)
        int seq [3];
        int ni;
        pl_kind.delete(); pl_cause.delete(); pl_info.delete();
        ni = (DUAL >= 2) ? 3 : 2;
        d_fbk = $urandom % NFB;
        seq[0] = 0; seq[1] = 1; seq[2] = 2;
        for (int i = ni - 1; i > 0; i--) begin
          int j, t;
          j = $urandom % (i + 1); t = seq[i]; seq[i] = seq[j]; seq[j] = t;
        end
        d_order = seq[0] * 100 + seq[1] * 10 + seq[2];
        d_mi = 0;
        for (int i = 0; i < ni; i++) if (seq[i] == 0) d_mi = i;
        ff_mi = d_mi;
        for (int i = 0; i < ni; i++) begin
          if (i > 0) begin
            d_gap = (DUAL >= 2 && ($urandom % 2) == 0) ? 0 : $urandom % 9;
            repeat (d_gap) @(negedge clk);
          end
          case (seq[i])
            0: issue_insn(w0, w1, ex, SLOT, nregs, mode_v);
            1: issue_insn(FILL_W0, 32'd0, 32'hffffffff, FA_SLOT, 128, 32'd0);
            default: issue_insn(FILLB_W0[d_fbk], 32'd0, 32'hffffffff, FB_SLOT, 128, 32'd0);
          endcase
        end
        waitc = 0;
        while (pl_kind.size() < ni && waitc < 20000) begin @(negedge clk); waitc++; end
        repeat (2) @(negedge clk);
        got_trap = 1'b0; got_done = 1'b0; d_bad = 1'b0;
        if (pl_kind.size() != ni) begin
          $display("FAIL %0s %0s: %0d completion pulses for %0d instructions (order %0d)", id_s, name_s, pl_kind.size(), ni, d_order);
          d_bad = 1'b1;
        end else begin
          for (int i = 0; i < ni; i++)
            if (i != d_mi && pl_kind[i] != 0) begin
              $display("FAIL %0s %0s: filler did not complete with done (order %0d)", id_s, name_s, d_order);
              d_bad = 1'b1;
            end
          got_trap = (pl_kind[d_mi] == 1); got_done = (pl_kind[d_mi] == 0);
          mc = pl_cause[d_mi]; mi = pl_info[d_mi];
        end
        if (d_bad) begin got_trap = 1'b0; got_done = 1'b0; end
        // keep the fill-in expectations: v10 += v11 (all lanes); filler B
        for (int l = 0; l < 32; l++) fil_a[l] = fil_a[l] + fil_b[l];
        if (DUAL >= 2) fb_apply(d_fbk);
      end else begin
        iss_w0 = w0; iss_w1 = w1; iss_exec = ex; iss_nregs = 8'(nregs); iss_mode = mode_v; iss_valid = 1'b1;
        @(negedge clk);
        iss_valid = 1'b0;
        got_trap = 1'b0; got_done = 1'b0; waitc = 0;
        while (!got_trap && !got_done && waitc < 20000) begin
          if (trap_valid) got_trap = 1'b1;
          if (done_valid) got_done = 1'b1;
          if (!got_trap && !got_done) begin
            @(negedge clk);
            waitc++;
          end
        end
        mc = trap_cause; mi = trap_info;
      end
      if ($test$plusargs("LAT") && got_done) $display("LAT %0s %0d %0d", name_s, waitc, $countones(ex));
      bad = 1'b0; nbad = 0;
      if (!got_trap && !got_done) begin
        $display("FAIL %0s %0s: timeout (no done/trap)", id_s, name_s);
        $display("     state: st=%0d cs=%0d pend=%b rsp_v=%b x1=%b x2=%b w_v=%b rd_valid=%b rd_ready=%b rst_n=%b dbg=%b iss_ready=%b",
                 dut.st, dut.cs, dut.pend, dut.rsp_v, dut.x1_v, dut.x2_v, dut.w_v,
                 v_rd_valid, rf_rd_ready, rst_n, dbg, iss_ready);
        bad = 1'b1;
      end else if (tr_f != 0) begin
        if (!got_trap || mc !== tr_cause[3:0] || mi !== tr_info) begin
          $display("FAIL %0s %0s: expected trap cause=%0d info=%08x, got trap=%0d cause=%0d info=%08x done=%0d",
                   id_s, name_s, tr_cause, tr_info, got_trap, mc, mi, got_done);
          bad = 1'b1;
        end
      end else if (got_trap) begin
        $display("FAIL %0s %0s: unexpected trap cause=%0d info=%08x", id_s, name_s, mc, mi);
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
      if (DUAL != 0) begin
        dbg_slot = SW'(FA_SLOT);
        read_vreg(10, g_v);
        for (int l = 0; l < 32; l++)
          if (g_v[l] !== fil_a[l]) begin
            if (nbad < 4 && shown < maxfail)
              $display("FAIL %0s %0s: filler v10 lane %0d expected %08x got %08x (order %0d gap %0d)", id_s, name_s, l, fil_a[l], g_v[l], d_order, d_gap);
            nbad++; bad = 1'b1;
          end
        if (DUAL >= 2) begin
          dbg_slot = SW'(FB_SLOT);
          for (int r = 9; r < 15; r++) begin
            if (r == 10 || r == 11) continue;
            read_vreg(r, g_v);
            for (int l = 0; l < 32; l++)
              if (g_v[l] !== fb_v[r][l]) begin
                if (nbad < 4 && shown < maxfail)
                  $display("FAIL %0s %0s: filler B v%0d lane %0d expected %08x got %08x (kind %0d order %0d)", id_s, name_s, r, l, fb_v[r][l], g_v[l], d_fbk, d_order);
                nbad++; bad = 1'b1;
              end
          end
          read_ureg(40, g_u);
          if (g_u !== fb_u40) begin
            if (nbad < 4 && shown < maxfail)
              $display("FAIL %0s %0s: filler B u40 expected %08x got %08x (kind %0d order %0d)", id_s, name_s, fb_u40, g_u, d_fbk, d_order);
            nbad++; bad = 1'b1;
          end
        end
        dbg_slot = SW'(SLOT);
        if (d_bad) bad = 1'b1;
      end
      if (bad) begin
        n_fail++; shown++; insn_fail[idx]++;
        if (shown <= maxfail) $display("     (w0=%08x w1=%08x exec=%08x mode=%08x ff0=%02x)", w0, w1, ex, mode_v, ff0);
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
      $display("NG_VALU_FP_VEC NL=%0d NFL=%0d VWP=%0d BG=%0d vectors pass=%0d fail=%0d of %0d; instructions pass=%0d fail=%0d of %0d",
               NL, NFL, VWP, BG, n_pass, n_fail, nvec, ok_insn, bad_insn, ninsn);
      if (DUAL != 0) $display("DUAL mode %0d: every vector ran next to %0d filler instruction(s)", DUAL, DUAL);
      $display("cov: BDEPTH=%0d back-half contexts >=2 cycles=%0d >=3 cycles=%0d queued_f32=%0d x2_stall=%0d gsafe_hold=%0d valu_err=%0d",
               BDEPTH, dut.sim_ctx2, dut.sim_ctx3, dut.sim_q_push, dut.sim_x2_stall, dut.sim_gs_blk, dut.sim_err);
      $display("cov: wr_stall_cycles=%0d rd_bank_conflict_cycles=%0d traps=%0d", cov_wr_stall, cov_rd_conf, cov_trap);
      if (n_fail == 0 && n_pass == nvec && dut.sim_err == 0) $display("NG_VALU_FP_VEC_ALL_OK");
      else $display("NG_VALU_FP_VEC_FAILED");
    end
    $finish;
  end

  // per-cycle unit trace (debug): -testplusarg VTRACE
  bit vtrace = 1'b0;
  initial vtrace = $test$plusargs("VTRACE");
  always @(negedge clk) if (vtrace && rst_n && !dbg)
    $display("VT %0t st=%0d cs=%0d pend=%b gs=%b chk=%b launch=%b xfer=%b x1=%b x2=%b w=%b/%0d/%0d q=%0d eng=%0d fpd=%b bn=%0d bh=%0d bt=%0d xk=%0d hd(all=%b bd=%0d wc=%0d) rdy=%b fw=%b done=%b trap=%b iss=%b/%b",
             $time, dut.st, dut.cs, dut.pend, dut.gsafe, dut.rd_chk, dut.launch, dut.xfer, dut.x1_v, dut.x2_v,
             dut.w_v, dut.w_k, dut.w_beat, dut.q_n, dut.c_eng, dut.fp_done, dut.bn, dut.bh, dut.bt, dut.xk,
             dut.k_all[dut.bh], dut.k_bd[dut.bh], dut.k_wc[dut.bh], dut.hd_rdy, dut.fw_on, done_valid, trap_valid,
             iss_valid, iss_ready);

  // global watchdog
  initial begin
    #2000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
