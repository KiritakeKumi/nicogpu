// ng_lsu_fatomic.sv -- FP32 atomic ALU (fadd / fmin / fmax), 3-stage pipelined.
//
// The L2 AMO ALU (ng_l2_amo_alu) has no FP32 operations, so the LSU
// executes atom.*.fadd.f32 / fmin.f32 / fmax.f32 itself (shared memory: inside
// the RMW; global memory: as a compare-and-swap loop against L2).
// Semantics =:
//   * fadd: IEEE binary32 add, round-to-nearest-even, subnormals kept (no FTZ/
//     DAZ), any NaN operand or inf-inf -> canonical qNaN 0x7fc00000; no flags;
//   * fmin/fmax: minimumNumber / maximumNumber (RISC-V): a single NaN is
//     ignored, both NaN -> 0x7fc00000, -0 < +0.
//
// Timing: the former single-cycle behavioural adder was ~11 ns.  Now the datapath has three
// register stages (A: special cases + order + align, B: add/sub + leading one, C: normalise shift +
// round decision); `r` is combinational from the last stage (round increment + pack).  The inputs
// must be held for 3 clock edges before `r` is valid (the LSU waits; no valid tracking here).
module ng_lsu_fatomic (
  input  logic        clk,
  input  logic [1:0]  op,    // 0 fadd, 1 fmin, 2 fmax
  input  logic [31:0] a,     // old memory value
  input  logic [31:0] b,     // operand
  output logic [31:0] r
);
  import ng_noc_pkg::F32_QNAN, ng_noc_pkg::f32_isnan, ng_noc_pkg::f32_minmax;

  // ---------------------------------------------------------------- stage A (comb)
  logic        a_sp;          // result already known (special / min / max)
  logic [31:0] a_sres;
  logic        a_same, a_sx, a_sb;
  logic [7:0]  a_exb;
  logic [63:0] a_Mb, a_Ms;
  always_comb begin
    logic        sx, sy;
    logic [7:0]  ex, ey, exb, exs;
    logic [23:0] mx, my, mb, ms;
    logic [63:0] Mb, Ms, lostm;
    logic        swap_;
    int          d;
    a_sp = 1'b0; a_sres = '0; a_same = 1'b0; a_sx = 1'b0; a_sb = 1'b0; a_exb = '0; a_Mb = '0; a_Ms = '0;
    sx = a[31]; sy = b[31];
    ex = a[30:23]; ey = b[30:23];
    mx = '0; my = '0; mb = '0; ms = '0; exb = '0; exs = '0; Mb = '0; Ms = '0; lostm = '0;
    swap_ = 1'b0; d = 0;
    if (op != 2'd0) begin
      a_sp = 1'b1; a_sres = f32_minmax(a, b, op == 2'd2);
    end else if (f32_isnan(a) || f32_isnan(b)) begin
      a_sp = 1'b1; a_sres = F32_QNAN;
    end else if (ex == 8'hff && ey == 8'hff) begin
      a_sp = 1'b1; a_sres = (sx == sy) ? a : F32_QNAN;
    end else if (ex == 8'hff) begin
      a_sp = 1'b1; a_sres = a;
    end else if (ey == 8'hff) begin
      a_sp = 1'b1; a_sres = b;
    end else begin
      mx = {(ex != 8'd0), a[22:0]};
      my = {(ey != 8'd0), b[22:0]};
      if (ex == 8'd0) ex = 8'd1;
      if (ey == 8'd0) ey = 8'd1;
      swap_ = ({ey, my} > {ex, mx});
      if (swap_) begin exb = ey; mb = my; a_sb = sy; exs = ex; ms = mx; end
      else       begin exb = ex; mb = mx; a_sb = sx; exs = ey; ms = my; end
      d  = int'(exb) - int'(exs);
      Mb = {8'd0, mb, 32'd0};
      Ms = {8'd0, ms, 32'd0};
      if (d >= 60) begin
        Ms = (Ms != 64'd0) ? 64'd1 : 64'd0;
      end else if (d > 0) begin
        lostm = Ms & ((64'd1 << d) - 64'd1);
        Ms = (Ms >> d) | ((lostm != 64'd0) ? 64'd1 : 64'd0);
      end
      a_same = (sx == sy);
      a_sx   = sx;
      a_exb  = exb;
      a_Mb   = Mb;
      a_Ms   = Ms;
    end
  end

  logic        r1_sp;
  logic [31:0] r1_sres;
  logic        r1_same, r1_sx, r1_sb;
  logic [7:0]  r1_exb;
  logic [63:0] r1_Mb, r1_Ms;
  always_ff @(posedge clk) begin
    r1_sp <= a_sp; r1_sres <= a_sres; r1_same <= a_same; r1_sx <= a_sx; r1_sb <= a_sb;
    r1_exb <= a_exb; r1_Mb <= a_Mb; r1_Ms <= a_Ms;
  end

  // ---------------------------------------------------------------- stage B (comb): add / sub, leading one
  logic        b_sp;
  logic [31:0] b_sres;
  logic [63:0] b_S;
  logic [5:0]  b_p;
  always_comb begin
    logic [63:0] S;
    int          p;
    S = r1_same ? (r1_Mb + r1_Ms) : (r1_Mb - r1_Ms);
    p = 0;
    for (int i = 0; i < 64; i++) if (S[i]) p = i;
    b_sp = r1_sp; b_sres = r1_sres; b_S = S; b_p = 6'(p);
    if (!r1_sp && S == 64'd0) begin
      b_sp = 1'b1;
      b_sres = {(r1_same ? r1_sx : 1'b0), 31'd0};
    end
  end

  logic        r2_sp;
  logic [31:0] r2_sres;
  logic [63:0] r2_S;
  logic [5:0]  r2_p;
  logic        r2_sb;
  logic [7:0]  r2_exb;
  always_ff @(posedge clk) begin
    r2_sp <= b_sp; r2_sres <= b_sres; r2_S <= b_S; r2_p <= b_p; r2_sb <= r1_sb; r2_exb <= r1_exb;
  end

  // ---------------------------------------------------------------- stage C (comb): normalise shift, round decision
  logic        c_sp, c_norm, c_rup, c_rs;
  logic [31:0] c_sres;
  logic [9:0]  c_E;
  logic [23:0] c_M24;
  always_comb begin
    int          p, E, sh;
    logic [63:0] M24, rem, half;
    p = int'(r2_p);
    E = p + int'(r2_exb) - 55;
    M24 = '0; rem = '0; half = '0; sh = 0;
    c_sp = r2_sp; c_sres = r2_sres; c_rs = r2_sb;
    c_norm = (E >= 1); c_rup = 1'b0;
    if (E >= 1) begin
      sh   = p - 23;
      M24  = r2_S >> sh;
      rem  = r2_S & ((64'd1 << sh) - 64'd1);
      half = 64'd1 << (sh - 1);
      c_rup = (rem > half) || (rem == half && M24[0]);
    end else begin
      sh  = 33 - int'(r2_exb);
      M24 = r2_S >> sh;
    end
    c_E = 10'(E);
    c_M24 = M24[23:0];
  end

  logic        r3_sp, r3_norm, r3_rup, r3_rs;
  logic [31:0] r3_sres;
  logic [9:0]  r3_E;
  logic [23:0] r3_M24;
  always_ff @(posedge clk) begin
    r3_sp <= c_sp; r3_norm <= c_norm; r3_rup <= c_rup; r3_rs <= c_rs; r3_sres <= c_sres;
    r3_E <= c_E; r3_M24 <= c_M24;
  end

  // ---------------------------------------------------------------- stage D (comb): round increment + pack
  always_comb begin
    logic [24:0] m;
    logic [9:0]  e;
    m = {1'b0, r3_M24} + {24'd0, r3_norm & r3_rup};
    e = r3_E;
    if (r3_sp) begin
      r = r3_sres;
    end else if (r3_norm) begin
      if (m == 25'h1000000) begin m = 25'h0800000; e = e + 10'd1; end
      if ($signed(e) >= 255) r = {r3_rs, 8'hff, 23'd0};
      else                   r = {r3_rs, e[7:0], m[22:0]};
    end else begin
      r = {r3_rs, 8'd0, r3_M24[22:0]};
    end
  end
endmodule
