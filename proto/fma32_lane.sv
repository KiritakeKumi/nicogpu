// fma32_lane.sv -- IEEE-754 binary32 fused multiply-add lane (P0 area/timing prototype)
//
//   r = a * b + c, single rounding, RNE only, subnormal inputs/outputs fully supported.
//   NaN results are canonical (0x7FC00000); invalid (inf*0, inf-inf) -> qNaN.
//   Exception flags are produced (NV/OF/UF/NX) but not used by the proto harness.
//
// Architecture (classic "addend pre-aligned against product" FMA, cf. Berkeley
// HardFloat MulAddRecFN / Vortex VX_fma_unit):
//   S1  unpack, exponent compare, addend alignment shift, DSP input regs
//   S2  DSP M register (24x24 -> 2x DSP48E2 with cascade)
//   S3  DSP P register (48-bit product)
//   S4  78-bit add / subtract (addend - product)
//   S5  absolute value + leading-zero count
//   S6  normalize shift + RNE round + pack  -> output register
// Latency = 6 cycles from input register to output register (plus input reg).
//
// Written from scratch for the nicogpu project; no external code copied.
// Alignment window layout (field bit k weighs 2^(k-46) * 2^(Eref-127)):
//   product P[47:0] at bits [47:0]; addend mantissa placed at [73:50] then
//   shifted right by sh = Eref+27-Ec; 3 extra bits below bit 0 (2 guard + sticky).

module fma32_lane (
  input  logic        clk,
  input  logic        in_valid,
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic [31:0] c,
  output logic        out_valid,
  output logic [31:0] r,
  output logic [4:0]  fflags        // {NV, DZ(0), OF, UF, NX}
);
  // ---------------------------------------------------------------- S0: input regs
  logic        v0;
  logic [31:0] a0, b0, c0;
  always_ff @(posedge clk) begin
    v0 <= in_valid;
    a0 <= a; b0 <= b; c0 <= c;
  end

  // ---------------------------------------------------------------- S1: unpack / align
  wire        sa = a0[31], sb = b0[31], sc = c0[31];
  wire [7:0]  ea = a0[30:23], eb = b0[30:23], ec = c0[30:23];
  wire [22:0] fa = a0[22:0],  fb = b0[22:0],  fc = c0[22:0];

  wire a_zero = (ea == 0) && (fa == 0);
  wire b_zero = (eb == 0) && (fb == 0);
  wire c_zero = (ec == 0) && (fc == 0);
  wire a_inf  = (ea == 8'hFF) && (fa == 0);
  wire b_inf  = (eb == 8'hFF) && (fb == 0);
  wire c_inf  = (ec == 8'hFF) && (fc == 0);
  wire a_nan  = (ea == 8'hFF) && (fa != 0);
  wire b_nan  = (eb == 8'hFF) && (fb != 0);
  wire c_nan  = (ec == 8'hFF) && (fc != 0);
  wire a_snan = a_nan && !fa[22];
  wire b_snan = b_nan && !fb[22];
  wire c_snan = c_nan && !fc[22];

  wire [23:0] ma = {ea != 0, fa};
  wire [23:0] mb = {eb != 0, fb};
  wire [23:0] mc = {ec != 0, fc};
  wire [9:0]  Ea = (ea == 0) ? 10'd1 : {2'b0, ea};
  wire [9:0]  Eb = (eb == 0) ? 10'd1 : {2'b0, eb};
  wire [9:0]  Ec = (ec == 0) ? 10'd1 : {2'b0, ec};

  wire        sp      = sa ^ sb;
  wire        prod_z  = a_zero | b_zero;
  // product exponent (biased); range -125 .. 381 -> signed 11 bit
  wire signed [11:0] Ep  = $signed({2'b0, Ea}) + $signed({2'b0, Eb}) - 12'sd127;
  wire signed [11:0] Ecs = $signed({2'b0, Ec});
  // Eref = max(Ep, Ec-27): reference exponent of the window
  wire signed [11:0] Ec27 = Ecs - 12'sd27;
  wire        c_dom  = (Ec27 > Ep) || prod_z;   // addend dominates: product -> sticky only
  wire signed [11:0] Eref = c_dom ? Ec27 : Ep;
  // shift of addend = Eref + 27 - Ec  (>= 0)
  wire signed [11:0] sh_s = Eref + 12'sd27 - Ecs;
  wire        sh_big = (sh_s > 12'sd77);
  wire [6:0]  sh     = sh_big ? 7'd77 : sh_s[6:0];

  // addend in 77-bit field [76:0]: bits [76:53] = window bits [73:50];
  // bits [2:0] of the field = window bits [-1:-3]; sticky collects the rest.
  wire [100:0] c_ext  = {mc, 77'b0};
  wire [100:0] c_shr  = c_ext >> sh;
  wire [76:0]  c_al   = {c_shr[100:25], |c_shr[24:0]};   // LSB = sticky (all bits shifted past -3)

  wire        eff_sub = sp ^ sc;

  // specials
  wire nan_in   = a_nan | b_nan | c_nan;
  wire inv_mul  = (a_inf & b_zero) | (b_inf & a_zero);
  wire prod_inf = (a_inf | b_inf);
  wire inv_add  = prod_inf & c_inf & eff_sub & !inv_mul;
  wire snan_in  = a_snan | b_snan | c_snan;

  logic        v1, sp1, sc1, eff_sub1, c_dom1, prod_z1;
  logic signed [11:0] Eref1;
  logic [76:0] c_al1;
  logic        s_nan1, s_inv1, s_inf1, s_infsign1;
  logic [23:0] ma1, mb1;
  always_ff @(posedge clk) begin
    v1       <= v0;
    sp1      <= sp;  sc1 <= sc; eff_sub1 <= eff_sub;
    c_dom1   <= c_dom; prod_z1 <= prod_z;
    Eref1    <= Eref;
    c_al1    <= c_zero ? 77'b0 : c_al;
    s_nan1   <= nan_in | inv_mul | inv_add;
    s_inv1   <= snan_in | inv_mul | inv_add;
    s_inf1   <= prod_inf | c_inf;
    s_infsign1 <= prod_inf ? sp : sc;
    ma1      <= ma;
    mb1      <= mb;
  end

  // ---------------------------------------------------------------- S2/S3: DSP multiply
  (* use_dsp = "yes" *) logic [47:0] m2, p3;
  always_ff @(posedge clk) begin
    m2 <= ma1 * mb1;    // M register
    p3 <= m2;           // P register
  end

  // side-band pipeline for S2,S3
  logic        v2, v3, sp2, sp3, sc2, sc3, es2, es3, cd2, cd3, pz2, pz3;
  logic signed [11:0] E2, E3;
  logic [76:0] ca2, ca3;
  logic [3:0]  spc2, spc3;
  always_ff @(posedge clk) begin
    {v2, sp2, sc2, es2, cd2, pz2, E2, ca2} <= {v1, sp1, sc1, eff_sub1, c_dom1, prod_z1, Eref1, c_al1};
    {v3, sp3, sc3, es3, cd3, pz3, E3, ca3} <= {v2, sp2, sc2, es2, cd2, pz2, E2, ca2};
    spc2 <= {s_nan1, s_inv1, s_inf1, s_infsign1};
    spc3 <= spc2;
  end

  // ---------------------------------------------------------------- S4: add
  // product into 77-bit field: window bits [47:0] -> field bits [50:3].
  // If the addend dominates, the product lies strictly below the guard bits:
  // it only needs to act as sticky -> squash it to the field LSB.
  wire [76:0] p_al = pz3 ? 77'b0 :
                     cd3 ? {76'b0, |p3} :
                           {26'b0, p3, 3'b0};
  // 78-bit two's complement: sum = c_al +/- p_al
  wire [77:0] sum4 = es3 ? ({1'b0, ca3} - {1'b0, p_al}) : ({1'b0, ca3} + {1'b0, p_al});

  logic        v4, sp4, sc4, es4;
  logic [77:0] s4;
  logic signed [11:0] E4;
  logic [3:0]  spc4;
  always_ff @(posedge clk) begin
    v4 <= v3; sp4 <= sp3; sc4 <= sc3; es4 <= es3;
    s4 <= sum4; E4 <= E3; spc4 <= spc3;
  end

  // ---------------------------------------------------------------- S5: abs + LZC
  wire        neg5  = es4 & s4[77];
  wire [77:0] mag5  = neg5 ? (~s4 + 78'd1) : s4;     // bit 77 is the carry bit (add case)
  // sign of a non-zero result: addend sign, flipped if the subtraction went negative
  wire        rs5   = sc4 ^ neg5;
  wire        zs5   = sp4 & sc4;                     // sign of exact zero under RNE

  function automatic [6:0] lzc78(input [77:0] x);
    lzc78 = 7'd78;
    for (int i = 0; i < 78; i++) if (x[i]) lzc78 = 7'(77 - i);
  endfunction
  wire [6:0]  lz5 = lzc78(mag5);

  logic        v5, rs5r, zs5r;
  logic [77:0] mag5r;
  logic [6:0]  lz5r;
  logic signed [11:0] E5;
  logic [3:0]  spc5;
  always_ff @(posedge clk) begin
    v5 <= v4; rs5r <= rs5; zs5r <= zs5;
    mag5r <= mag5; lz5r <= lz5; E5 <= E4; spc5 <= spc4;
  end

  // ---------------------------------------------------------------- S6: normalize, round, pack
  // field bit j (0..77) weighs Eres = Eref + j - 49 when it is the MSB.
  // normalizing MSB to bit 77: Eres = Eref + 28 - L;  keep Eres >= 1  -> L <= Eref + 27.
  wire signed [11:0] Lmax = E5 + 12'sd27;            // always >= 1 (Eref >= -26)
  wire        clampL = ($signed({5'b0, lz5r}) > Lmax);
  wire [6:0]  L      = clampL ? Lmax[6:0] : lz5r;
  wire [77:0] nrm    = mag5r << L;
  wire signed [11:0] Eres = E5 + 12'sd28 - $signed({5'b0, L});
  wire [23:0] man    = nrm[77:54];
  wire        g      = nrm[53];
  wire        st     = |nrm[52:0];
  wire        lsb    = nrm[54];
  wire        inc    = g & (st | lsb);
  wire        is_zero = (mag5r == 78'b0);
  wire        ovf    = (Eres >= 12'sd255);
  wire [7:0]  efield = man[23] ? Eres[7:0] : 8'd0;   // subnormal when hidden bit not set
  wire [30:0] pk_res = {efield, man[22:0]} + {30'b0, inc};
  wire        ovf_r  = ovf | (pk_res[30:23] == 8'hFF);
  wire        inexact = g | st;
  wire        tiny   = !man[23] || (Eres < 1);

  logic [31:0] r6;
  logic [4:0]  ff6;
  logic        v6;
  always_ff @(posedge clk) begin
    v6 <= v5;
    if (spc5[3]) begin
      r6 <= 32'h7FC00000; ff6 <= {spc5[2], 4'b0};
    end else if (spc5[1]) begin
      r6 <= {spc5[0], 8'hFF, 23'b0}; ff6 <= 5'b0;
    end else if (is_zero) begin
      r6 <= {zs5r, 31'b0}; ff6 <= 5'b0;
    end else if (ovf_r) begin
      r6 <= {rs5r, 8'hFF, 23'b0}; ff6 <= 5'b00101;
    end else begin
      r6 <= {rs5r, pk_res}; ff6 <= {2'b0, 1'b0, tiny & inexact, inexact};
    end
  end

  assign out_valid = v6;
  assign r         = r6;
  assign fflags    = ff6;
endmodule
