// ng_xlane_fsub.sv -- binary32 subtractor (a - b) for the derivative instructions
// (fddx/fddy fine/coarse); split into two combinational halves with a
// register between them in ng_xlane (the whole subtractor was one stage
// of ~11 ns, now A and B each fit the 10 ns clk_ts with margin).
//
// Bit-exact with the model's SoftFloat f32_sub (RISC-V NaN specialisation:
// every NaN result is the canonical 0x7fc00000, NV only for signalling NaN or
// inf - inf) wrapped in the model's FpScope conventions:
//   * rm 0..4 = rne, rtz, rdn, rup, rmm (rm > 4 is rejected by the caller);
//   * DAZ flushes subnormal inputs to signed zero without a flag;
//   * FTZ flushes a subnormal result to signed zero and raises UF | NX.
// flags[4:0] = {NV, DZ, OF, UF, NX}.
//
//   ng_xlane_fsub_a: classify, order, align, effective add / subtract -> fsub_t
//   ng_xlane_fsub_b: normalise, round, pack (and the special / zero results)
//   ng_xlane_fsub: a + b back to back (combinational; kept for reuse / unit tests)
package ng_xlane_fsub_pkg;
  typedef struct packed {
    logic        sp;       // NaN / infinity result (decided in stage A)
    logic        sp_nan;   // ... the canonical qNaN (else an infinity of sign sp_sgn)
    logic        sp_sgn;
    logic        nv;       // invalid (sNaN input or inf - inf)
    logic        sx;       // sign of the larger-magnitude operand (after the b negation)
    logic        sub;      // effective subtraction
    logic [7:0]  ex;       // exponent of the larger operand (subnormal -> 1)
    logic [27:0] ss;       // aligned sum / difference, 3 guard bits, sticky folded in
  } fsub_t;
endpackage

module ng_xlane_fsub_a
  import ng_xlane_fsub_pkg::*;
(
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic        daz,
  output fsub_t       o
);
  always_comb begin
    logic [31:0] ai, bi;
    logic        sa, sbe;
    logic [7:0]  ea, eb;
    logic [22:0] fa, fb;
    logic        a_nan, b_nan, a_snan, b_snan, a_inf, b_inf;
    logic        ge;
    logic        sx, sy, sub;
    logic [7:0]  ex, ey, dd;
    logic [23:0] mx, my;
    logic [26:0] xx, yy, ysh;
    logic        sticky;

    o = '0;
    ai = (daz && a[30:23] == 8'd0) ? {a[31], 31'd0} : a;
    bi = (daz && b[30:23] == 8'd0) ? {b[31], 31'd0} : b;
    sa  = ai[31];
    sbe = ~bi[31];
    ea = ai[30:23]; fa = ai[22:0];
    eb = bi[30:23]; fb = bi[22:0];
    a_nan  = (ea == 8'hff) && (fa != 23'd0);
    b_nan  = (eb == 8'hff) && (fb != 23'd0);
    a_snan = a_nan && !fa[22];
    b_snan = b_nan && !fb[22];
    a_inf  = (ea == 8'hff) && (fa == 23'd0);
    b_inf  = (eb == 8'hff) && (fb == 23'd0);

    ge = ({ea, fa} >= {eb, fb});
    sx = ge ? sa : sbe;
    sy = ge ? sbe : sa;
    ex = ge ? ((ea == 8'd0) ? 8'd1 : ea) : ((eb == 8'd0) ? 8'd1 : eb);
    ey = ge ? ((eb == 8'd0) ? 8'd1 : eb) : ((ea == 8'd0) ? 8'd1 : ea);
    mx = ge ? {ea != 8'd0, fa} : {eb != 8'd0, fb};
    my = ge ? {eb != 8'd0, fb} : {ea != 8'd0, fa};
    dd = ex - ey;
    sub = sx ^ sy;
    xx = {mx, 3'b000};
    yy = {my, 3'b000};
    if (dd >= 8'd27) begin
      ysh = {26'd0, (yy != 27'd0)};
    end else begin
      sticky = 1'b0;
      for (int i = 0; i < 27; i++) if (i < int'(dd) && yy[i]) sticky = 1'b1;
      ysh = (yy >> dd) | {26'd0, sticky};
    end
    o.ss  = sub ? ({1'b0, xx} - {1'b0, ysh}) : ({1'b0, xx} + {1'b0, ysh});
    o.sx  = sx;
    o.sub = sub;
    o.ex  = ex;

    if (a_nan || b_nan) begin
      o.sp = 1'b1; o.sp_nan = 1'b1; o.nv = a_snan | b_snan;
    end else if (a_inf || b_inf) begin
      o.sp = 1'b1;
      if (a_inf && b_inf && (sa != sbe)) begin
        o.sp_nan = 1'b1; o.nv = 1'b1;
      end else begin
        o.sp_sgn = a_inf ? sa : sbe;
      end
    end
  end
endmodule

module ng_xlane_fsub_b
  import ng_xlane_fsub_pkg::*;
(
  input  fsub_t       i,
  input  logic [2:0]  rm,
  input  logic        ftz,
  output logic [31:0] r,
  output logic [4:0]  flags
);
  localparam logic [31:0] QNAN = 32'h7fc00000;

  always_comb begin
    logic [27:0] ss;
    logic        sx, sub;
    logic [7:0]  ex;
    int          p;
    int          ls;
    logic [26:0] nn;
    int          eres;
    logic        lsb, g, rr, st, inexact, inc;
    logic [24:0] sig;
    int          efield;
    logic        sres;
    logic [22:0] frac;
    logic        ovf_inf;

    ss = i.ss; sx = i.sx; sub = i.sub; ex = i.ex;
    r = 32'd0;
    flags = 5'd0;

    // leading-one position of ss
    p = 0;
    for (int k = 0; k < 28; k++) if (ss[k]) p = k;

    if (p == 27) begin
      nn   = ss[27:1];
      nn[0] = ss[1] | ss[0];
      eres = int'(ex) + 1;
    end else begin
      ls = 26 - p;
      if (ls > int'(ex) - 1) ls = int'(ex) - 1;
      nn   = 27'(ss << ls);
      eres = int'(ex) - ls;
    end
    lsb = nn[3]; g = nn[2]; rr = nn[1]; st = nn[0];
    inexact = g | rr | st;
    case (rm)
      3'd0:    inc = g & (rr | st | lsb);
      3'd1:    inc = 1'b0;
      3'd2:    inc = sx & inexact;
      3'd3:    inc = ~sx & inexact;
      default: inc = g;
    endcase
    sig = {1'b0, nn[26:3]} + {24'd0, inc};
    if (sig[24]) begin
      frac   = 23'd0;
      efield = eres + 1;
    end else begin
      frac   = sig[22:0];
      efield = sig[23] ? eres : 0;
    end
    ovf_inf = (rm == 3'd0) || (rm == 3'd4) || (rm == 3'd2 && sx) || (rm == 3'd3 && !sx);

    if (i.sp) begin
      r = i.sp_nan ? QNAN : {i.sp_sgn, 8'hff, 23'd0};
      flags[4] = i.nv;
    end else if (ss == 28'd0) begin
      sres = sub ? (rm == 3'd2) : sx;
      r = {sres, 31'd0};
    end else if (efield >= 255) begin
      r = ovf_inf ? {sx, 8'hff, 23'd0} : {sx, 8'hfe, 23'h7fffff};
      flags[2] = 1'b1;   // OF
      flags[0] = 1'b1;   // NX
    end else begin
      r = {sx, 8'(efield), frac};
      flags[0] = inexact;
      if (ftz && efield == 0 && frac != 23'd0) begin
        r = {sx, 31'd0};
        flags[1] = 1'b1;  // UF
        flags[0] = 1'b1;  // NX
      end
    end
  end
endmodule

module ng_xlane_fsub
  import ng_xlane_fsub_pkg::*;
(
  input  logic [31:0] a,
  input  logic [31:0] b,
  input  logic [2:0]  rm,
  input  logic        daz,
  input  logic        ftz,
  output logic [31:0] r,
  output logic [4:0]  flags
);
  fsub_t t;
  ng_xlane_fsub_a u_a (.a(a), .b(b), .daz(daz), .o(t));
  ng_xlane_fsub_b u_b (.i(t), .rm(rm), .ftz(ftz), .r(r), .flags(flags));
endmodule
