// ng_valu_lane.sv -- combinational integer ALU of one VALU lane.
//
// Pure function of the (already routed) operands; semantics are those of
//  `prod` is the registered 64-bit product
// from ng_valu_mul.  For the swapped forms (subr, sllr, srlr, srar) the caller
// exchanges a_lo/b_lo before this module.
module ng_valu_lane
  import ng_valu_pkg::*;
  import ng_bitops_pkg::clz32, ng_bitops_pkg::ctz32, ng_bitops_pkg::cpop32, ng_bitops_pkg::brev32,
         ng_bitops_pkg::bfm_mask;
(
  input  logic [6:0]  uop,
  input  logic        cmps,      // cmp: signed
  input  logic [2:0]  cmpfn,
  input  logic [31:0] a_lo,
  input  logic [31:0] a_hi,
  input  logic [31:0] b_lo,
  input  logic [31:0] b_hi,
  input  logic [31:0] c_lo,
  input  logic [31:0] c_hi,
  input  logic [31:0] d_old,
  input  logic [63:0] prod,
  input  logic [4:0]  lane,      // lane index within the warp (0..31)
  output logic [31:0] r_lo,
  output logic [31:0] r_hi,
  output logic        cmp_bit,
  output logic        lane_we    // per-lane write enable (cndmask); 1 otherwise
);

  // ---- helpers (all on 32-bit values) ------------------------------------
  function automatic logic [31:0] f_sext(input logic [31:0] v, input int bits);
    logic [31:0] r;
    r = v;
    for (int i = 0; i < 32; i++) if (i >= bits) r[i] = v[bits-1];
    return r;
  endfunction
  function automatic logic [31:0] f_dp4a(input logic [31:0] a, input logic [31:0] b, input logic [31:0] c,
                                         input logic sgn);
    logic [31:0] s;
    logic signed [16:0] pp;
    logic signed [8:0] x, y;
    s = c;
    for (int k = 0; k < 4; k++) begin
      x  = $signed({sgn & a[8*k+7], a[8*k +: 8]});
      y  = $signed({sgn & b[8*k+7], b[8*k +: 8]});
      pp = x * y;
      s  = s + 32'(pp);
    end
    return s;
  endfunction

  logic        lt_s, lt_u, eq;
  logic        lt64_s, lt64_u, eq64;
  logic [63:0] a64, b64;
  logic [32:0] sum_s, dif_s, sum_u;
  logic [31:0] ubit_mask;
  logic [4:0]  sh;
  logic [5:0]  w6;
  logic [31:0] bf;
  logic [63:0] cat;
  logic [31:0] med_lo, med_hi;

  always_comb begin
    a64    = {a_hi, a_lo};
    b64    = {b_hi, b_lo};
    sh     = b_lo[4:0];
    lt_s   = $signed(a_lo) < $signed(b_lo);
    lt_u   = a_lo < b_lo;
    eq     = a_lo == b_lo;
    lt64_s = $signed(a64) < $signed(b64);
    lt64_u = a64 < b64;
    eq64   = a64 == b64;
    sum_s  = {a_lo[31], a_lo} + {b_lo[31], b_lo};
    dif_s  = {a_lo[31], a_lo} - {b_lo[31], b_lo};
    sum_u  = {1'b0, a_lo} + {1'b0, b_lo};
    ubit_mask = (32'd1 << lane) - 32'd1;
    cat    = {a_lo, b_lo};
    w6     = c_lo[5:0];
    bf     = 32'd0;
    med_lo = 32'd0;
    med_hi = 32'd0;

    r_lo    = 32'd0;
    r_hi    = 32'd0;
    cmp_bit = 1'b0;
    lane_we = 1'b1;

    case (uop)
      U_ADD:  r_lo = a_lo + b_lo;
      U_SUB:  r_lo = a_lo - b_lo;
      U_SLL:  r_lo = a_lo << sh;
      U_SRL:  r_lo = a_lo >> sh;
      U_SRA:  r_lo = $signed(a_lo) >>> sh;
      U_SLT:  r_lo = {31'd0, lt_s};
      U_SLTU: r_lo = {31'd0, lt_u};
      U_XOR:  r_lo = a_lo ^ b_lo;
      U_OR:   r_lo = a_lo | b_lo;
      U_AND:  r_lo = a_lo & b_lo;
      U_ANDN: r_lo = a_lo & ~b_lo;
      U_ORN:  r_lo = a_lo | ~b_lo;
      U_XNOR: r_lo = ~(a_lo ^ b_lo);
      U_MIN:  r_lo = lt_s ? a_lo : b_lo;
      U_MAX:  r_lo = (!lt_s && !eq) ? a_lo : b_lo;
      U_MINU: r_lo = lt_u ? a_lo : b_lo;
      U_MAXU: r_lo = (!lt_u && !eq) ? a_lo : b_lo;
      U_MUL:    r_lo = prod[31:0];
      U_MULH, U_MULHSU, U_MULHU: r_lo = prod[63:32];
      U_MAC:    r_lo = d_old + prod[31:0];
      U_ROL:    r_lo = (a_lo << sh) | (a_lo >> (5'(-sh)));
      U_ROR:    r_lo = (a_lo >> sh) | (a_lo << (5'(-sh)));
      U_CZEQ:   r_lo = (b_lo == 32'd0) ? 32'd0 : a_lo;
      U_CZNE:   r_lo = (b_lo != 32'd0) ? 32'd0 : a_lo;
      U_ADDSATI: r_lo = (sum_s[32] != sum_s[31]) ? (a_lo[31] ? 32'h80000000 : 32'h7fffffff) : sum_s[31:0];
      U_ADDSATU: r_lo = sum_u[32] ? 32'hffffffff : sum_u[31:0];
      U_SUBSATI: r_lo = (dif_s[32] != dif_s[31]) ? (a_lo[31] ? 32'h80000000 : 32'h7fffffff) : dif_s[31:0];
      U_SUBSATU: r_lo = lt_u ? 32'd0 : (a_lo - b_lo);
      U_BFM:    r_lo = bfm_mask(a_lo, b_lo);
      U_MULWU, U_MULW: {r_hi, r_lo} = prod;
      U_ADD64:  {r_hi, r_lo} = a64 + b64;
      U_SUB64:  {r_hi, r_lo} = a64 - b64;
      U_ADDSX64: {r_hi, r_lo} = a64 + {{32{b_lo[31]}}, b_lo};
      U_ADDZX64: {r_hi, r_lo} = a64 + {32'd0, b_lo};
      U_SLL64:  {r_hi, r_lo} = a64 << b_lo[5:0];
      U_SRL64:  {r_hi, r_lo} = a64 >> b_lo[5:0];
      U_SRA64:  {r_hi, r_lo} = $signed(a64) >>> b_lo[5:0];
      U_MBCNT:  r_lo = a_lo + cpop32(b_lo & ubit_mask);
      U_CNDMASK: begin r_lo = a_lo; lane_we = b_lo[lane]; end
      U_MOV:    r_lo = a_lo;
      U_NOT:    r_lo = ~a_lo;
      U_CLZ:    r_lo = clz32(a_lo);
      U_CTZ:    r_lo = ctz32(a_lo);
      U_CPOP:   r_lo = cpop32(a_lo);
      U_BREV:   r_lo = brev32(a_lo);
      U_REV8:   r_lo = {a_lo[7:0], a_lo[15:8], a_lo[23:16], a_lo[31:24]};
      U_SEXTB:  r_lo = {{24{a_lo[7]}}, a_lo[7:0]};
      U_SEXTH:  r_lo = {{16{a_lo[15]}}, a_lo[15:0]};
      U_ZEXTB:  r_lo = {24'd0, a_lo[7:0]};
      U_ZEXTH:  r_lo = {16'd0, a_lo[15:0]};
      U_LANEID: r_lo = {27'd0, lane};
      U_NEG:    r_lo = 32'd0 - a_lo;
      U_ABS:    r_lo = a_lo[31] ? (32'd0 - a_lo) : a_lo;
      U_MOV64:  {r_hi, r_lo} = a64;
      U_CMP: begin
        case (cmpfn)
          CMP_EQ: cmp_bit = eq;
          CMP_NE: cmp_bit = !eq;
          CMP_LT: cmp_bit = cmps ? lt_s : lt_u;
          CMP_GE: cmp_bit = cmps ? !lt_s : !lt_u;
          // gt (vi forms): a > b  <=>  b < a
          default: cmp_bit = cmps ? ($signed(b_lo) < $signed(a_lo)) : (b_lo < a_lo);
        endcase
      end
      U_CMP64: begin
        case (cmpfn)
          CMP_EQ: cmp_bit = eq64;
          CMP_NE: cmp_bit = !eq64;
          CMP_LT: cmp_bit = cmps ? lt64_s : lt64_u;
          default: cmp_bit = cmps ? !lt64_s : !lt64_u;  // ge
        endcase
      end
      U_SEL:    r_lo = (c_lo != 32'd0) ? a_lo : b_lo;
      U_BFEU:   r_lo = (a_lo >> sh) & bfm_mask(c_lo, 32'd0);
      U_BFEI: begin
        bf = a_lo >> sh;
        if (w6 == 6'd0) r_lo = 32'd0;
        else if (w6 >= 6'd32) r_lo = bf;
        else r_lo = f_sext(bf, int'(w6));
      end
      U_BFI:    r_lo = (a_lo & b_lo) | (~a_lo & c_lo);
      U_PERM: begin
        for (int k = 0; k < 4; k++) begin
          logic [7:0] s;
          s = c_lo[8*k +: 8];
          if (s < 8'd8)        r_lo[8*k +: 8] = cat[8*int'(s[2:0]) +: 8];
          else if (s == 8'h0d) r_lo[8*k +: 8] = 8'hff;
          else                 r_lo[8*k +: 8] = 8'h00;
        end
      end
      U_MAD:    r_lo = prod[31:0] + c_lo;
      U_MAD64:  {r_hi, r_lo} = prod + {c_hi, c_lo};
      U_ALIGNBIT: r_lo = 32'((cat >> c_lo[4:0]));
      U_MED3I: begin
        // a = x, b = y, c = z: median = clamp(z, min(x,y), max(x,y))
        med_lo = ($signed(a_lo) < $signed(b_lo)) ? a_lo : b_lo;
        med_hi = ($signed(a_lo) < $signed(b_lo)) ? b_lo : a_lo;
        r_lo = ($signed(c_lo) < $signed(med_lo)) ? med_lo : ($signed(c_lo) > $signed(med_hi)) ? med_hi : c_lo;
      end
      U_MED3U: begin
        med_lo = (a_lo < b_lo) ? a_lo : b_lo;
        med_hi = (a_lo < b_lo) ? b_lo : a_lo;
        r_lo = (c_lo < med_lo) ? med_lo : (c_lo > med_hi) ? med_hi : c_lo;
      end
      U_DP4AI:  r_lo = f_dp4a(a_lo, b_lo, c_lo, 1'b1);
      U_DP4AU:  r_lo = f_dp4a(a_lo, b_lo, c_lo, 1'b0);
      U_SEL64:  {r_hi, r_lo} = (c_lo != 32'd0) ? a64 : b64;
      default: ;
    endcase
  end
endmodule
