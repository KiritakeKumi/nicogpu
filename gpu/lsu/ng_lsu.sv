// ng_lsu.sv -- SP load/store unit core: decode, address generation,
// lane coalescing, MMU client, shared-memory rounds, atomics, writeback.
//
// Executes the 301 `unit: lsu` instructions (ld/st/atom x {global, ubase, buf,
// shared, scratch}, uld*, cache.inv.l1 / cache.inv.l2 / cache.wb.l2) with the
// semantics of  One instruction in flight
// (the dispatcher owns hazards; results are written to the register file just
// before `done`, so waitcnt semantics hold by construction).
//
// Flow of one instruction
//   CHK   decode + the model's pre-access trap sequence (cpol, operand
//         alignment, address-register range, data-register range)
//   URD   uniform operand reads (upair / udesc4 / uld.x register)
//   VRD   vector operand reads (address regs, store/atomic data regs)
//   ADDR  per-lane address, byte-valid mask (buf/scratch bounds rules),
//         shared-window bounds (first faulting lane wins, SharedBounds trap)
//   global-path (everything except shared):
//     pass 0 PROBE: coalesce lanes by 128 B line, translate every line
//       (MMU), find the first fault in model order (lane, 4 B segment);
//     a fault -> precise MemFault trap with NO memory effect;
//     pass 1 COMMIT: L1 line read / write-through store per line group (the translation of
//       each group comes from the pass-0 memo, see "translation memo"; atomics translate again
//       per lane, in ascending lane order).
//   shared-path: bank rounds through ng_lsu_shmem (conflict-serialised); one round serves a
//     prefix of ONE group of SH_LPB lanes (default 8), groups in ascending order.
//   WB    vector regs (EXEC lanes) / uniform regs, then done (or trap).
//
// Interfaces: regfile read port 0 / write port, uniform read / write port,
// ng_mmu translation client, ng_l1d request port (line granular), waitcnt
// accounting (+1 at accept, -1 at done/trap).
//
// Scratch (ERRATA v0.1.6): lane-private bytes are dword-interleaved -- byte o of lane l is at
//   R + (o / 4) * 128 + l * 4 + o % 4,   R = ScratchBase + GlobalWarpSlot * 32 * ScratchBytes,
// so a lane's access is contiguous only inside one dword.  The access is run as `ns` slices (one per dword column
// the widest active lane touches: ns = max ceil((o % 4 + n) / 4), 1 for an aligned b32 / b8 -- the common case
// costs nothing extra).  Slice s gives every lane the bytes j in [4s - o%4, 4s - o%4 + 4) of its access and the
// "byte 0" address  R + (o & ~3) * 32 + o % 4 + 124 * s + 4 * l, so the unchanged two-piece line engine below
// (lane byte j <-> line byte (ua + j) mod 128, piece 1 in the next line) places them in that column's line.  The
// probe pass walks slices 0..ns-1, then (after E6) the commit pass walks them again in the same order (same
// group order -> the translation memo stays valid); every slice re-walks S_ADDR (~34 cycles) when ns > 1.
// Fault key {lane, segment} is order-independent, so the first fault in model order still wins; the scratch fault
// info is the VA of the first byte of the faulting run (the model's per-dword-run access), not of the segment.
//
// non-pipelined, ~hundreds of cycles per
// instruction (ISA only says async:<counter>); atomics are one lane at a time;
// FP32 atomics on global memory = CAS loop at L2, line-crossing atomics are a
// non-atomic LSU-side read-modify-write; MMU is translated once per line group
// (probe; the commit pass reads the translation memo).
// Area/timing: the lane-wide data paths work on LPB lanes per beat --
// SH_LPB for shared memory (32 = the former one-round behaviour), L1_LPB for the global-path load
// scatter / store merge (ng_lsu_scat / ng_lsu_smerge, empty beats skipped); the result bytes (rb)
// are written group-wise through one write bus; large data arrays have no reset (cleared in S_CHK).
// Atomics: integer ops in ng_l2_amo_alu #(.EN_FP(0)), FP32 ops in the pipelined ng_lsu_fatomic.
// The state machine is ng_lsu_core (below), shared with ng_lsu_kn (LSU_K >= 2); ng_lsu is its K = 1 wrapper.

// ng_lsu_scat -- load scatter: per lane the 16 B window of a 128 B line starting at byte `off`
// (circular: bytes past the end of the line wrap to its start = piece 1 of a line-crossing lane).
module ng_lsu_scat
  import ng_lsu_pkg::LINE_BITS;
#(
  parameter int LPB = 8
) (
  input  logic [LINE_BITS-1:0] line,
  input  logic [6:0]           off [LPB],
  output logic [127:0]         dat [LPB]
);
  always_comb begin
    logic [2047:0] dbl;
    logic [2047:0] sh;
    dbl = {line, line};
    for (int i = 0; i < LPB; i++) begin
      sh = dbl >> (8 * int'(off[i]));
      dat[i] = sh[127:0];
    end
  end
endmodule

// ng_lsu_smerge -- store merge of one beat: per lane the 16 B payload is rotated to its line offset
// (same circular convention as the scatter) and the lanes are overlaid in ascending lane order
// (a later lane overwrites an earlier one byte-wise).  m_be / m_d = bytes written by the beat.
module ng_lsu_smerge
  import ng_lsu_pkg::LINE_BITS;
#(
  parameter int LPB = 8
) (
  input  logic [127:0]  pay [LPB],
  input  logic [6:0]    off [LPB],
  input  logic [15:0]   be  [LPB],
  output logic [127:0]  m_be,
  output logic [LINE_BITS-1:0] m_d
);
  always_comb begin
    logic [2047:0] dd;
    logic [255:0]  bb;
    logic [127:0]  be_l;
    logic [LINE_BITS-1:0] d_l;
    m_be = '0;
    m_d  = '0;
    for (int i = 0; i < LPB; i++) begin
      // rotate left by `off` bytes inside 128 B == shift the doubled image right by 128 - off
      dd   = {1024'(pay[i]), 1024'(pay[i])} >> (1024 - 8 * int'(off[i]));
      d_l  = dd[LINE_BITS-1:0];
      bb   = {128'(be[i]), 128'(be[i])} >> (128 - int'(off[i]));
      be_l = bb[127:0];
      for (int p = 0; p < 128; p++)
        if (be_l[p]) begin
          m_be[p] = 1'b1;
          m_d[8*p +: 8] = d_l[8*p +: 8];
        end
    end
  end
endmodule

// ================================================================================================================
// the combinational blocks of the LSU, shared by ng_lsu (LSU_K = 1, one instruction in flight)
// ng_lsu_kn (LSU_K >= 2: front end / ring / back end).  Pure extraction from ng_lsu (same expressions):
// ng_lsu is cycle for cycle unchanged.
// ================================================================================================================

// ng_lsu_fields -- operand fields of the M / UM / SYS formats
module ng_lsu_fields
  import ng_lsu_pkg::*;
(
  input  dec_t        dq,
  input  logic [31:0] w0,
  input  logic [31:0] w1,
  output logic [6:0]  f_rd,
  output logic [6:0]  f_rs1,
  output logic [6:0]  f_rs2,
  output logic [2:0]  f_cpol,
  output logic [63:0] f_imm
);
  always_comb begin
    if (dq.um) begin
      f_rd   = {1'b0, w0[12:7]};
      f_rs1  = {1'b0, w0[18:13]};
      f_rs2  = {1'b0, w0[24:19]};
      f_imm  = {{40{w1[23]}}, w1[23:0]};
      f_cpol = w1[26:24];
    end else begin
      f_rd   = w0[13:7];
      f_rs1  = w0[20:14];
      f_rs2  = {1'b0, w0[26:21]};
      f_imm  = {{40{w1[31]}}, w1[31:8]};
      f_cpol = w1[5:3];
    end
  end
endmodule

// ng_lsu_chk -- the model's pre-access trap sequence (CHK: cpol, operand alignment, address-register range,
// data-register range) and E6 (destination register range, evaluated after the lane accesses)
module ng_lsu_chk
  import ng_lsu_pkg::*;
(
  input  dec_t        dq,
  input  logic [31:0] w0,
  input  logic [6:0]  f_rd,
  input  logic [6:0]  f_rs1,
  input  logic [6:0]  f_rs2,
  input  logic [2:0]  f_cpol,
  input  logic [7:0]  nregs,
  output logic        pt_v,
  output logic [3:0]  pt_cause,
  output logic [31:0] pt_info,
  output logic        e6_v,
  output logic [31:0] e6_info
);
  wire       is_uld = (dq.kind == K_ULD);
  wire [6:0] nby    = dq.nbytes;
  always_comb begin
    logic [7:0] rr;
    pt_v = 1'b0; pt_cause = '0; pt_info = '0;
    if (!dq.hit) begin
      pt_v = 1'b1; pt_cause = CAUSE_ILLEGAL; pt_info = w0;
    end else if (dq.kind != K_CACHE) begin
      if (dq.has_cpol && f_cpol > 3'd3) begin
        pt_v = 1'b1; pt_cause = CAUSE_ILLEGAL; pt_info = w0;
      end else if (dq.um ? ((dq.ukind == UK_PLAIN || dq.ukind == UK_X) ? f_rs1[0] : (f_rs1[1:0] != 2'b00))
                         : ((dq.space == SP_GLOBAL) ? f_rs1[0] :
                            (dq.space == SP_UBASE)  ? f_rs2[0] :
                            (dq.space == SP_BUF)    ? (f_rs2[1:0] != 2'b00) : 1'b0)) begin
        pt_v = 1'b1; pt_cause = CAUSE_ILLEGAL; pt_info = w0;
      end else if (!dq.um) begin
        // address registers (model order: lo then hi of the pair, then data regs)
        if ({1'b0, f_rs1} >= nregs) begin
          pt_v = 1'b1; pt_cause = CAUSE_REGRANGE; pt_info = {25'd0, f_rs1};
        end else if (dq.space == SP_GLOBAL && ({1'b0, f_rs1} + 8'd1) >= nregs) begin
          pt_v = 1'b1; pt_cause = CAUSE_REGRANGE; pt_info = {24'd0, {1'b0, f_rs1} + 8'd1};
        end else begin
          for (int k = 3; k >= 0; k--) begin
            rr = {1'b0, f_rd} + 8'(k);
            if (k < int'(dq.ndata) && rr >= nregs) begin
              pt_v = 1'b1; pt_cause = CAUSE_REGRANGE; pt_info = {24'd0, rr};
            end
          end
        end
      end
    end
  end
  always_comb begin
    e6_v = 1'b0; e6_info = '0;
    if (is_uld) begin
      // ERRATA v0.1.4: info = first out-of-range register; uld is UM format (rd < 64), so always 64
      if ({2'b0, f_rd} + {3'b0, nby[6:2]} > 9'd64) begin e6_v = 1'b1; e6_info = 32'd64; end
    end else if (dq.ndst != 3'd0) begin
      // ERRATA v0.1.4: info = first out-of-range register = max(rd, nregs)
      if ({1'b0, f_rd} + {5'b0, dq.ndst} > {1'b0, nregs}) begin e6_v = 1'b1; e6_info = ({1'b0, f_rd} >= {1'b0, nregs}) ? {25'd0, f_rd} : {24'd0, nregs}; end
    end
  end
endmodule

// ng_lsu_lane -- address / byte-valid mask of the lane being walked by S_ADDR (lane li), the scratch slice count
// candidate and the shared-window check of that lane
module ng_lsu_lane
  import ng_lsu_pkg::*;
(
  input  dec_t        dq,
  input  ctx_t        cq,
  input  logic [63:0] f_imm,
  input  logic [31:0] rs1v_l,      // v[rs1] of the lane
  input  logic [31:0] rs1h_l,      // v[rs1+1] of the lane
  input  logic [63:0] ub,          // upair value
  input  logic [31:0] uxv,         // uld.x ureg value
  input  logic [31:0] ud [4],      // descriptor dwords
  input  logic [63:0] scr_lane,    // scratch: R + 124 * slice + 4 * li
  input  logic [2:0]  sl_q,        // scratch slice
  output logic [63:0] ln_off,      // signed offset (buf/scratch/shared)
  output logic [63:0] ln_a,        // VA, or shared offset
  output logic [63:0] ln_vm,       // valid byte mask
  output logic [2:0]  ln_ns,       // dword columns the lane's access touches
  output logic        ln_sh_bad    // shared window check: off + n must fit in [0, size]
);
  wire       is_ld   = (dq.kind == K_LD);
  wire       is_st   = (dq.kind == K_ST);
  wire       is_at   = (dq.kind == K_ATOM);
  wire       is_uld  = (dq.kind == K_ULD);
  wire       is_scr  = !dq.um && (dq.space == SP_SCRATCH) && (is_ld || is_st);
  wire [6:0] nby     = dq.nbytes;

  // descriptor decode (DESCRIPTORS.md buffer: base = {dw1[15:0], dw0}, size = dw2;
  // null = all zero; malformed = reserved bits or base+size > 2^48)
  logic        d_valid;
  logic [63:0] d_base;
  logic [31:0] d_limit;
  always_comb begin
    logic [48:0] sum;
    logic        nul;
    nul = (ud[0] == 32'd0) && (ud[1] == 32'd0) && (ud[2] == 32'd0) && (ud[3] == 32'd0);
    sum = {1'b0, ud[1][15:0], ud[0]} + {17'd0, ud[2]};
    d_base  = {16'd0, ud[1][15:0], ud[0]};
    d_valid = !nul && (ud[1][31:16] == 16'd0) && (ud[3] == 32'd0) && !sum[48];
    d_limit = d_valid ? ud[2] : 32'd0;
  end

  function automatic logic [63:0] vmask64(input logic [63:0] off, input int n, input logic [31:0] limit,
                                          input logic valid, input logic whole);
    logic [63:0] m;
    logic signed [63:0] o, lim;
    m = '0;
    lim = {32'd0, limit};
    if (valid) begin
      if (n <= 4 || whole) begin
        o = off;
        if (o >= 0 && o + n <= lim) m = (n >= 64) ? '1 : ((64'd1 << n) - 64'd1);
      end else begin
        for (int k = 0; k < 64; k += 4) begin
          if (k < n) begin
            o = off + k;
            if (o >= 0 && o + 4 <= lim) m[k +: 4] = 4'hf;
          end
        end
      end
    end
    return m;
  endfunction

  always_comb begin
    logic [63:0] zv;
    logic        ln_chk;
    logic [31:0] ln_lim;
    logic        ln_valid;
    zv = {32'd0, rs1v_l};
    ln_off = zv + f_imm;
    ln_a = '0; ln_chk = 1'b0; ln_lim = '0; ln_valid = 1'b1;
    if (is_uld) begin
      case (dq.ukind)
        UK_PLAIN: ln_a = ub + f_imm;
        UK_X:     ln_a = ub + {32'd0, uxv} + f_imm;
        UK_BUF:   begin ln_off = f_imm; ln_a = d_base + ln_off; ln_chk = 1'b1; ln_lim = d_limit; ln_valid = d_valid; end
        default:  begin ln_off = {32'd0, uxv} + f_imm; ln_a = d_base + ln_off; ln_chk = 1'b1; ln_lim = d_limit; ln_valid = d_valid; end
      endcase
    end else begin
      case (dq.space)
        SP_GLOBAL: ln_a = {rs1h_l, rs1v_l} + f_imm;
        SP_UBASE:  ln_a = ub + zv + f_imm;
        SP_BUF:    begin ln_a = d_base + ln_off; ln_chk = 1'b1; ln_lim = d_limit; ln_valid = d_valid; end
        SP_SHARED: ln_a = ln_off;
        // interleaved: (o & ~3) * 32 + o % 4 on top of R + 124 * slice + 4 * lane
        default:   begin ln_a = scr_lane + {ln_off[58:2], 5'd0, ln_off[1:0]}; ln_chk = 1'b1; ln_lim = cq.scr_bytes;
                         ln_valid = (cq.scr_bytes != 32'd0) && (cq.scr_base != 48'd0); end
      endcase
    end
    if (!ln_chk) ln_vm = (nby >= 7'd64) ? '1 : ((64'd1 << nby) - 64'd1);
    else         ln_vm = vmask64(ln_off, int'(nby), ln_lim, ln_valid, is_at);
    // scratch slice: only the lane bytes j in [4s - o%4, 4s - o%4 + 4) (one dword column)
    if (is_scr) ln_vm = ln_vm & {44'd0, (20'hf << (4 * int'(sl_q))) >> ln_off[1:0]};
  end
  assign ln_ns = 3'(({5'd0, ln_off[1:0]} + nby + 7'd3) >> 2);

  always_comb begin
    logic [33:0] e;
    e = {2'b0, ln_off[31:0]} + {27'd0, nby[6:0]};
    ln_sh_bad = (ln_off[63:32] != 32'd0) || (e > {2'b0, cq.sh_size});
  end
endmodule

// ng_lsu_piece -- line-piece membership of ONE lane (piece 0 = the bytes in the lane's first line, piece 1 = the
// bytes that cross into the next line); atomics never cross (whole access in piece 0)
module ng_lsu_piece
  import ng_lsu_pkg::n0_of, ng_lsu_pkg::LINE_B8;
(
  input  logic [63:0] a,        // lane VA
  input  logic [63:0] mk,       // lane byte-valid mask
  input  logic        act,      // lane accesses
  input  logic [6:0]  nby,
  input  logic        is_at,
  output logic        p0,
  output logic        p1
);
  always_comb begin
    logic [7:0] n0;
    n0 = n0_of(a[6:0], nby);
    if (is_at) begin
      p0 = act && (mk != 64'd0);
      p1 = 1'b0;
    end else begin
      p0 = act && ((mk & ((n0 >= 8'd64) ? '1 : ((64'd1 << n0) - 64'd1))) != 64'd0);
      p1 = act && (({1'b0, a[6:0]} + {1'b0, nby}) > LINE_B8) &&
           (((n0 >= 8'd64) ? 64'd0 : (mk >> n0)) != 64'd0);
    end
  end
endmodule

// ng_lsu_gsel -- line-group selection on the pending pieces: the lowest pending lane picks the line, every pending
// piece on that line joins; gb_new = beats (L1_LPB lanes each) that carry a member lane (uld: its 4 words)
module ng_lsu_gsel
  import ng_bitops_pkg::lowest_set;
#(
  parameter int L1_LPB = 4,
  localparam int NB1 = 32 / L1_LPB
) (
  input  logic [31:0]    pend0,
  input  logic [31:0]    pend1,
  input  logic [63:0]    ua [32],
  input  logic           is_uld,
  output logic           sel_found,
  output logic [4:0]     sel_l,
  output logic           sel_p1,
  output logic [63:0]    sel_line,
  output logic [31:0]    sel_m0,
  output logic [31:0]    sel_m1,
  output logic [NB1-1:0] gb_new
);
  logic [63:0] sel_lm1;
  always_comb begin
    sel_m0 = '0; sel_m1 = '0;
    sel_found = |(pend0 | pend1);
    sel_l     = 5'(lowest_set({32'd0, pend0 | pend1}));
    sel_p1    = sel_found && !pend0[sel_l];
    sel_line = sel_p1 ? ((ua[sel_l] & ~64'd127) + 64'd128) : (ua[sel_l] & ~64'd127);
    sel_lm1  = sel_line - 64'd128;           // line + 128 == sel_line  <=>  line == sel_line - 128
    for (int l = 0; l < 32; l++) begin
      sel_m0[l] = pend0[l] && ((ua[l] & ~64'd127) == sel_line);
      sel_m1[l] = pend1[l] && ((ua[l] & ~64'd127) == sel_lm1);
    end
  end
  always_comb begin
    gb_new = '0;
    for (int g = 0; g < NB1; g++) begin
      if (is_uld) gb_new[g] = (g * L1_LPB < 4);
      else        gb_new[g] = (sel_m0[g * L1_LPB +: L1_LPB] != '0) || (sel_m1[g * L1_LPB +: L1_LPB] != '0);
    end
  end
endmodule

// ng_lsu_gkey -- fault candidate of a line group (lowest member lane, lowest segment).  Keys are {lane, byte}: the
// lane part dominates, so the winning candidate is always the lowest member lane (a member lane is in exactly one
// of gm0 / gm1).  Only that lane is evaluated.
module ng_lsu_gkey
  import ng_lsu_pkg::n0_of;
  import ng_bitops_pkg::lowest_set;
(
  input  logic [31:0] gm0,
  input  logic [31:0] gm1,
  input  logic [63:0] ua [32],
  input  logic [15:0] vml [32],
  input  logic [63:0] vmu,
  input  logic [6:0]  nby,
  input  logic        is_uld,
  input  logic        wide_seg,
  input  logic        is_scr,
  output logic        gk_v,
  output logic [10:0] gk_key,
  output logic [31:0] gk_info,
  output logic [5:0]  gk_boff     // byte offset of the reported VA: segment (wide), 0, or the run's first byte (scratch)
);
  always_comb begin
    logic [63:0] mk, lo_m;
    logic [7:0]  n0;
    logic [5:0]  jl;
    logic [10:0] key;
    logic [31:0] info;
    logic        found;
    logic [4:0]  fl;
    gk_v = 1'b0; gk_key = '1; gk_info = '0; gk_boff = '0;
    found = |(gm0 | gm1);
    fl    = 5'(lowest_set({32'd0, gm0 | gm1}));
    mk = (fl == 5'd0 && is_uld) ? vmu : {48'd0, vml[fl]};
    n0 = n0_of(ua[fl][6:0], nby);
    lo_m = '0; jl = '0;
    if (found) begin
      if (gm0[fl]) begin
        lo_m = mk & ((n0 >= 8'd64) ? '1 : ((64'd1 << n0) - 64'd1));
        jl   = lowest_set(lo_m);
      end else begin
        lo_m = (n0 >= 8'd64) ? 64'd0 : (mk >> n0);
        jl   = lowest_set(lo_m) + n0[5:0];
      end
      key  = {fl, wide_seg ? {jl[5:2], 2'b00} : 6'd0};
      // scratch: the VA of the faulting run = the first member byte of this slice
      gk_boff = is_scr ? jl : (wide_seg ? {jl[5:2], 2'b00} : 6'd0);
      info = ua[fl][31:0] + {26'd0, gk_boff};
      gk_v = 1'b1; gk_key = key; gk_info = info;
    end
  end
endmodule

// ng_lsu_beat -- rotation and byte enables of one beat (L1_LPB lanes) of a line group: piece 0 / piece 1 of a lane
// are one rotation by the line offset (lane byte j <-> line byte (off + j) mod 128); j < n0 lands in this line
// (piece 0), j >= n0 wraps to the start of the NEXT line (piece 1); the enables select the piece of the group.
// Only the line offset ua[6:0] of a lane is needed (off7).  uld: lane 0's 16-byte words.
module ng_lsu_beat
  import ng_lsu_pkg::n0_of;
#(
  parameter int L1_LPB = 4,
  localparam int NB1 = 32 / L1_LPB,
  localparam int B1W = ng_noc_pkg::clog2_1(NB1)
) (
  input  logic [B1W-1:0] gbt,
  input  logic [6:0]     off7 [32],
  input  logic [15:0]    vml [32],
  input  logic [63:0]    vmu,
  input  logic [31:0]    gm0,
  input  logic [31:0]    gm1,
  input  logic [6:0]     nby,
  input  logic           is_uld,
  output logic [6:0]     bl_off [L1_LPB],    // rotation = line offset of lane byte 0 (uld: + 16 * word)
  output logic [15:0]    bl_be  [L1_LPB]     // lane bytes that take part in this group's piece
);
  always_comb begin
    int          l, src;
    logic [15:0] mk16;
    logic [7:0]  n0;
    int          jg;
    for (int i = 0; i < L1_LPB; i++) begin
      l   = int'(gbt) * L1_LPB + i;                 // rb slot (uld: 16-byte word of lane 0)
      src = is_uld ? 0 : l;
      n0  = n0_of(off7[src], nby);
      mk16 = is_uld ? ((l < 4) ? vmu[16*(l & 3) +: 16] : 16'd0) : vml[src];
      for (int j = 0; j < 16; j++) begin
        jg = (is_uld ? 16 * l : 0) + j;
        bl_be[i][j] = mk16[j] && (jg < int'(nby)) && ((gm0[src] && jg < int'(n0)) || (gm1[src] && jg >= int'(n0)));
      end
      bl_off[i] = off7[src] + (is_uld ? 7'(16 * l) : 7'd0);
    end
  end
endmodule

// ng_lsu_wbd -- write-back data: vector registers (beat wbt of destination register wk, EXEC lanes) and uniform
// registers (uld: word uwk) out of the result bytes rb
module ng_lsu_wbd #(
  parameter int NL = 16,
  localparam int NBEAT = 32 / NL,
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT)
) (
  input  logic [7:0]       rb [512],
  input  logic [BTW-1:0]   wbt,
  input  logic [2:0]       wk,
  input  logic [4:0]       uwk,
  input  logic [31:0]      exec,
  input  logic [6:0]       nby,
  input  logic             is_at,
  input  logic             sext,
  output logic [NL*32-1:0] wb_data,
  output logic [NL-1:0]    wb_lanes,
  output logic [31:0]      uw_data
);
  always_comb begin
    logic [31:0] val;
    int l;
    wb_data = '0; wb_lanes = '0;
    for (int i = 0; i < NL; i++) begin
      val = '0;
      for (int bi = 0; bi < NBEAT; bi++) begin
        l = bi * NL + i;
        if (int'(wbt) == bi) begin
          wb_lanes[i] = exec[l];
          if (nby == 7'd1 && !is_at) begin
            val = sext ? {{24{rb[l*16][7]}}, rb[l*16]} : {24'd0, rb[l*16]};
          end else if (nby == 7'd2 && !is_at) begin
            val = sext ? {{16{rb[l*16+1][7]}}, rb[l*16+1], rb[l*16]} : {16'd0, rb[l*16+1], rb[l*16]};
          end else begin
            for (int k = 0; k < 4; k++)
              if (int'(wk) == k) val = {rb[l*16 + 4*k + 3], rb[l*16 + 4*k + 2], rb[l*16 + 4*k + 1], rb[l*16 + 4*k]};
          end
        end
      end
      wb_data[32*i +: 32] = val;
    end
  end
  always_comb begin
    uw_data = '0;
    for (int k = 0; k < 16; k++)
      if (int'(uwk) == k) uw_data = {rb[4*k+3], rb[4*k+2], rb[4*k+1], rb[4*k]};
  end
endmodule

// ================================================================================================================
// ng_lsu_core -- the LSU proper, shared by ng_lsu (K = 1) and ng_lsu_kn (K >= 2); each of the two is a wrapper with its
// former port list, and both settings are cycle for cycle the former separate modules (every register, and the state
// encodings of the former enums).
//
// K = 1: one instruction in flight (the flow at the top of this file).  A line request waits for its answer in S_GL1
//   and a load line is scattered into rb by the state machine (S_GSC); no ring / IOF / back end, the uc channel is idle.
// K >= 2 (cross-instruction concurrency), split into:
//   FE (front end, one instruction at a time): accept, CHK, operand reads, per-lane addresses, PROBE pass (pass 0:
//      every translation / bound / pa_valid fault is found here -> precise trap, no memory effect), E6, then
//      S_RALLOC (wait for a free ring entry, copy the instruction's metadata into ctx[tail]) and the COMMIT pass
//      that ISSUES every line request (S_GISS) without waiting for its answer;
//   ring (K entries): the instructions between COMMIT and retirement, in acceptance order;
//   IOF (NQ entries, issue order): one entry per issued line request {ctx, fabric slot, read?, group members,
//      fault info of the group}; completion (and error) is recorded per IOF entry (a write slot is freed by
//      the fabric at its answer and may be reused before the BE gets to the entry);
//   BE (back end, acceptance order): waits for the IOF entries of the ring head, reads an answered line out of the
//      fabric's read buffer (u_rd_slot / u_rd_data, then u_free), scatters it into rb, writes the registers,
//      done / cnt_dec -- so done, trap and every waitcnt decrement come out in acceptance order.
// Instruction classes (decided at accept, K >= 2):
//   asynchronous (FE / ring / BE): non-atomic global-path ld / st (global, ubase, buf, scratch) and uld;
//   synchronous: shared ld / st / atom, global atomics, cache.* and illegal encodings -- S_SYNC waits until the
//      ring is empty and the BE idle, then the K = 1 state machine runs unchanged (it owns rb / the write port).
// Requests: uc = 1 (not VRAM; everything in the M1 RM) goes to the uc channel (u_*, the slot protocol of
//   ng_rsp_fab_ts, through ng_sp_top's mport); uc = 0 (VRAM) goes through the L1D
//   (one request at a time) and is only issued while the IOF is empty (no L1D hit can overtake an in-flight write).
// Traps: FE traps (precise) wait in S_TWAIT until the ring is empty and the BE idle, so the trap pulse follows the
//   completion of every earlier accepted instruction.  A fabric error answer ("late error": RRESP / BRESP or a PA the
//   RM cannot serve) is handled by the BE: the first erroring IOF entry of the instruction (issue order) gives
//   trap_info / ext_flt_va, the remaining entries are consumed (data dropped), then B_TRAP (BAD_PHYS_ADDR memory
//   fault, no register write).  Other groups of that instruction may already have been written (non-precise,
//, not approved; the FE does not stop issuing the instruction's remaining groups
//   --
// Scratch slices: every slice of the commit pass takes its own ring entry (the slice's lane offsets /
//   masks); only the first clears rb, only the last writes back / completes; an error is carried to the last one.
// Plan A: ng_sp_sched's per-warp in-flight bit stays, so a warp has at most one
//   LSU instruction in flight; the concurrency is between warps (and between the line groups of one instruction).
// Reset (rst_n = ng_sp_top crst_n): FE, ring, IOF and BE are reset; an instruction in flight produces no done / trap /
//   cnt_dec.  Answers of requests issued before the reset are dropped by ng_sp_top's mport (owner table).
//
//
// ================================================================================================================
module ng_lsu_core
  import ng_lsu_pkg::*;
  import ng_bitops_pkg::lowest_set;
#(
  parameter int NL      = 16,   // register-file lanes per beat (16 or 32 == NG_RF_NL)
  parameter int SLOTW   = 4,
  parameter int SH_AW   = 16,   // shared array byte-address width
  parameter int SH_LPB  = 8,    // shared memory: lanes per beat/round (32 = former single-round behaviour)
  parameter int L1_LPB  = 4,    // global path: lanes per beat of the load scatter / store merge (32 = widest)
  parameter int K       = 1,    // instruction contexts: 1 (ng_lsu), or ring entries 2 / 4 / 8 (ng_lsu_kn)
  parameter int MEM_OUT = 8,    // K >= 2: fabric slots of the uc channel (1, 2, 4, 8, 16)
  parameter int NQ      = (MEM_OUT < 1) ? 2 : 2 * MEM_OUT,  // K >= 2: IOF entries (power of two)
  localparam int MSW    = ng_noc_pkg::clog2_1(MEM_OUT)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- shared-memory scrub (ng_wd, between dispatches) ----
  input  logic              sh_clr_en,
  input  logic [SH_AW-7-1:0] sh_clr_row,

  // ---- issue ---------------------------------------------------------------
  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,
  input  logic [31:0]       iss_w1,
  input  logic [31:0]       iss_exec,
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,
  input  logic [15:0]       iss_tag,
  input  ctx_t              iss_ctx,

  // ---- completion (one-cycle pulses, exactly one per accepted instruction) ----
  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              busy,

  // ---- waitcnt accounting (ng_sp_sched cnt_inc_* / cnt_dec_*) --------------------
  output logic              cnt_inc_valid,
  output logic [SLOTW-1:0]  cnt_inc_slot,
  output logic [2:0]        cnt_inc_id,
  output logic              cnt_dec_valid,
  output logic [SLOTW-1:0]  cnt_dec_slot,
  output logic [2:0]        cnt_dec_id,

  // ---- vector register file: one read port, one write port -----------------------
  output logic                      rd_valid,
  input  logic                      rd_ready,
  output logic [SLOTW-1:0]          rd_slot,
  output logic [6:0]                rd_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat,
  output logic [1:0]                rd_hsel,
  input  logic                      rdv_valid,
  input  logic                      rdv_err,
  input  logic [NL*32-1:0]          rdv_data,

  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  // ---- uniform register file ------------------------------------------------------
  output logic                      ur_valid,
  output logic [SLOTW-1:0]          ur_slot,
  output logic [5:0]                ur_reg,
  input  logic                      urd_valid,
  input  logic [31:0]               urd_data,

  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data,

  // ---- MMU client (ng_mmu) ----------------------------------------------------------
  output logic                      mmu_req_valid,
  input  logic                      mmu_req_ready,
  output logic [3:0]                mmu_req_vmid,
  output logic [63:0]               mmu_req_va,
  output logic [1:0]                mmu_req_acc,
  output logic                      mmu_req_atomic,
  output logic [7:0]                mmu_req_tag,
  input  logic                      mmu_resp_valid,
  output logic                      mmu_resp_ready,
  input  logic [2:0]                mmu_resp_fault,
  input  logic                      mmu_resp_sparse,
  input  logic [47:0]               mmu_resp_pa,
  input  logic [63:0]               mmu_resp_pte,

  // BAD_PHYS_ADDR reported through the MMU fault latch (ng_mmu ext_flt_*)
  output logic                      ext_flt_valid,
  output logic [3:0]                ext_flt_vmid,
  output logic [63:0]               ext_flt_va,
  output logic [1:0]                ext_flt_acc,

  // ---- L1D request port (K >= 2: uc = 0 line requests and the synchronous-class requests) ----
  output logic                      l1_req_valid,
  input  logic                      l1_req_ready,
  output mreq_t                     l1_req,
  input  logic                      l1_rsp_valid,
  input  mrsp_t                     l1_rsp,

  // ---- uc channel (K >= 2): slot protocol (ng_rsp_fab_ts, through ng_sp_top's mport) ----
  output logic                      u_req_valid,
  input  logic                      u_req_ready,
  output mreq_t                     u_req,
  input  logic [MSW-1:0]            u_req_slot,     // slot the request takes (valid with u_req_ready)
  input  logic                      u_rsp_valid,    // answer of one of OUR slots (one-clock pulse)
  input  logic [MSW-1:0]            u_rsp_slot,
  input  logic [1:0]                u_rsp_err,
  output logic                      u_rd_req,       // the BE wants to read the line of u_rd_slot this clock
  output logic [MSW-1:0]            u_rd_slot,
  input  logic                      u_rd_gnt,       // read port free this clock (not used by an I$ / L1D answer)
  input  logic [LINE_BITS-1:0]      u_rd_data,
  output logic                      u_free,         // free u_rd_slot (only with u_rd_gnt)

  // ---- statistics (test / debug) -------------------------------------------------------
  output logic [31:0]               stat_l1_rd,     // line reads issued
  output logic [31:0]               stat_l1_wr,     // line writes issued
  output logic [31:0]               stat_l1_amo,    // atomics issued to L2
  output logic [31:0]               stat_sh_rounds, // shared-memory rounds
  // global (non-shared, non-atomic) ld / st / uld instructions that reached the line-group path, and the
  // line-group requests their commit pass issued (one per non-sparse group; ratio = lines per memory instruction)
  output logic [31:0]               stat_instr_mem,
  output logic [31:0]               stat_groups,
  // debug counters (K >= 2; RM debug page 0x980..), cleared by rst_n like stat_*:
  //   word 0 FE_BUSY (FE not idle), 1 FE_RALLOC (waiting for a ring entry), 2 FE_ISS_STALL (line request waiting for the
  //   fabric / L1D / an IOF entry), 3 FE_SYNC (synchronous instruction or FE trap waiting for the drain),
  //   4 / 5 RING_OCC lo / hi[15:0] (sum of the ring occupancy per cycle, 48 bit), 6 RING_MAX, 7 BE_WAIT (head entry
  //   waiting for its answer), 8 BE_WB_STALL (register write waiting for wr_ready), 9 LATE_ERR (fabric-error traps)
  output logic [319:0]              stat_kn
);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int DW    = NL * 32;
  localparam bit KN    = (K > 1);     // front end / ring / back end
  localparam int KW    = ng_noc_pkg::clog2_1(K);
  localparam int QW    = ng_noc_pkg::clog2_1(NQ);
  localparam int NSL   = 1 << MSW;

  if (K < 1 || (K & (K - 1)) != 0) begin : g_bad_k
    $error("ng_lsu_core: K must be a power of two");
  end
  if (KN && (NQ < 2 || (NQ & (NQ - 1)) != 0)) begin : g_bad_nq
    $error("ng_lsu_core: NQ must be a power of two >= 2");
  end

  // --------------------------------------------------------------
  // Explicit values: K = 1 / K >= 2 keep the encodings of the former ng_lsu / ng_lsu_kn enums.  S_GL1 / S_GSC exist only
  // for K = 1, S_GISS / S_SYNC / S_RALLOC / S_TWAIT only for K >= 2 (the other setting parks them on unused values).
  typedef enum logic [5:0] {
    S_IDLE, S_CHK, S_URD, S_VRD, S_PREP, S_ADDR, S_PINIT,
    S_GSEL, S_GMMU, S_GMW, S_GMH, S_GEXE,
    S_GL1  = KN ? 6'd46 : 6'd12,
    S_GISS = KN ? 6'd12 : 6'd44,
    S_ASEL = 6'd13, S_AMMU, S_AMW, S_AEXE, S_AL1, S_ARD, S_ARDW, S_ACMP, S_ACAL, S_ACAS, S_AWR, S_AWRW,
    S_E6, S_SH, S_SHW, S_SAL, S_SALW, S_SAC, S_SAS, S_SASW, S_GMRG,
    S_GSC  = KN ? 6'd47 : 6'd34,
    S_AWB  = KN ? 6'd34 : 6'd35, S_ACAI, S_SAW, S_PIN2, S_CACHE,
    S_SYNC = KN ? 6'd39 : 6'd45, S_RALLOC, S_TWAIT,
    S_WB   = KN ? 6'd42 : 6'd40, S_UWB, S_DONE, S_TRAP
  } st_t;
  st_t st;

  typedef enum logic [3:0] {B_IDLE, B_CLR, B_HEAD, B_RD, B_SC, B_WB, B_UWB, B_DONE, B_TRAP} bst_t;
  bst_t bs;                          // BE state (K = 1: B_IDLE)

  // ---------------------------------------------------------------- latched instruction (FE)
  logic [31:0]      w0_q, w1_q, exec_q;
  logic [SLOTW-1:0] slot_q;
  logic [7:0]       nregs_q;
  logic [15:0]      tag_q;
  ctx_t             cq;
  dec_t             dq, dec_c;
  logic             async_q;         // the FE's instruction is of the asynchronous class (K = 1: never)
  ng_lsu_dec u_dec (.w0(iss_w0), .w1(iss_w1), .d(dec_c));

  function automatic logic async_of(input dec_t d);
    return d.hit && ((d.kind == K_ULD) ||
                     (!d.um && (d.kind == K_LD || d.kind == K_ST) && (d.space != SP_SHARED)));
  endfunction

  logic [6:0]  f_rd, f_rs1, f_rs2;
  logic [2:0]  f_cpol;
  logic [63:0] f_imm;
  ng_lsu_fields u_fld (.dq(dq), .w0(w0_q), .w1(w1_q), .f_rd(f_rd), .f_rs1(f_rs1), .f_rs2(f_rs2), .f_cpol(f_cpol),
                       .f_imm(f_imm));

  wire is_ld   = (dq.kind == K_LD);
  wire is_st   = (dq.kind == K_ST);
  wire is_at   = (dq.kind == K_ATOM);
  wire is_uld  = (dq.kind == K_ULD);
  wire is_sh   = !dq.um && (dq.space == SP_SHARED) && (is_ld || is_st || is_at);
  wire is_fp   = is_at && (dq.amo >= AMO_FADD);
  // buffer-style bounds rules apply: buf, scratch, uld.buf, uld.bufx
  wire chk_sp  = is_uld ? (dq.ukind == UK_BUF || dq.ukind == UK_BUFX)
                        : (dq.space == SP_BUF || dq.space == SP_SCRATCH);
  wire wide_seg = chk_sp && !is_at && (dq.nbytes > 7'd4);   // per 4 B segment checks / fault info
  // interleaved scratch: sliced by dword column (scratch has no atomics; the guard keeps decode garbage out)
  wire is_scr  = !dq.um && (dq.space == SP_SCRATCH) && (is_ld || is_st);
  logic [2:0] sl_q;   // current scratch slice
  logic [2:0] ns_q;   // number of slices (max over the active lanes, found while slice 0 of the probe is walked)
  wire [6:0] nby = dq.nbytes;

  // ---------------------------------------------------------------- CHK: pre-access traps
  logic        pt_v;
  logic [3:0]  pt_cause;
  logic [31:0] pt_info;
  // E6: destination register range, evaluated after the lane accesses (model order)
  logic        e6_v;
  logic [31:0] e6_info;
  ng_lsu_chk u_chk (.dq(dq), .w0(w0_q), .f_rd(f_rd), .f_rs1(f_rs1), .f_rs2(f_rs2), .f_cpol(f_cpol), .nregs(nregs_q),
                    .pt_v(pt_v), .pt_cause(pt_cause), .pt_info(pt_info), .e6_v(e6_v), .e6_info(e6_info));

  // ---------------------------------------------------------------- operand storage (FE)
  logic [31:0] rs1v [32];     // v[rs1] (lo)
  logic [31:0] rs1h [32];     // v[rs1+1] (global high half)
  logic [31:0] dv   [4][32];  // data registers rd..rd+3
  logic [63:0] ub;            // upair value
  logic [31:0] uxv;           // uld.x ureg value
  logic [31:0] ud   [4];      // descriptor dwords
  logic [63:0] ua   [32];     // per-lane VA (shared: byte offset)
  logic [15:0] vml  [32];     // per-lane byte-valid mask (vector ops)
  logic [63:0] vmu;           // uld byte-valid mask
  logic [7:0]  rb   [512];    // result bytes: lane l at l*16 (uld: 0..63); K >= 2: BE (asynchronous class) / FE (synchronous)
  logic [31:0] act_q;         // lanes that access (exec; uld: lane 0)
  logic [15:0] shb_imm;       // shared: sh_base + imm (window address of lane offset 0)

  // ---------------------------------------------------------------- uniform read sequencer
  logic [2:0]  uj;        // job index 0..4
  logic        uph;       // 0 issue, 1 wait
  logic [2:0]  ucnt;      // consecutive uniform regs (0,2,4)
  logic [5:0]  ubase_r;   // first uniform reg
  logic        ux_need;
  always_comb begin
    ucnt = 3'd0; ubase_r = '0; ux_need = 1'b0;
    if (dq.um) begin
      ubase_r = f_rs1[5:0];
      ucnt    = (dq.ukind == UK_PLAIN || dq.ukind == UK_X) ? 3'd2 : 3'd4;
      ux_need = (dq.ukind == UK_X || dq.ukind == UK_BUFX);
    end else if (dq.space == SP_UBASE) begin
      ubase_r = f_rs2[5:0]; ucnt = 3'd2;
    end else if (dq.space == SP_BUF) begin
      ubase_r = f_rs2[5:0]; ucnt = 3'd4;
    end
  end

  // ---------------------------------------------------------------- vector read sequencer
  logic [2:0]  vj;        // job: 0 rs1, 1 rs1+1 (global), 2..5 data regs
  logic [BTW-1:0] vb;
  logic        vph;
  function automatic logic vjob_needed(input logic [2:0] j);
    case (j)
      3'd0: return !dq.um && (is_ld || is_st || is_at);
      3'd1: return !dq.um && (is_ld || is_st || is_at) && dq.space == SP_GLOBAL;
      default: return (is_st || is_at) && (int'(j) - 2) < int'(dq.ndata);
    endcase
  endfunction
  function automatic logic [6:0] vjob_reg(input logic [2:0] j);
    case (j)
      3'd0: return f_rs1;
      3'd1: return f_rs1 + 7'd1;
      default: return f_rd + 7'(j - 3'd2);
    endcase
  endfunction

  // ---------------------------------------------------------------- address generation (ADDR)
  logic [4:0]  li;                    // lane being computed
  logic [63:0] scr_lane;              // scratch: R + 124 * slice + 4 * li (lane li's dword column 0, slice-shifted)
  logic        flt_v;                 // first fault found (lane order)
  logic [10:0] flt_key;               // {lane, segment byte offset}
  logic [31:0] flt_info;
  logic        flt_badphys;
  logic [63:0] flt_va64;
  logic        flt_w;

  // vector-lane address / validity for lane li (combinational), shared-window check, scratch slice count candidate
  logic [63:0] ln_off;      // signed offset (buf/scratch/shared)
  logic [63:0] ln_a;        // VA, or shared offset
  logic [63:0] ln_vm;       // valid byte mask
  logic        ln_sh_bad;
  logic [2:0]  ln_ns;       // dword columns lane li's access touches (scratch slice count candidate)
  ng_lsu_lane u_lane (.dq(dq), .cq(cq), .f_imm(f_imm), .rs1v_l(rs1v[li]), .rs1h_l(rs1h[li]), .ub(ub), .uxv(uxv),
                      .ud(ud), .scr_lane(scr_lane), .sl_q(sl_q), .ln_off(ln_off), .ln_a(ln_a), .ln_vm(ln_vm),
                      .ln_ns(ln_ns), .ln_sh_bad(ln_sh_bad));

  // ---------------------------------------------------------------- group (line) engine (FE)
  logic [31:0] pend0, pend1;       // pieces waiting in the current pass
  logic [31:0] gm0, gm1;           // members of the current group
  logic [63:0] gline;              // VA of the group's line (128 B aligned)
  logic [63:0] a_new;              // registered atomic result (new value to write)
  logic [1:0]  ap;                 // atomic: piece being translated / written (0/1)
  logic [47:0] apa [2];            // atomic PA: [0] exact start byte, [1] line base of the next line
  logic        gp1;                // group was selected by a piece-1 lane
  logic [4:0]  gsl;                // lowest member lane
  logic        pass_c;             // 0 probe, 1 commit
  logic [47:0] g_pa;               // translated line PA
  logic        g_sparse;
  logic [63:0] g_pte;

  // Translation memo: pass 0 (probe) stores the MMU answer of every line group, pass 1 (commit) walks the
  // groups in the same order (the group selection depends only on pre0 / pre1 / ua, which do not change between the
  // passes) and reads the answer back instead of asking the MMU again -- halves the MMU requests of an access.
  // memo_i counts the groups of the current pass; entries are valid only inside one instruction (memo_v cleared in
  // S_CHK, i.e. on every accept, and by the reset); a group beyond MEMO_N entries, or a memo entry that is not valid,
  // is translated again as before.  Entry: {PA[47:7], sparse, PTE[6:5]} -- all the commit pass uses of g_pa / g_pte.
  localparam int MEMO_N  = 32;
  localparam int MEMO_IW = $clog2(MEMO_N);
  logic [6:0]            memo_i;            // group index within the pass (saturates at 127)
  logic [MEMO_N-1:0]     memo_v;
  (* ram_style = "distributed" *) logic [43:0] memo_d [MEMO_N];
  logic [43:0]           memo_q;
  assign memo_q = memo_d[memo_i[MEMO_IW-1:0]];
  wire memo_hit = pass_c && (memo_i < 7'(MEMO_N)) && memo_v[memo_i[MEMO_IW-1:0]];

  // group selection (combinational on pend0/pend1)
  logic [4:0]  sel_l;
  logic        sel_found, sel_p1;
  logic [63:0] sel_line;
  logic [31:0] sel_m0, sel_m1;
  localparam int NB1 = 32 / L1_LPB;
  localparam int B1W = ng_noc_pkg::clog2_1(NB1);
  logic [NB1-1:0]  gb_new;        // beats that carry a member lane of the group being selected (uld: its 4 words)
  ng_lsu_gsel #(.L1_LPB(L1_LPB)) u_gsel (.pend0(pend0), .pend1(pend1), .ua(ua), .is_uld(is_uld),
                                         .sel_found(sel_found), .sel_l(sel_l), .sel_p1(sel_p1), .sel_line(sel_line),
                                         .sel_m0(sel_m0), .sel_m1(sel_m1), .gb_new(gb_new));

  // fault candidate of the registered group (lowest member lane, lowest segment)
  logic        gk_v;
  logic [10:0] gk_key;
  logic [31:0] gk_info;
  logic        gkq_v;                 // ... registered (gm0 / gm1 are stable from S_GSEL, the MMU answers later)
  logic [10:0] gkq_key;
  logic [31:0] gkq_info;
  logic [63:0] gkq_ua;                // VA of the candidate lane (the byte offset is added when it is used)
  logic [5:0]  gk_boff, gkq_boff;     // byte offset of the reported VA: segment (wide), 0, or the run's first byte (scratch)
  ng_lsu_gkey u_gkey (.gm0(gm0), .gm1(gm1), .ua(ua), .vml(vml), .vmu(vmu), .nby(nby), .is_uld(is_uld),
                      .wide_seg(wide_seg), .is_scr(is_scr), .gk_v(gk_v), .gk_key(gk_key), .gk_info(gk_info),
                      .gk_boff(gk_boff));

  always_ff @(posedge clk) begin
    gkq_v <= gk_v; gkq_key <= gk_key; gkq_info <= gk_info; gkq_boff <= gk_boff;
    gkq_ua <= ua[gk_key[10:6]];
  end

  // ---------------------------------------------------------------- per-beat lane data path (global path)
  // The coalesced line image (stores) and the load scatter work on L1_LPB lanes per beat instead of
  // all 32 lanes at once.  Piece 0 / piece 1 of a lane are one rotation by the line offset
  // (lane byte j <-> line byte (off + j) mod 128): j < n0 lands in this line (piece 0), j >= n0
  // wraps to the start of the NEXT line (piece 1); the byte enables select the piece of the group.
  // Stores: merge the beat's lanes (ascending lane order, later lane wins) into g_be / g_wdata.
  // Loads: the line (K = 1: L1 response; K >= 2: the BE's line) is registered (gl_line) and scattered into rb beat by
  //         beat.
  logic [B1W-1:0]  gbt;           // current beat (global path)
  logic [NB1-1:0]  gbr;           // beats of the group that still have to be processed (empty beats are skipped)
  logic [LINE_B-1:0]      g_be;          // merged line byte enables (stores)
  logic [LINE_BITS-1:0]   g_wdata;       // merged line image (stores)
  logic [LINE_BITS-1:0]   gl_line;       // load: line being scattered

  function automatic logic [7:0] sbyte(input int l, input int j);
    logic [31:0] d;
    d = (j < 16) ? dv[(j / 4) & 3][l] : 32'd0;   // dv holds rd..rd+3 only (16 B)
    return d[8*(j % 4) +: 8];
  endfunction

  logic [6:0]   bl_off [L1_LPB];    // rotation = line offset of lane byte 0 (uld: + 16 * word)
  logic [15:0]  bl_be  [L1_LPB];    // lane bytes that take part in this group's piece
  logic [127:0] bl_pay [L1_LPB];    // store payload (rd..rd+3 of the lane)
  logic [6:0]   bg_off [L1_LPB];
  logic [15:0]  bg_be  [L1_LPB];
  logic [6:0]   ua7    [32];
  always_comb for (int l = 0; l < 32; l++) ua7[l] = ua[l][6:0];
  ng_lsu_beat #(.L1_LPB(L1_LPB)) u_beat (.gbt(gbt), .off7(ua7), .vml(vml), .vmu(vmu), .gm0(gm0), .gm1(gm1), .nby(nby),
                                         .is_uld(is_uld), .bl_off(bg_off), .bl_be(bg_be));
  always_comb begin
    int          l, src;
    logic [7:0]  n0;
    n0 = 8'd0;
    for (int i = 0; i < L1_LPB; i++) begin
      l   = int'(gbt) * L1_LPB + i;                 // rb slot (uld: 16-byte word of lane 0)
      src = is_uld ? 0 : l;
      bl_be[i]  = bg_be[i];
      bl_off[i] = bg_off[i];
      for (int j = 0; j < 16; j++) bl_pay[i][8*j +: 8] = sbyte(src, j);
      if (st == S_AWB) begin
        // line-crossing atomic write image: the 8 result bytes through lane 0 of the store merge
        // (piece ap: bytes below / at-or-above n0 = bytes that fit in the first line)
        n0 = n0_of(apa[0][6:0], nby);
        bl_pay[i] = (i == 0) ? {64'd0, a_new} : 128'd0;
        bl_off[i] = apa[0][6:0];
        for (int j = 0; j < 16; j++)
          bl_be[i][j] = (i == 0) && (j < int'(nby)) && ((ap == 2'd0) ? (j < int'(n0)) : (j >= int'(n0)));
      end
    end
  end

  logic [LINE_B-1:0] bm_be;
  logic [LINE_BITS-1:0] bm_d;
  ng_lsu_smerge #(.LPB(L1_LPB)) u_smerge (.pay(bl_pay), .off(bl_off), .be(bl_be), .m_be(bm_be), .m_d(bm_d));

  // ---------------------------------------------------------------- shared rounds (K >= 2: synchronous class)
  // One round serves (a prefix of) ONE group of SH_LPB lanes; the LSU picks the lowest group that still
  // has pending lanes and the shared unit answers 4 cycles after the request pulse.
  localparam int NBS = 32 / SH_LPB;
  localparam int SGW = ng_noc_pkg::clog2_1(NBS);
  logic [31:0]       sh_pend;
  logic              sh_wr_cmd;
  logic              sh_req;
  logic [SGW-1:0]    sh_gsel;            // group of the request in flight
  logic [SGW-1:0]    sh_fg;              // lowest group with a pending lane
  logic [SH_LPB-1:0] sh_pend_g;
  logic [SH_AW-1:0]  sh_addr [SH_LPB];   // (ua of a shared lane already includes sh_base)
  logic [127:0]      sh_wdata [SH_LPB];
  logic [SGW-1:0]    sh_rgrp;
  logic [SH_LPB-1:0] sh_rsel;
  logic [31:0]       sh_sel;             // served lanes of the response (global lane mask)
  logic              sh_rsp_valid;
  logic [127:0]      sh_rdata [SH_LPB];
  logic [4:0]        sh_nb;
  logic [4:0]        sal;                 // shared atomic: current lane
  logic [31:0]       sh_rem;              // shared atomic: lanes still to do
  logic [63:0]       res_new;             // atomic ALU result (combinational on am_old)

  always_comb begin
    int l;
    for (int i = 0; i < SH_LPB; i++) begin
      l = int'(sh_gsel) * SH_LPB + i;
      sh_addr[i]   = SH_AW'(ua[l][15:0]);
      sh_pend_g[i] = sh_pend[l];
      sh_wdata[i]  = '0;
      for (int q = 0; q < 16; q++) sh_wdata[i][8*q +: 8] = sbyte(l, q);
      if (st == S_SASW && l == int'(sal)) sh_wdata[i] = {64'd0, a_new};
    end
    sh_nb = 5'(nby - 7'd1);
    sh_fg = '0;
    for (int g = NBS - 1; g >= 0; g--) if (sh_pend[g * SH_LPB +: SH_LPB] != '0) sh_fg = SGW'(g);
    sh_sel = 32'(sh_rsel) << (int'(sh_rgrp) * SH_LPB);
  end

  ng_lsu_shmem #(.NB(32), .BYTES(1 << SH_AW), .AW(SH_AW), .LPB(SH_LPB)) u_shmem (
    .clk(clk), .rst_n(rst_n),
    .req_valid(sh_req), .req_wr(sh_wr_cmd), .req_grp(sh_gsel), .req_pend(sh_pend_g), .req_addr(sh_addr),
    .req_nb(sh_nb), .req_wdata(sh_wdata),
    .rsp_valid(sh_rsp_valid), .rsp_grp(sh_rgrp), .rsp_sel(sh_rsel), .rsp_rdata(sh_rdata),
    .clr_en(sh_clr_en), .clr_row(sh_clr_row)
  );

  // ---------------------------------------------------------------- atomics (K >= 2: synchronous class)
  logic [4:0]    al;                   // global atomic: current lane
  logic          a_cross;
  logic          asp [2];
  logic [LINE_BITS-1:0] aline [2];
  logic [63:0]   a_unit_va;
  logic [63:0]   am_old;               // old value being operated on
  logic [1:0]    fwc;                  // FP32 ALU pipeline wait counter (inputs must be stable for 3 edges)
  logic [2:0]    acb;                  // byte counter of the serial cross-line atomic read / write-image build

  // operand values of the lane in question: v = data[0] (lo[,hi]), v2 = swap value (cmpxchg)
  logic [4:0]  op_lane;
  logic [63:0] opc_v, opc_v2;   // combinational operand mux (dv[.][lane])
  logic [63:0] op_v, op_v2;     // ... registered: the lane (al / sal) is stable several cycles before use
  logic [63:0] am_new;
  logic [31:0] fa_r;
  always_comb begin
    op_lane = is_sh ? sal : al;
    opc_v  = {(dq.w64 ? dv[1][op_lane] : 32'd0), dv[0][op_lane]};
    opc_v2 = (dq.amo == ng_noc_pkg::AMO_CAS) ? {(dq.w64 ? dv[3][op_lane] : 32'd0), (dq.w64 ? dv[2][op_lane] : dv[1][op_lane])}
                               : 64'd0;
  end
  always_ff @(posedge clk) begin
    op_v <= opc_v;
    op_v2 <= opc_v2;
  end
  // integer ops: the L2 atomic ALU without its single-cycle FP32 adder (~10 ns; ops >= 13 -> "no change"); the FP32
  // ops run in the pipelined ng_lsu_fatomic
  ng_l2_amo_alu #(.EN_FP(0)) u_amo (.op(dq.amo), .is64(dq.w64), .a(am_old),
                                    .b(dq.amo == ng_noc_pkg::AMO_CAS ? op_v2 : op_v), .cmp(op_v), .r(am_new),
                                    .cas_hit());
  ng_lsu_fatomic u_fa (.clk(clk), .op(2'(dq.amo - AMO_FADD)), .a(am_old[31:0]), .b(op_v[31:0]), .r(fa_r));
  always_comb res_new = is_fp ? {32'd0, fa_r} : am_new;

  // ================================================================ K >= 2: ring / IOF / BE (g_ring below)
  // What the rest of the module sees of them; K = 1: constants (g_one).
  logic [KW:0]      rcnt;            // ring occupancy
  logic             q_empty, q_full; // IOF
  logic             be_err;          // late error recorded for the head instruction (carried across slices)
  logic [31:0]      be_info;
  logic [63:0]      be_va;
  logic [B1W-1:0]   be_gbt;
  logic [2:0]       be_wk;
  logic [BTW-1:0]   be_wbt;
  logic [4:0]       be_uwk;
  logic             be_l1take;       // the head's L1D line answer goes into gl_line
  // ring head entry (r_*[rh])
  logic [SLOTW-1:0] h_slot;
  logic [15:0]      h_tag;
  logic [31:0]      h_exec;
  logic [6:0]       h_rd;
  logic             h_st, h_sext;
  logic [6:0]       h_nby;
  logic [2:0]       h_cnt;
  logic [3:0]       h_vmid;
  // load scatter of a group into rb: K = 1 the FE's group (S_GSC), K >= 2 the BE's (B_SC)
  logic             sc_en;
  logic [B1W-1:0]   sc_gbt;
  logic [6:0]       sc_off [L1_LPB];
  logic [15:0]      sc_be  [L1_LPB];
  logic [127:0]     sc_d   [L1_LPB];
  ng_lsu_scat #(.LPB(L1_LPB)) u_scat (.line(gl_line), .off(sc_off), .dat(sc_d));

  wire drained   = (rcnt == '0) && (bs == B_IDLE);
  wire be_wb_act = (bs == B_WB) || (bs == B_UWB);

  // ---------------------------------------------------------------- misc / requests
  logic [3:0]  tr_cause_q;
  logic [31:0] tr_info_q;
  logic        tr_bad_q;
  mreq_t       rq_q;                  // L1 / uc request being issued
  logic        rq_v;
  wire [1:0]   pol = f_cpol[1:0];

  // FE events (K >= 2)
  wire fe_alloc = (st == S_RALLOC) && (rcnt < (KW + 1)'(K));
  wire fe_fdone = (st == S_GSEL) && !sel_found && pass_c && async_q;
  wire fe_iss_u = (st == S_GISS) && rq_v && rq_q.uc;
  wire fe_iss_l = (st == S_GISS) && rq_v && !rq_q.uc;
  wire fe_iss_hs = (fe_iss_u && u_req_valid && u_req_ready) || (fe_iss_l && l1_req_valid && l1_req_ready);

  // ---------------------------------------------------------------- writeback (FE; K >= 2: synchronous class / BE)
  logic [2:0]       wk;          // destination register index being written
  logic [BTW-1:0]   wbt;
  logic [4:0]       uwk;
  logic [DW-1:0]    wb_data;
  logic [NL-1:0]    wb_lanes;
  logic [31:0]      uwb_data;
  ng_lsu_wbd #(.NL(NL)) u_wbd (.rb(rb), .wbt(be_wb_act ? be_wbt : wbt), .wk(be_wb_act ? be_wk : wk),
                               .uwk(be_wb_act ? be_uwk : uwk), .exec(be_wb_act ? h_exec : exec_q),
                               .nby(be_wb_act ? h_nby : nby), .is_at(be_wb_act ? 1'b0 : is_at),
                               .sext(be_wb_act ? h_sext : dq.sext),
                               .wb_data(wb_data), .wb_lanes(wb_lanes), .uw_data(uwb_data));

  // ---------------------------------------------------------------- result-byte (rb) write bus
  // One write bus of WL lanes x 16 B shared by the three producers (load scatter, shared-memory
  // response, single-lane atomic result); rb is written group-wise from it.
  localparam int WL  = (SH_LPB > L1_LPB) ? SH_LPB : L1_LPB;
  localparam int WGW = (WL < 32) ? $clog2(32 / WL) : 1;
  localparam int R1  = WL / L1_LPB;
  localparam int RS  = WL / SH_LPB;
  logic              bus_en;
  logic [WGW-1:0]    bus_grp;
  logic [127:0]      bus_d  [WL];
  logic [15:0]       bus_be [WL];

  // atomic result (one lane, 8 B)
  logic        at_we;
  logic [4:0]  at_lane;
  logic [63:0] at_d;
  logic [7:0]  at_be;
  always_comb begin
    at_we = 1'b0; at_lane = al; at_d = '0; at_be = 8'hff;
    case (st)
      S_AL1: if (l1_rsp_valid && dq.rtn) begin
        at_we = 1'b1;
        for (int b = 0; b < 8; b++) at_d[8*b +: 8] = (b < int'(nby)) ? l1_rsp.old[8*b +: 8] : 8'd0;
      end
      S_ACAS: if (l1_rsp_valid && l1_rsp.old[31:0] == am_old[31:0] && dq.rtn) begin
        at_we = 1'b1; at_be = 8'h0f; at_d = {32'd0, am_old[31:0]};
      end
      S_AWRW: if (l1_rsp_valid && !(a_cross && ap == 2'd0) && dq.rtn) begin
        at_we = 1'b1;
        for (int b = 0; b < 8; b++) at_d[8*b +: 8] = (b < int'(nby)) ? am_old[8*b +: 8] : 8'd0;
      end
      S_SALW: if (sh_rsp_valid && dq.rtn) begin
        at_we = 1'b1; at_lane = sal;
        for (int b = 0; b < 8; b++) at_d[8*b +: 8] = (b < int'(nby)) ? sh_rdata[int'(sal) % SH_LPB][8*b +: 8] : 8'd0;
      end
      default: ;
    endcase
  end

  always_comb begin
    bus_en = 1'b0; bus_grp = '0;
    for (int s = 0; s < WL; s++) begin bus_d[s] = '0; bus_be[s] = '0; end
    if (sc_en) begin
      bus_en  = 1'b1;
      bus_grp = WGW'(int'(sc_gbt) / R1);
      for (int s = 0; s < WL; s++)
        if (s / L1_LPB == int'(sc_gbt) % R1) begin
          bus_d[s]  = sc_d[s % L1_LPB];
          bus_be[s] = sc_be[s % L1_LPB];
        end
    end else if (st == S_SHW && sh_rsp_valid && !is_st) begin
      bus_en  = 1'b1;
      bus_grp = WGW'(int'(sh_rgrp) / RS);
      for (int s = 0; s < WL; s++)
        if (s / SH_LPB == int'(sh_rgrp) % RS) begin
          bus_d[s]  = sh_rdata[s % SH_LPB];
          bus_be[s] = {16{sh_rsel[s % SH_LPB]}};
        end
    end else if (at_we) begin
      bus_en  = 1'b1;
      bus_grp = WGW'(int'(at_lane) / WL);
      for (int s = 0; s < WL; s++)
        if (s == int'(at_lane) % WL) begin
          bus_d[s][63:0] = at_d;
          bus_be[s][7:0] = at_be;
        end
    end
  end

  // --------------------------------------------------------------
  // K >= 2: FE and BE completions never coincide (the synchronous class and FE traps run only on a drained ring).
  assign iss_ready  = (st == S_IDLE);
  assign busy       = (st != S_IDLE) || (rcnt != '0) || (bs != B_IDLE);
  assign done_valid = (st == S_DONE) || (bs == B_DONE);
  assign done_tag   = (bs == B_DONE) ? h_tag : tag_q;
  assign trap_valid = (st == S_TRAP) || (bs == B_TRAP);
  assign trap_cause = (bs == B_TRAP) ? CAUSE_MEMFAULT : tr_cause_q;
  assign trap_info  = (bs == B_TRAP) ? be_info : tr_info_q;
  assign cnt_inc_valid = (st == S_IDLE) && iss_valid;
  assign cnt_inc_slot  = iss_slot;
  assign cnt_inc_id    = dec_c.cnt;
  assign cnt_dec_valid = (st == S_DONE) || (st == S_TRAP) || (bs == B_DONE) || (bs == B_TRAP);
  assign cnt_dec_slot  = ((bs == B_DONE) || (bs == B_TRAP)) ? h_slot : slot_q;
  assign cnt_dec_id    = ((bs == B_DONE) || (bs == B_TRAP)) ? h_cnt : dq.cnt;

  // (always_comb, not assign: the functions read module variables that a continuous
  //  assignment would not be sensitive to)
  always_comb begin
    rd_valid = (st == S_VRD) && !vph && vjob_needed(vj);
    rd_reg   = vjob_reg(vj);
  end
  assign rd_slot  = slot_q;
  assign rd_beat  = vb;
  assign rd_hsel  = 2'd0;
  assign wr_valid = ((st == S_WB) || (bs == B_WB)) && (wb_lanes != '0);
  assign wr_slot  = be_wb_act ? h_slot : slot_q;
  assign wr_reg   = be_wb_act ? h_rd + 7'(be_wk) : f_rd + 7'(wk);
  assign wr_beat  = be_wb_act ? be_wbt : wbt;
  assign wr_lanes = wb_lanes;
  assign wr_hsel  = 2'b11;
  assign wr_data  = wb_data;
  function automatic logic ujob_needed(input logic [2:0] j);
    return (j < 3'd4) ? (int'(j) < int'(ucnt)) : ux_need;
  endfunction
  always_comb ur_valid = (st == S_URD) && !uph && ujob_needed(uj);
  assign ur_slot  = slot_q;
  assign ur_reg   = (uj == 3'd4) ? f_rs2[5:0] : ubase_r + 6'(uj);
  assign uw_valid = (st == S_UWB) || (bs == B_UWB);
  assign uw_slot  = be_wb_act ? h_slot : slot_q;
  assign uw_reg   = be_wb_act ? h_rd[5:0] + {1'b0, be_uwk} : f_rd[5:0] + {1'b0, uwk};
  assign uw_data  = uwb_data;
  assign mmu_req_valid  = ((st == S_GMMU) && !memo_hit) || (st == S_AMMU);
  assign mmu_resp_ready = (st == S_GMW) || (st == S_AMW);
  assign mmu_req_vmid   = cq.vmid;
  assign mmu_req_tag    = 8'd0;
  assign mmu_req_atomic = is_at;
  assign mmu_req_acc    = (is_st || is_at) ? 2'd1 : 2'd0;
  assign mmu_req_va     = (st == S_GMMU) ? (gp1 ? gline : ua[gsl]) : a_unit_va;
  // L1D: every request for K = 1; K >= 2: synchronous-class requests, and uc = 0 line requests while the IOF is empty
  // (monotone while S_GISS waits)
  assign l1_req_valid = rq_v && ((st != S_GISS) || (!rq_q.uc && q_empty));
  // uc channel: valid only with a free IOF entry (monotone while S_GISS waits: only the FE pushes)
  assign u_req_valid  = fe_iss_u && !q_full;
  // The line image of a write comes straight from the (stable) merge registers instead of being copied
  // into rq_q (saves 1152 FFs and a 1024-bit mux); for non-writes `be` is 0 and `wdata` is don't care.
  always_comb begin
    l1_req = rq_q;
    l1_req.be    = (rq_q.kind == M_WR) ? g_be : 128'd0;
    l1_req.wdata = g_wdata;
  end
  assign u_req     = l1_req;
  assign u_rd_req  = (bs == B_RD);
  assign u_free    = (bs == B_RD) && u_rd_gnt;
  assign ext_flt_valid = ((st == S_TRAP) && tr_bad_q) || (bs == B_TRAP);
  assign ext_flt_vmid  = (bs == B_TRAP) ? h_vmid : cq.vmid;
  assign ext_flt_va    = (bs == B_TRAP) ? be_va : flt_va64;
  assign ext_flt_acc   = (bs == B_TRAP) ? (h_st ? 2'd1 : 2'd0) : (flt_w ? 2'd1 : 2'd0);

  function automatic mreq_t mk_req(input logic [2:0] kind, input logic [47:0] pa, input logic uc,
                                   input logic noalloc, input logic [1:0] pl);
    mreq_t r;
    r = '0;
    r.kind = kind; r.pa = pa; r.pol = pl; r.uc = uc; r.noalloc = noalloc;
    return r;
  endfunction

  // piece membership of ONE lane (lp), evaluated lane by lane while S_ADDR walks the lanes
  logic [4:0] lp;
  logic       lpv;
  logic [31:0] pre0, pre1;         // pend0 / pend1 of the lanes evaluated so far
  logic       lp_p0, lp_p1;
  ng_lsu_piece u_piece (.a(ua[lp]), .mk((lp == 5'd0 && is_uld) ? vmu : {48'd0, vml[lp]}), .act(act_q[lp]), .nby(nby),
                        .is_at(is_at), .p0(lp_p0), .p1(lp_p1));

  // lowest pending atomic lane
  wire [4:0] afl = 5'(lowest_set({32'd0, pend0}));
  wire [4:0] sfl = 5'(lowest_set({32'd0, sh_rem}));   // lowest remaining shared-atomic lane

  // ---------------------------------------------------------------- operand / data storage (no reset)
  // Large data arrays live outside the reset branch of the FSM: with an asynchronous reset AND a
  // synchronous clear they cost a LUT per bit; as plain registers the clear maps onto the FF reset pin.
  // They are always written (or cleared in S_CHK) before they are read within an instruction.
  // (plain `always`, not always_ff: the arrays are also zero-filled by the `initial` block below)
  always @(posedge clk) begin
    // rb: cleared in S_CHK, except for the asynchronous class (K >= 2), which the BE clears in B_CLR (first slice)
    if ((st == S_CHK && !async_q) || bs == B_CLR)
      for (int i = 0; i < 512; i++) rb[i] <= '0;
    if (st == S_CHK) begin
      for (int l = 0; l < 32; l++) begin rs1v[l] <= '0; rs1h[l] <= '0; end
      for (int k = 0; k < 4; k++) for (int l = 0; l < 32; l++) dv[k][l] <= '0;
      for (int i = 0; i < 4; i++) ud[i] <= '0;
      ub <= '0; uxv <= '0;
    end
    // uniform operands
    if (st == S_URD && ujob_needed(uj) && uph && urd_valid) begin
      if (uj == 3'd4) uxv <= urd_data;
      else begin
        ud[uj[1:0]] <= urd_data;
        if (uj == 3'd0) ub[31:0] <= urd_data;
        if (uj == 3'd1) ub[63:32] <= urd_data;
      end
    end
    // vector operands
    if (st == S_VRD && vjob_needed(vj) && vph && rdv_valid) begin
      for (int i = 0; i < NL; i++) begin
        case (vj)
          3'd0: rs1v[int'(vb) * NL + i] <= rdv_data[32*i +: 32];
          3'd1: rs1h[int'(vb) * NL + i] <= rdv_data[32*i +: 32];
          default: dv[int'(vj) - 2][int'(vb) * NL + i] <= rdv_data[32*i +: 32];
        endcase
      end
    end
    // per-lane addresses / byte-valid masks
    if (st == S_ADDR) begin
      ua[li]  <= is_sh ? {48'd0, rs1v[li][15:0] + shb_imm} : ln_a;
      if (li == 5'd0 && is_uld) vmu <= ln_vm; else vml[li] <= ln_vm[15:0];
    end
    // coalesced line image (stores, line-crossing atomics): clear / merge
    if ((st == S_GSEL && sel_found && pass_c && is_st) ||
        (st == S_ACAL && !(is_fp && fwc != 2'd3) && a_cross) ||
        (st == S_AWRW && l1_rsp_valid && a_cross && ap == 2'd0)) begin
      g_be <= '0; g_wdata <= '0;
    end
    if (st == S_GMRG || st == S_AWB) begin
      for (int p = 0; p < 128; p++)
        if (bm_be[p]) begin
          g_be[p] <= 1'b1;
          g_wdata[8*p +: 8] <= bm_d[8*p +: 8];
        end
    end
    // fetched lines: the load line (K = 1: L1 answer in S_GL1; K >= 2: the BE's fabric read buffer / L1D answer), the
    // atomic lines
    if (st == S_GL1 && l1_rsp_valid && !is_st) gl_line <= l1_rsp.rdata;
    if (bs == B_RD && u_rd_gnt && !be_err) gl_line <= u_rd_data;
    if (be_l1take) gl_line <= l1_rsp.rdata;
    if (st == S_ARDW && l1_rsp_valid) aline[ap[0]] <= l1_rsp.rdata;
    // result bytes from the shared write bus
    if (bus_en)
      for (int s = 0; s < WL; s++)
        for (int j = 0; j < 16; j++)
          if (bus_be[s][j])
            for (int g = 0; g < 32 / WL; g++)
              if (int'(bus_grp) == g) rb[(g * WL + s) * 16 + j] <= bus_d[s][8*j +: 8];
  end

  // simulation start-up: all-zero contents (FPGA init values)
  initial begin
    for (int i = 0; i < 512; i++) rb[i] = '0;
    for (int l = 0; l < 32; l++) begin rs1v[l] = '0; rs1h[l] = '0; ua[l] = '0; vml[l] = '0; end
    for (int k = 0; k < 4; k++) for (int l = 0; l < 32; l++) dv[k][l] = '0;
    for (int i = 0; i < 4; i++) ud[i] = '0;
    ub = '0; uxv = '0; vmu = '0;
    g_be = '0; g_wdata = '0; gl_line = '0;
    for (int i = 0; i < 2; i++) aline[i] = '0;
  end

  // ---------------------------------------------------------------- main FSM (K >= 2: the FE)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE;
      w0_q <= '0; w1_q <= '0; exec_q <= '0; slot_q <= '0; nregs_q <= '0; tag_q <= '0;
      cq <= '0; dq <= '0; async_q <= 1'b0;
      rq_v <= 1'b0; rq_q <= '0;
      tr_cause_q <= '0; tr_info_q <= '0; tr_bad_q <= 1'b0;
      flt_v <= 1'b0; flt_key <= '1; flt_info <= '0; flt_badphys <= 1'b0; flt_va64 <= '0; flt_w <= 1'b0;
      stat_l1_rd <= '0; stat_l1_wr <= '0; stat_l1_amo <= '0; stat_sh_rounds <= '0;
      stat_instr_mem <= '0; stat_groups <= '0;
      sh_req <= 1'b0; sh_pend <= '0; sh_rem <= '0; sh_wr_cmd <= 1'b0; sh_gsel <= '0;
      gbt <= '0; gbr <= '0; shb_imm <= '0; lp <= '0; lpv <= 1'b0; pre0 <= '0; pre1 <= '0;
      pass_c <= 1'b0; pend0 <= '0; pend1 <= '0; gm0 <= '0; gm1 <= '0; gline <= '0; gp1 <= 1'b0; gsl <= '0;
      memo_i <= '0; memo_v <= '0;
      g_pa <= '0; g_sparse <= 1'b0; g_pte <= '0;
      li <= '0; scr_lane <= '0; sl_q <= '0; ns_q <= 3'd1; uj <= '0; uph <= 1'b0; vj <= '0; vb <= '0; vph <= 1'b0;
      wk <= '0; wbt <= '0; uwk <= '0;
      fwc <= '0; acb <= '0;
      al <= '0; ap <= '0; sal <= '0; a_unit_va <= '0; a_cross <= 1'b0; am_old <= '0; a_new <= '0;
      act_q <= '0;
      for (int i = 0; i < 2; i++) begin apa[i] <= '0; asp[i] <= 1'b0; end
    end else begin
      sh_req <= 1'b0;
      case (st)
        // ----------------------------------------------------------
        S_IDLE: begin
          if (iss_valid) begin
            w0_q <= iss_w0; w1_q <= iss_w1; exec_q <= iss_exec; slot_q <= iss_slot; nregs_q <= iss_nregs;
            tag_q <= iss_tag; cq <= iss_ctx; dq <= dec_c;
            async_q <= KN && async_of(dec_c);
            st <= (KN && !async_of(dec_c)) ? S_SYNC : S_CHK;
          end
        end

        // synchronous class (K >= 2): wait until every earlier instruction has retired
        S_SYNC: if (drained) st <= S_CHK;

        S_CHK: begin
          mreq_t t;
          flt_v <= 1'b0; flt_key <= '1; flt_info <= '0; flt_badphys <= 1'b0; flt_w <= is_st || is_at;
          flt_va64 <= '0;
          pass_c <= 1'b0;
          sl_q <= '0; ns_q <= 3'd1;
          memo_i <= '0; memo_v <= '0;
          act_q <= is_uld ? 32'd1 : exec_q;
          uj <= '0; uph <= 1'b0; vj <= '0; vb <= '0; vph <= 1'b0;
          if (pt_v) begin
            tr_cause_q <= pt_cause; tr_info_q <= pt_info; tr_bad_q <= 1'b0;
            st <= async_q ? S_TWAIT : S_TRAP;
          end else if (dq.kind == K_CACHE) begin
            t = mk_req((dq.cache_op == CO_INV_L1) ? M_INV_L1 : (dq.cache_op == CO_INV_L2) ? M_INV_L2 : M_WB_L2,
                       '0, 1'b0, 1'b0, 2'd0);
            rq_q <= t;
            rq_v <= 1'b1;
            st <= S_CACHE;
          end else begin
            st <= S_URD;
          end
        end

        S_CACHE: begin
          if (rq_v && l1_req_ready) rq_v <= 1'b0;
          if (l1_rsp_valid) st <= S_DONE;
        end

        // ------------------------------------------------------------ uniform operands
        S_URD: begin
          if (!ujob_needed(uj)) begin
            if (uj == 3'd4) begin st <= S_VRD; vj <= '0; vb <= '0; vph <= 1'b0; end
            else uj <= uj + 3'd1;
          end else if (!uph) begin
            uph <= 1'b1;
          end else if (urd_valid) begin
            uph <= 1'b0;
            if (uj == 3'd4) begin st <= S_VRD; vj <= '0; vb <= '0; vph <= 1'b0; uj <= '0; end
            else uj <= uj + 3'd1;
          end
        end

        // ------------------------------------------------------------ vector operands
        S_VRD: begin
          if (!vjob_needed(vj)) begin
            if (vj == 3'd5) st <= S_PREP; else vj <= vj + 3'd1;
          end else if (!vph) begin
            if (rd_ready) vph <= 1'b1;
          end else if (rdv_valid) begin
            vph <= 1'b0;
            if (int'(vb) == NBEAT - 1) begin
              vb <= '0;
              if (vj == 3'd5) st <= S_PREP; else vj <= vj + 3'd1;
            end else begin
              vb <= vb + BTW'(1);
            end
          end
        end

        S_PREP: begin
          // scratch: warp region R = base + gslot * 32 * bytes, shifted for the slice (+124 per slice)
          scr_lane <= {16'd0, cq.scr_base} + ((64'(cq.gslot) * 64'(cq.scr_bytes)) << 5) + {54'd0, 10'(sl_q) * 10'd124};
          shb_imm <= cq.sh_base + f_imm[15:0];
          li <= '0;
          lpv <= 1'b0; pre0 <= '0; pre1 <= '0;
          st <= S_ADDR;
        end

        // ------------------------------------------------------------ per-lane addresses
        S_ADDR: begin
          // (ua / vml / vmu are written in the storage block above)
          // the line-piece membership of the PREVIOUS lane is evaluated one lane behind (one shared evaluator)
          if (lpv) begin pre0[lp] <= lp_p0; pre1[lp] <= lp_p1; end
          lp <= li; lpv <= 1'b1;
          if (is_sh && act_q[li] && ln_sh_bad && !flt_v) begin
            flt_v <= 1'b1; flt_info <= ln_off[31:0];
          end
          scr_lane <= scr_lane + 64'd4;   // next lane's dword (interleaved scratch)
          if (is_scr && !pass_c && sl_q == 3'd0 && act_q[li] && ln_ns > ns_q) ns_q <= ln_ns;
          if (li == 5'd31 || is_uld) st <= S_PINIT;
          li <= li + 5'd1;
        end

        S_PINIT: begin   // last lane's piece membership
          pre0[lp] <= lp_p0; pre1[lp] <= lp_p1;
          st <= S_PIN2;
        end

        S_PIN2: begin
          if (is_sh) begin
            if (flt_v) begin
              tr_cause_q <= CAUSE_SHARED; tr_info_q <= flt_info; tr_bad_q <= 1'b0; st <= S_TRAP;
            end else begin
              st <= S_E6;
            end
          end else begin
            pend0 <= pre0; pend1 <= pre1;
            if (!is_at && !pass_c && sl_q == 3'd0) stat_instr_mem <= stat_instr_mem + 1;
            // commit pass of a scratch slice > 0 (or of slice 0 of a sliced access): its own ring entry first
            st <= is_at ? S_ASEL : ((async_q && pass_c) ? S_RALLOC : S_GSEL);
          end
        end

        // asynchronous class (K >= 2): one ring entry per (scratch slice of the) commit pass
        S_RALLOC: begin
          if (fe_alloc) begin
            pend0 <= pre0; pend1 <= pre1;
            st <= S_GSEL;
          end
        end

        // ------------------------------------------------------------ line groups
        S_GSEL: begin
          if (!sel_found) begin
            if (is_scr && sl_q + 3'd1 < ns_q) begin
              // next scratch slice of the same pass
              sl_q <= sl_q + 3'd1;
              st <= S_PREP;
            end else if (!pass_c) begin
              if (flt_v) begin
                tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= flt_info; tr_bad_q <= flt_badphys;
                st <= async_q ? S_TWAIT : S_TRAP;
              end else begin
                st <= S_E6;
              end
            end else if (async_q) begin
              st <= S_IDLE;                 // every request issued; the BE completes the instruction
            end else begin
              wk <= '0; wbt <= '0; uwk <= '0;
              st <= is_st ? S_DONE : (is_uld ? S_UWB : S_WB);
            end
          end else begin
            gm0 <= sel_m0; gm1 <= sel_m1; gline <= sel_line; gp1 <= sel_p1; gsl <= sel_l;
            gbr <= gb_new; gbt <= B1W'(lowest_set(gb_new));
            if (pass_c && is_st) begin
              st <= S_GMRG;
            end else begin
              st <= S_GMMU;
            end
          end
        end

        S_GMMU: begin
          if (memo_hit) begin
            // commit pass: the pass-0 answer (registered read of the memo, decided in S_GMH)
            g_pa <= {memo_q[43:3], 7'd0}; g_sparse <= memo_q[2]; g_pte <= {57'd0, memo_q[1:0], 5'd0};
            st <= S_GMH;
          end else if (mmu_req_ready) begin
            st <= S_GMW;
          end
        end

        // memo hit: same continuation as a fault-free S_GMW answer of the commit pass
        S_GMH: begin
          memo_i <= memo_i + 7'd1;
          if (g_sparse) begin
            pend0 <= pend0 & ~gm0; pend1 <= pend1 & ~gm1;
            st <= S_GSEL;
          end else begin
            st <= S_GEXE;
          end
        end

        S_GMW: begin
          if (mmu_resp_valid) begin
            g_pa <= mmu_resp_pa; g_sparse <= mmu_resp_sparse; g_pte <= mmu_resp_pte;
            if (memo_i != 7'd127) memo_i <= memo_i + 7'd1;
            if (!pass_c && memo_i < 7'(MEMO_N)) begin
              memo_d[memo_i[MEMO_IW-1:0]] <= {mmu_resp_pa[47:7], mmu_resp_sparse, mmu_resp_pte[6:5]};
              memo_v[memo_i[MEMO_IW-1:0]] <= 1'b1;
            end
            if (mmu_resp_fault != 3'd0 || (!mmu_resp_sparse && !pa_valid({mmu_resp_pa[47:7], 7'd0}))) begin
              if (!pass_c && gkq_v && (!flt_v || gkq_key < flt_key)) begin
                flt_v <= 1'b1; flt_key <= gkq_key; flt_info <= gkq_info;
                flt_badphys <= (mmu_resp_fault == 3'd0);
                flt_va64 <= gkq_ua + {58'd0, gkq_boff};
              end
              pend0 <= pend0 & ~gm0; pend1 <= pend1 & ~gm1;
              st <= S_GSEL;
            end else if (!pass_c || mmu_resp_sparse) begin
              pend0 <= pend0 & ~gm0; pend1 <= pend1 & ~gm1;
              st <= S_GSEL;
            end else begin
              st <= S_GEXE;
            end
          end
        end

        S_GEXE: begin
          mreq_t t;
          t = mk_req(is_st ? M_WR : M_RD, {g_pa[47:7], 7'd0}, !pa_is_vram(g_pa),
                     (g_pte[6:5] == 2'd1) || (g_pte[6:5] == 2'd2), pol);
          rq_q <= t;
          rq_v <= 1'b1;
          st <= KN ? S_GISS : S_GL1;
        end

        // K = 1: issue the line request and wait for its answer
        S_GL1: begin
          if (rq_v && l1_req_ready) begin
            rq_v <= 1'b0;
            if (is_st) stat_l1_wr <= stat_l1_wr + 1; else stat_l1_rd <= stat_l1_rd + 1;
            stat_groups <= stat_groups + 1;
          end
          if (l1_rsp_valid && l1_rsp.err != 2'd0) begin
            // fabric error response (e.g. PA the RM cannot serve): BAD_PHYS_ADDR memory fault,
            // info = faulting VA[31:0] of the first lane of the group; no register writeback
            tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= gkq_info; tr_bad_q <= 1'b1;
            flt_va64 <= gkq_ua + {58'd0, gkq_boff};
            st <= S_TRAP;
          end else if (l1_rsp_valid) begin
            pend0 <= pend0 & ~gm0; pend1 <= pend1 & ~gm1;
            if (is_st) begin
              st <= S_GSEL;
            end else begin
              st <= S_GSC;
            end
          end
        end

        // K >= 2: issue the line request (uc channel or L1D) and go on without waiting for the answer
        S_GISS: begin
          if (fe_iss_hs) begin
            rq_v <= 1'b0;
            if (is_st) stat_l1_wr <= stat_l1_wr + 1; else stat_l1_rd <= stat_l1_rd + 1;
            stat_groups <= stat_groups + 1;
            pend0 <= pend0 & ~gm0; pend1 <= pend1 & ~gm1;
            st <= S_GSEL;
          end
        end

        // store merge: one beat of L1_LPB lanes per cycle into the line image
        S_GMRG: begin
          gbr <= gbr & ~(NB1'(1) << gbt);
          gbt <= B1W'(lowest_set(gbr & ~(NB1'(1) << gbt)));
          if ((gbr & ~(NB1'(1) << gbt)) == '0) st <= S_GMMU;
        end

        // K = 1 load scatter: one beat of L1_LPB lanes per cycle into rb (via the write bus)
        S_GSC: begin
          gbr <= gbr & ~(NB1'(1) << gbt);
          gbt <= B1W'(lowest_set(gbr & ~(NB1'(1) << gbt)));
          if ((gbr & ~(NB1'(1) << gbt)) == '0) st <= S_GSEL;
        end

        // ------------------------------------------------------------ E6 (destination range), then commit
        S_E6: begin
          if (e6_v) begin
            tr_cause_q <= CAUSE_REGRANGE; tr_info_q <= e6_info; tr_bad_q <= 1'b0;
            st <= async_q ? S_TWAIT : S_TRAP;
          end else if (is_sh) begin
            sh_pend <= act_q;
            sh_rem <= act_q;
            if (is_at) st <= S_SAL; else st <= S_SH;
          end else if (is_scr && ns_q != 3'd1) begin
            // sliced scratch: the commit pass re-walks slice 0.. (ua / vml hold the last probe slice)
            pass_c <= 1'b1;
            memo_i <= '0;
            sl_q <= '0;
            st <= S_PREP;
          end else begin
            pass_c <= 1'b1;
            memo_i <= '0;
            pend0 <= pre0; pend1 <= pre1;
            st <= is_at ? S_ASEL : (async_q ? S_RALLOC : S_GSEL);
          end
        end

        // FE trap of an asynchronous instruction (K >= 2): after every earlier instruction has retired
        S_TWAIT: if (drained) st <= S_TRAP;

        // ------------------------------------------------------------ global atomics (lane by lane)
        S_ASEL: begin
          if (pend0 == 32'd0) begin
            if (!pass_c) begin
              if (flt_v) begin
                tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= flt_info; tr_bad_q <= flt_badphys; st <= S_TRAP;
              end else begin
                st <= S_E6;
              end
            end else begin
              wk <= '0; wbt <= '0;
              st <= dq.rtn ? S_WB : S_DONE;
            end
          end else begin
            al <= afl;
            ap <= 2'd0;
            a_cross <= ({1'b0, ua[afl][6:0]} + {1'b0, nby}) > LINE_B8;
            a_unit_va <= ua[afl];
            st <= S_AMMU;
          end
        end

        S_AMMU: begin
          if (mmu_req_ready) st <= S_AMW;
        end

        S_AMW: begin
          if (mmu_resp_valid) begin
            logic bad;
            bad = (mmu_resp_fault != 3'd0) || (!mmu_resp_sparse && !pa_valid({mmu_resp_pa[47:7], 7'd0}));
            apa[ap[0]] <= mmu_resp_pa; asp[ap[0]] <= mmu_resp_sparse;
            if (bad) begin
              if (!pass_c && (!flt_v || ({al, 6'd0} < flt_key))) begin
                flt_v <= 1'b1; flt_key <= {al, 6'd0}; flt_info <= ua[al][31:0];
                flt_badphys <= (mmu_resp_fault == 3'd0); flt_va64 <= ua[al];
              end
              pend0[al] <= 1'b0;
              st <= S_ASEL;
            end else if (a_cross && ap == 2'd0) begin
              ap <= 2'd1;
              a_unit_va <= (ua[al] & ~64'd127) + 64'd128;
              st <= S_AMMU;
            end else if (!pass_c) begin
              pend0[al] <= 1'b0;
              st <= S_ASEL;
            end else if (asp[0] || mmu_resp_sparse) begin
              pend0[al] <= 1'b0;           // sparse-null: returns 0, write dropped
              st <= S_ASEL;
            end else if (!a_cross && !is_fp) begin
              st <= S_AEXE;
            end else begin
              ap <= 2'd0;
              st <= S_ARD;
            end
          end
        end

        S_AEXE: begin
          mreq_t t;
          t = mk_req(M_AMO, apa[0], !pa_is_vram(apa[0]), 1'b0, pol);
          t.amo_op = dq.amo; t.amo64 = dq.w64; t.amo_rtn = dq.rtn;
          t.amo_a = (dq.amo == ng_noc_pkg::AMO_CAS) ? op_v2 : op_v;
          t.amo_cmp = op_v;
          rq_q <= t;
          rq_v <= 1'b1;
          st <= S_AL1;
        end

        S_AL1: begin
          if (rq_v && l1_req_ready) begin rq_v <= 1'b0; stat_l1_amo <= stat_l1_amo + 1; end
          if (l1_rsp_valid && l1_rsp.err != 2'd0) begin
            tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= ua[al][31:0]; tr_bad_q <= 1'b1; flt_va64 <= ua[al]; st <= S_TRAP;
          end else if (l1_rsp_valid) begin
            pend0[al] <= 1'b0;
            st <= S_ASEL;
          end
        end

        // local read-modify-write: FP32 atomics on global memory (CAS loop), line-crossing atomics
        S_ARD: begin
          mreq_t t;
          t = mk_req(M_RD, {apa[ap[0]][47:7], 7'd0}, !pa_is_vram(apa[ap[0]]), 1'b1, 2'd3);   // sys: from L2
          rq_q <= t;
          rq_v <= 1'b1;
          st <= S_ARDW;
        end

        S_ARDW: begin
          if (rq_v && l1_req_ready) begin rq_v <= 1'b0; stat_l1_rd <= stat_l1_rd + 1; end
          if (l1_rsp_valid && l1_rsp.err != 2'd0) begin
            tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= ua[al][31:0]; tr_bad_q <= 1'b1; flt_va64 <= ua[al]; st <= S_TRAP;
          end else if (l1_rsp_valid) begin
            if (a_cross && ap == 2'd0) begin ap <= 2'd1; st <= S_ARD; end
            else begin acb <= '0; st <= S_ACMP; end
          end
        end

        // serial read of the old value (8 bytes, one per cycle) out of the one or two fetched lines
        S_ACMP: begin
          logic [7:0] idx;
          logic [7:0] byt;
          idx = {1'b0, apa[0][6:0]} + {5'd0, acb};
          byt = (idx < LINE_B8) ? aline[0][8*int'(idx[6:0]) +: 8] : aline[1][8*int'(idx[6:0]) +: 8];
          if (acb == 3'd0) am_old <= {56'd0, byt};
          else am_old[8*int'(acb) +: 8] <= (int'(acb) < int'(nby)) ? byt : 8'd0;
          if (acb == 3'd7) begin acb <= '0; fwc <= '0; st <= S_ACAL; end
          else acb <= acb + 3'd1;
        end

        S_ACAL: begin
          if (is_fp && fwc != 2'd3) begin
            fwc <= fwc + 2'd1;                    // FP32 ALU pipeline settles (3 edges after am_old)
          end else begin
            fwc <= '0;
            a_new <= res_new;
            if (!a_cross) begin
              st <= S_ACAI;
            end else begin
              ap <= 2'd0;
              st <= S_AWB;
            end
          end
        end

        S_ACAI: begin
          // CAS at L2: compare old, swap new
          mreq_t t;
          t = mk_req(M_AMO, apa[0], !pa_is_vram(apa[0]), 1'b0, pol);
          t.amo_op = ng_noc_pkg::AMO_CAS; t.amo64 = 1'b0; t.amo_rtn = 1'b1;
          t.amo_a = {32'd0, a_new[31:0]};
          t.amo_cmp = {32'd0, am_old[31:0]};
          rq_q <= t;
          rq_v <= 1'b1;
          st <= S_ACAS;
        end

        S_ACAS: begin
          if (rq_v && l1_req_ready) begin rq_v <= 1'b0; stat_l1_amo <= stat_l1_amo + 1; end
          if (l1_rsp_valid && l1_rsp.err != 2'd0) begin
            tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= ua[al][31:0]; tr_bad_q <= 1'b1; flt_va64 <= ua[al]; st <= S_TRAP;
          end else if (l1_rsp_valid) begin
            if (l1_rsp.old[31:0] == am_old[31:0]) begin
              pend0[al] <= 1'b0;
              st <= S_ASEL;
            end else begin
              am_old <= {32'd0, l1_rsp.old[31:0]};   // lost a race: retry on the observed value
              st <= S_ACAL;
            end
          end
        end

        // byte image of the new value for piece ap: lane 0 of the store merge (rotation by the line offset)
        S_AWB: begin
          st <= S_AWR;
        end

        S_AWR: begin   // write the new bytes back, line by line
          mreq_t t;
          t = mk_req(M_WR, {apa[ap[0]][47:7], 7'd0}, !pa_is_vram(apa[ap[0]]), 1'b0, pol);
          rq_q <= t;
          rq_v <= 1'b1;
          st <= S_AWRW;
        end

        S_AWRW: begin
          if (rq_v && l1_req_ready) begin rq_v <= 1'b0; stat_l1_wr <= stat_l1_wr + 1; end
          if (l1_rsp_valid && l1_rsp.err != 2'd0) begin
            tr_cause_q <= CAUSE_MEMFAULT; tr_info_q <= ua[al][31:0]; tr_bad_q <= 1'b1; flt_va64 <= ua[al]; st <= S_TRAP;
          end else if (l1_rsp_valid) begin
            if (a_cross && ap == 2'd0) begin ap <= 2'd1; st <= S_AWB; end
            else begin
              pend0[al] <= 1'b0;
              st <= S_ASEL;
            end
          end
        end

        // ------------------------------------------------------------ shared memory
        S_SH: begin
          if (sh_pend == 32'd0) begin
            wk <= '0; wbt <= '0;
            st <= is_st ? S_DONE : S_WB;
          end else begin
            sh_wr_cmd <= is_st;
            sh_gsel <= sh_fg;
            sh_req <= 1'b1;
            st <= S_SHW;
          end
        end

        S_SHW: begin
          if (sh_rsp_valid) begin
            stat_sh_rounds <= stat_sh_rounds + 1;
            sh_pend <= sh_pend & ~sh_sel;
            st <= S_SH;
          end
        end

        S_SAL: begin   // atomic: load round of the lowest remaining lane
          if (sh_rem == 32'd0) begin
            wk <= '0; wbt <= '0;
            st <= dq.rtn ? S_WB : S_DONE;
          end else begin
            sal <= sfl;
            sh_wr_cmd <= 1'b0;
            sh_pend <= (32'd1 << sfl);
            sh_gsel <= SGW'(int'(sfl) / SH_LPB);
            sh_req <= 1'b1;
            st <= S_SALW;
          end
        end

        S_SALW: begin
          if (sh_rsp_valid) begin
            stat_sh_rounds <= stat_sh_rounds + 1;
            begin
              logic [63:0] o;
              o = '0;
              for (int b = 0; b < 8; b++) if (b < int'(nby)) o[8*b +: 8] = sh_rdata[int'(sal) % SH_LPB][8*b +: 8];
              am_old <= o;
            end
            st <= S_SAC;
          end
        end

        S_SAC: begin   // am_old is registered; FP32 results need the ALU pipeline to settle (3 edges)
          if (is_fp && fwc != 2'd3) begin
            fwc <= fwc + 2'd1;
          end else begin
            fwc <= '0;
            a_new <= res_new;
            st <= S_SAW;
          end
        end

        S_SAW: begin   // result registered in a_new: send the write round
          sh_wr_cmd <= 1'b1;
          sh_req <= 1'b1;
          st <= S_SASW;
        end

        S_SASW: begin
          if (sh_rsp_valid) begin
            stat_sh_rounds <= stat_sh_rounds + 1;
            sh_rem <= sh_rem & ~(32'd1 << sal);
            st <= S_SAL;
          end
        end

        // ------------------------------------------------------------ writeback (K >= 2: synchronous class)
        S_WB: begin
          if (wb_lanes == '0 || wr_ready) begin
            if (int'(wbt) == NBEAT - 1) begin
              wbt <= '0;
              if (int'(wk) == int'(dq.ndst) - 1) st <= S_DONE; else wk <= wk + 3'd1;
            end else begin
              wbt <= wbt + BTW'(1);
            end
          end
        end

        S_UWB: begin
          if (int'(uwk) == int'(nby[6:2]) - 1) st <= S_DONE; else uwk <= uwk + 5'd1;
        end

        S_DONE: st <= S_IDLE;
        S_TRAP: st <= S_IDLE;
        default: st <= S_IDLE;
      endcase
    end
  end

`ifndef SYNTHESIS
  // K >= 2 simulation-only statistics (TB: ng_lsu_top sim_*, written by g_ring; 0 for K = 1)
  int unsigned sim_ring_max = 0;      // largest ring occupancy
  int unsigned sim_iof_max  = 0;      // largest IOF occupancy
  longint unsigned sim_ring_occ = 0;  // sum over cycles of the ring occupancy
  longint unsigned sim_cyc = 0;       // cycles with anything in the LSU
  int unsigned sim_fe_busy = 0;       // FE not idle
  int unsigned sim_ralloc = 0;        // FE waiting for a ring entry
  int unsigned sim_iss_stall = 0;     // FE request waiting for the fabric / L1D / IOF
  int unsigned sim_sync = 0;          // synchronous instruction waiting for the drain
  int unsigned sim_twait = 0;         // FE trap waiting for the drain
  int unsigned sim_be_wait = 0;       // BE waiting for an answer of the head
  int unsigned sim_late_err = 0;      // late (fabric) error traps
  int unsigned sim_err = 0;           // invariant violations
`endif

  if (!KN) begin : g_one
    // ================================================================ K = 1: no ring / IOF / BE
    assign bs = B_IDLE; assign rcnt = '0; assign q_empty = 1'b1; assign q_full = 1'b0;
    assign be_err = 1'b0; assign be_info = '0; assign be_va = '0; assign be_gbt = '0;
    assign be_wk = '0; assign be_wbt = '0; assign be_uwk = '0; assign be_l1take = 1'b0;
    assign h_slot = '0; assign h_tag = '0; assign h_exec = '0; assign h_rd = '0; assign h_st = 1'b0;
    assign h_sext = 1'b0; assign h_nby = '0; assign h_cnt = '0; assign h_vmid = '0;
    assign sc_en  = (st == S_GSC);
    assign sc_gbt = gbt;
    always_comb for (int i = 0; i < L1_LPB; i++) begin sc_off[i] = bl_off[i]; sc_be[i] = bl_be[i]; end
    assign u_rd_slot = '0;
    assign stat_kn = '0;
  end else begin : g_ring
    // ================================================================ ring (K instruction contexts)
    // FE writes an entry at S_RALLOC (data arrays: no reset, written before use); BE reads the head (rh).
    logic [SLOTW-1:0] r_slot [K];
    logic [15:0]      r_tag  [K];
    logic [31:0]      r_exec [K];
    logic [6:0]       r_rd   [K];
    logic             r_st   [K];
    logic             r_uld  [K];
    logic             r_sext [K];
    logic             r_cont [K];      // continuation slice: rb is not cleared
    logic             r_last [K];      // last (or only) slice: writes back, completes / traps
    logic [6:0]       r_nby  [K];
    logic [2:0]       r_ndst [K];
    logic [2:0]       r_cnt  [K];
    logic [3:0]       r_vmid [K];
    logic [6:0]       r_off  [K][32];  // per-lane line offset ua[6:0]
    logic [15:0]      r_vml  [K][32];
    logic [63:0]      r_vmu  [K];
    logic [K-1:0]     r_fed;           // FE has issued every line request of the entry
    logic [KW-1:0]    rh, rt;
    logic [KW-1:0]    fe_ctx;          // ring entry of the FE's commit pass

    // ================================================================ IOF (line requests in issue order)
    logic [KW-1:0]    q_ctx  [NQ];
    logic [MSW-1:0]   q_slot [NQ];
    logic             q_l1   [NQ];     // through the L1D (uc = 0), else the uc channel
    logic             q_rd   [NQ];
    logic [31:0]      q_gm0  [NQ];
    logic [31:0]      q_gm1  [NQ];
    logic [NB1-1:0]   q_gbr  [NQ];
    logic [31:0]      q_info [NQ];     // trap info of the group (late error)
    logic [63:0]      q_va   [NQ];     // ext_flt_va of the group
    logic [NQ-1:0]    q_done;
    logic [1:0]       q_err  [NQ];
    logic [QW-1:0]    qh, qt;
    logic [QW:0]      qcnt;
    logic [QW-1:0]    s2q [NSL];       // fabric slot -> IOF entry
    logic             l1p;             // an IOF entry waits for the L1D
    logic [QW-1:0]    l1q;

    assign q_empty = (qcnt == '0);
    assign q_full  = (qcnt == (QW + 1)'(NQ));
    wire q_mine    = !q_empty && (q_ctx[qh] == rh) && (rcnt != '0);   // IOF head belongs to the ring head

    // BE events
    logic [NB1-1:0] be_gbr;
    logic [31:0]    be_gm0, be_gm1;
    wire be_hd_done = (bs == B_HEAD) && q_mine && q_done[qh];
    assign be_l1take = be_hd_done && q_l1[qh] && q_rd[qh] && (q_err[qh] == 2'd0) && !be_err;
    wire q_pop      = (be_hd_done && !(q_rd[qh] && !q_l1[qh])) || ((bs == B_RD) && u_rd_gnt);
    wire be_retire  = (bs == B_DONE) || (bs == B_TRAP) ||
                      ((bs == B_HEAD) && !q_mine && r_fed[rh] && !r_last[rh]);

    assign h_slot = r_slot[rh]; assign h_tag = r_tag[rh]; assign h_exec = r_exec[rh]; assign h_rd = r_rd[rh];
    assign h_st = r_st[rh]; assign h_sext = r_sext[rh]; assign h_nby = r_nby[rh]; assign h_cnt = r_cnt[rh];
    assign h_vmid = r_vmid[rh];
    assign u_rd_slot = q_slot[qh];

    // ---------------------------------------------------------------- BE data path: scatter of the head's group
    logic [6:0]   be_off7 [32];
    logic [15:0]  be_vml  [32];
    always_comb for (int l = 0; l < 32; l++) begin be_off7[l] = r_off[rh][l]; be_vml[l] = r_vml[rh][l]; end
    logic [6:0]   bb_off [L1_LPB];
    logic [15:0]  bb_be  [L1_LPB];
    ng_lsu_beat #(.L1_LPB(L1_LPB)) u_bbeat (.gbt(be_gbt), .off7(be_off7), .vml(be_vml), .vmu(r_vmu[rh]), .gm0(be_gm0),
                                            .gm1(be_gm1), .nby(r_nby[rh]), .is_uld(r_uld[rh]), .bl_off(bb_off),
                                            .bl_be(bb_be));
    assign sc_en  = (bs == B_SC);
    assign sc_gbt = be_gbt;
    always_comb for (int i = 0; i < L1_LPB; i++) begin sc_off[i] = bb_off[i]; sc_be[i] = bb_be[i]; end

    // ---------------------------------------------------------------- ring / IOF entries (no reset)
    always @(posedge clk) begin
      // ring entry (S_RALLOC)
      if (fe_alloc) begin
        r_slot[rt] <= slot_q; r_tag[rt] <= tag_q; r_exec[rt] <= exec_q; r_rd[rt] <= f_rd;
        r_st[rt] <= is_st; r_uld[rt] <= is_uld; r_sext[rt] <= dq.sext; r_nby[rt] <= nby; r_ndst[rt] <= dq.ndst;
        r_cnt[rt] <= dq.cnt; r_vmid[rt] <= cq.vmid;
        r_cont[rt] <= is_scr && (sl_q != 3'd0);
        r_last[rt] <= !is_scr || (sl_q + 3'd1 >= ns_q);
        for (int l = 0; l < 32; l++) begin r_off[rt][l] <= ua[l][6:0]; r_vml[rt][l] <= vml[l]; end
        r_vmu[rt] <= vmu;
      end
      // IOF entry (S_GISS handshake)
      if (fe_iss_hs) begin
        q_ctx[qt] <= fe_ctx; q_slot[qt] <= u_req_slot; q_l1[qt] <= !rq_q.uc; q_rd[qt] <= !is_st;
        q_gm0[qt] <= gm0; q_gm1[qt] <= gm1; q_gbr[qt] <= gbr; q_info[qt] <= gkq_info;
        q_va[qt] <= gkq_ua + {58'd0, gkq_boff};
      end
    end

    initial begin
      for (int e = 0; e < K; e++) begin
        r_slot[e] = '0; r_tag[e] = '0; r_exec[e] = '0; r_rd[e] = '0; r_st[e] = 1'b0; r_uld[e] = 1'b0; r_sext[e] = 1'b0;
        r_cont[e] = 1'b0; r_last[e] = 1'b1; r_nby[e] = '0; r_ndst[e] = '0; r_cnt[e] = '0; r_vmid[e] = '0; r_vmu[e] = '0;
        for (int l = 0; l < 32; l++) begin r_off[e][l] = '0; r_vml[e][l] = '0; end
      end
      for (int e = 0; e < NQ; e++) begin
        q_ctx[e] = '0; q_slot[e] = '0; q_l1[e] = 1'b0; q_rd[e] = 1'b0; q_gm0[e] = '0; q_gm1[e] = '0; q_gbr[e] = '0;
        q_info[e] = '0; q_va[e] = '0;
      end
    end

    // ================================================================ ring / IOF bookkeeping
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        rh <= '0; rt <= '0; rcnt <= '0; r_fed <= '0; fe_ctx <= '0;
        qh <= '0; qt <= '0; qcnt <= '0; q_done <= '0; l1p <= 1'b0; l1q <= '0;
        for (int e = 0; e < NQ; e++) q_err[e] <= '0;
        for (int s = 0; s < NSL; s++) s2q[s] <= '0;
      end else begin
        if (fe_alloc) begin
          rt <= rt + 1'b1; fe_ctx <= rt; r_fed[rt] <= 1'b0;
        end
        if (fe_fdone) r_fed[fe_ctx] <= 1'b1;
        if (be_retire) rh <= rh + 1'b1;
        rcnt <= rcnt + (KW + 1)'(fe_alloc) - (KW + 1)'(be_retire);
        // IOF
        if (fe_iss_hs) begin
          qt <= qt + 1'b1;
          q_done[qt] <= 1'b0; q_err[qt] <= 2'd0;
          if (rq_q.uc) s2q[u_req_slot] <= qt;
          else begin l1p <= 1'b1; l1q <= qt; end
        end
        if (q_pop) qh <= qh + 1'b1;
        qcnt <= qcnt + (QW + 1)'(fe_iss_hs) - (QW + 1)'(q_pop);
        if (u_rsp_valid) begin
          q_done[s2q[u_rsp_slot]] <= 1'b1; q_err[s2q[u_rsp_slot]] <= u_rsp_err;
        end
        if (l1_rsp_valid && l1p) begin
          q_done[l1q] <= 1'b1; q_err[l1q] <= l1_rsp.err; l1p <= 1'b0;
        end
      end
    end

    // ================================================================ BE (acceptance order)
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        bs <= B_IDLE;
        be_err <= 1'b0; be_info <= '0; be_va <= '0;
        be_gbt <= '0; be_gbr <= '0; be_gm0 <= '0; be_gm1 <= '0;
        be_wk <= '0; be_wbt <= '0; be_uwk <= '0;
      end else begin
        case (bs)
          B_IDLE: if (rcnt != '0) bs <= r_cont[rh] ? B_HEAD : B_CLR;
          B_CLR:  bs <= B_HEAD;
          B_HEAD: begin
            if (q_mine) begin
              if (q_done[qh]) begin
                if (q_err[qh] != 2'd0 && !be_err) begin be_err <= 1'b1; be_info <= q_info[qh]; be_va <= q_va[qh]; end
                if (q_rd[qh] && !q_l1[qh]) begin
                  bs <= B_RD;
                end else if (be_l1take) begin
                  be_gm0 <= q_gm0[qh]; be_gm1 <= q_gm1[qh]; be_gbr <= q_gbr[qh]; be_gbt <= B1W'(lowest_set(q_gbr[qh]));
                  if (q_gbr[qh] != '0) bs <= B_SC;
                end
              end
            end else if (r_fed[rh]) begin
              if (!r_last[rh])  bs <= B_IDLE;        // earlier scratch slice: retire silently (error carried)
              else if (be_err)  bs <= B_TRAP;
              else if (r_st[rh]) bs <= B_DONE;
              else begin
                be_wk <= '0; be_wbt <= '0; be_uwk <= '0;
                bs <= r_uld[rh] ? B_UWB : B_WB;
              end
            end
          end
          B_RD: begin
            if (u_rd_gnt) begin
              if (!be_err) begin
                be_gm0 <= q_gm0[qh]; be_gm1 <= q_gm1[qh]; be_gbr <= q_gbr[qh]; be_gbt <= B1W'(lowest_set(q_gbr[qh]));
                bs <= (q_gbr[qh] != '0) ? B_SC : B_HEAD;
              end else begin
                bs <= B_HEAD;
              end
            end
          end
          B_SC: begin
            be_gbr <= be_gbr & ~(NB1'(1) << be_gbt);
            be_gbt <= B1W'(lowest_set(be_gbr & ~(NB1'(1) << be_gbt)));
            if ((be_gbr & ~(NB1'(1) << be_gbt)) == '0) bs <= B_HEAD;
          end
          B_WB: begin
            if (wb_lanes == '0 || wr_ready) begin
              if (int'(be_wbt) == NBEAT - 1) begin
                be_wbt <= '0;
                if (int'(be_wk) == int'(r_ndst[rh]) - 1) bs <= B_DONE; else be_wk <= be_wk + 3'd1;
              end else begin
                be_wbt <= be_wbt + BTW'(1);
              end
            end
          end
          B_UWB: begin
            if (int'(be_uwk) == int'(r_nby[rh][6:2]) - 1) bs <= B_DONE; else be_uwk <= be_uwk + 5'd1;
          end
          B_DONE: begin be_err <= 1'b0; bs <= B_IDLE; end
          B_TRAP: begin be_err <= 1'b0; bs <= B_IDLE; end
          default: bs <= B_IDLE;
        endcase
      end
    end

    // ---------------------------------------------------------------- debug counters
    logic [31:0] c_fe, c_ra, c_is, c_sy, c_mx, c_bw, c_ws, c_le;
    logic [47:0] c_oc;
    always_ff @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        c_fe <= '0; c_ra <= '0; c_is <= '0; c_sy <= '0; c_mx <= '0; c_bw <= '0; c_ws <= '0; c_le <= '0; c_oc <= '0;
      end else begin
        if (st != S_IDLE) c_fe <= c_fe + 32'd1;
        if (st == S_RALLOC && !fe_alloc) c_ra <= c_ra + 32'd1;
        if (st == S_GISS && !fe_iss_hs) c_is <= c_is + 32'd1;
        if ((st == S_SYNC || st == S_TWAIT) && !drained) c_sy <= c_sy + 32'd1;
        c_oc <= c_oc + 48'(rcnt);
        if (32'(rcnt) > c_mx) c_mx <= 32'(rcnt);
        if (bs == B_HEAD && q_mine && !q_done[qh]) c_bw <= c_bw + 32'd1;
        if (bs == B_WB && wb_lanes != '0 && !wr_ready) c_ws <= c_ws + 32'd1;
        if (bs == B_TRAP) c_le <= c_le + 32'd1;
      end
    end
    assign stat_kn = {c_le, c_ws, c_bw, c_mx, {16'h0, c_oc[47:32]}, c_oc[31:0], c_sy, c_is, c_ra, c_fe};

`ifndef SYNTHESIS
    // ---- simulation-only statistics and invariants
    always @(posedge clk) if (rst_n) begin
      if (busy) begin sim_cyc <= sim_cyc + 1; sim_ring_occ <= sim_ring_occ + longint'(rcnt); end
      if (int'(rcnt) > int'(sim_ring_max)) sim_ring_max <= rcnt;
      if (int'(qcnt) > int'(sim_iof_max)) sim_iof_max <= qcnt;
      if (st != S_IDLE) sim_fe_busy <= sim_fe_busy + 1;
      if (st == S_RALLOC && !fe_alloc) sim_ralloc <= sim_ralloc + 1;
      if (st == S_GISS && !fe_iss_hs) sim_iss_stall <= sim_iss_stall + 1;
      if (st == S_SYNC && !drained) sim_sync <= sim_sync + 1;
      if (st == S_TWAIT && !drained) sim_twait <= sim_twait + 1;
      if (bs == B_HEAD && q_mine && !q_done[qh]) sim_be_wait <= sim_be_wait + 1;
      if (bs == B_TRAP) sim_late_err <= sim_late_err + 1;
      if (fe_iss_hs && q_full) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: IOF overflow"); end
      if (q_pop && q_empty) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: IOF underflow"); end
      if (fe_iss_l && l1_req_valid && !q_empty) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: uc=0 request with a non-empty IOF"); end
      if (fe_alloc && be_retire && rcnt == '0) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: retire of an empty ring"); end
      if (be_retire && rcnt == '0) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: retire with an empty ring"); end
      if (u_rsp_valid && q_done[s2q[u_rsp_slot]]) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: second answer for IOF entry %0d", s2q[u_rsp_slot]); end
      if (!q_empty && rcnt != '0 && q_ctx[qh] != rh && !r_fed[rh]) begin
        sim_err <= sim_err + 1; $error("ng_lsu_kn: IOF head belongs to a younger entry while the ring head is still issuing");
      end
      if ((st == S_DONE || st == S_TRAP) && !drained) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: FE completion beside the BE"); end
      if (st == S_CHK && !async_q && !drained) begin sim_err <= sim_err + 1; $error("ng_lsu_kn: synchronous instruction ran beside the ring"); end
    end
`endif
  end
endmodule

// ng_lsu -- LSU_K = 1: one instruction in flight (ng_lsu_core K = 1; the uc channel is not used)
module ng_lsu
  import ng_lsu_pkg::*;
#(
  parameter int NL    = 16,   // register-file lanes per beat (16 or 32 == NG_RF_NL)
  parameter int SLOTW = 4,
  parameter int SH_AW = 16,   // shared array byte-address width
  parameter int SH_LPB = 8,   // shared memory: lanes per beat/round (32 = former single-round behaviour)
  parameter int L1_LPB = 4    // global path: lanes per beat of the load scatter / store merge (32 = widest)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- shared-memory scrub (ng_wd, between dispatches) ----
  input  logic              sh_clr_en,
  input  logic [SH_AW-7-1:0] sh_clr_row,

  // ---- issue ---------------------------------------------------------------
  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,
  input  logic [31:0]       iss_w1,
  input  logic [31:0]       iss_exec,
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,
  input  logic [15:0]       iss_tag,
  input  ctx_t              iss_ctx,

  // ---- completion (one-cycle pulses, exactly one per accepted instruction) ----
  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              busy,

  // ---- waitcnt accounting (ng_sp_sched cnt_inc_* / cnt_dec_*) --------------------
  output logic              cnt_inc_valid,
  output logic [SLOTW-1:0]  cnt_inc_slot,
  output logic [2:0]        cnt_inc_id,
  output logic              cnt_dec_valid,
  output logic [SLOTW-1:0]  cnt_dec_slot,
  output logic [2:0]        cnt_dec_id,

  // ---- vector register file: one read port, one write port -----------------------
  output logic                      rd_valid,
  input  logic                      rd_ready,
  output logic [SLOTW-1:0]          rd_slot,
  output logic [6:0]                rd_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat,
  output logic [1:0]                rd_hsel,
  input  logic                      rdv_valid,
  input  logic                      rdv_err,
  input  logic [NL*32-1:0]          rdv_data,

  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  // ---- uniform register file ------------------------------------------------------
  output logic                      ur_valid,
  output logic [SLOTW-1:0]          ur_slot,
  output logic [5:0]                ur_reg,
  input  logic                      urd_valid,
  input  logic [31:0]               urd_data,

  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data,

  // ---- MMU client (ng_mmu) ----------------------------------------------------------
  output logic                      mmu_req_valid,
  input  logic                      mmu_req_ready,
  output logic [3:0]                mmu_req_vmid,
  output logic [63:0]               mmu_req_va,
  output logic [1:0]                mmu_req_acc,
  output logic                      mmu_req_atomic,
  output logic [7:0]                mmu_req_tag,
  input  logic                      mmu_resp_valid,
  output logic                      mmu_resp_ready,
  input  logic [2:0]                mmu_resp_fault,
  input  logic                      mmu_resp_sparse,
  input  logic [47:0]               mmu_resp_pa,
  input  logic [63:0]               mmu_resp_pte,

  // BAD_PHYS_ADDR reported through the MMU fault latch (ng_mmu ext_flt_*)
  output logic                      ext_flt_valid,
  output logic [3:0]                ext_flt_vmid,
  output logic [63:0]               ext_flt_va,
  output logic [1:0]                ext_flt_acc,

  // ---- L1D request port --------------------------------------------------------------
  output logic                      l1_req_valid,
  input  logic                      l1_req_ready,
  output mreq_t                     l1_req,
  input  logic                      l1_rsp_valid,
  input  mrsp_t                     l1_rsp,

  // ---- statistics (test / debug) -------------------------------------------------------
  output logic [31:0]               stat_l1_rd,     // line reads issued
  output logic [31:0]               stat_l1_wr,     // line writes issued
  output logic [31:0]               stat_l1_amo,    // atomics issued to L2
  output logic [31:0]               stat_sh_rounds, // shared-memory rounds
  // global (non-shared, non-atomic) ld / st / uld instructions that reached the line-group path, and the
  // line-group requests their commit pass issued (one per non-sparse group; ratio = lines per memory instruction)
  output logic [31:0]               stat_instr_mem,
  output logic [31:0]               stat_groups
);
  ng_lsu_core #(.NL(NL), .SLOTW(SLOTW), .SH_AW(SH_AW), .SH_LPB(SH_LPB), .L1_LPB(L1_LPB), .K(1), .MEM_OUT(1)) u_core (
    .*,
    .u_req_valid(), .u_req_ready(1'b0), .u_req(), .u_req_slot(1'b0), .u_rsp_valid(1'b0), .u_rsp_slot(1'b0),
    .u_rsp_err(2'd0), .u_rd_req(), .u_rd_slot(), .u_rd_gnt(1'b0), .u_rd_data('0), .u_free(), .stat_kn()
  );
endmodule
