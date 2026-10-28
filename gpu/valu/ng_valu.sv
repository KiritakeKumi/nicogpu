// ng_valu.sv -- SP vector ALU (VALU), first cut  + f32 subset.
//
// Executes the 150 integer-VALU instructions of the ISA (vector int, .vu/.vi/
// .vl, 64-bit pair, cmp -> uniform mask, movrel, bit-field, dp4a ...) with the
// semantics of  With EN_FP = 1 it also executes
// the 88 f32 VALU instructions  (add/sub/mul/fma/ldexp,
// min/max/med3, compares incl. cmp.*.f32, fcvt, round-to-int, ...; semantics of
//, bit-exact incl. fflags and MODE rm/DAZ/FTZ/trap
// enables) in ng_valu_fp.  With EN_F16CVT = 1  the engine's side block also executes the 8 binary16 <->
// binary32 conversions fcvt.f32.f16.lo/hi, fcvt.f16.f32.lo/hi(.m) (rd also read: the other half is kept) and
// pkcvt.f16.f32(.m) (rounding MODE.rm16 or the rm operand, DAZ32 / DAZ16 / FTZ16); the other f16/half16 VALU ops
// stay out of scope.  SALU/branch are
// separate units.
//
// Structure (front/back split pipelined back half):
//   FRONT: issue -> CHK (decode checks, traps; uniform read request) -> [URW] ->
//   [PREP (movrel only)] -> collect the operands of each beat; the first beat's
//   launch moves the instruction into a free back-half context (xfer) and the last
//   beat's launch frees the front for the next instruction, whose CHK/URW/collect
//   then overlap the previous instructions' X1/X2/engine/write-back/drain.
//   BACK: BDEPTH instruction contexts (a ring in acceptance order).  The X stage
//   (X1/X2 or the f32 engine's issue cycles) reads op[] and the "X context" copies
//   (dq_b, exec_b, ...) of the most recently transferred instruction; behind it the
//   older contexts only drain: f32 beats in the engine (whose control signals and a
//   beat tag {exec slice, context, beat} travel with the data), write-back through
//   the W stage (w_slot/w_k per write) or the result queue, and completion.  Only
//   the oldest context completes (done / trap pulse in acceptance order, one per
//   instruction).  BDEPTH = 1 is the unit cycle for cycle: the back half
//   takes the next instruction only once it has completely drained.
//   The operand registers op[] stay a single set: with BDEPTH > 1 a read grant is
//   only issued when every reader of op[] is done by the time its data is written
//   (gsafe), and a launch waits for X1 / X2 to be free (they then never hold beats
//   of two instructions).  With BDEPTH = 1 the wait states (op_free, ex_idle,
//   fp_gate) do the same.
//   An instruction that can trap late (f32 with FpException enabled, or an f32
//   destination out of range) is "exclusive": it enters the back half only when all
//   contexts are free, and nothing is accepted behind it until it drained, so its
//   late trap finds no other instruction in the unit.  Traps and dones of the front
//   half (CHK traps, movrel drops) wait until the back half is empty, so the pulses
//   leave the unit in acceptance order (one per instruction).
//   Per beat of NL lanes (32/NL beats):
//        collect (<=3 vector read ports, bank-conflict retry)
//        X1 (per-lane 32x32 multiplier register)
//        X2 (ALU result) -> W (vector write port 0, lo then hi for pairs)
//   -> done/trap pulse (cmp: the uniform write is in the same cycle as done).
// f32 instructions replace X1/X2 by the ng_valu_fp engine (NFL lanes sweep the
// beat, 7-stage pipeline).  Trap order as the model: reserved rm (IllegalInsn)
// -> source RegRange -> FpException -> destination RegRange.
//   * If no FP exception can trap (MODE trap enables zero or a non-scope op) and
//     the destination is in range (direct), each beat is written as soon as the
//     engine delivers it; the result queue (QD entries) holds beats that find the
//     W stage busy.  f32 beats are only launched while the engine and the queue
//     hold fewer than QD beats, so the queue cannot overflow.
//   * Otherwise (precise FpException / dest-RegRange trap) all beats are queued
//     before anything is written; the drained instruction then either traps with
//     fflags updated and no write-back or writes the queue (fw_on).
// W stage priority: X2 (integer) > queued beats (precise write-back, or in order
// behind earlier queued ones) > engine write-through.
// Overlap: the collector of beat b+1 starts while beat b is in X1/X2
// (integer: only after beat 0, whose X2 cannot stall on the W stage; cmp: any
// beat) or while the engine still issues beat b (f32: operand registers are
// overwritten right after the last issue cycle; engine start of beat b+1 follows
// directly, so the engine sees a continuous stream of lane groups).  With
// BDEPTH > 1 these waits are the per-cycle gsafe condition instead, and the first
// beat's reads already go out in the CHK cycle (speculative: a read has no side
// effect, a CHK-trapped instruction's data is simply never used).  `launch`
// (all operands of the beat granted) starts X1 / the engine in the cycle the last
// read data arrives (it is written to op[] at the end of that cycle).  done is
// raised in the cycle the last register-file write is accepted.  No scoreboard is
// needed inside the unit: the instructions in flight belong to different warps
// (the scheduler's per-warp in-flight bit keeps a warp's next instruction back
// until the previous one's done/trap); the dispatcher owns hazards.
//
// Register file interface = ng_sp_regfile: read ports 0..2, write
// port 0, uniform read ports 0..1, uniform write port 0.  Write port 1 of the
// register file stays free for asynchronous writeback (load/SFU).
//
// Traps are precise: decode/alignment/range checks run before any state is
// touched.  cause 1 (IllegalInsn, info = low word) for unknown or non
// int-VALU encodings and odd pair registers; cause 10 (RegRange, info =
// register index) for vector registers >= iss_nregs.
module ng_valu
  import ng_valu_pkg::*;
  import ng_valu_fp_pkg::fop_scope;
  import ng_valu_fp_pkg::fop_rm16;
  import ng_bitops_pkg::chk_vreg;
#(
  parameter int NL     = 16,  // physical lanes per beat (16 or 32, must equal NG_RF_NL)
  parameter int SLOTW  = 4,   // warp slot id width (NG_RF_NUM_WARPS = 2**SLOTW)
  parameter bit EN_FP  = 1'b0,// 1: f32 VALU subset (ng_valu_fp)
  parameter int NFL    = 4,   // f32 physical lanes (divides NL)
  parameter int BDEPTH = 3,   // back-half instruction contexts; 1 = (back half drains first)
  parameter bit EN_F16CVT = 1'b0 // 1 (needs EN_FP): binary16 <-> binary32 conversions fcvt.f32.f16.lo/hi,
                                 // fcvt.f16.f32.lo/hi(.m), pkcvt.f16.f32(.m) ; 0: illegal, as before
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- issue ---------------------------------------------------------------
  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,      // instruction word 0 (bit0=0: 32-bit)
  input  logic [31:0]       iss_w1,      // word 1 (64-bit encodings)
  input  logic [31:0]       iss_exec,    // EXEC at issue
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [7:0]        iss_nregs,   // warp's vreg allocation (<=128)
  input  logic [31:0]       iss_mode,    // warp's MODE CSR (used by f32 instructions only)
  input  logic [15:0]       iss_tag,     // opaque, returned with done

  // ---- completion (one-cycle pulses; exactly one per accepted instruction) ---
  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              fflags_valid, // f32: new exception flags (with done, or with an FpException trap)
  output logic [4:0]        fflags_set,
  output logic              busy,
  output logic              bbusy,       // perf counter: back half holds an instruction (!b_idle)

  // ---- vector register file: read ports -------------------------------------
  output logic [2:0]                rd_valid,
  input  logic [2:0]                rd_ready,
  output logic [SLOTW-1:0]          rd_slot [3],
  output logic [6:0]                rd_reg  [3],
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] rd_beat [3],
  output logic [1:0]                rd_hsel [3],
  input  logic [2:0]                rdv_valid,
  input  logic [2:0]                rdv_err,
  input  logic [NL*32-1:0]          rdv_data [3],

  // ---- vector register file: write port --------------------------------------
  output logic                      wr_valid,
  input  logic                      wr_ready,
  output logic [SLOTW-1:0]          wr_slot,
  output logic [6:0]                wr_reg,
  output logic [(NL < 32 ? $clog2(32 / NL) : 1)-1:0] wr_beat,
  output logic [NL-1:0]             wr_lanes,
  output logic [1:0]                wr_hsel,
  output logic [NL*32-1:0]          wr_data,

  // ---- uniform register file --------------------------------------------------
  output logic [1:0]                ur_valid,
  output logic [SLOTW-1:0]          ur_slot [2],
  output logic [5:0]                ur_reg  [2],
  input  logic [1:0]                urd_valid,
  input  logic [31:0]               urd_data [2],

  output logic                      uw_valid,
  output logic [SLOTW-1:0]          uw_slot,
  output logic [5:0]                uw_reg,
  output logic [31:0]               uw_data
);
  localparam int NBEAT = 32 / NL;
  localparam int BTW   = ng_noc_pkg::clog2_1(NBEAT);
  localparam int DW    = NL * 32;
  localparam bit PIPE  = (BDEPTH > 1);                 // rules (gsafe, launch behind older contexts)
  localparam int KW    = ng_noc_pkg::clog2_1(BDEPTH);   // context index
  localparam int KCW   = $clog2(BDEPTH + 1);           // context count
  localparam int QD    = NBEAT + 2;                    // result queue entries (>= NBEAT for the precise path)
  localparam int QIW   = $clog2(QD);
  localparam int QCW   = $clog2(QD + 1);
  localparam int TGW   = NL + KW + BTW;                // f32 engine beat tag {exec slice, context, beat}

  // circular index increment (context ring: depth BDEPTH, result queue: depth QD); callers truncate to their width
  function automatic logic [31:0] wrap_inc(input int unsigned p, input int depth);
    return (int'(p) == depth - 1) ? 32'd0 : 32'(p + 1);
  endfunction

  // ---------------------------------------------------------------- control state
  // front half
  typedef enum logic [2:0] {
    ST_IDLE, ST_CHK, ST_URW, ST_PREP, ST_RUN
  } st_t;
  st_t st;

  // front copies (accepted instruction)
  logic [31:0]      w0_q, exec_q, mode_q;
  logic [SLOTW-1:0] slot_q;
  logic [7:0]       nregs_q;
  logic [15:0]      tag_q;
  dec_t             dq;
  logic [31:0]      uv0, uv1;
  logic [6:0]       rel_reg;    // movrel effective register (valid when !rel_bad)
  logic             rel_bad;    // movrel index outside the allocation
  logic             err_f;      // unexpected register-file range error (front instruction, before its transfer)
  logic             f_xd;       // the front instruction has already been moved to the back half
  logic             excl_q;     // the front instruction is exclusive (latched in CHK, static from then on)
  // X context: copies of the most recently transferred instruction (X1/X2, engine issue)
  logic [31:0]      exec_b, mode_b;
  logic [SLOTW-1:0] slot_b;
  dec_t             dq_b;
  logic [31:0]      uv0_b, uv1_b;
  logic [6:0]       rel_reg_b;
  logic             rel_bad_b;
  logic [2:0]       rm_sel_b;
  logic             excl_b;     // the most recently transferred instruction is exclusive: nothing is accepted behind it
  logic [KW-1:0]    xk;         // its context
  // back-half contexts (ring, acceptance order): bh = oldest, bt = next free, bn = in use
  logic [KW-1:0]    bh, bt;
  logic [KCW-1:0]   bn;
  logic             b_idle;     // no instruction in the back half
  logic             fw_on;      // the head writes its buffered beats (precise f32 path)
  assign b_idle = (bn == '0);
  logic [15:0]      k_tag  [BDEPTH];
  logic [SLOTW-1:0] k_slot [BDEPTH];
  logic [6:0]       k_rd   [BDEPTH];   // destination (f32 write-back, cmp uniform register)
  logic             k_ureg [BDEPTH];   // cmp: uniform destination
  logic             k_fp   [BDEPTH];
  logic             k_buf  [BDEPTH];   // precise f32 write-back: all beats queued, then trap or write
  logic [4:0]       k_ften [BDEPTH];   // FpException enables (MODE[20:16]) of an op in scope, else 0
  logic             k_late [BDEPTH];   // f32 destination out of range (late RegRange)
  logic [7:0]       k_lidx [BDEPTH];
  logic             k_err  [BDEPTH];   // unexpected register-file range error
  logic             k_all  [BDEPTH];   // every beat has been launched
  logic [BTW:0]     k_bd   [BDEPTH];   // beats out of X2 / the engine
  logic [BTW:0]     k_wc   [BDEPTH];   // beats whose write finished
  logic [4:0]       k_fl   [BDEPTH];   // f32 exception flags
  logic [31:0]      k_cmp  [BDEPTH];   // cmp mask

  dec_t dec_c;
  ng_valu_dec #(.EN_FP(EN_FP), .EN_F16CVT(EN_F16CVT)) u_dec (.w0(iss_w0), .w1(iss_w1), .d(dec_c));


  // ---------------------------------------------------------------- CHK logic (front)
  // One operand of the model's check sequence (sem_common.h read_v64/write_v64 +
  // ExecCtx::vreg): pair alignment first (IllegalInsn), then the register range
  // of the low and high half (RegRange, info = offending index).
  typedef struct packed {
    logic       f;       // fault
    logic       illegal; // IllegalInsn (else RegRange)
    logic [7:0] idx;
  } chk_t;
  function automatic chk_t chk_op(input logic vec, input logic pair, input logic skip_range,
                                  input logic [6:0] r, input logic [7:0] n);
    chk_t c;
    c = '0;
    if (pair && r[0]) begin
      c.f = 1'b1; c.illegal = 1'b1;
    end else begin
      {c.f, c.idx} = chk_vreg(vec && !skip_range, pair, {1'b0, r}, n);
    end
    return c;
  endfunction

  logic        chk_illegal, chk_range, rm_bad, late_fault;
  logic [7:0]  late_idx;
  logic [2:0]  rm_sel, rm_m16;
  logic [31:0] chk_info;
  logic        need_u0, need_u1;
  logic [5:0]  u0_idx;

  always_comb begin
    chk_t ca, cb_, cc, cd, cf;
    // operands are checked in the model's order: sources a, b, c, then the destination
    ca  = chk_op((dq.a_src == S_VREG) || (dq.a_src == S_VPAIR), (dq.a_src == S_VPAIR) || (dq.a_src == S_UPAIR),
                 dq.rel_s || dq.rel_d, dq.rs1, nregs_q);
    cb_ = chk_op((dq.b_src == S_VREG) || (dq.b_src == S_VPAIR), (dq.b_src == S_VPAIR), 1'b0, dq.rs2, nregs_q);
    cc  = chk_op((dq.c_src == S_VREG) || (dq.c_src == S_VPAIR), (dq.c_src == S_VPAIR), 1'b0, dq.rs3, nregs_q);
    cd  = chk_op((dq.d_kind == D_VREG) || (dq.d_kind == D_VPAIR), (dq.d_kind == D_VPAIR), dq.rel_d, dq.rd, nregs_q);
    // f32: the destination range is checked after the FpException check (late_*),
    // except for fmac whose destination is also a source (read first)
    cf  = ca.f ? ca : cb_.f ? cb_ : cc.f ? cc : (dq.fp && !dq.d_read) ? '0 : cd;
    late_fault  = dq.fp && !dq.d_read && cd.f;
    late_idx    = cd.idx;
    // reserved rounding mode (rm operand 5/6, or MODE.rm 5..7): IllegalInsn before any source read.
    // binary16 results  read MODE.rm16, which follows MODE.rm when it is 7 (model sem_fp.h).
    rm_m16      = (mode_q[5:3] == 3'd7) ? mode_q[2:0] : mode_q[5:3];
    rm_sel      = (dq.has_rm && dq.rm != 3'd7) ? dq.rm : (EN_F16CVT && fop_rm16(dq.fop)) ? rm_m16 : mode_q[2:0];
    rm_bad      = dq.fp && fop_scope(dq.fop) && (rm_sel > 3'd4);
    chk_illegal = !dq.hit || rm_bad || (cf.f && cf.illegal);
    chk_range   = dq.hit && !rm_bad && cf.f && !cf.illegal;
    chk_info    = chk_illegal ? w0_q : {24'd0, cf.idx};

    need_u0 = (dq.a_src == S_UREG) || (dq.a_src == S_UPAIR) || (dq.b_src == S_UREG);
    need_u1 = (dq.a_src == S_UPAIR);
    u0_idx  = ((dq.a_src == S_UREG) || (dq.a_src == S_UPAIR)) ? dq.rs1[5:0] : dq.rs2[5:0];
  end

  // ---------------------------------------------------------------- PREP (movrel)
  logic [32:0] rel_sum;
  always_comb begin
    rel_sum = {26'd0, dq.rel_d ? dq.rd : dq.rs1} + {1'b0, uv0};
  end
  wire rel_oor = (rel_sum >= {25'd0, nregs_q});
  wire rs1_oor_d = ({1'b0, dq.rs1} >= nregs_q);   // movrel.d source range

  // ---------------------------------------------------------------- collector (front)
  typedef enum logic [2:0] {CS_OFF, CS_WAITEX, CS_COLLECT, CS_WFP, CS_WAITOP} cs_t;
  cs_t                cs;
  logic [BTW-1:0]     cb;          // beat being collected
  logic [6:0]         need;        // slots needed per beat
  logic [6:0]         pend;        // slots not yet granted
  logic [2:0]         rsp_v;       // a grant happened last cycle on port k
  logic [2:0]         rsp_slot [3];// ... for this slot
  logic [DW-1:0]      op [7];      // operand registers: A0 A1 B0 B1 C0 C1 D (one set, shared by both halves)

  logic x1_v, x2_v;
  logic [BTW-1:0] xb;              // beat in X1/X2 (integer)
  logic           launch;          // all operands of beat cb collected: start X1 / the f32 engine

  // register-read slots an instruction needs: A0 A1 B0 B1 C0 C1 D
  function automatic logic [6:0] need_of(input dec_t d);
    logic [6:0] n;
    n[0] = (d.a_src == S_VREG) || (d.a_src == S_VPAIR);
    n[1] = (d.a_src == S_VPAIR);
    n[2] = (d.b_src == S_VREG) || (d.b_src == S_VPAIR);
    n[3] = (d.b_src == S_VPAIR);
    n[4] = (d.c_src == S_VREG) || (d.c_src == S_VPAIR);
    n[5] = (d.c_src == S_VPAIR);
    n[6] = d.d_read;
    return n;
  endfunction
  assign need = need_of(dq) & ~{6'd0, dq.rel_s && rel_bad};   // movrel with a bad A register reads nothing
  // ... of the instruction being accepted (its reads may start in the CHK cycle)
  logic [6:0] need_c;
  assign need_c = need_of(dec_c);

  function automatic logic [6:0] slot_reg(input dec_t d, input logic [6:0] rrel, input int s);
    case (s)
      0: return d.rel_s ? rrel : d.rs1;
      1: return d.rs1 + 7'd1;
      2: return d.rs2;
      3: return d.rs2 + 7'd1;
      4: return d.rs3;
      5: return d.rs3 + 7'd1;
      default: return d.rd;
    endcase
  endfunction

  logic [2:0] port_v;
  logic [2:0] port_slot [3];
  always_comb begin
    int cnt;
    cnt = 0;
    port_v = '0;
    for (int k = 0; k < 3; k++) port_slot[k] = 3'd0;
    for (int i = 0; i < 7; i++) begin
      if (pend[i] && cnt < 3) begin
        port_v[cnt]    = 1'b1;
        port_slot[cnt] = 3'(i);
        cnt++;
      end
    end
  end

  logic collecting, gsafe, chk1, rd_chk;
  assign collecting = (st == ST_RUN) && (cs == CS_COLLECT);
  // the first beat's operand reads already go out in the (first) CHK cycle, before the
  // checks are known (no movrel: its register depends on PREP).  A read has no side effect: if the
  // instruction traps, its data lands in op[] (gsafe: no reader left) and is never used.
  assign rd_chk = PIPE && (st == ST_CHK) && chk1 && !dq.rel_s && !dq.rel_d;

  always_comb begin
    for (int k = 0; k < 3; k++) begin
      rd_valid[k] = (collecting || rd_chk) && port_v[k] && gsafe;
      rd_slot[k]  = slot_q;
      rd_reg[k]   = slot_reg(dq, rel_reg, int'(port_slot[k]));
      rd_beat[k]  = cb;
      rd_hsel[k]  = 2'd0;
    end
  end

  logic [2:0] gnt;
  logic [6:0] gclr;    // slots granted in this cycle
  assign gnt = rd_valid & rd_ready;
  always_comb begin
    gclr = '0;
    for (int k = 0; k < 3; k++) if (gnt[k]) gclr[port_slot[k]] = 1'b1;
  end

  // exclusive instruction (can trap late; static once CHK passed) and the transfer to the back half
  logic excl_f, xfer;
  assign excl_f = dq.fp && (late_fault || (fop_scope(dq.fop) && (|mode_q[20:16])));
  assign xfer   = launch && (cb == '0);

  // ---------------------------------------------------------------- execute (X1/X2)
  // effective operands (routing of broadcast / immediate / unused sources)
  logic [31:0] ea_lo [NL], ea_hi [NL], eb_lo [NL], eb_hi [NL], ec_lo [NL], ec_hi [NL], ed [NL];
  logic [31:0] xa [NL], xb_ [NL];
  logic [63:0] prod [NL];
  logic [31:0] r_lo [NL], r_hi [NL];
  logic [NL-1:0] cmpb, lwe;
  logic [31:0] ln_lo [NL], ln_hi [NL];
  logic [NL-1:0] ln_cmpb, ln_lwe;
  logic [NL*32-1:0] fp_a, fp_b, fp_c, fp_res;
  logic [NL-1:0]    fp_cmp;
  logic [4:0]       fp_flags;
  logic             fp_start, fp_done, fp_busy, fp_can;
  localparam int NG  = NL / NFL;
  localparam int GWO = ng_noc_pkg::clog2_1(NG) + 1;
  logic [GWO-1:0]   fp_gi;
  logic [TGW-1:0]   fp_tag_i, fp_tag_o, fp_dtag;

  always_comb begin
    for (int i = 0; i < NL; i++) begin
      logic [31:0] a0, a1, b0, b1, c0, c1, d0;
      a0 = op[0][i*32 +: 32]; a1 = op[1][i*32 +: 32];
      b0 = op[2][i*32 +: 32]; b1 = op[3][i*32 +: 32];
      c0 = op[4][i*32 +: 32]; c1 = op[5][i*32 +: 32];
      d0 = op[6][i*32 +: 32];
      // A
      case (dq_b.a_src)
        S_VREG:  begin ea_lo[i] = dq_b.rel_s && rel_bad_b ? 32'd0 : a0; ea_hi[i] = 32'd0; end
        S_VPAIR: begin ea_lo[i] = a0; ea_hi[i] = a1; end
        S_UREG:  begin ea_lo[i] = uv0_b; ea_hi[i] = 32'd0; end
        S_UPAIR: begin ea_lo[i] = uv0_b; ea_hi[i] = uv1_b; end
        S_IMM:   begin ea_lo[i] = dq_b.imm; ea_hi[i] = 32'd0; end
        default: begin ea_lo[i] = 32'd0; ea_hi[i] = 32'd0; end
      endcase
      // B
      case (dq_b.b_src)
        S_VREG:  begin eb_lo[i] = b0; eb_hi[i] = 32'd0; end
        S_VPAIR: begin eb_lo[i] = b0; eb_hi[i] = b1; end
        S_UREG:  begin eb_lo[i] = uv0_b; eb_hi[i] = 32'd0; end
        S_IMM:   begin eb_lo[i] = dq_b.imm; eb_hi[i] = 32'd0; end
        default: begin eb_lo[i] = 32'd0; eb_hi[i] = 32'd0; end
      endcase
      // C, D
      ec_lo[i] = (dq_b.c_src == S_NONE) ? 32'd0 : c0;
      ec_hi[i] = (dq_b.c_src == S_VPAIR) ? c1 : 32'd0;
      ed[i]    = dq_b.d_read ? d0 : 32'd0;
      // swap (subr / sllr / srlr / srar)
      xa[i]  = dq_b.swap ? eb_lo[i] : ea_lo[i];
      xb_[i] = dq_b.swap ? ea_lo[i] : eb_lo[i];
    end
  end

  for (genvar i = 0; i < NL; i++) begin : g_lane
    ng_valu_mul u_mul (.clk(clk), .en(x1_v), .a(xa[i]), .b(xb_[i]), .sa(dq_b.sa), .sb(dq_b.sb), .p(prod[i]));
    ng_valu_lane u_lane (
      .uop(dq_b.uop), .cmps(dq_b.cmps), .cmpfn(dq_b.cmpfn),
      .a_lo(xa[i]), .a_hi(ea_hi[i]), .b_lo(xb_[i]), .b_hi(eb_hi[i]),
      .c_lo(ec_lo[i]), .c_hi(ec_hi[i]), .d_old(ed[i]),
      .prod(prod[i]), .lane(5'(int'(xb) * NL + i)),
      .r_lo(ln_lo[i]), .r_hi(ln_hi[i]), .cmp_bit(ln_cmpb[i]), .lane_we(ln_lwe[i])
    );
    // the X2 stage only carries integer results (f32 results come from the engine at fp_done)
    always_comb begin
      r_lo[i] = ln_lo[i]; r_hi[i] = ln_hi[i]; cmpb[i] = ln_cmpb[i]; lwe[i] = ln_lwe[i];
    end
  end

  // ---------------------------------------------------------------- f32 engine
  // EXEC slice of the beat in X1/X2 / at the engine output / at done
  logic [NL-1:0] exec_sl, exec_so, exec_sd;
  logic [KW-1:0] ko;                // context of the beat completed by fp_done
  logic [BTW-1:0] od;               // ... its beat
  always_comb begin
    exec_sl = '0;
    for (int i = 0; i < NL; i++) exec_sl[i] = exec_b[int'(xb) * NL + i];
    // beat tag {exec slice, context, beat}: the launched beat is the front instruction's
    fp_tag_i = {exec_q[int'(cb) * NL +: NL], (xfer ? bt : xk), cb};
    exec_so  = fp_tag_o[TGW-1 -: NL];
    exec_sd  = fp_dtag[TGW-1 -: NL];
    ko       = fp_dtag[BTW +: KW];
    od       = fp_dtag[0 +: BTW];
  end
  always_comb begin
    for (int i = 0; i < NL; i++) begin
      fp_a[i*32 +: 32] = ea_lo[i];
      fp_b[i*32 +: 32] = eb_lo[i];
      fp_c[i*32 +: 32] = dq_b.d_read ? ed[i] : ec_lo[i];
    end
  end
  if (EN_FP) begin : g_fp
    assign fp_start = launch && dq.fp;
    ng_valu_fp #(.NL(NL), .NFL(NFL), .TGW(TGW), .EN_F16(EN_F16CVT)) u_fp (
      .clk(clk), .rst_n(rst_n), .start(fp_start),
      .fop(dq_b.fop), .pred(dq_b.fpred), .rm(rm_sel_b), .ub(dq_b.fsub), .daz(mode_b[9]), .ftz(mode_b[10]),
      .daz16(mode_b[11]), .ftz16(mode_b[12]),
      .sat(dq_b.msat), .neg(dq_b.mneg), .abs_m(dq_b.mabs),
      .a_v(fp_a), .b_v(fp_b), .c_v(fp_c), .exec_sl(exec_so), .tag_i(fp_tag_i), .tag_o(fp_tag_o), .dtag_o(fp_dtag),
      .done(fp_done), .busy(fp_busy), .last_out(), .can_start(fp_can), .gi_o(fp_gi), .res_v(fp_res), .cmp_v(fp_cmp), .flags(fp_flags)
    );
  end else begin : g_nofp_eng
    assign fp_start = 1'b0;
    assign fp_done  = 1'b0;
    assign fp_busy  = 1'b0;
    assign fp_can   = 1'b1;
    assign fp_gi    = '0;
    assign fp_res   = '0;
    assign fp_cmp   = '0;
    assign fp_flags = 5'd0;
    assign fp_tag_o = '0;
    assign fp_dtag  = '0;
  end

  // f32 beats in the engine (credit for the result queue)
  logic [QCW-1:0] c_eng;
  logic [QCW-1:0] q_n;
  logic           fp_room;
  assign fp_room = ({1'b0, c_eng} + {1'b0, q_n}) < (QCW+1)'(QD);

  // ---------------------------------------------------------------- result queue
  logic [DW-1:0]  q_data  [QD];
  logic [NL-1:0]  q_lanes [QD];
  logic [KW-1:0]  q_k     [QD];
  logic [BTW-1:0] q_beat  [QD];
  logic [QIW-1:0] q_rp, q_wp;

  // ---------------------------------------------------------------- write stage
  logic              w_v, w_ph, w_pair;
  logic [DW-1:0]     w_lo, w_hi;
  logic [NL-1:0]     w_lanes;
  logic [6:0]        w_reg;
  logic [BTW-1:0]    w_beat;
  logic [SLOTW-1:0]  w_slot;
  logic [KW-1:0]     w_k;

  always_comb begin
    wr_valid = w_v && (w_lanes != '0);
    wr_slot  = w_slot;
    wr_reg   = w_reg + {6'd0, w_ph};
    wr_beat  = w_beat;
    wr_lanes = w_lanes;
    wr_hsel  = 2'b11;
    wr_data  = w_ph ? w_hi : w_lo;
  end

  logic [DW-1:0] r_lo_flat, r_hi_flat;
  always_comb begin
    for (int i = 0; i < NL; i++) begin
      r_lo_flat[i*32 +: 32] = r_lo[i];
      r_hi_flat[i*32 +: 32] = r_hi[i];
    end
  end

  // the W stage finishes in this cycle (idle, nothing to write, or the last phase is accepted)
  logic w_fin, ex_idle, last_beat, fp_gate;
  assign w_fin   = !w_v || (w_lanes == '0) || (wr_ready && !(w_pair && !w_ph));
  assign ex_idle = !x1_v && !x2_v;
  assign last_beat = (int'(cb) == NBEAT - 1);
  // f32: the next beat's operand reads may start when the write of their data (end of the
  // second cycle after the request) cannot precede the last engine issue cycle of this beat
  assign fp_gate = (int'(fp_gi) + 3 >= NG);

  // W stage sources (loaded only when it is empty): X2 > queue > engine write-through
  logic x2_ureg, x2_go, x2_w, q_out, thr_w, q_push, fp_vec;
  assign x2_ureg = (dq_b.d_kind == D_UREG);
  assign x2_go   = x2_ureg || !w_v;                  // X2 leaves in this cycle
  assign x2_w    = x2_v && !x2_ureg && !w_v;
  assign fp_vec  = fp_done && !k_ureg[ko];
  // the queue head goes out while the precise path writes back, or (direct results) in a cycle
  // without a new engine result (order; a precise instruction's beats wait for fw_on)
  assign q_out   = !w_v && !x2_w && (q_n != '0) && (fw_on || (!fp_done && !k_buf[q_k[q_rp]]));
  assign thr_w   = !w_v && !x2_w && fp_vec && !k_buf[ko] && (q_n == '0);
  assign q_push  = fp_vec && !thr_w;

  // ---------------------------------------------------------------- completion (oldest context)
  logic           wfin_h, ftrap_h, hd_base, hd_rdy, fw_fin, hd_pop, q_flush;
  logic [BTW:0]   wc_h;
  always_comb begin
    wfin_h  = w_v && w_fin && (w_k == bh);
    wc_h    = k_wc[bh] + (BTW+1)'(wfin_h);
    ftrap_h = k_fp[bh] && (|(k_fl[bh] & k_ften[bh]));
    hd_base = !b_idle && !fw_on && k_all[bh] && (k_bd[bh] == (BTW+1)'(NBEAT));
    hd_rdy  = hd_base && (k_ureg[bh] || k_buf[bh] || (wc_h == (BTW+1)'(NBEAT)));
    fw_fin  = fw_on && (wc_h == (BTW+1)'(NBEAT));
    // the head leaves (done or trap; a precise f32 instruction without trap goes on to fw_on first)
    hd_pop  = (hd_rdy && !(k_buf[bh] && !k_err[bh] && !ftrap_h && !k_late[bh])) || fw_fin;
    // a precise f32 instruction traps: its queued beats are dropped (it is alone in the back half,
    // so the queue holds nothing else and no other result can arrive in this cycle)
    q_flush = hd_rdy && k_buf[bh] && (k_err[bh] || ftrap_h || k_late[bh]);
  end

  // ---------------------------------------------------------------- op[] / launch rules
  // (BDEPTH = 1): op[] has no reader of the previous (back) instruction left: X1/X2 idle
  // and the f32 engine has issued (or is issuing the last cycles of) its final lane group, so
  // the data of reads requested now (written two cycles later) cannot disturb it.
  logic fp_gate2, op_free;
  assign fp_gate2 = fp_can || (int'(fp_gi) + 3 >= NG);
  assign op_free  = ex_idle && fp_gate2;
  // (BDEPTH > 1): a grant in this cycle writes op[] at the end of the next one, so every
  // reader must be done by then: X2 leaves now; an X1 beat's X2 can leave next cycle (W empty and
  // no f32 result enters W now, X2 has priority next cycle); the engine issues its last group by then.
  logic gs_fp, gs_x1, gs_x2;
  assign gs_fp = fp_can || (int'(fp_gi) + 2 >= NG);
  assign gs_x1 = !x1_v || x2_ureg || (!w_v && !fp_done && (q_n == '0) && !fw_on);
  assign gs_x2 = !x2_v || x2_go;
  assign gsafe = !PIPE || (gs_fp && gs_x1 && gs_x2);
  // a free context for the first beat (an exclusive instruction needs all of them)
  logic ctx_ok;
  assign ctx_ok = excl_q ? b_idle : (int'(bn) < BDEPTH);
  if (PIPE) begin : g_launch_pipe
    // X1 empty and X2 empty or leaving, and the f32 engine issuing its last lane group at most
    // (it reads the X context's operands / controls in every issue cycle): the launch may switch
    // the X context.  These are explicit because instructions without vector sources
    // (mov.vu, movi, ...) never wait for gsafe.
    assign launch = collecting && (pend == 7'd0) && !x1_v && (!x2_v || x2_go)
                    && fp_can && (!dq.fp || fp_room) && (cb != '0 || ctx_ok);
  end else begin : g_launch_23a
    assign launch = collecting && (pend == 7'd0) && !x1_v && (!dq.fp || fp_can) && (cb != '0 || b_idle);
  end

  // --------------------------------------------------------------
  // front-half trap / done / drop pulses leave only when the back half is idle (acceptance order)
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= ST_IDLE;
      cs <= CS_OFF;
      x1_v <= 1'b0; x2_v <= 1'b0; w_v <= 1'b0; w_ph <= 1'b0;
      rsp_v <= '0;
      pend <= '0;
      done_valid <= 1'b0; trap_valid <= 1'b0;
      trap_cause <= '0; trap_info <= '0; done_tag <= '0;
      err_f <= 1'b0; f_xd <= 1'b0; excl_q <= 1'b0; chk1 <= 1'b0;
      cb <= '0; xb <= '0;
      fflags_valid <= 1'b0; fflags_set <= '0;
      excl_b <= 1'b0; xk <= '0;
      bh <= '0; bt <= '0; bn <= '0; fw_on <= 1'b0;
      c_eng <= '0; q_n <= '0; q_rp <= '0; q_wp <= '0;
      w_k <= '0;
      for (int k = 0; k < BDEPTH; k++) begin
        k_all[k] <= 1'b0; k_bd[k] <= '0; k_wc[k] <= '0; k_err[k] <= 1'b0;
        k_ureg[k] <= 1'b0; k_buf[k] <= 1'b0; k_fp[k] <= 1'b0; k_late[k] <= 1'b0;
      end
    end else begin
      done_valid <= 1'b0;
      trap_valid <= 1'b0;
      fflags_valid <= 1'b0;

      // ================= front half =================
      case (st)
        ST_IDLE: begin
          if (iss_valid && iss_ready) begin
            w0_q    <= iss_w0;
            exec_q  <= iss_exec;
            mode_q  <= iss_mode;
            slot_q  <= iss_slot;
            nregs_q <= iss_nregs;
            tag_q   <= iss_tag;
            dq      <= dec_c;
            err_f   <= 1'b0;
            f_xd    <= 1'b0;
            st      <= ST_CHK;
            chk1    <= 1'b1;
            if (PIPE) begin pend <= need_c; cb <= '0; end
          end
        end

        ST_CHK: begin
          excl_q <= excl_f;
          chk1   <= 1'b0;
          if (PIPE) pend <= pend & ~gclr;
          if (chk_illegal || chk_range) begin
            if (b_idle) begin
              trap_valid <= 1'b1;
              trap_cause <= chk_illegal ? CAUSE_ILLEGAL : CAUSE_REGRANGE;
              trap_info  <= chk_info;
              st         <= ST_IDLE;
            end
          end else if (need_u0) begin
            st <= ST_URW;      // ur_valid driven in this state (data back in ST_URW)
          end else if (dq.rel_s || dq.rel_d) begin
            st <= ST_PREP;
          end else begin
            // no uniform read, no movrel: straight into the collector
            st   <= ST_RUN;
            if (!PIPE) pend <= need;   // (loaded at accept, reads may have started)
            cb   <= '0;
            cs   <= (PIPE || op_free) ? CS_COLLECT : CS_WAITOP;
          end
        end

        ST_URW: begin
          // urd_* valid this cycle (1 cycle after the request); uv0/uv1 are latched at the end of it
          if (dq.rel_s || dq.rel_d) begin
            st <= ST_PREP;
          end else begin
            st   <= ST_RUN;
            if (!PIPE) pend <= need;
            cb   <= '0;
            cs   <= (PIPE || op_free) ? CS_COLLECT : CS_WAITOP;
          end
        end

        ST_PREP: begin
          if (dq.rel_d && rel_oor) begin
            // out-of-allocation destination: write dropped, nothing else happens
            if (b_idle) begin done_valid <= 1'b1; done_tag <= tag_q; st <= ST_IDLE; end
          end else if (dq.rel_d && rs1_oor_d) begin
            if (b_idle) begin
              trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= {25'd0, dq.rs1};
              st <= ST_IDLE;
            end
          end else begin
            st <= ST_RUN;
            cs <= CS_WAITOP;   // rel_bad is latched at the end of this cycle: pend is set next cycle
            cb <= '0;
          end
        end

        ST_RUN: begin
          // ---- collector ----
          case (cs)
            CS_WAITOP: if (PIPE || op_free) begin pend <= need; cs <= CS_COLLECT; end
            CS_WAITEX: if (ex_idle) begin pend <= need; cs <= CS_COLLECT; end
            CS_WFP: if (fp_gate) begin pend <= need; cs <= CS_COLLECT; end
            CS_COLLECT: begin
              pend <= pend & ~gclr;
              if (launch) begin
                xb <= cb;
                cb <= cb + 1'b1;
                if (!dq.fp) x1_v <= 1'b1;
                if (last_beat) begin
                  cs <= CS_OFF;
                  st <= ST_IDLE;      // the whole instruction is in the back half now
                end
                else if (PIPE) pend <= need;   // gsafe holds the grants back
                else if (dq.fp) cs <= CS_WFP;
                else if (dq.d_kind == D_UREG || int'(cb) == 0) pend <= need;  // next beat's reads overlap X1/X2
                else cs <= CS_WAITEX;
              end
            end
            default: ;
          endcase
        end

        default: st <= ST_IDLE;
      endcase

      // ================= front -> back transfer (first beat launch): new context bt =================
      if (xfer) begin
        dq_b         <= dq;
        exec_b       <= exec_q;
        mode_b       <= mode_q;
        slot_b       <= slot_q;
        uv0_b        <= uv0;
        uv1_b        <= uv1;
        rel_reg_b    <= rel_reg;
        rel_bad_b    <= rel_bad;
        rm_sel_b     <= rm_sel;
        excl_b       <= excl_q;
        xk           <= bt;
        bt           <= KW'(wrap_inc(bt, BDEPTH));
        k_tag[bt]    <= tag_q;
        k_slot[bt]   <= slot_q;
        k_rd[bt]     <= dq.rd;
        k_ureg[bt]   <= (dq.d_kind == D_UREG);
        k_fp[bt]     <= dq.fp;
        k_buf[bt]    <= excl_q && (dq.d_kind != D_UREG);
        k_ften[bt]   <= (dq.fp && fop_scope(dq.fop)) ? mode_q[20:16] : 5'd0;
        k_late[bt]   <= late_fault;
        k_lidx[bt]   <= late_idx;
        k_err[bt]    <= err_f;
        k_all[bt]    <= last_beat;
        k_bd[bt]     <= '0;
        k_wc[bt]     <= '0;
        k_fl[bt]     <= 5'd0;
        k_cmp[bt]    <= 32'd0;
        f_xd         <= 1'b1;
      end
      if (launch && last_beat && !xfer) k_all[xk] <= 1'b1;

      // ================= completion of the oldest context =================
      if (hd_rdy) begin
        if (k_err[bh]) begin
          trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= 32'd0;
        end else if (ftrap_h) begin
          trap_valid <= 1'b1; trap_cause <= CAUSE_FPEXC; trap_info <= {27'd0, k_fl[bh]};
          fflags_valid <= 1'b1; fflags_set <= k_fl[bh];
        end else if (k_late[bh]) begin
          trap_valid <= 1'b1; trap_cause <= CAUSE_REGRANGE; trap_info <= {24'd0, k_lidx[bh]};
        end else if (k_buf[bh]) begin
          fw_on <= 1'b1;      // f32 precise path: write the queued beats through the W stage
        end else begin
          // (cmp: uw_valid is driven in this cycle)
          done_valid <= 1'b1; done_tag <= k_tag[bh];
          if (k_fp[bh]) begin fflags_valid <= 1'b1; fflags_set <= k_fl[bh]; end
        end
      end
      if (fw_fin) begin
        done_valid <= 1'b1; done_tag <= k_tag[bh]; fw_on <= 1'b0;
        fflags_valid <= 1'b1; fflags_set <= k_fl[bh];
      end
      if (hd_pop) bh <= KW'(wrap_inc(bh, BDEPTH));
      bn <= bn + KCW'(xfer) - KCW'(hd_pop);

      // ---- read responses (every cycle, independent of st) ----
      for (int k = 0; k < 3; k++) begin
        rsp_v[k] <= gnt[k];
        if (gnt[k]) rsp_slot[k] <= port_slot[k];
        if (rsp_v[k] && rdv_valid[k]) begin
          if (rdv_err[k]) begin
            if (xfer)                 k_err[bt] <= 1'b1;   // the instruction is being handed to the back half
            else if (f_xd)            k_err[xk] <= 1'b1;
            else if (st != ST_IDLE)   err_f <= 1'b1;       // (IDLE: a CHK-trapped instruction's early read)
          end
          else op[rsp_slot[k]] <= rdv_data[k];
        end
      end

      // ---- X1 -> X2 (integer only; f32 goes through the engine) ----
      if (x1_v) begin
        x1_v <= 1'b0;
        x2_v <= 1'b1;
      end
      // ---- f32 engine output: one complete beat per done pulse ----
      c_eng <= c_eng + QCW'(fp_start) - QCW'(fp_done);
      if (fp_done) begin
        k_fl[ko] <= k_fl[ko] | fp_flags;
        k_bd[ko] <= k_bd[ko] + 1'b1;
        if (k_ureg[ko]) begin
          for (int i = 0; i < NL; i++) k_cmp[ko][int'(od) * NL + i] <= fp_cmp[i] & exec_sd[i];
        end
      end
      // ---- X2 -> W / cmp accumulate ----
      if (x2_v) begin
        if (x2_ureg) begin
          for (int i = 0; i < NL; i++) k_cmp[xk][int'(xb) * NL + i] <= cmpb[i] & exec_sl[i];
          x2_v     <= 1'b0;
          k_bd[xk] <= k_bd[xk] + 1'b1;
        end else if (!w_v) begin
          x2_v     <= 1'b0;
          k_bd[xk] <= k_bd[xk] + 1'b1;
        end
      end
      // ---- result queue ----
      if (q_push) begin
        q_data[q_wp]  <= fp_res;
        q_lanes[q_wp] <= exec_sd;
        q_k[q_wp]     <= ko;
        q_beat[q_wp]  <= od;
        q_wp          <= QIW'(wrap_inc(q_wp, QD));
      end
      if (q_out) q_rp <= QIW'(wrap_inc(q_rp, QD));
      q_n <= q_n + QCW'(q_push) - QCW'(q_out);
      if (q_flush) begin q_n <= '0; q_rp <= '0; q_wp <= '0; end
      // ---- W stage load (only when empty) ----
      if (x2_w) begin
        w_v     <= 1'b1;
        w_ph    <= 1'b0;
        w_pair  <= (dq_b.d_kind == D_VPAIR);
        w_lo    <= r_lo_flat;
        w_hi    <= r_hi_flat;
        w_lanes <= exec_sl & lwe;
        w_reg   <= dq_b.rel_d ? rel_reg_b : dq_b.rd;
        w_beat  <= xb;
        w_slot  <= slot_b;
        w_k     <= xk;
      end else if (q_out) begin
        // queued beat (the W stage was busy when it was delivered, or the precise path)
        w_v     <= 1'b1;
        w_ph    <= 1'b0;
        w_pair  <= 1'b0;
        w_lo    <= q_data[q_rp];
        w_hi    <= '0;
        w_lanes <= q_lanes[q_rp];
        w_reg   <= k_rd[q_k[q_rp]];
        w_beat  <= q_beat[q_rp];
        w_slot  <= k_slot[q_k[q_rp]];
        w_k     <= q_k[q_rp];
      end else if (thr_w) begin
        // write-through: straight from the engine output
        w_v     <= 1'b1;
        w_ph    <= 1'b0;
        w_pair  <= 1'b0;
        w_lo    <= fp_res;
        w_hi    <= '0;
        w_lanes <= exec_sd;
        w_reg   <= k_rd[ko];
        w_beat  <= od;
        w_slot  <= k_slot[ko];
        w_k     <= ko;
      end
      // ---- W stage ----
      if (w_v) begin
        if (w_fin) k_wc[w_k] <= k_wc[w_k] + 1'b1;
        if (w_lanes == '0) begin
          w_v <= 1'b0;
        end else if (wr_ready) begin
          if (w_pair && !w_ph) w_ph <= 1'b1;
          else                 w_v  <= 1'b0;
        end
      end
    end
  end

  // ---- datapath registers without reset ----
  // uniform read request and capture, relative-register latch (front)
  always_ff @(posedge clk) begin
    if (st == ST_URW) begin
      if (urd_valid[0]) uv0 <= urd_data[0];
      if (urd_valid[1]) uv1 <= urd_data[1];
    end
    if (st == ST_PREP) begin
      rel_reg <= rel_sum[6:0];
      rel_bad <= rel_oor;
    end
  end

  // uniform read request / payload
  always_comb begin
    ur_valid = (st == ST_CHK && need_u0) ? {need_u1, 1'b1} : 2'b00;
    for (int k = 0; k < 2; k++) ur_slot[k] = slot_q;
    ur_reg[0] = u0_idx;
    ur_reg[1] = dq.rs1[5:0] + 6'd1;
  end

  // uniform write (cmp mask), in the completion cycle of the oldest context
  always_comb begin
    uw_valid = hd_base && k_ureg[bh] && !k_err[bh] && !ftrap_h && !k_late[bh];   // (no write-port term)
    uw_slot  = k_slot[bh];
    uw_reg   = k_rd[bh][5:0];
    uw_data  = k_cmp[bh];
  end

  assign iss_ready = (st == ST_IDLE) && !(excl_b && !b_idle);
  assign busy      = (st != ST_IDLE) || !b_idle;
  assign bbusy     = !b_idle;

`ifndef SYNTHESIS
  // simulation-only coverage of the front/back interplay (read by the SP-top TB)
  int unsigned sim_chk_defer = 0;   // cycles a CHK-stage trap waited for the back half to drain
  int unsigned sim_xfer_wait = 0;   // cycles a fully collected first beat waited for a back-half context
  int unsigned sim_excl_blk  = 0;   // cycles an issue was refused behind an exclusive (late-trap) instruction
  int unsigned sim_excl_n    = 0;   // exclusive instructions started
  int unsigned sim_late_trap = 0;   // late traps taken in the back half
  int unsigned sim_prep_defer = 0;  // cycles a movrel drop / trap waited for the back half to drain
  int unsigned sim_ctx2      = 0;   // cycles with >= 2 back-half contexts in use
  int unsigned sim_ctx3      = 0;   // cycles with >= 3 back-half contexts in use
  int unsigned sim_q_push    = 0;   // f32 results queued (W stage busy or precise path)
  int unsigned sim_x2_stall  = 0;   // cycles an integer X2 waited for the W stage
  int unsigned sim_gs_blk    = 0;   // cycles a read request was held back by gsafe
  int unsigned sim_err       = 0;   // internal invariant violations (must stay 0)
  always @(posedge clk) if (rst_n) begin
    if (st == ST_CHK && (chk_illegal || chk_range) && !b_idle) sim_chk_defer <= sim_chk_defer + 1;
    if (collecting && pend == 7'd0 && cb == '0 && !(PIPE ? ctx_ok : b_idle)) sim_xfer_wait <= sim_xfer_wait + 1;
    if (st == ST_IDLE && iss_valid && excl_b && !b_idle) sim_excl_blk <= sim_excl_blk + 1;
    if (st == ST_PREP && dq.rel_d && (rel_oor || rs1_oor_d) && !b_idle) sim_prep_defer <= sim_prep_defer + 1;
    if (xfer && excl_q) sim_excl_n <= sim_excl_n + 1;
    if (hd_rdy && !k_err[bh] && (ftrap_h || k_late[bh])) sim_late_trap <= sim_late_trap + 1;
    if (int'(bn) >= 2) sim_ctx2 <= sim_ctx2 + 1;
    if (int'(bn) >= 3) sim_ctx3 <= sim_ctx3 + 1;
    if (q_push) sim_q_push <= sim_q_push + 1;
    if (x2_v && !x2_go) sim_x2_stall <= sim_x2_stall + 1;
    if (collecting && (port_v != 3'd0) && !gsafe) sim_gs_blk <= sim_gs_blk + 1;
    // invariants
    if (x1_v && x2_v) begin sim_err <= sim_err + 1; $error("ng_valu: X1 and X2 both hold a beat"); end
    if (PIPE && xfer && (!fp_can || (x2_v && !x2_go))) begin sim_err <= sim_err + 1; $error("ng_valu: X context switch under an older reader"); end
    if (xfer && !(excl_q ? b_idle : (int'(bn) < BDEPTH))) begin sim_err <= sim_err + 1; $error("ng_valu: transfer without a free context"); end
    if (q_push && !q_out && (int'(q_n) == QD)) begin sim_err <= sim_err + 1; $error("ng_valu: result queue overflow"); end
    if (fp_done && c_eng == '0) begin sim_err <= sim_err + 1; $error("ng_valu: engine result without a credit"); end
    if (int'(c_eng) + int'(q_n) > QD) begin sim_err <= sim_err + 1; $error("ng_valu: engine + queue beats exceed the queue"); end
    if (q_flush && (q_push || int'(bn) != 1 || c_eng != '0)) begin sim_err <= sim_err + 1; $error("ng_valu: queue flush with other results in flight"); end
    if (b_idle && !fw_on && q_n != '0) begin sim_err <= sim_err + 1; $error("ng_valu: queued results without an instruction"); end
    if (fw_on && (bn != KCW'(1))) begin sim_err <= sim_err + 1; $error("ng_valu: precise write-back with another instruction in the back half"); end
    if (excl_b && !b_idle && int'(bn) > 1) begin sim_err <= sim_err + 1; $error("ng_valu: exclusive instruction shares the back half"); end
  end
`endif
endmodule
