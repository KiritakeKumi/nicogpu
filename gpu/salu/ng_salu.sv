// ng_salu.sv -- SP scalar/uniform execution unit: SALU + branch + sync/misc/CSR.
//
// Executes the 107 instructions of the ISA that run on the uniform side of an
// SP, with the semantics of (SALU, EXEC
// manipulation, branches) and (CSR, sync,
// misc):
//   * 81 SALU:   u* arithmetic/logic/shift/compare/bit-field/64-bit, immediate
//                and literal forms, exec.get/exec.set/saveexec.*/exec.wqm,
//                umovi/upc/upca;
//   * 10 branch: jmp bexecz bexecnz jal bz bnz bnone bany jr jalr (resolved
//                here, reported through br_*);
//   * 16 misc/sync/CSR: nop endpgm trap brk setprio sleep barrier icache.inv
//                waitcnt csrr csrw csrs csrc csrwi csrsi csrci.
// Per-warp state kept in the unit: EXEC and the CSR block (mode, fflags,
// trapctl, trapbase, trapcause, trappc, trapinfo) plus the launch parameters
// code_lo/hi, scratch_lo/hi/size, wgslot written by the integration layer.
//
// Port conventions mirror ng_valu: one instruction in flight (iss_valid /
// iss_ready), exactly one done_valid or trap_valid pulse per accepted
// instruction, uniform register file ports ur_*/uw_* (ng_sp_regfile, latency 1),
// precise traps (all checks complete before any state is touched).  Branches
// additionally pulse br_valid (same cycle as done_valid).  Docs:
//
// Pipeline: issue -> CHK (decode checks) -> RD0/RW0 (u[rs1], u[rs2]) ->
// [RD1/RW1 (high words of 64-bit pairs)] -> MUL (33x33 product register) ->
// EX (result, trap, state-update decision) -> WR (uniform writes, state update,
// completion pulses registered) -> idle.
module ng_salu
  import ng_salu_pkg::*;
  import ng_bitops_pkg::clz32, ng_bitops_pkg::ctz32, ng_bitops_pkg::cpop32, ng_bitops_pkg::brev32,
         ng_bitops_pkg::bfm_mask;
#(
  parameter int SLOTW = 4    // warp slot id width (16 slots)
) (
  input  logic              clk,
  input  logic              rst_n,

  // ---- issue ---------------------------------------------------------------
  input  logic              iss_valid,
  output logic              iss_ready,
  input  logic [31:0]       iss_w0,       // instruction word 0 (bit[1:0]==01: 64-bit)
  input  logic [31:0]       iss_w1,       // word 1 (64-bit encodings)
  input  logic [31:0]       iss_pc,       // code offset of this instruction (PC - CODE_BASE)
  input  logic [SLOTW-1:0]  iss_slot,
  input  logic [15:0]       iss_tag,      // opaque, returned with done

  // ---- completion (one-cycle pulses; exactly one per accepted instruction) ---
  output logic              done_valid,
  output logic [15:0]       done_tag,
  output logic              trap_valid,
  output logic [3:0]        trap_cause,
  output logic [31:0]       trap_info,
  output logic              busy,

  // ---- branch resolution (pulses together with done_valid) -------------------
  output logic              br_valid,
  output logic [SLOTW-1:0]  br_slot,
  output logic              br_taken,
  output logic [31:0]       br_target,    // code offset

  // ---- sync / misc event (pulses together with done_valid) --------------------
  output logic              ev_valid,
  output logic [SLOTW-1:0]  ev_slot,
  output logic [2:0]        ev_kind,      // EV_* in ng_salu_pkg
  output logic [19:0]       ev_imm,       // sleep / setprio immediate
  output logic [24:0]       ev_wait,      // waitcnt WAIT-format word bits [31:7]: {sm,sfu,ls,vst,tex,vm}

  // ---- uniform register file ---------------------------------------------------
  output logic [1:0]        ur_valid,
  input  logic              ur_ready,     // request accepted this cycle (tie 1 if never stalled)
  output logic [SLOTW-1:0]  ur_slot [2],
  output logic [5:0]        ur_reg  [2],
  input  logic [1:0]        urd_valid,    // response, 1 cycle after an accepted request
  input  logic [31:0]       urd_data [2],

  output logic [1:0]        uw_valid,
  input  logic              uw_ready,     // write accepted this cycle (tie 1 if never stalled)
  output logic [SLOTW-1:0]  uw_slot [2],
  output logic [5:0]        uw_reg  [2],
  output logic [31:0]       uw_data [2],

  // ---- per-warp state written by the integration layer (launch) ----------------
  input  logic              winit_valid,  // (re)initialise a warp slot: CSRs <- 0, EXEC and launch params loaded
  input  logic [SLOTW-1:0]  winit_slot,
  input  logic [31:0]       winit_exec,
  input  logic [31:0]       winit_code_lo,
  input  logic [31:0]       winit_code_hi,
  input  logic [31:0]       winit_scratch_lo,
  input  logic [31:0]       winit_scratch_hi,
  input  logic [31:0]       winit_scratch_size,
  input  logic [31:0]       winit_wgslot,

  // ---- static / free-running hardware information -----------------------------
  input  logic [31:0]       sp_id,
  input  logic [31:0]       core_id,
  output logic [SLOTW-1:0]  cur_slot,     // slot of the instruction in flight (selects clock/time below)
  input  logic [63:0]       hw_clock,     // retired-instruction counter of cur_slot's warp
  input  logic [63:0]       hw_time,      // core-wide commit counter

  // ---- state query for the other units (combinational) -------------------------
  input  logic [SLOTW-1:0]  q_slot,
  output logic [31:0]       q_exec,
  output logic [31:0]       q_mode,
  output logic [4:0]        q_fflags,
  output logic [31:0]       q_trapctl,
  input  logic [SLOTW-1:0]  qc_slot,        // second EXEC read port (commit trace; no issue/commit contention)
  output logic [31:0]       qc_exec,

  // ---- updates from the other units ----------------------------------------------
  input  logic              fflags_set_valid,   // fflags |= bits (FP unit)
  input  logic [SLOTW-1:0]  fflags_set_slot,
  input  logic [4:0]        fflags_set_bits,
  input  logic              xtrap_valid,        // trap raised by another unit: latch trapcause/pc/info
  input  logic [SLOTW-1:0]  xtrap_slot,
  input  logic [3:0]        xtrap_cause,
  input  logic [31:0]       xtrap_pc,
  input  logic [31:0]       xtrap_info
);
  localparam int NW = 1 << SLOTW;

  // ---------------------------------------------------------------- per-warp state
  logic [31:0] exec_r   [NW];
  logic [31:0] mode_r   [NW];
  logic [4:0]  fflags_r [NW];
  logic [31:0] trapctl_r [NW];
  logic [31:0] trapbase_r [NW];
  logic [31:0] trapcause_r [NW];
  logic [31:0] trappc_r [NW];
  logic [31:0] trapinfo_r [NW];
  logic [31:0] code_lo_r [NW], code_hi_r [NW];
  logic [31:0] scr_lo_r [NW], scr_hi_r [NW], scr_sz_r [NW], wgslot_r [NW];

  assign q_exec    = exec_r[q_slot];
  assign qc_exec   = exec_r[qc_slot];
  assign q_mode    = mode_r[q_slot];
  assign q_fflags  = fflags_r[q_slot];
  assign q_trapctl = trapctl_r[q_slot];

  // ---------------------------------------------------------------- control state
  typedef enum logic [3:0] {
    S_IDLE, S_CHK, S_RD0, S_RW0, S_RD1, S_RW1, S_MUL, S_EX, S_WR
  } st_t;
  st_t st;

  logic [31:0]      w0_q, w1_q, pc_q;
  logic [SLOTW-1:0] slot_q;
  logic [15:0]      tag_q;
  dec_t             dq;
  dec_t             dec_c;
  ng_salu_dec u_dec (.w0(iss_w0), .w1(iss_w1), .d(dec_c));

  assign cur_slot = slot_q;

  // operands
  logic [31:0] a_lo, a_hi, b_lo, b_hi;
  logic [65:0] prod_q;

  // ---------------------------------------------------------------- helper functions
  function automatic logic [31:0] wqm32(input logic [31:0] m);
    logic [31:0] r;
    for (int q = 0; q < 8; q++) r[4*q +: 4] = {4{|m[4*q +: 4]}};
    return r;
  endfunction

  // CSR classification
  function automatic logic csr_known(input logic [9:0] c);
    case (c)
      CSR_EXEC, CSR_MODE, CSR_FFLAGS, CSR_TRAPCTL, CSR_WARPID, CSR_SPID, CSR_COREID, CSR_WGSLOT,
      CSR_CLOCK, CSR_CLOCKH, CSR_TIME, CSR_TIMEH, CSR_SCRATCH_LO, CSR_SCRATCH_HI, CSR_SCRATCH_SIZE,
      CSR_CODE_LO, CSR_CODE_HI, CSR_TRAPBASE, CSR_TRAPCAUSE, CSR_TRAPPC, CSR_TRAPINFO: return 1'b1;
      default: return 1'b0;
    endcase
  endfunction
  function automatic logic csr_ro(input logic [9:0] c);
    case (c)
      CSR_WARPID, CSR_SPID, CSR_COREID, CSR_WGSLOT, CSR_CLOCK, CSR_CLOCKH, CSR_TIME, CSR_TIMEH,
      CSR_SCRATCH_LO, CSR_SCRATCH_HI, CSR_SCRATCH_SIZE, CSR_CODE_LO, CSR_CODE_HI: return 1'b1;
      default: return 1'b0;
    endcase
  endfunction
  // MODE reserved-value check (model misc_semantics.cpp mode_illegal)
  function automatic logic mode_illegal(input logic [31:0] v);
    logic [2:0] rm32, rm16, rm64;
    rm32 = v[2:0]; rm16 = v[5:3]; rm64 = v[8:6];
    if (v & ~32'h001f7fff) return 1'b1;
    if (rm32 > 3'd4) return 1'b1;
    if (rm16 > 3'd4 && rm16 != 3'd7) return 1'b1;
    if (rm64 > 3'd4 && rm64 != 3'd7) return 1'b1;
    return 1'b0;
  endfunction

  // --------------------------------------------------------------
  logic chk_illegal;
  always_comb begin
    chk_illegal = !dq.hit ||
                  (dq.rd_pair  && dq.rd[0]) ||
                  (dq.rs1_pair && dq.rs1[0]) ||
                  (dq.rs2_pair && dq.rs2[0]);
  end
  wire need_rd0 = dq.rs1_rd || dq.rs2_rd;
  wire need_rd1 = dq.rs1_pair || dq.rs2_pair;

  // ---------------------------------------------------------------- multiplier (MUL state)
  wire mul_sa = (dq.op == O_MULH) || (dq.op == O_MULHSU);
  wire mul_sb = (dq.op == O_MULH);
  wire [31:0] opb = dq.b_imm ? dq.imm : b_lo;
  wire signed [32:0] ma = {mul_sa & a_lo[31], a_lo};
  wire signed [32:0] mb = {mul_sb & opb[31], opb};
  wire signed [65:0] prod_c = ma * mb;

  // ---------------------------------------------------------------- EX (combinational on registered operands)
  logic [31:0] x_lo, x_hi;          // result words
  logic        x_wlo, x_whi;        // write u[rd] / u[rd+1]
  logic        x_exec_we;
  logic [31:0] x_exec;
  logic        x_trap;
  logic [3:0]  x_cause;
  logic [31:0] x_info;
  logic        x_br_taken;
  logic [31:0] x_br_target;
  logic [2:0]  x_ev;
  // csr update
  logic        x_csr_we;
  logic [9:0]  x_csr;
  logic [31:0] x_csr_val;

  logic [31:0] ex_cur;               // EXEC of the instruction's warp
  logic [31:0] csr_old;
  logic [31:0] csr_src;
  logic [31:0] csr_cand;
  logic [31:0] pc_plus_imm;
  logic [31:0] pc_plus_4;
  logic [63:0] a64, b64, s64;

  always_comb begin
    ex_cur = exec_r[slot_q];
    pc_plus_imm = pc_q + dq.imm;
    pc_plus_4   = pc_q + 32'd4;
    a64 = {a_hi, a_lo};
    b64 = dq.b_imm ? {{32{dq.imm[31]}}, dq.imm} : {b_hi, b_lo};
    s64 = '0;

    x_lo = '0; x_hi = '0;
    x_wlo = dq.rd_wr; x_whi = dq.rd_pair && dq.rd_wr;
    x_exec_we = 1'b0; x_exec = ex_cur;
    x_trap = 1'b0; x_cause = CAUSE_ILLEGAL; x_info = w0_q;
    x_br_taken = 1'b0; x_br_target = pc_plus_imm;
    x_ev = EV_NONE;
    x_csr_we = 1'b0; x_csr = dq.csr; x_csr_val = '0;

    // CSR read
    case (dq.csr)
      CSR_EXEC:         csr_old = ex_cur;
      CSR_MODE:         csr_old = mode_r[slot_q];
      CSR_FFLAGS:       csr_old = {27'd0, fflags_r[slot_q]};
      CSR_TRAPCTL:      csr_old = trapctl_r[slot_q];
      CSR_WARPID:       csr_old = 32'(slot_q);
      CSR_SPID:         csr_old = sp_id;
      CSR_COREID:       csr_old = core_id;
      CSR_WGSLOT:       csr_old = wgslot_r[slot_q];
      CSR_CLOCK:        csr_old = hw_clock[31:0];
      CSR_CLOCKH:       csr_old = hw_clock[63:32];
      CSR_TIME:         csr_old = hw_time[31:0];
      CSR_TIMEH:        csr_old = hw_time[63:32];
      CSR_SCRATCH_LO:   csr_old = scr_lo_r[slot_q];
      CSR_SCRATCH_HI:   csr_old = scr_hi_r[slot_q];
      CSR_SCRATCH_SIZE: csr_old = scr_sz_r[slot_q];
      CSR_CODE_LO:      csr_old = code_lo_r[slot_q];
      CSR_CODE_HI:      csr_old = code_hi_r[slot_q];
      CSR_TRAPBASE:     csr_old = trapbase_r[slot_q];
      CSR_TRAPCAUSE:    csr_old = trapcause_r[slot_q];
      CSR_TRAPPC:       csr_old = trappc_r[slot_q];
      CSR_TRAPINFO:     csr_old = trapinfo_r[slot_q];
      default:          csr_old = '0;
    endcase
    csr_src = ((dq.op == O_CSRWI) || (dq.op == O_CSRSI) || (dq.op == O_CSRCI)) ? {26'd0, dq.rs1} : a_lo;
    case (dq.op)
      O_CSRS, O_CSRSI: csr_cand = csr_old | csr_src;
      O_CSRC, O_CSRCI: csr_cand = csr_old & ~csr_src;
      default:         csr_cand = csr_src;
    endcase
    // fflags combine on the masked old value, result masked
    if (dq.csr == CSR_FFLAGS) begin
      case (dq.op)
        O_CSRS, O_CSRSI: csr_cand = ({27'd0, fflags_r[slot_q]} | (csr_src & 32'h1f)) & 32'h1f;
        O_CSRC, O_CSRCI: csr_cand = ({27'd0, fflags_r[slot_q]} & ~(csr_src & 32'h1f)) & 32'h1f;
        default:         csr_cand = csr_src & 32'h1f;
      endcase
    end

    case (dq.op)
      // ---- binary ALU (a = u[rs1], b = u[rs2] or immediate)
      O_ADD:  x_lo = a_lo + opb;
      O_SUB:  x_lo = a_lo - opb;
      O_SLL:  x_lo = a_lo << opb[4:0];
      O_SLT:  x_lo = {31'd0, $signed(a_lo) < $signed(opb)};
      O_SLTU: x_lo = {31'd0, a_lo < opb};
      O_XOR:  x_lo = a_lo ^ opb;
      O_SRL:  x_lo = a_lo >> opb[4:0];
      O_SRA:  x_lo = 32'($signed(a_lo) >>> opb[4:0]);
      O_OR:   x_lo = a_lo | opb;
      O_AND:  x_lo = a_lo & opb;
      O_MIN:  x_lo = ($signed(a_lo) < $signed(opb)) ? a_lo : opb;
      O_MAX:  x_lo = ($signed(a_lo) > $signed(opb)) ? a_lo : opb;
      O_MINU: x_lo = (a_lo < opb) ? a_lo : opb;
      O_MAXU: x_lo = (a_lo > opb) ? a_lo : opb;
      O_ANDN: x_lo = a_lo & ~opb;
      O_ORN:  x_lo = a_lo | ~opb;
      O_XNOR: x_lo = ~(a_lo ^ opb);
      O_MUL:  x_lo = prod_q[31:0];
      O_MULH, O_MULHU, O_MULHSU: x_lo = prod_q[63:32];
      O_CZEQ: x_lo = (opb == 32'd0) ? 32'd0 : a_lo;
      O_CZNE: x_lo = (opb != 32'd0) ? 32'd0 : a_lo;
      O_SEQ:  x_lo = {31'd0, a_lo == opb};
      O_SNE:  x_lo = {31'd0, a_lo != opb};
      O_SGE:  x_lo = {31'd0, $signed(a_lo) >= $signed(opb)};
      O_SGEU: x_lo = {31'd0, a_lo >= opb};
      O_BFM:  x_lo = bfm_mask(a_lo, opb);
      O_BFEU: x_lo = (a_lo >> opb[4:0]) & bfm_mask({16'd0, opb[31:16]}, 32'd0);
      O_BFEI: begin
        logic [5:0]  w6;
        logic [31:0] fld;
        w6  = opb[21:16];
        fld = (a_lo >> opb[4:0]) & bfm_mask({26'd0, w6}, 32'd0);
        if (w6 == 6'd0) x_lo = 32'd0;
        else if (w6 >= 6'd32) x_lo = fld;
        else x_lo = ($signed(fld << (6'd32 - w6)) >>> (6'd32 - w6));
      end
      O_ROL:  x_lo = (a_lo << opb[4:0]) | (a_lo >> ((6'd32 - {1'b0, opb[4:0]}) & 6'd31));
      O_ROR:  x_lo = (a_lo >> opb[4:0]) | (a_lo << ((6'd32 - {1'b0, opb[4:0]}) & 6'd31));
      // ---- 64-bit
      O_ADD64: begin s64 = a64 + b64; x_lo = s64[31:0]; x_hi = s64[63:32]; end
      O_SUB64: begin s64 = a64 - b64; x_lo = s64[31:0]; x_hi = s64[63:32]; end
      O_ADDSX64: begin s64 = a64 + {{32{b_lo[31]}}, b_lo}; x_lo = s64[31:0]; x_hi = s64[63:32]; end
      O_ADDZX64: begin s64 = a64 + {32'd0, b_lo}; x_lo = s64[31:0]; x_hi = s64[63:32]; end
      // --
      O_MOV:  x_lo = a_lo;
      O_NOT:  x_lo = ~a_lo;
      O_CLZ:  x_lo = clz32(a_lo);
      O_CTZ:  x_lo = ctz32(a_lo);
      O_CPOP: x_lo = cpop32(a_lo);
      O_BREV: x_lo = brev32(a_lo);
      O_SEXTB: x_lo = {{24{a_lo[7]}}, a_lo[7:0]};
      O_SEXTH: x_lo = {{16{a_lo[15]}}, a_lo[15:0]};
      O_MOV64: begin x_lo = a_lo; x_hi = a_hi; end
      O_WQM:  x_lo = wqm32(a_lo);
      O_MOVI: x_lo = dq.imm;
      O_UPC:  x_lo = pc_plus_imm;
      O_UPCA: begin
        s64 = {code_hi_r[slot_q], code_lo_r[slot_q]} + {32'd0, pc_plus_imm};
        x_lo = s64[31:0]; x_hi = s64[63:32];
      end
      // --
      O_EXEC_GET: x_lo = ex_cur;
      O_EXEC_SET: begin x_exec_we = 1'b1; x_exec = a_lo; end
      O_SVE_AND:   begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = ex_cur & a_lo; end
      O_SVE_OR:    begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = ex_cur | a_lo; end
      O_SVE_XOR:   begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = ex_cur ^ a_lo; end
      O_SVE_ANDN1: begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = a_lo & ~ex_cur; end
      O_SVE_ANDN2: begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = ex_cur & ~a_lo; end
      O_SVE_MOV:   begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = a_lo; end
      O_EXEC_WQM:  begin x_lo = ex_cur; x_exec_we = 1'b1; x_exec = wqm32(ex_cur); end
      // --
      O_JMP:    x_br_taken = 1'b1;
      O_BEXECZ:  x_br_taken = (ex_cur == 32'd0);
      O_BEXECNZ: x_br_taken = (ex_cur != 32'd0);
      O_JAL:    begin x_br_taken = 1'b1; x_lo = pc_plus_4; end
      O_BZ:     x_br_taken = (a_lo == 32'd0);
      O_BNZ:    x_br_taken = (a_lo != 32'd0);
      O_BNONE:  x_br_taken = ((a_lo & ex_cur) == 32'd0);
      O_BANY:   x_br_taken = ((a_lo & ex_cur) != 32'd0);
      O_JR:     begin x_br_taken = 1'b1; x_br_target = (a_lo + dq.imm) & 32'hfffffffc; end
      O_JALR:   begin x_br_taken = 1'b1; x_br_target = (a_lo + dq.imm) & 32'hfffffffc; x_lo = pc_plus_4; end
      // ---- sync / misc
      O_NOP:     x_ev = EV_NOP;
      O_ENDPGM:  x_ev = EV_ENDPGM;
      O_BARRIER: x_ev = EV_BARRIER;
      O_WAITCNT: x_ev = EV_WAITCNT;
      O_SLEEP:   x_ev = EV_SLEEP;
      O_ICACHE:  x_ev = EV_ICACHE;
      O_SETPRIO: x_ev = EV_SETPRIO;
      O_TRAP:    begin x_trap = 1'b1; x_cause = CAUSE_EXPLICIT; x_info = dq.imm; end
      O_BRK:     begin x_trap = 1'b1; x_cause = CAUSE_BRK; x_info = 32'd0; end
      // --
      O_CSRR: begin
        if (!csr_known(dq.csr)) begin x_trap = 1'b1; x_cause = CAUSE_ILLCSR; x_info = {22'd0, dq.csr}; end
        else x_lo = csr_old;
      end
      O_CSRW, O_CSRS, O_CSRC, O_CSRWI, O_CSRSI, O_CSRCI: begin
        if (!csr_known(dq.csr) || csr_ro(dq.csr)) begin
          x_trap = 1'b1; x_cause = CAUSE_ILLCSR; x_info = {22'd0, dq.csr};
        end else if (dq.csr == CSR_MODE && mode_illegal(csr_cand)) begin
          x_trap = 1'b1; x_cause = CAUSE_ILLMODE; x_info = csr_cand;
        end else begin
          x_csr_we = 1'b1; x_csr_val = csr_cand;
          if (dq.csr == CSR_EXEC) begin x_exec_we = 1'b1; x_exec = csr_cand; end
        end
      end
      default: ;
    endcase
    // instructions with no destination write nothing; csrr writes only when legal
    if (!dq.rd_wr) begin x_wlo = 1'b0; x_whi = 1'b0; end
    if (x_trap) begin x_wlo = 1'b0; x_whi = 1'b0; x_exec_we = 1'b0; x_csr_we = 1'b0; end
  end

  // --------------------------------------------------------------
  logic        f_trap, f_br, f_wlo, f_whi;
  logic [3:0]  f_cause;
  logic [31:0] f_info;
  logic        f_taken;
  logic [31:0] f_target;
  logic [2:0]  f_ev;
  logic [31:0] f_lo, f_hi;
  logic        f_exec_we;
  logic [31:0] f_exec;
  logic        f_csr_we;
  logic [9:0]  f_csr;
  logic [31:0] f_csr_val;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE;
      done_valid <= 1'b0; trap_valid <= 1'b0; br_valid <= 1'b0; ev_valid <= 1'b0;
      done_tag <= '0; trap_cause <= '0; trap_info <= '0;
      br_slot <= '0; br_taken <= 1'b0; br_target <= '0;
      ev_slot <= '0; ev_kind <= EV_NONE; ev_imm <= '0; ev_wait <= '0;
    end else begin
      done_valid <= 1'b0; trap_valid <= 1'b0; br_valid <= 1'b0; ev_valid <= 1'b0;
      case (st)
        S_IDLE: if (iss_valid) st <= S_CHK;

        S_CHK: begin
          if (chk_illegal) begin
            trap_valid <= 1'b1; trap_cause <= CAUSE_ILLEGAL; trap_info <= w0_q;
            st <= S_IDLE;
          end else if (need_rd0) begin
            st <= S_RD0;
          end else begin
            st <= S_MUL;
          end
        end

        S_RD0: if (ur_ready) st <= S_RW0;
        S_RW0: st <= need_rd1 ? S_RD1 : S_MUL;
        S_RD1: if (ur_ready) st <= S_RW1;
        S_RW1: st <= S_MUL;

        S_MUL: st <= S_EX;
        S_EX:  st <= S_WR;

        S_WR: begin
          if (!(f_wlo || f_whi) || uw_ready) begin
            st <= S_IDLE;
            if (f_trap) begin
              trap_valid <= 1'b1; trap_cause <= f_cause; trap_info <= f_info;
            end else begin
              done_valid <= 1'b1; done_tag <= tag_q;
              br_valid <= f_br; br_slot <= slot_q; br_taken <= f_taken; br_target <= f_target;
              ev_valid <= (f_ev != EV_NONE); ev_slot <= slot_q; ev_kind <= f_ev;
              ev_imm <= dq.imm[19:0]; ev_wait <= w0_q[31:7];
            end
          end
        end
        default: st <= S_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------- datapath registers (no reset)
  always_ff @(posedge clk) begin
    if (st == S_IDLE && iss_valid) begin
      w0_q <= iss_w0; w1_q <= iss_w1; pc_q <= iss_pc; slot_q <= iss_slot; tag_q <= iss_tag;
      dq <= dec_c;
    end
    if (st == S_RW0) begin
      if (dq.rs1_rd && urd_valid[0]) a_lo <= urd_data[0];
      if (dq.rs2_rd && urd_valid[1]) b_lo <= urd_data[1];
    end
    if (st == S_RW1) begin
      if (dq.rs1_pair && urd_valid[0]) a_hi <= urd_data[0];
      if (dq.rs2_pair && urd_valid[1]) b_hi <= urd_data[1];
    end
    if (st == S_MUL) prod_q <= 66'(prod_c);
    if (st == S_EX) begin
      f_trap <= x_trap; f_cause <= x_cause; f_info <= x_info;
      f_br <= dq.is_br; f_taken <= x_br_taken; f_target <= x_br_target;
      f_ev <= x_ev;
      f_lo <= x_lo; f_hi <= x_hi; f_wlo <= x_wlo; f_whi <= x_whi;
      f_exec_we <= x_exec_we; f_exec <= x_exec;
      f_csr_we <= x_csr_we; f_csr <= x_csr; f_csr_val <= x_csr_val;
    end
  end

  // uniform register file requests
  always_comb begin
    ur_valid = 2'b00;
    for (int k = 0; k < 2; k++) ur_slot[k] = slot_q;
    ur_reg[0] = dq.rs1;
    ur_reg[1] = dq.rs2;
    if (st == S_RD0) begin
      ur_valid = {dq.rs2_rd, dq.rs1_rd};
    end else if (st == S_RD1) begin
      ur_valid  = {dq.rs2_pair, dq.rs1_pair};
      ur_reg[0] = dq.rs1 + 6'd1;
      ur_reg[1] = dq.rs2 + 6'd1;
    end
  end

  // uniform register file writes (low and high word on the two write ports)
  always_comb begin
    uw_valid = (st == S_WR) ? {f_whi, f_wlo} & {2{!f_trap}} : 2'b00;
    for (int k = 0; k < 2; k++) uw_slot[k] = slot_q;
    uw_reg[0]  = dq.rd;
    uw_reg[1]  = dq.rd + 6'd1;
    uw_data[0] = f_lo;
    uw_data[1] = f_hi;
  end

  assign iss_ready = (st == S_IDLE);
  assign busy      = (st != S_IDLE);

  // ---------------------------------------------------------------- per-warp state update
  wire wr_done = (st == S_WR) && (!(f_wlo || f_whi) || uw_ready);

  always_ff @(posedge clk) begin
    // illegal-encoding trap found in CHK
    if (st == S_CHK && chk_illegal) begin
      trapcause_r[slot_q] <= {28'd0, CAUSE_ILLEGAL};
      trappc_r[slot_q]    <= pc_q;
      trapinfo_r[slot_q]  <= w0_q;
    end
    // internal commit (instruction in WR, completing this cycle)
    if (wr_done) begin
      if (f_trap) begin
        trapcause_r[slot_q] <= {28'd0, f_cause};
        trappc_r[slot_q]    <= pc_q;
        trapinfo_r[slot_q]  <= f_info;
      end else begin
        if (f_exec_we) exec_r[slot_q] <= f_exec;
        if (f_csr_we) begin
          case (f_csr)
            CSR_MODE:      mode_r[slot_q]      <= f_csr_val;
            CSR_FFLAGS:    fflags_r[slot_q]    <= f_csr_val[4:0];
            CSR_TRAPCTL:   trapctl_r[slot_q]   <= f_csr_val;
            CSR_TRAPBASE:  trapbase_r[slot_q]  <= f_csr_val;
            CSR_TRAPCAUSE: trapcause_r[slot_q] <= f_csr_val;
            CSR_TRAPPC:    trappc_r[slot_q]    <= f_csr_val;
            CSR_TRAPINFO:  trapinfo_r[slot_q]  <= f_csr_val;
            default: ;
          endcase
        end
      end
    end
    // FP unit flag accumulation (after the internal write: OR on top)
    if (fflags_set_valid) begin
      if (wr_done && !f_trap && f_csr_we && f_csr == CSR_FFLAGS && slot_q == fflags_set_slot)
        fflags_r[fflags_set_slot] <= f_csr_val[4:0] | fflags_set_bits;
      else
        fflags_r[fflags_set_slot] <= fflags_r[fflags_set_slot] | fflags_set_bits;
    end
    // trap raised by another unit
    if (xtrap_valid) begin
      trapcause_r[xtrap_slot] <= {28'd0, xtrap_cause};
      trappc_r[xtrap_slot]    <= xtrap_pc;
      trapinfo_r[xtrap_slot]  <= xtrap_info;
    end
    // launch / relaunch of a slot (highest priority)
    if (winit_valid) begin
      exec_r[winit_slot]      <= winit_exec;
      mode_r[winit_slot]      <= '0;
      fflags_r[winit_slot]    <= '0;
      trapctl_r[winit_slot]   <= '0;
      trapbase_r[winit_slot]  <= '0;
      trapcause_r[winit_slot] <= '0;
      trappc_r[winit_slot]    <= '0;
      trapinfo_r[winit_slot]  <= '0;
      code_lo_r[winit_slot]   <= winit_code_lo;
      code_hi_r[winit_slot]   <= winit_code_hi;
      scr_lo_r[winit_slot]    <= winit_scratch_lo;
      scr_hi_r[winit_slot]    <= winit_scratch_hi;
      scr_sz_r[winit_slot]    <= winit_scratch_size;
      wgslot_r[winit_slot]    <= winit_wgslot;
    end
  end
endmodule
