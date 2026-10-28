// ng_sp_sched_pkg.sv -- shared constants for the SP warp scheduler.
//
// Warp state machine, instruction predecode classes, trap causes (values are
// the functional model's TrapCause codes) and the
// six waitcnt counters (`counters:`, in YAML order).
package ng_sp_sched_pkg;

  // Warp slot states.  EMPTY slots are allocatable; READY warps are selectable
  // by the round-robin arbiter; WAIT is a waitcnt block; BAR is a workgroup
  // barrier arrival; TRAP is terminal (v0.1: halt and report; the slot stays
  // allocated until the CP disposes of the workgroup).  The model's Done
  // state maps to the ev_warp_done event: the slot frees at endpgm.
  typedef enum logic [2:0] {
    W_EMPTY = 3'd0,
    W_READY = 3'd1,
    W_WAIT  = 3'd2,
    W_BAR   = 3'd3,
    W_DONE  = 3'd4,  // reserved: done warps free their slot (see comment)
    W_TRAP  = 3'd5,
    W_BRWAIT = 3'd6, // branch issued, waiting for its resolution (only this warp is parked)
    W_ISSUE_PEND = 3'd7  // instruction sits in the issue buffer, not yet accepted by the pipe
  } warp_state_e;

  // Instruction class as seen by the scheduler predecoder (ng_sp_predec).
  // One code per trace-visible mnemonic so a commit event can be printed
  // with the same name the functional model's ISA table uses.
  typedef enum logic [3:0] {
    CLS_OTHER   = 4'd0,   // any opcode outside the predecoded set -> pipe
    CLS_NOP     = 4'd1,
    CLS_SLEEP   = 4'd2,
    CLS_SETPRIO = 4'd3,
    CLS_ICACHE  = 4'd4,   // icache.inv
    CLS_WAITCNT = 4'd5,
    CLS_BARRIER = 4'd6,
    CLS_ENDPGM  = 4'd7,
    CLS_JMP     = 4'd8,   // resolved in the scheduler (direct branch)
    CLS_BZ      = 4'd9,   // resolved by the pipe via br_* (reads uniforms)
    CLS_BNZ     = 4'd10,
    CLS_BNONE   = 4'd11,
    CLS_BANY    = 4'd12,
    CLS_BEXECZ  = 4'd13,
    CLS_BEXECNZ = 4'd14,
    CLS_NONE    = 4'd15
  } insn_class_e;

  // Trap causes: functional-model values (csr.h TrapCause).  Only the causes
  // the scheduler itself can raise are listed.
  localparam logic [3:0] CAUSE_ILLEGAL = 4'd1;  // illegal/reserved encoding
  localparam logic [3:0] CAUSE_EXPLICIT = 4'd2; // `trap imm`
  localparam logic [3:0] CAUSE_BRK = 4'd3;      // `brk`
  localparam logic [3:0] CAUSE_MEMFAULT = 4'd5; // instruction fetch fault

  // waitcnt counters, nicogpu_isa.yaml order.  Field widths are the WAIT-format
  // field widths; storage saturates at the per-counter outstanding maximum.
  localparam int CNT_N = 6;
  localparam int CNT_VM = 0, CNT_VST = 1, CNT_TEX = 2, CNT_LS = 3, CNT_SM = 4, CNT_SFU = 5;
  localparam int CNT_FIELD_W[CNT_N] = '{6, 5, 5, 4, 2, 3};  // WAIT field bits
  localparam int CNT_SAT[CNT_N]     = '{63, 31, 31, 15, 15, 7};  // saturate value

  // Trace-visible mnemonic for each class (index = class code).
  localparam string CLS_NAME[16] = '{
    "other", "nop", "sleep", "setprio", "icache.inv", "waitcnt", "barrier",
    "endpgm", "jmp", "bz", "bnz", "bnone", "bany", "bexecz", "bexecnz", "none"
  };

endpackage
