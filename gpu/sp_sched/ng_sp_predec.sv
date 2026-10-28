// ng_sp_predec.sv -- scheduler predecoder.
//
// Classifies one fetched instruction for the SP scheduler.  It mirrors the
// functional model's decode decisions for the scheduler-visible subset
// (misc/sync/branch):
//   SYS   opcode 0x4e: nop/endpgm/trap/brk/sleep/setprio/barrier/icache.inv;
//         reserved f5, or a nonzero imm on a no-immediate form, is an illegal
//         instruction (the model's decoder rejects nonzero reserved bits).
//         cache.inv.l1 / cache.inv.l2 / cache.wb.l2 (f5 = 9/0xa/0xb, unit lsu)
//         are CLS_OTHER: issued to the pipe, which routes them to the LSU
//         (ng_lsu_dec K_CACHE; counter vm / vm / vst is counted by the LSU at
//         accept) .  interlock.begin/end (f5 = 7/8, unit gfx) stay
//         illegal in this RTL path (IllegalInsn, info = w0): sp_top has no
//         gfx unit. The functional model has separate gfx semantics and an
//         experimental CP path; see
// Model support does not enable this RTL path.
//   WAIT  opcode 0x50: waitcnt (field split per the frozen WAIT bitmap).
//   J     opcode 0x46: jmp (f3=0, resolved here), bexecz/bexecnz (f3=1/2, the
//         pipe evaluates EXEC); f3 3..7 reserved -> illegal.
//   B     opcode 0x4a: bz/bnz/bnone/bany (f2=0..3, the pipe evaluates the
//         uniform + EXEC); f2 4..7 reserved -> illegal.
//   w0[1:0]=11 is the reserved length encoding -> illegal.  Everything else
//   is CLS_OTHER and is issued to the execution pipe untouched; full decode
//   legality for those opcodes belongs to the decode stage (+).
//   JL opcode 0x48 (jal) and JR opcode 0x4c (jr/jalr) stay CLS_OTHER (the
//   class names the trace mnemonic) but set `wbr`: like bz..bexecnz they are
//   resolved by the SALU/branch unit and the scheduler must wait for br_*
//   (before, they committed pc+4).
//
// Purely combinational.
module ng_sp_predec (
  input  logic [63:0] insn,        // {w1, w0}
  output logic [3:0]  cls,
  output logic        ill,         // illegal encoding: trap, info = w0
  output logic        tvalid,      // explicit trap/brk: trap with tcause/tinfo
  output logic [3:0]  tcause,
  output logic [31:0] tinfo,
  output logic [23:0] boff,        // signed byte offset (jmp target = pc + boff)
  output logic        len8,        // 64-bit encoding: sequential pc += 8
  output logic        wbr,         // issued instruction resolves its next pc through br_*
  output logic [5:0]  wfield [6]   // raw WAIT fields vm/vst/tex/ls/sm/sfu
);
  import ng_sp_sched_pkg::*;

  logic [31:0] w0;
  logic [6:0]  op;
  logic [31:12] imm20;
  logic [4:0]  f5;
  logic [2:0]  f3;
  logic [1:0]  f2;
  logic [21:0] jimm22;
  logic [16:0] bimm17;
  assign w0     = insn[31:0];
  assign op     = w0[6:0];
  assign imm20  = w0[31:12];
  assign f5     = w0[11:7];
  assign f3     = w0[9:7];
  assign f2     = w0[14:13];
  assign jimm22 = w0[31:10];
  assign bimm17 = w0[31:15];

  // WAIT bitmap: sm[31:30] sfu[29:27] ls[26:23] vst[22:18] tex[17:13] vm[12:7].
  assign wfield[CNT_VM]  = {4'b0, w0[12:7]};
  assign wfield[CNT_VST] = {5'b0, w0[22:18]};
  assign wfield[CNT_TEX] = {5'b0, w0[17:13]};
  assign wfield[CNT_LS]  = {2'b0, w0[26:23]};
  assign wfield[CNT_SM]  = {4'b0, w0[31:30]};
  assign wfield[CNT_SFU] = {3'b0, w0[29:27]};

  always_comb begin
    cls    = CLS_OTHER;
    ill    = 1'b0;
    tvalid = 1'b0;
    tcause = CAUSE_ILLEGAL;
    tinfo  = w0;
    boff   = '0;
    len8   = (w0[1:0] == 2'b01);
    if (w0[1:0] == 2'b11) begin
      ill = 1'b1;                            // reserved length encoding
    end else if (op == 7'h4e) begin          // SYS
      case (f5)
        5'h0:  begin cls = CLS_NOP;     ill = (imm20 != '0); end
        5'h1:  begin cls = CLS_ENDPGM;  ill = (imm20 != '0); end
        5'h2:  begin tvalid = 1'b1; tcause = CAUSE_EXPLICIT; tinfo = imm20; end
        5'h3:  begin tvalid = 1'b1; tcause = CAUSE_BRK;      tinfo = '0; ill = (imm20 != '0); end
        5'h4:  cls = CLS_SLEEP;
        5'h5:  cls = CLS_SETPRIO;
        5'h6:  begin cls = CLS_BARRIER; ill = (imm20 != '0); end
        5'hc:  begin cls = CLS_ICACHE;  ill = (imm20 != '0); end
        // cache.inv.l1 / cache.inv.l2 / cache.wb.l2 -> pipe (LSU)
        5'h9, 5'ha, 5'hb: begin cls = CLS_OTHER; ill = (imm20 != '0); end
        // 5'h7 / 5'h8 interlock.begin / interlock.end: no gfx unit -> illegal (default)
        default: ill = 1'b1;
      endcase
    end else if (op == 7'h50) begin          // WAIT
      cls = CLS_WAITCNT;
    end else if (op == 7'h46) begin          // J
      case (f3)
        3'd0: begin
          cls  = CLS_JMP;
          boff = {{2{jimm22[21]}}, jimm22, 2'b00};  // br22: signed, scale 4
        end
        3'd1: cls = CLS_BEXECZ;
        3'd2: cls = CLS_BEXECNZ;
        default: ill = 1'b1;
      endcase
    end else if (op == 7'h4a) begin          // B
      case (f2)
        2'd0: cls = CLS_BZ;
        2'd1: cls = CLS_BNZ;
        2'd2: cls = CLS_BNONE;
        2'd3: cls = CLS_BANY;
        default: ill = 1'b1;                 // f2 4..7 unused in v0.1
      endcase
      // bimm17 is evaluated by the pipe from the raw word; not needed here.
      if (!ill) boff = {{7{bimm17[16]}}, bimm17, 2'b00};  // br17 for the TB
    end
  end

  // waits for branch feedback: the pipe-resolved branch classes, jal, jr/jalr
  assign wbr = !ill && ((cls == CLS_BZ) || (cls == CLS_BNZ) || (cls == CLS_BNONE) || (cls == CLS_BANY) ||
                        (cls == CLS_BEXECZ) || (cls == CLS_BEXECNZ) ||
                        (cls == CLS_OTHER && w0[1:0] != 2'b11 && (op == 7'h48 || op == 7'h4c)));
endmodule
