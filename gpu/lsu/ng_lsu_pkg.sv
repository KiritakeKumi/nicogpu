// ng_lsu_pkg.sv -- shared types of the LSU / L1D / shared-memory unit.
//
// Golden semantics: .
// Everything here is tentative  except the ISA-derived
// constants (trap causes = model csr.h TrapCause, counter ids = scheduler
// ng_sp_sched_pkg::CNT_*).
package ng_lsu_pkg;

  // ---------------- decoded instruction ----------------------------------
  localparam logic [2:0] K_LD    = 3'd1;
  localparam logic [2:0] K_ST    = 3'd2;
  localparam logic [2:0] K_ATOM  = 3'd3;
  localparam logic [2:0] K_ULD   = 3'd4;
  localparam logic [2:0] K_CACHE = 3'd5;

  // address space == M-format `sp` field
  localparam logic [2:0] SP_GLOBAL  = 3'd0;
  localparam logic [2:0] SP_UBASE   = 3'd1;
  localparam logic [2:0] SP_BUF     = 3'd2;
  localparam logic [2:0] SP_SHARED  = 3'd3;
  localparam logic [2:0] SP_SCRATCH = 3'd4;

  // uld variants
  localparam logic [1:0] UK_PLAIN = 2'd0;
  localparam logic [1:0] UK_X     = 2'd1;
  localparam logic [1:0] UK_BUF   = 2'd2;
  localparam logic [1:0] UK_BUFX  = 2'd3;

  // cache maintenance ops (ng_lsu_dec cache_op; 2 = cache.wb.l2, the remaining case of ng_lsu's mapping)
  localparam logic [1:0] CO_INV_L1 = 2'd0;
  localparam logic [1:0] CO_INV_L2 = 2'd1;

  // AMO op codes: 0..12 = ng_noc_pkg::AMO_* (the L2 ALU encoding), 13..15 the
  // FP32 atomics the L2 does not implement (executed in the LSU).
  localparam logic [4:0] AMO_FADD = 5'd13;
  localparam logic [4:0] AMO_FMIN = 5'd14;
  localparam logic [4:0] AMO_FMAX = 5'd15;

  // waitcnt counter ids (ng_sp_sched_pkg::CNT_*)
  localparam logic [2:0] CNT_VM = 3'd0, CNT_VST = 3'd1, CNT_TEX = 3'd2, CNT_LS = 3'd3, CNT_SM = 3'd4;

  // trap causes (model csr.h TrapCause)
  localparam logic [3:0] CAUSE_ILLEGAL = 4'd1;
  localparam logic [3:0] CAUSE_MEMFAULT = 4'd5;
  localparam logic [3:0] CAUSE_REGRANGE = 4'd10;
  localparam logic [3:0] CAUSE_SHARED   = 4'd11;

  typedef struct packed {
    logic        hit;       // recognised LSU encoding (else IllegalInsn)
    logic        um;        // UM format (uld*), else M / SYS
    logic [2:0]  kind;      // K_*
    logic [2:0]  space;     // SP_* (K_ULD: SP_GLOBAL placeholder)
    logic [1:0]  ukind;     // UK_*
    logic [6:0]  nbytes;    // bytes per lane access (atomics: 4/8; uld: 4..64)
    logic        sext;      // sign-extend u8/u16 loads
    logic [4:0]  amo;       // AMO op (AMO_* above)
    logic        w64;       // 64-bit atomic
    logic        rtn;       // returning atomic
    logic [2:0]  cnt;       // waitcnt counter id
    logic [1:0]  cache_op;  // CO_*
    logic [2:0]  ndata;     // data registers read at issue (st / atom)
    logic [2:0]  ndst;      // vector registers written (ld / atom.rtn)
    logic        has_cpol;
  } dec_t;

  // ---------------- per-warp context (provided by the launch tables) ------
  // The model's HwInfo / DispatchInfo values the LSU needs.
  typedef struct packed {
    logic [3:0]  vmid;
    logic [15:0] sh_base;     // byte offset of the workgroup window in the shared array
    logic [31:0] sh_size;     // window bytes (0 = no shared memory)
    logic [47:0] scr_base;    // ScratchBase (SET_SHADER SCRATCH_VA, 0 = none)
    logic [31:0] scr_bytes;   // ScratchBytes per lane
    logic [31:0] gslot;       // GlobalWarpSlot
  } ctx_t;

  // ---------------- LSU -> L1D -> L2 line-granular memory request ----------
  localparam logic [2:0] M_RD     = 3'd0;  // read one 128 B line
  localparam logic [2:0] M_WR     = 3'd1;  // write bytes of one line (be, wdata)
  localparam logic [2:0] M_AMO    = 3'd2;  // single atomic at pa (<=8 B, inside a line)
  localparam logic [2:0] M_INV_L1 = 3'd3;
  localparam logic [2:0] M_INV_L2 = 3'd4;
  localparam logic [2:0] M_WB_L2  = 3'd5;

  localparam int LINE_B    = 128;           // cache line bytes
  localparam int LINE_BITS = 8 * LINE_B;    // ... and bits (1024)
  localparam logic [7:0] LINE_B8 = 8'd128;  // LINE_B as an unsigned 8-bit constant

  typedef struct packed {
    logic [2:0]    kind;
    logic [47:0]   pa;        // RD/WR: line base; AMO: exact byte address
    logic [1:0]    pol;       // cpol: 0 def, 1 nt, 2 l2, 3 sys
    logic          uc;        // not VRAM: never touches L1/L2 arrays (bypass port)
    logic          noalloc;   // do not allocate on a read miss (MTYPE / nt)
    logic [LINE_B-1:0] be;    // WR: byte enables
    logic [LINE_BITS-1:0] wdata;     // WR: line image (only be bytes meaningful)
    logic [4:0]    amo_op;    // AMO: AMO_* (0..12 only; FP is a CAS loop in the LSU)
    logic          amo64;
    logic          amo_rtn;
    logic [63:0]   amo_a;     // operand / swap value
    logic [63:0]   amo_cmp;   // CAS compare value
  } mreq_t;

  typedef struct packed {
    logic [LINE_BITS-1:0] rdata;     // RD: line image
    logic [63:0]   old;       // AMO: old value
    logic [1:0]    err;       // 0 ok (ng_noc_pkg::ERR_* from L2)
  } mrsp_t;

  // ---------------- helpers ------------------------------------------------
  // PA region of the physical map
  function automatic logic pa_is_vram(input logic [47:0] pa);
    return pa < 48'h0004_0000_0000;                 // 16 GiB
  endfunction
  function automatic logic pa_is_sys(input logic [47:0] pa);
    return (pa >= 48'h0010_0000_0000) && (pa < 48'h0010_8000_0000);
  endfunction
  function automatic logic pa_is_host(input logic [47:0] pa);
    return pa[47];                                   // 0x8000_0000_0000 and up
  endfunction
  function automatic logic pa_valid(input logic [47:0] pa);
    return pa_is_vram(pa) || pa_is_sys(pa) || pa_is_host(pa);
  endfunction

  // bytes of an access of n bytes at line offset a that fall into the first line (the rest wraps to the next line)
  function automatic logic [7:0] n0_of(input logic [6:0] a, input logic [6:0] n);
    logic [7:0] room;
    room = LINE_B8 - {1'b0, a};
    return ({1'b0, n} < room) ? {1'b0, n} : room;
  endfunction

endpackage : ng_lsu_pkg
