// ng_noc_pkg.sv -- NoC/L2 shared package: flit format, message classes,
// AMO op encoding.  Single source of truth for the network contract between
// requestors (cores, CP, copy engine -- future) and L2 slices.
//
// Message model (v1, tentative --):
//   * every message is a single flit (no wormhole head/body/tail split);
//     multi-flit extension is deferred until a client needs >64 B payloads.
//   * cls selects the virtual channel: CLS_REQ uses VC0, CLS_RESP uses VC1.
//     The two classes are separate virtual networks with independent credit
//     pools (protocol-level deadlock freedom
//   * dst is a NoC node id (mesh coordinates owned by ng_noc_mesh); the
//     PA -> slice-id -> node-id mapping lives in ng_l2_addrmap + the
//     integration layer, not in the router.
package ng_noc_pkg;

  // ---------------- message classes (= virtual channel index) -------------
  localparam logic [1:0] CLS_REQ  = 2'd0;  // requestor -> L2 slice
  localparam logic [1:0] CLS_RESP = 2'd1;  // L2 slice -> requestor
  localparam int         NUM_VC   = 2;

  // ---------------- request kinds (cls=CLS_REQ, opcode field reuse) -------
  // The request opcode is implied by op: op==AMO_NONE means plain RD/WR
  // selected by the wr bit below.  We reuse the AMO op field plus one extra
  // bit in `size`'s neighbourhood -- to keep the struct simple the request
  // kind is encoded in bits [17:16] (kind) instead: 0=RD, 1=WR, 2=AMO, 3=MAINT.
  localparam logic [1:0] KIND_RD  = 2'd0;
  localparam logic [1:0] KIND_WR  = 2'd1;
  localparam logic [1:0] KIND_AMO = 2'd2;
  // cache maintenance request (the formerly reserved kind 3).  The
  // operation is selected by flit.op (MAINT_*), addr/size/data are ignored;
  // the slice answers with a data-less response once its whole sweep is done.
  localparam logic [1:0] KIND_MAINT = 2'd3;
  localparam logic [4:0] MAINT_INV_L2 = 5'd0;   // write back dirty + invalidate
  localparam logic [4:0] MAINT_WB_L2  = 5'd1;   // write back dirty, keep lines

  // ---------------- access size (bytes = 1 << size) ------------------------
  localparam logic [1:0] SZ_2 = 2'd1;
  localparam logic [1:0] SZ_4 = 2'd2;
  localparam logic [1:0] SZ_8 = 2'd3;

  // ---------------- AMO ops (ISA atom.* f5 encodings re-derived, 5 bits) ---
  // Covers atom.global/ubase/buf families:
  //   add/sub/and/or/xor/min[.i/.u]/max[.i/.u]/xchg/cmpxchg/inc/dec,
  //   32-bit and 64-bit.  FP32 atomics (fadd/fmin/fmax, optional in ISA)
  //   were added by, codes 13..15 below.
  localparam logic [4:0] AMO_ADD   = 5'd0;
  localparam logic [4:0] AMO_SUB   = 5'd1;
  localparam logic [4:0] AMO_AND   = 5'd2;
  localparam logic [4:0] AMO_OR    = 5'd3;
  localparam logic [4:0] AMO_XOR   = 5'd4;
  localparam logic [4:0] AMO_MIN_S = 5'd5;
  localparam logic [4:0] AMO_MAX_S = 5'd6;
  localparam logic [4:0] AMO_MIN_U = 5'd7;
  localparam logic [4:0] AMO_MAX_U = 5'd8;
  localparam logic [4:0] AMO_XCHG  = 5'd9;
  localparam logic [4:0] AMO_CAS   = 5'd10;
  localparam logic [4:0] AMO_INC   = 5'd11;  // 32-bit only (CUDA wrap)
  localparam logic [4:0] AMO_DEC   = 5'd12;  // 32-bit only (CUDA wrap)
  // FP32 atomics (ISA f5 0xd/0xe/0xf), 32-bit only, executed in the L2.
  // ng_lsu_pkg carries the same three numbers for the LSU-internal decode; the
  // LSU does not send them to the L2 (it runs a CAS loop instead).
  localparam logic [4:0] AMO_FADD  = 5'd13;
  localparam logic [4:0] AMO_FMIN  = 5'd14;
  localparam logic [4:0] AMO_FMAX  = 5'd15;
  localparam logic [4:0] AMO_NONE  = 5'd31;
  localparam logic [31:0] F32_QNAN = 32'h7FC00000;  // canonical FP32 quiet NaN

  // FP32 atomic helpers (ng_l2_amo_alu, ng_lsu_fatomic)
  function automatic logic f32_isnan(input logic [31:0] x);
    return (x[30:23] == 8'hFF) && (x[22:0] != 23'd0);
  endfunction
  // quiet less-than for non-NaN operands that are not both zero
  function automatic logic f32_lt(input logic [31:0] x, input logic [31:0] y);
    if (x[31] != y[31]) return x[31];
    return x[31] ? (x[30:0] > y[30:0]) : (x[30:0] < y[30:0]);
  endfunction
  // fmin / fmax: minimumNumber / maximumNumber -- a single NaN operand is ignored, both NaN -> F32_QNAN, -0 < +0
  function automatic logic [31:0] f32_minmax(input logic [31:0] x, input logic [31:0] y, input logic is_max);
    logic lt;
    if (f32_isnan(x) && f32_isnan(y)) return F32_QNAN;
    if (f32_isnan(x)) return y;
    if (f32_isnan(y)) return x;
    if (((x | y) & 32'h7FFFFFFF) == 32'd0) return is_max ? (x & y) : (x | y);
    lt = f32_lt(x, y);
    return is_max ? (lt ? y : x) : (lt ? x : y);
  endfunction

  // ---------------- response error codes -----------------------------------
  localparam logic [1:0] ERR_NONE    = 2'd0;
  localparam logic [1:0] ERR_BAD_PA  = 2'd1;  // not VRAM / not this slice's PA space
  localparam logic [1:0] ERR_CROSS   = 2'd2;  // access crosses a cache line
  localparam logic [1:0] ERR_DDR     = 2'd3;  // DDR (AXI) side error response

  // ---------------- flit ----------------------------------------------------
  typedef struct packed {
    logic [1:0]  cls;    // CLS_REQ / CLS_RESP (also the VC selector)
    logic [1:0]  kind;   // request kind (KIND_*); 0 in responses
    logic [4:0]  op;     // AMO op (AMO_NONE for RD/WR); 0 in responses
    logic [1:0]  size;   // access size (SZ_*); responses echo nothing
    logic        rtn;    // AMO returns old value
    logic [47:0] addr;   // PA (request); unused (response)
    logic [5:0]  src;    // requester node id; response: slice node id
    logic [5:0]  dst;    // destination node id (slice for req, req for resp)
    logic [7:0]  tag;    // requester tag, echoed in the response
    logic [63:0] data;   // request: write data / AMO operand+swap; resp: read/old data
    logic [63:0] cmp;    // AMO compare value (cmpxchg); unused otherwise
    logic [7:0]  be;     // write byte enables for `data` (request only)
    logic [1:0]  err;    // response error (ERR_*); 0 in requests
  } flit_t;

  // ---------------- width helpers ------------------------------------------
  // index width of an n-entry array (at least one bit: n = 1 would give a zero-width $clog2)
  function automatic int clog2_1(input int n);
    return (n > 1) ? $clog2(n) : 1;
  endfunction

  // ---------------- FIFO pointer increment ---------------------------------
  // Wrap at the depth (the plain `+ 1` is only right for power-of-two depths); callers truncate to their
  // pointer width.
  function automatic logic [15:0] ptr_inc(input int unsigned p, input int depth);
    return (int'(p) >= depth - 1) ? 16'd0 : 16'(p + 1);
  endfunction

endpackage : ng_noc_pkg
