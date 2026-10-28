// ng_l1d.sv -- per-half-core L1 data cache, write-through, non-coherent.
//
// Design-doc 4.x / ISA: "L1D write-through, no coherence guarantee; acquire /
// release are software-inserted `cache.inv.l1` and `cache.wb.l2`".  Physical
// tags (the LSU translates before the access), 128 B lines, SETS x WAYS
// (default 256 x 4 = 128 KiB; the other 64 KiB of the half-core's 256 KiB URAM
// are the shared-memory array, design doc 4.x).
//
// Policy (cpol, ISA `cpol` enum):
//   def (0)  read: lookup, allocate on miss.   write: update on hit, no allocate.
//   nt  (1)  read: lookup but never allocate (streaming).  write: as def.
//   l2  (2)  read: bypass the arrays (always fetched from L2); write: as def.
//   sys (3)  as l2 (system scope is the fabric's business beyond L2).
//   mreq.noalloc (MTYPE) forces no-allocate; mreq.uc (non-VRAM PA) bypasses the
//   arrays for every kind.
// Writes are always forwarded (write-through) and complete on the L2 ack.
// Atomics execute at L2: the L1 line (if present) is invalidated.
// cache.inv.l1 clears every valid bit (1 cycle); cache.inv.l2 / cache.wb.l2
// are forwarded to the L2 side unchanged (this cache holds no dirty data, so
// "wb" has nothing to do locally).
//
// one request in flight; behavioural arrays
// with a combinational lookup in the LKP state (URAM mapping needs a registered
// read: +1 cycle, decided at OOC); round-robin victim per set; line fill =
// whole 128 B returned in one response beat (the NoC adapter splits).
// ng_l1d_ram -- single-port, read-first, byte-write synchronous RAM slice (synthesis addendum).
// One instance per (way, 64-bit slice) of the L1D data array and per way for the tags, so that no
// single variable exceeds the Vivado limit (1e6 bits) and every array maps to block RAM / LUTRAM.
module ng_l1d_ram #(
  parameter int BE    = 8,                 // bytes per word
  parameter int DEPTH = 256,
  parameter string STYLE = "block"
) (
  input  logic                  clk,
  input  logic                  en,        // access enable (read, and write when |we)
  input  logic [$clog2(DEPTH)-1:0] addr,
  input  logic [BE-1:0]         we,
  input  logic [8*BE-1:0]       wdata,
  output logic [8*BE-1:0]       rdata
);
  if (STYLE == "block") begin : g_blk
    (* ram_style = "block" *) logic [8*BE-1:0] m [DEPTH];
    always @(posedge clk) begin
      if (en) begin
        rdata <= m[addr];
        for (int i = 0; i < BE; i++)
          if (we[i]) m[addr][8*i +: 8] <= wdata[8*i +: 8];
      end
    end
  end else begin : g_dist
    (* ram_style = "distributed" *) logic [8*BE-1:0] m [DEPTH];
    always @(posedge clk) begin
      if (en) begin
        rdata <= m[addr];
        for (int i = 0; i < BE; i++)
          if (we[i]) m[addr][8*i +: 8] <= wdata[8*i +: 8];
      end
    end
  end
endmodule

module ng_l1d
  import ng_lsu_pkg::*;
#(
  parameter int SETS = 256,
  parameter int WAYS = 4
) (
  input  logic     clk,
  input  logic     rst_n,

  // ---- external whole-cache invalidate (ng_wd scrub, CP CACHE(INV_L1) packet).  A pulse is remembered and done
  // in the first idle cycle (stat_inv counts it like a cache.inv.l1 request); while it is pending req_ready is low, so a
  // request of the LSU is never caught between the invalidate and its lookup.  Tie 0: the previous L1D, cycle for cycle ----
  input  logic     ext_inv,

  // ---- LSU side (level valid/ready, one outstanding) ----
  input  logic     req_valid,
  output logic     req_ready,
  input  mreq_t    req,
  output logic     rsp_valid,     // one-cycle pulse
  output mrsp_t    rsp,

  // ---- L2 / fabric side ----
  output logic     mem_req_valid,
  input  logic     mem_req_ready,
  output mreq_t    mem_req,
  input  logic     mem_rsp_valid,
  input  mrsp_t    mem_rsp,

  // ---- statistics ----
  output logic [31:0] stat_hit,
  output logic [31:0] stat_miss,
  output logic [31:0] stat_fill,
  output logic [31:0] stat_inv
);
  localparam int SET_LB = $clog2(SETS);
  localparam int WAY_LB = ng_noc_pkg::clog2_1(WAYS);
  localparam int TAG_W  = 48 - 7 - SET_LB;

  logic               val_a  [WAYS][SETS];
  logic [WAY_LB-1:0]  rr_a   [SETS];

  typedef enum logic [2:0] {S_IDLE, S_LKP, S_FWD, S_WAIT, S_RSP} st_t;
  st_t st;

  mreq_t  rq;
  mrsp_t  rs;
  logic   rq_bypass;     // fetch/forward without touching the arrays
  logic   rq_noalloc;

  wire [SET_LB-1:0] rset = rq.pa[7 +: SET_LB];
  wire [TAG_W-1:0]  rtag = rq.pa[47:7+SET_LB];

  // ---- tag / data storage: block RAMs read in S_IDLE (address = the incoming request's set), the
  // registered read data is valid in S_LKP -- same cycle the old combinational lookup was used.
  wire [SET_LB-1:0] qset  = req.pa[7 +: SET_LB];
  wire              ram_rd = (st == S_IDLE);
  wire [SET_LB-1:0] ram_a = ram_rd ? qset : rset;

  logic [WAYS-1:0]    lkp_wr;       // S_LKP write hit (per way)
  logic [WAYS-1:0]    fill_we;      // S_WAIT line fill (per way)
  logic               fill_ev;
  logic [TAG_W-1:0]   tq    [WAYS];
  logic [LINE_BITS-1:0]      dq    [WAYS];
  logic [LINE_BITS-1:0]      d_wdata;
  assign d_wdata = (st == S_WAIT) ? mem_rsp.rdata : rq.wdata;

  for (genvar w = 0; w < WAYS; w++) begin : g_way
    localparam int TBE = (TAG_W + 7) / 8;
    logic [8*TBE-1:0] tq_w;
    ng_l1d_ram #(.BE(TBE), .DEPTH(SETS), .STYLE("distributed")) u_tag (
      .clk(clk), .en(ram_rd | fill_we[w]), .addr(ram_a),
      .we({TBE{fill_we[w]}}), .wdata((8 * TBE)'(rtag)), .rdata(tq_w));
    assign tq[w] = tq_w[TAG_W-1:0];
    for (genvar k = 0; k < 16; k++) begin : g_sl
      logic [7:0] sl_we;
      assign sl_we = lkp_wr[w] ? rq.be[8*k +: 8] : (fill_we[w] ? 8'hFF : 8'h00);
      ng_l1d_ram #(.BE(8), .DEPTH(SETS), .STYLE("block")) u_d (
        .clk(clk), .en(ram_rd | (|sl_we)), .addr(ram_a), .we(sl_we),
        .wdata(d_wdata[64*k +: 64]), .rdata(dq[w][64*k +: 64]));
    end
  end

  // lookup (combinational on the captured request and the registered tag read)
  logic              hit;
  logic [WAY_LB-1:0] hway;
  always_comb begin
    hit  = 1'b0;
    hway = '0;
    for (int w = 0; w < WAYS; w++)
      if (val_a[w][rset] && tq[w] == rtag) begin
        hit  = 1'b1;
        hway = WAY_LB'(w);
      end
  end

  // victim for a fill: first invalid way, else the round-robin pointer
  logic [WAY_LB-1:0] vway;
  always_comb begin
    vway = rr_a[rset];
    for (int w = WAYS - 1; w >= 0; w--)
      if (!val_a[w][rset]) vway = WAY_LB'(w);
  end

  logic ext_inv_p;     // external invalidate pending
  assign req_ready = (st == S_IDLE) && !ext_inv_p;
  assign rsp_valid = (st == S_RSP);
  assign rsp       = rs;
  assign mem_req_valid = (st == S_FWD);
  assign mem_req       = rq;

  // write / fill events (drive the RAM byte enables and the sim-only mirrors)
  always_comb begin
    lkp_wr  = '0;
    fill_we = '0;
    fill_ev = (st == S_WAIT) && mem_rsp_valid && rq.kind == M_RD && !rq_bypass && !rq_noalloc
              && mem_rsp.err == 2'd0;
    if (st == S_LKP && rq.kind == M_WR && !rq.uc && hit) lkp_wr[hway] = 1'b1;
    if (fill_ev) fill_we[vway] = 1'b1;
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE;
      for (int w = 0; w < WAYS; w++) for (int s = 0; s < SETS; s++) val_a[w][s] <= 1'b0;
      for (int s = 0; s < SETS; s++) rr_a[s] <= '0;
      rq <= '0;
      rs <= '0;
      rq_bypass <= 1'b0;
      rq_noalloc <= 1'b0;
      ext_inv_p <= 1'b0;
      stat_hit <= '0; stat_miss <= '0; stat_fill <= '0; stat_inv <= '0;
    end else begin
      if (ext_inv) ext_inv_p <= 1'b1;
      case (st)
        S_IDLE: begin
          if (ext_inv_p) begin
            for (int w = 0; w < WAYS; w++) for (int s = 0; s < SETS; s++) val_a[w][s] <= 1'b0;
            stat_inv <= stat_inv + 1;
            ext_inv_p <= ext_inv;     // (a pulse in this very cycle stays pending)
          end else if (req_valid) begin
            rq <= req;
            rs <= '0;
            rq_bypass  <= req.uc || (req.pol >= 2'd2);
            rq_noalloc <= req.noalloc || (req.pol == 2'd1);
            if (req.kind == M_INV_L1) begin
              for (int w = 0; w < WAYS; w++) for (int s = 0; s < SETS; s++) val_a[w][s] <= 1'b0;
              stat_inv <= stat_inv + 1;
              st <= S_RSP;
            end else if (req.kind == M_INV_L2 || req.kind == M_WB_L2) begin
              st <= S_FWD;
            end else begin
              st <= S_LKP;
            end
          end
        end

        S_LKP: begin
          st <= S_FWD;
          case (rq.kind)
            M_RD: begin
              if (!rq.uc && rq.pol < 2'd2) begin
                if (hit) begin
                  rs.rdata <= dq[hway];
                  stat_hit <= stat_hit + 1;
                  st <= S_RSP;
                end else begin
                  stat_miss <= stat_miss + 1;
                end
              end
            end
            M_WR: begin
              // (data write: byte-enabled RAM write, lkp_wr)
            end
            M_AMO: begin
              if (!rq.uc && hit) val_a[hway][rset] <= 1'b0;
            end
            default: ;
          endcase
        end

        S_FWD: begin
          if (mem_req_ready) st <= S_WAIT;
        end

        S_WAIT: begin
          if (mem_rsp_valid) begin
            rs <= mem_rsp;
            if (rq.kind == M_RD && !rq_bypass && !rq_noalloc && mem_rsp.err == 2'd0) begin
              val_a[vway][rset]  <= 1'b1;
              rr_a[rset]         <= rr_a[rset] + WAY_LB'(1);
              stat_fill <= stat_fill + 1;
            end
            st <= S_RSP;
          end
        end

        S_RSP: st <= S_IDLE;

        default: st <= S_IDLE;
      endcase
    end
  end

`ifndef SYNTHESIS
  // Simulation-only mirrors of the tag / data arrays (testbench peek: dut.tag_a / dut.data_a).
  logic [TAG_W-1:0] tag_a  [WAYS][SETS];
  logic [LINE_BITS-1:0]    data_a [WAYS][SETS];
  always @(posedge clk) begin
    if (|lkp_wr)
      for (int b = 0; b < 128; b++) if (rq.be[b]) data_a[hway][rset][8*b +: 8] <= rq.wdata[8*b +: 8];
    if (fill_ev) begin
      tag_a[vway][rset]  <= rtag;
      data_a[vway][rset] <= mem_rsp.rdata;
    end
  end
`endif
endmodule
