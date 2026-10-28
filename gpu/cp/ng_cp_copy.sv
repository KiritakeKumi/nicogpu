// ng_cp_copy.sv -- Copy engine: COPY (linear, byte granular) and FILL
// (32-bit pattern)
//
// One command at a time, programmed through the CP private port and run to
// completion (the firmware polls `busy`; wait_idle is therefore trivial and
// ENGINES_IDLE.bit1 = !busy).  Work is split into destination-aligned chunks
// of up to 8 bytes: each chunk needs one or two aligned 8-byte source reads
// (funnel shift) and one strobed 8-byte write, all through the shared memory
// request port, so accesses never cross a page or a 4 KiB interleave granule
// (the translator/adapter behind the port sees one aligned lane at a time).
//
// Faults: the first faulting read or write stops the engine and sets `fault`
// (sticky until fault_clr); nothing is written after it.  Destination bytes
// written before the fault are unspecified.  LINEAR overlap is
// undefined by spec; here the result follows chunk order.
// MODE 1/2 (t64k) are rejected by the firmware, never reach the engine.
//
// BULK = 1 (the RMs): the command is not split here.  It is handed as a whole to the clk_l2 DMA
// (ng_rcp_dma through ng_rcp_memif_ts / _l2) on the b_* port: b_valid stays high with the quasi-static command
// until the one-cycle b_done (b_fault with it); b_abort is the abort level.  The memory request port is unused.
//
// Abort (COPY_CTRL.ABORT from the firmware on a pending CP_CTRL.RESET, or
// CORE_RESET): no new request is issued from the next cycle on; a request
// already accepted by the port is drained (its response is consumed, a fault
// is not reported) and the engine then goes idle.  So `busy` only falls when
// nothing of this command is left in flight, and a response can never reach
// the boot loader or the next owner of the port.
module ng_cp_copy
  import ng_cp_pkg::*;
#(
  parameter bit BULK = 1'b0
) (
  input  logic        clk,
  input  logic        rst_n,

  // command (start is a 1-cycle pulse; ignored while busy)
  input  logic        start,
  input  logic        op_fill,
  input  logic [3:0]  vmid,
  input  logic [47:0] src,
  input  logic [47:0] dst,
  input  logic [63:0] size,
  input  logic [31:0] fill_value,
  output logic        busy,
  output logic        fault,
  input  logic        fault_clr,
  input  logic        abort,         // pulse or level, see above

  // memory port (valid/ready request, one outstanding, valid response)
  output logic        req_valid,
  input  logic        req_ready,
  output mem_req_t    req,
  input  logic        rsp_valid,
  input  logic        rsp_fault,
  input  logic [63:0] rsp_data,

  // BULK = 1: whole-command port towards the clk_l2 DMA
  output logic        b_valid,
  output logic        b_fill,
  output logic [3:0]  b_vmid,
  output logic [47:0] b_src,
  output logic [47:0] b_dst,
  output logic [63:0] b_size,
  output logic [31:0] b_fillv,
  output logic        b_abort,
  input  logic        b_done,
  input  logic        b_fault
);
  typedef enum logic [2:0] {S_IDLE, S_RD0, S_RD0W, S_RD1, S_RD1W, S_WR, S_WRW, S_BULK} st_t;
  st_t st;
  logic abort_q;

  logic [3:0]  vmid_q;
  logic [47:0] src_q, dst_q;
  logic [63:0] rem_q;
  logic [31:0] fill_q;
  logic        fill_op_q;
  logic [63:0] w0, w1;

  // chunk geometry (combinational from the current pointers)
  logic [2:0]  d_off, s_off;
  logic [3:0]  n;               // bytes in this chunk (1..8)
  logic        need_w1;
  logic [3:0]  s_end;
  always_comb begin
    d_off = dst_q[2:0];
    s_off = src_q[2:0];
    n = 4'd8 - {1'b0, d_off};
    if (rem_q < {60'h0, n}) n = rem_q[3:0];
    s_end = {1'b0, s_off} + n;
    need_w1 = (s_end > 4'd8);
  end

  logic [127:0] funnel;
  logic [63:0]  chunk_data;
  logic [7:0]   chunk_strb;
  always_comb begin
    funnel     = {w1, w0} >> (8 * s_off);
    chunk_data = fill_op_q ? {fill_q, fill_q} : (funnel[63:0] << (8 * d_off));
    chunk_strb = (8'((9'd1 << n) - 9'd1)) << d_off;
  end

  assign busy = (st != S_IDLE);

  assign b_valid = (st == S_BULK);
  assign b_fill  = fill_op_q;
  assign b_vmid  = vmid_q;
  assign b_src   = src_q;
  assign b_dst   = dst_q;
  assign b_size  = rem_q;
  assign b_fillv = fill_q;
  assign b_abort = abort_q;

  always_comb begin
    req_valid = 1'b0;
    req       = '0;
    req.vmid  = vmid_q;
    unique case (st)
      S_RD0: begin req_valid = !abort_q; req.addr = {src_q[47:3], 3'b000}; req.wstrb = 8'hff; end
      S_RD1: begin req_valid = !abort_q; req.addr = {src_q[47:3], 3'b000} + 48'd8; req.wstrb = 8'hff; end
      S_WR:  begin
        req_valid = !abort_q; req.write = 1'b1; req.addr = {dst_q[47:3], 3'b000};
        req.wstrb = chunk_strb; req.wdata = chunk_data;
      end
      default: ;
    endcase
  end

  // after a chunk's write: advance
  logic [47:0] src_n, dst_n;
  logic [63:0] rem_n;
  assign src_n = src_q + {44'h0, n};
  assign dst_n = dst_q + {44'h0, n};
  assign rem_n = rem_q - {60'h0, n};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; fault <= 1'b0; abort_q <= 1'b0;
      vmid_q <= '0; src_q <= '0; dst_q <= '0; rem_q <= '0; fill_q <= '0; fill_op_q <= 1'b0;
      w0 <= '0; w1 <= '0;
    end else begin
      if (fault_clr) fault <= 1'b0;
      if (st == S_IDLE)  abort_q <= 1'b0;
      else if (abort)    abort_q <= 1'b1;
      unique case (st)
        S_IDLE: if (start) begin
          vmid_q <= vmid; src_q <= src; dst_q <= dst; rem_q <= size;
          fill_q <= fill_value; fill_op_q <= op_fill;
          w0 <= '0; w1 <= '0;
          if (size != 64'h0) st <= BULK ? S_BULK : op_fill ? S_WR : S_RD0;
        end
        S_BULK: if (b_done) begin
          if (b_fault && !abort_q) fault <= 1'b1;
          st <= S_IDLE;
        end
        S_RD0:  if (abort_q) st <= S_IDLE; else if (req_ready) st <= S_RD0W;
        S_RD0W: if (rsp_valid) begin
          if (abort_q) st <= S_IDLE;
          else if (rsp_fault) begin fault <= 1'b1; st <= S_IDLE; end
          else begin
            w0 <= rsp_data;
            st <= need_w1 ? S_RD1 : S_WR;
          end
        end
        S_RD1:  if (abort_q) st <= S_IDLE; else if (req_ready) st <= S_RD1W;
        S_RD1W: if (rsp_valid) begin
          if (abort_q) st <= S_IDLE;
          else if (rsp_fault) begin fault <= 1'b1; st <= S_IDLE; end
          else begin w1 <= rsp_data; st <= S_WR; end
        end
        S_WR:   if (abort_q) st <= S_IDLE; else if (req_ready) st <= S_WRW;
        S_WRW:  if (rsp_valid) begin
          if (abort_q) st <= S_IDLE;
          else if (rsp_fault) begin fault <= 1'b1; st <= S_IDLE; end
          else begin
            src_q <= src_n; dst_q <= dst_n; rem_q <= rem_n; w1 <= '0;
            if (rem_n == 64'h0) st <= S_IDLE;
            else st <= fill_op_q ? S_WR : S_RD0;
          end
        end
        default: st <= S_IDLE;
      endcase
    end
  end
endmodule
