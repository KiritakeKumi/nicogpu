// tb_l2_axi_mem_model.sv -- testbench infrastructure: simple AXI4 slave
// memory standing in for a MIG.  Keyed by channel byte offset (the address a
// slice drives per DFX); unread bytes read back a deterministic hash
// the address so testbenches can check the interleave end to end:
//   tb_hash48(a) == bits [55:48] of (a * 0x9E37_79B9_7F4A_7C15).
// Random ready deassertion and response latency provide backpressure.
// Not synthesizable; TB-only (hence the tb_l2_ file name prefix).
module tb_l2_axi_mem_model #(
  parameter int AW = 33,
  parameter int DW = 512,
  parameter int IW = 6,
  parameter int MAX_DELAY = 3,  // max idle cycles between beats/handshakes
  parameter bit HASH_INIT = 1'b1  // 1: unwritten bytes read the address hash; 0: they read 0 (tb_core)
) (
  input  logic                 clk,
  input  logic                 rst_n,
  // AXI4 slave port (mirrors the master port list of ng_l2_slice)
  input  logic                 arvalid,
  output logic                 arready,
  input  logic [AW-1:0]        araddr,
  input  logic [7:0]           arlen,
  input  logic [2:0]           arsize,
  input  logic [1:0]           arburst,
  input  logic [IW-1:0]        arid,
  output logic                 rvalid,
  input  logic                 rready,
  output logic [DW-1:0]        rdata,
  output logic                 rlast,
  output logic [IW-1:0]        rid,
  output logic [1:0]           rresp,
  input  logic                 awvalid,
  output logic                 awready,
  input  logic [AW-1:0]        awaddr,
  input  logic [7:0]           awlen,
  input  logic [2:0]           awsize,
  input  logic [1:0]           awburst,
  input  logic [IW-1:0]        awid,
  input  logic                 wvalid,
  output logic                 wready,
  input  logic [DW-1:0]        wdata,
  input  logic [DW/8-1:0]      wstrb,
  input  logic                 wlast,
  output logic                 bvalid,
  input  logic                 bready,
  output logic [IW-1:0]        bid,
  output logic [1:0]           bresp
);

  localparam int BYTES = DW / 8;

  logic [7:0] mem [logic [AW-1:0]];

  function automatic logic [7:0] hash48(input logic [47:0] a);
    logic [63:0] p;
    p = a * 64'h9E3779B97F4A7C15;
    return p[55:48];
  endfunction

  function automatic logic [7:0] byte_at(input logic [AW-1:0] a);
    return mem.exists(a) ? mem[a] : (HASH_INIT ? hash48(48'(a)) : 8'h00);
  endfunction

  // ---------------- read channel --------------------------------------------
  int rd_delay, rd_beat, rd_len;
  logic [AW-1:0] rd_addr;
  logic [IW-1:0] rd_id;
  typedef enum {RD_IDLE, RD_WAIT, RD_BEAT} rd_e;
  rd_e rd_st;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rd_st    <= RD_IDLE;
      arready  <= 1'b1;
      rvalid   <= 1'b0;
      rresp    <= 2'b00;
      rd_delay <= 0;
    end else begin
      unique case (rd_st)
        RD_IDLE: begin
          if (arvalid && arready) begin
            arready  <= 1'b0;
            rd_addr  <= araddr;
            rd_id    <= arid;
            rd_len   <= int'(arlen) + 1;
            rd_beat  <= 0;
            rd_delay <= $urandom_range(MAX_DELAY);
            rd_st    <= RD_WAIT;
          end
        end
        RD_WAIT: begin
          if (rd_delay != 0) begin
            rd_delay <= rd_delay - 1;
          end else begin
            rvalid   <= 1'b1;
            rdata    <= beat_bytes(rd_addr + AW'(rd_beat * BYTES));
            rid      <= rd_id;
            rlast    <= (rd_beat == rd_len - 1);
            rd_st    <= RD_BEAT;
          end
        end
        RD_BEAT: begin
          if (rvalid && rready) begin
            rvalid <= 1'b0;
            if (rd_beat == rd_len - 1) begin
              arready <= 1'b1;
              rd_st   <= RD_IDLE;
            end else begin
              rd_beat  <= rd_beat + 1;
              rd_delay <= $urandom_range(MAX_DELAY);
              rd_st    <= RD_WAIT;
            end
          end
        end
        default: rd_st <= RD_IDLE;
      endcase
    end
  end

  function automatic logic [DW-1:0] beat_bytes(input logic [AW-1:0] base);
    logic [DW-1:0] d;
    for (int b = 0; b < BYTES; b++) d[8*b +: 8] = byte_at(base + AW'(b));
    return d;
  endfunction

  // ---------------- write channel -------------------------------------------
  typedef enum {WR_IDLE, WR_DATA, WR_RESP} wr_e;
  wr_e wr_st;
  int wr_beat, wr_len, wr_delay;
  logic [AW-1:0] wr_addr;
  logic [IW-1:0] wr_id;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_st    <= WR_IDLE;
      awready  <= 1'b1;
      wready   <= 1'b0;
      bvalid   <= 1'b0;
      bresp    <= 2'b00;
      wr_delay <= 0;
    end else begin
      unique case (wr_st)
        WR_IDLE: begin
          if (awvalid && awready) begin
            awready  <= 1'b0;
            wready   <= 1'b1;
            wr_addr  <= awaddr;
            wr_id    <= awid;
            wr_len   <= int'(awlen) + 1;
            wr_beat  <= 0;
            wr_st    <= WR_DATA;
          end
        end
        WR_DATA: begin
          if (wvalid && wready) begin
            for (int b = 0; b < BYTES; b++) begin
              if (wstrb[b]) mem[wr_addr + AW'(wr_beat * BYTES + b)] = wdata[8*b +: 8];
            end
            if (wlast) begin
              wready   <= 1'b0;
              wr_delay <= $urandom_range(0, MAX_DELAY);
              wr_st    <= WR_RESP;
            end else begin
              wr_beat <= wr_beat + 1;
            end
          end
        end
        WR_RESP: begin
          if (wr_delay != 0) begin
            wr_delay <= wr_delay - 1;
          end else begin
            bvalid <= 1'b1;
            bid    <= wr_id;
            bresp  <= 2'b00;
          end
          if (bvalid && bready) begin
            bvalid  <= 1'b0;
            awready <= 1'b1;
            wr_st   <= WR_IDLE;
          end
        end
        default: wr_st <= WR_IDLE;
      endcase
    end
  end

  // hierarchically visible to TBs for checking written-back data
  function automatic logic [7:0] peek(input logic [AW-1:0] a);
    return byte_at(a);
  endfunction
  // backdoor preload / clear (tb_core loads the bundle's VRAM pages before the dispatch)
  function automatic void poke(input logic [AW-1:0] a, input logic [7:0] v);
    mem[a] = v;
  endfunction
  function automatic void clear_all();
    mem.delete();
  endfunction
endmodule
