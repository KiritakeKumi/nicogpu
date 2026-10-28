// ng_prim_fifo_async.sv -- dual-clock FIFO with Gray-coded pointers.
// Behavioral model (FPGA: may map to XPM/LUTRAM in the static flow; ASIC: a
// register-file FIFO).  DEPTH must be a power of two.
//
// SYNC_RD = 0: combinational (show-ahead) read from distributed RAM.
// SYNC_RD = 1: same first-word-fall-through interface, but the memory is
//   read synchronously into an output register and mapped to block RAM.
//   Used for wide channels: a combinational LUTRAM read of several hundred
//   bits fans the read pointer out to hundreds of LUTs and does not meet
//   300 MHz (C1 512-bit path, P0-11 timing).  The output register adds one
//   entry of capacity; empty/dout follow the output register.
module ng_prim_fifo_async #(
  parameter int W       = 32,
  parameter int DEPTH   = 16,
  parameter bit SYNC_RD = 1'b0
) (
  input  logic         wclk,
  input  logic         wrst_n,
  input  logic         push,
  input  logic [W-1:0] din,
  output logic         full,
  output logic [$clog2(DEPTH):0] wcount,   // occupancy seen from the write side
  input  logic         rclk,
  input  logic         rrst_n,
  input  logic         pop,
  output logic [W-1:0] dout,
  output logic         empty
);
  localparam int AW = $clog2(DEPTH);
  logic [AW:0]  wbin, wgray, rbin, rgray, wgray_r, rgray_w, rbin_w;

  function automatic logic [AW:0] b2g(input logic [AW:0] b);
    return b ^ (b >> 1);
  endfunction
  function automatic logic [AW:0] g2b(input logic [AW:0] g);
    logic [AW:0] b;
    b[AW] = g[AW];
    for (int i = AW - 1; i >= 0; i--) b[i] = b[i+1] ^ g[i];
    return b;
  endfunction

  // write side
  always_ff @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
      wbin  <= '0;
      wgray <= '0;
    end else if (push && !full) begin
      wbin  <= wbin + 1'b1;
      wgray <= b2g(wbin + 1'b1);
    end
  end
  ng_prim_sync #(.W(AW + 1)) u_r2w (.clk(wclk), .rst_n(wrst_n), .d(rgray), .q(rgray_w));
  assign rbin_w = g2b(rgray_w);
  assign full   = (wgray == {~rgray_w[AW:AW-1], rgray_w[AW-2:0]});
  assign wcount = wbin - rbin_w;

  // read side (rbin/rgray count entries taken out of the memory)
  ng_prim_sync #(.W(AW + 1)) u_w2r (.clk(rclk), .rst_n(rrst_n), .d(wgray), .q(wgray_r));
  logic mem_empty;
  assign mem_empty = (rgray == wgray_r);

  if (!SYNC_RD) begin : g_comb
    logic [W-1:0] mem [DEPTH];
    always_ff @(posedge wclk) if (push && !full) mem[wbin[AW-1:0]] <= din;
    always_ff @(posedge rclk or negedge rrst_n) begin
      if (!rrst_n) begin
        rbin  <= '0;
        rgray <= '0;
      end else if (pop && !mem_empty) begin
        rbin  <= rbin + 1'b1;
        rgray <= b2g(rbin + 1'b1);
      end
    end
    assign empty = mem_empty;
    assign dout  = mem[rbin[AW-1:0]];
  end else begin : g_sync
    (* ram_style = "block" *) logic [W-1:0] mem [DEPTH];
    logic         out_v, fetch;
    logic [W-1:0] q;
    always_ff @(posedge wclk) if (push && !full) mem[wbin[AW-1:0]] <= din;
    // refill the output register whenever it is empty or being popped
    assign fetch = !mem_empty && (!out_v || pop);
    always_ff @(posedge rclk or negedge rrst_n) begin
      if (!rrst_n) begin
        rbin  <= '0;
        rgray <= '0;
        out_v <= 1'b0;
      end else begin
        if (fetch) begin
          rbin  <= rbin + 1'b1;
          rgray <= b2g(rbin + 1'b1);
        end
        if (fetch)    out_v <= 1'b1;
        else if (pop) out_v <= 1'b0;
      end
    end
    always_ff @(posedge rclk) if (fetch) q <= mem[rbin[AW-1:0]];
    assign empty = !out_v;
    assign dout  = q;
  end
endmodule
