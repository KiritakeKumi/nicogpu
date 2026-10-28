// ng_rcp_regs.sv -- s_axil_ctrl front end of the CP RM (rm_kind 0x8002) and of the M1 SP RM (rm_kind 0x8003).
// Two parameters adapt it to the SP RM (rm_sp used a copy, ng_rsp_regs, before); the defaults are the CP RM:
//   * CAPS (0x0008, rm_cp reads 0): the SP RM publishes cores=1, SP/core=1, warp 32, 16 VMIDs so the driver's GET_GPU_INFO
//     derives max_workgroup_threads = 1 * 16 * 32 = 512.  CAPS[27:24] carry the SP unit bits (
// [24] SFU, [25] XLANE, [26] FP64, [27] UNITS_VALID (set by the top).
//   * DBG_PAGE = 1: the read-only debug page 0x0800..0x08FF: 0x0800..0x08BF are clk_ts registers read
//     through the CP register bridge (ng_rcp_cpbus; the RM top routes those addresses to ng_rsp_dbg instead of the CP),
//     0x08C0..0x08FF are clk_l2 registers served locally from `l2dbg` (writes to the page are dropped);
// 0x0900..0x093F is a second clk_ts debug block (latency / LSU counters), routed like 0x0800..0x08BF.
// 0x0940..0x095F (SP performance counters) and 0x0960..0x097F (fabric slot counters), routed the
//     same way; 0x0980..0x09FF are not routed (0x0980..0x09BF reserved for) and read 0 here.
//     With DBG_PAGE = 0 (rm_cp) the page is not decoded (it reads 0 like every undefined offset) and l2dbg is unused.
//
//  identity (ID/REV/CAPS/SCRATCH), the C3 window registers (BAR2_INFO, C3_WIN_CTRL,
// C3_WIN_BASE) and a bridge to the CP register block, which lives in the clk_ts domain:
//   0x0040 IRQ_STATUS  0x0044 IRQ_ENABLE  0x0050/54/58 timestamp   0x0100..0x01FF CP page   0x1000..0x1FFF doorbells
// A CP access is one request/acknowledge handshake with toggle synchronisers (one outstanding, the AXI-Lite FSM
// below is serial anyway): clk_l2 side holds {wr, addr, wdata} stable, toggles `req`; clk_ts side performs the
// access for exactly one cycle (host_wr / host_rd of ng_cp_regs), captures the read data and toggles `ack`.
// MMU registers: 0x0200..0x02FF (MMU_CTX_*[n]) and 0x0300..0x033F (MMU_INVALIDATE, MMU_FAULT_*) are served by
// ng_mmu's csr bus, in this clk_l2 domain, in a single cycle like the local registers: the write strobe csr_wr is
// high for the one cycle in which the FSM takes the captured AW/W pair (csr_waddr / csr_wdata are those registers),
// csr_raddr is the captured read address and csr_rdata (combinational in ng_mmu_regs) is registered into s_rdata.
// a write with WSTRB != 4'hF is dropped to the MMU (the driver only issues 32-bit writes).
// Undefined offsets read 0, writes are dropped, responses are always OKAY (as the test RM).
// The AXI-Lite handshake front end follows ng_rmt_regs (proven against the shell): AW/W/AR are captured
// independently, B/R come from flip-flops.
module ng_rcp_regs #(
  parameter logic [15:0] RM_REV   = 16'h0001,
  parameter logic [31:0] CAPS     = 32'h0,
  parameter bit          DBG_PAGE = 1'b0
) (
  input  logic         clk,             // clk_l2
  input  logic         rst_n,
  input  logic [19:0]  s_awaddr,
  input  logic         s_awvalid,
  output logic         s_awready,
  input  logic [31:0]  s_wdata,
  input  logic [3:0]   s_wstrb,
  input  logic         s_wvalid,
  output logic         s_wready,
  output logic         s_bvalid,
  input  logic         s_bready,
  input  logic [19:0]  s_araddr,
  input  logic         s_arvalid,
  output logic         s_arready,
  output logic [31:0]  s_rdata,
  output logic         s_rvalid,
  input  logic         s_rready,
  // C3 window (BAR2 adapter)
  output logic         win_en,
  output logic [4:0]   win_log2,
  output logic [30:0]  win_base,
  // CP register bus bridge (to the clk_ts side, see ng_rcp_cpbus)
  output logic         cp_req,          // toggles per request
  output logic         cp_wr,
  output logic [12:0]  cp_addr,
  output logic [31:0]  cp_wdata,
  input  logic         cp_ack,          // synchronised toggle from the clk_ts side
  input  logic [31:0]  cp_rdata,
  // MMU register bus (ng_mmu csr_*)
  output logic         csr_wr,
  output logic [11:0]  csr_waddr,
  output logic [31:0]  csr_wdata,
  output logic [11:0]  csr_raddr,
  input  logic [31:0]  csr_rdata,
  // clk_l2 debug registers 0x08C0..0x08FC (DBG_PAGE = 1; tie off otherwise)
  input  logic [31:0]  l2dbg [16]
);
  localparam logic [31:0] REG_ID         = 32'h5550_474E;   // "NGPU"
  localparam logic [15:0] REGMAP_VERSION = 16'h0000;

  function automatic logic [31:0] wmask(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
    logic [31:0] r;
    r = old;
    for (int i = 0; i < 4; i++) if (s[i]) r[8*i +: 8] = d[8*i +: 8];
    return r;
  endfunction

  // CP-owned offsets
  function automatic logic is_cp(input logic [19:0] a);
    return (a[19:13] == 7'h0) && ((a[12:8] == 5'h01) ||                // 0x0100..0x01FF
                                  (a[12] == 1'b1) ||                   // 0x1000..0x1FFF
                                  (a[12:2] == 11'h010) || (a[12:2] == 11'h011) ||   // 0x40, 0x44
                                  (a[12:2] == 11'h014) || (a[12:2] == 11'h015) || (a[12:2] == 11'h016) ||  // 0x50,54,58
                                  (DBG_PAGE && (a[12:8] == 5'h08) && (a[7:6] != 2'b11)) ||  // debug page, clk_ts part 0x800..0x8BF
                                  (DBG_PAGE && (a[12:8] == 5'h09) && (a[7:6] != 2'b11)));  // clk_ts blocks 0x900..0x9BF:x900..0x93F,
                                                                                            //x940..0x95Fx960..0x97F,
                                                                                            //x980..0x9BF
  endfunction

  // MMU-owned offsets: 0x0200..0x02FF and 0x0300..0x033F
  function automatic logic is_mmu(input logic [19:0] a);
    return (a[19:12] == 8'h00) && ((a[11:8] == 4'h2) || (a[11:6] == 6'b0011_00));
  endfunction

  logic        aw_pend, w_pend, ar_pend;
  logic [19:0] aw_addr, ar_addr;
  logic [31:0] w_data;
  logic [3:0]  w_strb;
  logic [31:0] scratch;
  logic        win_err;
  logic        ack_q, req_q;

  typedef enum logic [2:0] {S_IDLE, S_CP, S_B, S_R} st_t;
  st_t st;
  logic b_hs_q, r_hs_q;

  assign cp_req = req_q;

  assign csr_waddr = aw_addr[11:0];
  assign csr_wdata = w_data;
  assign csr_raddr = ar_addr[11:0];
  assign csr_wr    = (st == S_IDLE) && aw_pend && w_pend && is_mmu(aw_addr) && (w_strb == 4'hF);

  // ---- C3 window register semantics: illegal -> ENABLE cleared, ERR set
  // (checked on the new values of an ENABLE=1 configuration; a write that leaves ENABLE=0 is never "illegal")
  function automatic logic legal(input logic [4:0] lg, input logic [30:0] base);
    logic [31:0] span;
    span = 32'h1 << lg;
    return (lg >= 5'd20) && (lg <= 5'd27) && ((base & (span[30:0] - 31'h1)) == 31'h0) &&
           ({1'b0, base} + span <= 32'h8000_0000);
  endfunction

  function automatic logic [31:0] local_rd(input logic [19:0] a);
    if (DBG_PAGE && a[19:6] == 14'h0023) return l2dbg[a[5:2]];       // 0x08C0..0x08FF
    unique case ({a[19:2], 2'b00})
      20'h00000: return REG_ID;
      20'h00004: return {REGMAP_VERSION, RM_REV};
      20'h00008: return CAPS;
      20'h00010: return scratch;
      20'h00400: return {8'h00, 8'd27, 8'd20, 8'd28};                  // BAR2_INFO
      20'h00404: return {win_err, 18'h0, win_log2, 7'h0, win_en};      // C3_WIN_CTRL
      20'h00408: return {1'b0, win_base};
      default:   return 32'h0;
    endcase
  endfunction

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s_awready <= 1'b1; s_wready <= 1'b1; s_arready <= 1'b1;
      s_bvalid <= 1'b0; s_rvalid <= 1'b0;
      aw_pend <= 1'b0; w_pend <= 1'b0; ar_pend <= 1'b0;
      st <= S_IDLE; scratch <= '0;
      win_en <= 1'b0; win_log2 <= 5'd0; win_base <= '0; win_err <= 1'b0;
      b_hs_q <= 1'b0; r_hs_q <= 1'b0;
      req_q <= 1'b0; ack_q <= 1'b0; cp_wr <= 1'b0; cp_addr <= '0; cp_wdata <= '0;
    end else begin
      b_hs_q <= s_bvalid && s_bready;
      r_hs_q <= s_rvalid && s_rready;
      ack_q  <= cp_ack;
      if (s_bvalid && s_bready) s_bvalid <= 1'b0;
      if (s_rvalid && s_rready) s_rvalid <= 1'b0;

      if (s_awvalid && s_awready) begin aw_pend <= 1'b1; aw_addr <= s_awaddr; s_awready <= 1'b0; end
      if (s_wvalid && s_wready) begin
        w_pend <= 1'b1; w_data <= s_wdata; w_strb <= s_wstrb; s_wready <= 1'b0;
      end
      if (s_arvalid && s_arready) begin ar_pend <= 1'b1; ar_addr <= s_araddr; s_arready <= 1'b0; end

      unique case (st)
        S_IDLE: begin
          if (aw_pend && w_pend) begin
            if (is_mmu(aw_addr)) begin
              st <= S_B;                  // csr_wr is the comb strobe of this cycle
              s_bvalid <= 1'b1;
            end else if (is_cp(aw_addr)) begin
              cp_wr <= 1'b1; cp_addr <= aw_addr[12:0]; cp_wdata <= w_data;
              req_q <= ~req_q;
              st <= S_CP;
            end else begin
              st <= S_B;
              s_bvalid <= 1'b1;
              unique case ({aw_addr[19:2], 2'b00})
                20'h00010: scratch <= wmask(scratch, w_data, w_strb);
                20'h00404: begin       // C3_WIN_CTRL: [0] ENABLE, [12:8] SIZE_LOG2
                  win_log2 <= w_strb[1] ? w_data[12:8] : win_log2;
                  if ((w_strb[0] ? w_data[0] : win_en) &&
                      !legal(w_strb[1] ? w_data[12:8] : win_log2, win_base)) begin
                    win_en <= 1'b0; win_err <= 1'b1;
                  end else begin
                    win_en <= w_strb[0] ? w_data[0] : win_en; win_err <= 1'b0;
                  end
                end
                20'h00408: begin       // C3_WIN_BASE
                  win_base <= wmask({1'b0, win_base}, w_data, w_strb) & 32'h7FFF_FFFF;
                  if (win_en && !legal(win_log2, wmask({1'b0, win_base}, w_data, w_strb) & 32'h7FFF_FFFF)) begin
                    win_en <= 1'b0; win_err <= 1'b1;
                  end else win_err <= 1'b0;
                end
                default: ;
              endcase
            end
          end else if (ar_pend) begin
            if (is_mmu(ar_addr)) begin
              st <= S_R;
              s_rvalid <= 1'b1;
              s_rdata <= csr_rdata;
            end else if (is_cp(ar_addr)) begin
              cp_wr <= 1'b0; cp_addr <= ar_addr[12:0];
              req_q <= ~req_q;
              st <= S_CP;
            end else begin
              st <= S_R;
              s_rvalid <= 1'b1;
              s_rdata <= local_rd(ar_addr);
            end
          end
        end
        S_CP: ;                    // waits for the acknowledge below
        S_B: if (b_hs_q) begin
          aw_pend <= 1'b0; w_pend <= 1'b0; s_awready <= 1'b1; s_wready <= 1'b1;
          st <= S_IDLE;
        end
        default: if (r_hs_q) begin
          ar_pend <= 1'b0; s_arready <= 1'b1;
          st <= S_IDLE;
        end
      endcase

      // CP access done: the clk_ts side toggled `ack` to the value of `req`
      if (st == S_CP && ack_q == req_q) begin
        if (cp_wr) begin
          st <= S_B; s_bvalid <= 1'b1;
        end else begin
          st <= S_R; s_rvalid <= 1'b1; s_rdata <= cp_rdata;
        end
      end
    end
  end
endmodule
