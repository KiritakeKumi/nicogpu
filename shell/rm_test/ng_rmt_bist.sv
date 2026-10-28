// ng_rmt_bist.sv -- per-channel BIST engine of the test RM.
//
// Register word index (BAR0 0x8100 + 0x40*n + 4*idx; idx 16 = FIRST_EXP_HI at
// 0x8500 + 4*n):
//   0 CTRL(W: b0 START, b1 CLEAR)  1 STATUS(b0 BUSY, b1 DONE, b2 CFG_ERR,
//   b3 AXI_ERR, b4 ERR; b1..b3 W1C)  2 MODE  3 SEED  4 BASE_LO  5 BASE_HI
//   6 LEN  7 BURST_BEATS  8 ERRCNT  9 AXI_ERRCNT  10 FIRST_ERR_ADDR  11 CYC_W
//   12 CYC_R  13 FIRST_GOT_LO  14 FIRST_GOT_HI  15 FIRST_EXP_LO  16 FIRST_EXP_HI
//
// START runs: config check (OWNER==BIST, 1<=BURST_BEATS<=16, LEN!=0, LEN and
// BASE multiples of BB = BURST_BEATS*W/8; the "multiple of BURST_BEATS" part
// is a 32-step serial remainder) -> optional write pass -> optional check
// pass -> DONE + irq pulse.  Write pass: <= 8 bursts outstanding; check pass:
// <= 16.  Data comes from ng_rmt_gen through one FIFO (W data in the write
// pass, expected data in the check pass), injected on credit.
module ng_rmt_bist #(
  parameter int DW     = 512,
  parameter int AW     = 33,
  parameter int MUL_ST = 4
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          owner_ok,
  output logic          busy,
  output logic          done_pulse,
  // register access (one cycle; rdata is combinational for rb_idx)
  input  logic          rb_wr,
  input  logic [4:0]    rb_idx,
  input  logic [3:0]    rb_wstrb,
  input  logic [31:0]   rb_wdata,
  output logic [31:0]   rb_rdata,
  // master (internal AXI, ID 0, full-size INCR)
  output logic          m_awvalid,
  input  logic          m_awready,
  output logic [AW-1:0] m_awaddr,
  output logic [7:0]    m_awlen,
  output logic          m_wvalid,
  input  logic          m_wready,
  output logic [DW-1:0] m_wdata,
  output logic          m_wlast,
  input  logic          m_bvalid,
  output logic          m_bready,
  input  logic [1:0]    m_bresp,
  output logic          m_arvalid,
  input  logic          m_arready,
  output logic [AW-1:0] m_araddr,
  output logic [7:0]    m_arlen,
  input  logic          m_rvalid,
  output logic          m_rready,
  input  logic [DW-1:0] m_rdata,
  input  logic [1:0]    m_rresp
);
  localparam int L   = DW / 64;
  localparam int LG  = $clog2(DW / 8);
  localparam int BIW = AW - LG;
  localparam int FD  = 32;

  function automatic logic [31:0] wmask(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
    logic [31:0] r;
    r = old;
    for (int i = 0; i < 4; i++) if (s[i]) r[8*i +: 8] = d[8*i +: 8];
    return r;
  endfunction

  // --------------------------------------------------------------
  logic [2:0]  mode;
  logic [31:0] seed, base_lo, len;
  logic        base_hi;
  logic [4:0]  bb;
  logic        st_done, st_cfg, st_axi;
  logic [31:0] errcnt, axierrcnt, first_addr, cyc_w, cyc_r;
  logic [63:0] first_got, first_exp;
  logic        first_v;

  logic [AW-1:0] base;
  assign base = AW'({base_hi, base_lo});

  // --------------------------------------------------------------
  typedef enum logic [2:0] {S_IDLE, S_CHK, S_WR, S_RD, S_FIN} st_t;
  st_t st;
  assign busy = (st != S_IDLE);

  logic [5:0]  chk_n;
  logic [31:0] chk_l, chk_b;
  logic [4:0]  rem_l, rem_b;
  logic        chk_bad;

  logic [AW-1:0]  a_addr;       // next AW/AR address
  logic [31:0]    a_left;       // beats still to be covered by AW/AR
  logic [31:0]    a_cnt, rsp_cnt;  // bursts issued / completed (B or RLAST)
  logic [BIW-1:0] g_beat;       // next generator beat index
  logic [31:0]    g_left;
  logic [6:0]     occ;          // generator + FIFO occupancy
  logic [4:0]     wbeat;        // beat within the current W / R burst
  logic [31:0]    r_left;       // R beats still expected (check pass)
  logic [AW-1:0]  r_addr;       // byte address of the next R beat
  logic           cyc_run;

  wire [AW-1:0] bb_bytes = AW'(bb) << LG;
  wire [31:0]   len_beats = len >> LG;

  // ---------------------------------------------------------------- generator + FIFO
  logic          g_in, g_out;
  logic [DW-1:0] g_data, f_dout;
  logic          f_empty, f_pop;
  logic [$clog2(FD):0] f_count;

  assign g_in = ((st == S_WR) || (st == S_RD)) && (g_left != 0) && (occ < 7'(FD));

  ng_rmt_gen #(.L(L), .LG(LG), .BIW(BIW), .MUL_ST(MUL_ST)) u_gen (
    .clk(clk), .rst_n(rst_n), .pat(mode[2]), .seed(seed),
    .in_valid(g_in), .in_beat(g_beat), .out_valid(g_out), .out_data(g_data));

  ng_prim_fifo_sync #(.W(DW), .DEPTH(FD), .REG_FLAGS(1'b1)) u_fifo (
    .clk(clk), .rst_n(rst_n), .push(g_out), .din(g_data), .pop(f_pop),
    .dout(f_dout), .empty(f_empty), .full(), .count(f_count));

  // --------------------------------------------------------------
  wire limit_ok = (st == S_WR) ? ((a_cnt - rsp_cnt) < 32'd8) : ((a_cnt - rsp_cnt) < 32'd16);
  assign m_awvalid = (st == S_WR) && (a_left != 0) && limit_ok;
  assign m_awaddr  = a_addr;
  assign m_awlen   = 8'(bb - 5'd1);
  assign m_arvalid = (st == S_RD) && (a_left != 0) && limit_ok;
  assign m_araddr  = a_addr;
  assign m_arlen   = 8'(bb - 5'd1);
  assign m_wvalid  = (st == S_WR) && !f_empty;
  assign m_wdata   = f_dout;
  assign m_wlast   = (wbeat == bb - 5'd1);
  assign m_bready  = 1'b1;
  assign m_rready  = (st == S_RD) && !f_empty;

  wire aw_hs = m_awvalid && m_awready;
  wire ar_hs = m_arvalid && m_arready;
  wire w_hs  = m_wvalid && m_wready;
  wire r_hs  = m_rvalid && m_rready;
  wire b_hs  = m_bvalid;              // bready = 1
  assign f_pop = w_hs || r_hs;

  // ---------------------------------------------------------------- compare pipeline
  logic          pa_v, pb_v, pc_v;
  logic          pa_rerr, pb_rerr;
  logic [DW-1:0] pa_d, pa_e, pb_d, pb_e, pc_d, pc_e;
  logic [AW-1:0] pa_addr, pb_addr, pc_addr;
  logic [L-1:0]  pb_mm, pc_oh;
  logic          pc_take;
  logic [63:0]   sel_got, sel_exp;

  always_comb begin
    sel_got = '0;
    sel_exp = '0;
    for (int k = 0; k < L; k++) begin
      sel_got |= pc_d[64*k +: 64] & {64{pc_oh[k]}};
      sel_exp |= pc_e[64*k +: 64] & {64{pc_oh[k]}};
    end
  end

  always_ff @(posedge clk) begin
    pa_d    <= m_rdata;
    pa_e    <= f_dout;
    pa_addr <= r_addr;
    pa_rerr <= (m_rresp != 2'b00);
    pb_d    <= pa_d;
    pb_e    <= pa_e;
    pb_addr <= pa_addr;
    pb_rerr <= pa_rerr;
    for (int k = 0; k < L; k++) pb_mm[k] <= (pa_d[64*k +: 64] != pa_e[64*k +: 64]);
    pc_d    <= pb_d;
    pc_e    <= pb_e;
    pc_addr <= pb_addr;
    pc_oh   <= pb_mm & (~pb_mm + 1'b1);
  end

  wire pipe_empty = !pa_v && !pb_v && !pc_v;
  wire [5:0] tl = {rem_l, chk_l[31]};
  wire [5:0] tb = {rem_b, chk_b[31]};

  // --------------------------------------------------------------
  wire rb_ctrl = rb_wr && (rb_idx == 5'd0);
  wire do_clear = rb_ctrl && rb_wstrb[0] && rb_wdata[1];
  wire do_start = rb_ctrl && rb_wstrb[0] && rb_wdata[0] && (st == S_IDLE);
  wire cfg_wr   = rb_wr && (st == S_IDLE);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      mode <= '0; seed <= '0; base_lo <= '0; base_hi <= 1'b0; len <= '0; bb <= 5'd16;
      st_done <= 1'b0; st_cfg <= 1'b0; st_axi <= 1'b0;
      errcnt <= '0; axierrcnt <= '0; first_addr <= '0; cyc_w <= '0; cyc_r <= '0;
      first_got <= '0; first_exp <= '0; first_v <= 1'b0;
      st <= S_IDLE; done_pulse <= 1'b0; cyc_run <= 1'b0;
      chk_n <= '0; chk_l <= '0; chk_b <= '0; rem_l <= '0; rem_b <= '0; chk_bad <= 1'b0;
      a_addr <= '0; a_left <= '0; a_cnt <= '0; rsp_cnt <= '0; g_beat <= '0; g_left <= '0;
      occ <= '0; wbeat <= '0; r_left <= '0; r_addr <= '0;
      pa_v <= 1'b0; pb_v <= 1'b0; pc_v <= 1'b0; pc_take <= 1'b0;
    end else begin
      done_pulse <= 1'b0;

      // ---- register writes
      if (cfg_wr) begin
        unique case (rb_idx)
          5'd2: mode    <= 3'(wmask(32'(mode), rb_wdata, rb_wstrb));
          5'd3: seed    <= wmask(seed, rb_wdata, rb_wstrb);
          5'd4: base_lo <= wmask(base_lo, rb_wdata, rb_wstrb);
          5'd5: if (rb_wstrb[0]) base_hi <= rb_wdata[0];
          5'd6: len     <= wmask(len, rb_wdata, rb_wstrb);
          5'd7: if (rb_wstrb[0]) bb <= rb_wdata[4:0];
          default: ;
        endcase
      end
      if (rb_wr && (rb_idx == 5'd1) && rb_wstrb[0]) begin
        if (rb_wdata[1]) st_done <= 1'b0;
        if (rb_wdata[2]) st_cfg  <= 1'b0;
        if (rb_wdata[3]) st_axi  <= 1'b0;
      end

      // ---- generator / FIFO occupancy
      occ <= occ + 7'(g_in) - 7'(f_pop);
      if (g_in) begin
        g_beat <= g_beat + 1'b1;
        g_left <= g_left - 1'b1;
      end

      // ---- address issue
      if (aw_hs || ar_hs) begin
        a_addr  <= a_addr + bb_bytes;
        a_left  <= a_left - 32'(bb);
        a_cnt   <= a_cnt + 1'b1;
        cyc_run <= 1'b1;
      end
      if (cyc_run) begin
        if (st == S_WR) cyc_w <= cyc_w + 1'b1;
        if (st == S_RD) cyc_r <= cyc_r + 1'b1;
      end

      // ---- W beats
      if (w_hs) wbeat <= m_wlast ? 5'd0 : wbeat + 5'd1;

      // --
      if (b_hs && st == S_WR) begin
        rsp_cnt <= rsp_cnt + 1'b1;
        if (m_bresp != 2'b00) begin
          st_axi    <= 1'b1;
          axierrcnt <= axierrcnt + 1'b1;
        end
      end

      // ---- R -> compare pipeline
      pa_v <= r_hs;
      if (r_hs) begin
        r_left <= r_left - 1'b1;
        r_addr <= r_addr + AW'(DW / 8);
        if (wbeat == bb - 5'd1) begin
          wbeat   <= 5'd0;
          rsp_cnt <= rsp_cnt + 1'b1;
        end else begin
          wbeat <= wbeat + 5'd1;
        end
      end
      pb_v <= pa_v;
      pc_v <= pb_v && (pb_rerr || (pb_mm != '0));
      pc_take <= 1'b0;
      if (pb_v) begin
        if (pb_rerr) begin
          st_axi    <= 1'b1;
          axierrcnt <= axierrcnt + 1'b1;
        end else if (pb_mm != '0) begin
          errcnt <= errcnt + 1'b1;
          if (!first_v) begin
            first_v <= 1'b1;
            pc_take <= 1'b1;
          end
        end
      end
      if (pc_take) begin
        first_addr <= 32'(pc_addr);
        first_got  <= sel_got;
        first_exp  <= sel_exp;
      end

      // --
      unique case (st)
        S_IDLE: if (do_start) begin
          st      <= S_CHK;
          st_done <= 1'b0;
          cyc_w   <= '0;
          cyc_r   <= '0;
          chk_n   <= '0;
          chk_l   <= len_beats;
          chk_b   <= 32'(base >> LG);
          rem_l   <= '0;
          rem_b   <= '0;
          chk_bad <= !owner_ok || (bb == 5'd0) || (bb > 5'd16) || (len == 0) ||
                     (len[LG-1:0] != '0) || (base[LG-1:0] != '0);
        end
        S_CHK: begin
          // serial remainder of (LEN/BPB) and (BASE/BPB) modulo BURST_BEATS
          if (chk_n != 6'd32) begin
            rem_l <= 5'((tl >= 6'(bb)) ? tl - 6'(bb) : tl);
            rem_b <= 5'((tb >= 6'(bb)) ? tb - 6'(bb) : tb);
            chk_l <= chk_l << 1;
            chk_b <= chk_b << 1;
            chk_n <= chk_n + 1'b1;
          end else if (chk_bad || rem_l != '0 || rem_b != '0) begin
            st_cfg <= 1'b1;
            st     <= S_IDLE;
          end else begin
            a_addr  <= base;
            a_left  <= len_beats;
            a_cnt   <= '0;
            rsp_cnt <= '0;
            g_beat  <= BIW'(base >> LG);
            g_left  <= len_beats;
            wbeat   <= '0;
            r_left  <= len_beats;
            r_addr  <= base;
            cyc_run <= 1'b0;
            st      <= mode[0] ? S_WR : mode[1] ? S_RD : S_FIN;
          end
        end
        S_WR: if (a_left == 0 && !m_awvalid && rsp_cnt == a_cnt && !(b_hs)) begin
          cyc_run <= 1'b0;
          a_addr  <= base;
          a_left  <= len_beats;
          a_cnt   <= '0;
          rsp_cnt <= '0;
          g_beat  <= BIW'(base >> LG);
          g_left  <= len_beats;
          wbeat   <= '0;
          st      <= mode[1] ? S_RD : S_FIN;
        end
        S_RD: if (a_left == 0 && r_left == 0 && pipe_empty && !pc_take) begin
          cyc_run <= 1'b0;
          st      <= S_FIN;
        end
        default: begin   // S_FIN
          st         <= S_IDLE;
          st_done    <= 1'b1;
          done_pulse <= 1'b1;
        end
      endcase

      // ---- CLEAR (after everything else: wins over same-cycle updates)
      if (do_clear) begin
        st_done   <= 1'b0;
        errcnt    <= '0;
        axierrcnt <= '0;
        cyc_w     <= '0;
        cyc_r     <= '0;
        first_v   <= 1'b0;
        first_addr <= '0;
        first_got <= '0;
        first_exp <= '0;
      end
    end
  end

  // ---------------------------------------------------------------- read mux
  always_comb begin
    unique case (rb_idx)
      5'd1:  rb_rdata = {27'h0, (errcnt != 0), st_axi, st_cfg, st_done, busy};
      5'd2:  rb_rdata = 32'(mode);
      5'd3:  rb_rdata = seed;
      5'd4:  rb_rdata = base_lo;
      5'd5:  rb_rdata = {31'h0, base_hi};
      5'd6:  rb_rdata = len;
      5'd7:  rb_rdata = 32'(bb);
      5'd8:  rb_rdata = errcnt;
      5'd9:  rb_rdata = axierrcnt;
      5'd10: rb_rdata = first_addr;
      5'd11: rb_rdata = cyc_w;
      5'd12: rb_rdata = cyc_r;
      5'd13: rb_rdata = first_got[31:0];
      5'd14: rb_rdata = first_got[63:32];
      5'd15: rb_rdata = first_exp[31:0];
      5'd16: rb_rdata = first_exp[63:32];
      default: rb_rdata = 32'h0;
    endcase
  end
endmodule
