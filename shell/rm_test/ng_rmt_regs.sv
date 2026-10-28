// ng_rmt_regs.sv -- s_axil_ctrl register block of the test RM.
//
// 0x0000 ID, 0x0004 REV, 0x0008 CAPS=0, 0x0010 SCRATCH,
//            0x0040 IRQ_STATUS (W1C), 0x0044 IRQ_ENABLE[2:0]
// Test block: 0x8000 T_ID, 0x8004 T_VERSION, 0x8008 T_MEM_READY, 0x800C T_STATUS
//            (b0 OWNER_ERR W1C), 0x8010+4n OWNER[n], 0x8020+4n WIN_SEL[n],
//            0x8030 T_FREQ_L2=0, 0x8100+0x40n BIST[n], 0x8500+4n FIRST_EXP_HI[n],
//            0x8600 DMA.  Undefined: read 0, write dropped; always OKAY.
// BIST and OWNER live in the channel units: accesses travel over the
// pipelined register bus and the B/R is returned when the unit answers (side
// effect done).  OWNER keeps a local mirror (read back here, used by the DMA
// config check); a write is refused (T_STATUS.OWNER_ERR) when the SLR1 side
// has BAR2/DMA work on that channel or the unit reports it is not idle.
// All s_axil_ctrl outputs are flip-flops.
module ng_rmt_regs #(
  parameter logic [15:0] RM_REV = 16'h0001
) (
  input  logic         clk,
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
  // channel register bus
  output logic [3:0]   rq_valid,
  output logic [41:0]  rq_data,     // {we, idx[4:0], wstrb, wdata}
  input  logic [3:0]   rs_valid,
  input  logic [31:0]  rs_data [4],
  // DMA registers
  output logic         d_wr,
  output logic [3:0]   d_idx,
  output logic [3:0]   d_wstrb,
  output logic [31:0]  d_wdata,
  input  logic [31:0]  d_rdata,
  // state
  output logic [6:0]   win_sel [4],
  output logic [1:0]   owner [4],
  input  logic [3:0]   ch_busy,
  input  logic [3:0]   mem_ready,
  input  logic         irq_set,
  output logic         rm_irq0
);
  localparam logic [31:0] REG_ID         = 32'h5550_474E;  // "NGPU"
  localparam logic [15:0] REGMAP_VERSION = 16'h0000;
  localparam logic [31:0] T_ID_VAL       = 32'h3154_5354;  // "TST1"
  localparam logic [31:0] T_VERSION_VAL  = 32'h0001_0000;
  localparam logic [6:0]  WIN_MASK [4]   = '{7'h7F, 7'h3F, 7'h1F, 7'h3F};

  function automatic logic [31:0] wmask(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
    logic [31:0] r;
    r = old;
    for (int i = 0; i < 4; i++) if (s[i]) r[8*i +: 8] = d[8*i +: 8];
    return r;
  endfunction

  logic        aw_pend, w_pend, ar_pend;
  logic [19:0] aw_addr, ar_addr;
  logic [31:0] w_data;
  logic [3:0]  w_strb;
  logic [31:0] scratch, irq_status, irq_enable;
  logic        owner_err;
  logic [3:0]  mem_q;

  typedef enum logic [2:0] {S_IDLE, S_WCH, S_WOWN, S_RCH, S_B, S_R} st_t;
  st_t        st;
  logic [1:0] wn;
  logic       b_hs_q, r_hs_q;   // registered B / R handshakes (BREADY/RREADY: one LUT to a flop)

  // --------------------------------------------------------------
  typedef struct packed {
    logic       bist;     // BIST[n] word idx (0..15) or FIRST_EXP_HI (idx 16)
    logic       own;
    logic       win;
    logic       dma;
    logic [1:0] n;
    logic [4:0] idx;
  } dec_t;

  function automatic dec_t decode(input logic [19:0] a);
    dec_t d;
    d = '0;
    if (a[19:8] == 12'h081) begin
      d.bist = 1'b1; d.n = a[7:6]; d.idx = {1'b0, a[5:2]};
    end else if (a[19:4] == 16'h0850) begin
      d.bist = 1'b1; d.n = a[3:2]; d.idx = 5'd16;
    end else if (a[19:4] == 16'h0801) begin
      d.own = 1'b1; d.n = a[3:2];
    end else if (a[19:4] == 16'h0802) begin
      d.win = 1'b1; d.n = a[3:2];
    end else if (a[19:6] == 14'h0218 && a[5:2] <= 4'd11) begin
      d.dma = 1'b1; d.idx = {1'b0, a[5:2]};
    end
    return d;
  endfunction

  dec_t wd, rd;
  assign wd = decode(aw_addr);
  assign rd = decode(ar_addr);

  function automatic logic [31:0] local_rd(input logic [19:0] a);
    unique case ({a[19:2], 2'b00})
      20'h00000: return REG_ID;
      20'h00004: return {REGMAP_VERSION, RM_REV};
      20'h00008: return 32'h0;
      20'h00010: return scratch;
      20'h00040: return irq_status;
      20'h00044: return irq_enable;
      20'h08000: return T_ID_VAL;
      20'h08004: return T_VERSION_VAL;
      20'h08008: return {28'h0, mem_q};
      20'h0800C: return {31'h0, owner_err};
      20'h08030: return 32'h0;
      default:   return 32'h0;
    endcase
  endfunction

  logic [3:0] d_widx;
  assign d_idx   = d_wr ? d_widx : rd.idx[3:0];
  assign d_wstrb = w_strb;
  assign d_wdata = w_data;

  // --------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      s_awready <= 1'b1; s_wready <= 1'b1; s_arready <= 1'b1;
      s_bvalid <= 1'b0; s_rvalid <= 1'b0;
      aw_pend <= 1'b0; w_pend <= 1'b0; ar_pend <= 1'b0;
      st <= S_IDLE; wn <= '0;
      scratch <= '0; irq_status <= '0; irq_enable <= '0; owner_err <= 1'b0;
      win_sel <= '{default: '0}; owner <= '{default: '0};
      rq_valid <= '0; d_wr <= 1'b0; rm_irq0 <= 1'b0; mem_q <= '0;
      b_hs_q <= 1'b0; r_hs_q <= 1'b0;
    end else begin
      rq_valid <= '0;
      d_wr     <= 1'b0;
      mem_q    <= mem_ready;
      b_hs_q   <= s_bvalid && s_bready;
      r_hs_q   <= s_rvalid && s_rready;
      if (s_bvalid && s_bready) s_bvalid <= 1'b0;
      if (s_rvalid && s_rready) s_rvalid <= 1'b0;

      if (s_awvalid && s_awready) begin
        aw_pend <= 1'b1; aw_addr <= s_awaddr; s_awready <= 1'b0;
      end
      if (s_wvalid && s_wready) begin
        w_pend <= 1'b1; w_data <= s_wdata; w_strb <= s_wstrb; s_wready <= 1'b0;
      end
      if (s_arvalid && s_arready) begin
        ar_pend <= 1'b1; ar_addr <= s_araddr; s_arready <= 1'b0;
      end

      unique case (st)
        S_IDLE: begin
          if (aw_pend && w_pend) begin
            wn <= wd.n;
            st <= S_B;
            s_bvalid <= 1'b1;
            if (wd.bist) begin
              if (wd.idx != 5'd16) begin
                rq_valid[wd.n] <= 1'b1;
                rq_data <= {1'b1, wd.idx, w_strb, w_data};
                st <= S_WCH;
                s_bvalid <= 1'b0;
              end
            end else if (wd.own) begin
              if (ch_busy[wd.n]) begin
                owner_err <= 1'b1;
              end else begin
                rq_valid[wd.n] <= 1'b1;
                rq_data <= {1'b1, 5'd17, w_strb, w_data};
                st <= S_WOWN;
                s_bvalid <= 1'b0;
              end
            end else if (wd.win) begin
              if (w_strb[0]) win_sel[wd.n] <= w_data[6:0] & WIN_MASK[wd.n];
            end else if (wd.dma) begin
              d_wr   <= 1'b1;
              d_widx <= wd.idx[3:0];
            end else begin
              unique case ({aw_addr[19:2], 2'b00})
                20'h00010: scratch    <= wmask(scratch, w_data, w_strb);
                20'h00040: irq_status <= irq_status & ~wmask('0, w_data, w_strb);
                20'h00044: irq_enable <= wmask(irq_enable, w_data, w_strb) & 32'h7;
                20'h0800C: if (w_strb[0] && w_data[0]) owner_err <= 1'b0;
                default: ;
              endcase
            end
          end else if (ar_pend) begin
            wn <= rd.n;
            st <= S_R;
            s_rvalid <= 1'b1;
            if (rd.bist) begin
              rq_valid[rd.n] <= 1'b1;
              rq_data <= {1'b0, rd.idx, 4'h0, 32'h0};
              st <= S_RCH;
              s_rvalid <= 1'b0;
            end else if (rd.own) begin
              s_rdata <= 32'(owner[rd.n]);
            end else if (rd.win) begin
              s_rdata <= 32'(win_sel[rd.n]);
            end else if (rd.dma) begin
              s_rdata <= d_rdata;
            end else begin
              s_rdata <= local_rd(ar_addr);
            end
          end
        end
        S_WCH: if (rs_valid[wn]) begin
          st <= S_B;
          s_bvalid <= 1'b1;
        end
        S_WOWN: if (rs_valid[wn]) begin
          owner[wn] <= rs_data[wn][1:0];
          if (!rs_data[wn][2]) owner_err <= 1'b1;
          st <= S_B;
          s_bvalid <= 1'b1;
        end
        S_RCH: if (rs_valid[wn]) begin
          s_rdata <= rs_data[wn];
          st <= S_R;
          s_rvalid <= 1'b1;
        end
        S_B: if (b_hs_q) begin
          aw_pend <= 1'b0; w_pend <= 1'b0;
          s_awready <= 1'b1; s_wready <= 1'b1;
          st <= S_IDLE;
        end
        default: if (r_hs_q) begin   // S_R
          ar_pend <= 1'b0;
          s_arready <= 1'b1;
          st <= S_IDLE;
        end
      endcase

      if (irq_set) irq_status[0] <= 1'b1;
      rm_irq0 <= |(irq_status & irq_enable);
    end
  end
endmodule
