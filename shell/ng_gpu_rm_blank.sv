// ng_gpu_rm_blank.sv -- the blank ("empty shell") reconfigurable module.
//
// Every RM declares exactly the generated port list (ng_gpu_rm_ports.svh).
// This one implements only what the static shell needs for bring-up (P0-11):
//   *  identification/IRQ registers on s_axil_ctrl, CAPS = 0 (no core)
//   * a blank-RM-only test page at BAR0 + 0xFF000 (DFX_BOUNDARY.md)
//   * s_axi_bar2 answers OKAY (reads return 0, writes are dropped)
//   * all master ports idle, quiesce acknowledged immediately
//   * rm_info with rm_kind = blank and a 32-bit BSCAN observation register
// All boundary outputs are driven directly by flip-flops or constants.
module ng_gpu_rm
  import ng_dfx_pkg::*;
#(
  parameter logic [31:0] BUILD_ID = 32'h0000_0000,
  parameter logic [15:0] RM_REV   = 16'h0001
) (
`include "ng_gpu_rm_ports.svh"
);
  localparam logic [31:0] REG_ID         = 32'h5550_474E;  // "NGPU",
  localparam logic [15:0] REGMAP_VERSION = 16'h0000;

  // ----------------------------------------------------------------
  assign rm_info = {12'h000, 4'h0, RM_REV,               // dw3: reserved, mem_mask = 0, rm_rev
                    BUILD_ID,                            // dw2
                    NG_RM_KIND_BLANK, NG_DFX_IFACE_VERSION,  // dw1
                    NG_RM_INFO_MAGIC};                   // dw0

  // ----------------------------------------------------------------
  always_ff @(posedge clk_l2) begin
    if (!rst_l2_n) begin
      rm_ready       <= 1'b0;
      rm_quiesce_ack <= 1'b0;
    end else begin
      rm_ready       <= 1'b1;
      rm_quiesce_ack <= shell_quiesce_req;   // no master traffic to drain
    end
  end

  // ------------------------------------------------------------------ control registers
  logic [31:0] scratch, irq_status, irq_enable;
  logic        aw_pend, w_pend;
  logic [19:0] aw_addr;
  logic [31:0] w_data;
  logic [3:0]  w_strb;

  assign s_axil_ctrl_awready = !aw_pend && !s_axil_ctrl_bvalid;
  assign s_axil_ctrl_wready  = !w_pend && !s_axil_ctrl_bvalid;
  assign s_axil_ctrl_arready = !s_axil_ctrl_rvalid;

  function automatic logic [31:0] wmask(input logic [31:0] old, input logic [31:0] d, input logic [3:0] s);
    logic [31:0] r;
    r = old;
    for (int i = 0; i < 4; i++) if (s[i]) r[8*i +: 8] = d[8*i +: 8];
    return r;
  endfunction

  always_ff @(posedge clk_l2) begin
    if (!rst_l2_n) begin
      aw_pend            <= 1'b0;
      w_pend             <= 1'b0;
      s_axil_ctrl_bvalid <= 1'b0;
      s_axil_ctrl_rvalid <= 1'b0;
      scratch            <= '0;
      irq_status         <= '0;
      irq_enable         <= '0;
      rm_irq             <= '0;
    end else begin
      if (s_axil_ctrl_awvalid && s_axil_ctrl_awready) begin
        aw_pend <= 1'b1;
        aw_addr <= s_axil_ctrl_awaddr;
      end
      if (s_axil_ctrl_wvalid && s_axil_ctrl_wready) begin
        w_pend <= 1'b1;
        w_data <= s_axil_ctrl_wdata;
        w_strb <= s_axil_ctrl_wstrb;
      end
      if (aw_pend && w_pend) begin
        aw_pend            <= 1'b0;
        w_pend             <= 1'b0;
        s_axil_ctrl_bvalid <= 1'b1;
        case ({aw_addr[19:2], 2'b00})
          20'h00010: scratch    <= wmask(scratch, w_data, w_strb);
          20'h00040: irq_status <= irq_status & ~wmask('0, w_data, w_strb);
          20'h00044: irq_enable <= wmask(irq_enable, w_data, w_strb) & 32'h7;
          20'hFF000: irq_status <= irq_status | (wmask('0, w_data, w_strb) & 32'h7);  // blank test: raise IRQ bits
          default: ;
        endcase
      end
      if (s_axil_ctrl_bvalid && s_axil_ctrl_bready) s_axil_ctrl_bvalid <= 1'b0;

      if (s_axil_ctrl_arvalid && s_axil_ctrl_arready) begin
        s_axil_ctrl_rvalid <= 1'b1;
        case ({s_axil_ctrl_araddr[19:2], 2'b00})
          20'h00000: s_axil_ctrl_rdata <= REG_ID;
          20'h00004: s_axil_ctrl_rdata <= {REGMAP_VERSION, RM_REV};
          20'h00008: s_axil_ctrl_rdata <= 32'h0;                // CAPS: no cores
          20'h00010: s_axil_ctrl_rdata <= scratch;
          20'h00040: s_axil_ctrl_rdata <= irq_status;
          20'h00044: s_axil_ctrl_rdata <= irq_enable;
          default:   s_axil_ctrl_rdata <= 32'h0;
        endcase
      end
      if (s_axil_ctrl_rvalid && s_axil_ctrl_rready) s_axil_ctrl_rvalid <= 1'b0;

      rm_irq <= {3'b000, |(irq_status & irq_enable)};
    end
  end
  assign s_axil_ctrl_bresp = 2'b00;
  assign s_axil_ctrl_rresp = 2'b00;

  // ------------------------------------------------------------------ BAR2 window: OKAY, zeros
  // One transaction at a time, answered in order (boundary rule).
  logic       rd_busy;
  logic [7:0] rd_left;
  logic       wr_busy, wr_data_done;

  assign s_axi_bar2_arready = !rd_busy;
  assign s_axi_bar2_awready = !wr_busy;
  assign s_axi_bar2_wready  = wr_busy && !wr_data_done;
  assign s_axi_bar2_rdata   = '0;
  assign s_axi_bar2_rresp   = 2'b00;
  assign s_axi_bar2_bresp   = 2'b00;

  always_ff @(posedge clk_l2) begin
    if (!rst_l2_n) begin
      rd_busy           <= 1'b0;
      s_axi_bar2_rvalid <= 1'b0;
      wr_busy           <= 1'b0;
      wr_data_done      <= 1'b0;
      s_axi_bar2_bvalid <= 1'b0;
    end else begin
      if (s_axi_bar2_arvalid && s_axi_bar2_arready) begin
        rd_busy           <= 1'b1;
        rd_left           <= s_axi_bar2_arlen;
        s_axi_bar2_rid    <= s_axi_bar2_arid;
        s_axi_bar2_rvalid <= 1'b1;
        s_axi_bar2_rlast  <= (s_axi_bar2_arlen == 8'd0);
      end else if (s_axi_bar2_rvalid && s_axi_bar2_rready) begin
        if (s_axi_bar2_rlast) begin
          rd_busy           <= 1'b0;
          s_axi_bar2_rvalid <= 1'b0;
        end else begin
          rd_left          <= rd_left - 8'd1;
          s_axi_bar2_rlast <= (rd_left == 8'd1);
        end
      end

      if (s_axi_bar2_awvalid && s_axi_bar2_awready) begin
        wr_busy        <= 1'b1;
        s_axi_bar2_bid <= s_axi_bar2_awid;
      end
      if (s_axi_bar2_wvalid && s_axi_bar2_wready && s_axi_bar2_wlast) begin
        wr_data_done      <= 1'b1;
        s_axi_bar2_bvalid <= 1'b1;
      end
      if (s_axi_bar2_bvalid && s_axi_bar2_bready) begin
        s_axi_bar2_bvalid <= 1'b0;
        wr_busy           <= 1'b0;
        wr_data_done      <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------ master ports: idle
  `define NG_RM_IDLE_MASTER(p) \
    assign p``_awvalid = 1'b0; assign p``_wvalid = 1'b0; assign p``_arvalid = 1'b0; \
    assign p``_bready = 1'b1;  assign p``_rready = 1'b1; \
    assign p``_awid = '0; assign p``_awaddr = '0; assign p``_awlen = '0; assign p``_awsize = '0; \
    assign p``_awburst = 2'b01; assign p``_awcache = 4'b0011; assign p``_awprot = '0; assign p``_awqos = '0; \
    assign p``_wdata = '0; assign p``_wstrb = '0; assign p``_wlast = 1'b0; \
    assign p``_arid = '0; assign p``_araddr = '0; assign p``_arlen = '0; assign p``_arsize = '0; \
    assign p``_arburst = 2'b01; assign p``_arcache = 4'b0011; assign p``_arprot = '0; assign p``_arqos = '0;
  `NG_RM_IDLE_MASTER(m_axi_host)
  `NG_RM_IDLE_MASTER(m_axi_c1)
  `NG_RM_IDLE_MASTER(m_axi_c2)
  `NG_RM_IDLE_MASTER(m_axi_c3)
  `NG_RM_IDLE_MASTER(m_axi_c4)
  `undef NG_RM_IDLE_MASTER

  // ----------------------------------------------------------------
  // Vivado debug hub tunnel unused in the blank RM.
  assign dbg_hub_tdo = 1'b0;

  // USER4 observation chain: 32-bit register, CAPTURE loads rm_info dw0.
  logic [31:0] obs_sr;
  always_ff @(posedge dbg_obs_tck) begin
    if (dbg_obs_sel && dbg_obs_capture) obs_sr <= NG_RM_INFO_MAGIC;
    else if (dbg_obs_sel && dbg_obs_shift) obs_sr <= {dbg_obs_tdi, obs_sr[31:1]};
  end
  always_ff @(negedge dbg_obs_tck) dbg_obs_tdo <= obs_sr[0];
endmodule
