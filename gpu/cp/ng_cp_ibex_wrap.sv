// ng_cp_ibex_wrap.sv -- socket for the real lowRISC Ibex core (RV32IMC small config).
//
// Replaces the DPI stub when `NG_CP_IBEX` is defined for ng_cp_top.  Ibex is
// lowRISC/ibex v2.1.0 (third_party/README.md); build with rtl/gpu/cp/ibex/ibex_files.f.
//
// Memory map seen by the core (fw/cp/ibex/link.ld,
//   0x0000_0000  IMEM 64 KiB   instruction fetch + read-only data (.rodata lives in .text);
//                              vector table 0x00..0x7f, reset vector 0x80 (boot_addr_i = 0)
//   0x0010_0000  DMEM 32 KiB .bss + stack
//   0x4000_0000  register mirror (BAR0 CP page), 0x4002_0000 private port -> ng_cp_port
// Everything else answers with a bus error.
//
// IMEM/DMEM loading: the hardware boot loader ng_cp_boot  writes the NGCF text/data segments
// and zeroes the bss through the ld_* port while the core is held in reset.  For the older testbenches
// (ng_cp_top BOOT_LOADER = 0) the IMEM array is preloaded in simulation from IMAGE_FILE ($readmemh, one 32-bit
// word per line, fw/cp/tools/bin2hex.py).  IMEM/DMEM contents survive a CORE_RESET until the loader rewrites them.
//
// Bus rules: one outstanding data access (the next grant is issued the cycle
// after the response), so DMEM/IMEM (1-cycle) and the private port (1-cycle)
// responses never reorder.  Configuration: RV32M Fast (selectable), no B, no
// PMP, no I-cache, no debug triggers, no writeback stage, 2 mcycle counters only.
module ng_cp_ibex_wrap #(
  parameter int unsigned IMEM_WORDS = 16384,
  parameter int unsigned DMEM_WORDS = 8192,
  parameter logic [31:0] DMEM_BASE  = 32'h0010_0000,
  parameter int unsigned QUIET_WORD = 0,           // DMEM word holding ngfw_ibex_quiet (sim settle detect)
  parameter              IMAGE_FILE = "",          // simulation preload ("" = none)
  parameter ibex_pkg::rv32m_e RV32M = ibex_pkg::RV32MFast
) (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        irq_i,
  input  logic        timer_irq_i,       // 1 ms tick (ng_cp_port): wakes the firmware's wfi so it can bump CP_HEARTBEAT

  // image loader (ng_cp_boot): one word per cycle while the core is in reset
  input  logic        ld_we_i,
  input  logic        ld_dmem_i,       // 0 = IMEM, 1 = DMEM
  input  logic [31:0] ld_addr_i,       // byte address inside the selected memory
  input  logic [31:0] ld_data_i,
  input  logic [31:0] boot_addr_i,     // Ibex boot address (reset vector = boot_addr_i + 0x80)

  // private bus master (towards ng_cp_port)
  output logic        d_req,
  input  logic        d_gnt,
  input  logic        d_rvalid,
  output logic        d_we,
  output logic [3:0]  d_be,
  output logic [31:0] d_addr,
  output logic [31:0] d_wdata,
  input  logic [31:0] d_rdata,
  input  logic        d_err,

  output logic        quiet_o,         // sim: firmware idle flag (DMEM[QUIET_WORD][0])
  output logic        sleep_o          // core in wfi
);
  localparam int unsigned IAW = $clog2(IMEM_WORDS);
  localparam int unsigned DAW = $clog2(DMEM_WORDS);

  // ---------------------------------------------------------------------------
  // memories
  logic [31:0] imem [IMEM_WORDS];
  logic [31:0] dmem [DMEM_WORDS];

`ifndef SYNTHESIS
  initial begin
    for (int i = 0; i < IMEM_WORDS; i++) imem[i] = 32'h0000_0013;   // nop
    for (int i = 0; i < DMEM_WORDS; i++) dmem[i] = 32'h0;
    if (IMAGE_FILE != "") $readmemh(IMAGE_FILE, imem);
  end
`endif

  always_ff @(posedge clk)
    if (ld_we_i && !ld_dmem_i && ld_addr_i[31:2+IAW] == '0) imem[ld_addr_i[2+:IAW]] <= ld_data_i;

  // ---------------------------------------------------------------------------
  // Ibex
  logic        i_req, i_gnt, i_rvalid, i_err;
  logic [31:0] i_addr, i_rdata;
  logic        c_req, c_we;
  logic [3:0]  c_be;
  logic [31:0] c_addr, c_wdata;
  logic        c_gnt, c_rvalid, c_err;
  logic [31:0] c_rdata;
  logic        core_sleep;

  ibex_top #(
    .PMPEnable       (1'b0),
    .MHPMCounterNum  (0),
    .RV32E           (1'b0),
    .RV32M           (RV32M),
    .RV32B           (ibex_pkg::RV32BNone),
    .RegFile         (ibex_pkg::RegFileFF),
    .BranchTargetALU (1'b0),
    .WritebackStage  (1'b0),
    .ICache          (1'b0),
    .ICacheECC       (1'b0),
    .BranchPredictor (1'b0),
    .DbgTriggerEn    (1'b0),
    .SecureIbex      (1'b0)
  ) u_ibex (
    .clk_i (clk), .rst_ni (rst_n), .test_en_i (1'b0), .scan_rst_ni (1'b1), .ram_cfg_i ('0),
    .hart_id_i (32'h0), .boot_addr_i (boot_addr_i),
    .instr_req_o (i_req), .instr_gnt_i (i_gnt), .instr_rvalid_i (i_rvalid),
    .instr_addr_o (i_addr), .instr_rdata_i (i_rdata), .instr_rdata_intg_i (7'h0),
    .instr_err_i (i_err),
    .data_req_o (c_req), .data_gnt_i (c_gnt), .data_rvalid_i (c_rvalid),
    .data_we_o (c_we), .data_be_o (c_be), .data_addr_o (c_addr), .data_wdata_o (c_wdata),
    .data_wdata_intg_o (), .data_rdata_i (c_rdata), .data_rdata_intg_i (7'h0),
    .data_err_i (c_err),
    .irq_software_i (1'b0), .irq_timer_i (timer_irq_i), .irq_external_i (irq_i),
    .irq_fast_i (15'h0), .irq_nm_i (1'b0),
    .scramble_key_valid_i (1'b0), .scramble_key_i ('0), .scramble_nonce_i ('0),
    .scramble_req_o (),
    .debug_req_i (1'b0), .crash_dump_o (), .double_fault_seen_o (),
    .fetch_enable_i (ibex_pkg::IbexMuBiOn),
    .alert_minor_o (), .alert_major_internal_o (), .alert_major_bus_o (),
    .core_sleep_o (core_sleep)
  );
  assign sleep_o = core_sleep;

  // ---------------------------------------------------------------------------
  // instruction side: IMEM, 1-cycle read, errors outside the array
  wire i_in = (i_addr[31:2+IAW] == '0);
  assign i_gnt = i_req;
  logic [31:0] imem_rd_i = '0;
  logic        i_in_q = 1'b1;
  always_ff @(posedge clk) if (i_req) imem_rd_i <= imem[i_addr[2+:IAW]];
  assign i_rdata = i_in_q ? imem_rd_i : 32'h0;
  assign i_err   = ~i_in_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      i_rvalid <= 1'b0; i_in_q <= 1'b1;
    end else begin
      i_rvalid <= i_req;
      if (i_req) i_in_q <= i_in;
    end
  end

  // ---------------------------------------------------------------------------
  // data side: IMEM (read-only), DMEM, private bus, error
  wire sel_imem = (c_addr[31:2+IAW] == '0);
  wire sel_dmem = (c_addr[31:2+DAW] == DMEM_BASE[31:2+DAW]);
  wire sel_ext  = (c_addr[31:30] == 2'b01);

  logic        busy, tgt_ext, loc_rvalid, loc_err, loc_imem;
  logic [31:0] loc_rdata;
  logic [31:0] imem_rd_d = '0, dmem_rd = '0;

  // memory arrays: clock-only blocks (block-RAM friendly).  DMEM has a single port; the boot loader takes it over
  // (whole words) while the core is held in reset.
  wire                dm_ld = ld_we_i & ld_dmem_i & (ld_addr_i[31:2+DAW] == '0);
  wire [DAW-1:0]      dm_a  = dm_ld ? ld_addr_i[2+:DAW] : c_addr[2+:DAW];
  wire [31:0]         dm_wd = dm_ld ? ld_data_i : c_wdata;
  wire [3:0]          dm_be = dm_ld ? 4'hF : c_be;
  wire                dm_we = dm_ld | (c_gnt & sel_dmem & c_we);
  always_ff @(posedge clk) begin
    if (c_gnt && sel_imem) imem_rd_d <= imem[c_addr[2+:IAW]];
    if (dm_we) begin
      for (int b = 0; b < 4; b++)
        if (dm_be[b]) dmem[dm_a][8*b+:8] <= dm_wd[8*b+:8];
    end else if (c_gnt && sel_dmem) begin
      dmem_rd <= dmem[dm_a];
    end
  end
  assign loc_rdata = loc_imem ? imem_rd_d : dmem_rd;

  assign c_gnt = c_req & ~busy;
  assign d_req   = c_gnt & sel_ext;
  assign d_we    = c_we;
  assign d_be    = c_be;
  assign d_addr  = c_addr;
  assign d_wdata = c_wdata;

  assign c_rvalid = tgt_ext ? d_rvalid : loc_rvalid;
  assign c_rdata  = tgt_ext ? d_rdata  : loc_rdata;
  assign c_err    = tgt_ext ? d_err    : loc_err;

  logic unused_gnt;
  assign unused_gnt = d_gnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy <= 1'b0; tgt_ext <= 1'b0; loc_rvalid <= 1'b0; loc_err <= 1'b0; loc_imem <= 1'b0;
    end else begin
      loc_rvalid <= 1'b0;
      if (c_gnt) begin
        busy    <= 1'b1;
        tgt_ext <= sel_ext;
        loc_imem <= sel_imem;
        if (sel_imem) begin
          loc_rvalid <= 1'b1;
          loc_err    <= c_we;                           // IMEM is read-only
        end else if (sel_dmem) begin
          loc_rvalid <= 1'b1;
          loc_err    <= 1'b0;
        end else if (!sel_ext) begin
          loc_rvalid <= 1'b1;
          loc_err    <= 1'b1;                           // unmapped (data reads as DMEM, ignored)
        end
      end
      if (c_rvalid) busy <= 1'b0;
    end
  end

`ifndef SYNTHESIS
  assign quiet_o = dmem[QUIET_WORD][0] & ~irq_i;
`else
  assign quiet_o = 1'b0;
`endif
endmodule
