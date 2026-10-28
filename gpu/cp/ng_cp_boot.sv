// ng_cp_boot.sv -- CP boot loader: the hardware "boot ROM"  (steps 3-4).
//
// When CP_CTRL.CORE_RESET is released (run_i rises) the loader reads the NGCF image the kernel wrote to C3
// (CP_FW_BASE/CP_FW_SIZE) through the CP memory port (VMID 0 = physical), checks the 64-byte header, streams the
// body once (computing the IEEE CRC-32 over [64, image_size) and writing the text segment into IMEM and the data
// segment into DMEM on the fly), compares the CRC, clears the DMEM bss and only then lets the Ibex run
// (ok_o, boot_addr_o).  On the first problem it stops with BOOT_ERR = err_o (and a one-cycle err_vld_o); the Ibex
// then stays in reset and the kernel sees BOOT_ERR != 0 / no READY.  A hardware FSM instead of an Ibex ROM: the
// boot step has no firmware to trust, the image is checked before a single instruction of it runs, and it is
// testable bit by bit (tb_cp_boot.sv).
//   * text -> IMEM[0 ..), data -> DMEM[0 ..) then bss (zeroed) after it; text_offset/data_offset/sizes are
//     4-byte aligned;
//   * entry is an IMEM byte offset with entry[7:0] == 0x80 (the Ibex reset vector is boot_addr + 0x80;
//     boot_addr_o = entry - 0x80), so the vector table sits at the 256-byte boundary below the entry;
//   * err 1 magic, 2 hdr_version / flags / reserved, 3 sizes / segments / entry / does not fit, 4 CRC, 5 read
//     fault, alignment of CP_FW_BASE, or no memory response (2^20 cycles).
// The command-stream version is not checked here (the driver does it, xim_ngcf_check, and step 5).
module ng_cp_boot
  import ng_cp_pkg::*;
#(
  parameter int unsigned IMEM_WORDS = 16384,
  parameter int unsigned DMEM_WORDS = 8192
) (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        run_i,           // ~CORE_RESET: boot starts on the rising edge, everything aborts when it falls

  input  logic [63:0] fw_base_i,       // CP_FW_BASE
  input  logic [31:0] fw_size_i,       // CP_FW_SIZE

  // CP memory port (master while busy_o)
  output logic        m_req_valid_o,
  input  logic        m_req_ready_i,
  output mem_req_t    m_req_o,
  input  logic        m_rsp_valid_i,
  input  logic        m_rsp_fault_i,
  input  logic [63:0] m_rsp_data_i,

  // IMEM / DMEM load port (one word per cycle)
  output logic        ld_we_o,
  output logic        ld_dmem_o,
  output logic [31:0] ld_addr_o,       // byte address inside the memory
  output logic [31:0] ld_data_o,

  output logic [31:0] boot_addr_o,     // Ibex boot_addr_i, valid with ok_o
  output logic        busy_o,          // loader owns the memory port
  output logic        ok_o,            // image loaded: release the Ibex
  output logic        err_vld_o,       // one cycle when BOOT_ERR is decided
  output logic [7:0]  err_o            // BOOT_ERR (valid with err_vld_o and after)
);
  localparam logic [31:0] IMEM_BYTES = 32'(IMEM_WORDS) * 4;
  localparam logic [31:0] DMEM_BYTES = 32'(DMEM_WORDS) * 4;
  localparam logic [31:0] NGCF_MAGIC        = 32'h4643_474E;  // "NGCF"
  localparam logic [31:0] NGCF_MAGIC_LEGACY = 32'h4643_4758;  // "XGCF", accepted for one version (images packed before b20)
  localparam logic [31:0] HDR_BYTES  = 32'd64;

  typedef enum logic [3:0] {
    S_IDLE, S_HREQ, S_HRSP, S_HCHK, S_BREQ, S_BRSP, S_BW0, S_BW1, S_CRC, S_BSS, S_DONE, S_ERR
  } state_t;
  state_t st;

  logic [63:0] base_q;
  logic [31:0] size_q;
  logic [31:0] hdr [16];
  logic [3:0]  hq;                         // header qword index 0..7
  logic [31:0] pos;                        // byte offset (inside the image) of the next body qword
  logic [63:0] qdata;
  logic [31:0] crc;
  logic        pend;                       // a memory request is outstanding (its response is still owed)
  logic [19:0] tmo;
  logic        run_q;

  // decoded header
  logic [31:0] text_off, text_size, data_off, data_size, bss_size, entry, crc_exp;
  logic [31:0] bss_ptr, bss_end;

  function automatic logic [31:0] crc32_w(input logic [31:0] c, input logic [31:0] w);
    logic [31:0] r;
    r = c ^ w;
    for (int i = 0; i < 32; i++) r = r[0] ? ((r >> 1) ^ 32'hEDB8_8320) : (r >> 1);
    return r;
  endfunction

  // segment check, 33-bit arithmetic so nothing wraps
  function automatic logic seg_ok(input logic [31:0] off, input logic [31:0] sz, input logic [31:0] img);
    return (off[1:0] == 2'b00) && (sz[1:0] == 2'b00) && (off >= HDR_BYTES) && (off <= img) && (sz <= img - off);
  endfunction

  // header check, result = error code (0 = fine); uses the raw header words
  function automatic logic [7:0] hdr_check();
    logic [31:0] ts, td, dd, ds, bs, en, im;
    logic        has_data;
    logic [32:0] need;
    // layout: [6]=text_off [7]=text_size [8]=data_off [9]=data_size [10]=bss_size [11]=entry [12]=crc
    ts = hdr[6]; td = hdr[7]; dd = hdr[8]; ds = hdr[9]; bs = hdr[10]; en = hdr[11]; im = hdr[5];
    has_data = (ds != 32'h0);
    if (hdr[0] != NGCF_MAGIC && hdr[0] != NGCF_MAGIC_LEGACY) return 8'd1;
    if (hdr[1] != 32'd1 || hdr[4] != 32'h0 || hdr[13] != 32'h0 || hdr[14] != 32'h0 || hdr[15] != 32'h0) return 8'd2;
    if (im != size_q) return 8'd3;
    if (td == 32'h0 || !seg_ok(ts, td, im)) return 8'd3;
    if (has_data && !seg_ok(dd, ds, im)) return 8'd3;
    if (has_data && ts < dd + ds && dd < ts + td) return 8'd3;       // overlapping segments (no wrap: seg_ok'd)
    if (en >= td || en[7:0] != 8'h80) return 8'd3;
    if (td > IMEM_BYTES) return 8'd3;
    need = {1'b0, ds} + {1'b0, bs[31:2], 2'b00} + ((bs[1:0] != 2'b00) ? 33'd4 : 33'd0);
    if (need > {1'b0, DMEM_BYTES}) return 8'd3;
    return 8'd0;
  endfunction

  assign m_req_o.write = 1'b0;
  assign m_req_o.vmid  = 4'd0;
  assign m_req_o.wdata = 64'h0;
  assign busy_o        = (st != S_IDLE) && (st != S_DONE) && (st != S_ERR);
  assign ok_o          = (st == S_DONE);

  // body qword: which bytes are inside the image
  wire         tail4    = (size_q - pos) == 32'd4;

  always_comb begin
    m_req_valid_o = 1'b0;
    m_req_o.addr  = '0;
    m_req_o.wstrb = 8'hFF;
    unique case (st)
      S_HREQ: begin
        m_req_valid_o = 1'b1;
        m_req_o.addr  = base_q[47:0] + 48'(hq) * 48'd8;
      end
      S_BREQ: begin
        m_req_valid_o = 1'b1;
        m_req_o.addr  = base_q[47:0] + 48'(pos);
        m_req_o.wstrb = tail4 ? 8'h0F : 8'hFF;
      end
      default: ;
    endcase
  end

  // load port
  always_comb begin
    ld_we_o   = 1'b0;
    ld_dmem_o = 1'b0;
    ld_addr_o = '0;
    ld_data_o = '0;
    unique case (st)
      S_BW0, S_BW1: begin
        logic [31:0] p, w;
        p = (st == S_BW0) ? pos : pos + 32'd4;
        w = (st == S_BW0) ? qdata[31:0] : qdata[63:32];
        ld_data_o = w;
        if (p >= text_off && p - text_off < text_size) begin
          ld_we_o = 1'b1; ld_dmem_o = 1'b0; ld_addr_o = p - text_off;
        end else if (data_size != 32'h0 && p >= data_off && p - data_off < data_size) begin
          ld_we_o = 1'b1; ld_dmem_o = 1'b1; ld_addr_o = p - data_off;
        end
      end
      S_BSS: begin
        ld_we_o = (bss_ptr < bss_end); ld_dmem_o = 1'b1; ld_addr_o = bss_ptr; ld_data_o = 32'h0;
      end
      default: ;
    endcase
  end

  assign boot_addr_o = entry - 32'h80;

  logic [7:0] err_q;
  assign err_o = err_q;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= S_IDLE; base_q <= '0; size_q <= '0; hq <= '0; pos <= '0; qdata <= '0; crc <= '0;
      pend <= 1'b0; tmo <= '0; run_q <= 1'b0; err_q <= 8'h0; err_vld_o <= 1'b0;
      text_off <= '0; text_size <= '0; data_off <= '0; data_size <= '0; bss_size <= '0; entry <= '0;
      crc_exp <= '0; bss_ptr <= '0; bss_end <= '0;
      for (int i = 0; i < 16; i++) hdr[i] <= '0;
    end else begin
      run_q <= run_i;
      err_vld_o <= 1'b0;
      if (m_rsp_valid_i) pend <= 1'b0;
      if (pend) tmo <= tmo + 20'd1;

      if (!run_i) begin
        // CORE_RESET: abort anything, forget the result; a response still owed stays "pend" until it arrives
        st <= S_IDLE; err_q <= 8'h0;
      end else begin
        unique case (st)
          S_IDLE: if (!run_q && !pend) begin       // rising edge of run (and no stale response in flight)
            base_q <= fw_base_i; size_q <= fw_size_i; hq <= '0; err_q <= 8'h0;
            if (fw_size_i < HDR_BYTES || fw_size_i[1:0] != 2'b00 || fw_base_i[2:0] != 3'b000 || (|fw_base_i[63:48])) begin
              st <= S_ERR; err_q <= 8'd3; err_vld_o <= 1'b1;
            end else begin
              st <= S_HREQ;
            end
          end
          S_HREQ: if (m_req_ready_i) begin pend <= 1'b1; tmo <= '0; st <= S_HRSP; end
          S_HRSP: begin
            if (m_rsp_valid_i) begin
              if (m_rsp_fault_i) begin st <= S_ERR; err_q <= 8'd5; err_vld_o <= 1'b1; end
              else begin
                hdr[{hq[2:0], 1'b0}] <= m_rsp_data_i[31:0];
                hdr[{hq[2:0], 1'b1}] <= m_rsp_data_i[63:32];
                if (hq == 4'd7) st <= S_HCHK;
                else begin hq <= hq + 4'd1; st <= S_HREQ; end
              end
            end else if (&tmo) begin st <= S_ERR; err_q <= 8'd5; err_vld_o <= 1'b1; pend <= 1'b0; end
          end
          S_HCHK: begin
            err_q <= hdr_check();
            if (hdr_check() != 8'd0) begin st <= S_ERR; err_vld_o <= 1'b1; end
            else begin
              text_off <= hdr[6]; text_size <= hdr[7]; data_off <= hdr[8]; data_size <= hdr[9];
              bss_size <= hdr[10]; entry <= hdr[11]; crc_exp <= hdr[12];
              crc <= 32'hFFFF_FFFF; pos <= HDR_BYTES;
              st <= (size_q > HDR_BYTES) ? S_BREQ : S_CRC;
            end
          end
          S_BREQ: if (m_req_ready_i) begin pend <= 1'b1; tmo <= '0; st <= S_BRSP; end
          S_BRSP: begin
            if (m_rsp_valid_i) begin
              if (m_rsp_fault_i) begin st <= S_ERR; err_q <= 8'd5; err_vld_o <= 1'b1; end
              else begin qdata <= m_rsp_data_i; st <= S_BW0; end
            end else if (&tmo) begin st <= S_ERR; err_q <= 8'd5; err_vld_o <= 1'b1; pend <= 1'b0; end
          end
          S_BW0: begin
            crc <= crc32_w(crc, qdata[31:0]);
            st  <= (size_q - pos > 32'd4) ? S_BW1 : S_CRC;
          end
          S_BW1: begin
            crc <= crc32_w(crc, qdata[63:32]);
            pos <= pos + 32'd8;
            st  <= (pos + 32'd8 < size_q) ? S_BREQ : S_CRC;
          end
          S_CRC: begin
            if ((crc ^ 32'hFFFF_FFFF) != crc_exp) begin st <= S_ERR; err_q <= 8'd4; err_vld_o <= 1'b1; end
            else begin
              bss_ptr <= data_size;
              bss_end <= data_size + {bss_size[31:2], 2'b00} + ((bss_size[1:0] != 2'b00) ? 32'd4 : 32'd0);
              st <= S_BSS;
            end
          end
          S_BSS: begin
            if (bss_ptr >= bss_end) st <= S_DONE;
            else bss_ptr <= bss_ptr + 32'd4;
          end
          default: ;     // S_DONE / S_ERR: wait for CORE_RESET
        endcase
      end
    end
  end
endmodule
