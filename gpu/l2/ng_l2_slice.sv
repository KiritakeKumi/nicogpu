// ng_l2_slice.sv -- single L2 slice (URAM-structured, write-back /
// write-allocate, set-associative, with the AMO ALU inline).
//
// One slice fronts exactly one DDR channel half:
//   slice 0 -> C1 even channel granules, slice 1 -> C1 odd, 2 -> C2, 3 -> C4
// (ng_l2_addrmap; golden definition).  The slice is
// the coherence point of the GPU (design doc): every VRAM access of the
// requestors goes through a slice, atomics execute here.
//
// v1 engine is deliberately serial (one request in flight, input via a
// valid/ready stream from the NoC endpoint adapter) -- see tentative notes
// in (known gap: hit-under-miss / multiple MSHRs later).
//
// DDR side is an AXI4 master following the DFX boundary rules
// (): INCR only, AxSIZE = port width,
// bursts never cross 4 KiB, address is the *channel* offset (interleave is
// applied here via ng_l2_addrmap, not in the shell).
//
// Semantics tentative:
//   * line 128 B; the 4 KiB interleave granule >= line, so a line never
//     straddles two slices and all accesses to one PA serialize at one slice;
//   * atomics are read-modify-write on the cached line, linearized at the
//     data-array write (the engine is serial, nothing can interleave);
//   * unaligned accesses are byte-precise inside a line; accesses crossing a
//     line boundary are rejected with ERR_CROSS (requestor splits them);
//   * requests must be VRAM PAs (SYS/HOST traffic never reaches an L2 slice).
module ng_l2_slice #(
  parameter int SETS      = 16,    // sets (power of two; full config 1024)
  parameter int WAYS      = 4,     // associativity (full config 8)
  parameter int LINE_BYTES = 128,  // line size (power of two, <= 4096)
  parameter int AXI_AW    = 33,    // C1: 33 (8 GiB), C2/C4: 32
  parameter int AXI_DW    = 512,   // C1: 512, C2/C4: 256
  parameter int AXI_IW    = 6,     // DFX ID width
  parameter int AXI_ID    = 0,     // fixed ID driven on AR/AW
  parameter int SLICE_ID  = 0,     // echoed as resp.src (NoC node id)
  parameter logic [5:0] SRC_ID = 6'(SLICE_ID)
) (
  input  logic clk,
  input  logic rst_n,

  // ---- NoC side: request stream in, response stream out (credit adaptation
  // ---- and FIFOing are done by ng_noc_ep; this is a plain stream).
  input  logic                    s_req_valid,
  output logic                    s_req_ready,
  input  ng_noc_pkg::flit_t       s_req_flit,
  output logic                    m_resp_valid,
  input  logic                    m_resp_ready,
  output ng_noc_pkg::flit_t       m_resp_flit,

  // ---- AXI4 master to the slice's DDR channel (all in clk domain).
  output logic                    arvalid,
  input  logic                    arready,
  output logic [AXI_AW-1:0]       araddr,
  output logic [7:0]              arlen,
  output logic [2:0]              arsize,
  output logic [1:0]              arburst,
  output logic [AXI_IW-1:0]       arid,
  input  logic                    rvalid,
  output logic                    rready,
  input  logic [AXI_DW-1:0]       rdata,
  input  logic                    rlast,
  input  logic [AXI_IW-1:0]       rid,
  input  logic [1:0]              rresp,
  output logic                    awvalid,
  input  logic                    awready,
  output logic [AXI_AW-1:0]       awaddr,
  output logic [7:0]              awlen,
  output logic [2:0]              awsize,
  output logic [1:0]              awburst,
  output logic [AXI_IW-1:0]       awid,
  output logic                    wvalid,
  input  logic                    wready,
  output logic [AXI_DW-1:0]       wdata,
  output logic [AXI_DW/8-1:0]     wstrb,
  output logic                    wlast,
  input  logic                    bvalid,
  output logic                    bready,
  input  logic [AXI_IW-1:0]       bid,
  input  logic [1:0]              bresp
);

  import ng_noc_pkg::*;
  import ng_l2_addrmap::*;

  localparam int LINE_LB     = $clog2(LINE_BYTES);
  localparam int SET_LB      = $clog2(SETS);
  localparam int LINE_BITS   = LINE_BYTES * 8;
  localparam int LINE_ADDR_W = 48 - LINE_LB;
  localparam int TAG_W       = LINE_ADDR_W - SET_LB;
  localparam int TAG_RAW     = TAG_W + 1;                 // {dirty, tag}
  localparam int TAG_RAM_W   = ((TAG_RAW + 7) / 8) * 8;
  localparam int AXI_BYTES   = AXI_DW / 8;
  localparam int BEATS       = LINE_BYTES / AXI_BYTES;    // 2 @512b, 4 @256b
  localparam int BEAT_LB     = $clog2(BEATS);
  localparam int WAY_LB      = clog2_1(WAYS);

  // ---------------- helpers ------------------------------------------------
  function automatic logic [7:0] size_be8(input logic [1:0] size);
    return (size == SZ_8) ? 8'hFF : (size == SZ_4) ? 8'h0F :
           (size == SZ_2) ? 8'h03 : 8'h01;
  endfunction

  function automatic logic [63:0] line_extract(input logic [LINE_BITS-1:0] line,
                                               input logic [LINE_LB-1:0] off,
                                               input logic [1:0] size);
    logic [63:0] v;
    logic [63:0] nbytes;
    v = '0;
    nbytes = 64'd1 << size;
    for (int b = 0; b < 8; b++) begin
      if (b < nbytes) v[8*b +: 8] = line[8*(off + b) +: 8];
    end
    return v;
  endfunction

  function automatic logic [LINE_BITS-1:0] line_merge(input logic [LINE_BITS-1:0] line,
                                                      input logic [LINE_LB-1:0] off,
                                                      input logic [7:0] be8,
                                                      input logic [63:0] data);
    logic [LINE_BITS-1:0] l;
    l = line;
    for (int b = 0; b < 8; b++) begin
      if (be8[b]) l[8*(off + b) +: 8] = data[8*b +: 8];
    end
    return l;
  endfunction

  function automatic logic [TAG_RAM_W-1:0] tag_word(input logic dirty,
                                                     input logic [TAG_W-1:0] tag);
    logic [TAG_RAM_W-1:0] w;
    w = '0;
    w[TAG_W-1:0] = tag;
    w[TAG_W]      = dirty;
    return w;
  endfunction

  function automatic logic amo_op_valid(input logic [4:0] op, input logic [1:0] size);
    if (size < SZ_4) return 1'b0;                     // ISA: 32/64-bit only
    if (size == SZ_8 && (op == AMO_INC || op == AMO_DEC)) return 1'b0;
    if (op == AMO_FADD || op == AMO_FMIN || op == AMO_FMAX)
      return (size == SZ_4);                          // FP32 only
    return (op <= AMO_DEC);                           // AMO_NONE (=31) invalid
  endfunction

  function automatic logic [1:0] req_err(input flit_t f);
    if (f.kind == KIND_MAINT)                         // addr/size ignored
      return (f.op == MAINT_INV_L2 || f.op == MAINT_WB_L2) ? ERR_NONE : ERR_BAD_PA;
    if (!l2_pa_is_vram(f.addr)) return ERR_BAD_PA;
    if ((64'(f.addr[LINE_LB-1:0]) + (64'd1 << f.size)) > 64'(LINE_BYTES))
      return ERR_CROSS;
    if (f.kind == KIND_AMO && !amo_op_valid(f.op, f.size)) return ERR_BAD_PA;
    return ERR_NONE;
  endfunction

  // ---------------- state --------------------------------------------------
  typedef enum logic [3:0] {
    S_IDLE, S_ACT, S_WB_AW, S_WB_W, S_WB_B, S_AR, S_FILL, S_FILL_W, S_RESP,
    // cache maintenance sweep (per set, per way)
    S_M_NEXT, S_M_RD, S_M_CHK, S_M_CLR, S_M_ADV
  } state_e;
  state_e state;

  flit_t              cur;
  logic [SET_LB-1:0]  cur_set;
  logic [TAG_W-1:0]   cur_tag;
  logic [LINE_LB-1:0] cur_off;

  logic               valid_ff [WAYS][SETS];
  logic [WAY_LB-1:0]  rr_ptr [SETS];

  // tag rams: word = {dirty, tag}; valid is a reset FF array.  The lookup
  // read is issued combinationally in S_IDLE so that tag_q/line_q are valid
  // in S_ACT (one registered read latency).
  logic                    tag_re;
  logic [SET_LB-1:0]       tag_raddr;
  logic                    tag_we [WAYS];
  logic [SET_LB-1:0]       tag_waddr;
  logic [TAG_RAM_W-1:0]    tag_wdata;
  logic [TAG_RAM_W-1:0]    tag_q [WAYS];

  // data rams: whole-line width, byte enables
  logic                    dat_re;
  logic [SET_LB-1:0]       dat_raddr;
  logic                    dat_we [WAYS];
  logic [SET_LB-1:0]       dat_waddr;
  logic [LINE_BITS-1:0]    dat_wdata;
  logic [LINE_BITS-1:0]    line_q [WAYS];

  logic [LINE_BITS-1:0]    wb_buf, fill_buf;
  logic [LINE_ADDR_W-1:0]  wb_line_addr;
  logic [WAY_LB-1:0]       victim_way, hit_way;
  logic                    hit_any, victim_dirty;
  logic [BEAT_LB:0]        beat;
  logic                    ddr_err;
  flit_t                   resp;

  // maintenance sweep state
  logic                    maint_active;
  logic [SET_LB-1:0]       m_set;
  logic [WAY_LB-1:0]       m_way;
  logic [TAG_W-1:0]        m_tag;
  logic                    m_way_last, m_set_last, m_is_inv;
  assign m_way_last = (m_way == WAY_LB'(WAYS-1));
  assign m_set_last = (m_set == SET_LB'(SETS-1));
  assign m_is_inv   = (cur.op == MAINT_INV_L2);

  // combinational operands (hit path reads line_q, fill path reads fill_buf)
  logic [63:0]             extract_val;
  logic [LINE_BITS-1:0]    merged_line, amo_line;

  // ---------------- per-way arrays -----------------------------------------
  for (genvar w = 0; w < WAYS; w++) begin : g_way
    ng_prim_l2ram #(
      .D_W    (TAG_RAM_W),
      .DEPTH  (SETS),
      .BE_W   (TAG_RAM_W / 8)
    ) u_tag (
      .clk     (clk),
      .we      (tag_we[w]),
      .wbe     ('1),
      .waddr   (tag_waddr),
      .wdata   (tag_wdata),
      .re      (tag_re),
      .raddr   (tag_raddr),
      .rdata_q (tag_q[w])
    );
    ng_prim_l2ram #(
      .D_W    (LINE_BITS),
      .DEPTH  (SETS),
      .BE_W   (LINE_BYTES)
    ) u_data (
      .clk     (clk),
      .we      (dat_we[w]),
      .wbe     ('1),
      .waddr   (dat_waddr),
      .wdata   (dat_wdata),
      .re      (dat_re),
      .raddr   (dat_raddr),
      .rdata_q (line_q[w])
    );
  end

  // ---------------- decode of current request ------------------------------
  always_comb begin
    cur_set = cur.addr[LINE_LB +: SET_LB];
    cur_tag = cur.addr[LINE_LB + SET_LB +: TAG_W];
    cur_off = cur.addr[LINE_LB-1:0];
  end

  // hit / victim selection
  always_comb begin
    hit_way = '0;
    hit_any = 1'b0;
    for (int w = WAYS-1; w >= 0; w--) begin
      if (valid_ff[w][cur_set] && (tag_q[w][TAG_W-1:0] == cur_tag)) begin
        hit_way = WAY_LB'(w);
        hit_any = 1'b1;
      end
    end
    victim_way = rr_ptr[cur_set];
    for (int w = WAYS-1; w >= 0; w--) begin
      if (!valid_ff[w][cur_set]) victim_way = WAY_LB'(w);  // first invalid wins
    end
    victim_dirty = valid_ff[victim_way][cur_set] && tag_q[victim_way][TAG_W];
  end

  // value / merge operands against the active line
  logic [LINE_BITS-1:0] line_now;
  logic [7:0]           beff;
  assign line_now = (state == S_ACT) ? line_q[hit_way] : fill_buf;
  assign beff     = (cur.kind == KIND_WR) ? cur.be :
                    (cur.kind == KIND_AMO) ? size_be8(cur.size) : 8'h00;
  assign extract_val = line_extract(line_now, cur_off, cur.size);
  assign merged_line = line_merge(line_now, cur_off, beff, cur.data);

  logic [63:0] amo_res;
  logic        amo_cas_hit;
  ng_l2_amo_alu u_alu (
    .op      (cur.op),
    .is64    (cur.size == SZ_8),
    .a       (extract_val),
    .b       (cur.data),
    .cmp     (cur.cmp),
    .r       (amo_res),
    .cas_hit (amo_cas_hit)
  );
  // WR merge is identity for AMO (be8 unused there); AMO merges the ALU result
  assign amo_line = (cur.kind == KIND_AMO)
                    ? line_merge(line_now, cur_off, size_be8(cur.size), amo_res)
                    : merged_line;

  // ---------------- main FSM ------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state    <= S_IDLE;
      tag_we   <= '{default: 1'b0};
      dat_we   <= '{default: 1'b0};
      arvalid  <= 1'b0;
      awvalid  <= 1'b0;
      wvalid   <= 1'b0;
      rready   <= 1'b0;
      bready   <= 1'b0;
      maint_active <= 1'b0;
      for (int s = 0; s < SETS; s++) rr_ptr[s] <= '0;
      for (int w = 0; w < WAYS; w++)
        for (int s = 0; s < SETS; s++) valid_ff[w][s] <= 1'b0;
    end else begin
      // default one-cycle pulses
      tag_we <= '{default: 1'b0};
      dat_we <= '{default: 1'b0};
      rready <= 1'b0;
      bready <= 1'b0;

      unique case (state)
        S_IDLE: begin
          if (s_req_valid) begin
            cur       <= s_req_flit;
            ddr_err   <= 1'b0;
            state     <= S_ACT;
          end
        end

        S_ACT: begin
          if (req_err(cur) != ERR_NONE) begin
            resp  <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                       tag: cur.tag, err: req_err(cur), default: '0};
            state <= S_RESP;
          end else if (cur.kind == KIND_MAINT) begin
            // inv.l2 / wb.l2: whole-slice sweep (ISA has no address operand)
            maint_active <= 1'b1;
            m_set        <= '0;
            m_way        <= '0;
            state        <= S_M_NEXT;
          end else if (hit_any) begin
            dat_waddr <= cur_set;
            tag_waddr <= cur_set;
            if (cur.kind == KIND_RD) begin
              resp  <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                         tag: cur.tag, data: extract_val, default: '0};
            end else begin
              // store or AMO: read-modify-write of the whole line (URAM has
              // no byte enables; merged data keeps the untouched bytes)
              dat_we[hit_way] <= 1'b1;
              dat_wdata       <= (cur.kind == KIND_AMO) ? amo_line : merged_line;
              tag_we[hit_way] <= 1'b1;
              tag_wdata       <= tag_word(1'b1, cur_tag);
              resp            <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                                    tag: cur.tag,
                                    data: (cur.rtn && cur.kind == KIND_AMO)
                                          ? extract_val : 64'd0,
                                    default: '0};
            end
            state <= S_RESP;
          end else begin
            beat <= '0;
            if (victim_dirty) begin
              wb_buf       <= line_q[victim_way];
              wb_line_addr <= {tag_q[victim_way][TAG_W-1:0], cur_set};
              state        <= S_WB_AW;
            end else begin
              state        <= S_AR;
            end
          end
        end

        S_WB_AW: begin
          awvalid <= 1'b1;
          awaddr  <= AXI_AW'(l2_ch_of(wb_line_addr << LINE_LB).off);
          awlen   <= BEATS - 1;
          awsize  <= $clog2(AXI_BYTES);
          awburst <= 2'b01;  // INCR
          awid    <= AXI_IW'(AXI_ID);
          if (awvalid && awready) begin
            awvalid <= 1'b0;
            beat    <= '0;
            state   <= S_WB_W;
          end
        end

        S_WB_W: begin
          wvalid <= 1'b1;
          if (wvalid && wready) begin
            if (beat == BEATS-1) begin
              wvalid <= 1'b0;
              state  <= S_WB_B;
            end else begin
              beat <= beat + 1'b1;
            end
          end
        end

        S_WB_B: begin
          bready <= 1'b1;
          if (bvalid && bready) begin
            bready <= 1'b0;
            if (bresp != 2'b00) ddr_err <= 1'b1;
            beat  <= '0;
            state <= maint_active ? S_M_CLR : S_AR;
          end
        end

        // ---- maintenance sweep ------------------------------------
        // inv.l2: every valid line is written back if dirty, then invalidated
        //         (data-preserving: the model has no caches, memory is always
        //         current, so an invalidate must never lose data).
        // wb.l2: every dirty line is written back and left valid + clean.
        S_M_NEXT: begin
          state <= valid_ff[m_way][m_set] ? S_M_RD : S_M_ADV;
        end

        S_M_RD: begin
          state <= S_M_CHK;               // tag/data read issued (see tag_re)
        end

        S_M_CHK: begin
          m_tag <= tag_q[m_way][TAG_W-1:0];
          if (tag_q[m_way][TAG_W]) begin
            wb_buf       <= line_q[m_way];
            wb_line_addr <= {tag_q[m_way][TAG_W-1:0], m_set};
            state        <= S_WB_AW;      // returns to S_M_CLR after B
          end else begin
            state        <= S_M_CLR;
          end
        end

        S_M_CLR: begin
          tag_waddr        <= m_set;
          tag_we[m_way]    <= 1'b1;
          tag_wdata        <= tag_word(1'b0, m_tag);   // clean
          if (m_is_inv) valid_ff[m_way][m_set] <= 1'b0;
          state            <= S_M_ADV;
        end

        S_M_ADV: begin
          if (m_way_last && m_set_last) begin
            maint_active <= 1'b0;
            resp  <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                       tag: cur.tag, err: ddr_err ? ERR_DDR : ERR_NONE,
                       default: '0};
            state <= S_RESP;
          end else begin
            m_way <= m_way_last ? '0 : m_way + 1'b1;
            if (m_way_last) m_set <= m_set + 1'b1;
            state <= S_M_NEXT;
          end
        end

        S_AR: begin
          arvalid <= 1'b1;
          araddr  <= AXI_AW'(l2_ch_of(cur.addr & ~48'(LINE_BYTES - 1)).off);
          arlen   <= BEATS - 1;
          arsize  <= $clog2(AXI_BYTES);
          arburst <= 2'b01;
          arid    <= AXI_IW'(AXI_ID);
          if (arvalid && arready) begin
            arvalid <= 1'b0;
            beat    <= '0;
            state   <= S_FILL;
          end
        end

        S_FILL: begin
          rready <= 1'b1;
          if (rvalid && rready) begin
            fill_buf[beat*AXI_DW +: AXI_DW] <= rdata;
            if (rresp != 2'b00) ddr_err <= 1'b1;
            if (beat == BEATS-1) begin
              rready <= 1'b0;
              state  <= S_FILL_W;
            end else begin
              beat <= beat + 1'b1;
            end
          end
        end

        S_FILL_W: begin
          // allocate the victim way and apply the pending request
          dat_waddr <= cur_set;
          tag_waddr <= cur_set;
          dat_we[victim_way] <= 1'b1;
          tag_we[victim_way] <= 1'b1;
          rr_ptr[cur_set] <= (victim_way == WAY_LB'(WAYS-1)) ? '0 : victim_way + 1'b1;
          valid_ff[victim_way][cur_set] <= 1'b1;
          if (cur.kind == KIND_RD) begin
            dat_wdata <= fill_buf;
            tag_wdata <= tag_word(1'b0, cur_tag);
            resp      <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                           tag: cur.tag, data: extract_val, default: '0};
          end else if (cur.kind == KIND_WR) begin
            dat_wdata <= merged_line;
            tag_wdata <= tag_word(1'b1, cur_tag);
            resp      <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                           tag: cur.tag, default: '0};
          end else begin
            dat_wdata <= amo_line;
            tag_wdata <= tag_word(1'b1, cur_tag);
            resp      <= '{cls: CLS_RESP, dst: cur.src, src: SRC_ID,
                           tag: cur.tag,
                           data: cur.rtn ? extract_val : 64'd0,
                           default: '0};
          end
          if (ddr_err) resp.err <= ERR_DDR;
          state <= S_RESP;
        end

        S_RESP: begin
          if (m_resp_ready) state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

  assign s_req_ready  = (state == S_IDLE);
  assign m_resp_valid = (state == S_RESP);
  assign m_resp_flit  = resp;
  assign tag_re   = (s_req_valid && (state == S_IDLE)) || (state == S_M_RD);
  assign dat_re   = tag_re;
  assign tag_raddr = (state == S_M_RD) ? m_set
                                       : SET_LB'(s_req_flit.addr[LINE_LB +: SET_LB]);
  assign dat_raddr = tag_raddr;
  // write-back beat steering is combinational off the beat counter (a
  // registered version lags one beat and misplaces WLAST)
  assign wdata = wb_buf >> (beat * AXI_DW);
  assign wstrb = '1;
  assign wlast = (beat == BEATS-1);

  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (rst_n) begin
      if (state == S_FILL && rvalid && rready && rlast && (beat != BEATS-1))
        $error("ng_l2_slice: unexpected RLAST before last beat");
      if (state == S_WB_W && wvalid && wready && wlast && (beat != BEATS-1))
        $error("ng_l2_slice: unexpected WLAST before last beat");
      if (state == S_IDLE && s_req_valid && s_req_flit.kind == KIND_AMO &&
          s_req_flit.cls != CLS_REQ)
        $error("ng_l2_slice: AMO request with wrong class");
    end
  end
  // synthesis translate_on
endmodule
