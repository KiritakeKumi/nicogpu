// ng_mmu_pkg.sv -- GPU MMU shared definitions.
//
// Golden definitions: (bit layout, decode rules D1..D15,
// fault / access encodings) and (decode_entry).
// decode_entry below is a line-by-line port of pte.h::decode_entry; the
// TB checks it against the ptegen.py reference vectors (tb_mmu_decode).
package ng_mmu_pkg;

  // ---------------- fault codes (MMU_FAULT_STATUS[7:4], == nicogpu::Fault) ----
  localparam logic [2:0] F_NONE          = 3'd0;
  localparam logic [2:0] F_NOT_PRESENT   = 3'd1;
  localparam logic [2:0] F_NON_CANONICAL = 3'd2;
  localparam logic [2:0] F_PERMISSION    = 3'd3;
  localparam logic [2:0] F_BAD_PTE       = 3'd4;
  localparam logic [2:0] F_BAD_PHYS      = 3'd5;
  localparam logic [2:0] F_MISALIGNED    = 3'd6;

  // ---------------- access type (MMU_FAULT_STATUS[9:8]) --------------------
  localparam logic [1:0] ACC_R = 2'd0;
  localparam logic [1:0] ACC_W = 2'd1;   // atomics also report WRITE
  localparam logic [1:0] ACC_X = 2'd2;

  // ---------------- page size code (TLB entry / response) ------------------
  // shift = 12, 16, 21, 30, 39.  Codes 3/4 only occur for sparse-null spans
  // (1 GiB / 512 GiB leaves are reserved, rule D9).
  localparam logic [2:0] SZ_4K   = 3'd0;
  localparam logic [2:0] SZ_64K  = 3'd1;
  localparam logic [2:0] SZ_2M   = 3'd2;
  localparam logic [2:0] SZ_1G   = 3'd3;
  localparam logic [2:0] SZ_512G = 3'd4;

  localparam int NUM_VMID = 16;

  // SYS (C3) region: 0x0010_0000_0000, 2 GiB  => PA[47:31] == 17'h20.
  localparam logic [16:0] SYS_PA_HI = 17'h00020;

  function automatic logic table_pa_ok(input logic [47:0] pa);
    return pa[47:31] == SYS_PA_HI;
  endfunction

  // Decoded physical regions (MMU.md 2.2; model: mem::pa_decoded, LSU: ng_lsu_pkg::pa_valid):
  //   VRAM  0x0000_0000_0000 + 16 GiB   => PA[47:34] == 0
  //   SYS   0x0010_0000_0000 +  2 GiB   => PA[47:31] == 17'h20
  //   HOST  0x8000_0000_0000 and up     => PA[47]
  // the VMID 0 bypass answers BAD_PHYS_ADDR for PA = VA outside these.
  function automatic logic pa_decoded(input logic [47:0] pa);
    return (pa[47:34] == 14'd0) || (pa[47:31] == SYS_PA_HI) || pa[47];
  endfunction

  // Mask over VA[47:12] of the bits that index *inside* a page of size `sz`.
  function automatic logic [35:0] size_lowmask(input logic [2:0] sz);
    case (sz)
      SZ_4K:   return 36'h0_0000_0000;
      SZ_64K:  return 36'h0_0000_000F;
      SZ_2M:   return 36'h0_0000_01FF;
      SZ_1G:   return 36'h0_0003_FFFF;
      default: return 36'h0_07FF_FFFF;   // 512 GiB span: VA[38:12]
    endcase
  endfunction

  // PA of `va` inside the page whose base frame is `base_pfn`.
  function automatic logic [47:0] page_pa(input logic [35:0] base_pfn,
                                          input logic [47:0] va,
                                          input logic [2:0]  sz);
    logic [35:0] m;
    m = size_lowmask(sz);
    return {(base_pfn & ~m) | (va[47:12] & m), va[11:0]};
  endfunction

  // ---------------- entry decode (pte.h::decode_entry) ---------------------
  localparam logic [2:0] K_NOT_PRESENT = 3'd0;
  localparam logic [2:0] K_BAD_PTE     = 3'd1;
  localparam logic [2:0] K_SPARSE      = 3'd2;
  localparam logic [2:0] K_TABLE       = 3'd3;
  localparam logic [2:0] K_LEAF        = 3'd4;

  typedef struct packed {
    logic [2:0]  kind;
    logic [3:0]  rule;   // deciding rule D1..D15 (debug / test only)
    logic [2:0]  size;   // SZ_* (leaf: page size; sparse: span of the entry)
    logic [35:0] pfn;    // leaf: page base frame; table: next-level table frame
  } dec_t;

  function automatic dec_t decode_entry(input logic [63:0] e, input logic [1:0] level);
    dec_t d;
    logic v, leafb, sparse, f64k, rsvd;
    logic [35:0] pfn;
    v = e[0]; leafb = e[1]; f64k = e[7]; sparse = e[8];
    rsvd = (|e[11:9]) | (|e[55:48]);
    pfn = e[47:12];
    d = '{kind: K_NOT_PRESENT, rule: 4'd1, size: SZ_4K, pfn: '0};
    if (!v && !sparse) begin
      d.kind = K_NOT_PRESENT; d.rule = 4'd1;
    end else if (rsvd) begin
      d.kind = K_BAD_PTE; d.rule = 4'd2;
    end else if (sparse) begin
      // SPARSE_MBZ = V|LEAF|R|W|X|MTYPE|F64K (bits 7:0) | PA
      if ((|e[7:0]) || (|e[47:12])) begin
        d.kind = K_BAD_PTE; d.rule = 4'd3;
      end else begin
        d.kind = K_SPARSE; d.rule = 4'd4;
        d.size = (level == 2'd0) ? SZ_4K : (3'(level) + 3'd1);
      end
    end else if (level != 2'd0 && !leafb) begin
      if (|e[7:2]) begin                 // R W X MTYPE F64K must be 0 in a PDE
        d.kind = K_BAD_PTE; d.rule = 4'd5;
      end else if (!table_pa_ok({pfn, 12'b0})) begin
        d.kind = K_BAD_PTE; d.rule = 4'd6;
      end else begin
        d.kind = K_TABLE; d.rule = 4'd7; d.pfn = pfn;
      end
    end else if (level == 2'd0 && !leafb) begin
      d.kind = K_BAD_PTE; d.rule = 4'd8;
    end else if (level >= 2'd2) begin
      d.kind = K_BAD_PTE; d.rule = 4'd9;
    end else if (e[6:5] == 2'd3) begin
      d.kind = K_BAD_PTE; d.rule = 4'd10;
    end else if (level == 2'd1) begin
      if (f64k || (|pfn[8:0])) begin
        d.kind = K_BAD_PTE; d.rule = 4'd11;
      end else begin
        d.kind = K_LEAF; d.rule = 4'd12; d.size = SZ_2M; d.pfn = pfn;
      end
    end else if (f64k) begin
      if (|pfn[3:0]) begin
        d.kind = K_BAD_PTE; d.rule = 4'd13;
      end else begin
        d.kind = K_LEAF; d.rule = 4'd14; d.size = SZ_64K; d.pfn = pfn;
      end
    end else begin
      d.kind = K_LEAF; d.rule = 4'd15; d.size = SZ_4K; d.pfn = pfn;
    end
    return d;
  endfunction

  // level of the entry that produced a cached translation, from its size code
  function automatic logic [1:0] size_level(input logic [2:0] sz);
    case (sz)
      SZ_4K, SZ_64K: return 2'd0;
      SZ_2M:         return 2'd1;
      SZ_1G:         return 2'd2;
      default:       return 2'd3;
    endcase
  endfunction

endpackage : ng_mmu_pkg
