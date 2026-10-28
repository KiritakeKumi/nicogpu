// ng_mmu_walker.sv ---level page-table walker.
//
// One walk at a time.  Reads 64-bit entries from SYS (C3) through a simple
// request / response port (VMID-0 physical read path; not through L1/L2,
// MMU.md 7.2).  Per-entry behaviour is ng_mmu_pkg::decode_entry (D1..D15);
// the walk adds only the root check.  The walker never writes page tables.
//
// Result (one-cycle `done` pulse, fields valid in that cycle):
//   fault   F_NONE / F_NOT_PRESENT / F_BAD_PTE / F_BAD_PHYS (walker read error,
//           tentative extension: model has no failing table read)
//   level   level of the leaf / faulting entry (root outside SYS: 3)
//   pte     entry value at `level` (0 for root fault / read error)
//   sparse  sparse-null result
//   size    SZ_* of the leaf / sparse span
//   pfn     leaf page base frame
//
// `abort` returns the walker to idle at once (used by the top level when the
// VMID is invalidated mid-walk).  A read already issued is drained before the
// next start is accepted (`start_ready`), so a late response is never mistaken
// for the next walk's.
module ng_mmu_walker
  import ng_mmu_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  input  logic        start,          // accepted when start_ready
  output logic        start_ready,
  input  logic [47:0] start_va,
  input  logic [35:0] start_root_pfn,
  input  logic        abort,

  // table read port (single outstanding read)
  output logic        rd_valid,
  input  logic        rd_ready,
  output logic [47:0] rd_addr,
  input  logic        rsp_valid,
  input  logic [63:0] rsp_data,
  input  logic        rsp_err,

  // result
  output logic        done,
  output logic [2:0]  res_fault,
  output logic [1:0]  res_level,
  output logic [63:0] res_pte,
  output logic        res_sparse,
  output logic [2:0]  res_size,
  output logic [35:0] res_pfn
);
  typedef enum logic [2:0] {W_IDLE, W_REQ, W_WAIT, W_DONE} wstate_t;
  wstate_t       st;
  logic [47:0]   va_q;
  logic [35:0]   tbl_pfn;
  logic [1:0]    level;
  logic          outst;      // a read was accepted and its response is pending

  logic [2:0]    r_fault;
  logic [1:0]    r_level;
  logic [63:0]   r_pte;
  logic          r_sparse;
  logic [2:0]    r_size;
  logic [35:0]   r_pfn;

  // index of the entry for `level`: VA[12+9*level +: 9]
  logic [8:0] idx;
  always_comb begin
    case (level)
      2'd0:    idx = va_q[20:12];
      2'd1:    idx = va_q[29:21];
      2'd2:    idx = va_q[38:30];
      default: idx = va_q[47:39];
    endcase
  end

  assign start_ready = (st == W_IDLE) && !outst;
  assign rd_valid    = (st == W_REQ);
  assign rd_addr     = {tbl_pfn, 12'b0} + {36'b0, idx, 3'b000};

  assign done       = (st == W_DONE);
  assign res_fault  = r_fault;
  assign res_level  = r_level;
  assign res_pte    = r_pte;
  assign res_sparse = r_sparse;
  assign res_size   = r_size;
  assign res_pfn    = r_pfn;

  dec_t dec;
  assign dec = decode_entry(rsp_data, level);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st    <= W_IDLE;
      outst <= 1'b0;
    end else begin
      // outstanding-read bookkeeping (independent of abort)
      if (rd_valid && rd_ready) outst <= 1'b1;
      if (rsp_valid)            outst <= 1'b0;

      if (abort) begin
        st <= W_IDLE;
      end else begin
        case (st)
          W_IDLE: begin
            if (start && start_ready) begin
              va_q     <= start_va;
              tbl_pfn  <= start_root_pfn;
              level    <= 2'd3;
              r_pte    <= '0;
              r_sparse <= 1'b0;
              r_size   <= SZ_4K;
              r_pfn    <= '0;
              if (!table_pa_ok({start_root_pfn, 12'b0})) begin
                r_fault <= F_BAD_PTE;
                r_level <= 2'd3;
                st      <= W_DONE;
              end else begin
                r_fault <= F_NONE;
                st      <= W_REQ;
              end
            end
          end
          W_REQ: if (rd_ready) st <= W_WAIT;
          W_WAIT: begin
            if (rsp_valid) begin
              r_level <= level;
              if (rsp_err) begin
                r_fault <= F_BAD_PHYS;
                r_pte   <= '0;
                st      <= W_DONE;
              end else begin
                r_pte <= rsp_data;
                case (dec.kind)
                  K_NOT_PRESENT: begin r_fault <= F_NOT_PRESENT; st <= W_DONE; end
                  K_BAD_PTE:     begin r_fault <= F_BAD_PTE;     st <= W_DONE; end
                  K_SPARSE: begin
                    r_fault <= F_NONE; r_sparse <= 1'b1; r_size <= dec.size;
                    st <= W_DONE;
                  end
                  K_TABLE: begin
                    tbl_pfn <= dec.pfn;
                    level   <= level - 2'd1;
                    st      <= W_REQ;
                  end
                  default: begin   // K_LEAF
                    r_fault <= F_NONE; r_size <= dec.size; r_pfn <= dec.pfn;
                    st <= W_DONE;
                  end
                endcase
              end
            end
          end
          default: st <= W_IDLE;   // W_DONE: one-cycle pulse
        endcase
      end
    end
  end

endmodule : ng_mmu_walker
