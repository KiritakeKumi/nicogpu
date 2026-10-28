// ng_l2_addrmap.sv -- L2 address interleave.  RTL port of the golden
// definition (lines 17-53); keep line-for-line
// aligned with it.  Testbench tb_l2_slice / tb_noc_l2map check this module
// against golden vectors exported by rtl/gpu/tb/gen_golden/
// gen_interleave_golden.py (which ports the same C++ code to Python).
//
// Physical VRAM layout:
//   VRAM PA 0x0000_0000_0000 .. 0x0003_FFFF_FFFF (16 GiB), 4 KiB granules,
//   every 4 consecutive granules map to channels C1,C2,C1,C4.
//   L2 slices: slice 0/1 -> C1 (even/odd channel granule), 2 -> C2, 3 -> C4.
//   Channel numbering here: 0=C1, 1=C2, 3=C4 (matches Channel enum
//   C1=0, C2=1, C3=2, C4=3 in common.h; C3 has no L2 slice).
package ng_l2_addrmap;

  localparam logic [47:0] L2_VRAM_BYTES = 48'd17179869184;  // 16 GiB
  localparam int          L2_GRAN_LB    = 12;               // 4 KiB granule

  typedef struct packed {
    logic [2:0]  ch;    // 0=C1, 1=C2, 3=C4
    logic [47:0] off;   // byte offset inside the channel
  } ch_addr_s;

  // interleave.cpp:50-53  l2_slice(off)
  function automatic int l2_slice_idx(logic [47:0] pa);
    // kSlot2Slice[4] = {0, 2, 1, 3}; slot = (off / kGranule) & 3
    case ((pa >> L2_GRAN_LB) & 2'd3)
      2'd0:    return 0;
      2'd1:    return 2;
      2'd2:    return 1;
      default: return 3;
    endcase
  endfunction

  function automatic logic l2_pa_is_vram(logic [47:0] pa);
    return (pa < L2_VRAM_BYTES);
  endfunction

  // interleave.cpp:17-26  vram_interleave(off)  (VRAM base is PA 0)
  function automatic ch_addr_s l2_ch_of(logic [47:0] pa);
    ch_addr_s ca;
    logic [47:0] g, grp, in;
    g   = pa >> L2_GRAN_LB;
    grp = g >> 2;
    in  = pa & ((48'd1 << L2_GRAN_LB) - 1);
    case (g & 2'd3)
      2'd0: begin  // -> C1, channel granule (group*2 + 0)
        ca.ch  = 3'd0;
        ca.off = ((grp << 1) << L2_GRAN_LB) | in;
      end
      2'd1: begin  // -> C2, channel granule group
        ca.ch  = 3'd1;
        ca.off = (grp << L2_GRAN_LB) | in;
      end
      2'd2: begin  // -> C1, channel granule (group*2 + 1)
        ca.ch  = 3'd0;
        ca.off = (((grp << 1) | 48'd1) << L2_GRAN_LB) | in;
      end
      default: begin  // -> C4, channel granule group
        ca.ch  = 3'd3;
        ca.off = (grp << L2_GRAN_LB) | in;
      end
    endcase
    return ca;
  endfunction

endpackage : ng_l2_addrmap
