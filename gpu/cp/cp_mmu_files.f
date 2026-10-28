# CP subsystem + GPU MMU source list, packages first (ng_cp_top with the Ibex wrapper, ng_mmu).  Used by the CP / SP RM
# builds and testbenches (syn/shell/build_rm.tcl rm_cp / rm_sp, rtl/shell/rm_{cp,sp}/tb/run_tb_rm_{cp,sp}.sh); the Ibex
# sources themselves are in ibex/ibex_files.f.  $NGRTL: the repo's rtl/ directory (see rtl/gpu/sp/sp_files.f).
$NGRTL/gpu/cp/ng_cp_pkg.sv
$NGRTL/gpu/cp/ng_cp_regs.sv
$NGRTL/gpu/cp/ng_cp_port.sv
$NGRTL/gpu/cp/ng_cp_copy.sv
$NGRTL/gpu/cp/ng_cp_boot.sv
$NGRTL/gpu/cp/ng_cp_ibex_wrap.sv
$NGRTL/gpu/cp/ng_cp_top.sv
$NGRTL/gpu/mmu/ng_mmu_pkg.sv
$NGRTL/gpu/mmu/ng_mmu_regs.sv
$NGRTL/gpu/mmu/ng_mmu_tlb.sv
$NGRTL/gpu/mmu/ng_mmu_walker.sv
$NGRTL/gpu/mmu/ng_mmu.sv
