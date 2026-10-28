# ooc_mmu.tcl -- out-of-context synthesis for ng_mmu (VMID contexts + TLB + walker + fault latch).
#
#   vivado -mode batch -source rtl/gpu/mmu/ooc_mmu.tcl -tclargs <work_dir> [<rtl/gpu dir>] [<period ns>]
#
# Reports WNS/TNS and utilization.  Default clock 4.0 ns (clk_l2, 250 MHz, the clock ng_mmu runs on in rm_cp / rm_sp);
# the optional second argument points at another checkout's rtl/gpu (before/after comparisons).
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set gpu [lindex $argv 1]
if {$gpu eq ""} { set gpu [file normalize [file join [file dirname [info script]] ..]] }
set period [lindex $argv 2]
if {$period eq ""} { set period 4.000 }
file mkdir $work
cd $work

read_verilog -sv [list \
  $gpu/mmu/ng_mmu_pkg.sv \
  $gpu/mmu/ng_mmu_tlb.sv \
  $gpu/mmu/ng_mmu_walker.sv \
  $gpu/mmu/ng_mmu_regs.sv \
  $gpu/mmu/ng_mmu.sv]

set xdc ooc_mmu.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top ng_mmu -part xcvu9p-flga2104-2-e -mode out_of_context
report_timing_summary -delay_type max -file ooc_mmu_timing.rpt
report_utilization -file ooc_mmu_util.rpt
puts "OOC_MMU_DONE"
