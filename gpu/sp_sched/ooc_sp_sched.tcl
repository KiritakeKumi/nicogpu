# ooc_sp_sched.tcl -- out-of-context synthesis for ng_sp_sched.
#
# DEFERRED: not run while the P0-11 full-chip build holds the machine
# .  Run later with:
#   vivado -mode batch -source rtl/gpu/sp_sched/ooc_sp_sched.tcl \
#     -tclargs <work_dir>
# or from the Vivado Tcl console after cd <work_dir>.
#
# Reports WNS/TNS and utilization for one SP scheduler (NG_SP_NUM_WARPS=16).
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

read_verilog -sv [list \
  $repo/gpu/sp_sched/ng_sp_sched_pkg.sv \
  $repo/gpu/sp_sched/ng_sp_predec.sv \
  $repo/gpu/sp_sched/ng_sp_sched.sv \
  $repo/gpu/prim/ng_sp_prim_rr_arb.sv]

set xdc ooc_sp_sched.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period 10.000 \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top ng_sp_sched -part xcvu9p-flga2104-2-e -mode out_of_context
report_timing_summary -delay_type max -file ooc_sp_sched_timing.rpt
report_utilization -file ooc_sp_sched_util.rpt
report_utilization -hierarchical -file ooc_sp_sched_util_h.rpt
puts "OOC_SP_SCHED_DONE"
