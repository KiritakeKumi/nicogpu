# ooc_l2.tcl -- R7 out-of-context synthesis of one full-size L2 slice (ng_l2_slice, SETS = 1024, WAYS = 8, C1 port),
# or of the NoC mesh (NG_TOP=ng_noc_mesh, 3 x 3).  Synthesis only.
#
# Run (Vivado 2026.1, xcvu9p -2; results in):
#   vivado -mode batch -source rtl/gpu/l2/ooc_l2.tcl -tclargs <work_dir>
# Environment: NG_PERIOD (clock, default 4 ns = clk_l2 250 MHz), NG_TOP (ng_l2_slice | ng_noc_mesh),
#   NG_PRIM (alternative ng_prim_l2ram.sv, e.g. a copy with ram_style = "ultra"), NG_GENERICS ("SETS=1024 WAYS=8" ...).
# Reports: ooc_l2_timing.rpt, ooc_l2_util.rpt, ooc_l2_util_hier.rpt, ooc_l2_ram.rpt.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]
set prim $repo/gpu/prim/ng_prim_l2ram.sv
if {[info exists ::env(NG_PRIM)]} { set prim $::env(NG_PRIM) }

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/noc/ng_noc_router.sv \
  $repo/gpu/noc/ng_noc_mesh.sv \
  $repo/gpu/noc/ng_noc_ep.sv \
  $repo/gpu/l2/ng_l2_addrmap.sv \
  $repo/gpu/l2/ng_l2_amo_alu.sv \
  $repo/gpu/l2/ng_l2_slice.sv \
  $prim]

set period 4.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }
set top ng_l2_slice
if {[info exists ::env(NG_TOP)]} { set top $::env(NG_TOP) }
set gen {}
if {$top eq "ng_l2_slice"} { set gen [list -generic SETS=1024 -generic WAYS=8 -generic AXI_DW=512 -generic AXI_AW=33] }
if {$top eq "ng_noc_mesh"} { set gen [list -generic GW=3 -generic GH=3] }
if {[info exists ::env(NG_GENERICS)]} {
  set gen {}
  foreach g $::env(NG_GENERICS) { lappend gen -generic $g }
}

set xdc ooc_l2.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
puts $fh "set_input_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == IN && NAME != clk}\]"
puts $fh "set_output_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == OUT}\]"
close $fh
read_xdc -mode out_of_context $xdc

set t0 [clock seconds]
synth_design -top $top -part xcvu9p-flga2104-2-e -mode out_of_context -directive RuntimeOptimized \
  -flatten_hierarchy rebuilt {*}$gen
puts "SYNTH_SECONDS [expr {[clock seconds] - $t0}]"
report_timing_summary -delay_type max -max_paths 3 -file ooc_l2_timing.rpt
report_utilization -file ooc_l2_util.rpt
report_utilization -hierarchical -file ooc_l2_util_hier.rpt
report_ram_utilization -file ooc_l2_ram.rpt
puts "OOC_L2_DONE"
