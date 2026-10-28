# ooc_lsu.tcl -- out-of-context synthesis for ng_lsu_top (LSU core + L1D + shared memory).
#
# Run (Vivado 2026.1, xcvu9p -2; see the synthesis addendum in):
#   vivado -mode batch -source rtl/gpu/lsu/ooc_lsu.tcl -tclargs <work_dir> [NL] [L1_SETS]
# NL = register-file lanes per beat (16 = L config, 32 = XL); default 16.  L1_SETS default 256.
#
# Environment: NG_PERIOD (clock, default 4 ns), NG_SYNTH_DIRECTIVE (default RuntimeOptimized),
#   NG_FLATTEN (default rebuilt; none = per-module utilisation in ooc_lsu_util_hier.rpt),
#   NG_GENERICS ("SH_LPB=8 L1_LPB=4" ... extra top generics), NG_TOP + NG_GENERICS (block-level run),
#   NG_DCP (write ooc_lsu.dcp).  Reports: ooc_lsu_timing.rpt, ooc_lsu_paths.txt (one line per endpoint with
#   slack < 0.5 ns), ooc_lsu_util*.rpt, ooc_lsu_ram.rpt.  OOC period estimate = clock period - WNS.
# Results: section 9.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
set sets [lindex $argv 2]
if {$sets eq ""} { set sets 256 }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/prim/ng_bitops_pkg.sv \
  $repo/gpu/l2/ng_l2_amo_alu.sv \
  $repo/gpu/lsu/ng_lsu_pkg.sv \
  $repo/gpu/lsu/ng_lsu_dec.sv \
  $repo/gpu/lsu/ng_lsu_fatomic.sv \
  $repo/gpu/lsu/ng_lsu_shmem.sv \
  $repo/gpu/lsu/ng_l1d.sv \
  $repo/gpu/lsu/ng_lsu.sv $repo/gpu/lsu/ng_lsu_kn.sv \
  $repo/gpu/lsu/ng_lsu_top.sv]

set period 4.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }
set directive RuntimeOptimized
if {[info exists ::env(NG_SYNTH_DIRECTIVE)]} { set directive $::env(NG_SYNTH_DIRECTIVE) }

# Clock + I/O budget: input/output ports get a 0 ns external delay so the port-to-register and
# register-to-port paths (the LSU is wired to registered neighbours) are timed as well.
set xdc ooc_lsu.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
puts $fh "set_input_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == IN && NAME != clk}\]"
puts $fh "set_output_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == OUT}\]"
close $fh
read_xdc -mode out_of_context $xdc

set flat rebuilt
if {[info exists ::env(NG_FLATTEN)]} { set flat $::env(NG_FLATTEN) }
set top ng_lsu_top
set gen [list -generic NL=$nl -generic L1_SETS=$sets]
# extra top-level generics, e.g. NG_GENERICS="SH_LPB=8 L1_LPB=4"
if {![info exists ::env(NG_TOP)] && [info exists ::env(NG_GENERICS)]} {
  foreach g $::env(NG_GENERICS) { lappend gen -generic $g }
}
if {[info exists ::env(NG_TOP)]} {
  # block-level run (e.g. NG_TOP=ng_lsu_shmem NG_GENERICS="LPB=8 AW=16"): generics from NG_GENERICS
  set top $::env(NG_TOP)
  set gen {}
  if {[info exists ::env(NG_GENERICS)]} { foreach g $::env(NG_GENERICS) { lappend gen -generic $g } }
}
set t0 [clock seconds]
synth_design -top $top -part xcvu9p-flga2104-2-e -mode out_of_context -directive $directive \
  -flatten_hierarchy $flat {*}$gen
puts "SYNTH_SECONDS [expr {[clock seconds] - $t0}]"
report_timing_summary -delay_type max -max_paths 3 -file ooc_lsu_timing.rpt
# one line per failing endpoint (worst path to each): slack, start -> end, logic levels
set fo [open ooc_lsu_paths.txt w]
foreach p [get_timing_paths -delay_type max -max_paths 600 -nworst 1 -slack_lesser_than 0.5] {
  puts $fo "[get_property SLACK $p] [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p] lv=[get_property LOGIC_LEVELS $p]"
}
close $fo
report_utilization -file ooc_lsu_util.rpt
report_utilization -hierarchical -file ooc_lsu_util_hier.rpt
report_ram_utilization -file ooc_lsu_ram.rpt
puts "OOC_LSU_DONE"
if {[info exists ::env(NG_DCP)]} { write_checkpoint -force ooc_lsu.dcp }
