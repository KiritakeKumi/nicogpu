# ooc_xlane.tcl -- out-of-context synthesis for ng_xlane (cross-lane unit).
#
# DEFERRED: not run (a long place-and-route occupied the machine while was
# written; synthesis was forbidden).  Run later with:
#   vivado -mode batch -source rtl/gpu/xlane/ooc_xlane.tcl -tclargs <work_dir> [NL]
# NL = lanes per beat (16 = L config, 32 = XL); default 16.
#
# Reports WNS/TNS and utilization for one xlane unit at NG_PERIOD ns (default 10 = clk_ts).  Expected
# critical paths: the 32-lane COMP stage (prefix-sum / reduction tree / 32:1
# lane muxes / f32 subtract + normalise), none of it pipelined in this first cut.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
set repo [file normalize [file join [file dirname [file normalize [info script]]] ../..]]
file mkdir $work
cd $work

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/prim/ng_bitops_pkg.sv \
  $repo/gpu/xlane/ng_xlane_pkg.sv \
  $repo/gpu/xlane/ng_xlane_dec.sv \
  $repo/gpu/xlane/ng_xlane_fsub.sv \
  $repo/gpu/xlane/ng_xlane.sv]

set period 10.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }
set xdc ooc_xlane.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

set flat rebuilt
if {[info exists ::env(NG_FLATTEN)]} { set flat $::env(NG_FLATTEN) }   ;# none: per-module utilisation
synth_design -top ng_xlane -part xcvu9p-flga2104-2-e -mode out_of_context -flatten_hierarchy $flat \
  -generic NL=$nl
report_timing_summary -delay_type max -max_paths 5 -file ooc_xlane_timing.rpt
report_timing -delay_type max -max_paths 10 -nworst 1 -unique_pins -file ooc_xlane_paths.rpt
report_utilization -file ooc_xlane_util.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file ooc_xlane_util_h.rpt
puts "OOC_XLANE_DONE"
