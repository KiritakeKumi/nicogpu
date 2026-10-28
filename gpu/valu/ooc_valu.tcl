# ooc_valu.tcl -- out-of-context synthesis for ng_valu (integer VALU f32 subset).
#
#   vivado.bat -mode batch -source <abs>/rtl/gpu/valu/ooc_valu.tcl -tclargs <abs workdir> [NL] [EN_FP] [NFL] [TOP] [F16]
# NL    = lanes per beat (16 = L config, 32 = XL); default 16.
# EN_FP = 1 (default): f32 VALU subset included; 0: integer VALU only.
# NFL   = f32 physical lanes (divides NL); default 4.
# TOP   = ng_valu (default) or ng_valu_fp (the f32 engine alone: its internal paths and area).
# F16   = 1: binary16 conversions (ng_valu EN_F16CVT / ng_valu_fp EN_F16); default 0.
# Env:  NG_PERIOD (default 10.0 ns), NG_FLATTEN (default none: keeps the hierarchy so that
#       report_utilization -hierarchical gives a per-module breakdown).
# Reports WNS/TNS and utilization; estimated minimum period = period - WNS (synthesis only,
# no routing delay).  Files: ooc_valu_timing.rpt, ooc_valu_util.rpt, ooc_valu_util_hier.rpt.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
set enfp [lindex $argv 2]
if {$enfp eq ""} { set enfp 1 }
set nfl [lindex $argv 3]
if {$nfl eq ""} { set nfl 4 }
set top [lindex $argv 4]
if {$top eq ""} { set top ng_valu }
set f16 [lindex $argv 5]
if {$f16 eq ""} { set f16 0 }
set period 10.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }
set flat none
if {[info exists ::env(NG_FLATTEN)]} { set flat $::env(NG_FLATTEN) }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/prim/ng_bitops_pkg.sv \
  $repo/gpu/valu/ng_valu_pkg.sv \
  $repo/gpu/valu/ng_valu_fp_pkg.sv \
  $repo/gpu/valu/ng_valu_dec.sv \
  $repo/gpu/valu/ng_valu_mul.sv \
  $repo/gpu/valu/ng_valu_lane.sv \
  $repo/gpu/valu/ng_valu_fp_side.sv \
  $repo/gpu/valu/ng_valu_fp_lane.sv \
  $repo/gpu/valu/ng_valu_fp.sv \
  $repo/gpu/valu/ng_valu.sv]

set xdc ooc_valu.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

if {$top eq "ng_valu_fp"} {
  synth_design -top $top -part xcvu9p-flga2104-2-e -mode out_of_context -flatten_hierarchy $flat \
    -generic NL=$nl -generic NFL=$nfl -generic EN_F16=$f16
} else {
  synth_design -top $top -part xcvu9p-flga2104-2-e -mode out_of_context -flatten_hierarchy $flat \
    -generic NL=$nl -generic EN_FP=$enfp -generic NFL=$nfl -generic EN_F16CVT=$f16
}
report_timing_summary -delay_type max -file ooc_valu_timing.rpt
report_timing -max_paths 3 -nworst 1 -delay_type max -file ooc_valu_paths.rpt
report_utilization -file ooc_valu_util.rpt
report_utilization -hierarchical -file ooc_valu_util_hier.rpt
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -delay_type max]]
puts "OOC_VALU_WNS $wns period $period est_min_period [expr {$period - $wns}]"
puts "OOC_VALU_DONE"
