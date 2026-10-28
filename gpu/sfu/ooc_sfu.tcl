# ooc_sfu.tcl -- out-of-context synthesis for ng_sfu (SFU).
#
#   vivado -mode batch -source rtl/gpu/sfu/ooc_sfu.tcl -tclargs <work_dir> [NL] [NSL] [TOP]
# NL  = register-file lanes per beat (16 = L config, 32 = XL); default 16.
# NSL = SFU physical lanes (each = FMA64 + radix-2 div/sqrt engine); default 1
#       (M1: one lane sweeps the warp, no throughput requirement).
# TOP = ng_sfu (default) or a sub-block (ng_sfu_lane, ng_sfu_fma, ng_sfu_divsqrt,
#       ng_sfu_rnd, ng_sfu_unp, ng_sfu_split) for per-block timing.
#
# Part xcvu9p-flga2104-2-e, 10 ns clock (100 MHz M1 target).  Estimated minimum
# period = 10 - WNS (synthesis only, no routing delay).  Outputs in <work_dir>:
#   ooc_sfu_timing.rpt   summary
#   ooc_sfu_paths.rpt    10 worst paths (max delay)
#   ooc_sfu_util.rpt     utilisation
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
set nsl [lindex $argv 2]
if {$nsl eq ""} { set nsl 1 }
set top [lindex $argv 3]
if {$top eq ""} { set top ng_sfu }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/prim/ng_bitops_pkg.sv \
  $repo/gpu/sfu/ng_sfu_fp_pkg.sv \
  $repo/gpu/sfu/ng_sfu_pkg.sv \
  $repo/gpu/sfu/ng_sfu_dec.sv \
  $repo/gpu/sfu/ng_sfu_fma.sv \
  $repo/gpu/sfu/ng_sfu_divsqrt.sv \
  $repo/gpu/sfu/ng_sfu_lane.sv \
  $repo/gpu/sfu/ng_sfu.sv]

set xdc ooc_sfu.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period 10.000 \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

if {$top eq "ng_sfu"} {
  synth_design -top $top -part xcvu9p-flga2104-2-e -mode out_of_context \
    -generic NL=$nl -generic NSL=$nsl
} else {
  synth_design -top $top -part xcvu9p-flga2104-2-e -mode out_of_context
}
report_timing_summary -delay_type max -file ooc_sfu_timing.rpt
report_timing -delay_type max -max_paths 10 -nworst 1 -unique_pins -file ooc_sfu_paths.rpt
report_utilization -file ooc_sfu_util.rpt
puts "OOC_SFU_DONE"
