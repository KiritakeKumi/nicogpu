# ooc_fp64.tcl -- out-of-context synthesis for ng_fp64 (FP64 unit).
#
# DEFERRED: not run (a long place-and-route regression occupied the machine while
# was written).  Run later with:
#   vivado -mode batch -source rtl/gpu/fp64/ooc_fp64.tcl -tclargs <work_dir> [NL] [NFL]
# NL  = register-file lanes per beat (16 = L config, 32 = XL); default 16.
# NFL = FP64 physical lanes (each = 3-stage FMA64 + conversion / compare side paths); default 2.
#
# Reports WNS/TNS and utilization for one FP64 unit.  The 400 MHz target (2.5 ns) is
# the VALU constraint; the first-cut lane is only 3 stages: expect the 53x53 multiply
# (S1), the 111-bit align/add (S2) and the normalise + rnd_pack barrel shifters (S3) to be the
# critical paths.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
set nfl [lindex $argv 2]
if {$nfl eq ""} { set nfl 2 }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/prim/ng_bitops_pkg.sv \
  $repo/gpu/sfu/ng_sfu_fp_pkg.sv \
  $repo/gpu/fp64/ng_fp64_pkg.sv \
  $repo/gpu/fp64/ng_fp64_fn_pkg.sv \
  $repo/gpu/fp64/ng_fp64_dec.sv \
  $repo/gpu/fp64/ng_fp64_lane.sv \
  $repo/gpu/fp64/ng_fp64.sv]

set xdc ooc_fp64.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period 2.500 \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top ng_fp64 -part xcvu9p-flga2104-2-e -mode out_of_context \
  -generic NL=$nl -generic NFL=$nfl
report_timing_summary -delay_type max -file ooc_fp64_timing.rpt
report_utilization -file ooc_fp64_util.rpt
puts "OOC_FP64_DONE"
