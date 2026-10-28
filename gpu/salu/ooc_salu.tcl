# ooc_salu.tcl -- out-of-context synthesis for ng_salu (SALU + branch + sync/misc/CSR unit).
#
# NOT RUN (simulation-only task; no Vivado was started).  Run later with:
#   vivado -mode batch -source rtl/gpu/salu/ooc_salu.tcl -tclargs <work_dir>
#
# Reports WNS/TNS and utilization for one unit.  400 MHz target (2.5 ns), like
# ooc_valu.tcl.  Expectations to compare with: 16 slots x (EXEC + 7 CSR + 6 launch
# parameters) = 16 x 14 x 32 b of LUTRAM/FF state, one 33x33 multiplier (DSP, product
# register in the MUL state), no BRAM.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

read_verilog -sv [list \
  $repo/gpu/noc/ng_noc_pkg.sv \
  $repo/gpu/prim/ng_bitops_pkg.sv \
  $repo/gpu/salu/ng_salu_pkg.sv \
  $repo/gpu/salu/ng_salu_dec.sv \
  $repo/gpu/salu/ng_salu.sv]

set xdc ooc_salu.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period 2.500 \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top ng_salu -part xcvu9p-flga2104-3-e -mode out_of_context
report_timing_summary -delay_type max -file ooc_salu_timing.rpt
report_utilization -file ooc_salu_util.rpt
puts "OOC_SALU_DONE"
