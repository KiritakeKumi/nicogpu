# ooc_regfile.tcl -- out-of-context synthesis of ng_sp_regfile in the ng_sp_top configuration
# (NL=16, 16 warps, 1024 warp-registers, 4 banks, 64 uniforms, 5 vector read / 3 write ports,
# 3 uniform read / 5 write ports).
#
#   vivado.bat -mode batch -source <abs>/rtl/gpu/regfile/ooc_regfile.tcl -tclargs <abs workdir> [NL]
# Env: NG_PERIOD (default 10.0).  Estimated minimum period = period - WNS (synthesis only).
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]
set period 10.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }

read_verilog -sv [list $repo/gpu/noc/ng_noc_pkg.sv $repo/gpu/regfile/ng_sp_regfile.sv]

set xdc ooc_regfile.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
puts $fh "set_input_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == IN && NAME != clk && NAME != rst_n}\]"
puts $fh "set_output_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == OUT}\]"
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top ng_sp_regfile -part xcvu9p-flga2104-2-e -mode out_of_context \
  -generic NG_RF_NL=$nl -generic NG_RF_NRP=5 -generic NG_RF_NWP=3 -generic NG_RF_NURP=3 -generic NG_RF_NUWP=5
report_timing_summary -delay_type max -max_paths 5 -file ooc_regfile_timing.rpt
report_utilization -file ooc_regfile_util.rpt
report_utilization -hierarchical -file ooc_regfile_util_h.rpt
check_timing -verbose -file ooc_regfile_check.rpt
puts "OOC_REGFILE_DONE"
