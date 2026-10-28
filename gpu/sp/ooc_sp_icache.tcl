# ooc_sp_icache.tcl -- out-of-context synthesis of the L0 I$ (ng_sp_icache) alone.
#
#   vivado.bat -mode batch -source <abs>/rtl/gpu/sp/ooc_sp_icache.tcl -tclargs <abs workdir> [LINES:WAYS ...]
# Default configurations: 8:0 (fully associative FIFO), 32:0 (default, 8 sets x 4 ways),
# 32:32 (fully associative FIFO with 32 lines, for comparison).  Clock 10 ns (clk_ts, 99.94 MHz); estimated
# minimum period = 10 - WNS (synthesis only, no routing delay).  Per configuration <work>/<L>_<W>/{util,timing}.rpt
# and one summary line "OOC_IC <L>:<W> LUT <n> LUTRAM <n> FF <n> BRAM <n> WNS <ns>" in <work>/summary.txt.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set cfgs [lrange $argv 1 end]
if {[llength $cfgs] == 0} { set cfgs {8:0 32:0 32:32} }
file mkdir $work
set repo [file normalize [file join [file dirname [info script]] ../..]]
set period 10.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }
cd $work
read_verilog -sv [list $repo/gpu/noc/ng_noc_pkg.sv $repo/gpu/lsu/ng_lsu_pkg.sv $repo/gpu/sp/ng_sp_icache.sv]
set fh [open ooc.xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
puts $fh "set_input_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == IN && NAME != clk && NAME != rst_n}\]"
puts $fh "set_output_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == OUT}\]"
close $fh
read_xdc -mode out_of_context ooc.xdc
set sum [open [file join $work summary.txt] w]
foreach c $cfgs {
  lassign [split $c :] lines ways
  set d [file join $work ${lines}_${ways}]
  file mkdir $d
  cd $d
  synth_design -top ng_sp_icache -part xcvu9p-flga2104-2-e -mode out_of_context -flatten_hierarchy rebuilt \
    -generic LINES=$lines -generic WAYS=$ways
  report_utilization -file util.rpt
  report_timing_summary -delay_type max -max_paths 10 -file timing.rpt
  report_timing -delay_type max -max_paths 10 -nworst 1 -unique_pins -file paths.rpt
  set lut  [llength [get_cells -hier -filter {PRIMITIVE_GROUP == CLB && PRIMITIVE_SUBGROUP == LUT}]]
  set lram [llength [get_cells -hier -filter {PRIMITIVE_SUBGROUP == LUTRAM || PRIMITIVE_SUBGROUP == dram}]]
  set ff   [llength [get_cells -hier -filter {PRIMITIVE_SUBGROUP == SDR || PRIMITIVE_SUBGROUP == flop}]]
  set bram [llength [get_cells -hier -filter {PRIMITIVE_GROUP == BLOCKRAM}]]
  set wns  [get_property SLACK [get_timing_paths -delay_type max -max_paths 1]]
  puts $sum "OOC_IC $lines:$ways LUTcells $lut LUTRAMcells $lram FF $ff BRAM $bram WNS $wns"
  flush $sum
  close_design
}
close $sum
puts "OOC_SP_ICACHE_DONE"
