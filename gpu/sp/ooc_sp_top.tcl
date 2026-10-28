# ooc_sp_top.tcl -- out-of-context synthesis of ng_sp_top (one SP), M1 configuration.
#
#   vivado.bat -mode batch -source <abs>/rtl/gpu/sp/ooc_sp_top.tcl -tclargs <abs workdir> [NL] [STUB]
# NL   = register-file lanes per beat (default 16).
# STUB = 1 (default): the LSU (ng_lsu_top, ~19 min / 360k LUT) is replaced by
#        rtl/gpu/sp/ooc_stub/ng_lsu_top_stub.sv (registered ports, same interface);
#        0 = the real LSU (L1_SETS=16, SH_AW=16).
# Env: NG_PERIOD (default 10.0), NG_FLATTEN (default none: keeps the hierarchy so that
#      report_utilization -hierarchical gives a per-module breakdown; "rebuilt" = Vivado default),
#      NG_EN_XLANE (default 0; 1 = EN_XLANE), NG_DIRECTIVE (default RuntimeOptimized), NG_IC_LINES (ng_sp_top IC_LINES,
#      default the RTL's 32; 8 = I$), NG_LSU_K / NG_MEM_OUT (ng_sp_top LSU_K / MEM_OUT; default 1 / 1 =
#      the pre-26c3 SP).
# (all optional, default = unchanged behaviour of this script): NG_EN_SFU (0), NG_EN_FP64 (0),
#      NG_EN_VALU_FP (RTL default 0), NG_F16CVT (EN_VALU_F16CVT, RTL default 0), NG_VALU_NFL (RTL default 4),
#      NG_VREGS (RTL default 1024), NG_SH_AW (RTL default 16), NG_L1_SETS (16 when STUB = 0), NG_NWG (RTL default 8),
#      NG_VALU_BDEPTH (RTL default 3), NG_SH_LPB / NG_L1_LPB (LSU lanes per beat, RTL default 8 / 4),
#      NG_HDEPTH (report_utilization -hierarchical_depth, default 3).  The board rm_sp SP (b16..b19) is
#      STUB = 0, NG_EN_SFU=1 NG_EN_XLANE=1 NG_EN_VALU_FP=1 NG_F16CVT=1 NG_LSU_K=4 NG_MEM_OUT=8.
# Clock period 10 ns (clk_ts, 99.94 MHz); estimated minimum period = 10 - WNS (synthesis only,
# no routing delay).  EN_SFU / EN_XLANE / EN_FP64 = 0 (M1) unless overridden above.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
set nl [lindex $argv 1]
if {$nl eq ""} { set nl 16 }
set stub [lindex $argv 2]
if {$stub eq ""} { set stub 1 }
file mkdir $work
cd $work

set repo [file normalize [file join [file dirname [info script]] ../..]]

set period 10.000
if {[info exists ::env(NG_PERIOD)]} { set period $::env(NG_PERIOD) }
set flat none
if {[info exists ::env(NG_FLATTEN)]} { set flat $::env(NG_FLATTEN) }
set directive RuntimeOptimized
if {[info exists ::env(NG_DIRECTIVE)]} { set directive $::env(NG_DIRECTIVE) }

source $repo/../syn/flist.tcl
# sp_files.f (noc_pkg ... sp_top); STUB: the LSU implementation (everything after ng_lsu_dec) is replaced by the stub
set files [list $repo/gpu/cp/ng_cp_pkg.sv $repo/gpu/cp/ng_cp_port.sv $repo/gpu/cp/ng_cp_copy.sv]
set spf [read_flist $repo/gpu/sp/sp_files.f]
if {$stub} {
  set spf [flist_insert_after [flist_drop $spf {ng_lsu_fatomic ng_lsu_shmem ng_l1d ng_lsu ng_lsu_kn ng_lsu_top}] \
    ng_lsu_dec.sv $repo/gpu/sp/ooc_stub/ng_lsu_top_stub.sv]
}
lappend files {*}$spf
read_verilog -sv $files

set xdc ooc_sp_top.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
puts $fh "set_input_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == IN && NAME != clk && NAME != rst_n}\]"
puts $fh "set_output_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == OUT}\]"
close $fh
read_xdc -mode out_of_context $xdc

set enx 0
if {[info exists ::env(NG_EN_XLANE)]} { set enx $::env(NG_EN_XLANE) }   ;# 1 = the rm_sp shape with XLANE
set ensfu 0
if {[info exists ::env(NG_EN_SFU)]} { set ensfu $::env(NG_EN_SFU) }     ;# 1 = the rm_sp shape with SFU
set enfp64 0
if {[info exists ::env(NG_EN_FP64)]} { set enfp64 $::env(NG_EN_FP64) }
set gen [list -generic NL=$nl -generic EN_SFU=$ensfu -generic EN_XLANE=$enx -generic EN_FP64=$enfp64]
if {!$stub} {
  if {[info exists ::env(NG_L1_SETS)]} { lappend gen -generic L1_SETS=$::env(NG_L1_SETS) } else { lappend gen -generic L1_SETS=16 }
}
if {[info exists ::env(NG_IC_LINES)]} { lappend gen -generic IC_LINES=$::env(NG_IC_LINES) }   ;# (default 32)
if {[info exists ::env(NG_LSU_K)]} { lappend gen -generic LSU_K=$::env(NG_LSU_K) }            ;# (default 1)
if {[info exists ::env(NG_MEM_OUT)]} { lappend gen -generic MEM_OUT=$::env(NG_MEM_OUT) }      ;# (default 1)
# overrides (absent = the RTL default)
foreach {ev gname} {NG_EN_VALU_FP EN_VALU_FP NG_F16CVT EN_VALU_F16CVT NG_VALU_NFL VALU_NFL NG_VREGS VREGS \
                    NG_SH_AW SH_AW NG_NWG NWG NG_VALU_BDEPTH VALU_BDEPTH NG_SH_LPB SH_LPB NG_L1_LPB L1_LPB} {
  if {[info exists ::env($ev)]} { lappend gen -generic $gname=$::env($ev) }
}
set hdepth 3
if {[info exists ::env(NG_HDEPTH)]} { set hdepth $::env(NG_HDEPTH) }
set t0 [clock seconds]
synth_design -top ng_sp_top -part xcvu9p-flga2104-2-e -mode out_of_context \
  -directive $directive -flatten_hierarchy $flat {*}$gen
set t1 [clock seconds]
report_timing_summary -delay_type max -max_paths 10 -file ooc_sp_top_timing.rpt
report_timing -delay_type max -max_paths 30 -nworst 1 -unique_pins -file ooc_sp_top_paths.rpt
# one line per failing endpoint (worst 3000): slack, levels, start -> end
set fh [open ooc_sp_top_endpoints.txt w]
foreach p [get_timing_paths -max_paths 3000 -nworst 1 -slack_lesser_than 10] {
  puts $fh [format "%8.3f %3d %s -> %s" [get_property SLACK $p] [get_property LOGIC_LEVELS $p] \
    [get_property STARTPOINT_PIN $p] [get_property ENDPOINT_PIN $p]]
}
close $fh
report_utilization -file ooc_sp_top_util.rpt
report_utilization -hierarchical -hierarchical_depth $hdepth -file ooc_sp_top_util_h.rpt
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set fh [open ooc_sp_top_summary.txt w]
puts $fh "ng_sp_top OOC: stub=$stub generics={$gen} period=$period flatten=$flat WNS=$wns synth_s=[expr {$t1 - $t0}]"
close $fh
report_methodology -file ooc_sp_top_meth.rpt
check_timing -verbose -file ooc_sp_top_check.rpt
puts "OOC_SP_TOP_DONE"
