# ooc_core.tcl -- out-of-context synthesis of ng_core (the multi-SP "L configuration" core: NSP x ng_sp_top,
# ng_core_wd, per-SP NoC adapters, 3x3 NoC mesh, 4 L2 slices).  Reference numbers only (simulation scope, Q2).
#
#   vivado.bat -mode batch -source <abs>/rtl/gpu/core/ooc_core.tcl -tclargs <abs workdir>
# Env (all optional):
#   NG_NSP       SPs (default 2: the "2 SP + 1 L2 slice" board trade-off  is read off the
#                hierarchical report: total - 3 slices)
#   NG_STUB      1 = the LSU of every SP is rtl/gpu/sp/ooc_stub/ng_lsu_top_stub.sv (fast; the adapters then
#                constant-folded mem ports, so read the NoC / L2 numbers from an NG_STUB=0 run); default 0
#   NG_L2_SETS / NG_L2_WAYS   slice geometry (default 1024 / 8 = 1 MiB per slice, the full configuration)
#   NG_EN_SFU    ng_sp_top EN_SFU (default 1, as the board RM rm_sp); XLANE / FP64 off, VALU f32 on
#   NG_PERIOD    clock period in ns (default 10.0 = clk_ts; the design target of the L2 / NoC is 250 MHz, clk_l2)
#   NG_DIRECTIVE synth_design -directive (default RuntimeOptimized); NG_FLATTEN (default none: per-module report)
# Outputs in the work dir: ooc_core_util.rpt, ooc_core_util_h.rpt (hierarchical, depth 3), ooc_core_timing.rpt,
# ooc_core_paths.rpt, ooc_core_summary.txt.
set work [lindex $argv 0]
if {$work eq ""} { set work [pwd] }
file mkdir $work
cd $work
set repo [file normalize [file join [file dirname [info script]] ../..]]

proc envd {name def} { if {[info exists ::env($name)]} { return $::env($name) } else { return $def } }
set nsp       [envd NG_NSP 2]
set stub      [envd NG_STUB 0]
set l2sets    [envd NG_L2_SETS 1024]
set l2ways    [envd NG_L2_WAYS 8]
set ensfu     [envd NG_EN_SFU 1]
set period    [envd NG_PERIOD 10.000]
set directive [envd NG_DIRECTIVE RuntimeOptimized]
set flat      [envd NG_FLATTEN none]

set g $repo/gpu
source $repo/../syn/flist.tcl
# sp_files.f (noc_pkg ... sp_top) without ng_lsu_kn: the core runs LSU_K = 1 only; STUB: the LSU implementation (everything
# after ng_lsu_dec) is replaced by the stub
set spf [flist_drop [read_flist $g/sp/sp_files.f] ng_lsu_kn]
if {$stub} {
  set spf [flist_insert_after [flist_drop $spf {ng_lsu_fatomic ng_lsu_shmem ng_l1d ng_lsu ng_lsu_top}] \
    ng_lsu_dec.sv $g/sp/ooc_stub/ng_lsu_top_stub.sv]
}
set files [concat [list $g/l2/ng_l2_addrmap.sv $g/cp/ng_cp_pkg.sv] $spf [list \
  $g/noc/ng_noc_router.sv $g/noc/ng_noc_mesh.sv $g/noc/ng_noc_ep.sv \
  $g/l2/ng_l2_slice.sv $g/prim/ng_prim_l2ram.sv $g/lsu/ng_lsu_noc_adapter.sv \
  $g/core/ng_core_arb.sv $g/core/ng_core_wd.sv $g/core/ng_core.sv]]
read_verilog -sv $files

set xdc ooc_core.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
puts $fh "set_input_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == IN && NAME != clk && NAME != rst_n}\]"
puts $fh "set_output_delay -clock clk 0.0 \[get_ports -filter {DIRECTION == OUT}\]"
close $fh
read_xdc -mode out_of_context $xdc

set gen [list -generic NSP=$nsp -generic L2_SETS=$l2sets -generic L2_WAYS=$l2ways -generic EN_SFU=$ensfu \
  -generic EN_XLANE=0 -generic EN_FP64=0 -generic EN_VALU_FP=1]
set t0 [clock seconds]
synth_design -top ng_core -part xcvu9p-flga2104-2-e -mode out_of_context -directive $directive \
  -flatten_hierarchy $flat {*}$gen
set t1 [clock seconds]
report_timing_summary -delay_type max -max_paths 10 -file ooc_core_timing.rpt
report_timing -delay_type max -max_paths 30 -nworst 1 -unique_pins -file ooc_core_paths.rpt
report_utilization -file ooc_core_util.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file ooc_core_util_h.rpt
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set fh [open ooc_core_summary.txt w]
puts $fh "ng_core OOC: NSP=$nsp STUB=$stub L2=${l2sets}x${l2ways} EN_SFU=$ensfu period=$period WNS=$wns synth_s=[expr {$t1 - $t0}]"
close $fh
puts "OOC_CORE_DONE WNS=$wns"
