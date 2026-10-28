# SP (ng_sp_top) source list, packages first: noc_pkg ... sp_top.  Shared by the RM builds (syn/shell/build_rm.tcl rm_sp),
# the Verilator / xsim testbench runners (rtl/shell/rm_sp/tb/run_tb_rm_sp.sh, rtl/gpu/tb/run_tb_sp_top.sh, run_tb_core.sh)
# and the OOC syntheses (rtl/gpu/sp/ooc_sp_top.tcl, rtl/gpu/core/ooc_core.tcl).
# $NGRTL is the repo's rtl/ directory (expand with envsubst '$NGRTL', syn/flist.tcl read_flist or tb_lib.sh flist,
# like $IBEX / $SHIM in rtl/gpu/cp/ibex/ibex_files.f).  Not listed here (added by the users): ng_l2_addrmap (core),
# ng_cp_pkg / ng_cp_port / ng_cp_copy (cp_mmu_files.f), the testbench / NoC / L2 files.
$NGRTL/gpu/noc/ng_noc_pkg.sv
$NGRTL/gpu/prim/ng_bitops_pkg.sv
$NGRTL/gpu/l2/ng_l2_amo_alu.sv
$NGRTL/gpu/lsu/ng_lsu_pkg.sv
$NGRTL/gpu/lsu/ng_lsu_dec.sv
$NGRTL/gpu/lsu/ng_lsu_fatomic.sv
$NGRTL/gpu/lsu/ng_lsu_shmem.sv
$NGRTL/gpu/lsu/ng_l1d.sv
$NGRTL/gpu/lsu/ng_lsu.sv
$NGRTL/gpu/lsu/ng_lsu_kn.sv
$NGRTL/gpu/lsu/ng_lsu_top.sv
$NGRTL/gpu/regfile/ng_sp_regfile.sv
$NGRTL/gpu/valu/ng_valu_pkg.sv
$NGRTL/gpu/valu/ng_valu_fp_pkg.sv
$NGRTL/gpu/valu/ng_valu_dec.sv
$NGRTL/gpu/valu/ng_valu_mul.sv
$NGRTL/gpu/valu/ng_valu_lane.sv
$NGRTL/gpu/valu/ng_valu_fp_side.sv
$NGRTL/gpu/valu/ng_valu_fp_lane.sv
$NGRTL/gpu/valu/ng_valu_fp.sv
$NGRTL/gpu/valu/ng_valu.sv
$NGRTL/gpu/sfu/ng_sfu_fp_pkg.sv
$NGRTL/gpu/sfu/ng_sfu_pkg.sv
$NGRTL/gpu/sfu/ng_sfu_dec.sv
$NGRTL/gpu/sfu/ng_sfu_fma.sv
$NGRTL/gpu/sfu/ng_sfu_divsqrt.sv
$NGRTL/gpu/sfu/ng_sfu_lane.sv
$NGRTL/gpu/sfu/ng_sfu.sv
$NGRTL/gpu/xlane/ng_xlane_pkg.sv
$NGRTL/gpu/xlane/ng_xlane_dec.sv
$NGRTL/gpu/xlane/ng_xlane_fsub.sv
$NGRTL/gpu/xlane/ng_xlane.sv
$NGRTL/gpu/fp64/ng_fp64_pkg.sv
$NGRTL/gpu/fp64/ng_fp64_fn_pkg.sv
$NGRTL/gpu/fp64/ng_fp64_dec.sv
$NGRTL/gpu/fp64/ng_fp64_lane.sv
$NGRTL/gpu/fp64/ng_fp64.sv
$NGRTL/gpu/sp_sched/ng_sp_sched_pkg.sv
$NGRTL/gpu/sp_sched/ng_sp_predec.sv
$NGRTL/gpu/sp_sched/ng_sp_sched.sv
$NGRTL/gpu/prim/ng_sp_prim_rr_arb.sv
$NGRTL/gpu/salu/ng_salu_pkg.sv
$NGRTL/gpu/salu/ng_salu_dec.sv
$NGRTL/gpu/salu/ng_salu.sv
$NGRTL/gpu/sp/ng_sp_pkg.sv
$NGRTL/gpu/sp/ng_sp_icache.sv
$NGRTL/gpu/sp/ng_wd.sv
$NGRTL/gpu/sp/ng_sp_top.sv
