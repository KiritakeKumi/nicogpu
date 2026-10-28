# Ibex (lowRISC v2.1.0, see third_party/README.md) file list for a
# Verilator/xsim build.  $IBEX is the checkout, $SHIM the output of
# gen_prim_shim.sh (expand with envsubst/sed).  Include dirs: $IBEX/rtl
# $IBEX/vendor/lowrisc_ip/ip/prim/rtl $IBEX/vendor/lowrisc_ip/dv/sv/dv_utils
# (Verilator: +incdir+ ; xvlog: -i).  Ibex configured top-down by ng_cp_ibex_wrap.
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_util_pkg.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_secded_pkg.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_ram_1p_pkg.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_count_pkg.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_cipher_pkg.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_mubi_pkg.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_onehot_enc.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_onehot_mux.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_onehot_check.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_secded_inv_28_22_enc.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_secded_inv_28_22_dec.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_secded_inv_39_32_enc.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_secded_inv_39_32_dec.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_lfsr.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_prince.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_subst_perm.sv
$IBEX/vendor/lowrisc_ip/ip/prim/rtl/prim_ram_1p_scr.sv
$SHIM/prim_buf.sv
$SHIM/prim_flop.sv
$SHIM/prim_ram_1p.sv
$SHIM/prim_clock_gating.sv
$IBEX/rtl/ibex_pkg.sv
$IBEX/rtl/ibex_alu.sv
$IBEX/rtl/ibex_compressed_decoder.sv
$IBEX/rtl/ibex_controller.sv
$IBEX/rtl/ibex_counter.sv
$IBEX/rtl/ibex_csr.sv
$IBEX/rtl/ibex_cs_registers.sv
$IBEX/rtl/ibex_decoder.sv
$IBEX/rtl/ibex_ex_block.sv
$IBEX/rtl/ibex_id_stage.sv
$IBEX/rtl/ibex_if_stage.sv
$IBEX/rtl/ibex_load_store_unit.sv
$IBEX/rtl/ibex_multdiv_slow.sv
$IBEX/rtl/ibex_multdiv_fast.sv
$IBEX/rtl/ibex_prefetch_buffer.sv
$IBEX/rtl/ibex_fetch_fifo.sv
$IBEX/rtl/ibex_register_file_ff.sv
$IBEX/rtl/ibex_pmp.sv
$IBEX/rtl/ibex_icache.sv
$IBEX/rtl/ibex_wb_stage.sv
$IBEX/rtl/ibex_branch_predict.sv
$IBEX/rtl/ibex_dummy_instr.sv
$IBEX/rtl/ibex_core.sv
$IBEX/rtl/ibex_top.sv
