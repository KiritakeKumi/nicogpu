# ng_rm_cp.xdc -- constraints carried by the CP RM, read by syn/shell/build_rm.tcl with `read_xdc -cells <RP>`
# after the RM checkpoint is linked into the static (cell names are relative to ng_gpu_rm; the checkpoint itself does not
# carry them into the in-context implementation).
#
# clk_ts (CP) and clk_l2 (boundary) come from one MMCM, so the static constraints time every path between them as
# synchronous (2.0 ns window).  The RM crosses the domains only with toggle handshakes: the data bundle is written
# by the source domain, the toggle goes through a two-flop ASYNC_REG synchroniser, and the destination samples the
# bundle at least two of its own clocks after the toggle arrived -- the bundle is quasi-static.  Give those paths a
# datapath-only bound instead of the artificial 2 ns window (it still catches a badly routed bundle).
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_mts/x_*_reg*}]        ;# CP request -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_ml2/y_*_reg*}]         ;# l2 response -> CP
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_cpbus/rdata_o_reg*}]   ;# CP reg read data -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_regs/cp_*_reg*}]       ;# BAR0 request -> CP bus
set_max_delay -datapath_only 6.0 -from [all_registers] -to [get_cells -hier -filter {ASYNC_REG == TRUE}]               ;# toggles / irq level into the synchronisers
