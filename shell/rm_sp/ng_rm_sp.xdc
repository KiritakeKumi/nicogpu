# ng_rm_sp.xdc -- constraints carried by the M1 SP RM, read by syn/shell/build_rm.tcl with `read_xdc -cells <RP>`
# after the RM checkpoint is linked into the static (cell names are relative to ng_gpu_rm; with -cells every pattern
# needs the leading `*` to match the hierarchical names below the RP cell).
#
# clk_ts (CP + SP) and clk_l2 (boundary) come from one MMCM, so the static constraints time every path between them as
# synchronous (2.0 ns window).  The RM crosses the domains only with toggle handshakes: the data bundle is written by
# the source domain into flops, the toggle goes through a two-flop ASYNC_REG synchroniser, and the destination samples
# the bundle at least two of its own clocks after the toggle arrived -- the bundle is quasi-static.  Give those paths a
# datapath-only bound instead of the artificial 2 ns window (it still catches a badly routed bundle).
#
# ---- rm_cp part (identical to rtl/shell/rm_cp/ng_rm_cp.xdc)
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_mts/x_*_reg*}]        ;# CP request -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_ml2/y_*_reg*}]         ;# l2 response -> CP
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_cpbus/rdata_o_reg*}]   ;# CP reg / debug page read data -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_regs/cp_*_reg*}]       ;# BAR0 request -> CP bus
# ---- SP bridges
# ---- SP fabric port: asynchronous FIFOs + slot-indexed LUTRAMs, written by one domain and read
# asynchronously by the other; the writer fills the RAM before it moves the Gray pointer (>= 2 synchroniser clocks of
# the reader), so a 6 ns datapath bound from the RAM cells is ample.  The Gray pointers are covered by the
# ASYNC_REG rule at the end (each pointer changes at most once per 8 ns: clk_ts pushes, every second clk_l2 clock).
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_sfab_ts/hmem*}]        ;# request heads -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_sfab_ts/wb*}]          ;# write data / AMO operands -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_sfab_l2/cmem*}]        ;# completions {slot, err} -> SP
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_sfab_l2/g_rb*}]        ;# read-back lines -> SP
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_sfab_l2/rold*}]        ;# AMO old values -> SP
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_smmu_ts/x_*_reg*}]     ;# SP translation request -> l2
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_smmu_l2/y_*_reg*}]     ;# translation result -> SP
set_max_delay -datapath_only 6.0 -from [get_cells -hier -filter {NAME =~ *u_sflt_ts/x_*_reg*}]     ;# LSU BAD_PHYS report -> MMU fault latch
# ---- toggles / irq level into the synchronisers (all of them carry ASYNC_REG)
set_max_delay -datapath_only 6.0 -from [all_registers] -to [get_cells -hier -filter {ASYNC_REG == TRUE}]
# ---- placement (b16): the RP pblock spans SLR0..SLR2 and the static's AXI / AXI-Lite interfaces (u_iso_m_axi_c3,
# u_iso_m_axi_host, u_skid_*) sit in SLR1.  Once the SP grew (XLANE + LSU ring) the placer moved the
# clk_l2-side blocks to SLR0, so every boundary path crossed SLR1 -> SLR0 -> SLR1 (b16: ~5.5K clk_l2 endpoints at
# -0.5 ns, 0 logic levels, ~4 ns of route); b15 had them all in SLR1.  Keep them, the CP and the SP's fabric / MMU
# bridges next to the interfaces; the SP core itself may spread (it only meets clk_l2 through the async bridges).
# Scoped by read_xdc -cells <RP>, so a plain get_cells returns the RP's top-level children: all of them except the SP
# core (g_sp.u_sp) go to SLR1.
set_property USER_SLR_ASSIGNMENT SLR1 [get_cells -filter {NAME !~ *g_sp.u_sp}]
