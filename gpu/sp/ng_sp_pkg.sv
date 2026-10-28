// ng_sp_pkg.sv -- shared constants of the SP top level and the work
// distributor.  tentative as a whole.
package ng_sp_pkg;

  // Execution units: issue routing targets and trap source ids
  // (dbg_trap_unit of ng_sp_top).
  localparam logic [2:0] UNIT_SCHED = 3'd0;  // scheduler-resolved (fetch fault, illegal, trap/brk)
  localparam logic [2:0] UNIT_VALU  = 3'd1;
  localparam logic [2:0] UNIT_LSU   = 3'd2;
  localparam logic [2:0] UNIT_SALU  = 3'd3;  // SALU / branch / CSR (ng_salu)
  localparam logic [2:0] UNIT_SFU   = 3'd4;
  localparam logic [2:0] UNIT_XLANE = 3'd5;
  localparam logic [2:0] UNIT_FP64  = 3'd6;
  localparam int         NUNIT      = 7;

endpackage
