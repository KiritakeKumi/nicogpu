// ng_valu_fp.sv -- f32 execution engine of the VALU: NFL physical lanes sweep the
// NL lanes of one register-file beat in NL/NFL groups.
//
// `start` pulses once the operand vectors a_v/b_v/c_v are stable; the operands are
// only sampled during the NG issue cycles that follow `start` (cycle k = 1..NG reads
// group k-1), so the caller may overwrite them right after the last issue cycle.
// The engine feeds one lane group per cycle into the NFL ng_valu_fp_lane pipelines
// (latency LAT = 7), collects the results into res_v / cmp_v and ORs the fflags of
// the lanes selected by exec_sl (the mask of the beat at the OUTPUT end of the
// pipeline).  `done` is a one-cycle pulse; res_v / cmp_v / flags hold the complete
// beat in the done cycle only (a following beat may start as soon as `can_start`,
// i.e. while this beat is still in the pipeline, and overwrites them one cycle
// later).  flags is cleared by the first group of a beat, not by `start`.
// consecutive beats may belong to different instructions.  The control
// inputs (fop, rm, daz, ...) are sampled per issue cycle and travel down the lane
// pipelines; `tag_i` (sampled at `start`) travels with every group of the beat and
// is visible at the output stage (tag_o, the beat whose groups are at the output)
// and with done (dtag_o); the caller derives exec_sl from it.
module ng_valu_fp
  import ng_valu_fp_pkg::*;
#(
  parameter int NL  = 16,
  parameter int NFL = 4,       // physical lanes (divides NL)
  parameter int TGW = 1,       // beat tag width
  parameter bit EN_F16 = 1'b0, // binary16 <-> binary32 conversions
  localparam int GWO = ng_noc_pkg::clog2_1(NL / NFL) + 1
) (
  input  logic              clk,
  input  logic              rst_n,
  input  logic              start,
  input  logic [5:0]        fop,
  input  logic [4:0]        pred,
  input  logic [2:0]        rm,
  input  logic [1:0]        ub,
  input  logic              daz,
  input  logic              ftz,
  input  logic              daz16,       // MODE.DAZ16 / FTZ16 (binary16 conversions)
  input  logic              ftz16,
  input  logic              sat,
  input  logic [2:0]        neg,
  input  logic [2:0]        abs_m,
  input  logic [NL*32-1:0]  a_v,
  input  logic [NL*32-1:0]  b_v,
  input  logic [NL*32-1:0]  c_v,
  input  logic [NL-1:0]     exec_sl,
  input  logic [TGW-1:0]    tag_i,       // tag of the beat that `start` launches
  output logic [TGW-1:0]    tag_o,       // comb: tag of the beat whose group is at the output stage
  output logic [TGW-1:0]    dtag_o,      // tag of the beat completed by `done` (valid with done)
  output logic              done,
  output logic              busy,
  output logic              last_out,    // comb: the last group of a beat is at the output stage (done is raised at its end)
  output logic              can_start,   // no group left to issue (or issuing the last one): start may pulse
  output logic [GWO-1:0]    gi_o,        // issue counter (valid while a beat is issuing)
  output logic [NL*32-1:0]  res_v,
  output logic [NL-1:0]     cmp_v,
  output logic [4:0]        flags
);
  localparam int LAT = 7;
  localparam int NG  = NL / NFL;
  localparam int GW  = ng_noc_pkg::clog2_1(NG);

  logic            run;
  logic [GW:0]     gi;
  logic            issue;
  logic [LAT-1:0]  pv;
  logic [GW-1:0]   pg [LAT];
  logic [TGW-1:0]  tg_q;
  logic [TGW-1:0]  pt [LAT];

  assign issue = run;
  assign busy  = run || (|pv);
  assign last_out = pv[LAT-1] && (int'(pg[LAT-1]) == NG - 1);
  assign can_start = !run || (int'(gi) == NG - 1);
  assign gi_o = gi;
  assign tag_o = pt[LAT-1];

  logic [31:0] ln_a [NFL], ln_b [NFL], ln_c [NFL], ln_res [NFL];
  logic [4:0]  ln_fl [NFL];
  logic        ln_cb [NFL];

  for (genvar j = 0; j < NFL; j++) begin : g_lane
    always_comb begin
      ln_a[j] = a_v[(int'(gi[GW-1:0]) * NFL + j) * 32 +: 32];
      ln_b[j] = b_v[(int'(gi[GW-1:0]) * NFL + j) * 32 +: 32];
      ln_c[j] = c_v[(int'(gi[GW-1:0]) * NFL + j) * 32 +: 32];
    end
    ng_valu_fp_lane #(.EN_F16(EN_F16)) u_lane (
      .clk(clk), .fop(fop), .pred(pred), .rm(rm), .ub(ub), .daz(daz), .ftz(ftz), .daz16(daz16), .ftz16(ftz16), .sat(sat),
      .neg(neg), .abs_m(abs_m), .a(ln_a[j]), .b(ln_b[j]), .c(ln_c[j]),
      .res(ln_res[j]), .fl(ln_fl[j]), .cbit(ln_cb[j])
    );
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      run <= 1'b0; gi <= '0; pv <= '0; done <= 1'b0; flags <= 5'd0;
    end else begin
      done <= 1'b0;
      pv   <= {pv[LAT-2:0], issue};
      pg[0] <= gi[GW-1:0];
      for (int k = 1; k < LAT; k++) pg[k] <= pg[k-1];
      pt[0] <= tg_q;
      for (int k = 1; k < LAT; k++) pt[k] <= pt[k-1];
      if (start) tg_q <= tag_i;
      if (start) begin
        run <= 1'b1; gi <= '0;
      end else if (run) begin
        if (int'(gi) == NG - 1) run <= 1'b0;
        else gi <= gi + 1'b1;
      end
      if (pv[LAT-1]) begin
        logic [4:0] fo;
        fo = (int'(pg[LAT-1]) == 0) ? 5'd0 : flags;
        for (int j = 0; j < NFL; j++) begin
          if (exec_sl[int'(pg[LAT-1]) * NFL + j]) fo = fo | ln_fl[j];
        end
        flags <= fo;
        if (int'(pg[LAT-1]) == NG - 1) begin done <= 1'b1; dtag_o <= pt[LAT-1]; end
      end
    end
  end

  always_ff @(posedge clk) begin
    if (pv[LAT-1]) begin
      for (int j = 0; j < NFL; j++) begin
        res_v[(int'(pg[LAT-1]) * NFL + j) * 32 +: 32] <= ln_res[j];
        cmp_v[int'(pg[LAT-1]) * NFL + j] <= ln_cb[j];
      end
    end
  end
endmodule
