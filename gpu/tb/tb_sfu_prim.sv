// tb_sfu_prim.sv -- SFU arithmetic blocks vs SoftFloat / model-kernel vectors.
//
// Reads the text produced by sim/sfu/dump_prim.cpp (one vector per line:
// op fmt rm daz ftz a b c exp_res exp_fl exp_r2, hex) and drives
//   ng_sfu_fma, ng_sfu_divsqrt, the pure functions of ng_sfu_fp_pkg and ng_sfu_lane.
// Plusargs: +VEC=<file>  +MAXFAIL=<n>.  Prints NG_SFU_PRIM_ALL_OK on success.
`timescale 1ns / 1ps

module tb_sfu_prim;
  import ng_sfu_fp_pkg::*;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  // fma
  logic        fma_start = 1'b0;
  logic [63:0] fma_a = '0, fma_b = '0, fma_c = '0, fma_res;
  logic        fma_done;
  ng_sfu_fma u_fma (.clk(clk), .rst_n(rst_n), .start(fma_start), .a(fma_a), .b(fma_b), .c(fma_c),
                    .done(fma_done), .res(fma_res));

  // divsqrt
  logic        ds_start = 1'b0;
  logic [1:0]  ds_op = '0, ds_iop = '0, ds_fmt = '0;
  logic [2:0]  ds_rm = '0;
  logic        ds_daz = 1'b0, ds_ftz = 1'b0;
  logic [63:0] ds_a = '0, ds_b = '0, ds_res;
  logic [4:0]  ds_fl;
  logic        ds_done, ds_busy;
  ng_sfu_divsqrt u_ds (.clk(clk), .rst_n(rst_n), .start(ds_start), .op(ds_op), .iop(ds_iop), .fmt(ds_fmt),
                       .rm(ds_rm), .daz(ds_daz), .ftz(ds_ftz), .a(ds_a), .b(ds_b),
                       .done(ds_done), .res(ds_res), .fl(ds_fl), .busy(ds_busy));

  // lane
  logic        ln_start = 1'b0;
  logic [3:0]  ln_op = '0;
  logic [1:0]  ln_iop = '0, ln_fmt = '0;
  logic [2:0]  ln_rm = '0;
  logic        ln_daz = 1'b0, ln_ftz = 1'b0;
  logic [63:0] ln_a = '0, ln_b = '0, ln_res;
  logic [4:0]  ln_fl;
  logic        ln_done;
  ng_sfu_lane u_ln (.clk(clk), .rst_n(rst_n), .start(ln_start), .op(ln_op), .iop(ln_iop), .fmt(ln_fmt),
                    .rm(ln_rm), .daz(ln_daz), .ftz(ln_ftz), .a(ln_a), .b(ln_b),
                    .done(ln_done), .res(ln_res), .fl(ln_fl));

  int fd;
  logic [8*256-1:0] vecfile;
  int maxfail, nfail, npass, nshown;
  int per_op_pass [32];
  int per_op_fail [32];

  logic [31:0] v_op, v_fmt, v_rm, v_daz, v_ftz, v_fl;
  logic [63:0] v_a, v_b, v_c, v_r, v_r2;
  logic [63:0] got_r;
  logic [4:0]  got_fl;
  logic        chk_fl;
  int          rc, waitc;

  task automatic report(input bit ok);
    if (ok) begin npass++; per_op_pass[v_op[4:0]]++; end
    else begin
      nfail++; per_op_fail[v_op[4:0]]++;
      if (nshown < maxfail) begin
        nshown++;
        $display("FAIL op=%0d fmt=%0d rm=%0d daz=%0d ftz=%0d a=%016h b=%016h c=%016h exp=%016h/%02h got=%016h/%02h",
                 v_op, v_fmt, v_rm, v_daz, v_ftz, v_a, v_b, v_c, v_r, v_fl[4:0], got_r, got_fl);
      end
    end
  endtask

  rnd_t   cr;
  split_t sr;

  initial begin
    if (!$value$plusargs("VEC=%s", vecfile)) vecfile = "prim.txt";
    if (!$value$plusargs("MAXFAIL=%d", maxfail)) maxfail = 12;
    fd = $fopen(vecfile, "r");
    if (fd == 0) begin $display("TB_ERROR: cannot open vector file"); $finish; end
    nfail = 0; npass = 0; nshown = 0;
    for (int i = 0; i < 32; i++) begin per_op_pass[i] = 0; per_op_fail[i] = 0; end
    repeat (4) @(posedge clk);
    rst_n <= 1'b1;
    repeat (2) @(posedge clk);

    while (!$feof(fd)) begin
      rc = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h\n", v_op, v_fmt, v_rm, v_daz, v_ftz, v_a, v_b, v_c, v_r, v_fl, v_r2);
      if (rc == 11) begin
        got_r = '0; got_fl = '0; chk_fl = 1'b1;
        case (v_op)
          0: begin
            fma_a <= v_a; fma_b <= v_b; fma_c <= v_c; fma_start <= 1'b1;
            @(posedge clk); fma_start <= 1'b0;
            waitc = 0;
            while (!fma_done && waitc < 60) begin @(posedge clk); waitc++; end
            got_r = fma_res; chk_fl = 1'b0;
            report(fma_done && got_r === v_r);
          end
          1, 2, 3: begin
            ds_op <= (v_op == 1) ? 2'd0 : (v_op == 2) ? 2'd1 : 2'd2;
            ds_iop <= v_fmt[1:0]; ds_fmt <= v_fmt[1:0]; ds_rm <= v_rm[2:0];
            ds_daz <= v_daz[0]; ds_ftz <= v_ftz[0]; ds_a <= v_a; ds_b <= v_b; ds_start <= 1'b1;
            @(posedge clk); ds_start <= 1'b0;
            waitc = 0;
            while (!ds_done && waitc < 400) begin @(posedge clk); waitc++; end
            got_r = ds_res; got_fl = ds_fl;
            if (v_op == 3) begin got_r = {32'd0, ds_res[31:0]}; end
            report(ds_done && got_r === v_r && (v_op == 3 || got_fl === v_fl[4:0]));
          end
          4: begin
            cr = f64_to_fmt(v_a, v_fmt[1:0], v_rm[2:0]);
            got_r = cr.bits; got_fl = {2'b00, cr.of, cr.uf, cr.nx};
            report(got_r === v_r && got_fl === v_fl[4:0]);
          end
          5: begin
            sr = rne_split(v_a);
            got_r = sr.r; got_fl = '0;
            report(got_r === v_r && (v_r2[12] || ({52'd0, sr.n[11:0]} === v_r2)));
          end
          default: begin
            ln_op <= 4'(v_op - 16); ln_iop <= v_fmt[1:0]; ln_fmt <= v_fmt[1:0]; ln_rm <= v_rm[2:0];
            ln_daz <= v_daz[0]; ln_ftz <= v_ftz[0]; ln_a <= v_a; ln_b <= v_b; ln_start <= 1'b1;
            @(posedge clk); ln_start <= 1'b0;
            waitc = 0;
            while (!ln_done && waitc < 8000) begin @(posedge clk); waitc++; end
            got_r = ln_res; got_fl = ln_fl;
            if (v_op == 24) got_r = {32'd0, ln_res[31:0]};
            report(ln_done && got_r === v_r && (v_op == 24 || got_fl === v_fl[4:0]));
          end
        endcase
        @(posedge clk);
      end
    end
    for (int i = 0; i < 32; i++)
      if (per_op_pass[i] + per_op_fail[i] != 0)
        $display("op %0d: pass=%0d fail=%0d", i, per_op_pass[i], per_op_fail[i]);
    $display("NG_SFU_PRIM pass=%0d fail=%0d", npass, nfail);
    if (nfail == 0 && npass > 0) $display("NG_SFU_PRIM_ALL_OK");
    else $display("NG_SFU_PRIM_FAILED");
    $finish;
  end

  initial begin
    #2000000000;
    $display("TB_ERROR: global timeout");
    $finish;
  end
endmodule
