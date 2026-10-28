`timescale 1ns / 1ps

// In-fabric SI5381 bring-up (ported verbatim from the board-verified
// work/ddr_siclk design; see C:/VU9P-rescue-20260826/work/ddr_siclk).
//
// This module replays the ClockBuilder Pro register export into the
// synthesizer, including the 625 ms calibration-drain delay, then waits a
// fixed grace period before releasing the DDR4 MIG reset. The whole sequence
// is driven by CFGMCLK, so it works before the SI5381 output clocks exist.
// Only change in this port: the ROM image is a parameter.  The default is
// si_config_out4c.mem -- the register set that moved the C3 reference
// (OUT4) onto DSPLL C and made all four DDR channels calibrate on the
// board.  si_config.mem (the raw ClockBuilder Pro export) is kept next to
// it for reference.

module si_init #(
    parameter SI_MEM = "si_config_out4c.mem"
) (
    input  wire       clk,
    output reg        rstn,
    output reg        done,
    output reg        err,
    inout  wire       scl,
    inout  wire       sda,
    inout  wire       csb
);

    localparam [23:0] SI_ADDR_DATA = {8'hD6, 8'h00, 8'h00}; // 0x6b << 1, write
    localparam [31:0] RESET_WAIT   = 32'd5_000_000;         // ~100 ms
    localparam [31:0] DELAY_625MS  = 32'd30_000_000;        // ~625 ms
    localparam [31:0] SETTLE_WAIT  = 32'd500_000_000;       // ~10 s

    localparam IDLE           = 4'd0;
    localparam INIT_WAIT      = 4'd1;
    localparam LOAD           = 4'd2;
    localparam PAGE_GO        = 4'd3;
    localparam PAGE_WAIT      = 4'd4;
    localparam REG_GO         = 4'd5;
    localparam REG_WAIT       = 4'd6;
    localparam DELAY_WAIT     = 4'd7;
    localparam FIXUP_PAGE_GO  = 4'd8;
    localparam FIXUP_PAGE_WAIT = 4'd9;
    localparam FIXUP_REG_GO   = 4'd10;
    localparam FIXUP_REG_WAIT = 4'd11;
    localparam SETTLE         = 4'd12;
    localparam DONE_ST        = 4'd13;
    localparam ERROR_ST       = 4'd14;

    reg [23:0] rom [0:1023];
    // The ROM image is selectable at synthesis time so an experiment can swap
    // the SI5381 register table without forking this file. The default branch
    // is unchanged, so every existing build (C1/C2/C4, ax45mpv) stays byte
    // identical to before.
    //
    // SI_CONFIG_MEM_OUT4C moves OUT4 -- the DDR4 C3 reference clock
    // DSPLL B onto DSPLL C. DSPLL B is the only PLL the CBPro export leaves in
    // holdover (it assigns IN0..IN3, which this board does not wire), and it is
    // the reference for C3, one of the two channels that fails. DSPLL C already
    // feeds OUT5 and therefore the working C2 channel. Generated and guarded by
    // gen_si_config_out4c.py; validated live over the JTAG I2C bridge before
    // being baked in here -- OUT4 still measured 299.939 MHz afterwards.
    initial $readmemh(SI_MEM, rom);

    reg [10:0] idx = 11'd0;
    reg [3:0]  state = IDLE;
    reg [31:0] wait_cnt = 32'd0;
    reg [7:0]  cur_page = 8'hFF;
    reg [7:0]  tx_page = 8'h00;
    reg [7:0]  tx_reg  = 8'h00;
    reg [7:0]  tx_val  = 8'h00;
    reg [1:0]  fixup_idx = 2'd0;
    reg        i2c_go = 1'b0;
    wire       i2c_done;
    wire       i2c_err;

    wire [23:0] entry = rom[idx];
    wire [7:0]  entry_page = entry[23:16];
    wire [7:0]  entry_reg  = entry[15:8];
    wire [7:0]  entry_val  = entry[7:0];
    wire [23:0] tx_data = (state == PAGE_GO || state == FIXUP_PAGE_GO) ?
                          {8'hD6, 8'h01, tx_page} :
                          {8'hD6, tx_reg, tx_val};

    IOBUF csb_iob (.I(1'b0), .T(1'b1), .O(), .IO(csb));

    si_i2c_write3 i2c (
        .clk  (clk),
        .go   (i2c_go),
        .data (tx_data),
        .done (i2c_done),
        .err  (i2c_err),
        .scl  (scl),
        .sda  (sda)
    );

    always @(posedge clk) begin
        done <= 1'b0;
        err  <= 1'b0;

        case (state)
            IDLE: begin
                rstn <= 1'b1;
                i2c_go <= 1'b0;
                wait_cnt <= 32'd0;
                state <= INIT_WAIT;
            end

            INIT_WAIT: begin
                if (wait_cnt == RESET_WAIT) begin
                    wait_cnt <= 32'd0;
                    state <= LOAD;
                end else begin
                    wait_cnt <= wait_cnt + 32'd1;
                end
            end

            LOAD: begin
                // si_config.mem holds 842 lines: 841 register writes plus the
                // 0xFF delay marker. Stopping at 841 dropped the last entry,
                // 0x0B25=0x06, which is half of the CBPro postamble pair.
                if (idx >= 11'd842) begin
                    fixup_idx <= 2'd0;
                    state <= FIXUP_PAGE_GO;
                end else if (entry_page == 8'hFF) begin
                    idx <= idx + 11'd1;
                    wait_cnt <= 32'd0;
                    state <= DELAY_WAIT;
                end else begin
                    tx_page <= entry_page;
                    tx_reg  <= entry_reg;
                    tx_val  <= entry_val;
                    if (entry_page != cur_page) begin
                        cur_page <= entry_page;
                        state <= PAGE_GO;
                    end else begin
                        state <= REG_GO;
                    end
                end
            end

            PAGE_GO: begin
                tx_reg <= 8'h01;
                tx_val <= tx_page;
                i2c_go <= 1'b1;
                state  <= PAGE_WAIT;
            end

            PAGE_WAIT: begin
                i2c_go <= 1'b0;
                if (i2c_done) begin
                    if (i2c_err) begin
                        state <= ERROR_ST;
                    end else begin
                        // The page byte is only a prefix, not the entry
                        // itself: this ROM entry still owes a register
                        // write. Advancing idx here dropped one register per
                        // page transition (19 of them, including the
                        // 0x0B24=0xC0 preamble, 0x0B24=0xC3 postamble and
                        // 0x0102=0x01 output enable) while every I2C
                        // transaction still ACKed, so err/done looked clean
                        // and the part stayed unconfigured. PAGE_GO clobbered
                        // tx_reg/tx_val, so reload them from the current --
                        // still unadvanced -- ROM entry.
                        tx_reg <= entry_reg;
                        tx_val <= entry_val;
                        state  <= REG_GO;
                    end
                end
            end

            REG_GO: begin
                i2c_go <= 1'b1;
                state  <= REG_WAIT;
            end

            REG_WAIT: begin
                i2c_go <= 1'b0;
                if (i2c_done) begin
                    if (i2c_err) begin
                        state <= ERROR_ST;
                    end else begin
                        idx <= idx + 11'd1;
                        state <= LOAD;
                    end
                end
            end

            DELAY_WAIT: begin
                if (wait_cnt == DELAY_625MS) begin
                    wait_cnt <= 32'd0;
                    state <= LOAD;
                end else begin
                    wait_cnt <= wait_cnt + 32'd1;
                end
            end

            FIXUP_PAGE_GO: begin
                // tx_data is {addr, tx_reg, tx_val}, so a page write has to
                // load tx_reg=0x01 and tx_val=<page> exactly like PAGE_GO
                // does. Setting only tx_page left tx_reg/tx_val holding the
                // last table entry, so this re-sent that entry instead of
                // switching page -- and both board fixups then landed on
                // page 0x0B. Losing 0x090E=0x02 leaves XAXB in XO mode on a
                // board that has a crystal, so LOSXAXB stays 1, every DSPLL
                // is referenceless and all four outputs are dead.
                tx_reg <= 8'h01;
                if (fixup_idx == 2'd0) begin
                    tx_page <= 8'h09;
                    tx_val  <= 8'h09;
                end else begin
                    tx_page <= 8'h00;
                    tx_val  <= 8'h00;
                end
                i2c_go <= 1'b1;
                state  <= FIXUP_PAGE_WAIT;
            end

            FIXUP_PAGE_WAIT: begin
                i2c_go <= 1'b0;
                if (i2c_done) begin
                    if (i2c_err) begin
                        state <= ERROR_ST;
                    end else begin
                        if (fixup_idx == 2'd0) begin
                            tx_reg <= 8'h0E;
                            tx_val <= 8'h02;
                        end else begin
                            tx_reg <= 8'h16;
                            tx_val <= 8'h00;
                        end
                        state <= FIXUP_REG_GO;
                    end
                end
            end

            FIXUP_REG_GO: begin
                i2c_go <= 1'b1;
                state  <= FIXUP_REG_WAIT;
            end

            FIXUP_REG_WAIT: begin
                i2c_go <= 1'b0;
                if (i2c_done) begin
                    if (i2c_err) begin
                        state <= ERROR_ST;
                    end else if (fixup_idx == 2'd1) begin
                        wait_cnt <= 32'd0;
                        state <= SETTLE;
                    end else begin
                        fixup_idx <= fixup_idx + 2'd1;
                        state <= FIXUP_PAGE_GO;
                    end
                end
            end

            SETTLE: begin
                if (wait_cnt == SETTLE_WAIT) begin
                    state <= DONE_ST;
                end else begin
                    wait_cnt <= wait_cnt + 32'd1;
                end
            end

            DONE_ST: begin
                done <= 1'b1;
                state <= DONE_ST;
            end

            ERROR_ST: begin
                err <= 1'b1;
                state <= ERROR_ST;
            end
        endcase
    end

endmodule
