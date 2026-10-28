`timescale 1ns / 1ps

// Three-byte I2C write transaction with open-drain SCL/SDA pins.
//
//   go: start pulse
//   data: {byte2, byte1, byte0}
//
// The bus is bit-banged at roughly 100 kHz from the CFGMCLK input. The module
// always emits START and STOP, so it can be used for both SI5381 page writes
// and normal register writes.

module si_i2c_write3 (
    input  wire       clk,
    input  wire       go,
    input  wire [23:0] data,
    output reg        done,
    output reg        err,
    inout  wire       scl,
    inout  wire       sda
);

    localparam [15:0] HALF_PERIOD = 16'd486; // ~10 us at 48.585 MHz

    localparam IDLE       = 4'd0;
    localparam START_1    = 4'd1;
    localparam START_2    = 4'd2;
    localparam START_3    = 4'd3;
    localparam BYTE_SET   = 4'd4;
    localparam BYTE_HIGH  = 4'd5;
    localparam BYTE_LOW   = 4'd6;
    localparam ACK_HIGH   = 4'd7;
    localparam ACK_LOW    = 4'd8;
    localparam NEXT_BYTE  = 4'd9;
    localparam STOP_1     = 4'd10;
    localparam STOP_2     = 4'd11;
    localparam STOP_3     = 4'd12;
    localparam DONE_ST    = 4'd13;

    reg [3:0]  state = IDLE;
    reg [15:0] timer = 16'd0;
    reg [3:0]  byte_idx = 3'd0;
    reg [2:0]  bit_idx = 3'd7;
    reg [7:0]  cur_byte = 8'h00;
    reg        scl_oe = 1'b0;
    reg        sda_oe = 1'b0;
    reg        ack_ok = 1'b1;

    wire sda_in;
    wire sda_level = sda_in;

    IOBUF scl_iob (.I(1'b0), .T(~scl_oe), .O(), .IO(scl));
    IOBUF sda_iob (.I(1'b0), .T(~sda_oe), .O(sda_in), .IO(sda));

    always @(posedge clk) begin
        done <= 1'b0;
        err  <= 1'b0;

        case (state)
            IDLE: begin
                scl_oe <= 1'b0;
                sda_oe <= 1'b0;
                if (go) begin
                    byte_idx <= 3'd0;
                    bit_idx  <= 3'd7;
                    ack_ok   <= 1'b1;
                    state    <= START_1;
                end
            end

            START_1: begin
                // SCL is high from idle; pull SDA low to create START.
                sda_oe <= 1'b1;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= START_2;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            START_2: begin
                scl_oe <= 1'b1;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= START_3;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            START_3: begin
                scl_oe <= 1'b1;
                cur_byte <= data[23:16];
                bit_idx  <= 3'd7;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= BYTE_SET;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            BYTE_SET: begin
                sda_oe <= ~cur_byte[bit_idx];
                if (timer == 1) begin
                    timer <= 16'd0;
                    state <= BYTE_HIGH;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            BYTE_HIGH: begin
                scl_oe <= 1'b0;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= BYTE_LOW;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            BYTE_LOW: begin
                scl_oe <= 1'b1;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    if (bit_idx == 3'd0) begin
                        state <= ACK_HIGH;
                    end else begin
                        bit_idx <= bit_idx - 3'd1;
                        state <= BYTE_SET;
                    end
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            ACK_HIGH: begin
                // Release SDA and sample the slave ACK while SCL is high.
                sda_oe <= 1'b0;
                scl_oe <= 1'b0;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    ack_ok <= ~sda_level;
                    state  <= ACK_LOW;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            ACK_LOW: begin
                scl_oe <= 1'b1;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    if (!ack_ok) begin
                        state <= DONE_ST;
                        err   <= 1'b1;
                    end else if (byte_idx == 3'd2) begin
                        state <= STOP_1;
                    end else begin
                        if (byte_idx == 3'd0) begin
                            cur_byte <= data[15:8];
                        end else begin
                            cur_byte <= data[7:0];
                        end
                        byte_idx <= byte_idx + 3'd1;
                        bit_idx  <= 3'd7;
                        state    <= BYTE_SET;
                    end
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            STOP_1: begin
                sda_oe <= 1'b1;
                scl_oe <= 1'b0;
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= STOP_2;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            STOP_2: begin
                scl_oe <= 1'b0; // SCL high
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= STOP_3;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            STOP_3: begin
                sda_oe <= 1'b0; // SDA high
                if (timer == HALF_PERIOD) begin
                    timer <= 16'd0;
                    state <= DONE_ST;
                end else begin
                    timer <= timer + 16'd1;
                end
            end

            DONE_ST: begin
                scl_oe <= 1'b0;
                sda_oe <= 1'b0;
                done   <= 1'b1;
                state  <= IDLE;
            end
        endcase
    end

endmodule
