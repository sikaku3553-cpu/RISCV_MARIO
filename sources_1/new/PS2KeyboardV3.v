`timescale 1ns/1ps
`default_nettype none

// Receive-only PS/2 keyboard interface. Both external wires pass through
// two flip-flops and an eight-sample digital filter before falling edges are
// accepted. The output is a held-key bitmap suitable for polling through MMIO.
module PS2KeyboardV3 #(
    // A complete PS/2 frame takes at most about 1.1 ms.  Requiring 2 ms of
    // idle-high bus time after reset or a malformed frame allows a
    // receiver released in the middle of keyboard traffic to start again at a
    // real start bit instead of remaining permanently one or more bits out of
    // phase.  Tests override this value to keep simulation short.
    parameter integer IDLE_RECOVERY_CYCLES = 200_000
) (
    input  wire       clk,
    input  wire       reset,
    input  wire       ps2_clk,
    input  wire       ps2_data,
    output reg  [4:0] keys,
    output reg  [7:0] last_scan_code,
    output reg        scan_valid,
    output reg        parity_error
);
    localparam integer KEY_LEFT    = 0;
    localparam integer KEY_RIGHT   = 1;
    localparam integer KEY_JUMP    = 2;
    localparam integer KEY_DOWN    = 3;
    localparam [1:0] RX_IDLE    = 2'd0;
    localparam [1:0] RX_FRAME   = 2'd1;
    localparam [1:0] RX_RECOVER = 2'd2;
    localparam integer RECOVERY_BITS =
        (IDLE_RECOVERY_CYCLES <= 1) ? 1 : $clog2(IDLE_RECOVERY_CYCLES);

    (* ASYNC_REG = "TRUE" *) reg ps2_clk_meta;
    (* ASYNC_REG = "TRUE" *) reg ps2_clk_sync;
    (* ASYNC_REG = "TRUE" *) reg ps2_data_meta;
    (* ASYNC_REG = "TRUE" *) reg ps2_data_sync;
    reg [7:0] clk_filter;
    reg [7:0] data_filter;
    reg filtered_clk;
    reg filtered_data;
    reg filtered_clk_d;

    reg [1:0] rx_state;
    reg [3:0] bit_count;
    reg [7:0] receive_byte;
    reg       parity_bit;
    reg       extended_code;
    reg       break_code;
    reg [RECOVERY_BITS-1:0] idle_counter;
    reg [RECOVERY_BITS-1:0] frame_counter;

    wire ps2_falling = filtered_clk_d && !filtered_clk;
    wire odd_parity_ok = ^{receive_byte, parity_bit};

    always @(posedge clk) begin
        if (reset) begin
            ps2_clk_meta  <= 1'b1;
            ps2_clk_sync  <= 1'b1;
            ps2_data_meta <= 1'b1;
            ps2_data_sync <= 1'b1;
            clk_filter    <= 8'hff;
            data_filter   <= 8'hff;
            filtered_clk  <= 1'b1;
            filtered_data <= 1'b1;
            filtered_clk_d <= 1'b1;
        end else begin
            ps2_clk_meta  <= ps2_clk;
            ps2_clk_sync  <= ps2_clk_meta;
            ps2_data_meta <= ps2_data;
            ps2_data_sync <= ps2_data_meta;

            clk_filter  <= {clk_filter[6:0], ps2_clk_sync};
            data_filter <= {data_filter[6:0], ps2_data_sync};
            if (&clk_filter)
                filtered_clk <= 1'b1;
            else if (~|clk_filter)
                filtered_clk <= 1'b0;
            if (&data_filter)
                filtered_data <= 1'b1;
            else if (~|data_filter)
                filtered_data <= 1'b0;
            filtered_clk_d <= filtered_clk;
        end
    end

    always @(posedge clk) begin
        scan_valid <= 1'b0;

        if (reset) begin
            // Power-up traffic from the USB-HID bridge may already be in
            // progress when the FPGA leaves reset.  Wait for a genuine idle
            // interval before accepting a start bit.
            rx_state         <= RX_RECOVER;
            bit_count        <= 4'd0;
            receive_byte     <= 8'd0;
            parity_bit       <= 1'b0;
            extended_code    <= 1'b0;
            break_code       <= 1'b0;
            idle_counter     <= {RECOVERY_BITS{1'b0}};
            frame_counter    <= {RECOVERY_BITS{1'b0}};
            keys             <= 5'd0;
            last_scan_code   <= 8'd0;
            parity_error     <= 1'b0;
        end else begin
            case (rx_state)
                RX_RECOVER: begin
                    bit_count     <= 4'd0;
                    frame_counter <= {RECOVERY_BITS{1'b0}};
                    // Both open-collector wires must remain released.  Any
                    // activity restarts the qualification interval.
                    if (!filtered_clk || !filtered_data) begin
                        idle_counter <= {RECOVERY_BITS{1'b0}};
                    end else if (idle_counter ==
                                 IDLE_RECOVERY_CYCLES - 1) begin
                        idle_counter <= {RECOVERY_BITS{1'b0}};
                        rx_state     <= RX_IDLE;
                    end else begin
                        idle_counter <= idle_counter + 1'b1;
                    end
                end

                RX_IDLE: begin
                    bit_count <= 4'd0;
                    frame_counter <= {RECOVERY_BITS{1'b0}};
                    // Ignore falling edges whose data line is high.  This is
                    // essential when reset is released near existing traffic.
                    if (ps2_falling && !filtered_data) begin
                        bit_count     <= 4'd1;
                        receive_byte  <= 8'd0;
                        parity_bit    <= 1'b0;
                        rx_state      <= RX_FRAME;
                    end
                end

                default: begin // RX_FRAME
                    if (!ps2_falling) begin
                        // A truncated frame must not leave the decoder wedged.
                        if (frame_counter ==
                            IDLE_RECOVERY_CYCLES - 1) begin
                            rx_state      <= RX_RECOVER;
                            bit_count     <= 4'd0;
                            idle_counter  <= {RECOVERY_BITS{1'b0}};
                            frame_counter <= {RECOVERY_BITS{1'b0}};
                            extended_code <= 1'b0;
                            break_code    <= 1'b0;
                            keys          <= 5'd0;
                            parity_error  <= 1'b1;
                        end else begin
                            frame_counter <= frame_counter + 1'b1;
                        end
                    end else begin
                        frame_counter <= {RECOVERY_BITS{1'b0}};
                        case (bit_count)
                            4'd1, 4'd2, 4'd3, 4'd4,
                            4'd5, 4'd6, 4'd7, 4'd8: begin
                                receive_byte[bit_count - 1]
                                    <= filtered_data;
                                bit_count <= bit_count + 4'd1;
                            end
                            4'd9: begin
                                parity_bit <= filtered_data;
                                bit_count  <= 4'd10;
                            end
                            default: begin
                                bit_count <= 4'd0;
                                if (filtered_data && odd_parity_ok) begin
                                    rx_state         <= RX_IDLE;
                                    parity_error     <= 1'b0;
                                    last_scan_code   <= receive_byte;
                                    scan_valid       <= 1'b1;
                                    if (receive_byte == 8'he0) begin
                                        extended_code <= 1'b1;
                                    end else if (receive_byte == 8'hf0) begin
                                        break_code <= 1'b1;
                                    end else begin
                                        // Requested control scheme, PS/2
                                        // scan-code set 2.  Extended arrows
                                        // and W/R aliases are intentionally
                                        // ignored so the mapping is exact.
                                        if (!extended_code) begin
                                            case (receive_byte)
                                                8'h1c:
                                                    keys[KEY_LEFT]
                                                        <= !break_code; // A
                                                8'h23:
                                                    keys[KEY_RIGHT]
                                                        <= !break_code; // D
                                                8'h29:
                                                    keys[KEY_JUMP]
                                                        <= !break_code; // Space
                                                8'h1b:
                                                    keys[KEY_DOWN]
                                                        <= !break_code; // S, pipe entry
                                                default: ;
                                            endcase
                                        end
                                        extended_code <= 1'b0;
                                        break_code    <= 1'b0;
                                    end
                                end else begin
                                    // Drop every prefix and held key on a bad
                                    // frame.  Waiting for an idle interval
                                    // prevents a corrupted break code from
                                    // leaving movement stuck on.
                                    rx_state         <= RX_RECOVER;
                                    idle_counter     <= {RECOVERY_BITS{1'b0}};
                                    extended_code    <= 1'b0;
                                    break_code       <= 1'b0;
                                    keys             <= 5'd0;
                                    parity_error     <= 1'b1;
                                end
                            end
                        endcase
                    end
                end
            endcase
        end
    end
endmodule

`default_nettype wire
