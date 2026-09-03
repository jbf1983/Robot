// -----------------------------------------------------------------------------
// Dabble/GamePad parser with iOS-safe digital-button + analog-joystick support.
//
// Dabble frame format used by the Arduino library:
//   FF  MODULE  FUNCTION  ARG_COUNT  ARG_LEN  ARG_DATA...  00
// This bring-up accepts one argument (the GamePad frames use one 2-byte arg).
//
// GamePad:
//   module   = 0x01
//   digital  = 0x01 (buttons; accepted for iOS joystick-mode button events)
//   analog   = 0x02
//   data[0]  = digital buttons
//   data[1]  = AAAAA RRR
//              AAAAA * 15 degrees = joystick angle
//              RRR = radius 0..7
//
// Board-identification request (module 0, function 3) receives a minimal
// "Other board / library 1.5.2" response so Dabble can complete its normal
// discovery exchange. No robot command is produced in this version.
// -----------------------------------------------------------------------------
module dabble_bringup #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer ALIVE_MS = 500,
    parameter integer FRAME_TIMEOUT_MS = 50
)(
    input  logic       clk,
    input  logic       rst,
    input  logic [7:0] rx_byte,
    input  logic       rx_valid,
    input  logic       rx_framing_error,

    input  logic       tx_busy,
    input  logic       tx_done,
    output logic       tx_start,
    output logic [7:0] tx_byte,

    output logic       uart_seen,
    output logic [7:0] last_uart_byte,
    output logic       gamepad_seen,
    output logic       gamepad_alive,
    output logic       gamepad_frame_pulse,
    output logic [8:0] joystick_angle_deg,
    output logic [2:0] joystick_radius,
    output logic [7:0] gamepad_buttons,
    output logic [15:0] gamepad_frame_count,
    output logic [15:0] parser_error_count,
    output logic [15:0] uart_error_count
);
    localparam integer ALIVE_CYCLES = (CLK_HZ / 1000) * ALIVE_MS;
    localparam integer ALIVE_W = (ALIVE_CYCLES <= 2) ? 1 : $clog2(ALIVE_CYCLES + 1);
    localparam integer TIMEOUT_CYCLES = (CLK_HZ / 1000) * FRAME_TIMEOUT_MS;
    localparam integer TIMEOUT_W = (TIMEOUT_CYCLES <= 2) ? 1 : $clog2(TIMEOUT_CYCLES + 1);

    typedef enum logic [2:0] {
        P_WAIT_FF,
        P_MODULE,
        P_FUNCTION,
        P_ARGCOUNT,
        P_ARGLEN,
        P_DATA,
        P_END
    } parser_state_t;

    parser_state_t pstate;

    logic [7:0] module_id;
    logic [7:0] function_id;
    logic [7:0] arg_count;
    logic [7:0] arg_len;
    logic [7:0] payload [0:7];
    logic [3:0] payload_index;

    logic [ALIVE_W-1:0] alive_count;
    logic [TIMEOUT_W-1:0] timeout_count;

    logic reply_pending;
    logic reply_active;
    logic reply_sending;
    logic [3:0] reply_index;

    function automatic logic [7:0] board_reply_byte(input logic [3:0] idx);
        case (idx)
            4'd0: board_reply_byte = 8'hFF; // SOF
            4'd1: board_reply_byte = 8'h00; // Dabble module
            4'd2: board_reply_byte = 8'h03; // board ID response
            4'd3: board_reply_byte = 8'h01; // one argument
            4'd4: board_reply_byte = 8'h04; // argument length
            4'd5: board_reply_byte = 8'h05; // "Other" board
            4'd6: board_reply_byte = 8'h01; // library 1
            4'd7: board_reply_byte = 8'h05; // library .5
            4'd8: board_reply_byte = 8'h02; // library .2
            4'd9: board_reply_byte = 8'h00; // EOF
            default: board_reply_byte = 8'h00;
        endcase
    endfunction

    integer k;
    always_ff @(posedge clk) begin
        if (rst) begin
            pstate               <= P_WAIT_FF;
            module_id            <= 8'd0;
            function_id          <= 8'd0;
            arg_count            <= 8'd0;
            arg_len              <= 8'd0;
            payload_index        <= 4'd0;
            uart_seen            <= 1'b0;
            last_uart_byte       <= 8'd0;
            gamepad_seen         <= 1'b0;
            gamepad_alive        <= 1'b0;
            gamepad_frame_pulse  <= 1'b0;
            joystick_angle_deg   <= 9'd0;
            joystick_radius      <= 3'd0;
            gamepad_buttons      <= 8'd0;
            gamepad_frame_count  <= 16'd0;
            parser_error_count   <= 16'd0;
            uart_error_count     <= 16'd0;
            alive_count          <= '0;
            timeout_count        <= '0;
            reply_pending        <= 1'b0;
            reply_active         <= 1'b0;
            reply_sending        <= 1'b0;
            reply_index          <= 4'd0;
            tx_start             <= 1'b0;
            tx_byte              <= 8'h00;
            for (k = 0; k < 8; k = k + 1)
                payload[k] <= 8'd0;
        end else begin
            gamepad_frame_pulse <= 1'b0;
            tx_start            <= 1'b0;

            // GamePad link-alive watchdog.
            if (alive_count != 0) begin
                alive_count   <= alive_count - 1'b1;
                gamepad_alive <= 1'b1;
            end else begin
                gamepad_alive <= 1'b0;
            end

            // Parser timeout: a partial frame must not hold the decoder forever.
            if (pstate != P_WAIT_FF) begin
                if (rx_valid) begin
                    timeout_count <= TIMEOUT_CYCLES;
                end else if (timeout_count != 0) begin
                    timeout_count <= timeout_count - 1'b1;
                end else begin
                    pstate <= P_WAIT_FF;
                    if (parser_error_count != 16'hFFFF)
                        parser_error_count <= parser_error_count + 1'b1;
                end
            end else begin
                timeout_count <= '0;
            end

            if (rx_framing_error && uart_error_count != 16'hFFFF)
                uart_error_count <= uart_error_count + 1'b1;

            if (rx_valid) begin
                uart_seen      <= 1'b1;
                last_uart_byte <= rx_byte;

                case (pstate)
                    P_WAIT_FF: begin
                        if (rx_byte == 8'hFF) begin
                            pstate        <= P_MODULE;
                            payload_index <= 4'd0;
                            timeout_count <= TIMEOUT_CYCLES;
                        end
                    end

                    P_MODULE: begin
                        module_id <= rx_byte;
                        pstate    <= P_FUNCTION;
                    end

                    P_FUNCTION: begin
                        function_id <= rx_byte;
                        pstate      <= P_ARGCOUNT;
                    end

                    P_ARGCOUNT: begin
                        arg_count <= rx_byte;
                        if (rx_byte == 8'd0) begin
                            arg_len <= 8'd0;
                            pstate  <= P_END;
                        end else if (rx_byte == 8'd1) begin
                            pstate <= P_ARGLEN;
                        end else begin
                            pstate <= P_WAIT_FF;
                            if (parser_error_count != 16'hFFFF)
                                parser_error_count <= parser_error_count + 1'b1;
                        end
                    end

                    P_ARGLEN: begin
                        arg_len       <= rx_byte;
                        payload_index <= 4'd0;
                        if (rx_byte == 8'd0) begin
                            pstate <= P_END;
                        end else if (rx_byte <= 8'd8) begin
                            pstate <= P_DATA;
                        end else begin
                            pstate <= P_WAIT_FF;
                            if (parser_error_count != 16'hFFFF)
                                parser_error_count <= parser_error_count + 1'b1;
                        end
                    end

                    P_DATA: begin
                        payload[payload_index] <= rx_byte;
                        if ((payload_index + 1'b1) >= arg_len[3:0]) begin
                            pstate <= P_END;
                        end else begin
                            payload_index <= payload_index + 1'b1;
                        end
                    end

                    P_END: begin
                        if (rx_byte == 8'h00) begin
                            // Valid Dabble GamePad DIGITAL frame (function 0x01).
                            // iOS may send the six face/start/select buttons using
                            // this function even while the UI is in Joystick mode.
                            // Update ONLY button state here: do not disturb the
                            // last analog joystick command held by the robot.
                            if ((module_id == 8'h01) &&
                                (function_id == 8'h01) &&
                                (arg_count == 8'd1) &&
                                (arg_len >= 8'd1)) begin
                                gamepad_seen    <= 1'b1;
                                gamepad_alive   <= 1'b1;
                                alive_count     <= ALIVE_CYCLES;
                                gamepad_buttons <= payload[0];
                                if (gamepad_frame_count != 16'hFFFF)
                                    gamepad_frame_count <= gamepad_frame_count + 1'b1;
                            end

                            // Valid Dabble GamePad ANALOG / joystick frame
                            // (function 0x02). The official library also carries
                            // the six button bits in payload[0], so accept them
                            // from this path as well.
                            if ((module_id == 8'h01) &&
                                (function_id == 8'h02) &&
                                (arg_count == 8'd1) &&
                                (arg_len >= 8'd2)) begin
                                gamepad_seen        <= 1'b1;
                                gamepad_alive       <= 1'b1;
                                alive_count         <= ALIVE_CYCLES;
                                gamepad_frame_pulse <= 1'b1;
                                gamepad_buttons     <= payload[0];
                                joystick_angle_deg  <= payload[1][7:3] * 9'd15;
                                joystick_radius     <= payload[1][2:0];
                                if (gamepad_frame_count != 16'hFFFF)
                                    gamepad_frame_count <= gamepad_frame_count + 1'b1;
                            end

                            // Dabble board identification request.
                            if ((module_id == 8'h00) && (function_id == 8'h03))
                                reply_pending <= 1'b1;
                        end else begin
                            if (parser_error_count != 16'hFFFF)
                                parser_error_count <= parser_error_count + 1'b1;
                        end
                        pstate <= P_WAIT_FF;
                    end

                    default: pstate <= P_WAIT_FF;
                endcase
            end

            // Minimal board-ID reply UART sequencer.
            if (!reply_active && reply_pending) begin
                reply_pending <= 1'b0;
                reply_active  <= 1'b1;
                reply_sending <= 1'b0;
                reply_index   <= 4'd0;
            end else if (reply_active) begin
                if (!reply_sending && !tx_busy) begin
                    tx_byte       <= board_reply_byte(reply_index);
                    tx_start      <= 1'b1;
                    reply_sending <= 1'b1;
                end else if (reply_sending && tx_done) begin
                    reply_sending <= 1'b0;
                    if (reply_index == 4'd9) begin
                        reply_active <= 1'b0;
                        reply_index  <= 4'd0;
                    end else begin
                        reply_index <= reply_index + 1'b1;
                    end
                end
            end
        end
    end
endmodule
