// -----------------------------------------------------------------------------
// Bring-up display for the HM-10 / Dabble GamePad link.
//
// Before any UART byte:       "---"
// UART bytes but no GamePad:  "b 123"  (last raw byte in decimal)
// Valid analog GamePad frame: "A090r7"  (angle 90 deg, radius 7)
// DP5 is ON while fresh GamePad frames have been seen within the alive window.
// -----------------------------------------------------------------------------
module dabble_display (
    input  logic       uart_seen,
    input  logic [7:0] last_uart_byte,
    input  logic       gamepad_seen,
    input  logic       gamepad_alive,
    input  logic [8:0] joystick_angle_deg,
    input  logic [2:0] joystick_radius,

    output logic [0:6] HEX5,
    output logic [0:6] HEX4,
    output logic [0:6] HEX3,
    output logic [0:6] HEX2,
    output logic [0:6] HEX1,
    output logic [0:6] HEX0,
    output logic       DP5,
    output logic       DP4,
    output logic       DP3,
    output logic       DP2,
    output logic       DP1,
    output logic       DP0
);
    localparam logic [0:6] SEG_BLANK = 7'b1111111;
    localparam logic [0:6] SEG_MINUS = 7'b1111110;
    localparam logic [0:6] SEG_A     = 7'b0001000;
    localparam logic [0:6] SEG_r     = 7'b1111010;
    localparam logic [0:6] SEG_b     = 7'b1100000;

    function automatic logic [0:6] seg_digit(input logic [3:0] d);
        case (d)
            4'd0: seg_digit = 7'b0000001;
            4'd1: seg_digit = 7'b1001111;
            4'd2: seg_digit = 7'b0010010;
            4'd3: seg_digit = 7'b0000110;
            4'd4: seg_digit = 7'b1001100;
            4'd5: seg_digit = 7'b0100100;
            4'd6: seg_digit = 7'b0100000;
            4'd7: seg_digit = 7'b0001111;
            4'd8: seg_digit = 7'b0000000;
            4'd9: seg_digit = 7'b0000100;
            default: seg_digit = SEG_BLANK;
        endcase
    endfunction

    integer angle;
    integer raw;
    integer h, t, u;

    always_comb begin
        HEX5 = SEG_BLANK; HEX4 = SEG_BLANK; HEX3 = SEG_BLANK;
        HEX2 = SEG_BLANK; HEX1 = SEG_BLANK; HEX0 = SEG_BLANK;
        DP5  = 1'b1; DP4 = 1'b1; DP3 = 1'b1;
        DP2  = 1'b1; DP1 = 1'b1; DP0 = 1'b1;

        angle = joystick_angle_deg;
        raw   = last_uart_byte;
        h = 0; t = 0; u = 0;

        if (gamepad_seen) begin
            // AxxxrR
            if (angle > 999) angle = 999;
            h = angle / 100;
            t = (angle / 10) % 10;
            u = angle % 10;

            HEX5 = SEG_A;
            HEX4 = seg_digit(h[3:0]);
            HEX3 = seg_digit(t[3:0]);
            HEX2 = seg_digit(u[3:0]);
            HEX1 = SEG_r;
            HEX0 = seg_digit({1'b0, joystick_radius});

            // Activity dot after A while frames are fresh.
            DP5 = ~gamepad_alive;
        end else if (uart_seen) begin
            // b xxx : raw UART byte in decimal, proves physical link even if
            // Dabble frame parsing has not succeeded yet.
            h = raw / 100;
            t = (raw / 10) % 10;
            u = raw % 10;

            HEX5 = SEG_b;
            HEX4 = SEG_BLANK;
            HEX3 = seg_digit(h[3:0]);
            HEX2 = seg_digit(t[3:0]);
            HEX1 = seg_digit(u[3:0]);
            HEX0 = SEG_BLANK;
        end else begin
            HEX3 = SEG_MINUS;
            HEX2 = SEG_MINUS;
            HEX1 = SEG_MINUS;
        end
    end
endmodule
