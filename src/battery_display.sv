// -----------------------------------------------------------------------------
// Battery voltage formatter for the DE10-Lite 7-segment displays.
//
// battery_mv is in millivolts. Normal display example:
//     4.08
//
// Physical layout:
//   HEX5 : blank
//   HEX4 : volts digit, decimal point ON
//   HEX3 : tenths of volt (100 mV)
//   HEX2 : hundredths of volt (10 mV)
//   HEX1 : blank
//   HEX0 : blank
//
// Voltage is rounded to the nearest 10 mV before display.
// If the ADC value is not valid, display "Err".
// Segments are [0:6] = A..G, active low.
// -----------------------------------------------------------------------------
module battery_display (
    input  logic        valid,
    input  logic [12:0] battery_mv,

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
    localparam logic [0:6] SEG_E     = 7'b0110000;
    localparam logic [0:6] SEG_r     = 7'b1111010;

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

    integer mv;
    integer rounded_10mv;
    integer dV;
    integer d100m;
    integer d10m;

    always_comb begin
        HEX5 = SEG_BLANK;
        HEX4 = SEG_BLANK;
        HEX3 = SEG_BLANK;
        HEX2 = SEG_BLANK;
        HEX1 = SEG_BLANK;
        HEX0 = SEG_BLANK;

        DP5 = 1'b1;
        DP4 = 1'b1;
        DP3 = 1'b1;
        DP2 = 1'b1;
        DP1 = 1'b1;
        DP0 = 1'b1;

        mv           = battery_mv;
        rounded_10mv = 0;
        dV           = 0;
        d100m        = 0;
        d10m         = 0;

        if (!valid) begin
            // "Err"
            HEX3 = SEG_E;
            HEX2 = SEG_r;
            HEX1 = SEG_r;
        end else begin
            if (mv < 0)
                mv = 0;
            else if (mv > 9999)
                mv = 9999;

            // Round to nearest 10 mV, then display x.xx.
            rounded_10mv = ((mv + 5) / 10) * 10;
            if (rounded_10mv > 9990)
                rounded_10mv = 9990;

            dV    = (rounded_10mv / 1000) % 10;
            d100m = (rounded_10mv / 100)  % 10;
            d10m  = (rounded_10mv / 10)   % 10;

            HEX4 = seg_digit(dV[3:0]);
            HEX3 = seg_digit(d100m[3:0]);
            HEX2 = seg_digit(d10m[3:0]);

            // Decimal point after volts digit: x.xx
            DP4 = 1'b0;
        end
    end
endmodule
