// -----------------------------------------------------------------------------
// Six-digit decimal display for STEP rate in Hz.
// Example: 0 -> 000000, 500 -> 000500, 60000 -> 060000.
// DE10-Lite/custom segments are [0:6] = A..G, active low.
// -----------------------------------------------------------------------------
module speed_display (
    input  logic [15:0] value,
    output logic [0:6] HEX5,
    output logic [0:6] HEX4,
    output logic [0:6] HEX3,
    output logic [0:6] HEX2,
    output logic [0:6] HEX1,
    output logic [0:6] HEX0,
    output logic DP5, output logic DP4, output logic DP3,
    output logic DP2, output logic DP1, output logic DP0
);
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
            default: seg_digit = 7'b1111111;
        endcase
    endfunction

    integer v;
    integer d5, d4, d3, d2, d1, d0;

    always_comb begin
        v  = value;
        d5 = (v / 100000) % 10;
        d4 = (v / 10000)  % 10;
        d3 = (v / 1000)   % 10;
        d2 = (v / 100)    % 10;
        d1 = (v / 10)     % 10;
        d0 = v % 10;

        HEX5 = seg_digit(d5[3:0]);
        HEX4 = seg_digit(d4[3:0]);
        HEX3 = seg_digit(d3[3:0]);
        HEX2 = seg_digit(d2[3:0]);
        HEX1 = seg_digit(d1[3:0]);
        HEX0 = seg_digit(d0[3:0]);

        DP5 = 1'b1; DP4 = 1'b1; DP3 = 1'b1;
        DP2 = 1'b1; DP1 = 1'b1; DP0 = 1'b1;
    end
endmodule
