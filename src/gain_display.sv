// -----------------------------------------------------------------------------
// Runtime tuning display.
// Shows selected value as "P 150", "I 020", or "S 100".
// S = outer speed-integral strength, where 100 is the historical reference.
// -----------------------------------------------------------------------------
module gain_display (
    input  logic       select_kp,
    input  logic       select_ki,
    input  logic       select_speed_i,
    input  logic [7:0] kp_value,
    input  logic [7:0] ki_value,
    input  logic [7:0] speed_i_value,

    output logic [0:6] HEX5,
    output logic [0:6] HEX4,
    output logic [0:6] HEX3,
    output logic [0:6] HEX2,
    output logic [0:6] HEX1,
    output logic [0:6] HEX0,
    output logic DP5, output logic DP4, output logic DP3,
    output logic DP2, output logic DP1, output logic DP0
);
    localparam logic [0:6] SEG_BLANK = 7'b1111111;
    localparam logic [0:6] SEG_P     = 7'b0011000;
    localparam logic [0:6] SEG_I     = 7'b1001111;
    localparam logic [0:6] SEG_S     = 7'b0100100; // same shape as 5, readable as S

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

    integer value;
    integer h, t, u;

    always_comb begin
        value = 0;
        HEX5 = SEG_BLANK; HEX4 = SEG_BLANK; HEX3 = SEG_BLANK;
        HEX2 = SEG_BLANK; HEX1 = SEG_BLANK; HEX0 = SEG_BLANK;
        DP5 = 1'b1; DP4 = 1'b1; DP3 = 1'b1;
        DP2 = 1'b1; DP1 = 1'b1; DP0 = 1'b1;

        if (select_kp && !select_ki && !select_speed_i) begin
            HEX5 = SEG_P;
            value = kp_value;
        end else if (!select_kp && select_ki && !select_speed_i) begin
            HEX5 = SEG_I;
            value = ki_value;
        end else if (!select_kp && !select_ki && select_speed_i) begin
            HEX5 = SEG_S;
            value = speed_i_value;
        end

        h = value / 100;
        t = (value / 10) % 10;
        u = value % 10;
        HEX3 = seg_digit(h[3:0]);
        HEX2 = seg_digit(t[3:0]);
        HEX1 = seg_digit(u[3:0]);
    end
endmodule
