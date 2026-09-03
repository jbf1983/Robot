// -----------------------------------------------------------------------------
// Six active-low 7-segment display formatter for DE10-Lite/custom pinout.
// Normal format:  -180.0°  /  12.3°
//   HEX5 sign
//   HEX4 hundreds of degrees (blank when unused)
//   HEX3 tens of degrees (blank when unused)
//   HEX2 units of degrees, DP2 active
//   HEX1 tenths
//   HEX0 degree symbol
// During gyro startup calibration: "CAL"
// On MPU WHO_AM_I / init error: "Err"
// Segments are [0:6] = A..G, active low, matching the legacy project.
// -----------------------------------------------------------------------------
module pitch_display (
    input  logic               imu_ok,
    input  logic               imu_error,
    input  logic               calibrated,
    input  logic signed [15:0] pitch_tenths,

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
    localparam logic [0:6] SEG_DEG   = 7'b0011100;
    localparam logic [0:6] SEG_C     = 7'b0110001;
    localparam logic [0:6] SEG_A     = 7'b0001000;
    localparam logic [0:6] SEG_L     = 7'b1110001;
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

    integer p;
    integer mag;
    integer d100, d10, d1, d01;

    always_comb begin
        p    = 0;
        mag  = 0;
        d100 = 0;
        d10  = 0;
        d1   = 0;
        d01  = 0;

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

        if (imu_error) begin
            //                    E r r
            HEX3 = SEG_E;
            HEX2 = SEG_r;
            HEX1 = SEG_r;
        end else if (!imu_ok) begin
            // MPU still starting: dashes.
            HEX3 = SEG_MINUS;
            HEX2 = SEG_MINUS;
            HEX1 = SEG_MINUS;
        end else if (!calibrated) begin
            // Startup gyro bias calibration (~4.1 s).
            HEX3 = SEG_C;
            HEX2 = SEG_A;
            HEX1 = SEG_L;
        end else begin
            p = pitch_tenths;
            if (p < 0) begin
                HEX5 = SEG_MINUS;
                mag = -p;
            end else begin
                HEX5 = SEG_BLANK;
                mag = p;
            end

            // Clamp to the physically meaningful atan2 range.
            if (mag > 1800)
                mag = 1800;

            d100 = mag / 1000;          // hundreds of degrees
            d10  = (mag / 100) % 10;   // tens of degrees
            d1   = (mag / 10)  % 10;   // units of degrees
            d01  = mag % 10;            // tenths

            if (mag >= 1000)
                HEX4 = seg_digit(d100[3:0]);
            else
                HEX4 = SEG_BLANK;

            if (mag >= 100)
                HEX3 = seg_digit(d10[3:0]);
            else
                HEX3 = SEG_BLANK;

            HEX2 = seg_digit(d1[3:0]);
            HEX1 = seg_digit(d01[3:0]);
            HEX0 = SEG_DEG;

            // Decimal point after units digit: xx.x degrees.
            DP2 = 1'b0;
        end
    end

endmodule
