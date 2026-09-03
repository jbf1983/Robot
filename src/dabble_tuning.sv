// -----------------------------------------------------------------------------
// Dabble GamePad runtime tuning.
// Button byte mapping:
//   bit0 Start, bit1 Select, bit2 Triangle, bit3 Circle, bit4 Cross, bit5 Square
//
// Runtime mapping:
//   Triangle : Kp +1
//   Square   : Kp -1
//   Circle   : Ki +1
//   Cross    : Ki -1
//   Start    : speed integral strength +1
//   Select   : speed integral strength -1
//
// speed_i_strength convention:
//   100 = historical speed-I strength
//    50 = half
//   200 = double
//     0 = disabled
// Values wrap naturally modulo 256. Each press refreshes a 5-second display.
// -----------------------------------------------------------------------------
module dabble_tuning #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer DISPLAY_MS = 5000,
    parameter integer KP_INIT = 150,
    parameter integer KI_INIT = 20,
    parameter integer KD_INIT = 0,
    parameter integer SPEED_I_INIT = 100
)(
    input  logic       clk,
    input  logic       rst,
    input  logic [7:0] gamepad_buttons,

    output logic [7:0] kp_value,
    output logic [7:0] ki_value,
    output logic [7:0] kd_value,
    output logic [7:0] speed_i_strength,

    output logic       display_active,
    output logic [1:0] display_mode // 1=Kp, 2=Ki, 3=speed-I strength
);
    localparam integer DISPLAY_CYCLES = (CLK_HZ / 1000) * DISPLAY_MS;
    localparam integer DISPLAY_CNT_W = (DISPLAY_CYCLES <= 2) ? 1 : $clog2(DISPLAY_CYCLES + 1);

    localparam integer START_BIT    = 0;
    localparam integer SELECT_BIT   = 1;
    localparam integer TRIANGLE_BIT = 2;
    localparam integer CIRCLE_BIT   = 3;
    localparam integer CROSS_BIT    = 4;
    localparam integer SQUARE_BIT   = 5;

    logic [7:0] buttons_prev;
    logic [7:0] buttons_rise;
    logic [DISPLAY_CNT_W-1:0] display_count;

    always_comb begin
        buttons_rise = gamepad_buttons & ~buttons_prev;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            buttons_prev      <= 8'd0;
            kp_value          <= KP_INIT[7:0];
            ki_value          <= KI_INIT[7:0];
            kd_value          <= KD_INIT[7:0];
            speed_i_strength  <= SPEED_I_INIT[7:0];
            display_count     <= '0;
            display_active    <= 1'b0;
            display_mode      <= 2'd0;
        end else begin
            buttons_prev     <= gamepad_buttons;

            if (display_count != 0) begin
                display_count  <= display_count - 1'b1;
                display_active <= 1'b1;
            end else begin
                display_active <= 1'b0;
            end

            if (buttons_rise[TRIANGLE_BIT]) begin
                kp_value         <= kp_value + 8'd1;
                display_mode     <= 2'd1;
                display_count    <= DISPLAY_CYCLES;
                display_active   <= 1'b1;
            end else if (buttons_rise[SQUARE_BIT]) begin
                kp_value         <= kp_value - 8'd1;
                display_mode     <= 2'd1;
                display_count    <= DISPLAY_CYCLES;
                display_active   <= 1'b1;
            end else if (buttons_rise[CIRCLE_BIT]) begin
                ki_value         <= ki_value + 8'd1;
                display_mode     <= 2'd2;
                display_count    <= DISPLAY_CYCLES;
                display_active   <= 1'b1;
            end else if (buttons_rise[CROSS_BIT]) begin
                ki_value         <= ki_value - 8'd1;
                display_mode     <= 2'd2;
                display_count    <= DISPLAY_CYCLES;
                display_active   <= 1'b1;
            end else if (buttons_rise[START_BIT]) begin
                speed_i_strength <= speed_i_strength + 8'd1;
                display_mode     <= 2'd3;
                display_count    <= DISPLAY_CYCLES;
                display_active   <= 1'b1;
            end else if (buttons_rise[SELECT_BIT]) begin
                speed_i_strength <= speed_i_strength - 8'd1;
                display_mode     <= 2'd3;
                display_count    <= DISPLAY_CYCLES;
                display_active   <= 1'b1;
            end
        end
    end
endmodule
