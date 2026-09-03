// -----------------------------------------------------------------------------
// Dabble joystick -> robot motion commands.
//
// Dabble convention:
//   angle 0 deg   = +X = right
//   angle 90 deg  = +Y = up
//   radius 0..7
//
// This module converts polar joystick data into:
//   speed_target_rpm_tenths : outer wheel-speed-loop target
//   turn_hz                 : differential wheel command in 1/4-equiv STEP/s
//
// Safety:
// - robot disable => commands immediately return to zero
// - the last valid joystick command is held continuously until a new Dabble
//   joystick frame changes it (including a center frame). Dabble on iOS may
//   transmit joystick values only when they change, so UART inactivity must
//   NOT cancel the command.
// - after enable/reconnect, joystick must first be centered before commands arm
// - radius <= DEADZONE_RADIUS is treated as centered
// - command outputs are slew-limited during normal operation
//
// FORWARD_INVERT is intentional for the tested speed-loop sign convention:
// positive pitch target was the historical "forward" command, while the outer
// speed PI uses (measured_speed - requested_speed) as its error.
// -----------------------------------------------------------------------------
module joystick_command #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer UPDATE_MS = 10,
    parameter integer DEADZONE_RADIUS = 1,
    parameter integer MAX_SPEED_RPM_TENTHS = 1500, // 150 rpm ~= 2.71 km/h, 96-mm wheel
    parameter integer MAX_TURN_HZ = 700,
    parameter integer SPEED_SLEW_RPM_TENTHS = 20, // 2 rpm per 10 ms
    parameter integer TURN_SLEW_HZ = 50,
    parameter bit     FORWARD_INVERT = 1'b1,
    // Physical steering correction: on this chassis the tested steering sign
    // is correct in reverse but opposite in forward. Keep reverse and pivot
    // steering unchanged, invert only when joystick Y requests forward.
    parameter bit     INVERT_TURN_FORWARD = 1'b1
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               robot_enable,
    input  logic               gamepad_seen,
    input  logic [8:0]         joystick_angle_deg,
    input  logic [2:0]         joystick_radius,

    output logic signed [15:0] speed_target_rpm_tenths,
    output logic signed [15:0] turn_hz,
    output logic               control_armed,
    output logic signed [15:0] joystick_x_q8,
    output logic signed [15:0] joystick_y_q8
);
    localparam integer UPDATE_DIV = (CLK_HZ / 1000) * UPDATE_MS;
    localparam integer CNT_W = (UPDATE_DIV <= 2) ? 1 : $clog2(UPDATE_DIV);
    localparam integer AXIS_FULL_Q8 = 7 * 256; // radius 7 at unit cosine/sine

    logic [CNT_W-1:0] update_count;
    logic update_tick;

    logic signed [9:0] cos_q8;
    logic signed [9:0] sin_q8;
    logic signed [15:0] x_q8;
    logic signed [15:0] y_q8;
    logic signed [31:0] speed_product;
    logic signed [31:0] turn_product;
    logic signed [15:0] requested_speed;
    logic signed [15:0] requested_turn;

    function automatic logic signed [15:0] slew16(
        input logic signed [15:0] current,
        input logic signed [15:0] target,
        input integer step_size
    );
        logic signed [16:0] diff;
        begin
            diff = $signed({target[15],target}) - $signed({current[15],current});
            if (diff > step_size)
                slew16 = current + step_size;
            else if (diff < -step_size)
                slew16 = current - step_size;
            else
                slew16 = target;
        end
    endfunction

    // Exact 15-degree Dabble sectors, Q8 sine/cosine.
    always_comb begin
        cos_q8 = 10'sd256;
        sin_q8 = 10'sd0;
        unique case (joystick_angle_deg)
            9'd0, 9'd360: begin cos_q8= 10'sd256; sin_q8=   10'sd0; end
            9'd15:  begin cos_q8= 10'sd247; sin_q8=  10'sd66; end
            9'd30:  begin cos_q8= 10'sd222; sin_q8= 10'sd128; end
            9'd45:  begin cos_q8= 10'sd181; sin_q8= 10'sd181; end
            9'd60:  begin cos_q8= 10'sd128; sin_q8= 10'sd222; end
            9'd75:  begin cos_q8=  10'sd66; sin_q8= 10'sd247; end
            9'd90:  begin cos_q8=   10'sd0; sin_q8= 10'sd256; end
            9'd105: begin cos_q8= -10'sd66; sin_q8= 10'sd247; end
            9'd120: begin cos_q8=-10'sd128; sin_q8= 10'sd222; end
            9'd135: begin cos_q8=-10'sd181; sin_q8= 10'sd181; end
            9'd150: begin cos_q8=-10'sd222; sin_q8= 10'sd128; end
            9'd165: begin cos_q8=-10'sd247; sin_q8=  10'sd66; end
            9'd180: begin cos_q8=-10'sd256; sin_q8=   10'sd0; end
            9'd195: begin cos_q8=-10'sd247; sin_q8= -10'sd66; end
            9'd210: begin cos_q8=-10'sd222; sin_q8=-10'sd128; end
            9'd225: begin cos_q8=-10'sd181; sin_q8=-10'sd181; end
            9'd240: begin cos_q8=-10'sd128; sin_q8=-10'sd222; end
            9'd255: begin cos_q8= -10'sd66; sin_q8=-10'sd247; end
            9'd270: begin cos_q8=   10'sd0; sin_q8=-10'sd256; end
            9'd285: begin cos_q8=  10'sd66; sin_q8=-10'sd247; end
            9'd300: begin cos_q8= 10'sd128; sin_q8=-10'sd222; end
            9'd315: begin cos_q8= 10'sd181; sin_q8=-10'sd181; end
            9'd330: begin cos_q8= 10'sd222; sin_q8=-10'sd128; end
            9'd345: begin cos_q8= 10'sd247; sin_q8= -10'sd66; end
            default: begin cos_q8=10'sd256; sin_q8=10'sd0; end
        endcase

        if (joystick_radius <= DEADZONE_RADIUS) begin
            x_q8 = 16'sd0;
            y_q8 = 16'sd0;
        end else begin
            x_q8 = $signed(cos_q8) * $signed({1'b0, joystick_radius});
            y_q8 = $signed(sin_q8) * $signed({1'b0, joystick_radius});
        end

        joystick_x_q8 = x_q8;
        joystick_y_q8 = y_q8;

        speed_product = $signed(y_q8) * MAX_SPEED_RPM_TENTHS;
        if (FORWARD_INVERT)
            requested_speed = -$signed(speed_product / AXIS_FULL_Q8);
        else
            requested_speed =  $signed(speed_product / AXIS_FULL_Q8);

        turn_product  = $signed(x_q8) * MAX_TURN_HZ;
        requested_turn = $signed(turn_product / AXIS_FULL_Q8);

        // Dabble +Y (joystick up) is the physical forward request. The robot
        // has opposite steering kinematics in forward versus the currently
        // correct reverse behavior, so invert only the forward steering sign.
        // With Y <= 0 (reverse or pivot/center), preserve the existing sign.
        if (INVERT_TURN_FORWARD && (y_q8 > 0))
            requested_turn = -requested_turn;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            update_count <= '0;
            update_tick  <= 1'b0;
        end else begin
            update_tick <= 1'b0;
            if (update_count == UPDATE_DIV-1) begin
                update_count <= '0;
                update_tick  <= 1'b1;
            end else begin
                update_count <= update_count + 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            control_armed          <= 1'b0;
            speed_target_rpm_tenths <= 16'sd0;
            turn_hz                 <= 16'sd0;
        end else if (!robot_enable || !gamepad_seen) begin
            control_armed          <= 1'b0;
            speed_target_rpm_tenths <= 16'sd0;
            turn_hz                 <= 16'sd0;
        end else begin
            // Must see center once after every enable/reconnect.
            if (!control_armed) begin
                speed_target_rpm_tenths <= 16'sd0;
                turn_hz                 <= 16'sd0;
                if (joystick_radius <= DEADZONE_RADIUS)
                    control_armed <= 1'b1;
            end else if (update_tick) begin
                speed_target_rpm_tenths <= slew16(
                    speed_target_rpm_tenths, requested_speed, SPEED_SLEW_RPM_TENTHS);
                turn_hz <= slew16(turn_hz, requested_turn, TURN_SLEW_HZ);
            end
        end
    end
endmodule
