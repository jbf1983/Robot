// -----------------------------------------------------------------------------
// Slow outer PI wheel-speed loop for the balance robot.
//
// Validated V69/V70 behavior:
//   - P speed term is always active.
//   - Speed integral is UPDATED only when longitudinal target == 0 rpm.
//   - As soon as forward/reverse motion is requested, the speed-I memory is
//     FROZEN at its last neutral value instead of being cleared.
//   - This preserves the learned mechanical balance bias during motion while
//     preventing the speed-I from winding up and creating forward/reverse asymmetry.
//   - Runtime speed-I strength comes from Dabble Start/Select.
//
// Historical integral reference:
//   SPEED_KI_SHIFT = 7, I limit = +/-3.5 deg.
//   speed_ki_strength = 100 reproduces that historical strength exactly.
//   50 = half strength, 200 = double strength.
//   0 explicitly clears/disables the speed integral.
//
// Internally the integral accumulator carries an extra x100 scale so arbitrary
// strength values do not lose small corrections through integer truncation.
// -----------------------------------------------------------------------------
module speed_controller #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer UPDATE_MS = 100,
    parameter integer STEPS_PER_REV = 3200,

    parameter integer SPEED_KP_NUM = 1,
    parameter integer SPEED_KP_DIV = 20,

    // Historical speed-I scaling and limit.
    parameter integer SPEED_KI_SHIFT = 7,
    parameter integer I_PITCH_LIMIT_TENTHS = 35, // +/-3.5 deg
    parameter integer PITCH_LIMIT_TENTHS   = 200 // +/-20.0 deg total target
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               enable,
    input  logic signed [31:0] position_right,
    input  logic signed [31:0] position_left,
    input  logic signed [15:0] speed_target_rpm_tenths,
    input  logic [7:0]         speed_ki_strength, // 100 = historical strength

    output logic signed [15:0] pitch_target_tenths,
    output logic signed [15:0] speed_rpm_tenths,
    output logic               saturated
);
    localparam integer UPDATE_DIV = (CLK_HZ / 1000) * UPDATE_MS;
    localparam integer CNT_W = (UPDATE_DIV <= 2) ? 1 : $clog2(UPDATE_DIV);
    localparam integer I_SCALE = (1 << SPEED_KI_SHIFT) * 100;

    localparam logic signed [47:0] I_ACC_MAX_Q100 =
        $signed(I_PITCH_LIMIT_TENTHS * I_SCALE);
    localparam logic signed [47:0] I_ACC_MIN_Q100 =
        -$signed(I_PITCH_LIMIT_TENTHS * I_SCALE);

    logic [CNT_W-1:0] tick_count;
    logic update_tick;

    logic signed [31:0] prev_pos_r, prev_pos_l;
    logic signed [32:0] delta_r_wide, delta_l_wide;
    logic signed [33:0] delta_sum_wide;
    logic signed [32:0] delta_avg;

    logic signed [63:0] speed_product_wide;
    logic signed [63:0] speed_now_wide;
    logic signed [31:0] speed_now_tenths;
    logic signed [31:0] speed_prev_tenths;
    logic signed [32:0] speed_pair_sum;
    logic signed [31:0] speed_filtered_tenths;
    logic signed [32:0] speed_error_wide;
    logic signed [31:0] speed_error_tenths;

    logic signed [47:0] p_calc_wide;
    logic signed [31:0] p_pitch_tenths;

    // Extra x100 precision around the historical Q7 accumulator.
    logic signed [47:0] i_accum_q100;
    logic signed [47:0] i_increment_q100;
    logic signed [48:0] i_candidate_wide;
    logic signed [47:0] i_candidate_q100;
    logic signed [47:0] i_pitch_calc_wide;
    logic signed [31:0] i_pitch_tenths;

    logic signed [32:0] target_sum_wide;
    logic signed [31:0] target_calc_tenths;

    always_ff @(posedge clk) begin
        if (rst || !enable) begin
            tick_count  <= '0;
            update_tick <= 1'b0;
        end else begin
            update_tick <= 1'b0;
            if (tick_count == UPDATE_DIV-1) begin
                tick_count  <= '0;
                update_tick <= 1'b1;
            end else begin
                tick_count <= tick_count + 1'b1;
            end
        end
    end

    always_comb begin
        delta_r_wide   = $signed({position_right[31], position_right}) -
                         $signed({prev_pos_r[31], prev_pos_r});
        delta_l_wide   = $signed({position_left[31], position_left}) -
                         $signed({prev_pos_l[31], prev_pos_l});
        delta_sum_wide = $signed({delta_r_wide[32], delta_r_wide}) +
                         $signed({delta_l_wide[32], delta_l_wide});
        delta_avg      = delta_sum_wide >>> 1;

        // rpm*10 = delta_units16 * 600000 / (units_per_rev * update_ms)
        speed_product_wide = $signed(delta_avg) * 64'sd600000;
        speed_now_wide = speed_product_wide / (STEPS_PER_REV * UPDATE_MS);
        if (speed_now_wide > 64'sd2147483647)
            speed_now_tenths = 32'sh7fffffff;
        else if (speed_now_wide < -64'sd2147483647)
            speed_now_tenths = -32'sd2147483647;
        else
            speed_now_tenths = speed_now_wide[31:0];

        speed_pair_sum        = $signed({speed_now_tenths[31], speed_now_tenths}) +
                                $signed({speed_prev_tenths[31], speed_prev_tenths});
        speed_filtered_tenths = speed_pair_sum >>> 1;

        // Physical sign already validated on the robot: measured - target.
        speed_error_wide = $signed({speed_filtered_tenths[31], speed_filtered_tenths}) -
                           $signed({{17{speed_target_rpm_tenths[15]}}, speed_target_rpm_tenths});
        if (speed_error_wide > 33'sd2147483647)
            speed_error_tenths = 32'sh7fffffff;
        else if (speed_error_wide < -33'sd2147483647)
            speed_error_tenths = -32'sd2147483647;
        else
            speed_error_tenths = speed_error_wide[31:0];

        p_calc_wide = $signed(speed_error_tenths) * SPEED_KP_NUM;
        p_calc_wide = p_calc_wide / SPEED_KP_DIV;
        if (p_calc_wide > 48'sd2147483647)
            p_pitch_tenths = 32'sh7fffffff;
        else if (p_calc_wide < -48'sd2147483647)
            p_pitch_tenths = -32'sd2147483647;
        else
            p_pitch_tenths = p_calc_wide[31:0];

        // Historical I, but runtime-scalable. 100 reproduces exactly the old
        // i_accum += speed_error followed by >>>7 behavior.
        i_increment_q100 = $signed(speed_error_tenths) * $signed({1'b0, speed_ki_strength});

        if (speed_ki_strength == 8'd0) begin
            // S=0 means genuinely disabled: clear any previously learned bias.
            i_candidate_wide = 49'sd0;
            i_candidate_q100 = 48'sd0;
        end else if (speed_target_rpm_tenths == 16'sd0) begin
            // Neutral command: update the slow speed integral so it can learn
            // the static mechanical balance bias and cancel residual drift.
            i_candidate_wide = $signed({i_accum_q100[47], i_accum_q100}) +
                               $signed({i_increment_q100[47], i_increment_q100});
            if (i_candidate_wide > $signed({1'b0, I_ACC_MAX_Q100}))
                i_candidate_q100 = I_ACC_MAX_Q100;
            else if (i_candidate_wide < $signed({I_ACC_MIN_Q100[47], I_ACC_MIN_Q100}))
                i_candidate_q100 = I_ACC_MIN_Q100;
            else
                i_candidate_q100 = i_candidate_wide[47:0];
        end else begin
            // Movement requested: HOLD the neutral-learned speed-I memory.
            // Do not integrate while moving, but keep the mechanical balance
            // bias learned at zero speed so forward/reverse commands remain
            // symmetric around the true equilibrium point.
            i_candidate_wide = $signed({i_accum_q100[47], i_accum_q100});
            i_candidate_q100 = i_accum_q100;
        end

        i_pitch_calc_wide = i_candidate_q100 / I_SCALE;
        if (i_pitch_calc_wide > 48'sd2147483647)
            i_pitch_tenths = 32'sh7fffffff;
        else if (i_pitch_calc_wide < -48'sd2147483647)
            i_pitch_tenths = -32'sd2147483647;
        else
            i_pitch_tenths = i_pitch_calc_wide[31:0];

        target_sum_wide = $signed({p_pitch_tenths[31], p_pitch_tenths}) +
                          $signed({i_pitch_tenths[31], i_pitch_tenths});
        if (target_sum_wide > PITCH_LIMIT_TENTHS)
            target_calc_tenths = PITCH_LIMIT_TENTHS;
        else if (target_sum_wide < -PITCH_LIMIT_TENTHS)
            target_calc_tenths = -PITCH_LIMIT_TENTHS;
        else
            target_calc_tenths = target_sum_wide[31:0];
    end

    always_ff @(posedge clk) begin
        if (rst || !enable) begin
            prev_pos_r          <= position_right;
            prev_pos_l          <= position_left;
            speed_prev_tenths   <= 32'sd0;
            speed_rpm_tenths    <= 16'sd0;
            i_accum_q100        <= 48'sd0;
            pitch_target_tenths <= 16'sd0;
            saturated           <= 1'b0;
        end else if (update_tick) begin
            prev_pos_r        <= position_right;
            prev_pos_l        <= position_left;
            speed_prev_tenths <= speed_now_tenths;
            i_accum_q100      <= i_candidate_q100;

            if (speed_filtered_tenths > 32'sd32767)
                speed_rpm_tenths <= 16'sd32767;
            else if (speed_filtered_tenths < -32'sd32767)
                speed_rpm_tenths <= -16'sd32767;
            else
                speed_rpm_tenths <= speed_filtered_tenths[15:0];

            if (target_calc_tenths > 32'sd32767)
                pitch_target_tenths <= 16'sd32767;
            else if (target_calc_tenths < -32'sd32767)
                pitch_target_tenths <= -16'sd32767;
            else
                pitch_target_tenths <= target_calc_tenths[15:0];

            saturated <= (target_sum_wide > PITCH_LIMIT_TENTHS) ||
                         (target_sum_wide < -PITCH_LIMIT_TENTHS) ||
                         (i_candidate_q100 == I_ACC_MAX_Q100) ||
                         (i_candidate_q100 == I_ACC_MIN_Q100);
        end
    end
endmodule
