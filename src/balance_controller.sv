// -----------------------------------------------------------------------------
// Old-style pitch PID translated/rebuilt in SystemVerilog.
// Reference behavior from the original VHDL project:
//   - IMU/Kalman attitude update effectively at 100 Hz (10 ms)
//   - pitch moving average: 2 taps
//   - P/D + motor command update: 1 kHz
//   - I update: 10 us (100 kHz), gain internally divided by 1000
//   - motor command moving average: 16 taps at 1 kHz
//   - current defaults supplied by top-level Dabble tuning: Kp=150, Ki=20, Kd=0
//   - legacy command response preserved through +/-30000; wide recovery path above
//   - legacy motor mapping approximately fSTEP = 10 + |cmd|/16 Hz
//
// Improvements intentionally made versus the old VHDL:
//   - command==0 => 0 STEP/s (old LUT crawled at ~10 Hz even at zero)
//   - explicit saturation instead of stale-value behavior on overflow
//   - proper D branch using gyro Y (default Kd=0, so legacy behavior unchanged)
//   - wide command/filter path avoids 16-bit truncation during recovery
// -----------------------------------------------------------------------------
module balance_controller #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer START_WINDOW_TENTHS = 80,
    parameter integer FALL_LIMIT_TENTHS   = 450,
    parameter integer PID_CMD_LIMIT       = 100000,
    parameter integer I_LIMIT             = 30000,
    parameter integer I_UPDATE_US         = 10,
    parameter integer I_GAIN_DIV          = 1000,
    parameter integer MAX_STEP_HZ         = 4000,
    parameter integer CMD_TO_HZ_DIV       = 16,
    // Keep the proven legacy /16 response up to this command, then use a
    // steeper high-speed recovery slope. The final mechanical command is
    // limited to 4000 quarter-step-equivalent STEP/s.
    parameter integer HIGH_SPEED_KNEE_CMD = 30000,
    parameter integer HIGH_SPEED_DIV      = 4,
    parameter integer MIN_NONZERO_HZ      = 10
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               sample_valid,
    input  logic               sensor_ready,
    input  logic               operator_enable,

    // Dynamic pitch setpoint supplied by the slow outer speed loop.
    // This mirrors the CONSIGNE input of the original VHDL PID.
    input  logic signed [15:0] pitch_target_tenths_in,
    input  logic signed [15:0] pitch_tenths_in,
    input  logic signed [15:0] gyro_y_raw,
    input  logic signed [31:0] gyro_bias_q12,

    // Runtime gains supplied by the Dabble tuning layer.
    input  logic [7:0]         kp_value,
    input  logic [7:0]         ki_value,
    input  logic [7:0]         kd_value,

    output logic signed [15:0] motor_speed_hz,
    output logic               balance_active,
    output logic               fault_latched,
    output logic               saturated,
    output logic signed [15:0] filtered_pitch_tenths
);

    // -------------------------------------------------------------------------
    // 1 ms and 10 ms ticks, matching the timing structure of the old project.
    // -------------------------------------------------------------------------
    localparam integer TICK_1MS_DIV = CLK_HZ / 1000;
    localparam integer TICK1_W = (TICK_1MS_DIV <= 2) ? 1 : $clog2(TICK_1MS_DIV);

    logic [TICK1_W-1:0] tick1_count;
    logic [3:0] ms10_count;
    logic tick_1ms;
    logic tick_10ms;

    always_ff @(posedge clk) begin
        if (rst) begin
            tick1_count <= '0;
            ms10_count  <= 4'd0;
            tick_1ms    <= 1'b0;
            tick_10ms   <= 1'b0;
        end else begin
            tick_1ms  <= 1'b0;
            tick_10ms <= 1'b0;
            if (tick1_count == TICK_1MS_DIV-1) begin
                tick1_count <= '0;
                tick_1ms    <= 1'b1;
                if (ms10_count == 4'd9) begin
                    ms10_count <= 4'd0;
                    tick_10ms  <= 1'b1;
                end else begin
                    ms10_count <= ms10_count + 1'b1;
                end
            end else begin
                tick1_count <= tick1_count + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Fast integral tick.  The displayed Ki remains an integer (0..255), but
    // its physical contribution is scaled by I_GAIN_DIV.  A fixed-point
    // accumulator preserves sub-command-unit increments, so small errors are
    // not lost through integer division.
    // -------------------------------------------------------------------------
    localparam integer I_TICK_DIV = (CLK_HZ / 1_000_000) * I_UPDATE_US;
    localparam integer ITICK_W = (I_TICK_DIV <= 2) ? 1 : $clog2(I_TICK_DIV);

    logic [ITICK_W-1:0] i_tick_count;
    logic tick_i;

    always_ff @(posedge clk) begin
        if (rst) begin
            i_tick_count <= '0;
            tick_i       <= 1'b0;
        end else begin
            tick_i <= 1'b0;
            if (i_tick_count == I_TICK_DIV-1) begin
                i_tick_count <= '0;
                tick_i       <= 1'b1;
            end else begin
                i_tick_count <= i_tick_count + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Kp/Ki/Kd are provided externally by the Dabble tuning layer.
    // -------------------------------------------------------------------------

    // -------------------------------------------------------------------------
    // Legacy timing emulation.
    // The modern estimator may update faster, but the old MPU/Kalman path was
    // effectively refreshed every 10 ms. We therefore sample pitch at 100 Hz.
    // The old 2-tap pitch MAVG itself ran at 1 kHz, so a new 10-ms value is
    // blended with the previous value for one 1-ms interval.
    // -------------------------------------------------------------------------
    logic signed [15:0] pitch_latest;
    logic signed [15:0] gyro_latest_tenths;
    logic signed [15:0] pitch_hold_100hz;
    logic signed [15:0] pitch_prev_1ms;
    logic signed [16:0] pitch_pair_sum;

    logic signed [31:0] gyro_bias_raw;
    logic signed [31:0] gyro_corr_raw;
    logic signed [47:0] gyro_tenths_wide;
    logic signed [15:0] gyro_tenths_now;
    logic signed [15:0] gyro_prev_100hz;
    logic signed [16:0] gyro_pair_sum;
    logic signed [15:0] gyro_filtered_tenths;

    always_comb begin
        gyro_bias_raw = gyro_bias_q12 >>> 12;
        gyro_corr_raw = {{16{gyro_y_raw[15]}}, gyro_y_raw} - gyro_bias_raw;
        // +/-500 dps config: 65.5 LSB/(deg/s). Convert to 0.1 deg/s:
        // raw * 10 / 65.5 = raw * 20 / 131.
        gyro_tenths_wide = $signed(gyro_corr_raw) * 48'sd20;
        gyro_tenths_wide = gyro_tenths_wide / 48'sd131;
        if (gyro_tenths_wide > 48'sd32767)
            gyro_tenths_now = 16'sd32767;
        else if (gyro_tenths_wide < -48'sd32767)
            gyro_tenths_now = -16'sd32767;
        else
            gyro_tenths_now = gyro_tenths_wide[15:0];
    end

    // -------------------------------------------------------------------------
    // PID and 16-sample motor-command moving average.
    // All control math is in the original command units.
    // -------------------------------------------------------------------------
    logic signed [15:0] error_tenths;
    logic signed [31:0] p_term;
    logic signed [31:0] i_term;
    logic signed [31:0] d_term;
    logic signed [31:0] pid_cmd;

    // Wide command path: the historical 16-bit path capped useful speed at
    // ~1885 qtr-equivalent Hz. 32-bit samples preserve large recovery commands
    // without wrap/truncation while keeping identical behavior below 30000.
    logic signed [31:0] cmd_hist [0:15];
    logic [3:0] cmd_hist_index;
    logic signed [36:0] cmd_sum;
    logic signed [36:0] cmd_sum_next;
    logic signed [31:0] cmd_filtered;

    logic signed [47:0] p_wide;
    logic signed [47:0] d_wide;
    logic signed [47:0] sum_wide;
    logic signed [31:0] p_next;
    logic signed [31:0] d_next;
    logic signed [31:0] pid_next;

    // Integral accumulator is stored in command * I_GAIN_DIV units.
    logic signed [47:0] i_accum_scaled;
    logic signed [47:0] i_increment_scaled;
    logic signed [48:0] i_accum_candidate;
    logic signed [47:0] i_accum_next;
    logic signed [47:0] i_term_scaled_wide;
    localparam logic signed [47:0] I_ACC_LIMIT_SCALED = I_LIMIT * I_GAIN_DIV;

    integer idx;

    always_comb begin
        pitch_pair_sum = $signed({pitch_hold_100hz[15], pitch_hold_100hz}) +
                         $signed({pitch_prev_1ms[15], pitch_prev_1ms});

        gyro_pair_sum = $signed({gyro_tenths_now[15], gyro_tenths_now}) +
                        $signed({gyro_prev_100hz[15], gyro_prev_100hz});

        p_wide = $signed({{32{error_tenths[15]}}, error_tenths}) *
                 $signed({40'd0, kp_value});
        if (p_wide > PID_CMD_LIMIT)
            p_next = PID_CMD_LIMIT;
        else if (p_wide < -PID_CMD_LIMIT)
            p_next = -PID_CMD_LIMIT;
        else
            p_next = p_wide[31:0];

        // D opposes measured angular velocity. Gyro is expressed in 0.1 deg/s;
        // divide by 10 so Kd=1 corresponds roughly to 1 command unit/(deg/s).
        d_wide = -($signed({{32{gyro_filtered_tenths[15]}}, gyro_filtered_tenths}) *
                   $signed({40'd0, kd_value}));
        d_wide = d_wide / 48'sd10;
        if (d_wide > PID_CMD_LIMIT)
            d_next = PID_CMD_LIMIT;
        else if (d_wide < -PID_CMD_LIMIT)
            d_next = -PID_CMD_LIMIT;
        else
            d_next = d_wide[31:0];

        // Fast I path: every I_UPDATE_US, add error*Ki into a scaled
        // accumulator. Since i_accum_scaled is in command*I_GAIN_DIV units,
        // this is exactly equivalent to an instantaneous gain Ki/I_GAIN_DIV.
        i_increment_scaled =
            $signed({{32{error_tenths[15]}}, error_tenths}) *
            $signed({40'd0, ki_value});
        i_accum_candidate =
            $signed({i_accum_scaled[47], i_accum_scaled}) +
            $signed({i_increment_scaled[47], i_increment_scaled});

        if (i_accum_candidate > $signed({1'b0, I_ACC_LIMIT_SCALED}))
            i_accum_next = I_ACC_LIMIT_SCALED;
        else if (i_accum_candidate < -$signed({1'b0, I_ACC_LIMIT_SCALED}))
            i_accum_next = -I_ACC_LIMIT_SCALED;
        else
            i_accum_next = i_accum_candidate[47:0];

        i_term_scaled_wide = i_accum_scaled / I_GAIN_DIV;

        sum_wide = $signed({{16{p_next[31]}}, p_next}) +
                   $signed({{16{i_term[31]}}, i_term}) +
                   $signed({{16{d_next[31]}}, d_next});
        if (sum_wide > PID_CMD_LIMIT)
            pid_next = PID_CMD_LIMIT;
        else if (sum_wide < -PID_CMD_LIMIT)
            pid_next = -PID_CMD_LIMIT;
        else
            pid_next = sum_wide[31:0];

        cmd_sum_next = cmd_sum
                     - {{5{cmd_hist[cmd_hist_index][31]}}, cmd_hist[cmd_hist_index]}
                     + {{5{pid_next[31]}}, pid_next};
    end

    // -------------------------------------------------------------------------
    // Original command -> STEP-frequency law, with zero-crawl bug fixed.
    // Command sign is applied only after magnitude calculation and saturation.
    // -------------------------------------------------------------------------
    logic [31:0] cmd_mag;
    logic [31:0] rate_calc;
    logic [31:0] rate_low_knee;
    logic [15:0] rate_mag;
    logic signed [15:0] speed_from_cmd;
    logic rate_saturated;

    always_comb begin
        if (cmd_filtered == 32'sh80000000)
            cmd_mag = 32'h80000000;
        else if (cmd_filtered[31])
            cmd_mag = $unsigned(-$signed(cmd_filtered));
        else
            cmd_mag = $unsigned(cmd_filtered);

        // Piecewise command -> speed law.
        //   0..30000 : EXACT legacy response = 10 + cmd/16
        //   >30000   : continue continuously with a steeper /4 slope
        // This leaves equilibrium behavior unchanged while providing enough
        // recovery authority to reach the 4000 qtr-equivalent STEP/s limit.
        // The motor driver itself uses only 1/16, 1/8 and 1/4 modes.
        rate_low_knee = MIN_NONZERO_HZ + (HIGH_SPEED_KNEE_CMD / CMD_TO_HZ_DIV);
        rate_saturated = 1'b0;
        if (cmd_mag == 32'd0) begin
            rate_calc = 32'd0;
            rate_mag  = 16'd0;
        end else begin
            if (cmd_mag <= HIGH_SPEED_KNEE_CMD)
                rate_calc = MIN_NONZERO_HZ + (cmd_mag / CMD_TO_HZ_DIV);
            else
                rate_calc = rate_low_knee + ((cmd_mag - HIGH_SPEED_KNEE_CMD) / HIGH_SPEED_DIV);

            if (rate_calc > MAX_STEP_HZ) begin
                rate_mag = MAX_STEP_HZ[15:0];
                rate_saturated = 1'b1;
            end else begin
                rate_mag = rate_calc[15:0];
            end
        end

        if (rate_mag == 0)
            speed_from_cmd = 16'sd0;
        else if (cmd_filtered[31])
            speed_from_cmd = -$signed({1'b0, rate_mag[14:0]});
        else
            speed_from_cmd = $signed({1'b0, rate_mag[14:0]});
    end

    // -------------------------------------------------------------------------
    // State/update sequencing.
    // -------------------------------------------------------------------------
    logic arm_seen_low;
    logic [15:0] abs_pitch_tenths;

    always_comb begin
        if (filtered_pitch_tenths[15]) begin
            if (filtered_pitch_tenths == 16'sh8000)
                abs_pitch_tenths = 16'd32768;
            else
                abs_pitch_tenths = $unsigned(-$signed(filtered_pitch_tenths));
        end else begin
            abs_pitch_tenths = $unsigned(filtered_pitch_tenths);
        end
        saturated = rate_saturated ||
                    (pid_cmd >= PID_CMD_LIMIT) || (pid_cmd <= -PID_CMD_LIMIT) ||
                    (i_term >= I_LIMIT) || (i_term <= -I_LIMIT);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            pitch_latest          <= 16'sd0;
            gyro_latest_tenths    <= 16'sd0;
            pitch_hold_100hz      <= 16'sd0;
            pitch_prev_1ms        <= 16'sd0;
            filtered_pitch_tenths <= 16'sd0;
            gyro_prev_100hz       <= 16'sd0;
            gyro_filtered_tenths  <= 16'sd0;
            error_tenths          <= 16'sd0;
            p_term                <= 32'sd0;
            i_term                <= 32'sd0;
            i_accum_scaled        <= 48'sd0;
            d_term                <= 32'sd0;
            pid_cmd               <= 32'sd0;
            cmd_hist_index        <= 4'd0;
            cmd_sum               <= 37'sd0;
            cmd_filtered          <= 32'sd0;
            motor_speed_hz        <= 16'sd0;
            arm_seen_low          <= 1'b0;
            balance_active        <= 1'b0;
            fault_latched         <= 1'b0;
            for (idx = 0; idx < 16; idx = idx + 1)
                cmd_hist[idx] <= 32'sd0;
        end else begin
            // Capture each valid estimator result immediately.  pitch_valid is a
            // one-clock pulse and is NOT phase-aligned with our local 10-ms tick.
            // Requiring both on the same FPGA cycle can therefore leave the held
            // control pitch permanently at zero.  Keep the latest valid sample,
            // then decimate that held value at 100 Hz like the original project.
            if (sample_valid && sensor_ready) begin
                pitch_latest       <= pitch_tenths_in;
                gyro_latest_tenths <= gyro_tenths_now;
            end

            if (tick_10ms && sensor_ready) begin
                pitch_hold_100hz      <= pitch_latest;
                gyro_prev_100hz       <= gyro_latest_tenths;
                gyro_filtered_tenths  <= gyro_pair_sum >>> 1;
            end

            if (!operator_enable) begin
                arm_seen_low   <= 1'b1;
                balance_active <= 1'b0;
                fault_latched  <= 1'b0;
                p_term         <= 32'sd0;
                i_term         <= 32'sd0;
                i_accum_scaled <= 48'sd0;
                d_term         <= 32'sd0;
                pid_cmd        <= 32'sd0;
                cmd_sum        <= 37'sd0;
                cmd_hist_index <= 4'd0;
                cmd_filtered   <= 32'sd0;
                motor_speed_hz <= 16'sd0;
                for (idx = 0; idx < 16; idx = idx + 1)
                    cmd_hist[idx] <= 32'sd0;
            end else if (!sensor_ready) begin
                arm_seen_low   <= 1'b0;
                balance_active <= 1'b0;
                fault_latched  <= 1'b1;
                motor_speed_hz <= 16'sd0;
                i_term         <= 32'sd0;
                i_accum_scaled <= 48'sd0;
            end else begin
                // Integral runs independently at 100 kHz (10 us).  Error is
                // intentionally held between the 1-ms control updates.
                if (tick_i) begin
                    if (balance_active) begin
                        i_accum_scaled <= i_accum_next;
                        if ((i_accum_next / I_GAIN_DIV) > 48'sd2147483647)
                            i_term <= 32'sh7fffffff;
                        else if ((i_accum_next / I_GAIN_DIV) < -48'sd2147483647)
                            i_term <= -32'sd2147483647;
                        else
                            i_term <= (i_accum_next / I_GAIN_DIV);
                    end else begin
                        i_accum_scaled <= 48'sd0;
                        i_term         <= 32'sd0;
                    end
                end

                if (tick_1ms) begin
                    // 2-tap pitch average at 1 kHz, same structure as old project.
                    pitch_prev_1ms        <= pitch_hold_100hz;
                    filtered_pitch_tenths <= pitch_pair_sum >>> 1;

                    // Original error convention: setpoint - measured pitch.
                    error_tenths <= pitch_target_tenths_in - (pitch_pair_sum >>> 1);

                    if (balance_active) begin
                        p_term  <= p_next;
                        d_term  <= d_next;
                        pid_cmd <= pid_next;

                        // 16-sample command moving average at 1 kHz.
                        cmd_hist[cmd_hist_index] <= pid_next;
                        cmd_sum                  <= cmd_sum_next;
                        cmd_hist_index           <= cmd_hist_index + 1'b1;
                        cmd_filtered             <= cmd_sum_next >>> 4;
                        motor_speed_hz            <= speed_from_cmd;
                    end else begin
                        p_term         <= 32'sd0;
                        i_term         <= 32'sd0;
                        i_accum_scaled <= 48'sd0;
                        d_term         <= 32'sd0;
                        pid_cmd        <= 32'sd0;
                        cmd_sum        <= 37'sd0;
                        cmd_hist_index <= 4'd0;
                        cmd_filtered   <= 32'sd0;
                        motor_speed_hz <= 16'sd0;
                        for (idx = 0; idx < 16; idx = idx + 1)
                            cmd_hist[idx] <= 32'sd0;
                    end
                end

                if (balance_active && abs_pitch_tenths >= FALL_LIMIT_TENTHS) begin
                    arm_seen_low   <= 1'b0;
                    balance_active <= 1'b0;
                    fault_latched  <= 1'b1;
                    motor_speed_hz <= 16'sd0;
                    i_term         <= 32'sd0;
                    i_accum_scaled <= 48'sd0;
                end else if (!balance_active && arm_seen_low &&
                             abs_pitch_tenths <= START_WINDOW_TENTHS) begin
                    balance_active <= 1'b1;
                    fault_latched  <= 1'b0;
                    motor_speed_hz <= 16'sd0;
                end
            end
        end
    end

endmodule
