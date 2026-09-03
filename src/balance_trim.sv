// -----------------------------------------------------------------------------
// Automatic mechanical balance-angle learner.
//
// Goal: learn the robot's real zero-speed equilibrium angle without a hard-coded
// pitch offset.  Before trim_valid is asserted the existing outer speed PI is
// allowed to find the static equilibrium.  Once the robot has been genuinely
// stationary and pitch ~= pitch_target for a sustained period, that equilibrium
// angle is captured as balance_trim_tenths.
//
// After capture, the trim is refined very slowly only when:
//   - longitudinal speed target is zero,
//   - steering command is zero,
//   - measured wheel speed is close to zero,
//   - pitch is close to the current pitch target,
//   - current pitch target is already close to the learned trim.
// This prevents acceleration/deceleration lean from being learned as trim.
//
// Units: 0.1 degree and 0.1 rpm.
// -----------------------------------------------------------------------------
module balance_trim #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer UPDATE_MS = 100,
    parameter integer CAPTURE_STABLE_SAMPLES = 20, // 2.0 s @ 100 ms
    parameter integer MAX_STATIONARY_SPEED_TENTHS = 10, // 1.0 rpm
    parameter integer MAX_TRACK_ERROR_TENTHS = 5,       // 0.5 deg
    parameter integer MAX_DELTA_FOR_ADAPT_TENTHS = 5,   // target within 0.5 deg of trim
    parameter integer ADAPT_SHIFT = 5                   // 1/32 per 100 ms ~= 3.2 s tau
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               enable,
    input  logic signed [15:0] pitch_tenths,
    input  logic signed [15:0] pitch_target_tenths,
    input  logic signed [15:0] speed_rpm_tenths,
    input  logic signed [15:0] speed_target_rpm_tenths,
    input  logic signed [15:0] turn_hz,

    output logic signed [15:0] balance_trim_tenths,
    output logic               trim_valid,
    output logic               learning_active
);
    localparam integer UPDATE_DIV = (CLK_HZ / 1000) * UPDATE_MS;
    localparam integer CNT_W = (UPDATE_DIV <= 2) ? 1 : $clog2(UPDATE_DIV);
    localparam integer STABLE_CNT_W = (CAPTURE_STABLE_SAMPLES <= 2) ? 1 : $clog2(CAPTURE_STABLE_SAMPLES + 1);

    logic [CNT_W-1:0] tick_count;
    logic update_tick;

    logic signed [31:0] pitch_sum;
    logic [STABLE_CNT_W-1:0] stable_count;

    // Q8 in units of 0.1 degree for smooth post-capture adaptation.
    logic signed [31:0] trim_q8;
    logic signed [31:0] pitch_q8;
    logic signed [31:0] adapt_diff_q8;
    logic signed [31:0] adapt_step_q8;

    logic signed [16:0] speed_abs;
    logic signed [16:0] track_error_abs;
    logic signed [16:0] target_trim_error_abs;
    logic pre_capture_stable;
    logic post_capture_stable;

    always_ff @(posedge clk) begin
        if (rst) begin
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
        if (speed_rpm_tenths[15])
            speed_abs = -$signed({speed_rpm_tenths[15], speed_rpm_tenths});
        else
            speed_abs =  $signed({speed_rpm_tenths[15], speed_rpm_tenths});

        if (($signed(pitch_tenths) - $signed(pitch_target_tenths)) < 0)
            track_error_abs = -($signed({pitch_tenths[15],pitch_tenths}) -
                                $signed({pitch_target_tenths[15],pitch_target_tenths}));
        else
            track_error_abs =  ($signed({pitch_tenths[15],pitch_tenths}) -
                                $signed({pitch_target_tenths[15],pitch_target_tenths}));

        if (($signed(pitch_target_tenths) - $signed(balance_trim_tenths)) < 0)
            target_trim_error_abs = -($signed({pitch_target_tenths[15],pitch_target_tenths}) -
                                     $signed({balance_trim_tenths[15],balance_trim_tenths}));
        else
            target_trim_error_abs =  ($signed({pitch_target_tenths[15],pitch_target_tenths}) -
                                     $signed({balance_trim_tenths[15],balance_trim_tenths}));

        pre_capture_stable = enable &&
                             (speed_target_rpm_tenths == 16'sd0) &&
                             (turn_hz == 16'sd0) &&
                             (speed_abs <= MAX_STATIONARY_SPEED_TENTHS) &&
                             (track_error_abs <= MAX_TRACK_ERROR_TENTHS);

        post_capture_stable = pre_capture_stable &&
                              (target_trim_error_abs <= MAX_DELTA_FOR_ADAPT_TENTHS);

        pitch_q8      = $signed(pitch_tenths) <<< 8;
        adapt_diff_q8 = pitch_q8 - trim_q8;
        adapt_step_q8 = adapt_diff_q8 >>> ADAPT_SHIFT;

        learning_active = (!trim_valid && pre_capture_stable) ||
                          (trim_valid && post_capture_stable);
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            pitch_sum             <= 32'sd0;
            stable_count          <= '0;
            trim_q8               <= 32'sd0;
            balance_trim_tenths   <= 16'sd0;
            trim_valid            <= 1'b0;
        end else if (update_tick) begin
            if (!trim_valid) begin
                if (pre_capture_stable) begin
                    if (stable_count == CAPTURE_STABLE_SAMPLES-1) begin
                        // Include the current sample in the average.
                        trim_q8 <= (($signed(pitch_sum) + $signed(pitch_tenths)) /
                                    CAPTURE_STABLE_SAMPLES) <<< 8;
                        balance_trim_tenths <= (($signed(pitch_sum) + $signed(pitch_tenths)) /
                                                CAPTURE_STABLE_SAMPLES);
                        trim_valid   <= 1'b1;
                        stable_count <= '0;
                        pitch_sum    <= 32'sd0;
                    end else begin
                        pitch_sum    <= pitch_sum + $signed(pitch_tenths);
                        stable_count <= stable_count + 1'b1;
                    end
                end else begin
                    pitch_sum    <= 32'sd0;
                    stable_count <= '0;
                end
            end else begin
                stable_count <= '0;
                pitch_sum    <= 32'sd0;

                if (post_capture_stable) begin
                    trim_q8 <= trim_q8 + adapt_step_q8;
                    balance_trim_tenths <= (trim_q8 + adapt_step_q8) >>> 8;
                end else begin
                    balance_trim_tenths <= trim_q8 >>> 8;
                end
            end
        end
    end
endmodule
