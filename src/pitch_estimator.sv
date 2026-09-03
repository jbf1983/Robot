// -----------------------------------------------------------------------------
// Pitch estimator - robust sequenced complementary filter.
//
// Legacy robot convention:
//   accel pitch = -atan2(AccelX, AccelZ)
//   pitch rate  = +GyroY
//
// MPU configuration assumed:
//   gyro  +/-500 dps  => 65.5 LSB/(deg/s)
//   accel +/-4 g      => 8192 LSB/g
//   acquisition       => 1 kHz
//
// Important implementation detail:
// The CORDIC result is first LATCHED, and the fusion state is updated on the
// following FPGA clock. This makes the feedback path explicit and avoids any
// ambiguity about same-cycle CORDIC done/result timing.
// -----------------------------------------------------------------------------
module pitch_estimator #(
    parameter int CAL_SAMPLES = 4096
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               sample_valid,
    input  logic signed [15:0] ax,
    input  logic signed [15:0] ay,
    input  logic signed [15:0] az,
    input  logic signed [15:0] gy,
    input  logic        [7:0] gyro_config_rb,

    output logic signed [31:0] pitch_q16,
    output logic signed [15:0] pitch_tenths,
    output logic signed [15:0] accel_pitch_tenths,
    output logic signed [15:0] gyro_pitch_tenths,
    output logic               pitch_valid,
    output logic               calibrated,
    output logic signed [31:0] gyro_bias_q12,
    output logic               accel_trusted
);

    localparam int CAL_CNT_W = (CAL_SAMPLES <= 2) ? 1 : $clog2(CAL_SAMPLES);

    // 0.80g^2 .. 1.20g^2, with 1g = 8192 counts at +/-4g.
    localparam logic [33:0] ACC_NORM2_MIN = 34'd42949673;
    localparam logic [33:0] ACC_NORM2_MAX = 34'd96636764;

    // -------------------------------------------------------------------------
    // Acceleration norm (no sqrt required).
    // -------------------------------------------------------------------------
    logic signed [31:0] ax_ext, ay_ext, az_ext;
    logic signed [63:0] ax_mul, ay_mul, az_mul;
    logic        [33:0] acc_norm2;

    always_comb begin
        ax_ext = {{16{ax[15]}}, ax};
        ay_ext = {{16{ay[15]}}, ay};
        az_ext = {{16{az[15]}}, az};
        ax_mul = ax_ext * ax_ext;
        ay_mul = ay_ext * ay_ext;
        az_mul = az_ext * az_ext;
        acc_norm2 = ax_mul[33:0] + ay_mul[33:0] + az_mul[33:0];
    end

    // -------------------------------------------------------------------------
    // Accelerometer angle CORDIC.
    // -------------------------------------------------------------------------
    logic cordic_start, cordic_busy, cordic_done;
    logic signed [31:0] accel_angle_raw_q16;

    cordic_atan2 u_atan2 (
        .clk       (clk),
        .rst       (rst),
        .start     (cordic_start),
        .y_in      (ax),
        .x_in      (az),
        .busy      (cordic_busy),
        .done      (cordic_done),
        .angle_q16 (accel_angle_raw_q16)
    );

    // Latched CORDIC result: Q16.16 degrees, old robot sign convention.
    logic signed [31:0] accel_pitch_latched_q16;
    logic signed [47:0] accel_pitch_latched_q28;
    assign accel_pitch_latched_q28 =
        {{16{accel_pitch_latched_q16[31]}}, accel_pitch_latched_q16} <<< 12;

    // -------------------------------------------------------------------------
    // Gyro startup bias. Sum of exactly 4096 raw samples is mean*4096, i.e.
    // raw-LSB in Q12. CAL_SAMPLES is intentionally 4096 in the top level.
    // -------------------------------------------------------------------------
    logic signed [31:0] gyro_sum;
    logic [CAL_CNT_W-1:0] cal_count;

    // Current sample's corrected gyro, held while CORDIC runs, raw-LSB Q12.
    logic signed [31:0] gyro_corr_q12_hold;
    logic               accel_trusted_hold;

    // -------------------------------------------------------------------------
    // Angle states, Q20.28 degrees.
    // Fused and gyro-only are kept independently for diagnostics.
    // -------------------------------------------------------------------------
    logic signed [47:0] fused_q28;
    logic signed [47:0] gyro_only_q28;

    logic signed [47:0] gyro_corr_ext;
    logic signed [47:0] gyro_delta_q28;
    logic signed [47:0] fused_pred_q28;
    logic signed [47:0] fused_error_q28;
    logic signed [47:0] fused_next_q28;

    // Convert raw gyro Q12 to angle increment Q28 at 1 kHz.
    // Use the ACTUAL FS_SEL bits read back from GYRO_CONFIG:
    //   00 +/-250  dps : 131.0 LSB/(deg/s) -> factor ~0.500275
    //   01 +/-500  dps :  65.5 LSB/(deg/s) -> factor ~1.00055
    //   10 +/-1000 dps :  32.8 LSB/(deg/s) -> factor ~1.99805
    //   11 +/-2000 dps :  16.4 LSB/(deg/s) -> factor ~3.99610
    // Shift/add approximations avoid a multiplier and are far more accurate
    // than needed for this application.
    always_comb begin
        gyro_corr_ext = {{16{gyro_corr_q12_hold[31]}}, gyro_corr_q12_hold};

        case (gyro_config_rb[4:3])
            2'b00: gyro_delta_q28 = (gyro_corr_ext >>> 1) + (gyro_corr_ext >>> 12);
            2'b01: gyro_delta_q28 = gyro_corr_ext + (gyro_corr_ext >>> 11);
            2'b10: gyro_delta_q28 = (gyro_corr_ext <<< 1) - (gyro_corr_ext >>> 9);
            2'b11: gyro_delta_q28 = (gyro_corr_ext <<< 2) - (gyro_corr_ext >>> 8);
            default: gyro_delta_q28 = gyro_corr_ext;
        endcase

        fused_pred_q28  = fused_q28 + gyro_delta_q28;
        fused_error_q28 = accel_pitch_latched_q28 - fused_pred_q28;

        // Normal complementary correction: beta = 1/256.
        // At 1kHz this is ~0.256s time constant.
        fused_next_q28 = fused_pred_q28 + (fused_error_q28 >>> 8);
    end

    // -------------------------------------------------------------------------
    // Sequencing.
    // -------------------------------------------------------------------------
    logic fusion_pending;
    logic initialize_from_accel;

    always_ff @(posedge clk) begin
        cordic_start <= 1'b0;
        pitch_valid  <= 1'b0;

        if (rst) begin
            gyro_sum                 <= 32'sd0;
            cal_count                <= '0;
            calibrated               <= 1'b0;
            gyro_bias_q12            <= 32'sd0;
            gyro_corr_q12_hold       <= 32'sd0;
            accel_trusted_hold       <= 1'b0;
            accel_trusted            <= 1'b0;
            accel_pitch_latched_q16  <= 32'sd0;
            fused_q28                <= 48'sd0;
            gyro_only_q28            <= 48'sd0;
            fusion_pending           <= 1'b0;
            initialize_from_accel    <= 1'b0;
        end else begin

            // Start one CORDIC for each acquired IMU sample.
            if (sample_valid && !cordic_busy && !fusion_pending) begin
                cordic_start <= 1'b1;

                if (!calibrated) begin
                    if (cal_count == CAL_SAMPLES-1) begin
                        // For CAL_SAMPLES=4096 this directly is raw bias Q12.
                        gyro_bias_q12         <= gyro_sum + $signed(gy);
                        calibrated            <= 1'b1;
                        initialize_from_accel <= 1'b1;
                    end else begin
                        gyro_sum  <= gyro_sum + $signed(gy);
                        cal_count <= cal_count + 1'b1;
                    end
                end else begin
                    gyro_corr_q12_hold <=
                        ($signed({{16{gy[15]}}, gy}) <<< 12) - gyro_bias_q12;

                    accel_trusted_hold <=
                        (acc_norm2 >= ACC_NORM2_MIN) &&
                        (acc_norm2 <= ACC_NORM2_MAX);
                end
            end

            // CORDIC result is valid here. Latch it; do NOT update the feedback
            // state on this same event. Fusion occurs one FPGA clock later.
            if (cordic_done) begin
                accel_pitch_latched_q16 <= -accel_angle_raw_q16;
                fusion_pending          <= 1'b1;
            end

            if (fusion_pending) begin
                fusion_pending <= 1'b0;

                if (initialize_from_accel) begin
                    // Use the latched acceleration angle as absolute initial
                    // attitude after gyro calibration.
                    fused_q28             <= accel_pitch_latched_q28;
                    gyro_only_q28         <= accel_pitch_latched_q28;
                    initialize_from_accel <= 1'b0;
                    accel_trusted         <= 1'b1;
                    pitch_valid           <= 1'b1;
                end else if (calibrated) begin
                    // Gyro-only diagnostic state.
                    gyro_only_q28 <= gyro_only_q28 + gyro_delta_q28;

                    // Complementary feedback state.
                    if (accel_trusted_hold)
                        fused_q28 <= fused_next_q28;
                    else
                        fused_q28 <= fused_pred_q28;

                    accel_trusted <= accel_trusted_hold;
                    pitch_valid   <= 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Q28/Q16 conversions and display values.
    // -------------------------------------------------------------------------
    logic signed [31:0] gyro_only_q16;
    assign pitch_q16     = fused_q28     >>> 12;
    assign gyro_only_q16 = gyro_only_q28 >>> 12;

    function automatic logic signed [15:0] q16_to_tenths(
        input logic signed [31:0] angle_q16
    );
        logic signed [32:0] centered;
        logic [32:0] mag;
        logic [36:0] x10;
        logic [20:0] tenth_mag;
        begin
            centered = $signed(angle_q16);

            if (centered < 0)
                mag = $unsigned(-centered);
            else
                mag = $unsigned(centered);

            x10 = (mag << 3) + (mag << 1);
            tenth_mag = (x10 + 37'd32768) >> 16;

            if (centered < 0)
                q16_to_tenths = -$signed({1'b0, tenth_mag[14:0]});
            else
                q16_to_tenths =  $signed({1'b0, tenth_mag[14:0]});
        end
    endfunction

    always_comb begin
        pitch_tenths       = q16_to_tenths(pitch_q16);
        accel_pitch_tenths = q16_to_tenths(accel_pitch_latched_q16);
        gyro_pitch_tenths  = q16_to_tenths(gyro_only_q16);
    end

endmodule
