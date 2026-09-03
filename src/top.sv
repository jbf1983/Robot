// -----------------------------------------------------------------------------
// Robot balance V70 cleanup - behavior-preserving cleanup of validated V69.
// Both A4988 drivers always use the same adaptive 1/16, 1/8 or 1/4 mode.
// The common microstep mode is selected from the balance command BEFORE steering.
// Dabble joystick Y commands the outer speed loop; X commands differential steering.
// Speed-I updates at neutral and is held during commanded motion.
// -----------------------------------------------------------------------------
module top (
    input  logic CLK,
    input  logic RESETn,
    input  logic ROT_EN,
    input  logic TARGET_DISPLAY,  // physical SW2: show dynamic pitch target
    input  logic BATT_DISPLAY,    // physical SW3: show LiPo voltage on 7-segment displays
    input  logic BT_DISPLAY,      // physical SW4: HM-10 / Dabble bring-up display


    output logic LED9,
    output logic LED8,
    output logic LED7,
    output logic LED6,
    output logic LED5,
    output logic LED4,
    output logic LED3,
    output logic LED2,
    output logic LED1,
    output logic LED0,

    output logic MPU9250_CSn,
    output logic MPU9250_SCLK,
    input  logic MPU9250_MISO,
    input  logic MPU9250_INT,       // corrected direction; same physical PIN_AB10
    input  logic MPU9250_FSYNC,
    output logic MPU9250_MOSI,

    output logic MR_A4988_STEP,
    output logic MR_A4988_DIR,
    output logic MR_A4988_MS1,
    output logic MR_A4988_MS2,
    output logic MR_A4988_MS3,
    output logic MR_A4988_ENn,

    output logic ML_A4988_STEP,
    output logic ML_A4988_DIR,
    output logic ML_A4988_MS1,
    output logic ML_A4988_MS2,
    output logic ML_A4988_MS3,
    output logic ML_A4988_ENn,

    input  logic HM10_RX_SYS,
    output logic HM10_TX_SYS,

    output logic [0:6] HEX5,
    output logic [0:6] HEX4,
    output logic [0:6] HEX3,
    output logic [0:6] HEX2,
    output logic [0:6] HEX1,
    output logic [0:6] HEX0,
    output logic DP5,
    output logic DP4,
    output logic DP3,
    output logic DP2,
    output logic DP1,
    output logic DP0
);

    // -------------------------------------------------------------------------
    // Power-on reset (POR).
    //
    // After loading a .sof, RESETn may already be HIGH, so logic that only
    // initializes inside `if (rst)` would otherwise keep FPGA power-up zeros
    // until the physical reset switch is pressed. MAX10/Quartus synthesizes
    // these declaration initial values into the configuration image.
    // Hold the whole design in reset for 20 ms after every FPGA configuration.
    // The physical RESETn input still works normally and restarts the POR delay.
    // -------------------------------------------------------------------------
    localparam integer POR_CYCLES = 1_000_000; // 20 ms @ 50 MHz
    localparam integer POR_CNT_W  = $clog2(POR_CYCLES + 1);

    logic [POR_CNT_W-1:0] por_count = '0;
    logic                 por_active = 1'b1;
    logic                 rst;

    always_ff @(posedge CLK) begin
        if (!RESETn) begin
            por_count  <= '0;
            por_active <= 1'b1;
        end else if (por_active) begin
            if (por_count >= POR_CYCLES-1) begin
                por_count  <= por_count;
                por_active <= 1'b0;
            end else begin
                por_count <= por_count + 1'b1;
            end
        end
    end

    assign rst = (~RESETn) | por_active;

    // -------------------------------------------------------------------------
    // Synchronize the operator-enable switch before it enters control logic.
    // Two flip-flops eliminate metastability risk from the mechanical switch.
    // -------------------------------------------------------------------------
    logic rot_en_meta;
    logic rot_en_sync;

    always_ff @(posedge CLK) begin
        if (rst) begin
            rot_en_meta <= 1'b0;
            rot_en_sync <= 1'b0;
        end else begin
            rot_en_meta <= ROT_EN;
            rot_en_sync <= rot_en_meta;
        end
    end

    // -------------------------------------------------------------------------
    // LiPo battery protection - restored from the original Robot_new ADC path.
    // Original scaling/channel are preserved, with filtering + hysteresis:
    //   cutoff <3.40 V for 300 ms; re-enable >3.55 V for 500 ms.
    // -------------------------------------------------------------------------
    logic        battery_ok;
    logic        battery_valid;
    logic        battery_fault;
    logic        battery_adc_timeout;
    logic [12:0] battery_mv;

    max10_battery_monitor u_battery (
        .clk50         (CLK),
        .rst           (rst),
        .battery_ok    (battery_ok),
        .battery_valid (battery_valid),
        .battery_fault (battery_fault),
        .adc_timeout   (battery_adc_timeout),
        .battery_mv    (battery_mv)
    );


    // -------------------------------------------------------------------------
    // HM-10 / Dabble GamePad communication and diagnostics. Joystick motion
    // is converted into a longitudinal speed target plus differential steering.
    // Original Robot_new hardware: HM10 RX on PIN_W5, TX on PIN_AA15, 115200.
    // -------------------------------------------------------------------------
    logic [7:0] hm10_rx_byte;
    logic       hm10_rx_valid;
    logic       hm10_rx_error;
    logic       hm10_tx_start;
    logic [7:0] hm10_tx_byte;
    logic       hm10_tx_busy;
    logic       hm10_tx_done;

    logic       dabble_uart_seen;
    logic [7:0] dabble_last_uart_byte;
    logic       dabble_gamepad_seen;
    logic       dabble_gamepad_alive;
    logic       dabble_gamepad_frame_pulse;
    logic [8:0] dabble_joystick_angle_deg;
    logic [2:0] dabble_joystick_radius;
    logic [7:0] dabble_gamepad_buttons;

    uart_rx_8n1 #(
        .CLK_HZ (50_000_000),
        .BAUD   (115_200)
    ) u_hm10_uart_rx (
        .clk           (CLK),
        .rst           (rst),
        .rx            (HM10_RX_SYS),
        .data          (hm10_rx_byte),
        .valid         (hm10_rx_valid),
        .framing_error (hm10_rx_error)
    );

    uart_tx_8n1 #(
        .CLK_HZ (50_000_000),
        .BAUD   (115_200)
    ) u_hm10_uart_tx (
        .clk   (CLK),
        .rst   (rst),
        .start (hm10_tx_start),
        .data  (hm10_tx_byte),
        .tx    (HM10_TX_SYS),
        .busy  (hm10_tx_busy),
        .done  (hm10_tx_done)
    );

    dabble_bringup #(
        .CLK_HZ           (50_000_000),
        .ALIVE_MS         (500),
        .FRAME_TIMEOUT_MS (50)
    ) u_dabble_bringup (
        .clk                 (CLK),
        .rst                 (rst),
        .rx_byte             (hm10_rx_byte),
        .rx_valid            (hm10_rx_valid),
        .rx_framing_error    (hm10_rx_error),
        .tx_busy             (hm10_tx_busy),
        .tx_done             (hm10_tx_done),
        .tx_start            (hm10_tx_start),
        .tx_byte             (hm10_tx_byte),
        .uart_seen           (dabble_uart_seen),
        .last_uart_byte      (dabble_last_uart_byte),
        .gamepad_seen        (dabble_gamepad_seen),
        .gamepad_alive       (dabble_gamepad_alive),
        .gamepad_frame_pulse (dabble_gamepad_frame_pulse),
        .joystick_angle_deg  (dabble_joystick_angle_deg),
        .joystick_radius     (dabble_joystick_radius),
        .gamepad_buttons     (dabble_gamepad_buttons)
    );

    // -------------------------------------------------------------------------
    // Runtime tuning from Dabble GamePad buttons (Kp, Ki, speed-I strength).
    // Official button byte mapping in joystick mode:
    // Start bit0, Select bit1, Triangle bit2, Circle bit3, Cross bit4, Square bit5.
    // -------------------------------------------------------------------------
    logic [7:0]         kp_value, ki_value, kd_value;
    logic [7:0]         speed_i_strength;
    logic               tuning_display_active;
    logic [1:0]         tuning_display_mode;

    dabble_tuning #(
        .CLK_HZ                  (50_000_000),
        .DISPLAY_MS              (5000),
        .KP_INIT                  (150),
        .KI_INIT                  (20),
        .KD_INIT                  (0),
        .SPEED_I_INIT            (100)
    ) u_dabble_tuning (
        .clk                (CLK),
        .rst                (rst),
        .gamepad_buttons    (dabble_gamepad_buttons),
        .kp_value           (kp_value),
        .ki_value           (ki_value),
        .kd_value           (kd_value),
        .speed_i_strength   (speed_i_strength),
        .display_active     (tuning_display_active),
        .display_mode       (tuning_display_mode)
    );

    logic signed [15:0] ax, ay, az;
    logic signed [15:0] gx, gy, gz;
    logic signed [15:0] imu_temperature;
    logic               imu_sample_valid;
    logic               imu_ok;
    logic               imu_error;
    logic [7:0]         who_am_i;

    logic signed [31:0] pitch_q16;
    logic signed [15:0] pitch_tenths_raw;
    logic signed [15:0] pitch_tenths;
    logic signed [15:0] accel_pitch_tenths;
    logic signed [15:0] gyro_pitch_tenths;
    logic               pitch_valid;
    logic               calibrated;
    logic signed [31:0] gyro_bias_q12;
    logic               accel_trusted;

    // MPU9250 configuration readback diagnostics.
    logic [7:0]         config_rb;
    logic [7:0]         gyro_config_rb;
    logic [7:0]         accel_config_rb;
    logic [7:0]         accel_config2_rb;
    logic               config_verified;

    // -------------------------------------------------------------------------
    // MPU9250
    // -------------------------------------------------------------------------
    mpu9250 #(
        .CLK_HZ   (50_000_000),
        .SPI_HZ   (500_000),
        .USE_DRDY (1'b0)
    ) u_mpu9250 (
        .clk         (CLK),
        .rst         (rst),
        .mpu_cs_n    (MPU9250_CSn),
        .mpu_sclk    (MPU9250_SCLK),
        .mpu_mosi    (MPU9250_MOSI),
        .mpu_miso    (MPU9250_MISO),
        .mpu_int     (MPU9250_INT),
        .ax          (ax),
        .ay          (ay),
        .az          (az),
        .temperature (imu_temperature),
        .gx          (gx),
        .gy          (gy),
        .gz          (gz),
        .sample_valid(imu_sample_valid),
        .imu_ok      (imu_ok),
        .imu_error   (imu_error),
        .who_am_i    (who_am_i),
        .config_rb       (config_rb),
        .gyro_config_rb  (gyro_config_rb),
        .accel_config_rb (accel_config_rb),
        .accel_config2_rb(accel_config2_rb),
        .config_verified (config_verified)
    );

    // -------------------------------------------------------------------------
    // Pitch: legacy axes X/Z + Gyro Y, but now pure RTL complementary fusion.
    // -------------------------------------------------------------------------
    pitch_estimator #(
        .CAL_SAMPLES (4096)
    ) u_pitch (
        .clk          (CLK),
        .rst          (rst),
        .sample_valid (imu_sample_valid),
        .ax           (ax),
        .ay           (ay),
        .az           (az),
        .gy           (gy),
        .gyro_config_rb(gyro_config_rb),
        .pitch_q16    (pitch_q16),
        .pitch_tenths (pitch_tenths_raw),
        .accel_pitch_tenths(accel_pitch_tenths),
        .gyro_pitch_tenths (gyro_pitch_tenths),
        .pitch_valid  (pitch_valid),
        .calibrated   (calibrated),
        .gyro_bias_q12(gyro_bias_q12),
        .accel_trusted(accel_trusted)
    );

    // No artificial pitch offset. Mechanical imbalance is learned by the
    // outer speed integral at neutral and its learned value is held in motion.
    always_comb begin
        pitch_tenths = pitch_tenths_raw;
    end


    // -------------------------------------------------------------------------
    // Dabble joystick command layer.
    // Up/down => wheel-speed target through the OUTER speed PI.
    // Left/right => differential steering added after the balance controller.
    // ROT_EN off => commands return to zero after the 2-FF synchronizer. Once a Dabble joystick frame has
    // been received, the LAST command is held continuously until Dabble sends
    // another joystick value (including center). This is required because the
    // iOS GamePad may send joystick frames only when the value changes.
    // After enable, joystick must pass through center before it arms.
    // -------------------------------------------------------------------------
    logic signed [15:0] joystick_speed_target_rpm_tenths;
    logic signed [15:0] joystick_turn_hz;
    logic               joystick_control_armed;
    logic signed [15:0] joystick_x_q8, joystick_y_q8;

    joystick_command #(
        .CLK_HZ                  (50_000_000),
        .UPDATE_MS               (10),
        .DEADZONE_RADIUS         (1),
        .MAX_SPEED_RPM_TENTHS    (1500),
        .MAX_TURN_HZ             (700),
        .SPEED_SLEW_RPM_TENTHS   (20),
        .TURN_SLEW_HZ            (50),
        .FORWARD_INVERT           (1'b1),
        .INVERT_TURN_FORWARD      (1'b1)
    ) u_joystick_command (
        .clk                     (CLK),
        .rst                     (rst),
        .robot_enable            (rot_en_sync && battery_ok && sensor_ready),
        .gamepad_seen            (dabble_gamepad_seen),
        .joystick_angle_deg      (dabble_joystick_angle_deg),
        .joystick_radius         (dabble_joystick_radius),
        .speed_target_rpm_tenths (joystick_speed_target_rpm_tenths),
        .turn_hz                 (joystick_turn_hz),
        .control_armed           (joystick_control_armed),
        .joystick_x_q8           (joystick_x_q8),
        .joystick_y_q8           (joystick_y_q8)
    );

    // -------------------------------------------------------------------------
    // Old-style pitch PID rebuilt from the original VHDL timing/scaling.
    // Runtime Kp/Ki come from Dabble; Kd remains initialized at 0.
    // Integral updates every 10 us with /1000 scaling.
    // -------------------------------------------------------------------------
    logic signed [15:0] motor_speed_hz;
    logic signed [15:0] motor_speed_right_hz;
    logic signed [15:0] motor_speed_left_hz;
    logic signed [15:0] steering_applied_hz;
    logic               balance_active;
    logic               balance_fault;
    logic               balance_saturated;
    logic signed [15:0] filtered_pitch_tenths;

    logic sensor_ready;
    assign sensor_ready = imu_ok && config_verified && calibrated && !imu_error;

    // Slow outer wheel-speed loop. Joystick Y commands the longitudinal target;
    // the resulting balance command also drives the COMMON microstep selection.
    logic signed [15:0] speed_pitch_target_tenths;
    logic signed [15:0] wheel_speed_rpm_tenths;
    logic               speed_loop_saturated;

    balance_controller #(
        .CLK_HZ                  (50_000_000),
        .START_WINDOW_TENTHS     (80),
        .FALL_LIMIT_TENTHS       (450),
        .PID_CMD_LIMIT           (100000),
        .I_LIMIT                 (30000),
        .MAX_STEP_HZ             (4000),
        .CMD_TO_HZ_DIV           (16),
        .HIGH_SPEED_KNEE_CMD    (30000),
        .HIGH_SPEED_DIV         (4),
        .MIN_NONZERO_HZ          (10)
    ) u_balance (
        .clk                     (CLK),
        .rst                     (rst),
        .sample_valid            (pitch_valid),
        .sensor_ready            (sensor_ready),
        .operator_enable         (rot_en_sync && battery_ok),
        .pitch_target_tenths_in  (speed_pitch_target_tenths),
        .pitch_tenths_in         (pitch_tenths),
        .gyro_y_raw              (gy),
        .gyro_bias_q12           (gyro_bias_q12),
        .kp_value                (kp_value),
        .ki_value                (ki_value),
        .kd_value                (kd_value),
        .motor_speed_hz          (motor_speed_hz),
        .balance_active          (balance_active),
        .fault_latched           (balance_fault),
        .saturated               (balance_saturated),
        .filtered_pitch_tenths   (filtered_pitch_tenths)
    );

    // -------------------------------------------------------------------------
    // Differential steering mixer. Preserve the common balance command exactly:
    //   right = common - turn
    //   left  = common + turn
    // Turn is dynamically reduced near the +/-4000 motor limit so one wheel
    // never clips independently and disturbs the average balance command.
    // -------------------------------------------------------------------------
    logic signed [16:0] common_abs_wide;
    logic signed [16:0] turn_abs_wide;
    logic signed [16:0] turn_margin_wide;
    logic signed [16:0] turn_mag_applied_wide;
    logic signed [16:0] turn_signed_applied_wide;
    logic signed [16:0] right_mix_wide, left_mix_wide;

    always_comb begin
        if (motor_speed_hz[15])
            common_abs_wide = -$signed({motor_speed_hz[15],motor_speed_hz});
        else
            common_abs_wide =  $signed({motor_speed_hz[15],motor_speed_hz});

        if (joystick_turn_hz[15])
            turn_abs_wide = -$signed({joystick_turn_hz[15],joystick_turn_hz});
        else
            turn_abs_wide =  $signed({joystick_turn_hz[15],joystick_turn_hz});

        if (common_abs_wide >= 17'sd4000)
            turn_margin_wide = 17'sd0;
        else
            turn_margin_wide = 17'sd4000 - common_abs_wide;

        if (turn_abs_wide > turn_margin_wide)
            turn_mag_applied_wide = turn_margin_wide;
        else
            turn_mag_applied_wide = turn_abs_wide;

        if (joystick_turn_hz[15])
            turn_signed_applied_wide = -turn_mag_applied_wide;
        else
            turn_signed_applied_wide = turn_mag_applied_wide;

        right_mix_wide = $signed({motor_speed_hz[15],motor_speed_hz}) - turn_signed_applied_wide;
        left_mix_wide  = $signed({motor_speed_hz[15],motor_speed_hz}) + turn_signed_applied_wide;

        motor_speed_right_hz = right_mix_wide[15:0];
        motor_speed_left_hz  = left_mix_wide[15:0];
        steering_applied_hz  = turn_signed_applied_wide[15:0];
    end

    // -------------------------------------------------------------------------
    // COMMON adaptive microstep manager.
    //
    // Critical V56 change: steering is NOT used to select microstep. The mode
    // request is based only on the common balance command (motor_speed_hz) before
    // differential steering is added. Both axes therefore remain in exactly the
    // same resolution while pivoting near equilibrium.
    //
    // Hysteresis retained from the stable adaptive versions:
    //   1/16 -> 1/8 at >=150 ; 1/8 -> 1/16 at <=80
    //   1/8  -> 1/4 at >=500 ; 1/4 -> 1/8  at <=300
    //
    // A transition is committed only when BOTH motor drivers report that they
    // are simultaneously ready (STEP low + compatible electrical position).
    // -------------------------------------------------------------------------
    logic [2:0]         common_microstep_request;
    logic [15:0]        common_balance_abs_hz;
    logic               common_mode_change_enable;
    logic               mr_mode_ready, ml_mode_ready;
    logic [2:0]         mr_mode, ml_mode;
    logic [15:0]        mr_step_rate_hz, ml_step_rate_hz;
    logic signed [31:0] mr_position_16, ml_position_16;

    always_comb begin
        if (motor_speed_hz == 16'sh8000)
            common_balance_abs_hz = 16'd32768;
        else if (motor_speed_hz[15])
            common_balance_abs_hz = $unsigned(-$signed(motor_speed_hz));
        else
            common_balance_abs_hz = $unsigned(motor_speed_hz);
    end

    // Move only one adjacent microstep level at a time, and wait until both
    // physical drivers have reached the current request before requesting the
    // next level. This guarantees synchronized transitions even on large PID
    // command jumps.
    always_ff @(posedge CLK) begin
        if (rst) begin
            common_microstep_request <= 3'd0; // 1/16
        end else if ((mr_mode == common_microstep_request) &&
                     (ml_mode == common_microstep_request)) begin
            unique case (common_microstep_request)
                3'd0: begin // 1/16
                    if (common_balance_abs_hz >= 16'd150)
                        common_microstep_request <= 3'd1;
                end
                3'd1: begin // 1/8
                    if (common_balance_abs_hz <= 16'd80)
                        common_microstep_request <= 3'd0;
                    else if (common_balance_abs_hz >= 16'd500)
                        common_microstep_request <= 3'd2;
                end
                3'd2: begin // 1/4
                    if (common_balance_abs_hz <= 16'd300)
                        common_microstep_request <= 3'd1;
                end
                default: common_microstep_request <= 3'd0;
            endcase
        end
    end

    assign common_mode_change_enable =
        mr_mode_ready && ml_mode_ready &&
        ((mr_mode != common_microstep_request) ||
         (ml_mode != common_microstep_request));

    a4988_axis #(
        .CLK_HZ              (50_000_000),
        .REVERSE_DIR         (1'b0),
        .STEP_HIGH_US        (2),
        .DIR_SETUP_US        (5),
        .MAX_QTR_EQ_HZ              (4000),
        .MAX_ELECTRICAL_STEP_HZ      (5000),
        .EIGHTH_FROM_16_ENTER_HZ     (150),
        .SIXTEENTH_FROM_8_ENTER_HZ   (80),
        .QUARTER_FROM_8_ENTER_HZ     (500),
        .EIGHTH_FROM_4_ENTER_HZ      (300),
        .MODE_SETUP_US               (5)
    ) u_motor_right (
        .clk            (CLK),
        .rst            (rst),
        .enable         (balance_active),
        .emergency_stop (!sensor_ready || balance_fault || !battery_ok),
        .speed_hz       (motor_speed_right_hz),
        .requested_microstep_mode (common_microstep_request),
        .common_mode_change_enable(common_mode_change_enable),
        .step           (MR_A4988_STEP),
        .dir            (MR_A4988_DIR),
        .ms1            (MR_A4988_MS1),
        .ms2            (MR_A4988_MS2),
        .ms3            (MR_A4988_MS3),
        .enn            (MR_A4988_ENn),
        .microstep_mode (mr_mode),
        .mode_change_ready(mr_mode_ready),
        .step_rate_hz   (mr_step_rate_hz),
        .position_16    (mr_position_16)
    );

    a4988_axis #(
        .CLK_HZ              (50_000_000),
        .REVERSE_DIR         (1'b1),
        .STEP_HIGH_US        (2),
        .DIR_SETUP_US        (5),
        .MAX_QTR_EQ_HZ              (4000),
        .MAX_ELECTRICAL_STEP_HZ      (5000),
        .EIGHTH_FROM_16_ENTER_HZ     (150),
        .SIXTEENTH_FROM_8_ENTER_HZ   (80),
        .QUARTER_FROM_8_ENTER_HZ     (500),
        .EIGHTH_FROM_4_ENTER_HZ      (300),
        .MODE_SETUP_US               (5)
    ) u_motor_left (
        .clk            (CLK),
        .rst            (rst),
        .enable         (balance_active),
        .emergency_stop (!sensor_ready || balance_fault || !battery_ok),
        .speed_hz       (motor_speed_left_hz),
        .requested_microstep_mode (common_microstep_request),
        .common_mode_change_enable(common_mode_change_enable),
        .step           (ML_A4988_STEP),
        .dir            (ML_A4988_DIR),
        .ms1            (ML_A4988_MS1),
        .ms2            (ML_A4988_MS2),
        .ms3            (ML_A4988_MS3),
        .enn            (ML_A4988_ENn),
        .microstep_mode (ml_mode),
        .mode_change_ready(ml_mode_ready),
        .step_rate_hz   (ml_step_rate_hz),
        .position_16    (ml_position_16)
    );

    // -------------------------------------------------------------------------
    // Outer speed loop: fixed 0 rpm target. position_16 is invariant across
    // adaptive microstep changes: 200 full steps * 16 = 3200 position units/rev.
    // -------------------------------------------------------------------------
    speed_controller #(
        .CLK_HZ               (50_000_000),
        .UPDATE_MS            (100),
        .STEPS_PER_REV        (3200),
        .SPEED_KP_NUM         (1),
        .SPEED_KP_DIV         (20),
        .SPEED_KI_SHIFT       (7),
        .I_PITCH_LIMIT_TENTHS (35),
        .PITCH_LIMIT_TENTHS   (200)
    ) u_speed_controller (
        .clk                  (CLK),
        .rst                  (rst),
        .enable               (balance_active),
        .position_right       (mr_position_16),
        .position_left        (ml_position_16),
        .speed_target_rpm_tenths(joystick_speed_target_rpm_tenths),
        .speed_ki_strength    (speed_i_strength),
        .pitch_target_tenths  (speed_pitch_target_tenths),
        .speed_rpm_tenths     (wheel_speed_rpm_tenths),
        .saturated            (speed_loop_saturated)
    );

    // -------------------------------------------------------------------------
    // Display: pitch normally. After a Dabble tuning command, the adjusted
    // Kp/Ki/speed-I value has highest priority for 5 seconds.
    // -------------------------------------------------------------------------
    logic [0:6] pHEX5,pHEX4,pHEX3,pHEX2,pHEX1,pHEX0;
    logic pDP5,pDP4,pDP3,pDP2,pDP1,pDP0;
    logic [0:6] gHEX5,gHEX4,gHEX3,gHEX2,gHEX1,gHEX0;
    logic gDP5,gDP4,gDP3,gDP2,gDP1,gDP0;

    pitch_display u_pitch_display (
        .imu_ok(imu_ok), .imu_error(imu_error), .calibrated(calibrated),
        .pitch_tenths(pitch_tenths),
        .HEX5(pHEX5), .HEX4(pHEX4), .HEX3(pHEX3),
        .HEX2(pHEX2), .HEX1(pHEX1), .HEX0(pHEX0),
        .DP5(pDP5), .DP4(pDP4), .DP3(pDP3),
        .DP2(pDP2), .DP1(pDP1), .DP0(pDP0)
    );

    // Dynamic pitch-target display selected by physical SW2.
    logic [0:6] tHEX5,tHEX4,tHEX3,tHEX2,tHEX1,tHEX0;
    logic tDP5,tDP4,tDP3,tDP2,tDP1,tDP0;

    pitch_display u_target_display (
        .imu_ok(1'b1), .imu_error(1'b0), .calibrated(1'b1),
        .pitch_tenths(speed_pitch_target_tenths),
        .HEX5(tHEX5), .HEX4(tHEX4), .HEX3(tHEX3),
        .HEX2(tHEX2), .HEX1(tHEX1), .HEX0(tHEX0),
        .DP5(tDP5), .DP4(tDP4), .DP3(tDP3),
        .DP2(tDP2), .DP1(tDP1), .DP0(tDP0)
    );

    // Battery voltage display selected by SW3 (below temporary tuning display priority).
    // Format: x.xx (for example 4.08), no unit letter.
    logic [0:6] bHEX5,bHEX4,bHEX3,bHEX2,bHEX1,bHEX0;
    logic bDP5,bDP4,bDP3,bDP2,bDP1,bDP0;

    battery_display u_battery_display (
        .valid(battery_valid && !battery_adc_timeout),
        .battery_mv(battery_mv),
        .HEX5(bHEX5), .HEX4(bHEX4), .HEX3(bHEX3),
        .HEX2(bHEX2), .HEX1(bHEX1), .HEX0(bHEX0),
        .DP5(bDP5), .DP4(bDP4), .DP3(bDP3),
        .DP2(bDP2), .DP1(bDP1), .DP0(bDP0)
    );


    // Dabble bring-up display selected by SW4.
    //   no UART yet             -> "---"
    //   raw UART only           -> "b xxx" (last byte decimal)
    //   valid joystick frame    -> "A090r7" (angle/radius example)
    logic [0:6] dHEX5,dHEX4,dHEX3,dHEX2,dHEX1,dHEX0;
    logic dDP5,dDP4,dDP3,dDP2,dDP1,dDP0;

    dabble_display u_dabble_display (
        .uart_seen          (dabble_uart_seen),
        .last_uart_byte     (dabble_last_uart_byte),
        .gamepad_seen       (dabble_gamepad_seen),
        .gamepad_alive      (dabble_gamepad_alive),
        .joystick_angle_deg (dabble_joystick_angle_deg),
        .joystick_radius    (dabble_joystick_radius),
        .HEX5(dHEX5), .HEX4(dHEX4), .HEX3(dHEX3),
        .HEX2(dHEX2), .HEX1(dHEX1), .HEX0(dHEX0),
        .DP5(dDP5), .DP4(dDP4), .DP3(dDP3),
        .DP2(dDP2), .DP1(dDP1), .DP0(dDP0)
    );

    gain_display u_gain_display (
        .select_kp(tuning_display_active && (tuning_display_mode == 2'd1)),
        .select_ki(tuning_display_active && (tuning_display_mode == 2'd2)),
        .select_speed_i(tuning_display_active && (tuning_display_mode == 2'd3)),
        .kp_value(kp_value), .ki_value(ki_value), .speed_i_value(speed_i_strength),
        .HEX5(gHEX5), .HEX4(gHEX4), .HEX3(gHEX3),
        .HEX2(gHEX2), .HEX1(gHEX1), .HEX0(gHEX0),
        .DP5(gDP5), .DP4(gDP4), .DP3(gDP3),
        .DP2(gDP2), .DP1(gDP1), .DP0(gDP0)
    );

    always_comb begin
        if (tuning_display_active) begin
            HEX5=gHEX5; HEX4=gHEX4; HEX3=gHEX3; HEX2=gHEX2; HEX1=gHEX1; HEX0=gHEX0;
            DP5=gDP5; DP4=gDP4; DP3=gDP3; DP2=gDP2; DP1=gDP1; DP0=gDP0;
        end else if (BATT_DISPLAY) begin
            HEX5=bHEX5; HEX4=bHEX4; HEX3=bHEX3; HEX2=bHEX2; HEX1=bHEX1; HEX0=bHEX0;
            DP5=bDP5; DP4=bDP4; DP3=bDP3; DP2=bDP2; DP1=bDP1; DP0=bDP0;
        end else if (BT_DISPLAY) begin
            HEX5=dHEX5; HEX4=dHEX4; HEX3=dHEX3; HEX2=dHEX2; HEX1=dHEX1; HEX0=dHEX0;
            DP5=dDP5; DP4=dDP4; DP3=dDP3; DP2=dDP2; DP1=dDP1; DP0=dDP0;
        end else if (TARGET_DISPLAY) begin
            HEX5=tHEX5; HEX4=tHEX4; HEX3=tHEX3; HEX2=tHEX2; HEX1=tHEX1; HEX0=tHEX0;
            DP5=tDP5; DP4=tDP4; DP3=tDP3; DP2=tDP2; DP1=tDP1; DP0=tDP0;
        end else begin
            HEX5=pHEX5; HEX4=pHEX4; HEX3=pHEX3; HEX2=pHEX2; HEX1=pHEX1; HEX0=pHEX0;
            DP5=pDP5; DP4=pDP4; DP3=pDP3; DP2=pDP2; DP1=pDP1; DP0=pDP0;
        end
    end

    // -------------------------------------------------------------------------
    // Diagnostics.
    // -------------------------------------------------------------------------
    logic [9:0] sample_counter;

    always_ff @(posedge CLK) begin
        if (rst) begin
            sample_counter <= 10'd0;
        end else if (imu_sample_valid) begin
            sample_counter <= sample_counter + 1'b1;
        end
    end

    always_comb begin
        // Base diagnostics kept on LED0..LED4.
        LED0 = imu_ok && config_verified;
        LED1 = imu_error || balance_fault || battery_fault;
        LED2 = sample_counter[8];
        LED3 = calibrated;
        LED4 = balance_active;

        // Current common microstep mode. Both axes must remain synchronized.
        LED5 = (mr_mode == 3'd0) && (ml_mode == 3'd0); // both 1/16
        LED6 = (mr_mode == 3'd1) && (ml_mode == 3'd1); // both 1/8
        LED7 = (mr_mode == 3'd2) && (ml_mode == 3'd2); // both 1/4

        // Battery diagnostics on the two now-free high LEDs:
        //   LED8 = battery accepted / robot allowed to arm
        //   LED9 = battery fault (low voltage, ADC missing, or startup validation)
        LED8 = battery_ok;
        LED9 = battery_fault;

    end

    // MPU acquisition remains timer-driven as in the stable build.
    // HM-10/Dabble full control: forward/backward + left/right are active.
    // Common microstep remains synchronized between both motors.

endmodule
