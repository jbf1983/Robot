// -----------------------------------------------------------------------------
// MPU9250 SPI controller
// - 50 MHz default FPGA clock
// - SPI mode 0, 1 MHz
// - Gyro: +/-500 dps, DLPF ~41 Hz
// - Accel: +/-4 g, DLPF ~21.2 Hz
// - Sensor output rate: 1 kHz
// - Uses MPU9250 INT as latched RAW_DATA_READY
// - Reads AX..GZ as one 14-byte SPI burst
// -----------------------------------------------------------------------------
module mpu9250 #(
    parameter int CLK_HZ = 50_000_000,
    parameter int SPI_HZ = 1_000_000,
    parameter bit USE_DRDY = 1'b1
)(
    input  logic clk,
    input  logic rst,

    output logic mpu_cs_n,
    output logic mpu_sclk,
    output logic mpu_mosi,
    input  logic mpu_miso,
    input  logic mpu_int,

    output logic signed [15:0] ax,
    output logic signed [15:0] ay,
    output logic signed [15:0] az,
    output logic signed [15:0] temperature,
    output logic signed [15:0] gx,
    output logic signed [15:0] gy,
    output logic signed [15:0] gz,
    output logic               sample_valid,

    output logic       imu_ok,
    output logic       imu_error,
    output logic [7:0] who_am_i,

    // Configuration register readback for diagnostics / scale selection.
    output logic [7:0] config_rb,
    output logic [7:0] gyro_config_rb,
    output logic [7:0] accel_config_rb,
    output logic [7:0] accel_config2_rb,
    output logic       config_verified
);

    localparam logic [7:0] REG_SMPLRT_DIV    = 8'h19;
    localparam logic [7:0] REG_CONFIG        = 8'h1A;
    localparam logic [7:0] REG_GYRO_CONFIG   = 8'h1B;
    localparam logic [7:0] REG_ACCEL_CONFIG  = 8'h1C;
    localparam logic [7:0] REG_ACCEL_CONFIG2 = 8'h1D;
    localparam logic [7:0] REG_INT_PIN_CFG   = 8'h37;
    localparam logic [7:0] REG_INT_ENABLE    = 8'h38;
    localparam logic [7:0] REG_ACCEL_XOUT_H  = 8'h3B;
    localparam logic [7:0] REG_USER_CTRL     = 8'h6A;
    localparam logic [7:0] REG_PWR_MGMT_1    = 8'h6B;
    localparam logic [7:0] REG_PWR_MGMT_2    = 8'h6C;
    localparam logic [7:0] REG_WHO_AM_I      = 8'h75;

    localparam int BOOT_TICKS   = CLK_HZ / 10;     // 100 ms
    localparam int RESET_TICKS  = CLK_HZ / 10;     // 100 ms
    localparam int WAKE_TICKS   = CLK_HZ / 100;    // 10 ms
    localparam int SAMPLE_TICKS = CLK_HZ / 1000;   // 1 ms fallback
    // Keep CS high between distinct SPI commands. The legacy working RTL
    // used 10 us; retain that conservative margin during bring-up.
    localparam int CS_GAP_TICKS = CLK_HZ / 100_000; // 10 us

    // -------------------------------------------------------------------------
    // SPI byte engine
    // -------------------------------------------------------------------------
    logic       spi_start;
    logic [7:0] spi_tx;
    logic [7:0] spi_rx;
    logic       spi_busy;
    logic       spi_done;

    spi_master #(
        .CLK_HZ(CLK_HZ),
        .SPI_HZ(SPI_HZ)
    ) u_spi (
        .clk    (clk),
        .rst    (rst),
        .start  (spi_start),
        .tx_data(spi_tx),
        .rx_data(spi_rx),
        .busy   (spi_busy),
        .done   (spi_done),
        .sclk   (mpu_sclk),
        .mosi   (mpu_mosi),
        .miso   (mpu_miso)
    );

    // -------------------------------------------------------------------------
    // Synchronize MPU interrupt into FPGA clock domain.
    // INT is configured later as latched, active-high, clear-on-read.
    // -------------------------------------------------------------------------
    logic int_ff1, int_ff2;

    always_ff @(posedge clk) begin
        if (rst) begin
            int_ff1 <= 1'b0;
            int_ff2 <= 1'b0;
        end else begin
            int_ff1 <= mpu_int;
            int_ff2 <= int_ff1;
        end
    end

    // Optional 1 kHz fallback. Normally USE_DRDY stays 1.
    logic [31:0] sample_timer;
    logic        sample_tick;

    always_ff @(posedge clk) begin
        sample_tick <= 1'b0;
        if (rst || !imu_ok) begin
            sample_timer <= 32'd0;
        end else if (sample_timer == SAMPLE_TICKS-1) begin
            sample_timer <= 32'd0;
            sample_tick  <= 1'b1;
        end else begin
            sample_timer <= sample_timer + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Initialization table, executed after wake and WHO_AM_I check.
    // -------------------------------------------------------------------------
    localparam int INIT_COUNT = 8;

    function automatic logic [7:0] init_addr(input logic [3:0] index);
        case (index)
            4'd0: init_addr = REG_USER_CTRL;
            4'd1: init_addr = REG_PWR_MGMT_2;
            4'd2: init_addr = REG_CONFIG;
            4'd3: init_addr = REG_SMPLRT_DIV;
            4'd4: init_addr = REG_GYRO_CONFIG;
            4'd5: init_addr = REG_ACCEL_CONFIG;
            4'd6: init_addr = REG_ACCEL_CONFIG2;
            4'd7: init_addr = REG_INT_PIN_CFG;
            default: init_addr = 8'h00;
        endcase
    endfunction

    function automatic logic [7:0] init_data(input logic [3:0] index);
        case (index)
            4'd0: init_data = 8'h10; // USER_CTRL.I2C_IF_DIS = 1
            4'd1: init_data = 8'h00; // enable all accel/gyro axes
            4'd2: init_data = 8'h03; // gyro DLPF_CFG=3 -> ~41 Hz
            4'd3: init_data = 8'h00; // sample divider 0 -> 1 kHz
            4'd4: init_data = 8'h08; // gyro FS_SEL=01 -> +/-500 dps
            4'd5: init_data = 8'h08; // accel AFS_SEL=01 -> +/-4 g
            4'd6: init_data = 8'h04; // accel DLPF -> ~21.2 Hz
            // LATCH_INT_EN=1, INT_ANYRD_2CLEAR=1, active high, push-pull
            4'd7: init_data = 8'h30;
            default: init_data = 8'h00;
        endcase
    endfunction

    logic [7:0] raw [0:13];
    logic [3:0] init_index;
    logic [3:0] burst_index;
    logic [2:0] verify_index;
    logic [31:0] delay_counter;

    function automatic logic [7:0] verify_addr(input logic [2:0] index);
        case (index)
            3'd0: verify_addr = REG_CONFIG;
            3'd1: verify_addr = REG_GYRO_CONFIG;
            3'd2: verify_addr = REG_ACCEL_CONFIG;
            3'd3: verify_addr = REG_ACCEL_CONFIG2;
            default: verify_addr = 8'h00;
        endcase
    endfunction

    typedef enum logic [5:0] {
        ST_BOOT_WAIT,
        ST_RESET_ADDR_START, ST_RESET_ADDR_WAIT,
        ST_RESET_DATA_START, ST_RESET_DATA_WAIT,
        ST_RESET_WAIT,
        ST_WAKE_ADDR_START, ST_WAKE_ADDR_WAIT,
        ST_WAKE_DATA_START, ST_WAKE_DATA_WAIT,
        ST_WAKE_WAIT,
        ST_WHO_ADDR_START, ST_WHO_ADDR_WAIT,
        ST_WHO_DATA_START, ST_WHO_DATA_WAIT,
        ST_INIT_ADDR_START, ST_INIT_ADDR_WAIT,
        ST_INIT_DATA_START, ST_INIT_DATA_WAIT,
        ST_INTEN_ADDR_START, ST_INTEN_ADDR_WAIT,
        ST_INTEN_DATA_START, ST_INTEN_DATA_WAIT,
        ST_VERIFY_ADDR_START, ST_VERIFY_ADDR_WAIT,
        ST_VERIFY_DATA_START, ST_VERIFY_DATA_WAIT,
        ST_VERIFY_CHECK,
        ST_CS_GAP,
        ST_IDLE,
        ST_BURST_ADDR_START, ST_BURST_ADDR_WAIT,
        ST_BURST_DATA_START, ST_BURST_DATA_WAIT,
        ST_PUBLISH,
        ST_ERROR
    } state_t;

    state_t state;
    state_t gap_return_state;

    always_ff @(posedge clk) begin
        spi_start    <= 1'b0;
        sample_valid <= 1'b0;

        if (rst) begin
            state         <= ST_BOOT_WAIT;
            gap_return_state <= ST_BOOT_WAIT;
            mpu_cs_n      <= 1'b1;
            spi_tx        <= 8'h00;
            delay_counter <= 32'd0;
            init_index    <= 4'd0;
            burst_index   <= 4'd0;
            imu_ok        <= 1'b0;
            imu_error     <= 1'b0;
            who_am_i      <= 8'h00;
            config_rb       <= 8'h00;
            gyro_config_rb  <= 8'h00;
            accel_config_rb <= 8'h00;
            accel_config2_rb<= 8'h00;
            config_verified <= 1'b0;
            verify_index    <= 3'd0;
            ax            <= '0;
            ay            <= '0;
            az            <= '0;
            temperature   <= '0;
            gx            <= '0;
            gy            <= '0;
            gz            <= '0;
        end else begin
            case (state)
                ST_BOOT_WAIT: begin
                    mpu_cs_n <= 1'b1;
                    if (delay_counter == BOOT_TICKS-1) begin
                        delay_counter <= 32'd0;
                        state <= ST_RESET_ADDR_START;
                    end else delay_counter <= delay_counter + 1'b1;
                end

                // PWR_MGMT_1 <- 0x80 : device reset
                ST_RESET_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= REG_PWR_MGMT_1;
                    spi_start <= 1'b1;
                    state <= ST_RESET_ADDR_WAIT;
                end
                ST_RESET_ADDR_WAIT: if (spi_done) state <= ST_RESET_DATA_START;
                ST_RESET_DATA_START: if (!spi_busy) begin
                    spi_tx <= 8'h80;
                    spi_start <= 1'b1;
                    state <= ST_RESET_DATA_WAIT;
                end
                ST_RESET_DATA_WAIT: if (spi_done) begin
                    mpu_cs_n <= 1'b1;
                    delay_counter <= 32'd0;
                    state <= ST_RESET_WAIT;
                end
                ST_RESET_WAIT: begin
                    if (delay_counter == RESET_TICKS-1) begin
                        delay_counter <= 32'd0;
                        state <= ST_WAKE_ADDR_START;
                    end else delay_counter <= delay_counter + 1'b1;
                end

                // PWR_MGMT_1 <- 0x01 : wake and use X gyro PLL clock
                ST_WAKE_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= REG_PWR_MGMT_1;
                    spi_start <= 1'b1;
                    state <= ST_WAKE_ADDR_WAIT;
                end
                ST_WAKE_ADDR_WAIT: if (spi_done) state <= ST_WAKE_DATA_START;
                ST_WAKE_DATA_START: if (!spi_busy) begin
                    spi_tx <= 8'h01;
                    spi_start <= 1'b1;
                    state <= ST_WAKE_DATA_WAIT;
                end
                ST_WAKE_DATA_WAIT: if (spi_done) begin
                    mpu_cs_n <= 1'b1;
                    delay_counter <= 32'd0;
                    state <= ST_WAKE_WAIT;
                end
                ST_WAKE_WAIT: begin
                    if (delay_counter == WAKE_TICKS-1) begin
                        delay_counter <= 32'd0;
                        state <= ST_WHO_ADDR_START;
                    end else delay_counter <= delay_counter + 1'b1;
                end

                // WHO_AM_I should be 0x71 on MPU9250.
                ST_WHO_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= REG_WHO_AM_I | 8'h80;
                    spi_start <= 1'b1;
                    state <= ST_WHO_ADDR_WAIT;
                end
                ST_WHO_ADDR_WAIT: if (spi_done) state <= ST_WHO_DATA_START;
                ST_WHO_DATA_START: if (!spi_busy) begin
                    spi_tx <= 8'h00;
                    spi_start <= 1'b1;
                    state <= ST_WHO_DATA_WAIT;
                end
                ST_WHO_DATA_WAIT: if (spi_done) begin
                    mpu_cs_n <= 1'b1;
                    who_am_i <= spi_rx;
                    if (spi_rx == 8'h71) begin
                        init_index       <= 4'd0;
                        delay_counter    <= 32'd0;
                        gap_return_state <= ST_INIT_ADDR_START;
                        state            <= ST_CS_GAP;
                    end else begin
                        imu_error <= 1'b1;
                        state <= ST_ERROR;
                    end
                end

                // Generic initialization register write.
                ST_INIT_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= init_addr(init_index);
                    spi_start <= 1'b1;
                    state <= ST_INIT_ADDR_WAIT;
                end
                ST_INIT_ADDR_WAIT: if (spi_done) state <= ST_INIT_DATA_START;
                ST_INIT_DATA_START: if (!spi_busy) begin
                    spi_tx <= init_data(init_index);
                    spi_start <= 1'b1;
                    state <= ST_INIT_DATA_WAIT;
                end
                ST_INIT_DATA_WAIT: if (spi_done) begin
                    mpu_cs_n <= 1'b1;
                    delay_counter <= 32'd0;
                    if (init_index == INIT_COUNT-1) begin
                        gap_return_state <= ST_INTEN_ADDR_START;
                    end else begin
                        init_index       <= init_index + 1'b1;
                        gap_return_state <= ST_INIT_ADDR_START;
                    end
                    state <= ST_CS_GAP;
                end

                // INT_ENABLE <- 0x01 : RAW_DATA_READY
                ST_INTEN_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= REG_INT_ENABLE;
                    spi_start <= 1'b1;
                    state <= ST_INTEN_ADDR_WAIT;
                end
                ST_INTEN_ADDR_WAIT: if (spi_done) state <= ST_INTEN_DATA_START;
                ST_INTEN_DATA_START: if (!spi_busy) begin
                    spi_tx <= 8'h01;
                    spi_start <= 1'b1;
                    state <= ST_INTEN_DATA_WAIT;
                end
                ST_INTEN_DATA_WAIT: if (spi_done) begin
                    mpu_cs_n          <= 1'b1;
                    verify_index      <= 3'd0;
                    delay_counter     <= 32'd0;
                    gap_return_state  <= ST_VERIFY_ADDR_START;
                    state             <= ST_CS_GAP;
                end

                // Read back the key configuration registers. This removes any
                // ambiguity about the actual gyro/accel full-scale settings.
                ST_VERIFY_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= verify_addr(verify_index) | 8'h80;
                    spi_start <= 1'b1;
                    state <= ST_VERIFY_ADDR_WAIT;
                end
                ST_VERIFY_ADDR_WAIT: if (spi_done) begin
                    state <= ST_VERIFY_DATA_START;
                end
                ST_VERIFY_DATA_START: if (!spi_busy) begin
                    spi_tx <= 8'h00;
                    spi_start <= 1'b1;
                    state <= ST_VERIFY_DATA_WAIT;
                end
                ST_VERIFY_DATA_WAIT: if (spi_done) begin
                    mpu_cs_n <= 1'b1;
                    case (verify_index)
                        3'd0: config_rb        <= spi_rx;
                        3'd1: gyro_config_rb   <= spi_rx;
                        3'd2: accel_config_rb  <= spi_rx;
                        3'd3: accel_config2_rb <= spi_rx;
                        default: ;
                    endcase
                    delay_counter <= 32'd0;
                    if (verify_index == 3'd3) begin
                        gap_return_state <= ST_VERIFY_CHECK;
                    end else begin
                        verify_index     <= verify_index + 1'b1;
                        gap_return_state <= ST_VERIFY_ADDR_START;
                    end
                    state <= ST_CS_GAP;
                end
                ST_VERIFY_CHECK: begin
                    // Do not start acquisitions unless every critical register
                    // reads back exactly as programmed. This prevents a wrong
                    // full-scale setting from silently corrupting gyro scaling.
                    if ((config_rb == 8'h03) &&
                        (gyro_config_rb == 8'h08) &&
                        (accel_config_rb == 8'h08) &&
                        (accel_config2_rb == 8'h04)) begin
                        config_verified <= 1'b1;
                        imu_ok           <= 1'b1;
                        imu_error        <= 1'b0;
                        state            <= ST_IDLE;
                    end else begin
                        config_verified <= 1'b0;
                        imu_ok           <= 1'b0;
                        imu_error        <= 1'b1;
                        state            <= ST_ERROR;
                    end
                end

                // Conservative CS-high inter-command gap. This mirrors the
                // 10 us delay used by the legacy controller that was known to
                // communicate reliably with this exact board/module.
                ST_CS_GAP: begin
                    mpu_cs_n <= 1'b1;
                    if (delay_counter == CS_GAP_TICKS-1) begin
                        delay_counter <= 32'd0;
                        state <= gap_return_state;
                    end else begin
                        delay_counter <= delay_counter + 1'b1;
                    end
                end

                ST_IDLE: begin
                    if ((USE_DRDY && int_ff2) || (!USE_DRDY && sample_tick)) begin
                        burst_index <= 4'd0;
                        state <= ST_BURST_ADDR_START;
                    end
                end

                // Burst read 0x3B..0x48: accel XYZ, temp, gyro XYZ.
                ST_BURST_ADDR_START: if (!spi_busy) begin
                    mpu_cs_n <= 1'b0;
                    spi_tx <= REG_ACCEL_XOUT_H | 8'h80;
                    spi_start <= 1'b1;
                    state <= ST_BURST_ADDR_WAIT;
                end
                ST_BURST_ADDR_WAIT: if (spi_done) begin
                    burst_index <= 4'd0;
                    state <= ST_BURST_DATA_START;
                end
                ST_BURST_DATA_START: if (!spi_busy) begin
                    spi_tx <= 8'h00;
                    spi_start <= 1'b1;
                    state <= ST_BURST_DATA_WAIT;
                end
                ST_BURST_DATA_WAIT: if (spi_done) begin
                    raw[burst_index] <= spi_rx;
                    if (burst_index == 4'd13) begin
                        mpu_cs_n <= 1'b1;
                        state <= ST_PUBLISH;
                    end else begin
                        burst_index <= burst_index + 1'b1;
                        state <= ST_BURST_DATA_START;
                    end
                end

                ST_PUBLISH: begin
                    ax          <= $signed({raw[0],  raw[1]});
                    ay          <= $signed({raw[2],  raw[3]});
                    az          <= $signed({raw[4],  raw[5]});
                    temperature <= $signed({raw[6],  raw[7]});
                    gx          <= $signed({raw[8],  raw[9]});
                    gy          <= $signed({raw[10], raw[11]});
                    gz          <= $signed({raw[12], raw[13]});
                    sample_valid <= 1'b1;
                    state <= ST_IDLE;
                end

                ST_ERROR: begin
                    mpu_cs_n  <= 1'b1;
                    imu_ok    <= 1'b0;
                    imu_error <= 1'b1;
                end

                default: state <= ST_ERROR;
            endcase
        end
    end

endmodule
