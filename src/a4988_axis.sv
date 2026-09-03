// -----------------------------------------------------------------------------
// Reliable A4988 axis driver with EXTERNALLY synchronized 1/16 <-> 1/8 <-> 1/4
// microstepping.
//
// V66 transition fix:
//   Previous versions required BOTH axes to be simultaneously at a compatible
//   electrical position. A ready axis kept stepping, so two slightly different
//   wheel speeds could miss each other forever and leave the common mode stuck
//   in 1/16. In 1/16 that also clamps the mechanical command to 1250 qtr-eq Hz
//   because of the 5000 electrical STEP/s ceiling.
//
//   Each axis now LATCHES readiness:
//     1. when a new common mode is requested, reach the next compatible point;
//     2. stop there in ST_MODE_HOLD with STEP low;
//     3. keep mode_change_ready asserted until the other axis is also ready;
//     4. both axes change MS on the same 1-us tick, then wait MODE_SETUP_US.
//
//   If an axis is stopped on an incompatible fine-step position, one alignment
//   microstep is generated so the transition cannot deadlock. At most one such
//   step is needed for each adjacent transition (1/16->1/8 or 1/8->1/4).
//
// speed_hz is a MECHANICAL speed command expressed as the STEP frequency that
// would be required in fixed 1/4 mode (quarter-step-equivalent Hz):
//   1/16 : electrical STEP rate = 4 * speed_hz
//   1/8  : electrical STEP rate = 2 * speed_hz
//   1/4  : electrical STEP rate = 1 * speed_hz
//
// position_16 is invariant in 1/16-full-step units:
//   1/16 STEP = 1 unit
//   1/8  STEP = 2 units
//   1/4  STEP = 4 units
// 3200 units = one revolution for a 200-full-step motor.
// -----------------------------------------------------------------------------
module a4988_axis #(
    parameter integer CLK_HZ = 50_000_000,
    parameter bit     REVERSE_DIR = 1'b0,
    parameter integer STEP_HIGH_US = 2,
    parameter integer DIR_SETUP_US = 5,
    parameter integer MODE_SETUP_US = 5,

    parameter integer MAX_QTR_EQ_HZ = 4000,
    parameter integer MAX_ELECTRICAL_STEP_HZ = 5000,

    parameter integer EIGHTH_FROM_16_ENTER_HZ   = 150,
    parameter integer SIXTEENTH_FROM_8_ENTER_HZ = 80,
    parameter integer QUARTER_FROM_8_ENTER_HZ   = 500,
    parameter integer EIGHTH_FROM_4_ENTER_HZ    = 300
)(
    input  logic               clk,
    input  logic               rst,
    input  logic               enable,
    input  logic               emergency_stop,
    input  logic signed [15:0] speed_hz,

    input  logic [2:0]         requested_microstep_mode,
    input  logic               common_mode_change_enable,

    output logic               step,
    output logic               dir,
    output logic               ms1,
    output logic               ms2,
    output logic               ms3,
    output logic               enn,

    output logic [2:0]         microstep_mode,
    output logic               mode_change_ready,

    output logic [15:0]        step_rate_hz,
    output logic signed [31:0] position_16
);

    localparam integer US_DIV = CLK_HZ / 1_000_000;
    localparam integer US_CNT_W = (US_DIV <= 2) ? 1 : $clog2(US_DIV);

    localparam logic [23:0] THRESH_16 = 24'd1_000_000;
    localparam logic [23:0] THRESH_8  = 24'd2_000_000;
    localparam logic [23:0] THRESH_4  = 24'd4_000_000;

    typedef enum logic [1:0] {
        MODE_16 = 2'd0,
        MODE_8  = 2'd1,
        MODE_4  = 2'd2
    } micro_mode_t;

    typedef enum logic [2:0] {
        ST_WAIT,
        ST_STEP_HIGH,
        ST_DIR_SETUP,
        ST_MODE_SETUP,
        ST_MODE_HOLD
    } state_t;

    state_t      state;
    micro_mode_t mode_cur;
    micro_mode_t mode_desired;

    logic [US_CNT_W-1:0] us_count;
    logic us_tick;
    logic [15:0] timing_us;

    logic [15:0] speed_mag;
    logic [15:0] requested_qtr_eq_hz;
    logic [15:0] effective_qtr_eq_hz;
    logic [15:0] mode_qtr_limit_hz;
    logic requested_dir;

    logic [18:0] actual_step_rate_wide;
    logic [17:0] requested_units16_per_s;
    logic [23:0] phase_acc;
    logic [24:0] phase_sum;
    logic [23:0] phase_threshold;
    logic [2:0]  step_units16;
    logic mode_change_allowed;

    always_comb begin
        enn = ~(enable && !emergency_stop);

        if (speed_hz == 16'sh8000)
            speed_mag = 16'd32768;
        else if (speed_hz[15])
            speed_mag = $unsigned(-$signed(speed_hz));
        else
            speed_mag = $unsigned(speed_hz);

        if (speed_mag > MAX_QTR_EQ_HZ)
            requested_qtr_eq_hz = MAX_QTR_EQ_HZ[15:0];
        else
            requested_qtr_eq_hz = speed_mag;

        requested_dir = speed_hz[15] ^ REVERSE_DIR;

        unique case (requested_microstep_mode)
            3'd0: mode_desired = MODE_16;
            3'd1: mode_desired = MODE_8;
            3'd2: mode_desired = MODE_4;
            default: mode_desired = MODE_16;
        endcase

        unique case (mode_cur)
            MODE_16: mode_qtr_limit_hz = (MAX_ELECTRICAL_STEP_HZ / 4);
            MODE_8:  mode_qtr_limit_hz = (MAX_ELECTRICAL_STEP_HZ / 2);
            MODE_4:  mode_qtr_limit_hz = MAX_ELECTRICAL_STEP_HZ;
            default: mode_qtr_limit_hz = (MAX_ELECTRICAL_STEP_HZ / 4);
        endcase

        if (requested_qtr_eq_hz > mode_qtr_limit_hz)
            effective_qtr_eq_hz = mode_qtr_limit_hz;
        else
            effective_qtr_eq_hz = requested_qtr_eq_hz;

        unique case (mode_cur)
            MODE_16: begin
                ms1 = 1'b1; ms2 = 1'b1; ms3 = 1'b1;
                microstep_mode = 3'd0;
                phase_threshold = THRESH_16;
                step_units16 = 3'd1;
                actual_step_rate_wide = {3'b000, effective_qtr_eq_hz} << 2;
            end
            MODE_8: begin
                ms1 = 1'b1; ms2 = 1'b1; ms3 = 1'b0;
                microstep_mode = 3'd1;
                phase_threshold = THRESH_8;
                step_units16 = 3'd2;
                actual_step_rate_wide = {3'b000, effective_qtr_eq_hz} << 1;
            end
            default: begin
                ms1 = 1'b0; ms2 = 1'b1; ms3 = 1'b0;
                microstep_mode = 3'd2;
                phase_threshold = THRESH_4;
                step_units16 = 3'd4;
                actual_step_rate_wide = {3'b000, effective_qtr_eq_hz};
            end
        endcase

        if (actual_step_rate_wide > 19'd65535)
            step_rate_hz = 16'hffff;
        else
            step_rate_hz = actual_step_rate_wide[15:0];

        requested_units16_per_s = {2'b00, effective_qtr_eq_hz} << 2;
        phase_sum = {1'b0, phase_acc} + {{7{1'b0}}, requested_units16_per_s};

        // Adjacent resolution transitions only. Fine->coarse may need one
        // alignment microstep; coarse->fine is already on the finer lattice.
        mode_change_allowed = 1'b0;
        unique case (mode_cur)
            MODE_16: begin
                if (mode_desired == MODE_8)
                    mode_change_allowed = (position_16[0] == 1'b0);
            end
            MODE_8: begin
                if (mode_desired == MODE_16)
                    mode_change_allowed = (position_16[0] == 1'b0);
                else if (mode_desired == MODE_4)
                    mode_change_allowed = (position_16[1:0] == 2'b00);
            end
            MODE_4: begin
                if (mode_desired == MODE_8)
                    mode_change_allowed = (position_16[1:0] == 2'b00);
            end
            default: mode_change_allowed = 1'b0;
        endcase

        // Unlike V65, readiness is persistent once ST_MODE_HOLD is reached.
        mode_change_ready = (mode_cur == mode_desired) ||
                            ((state == ST_MODE_HOLD) && (mode_cur != mode_desired));
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            us_count <= '0;
            us_tick  <= 1'b0;
        end else begin
            us_tick <= 1'b0;
            if (us_count == US_DIV-1) begin
                us_count <= '0;
                us_tick  <= 1'b1;
            end else begin
                us_count <= us_count + 1'b1;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state       <= ST_WAIT;
            step        <= 1'b0;
            dir         <= REVERSE_DIR;
            mode_cur    <= MODE_16;
            phase_acc   <= 24'd0;
            timing_us   <= 16'd0;
            position_16 <= 32'sd0;
        end else if (!enable || emergency_stop) begin
            state       <= ST_WAIT;
            step        <= 1'b0;
            phase_acc   <= 24'd0;
            timing_us   <= 16'd0;
        end else if (us_tick) begin
            unique case (state)
                ST_WAIT: begin
                    step <= 1'b0;

                    // A pending common mode change has priority over normal DDA
                    // stepping. Once compatible, latch this axis in HOLD.
                    if (mode_desired != mode_cur) begin
                        if (mode_change_allowed) begin
                            state <= ST_MODE_HOLD;

                        end else if ((requested_qtr_eq_hz != 16'd0) &&
                                     (dir != requested_dir)) begin
                            // If an alignment step is needed, first establish
                            // the intended direction with generous setup time.
                            dir         <= requested_dir;
                            phase_acc   <= 24'd0;
                            timing_us   <= DIR_SETUP_US[15:0];
                            state       <= ST_DIR_SETUP;

                        end else begin
                            // Guaranteed progress: for adjacent fine->coarse
                            // transitions one fine-mode STEP is sufficient to
                            // reach a common lattice point. This also resolves
                            // the zero-speed / incompatible-position deadlock.
                            step        <= 1'b1;
                            timing_us   <= STEP_HIGH_US[15:0];
                            state       <= ST_STEP_HIGH;
                            phase_acc   <= 24'd0;

                            if (dir ^ REVERSE_DIR)
                                position_16 <= position_16 - $signed({29'd0, step_units16});
                            else
                                position_16 <= position_16 + $signed({29'd0, step_units16});
                        end

                    end else if ((requested_qtr_eq_hz != 16'd0) &&
                                 (dir != requested_dir)) begin
                        dir         <= requested_dir;
                        phase_acc   <= 24'd0;
                        timing_us   <= DIR_SETUP_US[15:0];
                        state       <= ST_DIR_SETUP;

                    end else if ((requested_qtr_eq_hz != 16'd0) &&
                                 (phase_sum >= {1'b0, phase_threshold})) begin
                        phase_acc <= phase_sum - {1'b0, phase_threshold};
                        step      <= 1'b1;
                        timing_us <= STEP_HIGH_US[15:0];
                        state     <= ST_STEP_HIGH;

                        if (dir ^ REVERSE_DIR)
                            position_16 <= position_16 - $signed({29'd0, step_units16});
                        else
                            position_16 <= position_16 + $signed({29'd0, step_units16});
                    end else begin
                        if (requested_qtr_eq_hz == 16'd0)
                            phase_acc <= 24'd0;
                        else
                            phase_acc <= phase_sum[23:0];
                    end
                end

                ST_STEP_HIGH: begin
                    step <= 1'b1;
                    if (timing_us <= 16'd1) begin
                        step      <= 1'b0;
                        timing_us <= 16'd0;
                        state     <= ST_WAIT;
                    end else begin
                        timing_us <= timing_us - 1'b1;
                    end
                end

                ST_DIR_SETUP: begin
                    step <= 1'b0;
                    if (timing_us <= 16'd1) begin
                        timing_us <= 16'd0;
                        state     <= ST_WAIT;
                    end else begin
                        timing_us <= timing_us - 1'b1;
                    end
                end

                ST_MODE_HOLD: begin
                    // Persistent rendezvous point. No STEP pulses are emitted
                    // until both axes are ready and top asserts the shared commit.
                    step <= 1'b0;
                    if (mode_desired == mode_cur) begin
                        state <= ST_WAIT;
                    end else if (common_mode_change_enable) begin
                        mode_cur  <= mode_desired;
                        timing_us <= MODE_SETUP_US[15:0];
                        state     <= ST_MODE_SETUP;
                    end
                end

                ST_MODE_SETUP: begin
                    step <= 1'b0;
                    if (timing_us <= 16'd1) begin
                        timing_us <= 16'd0;
                        state     <= ST_WAIT;
                    end else begin
                        timing_us <= timing_us - 1'b1;
                    end
                end

                default: begin
                    state     <= ST_WAIT;
                    step      <= 1'b0;
                    phase_acc <= 24'd0;
                    timing_us <= 16'd0;
                    mode_cur  <= MODE_16;
                end
            endcase
        end
    end
endmodule
