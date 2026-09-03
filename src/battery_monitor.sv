// -----------------------------------------------------------------------------
// LiPo undervoltage monitor.
//
// ADC scaling inherited from the original Robot_new VHDL project:
//   battery_mV = ADC * 2500 / 4095 * 2 ~= ADC * 5000 / 4095
// The external divider therefore maps about 0..5.0 V battery to the 0..2.5 V ADC.
//
// Protection used in this version:
//   - low-voltage threshold: 3.40 V
//   - low voltage must persist for 300 ms before battery_ok is cleared
//   - battery must recover above 3.55 V for 500 ms before battery_ok is set
//   - ADC samples are low-pass filtered (1/1024 IIR) before comparison
//   - if ADC samples disappear for 250 ms, fail safe: battery_ok = 0
// -----------------------------------------------------------------------------
module battery_monitor #(
    parameter integer CLK_HZ          = 100_000_000,
    parameter integer CUTOFF_MV       = 3400,
    parameter integer RECOVER_MV      = 3550,
    parameter integer LOW_HOLD_MS     = 300,
    parameter integer RECOVER_HOLD_MS = 500,
    parameter integer WATCHDOG_MS     = 250,
    parameter integer FILTER_SHIFT    = 10
)(
    input  logic        clk,
    input  logic        rst,
    input  logic        sample_valid,
    input  logic [11:0] sample_raw,

    output logic        battery_ok,
    output logic        battery_valid,
    output logic        adc_timeout,
    output logic [11:0] filtered_raw,
    output logic [12:0] battery_mv
);
    localparam integer CUTOFF_RAW_I  = (CUTOFF_MV  * 4095 + 4999) / 5000;
    localparam integer RECOVER_RAW_I = (RECOVER_MV * 4095 + 4999) / 5000;

    localparam integer LOW_HOLD_CYCLES = (CLK_HZ / 1000) * LOW_HOLD_MS;
    localparam integer REC_HOLD_CYCLES = (CLK_HZ / 1000) * RECOVER_HOLD_MS;
    localparam integer WD_CYCLES       = (CLK_HZ / 1000) * WATCHDOG_MS;

    localparam integer LOW_W = (LOW_HOLD_CYCLES <= 2) ? 1 : $clog2(LOW_HOLD_CYCLES + 1);
    localparam integer REC_W = (REC_HOLD_CYCLES <= 2) ? 1 : $clog2(REC_HOLD_CYCLES + 1);
    localparam integer WD_W  = (WD_CYCLES       <= 2) ? 1 : $clog2(WD_CYCLES + 1);

    // Q(FILTER_SHIFT) accumulator. 12-bit sample + fractional bits + sign margin.
    logic signed [23:0] filter_q;
    logic               filter_initialized;
    logic signed [24:0] sample_q;
    logic signed [24:0] filter_error;
    logic signed [24:0] filter_next_wide;

    logic [LOW_W-1:0] low_count;
    logic [REC_W-1:0] recover_count;
    logic [WD_W-1:0]  watchdog_count;

    logic below_cutoff;
    logic above_recover;
    logic [24:0] mv_product;

    always_comb begin
        sample_q         = $signed({1'b0, sample_raw, {FILTER_SHIFT{1'b0}}});
        filter_error     = sample_q - $signed({filter_q[23], filter_q});
        filter_next_wide = $signed({filter_q[23], filter_q}) + (filter_error >>> FILTER_SHIFT);

        below_cutoff = (filtered_raw < CUTOFF_RAW_I[11:0]);
        above_recover = (filtered_raw >= RECOVER_RAW_I[11:0]);

        mv_product = filtered_raw * 13'd5000;
        battery_mv = (mv_product + 25'd2047) / 12'd4095;
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            filter_q           <= 24'sd0;
            filter_initialized <= 1'b0;
            filtered_raw       <= 12'd0;
            battery_ok         <= 1'b0;
            battery_valid      <= 1'b0;
            adc_timeout        <= 1'b0;
            low_count          <= '0;
            recover_count      <= '0;
            watchdog_count     <= '0;
        end else begin
            // ADC watchdog. A missing ADC stream always disables the robot.
            if (sample_valid) begin
                watchdog_count <= '0;
                adc_timeout    <= 1'b0;
                battery_valid  <= 1'b1;
            end else if (watchdog_count < WD_CYCLES[WD_W-1:0]) begin
                watchdog_count <= watchdog_count + 1'b1;
            end else begin
                adc_timeout   <= 1'b1;
                battery_valid <= 1'b0;
                battery_ok    <= 1'b0;
                low_count     <= '0;
                recover_count <= '0;
            end

            // Filter only real ADC responses. Initialize from first sample to
            // avoid a false low-voltage ramp from zero after configuration.
            if (sample_valid) begin
                if (!filter_initialized) begin
                    filter_q           <= $signed({1'b0, sample_raw, {FILTER_SHIFT{1'b0}}});
                    filtered_raw       <= sample_raw;
                    filter_initialized <= 1'b1;
                end else begin
                    filter_q     <= filter_next_wide[23:0];
                    filtered_raw <= filter_next_wide[FILTER_SHIFT+11:FILTER_SHIFT];
                end
            end

            if (!filter_initialized || adc_timeout) begin
                battery_ok    <= 1'b0;
                low_count     <= '0;
                recover_count <= '0;
            end else if (battery_ok) begin
                recover_count <= '0;
                if (below_cutoff) begin
                    if (low_count >= LOW_HOLD_CYCLES-1) begin
                        battery_ok <= 1'b0;
                        low_count  <= '0;
                    end else begin
                        low_count <= low_count + 1'b1;
                    end
                end else begin
                    low_count <= '0;
                end
            end else begin
                low_count <= '0;
                if (above_recover) begin
                    if (recover_count >= REC_HOLD_CYCLES-1) begin
                        battery_ok    <= 1'b1;
                        recover_count <= '0;
                    end else begin
                        recover_count <= recover_count + 1'b1;
                    end
                end else begin
                    recover_count <= '0;
                end
            end
        end
    end
endmodule
