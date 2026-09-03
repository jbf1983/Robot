// -----------------------------------------------------------------------------
// MAX10 on-chip ADC wrapper for the Robot_new LiPo input.
// Reuses the exact ADC channel/configuration generated in the original project:
//   Modular ADC #1, command channel 1, 100-MHz soft-IP clock, 2-MHz ADC PLL clock.
// No external digital ADC pins are needed: the MAX10 ADC hard block reads the
// dedicated analog input used by the original board wiring.
// -----------------------------------------------------------------------------
module max10_battery_monitor (
    input  logic        clk50,
    input  logic        rst,
    output logic        battery_ok,
    output logic        battery_valid,
    output logic        battery_fault,
    output logic        adc_timeout,
    output logic [12:0] battery_mv
);
    logic adc_clk_100;
    logic adc_clk_2;
    logic pll_locked;

    logic        command_ready;
    logic        response_valid;
    logic [4:0]  response_channel;
    logic [11:0] response_data;
    logic        response_sop;
    logic        response_eop;

    logic        batt_ok_adc;
    logic        batt_valid_adc;
    logic        adc_timeout_adc;
    logic [11:0] filtered_raw_adc;
    logic [12:0] battery_mv_adc;

    // Original Qsys PLL: 50 MHz input -> c1=100 MHz, c2=2 MHz.
    kalman_pll u_batt_pll (
        .address            (2'b00),
        .areset             (rst),
        .c0                 (),
        .c1                 (adc_clk_100),
        .c2                 (adc_clk_2),
        .c3                 (),
        .c4                 (),
        .clk                (clk50),
        .configupdate       (1'b0),
        .locked             (pll_locked),
        .phasecounterselect (3'b000),
        .phasedone          (),
        .phasestep          (1'b0),
        .phaseupdown        (1'b0),
        .read               (1'b0),
        .readdata           (),
        .reset              (rst),
        .scanclk            (1'b0),
        .scanclkena         (1'b0),
        .scandata           (1'b0),
        .scandataout        (),
        .scandone           (),
        .write              (1'b0),
        .writedata          (32'd0)
    );

    // Same continuous command as the original top.vhd: channel "00001".
    kalman_adc_batt u_batt_adc (
        .clock_clk                  (adc_clk_100),
        .reset_sink_reset_n         (~rst),
        .adc_pll_clock_clk          (adc_clk_2),
        .adc_pll_locked_export      (pll_locked),
        .command_valid              (1'b1),
        .command_channel            (5'b00001),
        .command_startofpacket      (1'b1),
        .command_endofpacket        (1'b1),
        .command_ready              (command_ready),
        .response_valid             (response_valid),
        .response_channel           (response_channel),
        .response_data              (response_data),
        .response_startofpacket     (response_sop),
        .response_endofpacket       (response_eop)
    );

    battery_monitor #(
        .CLK_HZ          (100_000_000),
        .CUTOFF_MV       (3400),
        .RECOVER_MV      (3550),
        .LOW_HOLD_MS     (300),
        .RECOVER_HOLD_MS (500),
        .WATCHDOG_MS     (250),
        .FILTER_SHIFT    (10)
    ) u_batt_filter (
        .clk           (adc_clk_100),
        .rst           (rst || !pll_locked),
        .sample_valid  (response_valid && (response_channel == 5'b00001)),
        .sample_raw    (response_data),
        .battery_ok    (batt_ok_adc),
        .battery_valid (batt_valid_adc),
        .adc_timeout   (adc_timeout_adc),
        .filtered_raw  (filtered_raw_adc),
        .battery_mv    (battery_mv_adc)
    );

    // Synchronize single-bit status back to the 50-MHz control domain.
    logic [1:0] ok_sync;
    logic [1:0] valid_sync;
    logic [1:0] timeout_sync;

    always_ff @(posedge clk50) begin
        if (rst) begin
            ok_sync      <= 2'b00;
            valid_sync   <= 2'b00;
            timeout_sync <= 2'b00;
            battery_mv   <= 13'd0;
        end else begin
            ok_sync      <= {ok_sync[0], batt_ok_adc};
            valid_sync   <= {valid_sync[0], batt_valid_adc};
            timeout_sync <= {timeout_sync[0], adc_timeout_adc};
            // Diagnostic value only. It changes slowly after filtering; sampling
            // it directly here is acceptable because it is not used for control.
            battery_mv   <= battery_mv_adc;
        end
    end

    always_comb begin
        battery_ok    = ok_sync[1];
        battery_valid = valid_sync[1];
        adc_timeout   = timeout_sync[1];
        battery_fault = !ok_sync[1];
    end
endmodule
