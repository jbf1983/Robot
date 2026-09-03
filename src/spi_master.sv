// -----------------------------------------------------------------------------
// 8-bit SPI master, mode 0, MSB first.
// CS is handled by the caller so it can remain asserted across burst transfers.
//
// IMPORTANT for MPU9250 at 1 MHz:
//   tHD.CS (CS hold after final SCLK edge) >= 500 ns.
// This engine therefore keeps busy asserted for CS_HOLD_NS after the final
// falling SCLK edge before pulsing done. The caller cannot deassert CS early.
// -----------------------------------------------------------------------------
module spi_master #(
    parameter int CLK_HZ     = 50_000_000,
    parameter int SPI_HZ     = 1_000_000,
    parameter int CS_HOLD_NS = 500
)(
    input  logic       clk,
    input  logic       rst,

    input  logic       start,
    input  logic [7:0] tx_data,
    output logic [7:0] rx_data,
    output logic       busy,
    output logic       done,

    output logic       sclk,
    output logic       mosi,
    input  logic       miso
);

    localparam int HALF_DIV = CLK_HZ / (2 * SPI_HZ);
    localparam int DIV_W    = (HALF_DIV <= 1) ? 1 : $clog2(HALF_DIV);

    // ceil(CLK_HZ * CS_HOLD_NS / 1e9)
    localparam int HOLD_TICKS_RAW =
        ((CLK_HZ / 1000) * CS_HOLD_NS + 999_999) / 1_000_000;
    localparam int HOLD_TICKS = (HOLD_TICKS_RAW < 1) ? 1 : HOLD_TICKS_RAW;
    localparam int HOLD_W     = (HOLD_TICKS <= 1) ? 1 : $clog2(HOLD_TICKS);

    logic [DIV_W-1:0]  div_cnt;
    logic [HOLD_W-1:0] hold_cnt;
    logic [2:0]        bit_cnt;
    logic [7:0]        tx_shift;
    logic [7:0]        rx_shift;
    logic              hold_phase;

    initial begin
        if (CLK_HZ < 2*SPI_HZ)
            $error("spi_master: CLK_HZ must be >= 2*SPI_HZ");
    end

    always_ff @(posedge clk) begin
        done <= 1'b0;

        if (rst) begin
            div_cnt     <= '0;
            hold_cnt    <= '0;
            bit_cnt     <= '0;
            tx_shift    <= '0;
            rx_shift    <= '0;
            rx_data     <= '0;
            busy        <= 1'b0;
            sclk        <= 1'b0;
            mosi        <= 1'b0;
            hold_phase  <= 1'b0;
        end else begin
            if (!busy) begin
                sclk       <= 1'b0;
                hold_phase <= 1'b0;

                if (start) begin
                    busy       <= 1'b1;
                    div_cnt    <= '0;
                    hold_cnt   <= '0;
                    bit_cnt    <= '0;
                    tx_shift   <= tx_data;
                    rx_shift   <= '0;
                    mosi       <= tx_data[7];
                end
            end else if (hold_phase) begin
                // SCLK stays low. Keep busy high so the caller keeps CS in its
                // current state for at least the MPU9250 CS hold requirement.
                sclk <= 1'b0;

                if (hold_cnt == HOLD_TICKS-1) begin
                    hold_cnt   <= '0;
                    hold_phase <= 1'b0;
                    busy       <= 1'b0;
                    done       <= 1'b1;
                end else begin
                    hold_cnt <= hold_cnt + 1'b1;
                end
            end else begin
                if (div_cnt == HALF_DIV-1) begin
                    div_cnt <= '0;

                    if (!sclk) begin
                        // Rising edge: sample MISO (SPI mode 0).
                        sclk     <= 1'b1;
                        rx_shift <= {rx_shift[6:0], miso};
                    end else begin
                        // Falling edge: prepare next MOSI bit or finish byte.
                        sclk <= 1'b0;

                        if (bit_cnt == 3'd7) begin
                            // rx_shift already contains all 8 sampled bits.
                            rx_data    <= rx_shift;
                            mosi       <= 1'b0;
                            hold_cnt   <= '0;
                            hold_phase <= 1'b1;
                        end else begin
                            bit_cnt  <= bit_cnt + 1'b1;
                            tx_shift <= {tx_shift[6:0], 1'b0};
                            mosi     <= tx_shift[6];
                        end
                    end
                end else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end

endmodule
