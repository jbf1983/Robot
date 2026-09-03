// -----------------------------------------------------------------------------
// Simple synthesizable UART receiver, 8 data bits, no parity, 1 stop bit.
// Designed for the HM-10 link used by the original Robot_new project.
// -----------------------------------------------------------------------------
module uart_rx_8n1 #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       rx,
    output logic [7:0] data,
    output logic       valid,
    output logic       framing_error
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD/2)) / BAUD;
    localparam integer CNT_W = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT + 1);

    typedef enum logic [2:0] {RX_IDLE, RX_START, RX_DATA, RX_STOP} rx_state_t;
    rx_state_t state;

    logic rx_meta, rx_sync;
    logic [CNT_W-1:0] clk_count;
    logic [2:0] bit_index;
    logic [7:0] shift_reg;

    always_ff @(posedge clk) begin
        if (rst) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            state         <= RX_IDLE;
            clk_count     <= '0;
            bit_index     <= 3'd0;
            shift_reg     <= 8'd0;
            data          <= 8'd0;
            valid         <= 1'b0;
            framing_error <= 1'b0;
        end else begin
            valid         <= 1'b0;
            framing_error <= 1'b0;

            case (state)
                RX_IDLE: begin
                    clk_count <= '0;
                    bit_index <= 3'd0;
                    if (!rx_sync) begin
                        state     <= RX_START;
                        clk_count <= '0;
                    end
                end

                RX_START: begin
                    // Re-sample in the middle of the start bit.
                    if (clk_count >= (CLKS_PER_BIT/2)-1) begin
                        clk_count <= '0;
                        if (!rx_sync) begin
                            state <= RX_DATA;
                        end else begin
                            state <= RX_IDLE; // false start
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                RX_DATA: begin
                    if (clk_count >= CLKS_PER_BIT-1) begin
                        clk_count            <= '0;
                        shift_reg[bit_index] <= rx_sync;
                        if (bit_index == 3'd7) begin
                            bit_index <= 3'd0;
                            state     <= RX_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                RX_STOP: begin
                    if (clk_count >= CLKS_PER_BIT-1) begin
                        clk_count <= '0;
                        state     <= RX_IDLE;
                        if (rx_sync) begin
                            data  <= shift_reg;
                            valid <= 1'b1;
                        end else begin
                            framing_error <= 1'b1;
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                default: state <= RX_IDLE;
            endcase
        end
    end
endmodule
