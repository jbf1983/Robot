// -----------------------------------------------------------------------------
// Simple synthesizable UART transmitter, 8 data bits, no parity, 1 stop bit.
// -----------------------------------------------------------------------------
module uart_tx_8n1 #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200
)(
    input  logic       clk,
    input  logic       rst,
    input  logic       start,
    input  logic [7:0] data,
    output logic       tx,
    output logic       busy,
    output logic       done
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD/2)) / BAUD;
    localparam integer CNT_W = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT + 1);

    typedef enum logic [2:0] {TX_IDLE, TX_START, TX_DATA, TX_STOP} tx_state_t;
    tx_state_t state;

    logic [CNT_W-1:0] clk_count;
    logic [2:0] bit_index;
    logic [7:0] shift_reg;

    always_ff @(posedge clk) begin
        if (rst) begin
            state     <= TX_IDLE;
            clk_count <= '0;
            bit_index <= 3'd0;
            shift_reg <= 8'd0;
            tx         <= 1'b1;
            busy       <= 1'b0;
            done       <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                TX_IDLE: begin
                    tx         <= 1'b1;
                    busy       <= 1'b0;
                    clk_count  <= '0;
                    bit_index  <= 3'd0;
                    if (start) begin
                        shift_reg <= data;
                        busy      <= 1'b1;
                        tx        <= 1'b0;
                        state     <= TX_START;
                    end
                end

                TX_START: begin
                    tx   <= 1'b0;
                    busy <= 1'b1;
                    if (clk_count >= CLKS_PER_BIT-1) begin
                        clk_count <= '0;
                        tx        <= shift_reg[0];
                        state     <= TX_DATA;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                TX_DATA: begin
                    busy <= 1'b1;
                    tx   <= shift_reg[bit_index];
                    if (clk_count >= CLKS_PER_BIT-1) begin
                        clk_count <= '0;
                        if (bit_index == 3'd7) begin
                            bit_index <= 3'd0;
                            tx        <= 1'b1;
                            state     <= TX_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                        end
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                TX_STOP: begin
                    tx   <= 1'b1;
                    busy <= 1'b1;
                    if (clk_count >= CLKS_PER_BIT-1) begin
                        clk_count <= '0;
                        state     <= TX_IDLE;
                        busy      <= 1'b0;
                        done      <= 1'b1;
                    end else begin
                        clk_count <= clk_count + 1'b1;
                    end
                end

                default: state <= TX_IDLE;
            endcase
        end
    end
endmodule
