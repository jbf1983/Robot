// -----------------------------------------------------------------------------
// Iterative atan2 CORDIC.
// Input:  signed Cartesian y,x (raw sensor units are fine).
// Output: signed angle in Q16.16 degrees, range approximately [-180, +180].
// Latency: 16 FPGA clocks after start.
// -----------------------------------------------------------------------------
module cordic_atan2 (
    input  logic               clk,
    input  logic               rst,
    input  logic               start,
    input  logic signed [15:0] y_in,
    input  logic signed [15:0] x_in,
    output logic               busy,
    output logic               done,
    output logic signed [31:0] angle_q16
);

    localparam logic signed [31:0] DEG180_Q16 = 32'sd11796480;

    function automatic logic signed [31:0] atan_lut(input logic [3:0] i);
        case (i)
            4'd0:  atan_lut = 32'sd2949120; // 45.000000 deg
            4'd1:  atan_lut = 32'sd1740967; // 26.565051
            4'd2:  atan_lut = 32'sd919879;  // 14.036243
            4'd3:  atan_lut = 32'sd466945;  // 7.125016
            4'd4:  atan_lut = 32'sd234379;  // 3.576334
            4'd5:  atan_lut = 32'sd117304;  // 1.789911
            4'd6:  atan_lut = 32'sd58666;   // 0.895174
            4'd7:  atan_lut = 32'sd29335;   // 0.447614
            4'd8:  atan_lut = 32'sd14668;   // 0.223811
            4'd9:  atan_lut = 32'sd7334;    // 0.111906
            4'd10: atan_lut = 32'sd3667;    // 0.055953
            4'd11: atan_lut = 32'sd1833;    // 0.027976
            4'd12: atan_lut = 32'sd917;     // 0.013988
            4'd13: atan_lut = 32'sd458;     // 0.006994
            4'd14: atan_lut = 32'sd229;     // 0.003497
            default: atan_lut = 32'sd115;    // 0.001749
        endcase
    endfunction

    logic signed [31:0] x_reg, y_reg, z_reg;
    logic signed [31:0] x_next, y_next, z_next;
    logic [3:0]         iter;

    always_comb begin
        x_next = x_reg;
        y_next = y_reg;
        z_next = z_reg;

        if (y_reg > 0) begin
            x_next = x_reg + (y_reg >>> iter);
            y_next = y_reg - (x_reg >>> iter);
            z_next = z_reg + atan_lut(iter);
        end else if (y_reg < 0) begin
            x_next = x_reg - (y_reg >>> iter);
            y_next = y_reg + (x_reg >>> iter);
            z_next = z_reg - atan_lut(iter);
        end
    end

    always_ff @(posedge clk) begin
        done <= 1'b0;

        if (rst) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            iter      <= 4'd0;
            x_reg     <= '0;
            y_reg     <= '0;
            z_reg     <= '0;
            angle_q16 <= '0;
        end else begin
            if (!busy) begin
                if (start) begin
                    busy <= 1'b1;
                    iter <= 4'd0;

                    if ((x_in == 0) && (y_in == 0)) begin
                        x_reg <= 32'sd0;
                        y_reg <= 32'sd0;
                        z_reg <= 32'sd0;
                    end else if (x_in < 0) begin
                        // Pre-rotate 180 degrees so iterative vectoring starts
                        // with x >= 0 and still returns a true atan2 result.
                        x_reg <= -$signed({{16{x_in[15]}}, x_in});
                        y_reg <= -$signed({{16{y_in[15]}}, y_in});
                        z_reg <= (y_in >= 0) ? DEG180_Q16 : -DEG180_Q16;
                    end else begin
                        x_reg <= $signed({{16{x_in[15]}}, x_in});
                        y_reg <= $signed({{16{y_in[15]}}, y_in});
                        z_reg <= 32'sd0;
                    end
                end
            end else begin
                x_reg <= x_next;
                y_reg <= y_next;
                z_reg <= z_next;

                if (iter == 4'd15) begin
                    angle_q16 <= z_next;
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    iter <= iter + 1'b1;
                end
            end
        end
    end

endmodule
