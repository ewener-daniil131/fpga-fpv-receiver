// =============================================================================
// cordic_atan2.v
// Pipelined CORDIC vectoring mode calculating angle Z = atan2(Y, X)
// 10-stage pipeline, unrolled for universal Verilog-2001 synthesizer compatibility
// =============================================================================

`timescale 1ns / 1ps

module cordic_atan2 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire signed [15:0] x_in,
    input  wire signed [15:0] y_in,

    output reg         valid_out,
    output reg  signed [15:0] angle_out
);

    localparam signed [15:0] PI_POS = 16'sd32767;
    localparam signed [15:0] PI_NEG = -16'sd32768;

    // Stage 0: Initial Quadrant Mapping
    reg signed [15:0] x0, y0, z0;
    reg               v0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v0 <= 1'b0; x0 <= 16'sd0; y0 <= 16'sd0; z0 <= 16'sd0;
        end else begin
            v0 <= valid_in;
            if (x_in < 16'sd0) begin
                x0 <= -x_in;
                y0 <= -y_in;
                z0 <= (y_in >= 16'sd0) ? PI_POS : PI_NEG;
            end else begin
                x0 <= x_in;
                y0 <= y_in;
                z0 <= 16'sd0;
            end
        end
    end

    // Stage 1: i=0, atan(1) = 8192
    reg signed [15:0] x1, y1, z1; reg v1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v1 <= 0; x1 <= 0; y1 <= 0; z1 <= 0; end
        else begin
            v1 <= v0;
            if (y0 < 0) begin
                x1 <= x0 - y0; y1 <= y0 + x0; z1 <= z0 - 16'sd8192;
            end else begin
                x1 <= x0 + y0; y1 <= y0 - x0; z1 <= z0 + 16'sd8192;
            end
        end
    end

    // Stage 2: i=1, atan(0.5) = 4836
    reg signed [15:0] x2, y2, z2; reg v2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v2 <= 0; x2 <= 0; y2 <= 0; z2 <= 0; end
        else begin
            v2 <= v1;
            if (y1 < 0) begin
                x2 <= x1 - (y1 >>> 1); y2 <= y1 + (x1 >>> 1); z2 <= z1 - 16'sd4836;
            end else begin
                x2 <= x1 + (y1 >>> 1); y2 <= y1 - (x1 >>> 1); z2 <= z1 + 16'sd4836;
            end
        end
    end

    // Stage 3: i=2, atan(0.25) = 2555
    reg signed [15:0] x3, y3, z3; reg v3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v3 <= 0; x3 <= 0; y3 <= 0; z3 <= 0; end
        else begin
            v3 <= v2;
            if (y2 < 0) begin
                x3 <= x2 - (y2 >>> 2); y3 <= y2 + (x2 >>> 2); z3 <= z2 - 16'sd2555;
            end else begin
                x3 <= x2 + (y2 >>> 2); y3 <= y2 - (x2 >>> 2); z3 <= z2 + 16'sd2555;
            end
        end
    end

    // Stage 4: i=3, atan(1/8) = 1297
    reg signed [15:0] x4, y4, z4; reg v4;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v4 <= 0; x4 <= 0; y4 <= 0; z4 <= 0; end
        else begin
            v4 <= v3;
            if (y3 < 0) begin
                x4 <= x3 - (y3 >>> 3); y4 <= y3 + (x3 >>> 3); z4 <= z3 - 16'sd1297;
            end else begin
                x4 <= x3 + (y3 >>> 3); y4 <= y3 - (x3 >>> 3); z4 <= z3 + 16'sd1297;
            end
        end
    end

    // Stage 5: i=4, atan(1/16) = 651
    reg signed [15:0] x5, y5, z5; reg v5;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v5 <= 0; x5 <= 0; y5 <= 0; z5 <= 0; end
        else begin
            v5 <= v4;
            if (y4 < 0) begin
                x5 <= x4 - (y4 >>> 4); y5 <= y4 + (x4 >>> 4); z5 <= z4 - 16'sd651;
            end else begin
                x5 <= x4 + (y4 >>> 4); y5 <= y4 - (x4 >>> 4); z5 <= z4 + 16'sd651;
            end
        end
    end

    // Stage 6: i=5, atan(1/32) = 326
    reg signed [15:0] x6, y6, z6; reg v6;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v6 <= 0; x6 <= 0; y6 <= 0; z6 <= 0; end
        else begin
            v6 <= v5;
            if (y5 < 0) begin
                x6 <= x5 - (y5 >>> 5); y6 <= y5 + (x5 >>> 5); z6 <= z5 - 16'sd326;
            end else begin
                x6 <= x5 + (y5 >>> 5); y6 <= y5 - (x5 >>> 5); z6 <= z5 + 16'sd326;
            end
        end
    end

    // Stage 7: i=6, atan(1/64) = 163
    reg signed [15:0] x7, y7, z7; reg v7;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v7 <= 0; x7 <= 0; y7 <= 0; z7 <= 0; end
        else begin
            v7 <= v6;
            if (y6 < 0) begin
                x7 <= x6 - (y6 >>> 6); y7 <= y6 + (x6 >>> 6); z7 <= z6 - 16'sd163;
            end else begin
                x7 <= x6 + (y6 >>> 6); y7 <= y6 - (x6 >>> 6); z7 <= z6 + 16'sd163;
            end
        end
    end

    // Stage 8: i=7, atan(1/128) = 81
    reg signed [15:0] x8, y8, z8; reg v8;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v8 <= 0; x8 <= 0; y8 <= 0; z8 <= 0; end
        else begin
            v8 <= v7;
            if (y7 < 0) begin
                x8 <= x7 - (y7 >>> 7); y8 <= y7 + (x7 >>> 7); z8 <= z7 - 16'sd81;
            end else begin
                x8 <= x7 + (y7 >>> 7); y8 <= y7 - (x7 >>> 7); z8 <= z7 + 16'sd81;
            end
        end
    end

    // Stage 9: i=8, atan(1/256) = 41
    reg signed [15:0] x9, y9, z9; reg v9;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v9 <= 0; x9 <= 0; y9 <= 0; z9 <= 0; end
        else begin
            v9 <= v8;
            if (y8 < 0) begin
                x9 <= x8 - (y8 >>> 8); y9 <= y8 + (x8 >>> 8); z9 <= z8 - 16'sd41;
            end else begin
                x9 <= x8 + (y8 >>> 8); y9 <= y8 - (x8 >>> 8); z9 <= z8 + 16'sd41;
            end
        end
    end

    // Stage 10: i=9, atan(1/512) = 20
    reg signed [15:0] x10, y10, z10; reg v10;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin v10 <= 0; x10 <= 0; y10 <= 0; z10 <= 0; end
        else begin
            v10 <= v9;
            if (y9 < 0) begin
                x10 <= x9 - (y9 >>> 9); y10 <= y9 + (x9 >>> 9); z10 <= z9 - 16'sd20;
            end else begin
                x10 <= x9 + (y9 >>> 9); y10 <= y9 - (x9 >>> 9); z10 <= z9 + 16'sd20;
            end
        end
    end

    // Output registration
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            angle_out <= 16'sd0;
        end else begin
            valid_out <= v10;
            angle_out <= z10;
        end
    end

endmodule
