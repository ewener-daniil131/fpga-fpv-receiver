// =============================================================================
// fm_demod.v
// FM Polar Discriminator Demodulator for NTSC Analog FPV Video
//
// Converts 8 Msps complex IQ samples into raw composite video (CVBS) stream
// Algorithm:
//   1. Complex conjugate product: z[n] * conj(z[n-1]) -> (X, Y)
//   2. Phase difference: d_theta = atan2(Y, X) via pipelined CORDIC
//   3. Single-pole IIR de-emphasis filter (tau = 50 us NTSC standard)
//   4. Continuous RSSI / carrier power estimation
// =============================================================================

`timescale 1ns / 1ps

module fm_demod (
    input  wire        clk,
    input  wire        rst_n,

    // Stream input from usb_iq_fifo
    input  wire [15:0] sample_in,        // {I[7:0], Q[7:0]} signed 8-bit
    input  wire        sample_valid,

    // Demodulated video output
    output wire [7:0]  video_out,        // 8-bit composite video baseband (0..255)
    output wire        video_valid,      // 1-cycle strobe per video sample
    output wire signed [15:0] raw_dtheta,// Raw instantaneous frequency deviation
    output reg  [15:0] rssi              // Smoothed carrier power indicator
);

    wire signed [7:0] cur_i = sample_in[15:8];
    wire signed [7:0] cur_q = sample_in[7:0];

    reg  signed [7:0] prev_i;
    reg  signed [7:0] prev_q;
    reg               prev_valid;

    // -------------------------------------------------------------------------
    // 1. Complex Conjugate Product: z[n] * z*[n-1]
    // X = I[n]*I[n-1] + Q[n]*Q[n-1]
    // Y = Q[n]*I[n-1] - I[n]*Q[n-1]
    // -------------------------------------------------------------------------
    reg signed [15:0] prod_x;
    reg signed [15:0] prod_y;
    reg               prod_valid;

    // Magnitude estimation for RSSI
    wire [7:0] abs_i = (cur_i < 8'sd0) ? -cur_i : cur_i;
    wire [7:0] abs_q = (cur_q < 8'sd0) ? -cur_q : cur_q;
    wire [8:0] inst_mag = {1'b0, abs_i} + {1'b0, abs_q};

    reg [19:0] mag_acc;
    reg [10:0] mag_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_i     <= 8'sd0;
            prev_q     <= 8'sd0;
            prev_valid <= 1'b0;
            prod_x     <= 16'sd0;
            prod_y     <= 16'sd0;
            prod_valid <= 1'b0;
            mag_acc    <= 20'd0;
            mag_cnt    <= 11'd0;
            rssi       <= 16'd0;
        end else begin
            prod_valid <= 1'b0;
            if (sample_valid) begin
                prev_i <= cur_i;
                prev_q <= cur_q;

                // Accumulate RSSI over 1024 samples
                mag_acc <= mag_acc + {11'd0, inst_mag};
                if (mag_cnt == 11'd1023) begin
                    mag_cnt <= 11'd0;
                    rssi    <= mag_acc[19:4]; // Scale to 16-bit
                    mag_acc <= 20'd0;
                end else begin
                    mag_cnt <= mag_cnt + 11'd1;
                end

                if (prev_valid) begin
                    // 16-bit dot and cross products
                    prod_x     <= (cur_i * prev_i) + (cur_q * prev_q);
                    prod_y     <= (cur_q * prev_i) - (cur_i * prev_q);
                    prod_valid <= 1'b1;
                end
                prev_valid <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 2. Pipelined CORDIC atan2(Y, X) -> d_theta
    // -------------------------------------------------------------------------
    wire signed [15:0] cordic_angle;
    wire               cordic_valid;

    cordic_atan2 u_cordic (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (prod_valid),
        .x_in       (prod_x),
        .y_in       (prod_y),
        .valid_out  (cordic_valid),
        .angle_out  (cordic_angle)
    );

    assign raw_dtheta = cordic_angle;

    // -------------------------------------------------------------------------
    // 3. Post-demod filter: 3-tap moving average
    //
    // Analog FPV carries an AUDIO SUBCARRIER at 5.5, 6.0 or 6.5 MHz from the
    // video carrier. It demodulates along with the picture and lands on the
    // image as interference. A single-pole IIR is far too shallow to remove it
    // (only ~12 dB at 6 MHz), which is why narrowing that filter reduced detail
    // without reducing noise - the interference is concentrated, not broadband.
    //
    // A 3-tap moving average at 20 Msps puts its first null at 6.67 MHz:
    //   video  4.0 MHz  -5.4 dB      subcarrier 6.0 MHz  -17.9 dB
    //   video  4.2 MHz  -6.0 dB      subcarrier 6.5 MHz  -30.3 dB
    // -------------------------------------------------------------------------
    reg signed [17:0] ma_sum;
    reg signed [15:0] tap1, tap2;
    reg signed [15:0] deemph_out;
    reg               deemph_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tap1 <= 16'sd0; tap2 <= 16'sd0;
            ma_sum <= 18'sd0; deemph_out <= 16'sd0; deemph_valid <= 1'b0;
        end else begin
            deemph_valid <= 1'b0;
            if (cordic_valid) begin
                tap1 <= cordic_angle;
                tap2 <= tap1;
                ma_sum <= $signed({{2{cordic_angle[15]}}, cordic_angle})
                        + $signed({{2{tap1[15]}}, tap1})
                        + $signed({{2{tap2[15]}}, tap2});
                deemph_out   <= ma_sum[17:2];   // sum of 3, kept at ~1.5x gain
                deemph_valid <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. Output Scaling to 8-bit Unsigned Video (0..255)
    // -------------------------------------------------------------------------
    // The CORDIC uses pi = 32768. Measured FPV deviation is about -1.6..+2.3 MHz,
    // which at 20 Msps is roughly -5100..+7500 angle units. deemph_out carries
    // the 3-tap sum scaled by 1/4, i.e. about 0.75x, so >>> 6 keeps roughly the
    // previous full-scale mapping with a little extra contrast.
    wire signed [15:0] scaled_sample = (deemph_out >>> 6) + 16'sd128;
    wire [7:0] clamped_video = (scaled_sample < 16'sd0)   ? 8'd0 :
                               (scaled_sample > 16'sd255) ? 8'd255 :
                               scaled_sample[7:0];

    assign video_out   = clamped_video;
    assign video_valid = deemph_valid;

endmodule
