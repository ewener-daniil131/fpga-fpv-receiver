// =============================================================================
// video_diag.v
// Instrumentation for the demodulated CVBS stream. Answers three questions
// that cannot be told apart from the ntsc_locked flag alone:
//   1. Is the video pinned to both rails? -> FM deviation exceeds Nyquist and
//      the CORDIC phase step wraps (over-deviation / aliasing).
//   2. Are there low-going pulses at all, and how wide are they? -> tells a
//      missing sync tip from an inverted one.
//   3. What is the actual interval between sync-width pulses? -> the true line
//      period, independent of the accept window in the sync separator.
// Statistics are accumulated over a window of 2^WIN_LOG2 video samples and
// latched, so the UART can read a coherent snapshot.
// =============================================================================

`timescale 1ns / 1ps

module video_diag #(
    parameter WIN_LOG2 = 17,         // 131072 samples ~ 6.5 ms @ 20 Msps (~103 lines)
    // pulse-width classes, in input samples (defaults for 20 Msps)
    parameter [15:0] HS_MIN = 16'd60,    // 4.7 us H-sync tip is 94 samples
    parameter [15:0] HS_MAX = 16'd140,
    parameter [15:0] BR_MIN = 16'd300    // 27 us broad serration is 540 samples
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  video_in,
    input  wire        video_valid,

    output reg  [7:0]  vmin,
    output reg  [7:0]  vmax,
    output reg  [63:0] hist_flat,    // 8 bins, each scaled to parts-per-256
    output reg  [15:0] n_tiny,       // low pulses narrower than HS_MIN
    output reg  [15:0] n_hsync,      // low pulses HS_MIN..HS_MAX (H-sync tip)
    output reg  [15:0] n_mid,        // low pulses between HS_MAX and BR_MIN
    output reg  [15:0] n_broad,      // low pulses >=BR_MIN (vertical serration)
    output reg  [15:0] per_last,     // last measured hsync-to-hsync interval
    output reg  [15:0] per_min,
    output reg  [15:0] per_max
);

    reg [WIN_LOG2:0] cnt;
    reg [7:0]  cur_min, cur_max;
    reg [17:0] bin0, bin1, bin2, bin3, bin4, bin5, bin6, bin7;
    reg [15:0] c_tiny, c_hsync, c_mid, c_broad;
    reg [15:0] low_run, since_h;
    reg [15:0] p_min, p_max, p_last;
    reg [7:0]  thresh;

    wire [2:0] binsel = video_in[7:5];
    wire       is_low = (video_in < thresh);

    // 18-bit count -> parts-per-256 of the window
    function [7:0] scale(input [17:0] c);
        scale = c[17] ? 8'hFF : c[16:9];
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 0; cur_min <= 8'hFF; cur_max <= 8'h00;
            bin0<=0; bin1<=0; bin2<=0; bin3<=0; bin4<=0; bin5<=0; bin6<=0; bin7<=0;
            c_tiny<=0; c_hsync<=0; c_mid<=0; c_broad<=0;
            low_run<=0; since_h<=0;
            p_min<=16'hFFFF; p_max<=16'd0; p_last<=16'd0;
            thresh<=8'd64;
            vmin<=8'd0; vmax<=8'd0; hist_flat<=64'd0;
            n_tiny<=0; n_hsync<=0; n_mid<=0; n_broad<=0;
            per_last<=0; per_min<=0; per_max<=0;
        end else if (video_valid) begin
            cnt <= cnt + 1'b1;

            if (video_in < cur_min) cur_min <= video_in;
            if (video_in > cur_max) cur_max <= video_in;

            case (binsel)
                3'd0: bin0 <= bin0 + 18'd1;  3'd1: bin1 <= bin1 + 18'd1;
                3'd2: bin2 <= bin2 + 18'd1;  3'd3: bin3 <= bin3 + 18'd1;
                3'd4: bin4 <= bin4 + 18'd1;  3'd5: bin5 <= bin5 + 18'd1;
                3'd6: bin6 <= bin6 + 18'd1;  default: bin7 <= bin7 + 18'd1;
            endcase

            if (since_h != 16'hFFFF) since_h <= since_h + 16'd1;

            if (is_low) begin
                if (low_run != 16'hFFFF) low_run <= low_run + 16'd1;
            end else begin
                if (low_run != 16'd0) begin
                    if (low_run < HS_MIN) c_tiny <= c_tiny + 16'd1;
                    else if (low_run <= HS_MAX) begin
                        c_hsync <= c_hsync + 16'd1;
                        p_last  <= since_h;
                        if (since_h < p_min) p_min <= since_h;
                        if (since_h > p_max) p_max <= since_h;
                        since_h <= 16'd0;
                    end
                    else if (low_run < BR_MIN)  c_mid   <= c_mid   + 16'd1;
                    else                        c_broad <= c_broad + 16'd1;
                end
                low_run <= 16'd0;
            end

            // ---- window rollover: latch the snapshot, restart accumulation ----
            // placed last so its assignments override the per-sample ones above
            if (cnt[WIN_LOG2]) begin
                vmin      <= cur_min;
                vmax      <= cur_max;
                hist_flat <= {scale(bin7), scale(bin6), scale(bin5), scale(bin4),
                              scale(bin3), scale(bin2), scale(bin1), scale(bin0)};
                n_tiny    <= c_tiny;
                n_hsync   <= c_hsync;
                n_mid     <= c_mid;
                n_broad   <= c_broad;
                per_last  <= p_last;
                per_min   <= (p_min == 16'hFFFF) ? 16'd0 : p_min;
                per_max   <= p_max;

                // slice at the bottom quarter of the observed range: that is
                // where a real sync tip sits
                thresh    <= (cur_max > cur_min)
                             ? (cur_min + ((cur_max - cur_min) >> 2))
                             : 8'd64;

                cnt <= 0;
                cur_min <= 8'hFF; cur_max <= 8'h00;
                bin0<=0; bin1<=0; bin2<=0; bin3<=0; bin4<=0; bin5<=0; bin6<=0; bin7<=0;
                c_tiny<=0; c_hsync<=0; c_mid<=0; c_broad<=0;
                low_run<=0; since_h<=0;
                p_min<=16'hFFFF; p_max<=16'd0;
            end
        end
    end

endmodule
