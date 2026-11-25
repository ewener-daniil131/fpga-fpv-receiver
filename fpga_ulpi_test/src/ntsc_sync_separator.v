// =============================================================================
// ntsc_sync_separator.v
// NTSC Video Sync Separator & Active Pixel Coordinate Generator
//
// Designed for 8.000 Msps composite video baseband (CVBS):
//   - Line frequency: 15,734.26 Hz -> Period ~63.55 us (~508.4 samples @ 8Msps)
//   - Horizontal Sync Tip: ~4.7 us (~38 samples)
//   - Vertical Sync Serration: ~27 us (~216 samples)
//   - Total lines: 525 lines/frame (2 fields of 262.5 lines)
//   - Active video per line: ~52.6 us (~420 samples)
// =============================================================================

`timescale 1ns / 1ps

module ntsc_sync_separator #(
    // Timing constants in input samples. Defaults are for 20 Msps, the rate
    // needed to fit the full FM deviation of an analog FPV transmitter.
    //   H-sync tip 4.7 us -> 94 samples
    //   Line period: NTSC 63.556 us -> 1271, PAL 64 us -> 1280 (window covers both)
    //   Broad vertical serration 27 us -> 540 samples
    //   Active video starts 9.5 us (190) after sync, lasts 52.6 us (1052)
    parameter [10:0] HSYNC_MIN = 11'd60,
    parameter [10:0] HSYNC_MAX = 11'd140,
    parameter [10:0] BROAD_MIN = 11'd300,
    parameter [10:0] LOW_SAT   = 11'd1500,
    // The acceptance window must be wide enough to still contain a real sync
    // pulse after the flywheel has been running: the flywheel carries a
    // remainder of (LINE_TIMEOUT - LINE_NOMINAL), which shifts the counter
    // phase by that much, so a real pulse arrives that many samples early.
    // With a window of 1250..1295 and a remainder of 29 the pulse landed at
    // 1242 and was rejected forever - the flywheel then ran the whole field
    // and its drift produced false field starts every ~50 lines.
    parameter [11:0] LINE_MIN  = 12'd1220,
    parameter [11:0] LINE_MAX  = 12'd1310,
    parameter [11:0] ACT_START = 12'd190,
    parameter [11:0] ACT_END   = 12'd1240,
    parameter        PIX_SHIFT = 1,         // horizontal decimation: 1052 -> 526
    // Vertical: each field is rendered as its own frame (240 active lines),
    // which is what low-latency FPV wants - no interlace weave, no field delay.
    parameter [9:0]  VACT_START = 10'd20,
    parameter [9:0]  VACT_LINES = 10'd240,
    // A field's vertical interval contains 6 broad serrations; only the first
    // may raise vsync, so this blanks the detector for the rest of them.
    parameter [9:0]  VS_GUARD   = 10'd200,
    // Drop lock if no H-sync arrives for this many input samples. ntsc_locked
    // is otherwise only updated when a sync pulse is found, so on signal loss
    // it would hold its last value forever. 65535 samples @20 Msps ~ 3.3 ms,
    // about 52 line periods.
    parameter [15:0] LOSS_SAMPLES = 16'd65535,
    // A line period plus margin: if no sync arrives within this, the line is
    // counted as missed rather than waited for (the counter is only 12 bits and
    // could never span the ~18-line vertical interval on its own).
    parameter [11:0] LINE_TIMEOUT = 12'd1320,
    // Nominal line period. The flywheel must free-run at exactly this, or the
    // picture walks sideways by the difference every line.
    parameter [11:0] LINE_NOMINAL = 12'd1271,
    // Consecutive missed lines that mark the vertical interval. The real
    // interval is ~18 lines, while noise loses one line at a time, so 4 is
    // clear of the noise and well inside the interval.
    parameter [3:0]  VS_MISSES = 4'd4
)(
    input  wire        clk,
    input  wire        rst_n,

    // Baseband video stream from fm_demod
    input  wire [7:0]  video_in,
    input  wire        video_valid,

    // Detected sync pulses & lock status
    output reg         hsync,            // Strobe on start of new line
    output reg         line_advance,     // hsync, or a line given up as lost
    output reg         vsync,            // Strobe on start of new frame
    output reg         ntsc_locked,      // High when periodic NTSC sync is verified
    output reg         field,            // 0: Field 1, 1: Field 2

    // Active raster coordinates for video display / framebuffer
    output reg  [9:0]  line_cnt,         // Line within the current field
    output reg  [9:0]  pixel_x,          // Horizontal pixel coordinate (0..419)
    output reg  [8:0]  pixel_y,          // Vertical pixel coordinate (0..239)
    output reg  [7:0]  pixel_val,        // Scaled luminance pixel value (0..255)
    output reg         pixel_valid       // Strobe when pixel_x/y is valid visible video
);

    // -------------------------------------------------------------------------
    // 1. Adaptive Sync Slicer
    // Tracks minimum sync tip level and blanking level
    // -------------------------------------------------------------------------
    reg [7:0] tip_level;
    reg [7:0] blank_level;
    reg [7:0] sync_thresh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tip_level   <= 8'd40;
            blank_level <= 8'd100;
            sync_thresh <= 8'd70;
        end else if (video_valid) begin
            // Track sync tip: fast attack downwards, slow leak upwards
            if (video_in < tip_level)
                tip_level <= video_in;
            else if (tip_level < 8'd80)
                tip_level <= tip_level + 8'd1;

            // Simple robust slicing threshold
            sync_thresh <= tip_level + 8'd25;
        end
    end

    wire is_sync_low = (video_in < sync_thresh);


    // -------------------------------------------------------------------------
    // 2. Pulse Width Classifier & Sync Tip Detector
    // -------------------------------------------------------------------------
    reg [10:0] low_cnt;
    reg [11:0] samples_since_hsync;
    reg [4:0] good_sync_lines;
    reg [9:0] vs_guard;        // lines remaining in which vsync is suppressed
    reg [3:0] miss_cnt;        // consecutive lines with no sync pulse

    // ---- sync flywheel ------------------------------------------------------
    // Once locked, a sync pulse is only believed if it falls in the window where
    // the next line is due. Noise otherwise injects extra pulses that cut a line
    // short, and every following line then starts at a different horizontal
    // offset - which is what turns a real picture into diagonal mush.
    wire sync_in_window = (samples_since_hsync >= LINE_MIN) &&
                          (samples_since_hsync <= LINE_MAX);
    wire accept_sync    = (!ntsc_locked) || sync_in_window;
    reg [8:0] pix_acc;         // accumulates the samples that form one pixel
    reg [15:0] since_sync;     // samples since the last accepted H-sync

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            low_cnt            <= 11'd0;
            samples_since_hsync<= 12'd0;
            good_sync_lines    <= 5'd0;
            vs_guard           <= 10'd0;
            miss_cnt           <= 4'd0;
            pix_acc            <= 9'd0;
            since_sync         <= 16'd0;
            ntsc_locked        <= 1'b0;
            hsync              <= 1'b0;
            line_advance       <= 1'b0;
            vsync              <= 1'b0;
            field              <= 1'b0;
            line_cnt           <= 10'd0;
            pixel_x            <= 10'd0;
            pixel_y            <= 9'd0;
            pixel_val          <= 8'd0;
            pixel_valid        <= 1'b0;
        end else begin
            hsync        <= 1'b0;
            vsync        <= 1'b0;
            line_advance <= 1'b0;
            pixel_valid <= 1'b0;

            if (video_valid) begin
                samples_since_hsync <= samples_since_hsync + 12'd1;

                // Vertical interval detection. No pulse of line width occurs
                // during it, but the sample counter is far too narrow to span
                // it, so count missed LINES instead. Noise drops single lines;
                // only the vertical interval drops several in a row.
                if (samples_since_hsync >= LINE_TIMEOUT) begin
                    // Carry the remainder instead of zeroing: this keeps the
                    // flywheel period at exactly LINE_NOMINAL and keeps the
                    // phase, so a real sync pulse still lands inside the
                    // acceptance window on the next line and can retake control.
                    samples_since_hsync <= samples_since_hsync - LINE_NOMINAL;
                    line_advance        <= 1'b1;   // flywheel: run the line anyway
                    if (line_cnt == 10'd524) line_cnt <= 10'd0;
                    else                     line_cnt <= line_cnt + 10'd1;
                    if (vs_guard != 10'd0)   vs_guard <= vs_guard - 10'd1;
                    if (miss_cnt != 4'd15) miss_cnt <= miss_cnt + 4'd1;
                    if (miss_cnt == (VS_MISSES - 4'd1) && vs_guard == 10'd0) begin
                        vsync    <= 1'b1;
                        line_cnt <= 10'd0;
                        field    <= ~field;
                        vs_guard <= VS_GUARD;
                    end
                end

                // watchdog: no sync for a long time means the signal is gone
                if (since_sync >= LOSS_SAMPLES) begin
                    ntsc_locked     <= 1'b0;
                    good_sync_lines <= 5'd0;
                end else
                    since_sync <= since_sync + 16'd1;

                if (is_sync_low) begin
                    if (low_cnt < LOW_SAT) low_cnt <= low_cnt + 11'd1;
                end else begin
                    // Rising edge of sync pulse (end of low tip)
                    if (low_cnt >= HSYNC_MIN && low_cnt <= HSYNC_MAX &&
                        accept_sync) begin
                        // Standard Horizontal Sync Tip (~4.7 us = 38 samples)
                        hsync        <= 1'b1;
                        line_advance <= 1'b1;

                        // Check line period: NTSC line is 508 samples @ 8 Msps
                        if (samples_since_hsync >= LINE_MIN && samples_since_hsync <= LINE_MAX) begin
                            if (good_sync_lines < 5'd20)
                                good_sync_lines <= good_sync_lines + 5'd1;
                        end else begin
                            if (good_sync_lines > 5'd0)
                                good_sync_lines <= good_sync_lines - 5'd1;
                        end

                        // Lock indicator: requires 8 consecutive valid lines
                        ntsc_locked <= (good_sync_lines >= 5'd8);
                        since_sync  <= 16'd0;
                        miss_cnt    <= 4'd0;

                        // Line counter advance. line_cnt counts lines within
                        // the current field and is reset by vsync.
                        if (line_cnt == 10'd524) line_cnt <= 10'd0;
                        else line_cnt <= line_cnt + 10'd1;

                        if (vs_guard != 10'd0) vs_guard <= vs_guard - 10'd1;

                        samples_since_hsync <= 12'd0;
                    end else if (low_cnt >= BROAD_MIN && vs_guard == 10'd0) begin
                        // Broad vertical serration. Six of them occur back to
                        // back, so only the first one starts a new field.
                        if (vs_guard == 10'd0) begin
                            vsync    <= 1'b1;
                            line_cnt <= 10'd0;
                            field    <= ~field;
                            vs_guard <= VS_GUARD;
                        end
                    end
                    low_cnt <= 11'd0;
                end

                // -------------------------------------------------------------
                // 3. Active Video Pixel Generation
                // Active video starts ~9.5 us (~76 samples) after H-sync and
                // lasts ~52.6 us (~420 samples).
                // -------------------------------------------------------------
                if (samples_since_hsync >= ACT_START && samples_since_hsync < ACT_END) begin
                    // Active visible video range
                    // Average the two samples that form one pixel and emit a
                    // single strobe per pixel, not one per input sample.
                    pixel_x <= (samples_since_hsync - ACT_START) >> PIX_SHIFT;
                    if (((samples_since_hsync - ACT_START) & 12'd1) == 12'd0)
                        pix_acc <= {1'b0, video_in};
                    else
                        pixel_val <= (pix_acc + {1'b0, video_in}) >> 1;

                    // One field -> one 240-line frame; the display side doubles
                    // the lines. line_cnt is field-relative because vsync resets it.
                    if (line_cnt >= VACT_START &&
                        line_cnt <  VACT_START + VACT_LINES &&
                        ((samples_since_hsync - ACT_START) & 12'd1) == 12'd1) begin
                        pixel_y     <= line_cnt - VACT_START;   // 0..239
                        pixel_valid <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
