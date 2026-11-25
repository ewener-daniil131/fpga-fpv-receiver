// =============================================================================
// fpv_video_out.v
// Turns the NTSC field stream into a 1024x600@60 DVI raster - the native mode
// of the panel in use, so the panel does no rescaling of its own.
//
// A field carries 240 active lines and the panel wants 600, a ratio of 2.5, so
// lines cannot simply be doubled as they were for 640x480. Both axes are
// resampled by a DDA instead: 525 source pixels to 1024 across, 240 source
// lines to 600 down. Still no frame buffer - the elastic line FIFO holds the
// few lines the vertical DDA needs, and one NTSC field at 8 bpp would be about
// 1 Mbit against the 828 kbit of block RAM this device has.
//
// The two clocks are independent (capture is paced by the radio, the raster by
// the local PLL), so the vertical blank is stretched or shortened by a few
// lines to track the incoming field rate. That is the genlock: without it the
// picture would slip by a field every few seconds.
// =============================================================================

`timescale 1ns / 1ps

module fpv_video_out #(
    // 1024x600@60.04 with a 48.6 MHz pixel clock (27 MHz x9 /5)
    parameter H_ACT = 1024, parameter H_FP = 40, parameter H_SYNC = 104, parameter H_BP = 144,
    parameter V_ACT = 600,  parameter V_FP = 3,  parameter V_SYNC = 4,   parameter V_BP = 10,
    parameter SRC_LINES = 240,   // active lines in one NTSC field
    parameter SRC_PIX = 525,     // pixels per captured line
    parameter GENLOCK = 1        // adjust vertical blanking to track the source
)(
    // ---- capture domain ----
    input  wire        cap_clk,
    input  wire        cap_rst_n,
    input  wire [9:0]  cap_x,
    input  wire [8:0]  cap_y,
    input  wire [7:0]  cap_val,
    input  wire        cap_valid,
    input  wire        cap_hsync,     // pulse per captured line, lost ones included
    input  wire        cap_vsync,     // pulse at start of each field
    input  wire        cap_locked,

    // ---- pixel domain ----
    input  wire        pix_clk,
    input  wire        pix_rst_n,
    output reg  [7:0]  o_r,
    output reg  [7:0]  o_g,
    output reg  [7:0]  o_b,
    output reg         o_de,
    output reg         o_hs,
    output reg         o_vs
);
    localparam H_TOTAL = H_ACT + H_FP + H_SYNC + H_BP;   // 1312
    localparam V_TOTAL = V_ACT + V_FP + V_SYNC + V_BP;   // 617

    // -------------------------------------------------------------------------
    // line buffer
    // -------------------------------------------------------------------------
    wire [7:0] lb_val;
    wire       lb_ready;
    wire       lb_field;
    // one pulse as the active raster begins, used to anchor to a field
    wire       frame_start;
    reg        lb_next;
    reg [9:0]  lb_rd_x;

    video_line_fifo #(.LINE_BITS(4), .AW(10)) u_lb (
        .wr_clk       (cap_clk),
        .wr_rst_n     (cap_rst_n),
        .wr_x         (cap_x),
        .wr_val       (cap_val),
        .wr_en        (cap_valid),
        .wr_line_done   (cap_hsync),
        .wr_field_start (cap_vsync),
        .rd_clk         (pix_clk),
        .rd_rst_n       (pix_rst_n),
        .rd_x           (lb_rd_x),
        .rd_val         (lb_val),
        .rd_line_done   (lb_next),
        .rd_frame_start (frame_start),
        .rd_field       (lb_field),
        .rd_ready       (lb_ready)
    );

    // carry the lock flag across
    reg [2:0] lk_sync;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) lk_sync <= 3'd0;
        else            lk_sync <= {lk_sync[1:0], cap_locked};
    end
    wire locked_p = lk_sync[2];

    // -------------------------------------------------------------------------
    // raster counters. v_total_adj is nudged by the genlock.
    // -------------------------------------------------------------------------
    reg [11:0] hc, vc;
    reg [11:0] v_total_adj;
    reg signed [7:0] drift;      // captured fields minus displayed frames

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            hc <= 12'd0; vc <= 12'd0;
            v_total_adj <= V_TOTAL[11:0];
            drift <= 8'sd0;
        end else begin
            if (hc == H_TOTAL-1) begin
                hc <= 12'd0;
                if (vc >= v_total_adj-1) begin
                    vc <= 12'd0;
                    if (drift > 8'sd0) drift <= drift - 8'sd1;
                    // shorten the frame when the source is ahead, lengthen it
                    // when it is behind; +-8 lines is invisible to a monitor
                    if (GENLOCK) begin
                        if (drift > 8'sd1)       v_total_adj <= V_TOTAL[11:0] - 12'd8;
                        else if (drift < -8'sd1) v_total_adj <= V_TOTAL[11:0] + 12'd8;
                        else                     v_total_adj <= V_TOTAL[11:0];
                    end
                end else vc <= vc + 12'd1;
            end else hc <= hc + 12'd1;

            if (lb_field && drift < 8'sd120) drift <= drift + 8'sd1;
        end
    end

    // -------------------------------------------------------------------------
    // fetch: one captured line feeds two raster lines
    // -------------------------------------------------------------------------
    // horizontal upscale 525 -> 640 by a DDA: advance the source position by
    // SRC_PIX/H_ACT each output pixel, tracked in 1/1024ths.
    localparam [19:0] STEP  = (SRC_PIX * 1024) / H_ACT;      // source px per output px
    localparam [19:0] VSTEP = (SRC_LINES * 1024) / V_ACT;    // source lines per output line
    reg [19:0] dda;
    reg [19:0] vdda;

    wire in_hact = (hc >= H_SYNC + H_BP) && (hc < H_SYNC + H_BP + H_ACT);
    wire in_vact = (vc >= V_SYNC + V_BP) && (vc < V_SYNC + V_BP + V_ACT);
    assign frame_start = (vc == V_SYNC + V_BP - 1) && (hc == H_TOTAL - 1);

    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            dda <= 20'd0; lb_rd_x <= 10'd0; lb_next <= 1'b0; vdda <= 20'd0;
        end else begin
            lb_next <= 1'b0;

            if (hc == H_SYNC + H_BP - 2) begin
                dda     <= 20'd0;
                lb_rd_x <= 10'd0;
            end else if (in_hact) begin
                dda     <= dda + STEP;
                lb_rd_x <= (dda + STEP) >> 10;
            end

            // vertical resample: advance to the next captured line whenever the
            // accumulator crosses a whole source line (240 -> 600 is 2.5 output
            // lines per source line, so this fires on average every 2.5 lines)
            if (hc == H_TOTAL-1) begin
                if (in_vact) begin
                    if (vdda + VSTEP >= 20'd1024) begin
                        vdda    <= vdda + VSTEP - 20'd1024;
                        lb_next <= 1'b1;
                    end else vdda <= vdda + VSTEP;
                end else vdda <= 20'd0;   // reset the phase during vertical blank
            end
        end
    end

    // -------------------------------------------------------------------------
    // output stage
    // -------------------------------------------------------------------------
    reg de_d, hs_d, vs_d;
    always @(posedge pix_clk or negedge pix_rst_n) begin
        if (!pix_rst_n) begin
            o_r <= 8'd0; o_g <= 8'd0; o_b <= 8'd0;
            o_de <= 1'b0; o_hs <= 1'b0; o_vs <= 1'b0;
            de_d <= 1'b0; hs_d <= 1'b0; vs_d <= 1'b0;
        end else begin
            // one cycle of pipeline to match the block RAM read latency
            de_d <= in_hact && in_vact;
            hs_d <= (hc < H_SYNC);
            vs_d <= (vc < V_SYNC);

            o_de <= de_d;
            o_hs <= hs_d;
            o_vs <= vs_d;

            if (de_d) begin
                if (locked_p && lb_ready) begin
                    o_r <= lb_val; o_g <= lb_val; o_b <= lb_val;
                end else begin
                    // no lock: a dim blue field, so "no signal" is unmistakable
                    o_r <= 8'd0; o_g <= 8'd0; o_b <= 8'd40;
                end
            end else begin
                o_r <= 8'd0; o_g <= 8'd0; o_b <= 8'd0;
            end
        end
    end

endmodule
