// =============================================================================
// video_line_fifo.v
// Elastic line FIFO between the capture domain and the display domain.
//
// Why not a two-line ping-pong: IQ samples arrive from USB in bursts, and the
// sample FIFO is drained whenever it is non-empty, so the demodulator advances
// in spurts. Line strobes therefore carry tens of microseconds of jitter while
// the raster is perfectly regular. With only two buffers the reader lands in
// the middle of a buffer swap and stitches together halves of two different
// lines. Holding several lines absorbs the jitter.
//
// The reader asks for the next line when it has finished one; if none is ready
// it repeats the current line instead of tearing, and if the writer runs ahead
// the oldest lines are simply overwritten.
// =============================================================================

`timescale 1ns / 1ps

module video_line_fifo #(
    parameter LINE_BITS = 4,           // 16 lines
    parameter AW        = 10           // 1024 pixels per line
)(
    // ---- capture domain ----
    input  wire            wr_clk,
    input  wire            wr_rst_n,
    input  wire [AW-1:0]   wr_x,
    input  wire [7:0]      wr_val,
    input  wire            wr_en,
    input  wire            wr_line_done,
    input  wire            wr_field_start,   // pulse at the start of a field

    // ---- pixel domain ----
    input  wire            rd_clk,
    input  wire            rd_rst_n,
    input  wire [AW-1:0]   rd_x,
    output reg  [7:0]      rd_val,
    input  wire            rd_line_done,   // reader finished a source line
    input  wire            rd_frame_start, // reader is starting a new frame
    output reg             rd_field,       // pulse: a field started (for genlock)
    output wire            rd_ready        // enough lines buffered to start
);
    localparam LINES = (1 << LINE_BITS);

    (* ram_style = "block" *) reg [7:0] mem [0:(LINES << AW)-1];

    // ---- write side ----
    reg [LINE_BITS:0] wr_line;           // one extra bit for wrap detection
    reg [LINE_BITS:0] wr_field_base;      // wr_line when the field began
    reg               wr_field_tog;
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_line <= 0; wr_field_base <= 0; wr_field_tog <= 1'b0;
        end else begin
            if (wr_en) mem[{wr_line[LINE_BITS-1:0], wr_x}] <= wr_val;
            if (wr_line_done) wr_line <= wr_line + 1'b1;
            if (wr_field_start) begin
                wr_field_base <= wr_line;
                wr_field_tog  <= ~wr_field_tog;
            end
        end
    end

    // ---- write pointer into the read domain, Gray coded ----
    function [LINE_BITS:0] bin2gray(input [LINE_BITS:0] b);
        bin2gray = b ^ (b >> 1);
    endfunction
    function [LINE_BITS:0] gray2bin(input [LINE_BITS:0] g);
        integer k;
        begin
            gray2bin[LINE_BITS] = g[LINE_BITS];
            for (k = LINE_BITS-1; k >= 0; k = k - 1)
                gray2bin[k] = gray2bin[k+1] ^ g[k];
        end
    endfunction

    reg [LINE_BITS:0] wr_gray;
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) wr_gray <= 0;
        else           wr_gray <= bin2gray(wr_line);
    end

    reg [LINE_BITS:0] wr_gray_s0, wr_gray_s1;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin wr_gray_s0 <= 0; wr_gray_s1 <= 0; end
        else begin wr_gray_s0 <= wr_gray; wr_gray_s1 <= wr_gray_s0; end
    end
    wire [LINE_BITS:0] wr_line_r = gray2bin(wr_gray_s1);

    // field marker across the domain boundary, as a toggle
    reg [LINE_BITS:0] base_gray;
    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) base_gray <= 0;
        else           base_gray <= bin2gray(wr_field_base);
    end
    reg [LINE_BITS:0] base_s0, base_s1;
    reg [2:0] ftog_s;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin base_s0<=0; base_s1<=0; ftog_s<=3'd0; rd_field<=1'b0; end
        else begin
            base_s0 <= base_gray; base_s1 <= base_s0;
            ftog_s  <= {ftog_s[1:0], wr_field_tog};
            rd_field <= ftog_s[2] ^ ftog_s[1];
        end
    end
    wire [LINE_BITS:0] field_base_r = gray2bin(base_s1);

    // ---- read side ----
    reg [LINE_BITS:0] rd_line;
    wire [LINE_BITS:0] level = wr_line_r - rd_line;

    // start only once the buffer is half full, so the first frames do not
    // stutter while the pipeline fills
    reg started;
    assign rd_ready = started;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_line <= 0; started <= 1'b0;
        end else begin
            if (!started) begin
                if (level >= (LINES/2)) started <= 1'b1;
                rd_line <= wr_line_r - (LINES/2);
            end else if (rd_frame_start) begin
                // anchor the frame to the start of a captured field, so a
                // single frame never mixes lines from two different fields
                rd_line <= field_base_r;
            end else if (rd_line_done) begin
                // advance only if a complete newer line exists
                if (level > 1) rd_line <= rd_line + 1'b1;
            end
            // if the writer lapped us, jump back into the middle of the window
            if (started && level > (LINES - 2))
                rd_line <= wr_line_r - (LINES/2);
        end
    end

    always @(posedge rd_clk) rd_val <= mem[{rd_line[LINE_BITS-1:0], rd_x}];

endmodule
