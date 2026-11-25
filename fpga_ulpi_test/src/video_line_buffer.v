// =============================================================================
// video_line_buffer.v
// Ping-pong line buffer bridging the NTSC capture domain (27 MHz, driven by
// samples arriving over USB) and the display domain (25.2 MHz pixel clock).
//
// The writer fills one half while the reader scans the other. A line is handed
// over by toggling a flag that is synchronised into the read domain, so the two
// clocks need no fixed relationship. The reader always has a complete line, so
// a torn line can never be displayed; when the domains drift the same line is
// shown twice or one is skipped, which is what the genlock in the timing
// generator is there to minimise.
// =============================================================================

`timescale 1ns / 1ps

module video_line_buffer #(
    parameter LINE_PIX  = 525,      // pixels per NTSC line
    parameter ADDR_BITS = 10
)(
    // ---- write side: NTSC capture domain ----
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire [ADDR_BITS-1:0]  wr_x,
    input  wire [7:0]            wr_val,
    input  wire                  wr_en,
    input  wire                  wr_line_done,   // pulse at end of a line
    input  wire                  wr_field_start, // pulse at vsync

    // ---- read side: pixel clock domain ----
    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire [ADDR_BITS-1:0]  rd_x,
    output wire [7:0]            rd_val,
    output reg                   rd_line_avail,  // a fresh line is ready
    input  wire                  rd_line_ack,    // reader consumed it
    output reg                   rd_field_sync   // pulse: new field started
);

    localparam DEPTH = (1 << ADDR_BITS);

    (* ram_style = "block" *) reg [7:0] buf0 [0:DEPTH-1];
    (* ram_style = "block" *) reg [7:0] buf1 [0:DEPTH-1];

    // ---- write side ----
    reg wr_sel;              // which half is being written
    reg wr_toggle;           // toggles once per completed line
    reg wr_field_toggle;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_sel <= 1'b0; wr_toggle <= 1'b0; wr_field_toggle <= 1'b0;
        end else begin
            if (wr_en) begin
                if (wr_sel) buf1[wr_x] <= wr_val;
                else        buf0[wr_x] <= wr_val;
            end
            if (wr_line_done) begin
                wr_sel    <= ~wr_sel;
                wr_toggle <= ~wr_toggle;
            end
            if (wr_field_start) wr_field_toggle <= ~wr_field_toggle;
        end
    end

    // ---- cross into the read domain ----
    reg [2:0] tog_sync, fld_sync;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            tog_sync <= 3'd0; fld_sync <= 3'd0;
            rd_line_avail <= 1'b0; rd_field_sync <= 1'b0;
        end else begin
            tog_sync <= {tog_sync[1:0], wr_toggle};
            fld_sync <= {fld_sync[1:0], wr_field_toggle};

            rd_field_sync <= fld_sync[2] ^ fld_sync[1];

            // a new line arrived
            if (tog_sync[2] ^ tog_sync[1]) rd_line_avail <= 1'b1;
            else if (rd_line_ack)          rd_line_avail <= 1'b0;
        end
    end

    // The reader scans the half that is NOT being written. wr_sel is stable for
    // a whole line and only changes at the same instant as wr_toggle, so
    // sampling it through the same synchroniser keeps reader and writer apart.
    reg rd_sel_s0, rd_sel_s1;
    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin rd_sel_s0 <= 1'b0; rd_sel_s1 <= 1'b0; end
        else begin rd_sel_s0 <= wr_sel; rd_sel_s1 <= rd_sel_s0; end
    end

    reg [7:0] rd_q0, rd_q1;
    always @(posedge rd_clk) begin
        rd_q0 <= buf0[rd_x];
        rd_q1 <= buf1[rd_x];
    end
    // read the half the writer is not in
    assign rd_val = rd_sel_s1 ? rd_q0 : rd_q1;

endmodule
