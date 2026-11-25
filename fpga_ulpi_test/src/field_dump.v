// =============================================================================
// field_dump.v
// Captures one whole NTSC field of demodulated video and streams it out over
// the debug UART as ASCII hex.
//
// Why not dump IQ: a field is 33 ms, about 660k IQ samples = 2.6 MB, while the
// device has 103 kB of block RAM in total. Dumping raw IQ therefore only ever
// yields ~24 lines - 5% of the picture height - and stretching that to full
// height turns every line into a 20-pixel band, which looks like a defect in
// the processing when it is really just a lack of data.
//
// Storing the demodulated picture instead costs 8 bits per pixel rather than
// 16, and only the active part of the line is kept, so a whole field fits:
//   240 lines x 256 pixels = 61440 bytes.
// =============================================================================

`timescale 1ns / 1ps

module field_dump #(
    parameter LINES  = 240,
    parameter PIXELS = 256          // pixel_x is halved again to reach this
)(
    input  wire        clk,
    input  wire        rst_n,

    // from the sync separator
    input  wire [9:0]  pix_x,
    input  wire [8:0]  pix_y,
    input  wire [7:0]  pix_val,
    input  wire        pix_valid,
    input  wire        field_start,   // vsync
    input  wire        arm,

    output reg         busy,
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready
);
    localparam AW = 16;             // 240*256 = 61440 fits in 16 bits

    (* ram_style = "block" *) reg [7:0] mem [0:(LINES*PIXELS)-1];

    wire [7:0] xq = pix_x[8:1];     // 0..524 -> 0..255
    wire [7:0] yq = pix_y[7:0];
    wire       in_range = (pix_y < LINES) && (pix_x[9:1] < PIXELS);
    wire [AW-1:0] waddr = yq * PIXELS + xq;

    localparam S_IDLE=3'd0, S_CAP=3'd1, S_HDR=3'd2, S_ADDR=3'd3,
               S_WAIT=3'd4, S_HI=3'd5, S_LO=3'd6, S_SEP=3'd7;
    reg [2:0] st;
    reg [AW-1:0] idx;
    reg [7:0] q;
    reg [3:0] hdr_i;
    reg [25:0] pause;

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? (8'h30 + {4'd0,n}) : (8'h41 + {4'd0,n} - 8'd10);
    endfunction

    reg [7:0] hdr [0:7];
    initial begin
        hdr[0]="V"; hdr[1]="F"; hdr[2]="L"; hdr[3]="D";
        hdr[4]=8'h0D; hdr[5]=8'h0A; hdr[6]=8'h00; hdr[7]=8'h00;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; idx <= 0; busy <= 1'b0;
            tx_valid <= 1'b0; tx_data <= 8'd0; hdr_i <= 4'd0; pause <= 26'd0;
        end else begin
            tx_valid <= 1'b0;
            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    // start capturing at the top of a field
                    if (arm && field_start) begin st <= S_CAP; end
                end

                // fill for one whole field, then hand the buffer to the UART
                S_CAP: begin
                    if (pix_valid && in_range) mem[waddr] <= pix_val;
                    if (field_start) begin
                        idx <= 0; hdr_i <= 4'd0; busy <= 1'b1; st <= S_HDR;
                    end
                end

                S_HDR: if (tx_ready && !tx_valid) begin
                    tx_data <= hdr[hdr_i[2:0]]; tx_valid <= 1'b1;
                    if (hdr_i == 4'd5) st <= S_ADDR; else hdr_i <= hdr_i + 4'd1;
                end

                S_ADDR: begin q <= mem[idx]; st <= S_WAIT; end
                S_WAIT: st <= S_HI;

                S_HI: if (tx_ready && !tx_valid) begin
                    tx_data <= hexc(q[7:4]); tx_valid <= 1'b1; st <= S_LO;
                end
                S_LO: if (tx_ready && !tx_valid) begin
                    tx_data <= hexc(q[3:0]); tx_valid <= 1'b1; st <= S_SEP;
                end
                S_SEP: if (tx_ready && !tx_valid) begin
                    // newline at the end of every stored line
                    tx_data  <= (idx[7:0] == 8'hFF) ? 8'h0A : 8'h20;
                    tx_valid <= 1'b1;
                    if (idx == (LINES*PIXELS)-1) begin
                        pause <= 26'd0; st <= S_IDLE; busy <= 1'b0;
                    end else begin
                        idx <= idx + 1'b1; st <= S_ADDR;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
