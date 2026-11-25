// =============================================================================
// iq_dump.v
// Captures a contiguous block of IQ samples into BSRAM and streams it out over
// the debug UART as ASCII hex, so the raw data can be demodulated off-line on
// a PC. This is ground truth: it separates "the RF capture is wrong" from
// "the on-chip demodulator is wrong", which no aggregate statistic can do.
//
// Cycle: capture DEPTH samples -> dump them -> pause -> repeat.
// =============================================================================

`timescale 1ns / 1ps

module iq_dump #(
    parameter DEPTH_LOG2 = 13,              // 8192 samples = 410 us @ 20 Msps (~6 lines)
    parameter PAUSE_CYC  = 27_000_000       // 1 s between dumps
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] sample_in,
    input  wire        sample_valid,
    input  wire        arm,                 // hold high once the radio is streaming

    output reg         busy,                // high while capturing or dumping
    output reg  [7:0]  tx_data,
    output reg         tx_valid,
    input  wire        tx_ready
);
    localparam DEPTH = (1 << DEPTH_LOG2);

    reg [15:0] mem [0:DEPTH-1];
    reg [15:0] mem_q;
    reg [DEPTH_LOG2-1:0] idx;

    localparam S_IDLE = 3'd0, S_CAP = 3'd1, S_ADDR = 3'd2, S_WAIT = 3'd3,
               S_EMIT = 3'd4, S_SEP  = 3'd5, S_HDR = 3'd6, S_PAUSE = 3'd7;
    reg [2:0] st;
    reg [2:0] nib;            // which character of the current sample
    reg [3:0] hdr_i;
    reg [25:0] pause_cnt;

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h41 + {4'd0, n} - 8'd10);
    endfunction

    // "IQDUMP\r\n" marks the start of a block so the PC side can resynchronise
    reg [7:0] hdr [0:7];
    initial begin
        hdr[0]="I"; hdr[1]="Q"; hdr[2]="D"; hdr[3]="U";
        hdr[4]="M"; hdr[5]="P"; hdr[6]=8'h0D; hdr[7]=8'h0A;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= S_IDLE; idx <= 0; nib <= 3'd0; hdr_i <= 4'd0;
            tx_valid <= 1'b0; tx_data <= 8'd0; busy <= 1'b0;
            pause_cnt <= 26'd0; mem_q <= 16'd0;
        end else begin
            tx_valid <= 1'b0;

            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    idx  <= 0;
                    if (arm) begin st <= S_CAP; end
                end

                // ---- fill the buffer with consecutive samples ----
                S_CAP: begin
                    busy <= 1'b0;
                    if (sample_valid) begin
                        mem[idx] <= sample_in;
                        if (idx == DEPTH-1) begin
                            idx <= 0; hdr_i <= 4'd0; st <= S_HDR; busy <= 1'b1;
                        end else idx <= idx + 1'b1;
                    end
                end

                S_HDR: begin
                    if (tx_ready && !tx_valid) begin
                        tx_data  <= hdr[hdr_i[2:0]];
                        tx_valid <= 1'b1;
                        if (hdr_i == 4'd7) st <= S_ADDR;
                        else hdr_i <= hdr_i + 4'd1;
                    end
                end

                // ---- stream the buffer as hex ----
                S_ADDR: begin mem_q <= mem[idx]; st <= S_WAIT; end
                S_WAIT: begin nib <= 3'd0; st <= S_EMIT; end

                S_EMIT: begin
                    if (tx_ready && !tx_valid) begin
                        case (nib)
                            3'd0: tx_data <= hexc(mem_q[15:12]);
                            3'd1: tx_data <= hexc(mem_q[11:8]);
                            3'd2: tx_data <= hexc(mem_q[7:4]);
                            default: tx_data <= hexc(mem_q[3:0]);
                        endcase
                        tx_valid <= 1'b1;
                        if (nib == 3'd3) st <= S_SEP;
                        else nib <= nib + 3'd1;
                    end
                end

                S_SEP: begin
                    if (tx_ready && !tx_valid) begin
                        // newline every 16 samples, space otherwise
                        tx_data  <= (idx[3:0] == 4'd15) ? 8'h0A : 8'h20;
                        tx_valid <= 1'b1;
                        if (idx == DEPTH-1) begin
                            pause_cnt <= 26'd0; st <= S_PAUSE; busy <= 1'b0;
                        end else begin
                            idx <= idx + 1'b1; st <= S_ADDR;
                        end
                    end
                end

                S_PAUSE: begin
                    busy <= 1'b0;
                    if (pause_cnt >= PAUSE_CYC[25:0]) begin
                        idx <= 0; st <= S_IDLE;
                    end else pause_cnt <= pause_cnt + 26'd1;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
