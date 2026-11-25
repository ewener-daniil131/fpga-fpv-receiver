// =============================================================================
// async_fifo.v
// Asynchronous Dual-Clock FIFO with Packet-Commit / Rollback Support
// Designed for Gowin GW2A FPGA (Block RAM BSRAM inference)
// =============================================================================

`timescale 1ns / 1ps

module async_fifo #(
    parameter DATA_WIDTH   = 16,
    parameter ADDR_WIDTH   = 12,    // Depth = 2^ADDR_WIDTH = 4096 words
    parameter PACKET_MODE  = 1      // 1: commit/rollback mode, 0: direct streaming
)(
    // ---- Write Clock Domain (e.g. 60 MHz ulpi_clk_i) ----
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    input  wire                  wr_commit,
    input  wire                  wr_rollback,
    output wire                  wr_full,
    output wire                  wr_almost_full, // Less than 256 words remaining
    output wire [ADDR_WIDTH:0]   wr_occupancy,
    output reg                   wr_overflow,

    // ---- Read Clock Domain (e.g. 27 MHz clk27 or DSP clock) ----
    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output reg  [DATA_WIDTH-1:0] rd_data,
    output reg                   rd_valid,
    output wire                  rd_empty,
    output wire [ADDR_WIDTH:0]   rd_occupancy,
    output reg                   rd_underflow
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    // -------------------------------------------------------------------------
    // Dual-Port RAM (Infers Gowin BSRAM Block Memory)
    // -------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // -------------------------------------------------------------------------
    // Write Domain Pointers & Logic
    // -------------------------------------------------------------------------
    reg [ADDR_WIDTH:0] wr_ptr_cur;       // Current uncommitted write pointer
    reg [ADDR_WIDTH:0] wr_ptr_commit;    // Committed write pointer
    reg [ADDR_WIDTH:0] wr_ptr_gray;      // Gray-code committed write pointer

    reg [ADDR_WIDTH:0] wr_ptr_next;
    always @(*) begin
        wr_ptr_next = wr_ptr_cur;
        if (wr_en && !wr_full)
            wr_ptr_next = wr_ptr_cur + 1'b1;
    end

    // Synchronize read pointer into write clock domain (2-stage synchronizer)
    reg [ADDR_WIDTH:0] rd_ptr_gray_s1;
    reg [ADDR_WIDTH:0] rd_ptr_gray_s2;
    wire [ADDR_WIDTH:0] rd_ptr_gray_w;   // From rd domain

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_ptr_gray_s1 <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray_s2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            rd_ptr_gray_s1 <= rd_ptr_gray_w;
            rd_ptr_gray_s2 <= rd_ptr_gray_s1;
        end
    end

    // Convert synchronized Gray read pointer back to binary
    reg [ADDR_WIDTH:0] rd_ptr_bin_wr;
    integer i;
    always @(*) begin
        rd_ptr_bin_wr[ADDR_WIDTH] = rd_ptr_gray_s2[ADDR_WIDTH];
        for (i = ADDR_WIDTH - 1; i >= 0; i = i - 1)
            rd_ptr_bin_wr[i] = rd_ptr_bin_wr[i+1] ^ rd_ptr_gray_s2[i];
    end

    // Write occupancy and full flags based on uncommitted pointer
    assign wr_occupancy = wr_ptr_cur - rd_ptr_bin_wr;
    wire [ADDR_WIDTH:0] words_free = DEPTH - wr_occupancy;
    assign wr_full        = (words_free == 0);
    assign wr_almost_full = (words_free < 13'd256);

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_cur    <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_commit <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray   <= {(ADDR_WIDTH+1){1'b0}};
            wr_overflow   <= 1'b0;
        end else begin
            // Memory write and pointer advance
            if (wr_en && !wr_full) begin
                mem[wr_ptr_cur[ADDR_WIDTH-1:0]] <= wr_data;
                wr_ptr_cur <= wr_ptr_next;
                if (PACKET_MODE == 0) begin
                    wr_ptr_commit <= wr_ptr_next;
                    wr_ptr_gray   <= (wr_ptr_next >> 1) ^ wr_ptr_next;
                end
            end else if (wr_en && wr_full) begin
                wr_overflow <= 1'b1;
            end

            // Packet commit / rollback handling
            if (PACKET_MODE != 0) begin
                if (wr_commit) begin
                    wr_ptr_commit <= wr_ptr_next;
                    wr_ptr_gray   <= (wr_ptr_next >> 1) ^ wr_ptr_next;
                end else if (wr_rollback) begin
                    wr_ptr_cur    <= wr_ptr_commit;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read Domain Pointers & Logic
    // -------------------------------------------------------------------------
    reg [ADDR_WIDTH:0] rd_ptr_bin;
    reg [ADDR_WIDTH:0] rd_ptr_gray;
    assign rd_ptr_gray_w = rd_ptr_gray;

    // Synchronize committed write pointer into read clock domain (2-stage)
    reg [ADDR_WIDTH:0] wr_ptr_gray_s1;
    reg [ADDR_WIDTH:0] wr_ptr_gray_s2;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_ptr_gray_s1 <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray_s2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            wr_ptr_gray_s1 <= wr_ptr_gray;
            wr_ptr_gray_s2 <= wr_ptr_gray_s1;
        end
    end

    // Convert synchronized Gray write pointer back to binary
    reg [ADDR_WIDTH:0] wr_ptr_bin_rd;
    integer j;
    always @(*) begin
        wr_ptr_bin_rd[ADDR_WIDTH] = wr_ptr_gray_s2[ADDR_WIDTH];
        for (j = ADDR_WIDTH - 1; j >= 0; j = j - 1)
            wr_ptr_bin_rd[j] = wr_ptr_bin_rd[j+1] ^ wr_ptr_gray_s2[j];
    end

    // Read domain flags
    assign rd_empty     = (rd_ptr_gray == wr_ptr_gray_s2);
    assign rd_occupancy = wr_ptr_bin_rd - rd_ptr_bin;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin   <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray  <= {(ADDR_WIDTH+1){1'b0}};
            rd_data      <= {DATA_WIDTH{1'b0}};
            rd_valid     <= 1'b0;
            rd_underflow <= 1'b0;
        end else begin
            rd_valid <= 1'b0;
            if (rd_en && !rd_empty) begin
                rd_data     <= mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
                rd_valid    <= 1'b1;
                rd_ptr_bin  <= rd_ptr_bin + 1'b1;
                rd_ptr_gray <= ((rd_ptr_bin + 1'b1) >> 1) ^ (rd_ptr_bin + 1'b1);
            end else if (rd_en && rd_empty) begin
                rd_underflow <= 1'b1;
            end
        end
    end

endmodule
