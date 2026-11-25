// =============================================================================
// usb_iq_fifo.v
// Top-level USB Bulk IQ Depacketizer & Asynchronous FIFO Buffer
// Bridges HackRF 60 MHz ULPI USB stream to FPGA DSP clock domain (27 MHz / etc.)
// =============================================================================

`timescale 1ns / 1ps

module usb_iq_fifo #(
    parameter ADDR_WIDTH = 12   // Depth = 4096 words = 4096 complex IQ samples
)(
    // ---- ULPI / USB Host Clock Domain (60 MHz ulpi_clk_i) ----
    input  wire        wr_clk,
    input  wire        wr_rst_n,
    input  wire        in_bulk_rx,       // High when host is in H_BRXW
    input  wire        rx_byte_valid,    // High when PHY delivers data byte
    input  wire [7:0]  rx_data,          // Data byte from PHY
    input  wire        rx_pkt_done,      // Pulse when packet ends
    input  wire [9:0]  rx_pkt_len,       // Length of received packet

    // Write-domain telemetry
    output wire [ADDR_WIDTH:0] wr_occupancy,
    output wire                wr_full,
    output wire                wr_overflow,
    output wire [15:0]         pkt_count,
    output wire [31:0]         sample_count,
    output wire [15:0]         err_count,
    output wire                is_bulk_active,

    // ---- DSP / Video Clock Domain (27 MHz clk27 or DSP clock) ----
    input  wire        rd_clk,
    input  wire        rd_rst_n,
    input  wire        rd_en,            // Read strobe from DSP
    output wire [15:0] rd_data,          // {I[7:0], Q[7:0]} signed 8-bit samples
    output wire        rd_valid,         // Data valid strobe (1 cycle latency)
    output wire        rd_empty,         // FIFO empty status
    output wire [ADDR_WIDTH:0] rd_occupancy, // Available sample count in FIFO
    output wire        rd_underflow
);

    wire [15:0] depack_sample_data;
    wire        depack_sample_valid;
    wire        depack_pkt_commit;
    wire        depack_pkt_rollback;
    wire        fifo_almost_full;

    // -------------------------------------------------------------------------
    // Depacketizer: extracts {I, Q} and strips PID & CRC16
    // -------------------------------------------------------------------------
    usb_iq_depacketizer u_depacketizer (
        .clk            (wr_clk),
        .rst_n          (wr_rst_n),
        .in_bulk_rx     (in_bulk_rx),
        .rx_byte_valid  (rx_byte_valid),
        .rx_data        (rx_data),
        .rx_pkt_done    (rx_pkt_done),
        .rx_pkt_len     (rx_pkt_len),
        .fifo_full      (fifo_almost_full),

        .sample_data    (depack_sample_data),
        .sample_valid   (depack_sample_valid),
        .pkt_commit     (depack_pkt_commit),
        .pkt_rollback   (depack_pkt_rollback),
        .pkt_count      (pkt_count),
        .sample_count   (sample_count),
        .err_count      (err_count),
        .is_bulk_active (is_bulk_active)
    );

    // -------------------------------------------------------------------------
    // Dual-Clock FIFO with atomic packet commit
    // -------------------------------------------------------------------------
    async_fifo #(
        .DATA_WIDTH  (16),
        .ADDR_WIDTH  (ADDR_WIDTH),
        .PACKET_MODE (1)
    ) u_async_fifo (
        .wr_clk         (wr_clk),
        .wr_rst_n       (wr_rst_n),
        .wr_en          (depack_sample_valid),
        .wr_data        (depack_sample_data),
        .wr_commit      (depack_pkt_commit),
        .wr_rollback    (depack_pkt_rollback),
        .wr_full        (wr_full),
        .wr_almost_full (fifo_almost_full),
        .wr_occupancy   (wr_occupancy),
        .wr_overflow    (wr_overflow),

        .rd_clk         (rd_clk),
        .rd_rst_n       (rd_rst_n),
        .rd_en          (rd_en),
        .rd_data        (rd_data),
        .rd_valid       (rd_valid),
        .rd_empty       (rd_empty),
        .rd_occupancy   (rd_occupancy),
        .rd_underflow   (rd_underflow)
    );

endmodule
