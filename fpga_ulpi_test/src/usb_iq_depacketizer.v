// =============================================================================
// usb_iq_depacketizer.v
// Extracts signed 8-bit {I, Q} sample pairs from USB 2.0 High-Speed bulk packets
// Strips USB PID (0x4B/0xC3) and CRC16 (2 bytes).
// Generates 16-bit sample words {I[7:0], Q[7:0]} for the asynchronous FIFO.
// =============================================================================

`timescale 1ns / 1ps

module usb_iq_depacketizer (
    input  wire        clk,              // ulpi_clk_i (60 MHz)
    input  wire        rst_n,            // rst_n_u (active low reset)

    // Link interface
    input  wire        in_bulk_rx,       // High when host expects bulk IN data (H_BRXW)
    input  wire        rx_byte_valid,    // High for 1 cycle per valid ULPI byte
    input  wire [7:0]  rx_data,          // ULPI byte data
    input  wire        rx_pkt_done,      // Pulse when packet ends (rx_valid)
    input  wire [9:0]  rx_pkt_len,       // Total packet byte count (rx_len)

    // FIFO backpressure / flow control
    input  wire        fifo_full,        // High if FIFO cannot accept 256 samples

    // FIFO write interface
    output reg  [15:0] sample_data,      // {I[7:0], Q[7:0]} signed two's complement
    output reg         sample_valid,     // Write strobe to FIFO
    output reg         pkt_commit,       // Commit strobe to FIFO at valid packet end
    output reg         pkt_rollback,     // Rollback strobe on error/abort

    // Telemetry & diagnostics
    output reg  [15:0] pkt_count,        // Number of successfully unpacked 512B packets
    output reg  [31:0] sample_count,     // Number of IQ sample pairs extracted (256/pkt)
    output reg  [15:0] err_count,        // Dropped / invalid packet count
    output wire        is_bulk_active    // Active receiving bulk packet
);

    reg [9:0] byte_cnt;
    reg       is_iq;
    reg [7:0] i_reg;

    assign is_bulk_active = is_iq;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_cnt     <= 10'd0;
            is_iq        <= 1'b0;
            i_reg        <= 8'h00;
            sample_data  <= 16'h0000;
            sample_valid <= 1'b0;
            pkt_commit   <= 1'b0;
            pkt_rollback <= 1'b0;
            pkt_count    <= 16'd0;
            sample_count <= 32'd0;
            err_count    <= 16'd0;
        end else begin
            // Default single-cycle strobes
            sample_valid <= 1'b0;
            pkt_commit   <= 1'b0;
            pkt_rollback <= 1'b0;

            // Host left bulk RX state unexpectedly while mid-packet
            if (!in_bulk_rx && is_iq) begin
                pkt_rollback <= 1'b1;
                err_count    <= err_count + 16'd1;
                is_iq        <= 1'b0;
                byte_cnt     <= 10'd0;
            end else if (rx_byte_valid) begin
                if (byte_cnt == 10'd0) begin
                    // Byte 0: USB Packet Identifier (PID)
                    // High-Speed Bulk DATA0 is 0x4B, DATA1 is 0xC3
                    if (in_bulk_rx && ((rx_data == 8'h4B) || (rx_data == 8'hC3))) begin
                        if (!fifo_full) begin
                            is_iq    <= 1'b1;
                            byte_cnt <= 10'd1;
                        end else begin
                            // FIFO full: drop packet
                            is_iq     <= 1'b0;
                            err_count <= err_count + 16'd1;
                        end
                    end else begin
                        // Non-data response (e.g. NAK 0x4A, STALL, NYET) or not in bulk state
                        is_iq    <= 1'b0;
                        byte_cnt <= 10'd0;
                    end
                end else if (is_iq) begin
                    byte_cnt <= byte_cnt + 10'd1;

                    if (byte_cnt <= 10'd512) begin
                        if (byte_cnt[0] == 1'b1) begin
                            // Odd index (1, 3, ..., 511): In-phase sample (I)
                            i_reg <= rx_data;
                        end else begin
                            // Even index (2, 4, ..., 512): Quadrature sample (Q)
                            sample_data  <= {i_reg, rx_data};
                            sample_valid <= 1'b1;
                        end
                    end
                    // Bytes 513 and 514 are USB CRC16 - ignored
                end
            end

            // Packet completed signal from link layer
            if (rx_pkt_done) begin
                if (is_iq && (rx_pkt_len == 10'd515)) begin
                    // Valid 512-byte payload bulk packet (1 PID + 512 payload + 2 CRC16)
                    pkt_commit   <= 1'b1;
                    pkt_count    <= pkt_count + 16'd1;
                    sample_count <= sample_count + 32'd256;
                end else if (is_iq) begin
                    // Truncated, aborted, or oversized packet: rollback
                    pkt_rollback <= 1'b1;
                    err_count    <= err_count + 16'd1;
                end
                is_iq    <= 1'b0;
                byte_cnt <= 10'd0;
            end
        end
    end

endmodule
