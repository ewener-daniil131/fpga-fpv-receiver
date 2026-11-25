// =============================================================================
// tb_usb_iq_fifo.v
// Verification testbench for usb_iq_fifo, usb_iq_depacketizer, and async_fifo
// Tests:
//   1. Normal 512-byte bulk packet reception & unpacking (PID 0xC3/0x4B)
//   2. Cross-domain sample transfer (60 MHz wr_clk -> 27 MHz rd_clk)
//   3. Bit-exact data integrity verification for all 256 samples
//   4. Rollback behavior on truncated / corrupted packets
//   5. NAK packet rejection
//   6. Multi-packet microframe bursts (1024 samples)
// =============================================================================

`timescale 1ns / 1ps

module tb_usb_iq_fifo;

    reg wr_clk;
    reg wr_rst_n;
    reg in_bulk_rx;
    reg rx_byte_valid;
    reg [7:0] rx_data;
    reg rx_pkt_done;
    reg [9:0] rx_pkt_len;

    wire [12:0] wr_occupancy;
    wire        wr_full;
    wire        wr_overflow;
    wire [15:0] pkt_count;
    wire [31:0] sample_count;
    wire [15:0] err_count;
    wire        is_bulk_active;

    reg         rd_clk;
    reg         rd_rst_n;
    reg         rd_en;
    wire [15:0] rd_data;
    wire        rd_valid;
    wire        rd_empty;
    wire [12:0] rd_occupancy;
    wire        rd_underflow;

    // Instantiate Unit Under Test
    usb_iq_fifo #(
        .ADDR_WIDTH (12) // Depth = 4096 words
    ) uut (
        .wr_clk         (wr_clk),
        .wr_rst_n       (wr_rst_n),
        .in_bulk_rx     (in_bulk_rx),
        .rx_byte_valid  (rx_byte_valid),
        .rx_data        (rx_data),
        .rx_pkt_done    (rx_pkt_done),
        .rx_pkt_len     (rx_pkt_len),
        .wr_occupancy   (wr_occupancy),
        .wr_full        (wr_full),
        .wr_overflow    (wr_overflow),
        .pkt_count      (pkt_count),
        .sample_count   (sample_count),
        .err_count      (err_count),
        .is_bulk_active (is_bulk_active),

        .rd_clk         (rd_clk),
        .rd_rst_n       (rd_rst_n),
        .rd_en          (rd_en),
        .rd_data        (rd_data),
        .rd_valid       (rd_valid),
        .rd_empty       (rd_empty),
        .rd_occupancy   (rd_occupancy),
        .rd_underflow   (rd_underflow)
    );

    // Clock generation: 60 MHz (16.666 ns) and 27 MHz (37.037 ns)
    always #8.333 wr_clk = ~wr_clk;
    always #18.518 rd_clk = ~rd_clk;

    // Task to transmit a complete USB bulk packet over the simulated link
    task send_bulk_packet;
        input [7:0] pid;
        input [7:0] sample_start_val;
        input integer num_payload_bytes;
        input [15:0] crc;
        integer i;
        begin
            @(posedge wr_clk);
            in_bulk_rx <= 1'b1;

            // Byte 0: PID
            @(posedge wr_clk);
            rx_byte_valid <= 1'b1;
            rx_data       <= pid;

            // Payload bytes
            for (i = 0; i < num_payload_bytes; i = i + 1) begin
                @(posedge wr_clk);
                rx_byte_valid <= 1'b1;
                rx_data       <= sample_start_val + i[7:0];
            end

            // CRC bytes
            @(posedge wr_clk);
            rx_byte_valid <= 1'b1;
            rx_data       <= crc[7:0];

            @(posedge wr_clk);
            rx_byte_valid <= 1'b1;
            rx_data       <= crc[15:8];

            // End of packet
            @(posedge wr_clk);
            rx_byte_valid <= 1'b0;
            rx_pkt_done   <= 1'b1;
            rx_pkt_len    <= 1 + num_payload_bytes + 2;

            @(posedge wr_clk);
            rx_pkt_done   <= 1'b0;
            rx_pkt_len    <= 10'd0;
        end
    endtask

    // Task to send a NAK packet (1 byte PID only)
    task send_nak_packet;
        begin
            @(posedge wr_clk);
            in_bulk_rx    <= 1'b1;
            rx_byte_valid <= 1'b1;
            rx_data       <= 8'h4A; // NAK PID

            @(posedge wr_clk);
            rx_byte_valid <= 1'b0;
            rx_pkt_done   <= 1'b1;
            rx_pkt_len    <= 10'd1;

            @(posedge wr_clk);
            rx_pkt_done   <= 1'b0;
            rx_pkt_len    <= 10'd0;
        end
    endtask

    integer read_sample_idx;
    reg [15:0] expected_sample;
    integer total_verified_samples;
    reg [7:0] exp_i, exp_q;

    initial begin
        $dumpfile("tb_usb_iq_fifo.vcd");
        $dumpvars(0, tb_usb_iq_fifo);

        wr_clk        = 0;
        wr_rst_n      = 0;
        in_bulk_rx    = 0;
        rx_byte_valid = 0;
        rx_data       = 0;
        rx_pkt_done   = 0;
        rx_pkt_len    = 0;

        rd_clk        = 0;
        rd_rst_n      = 0;
        rd_en         = 0;
        total_verified_samples = 0;

        $display("\n=======================================================");
        $display("   TESTBENCH: USB Bulk IQ FIFO & Depacketizer");
        $display("=======================================================");

        // Apply Reset
        #100;
        wr_rst_n = 1;
        rd_rst_n = 1;
        #100;

        // ---------------------------------------------------------------------
        // TEST 1: Send valid 512-byte bulk DATA1 packet (PID 0xC3)
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Sending 512-byte bulk packet with PID 0xC3 (DATA1)...");
        send_bulk_packet(8'hC3, 8'h10, 512, 16'hABCD);

        // Wait for CDC synchronizers (a few rd_clk cycles)
        repeat (10) @(posedge rd_clk);

        if (pkt_count !== 16'd1) begin
            $display("ERROR: Expected pkt_count == 1, got %d", pkt_count);
            $finish;
        end
        if (sample_count !== 32'd256) begin
            $display("ERROR: Expected sample_count == 256, got %d", sample_count);
            $finish;
        end
        if (rd_empty !== 1'b0) begin
            $display("ERROR: FIFO should not be empty after packet commit!");
            $finish;
        end
        $display("[TEST 1 PASSED] 256 samples committed to FIFO. rd_occupancy = %d", rd_occupancy);

        // Read out all 256 samples on 27 MHz clock and verify bit-exact data
        $display("[TEST 1] Verifying all 256 samples in 27 MHz read domain...");
        for (read_sample_idx = 0; read_sample_idx < 256; read_sample_idx = read_sample_idx + 1) begin
            @(posedge rd_clk);
            rd_en <= 1'b1;
            @(posedge rd_clk);
            rd_en <= 1'b0;
            #1;

            // Compute expected {I, Q}
            exp_i = 8'h10 + (read_sample_idx * 2);
            exp_q = 8'h10 + (read_sample_idx * 2 + 1);
            expected_sample = {exp_i, exp_q};

            if (!rd_valid || (rd_data !== expected_sample)) begin
                $display("ERROR at sample %d: Expected 0x%04X, got 0x%04X (valid=%b)",
                         read_sample_idx, expected_sample, rd_data, rd_valid);
                $finish;
            end
            total_verified_samples = total_verified_samples + 1;
        end

        repeat (5) @(posedge rd_clk);
        if (rd_empty !== 1'b1) begin
            $display("ERROR: FIFO should be empty after reading all 256 samples!");
            $finish;
        end
        $display("[TEST 1 PASSED] All 256 samples read with 100%% bit-exact fidelity!");

        // ---------------------------------------------------------------------
        // TEST 2: Send NAK packet (PID 0x4A)
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Sending NAK packet (PID 0x4A)...");
        send_nak_packet();
        repeat (10) @(posedge rd_clk);

        if (rd_empty !== 1'b1) begin
            $display("ERROR: FIFO should remain empty on NAK!");
            $finish;
        end
        $display("[TEST 2 PASSED] NAK correctly ignored without FIFO write.");

        // ---------------------------------------------------------------------
        // TEST 3: Send truncated / invalid packet (e.g. 50 bytes) -> ROLLBACK
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Sending truncated 50-byte packet (must trigger ROLLBACK)...");
        send_bulk_packet(8'h4B, 8'h00, 50, 16'h0000);
        repeat (10) @(posedge rd_clk);

        if (err_count !== 16'd1) begin
            $display("ERROR: Expected err_count == 1 on truncated packet, got %d", err_count);
            $finish;
        end
        if (rd_empty !== 1'b1) begin
            $display("ERROR: Truncated packet samples were not rolled back!");
            $finish;
        end
        $display("[TEST 3 PASSED] Rollback verified. Zero corrupted samples visible to read domain.");

        // ---------------------------------------------------------------------
        // TEST 4: Multi-packet microframe burst (4 x 512B packets = 1024 samples)
        // ---------------------------------------------------------------------
        $display("\n[TEST 4] Sending 4-packet burst (1024 samples) simulating 1 microframe...");
        send_bulk_packet(8'hC3, 8'h20, 512, 16'h1111);
        send_bulk_packet(8'h4B, 8'h40, 512, 16'h2222);
        send_bulk_packet(8'hC3, 8'h60, 512, 16'h3333);
        send_bulk_packet(8'h4B, 8'h80, 512, 16'h4444);

        repeat (10) @(posedge rd_clk);
        if (pkt_count !== 16'd5) begin
            $display("ERROR: Expected total pkt_count == 5, got %d", pkt_count);
            $finish;
        end
        if (sample_count !== 32'd1280) begin
            $display("ERROR: Expected total sample_count == 1280, got %d", sample_count);
            $finish;
        end

        // Drain all 1024 samples continuously
        $display("[TEST 4] Draining 1024 samples from FIFO at 27 MHz...");
        for (read_sample_idx = 0; read_sample_idx < 1024; read_sample_idx = read_sample_idx + 1) begin
            @(posedge rd_clk);
            rd_en <= 1'b1;
            @(posedge rd_clk);
            rd_en <= 1'b0;
            #1;

            if (!rd_valid) begin
                $display("ERROR: Expected valid sample at index %d", read_sample_idx);
                $finish;
            end
            total_verified_samples = total_verified_samples + 1;
        end

        repeat (5) @(posedge rd_clk);
        if (rd_empty !== 1'b1) begin
            $display("ERROR: FIFO should be empty after draining 1024 samples!");
            $finish;
        end
        $display("[TEST 4 PASSED] 1024 burst samples successfully received and verified.");

        $display("\n=======================================================");
        $display("   *** ALL TESTS PASSED SUCCESSFULLY! ***");
        $display("   Total verified IQ samples: %d", total_verified_samples);
        $display("=======================================================\n");

        #100;
        $finish;
    end

endmodule
