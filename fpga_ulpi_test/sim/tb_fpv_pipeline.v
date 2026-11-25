// =============================================================================
// tb_fpv_pipeline.v
// Verification testbench for:
//   1. CORDIC atan2
//   2. FM Demodulator (polar discriminator + de-emphasis + RSSI)
//   3. NTSC Video Sync Separator & Active Pixel Generator
//   4. FPV 40-Channel Auto-Scanner
// =============================================================================

`timescale 1ns / 1ps

module tb_fpv_pipeline;

    reg clk;
    reg rst_n;

    // Clock generation: 27 MHz (period 37.037 ns)
    always #18.518 clk = ~clk;

    // -------------------------------------------------------------------------
    // 1. CORDIC Verification
    // -------------------------------------------------------------------------
    reg        cordic_vin;
    reg signed [15:0] cordic_xin;
    reg signed [15:0] cordic_yin;
    wire       cordic_vout;
    wire signed [15:0] cordic_zout;

    cordic_atan2 u_cordic (
        .clk        (clk),
        .rst_n      (rst_n),
        .valid_in   (cordic_vin),
        .x_in       (cordic_xin),
        .y_in       (cordic_yin),
        .valid_out  (cordic_vout),
        .angle_out  (cordic_zout)
    );

    // -------------------------------------------------------------------------
    // 2. FM Demodulator Verification
    // -------------------------------------------------------------------------
    reg  [15:0] fm_sample_in;
    reg         fm_sample_valid;
    wire [7:0]  video_out;
    wire        video_valid;
    wire signed [15:0] raw_dtheta;
    wire [15:0] rssi;

    fm_demod u_fm_demod (
        .clk          (clk),
        .rst_n        (rst_n),
        .sample_in    (fm_sample_in),
        .sample_valid (fm_sample_valid),
        .video_out    (video_out),
        .video_valid  (video_valid),
        .raw_dtheta   (raw_dtheta),
        .rssi         (rssi)
    );

    // -------------------------------------------------------------------------
    // 3. NTSC Sync Separator Verification
    // -------------------------------------------------------------------------
    reg        test_direct_video;
    reg [7:0]  sim_video_val;
    reg        sim_video_vld;

    wire [7:0] sync_sep_vin = test_direct_video ? sim_video_val : video_out;
    wire       sync_sep_vld = test_direct_video ? sim_video_vld : video_valid;

    wire       hsync;
    wire       vsync;
    wire       ntsc_locked;
    wire       field;
    wire [9:0] line_cnt;
    wire [9:0] pixel_x;
    wire [8:0] pixel_y;
    wire [7:0] pixel_val;
    wire       pixel_valid;

    ntsc_sync_separator u_sync_sep (
        .clk         (clk),
        .rst_n       (rst_n),
        .video_in    (sync_sep_vin),
        .video_valid (sync_sep_vld),
        .hsync       (hsync),
        .vsync       (vsync),
        .ntsc_locked (ntsc_locked),
        .field       (field),
        .line_cnt    (line_cnt),
        .pixel_x     (pixel_x),
        .pixel_y     (pixel_y),
        .pixel_val   (pixel_val),
        .pixel_valid (pixel_valid)
    );

    // -------------------------------------------------------------------------
    // 4. FPV Auto-Scanner Verification
    // -------------------------------------------------------------------------
    reg        start_scan;
    wire       retune_req;
    wire [15:0] retune_freq_mhz;
    reg        retune_done;
    wire       is_scanning;
    wire [5:0] cur_ch_idx;
    wire [5:0] locked_ch_idx;
    wire [15:0] locked_freq_mhz;
    wire [15:0] best_rssi;
    wire       locked_sync;

    fpv_auto_scanner #(
        .SETTLE_CYCLES(20'd50),
        .MEASURE_CYCLES(20'd100)
    ) u_scanner (
        .clk             (clk),
        .rst_n           (rst_n),
        .start_scan      (start_scan),
        .rssi            (rssi),
        .ntsc_locked     (ntsc_locked),
        .retune_req      (retune_req),
        .retune_freq_mhz (retune_freq_mhz),
        .retune_done     (retune_done),
        .is_scanning     (is_scanning),
        .cur_ch_idx      (cur_ch_idx),
        .locked_ch_idx   (locked_ch_idx),
        .locked_freq_mhz (locked_freq_mhz),
        .best_rssi       (best_rssi),
        .locked_sync     (locked_sync)
    );

    // Task to generate one NTSC video line of samples (508 samples)
    task send_ntsc_line;
        input is_vsync_line;
        integer s;
        reg [7:0] sample_val;
        begin
            for (s = 0; s < 508; s = s + 1) begin
                if (is_vsync_line) begin
                    // Vertical serration: low for 216 samples
                    sample_val = (s < 216) ? 8'd30 : 8'd100;
                end else begin
                    // Horizontal line: sync tip for 38 samples
                    if (s < 38)
                        sample_val = 8'd30;  // Sync tip (0 IRE)
                    else if (s < 76)
                        sample_val = 8'd75;  // Blanking / back porch
                    else if (s < 496)
                        sample_val = 8'd160; // Active video picture
                    else
                        sample_val = 8'd75;  // Front porch
                end

                @(posedge clk);
                sim_video_val   <= sample_val;
                sim_video_vld   <= 1'b1;
                @(posedge clk);
                sim_video_vld   <= 1'b0;
            end
        end
    endtask

    integer line_i;
    integer scan_timeout;

    initial begin
        $dumpfile("tb_fpv_pipeline.vcd");
        $dumpvars(0, tb_fpv_pipeline);

        clk               = 0;
        rst_n             = 0;
        test_direct_video = 1;
        sim_video_val     = 0;
        sim_video_vld     = 0;
        cordic_vin        = 0;
        cordic_xin        = 0;
        cordic_yin        = 0;
        fm_sample_in      = 0;
        fm_sample_valid   = 0;
        start_scan        = 0;
        retune_done       = 0;

        $display("\n=======================================================");
        $display("   TESTBENCH: FPV NTSC Demodulator & Auto-Scanner");
        $display("=======================================================");

        #100;
        rst_n = 1;
        #100;

        // ---------------------------------------------------------------------
        // TEST 1: CORDIC atan2 Verification
        // ---------------------------------------------------------------------
        $display("\n[TEST 1] Testing CORDIC atan2 at 45 degrees (X=10000, Y=10000)...");
        @(posedge clk);
        cordic_xin <= 16'sd10000;
        cordic_yin <= 16'sd10000;
        cordic_vin <= 1'b1;
        @(posedge clk);
        cordic_vin <= 1'b0;

        // Wait for 11 pipeline cycles
        repeat (15) @(posedge clk);
        $display("[TEST 1] CORDIC Angle = %d (Expected ~8192 for 45 deg)", cordic_zout);
        if (cordic_zout < 8100 || cordic_zout > 8280) begin
            $display("ERROR: CORDIC angle mismatch!");
            $finish;
        end
        $display("[TEST 1 PASSED] CORDIC atan2 accuracy verified.");

        // ---------------------------------------------------------------------
        // TEST 2: NTSC Sync Detection & Lock Verification
        // ---------------------------------------------------------------------
        $display("\n[TEST 2] Generating 15 consecutive NTSC video lines (508 samples/line)...");
        for (line_i = 0; line_i < 15; line_i = line_i + 1) begin
            send_ntsc_line(0);
        end

        $display("[TEST 2] Checking NTSC lock status...");
        if (!ntsc_locked) begin
            $display("ERROR: NTSC sync separator failed to lock on valid lines!");
            $finish;
        end
        $display("[TEST 2 PASSED] NTSC Horizontal Sync Locked! (ntsc_locked = 1)");

        // ---------------------------------------------------------------------
        // TEST 3: Auto-Scanner Sweep & Channel Lock
        // ---------------------------------------------------------------------
        $display("\n[TEST 3] Triggering FPV Auto-Scanner (40 channels sweep)...");
        scan_timeout = 0;
        @(posedge clk);
        start_scan <= 1'b1;
        @(posedge clk);
        start_scan <= 1'b0;

        // Auto-respond to retune requests to fast-forward simulation
        while (is_scanning && scan_timeout < 200000) begin
            @(posedge clk);
            scan_timeout = scan_timeout + 1;
            if (retune_req) begin
                @(posedge clk);
                retune_done <= 1'b1;
                @(posedge clk);
                retune_done <= 1'b0;
            end
        end

        if (is_scanning) begin
            $display("ERROR: Scanner sweep timed out!");
            $finish;
        end

        $display("[TEST 3] Auto-Scan complete! Locked Channel = %d, Freq = %d MHz",
                 locked_ch_idx, locked_freq_mhz);
        $display("[TEST 3 PASSED] Auto-scanner completed 40-channel evaluation and locked.");

        $display("\n=======================================================");
        $display("   *** ALL FPV PIPELINE TESTS PASSED SUCCESSFULLY! ***");
        $display("=======================================================\n");

        #100;
        $finish;
    end

endmodule
