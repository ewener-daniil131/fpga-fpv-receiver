// =============================================================================
// tb_sync_dpll.v
// Drives ntsc_sync_dpll from a file of baseband video samples and prints the
// same numbers tools/sync_model.py prints, so the Verilog and the reference
// model can be compared directly.
//
//   iverilog -o /tmp/tb -DVIDFILE='"/tmp/vid.hex"' -DNSAMP=332992 \
//            sim/tb_sync_dpll.v src/ntsc_sync_dpll.v && vvp /tmp/tb
//
// The file is one 8-bit video level per line in hex, which is what
// tools/ntsc_gen.py --video-out and tools/vid2hex.py produce.
// =============================================================================

`timescale 1ns / 1ps

`ifndef VIDFILE
 `define VIDFILE "/tmp/vid.hex"
`endif
`ifndef NSAMP
 `define NSAMP 332992
`endif
`ifndef MAXSAMP
 `define MAXSAMP 1200000
`endif

module tb_sync_dpll;
    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    integer i;                   // declared before use in the gap generator
    reg [7:0] vid [0:`MAXSAMP-1];
    reg [7:0] video_in;
    reg       video_valid;

    wire hsync, line_advance, vsync, locked, fld, pvalid;
    wire [9:0] lcnt, px;
    wire [8:0] py;
    wire [7:0] pval;
    wire [7:0] dslice, dtip, dblank;
    wire [19:0] dperiod;

    // -DGAPEVERY=n pulses stream_gap every n samples, which is how the USB
    // transport behaves: a block arrives, then a chunk of real time is missing.
    reg gap;
`ifdef GAPEVERY
    always @(posedge clk) gap <= rst_n && video_valid && (i % `GAPEVERY == 0) && (i != 0);
`else
    always @(posedge clk) gap <= 1'b0;
`endif

    // -DOLD runs the same measurement against the module this one replaces, so
    // the two can be compared on identical input rather than by argument.
`ifdef OLD
    ntsc_sync_separator u_sep (
        .clk(clk), .rst_n(rst_n),
        .video_in(video_in), .video_valid(video_valid),
        .hsync(hsync), .line_advance(line_advance), .vsync(vsync),
        .ntsc_locked(locked), .field(fld),
        .line_cnt(lcnt), .pixel_x(px), .pixel_y(py),
        .pixel_val(pval), .pixel_valid(pvalid)
    );
    assign dslice = 8'd0, dtip = 8'd0, dblank = 8'd0, dperiod = 20'd0;
`else
    ntsc_sync_dpll u_sep (
        .clk(clk), .rst_n(rst_n),
        .video_in(video_in), .video_valid(video_valid),
        .stream_gap(gap),
        .hsync(hsync), .line_advance(line_advance), .vsync(vsync),
        .ntsc_locked(locked), .field(fld),
        .line_cnt(lcnt), .pixel_x(px), .pixel_y(py),
        .pixel_val(pval), .pixel_valid(pvalid),
        .dbg_slice(dslice), .dbg_tip(dtip), .dbg_blank(dblank), .dbg_period(dperiod)
    );
`endif

    integer n_hs = 0, n_vs = 0, n_pix = 0, n_adv = 0;
    integer lock_at = -1, unlock_n = 0;
    integer last_hs = -1, d, nd = 0;
    integer sum_d = 0, sum_d2 = 0, first_d = 1;
    reg was_locked = 0;

    always @(posedge clk) begin
        if (rst_n && video_valid) begin
            if (hsync) begin
                n_hs = n_hs + 1;
                if (last_hs >= 0) begin
                    d = i - last_hs;
                    // skip the first interval: the slicer is still settling
                    if (first_d) first_d = 0;
                    else if (d > 600 && d < 2000) begin
                        nd     = nd + 1;
                        sum_d  = sum_d + d;
                        sum_d2 = sum_d2 + d*d;
                    end
                end
                last_hs = i;
            end
            if (vsync)        n_vs  = n_vs + 1;
            if (line_advance) n_adv = n_adv + 1;
            if (pvalid)       n_pix = n_pix + 1;
            if (locked && lock_at < 0) lock_at = i;
            if (was_locked && !locked) unlock_n = unlock_n + 1;
            was_locked <= locked;
        end
    end

    real mean_d, var_d;

    initial begin
        for (i = 0; i < `MAXSAMP; i = i + 1) vid[i] = 8'd128;
        $readmemh(`VIDFILE, vid);
        video_in = 8'd128; video_valid = 1'b0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (4) @(posedge clk);

        for (i = 0; i < `NSAMP; i = i + 1) begin
            @(posedge clk);
            video_in    <= vid[i];
            video_valid <= 1'b1;
        end
        @(posedge clk);
        video_valid <= 1'b0;
        repeat (4) @(posedge clk);

        mean_d = (nd > 0) ? (sum_d * 1.0 / nd) : 0.0;
        var_d  = (nd > 0) ? (sum_d2 * 1.0 / nd - mean_d*mean_d) : 0.0;
        if (var_d < 0.0) var_d = 0.0;

        $display("RESULT samples=%0d hsync=%0d vsync=%0d line_adv=%0d pixels=%0d",
                 `NSAMP, n_hs, n_vs, n_adv, n_pix);
        $display("       lock_at=%0d unlocks=%0d locked_at_end=%0d",
                 lock_at, unlock_n, locked);
        $display("       tip=%0d blank=%0d slice=%0d period=%0d (%0d.%03d samples)",
                 dtip, dblank, dslice, dperiod, dperiod/256,
                 ((dperiod % 256) * 1000) / 256);
        $display("       line period mean=%0d.%02d jitter=%0d.%02d over %0d intervals",
                 $rtoi(mean_d), $rtoi((mean_d - $rtoi(mean_d))*100.0),
                 $rtoi($sqrt(var_d)), $rtoi(($sqrt(var_d) - $rtoi($sqrt(var_d)))*100.0),
                 nd);
        $finish;
    end
endmodule
