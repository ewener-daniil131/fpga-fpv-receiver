`timescale 1ns/1ps
module tb_chk;
    reg clk=0, rst_n=0; always #5 clk=~clk;
    reg [15:0] mem [0:2097151];
    reg [15:0] sample_in; reg sample_valid;
    integer i, nh=0, nv=0, npix=0, first_lock=-1, lost=-1;
    wire [7:0] video; wire vvalid; wire [15:0] rssi; wire signed [15:0] dth;
    fm_demod u_d(.clk(clk),.rst_n(rst_n),.sample_in(sample_in),.sample_valid(sample_valid),
                 .video_out(video),.video_valid(vvalid),.raw_dtheta(dth),.rssi(rssi));
    wire hsync,vsync,locked,field,pvalid;
    wire [9:0] lcnt,px; wire [8:0] py; wire [7:0] pval;
    ntsc_sync_separator u_s(.clk(clk),.rst_n(rst_n),.video_in(video),.video_valid(vvalid),
        .hsync(hsync),.vsync(vsync),.ntsc_locked(locked),.field(field),
        .line_cnt(lcnt),.pixel_x(px),.pixel_y(py),.pixel_val(pval),.pixel_valid(pvalid));
    always @(posedge clk) begin
        if (hsync) nh=nh+1;
        if (vsync) nv=nv+1;
        if (pvalid) npix=npix+1;
    end
    initial begin
        for (i=0;i<2097152;i=i+1) mem[i]=16'h0000;
        $readmemh("sim/data/iq_synth.hex", mem);
        sample_in=0; sample_valid=0;
        repeat(4) @(posedge clk); rst_n=1; repeat(4) @(posedge clk);
        for (i=0;i<1399093;i=i+1) begin
            @(posedge clk); sample_in<=mem[i]; sample_valid<=1'b1;
            if (locked && first_lock<0) first_lock=i;
            if (first_lock>=0 && !locked && lost<0 && i>first_lock+1000) lost=i;
        end
        $display("RESULT hsync=%0d vsync=%0d pixels=%0d lock_at=%.2fms lost_at=%.2fms final_lk=%0d",
                 nh, nv, npix, first_lock/20000.0, lost/20000.0, locked);
        $display("       expect vsync=3 (3 fields), pixels ~ 3*240*525 = 378000");
        $finish;
    end
endmodule
