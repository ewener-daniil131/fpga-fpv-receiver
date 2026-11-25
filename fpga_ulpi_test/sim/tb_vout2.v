`timescale 1ns/1ps
// Feeds the display path in BURSTS, the way USB delivers IQ, and checks that
// each output raster line is one intact source line (a monotonic ramp).
module tb_vout2;
    reg cap_clk=0; always #18.518 cap_clk=~cap_clk;   // 27 MHz
    reg pix_clk=0; always #19.841 pix_clk=~pix_clk;   // 25.2 MHz
    reg rst_n=0;
    reg [9:0] cap_x; reg [8:0] cap_y; reg [7:0] cap_val;
    reg cap_valid, cap_hsync, cap_vsync, cap_locked;
    wire [7:0] r,g,b; wire de,hs,vs;

    fpv_video_out u_out(
        .cap_clk(cap_clk), .cap_rst_n(rst_n),
        .cap_x(cap_x), .cap_y(cap_y), .cap_val(cap_val), .cap_valid(cap_valid),
        .cap_hsync(cap_hsync), .cap_vsync(cap_vsync), .cap_locked(cap_locked),
        .pix_clk(pix_clk), .pix_rst_n(rst_n),
        .o_r(r), .o_g(g), .o_b(b), .o_de(de), .o_hs(hs), .o_vs(vs));

    integer li, xi, fi, gap;
    integer seed = 3;
    initial begin
        cap_x=0; cap_y=0; cap_val=0; cap_valid=0; cap_hsync=0; cap_vsync=0; cap_locked=0;
        @(posedge cap_clk); #200 rst_n=1; cap_locked=1;
        for (fi=0; fi<6; fi=fi+1) begin
            @(posedge cap_clk); cap_vsync<=1'b1;
            @(posedge cap_clk); cap_vsync<=1'b0;
            for (li=0; li<240; li=li+1) begin
                xi = 0;
                while (xi < 525) begin
                    // burst of up to 256 pixels, then an idle gap - USB-like
                    gap = ($random(seed) % 40); if (gap<0) gap=-gap;
                    repeat (gap) @(posedge cap_clk);
                    repeat (200) begin
                        if (xi < 525) begin
                            @(posedge cap_clk);
                            cap_x<=xi[9:0]; cap_y<=li[8:0];
                            cap_val<=(xi*255)/525; cap_valid<=1'b1;
                            xi=xi+1;
                        end
                    end
                    @(posedge cap_clk); cap_valid<=1'b0;
                end
                @(posedge cap_clk); cap_hsync<=1'b1;
                @(posedge cap_clk); cap_hsync<=1'b0;
                repeat (300) @(posedge cap_clk);
            end
            repeat (22*1715) @(posedge cap_clk);
        end
    end

    // check output lines
    integer bad=0, good=0, nline=0, frames=0;
    reg [7:0] prevpix; integer inline=0; integer viol=0;
    reg de_q=0, vs_q=0;
    always @(posedge pix_clk) begin
        de_q<=de; vs_q<=vs;
        if (vs && !vs_q) frames<=frames+1;
        if (de) begin
            if (!de_q) begin inline<=1; viol<=0; end
            else begin
                // a source line is a ramp: the value must never fall
                if (r + 2 < prevpix) viol<=viol+1;
            end
            prevpix<=r;
        end
        if (de_q && !de && frames>3) begin
            nline<=nline+1;
            if (viol>0) bad<=bad+1; else good<=good+1;
        end
    end
    initial begin
        #90_000_000;
        $display("RESULT frames=%0d lines_checked=%0d intact=%0d torn=%0d (%.1f%% intact)",
                 frames, nline, good, bad, 100.0*good/(nline>0?nline:1));
        $finish;
    end
endmodule
