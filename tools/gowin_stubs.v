`timescale 1ns/1ps
module rPLL (output reg CLKOUT, output reg LOCK, output CLKOUTP, output CLKOUTD,
             output CLKOUTD3, input CLKIN, input CLKFB, input RESET, input RESET_P,
             input [5:0] FBDSEL, input [5:0] IDSEL, input [5:0] ODSEL,
             input [3:0] PSDA, input [3:0] DUTYDA, input [3:0] FDLY);
    parameter FCLKIN="27"; parameter DEVICE="GW2A-18C";
    parameter DYN_IDIV_SEL="false"; parameter IDIV_SEL=0;
    parameter DYN_FBDIV_SEL="false"; parameter FBDIV_SEL=0;
    parameter DYN_ODIV_SEL="false"; parameter ODIV_SEL=8;
    parameter PSDA_SEL="0000"; parameter DYN_DA_EN="false"; parameter DUTYDA_SEL="1000";
    parameter CLKOUT_FT_DIR=1'b1; parameter CLKOUTP_FT_DIR=1'b1;
    parameter CLKOUT_DLY_STEP=0; parameter CLKOUTP_DLY_STEP=0;
    parameter CLKFB_SEL="internal"; parameter CLKOUT_BYPASS="false";
    parameter CLKOUTP_BYPASS="false"; parameter CLKOUTD_BYPASS="false";
    parameter DYN_SDIV_SEL=2; parameter CLKOUTD_SRC="CLKOUT";
    parameter CLKOUTD3_SRC="CLKOUT"; parameter CLKOUT_DLY=0;
    assign CLKOUTP=1'b0; assign CLKOUTD=1'b0; assign CLKOUTD3=1'b0;
    initial begin CLKOUT=0; LOCK=0; #200 LOCK=1; end
    always #3.968 CLKOUT = ~CLKOUT;
endmodule
module CLKDIV (output reg CLKOUT, input HCLKIN, input RESETN, input CALIB);
    parameter DIV_MODE="5"; parameter GSREN="false";
    integer n=0; initial CLKOUT=0;
    always @(posedge HCLKIN or negedge RESETN) begin
        if (!RESETN) begin n<=0; CLKOUT<=0; end
        else begin if (n==4) begin n<=0; CLKOUT<=~CLKOUT; end else n<=n+1; end
    end
endmodule
module GSR (input GSRI); endmodule
module DVI_TX_Top (input I_rst_n, input I_serial_clk, input I_rgb_clk,
                   input I_rgb_vs, input I_rgb_hs, input I_rgb_de,
                   input [7:0] I_rgb_r, input [7:0] I_rgb_g, input [7:0] I_rgb_b,
                   output O_tmds_clk_p, output O_tmds_clk_n,
                   output [2:0] O_tmds_data_p, output [2:0] O_tmds_data_n);
    assign O_tmds_clk_p=1'b0; assign O_tmds_clk_n=1'b1;
    assign O_tmds_data_p=3'b0; assign O_tmds_data_n=3'b111;
endmodule
