// Minimal liveness test: is the FPGA configured, is clk27 running, does the
// UART pin work? No USB, no PLL, and deliberately NOT gated on the reset
// button (T10) - a stuck button would hold the real design in reset forever.
`timescale 1ns/1ps
module hello_top (
    input  wire clk27,
    output wire uart_tx,
    output wire [3:0] led
);
    // internal power-on reset only
    reg [7:0] por = 8'd0;
    wire rst_n = (por == 8'd255);
    always @(posedge clk27) if (!rst_n) por <= por + 8'd1;

    reg [24:0] hb = 25'd0;
    always @(posedge clk27) hb <= hb + 25'd1;
    assign led = {~hb[24], ~hb[23], ~hb[22], ~hb[21]};

    // print "HELLO nnnn\r\n" about twice a second
    reg [23:0] tick = 24'd0;
    wire do_print = (tick == 24'd13_500_000);
    always @(posedge clk27 or negedge rst_n) begin
        if (!rst_n) tick <= 24'd0;
        else tick <= do_print ? 24'd0 : tick + 24'd1;
    end

    reg [15:0] cnt = 16'd0;
    localparam LEN = 13;
    reg [7:0] buf_ [0:LEN-1];
    reg [4:0] idx; reg busy, pending;
    reg [7:0] txd; reg txv; wire txr;

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? (8'h30 + {4'd0,n}) : (8'h41 + {4'd0,n} - 8'd10);
    endfunction

    always @(posedge clk27 or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0; pending <= 1'b0; idx <= 5'd0; txv <= 1'b0; cnt <= 16'd0;
        end else begin
            txv <= 1'b0;
            if (do_print) pending <= 1'b1;
            if (!busy) begin
                if (pending) begin
                    pending <= 1'b0; busy <= 1'b1; idx <= 5'd0; cnt <= cnt + 16'd1;
                    buf_[0] <= "H"; buf_[1] <= "E"; buf_[2] <= "L"; buf_[3] <= "L";
                    buf_[4] <= "O"; buf_[5] <= " ";
                    buf_[6] <= hexc(cnt[15:12]); buf_[7] <= hexc(cnt[11:8]);
                    buf_[8] <= hexc(cnt[7:4]);   buf_[9] <= hexc(cnt[3:0]);
                    buf_[10] <= 8'h0D; buf_[11] <= 8'h0A; buf_[12] <= 8'h00;
                end
            end else if (txr && !txv) begin
                txd <= buf_[idx]; txv <= 1'b1;
                if (idx == LEN-2) busy <= 1'b0; else idx <= idx + 5'd1;
            end
        end
    end

    uart_tx #(.CLK_FRE(27), .BAUD_RATE(115200)) u_tx (
        .clk(clk27), .rst_n(rst_n), .tx_data(txd), .tx_data_valid(txv),
        .tx_data_ready(txr), .tx_pin(uart_tx), .tx_busy());
endmodule
