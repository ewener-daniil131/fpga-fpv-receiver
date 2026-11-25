// ULPI data-bus integrity test - Tang Primer 20K Dock.
//
// Holds the USB3317 in hardware reset (RESETB low), which per the ULPI POR
// timing makes the PHY tri-state DATA. The FPGA then drives a walking-ones /
// all-zeros / all-ones pattern set onto ulpi_data and reads each one back.
// Any bit that cannot be driven to both levels is a stuck line.
//
// Everything runs on the board's 27MHz oscillator: with the PHY in reset it
// stops CLKOUT, so nothing may depend on ulpi_clk here.
//
// Output @115200 on M11: "P=xx R=xx OK" / "P=xx R=xx BAD" per pattern.

module bustest_top (
    input  wire       clk27,
    input  wire       rst_n,
    output wire       ulpi_rst,
    output wire       ulpi_stp,
    inout  wire [7:0] ulpi_data,
    output wire       uart_tx,
    output wire [3:0] led
);
    assign ulpi_rst = 1'b0;   // keep the PHY in reset for the whole test
    assign ulpi_stp = 1'b0;

    reg [1:0] rs;
    wire rst_nc = rs[1];
    always @(posedge clk27 or negedge rst_n) begin
        if (!rst_n) rs <= 2'b00; else rs <= {rs[0], 1'b1};
    end

    reg  [7:0] pat;
    reg        oe;
    assign ulpi_data = oe ? pat : 8'hzz;
    wire [7:0] rback = ulpi_data;

    reg [3:0]  pidx;
    reg [15:0] dly;
    reg [7:0]  rb_latched;
    reg        do_line;
    reg [2:0]  st;

    function [7:0] pattern(input [3:0] i);
        case (i)
            4'd0: pattern = 8'h00;
            4'd1: pattern = 8'hFF;
            4'd2: pattern = 8'h01;
            4'd3: pattern = 8'h02;
            4'd4: pattern = 8'h04;
            4'd5: pattern = 8'h08;
            4'd6: pattern = 8'h10;
            4'd7: pattern = 8'h20;
            4'd8: pattern = 8'h40;
            4'd9: pattern = 8'h80;
            4'd10: pattern = 8'hAA;
            default: pattern = 8'h55;
        endcase
    endfunction

    localparam T_DRIVE = 3'd0, T_SAMPLE = 3'd1, T_PRINT = 3'd2, T_WAIT = 3'd3;

    always @(posedge clk27 or negedge rst_nc) begin
        if (!rst_nc) begin
            pidx <= 4'd0; dly <= 16'd0; oe <= 1'b0; pat <= 8'h00;
            rb_latched <= 8'h00; do_line <= 1'b0; st <= T_DRIVE;
        end else begin
            do_line <= 1'b0;
            case (st)
                T_DRIVE: begin
                    pat <= pattern(pidx);
                    oe  <= 1'b1;
                    dly <= 16'd0;
                    st  <= T_SAMPLE;
                end
                T_SAMPLE: begin
                    if (dly == 16'd2700) begin      // ~100us settle
                        rb_latched <= rback;
                        st         <= T_PRINT;
                    end else dly <= dly + 16'd1;
                end
                T_PRINT: begin
                    do_line <= 1'b1;
                    dly     <= 16'd0;
                    st      <= T_WAIT;
                end
                T_WAIT: begin
                    if (dly == 16'd60000) begin
                        pidx <= (pidx == 4'd11) ? 4'd0 : pidx + 4'd1;
                        st   <= T_DRIVE;
                    end else dly <= dly + 16'd1;
                end
                default: st <= T_DRIVE;
            endcase
        end
    end

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h41 + {4'd0, n} - 8'd10);
    endfunction

    localparam LN = 16;
    reg [7:0] lb [0:LN-1];
    reg [4:0] li;
    reg       printing;
    reg [7:0] txd;
    reg       txv;
    wire      txr;
    wire      match = (rb_latched == pat);

    always @(posedge clk27 or negedge rst_nc) begin
        if (!rst_nc) begin
            printing <= 1'b0; li <= 5'd0; txv <= 1'b0; txd <= 8'd0;
        end else begin
            txv <= 1'b0;
            if (!printing) begin
                if (do_line) begin
                    printing <= 1'b1; li <= 5'd0;
                    lb[0] <= "P"; lb[1] <= "=";
                    lb[2] <= hexc(pat[7:4]);   lb[3] <= hexc(pat[3:0]);
                    lb[4] <= " ";
                    lb[5] <= "R"; lb[6] <= "=";
                    lb[7] <= hexc(rb_latched[7:4]); lb[8] <= hexc(rb_latched[3:0]);
                    lb[9] <= " ";
                    lb[10] <= match ? "O" : "B";
                    lb[11] <= match ? "K" : "A";
                    lb[12] <= match ? " " : "D";
                    lb[13] <= " ";
                    lb[14] <= 8'h0D; lb[15] <= 8'h0A;
                end
            end else if (txr && !txv) begin
                txd <= lb[li];
                txv <= 1'b1;
                if (li == LN-1) printing <= 1'b0; else li <= li + 5'd1;
            end
        end
    end

    uart_tx #(.CLK_FRE(27), .BAUD_RATE(115200)) u_tx (
        .clk(clk27), .rst_n(rst_nc), .tx_data(txd), .tx_data_valid(txv),
        .tx_data_ready(txr), .tx_pin(uart_tx), .tx_busy());

    reg [24:0] hb;
    always @(posedge clk27 or negedge rst_nc) begin
        if (!rst_nc) hb <= 25'd0; else hb <= hb + 25'd1;
    end
    assign led = {4{hb[24]}};
endmodule
