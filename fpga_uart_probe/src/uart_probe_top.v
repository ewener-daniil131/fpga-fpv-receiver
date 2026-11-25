// UART connectivity probe for Tang Primer 20K + Dock.
//
// Purpose: find out whether FPGA pin M11 (FPGA_TXD per the Dock schematic)
// actually reaches a serial port on the host, and at what baud rate.
// Sends a fixed ASCII string forever. Any bytes arriving on a host tty
// prove the wire exists; garbage instead of the string means the wire is
// fine but the baud rate assumption is wrong.
//
// Clock: H11, 27MHz (same pin the working HDMI example uses).
// LEDs blink slowly so it is obvious the design is running.

module uart_probe_top #(
    parameter integer CLK_FRE   = 27,     // MHz on H11
    parameter integer BAUD_RATE = 115200
)(
    input  wire       clk,       // H11
    input  wire       rst_n,     // T10
    output wire       uart_tx,   // M11 -> debug chip UART RX
    output wire [3:0] led
);

    reg [5:0] por = 6'd0;
    wire internal_rst_n = (por == 6'd63);
    always @(posedge clk) begin
        if (!internal_rst_n) por <= por + 6'd1;
    end

    // "T10=x ULPI\r\n" = 12 bytes
    localparam integer DATA_NUM = 12;
    reg rst_n_s;
    always @(posedge clk) rst_n_s <= rst_n;

    wire [DATA_NUM*8-1:0] msg = {
        8'h54, 8'h31, 8'h30, 8'h3D,  // T 1 0 =
        (rst_n_s ? 8'h31 : 8'h30),   // '1' or '0'
        8'h20,                       // ' '
        8'h55, 8'h4C, 8'h50, 8'h49,  // U L P I
        8'h0D, 8'h0A                 // CR LF
    };

    reg  [3:0]  idx;
    reg  [21:0] gap_cnt;
    reg         sending;
    reg  [7:0]  tx_data;
    reg         tx_valid;
    wire        tx_ready;

    wire [7:0] cur_byte = msg[(DATA_NUM-1-idx)*8 +: 8];

    localparam [21:0] GAP_MAX = 22'd2_700_000; // ~100ms @ 27MHz

    always @(posedge clk or negedge internal_rst_n) begin
        if (!internal_rst_n) begin
            idx      <= 4'd0;
            gap_cnt  <= 22'd0;
            sending  <= 1'b0;
            tx_data  <= 8'd0;
            tx_valid <= 1'b0;
        end else begin
            tx_valid <= 1'b0;

            if (!sending) begin
                if (gap_cnt >= GAP_MAX) begin
                    gap_cnt <= 22'd0;
                    idx     <= 4'd0;
                    sending <= 1'b1;
                end else begin
                    gap_cnt <= gap_cnt + 22'd1;
                end
            end else if (tx_ready && !tx_valid) begin
                tx_data  <= cur_byte;
                tx_valid <= 1'b1;
                if (idx == DATA_NUM-1) begin
                    sending <= 1'b0;
                    idx     <= 4'd0;
                end else begin
                    idx <= idx + 4'd1;
                end
            end
        end
    end

    uart_tx #(
        .CLK_FRE   (CLK_FRE),
        .BAUD_RATE (BAUD_RATE)
    ) u_tx (
        .clk           (clk),
        .rst_n         (internal_rst_n),
        .tx_data       (tx_data),
        .tx_data_valid (tx_valid),
        .tx_data_ready (tx_ready),
        .tx_pin        (uart_tx),
        .tx_busy       ()
    );

    // slow heartbeat so "design is alive" is visible
    reg [24:0] hb;
    always @(posedge clk or negedge internal_rst_n) begin
        if (!internal_rst_n) hb <= 25'd0;
        else                 hb <= hb + 25'd1;
    end
    assign led = {4{hb[24]}};

endmodule
