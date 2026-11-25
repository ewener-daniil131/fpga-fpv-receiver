// Testbench: our USB host against a model of the ULPI PHY and a USB device.
//
// Purpose: watch, cycle by cycle, what the host puts on the bus - in
// particular whether the ACK handshake that must follow a received data
// packet is emitted at all, and how long after the packet it arrives. The
// real device stops completing control transfers if that ACK is missing or
// late, which disables its bulk endpoints entirely.
//
// The model is deliberately simple: it speaks enough ULPI to accept register
// reads/writes and to answer an IN token with a DATA1 packet, then reports
// what the host does next.

`timescale 1ns / 1ps

module tb_ulpi;

    reg clk27 = 0;
    reg rst_n = 0;
    always #18.5 clk27 = ~clk27;        // 27 MHz

    reg ulpi_clk = 0;
    always #8.333 ulpi_clk = ~ulpi_clk; // 60 MHz

    // ---- ULPI bus ----
    wire [7:0] ulpi_data;
    reg  [7:0] phy_do = 8'h00;
    reg        phy_oe = 1'b0;
    assign ulpi_data = phy_oe ? phy_do : 8'hzz;

    reg  dir = 1'b1;    // PHY owns the bus out of reset
    reg  nxt = 1'b0;
    wire stp;
    wire ulpi_rst_n;
    wire uart_tx;
    wire [3:0] led;

    ulpi_enum_test dut (
        .clk27    (clk27),
        .rst_n    (rst_n),
        .ulpi_clk (ulpi_clk),
        .ulpi_dir (dir),
        .ulpi_nxt (nxt),
        .ulpi_stp (stp),
        .ulpi_rst (ulpi_rst_n),
        .ulpi_data(ulpi_data),
        .uart_tx  (uart_tx),
        .led      (led)
    );

    // ---- what the link is driving, sampled on the PHY's edge ----
    wire [7:0] link_data = ulpi_data;

    integer cyc = 0;
    always @(posedge ulpi_clk) cyc = cyc + 1;

    // ---- PHY / device model ----
    localparam S_POR    = 0, S_IDLE = 1, S_REGW = 2, S_REGR = 3,
               S_RXPKT  = 4, S_SEND = 5, S_WAITACK = 6;
    integer state = S_POR;
    integer por_cnt = 0;
    reg [7:0] cmd = 0;
    reg [7:0] pkt_pid = 0;
    integer   pkt_len = 0;
    integer   send_i = 0;
    integer   ack_wait = 0;
    integer   data_done_cyc = 0;

    // DATA1 packet the model answers an IN token with: PID + 8 bytes + CRC
    reg [7:0] resp [0:10];
    reg [7:0] send_buf [0:15];
    integer   send_n = 0;
    reg [7:0] last_token = 0;
    initial begin
        resp[0]  = 8'h4B;            // DATA1
        resp[1]  = 8'h12; resp[2] = 8'h01; resp[3] = 8'h00; resp[4] = 8'h02;
        resp[5]  = 8'h00; resp[6] = 8'h00; resp[7] = 8'h00; resp[8] = 8'h40;
        resp[9]  = 8'h00; resp[10] = 8'h00;   // CRC placeholder
    end

    integer acks_seen = 0;

    // Scripted device chirp so the host can finish the Hi-Speed handshake:
    // RX CMD with LineState=10 (chirp K), then back to 01.
    integer rxcmd_q = -1;      // byte to deliver, -1 = nothing queued
    integer rxcmd_st = 0;
    integer chirp_t = 0;
    reg     reset_seen = 1'b0;
    integer attach_t = 0;
    reg     attached = 1'b0;
    always @(posedge ulpi_clk) begin
        // device appears on the bus: D+ pulled high -> LineState 01
        if (state != S_POR && !attached) begin
            attach_t <= attach_t + 1;
            if (attach_t == 100) begin
                rxcmd_q  <= 8'h0D;
                attached <= 1'b1;
            end
        end
        // start the chirp only once the host has driven the bus reset,
        // otherwise it arrives before the host is listening for it
        if (state == S_REGW) reset_seen <= 1'b1;
        if (reset_seen) begin
            chirp_t <= chirp_t + 1;
            if (chirp_t == 200) rxcmd_q <= 8'h0E;   // device chirp K
            if (chirp_t == 800) rxcmd_q <= 8'h0D;   // chirp over
        end
    end

    always @(posedge ulpi_clk) begin
        case (state)
            S_POR: begin
                dir <= 1'b1; nxt <= 1'b0; phy_oe <= 1'b0;
                por_cnt <= por_cnt + 1;
                if (por_cnt > 40) begin
                    dir   <= 1'b0;
                    state <= S_IDLE;
                end
            end

            S_IDLE: begin
                nxt    <= 1'b0;
                phy_oe <= 1'b0;
                if (rxcmd_q >= 0) begin
                    // hand the link an RX CMD: dir up, turnaround, then byte
                    case (rxcmd_st)
                        0: begin dir <= 1'b1; phy_oe <= 1'b1; phy_do <= 8'h00;
                                 rxcmd_st <= 1; end
                        1: begin
                             phy_oe <= 1'b1;        // keep driving the byte
                             phy_do <= rxcmd_q[7:0]; rxcmd_st <= 2;
                             $display("[cyc %0d] MODEL sends RX CMD %02h", cyc, rxcmd_q[7:0]);
                           end
                        default: begin
                            dir <= 1'b0; phy_oe <= 1'b0;
                            rxcmd_st <= 0; rxcmd_q <= -1;
                        end
                    endcase
                end else if (link_data !== 8'h00 && link_data !== 8'hzz) begin
                    cmd <= link_data;
                    nxt <= 1'b1;             // acknowledge the TX CMD
                    casez (link_data[7:6])
                        2'b10: state <= S_REGW;
                        2'b11: begin send_i <= 0; state <= S_REGR; end
                        2'b01: begin
                            pkt_pid <= link_data;
                            pkt_len <= 0;
                            state   <= S_RXPKT;
                            $display("[%0t] cyc%0d LINK TX CMD %02h", $time, cyc, link_data);
                        end
                        default: ;
                    endcase
                end
            end

            // hold nxt through the write, like a real PHY throttling the link
            S_REGW: begin
                nxt <= 1'b1;
                if (stp) begin
                    nxt   <= 1'b0;
                    state <= S_IDLE;
                end
            end

            // register read: turnaround, one data byte, release
            S_REGR: begin
                nxt <= 1'b0;
                case (send_i)
                    0: begin dir <= 1'b1; phy_oe <= 1'b1; phy_do <= 8'h00;
                             send_i <= 1; end
                    1: begin phy_do <= 8'h24; send_i <= 2; end
                    default: begin
                        dir <= 1'b0; phy_oe <= 1'b0; send_i <= 0;
                        state <= S_IDLE;
                    end
                endcase
            end

            // link is sending a USB packet: swallow bytes until stp
            S_RXPKT: begin
                nxt <= 1'b1;
                if (stp) begin
                    nxt <= 1'b0;
                    $display("[%0t] cyc%0d LINK packet PID=%02h, %0d payload bytes",
                             $time, cyc, pkt_pid, pkt_len);
                    if (pkt_pid == 8'h49) begin      // IN token -> answer data
                        last_token <= pkt_pid;
                        send_buf[0] <= resp[0]; send_buf[1] <= resp[1];
                        send_buf[2] <= resp[2]; send_buf[3] <= resp[3];
                        send_buf[4] <= resp[4]; send_buf[5] <= resp[5];
                        send_buf[6] <= resp[6]; send_buf[7] <= resp[7];
                        send_buf[8] <= resp[8]; send_buf[9] <= resp[9];
                        send_buf[10] <= resp[10];
                        send_n <= 11;
                        send_i <= 0;
                        state  <= S_SEND;
                    end else if ((pkt_pid == 8'h43) || (pkt_pid == 8'h4B)) begin
                        // a DATA packet from the host: acknowledge it
                        send_buf[0] <= 8'hD2;   // ACK
                        send_n      <= 1;
                        send_i      <= 0;
                        state       <= S_SEND;
                        $display("[cyc %0d] MODEL acknowledges host DATA packet", cyc);
                    end else if (pkt_pid == 8'h42) begin
                        acks_seen = acks_seen + 1;
                        $display("[%0t] cyc%0d *** HOST ACK RECEIVED (%0d cycles after data) ***",
                                 $time, cyc, cyc - data_done_cyc);
                        state <= S_IDLE;
                    end else begin
                        state <= S_IDLE;
                    end
                end else begin
                    pkt_len <= pkt_len + 1;
                end
            end

            // send the DATA1 response: dir high, then bytes with nxt high
            S_SEND: begin
                dir <= 1'b1;
                if (send_i == 0) begin
                    phy_oe <= 1'b1; nxt <= 1'b0; phy_do <= 8'h00; // turnaround
                    send_i <= send_i + 1;
                end else if (send_i <= send_n) begin
                    nxt    <= 1'b1;
                    phy_do <= send_buf[send_i-1];
                    send_i <= send_i + 1;
                end else begin
                    nxt    <= 1'b0;
                    phy_do <= 8'h00;          // RX CMD: RxActive cleared
                    phy_oe <= 1'b1;
                    dir    <= 1'b0;
                    state  <= S_WAITACK;
                    ack_wait <= 0;
                    data_done_cyc <= cyc;
                    if (send_n > 1)
                        $display("[cyc %0d] MODEL finished DATA1 packet, waiting for host ACK", cyc);
                end
            end

            S_WAITACK: begin
                phy_oe <= 1'b0;
                nxt    <= 1'b0;
                ack_wait <= ack_wait + 1;
                if (link_data !== 8'h00 && link_data !== 8'hzz) begin
                    cmd <= link_data;
                    nxt <= 1'b1;
                    if (link_data[7:6] == 2'b01) begin
                        pkt_pid <= link_data;
                        pkt_len <= 0;
                        state   <= S_RXPKT;
                    end else if (link_data[7:6] == 2'b11) state <= S_REGR;
                    else if (link_data[7:6] == 2'b10) state <= S_REGW;
                end else if (ack_wait > 400) begin
                    $display("[%0t] cyc%0d !!! NO ACK from host within 400 cycles !!!",
                             $time, cyc);
                    state <= S_IDLE;
                end
            end
        endcase
    end

    // did the host latch it?
    always @(posedge ulpi_clk) begin
        if (dut.rxcmd_stb)
            $display("[cyc %0d] HOST latched RX CMD %02h (LineState=%b)",
                     cyc, dut.rxcmd, dut.rxcmd[1:0]);
    end

    always @(posedge ulpi_clk) begin
        if (dut.rx_valid)
            $display("[cyc %0d] HOST rx_valid: pid=%02h len=%0d auto_ack=%b ack_pending=%b",
                     cyc, dut.rxbuf[0], dut.rx_len, dut.auto_ack, dut.ack_pending);

    end
    reg ack_pending_d_tb = 0;

    // acceptance criterion: the bus must never be quiet for 3ms
    integer last_sof_cyc = 0;
    integer max_sof_gap = 0;
    always @(posedge ulpi_clk) begin
        if (state == S_RXPKT && stp && pkt_pid == 8'h45) begin
            if (cyc - last_sof_cyc > max_sof_gap) max_sof_gap = cyc - last_sof_cyc;
            last_sof_cyc = cyc;
        end
    end

    reg [3:0] step_d = 0;
    always @(posedge ulpi_clk) begin
        if (dut.step != step_d) begin
            $display("[cyc %0d] >>> script advanced to step %0d (step_ok=%04h)",
                     cyc, dut.step, dut.step_ok);
            step_d <= dut.step;
        end
    end

    // periodic peek inside the host
    integer mon = 0;
    always @(posedge ulpi_clk) begin
        mon = mon + 1;
        if (mon % 2000 == 0)
            $display("[cyc %0d] hstate=%0d estate=%0d data_oe=%b dir=%b nxt=%b rq_req=%b step=%0d",
                     cyc, dut.hstate, dut.estate, dut.data_oe, dir, nxt,
                     dut.rq_req, dut.step);
    end

    initial begin
        $dumpfile("/tmp/ulpi.vcd");
        $dumpvars(0, tb_ulpi);
        #200 rst_n = 1;
        #30000000;                // 30 ms of simulated time
        $display("=== simulation end: %0d host ACKs seen ===", acks_seen);
        $display("=== worst gap between SOFs: %0d cycles = %0d us (limit 3000 us) ===",
                 max_sof_gap, max_sof_gap / 60);
        $finish;
    end

endmodule
