// USB host bring-up step 2: complete the Hi-Speed detection handshake.
// Tang Primer 20K Dock, SMSC USB3317 ULPI PHY, HackRF One attached.
//
// Step 1 (proven): driving SE0 makes the device answer with chirp K at ~0.9ms,
// held ~2ms - verified against an unplugged-device control run.
//
// This module finishes the handshake per ULPI 3.8.5.1 / USB2.0 7.1.7.5:
//   1. write Function Control = 0x50  (XcvrSelect=HS, TermSelect=0,
//      OpMode=10 chirp, SuspendM=1)  -> PHY drives SE0, this is T0
//   2. wait for the device chirp K   (RX CMD LineState = 10b)
//   3. when the bus leaves chirp K, within 100us start the host chirp:
//      alternating K and J segments, each ~50us (spec wants 40..60us),
//      12 segments so the device comfortably sees its required K-J-K-J-K-J
//   4. write Function Control = 0x40  (OpMode=00 normal, still HS) and log
//      what the line does next
//
// Chirp transmission uses TX CMD NOPID (0x40) followed by a stream of data
// bytes: with OpMode=10 the PHY inserts no stuff bits and holds nxt high
// until it sees stp, so the Link just feeds one byte per cycle and closes
// with a single stp. Chirp K is a stream of 0x00, chirp J a stream of 0xFF.
//
// Success indicator: after the handshake a device that went Hi-Speed removes
// its 1.5k pull-up, so LineState reads 00 (squelch / HS idle). A device that
// fell back to Full Speed shows LineState 01 (D+ high) again.
//
// UART: M11 -> /dev/ttyUSB1 @115200, clocked from the 27MHz oscillator.

// DIAG_HOLD: park the radio on the boot frequency (5800 MHz) and never run
// the channel sweep, so the video diagnostics observe one constant signal.
// Left defined, this is indistinguishable from "the channel search is broken":
// FPV telemetry just reads CH=1B SC=0 for ever. Enable it only for a specific
// measurement, and turn it off again afterwards.
//`define DIAG_HOLD

module ulpi_enum_test (
    input  wire       clk27,
    input  wire       rst_n,
    input  wire       ulpi_clk,
    input  wire       ulpi_dir,
    input  wire       ulpi_nxt,
    output wire       ulpi_stp,
    output wire       ulpi_rst,
    inout  wire [7:0] ulpi_data,
    output wire       uart_tx,
    output wire [3:0] led,
    // HDMI (DVI) output
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_data_p,
    output wire [2:0] tmds_data_n
);

    // ---------------- 27MHz domain: PHY hardware reset ----------------
    // Internal Power-On Reset generator: guarantees 255 cycles of reset upon SRAM config
    reg [7:0] boot_por_cnt = 8'd0;
    wire      boot_por_n   = (boot_por_cnt == 8'd255);
    always @(posedge clk27) begin
        if (!boot_por_n) boot_por_cnt <= boot_por_cnt + 8'd1;
    end
    wire sys_rst_n = boot_por_n & rst_n;

    reg [1:0] rst_sync_c;
    wire      rst_n_c = rst_sync_c[1];
    always @(posedge clk27 or negedge sys_rst_n) begin
        if (!sys_rst_n) rst_sync_c <= 2'b00;
        else            rst_sync_c <= {rst_sync_c[0], 1'b1};
    end

    reg [15:0] phy_rst_cnt;
    reg        phy_rst_n;
`ifdef SIM
    localparam [15:0] PHY_RST_CYCLES = 16'd20;
`else
    localparam [15:0] PHY_RST_CYCLES = 16'd27_000;
`endif
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            phy_rst_cnt <= 16'd0; phy_rst_n <= 1'b0;
        end else if (phy_rst_cnt != PHY_RST_CYCLES) begin
            phy_rst_cnt <= phy_rst_cnt + 16'd1; phy_rst_n <= 1'b0;
        end else phy_rst_n <= 1'b1;
    end
    assign ulpi_rst = phy_rst_n;

    // ---------------- ULPI domain ----------------
    wire ulpi_clk_i = ~ulpi_clk;

    reg [1:0] rst_sync_u;
    wire      rst_n_u = rst_sync_u[1] & phy_rst_n;
    always @(posedge ulpi_clk_i or negedge sys_rst_n) begin
        if (!sys_rst_n) rst_sync_u <= 2'b00;
        else            rst_sync_u <= {rst_sync_u[0], 1'b1};
    end

    reg        dir_d, data_oe, stp_r;
    reg  [7:0] data_out;
    // Release the bus in the same cycle the PHY asserts dir. data_oe is a
    // register, so without this gate the Link keeps driving for one more
    // cycle and fights the PHY exactly when it is handing us the device's
    // response. Releasing early is always safe; driving late is not.
    assign ulpi_data = (data_oe && !ulpi_dir) ? data_out : 8'hzz;
    assign ulpi_stp  = stp_r;

    // request interface: mode 0 = reg read, 1 = reg write, 2 = chirp stream
    reg        rq_req;
    reg  [2:0] rq_mode;
    reg  [5:0] rq_addr;
    reg  [7:0] rq_wdata;
    reg  [7:0] rq_rdata;
    reg  [7:0] rq_b0, rq_b1; // token payload (frame number + CRC5)
    reg  [7:0] rq_pid;       // TX CMD byte for token/data/handshake packets
    reg  [7:0] txbuf [0:7];  // control transfer payload
    reg  [3:0] rq_dlen;      // payload length
    reg  [3:0] tx_i;
    reg [15:0] crc16;
    reg        crc_phase;
    reg [11:0] rq_len;      // bytes per chirp segment
    reg  [7:0] rq_segs;     // number of alternating segments in one packet
    reg        rq_done, rq_abort;
    reg        cmd_first;
    reg [11:0] tx_cnt;
    reg  [7:0] seg_cnt;
    reg  [1:0] gap_c;     // lets the token's stp pulse clear before the data
    // USB gives the host a hard window to acknowledge a received data packet.
    // Going back through the sequencer to send the ACK costs hundreds of ns,
    // the device times out and retransmits, and every later stage of the
    // transfer is then ignored. The engine therefore sends the ACK itself,
    // the moment the packet ends.
    reg        auto_ack;      // armed by the sequencer before an IN
    reg        ack_pending;

    localparam M_READ = 3'd0, M_WRITE = 3'd1, M_CHIRP = 3'd2,
               M_TOKEN = 3'd3, M_DATA  = 3'd4, M_PIDONLY = 3'd5,
               M_OUTDATA = 3'd6;   // OUT token + DATA packet, back to back

    localparam E_IDLE  = 5'd0,  E_CMD   = 5'd1,  E_WDATA = 5'd2,
               E_WSTP  = 5'd3,  E_TURN  = 5'd4,  E_CAP   = 5'd5,
               E_REL   = 5'd6,  E_ACKW  = 5'd7,  E_TXCMD = 5'd8,
               E_TXDAT = 5'd9,  E_TXSTP = 5'd10, E_TKCMD = 5'd11,
               E_TKD0  = 5'd12, E_TKD1  = 5'd13, E_DPAY  = 5'd14,
               E_DCRC  = 5'd15, E_OTK   = 5'd16, E_OTK0  = 5'd17,
               E_OTK1  = 5'd18, E_OGAP  = 5'd19, E_AACK  = 5'd20;
    reg [4:0] estate;

    wire in_reg_read = (estate == E_TURN) || (estate == E_CAP) ||
                       (estate == E_REL);

    reg [3:0] step;        // control-transfer script position (declared early:
                           // the bus trace arms on it)
    reg       ctrl_done;   // declared early so trace can arm on bulk streaming
    reg [7:0] rxcmd;
    reg       rxcmd_stb;
    reg       burst_captured;

    // USB packet receive: per ULPI 3.8.2.4, while dir is asserted a byte with
    // nxt high is packet data and a byte with nxt low is an RX CMD; the packet
    // ends when RxActive clears in an RX CMD or dir drops, whichever is first.
    reg  [7:0] rxbuf [0:47];
    reg  [9:0] rx_n;
    reg        rx_busy;
    reg        rx_valid;
    reg  [9:0] rx_len;
    reg [15:0] rq_crc;

    // Bus trace: 32 cycles of {dir, nxt, data} starting at the status-OUT
    // transaction, which is the one the device never answers.
    reg [9:0] tr [0:63];
    reg [5:0] tr_idx;
    reg       tr_active;
    reg       tr_full;

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            estate <= E_IDLE; dir_d <= 1'b1; data_oe <= 1'b0;
            data_out <= 8'h00; stp_r <= 1'b0; rq_done <= 1'b0;
            rq_rdata <= 8'h00; cmd_first <= 1'b0; tx_cnt <= 12'd0;
            seg_cnt <= 8'd0; gap_c <= 2'd0; ack_pending <= 1'b0;
            rxcmd <= 8'h00; rxcmd_stb <= 1'b0; burst_captured <= 1'b0;
            rx_n <= 10'd0; rx_busy <= 1'b0; rx_valid <= 1'b0; rx_len <= 10'd0;
            tx_i <= 4'd0; crc_phase <= 1'b0;
            tr_idx <= 6'd0; tr_active <= 1'b0; tr_full <= 1'b0;
        end else begin
            dir_d     <= ulpi_dir;
            stp_r     <= 1'b0;
            rq_done   <= 1'b0;
            rxcmd_stb <= 1'b0;

            if (tr_active) begin
                tr[tr_idx] <= {ulpi_dir, ulpi_nxt, ulpi_data};
                if (tr_idx == 6'd63) begin
                    tr_active <= 1'b0; tr_full <= 1'b1;
                end else tr_idx <= tr_idx + 6'd1;
            end else if (!tr_full && (estate == E_IDLE) && rq_req &&
                         (rq_mode == M_TOKEN) && (rq_pid == 8'h49) &&
                         ctrl_done) begin
                // the bulk IN on EP 0x81, end to end
                tr_active <= 1'b1; tr_idx <= 6'd0;
            end

            rx_valid <= 1'b0;
            if (ulpi_dir && dir_d && !in_reg_read) begin
                if (ulpi_nxt) begin
                    if (rx_n < 10'd48) rxbuf[rx_n[5:0]] <= ulpi_data;
                    rx_n    <= rx_n + 10'd1;
                    rx_busy <= 1'b1;
                end else begin
                    rxcmd          <= ulpi_data;
                    rxcmd_stb      <= 1'b1;
                    burst_captured <= 1'b1;
                    if (rx_busy && (ulpi_data[5:4] != 2'b01)) begin
                        rx_busy  <= 1'b0;
                        rx_len   <= rx_n;
                        rx_valid <= 1'b1;
                        rx_n     <= 10'd0;
                        // Every DATA packet we receive is one we asked for,
                        // so acknowledge it unconditionally. Gating this on a
                        // flag meant retries, probes and status stages went
                        // unacknowledged, and the device then never completed
                        // the transfer.
                        if ((rxbuf[0] == 8'h4B) || (rxbuf[0] == 8'hC3))
                            ack_pending <= 1'b1;
                    end
                end
            end else if (!ulpi_dir) begin
                burst_captured <= 1'b0;
                if (rx_busy) begin
                    rx_busy  <= 1'b0;
                    rx_len   <= rx_n;
                    rx_valid <= 1'b1;
                    rx_n     <= 10'd0;
                    if ((rxbuf[0] == 8'h4B) || (rxbuf[0] == 8'hC3))
                        ack_pending <= 1'b1;
                end
            end

            if (rq_abort) begin
                estate <= E_IDLE;
                if (!ulpi_dir) begin
                    data_oe <= 1'b1; data_out <= 8'h00; stp_r <= 1'b1;
                end else data_oe <= 1'b0;
            end else
            case (estate)
                E_IDLE: begin
                    data_oe <= !ulpi_dir;
                    if (ack_pending && !ulpi_dir && !dir_d && !ulpi_nxt) begin
                        data_oe   <= 1'b1;
                        data_out  <= 8'h42;      // TX CMD PID ACK
                        cmd_first <= 1'b1;
                        estate    <= E_AACK;
                    end else if (!ulpi_dir && !dir_d && rq_req && !ulpi_nxt) begin
                        cmd_first <= 1'b1;
                        tx_cnt    <= 12'd0;
                        seg_cnt   <= 8'd0;
                        case (rq_mode)
                            M_READ:  begin
                                data_out <= {2'b11, rq_addr}; estate <= E_CMD;
                            end
                            M_WRITE: begin
                                data_out <= {2'b10, rq_addr}; estate <= E_CMD;
                            end
                            M_CHIRP: begin
                                data_out <= 8'h40;  // TX CMD NOPID
                                estate   <= E_TXCMD;
                            end
                            M_TOKEN: begin
                                data_out <= rq_pid; // SOF / SETUP / IN / OUT
                                estate   <= E_TKCMD;
                            end
                            M_DATA: begin
                                data_out  <= rq_pid; // DATA0 / DATA1
                                tx_i      <= 4'd0;
                                crc_phase <= 1'b0;
                                estate    <= E_DPAY;
                            end
                            M_OUTDATA: begin
                                data_out  <= 8'h41;  // TX CMD PID OUT
                                tx_i      <= 4'd0;
                                crc_phase <= 1'b0;
                                estate    <= E_OTK;
                            end
                            default: begin
                                data_out <= rq_pid;  // handshake, PID only
                                estate   <= E_TKCMD;
                            end
                        endcase
                    end else if (!ulpi_dir) data_out <= 8'h00;
                end

                E_CMD: begin
                    cmd_first <= 1'b0;
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt && !cmd_first) begin
                        if (rq_mode == M_WRITE) begin
                            data_out <= rq_wdata; estate <= E_WDATA;
                        end else begin
                            data_oe <= 1'b0; data_out <= 8'h00;
                            estate  <= E_TURN;
                        end
                    end
                end

                E_WDATA: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else begin
                        stp_r <= 1'b1; estate <= E_WSTP;
                    end
                end
                E_WSTP: begin
                    data_oe <= 1'b1; data_out <= 8'h00;
                    rq_done <= 1'b1; estate <= E_ACKW;
                end

                // chirp: TX CMD is on the bus this cycle (the PHY must not
                // assert nxt yet), present the first data byte next cycle
                E_TXCMD: begin
                    data_out <= rq_wdata;
                    estate   <= E_TXDAT;
                end
                // Hold the chirp byte; the PHY keeps nxt high for the whole
                // stream, one byte consumed per asserted cycle. The byte is
                // inverted (K 0x00 <-> J 0xFF) every rq_len bytes so the whole
                // alternating sequence goes out as ONE continuous packet -
                // closing and reopening a packet per segment would insert SE0
                // gaps between the chirps, which is not what the device is
                // waiting for.
                E_TXDAT: begin
                    if (ulpi_dir) begin            // PHY aborted the packet
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt) begin
                        if (tx_cnt == rq_len - 12'd1) begin
                            tx_cnt <= 12'd0;
                            if (seg_cnt == rq_segs - 8'd1) begin
                                stp_r    <= 1'b1;
                                data_out <= 8'h00;
                                estate   <= E_TXSTP;
                            end else begin
                                seg_cnt  <= seg_cnt + 8'd1;
                                data_out <= ~data_out;   // K <-> J
                            end
                        end else tx_cnt <= tx_cnt + 12'd1;
                    end
                end
                E_TXSTP: begin
                    data_oe <= 1'b1; data_out <= 8'h00;
                    rq_done <= 1'b1; estate <= E_ACKW;
                end

                // token packet: TX CMD with PID, then the two payload bytes,
                // closed with a single stp - same throttling rules as a write
                E_TKCMD: begin
                    cmd_first <= 1'b0;
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt && !cmd_first) begin
                        if (rq_mode == M_PIDONLY) begin
                            stp_r <= 1'b1; data_out <= 8'h00;
                            estate <= E_TXSTP;      // ACK/NAK carry no data
                        end else begin
                            data_out <= rq_b0; estate <= E_TKD0;
                        end
                    end
                end
                E_TKD0: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt) begin
                        data_out <= rq_b1; estate <= E_TKD1;
                    end
                end
                E_TKD1: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt) begin
                        stp_r <= 1'b1; data_out <= 8'h00; estate <= E_TXSTP;
                    end
                end

                // DATA packet: the TX CMD (PID) is on the bus; feed payload
                // bytes as the PHY consumes them, then the two CRC16 bytes,
                // then close with stp.
                E_DPAY: begin
                    cmd_first <= 1'b0;
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt && !cmd_first) begin
                        if (tx_i == rq_dlen) begin
                            data_out  <= rq_crc[7:0];
                            crc_phase <= 1'b0;
                            estate    <= E_DCRC;
                        end else begin
                            data_out <= txbuf[tx_i[2:0]];
                            tx_i     <= tx_i + 4'd1;
                        end
                    end
                end
                E_DCRC: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt) begin
                        if (!crc_phase) begin
                            data_out  <= rq_crc[15:8];
                            crc_phase <= 1'b1;
                        end else begin
                            stp_r <= 1'b1; data_out <= 8'h00;
                            estate <= E_TXSTP;
                        end
                    end
                end

                // OUT token, immediately followed by its DATA packet. Issuing
                // these as one engine operation keeps the inter-packet gap
                // tight and stops the sequencer handshake (and any RX CMD the
                // PHY squeezes in) from landing between them.
                E_OTK: begin
                    cmd_first <= 1'b0;
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt && !cmd_first) begin
                        data_out <= rq_b0; estate <= E_OTK0;
                    end
                end
                E_OTK0: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt) begin
                        data_out <= rq_b1; estate <= E_OTK1;
                    end
                end
                E_OTK1: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt) begin
                        stp_r <= 1'b1; data_out <= 8'h00;
                        gap_c <= 2'd0; estate <= E_OGAP;
                    end
                end
                E_OGAP: begin
                    // stp has gone out; as soon as it has cleared and the bus
                    // is ours again, start the data packet without handing
                    // control back to the sequencer.
                    if (gap_c != 2'd2) begin
                        gap_c <= gap_c + 2'd1;
                    end else if (!ulpi_dir && !ulpi_nxt) begin
                        data_oe   <= 1'b1;
                        data_out  <= 8'h4B;      // DATA1
                        cmd_first <= 1'b1;
                        tx_i      <= 4'd0;
                        crc_phase <= 1'b0;
                        estate    <= E_DPAY;
                    end else if (ulpi_dir) begin
                        data_oe <= 1'b0;
                    end
                end

                E_AACK: begin
                    cmd_first <= 1'b0;
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt && !cmd_first) begin
                        stp_r       <= 1'b1;
                        data_out    <= 8'h00;
                        ack_pending <= 1'b0;
                        estate      <= E_IDLE;
                    end
                end

                E_TURN: begin
                    data_oe <= 1'b0;
                    if (ulpi_dir) estate <= E_CAP;
                end
                E_CAP: begin
                    rq_rdata <= ulpi_data; estate <= E_REL;
                end
                E_REL: begin
                    if (!ulpi_dir) begin
                        data_oe <= 1'b1; data_out <= 8'h00;
                        rq_done <= 1'b1; estate <= E_ACKW;
                    end
                end
                E_ACKW: begin
                    data_oe <= !ulpi_dir;
                    if (!ulpi_dir) data_out <= 8'h00;
                    if (!rq_req) estate <= E_IDLE;
                end
                default: estate <= E_IDLE;
            endcase
        end
    end

    // USB CRC5: LSB-first form, reversed polynomial 0x14, init and final
    // XOR 0x1F, as used for token packets.
    function [4:0] crc5_11;
        input [10:0] d;
        reg [4:0] c;
        integer i;
        begin
            c = 5'h1F;
            for (i = 0; i < 11; i = i + 1) begin
                if (d[i] ^ c[0]) c = {1'b0, c[4:1]} ^ 5'h14;
                else             c = {1'b0, c[4:1]};
            end
            crc5_11 = c ^ 5'h1F;
        end
    endfunction

    // USB CRC16 for DATA packets: LSB-first, reversed polynomial 0xA001,
    // init and final XOR 0xFFFF, transmitted low byte first.
    function [15:0] crc16_b;
        input [15:0] c;
        input [7:0]  d;
        reg   [15:0] cc;
        integer i;
        begin
            cc = c;
            for (i = 0; i < 8; i = i + 1) begin
                if (d[i] ^ cc[0]) cc = {1'b0, cc[15:1]} ^ 16'hA001;
                else              cc = {1'b0, cc[15:1]};
            end
            crc16_b = cc;
        end
    endfunction

    // ---------------- timestamped RX CMD log ----------------
    localparam LOG_N = 32;
    reg [23:0] logmem [0:LOG_N-1];
    reg [5:0]  log_wr;
    reg        log_en;
    reg [21:0] tick_div;
    reg [15:0] tstamp;

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            tick_div <= 22'd0; tstamp <= 16'd0;
        end else if (log_en) begin
            if (tick_div == 22'd63) begin
                tick_div <= 22'd0; tstamp <= tstamp + 16'd1;
            end else tick_div <= tick_div + 22'd1;
        end else begin
            tick_div <= 22'd0; tstamp <= 16'd0;
        end
    end



    // ---------------- host sequencer ----------------
    localparam H_POR    = 6'd0,  H_SETTLE = 6'd1,  H_RESET  = 6'd2,
               H_RESETW = 6'd3,  H_WAITK  = 6'd4,  H_INK    = 6'd5,
               H_HCHIRP = 6'd6,  H_HCHW   = 6'd7,  H_HSMODE = 6'd8,
               H_HSMODW = 6'd9,  H_OBS    = 6'd10, H_FS     = 6'd11,
               H_FSW    = 6'd12, H_DONE   = 6'd13, H_WAIT   = 6'd14,
               H_CLR    = 6'd15, H_SOF    = 6'd16, H_SOFW   = 6'd17,
               H_CPREP  = 6'd18, H_CCRC   = 6'd19, H_CTOK   = 6'd20,
               H_CTOKW  = 6'd21, H_CDAT   = 6'd22, H_CDATW  = 6'd23,
               H_CACK   = 6'd24, H_CIN    = 6'd25, H_CINW   = 6'd26,
               H_CRXW   = 6'd27, H_CSACK  = 6'd28, H_CSACKW = 6'd29,
               H_OTOK   = 6'd30, H_OTOKW  = 6'd31, H_ODAT   = 6'd32,
               H_ODATW  = 6'd33, H_OACKW  = 6'd34, H_NEXT   = 6'd35,
               H_PARAMS = 6'd36, H_GAP2   = 6'd37, H_BTOK   = 6'd38,
               H_BTOKW  = 6'd39, H_BRXW   = 6'd40, H_BACK   = 6'd41,
               H_BACKW  = 6'd42, H_OPREP  = 6'd43, H_OCRC   = 6'd44,
               H_SOFI   = 6'd45, H_SOFIW  = 6'd46, H_PIN    = 6'd47,
               H_PINW   = 6'd48, H_PRXW   = 6'd49, H_RTUNE  = 6'd50, H_AGC    = 6'd51;

    localparam [7:0] FN_CHIRP  = 8'h50; // HS xcvr, TermSel=0, OpMode=chirp
    localparam [7:0] FN_HS     = 8'h40; // HS xcvr, TermSel=0, OpMode=normal
    localparam [7:0] FN_FS     = 8'h41; // FS xcvr, normal - safe idle state
`ifdef SIM
    localparam [15:0] SOF_BEFORE_SCRIPT = 16'd3;
    localparam [23:0] POR_CYCLES    = 24'd50;
    localparam [22:0] SETTLE_CYCLES = 23'd100;
    localparam [11:0] CHIRP_LEN     = 12'd20;
    localparam [15:0] GAP_ENUM      = 16'd2;
    localparam [15:0] GAP_CFG       = 16'd4;
    localparam [15:0] GAP_PRERX     = 16'd4;
    localparam [15:0] GAP_POSTRX    = 16'd4;
    localparam [15:0] GAP_BULK      = 16'd4;
`else
    localparam [15:0] SOF_BEFORE_SCRIPT = 16'd800;       // 100ms clean SOF before script
    localparam [23:0] POR_CYCLES    = 24'd600_000;
    localparam [22:0] SETTLE_CYCLES = 23'd6_000_000;     // 100ms attach debounce
    localparam [11:0] CHIRP_LEN     = 12'd3000;          // ~50us of bytes at 60MHz
    localparam [15:0] GAP_ENUM      = 16'd64;            // 8ms normal gap between steps
    localparam [15:0] GAP_CFG       = 16'd4000;          // 500ms after SET_CONFIGURATION(1)
    localparam [15:0] GAP_PRERX     = 16'd2000;          // 250ms with radio configured before RX
    localparam [15:0] GAP_POSTRX    = 16'd2000;          // 250ms after RX mode for PLL/DMA to settle
    localparam [15:0] GAP_BULK      = 16'd800;           // 100ms before starting bulk IN
`endif
    localparam [7:0]  CHIRP_SEGS = 8'd130;  // ~6.5ms of K/J at 50us each

    reg [5:0]  hstate;
    reg [5:0]  resume_state;   // where to go after an interleaved SOF
    // A NAKed transaction is retried in the NEXT microframe, like a real host
    // does, instead of being hammered in place: the bus therefore never goes
    // quiet long enough (3ms) for the device to suspend and drop its
    // configuration.
    reg [5:0]  ctl_resume;
    reg [3:0]  reset_tries;    // bus-reset attempts, capped on purpose
    reg        attach_seen;    // a device has appeared on the bus
    reg [23:0] por_cnt;
    reg [19:0] wdog;
    reg [22:0] twait;
    reg [3:0]  seg;
    reg        dev_chirped;
    reg [7:0]  ls_after;
    reg [10:0] frame_no;
    reg [2:0]  uframe;
    reg [15:0] sof_cnt;    // SOFs actually accepted by the PHY
    reg [7:0]  desc [0:3]; // bLength, type, bcdUSB lo/hi
    reg [7:0]  dvid_lo, dvid_hi, dpid_lo, dpid_hi, dmps;
    reg [7:0]  ack_pid;    // handshake PID the device answered SETUP with
    reg [9:0]  rx_len_l;   // length of the IN packet we received
    reg [7:0]  in_pid;     // PID the device answered the IN token with
    reg [15:0] retries;    // NAK retries counter
    reg [6:0]  dev_addr;   // USB address currently assigned to the device
    reg [7:0]  ctl_bm, ctl_req, ctl_vlo, ctl_vhi, ctl_len;
    reg [7:0]  ctl_ilo, ctl_ihi;      // wIndex
    reg        ctl_dout;              // transfer has an OUT data stage
    reg        st_stage;              // 0 = data stage, 1 = status stage
    reg [7:0]  ctl_pl [0:7];          // OUT payload
    reg [7:0]  vbuf [0:15];   // payload of vendor IN requests
    reg [7:0]  board_id;
    reg [7:0]  oack_pid;   // handshake the device gave our OUT data stage
    reg        ok_flag;    // current script step completed without a timeout
    reg [15:0] fail_mask;  // steps that did not complete cleanly
    reg [7:0]  cfg_val;    // value reported by GET_CONFIGURATION
    reg        addr0_alive; // device answered a descriptor read at address 0
    reg [7:0]  si_val2;    // Si5351C register 27 (reference value 0x01)
    reg [7:0]  si_val;     // Si5351C register 29 - proves the sample-rate
                           // config actually reached the firmware
    reg [7:0]  ost_pid;    // reply to the zero-length status OUT
    reg [7:0]  probe_pid; // PID of a second IN issued right after our ACK:
                          // the same DATA PID again means the device never
                          // accepted the ACK and is retransmitting
    reg [7:0]  probe_b1;
    reg        probe_done;
    reg        ost_seen;
    reg [15:0] pkt_cnt;    // bulk IN data packets received
    reg [31:0] byte_cnt;   // payload bytes received
    reg [7:0]  bulk_pid;
    reg [7:0]  iq0, iq1, iq2, iq3;   // first samples of the latest packet
    reg [15:0] bulk_nak;
    reg [3:0]  bburst;
    reg        ctl_din;    // transfer has an IN data stage
    reg [15:0] step_ok;    // one bit per completed script step
    reg        ctl_pending;
    reg [15:0] gap_sofs;   // microframes to wait before the next script step
    reg [7:0]  m0_act;
    reg [7:0]  m0_err;
    reg [31:0] m0_cnt;
    reg [31:0] m4_cnt;
    reg [15:0] m0_sf;
    reg        retune_pending_u;
    reg        retune_active;
    reg [15:0] retune_freq_u;
    reg        retune_done_toggle_u;
    reg [1:0]  retune_req_sync_u;
    // AGC registers live in the 27 MHz domain; the loop itself sits further
    // down, next to rf_rssi which drives it.
    reg [5:0]  agc_gain;      // VGA gain in 2 dB units
    reg        agc_req;
    reg [23:0] agc_timer;
    reg        agc_pending_u;
    reg        agc_active;
    reg [5:0]  agc_gain_u;
    reg [1:0]  agc_req_sync_u;
    wire        scanner_retune_req;
    wire [15:0] scanner_retune_freq;

    // Safety interlock. HackRF vendor requests 10, 11 and 12 are SPIFLASH
    // ERASE / WRITE / READ. This design has no business touching the device's
    // flash, so those request numbers are physically substituted with a
    // harmless BOARD_ID_READ before they can reach a SETUP packet. Standard
    // request 5 (SET_ADDRESS) is untouched because it is not a vendor request.
    wire is_vendor_req = (ctl_bm[6:5] == 2'b10);
    wire req_is_flash  = (ctl_req >= 8'd10) && (ctl_req <= 8'd12);
    wire [7:0] safe_req = (is_vendor_req && req_is_flash) ? 8'd14 : ctl_req;

    // Token payload for endpoint 0 at the current device address: the 11-bit
    // field is {endpoint[3:0], address[6:0]} sent LSB first.
    wire [7:0] tok_b0 = {1'b0, dev_addr};
    wire [7:0] tok_b1 = {crc5_11({4'd0, dev_addr}), 3'd0};
    // bulk endpoint 1 (HackRF streams IQ on 0x81): field {ep,addr}
    wire [10:0] ep1_field = {4'd1, dev_addr};
    wire [7:0]  tok1_b0 = ep1_field[7:0];
    wire [7:0]  tok1_b1 = {crc5_11(ep1_field), ep1_field[10:8]};
    reg [15:0] crc_acc;
    reg [3:0]  crc_i;

    wire [1:0] ls_now = rxcmd[1:0];

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            hstate <= H_POR; resume_state <= H_OBS; reset_tries <= 4'd0;
            attach_seen <= 1'b0; ctl_resume <= 6'd0;
            por_cnt <= 24'd0; wdog <= 20'd0; twait <= 23'd0;
            rq_req <= 1'b0; rq_mode <= M_READ; rq_addr <= 6'd0;
            rq_wdata <= 8'd0; rq_len <= 12'd1; rq_segs <= 8'd1;
            rq_abort <= 1'b0;
            log_wr <= 6'd0; log_en <= 1'b0; seg <= 4'd0;
            dev_chirped <= 1'b0; ls_after <= 8'h00;
            frame_no <= 11'd0; uframe <= 3'd0; rq_b0 <= 8'd0; rq_b1 <= 8'd0;
            sof_cnt <= 16'd0; ack_pid <= 8'h00; rx_len_l <= 10'd0;
            ctrl_done <= 1'b0; crc_acc <= 16'hFFFF; crc_i <= 4'd0;
            in_pid <= 8'h00; retries <= 8'd0;
            dev_addr <= 7'd0; step <= 4'd0; step_ok <= 16'd0;
            ctl_bm <= 8'h80; ctl_req <= 8'h06; ctl_vlo <= 8'h00;
            ctl_vhi <= 8'h01; ctl_len <= 8'h12; ctl_din <= 1'b1;
            ctl_pending <= 1'b0; gap_sofs <= 16'd0; board_id <= 8'hFF;
            auto_ack <= 1'b0;
            ctl_ilo <= 8'h00; ctl_ihi <= 8'h00; ctl_dout <= 1'b0;
            st_stage <= 1'b0; oack_pid <= 8'h00; ok_flag <= 1'b1;
            fail_mask <= 16'd0; cfg_val <= 8'hFF;
            ost_pid <= 8'h00; ost_seen <= 1'b0; si_val <= 8'hFF;
            probe_pid <= 8'h00; probe_b1 <= 8'h00; probe_done <= 1'b0;
            si_val2 <= 8'hFF; addr0_alive <= 1'b0;
            pkt_cnt <= 16'd0; byte_cnt <= 32'd0; bulk_pid <= 8'h00;
            iq0 <= 8'd0; iq1 <= 8'd0; iq2 <= 8'd0; iq3 <= 8'd0;
            bulk_nak <= 16'd0; bburst <= 4'd0;
            m0_act <= 8'h00; m0_err <= 8'h00;
            m0_cnt <= 32'd0; m4_cnt <= 32'd0; m0_sf <= 16'd0;
            rq_pid <= 8'h45; rq_dlen <= 4'd0; rq_crc <= 16'h0000;
            retune_pending_u     <= 1'b0;
            retune_active        <= 1'b0;
            retune_freq_u        <= 16'd5800;
            retune_done_toggle_u <= 1'b0;
            retune_req_sync_u    <= 2'b00;
            agc_pending_u        <= 1'b0;
            agc_active           <= 1'b0;
            agc_gain_u           <= 6'd12;
            agc_req_sync_u       <= 2'b00;
        end else begin
            rq_abort <= 1'b0;

            agc_req_sync_u <= {agc_req_sync_u[0], agc_req};
            if (agc_req_sync_u[0] && !agc_req_sync_u[1] && ctrl_done) begin
                agc_pending_u <= 1'b1;
                agc_gain_u    <= agc_gain;
            end
            retune_req_sync_u <= {retune_req_sync_u[0], scanner_retune_req};
            if (retune_req_sync_u[0] && !retune_req_sync_u[1] && ctrl_done) begin
                retune_pending_u <= 1'b1;
                retune_freq_u    <= scanner_retune_freq;
            end

            if (log_en && rxcmd_stb && (log_wr != LOG_N)) begin
                logmem[log_wr[4:0]] <= {tstamp, rxcmd};
                log_wr              <= log_wr + 6'd1;
            end

            case (hstate)
                H_POR: begin
                    if (ulpi_dir) por_cnt <= 24'd0;
                    else if (por_cnt == POR_CYCLES) begin
                        twait <= 23'd0; hstate <= H_SETTLE;
                    end else por_cnt <= por_cnt + 24'd1;
                end

                // Wait for a device to actually appear before touching the
                // bus: LineState leaves SE0 when the device powers up and
                // pulls D+ high. Resetting an empty port is pointless and
                // lets the board be configured before the radio is plugged in.
                H_SETTLE: begin
                    // Wait for device to be stably connected with D+ high (Full-Speed J state: 2'b01)
                    if (rxcmd[1:0] == 2'b01) begin
                        if (twait >= SETTLE_CYCLES) begin
                            log_en      <= 1'b1;
                            dev_chirped <= 1'b0;
                            hstate      <= H_RESET;
                        end else twait  <= twait + 23'd1;
                    end else begin
                        twait <= 23'd0;
                    end
                end

                // T0: drive SE0
                H_RESET: begin
                    rq_addr <= 6'h04; rq_wdata <= FN_CHIRP;
                    rq_mode <= M_WRITE; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_RESETW;
                end
                H_RESETW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_WAITK;
                    end else if (wdog == 20'd600_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_RESET;
                    end
                end

                // wait for the device chirp K (LineState = 10b)
                H_WAITK: begin
                    if (rxcmd_stb && (ls_now == 2'b10)) begin
                        dev_chirped <= 1'b1; hstate <= H_INK;
                    end else if (twait == 23'd3_000_000) begin  // 50ms, no chirp
                        twait  <= 23'd0;
                        hstate <= H_SETTLE;
                    end else twait <= twait + 23'd1;
                end
                // in chirp K: leave as soon as the bus leaves that state
                H_INK: begin
                    if (rxcmd_stb && (ls_now != 2'b10)) begin
                        seg <= 4'd0; hstate <= H_HCHIRP;
                    end
                end

                // host chirp: one continuous packet, K/J alternating inside
                H_HCHIRP: begin
                    rq_mode  <= M_CHIRP;
                    rq_wdata <= 8'h00;          // start on chirp K
                    rq_len   <= CHIRP_LEN;
                    rq_segs  <= CHIRP_SEGS;
                    rq_req   <= 1'b1;
                    wdog     <= 20'd0;
                    hstate   <= H_HCHW;
                end
                H_HCHW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; hstate <= H_HSMODE;
                    end else if (wdog == 20'd900_000) begin  // ~15ms
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_HSMODE;
                    end
                end

                // leave chirp mode, stay Hi-Speed
                H_HSMODE: begin
                    rq_addr <= 6'h04; rq_wdata <= FN_HS;
                    rq_mode <= M_WRITE; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_HSMODW;
                end
                H_HSMODW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_OBS;
                    end else if (wdog == 20'd600_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_HSMODE;
                    end
                end

                // After the handshake the host MUST keep the bus alive: a
                // Hi-Speed device that sees 3ms of idle treats it as suspend
                // and drops back to Full Speed - which is exactly what we
                // measured before SOFs existed. Send one SOF per 125us
                // microframe from here on and never leave this loop.
                H_OBS: begin
                    if (rxcmd_stb) ls_after <= rxcmd;
                    // NOTE: do not try to detect a Full Speed fallback by
                    // sampling LineState here. While a SOF is on the wire the
                    // line legitimately reads 01, so such a check fires on a
                    // perfectly healthy Hi-Speed link and tears it down.
                    if (twait >= 23'd5_000) begin   // ~83us @60MHz (total microframe ~125us)
                        twait  <= 23'd0;
                        hstate <= H_SOF;
                    end else twait <= twait + 23'd1;
                end

                H_SOF: begin
                    rq_mode <= M_TOKEN;
                    rq_pid  <= 8'h45;      // SOF PID
                    rq_b0   <= frame_no[7:0];
                    rq_b1   <= {crc5_11(frame_no), frame_no[10:8]};
                    rq_req  <= 1'b1;
                    wdog    <= 20'd0;
                    hstate  <= H_SOFW;
                end
                H_SOFW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req  <= 1'b0;
                        sof_cnt <= sof_cnt + 16'd1;
                        uframe  <= uframe + 3'd1;
                        if (uframe == 3'd7) frame_no <= frame_no + 11'd1;
                        // Drive the control-transfer script from the SOF loop
                        // so the bus keeps getting microframes between steps
                        // (3ms of silence would suspend the device).
                        if (gap_sofs != 16'd0) gap_sofs <= gap_sofs - 16'd1;
                        if (!ctrl_done && !ctl_pending && (sof_cnt > SOF_BEFORE_SCRIPT))
                            ctl_pending <= 1'b1;
                        if (ctl_resume != 6'd0) begin
                            hstate     <= ctl_resume;   // resume the retry
                            ctl_resume <= 6'd0;
                        end else if (!ctrl_done && ctl_pending && (gap_sofs == 16'd0))
                            hstate <= H_PARAMS;
                        else if (ctrl_done && (gap_sofs == 16'd0)) begin
                            if (retune_pending_u && !retune_active) begin
                                retune_pending_u <= 1'b0;
                                retune_active    <= 1'b1;
                                hstate           <= H_RTUNE;
                            end else if (agc_pending_u && !agc_active) begin
                                agc_pending_u <= 1'b0;
                                agc_active    <= 1'b1;
                                hstate        <= H_AGC;
                            end else begin
                                bburst <= 4'd0;
                                twait  <= 23'd0;
                                hstate <= H_GAP2;
                            end
                        end
                        else hstate <= H_OBS;
                    end else if (wdog == 20'd60_000) begin  // 1ms
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_OBS;
                    end
                end

                // 20us post-SOF inter-packet gap to satisfy USB 2.0 host IPG
                H_GAP2: begin
                    if (twait >= 23'd1_200) begin
                        twait  <= 23'd0;
                        hstate <= H_BTOK;
                    end else twait <= twait + 23'd1;
                end

                // park the PHY back in a safe FS state before re-running
                H_FS: begin
                    rq_addr <= 6'h04; rq_wdata <= FN_FS;
                    rq_mode <= M_WRITE; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_FSW;
                end
                H_FSW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0;
                        // SAFETY: stop here. Never drive a second bus reset.
                        hstate <= H_DONE;
                    end else if (wdog == 20'd600_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_FS;
                    end
                end

                // ---- generic control transfer ----
                // Script: 0 GET_DESCRIPTOR @addr0, 1 SET_ADDRESS(1),
                //         2 GET_DESCRIPTOR @addr1 (proves the address stuck),
                //         3 SET_CONFIGURATION(1)
                H_PARAMS: begin
                    ctl_ilo <= 8'h00; ctl_ihi <= 8'h00; ctl_dout <= 1'b0;
                    case (step)
                        4'd0: begin                       // GET_DESCRIPTOR
                            ctl_bm <= 8'h80; ctl_req <= 8'h06;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h01;
                            ctl_len <= 8'h12; ctl_din <= 1'b1;
                        end
                        4'd1: begin                       // SET_ADDRESS(1)
                            ctl_bm <= 8'h00; ctl_req <= 8'h05;
                            ctl_vlo <= 8'h01; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h00; ctl_din <= 1'b0;
                        end
                        4'd2: begin                       // GET_DESCRIPTOR @1
                            ctl_bm <= 8'h80; ctl_req <= 8'h06;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h01;
                            ctl_len <= 8'h12; ctl_din <= 1'b1;
                        end
                        4'd3: begin                       // SET_CONFIGURATION
                            ctl_bm <= 8'h00; ctl_req <= 8'h09;
                            ctl_vlo <= 8'h01; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h00; ctl_din <= 1'b0;
                        end
                        4'd4: begin                       // BOARD_ID_READ
                            ctl_bm <= 8'hC0; ctl_req <= 8'd14;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h01; ctl_din <= 1'b1;
                        end
                        // ---- from here on the sequence mirrors libhackrf,
                        // ---- captured from a working receive on the PC
                        4'd5: begin                       // SAMPLE_RATE_SET
                            ctl_bm <= 8'h40; ctl_req <= 8'd6;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h08; ctl_din <= 1'b0;
                            ctl_dout <= 1'b1;
                            ctl_pl[0] <= 8'h00; ctl_pl[1] <= 8'h2D;
                            ctl_pl[2] <= 8'h31; ctl_pl[3] <= 8'h01; // 20 MHz
                            ctl_pl[4] <= 8'h01; ctl_pl[5] <= 8'h00;
                            ctl_pl[6] <= 8'h00; ctl_pl[7] <= 8'h00; // divider 1
                        end
                        4'd6: begin      // baseband filter 10 MHz: an analog FPV
                                         // FM signal is ~18 MHz wide, a narrower
                                         // filter removes the deviation extremes
                                         // (and with them the sync tip)
                            ctl_bm <= 8'h40; ctl_req <= 8'd7;
                            // 10 MHz: measured deviation is only +-2.3 MHz, so
                            // Carson's rule wants ~13 MHz. At 20 MHz more than
                            // half the passband was collecting noise only.
                            ctl_vlo <= 8'h80; ctl_vhi <= 8'h96;   // 0x9680
                            ctl_ilo <= 8'h98; ctl_ihi <= 8'h00;   // 0x0098
                            ctl_len <= 8'h00; ctl_din <= 1'b0;
                        end
                        4'd7: begin                       // AMP_ENABLE(1) = +14 dB RF amplifier
                            ctl_bm <= 8'h40; ctl_req <= 8'd17;
                            ctl_vlo <= 8'h01; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h00; ctl_din <= 1'b0;
                        end
                        4'd8: begin                       // SET_FREQ 5800 MHz
                            ctl_bm <= 8'h40; ctl_req <= 8'd16;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h08; ctl_din <= 1'b0;
                            ctl_dout <= 1'b1;
                            ctl_pl[0] <= 8'hA8; ctl_pl[1] <= 8'h16;
                            ctl_pl[2] <= 8'h00; ctl_pl[3] <= 8'h00; // 5800
                            ctl_pl[4] <= 8'h00; ctl_pl[5] <= 8'h00;
                            ctl_pl[6] <= 8'h00; ctl_pl[7] <= 8'h00; // +0 Hz
                        end
                        // Gain note: these are IN requests, and their status
                        // stage never completes, so step_ok bits 9/10 read 0.
                        // The device applies them anyway - proven by sweeping
                        // VGA 36 -> 56 dB and watching the noise floor rise
                        // 258 -> 2425 (x9.4 for +20 dB). Do not trust the flag.
                        4'd9: begin                       // SET_VGA_GAIN 24dB - 38 clipped 50..100% on a close transmitter
                            ctl_bm <= 8'hC0; ctl_req <= 8'd20;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_ilo <= 8'd24; ctl_ihi <= 8'h00;
                            ctl_len <= 8'h01; ctl_din <= 1'b1;
                        end
                        4'd10: begin                      // SET_LNA_GAIN 40dB
                            ctl_bm <= 8'hC0; ctl_req <= 8'd19;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_ilo <= 8'd40; ctl_ihi <= 8'h00;
                            ctl_len <= 8'h01; ctl_din <= 1'b1;
                        end
                        4'd11: begin     // SET_TRANSCEIVER_MODE = RECEIVE
                            ctl_bm <= 8'h40; ctl_req <= 8'd1;
                            ctl_vlo <= 8'h01; ctl_vhi <= 8'h00;
                            ctl_len <= 8'h00; ctl_din <= 1'b0;
                        end
                        4'd12: begin
                            // Read Si5351C register 29 back. The PC leaves
                            // 0x0E there after configuring 8 Msps; 0x00 would
                            // mean our sample-rate request was accepted but
                            // never applied.
                            ctl_bm <= 8'hC0; ctl_req <= 8'd5;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_ilo <= 8'd29; ctl_ihi <= 8'h00;
                            ctl_len <= 8'h01; ctl_din <= 1'b1;
                        end
                        default: begin
                            // Step 13: HACKRF_VENDOR_REQUEST_GET_M0_STATE = 41 (0x29)
                            ctl_bm <= 8'hC0; ctl_req <= 8'd41;
                            ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                            ctl_ilo <= 8'h00; ctl_ihi <= 8'h00;
                            ctl_len <= 8'd40; ctl_din <= 1'b1;
                        end
                    endcase
                    retries <= 8'd0;
                    ok_flag <= 1'b1;
                    hstate  <= H_CPREP;
                end

                // SET_VGA_GAIN, same shape as script step 9. Note this is an
                // IN request whose status stage never completes, so step_ok
                // would read 0 - the device applies it regardless (verified by
                // sweeping the gain and watching the noise floor track it).
                H_AGC: begin
                    ctl_bm  <= 8'hC0;
                    ctl_req <= 8'd20;
                    ctl_vlo <= 8'h00; ctl_vhi <= 8'h00;
                    ctl_ilo <= {agc_gain_u, 1'b0};  // gain in dB = units * 2
                    ctl_ihi <= 8'h00;
                    ctl_len <= 8'h01;
                    ctl_din <= 1'b1;  ctl_dout <= 1'b0;
                    retries <= 8'd0;
                    ok_flag <= 1'b1;
                    hstate  <= H_CPREP;
                end

                H_RTUNE: begin
                    ctl_bm    <= 8'h40;
                    ctl_req   <= 8'd16; // SET_FREQ
                    ctl_vlo   <= 8'h00; ctl_vhi <= 8'h00;
                    ctl_ilo   <= 8'h00; ctl_ihi <= 8'h00;
                    ctl_len   <= 8'h08;
                    ctl_din   <= 1'b0;  ctl_dout <= 1'b1;
                    ctl_pl[0] <= retune_freq_u[7:0];
                    ctl_pl[1] <= retune_freq_u[15:8];
                    ctl_pl[2] <= 8'h00; ctl_pl[3] <= 8'h00;
                    ctl_pl[4] <= 8'h00; ctl_pl[5] <= 8'h00;
                    ctl_pl[6] <= 8'h00; ctl_pl[7] <= 8'h00;
                    retries   <= 8'd0;
                    ok_flag   <= 1'b1;
                    hstate    <= H_CPREP;
                end

                H_CPREP: begin
                    txbuf[0] <= ctl_bm;
                    txbuf[1] <= safe_req;
                    txbuf[2] <= ctl_vlo;
                    txbuf[3] <= ctl_vhi;
                    txbuf[4] <= ctl_ilo;
                    txbuf[5] <= ctl_ihi;
                    txbuf[6] <= ctl_len;
                    txbuf[7] <= 8'h00;
                    rq_dlen  <= 4'd8;
                    crc_acc  <= 16'hFFFF;
                    crc_i    <= 4'd0;
                    hstate   <= H_CCRC;
                end
                H_CCRC: begin
                    if (crc_i == 4'd8) begin
                        rq_crc <= ~crc_acc;
                        hstate <= H_CTOK;
                    end else begin
                        crc_acc <= crc16_b(crc_acc, txbuf[crc_i[2:0]]);
                        crc_i   <= crc_i + 4'd1;
                    end
                end
                H_CTOK: begin
                    rq_mode <= M_TOKEN; rq_pid <= 8'h4D;   // SETUP
                    rq_b0 <= tok_b0; rq_b1 <= tok_b1;
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_CTOKW;
                end
                H_CTOKW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin rq_req <= 1'b0; hstate <= H_CDAT; end
                    else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1;
                        ok_flag <= 1'b0; hstate <= H_NEXT;
                    end
                end
                H_CDAT: begin
                    rq_mode <= M_DATA; rq_pid <= 8'h43;    // DATA0
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_CDATW;
                end
                H_CDATW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_CACK;
                    end else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1;
                        ok_flag <= 1'b0; hstate <= H_NEXT;
                    end
                end
                H_CACK: begin
                    if (rx_valid || (twait == 23'd3_000)) begin
                        if (rx_valid) ack_pid <= rxbuf[0];
                        // pick the stage that follows SETUP: an OUT data
                        // stage, an IN data stage, or straight to status
                        if (ctl_dout) begin
                            st_stage <= 1'b0; oack_pid <= 8'h00; ok_flag <= 1'b1;
                            hstate <= H_OPREP;
                        end else if (ctl_din) begin
                            st_stage <= 1'b0; oack_pid <= 8'h00; ok_flag <= 1'b1;
                            hstate <= H_CIN;
                        end else begin
                            st_stage <= 1'b1; hstate <= H_CIN;
                        end
                    end else twait <= twait + 23'd1;
                end

                // IN transaction: fetches the data stage when the transfer has
                // one, otherwise it is the status stage (zero-length DATA1).
                H_CIN: begin
                    rq_mode <= M_TOKEN; rq_pid <= 8'h49;   // IN
                    rq_b0 <= tok_b0; rq_b1 <= tok_b1;
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_CINW;
                end
                H_CINW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_CRXW;
                    end else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1;
                        ok_flag <= 1'b0; hstate <= H_NEXT;
                    end
                end
                // A one-byte reply is a handshake: the device NAKs until it
                // has the data ready, so the host keeps asking.
                H_CRXW: begin
                    if (rx_valid) begin
                        in_pid   <= rxbuf[0];
                        rx_len_l <= rx_len;
                        if ((rxbuf[0] == 8'h4B) || (rxbuf[0] == 8'hC3)) begin
                            if (ctl_din && (step < 4'd3)) begin
                                desc[0] <= rxbuf[1];
                                desc[1] <= rxbuf[2];
                                desc[2] <= rxbuf[3];
                                desc[3] <= rxbuf[4];
                                dmps    <= rxbuf[8];
                                dvid_lo <= rxbuf[9];
                                dvid_hi <= rxbuf[10];
                                dpid_lo <= rxbuf[11];
                                dpid_hi <= rxbuf[12];
                            end
                            if (ctl_din && (step == 4'd4)) board_id <= rxbuf[1];
                            if (ctl_din && (step == 4'd9)) cfg_val <= rxbuf[1];
                            if (ctl_din && (step == 4'd12)) si_val  <= rxbuf[1];
                            if (ctl_din && (step == 4'd13)) begin
                                m0_act <= rxbuf[5];
                                m0_cnt <= {rxbuf[12], rxbuf[11], rxbuf[10], rxbuf[9]};
                                m4_cnt <= {rxbuf[16], rxbuf[15], rxbuf[14], rxbuf[13]};
                                m0_sf  <= {rxbuf[18], rxbuf[17]};
                                m0_err <= rxbuf[37];
                            end


                            if (ctl_din && (step == 4'd5)) begin
                                vbuf[0]  <= rxbuf[1];  vbuf[1]  <= rxbuf[2];
                                vbuf[2]  <= rxbuf[3];  vbuf[3]  <= rxbuf[4];
                                vbuf[4]  <= rxbuf[5];  vbuf[5]  <= rxbuf[6];
                                vbuf[6]  <= rxbuf[7];  vbuf[7]  <= rxbuf[8];
                                vbuf[8]  <= rxbuf[9];  vbuf[9]  <= rxbuf[10];
                                vbuf[10] <= rxbuf[11]; vbuf[11] <= rxbuf[12];
                                vbuf[12] <= rxbuf[13]; vbuf[13] <= rxbuf[14];
                                vbuf[14] <= rxbuf[15]; vbuf[15] <= rxbuf[16];
                            end
                            hstate <= H_CSACK;
                        end else if (retries != 16'd10000) begin
                            retries <= retries + 16'd1;
                            twait   <= 23'd0;
                            hstate  <= H_CIN;
                        end else begin
                            ok_flag <= 1'b0; hstate <= H_NEXT;
                        end
                    end else if (twait == 23'd60_000) begin
                        ok_flag <= 1'b0; hstate <= H_NEXT;
                    end else twait <= twait + 23'd1;
                end
                // the ACK has already gone out from the engine
                H_CSACK: hstate <= H_CSACKW;   // the engine already ACKed
                H_CSACKW: begin
                    if (st_stage) hstate <= H_NEXT;
                    else begin
                        // an IN data stage still owes the device a
                        // zero-length OUT status stage
                        st_stage <= 1'b1;
                        retries  <= 8'd0;
                        hstate   <= H_OTOK;
                    end
                end

                // OUT status stage: zero-length DATA1, device answers ACK
                H_OPREP: begin
                    txbuf[0] <= ctl_pl[0]; txbuf[1] <= ctl_pl[1];
                    txbuf[2] <= ctl_pl[2]; txbuf[3] <= ctl_pl[3];
                    txbuf[4] <= ctl_pl[4]; txbuf[5] <= ctl_pl[5];
                    txbuf[6] <= ctl_pl[6]; txbuf[7] <= ctl_pl[7];
                    crc_acc <= 16'hFFFF; crc_i <= 4'd0;
                    hstate  <= H_OCRC;
                end
                H_OCRC: begin
                    if (crc_i == 4'd8) begin
                        rq_crc <= ~crc_acc;
                        hstate <= H_OTOK;
                    end else begin
                        crc_acc <= crc16_b(crc_acc, txbuf[crc_i[2:0]]);
                        crc_i   <= crc_i + 4'd1;
                    end
                end
                H_OTOK: begin
                    rq_mode <= M_OUTDATA;
                    rq_b0 <= tok_b0; rq_b1 <= tok_b1;
                    if (st_stage) begin
                        rq_dlen <= 4'd0;
                        rq_crc  <= 16'h0000;               // CRC16 of nothing
                    end else rq_dlen <= 4'd8;
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_ODATW;
                end
                H_OTOKW: hstate <= H_ODATW;
                H_ODAT:  hstate <= H_ODATW;
                H_ODATW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_OACKW;
                    end else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1;
                        ok_flag <= 1'b0; hstate <= H_NEXT;
                    end
                end
                // The device NAKs an OUT stage it is not ready for, and the
                // host is required to repeat the whole transaction until it
                // is acknowledged. Without this the payload of a vendor
                // request (sample rate, frequency) is silently dropped.
                H_OACKW: begin
                    if (rx_valid) begin
                        oack_pid <= rxbuf[0];
                        if (st_stage) begin
                            ost_pid  <= rxbuf[0];
                            ost_seen <= 1'b1;
                        end
                        // NYET (0x96) is a Hi-Speed "data accepted, not ready
                        // for more" - for a single-packet stage it counts as
                        // delivered, so treat it like ACK.
                        if ((rxbuf[0] == 8'hD2) || (rxbuf[0] == 8'h96)) begin
                            if (st_stage) hstate <= H_NEXT;
                            else begin
                                st_stage <= 1'b1;
                                retries  <= 8'd0;
                                hstate   <= H_CIN;
                            end
                        end else if ((rxbuf[0] == 8'h5A) &&
                                     (retries != 16'd10000)) begin   // NAK: retry
                            retries <= retries + 16'd1;
                            twait   <= 23'd0;
                            hstate  <= H_OTOK;
                        end else begin
                            ok_flag <= 1'b0; hstate <= H_NEXT;    // STALL etc.
                        end
                    end else if (twait == 23'd60_000) begin
                        ok_flag <= 1'b0; hstate <= H_NEXT;
                    end else twait <= twait + 23'd1;
                end

                H_NEXT: begin
                    if (agc_active) begin
                        agc_active <= 1'b0;
                        gap_sofs   <= 16'd10;
                        twait      <= 23'd0;
                        hstate     <= H_OBS;
                    end else if (retune_active) begin
                        retune_active        <= 1'b0;
                        retune_done_toggle_u <= ~retune_done_toggle_u;
                        gap_sofs             <= 16'd10; // ~1.25 ms
                        twait                <= 23'd0;
                        hstate               <= H_OBS;
                    end else begin
                        step_ok[step]   <= ok_flag;
                        fail_mask[step] <= ~ok_flag;
                        if ((step == 4'd1) && ok_flag) dev_addr <= 7'd1;  // SET_ADDRESS applied
                        if (step == 4'd13) begin
                            ctrl_done   <= 1'b1;
                            ctl_pending <= 1'b0;
                        end else step <= step + 4'd1;

                        // Phased delays:
                        // Step 3 (SET_CONFIGURATION): 500ms for HackRF to connect, light LED1, settle
                        // Step 10 (GAINS): 250ms with radio configured in idle before starting RX
                        // Step 11 (SET_TRANSCEIVER_MODE): 250ms for PLL to lock and LED2 to light
                        // Step 12 (Si5351 readback): 8ms
                        // Step 13 (GET_M0_STATE): 100ms before entering bulk streaming
                        if (step == 4'd3)       gap_sofs <= GAP_CFG;
                        else if (step == 4'd10) gap_sofs <= GAP_PRERX;
                        else if (step == 4'd11) gap_sofs <= GAP_POSTRX;
                        else if (step == 4'd12) gap_sofs <= GAP_ENUM;
                        else if (step == 4'd13) gap_sofs <= GAP_BULK;
                        else                    gap_sofs <= GAP_ENUM;

                        twait    <= 23'd0;
                        hstate   <= H_OBS;
                    end
                end

                // Diagnostic: a second IN right after our ACK. If the device
                // hands back the same DATA packet it never saw the ACK.
                H_PIN: begin
                    rq_mode <= M_TOKEN; rq_pid <= 8'h49;
                    rq_b0 <= tok_b0; rq_b1 <= tok_b1;
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_PINW;
                end
                H_PINW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_PRXW;
                    end else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1;
                        st_stage <= 1'b1; hstate <= H_OTOK;
                    end
                end
                H_PRXW: begin
                    if (rx_valid) begin
                        probe_pid <= rxbuf[0];
                        probe_b1  <= rxbuf[1];
                        st_stage  <= 1'b1;
                        hstate    <= H_OTOK;
                    end else if (twait == 23'd3_000) begin
                        probe_pid <= 8'hEE;      // no reply at all
                        st_stage  <= 1'b1;
                        hstate    <= H_OTOK;
                    end else twait <= twait + 23'd1;
                end

                // A microframe issued from inside a control transfer, so the
                // device never sees more than ~125us of silence.
                H_SOFI: begin
                    rq_mode <= M_TOKEN; rq_pid <= 8'h45;
                    rq_b0   <= frame_no[7:0];
                    rq_b1   <= {crc5_11(frame_no), frame_no[10:8]};
                    rq_req  <= 1'b1; wdog <= 20'd0; hstate <= H_SOFIW;
                end
                H_SOFIW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req  <= 1'b0;
                        sof_cnt <= sof_cnt + 16'd1;
                        uframe  <= uframe + 3'd1;
                        if (uframe == 3'd7) frame_no <= frame_no + 11'd1;
                        hstate <= resume_state;
                    end else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1;
                        hstate <= resume_state;
                    end
                end

                // ---- bulk IN from endpoint 1: the IQ stream ----
                H_BTOK: begin
                    rq_mode <= M_TOKEN; rq_pid <= 8'h49;   // IN
                    rq_b0 <= tok1_b0; rq_b1 <= tok1_b1;
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_BTOKW;
                end
                H_BTOKW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; twait <= 23'd0; hstate <= H_BRXW;
                    end else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; twait <= 23'd0; hstate <= H_OBS;
                    end
                end
                H_BRXW: begin
                    if (rx_valid) begin
                        bulk_pid <= rxbuf[0];
                        if ((rxbuf[0] == 8'h4B) || (rxbuf[0] == 8'hC3)) begin
                            // payload = packet minus PID and the two CRC bytes
                            pkt_cnt  <= pkt_cnt + 16'd1;
                            byte_cnt <= byte_cnt + {22'd0, rx_len} - 32'd3;
                            iq0 <= rxbuf[1]; iq1 <= rxbuf[2];
                            iq2 <= rxbuf[3]; iq3 <= rxbuf[4];
                            twait  <= 23'd0;
                            if (bburst != 4'd11) begin
                                bburst <= bburst + 4'd1;
                                hstate <= H_BTOK;
                            end else begin
                                bburst <= 4'd0;
                                hstate <= H_OBS;
                            end
                        end else begin
                            bulk_nak <= bulk_nak + 16'd1;
                            twait    <= 23'd0;
                            bburst   <= 4'd0;
                            hstate   <= H_OBS;
                        end
                    end else if (twait == 23'd1_200) begin
                        bulk_pid <= 8'hEE;     // timeout waiting for bulk response
                        twait  <= 23'd0;
                        bburst <= 4'd0;
                        hstate <= H_OBS;
                    end else twait <= twait + 23'd1;
                end
                H_BACK: begin
                    rq_mode <= M_PIDONLY; rq_pid <= 8'h42;  // ACK
                    rq_req <= 1'b1; wdog <= 20'd0; hstate <= H_BACKW;
                end
                H_BACKW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin rq_req <= 1'b0; hstate <= H_OBS; end
                    else if (wdog == 20'd60_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_OBS;
                    end
                end

                // Terminal: the device did not complete the handshake after a
                // couple of attempts. Sit still rather than keep resetting it.
                H_DONE: ;
                H_WAIT: begin
                    log_wr <= 6'd0; hstate <= H_CLR;
                end
                H_CLR: begin
                    logmem[log_wr[4:0]] <= 24'd0;
                    if (log_wr == LOG_N-1) begin
                        log_wr <= 6'd0; twait <= 23'd0;
                        hstate <= H_DONE;   // SAFETY: no re-arm, no re-reset
                    end else log_wr <= log_wr + 6'd1;
                end

                default: hstate <= H_POR;
            endcase
        end
    end

    // ---------------- Stage 1: USB Bulk IQ FIFO & Depacketizer ----------------
    wire in_bulk_rx_w = (hstate == H_BRXW);
    wire rx_byte_valid_w = (ulpi_dir && dir_d && !in_reg_read && ulpi_nxt);

    wire [12:0] fifo_wr_occ;
    wire        fifo_wr_full;
    wire        fifo_wr_ovf;
    wire [15:0] fifo_pkt_cnt;
    wire [31:0] fifo_sample_cnt;
    wire [15:0] fifo_err_cnt;
    wire        fifo_bulk_active;

    wire [15:0] fifo_rd_data;
    wire        fifo_rd_valid;
    wire        fifo_rd_empty;
    wire [12:0] fifo_rd_occ;
    wire        fifo_rd_unf;

    // In clk27 domain: drain FIFO continuously whenever samples are present
    wire fifo_rd_en = !fifo_rd_empty;

    reg [31:0] fifo_rd_cnt;
    reg [7:0]  live_i, live_q;
    reg [12:0] fifo_max_occ;

    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            fifo_rd_cnt  <= 32'd0;
            live_i       <= 8'd0;
            live_q       <= 8'd0;
            fifo_max_occ <= 13'd0;
        end else begin
            if (fifo_rd_valid) begin
                fifo_rd_cnt <= fifo_rd_cnt + 32'd1;
                live_i      <= fifo_rd_data[15:8];
                live_q      <= fifo_rd_data[7:0];
            end
            if (fifo_rd_occ > fifo_max_occ)
                fifo_max_occ <= fifo_rd_occ;
        end
    end

    // Synchronize wr_overflow to clk27 domain for telemetry
    reg [1:0] ovf_sync_c;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) ovf_sync_c <= 2'b00;
        else          ovf_sync_c <= {ovf_sync_c[0], fifo_wr_ovf};
    end
    wire fifo_ovf_c = ovf_sync_c[1];

    usb_iq_fifo #(
        .ADDR_WIDTH(12) // Depth 4096 words
    ) u_usb_iq_fifo (
        // Write domain (60 MHz ulpi_clk_i)
        .wr_clk         (ulpi_clk_i),
        .wr_rst_n       (rst_n_u),
        .in_bulk_rx     (in_bulk_rx_w),
        .rx_byte_valid  (rx_byte_valid_w),
        .rx_data        (ulpi_data),
        .rx_pkt_done    (rx_valid),
        .rx_pkt_len     (rx_len),
        .wr_occupancy   (fifo_wr_occ),
        .wr_full        (fifo_wr_full),
        .wr_overflow    (fifo_wr_ovf),
        .pkt_count      (fifo_pkt_cnt),
        .sample_count   (fifo_sample_cnt),
        .err_count      (fifo_err_cnt),
        .is_bulk_active (fifo_bulk_active),

        // Read domain (27 MHz clk27)
        .rd_clk         (clk27),
        .rd_rst_n       (rst_n_c),
        .rd_en          (fifo_rd_en),
        .rd_data        (fifo_rd_data),
        .rd_valid       (fifo_rd_valid),
        .rd_empty       (fifo_rd_empty),
        .rd_occupancy   (fifo_rd_occ),
        .rd_underflow   (fifo_rd_unf)
    );

    // ---------------- Stage 2: FM Demodulator & NTSC Video Processing ----------------
    wire [7:0]  cvbs_video;
    wire        cvbs_valid;
    wire signed [15:0] demod_dtheta;
    wire [15:0] rf_rssi;

    fm_demod u_fm_demod (
        .clk          (clk27),
        .rst_n        (rst_n_c),
        .sample_in    (fifo_rd_data),
        .sample_valid (fifo_rd_valid),
        .video_out    (cvbs_video),
        .video_valid  (cvbs_valid),
        .raw_dtheta   (demod_dtheta),
        .rssi         (rf_rssi)
    );

    // ---------------- Stage 3: NTSC Sync Separator & Video Lock ----------------
    wire       ntsc_hsync;
    wire       ntsc_ladv;
    wire       ntsc_vsync;
    wire       ntsc_locked;
    wire       ntsc_field;
    wire [9:0] ntsc_line;
    wire [9:0] ntsc_px;
    wire [8:0] ntsc_py;
    wire [7:0] ntsc_pval;
    wire       ntsc_pval_valid;
    wire [7:0] sep_slice, sep_tip, sep_blank;
    wire [19:0] sep_period;

    // The sample stream arrives in islands: the USB transport delivers a block
    // and loses the time between blocks (measured on captures/iq_session.txt -
    // whole buffer halves missing, splices exactly on the 8192-sample grid). The
    // phase either side of such a hole is unrelated, so tell the separator and
    // let it re-acquire rather than drag a stale phase across the seam. The FIFO
    // going empty for longer than a line is the symptom available on chip.
    reg [11:0] iq_dry;
    reg        iq_gap;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin iq_dry <= 12'd0; iq_gap <= 1'b0; end
        else begin
            iq_gap <= 1'b0;
            if (fifo_rd_valid) iq_dry <= 12'd0;
            else if (iq_dry != 12'd4095) begin
                iq_dry <= iq_dry + 12'd1;
                if (iq_dry == 12'd2047) iq_gap <= 1'b1;   // ~76 us, over one line
            end
        end
    end

    ntsc_sync_dpll u_ntsc_sync (
        .clk         (clk27),
        .rst_n       (rst_n_c),
        .video_in    (cvbs_video),
        .video_valid (cvbs_valid),
        .stream_gap  (iq_gap),
        .hsync       (ntsc_hsync),
        .line_advance(ntsc_ladv),
        .vsync       (ntsc_vsync),
        .ntsc_locked (ntsc_locked),
        .field       (ntsc_field),
        .line_cnt    (ntsc_line),
        .pixel_x     (ntsc_px),
        .pixel_y     (ntsc_py),
        .pixel_val   (ntsc_pval),
        .pixel_valid (ntsc_pval_valid),
        .dbg_slice   (sep_slice),
        .dbg_tip     (sep_tip),
        .dbg_blank   (sep_blank),
        .dbg_period  (sep_period)
    );

    // ---------------- Video-path instrumentation (diagnostics) ----------------
    wire [7:0]  vd_min, vd_max;
    wire [63:0] vd_hist;
    wire [15:0] vd_tiny, vd_hsync, vd_mid, vd_broad;
    wire [15:0] vd_plast, vd_pmin, vd_pmax;

    video_diag u_vdiag (
        .clk         (clk27),
        .rst_n       (rst_n_c),
        .video_in    (cvbs_video),
        .video_valid (cvbs_valid),
        .vmin        (vd_min),
        .vmax        (vd_max),
        .hist_flat   (vd_hist),
        .n_tiny      (vd_tiny),
        .n_hsync     (vd_hsync),
        .n_mid       (vd_mid),
        .n_broad     (vd_broad),
        .per_last    (vd_plast),
        .per_min     (vd_pmin),
        .per_max     (vd_pmax)
    );

    // ---------------- Stage 4: 40-Channel FPV Auto-Scanner ----------------
    // scanner_retune_req and scanner_retune_freq declared above
    wire        is_scanning;
    wire [5:0]  scan_cur_ch;
    wire [5:0]  scan_locked_ch;
    wire [15:0] scan_locked_freq;
    wire [15:0] scan_best_rssi;
    wire        scan_locked_sync;

    // CDC handshake: retune_done toggle back to clk27
    reg [2:0] retune_done_sync_c;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) retune_done_sync_c <= 3'd0;
        else          retune_done_sync_c <= {retune_done_sync_c[1:0], retune_done_toggle_u};
    end
    wire retune_done_pulse_c = retune_done_sync_c[2] ^ retune_done_sync_c[1];

    // Auto-trigger scan 1.0 second after enumeration completes, then re-scan periodically if unlocked
    reg [25:0] scan_boot_timer;
    reg        auto_scan_triggered;
    reg        scanner_start;

    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            scan_boot_timer     <= 26'd0;
            auto_scan_triggered <= 1'b0;
            scanner_start       <= 1'b0;
        end else begin
            scanner_start <= 1'b0;
            if (step_ok[11]) begin
                if (!auto_scan_triggered) begin
                    if (scan_boot_timer == 26'd27_000_000) begin // 1.0 sec initial scan
`ifndef DIAG_HOLD
                        scanner_start       <= 1'b1;
`endif
                        auto_scan_triggered <= 1'b1;
                        scan_boot_timer     <= 26'd0;
                    end else scan_boot_timer <= scan_boot_timer + 26'd1;
                end else if (!is_scanning && !ntsc_locked) begin
                    if (scan_boot_timer >= 26'd54_000_000) begin // 2.0 sec periodic rescan if unlocked
`ifndef DIAG_HOLD
                        scanner_start   <= 1'b1;
`endif
                        scan_boot_timer <= 26'd0;
                    end else scan_boot_timer <= scan_boot_timer + 26'd1;
                end else begin
                    scan_boot_timer <= 26'd0;
                end
            end
        end
    end

    fpv_auto_scanner u_scanner (
        .clk             (clk27),
        .rst_n           (rst_n_c),
        .start_scan      (scanner_start),
        .rssi            (rf_rssi),
        .ntsc_locked     (ntsc_locked),
        .retune_req      (scanner_retune_req),
        .retune_freq_mhz (scanner_retune_freq),
        .retune_done     (retune_done_pulse_c),
        .is_scanning     (is_scanning),
        .cur_ch_idx      (scan_cur_ch),
        .locked_ch_idx   (scan_locked_ch),
        .locked_freq_mhz (scan_locked_freq),
        .best_rssi       (scan_best_rssi),
        .locked_sync     (scan_locked_sync)
    );

    // ---------------- CDC + UART ----------------
    reg [21:0] pub_cnt;
    reg        snap_tog;
    reg [31:0] snap;
    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            pub_cnt <= 22'd0; snap_tog <= 1'b0; snap <= 32'd0;
        end else if (pub_cnt == 22'd3_000_000) begin
            pub_cnt  <= 22'd0;
            snap     <= {rxcmd, sof_cnt[15:8], sof_cnt[7:0],
                         {2'd0, dev_chirped, hstate[4:0]}};
            snap_tog <= ~snap_tog;
        end else pub_cnt <= pub_cnt + 22'd1;
    end

    reg [2:0] tog_sync;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) tog_sync <= 3'd0;
        else          tog_sync <= {tog_sync[1:0], snap_tog};
    end
    wire snap_event = tog_sync[2] ^ tog_sync[1];

    reg [31:0] snap_c;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) snap_c <= 32'd0;
        else if (snap_event) snap_c <= snap;
    end

    // ---------------- Automatic gain control ----------------
    // The carrier level swings about 19 dB depending on where the transmitter
    // is (measured |z| of 17, 90 and 152 on the same camera), so no fixed VGA
    // setting works: set it for the weak case and a close transmitter clips at
    // 50..100%, which destroys FM demodulation outright.
    //
    // rf_rssi accumulates |I|+|Q| over 1024 samples and keeps bits [19:4], so
    // rssi is about 81 * |z|. Aiming for |z| of 50..100 means holding rssi
    // between roughly 4000 and 8000, i.e. half the ADC range with headroom.
    localparam [15:0] AGC_HI   = 16'd8500;   // above this, back the gain off
    localparam [15:0] AGC_LO   = 16'd5000;   // below this, add gain
    localparam [5:0]  AGC_MAX  = 6'd31;      // 62 dB (VGA steps are 2 dB)
    localparam [23:0] AGC_TICK = 24'd5_400_000;  // 200 ms between adjustments

    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            agc_gain  <= 6'd12;        // 24 dB, the value the script starts with
            agc_req   <= 1'b0;
            agc_timer <= 24'd0;
        end else begin
            agc_req <= 1'b0;
            // Hold the gain still while the scanner sweeps. fpv_auto_scanner
            // ranks channels by {ntsc_locked, rssi}, so rssi IS its measurement;
            // an AGC that regulates rssi to a fixed band makes every channel
            // score the same and the sweep picks one at random. Channels can
            // only be compared at one gain, so the loop runs after the scan has
            // parked, not during it.
            if (scanner_start) begin
                // Put the gain back where the control script left it before the
                // sweep. A re-scan otherwise starts from wherever the loop had
                // wound up - at the 62 dB end every channel saturates and the
                // comparison is just as meaningless as with the loop running.
                agc_gain  <= 6'd12;        // 24 dB
                agc_req   <= 1'b1;
                agc_timer <= 24'd0;
            end else if (step_ok[11] && !is_scanning) begin
                if (agc_timer >= AGC_TICK) begin
                    agc_timer <= 24'd0;
                    if ((rf_rssi > AGC_HI) && (agc_gain != 6'd0)) begin
                        agc_gain <= agc_gain - 6'd1;
                        agc_req  <= 1'b1;
                    end else if ((rf_rssi < AGC_LO) && (agc_gain < AGC_MAX)) begin
                        agc_gain <= agc_gain + 6'd1;
                        agc_req  <= 1'b1;
                    end
                end else agc_timer <= agc_timer + 24'd1;
            end else agc_timer <= 24'd0;   // start a full tick after the sweep
        end
    end

    // ---------------- display-path instrumentation ----------------
    // Counts what actually reaches the line buffer, so a black screen can be
    // told from "pixels written but not displayed".
    reg [15:0] vo_pix_cnt, vo_line_cnt, vo_field_cnt;
    reg [23:0] vo_sum;
    reg [7:0]  vo_avg;
    reg [15:0] vo_avg_div;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            vo_pix_cnt <= 16'd0; vo_line_cnt <= 16'd0; vo_field_cnt <= 16'd0;
            vo_sum <= 24'd0; vo_avg <= 8'd0; vo_avg_div <= 16'd0;
        end else begin
            if (ntsc_pval_valid) begin
                vo_pix_cnt <= vo_pix_cnt + 16'd1;
                vo_sum     <= vo_sum + {16'd0, ntsc_pval};
                vo_avg_div <= vo_avg_div + 16'd1;
                if (vo_avg_div == 16'd8191) begin
                    vo_avg     <= vo_sum[20:13];   // mean over 8192 pixels
                    vo_sum     <= 24'd0;
                    vo_avg_div <= 16'd0;
                end
            end
            if (ntsc_hsync) vo_line_cnt  <= vo_line_cnt  + 16'd1;
            if (ntsc_vsync) vo_field_cnt <= vo_field_cnt + 16'd1;
        end
    end

    wire       dump_busy;
    wire [7:0] dump_data;
    wire       dump_valid;

    // hold the dumper off until the radio has been in RX for ~3 s
    reg [26:0] dump_arm_cnt;
    reg        dump_arm;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin dump_arm_cnt <= 27'd0; dump_arm <= 1'b0; end
        else if (step_ok[11]) begin
            if (dump_arm_cnt >= 27'd81_000_000) dump_arm <= 1'b1;
            else dump_arm_cnt <= dump_arm_cnt + 27'd1;
        end
    end

    reg [22:0] print_tick;
    wire       do_print = (print_tick == 23'd6_750_000);
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) print_tick <= 23'd0;
        else if (do_print) print_tick <= 23'd0;
        else print_tick <= print_tick + 23'd1;
    end

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h41 + {4'd0, n} - 8'd10);
    endfunction

    wire [7:0] c_rx    = snap_c[31:24];
    wire [7:0] c_sofh  = snap_c[23:16];
    wire [7:0] c_sofl  = snap_c[15:8];
    wire [4:0] c_hs    = snap_c[4:0];
    wire       c_chir  = snap_c[5];

    localparam SLEN = 30;
    reg [7:0] sbuf [0:SLEN-1];

    reg [8:0] pidx;
    reg       printing, pending;
    reg [3:0] pmode;          // 0 = status, 1 = RX CMD log, 2 = descriptor, 3 = M0 state, 4 = IQ FIFO
    localparam DLEN = 54;
    reg [7:0] dbuf [0:DLEN-1];
    localparam MLEN = 43;
    reg [7:0] mbuf [0:MLEN-1];
    localparam FLEN = 45;
    reg [7:0] fbuf [0:FLEN-1];
    localparam VLEN = 48;
    reg [7:0] vbuf_fpv [0:VLEN-1];
    localparam GLEN = 38;
    reg [7:0] gbuf [0:GLEN-1];
    localparam HLEN = 83;   // + the sync-separator slicer and PLL state
    reg [7:0] hbuf [0:HLEN-1];
    localparam OLEN = 62;
    reg [7:0] obuf [0:OLEN-1];

    wire [6:0]  agc_db   = {agc_gain, 1'b0};   // VGA gain in dB, for telemetry
    wire [5:0]  disp_ch   = is_scanning ? scan_cur_ch : scan_locked_ch;
    wire [15:0] disp_freq = is_scanning ? scanner_retune_freq : scan_locked_freq;

    reg [7:0] txd;
    reg       txv;
    wire      txr;

    // cycle-by-cycle bus trace {dir,nxt,data}
    wire [5:0] te  = pidx[7:2];
    wire [1:0] tsb = pidx[1:0];
    wire [9:0] tv  = tr[te];
    wire [7:0] log_char =
        (pidx == 9'd256) ? 8'h0D :
        (pidx == 9'd257) ? 8'h0A :
        (tsb == 2'd0) ? hexc({2'b00, tv[9:8]}) :
        (tsb == 2'd1) ? hexc(tv[7:4]) :
        (tsb == 2'd2) ? hexc(tv[3:0]) : " ";

    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            printing <= 1'b0; pidx <= 9'd0; txv <= 1'b0; txd <= 8'd0;
            pending  <= 1'b0; pmode <= 4'd0;
        end else begin
            txv <= 1'b0;
            if (do_print && !dump_busy) pending <= 1'b1;

            if (!printing) begin
                if (pending) begin
                    pending  <= 1'b0; printing <= 1'b1;
                    pidx     <= 9'd0;
                    pmode    <= (pmode == 4'd8) ? 4'd0 : pmode + 4'd1;
                    dbuf[0]  <= "V"; dbuf[1] <= "=";
                    dbuf[2]  <= hexc(dvid_hi[7:4]); dbuf[3] <= hexc(dvid_hi[3:0]);
                    dbuf[4]  <= hexc(dvid_lo[7:4]); dbuf[5] <= hexc(dvid_lo[3:0]);
                    dbuf[6]  <= " ";
                    dbuf[7]  <= "A"; dbuf[8] <= "D"; dbuf[9] <= "=";
                    dbuf[10] <= hexc({1'b0, dev_addr[2:0]});
                    dbuf[11] <= " ";
                    dbuf[12] <= "S"; dbuf[13] <= "T"; dbuf[14] <= "=";
                    dbuf[15] <= hexc(step_ok[15:12]); dbuf[16] <= hexc(step_ok[11:8]);
                    dbuf[17] <= hexc(step_ok[7:4]);   dbuf[18] <= hexc(step_ok[3:0]);
                    dbuf[19] <= " ";
                    dbuf[20] <= "F"; dbuf[21] <= "L"; dbuf[22] <= "=";
                    dbuf[23] <= hexc(fail_mask[15:12]);
                    dbuf[24] <= hexc(fail_mask[11:8]);
                    dbuf[25] <= hexc(fail_mask[7:4]); dbuf[26] <= hexc(fail_mask[3:0]);
                    dbuf[27] <= " ";
                    dbuf[28] <= "P"; dbuf[29] <= "R"; dbuf[30] <= "=";
                    dbuf[31] <= hexc(probe_pid[7:4]); dbuf[32] <= hexc(probe_pid[3:0]);
                    dbuf[33] <= " ";
                    dbuf[34] <= "O"; dbuf[35] <= "A"; dbuf[36] <= "=";
                    dbuf[37] <= hexc(oack_pid[7:4]); dbuf[38] <= hexc(oack_pid[3:0]);
                    dbuf[39] <= " ";
                    dbuf[40] <= "S"; dbuf[41] <= "I"; dbuf[42] <= "=";
                    dbuf[43] <= hexc(si_val[7:4]); dbuf[44] <= hexc(si_val[3:0]);
                    dbuf[45] <= " ";
                    dbuf[46] <= "N"; dbuf[47] <= "="; 
                    dbuf[48] <= hexc(pkt_cnt[15:12]); dbuf[49] <= hexc(pkt_cnt[11:8]);
                    dbuf[50] <= hexc(pkt_cnt[7:4]);   dbuf[51] <= hexc(pkt_cnt[3:0]);
                    dbuf[52] <= 8'h0D; dbuf[53] <= 8'h0A;
                    sbuf[0]  <= "R"; sbuf[1] <= "X"; sbuf[2] <= "=";
                    sbuf[3]  <= hexc(c_rx[7:4]);  sbuf[4] <= hexc(c_rx[3:0]);
                    sbuf[5]  <= " ";
                    sbuf[6]  <= "S"; sbuf[7] <= "O"; sbuf[8] <= "F";
                    sbuf[9]  <= "=";
                    sbuf[10] <= hexc(c_sofh[7:4]); sbuf[11] <= hexc(c_sofh[3:0]);
                    sbuf[12] <= hexc(c_sofl[7:4]); sbuf[13] <= hexc(c_sofl[3:0]);
                    sbuf[14] <= " ";
                    sbuf[15] <= "H"; sbuf[16] <= "=";
                    sbuf[17] <= hexc({3'd0, c_hs[4]});
                    sbuf[18] <= hexc(c_hs[3:0]);
                    sbuf[19] <= " ";
                    sbuf[20] <= "B"; sbuf[21] <= "P"; sbuf[22] <= "=";
                    sbuf[23] <= hexc(bulk_pid[7:4]); sbuf[24] <= hexc(bulk_pid[3:0]);
                    sbuf[25] <= " ";
                    sbuf[26] <= "S"; sbuf[27] <= hexc(step);
                    sbuf[28] <= 8'h0D; sbuf[29] <= 8'h0A;
                    mbuf[0]  <= "M"; mbuf[1]  <= "0"; mbuf[2]  <= ":";
                    mbuf[3]  <= "A"; mbuf[4]  <= "=";
                    mbuf[5]  <= hexc(m0_act[7:4]);   mbuf[6]  <= hexc(m0_act[3:0]);
                    mbuf[7]  <= " "; mbuf[8]  <= "E"; mbuf[9]  <= "=";
                    mbuf[10] <= hexc(m0_err[7:4]);   mbuf[11] <= hexc(m0_err[3:0]);
                    mbuf[12] <= " "; mbuf[13] <= "C"; mbuf[14] <= "=";
                    mbuf[15] <= hexc(m0_cnt[31:28]); mbuf[16] <= hexc(m0_cnt[27:24]);
                    mbuf[17] <= hexc(m0_cnt[23:20]); mbuf[18] <= hexc(m0_cnt[19:16]);
                    mbuf[19] <= hexc(m0_cnt[15:12]); mbuf[20] <= hexc(m0_cnt[11:8]);
                    mbuf[21] <= hexc(m0_cnt[7:4]);   mbuf[22] <= hexc(m0_cnt[3:0]);
                    mbuf[23] <= " "; mbuf[24] <= "M"; mbuf[25] <= "=";
                    mbuf[26] <= hexc(m4_cnt[31:28]); mbuf[27] <= hexc(m4_cnt[27:24]);
                    mbuf[28] <= hexc(m4_cnt[23:20]); mbuf[29] <= hexc(m4_cnt[19:16]);
                    mbuf[30] <= hexc(m4_cnt[15:12]); mbuf[31] <= hexc(m4_cnt[11:8]);
                    mbuf[32] <= hexc(m4_cnt[7:4]);   mbuf[33] <= hexc(m4_cnt[3:0]);
                    mbuf[34] <= " "; mbuf[35] <= "S"; mbuf[36] <= "=";
                    mbuf[37] <= hexc(m0_sf[15:12]);  mbuf[38] <= hexc(m0_sf[11:8]);
                    mbuf[39] <= hexc(m0_sf[7:4]);    mbuf[40] <= hexc(m0_sf[3:0]);
                    mbuf[41] <= 8'h0D; mbuf[42] <= 8'h0A;
                    fbuf[0]  <= "I"; fbuf[1]  <= "Q"; fbuf[2]  <= ":"; fbuf[3]  <= " ";
                    fbuf[4]  <= "W"; fbuf[5]  <= "R"; fbuf[6]  <= "=";
                    fbuf[7]  <= hexc(pkt_cnt[15:12]); fbuf[8]  <= hexc(pkt_cnt[11:8]);
                    fbuf[9]  <= hexc(pkt_cnt[7:4]);   fbuf[10] <= hexc(pkt_cnt[3:0]);
                    fbuf[11] <= " ";
                    fbuf[12] <= "R"; fbuf[13] <= "D"; fbuf[14] <= "=";
                    fbuf[15] <= hexc(fifo_rd_cnt[23:20]); fbuf[16] <= hexc(fifo_rd_cnt[19:16]);
                    fbuf[17] <= hexc(fifo_rd_cnt[15:12]); fbuf[18] <= hexc(fifo_rd_cnt[11:8]);
                    fbuf[19] <= " ";
                    fbuf[20] <= "O"; fbuf[21] <= "C"; fbuf[22] <= "C"; fbuf[23] <= "=";
                    fbuf[24] <= hexc({3'd0, fifo_rd_occ[12]});
                    fbuf[25] <= hexc(fifo_rd_occ[11:8]);
                    fbuf[26] <= hexc(fifo_rd_occ[7:4]);
                    fbuf[27] <= hexc(fifo_rd_occ[3:0]);
                    fbuf[28] <= " ";
                    fbuf[29] <= "I"; fbuf[30] <= "=";
                    fbuf[31] <= hexc(live_i[7:4]); fbuf[32] <= hexc(live_i[3:0]);
                    fbuf[33] <= " ";
                    fbuf[34] <= "Q"; fbuf[35] <= "=";
                    fbuf[36] <= hexc(live_q[7:4]); fbuf[37] <= hexc(live_q[3:0]);
                    fbuf[38] <= " ";
                    fbuf[39] <= "O"; fbuf[40] <= "V"; fbuf[41] <= "=";
                    fbuf[42] <= hexc({3'd0, fifo_ovf_c});
                    fbuf[43] <= 8'h0D; fbuf[44] <= 8'h0A;

                    vbuf_fpv[0]  <= "F"; vbuf_fpv[1]  <= "P"; vbuf_fpv[2]  <= "V";
                    vbuf_fpv[3]  <= ":"; vbuf_fpv[4]  <= " ";
                    vbuf_fpv[5]  <= "C"; vbuf_fpv[6]  <= "H"; vbuf_fpv[7]  <= "=";
                    vbuf_fpv[8]  <= hexc({2'b0, disp_ch[5:4]});
                    vbuf_fpv[9]  <= hexc(disp_ch[3:0]);
                    vbuf_fpv[10] <= " ";
                    vbuf_fpv[11] <= "F"; vbuf_fpv[12] <= "=";
                    vbuf_fpv[13] <= hexc(disp_freq[15:12]);
                    vbuf_fpv[14] <= hexc(disp_freq[11:8]);
                    vbuf_fpv[15] <= hexc(disp_freq[7:4]);
                    vbuf_fpv[16] <= hexc(disp_freq[3:0]);
                    vbuf_fpv[17] <= " ";
                    vbuf_fpv[18] <= "R"; vbuf_fpv[19] <= "S"; vbuf_fpv[20] <= "=";
                    vbuf_fpv[21] <= hexc(rf_rssi[15:12]);
                    vbuf_fpv[22] <= hexc(rf_rssi[11:8]);
                    vbuf_fpv[23] <= hexc(rf_rssi[7:4]);
                    vbuf_fpv[24] <= hexc(rf_rssi[3:0]);
                    vbuf_fpv[25] <= " ";
                    vbuf_fpv[26] <= "L"; vbuf_fpv[27] <= "K"; vbuf_fpv[28] <= "=";
                    vbuf_fpv[29] <= hexc({3'd0, ntsc_locked});
                    vbuf_fpv[30] <= " ";
                    vbuf_fpv[31] <= "S"; vbuf_fpv[32] <= "C"; vbuf_fpv[33] <= "=";
                    vbuf_fpv[34] <= hexc({3'd0, is_scanning});
                    vbuf_fpv[35] <= " ";
                    vbuf_fpv[36] <= "V"; vbuf_fpv[37] <= "=";
                    vbuf_fpv[38] <= hexc(cvbs_video[7:4]);
                    vbuf_fpv[39] <= hexc(cvbs_video[3:0]);
                    vbuf_fpv[40] <= " ";
                    vbuf_fpv[41] <= "Y"; vbuf_fpv[42] <= "=";
                    vbuf_fpv[43] <= hexc({2'b0, ntsc_line[9:8]});
                    vbuf_fpv[44] <= hexc(ntsc_line[7:4]);
                    vbuf_fpv[45] <= hexc(ntsc_line[3:0]);
                    vbuf_fpv[46] <= 8'h0D;
                    vbuf_fpv[47] <= 8'h0A;

                    // ---- video level distribution ----
                    gbuf[0] <= "V"; gbuf[1] <= "D"; gbuf[2] <= ":"; gbuf[3] <= " ";
                    gbuf[4] <= "M"; gbuf[5] <= "I"; gbuf[6] <= "N"; gbuf[7] <= "=";
                    gbuf[8] <= hexc(vd_min[7:4]); gbuf[9] <= hexc(vd_min[3:0]);
                    gbuf[10] <= " ";
                    gbuf[11] <= "M"; gbuf[12] <= "A"; gbuf[13] <= "X"; gbuf[14] <= "=";
                    gbuf[15] <= hexc(vd_max[7:4]); gbuf[16] <= hexc(vd_max[3:0]);
                    gbuf[17] <= " "; gbuf[18] <= "H"; gbuf[19] <= "=";
                    gbuf[20] <= hexc(vd_hist[7:4]);   gbuf[21] <= hexc(vd_hist[3:0]);
                    gbuf[22] <= hexc(vd_hist[15:12]); gbuf[23] <= hexc(vd_hist[11:8]);
                    gbuf[24] <= hexc(vd_hist[23:20]); gbuf[25] <= hexc(vd_hist[19:16]);
                    gbuf[26] <= hexc(vd_hist[31:28]); gbuf[27] <= hexc(vd_hist[27:24]);
                    gbuf[28] <= hexc(vd_hist[39:36]); gbuf[29] <= hexc(vd_hist[35:32]);
                    gbuf[30] <= hexc(vd_hist[47:44]); gbuf[31] <= hexc(vd_hist[43:40]);
                    gbuf[32] <= hexc(vd_hist[55:52]); gbuf[33] <= hexc(vd_hist[51:48]);
                    gbuf[34] <= hexc(vd_hist[63:60]); gbuf[35] <= hexc(vd_hist[59:56]);
                    gbuf[36] <= 8'h0D; gbuf[37] <= 8'h0A;

                    // ---- sync pulse census and measured line period ----
                    hbuf[0] <= "S"; hbuf[1] <= "Y"; hbuf[2] <= ":"; hbuf[3] <= " ";
                    hbuf[4] <= "T"; hbuf[5] <= "=";
                    hbuf[6] <= hexc(vd_tiny[15:12]); hbuf[7] <= hexc(vd_tiny[11:8]);
                    hbuf[8] <= hexc(vd_tiny[7:4]);   hbuf[9] <= hexc(vd_tiny[3:0]);
                    hbuf[10] <= " "; hbuf[11] <= "H"; hbuf[12] <= "=";
                    hbuf[13] <= hexc(vd_hsync[15:12]); hbuf[14] <= hexc(vd_hsync[11:8]);
                    hbuf[15] <= hexc(vd_hsync[7:4]);   hbuf[16] <= hexc(vd_hsync[3:0]);
                    hbuf[17] <= " "; hbuf[18] <= "M"; hbuf[19] <= "=";
                    hbuf[20] <= hexc(vd_mid[15:12]); hbuf[21] <= hexc(vd_mid[11:8]);
                    hbuf[22] <= hexc(vd_mid[7:4]);   hbuf[23] <= hexc(vd_mid[3:0]);
                    hbuf[24] <= " "; hbuf[25] <= "B"; hbuf[26] <= "=";
                    hbuf[27] <= hexc(vd_broad[15:12]); hbuf[28] <= hexc(vd_broad[11:8]);
                    hbuf[29] <= hexc(vd_broad[7:4]);   hbuf[30] <= hexc(vd_broad[3:0]);
                    hbuf[31] <= " "; hbuf[32] <= "P"; hbuf[33] <= "=";
                    hbuf[34] <= hexc(vd_plast[15:12]); hbuf[35] <= hexc(vd_plast[11:8]);
                    hbuf[36] <= hexc(vd_plast[7:4]);   hbuf[37] <= hexc(vd_plast[3:0]);
                    hbuf[38] <= " "; hbuf[39] <= "P"; hbuf[40] <= "N"; hbuf[41] <= "=";
                    hbuf[42] <= hexc(vd_pmin[15:12]); hbuf[43] <= hexc(vd_pmin[11:8]);
                    hbuf[44] <= hexc(vd_pmin[7:4]);   hbuf[45] <= hexc(vd_pmin[3:0]);
                    hbuf[46] <= " "; hbuf[47] <= "P"; hbuf[48] <= "X"; hbuf[49] <= "=";
                    hbuf[50] <= hexc(vd_pmax[15:12]); hbuf[51] <= hexc(vd_pmax[11:8]);
                    hbuf[52] <= hexc(vd_pmax[7:4]);   hbuf[53] <= hexc(vd_pmax[3:0]);
                    // The slicer levels and the learned line period. Without
                    // these the previous separator's failure was invisible: its
                    // slicing level sat at 105, inside the picture, and nothing
                    // in the telemetry said so. SL is the slice, TP the sync
                    // tip, BK the blanking level, LP the NCO period in 1/256
                    // of a sample (0x4F700 = 1271.00).
                    hbuf[54] <= " "; hbuf[55] <= "S"; hbuf[56] <= "L"; hbuf[57] <= "=";
                    hbuf[58] <= hexc(sep_slice[7:4]); hbuf[59] <= hexc(sep_slice[3:0]);
                    hbuf[60] <= " "; hbuf[61] <= "T"; hbuf[62] <= "P"; hbuf[63] <= "=";
                    hbuf[64] <= hexc(sep_tip[7:4]);   hbuf[65] <= hexc(sep_tip[3:0]);
                    hbuf[66] <= " "; hbuf[67] <= "B"; hbuf[68] <= "K"; hbuf[69] <= "=";
                    hbuf[70] <= hexc(sep_blank[7:4]); hbuf[71] <= hexc(sep_blank[3:0]);
                    hbuf[72] <= " "; hbuf[73] <= "L"; hbuf[74] <= "P"; hbuf[75] <= "=";
                    hbuf[76] <= hexc(sep_period[19:16]);
                    hbuf[77] <= hexc(sep_period[15:12]); hbuf[78] <= hexc(sep_period[11:8]);
                    hbuf[79] <= hexc(sep_period[7:4]);   hbuf[80] <= hexc(sep_period[3:0]);
                    hbuf[81] <= 8'h0D; hbuf[82] <= 8'h0A;

                    // ---- what reaches the display path ----
                    obuf[0]<="V"; obuf[1]<="O"; obuf[2]<=":"; obuf[3]<=" ";
                    obuf[4]<="P"; obuf[5]<="X"; obuf[6]<="=";
                    obuf[7]  <= hexc(vo_pix_cnt[15:12]); obuf[8]  <= hexc(vo_pix_cnt[11:8]);
                    obuf[9]  <= hexc(vo_pix_cnt[7:4]);   obuf[10] <= hexc(vo_pix_cnt[3:0]);
                    obuf[11]<=" "; obuf[12]<="L"; obuf[13]<="N"; obuf[14]<="=";
                    obuf[15] <= hexc(vo_line_cnt[15:12]); obuf[16] <= hexc(vo_line_cnt[11:8]);
                    obuf[17] <= hexc(vo_line_cnt[7:4]);   obuf[18] <= hexc(vo_line_cnt[3:0]);
                    obuf[19]<=" "; obuf[20]<="F"; obuf[21]<="D"; obuf[22]<="=";
                    obuf[23] <= hexc(vo_field_cnt[15:12]); obuf[24] <= hexc(vo_field_cnt[11:8]);
                    obuf[25] <= hexc(vo_field_cnt[7:4]);   obuf[26] <= hexc(vo_field_cnt[3:0]);
                    obuf[27]<=" "; obuf[28]<="A"; obuf[29]<="V"; obuf[30]<="=";
                    obuf[31] <= hexc(vo_avg[7:4]); obuf[32] <= hexc(vo_avg[3:0]);
                    obuf[33]<=" "; obuf[34]<="Y"; obuf[35]<="L"; obuf[36]<="=";
                    obuf[37] <= hexc({2'b0, ntsc_line[9:8]});
                    obuf[38] <= hexc(ntsc_line[7:4]); obuf[39] <= hexc(ntsc_line[3:0]);
                    obuf[40]<=" "; obuf[41]<="P"; obuf[42]<="Y"; obuf[43]<="=";
                    obuf[44] <= hexc({3'b0, ntsc_py[8]});
                    obuf[45] <= hexc(ntsc_py[7:4]); obuf[46] <= hexc(ntsc_py[3:0]);
                    obuf[47] <= " ";
                    // AGC state: gain in dB and the level it is regulating on
                    obuf[48] <= "G"; obuf[49] <= "=";
                    obuf[50] <= hexc({1'b0, agc_db[6:4]});
                    obuf[51] <= hexc(agc_db[3:0]);
                    obuf[52] <= " ";
                    obuf[53] <= " ";
                    obuf[54] <= "R"; obuf[55] <= "S"; obuf[56] <= "=";
                    obuf[57] <= hexc(rf_rssi[15:12]); obuf[58] <= hexc(rf_rssi[11:8]);
                    obuf[59] <= hexc(rf_rssi[7:4]);   obuf[60] <= hexc(rf_rssi[3:0]);
                    obuf[61] <= 8'h0A;
                end
            end else if (txr && !txv) begin
                txd <= (pmode == 4'd0) ? sbuf[pidx[4:0]] :
                       (pmode == 4'd1) ? log_char :
                       (pmode == 4'd2) ? dbuf[pidx[5:0]] :
                       (pmode == 4'd3) ? mbuf[pidx[5:0]] :
                       (pmode == 4'd4) ? fbuf[pidx[5:0]] :
                       (pmode == 4'd5) ? vbuf_fpv[pidx[5:0]] :
                       (pmode == 4'd6) ? gbuf[pidx[5:0]] :
                       // hbuf is the one buffer longer than 64 characters, so it
                       // needs seven index bits; six wrap and the line restarts
                       // in the middle of itself.
                       (pmode == 4'd7) ? hbuf[pidx[6:0]] : obuf[pidx[5:0]];
                txv <= 1'b1;
                if (pidx == ((pmode == 4'd0) ? SLEN-1 :
                             (pmode == 4'd1) ? 9'd257 :
                             (pmode == 4'd2) ? DLEN-1 :
                             (pmode == 4'd3) ? MLEN-1 :
                             (pmode == 4'd4) ? FLEN-1 :
                             (pmode == 4'd5) ? VLEN-1 :
                             (pmode == 4'd6) ? GLEN-1 :
                             (pmode == 4'd7) ? HLEN-1 : OLEN-1))
                    printing <= 1'b0;
                else pidx <= pidx + 9'd1;
            end
        end
    end

    // A carrier this weak is indistinguishable from noise: the measured floor
    // sits near 0x0190 and real video ran 0x0B00 and above.
    wire carrier_ok = (rf_rssi > 16'h0600);

    // ---------------- Stage 5: HDMI display output ----------------
    // 640x480@60: its 31500 Hz line rate is twice the NTSC line rate, so each
    // captured line is shown twice and no full frame buffer is needed.
    wire serial_clk, pll_lock, pix_clk;

    TMDS_rPLL u_tmds_pll (
        .clkin  (clk27),
        .clkout (serial_clk),
        .lock   (pll_lock)
    );

    CLKDIV u_clkdiv (
        .RESETN (pll_lock),
        .HCLKIN (serial_clk),
        .CLKOUT (pix_clk),
        .CALIB  (1'b1)
    );
    defparam u_clkdiv.DIV_MODE = "5";
    defparam u_clkdiv.GSREN    = "false";

    // release the pixel domain only once the PLL is locked
    reg [3:0] pixrst_sr;
    always @(posedge pix_clk or negedge pll_lock) begin
        if (!pll_lock) pixrst_sr <= 4'd0;
        else           pixrst_sr <= {pixrst_sr[2:0], 1'b1};
    end
    wire pix_rst_n = pixrst_sr[3] & rst_n_c;

    wire [7:0] vo_r, vo_g, vo_b;
    wire       vo_de, vo_hs, vo_vs;

    fpv_video_out u_vout (
        .cap_clk    (clk27),
        .cap_rst_n  (rst_n_c),
        .cap_x      (ntsc_px),
        .cap_y      (ntsc_py),
        .cap_val    (ntsc_pval),
        .cap_valid  (ntsc_pval_valid),
        .cap_hsync  (ntsc_ladv),
        .cap_vsync  (ntsc_vsync),
        .cap_locked (ntsc_locked & carrier_ok),
        .pix_clk    (pix_clk),
        .pix_rst_n  (pix_rst_n),
        .o_r        (vo_r), .o_g(vo_g), .o_b(vo_b),
        .o_de       (vo_de), .o_hs(vo_hs), .o_vs(vo_vs)
    );

    DVI_TX_Top u_dvi (
        .I_rst_n       (pix_rst_n),
        .I_serial_clk  (serial_clk),
        .I_rgb_clk     (pix_clk),
        .I_rgb_vs      (vo_vs),
        .I_rgb_hs      (vo_hs),
        .I_rgb_de      (vo_de),
        .I_rgb_r       (vo_r),
        .I_rgb_g       (vo_g),
        .I_rgb_b       (vo_b),
        .O_tmds_clk_p  (tmds_clk_p),
        .O_tmds_clk_n  (tmds_clk_n),
        .O_tmds_data_p (tmds_data_p),
        .O_tmds_data_n (tmds_data_n)
    );

    // ---------------- raw IQ capture for off-line analysis ----------------
    // While the dumper is busy it owns the UART; telemetry resumes afterwards.
    // sticky boundary flags, cleared as each pixel is captured
    reg vd_hs_s, vd_vs_s;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin vd_hs_s <= 1'b0; vd_vs_s <= 1'b0; end
        else begin
            if (ntsc_ladv)  vd_hs_s <= 1'b1;
            if (ntsc_vsync) vd_vs_s <= 1'b1;
            if (ntsc_pval_valid) begin vd_hs_s <= 1'b0; vd_vs_s <= 1'b0; end
        end
    end

    field_dump u_fdump (
        .clk         (clk27),
        .rst_n       (rst_n_c),
        .pix_x       (ntsc_px),
        .pix_y       (ntsc_py),
        .pix_val     (ntsc_pval),
        .pix_valid   (ntsc_pval_valid),
        .field_start (ntsc_vsync),
        // Disarmed. A whole field is 184 kB, which at 115200 is sixteen seconds
        // during which dump_busy mutes all telemetry - and the dumper restarts on
        // every vsync, so with the scanner sweeping and noise producing vsyncs
        // the UART never gets back to the telemetry at all. Re-arm with
        // (step_ok[11] & dump_arm) when a field capture is actually wanted.
        .arm         (1'b0),
        .busy        (dump_busy),
        .tx_data     (dump_data),
        .tx_valid    (dump_valid),
        .tx_ready    (txr)
    );

    uart_tx #(.CLK_FRE(27), .BAUD_RATE(115200)) u_tx (
        .clk(clk27), .rst_n(rst_n_c),
        .tx_data(dump_busy ? dump_data : txd),
        .tx_data_valid(dump_busy ? dump_valid : txv),
        .tx_data_ready(txr), .tx_pin(uart_tx), .tx_busy());

    reg [24:0] hb27;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) hb27 <= 25'd0; else hb27 <= hb27 + 25'd1;
    end
    assign led = {~is_scanning, ~ntsc_locked, ~step_ok[11], ~hb27[24]};

endmodule
