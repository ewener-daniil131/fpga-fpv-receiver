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

module ulpi_hs_test (
    input  wire       clk27,
    input  wire       rst_n,
    input  wire       ulpi_clk,
    input  wire       ulpi_dir,
    input  wire       ulpi_nxt,
    output wire       ulpi_stp,
    output wire       ulpi_rst,
    inout  wire [7:0] ulpi_data,
    output wire       uart_tx,
    output wire [3:0] led
);

    // ---------------- 27MHz domain: PHY hardware reset ----------------
    reg [1:0] rst_sync_c;
    wire      rst_n_c = rst_sync_c[1];
    always @(posedge clk27 or negedge rst_n) begin
        if (!rst_n) rst_sync_c <= 2'b00;
        else        rst_sync_c <= {rst_sync_c[0], 1'b1};
    end

    reg [15:0] phy_rst_cnt;
    reg        phy_rst_n;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            phy_rst_cnt <= 16'd0; phy_rst_n <= 1'b0;
        end else if (phy_rst_cnt != 16'd27_000) begin
            phy_rst_cnt <= phy_rst_cnt + 16'd1; phy_rst_n <= 1'b0;
        end else phy_rst_n <= 1'b1;
    end
    assign ulpi_rst = phy_rst_n;

    // ---------------- ULPI domain ----------------
    wire ulpi_clk_i = ~ulpi_clk;

    reg [1:0] rst_sync_u;
    wire      rst_n_u = rst_sync_u[1] & phy_rst_n;
    always @(posedge ulpi_clk_i or negedge rst_n) begin
        if (!rst_n) rst_sync_u <= 2'b00;
        else        rst_sync_u <= {rst_sync_u[0], 1'b1};
    end

    reg        dir_d, data_oe, stp_r;
    reg  [7:0] data_out;
    assign ulpi_data = data_oe ? data_out : 8'hzz;
    assign ulpi_stp  = stp_r;

    // request interface: mode 0 = reg read, 1 = reg write, 2 = chirp stream
    reg        rq_req;
    reg  [1:0] rq_mode;
    reg  [5:0] rq_addr;
    reg  [7:0] rq_wdata;
    reg  [7:0] rq_rdata;
    reg  [7:0] rq_b0, rq_b1; // token payload (frame number + CRC5)
    reg [11:0] rq_len;      // bytes per chirp segment
    reg  [7:0] rq_segs;     // number of alternating segments in one packet
    reg        rq_done, rq_abort;
    reg        cmd_first;
    reg [11:0] tx_cnt;
    reg  [7:0] seg_cnt;

    localparam M_READ = 2'd0, M_WRITE = 2'd1, M_CHIRP = 2'd2,
               M_TOKEN = 2'd3;

    localparam E_IDLE  = 4'd0,  E_CMD   = 4'd1,  E_WDATA = 4'd2,
               E_WSTP  = 4'd3,  E_TURN  = 4'd4,  E_CAP   = 4'd5,
               E_REL   = 4'd6,  E_ACKW  = 4'd7,  E_TXCMD = 4'd8,
               E_TXDAT = 4'd9,  E_TXSTP = 4'd10, E_TKCMD = 4'd11,
               E_TKD0  = 4'd12, E_TKD1  = 4'd13;
    reg [3:0] estate;

    wire in_reg_read = (estate == E_TURN) || (estate == E_CAP) ||
                       (estate == E_REL);

    reg [7:0] rxcmd;
    reg       rxcmd_stb;
    reg       burst_captured;

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            estate <= E_IDLE; dir_d <= 1'b1; data_oe <= 1'b0;
            data_out <= 8'h00; stp_r <= 1'b0; rq_done <= 1'b0;
            rq_rdata <= 8'h00; cmd_first <= 1'b0; tx_cnt <= 12'd0;
            seg_cnt <= 8'd0;
            rxcmd <= 8'h00; rxcmd_stb <= 1'b0; burst_captured <= 1'b0;
        end else begin
            dir_d     <= ulpi_dir;
            stp_r     <= 1'b0;
            rq_done   <= 1'b0;
            rxcmd_stb <= 1'b0;

            if (ulpi_dir && dir_d) begin
                if (!burst_captured && !in_reg_read) begin
                    rxcmd <= ulpi_data; rxcmd_stb <= 1'b1;
                    burst_captured <= 1'b1;
                end
            end else if (!ulpi_dir) burst_captured <= 1'b0;

            if (rq_abort) begin
                estate <= E_IDLE;
                if (!ulpi_dir) begin
                    data_oe <= 1'b1; data_out <= 8'h00; stp_r <= 1'b1;
                end else data_oe <= 1'b0;
            end else
            case (estate)
                E_IDLE: begin
                    data_oe <= !ulpi_dir;
                    if (!ulpi_dir && !dir_d && rq_req && !ulpi_nxt) begin
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
                            default: begin
                                data_out <= 8'h45;  // TX CMD PID SOF
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
                        data_out <= rq_b0; estate <= E_TKD0;
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
    localparam H_POR    = 5'd0,  H_SETTLE = 5'd1,  H_RESET  = 5'd2,
               H_RESETW = 5'd3,  H_WAITK  = 5'd4,  H_INK    = 5'd5,
               H_HCHIRP = 5'd6,  H_HCHW   = 5'd7,  H_HSMODE = 5'd8,
               H_HSMODW = 5'd9,  H_OBS    = 5'd10, H_FS     = 5'd11,
               H_FSW    = 5'd12, H_DONE   = 5'd13, H_WAIT   = 5'd14,
               H_CLR    = 5'd15, H_SOF    = 5'd16, H_SOFW   = 5'd17;

    localparam [7:0] FN_CHIRP  = 8'h50; // HS xcvr, TermSel=0, OpMode=chirp
    localparam [7:0] FN_HS     = 8'h40; // HS xcvr, TermSel=0, OpMode=normal
    localparam [7:0] FN_FS     = 8'h41; // FS xcvr, normal - safe idle state
    localparam [11:0] CHIRP_LEN = 12'd3000; // ~50us of bytes at 60MHz
    localparam [7:0]  CHIRP_SEGS = 8'd130;  // ~6.5ms of K/J at 50us each

    reg [4:0]  hstate;
    reg [23:0] por_cnt;
    reg [19:0] wdog;
    reg [22:0] twait;
    reg [3:0]  seg;
    reg        dev_chirped;
    reg [7:0]  ls_after;
    reg [10:0] frame_no;
    reg [2:0]  uframe;
    reg [15:0] sof_cnt;    // SOFs actually accepted by the PHY

    wire [1:0] ls_now = rxcmd[1:0];

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            hstate <= H_POR; por_cnt <= 24'd0; wdog <= 20'd0; twait <= 23'd0;
            rq_req <= 1'b0; rq_mode <= M_READ; rq_addr <= 6'd0;
            rq_wdata <= 8'd0; rq_len <= 12'd1; rq_segs <= 8'd1;
            rq_abort <= 1'b0;
            log_wr <= 6'd0; log_en <= 1'b0; seg <= 4'd0;
            dev_chirped <= 1'b0; ls_after <= 8'h00;
            frame_no <= 11'd0; uframe <= 3'd0; rq_b0 <= 8'd0; rq_b1 <= 8'd0;
            sof_cnt <= 16'd0;
        end else begin
            rq_abort <= 1'b0;

            if (log_en && rxcmd_stb && (log_wr != LOG_N)) begin
                logmem[log_wr[4:0]] <= {tstamp, rxcmd};
                log_wr              <= log_wr + 6'd1;
            end

            case (hstate)
                H_POR: begin
                    if (ulpi_dir) por_cnt <= 24'd0;
                    else if (por_cnt == 24'd600_000) begin
                        twait <= 23'd0; hstate <= H_SETTLE;
                    end else por_cnt <= por_cnt + 24'd1;
                end

                H_SETTLE: begin
                    if (twait == 23'd6_000_000) begin   // ~100ms idle watch
                        log_en <= 1'b1; dev_chirped <= 1'b0;
                        hstate <= H_RESET;
                    end else twait <= twait + 23'd1;
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
                    end else if (twait == 23'd600_000) begin  // 10ms, no chirp
                        hstate <= H_FS;                        // not HS capable
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
                    if (twait == 23'd7_500) begin   // 125us @60MHz
                        twait  <= 23'd0;
                        hstate <= H_SOF;
                    end else twait <= twait + 23'd1;
                end

                H_SOF: begin
                    rq_mode <= M_TOKEN;
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
                        hstate <= H_OBS;
                    end else if (wdog == 20'd60_000) begin  // 1ms
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_OBS;
                    end
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
                        rq_req <= 1'b0; log_en <= 1'b0;
                        twait  <= 23'd0; hstate <= H_DONE;
                    end else if (wdog == 20'd600_000) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_FS;
                    end
                end

                H_DONE: begin
                    if (twait == 23'd30_000_000) begin  // hold ~500ms
                        twait <= 23'd0; hstate <= H_WAIT;
                    end else twait <= twait + 23'd1;
                end
                H_WAIT: begin
                    log_wr <= 6'd0; hstate <= H_CLR;
                end
                H_CLR: begin
                    logmem[log_wr[4:0]] <= 24'd0;
                    if (log_wr == LOG_N-1) begin
                        log_wr <= 6'd0; twait <= 23'd0; hstate <= H_SETTLE;
                    end else log_wr <= log_wr + 6'd1;
                end

                default: hstate <= H_POR;
            endcase
        end
    end

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
    reg       printing, pmode, pending;
    reg [7:0] txd;
    reg       txv;
    wire      txr;

    wire [4:0]  le  = pidx[7:3];
    wire [2:0]  lsb = pidx[2:0];
    wire [23:0] lv  = logmem[le];
    wire [7:0]  log_char =
        (pidx == 9'd256) ? 8'h0D :
        (pidx == 9'd257) ? 8'h0A :
        (lsb == 3'd0) ? hexc(lv[23:20]) :
        (lsb == 3'd1) ? hexc(lv[19:16]) :
        (lsb == 3'd2) ? hexc(lv[15:12]) :
        (lsb == 3'd3) ? hexc(lv[11:8])  :
        (lsb == 3'd4) ? ":"             :
        (lsb == 3'd5) ? hexc(lv[7:4])   :
        (lsb == 3'd6) ? hexc(lv[3:0])   : " ";

    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            printing <= 1'b0; pidx <= 9'd0; txv <= 1'b0; txd <= 8'd0;
            pending  <= 1'b0; pmode <= 1'b0;
        end else begin
            txv <= 1'b0;
            if (do_print) pending <= 1'b1;

            if (!printing) begin
                if (pending) begin
                    pending  <= 1'b0; printing <= 1'b1;
                    pidx     <= 9'd0; pmode    <= ~pmode;
                    sbuf[0]  <= "R"; sbuf[1] <= "X"; sbuf[2] <= "=";
                    sbuf[3]  <= hexc(c_rx[7:4]);  sbuf[4] <= hexc(c_rx[3:0]);
                    sbuf[5]  <= " ";
                    sbuf[6]  <= "S"; sbuf[7] <= "O"; sbuf[8] <= "F";
                    sbuf[9]  <= "=";
                    sbuf[10] <= hexc(c_sofh[7:4]); sbuf[11] <= hexc(c_sofh[3:0]);
                    sbuf[12] <= hexc(c_sofl[7:4]); sbuf[13] <= hexc(c_sofl[3:0]);
                    sbuf[14] <= " ";
                    sbuf[15] <= "C"; sbuf[16] <= "K"; sbuf[17] <= "=";
                    sbuf[16] <= c_chir ? "1" : "0";
                    sbuf[17] <= " ";
                    sbuf[18] <= "H"; sbuf[19] <= "=";
                    sbuf[20] <= hexc({3'd0, c_hs[4]});
                    sbuf[21] <= hexc(c_hs[3:0]);
                    sbuf[22] <= " "; sbuf[23] <= " "; sbuf[24] <= " ";
                    sbuf[25] <= " "; sbuf[26] <= " "; sbuf[27] <= " ";
                    sbuf[28] <= 8'h0D; sbuf[29] <= 8'h0A;
                end
            end else if (txr && !txv) begin
                txd <= pmode ? log_char : sbuf[pidx[4:0]];
                txv <= 1'b1;
                if (pidx == (pmode ? 9'd257 : SLEN-1)) printing <= 1'b0;
                else pidx <= pidx + 9'd1;
            end
        end
    end

    uart_tx #(.CLK_FRE(27), .BAUD_RATE(115200)) u_tx (
        .clk(clk27), .rst_n(rst_n_c), .tx_data(txd), .tx_data_valid(txv),
        .tx_data_ready(txr), .tx_pin(uart_tx), .tx_busy());

    reg [24:0] hb27;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) hb27 <= 25'd0; else hb27 <= hb27 + 25'd1;
    end
    assign led = {4{hb27[24]}};

endmodule
