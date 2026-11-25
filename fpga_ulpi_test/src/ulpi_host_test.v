// USB host bring-up step 1: bus reset + Hi-Speed chirp detection.
// Tang Primer 20K Dock, SMSC USB3317 ULPI PHY.
//
// Builds on the now-working ULPI link layer (see ulpi_monitor_top.v). All the
// hard-won rules are kept here:
//   * ULPI logic clocked on the INVERTED ulpi_clk (as in Sipeed's own design)
//   * the Link drives IDLE (0x00) whenever it owns the bus, and takes the bus
//     back in the same cycle DIR drops - leaving the returned register value
//     floating gets it re-sampled by the PHY as a TX CMD (0x41 = PID OUT)
//   * full req/done handshake with an explicit wait for the request to drop
//   * stale NXT from the previous transaction is never taken as an ack
//   * watchdog + STP pulse to recover a wedged PHY
//
// Sequence (ULPI spec 3.8.5.1, "Hi-Speed Detection Handshake"):
//   1. read Function Control / OTG Control, report LineState while idle
//   2. write Function Control = 0x50: XcvrSelect=00 (HS), TermSelect=0,
//      OpMode=10 (chirp), SuspendM=1  -> the PHY drives SE0 = USB reset
//   3. log every RX CMD with a timestamp for ~20ms. A Hi-Speed capable
//      device must answer with chirp K (LineState=10) within 7ms of T0.
//   4. restore Function Control = 0x41 (FS, normal) so the bus is not left
//      held in reset, then dump the log over UART forever.
//
// UART: M11 -> /dev/ttyUSB1 @ 115200 8N1, clocked from the 27MHz oscillator
// so the debug channel survives anything the PHY does.

module ulpi_host_test (
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

    // =====================================================================
    // 27MHz domain: PHY hardware reset
    // =====================================================================
    reg [1:0] rst_sync_c;
    wire      rst_n_c = rst_sync_c[1];
    always @(posedge clk27 or negedge rst_n) begin
        if (!rst_n) rst_sync_c <= 2'b00;
        else        rst_sync_c <= {rst_sync_c[0], 1'b1};
    end

    reg [15:0] phy_rst_cnt;
    reg        phy_rst_n;
    localparam [15:0] PHY_RST_LEN = 16'd27_000; // ~1ms
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            phy_rst_cnt <= 16'd0;
            phy_rst_n   <= 1'b0;
        end else if (phy_rst_cnt != PHY_RST_LEN) begin
            phy_rst_cnt <= phy_rst_cnt + 16'd1;
            phy_rst_n   <= 1'b0;
        end else begin
            phy_rst_n <= 1'b1;
        end
    end
    assign ulpi_rst = phy_rst_n;

    // =====================================================================
    // ULPI domain
    // =====================================================================
    wire ulpi_clk_i = ~ulpi_clk;

    reg [1:0] rst_sync_u;
    wire      rst_n_u = rst_sync_u[1] & phy_rst_n;
    always @(posedge ulpi_clk_i or negedge rst_n) begin
        if (!rst_n) rst_sync_u <= 2'b00;
        else        rst_sync_u <= {rst_sync_u[0], 1'b1};
    end

    reg        dir_d;
    reg        data_oe;
    reg  [7:0] data_out;
    reg        stp_r;
    assign ulpi_data = data_oe ? data_out : 8'hzz;
    assign ulpi_stp  = stp_r;

    // ---- register access engine ----
    reg        rq_req, rq_is_wr;
    reg  [5:0] rq_addr;
    reg  [7:0] rq_wdata, rq_rdata;
    reg        rq_done, rq_abort;
    reg        cmd_first;

    localparam E_IDLE = 3'd0, E_CMD = 3'd1, E_WDATA = 3'd2, E_WSTP = 3'd3,
               E_TURN = 3'd4, E_CAP = 3'd5, E_REL  = 3'd6, E_ACKW = 3'd7;
    reg [2:0] estate;

    // A DIR assertion belongs to a register read only while the engine is
    // actually waiting for one; any other DIR burst is a genuine RX CMD.
    wire in_reg_read = (estate == E_TURN) || (estate == E_CAP) ||
                       (estate == E_REL);

    reg [7:0] rxcmd;          // last genuine RX CMD
    reg       rxcmd_stb;      // one-cycle strobe when a new one arrives
    reg       burst_captured;

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            estate <= E_IDLE; dir_d <= 1'b1; data_oe <= 1'b0;
            data_out <= 8'h00; stp_r <= 1'b0; rq_done <= 1'b0;
            rq_rdata <= 8'h00; cmd_first <= 1'b0;
            rxcmd <= 8'h00; rxcmd_stb <= 1'b0; burst_captured <= 1'b0;
        end else begin
            dir_d     <= ulpi_dir;
            stp_r     <= 1'b0;
            rq_done   <= 1'b0;
            rxcmd_stb <= 1'b0;

            // genuine RX CMD capture
            if (ulpi_dir && dir_d) begin
                if (!burst_captured && !in_reg_read) begin
                    rxcmd          <= ulpi_data;
                    rxcmd_stb      <= 1'b1;
                    burst_captured <= 1'b1;
                end
            end else if (!ulpi_dir) begin
                burst_captured <= 1'b0;
            end

            if (rq_abort) begin
                estate <= E_IDLE;
                if (!ulpi_dir) begin
                    data_oe  <= 1'b1;
                    data_out <= 8'h00;
                    stp_r    <= 1'b1;   // unstick a PHY mid-transfer
                end else begin
                    data_oe <= 1'b0;
                end
            end else
            case (estate)
                E_IDLE: begin
                    data_oe <= !ulpi_dir;
                    if (!ulpi_dir && !dir_d && rq_req && !ulpi_nxt) begin
                        data_out  <= {(rq_is_wr ? 2'b10 : 2'b11), rq_addr};
                        cmd_first <= 1'b1;
                        estate    <= E_CMD;
                    end else if (!ulpi_dir) begin
                        data_out <= 8'h00;
                    end
                end
                E_CMD: begin
                    cmd_first <= 1'b0;
                    if (ulpi_dir) begin
                        data_oe <= 1'b0; estate <= E_IDLE;
                    end else if (ulpi_nxt && !cmd_first) begin
                        if (rq_is_wr) begin
                            data_out <= rq_wdata; estate <= E_WDATA;
                        end else begin
                            data_oe  <= 1'b0;
                            data_out <= 8'h00;
                            estate   <= E_TURN;
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
                    data_oe  <= 1'b1;
                    data_out <= 8'h00;
                    rq_done  <= 1'b1;
                    estate   <= E_ACKW;
                end
                E_TURN: begin
                    data_oe <= 1'b0;
                    if (ulpi_dir) estate <= E_CAP;
                end
                E_CAP: begin
                    rq_rdata <= ulpi_data;
                    estate   <= E_REL;
                end
                E_REL: begin
                    if (!ulpi_dir) begin
                        data_oe  <= 1'b1;   // reclaim the bus immediately
                        data_out <= 8'h00;
                        rq_done  <= 1'b1;
                        estate   <= E_ACKW;
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

    // ---- timestamped RX CMD event log ----
    localparam LOG_N = 32;
    reg [23:0] logmem [0:LOG_N-1];   // {time[15:0], rxcmd[7:0]}
    reg [5:0]  log_wr;
    reg        log_en;
    reg [21:0] tick_div;
    reg [15:0] tstamp;               // ~1.07us per tick

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            tick_div <= 22'd0; tstamp <= 16'd0;
        end else if (log_en) begin
            if (tick_div == 22'd63) begin
                tick_div <= 22'd0;
                tstamp   <= tstamp + 16'd1;
            end else begin
                tick_div <= tick_div + 22'd1;
            end
        end else begin
            tick_div <= 22'd0;
            tstamp   <= 16'd0;
        end
    end

    // ---- host sequencer ----
    localparam H_POR     = 4'd0,  H_RD_FN   = 4'd1,  H_RD_FNW = 4'd2,
               H_RD_OTG  = 4'd3,  H_RD_OTGW = 4'd4,  H_SETTLE = 4'd5,
               H_RESET   = 4'd6,  H_RESETW  = 4'd7,  H_CHIRP  = 4'd8,
               H_REST    = 4'd9,  H_RESTW   = 4'd10, H_DONE   = 4'd11,
               H_WAIT    = 4'd12, H_CLR     = 4'd13;

    localparam [7:0] FN_RESET_CHIRP = 8'h50; // HS xcvr, TermSel=0, OpMode=chirp
    localparam [7:0] FN_FS_NORMAL   = 8'h41; // FS xcvr, normal opmode

    reg [3:0]  hstate;
    reg [23:0] por_cnt;
    reg [19:0] wdog;
    reg [7:0]  fn_val, otg_val, ls_idle;
    reg [22:0] chirp_cnt;

    localparam [23:0] POR_STABLE  = 24'd600_000;   // ~10ms
    localparam [19:0] WDOG_MAX    = 20'd600_000;   // ~10ms
    localparam [22:0] CHIRP_WIN   = 23'd1_200_000; // ~20ms @60MHz
    localparam [22:0] SETTLE_WIN  = 23'd6_000_000; // ~100ms

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            hstate <= H_POR; por_cnt <= 24'd0; wdog <= 20'd0;
            rq_req <= 1'b0; rq_is_wr <= 1'b0; rq_addr <= 6'd0; rq_wdata <= 8'd0;
            rq_abort <= 1'b0; fn_val <= 8'h00; otg_val <= 8'h00;
            ls_idle <= 8'h00; chirp_cnt <= 23'd0;
            log_wr <= 6'd0; log_en <= 1'b0;
        end else begin
            rq_abort <= 1'b0;

            // log every genuine RX CMD while logging is armed
            if (log_en && rxcmd_stb && (log_wr != LOG_N)) begin
                logmem[log_wr[4:0]] <= {tstamp, rxcmd};
                log_wr              <= log_wr + 6'd1;
            end

            case (hstate)
                H_POR: begin
                    if (ulpi_dir) por_cnt <= 24'd0;
                    else if (por_cnt == POR_STABLE) hstate <= H_RD_FN;
                    else por_cnt <= por_cnt + 24'd1;
                end

                H_RD_FN: begin
                    rq_addr <= 6'h04; rq_is_wr <= 1'b0; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_RD_FNW;
                end
                H_RD_FNW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        fn_val <= rq_rdata; rq_req <= 1'b0; hstate <= H_RD_OTG;
                    end else if (wdog == WDOG_MAX) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_RD_FN;
                    end
                end

                H_RD_OTG: begin
                    rq_addr <= 6'h0A; rq_is_wr <= 1'b0; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_RD_OTGW;
                end
                H_RD_OTGW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        otg_val <= rq_rdata; rq_req <= 1'b0;
                        chirp_cnt <= 23'd0; hstate <= H_SETTLE;
                    end else if (wdog == WDOG_MAX) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_RD_OTG;
                    end
                end

                // watch the idle bus for a while, remember the LineState the
                // device presents before we touch anything
                H_SETTLE: begin
                    if (rxcmd_stb) ls_idle <= rxcmd;
                    if (chirp_cnt == SETTLE_WIN) begin
                        log_en <= 1'b1;          // start the clock and the log
                        hstate <= H_RESET;
                    end else chirp_cnt <= chirp_cnt + 23'd1;
                end

                // drive SE0: this is T0 of the reset
                H_RESET: begin
                    rq_addr <= 6'h04; rq_wdata <= FN_RESET_CHIRP;
                    rq_is_wr <= 1'b1; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_RESETW;
                end
                H_RESETW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; chirp_cnt <= 23'd0; hstate <= H_CHIRP;
                    end else if (wdog == WDOG_MAX) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_RESET;
                    end
                end

                // hold reset and record whatever the device does
                H_CHIRP: begin
                    if (chirp_cnt == CHIRP_WIN) hstate <= H_REST;
                    else chirp_cnt <= chirp_cnt + 23'd1;
                end

                // put the bus back to FS idle so it is not left in reset
                H_REST: begin
                    rq_addr <= 6'h04; rq_wdata <= FN_FS_NORMAL;
                    rq_is_wr <= 1'b1; rq_req <= 1'b1;
                    wdog <= 20'd0; hstate <= H_RESTW;
                end
                H_RESTW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req <= 1'b0; log_en <= 1'b0;
                        chirp_cnt <= 23'd0; hstate <= H_DONE;
                    end else if (wdog == WDOG_MAX) begin
                        rq_req <= 1'b0; rq_abort <= 1'b1; hstate <= H_REST;
                    end
                end

                // hold the finished log for a while so it can be read out,
                // then re-arm and run the whole reset/chirp probe again -
                // this makes unplug/replug a live experiment.
                H_DONE: begin
                    if (chirp_cnt == 23'd7_000_000) begin  // ~117ms
                        chirp_cnt <= 23'd0;
                        hstate    <= H_WAIT;
                    end else chirp_cnt <= chirp_cnt + 23'd1;
                end

                H_WAIT: begin
                    if (chirp_cnt == 23'd7_000_000) begin
                        log_wr    <= 6'd0;
                        ls_idle   <= 8'h00;
                        chirp_cnt <= 23'd0;
                        hstate    <= H_CLR;
                    end else chirp_cnt <= chirp_cnt + 23'd1;
                end

                // Wipe the log before re-arming. Without this a run that
                // records nothing still displays the previous run's entries,
                // which makes an unplugged-device control test meaningless.
                H_CLR: begin
                    logmem[log_wr[4:0]] <= 24'd0;
                    if (log_wr == LOG_N-1) begin
                        log_wr <= 6'd0;
                        hstate <= H_SETTLE;
                    end else log_wr <= log_wr + 6'd1;
                end
                default: hstate <= H_POR;
            endcase
        end
    end

    // =====================================================================
    // 27MHz domain: snapshot CDC + UART
    // =====================================================================
    reg [21:0] pub_cnt;
    reg        snap_tog;
    reg [39:0] snap;
    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            pub_cnt <= 22'd0; snap_tog <= 1'b0; snap <= 40'd0;
        end else begin
            if (pub_cnt == 22'd3_000_000) begin
                pub_cnt  <= 22'd0;
                snap     <= {fn_val, otg_val, ls_idle, rxcmd,
                             {4'd0, hstate}};
                snap_tog <= ~snap_tog;
            end else pub_cnt <= pub_cnt + 22'd1;
        end
    end

    reg [2:0] tog_sync;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) tog_sync <= 3'd0;
        else          tog_sync <= {tog_sync[1:0], snap_tog};
    end
    wire snap_event = tog_sync[2] ^ tog_sync[1];

    reg [39:0] snap_c;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) snap_c <= 40'd0;
        else if (snap_event) snap_c <= snap;
    end

    reg [22:0] print_tick;
    wire       do_print = (print_tick == 23'd6_750_000); // ~250ms
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) print_tick <= 23'd0;
        else if (do_print) print_tick <= 23'd0;
        else print_tick <= print_tick + 23'd1;
    end

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h41 + {4'd0, n} - 8'd10);
    endfunction

    wire [7:0] c_fn  = snap_c[39:32];
    wire [7:0] c_otg = snap_c[31:24];
    wire [7:0] c_ls  = snap_c[23:16];
    wire [7:0] c_rx  = snap_c[15:8];
    wire [3:0] c_hs  = snap_c[3:0];

    // status line: "FN=xx OTG=xx LS=xx RX=xx H=x\r\n" = 30 chars
    localparam SLEN = 30;
    reg [7:0] sbuf [0:SLEN-1];

    // log dump: 32 x "tttt:rr " = 256 chars + CRLF
    reg [8:0] pidx;
    reg       printing, pmode, pending;
    reg [7:0] txd;
    reg       txv;
    wire      txr;

    wire [4:0] le  = pidx[7:3];
    wire [2:0] lsb = pidx[2:0];
    wire [23:0] lv = logmem[le];
    wire [7:0] log_char =
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
                    pending  <= 1'b0;
                    printing <= 1'b1;
                    pidx     <= 9'd0;
                    pmode    <= ~pmode;
                    sbuf[0]  <= "F"; sbuf[1] <= "N"; sbuf[2] <= "=";
                    sbuf[3]  <= hexc(c_fn[7:4]);  sbuf[4] <= hexc(c_fn[3:0]);
                    sbuf[5]  <= " ";
                    sbuf[6]  <= "O"; sbuf[7] <= "T"; sbuf[8] <= "G";
                    sbuf[9]  <= "=";
                    sbuf[10] <= hexc(c_otg[7:4]); sbuf[11] <= hexc(c_otg[3:0]);
                    sbuf[12] <= " ";
                    sbuf[13] <= "L"; sbuf[14] <= "S"; sbuf[15] <= "=";
                    sbuf[16] <= hexc(c_ls[7:4]);  sbuf[17] <= hexc(c_ls[3:0]);
                    sbuf[18] <= " ";
                    sbuf[19] <= "R"; sbuf[20] <= "X"; sbuf[21] <= "=";
                    sbuf[22] <= hexc(c_rx[7:4]);  sbuf[23] <= hexc(c_rx[3:0]);
                    sbuf[24] <= " ";
                    sbuf[25] <= "H"; sbuf[26] <= "=";
                    sbuf[27] <= hexc(c_hs);
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
