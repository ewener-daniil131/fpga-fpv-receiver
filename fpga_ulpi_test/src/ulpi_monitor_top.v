// ULPI monitor with UART debug output - Tang Primer 20K Dock (USB3317).
//
// Debug path: FPGA M11 -> Dock debug chip -> host /dev/ttyUSB1 @ 115200 8N1.
// The UART is clocked from the board's own 27MHz oscillator (H11), never from
// ulpi_clk, so the debug channel survives the PHY stopping its CLKOUT.
//
// Fixes over the previous revision:
//  * The PHY now gets a real hardware reset. RESETB (F10) is driven from the
//    27MHz domain - it must be, because asserting RESETB stops CLKOUT, so a
//    reset pulse timed in the ulpi_clk domain would freeze itself mid-pulse.
//  * The register-engine request is a level with a done-handshake instead of
//    a one-cycle pulse that could be dropped while the bus was busy, which
//    deadlocked the sequencer.
//  * dir/nxt levels and edge counters are sampled directly in the 27MHz
//    domain, so pin activity is visible even if the ULPI FSM is wedged.
//
// Line: V=xxxx P=xxxx F=xx O=xx R=xx S=x U=x C=x D=x N=x DE=x NE=x

module ulpi_monitor_top (
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
    // 27MHz domain: reset generation for the PHY, liveness counters, UART
    // =====================================================================
    reg [1:0] rst_sync_c;
    wire      rst_n_c = rst_sync_c[1];
    always @(posedge clk27 or negedge rst_n) begin
        if (!rst_n) rst_sync_c <= 2'b00;
        else        rst_sync_c <= {rst_sync_c[0], 1'b1};
    end

    // Hold RESETB low ~1ms after configuration, then release. Active low.
    reg [15:0] phy_rst_cnt;
    reg        phy_rst_n;
    localparam [15:0] PHY_RST_LEN = 16'd27_000; // ~1ms @27MHz
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            phy_rst_cnt <= 16'd0;
            phy_rst_n   <= 1'b0;
        end else if (phy_rst_cnt != PHY_RST_LEN) begin
            phy_rst_cnt <= phy_rst_cnt + 16'd1;
            phy_rst_n   <= 1'b0;
        end else begin
            phy_rst_n   <= 1'b1;
        end
    end
    assign ulpi_rst = phy_rst_n;

    // =====================================================================
    // ULPI clock domain
    //
    // Clocked on the INVERTED ulpi_clk, matching Sipeed's own working
    // usb_serial design for this board (wire int_clk = ~ulpi_clk). The PHY
    // samples the Link's data on its rising edge; driving from the falling
    // edge gives half a clock (8.3ns) of settling margin and is what makes
    // the PHY latch our TX CMD bytes reliably.
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

    reg [7:0] last_rxcmd;
    reg       rxcmd_seen;
    reg       burst_captured;

    // ---- register access engine: level request + done handshake ----
    reg        rq_req;      // held until rq_done
    reg        rq_is_wr;
    reg  [5:0] rq_addr;
    reg  [7:0] rq_wdata;
    reg  [7:0] rq_rdata;
    reg        rq_done;
    reg        rq_abort;   // sequencer watchdog forces the engine back to idle
    reg  [3:0] rd_ok;      // completed accesses, for visibility
    reg        cmd_first;  // first cycle of a TX CMD: NXT cannot be ours yet

    localparam E_IDLE = 3'd0, E_CMD = 3'd1, E_WDATA = 3'd2, E_WSTP = 3'd3,
               E_TURN = 3'd4, E_CAP = 3'd5, E_REL  = 3'd6, E_ACKW = 3'd7;
    reg [2:0] estate;

    // In-FPGA logic analyser: records {dir,nxt,data} for 32 consecutive
    // ulpi_clk cycles starting at the first command issued AFTER the first
    // full polling pass has succeeded - i.e. exactly the access that hangs.
    reg [9:0] tr [0:31];
    reg [4:0] tr_idx;
    reg       tr_active;
    reg       tr_full;
    wire      cmd_start = (estate == E_IDLE) && rq_req && !ulpi_dir && !dir_d;

    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            estate <= E_IDLE; dir_d <= 1'b1; data_oe <= 1'b0;
            data_out <= 8'h00; stp_r <= 1'b0; rq_done <= 1'b0;
            rq_rdata <= 8'h00; last_rxcmd <= 8'h00; rxcmd_seen <= 1'b0;
            burst_captured <= 1'b0; rd_ok <= 4'd0; cmd_first <= 1'b0;
            tr_idx <= 5'd0; tr_active <= 1'b0; tr_full <= 1'b0;
        end else begin
            dir_d   <= ulpi_dir;

            if (tr_active) begin
                tr[tr_idx] <= {ulpi_dir, ulpi_nxt, ulpi_data};
                if (tr_idx == 5'd31) begin
                    tr_active <= 1'b0;
                    tr_full   <= 1'b1;
                end else begin
                    tr_idx <= tr_idx + 5'd1;
                end
            end else if (!tr_full && cmd_start && (ridx == 3'd4)) begin
                tr_active <= 1'b1;
                tr_idx    <= 5'd0;
            end
            stp_r   <= 1'b0;
            rq_done <= 1'b0;

            if (ulpi_dir && dir_d) begin
                if (!burst_captured) begin
                    last_rxcmd     <= ulpi_data;
                    rxcmd_seen     <= 1'b1;
                    burst_captured <= 1'b1;
                end
            end else if (!ulpi_dir) begin
                burst_captured <= 1'b0;
            end

            // Recovery: if an access timed out, the PHY is most likely stuck
            // part-way through a transfer it believes the Link started (it
            // keeps asserting NXT asking for bytes and never turns the bus
            // around). STP terminates the current transfer, so pulse it while
            // driving IDLE to resynchronise, then retry.
            if (rq_abort) begin
                estate <= E_IDLE;
                if (!ulpi_dir) begin
                    data_oe  <= 1'b1;
                    data_out <= 8'h00;
                    stp_r    <= 1'b1;
                end else begin
                    data_oe <= 1'b0;
                end
            end else
            case (estate)
                // While the Link owns the bus it must actively drive the IDLE
                // byte (0x00). Leaving it floating lets the PHY's interface
                // protect pulls put a random value on DATA, which the PHY
                // latches as a bogus TX CMD - it then sits waiting for packet
                // bytes and an STP that never come, asserting NXT, and stops
                // servicing register commands entirely.
                E_IDLE: begin
                    // Take the bus back the moment the PHY releases it. A
                    // second floating cycle leaves the just-returned register
                    // value sitting on DATA, and the PHY samples it as a TX
                    // CMD (Function Control reads back 0x41 = TX PID OUT),
                    // after which it waits for packet bytes that never come.
                    data_oe <= !ulpi_dir;
                    if (!ulpi_dir && !dir_d) begin
                        // Only start while NXT is idle: a command issued on
                        // top of a still-asserted NXT from the previous
                        // transaction gets that stale NXT mistaken for its
                        // own acknowledge.
                        if (rq_req && !ulpi_nxt) begin
                            data_out  <= {(rq_is_wr ? 2'b10 : 2'b11), rq_addr};
                            cmd_first <= 1'b1;
                            estate    <= E_CMD;
                        end else begin
                            data_out <= 8'h00;
                        end
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
                            data_out <= 8'h00;  // never re-drive a stale cmd
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
                    data_oe <= 1'b0; rq_done <= 1'b1; estate <= E_ACKW;
                end
                E_TURN: begin
                    data_oe <= 1'b0;
                    if (ulpi_dir) estate <= E_CAP;
                end
                E_CAP: begin
                    rq_rdata <= ulpi_data; estate <= E_REL;
                end
                E_REL: begin
                    // Grab the bus in the very cycle the PHY drops DIR,
                    // so only the one legitimate turnaround cycle floats
                    // and the returned register value is never left on DATA
                    // long enough for the PHY to read it back as a TX CMD.
                    if (!ulpi_dir) begin
                        data_oe  <= 1'b1;
                        data_out <= 8'h00;
                        rq_done  <= 1'b1;
                        estate   <= E_ACKW;
                        rd_ok    <= rd_ok + 4'd1;
                    end
                end

                // Wait for the requester to drop rq_req before accepting a new
                // command. Without this the engine re-launches the same access
                // one cycle after finishing it (rq_req is still high because
                // the sequencer only reacts to rq_done on the next cycle),
                // which desynchronises engine and sequencer and ends with a
                // command being driven into the middle of a PHY response.
                E_ACKW: begin
                    data_oe <= !ulpi_dir;
                    if (!ulpi_dir) data_out <= 8'h00;
                    if (!rq_req) estate <= E_IDLE;
                end
                default: estate <= E_IDLE;
            endcase
        end
    end

    // ---- polling sequencer with a watchdog so one stuck access can't
    //      wedge the whole monitor ----
    localparam S_POR = 3'd0, S_RD = 3'd1, S_RDW = 3'd2, S_PUB = 3'd3,
               S_GAP = 3'd4;

    reg [2:0]  state;
    reg [6:0]  gap_cnt;   // idle cycles between register accesses
    reg [23:0] por_cnt;
    reg [2:0]  ridx;
    reg [7:0]  regval [0:5];
    reg [21:0] pub_cnt;
    reg [63:0] snap;
    reg        snap_tog;
    reg [23:0] ucnt;
    reg [19:0] wdog;

    localparam [23:0] POR_STABLE = 24'd600_000;   // ~10ms @60MHz
    localparam [21:0] PUB_PERIOD = 22'd3_000_000; // ~50ms @60MHz
    localparam [19:0] WDOG_MAX   = 20'd600_000;   // ~10ms @60MHz

    wire [5:0] addr_of_idx = (ridx == 3'd0) ? 6'h00 :
                             (ridx == 3'd1) ? 6'h01 :
                             (ridx == 3'd2) ? 6'h02 :
                             (ridx == 3'd3) ? 6'h03 :
                             (ridx == 3'd4) ? 6'h04 : 6'h0A;

    integer i;
    always @(posedge ulpi_clk_i or negedge rst_n_u) begin
        if (!rst_n_u) begin
            state <= S_POR; por_cnt <= 24'd0; ridx <= 3'd0; gap_cnt <= 7'd0;
            rq_req <= 1'b0; rq_is_wr <= 1'b0; rq_addr <= 6'd0;
            rq_wdata <= 8'd0; pub_cnt <= 22'd0; snap <= 64'd0;
            snap_tog <= 1'b0; ucnt <= 24'd0; wdog <= 20'd0; rq_abort <= 1'b0;
            for (i = 0; i < 6; i = i + 1) regval[i] <= 8'h00;
        end else begin
            ucnt     <= ucnt + 24'd1;
            rq_abort <= 1'b0;
            if (pub_cnt != PUB_PERIOD) pub_cnt <= pub_cnt + 22'd1;

            case (state)
                S_POR: begin
                    if (ulpi_dir) por_cnt <= 24'd0;
                    else if (por_cnt == POR_STABLE) state <= S_RD;
                    else por_cnt <= por_cnt + 24'd1;
                end
                S_RD: begin
                    rq_addr  <= addr_of_idx;
                    rq_is_wr <= 1'b0;
                    rq_req   <= 1'b1;
                    wdog     <= 20'd0;
                    state    <= S_RDW;
                end
                S_RDW: begin
                    wdog <= wdog + 20'd1;
                    if (rq_done) begin
                        rq_req       <= 1'b0;
                        regval[ridx] <= rq_rdata;
                        gap_cnt      <= 7'd0;
                        if (ridx == 3'd5) begin
                            ridx  <= 3'd0;
                            state <= S_PUB;
                        end else begin
                            ridx  <= ridx + 3'd1;
                            state <= S_GAP;
                        end
                    end else if (wdog == WDOG_MAX) begin
                        // access never completed - give up on this register,
                        // reset the engine, and keep the loop alive.
                        rq_req       <= 1'b0;
                        rq_abort     <= 1'b1;
                        regval[ridx] <= 8'hEE;
                        gap_cnt      <= 7'd0;
                        if (ridx == 3'd5) begin
                            ridx  <= 3'd0;
                            state <= S_PUB;
                        end else begin
                            ridx  <= ridx + 3'd1;
                            state <= S_GAP;
                        end
                    end
                end
                // Let the bus go fully idle between accesses. Issuing the
                // next command immediately after the previous one completes
                // lands it on the tail of that transaction (e.g. on a still
                // asserted NXT), which the PHY then mis-parses.
                S_GAP: begin
                    if (gap_cnt == 7'd64) begin
                        if (!ulpi_nxt && !ulpi_dir) state <= S_RD;
                    end else begin
                        gap_cnt <= gap_cnt + 7'd1;
                    end
                end

                S_PUB: begin
                    if (pub_cnt == PUB_PERIOD) begin
                        pub_cnt  <= 22'd0;
                        // exactly 64 bits: 6 registers + rxcmd + status
                        snap     <= {regval[0], regval[1],   // [63:48]
                                     regval[2], regval[3],   // [47:32]
                                     regval[4], regval[5],   // [31:16]
                                     last_rxcmd,             // [15:8]
                                     rd_ok, 1'b0, estate};   // [7:0]
                        snap_tog <= ~snap_tog;
                        gap_cnt  <= 7'd0;
                        state    <= S_GAP;
                    end
                end
                default: state <= S_POR;
            endcase
        end
    end

    // =====================================================================
    // 27MHz domain: CDC, pin observation, formatting
    // =====================================================================
    reg [2:0] tog_sync;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) tog_sync <= 3'd0;
        else          tog_sync <= {tog_sync[1:0], snap_tog};
    end
    wire snap_event = tog_sync[2] ^ tog_sync[1];

    reg [63:0] snap_c;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) snap_c <= 64'd0;
        else if (snap_event) snap_c <= snap;
    end

    // Direct pin observation, independent of the ULPI FSM.
    reg [2:0] alive_sync, dir_sync, nxt_sync;
    reg [3:0] uedge, dedge, nedge;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            alive_sync <= 3'd0; dir_sync <= 3'd0; nxt_sync <= 3'd0;
            uedge <= 4'd0; dedge <= 4'd0; nedge <= 4'd0;
        end else begin
            alive_sync <= {alive_sync[1:0], ucnt[18]};
            dir_sync   <= {dir_sync[1:0],   ulpi_dir};
            nxt_sync   <= {nxt_sync[1:0],   ulpi_nxt};
            if (alive_sync[2] ^ alive_sync[1]) uedge <= uedge + 4'd1;
            if (dir_sync[2]   ^ dir_sync[1])   dedge <= dedge + 4'd1;
            if (nxt_sync[2]   ^ nxt_sync[1])   nedge <= nedge + 4'd1;
        end
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

    localparam LINE_LEN = 52;
    reg  [7:0] linebuf [0:LINE_LEN-1];
    reg  [7:0] line_idx;
    reg        printing;
    reg        pmode;        // 0 = status line, 1 = trace dump
    reg  [7:0] tx_data;
    reg        tx_valid;
    wire       tx_ready;
    reg        pending;

    // trace dump: 32 entries x {2 bits dir/nxt, 2 hex data, space} = 128 chars
    wire [4:0] tr_e   = line_idx[6:2];
    wire [1:0] tr_s   = line_idx[1:0];
    wire [9:0] tr_v   = tr[tr_e];
    wire [7:0] trace_char =
        (line_idx == 8'd128) ? 8'h0D :
        (line_idx == 8'd129) ? 8'h0A :
        (tr_s == 2'd0) ? hexc({2'b00, tr_v[9:8]}) :
        (tr_s == 2'd1) ? hexc(tr_v[7:4]) :
        (tr_s == 2'd2) ? hexc(tr_v[3:0]) : " ";

    wire [7:0] s_r0   = snap_c[63:56];  // VendorID Low
    wire [7:0] s_r1   = snap_c[55:48];  // VendorID High
    wire [7:0] s_r2   = snap_c[47:40];  // ProductID Low
    wire [7:0] s_r3   = snap_c[39:32];  // ProductID High
    wire [7:0] s_r4   = snap_c[31:24];  // Function Control
    wire [7:0] s_otg  = snap_c[23:16];  // OTG Control
    wire [7:0] s_rx   = snap_c[15:8];   // last RX CMD
    wire [3:0] s_rdok = snap_c[7:4];
    wire [3:0] s_est  = snap_c[3:0];

    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) begin
            printing <= 1'b0; line_idx <= 8'd0; tx_valid <= 1'b0;
            tx_data  <= 8'd0; pending  <= 1'b0; pmode <= 1'b0;
        end else begin
            tx_valid <= 1'b0;
            if (do_print) pending <= 1'b1;

            if (!printing) begin
                if (pending) begin
                    pending  <= 1'b0;
                    printing <= 1'b1;
                    line_idx <= 8'd0;
                    pmode    <= tr_full ? ~pmode : 1'b0;
                    linebuf[0]  <= "V"; linebuf[1] <= "=";
                    linebuf[2]  <= hexc(s_r1[7:4]); linebuf[3] <= hexc(s_r1[3:0]);
                    linebuf[4]  <= hexc(s_r0[7:4]); linebuf[5] <= hexc(s_r0[3:0]);
                    linebuf[6]  <= " ";
                    linebuf[7]  <= "P"; linebuf[8] <= "=";
                    linebuf[9]  <= hexc(s_r3[7:4]); linebuf[10] <= hexc(s_r3[3:0]);
                    linebuf[11] <= hexc(s_r2[7:4]); linebuf[12] <= hexc(s_r2[3:0]);
                    linebuf[13] <= " ";
                    linebuf[14] <= "F"; linebuf[15] <= "=";
                    linebuf[16] <= hexc(s_r4[7:4]); linebuf[17] <= hexc(s_r4[3:0]);
                    linebuf[18] <= " ";
                    linebuf[19] <= "O"; linebuf[20] <= "=";
                    linebuf[21] <= hexc(s_otg[7:4]); linebuf[22] <= hexc(s_otg[3:0]);
                    linebuf[23] <= " ";
                    linebuf[24] <= "R"; linebuf[25] <= "=";
                    linebuf[26] <= hexc(s_rx[7:4]); linebuf[27] <= hexc(s_rx[3:0]);
                    linebuf[28] <= " ";
                    linebuf[29] <= "R"; linebuf[30] <= "D"; linebuf[31] <= "=";
                    linebuf[32] <= hexc(s_rdok);
                    linebuf[33] <= " ";
                    linebuf[34] <= "E"; linebuf[35] <= "=";
                    linebuf[36] <= hexc(s_est);
                    linebuf[37] <= " ";
                    linebuf[38] <= "D"; linebuf[39] <= "E"; linebuf[40] <= "=";
                    linebuf[41] <= hexc(dedge);
                    linebuf[42] <= " ";
                    linebuf[43] <= "C"; linebuf[44] <= "=";
                    linebuf[45] <= hexc(uedge);
                    linebuf[46] <= " "; linebuf[47] <= " ";
                    linebuf[48] <= " "; linebuf[49] <= " ";
                    linebuf[50] <= 8'h0D; linebuf[51] <= 8'h0A;
                end
            end else if (tx_ready && !tx_valid) begin
                tx_data  <= pmode ? trace_char : linebuf[line_idx[5:0]];
                tx_valid <= 1'b1;
                if (line_idx == (pmode ? 8'd129 : LINE_LEN-1)) printing <= 1'b0;
                else line_idx <= line_idx + 8'd1;
            end
        end
    end

    uart_tx #(.CLK_FRE(27), .BAUD_RATE(115200)) u_tx (
        .clk           (clk27),
        .rst_n         (rst_n_c),
        .tx_data       (tx_data),
        .tx_data_valid (tx_valid),
        .tx_data_ready (tx_ready),
        .tx_pin        (uart_tx),
        .tx_busy       ()
    );

    reg [24:0] hb27;
    always @(posedge clk27 or negedge rst_n_c) begin
        if (!rst_n_c) hb27 <= 25'd0;
        else          hb27 <= hb27 + 25'd1;
    end
    assign led = {4{hb27[24]}};

endmodule
