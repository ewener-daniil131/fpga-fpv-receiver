// ULPI bring-up test #4 for Tang Primer 20K Dock (USB3317 PHY).
//
// Proven on hardware so far:
//   - RegRead:  VendorID Low (0x00) -> 0x24 (real SMSC USB-IF VID).
//   - RegWrite: Scratch reg (0x16) round-tripped 0x35.
//   - Passive RX CMD monitoring works once the Link stops fighting the PHY
//     for the bus (registered OE + turnaround guard, see below).
//   - With VBUS off: VbusState=00 (SessEnd), LineState=00 (SE0) - as expected,
//     an unpowered device cannot pull up D+.
//
// This test turns VBUS on. As host we must drive 5V on VBUS ourselves:
// OTG Control (0x0A) = 0x26 -> DrvVbus(bit5)=1, DmPulldown(bit2)=1,
// DpPulldown(bit1)=1 (host termination). Then it passively watches RX CMD
// for VbusState / LineState to change as the device powers up and pulls
// up D+ (FS/HS device -> LineState=01b).
//
// Display: all 4 LEDs flash together. Each lap starts with one LONG marker
// flash (~600ms) so group boundaries cannot be misread, then:
//   group 1: 1 flash  = no RX CMD ever seen
//            2..5     = RX CMD seen, VbusState = 0..3   (5 => VbusValid)
//   group 2: 1..4     = LineState + 1 (1=SE0, 2=D+ high, 3=D- high, 4=both)

module ulpi_vendor_id_test (
    input  wire       rst_n,       // T10, board button, active low
    input  wire       ulpi_clk,    // T15, PHY 60MHz output clock
    input  wire       ulpi_dir,    // K12
    input  wire       ulpi_nxt,    // K13
    output wire       ulpi_stp,    // K11
    output wire       ulpi_rst,    // F10, PHY RESETB (active low) - tied inactive
    inout  wire [7:0] ulpi_data,   // R12,P13,R13,T14,H13,J12,H12,G11
    output wire [3:0] led
);

    assign ulpi_rst = 1'b1; // rely on the PHY's own POR

    reg [1:0] rst_sync;
    wire      rst_n_ulpi = rst_sync[1];
    always @(posedge ulpi_clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b00;
        else        rst_sync <= {rst_sync[0], 1'b1};
    end

    // ---------------------------------------------------------------
    // ULPI bus: registered OE, never driven during/right after turnaround
    // ---------------------------------------------------------------
    reg        dir_d;
    reg        data_oe;
    reg  [7:0] data_out;
    reg        stp;
    assign ulpi_data = data_oe ? data_out : 8'hzz;
    assign ulpi_stp  = stp;

    localparam ADDR_OTG_CTRL = 6'h0A;
    localparam OTG_CTRL_VAL  = 8'h26; // DrvVbus | DmPulldown | DpPulldown

    localparam S_POR_WAIT = 3'd0;
    localparam S_IDLE     = 3'd1;
    localparam S_WCMD     = 3'd2;
    localparam S_WDATA    = 3'd3;
    localparam S_WSTP     = 3'd4;
    localparam S_MONITOR  = 3'd5;

    reg [2:0]  state;
    reg [23:0] por_wait_cnt;
    reg [7:0]  last_rxcmd;
    reg        rxcmd_seen;
    reg        burst_captured;

    localparam [23:0] POR_STABLE_CNT = 24'd600_000; // ~10ms @ 60MHz

    always @(posedge ulpi_clk or negedge rst_n_ulpi) begin
        if (!rst_n_ulpi) begin
            state          <= S_POR_WAIT;
            por_wait_cnt   <= 24'd0;
            dir_d          <= 1'b1;
            data_oe        <= 1'b0;
            data_out       <= 8'h00;
            stp            <= 1'b0;
            last_rxcmd     <= 8'h00;
            rxcmd_seen     <= 1'b0;
            burst_captured <= 1'b0;
        end else begin
            dir_d <= ulpi_dir;
            stp   <= 1'b0;

            // Capture the first real byte of every PHY burst (the RX CMD).
            if (ulpi_dir && dir_d) begin
                if (!burst_captured) begin
                    last_rxcmd     <= ulpi_data;
                    rxcmd_seen     <= 1'b1;
                    burst_captured <= 1'b1;
                end
            end else if (!ulpi_dir) begin
                burst_captured <= 1'b0;
            end

            case (state)
                S_POR_WAIT: begin
                    data_oe <= 1'b0;
                    if (ulpi_dir) por_wait_cnt <= 24'd0;
                    else if (por_wait_cnt == POR_STABLE_CNT) state <= S_IDLE;
                    else por_wait_cnt <= por_wait_cnt + 24'd1;
                end

                // Launch the OTG Control write once the bus is ours and
                // has been idle for a full cycle (turnaround respected).
                S_IDLE: begin
                    if (!ulpi_dir && !dir_d) begin
                        data_oe  <= 1'b1;
                        data_out <= {2'b10, ADDR_OTG_CTRL}; // RegWrite cmd
                        state    <= S_WCMD;
                    end else begin
                        data_oe <= 1'b0;
                    end
                end

                S_WCMD: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0;
                        state   <= S_IDLE;      // PHY aborted, retry
                    end else if (ulpi_nxt) begin
                        data_out <= OTG_CTRL_VAL;
                        state    <= S_WDATA;
                    end
                end

                S_WDATA: begin
                    if (ulpi_dir) begin
                        data_oe <= 1'b0;
                        state   <= S_IDLE;      // PHY aborted, retry
                    end else begin
                        stp   <= 1'b1;          // close the write
                        state <= S_WSTP;
                    end
                end

                S_WSTP: begin
                    data_oe <= 1'b0;
                    state   <= S_MONITOR;
                end

                // VBUS is on; from here just watch RX CMD updates.
                S_MONITOR: begin
                    data_oe <= 1'b0;
                end

                default: state <= S_POR_WAIT;
            endcase
        end
    end

    // ---------------------------------------------------------------
    // Flash-count display with a long start-of-lap marker
    // ---------------------------------------------------------------
    localparam [25:0] T_PULSE  = 26'd6_000_000;  // ~100ms short flash
    localparam [25:0] T_MARK   = 26'd36_000_000; // ~600ms marker flash
    localparam [25:0] T_GAP    = 26'd24_000_000; // ~400ms gaps
    localparam [25:0] T_LONG   = 26'd90_000_000; // ~1.5s between laps

    localparam P_LAP_GAP  = 3'd0;
    localparam P_MARK_ON  = 3'd1;
    localparam P_MARK_OFF = 3'd2;
    localparam P_G1_ON    = 3'd3;
    localparam P_G1_OFF   = 3'd4;
    localparam P_GAP_MID  = 3'd5;
    localparam P_G2_ON    = 3'd6;
    localparam P_G2_OFF   = 3'd7;

    reg [2:0]  phase;
    reg [26:0] timer;
    reg [3:0]  pulse_idx;
    reg [3:0]  g1_count, g2_count;
    reg        lamp_on;

    wire [3:0] g1_next = rxcmd_seen ? ({2'b00, last_rxcmd[3:2]} + 4'd2) : 4'd1;
    wire [3:0] g2_next = {2'b00, last_rxcmd[1:0]} + 4'd1;

    always @(posedge ulpi_clk or negedge rst_n_ulpi) begin
        if (!rst_n_ulpi) begin
            phase     <= P_LAP_GAP;
            timer     <= 27'd0;
            pulse_idx <= 4'd0;
            g1_count  <= 4'd1;
            g2_count  <= 4'd1;
            lamp_on   <= 1'b0;
        end else begin
            timer <= timer + 27'd1;
            case (phase)
                P_LAP_GAP: begin
                    lamp_on <= 1'b0;
                    if (timer >= T_LONG) begin
                        timer    <= 27'd0;
                        g1_count <= g1_next;   // sample live values per lap
                        g2_count <= g2_next;
                        phase    <= P_MARK_ON;
                    end
                end
                P_MARK_ON: begin
                    lamp_on <= 1'b1;
                    if (timer >= T_MARK) begin
                        timer <= 27'd0;
                        phase <= P_MARK_OFF;
                    end
                end
                P_MARK_OFF: begin
                    lamp_on <= 1'b0;
                    if (timer >= T_GAP) begin
                        timer     <= 27'd0;
                        pulse_idx <= 4'd0;
                        phase     <= P_G1_ON;
                    end
                end
                P_G1_ON: begin
                    lamp_on <= 1'b1;
                    if (timer >= T_PULSE) begin
                        timer <= 27'd0;
                        phase <= P_G1_OFF;
                    end
                end
                P_G1_OFF: begin
                    lamp_on <= 1'b0;
                    if (timer >= T_PULSE) begin
                        timer     <= 27'd0;
                        pulse_idx <= pulse_idx + 4'd1;
                        if (pulse_idx + 4'd1 >= g1_count) phase <= P_GAP_MID;
                        else                              phase <= P_G1_ON;
                    end
                end
                P_GAP_MID: begin
                    lamp_on <= 1'b0;
                    if (timer >= T_GAP) begin
                        timer     <= 27'd0;
                        pulse_idx <= 4'd0;
                        phase     <= P_G2_ON;
                    end
                end
                P_G2_ON: begin
                    lamp_on <= 1'b1;
                    if (timer >= T_PULSE) begin
                        timer <= 27'd0;
                        phase <= P_G2_OFF;
                    end
                end
                P_G2_OFF: begin
                    lamp_on <= 1'b0;
                    if (timer >= T_PULSE) begin
                        timer     <= 27'd0;
                        pulse_idx <= pulse_idx + 4'd1;
                        if (pulse_idx + 4'd1 >= g2_count) phase <= P_LAP_GAP;
                        else                              phase <= P_G2_ON;
                    end
                end
                default: phase <= P_LAP_GAP;
            endcase
        end
    end

    assign led = {4{lamp_on}};

endmodule
