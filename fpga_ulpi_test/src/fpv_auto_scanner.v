// =============================================================================
// fpv_auto_scanner.v
// Automatic 5.8 GHz FPV Frequency Scanner & Channel Lock Engine
//
// Automatically scans the 40 standard FPV channels across Bands A, B, E, F, R
// (5645 MHz to 5945 MHz) to locate active drone video transmissions.
//
// Detection Criteria:
//   1. Carrier Power (RSSI)
//   2. NTSC Horizontal Video Sync Lock (15.734 kHz periodicity)
// =============================================================================

`timescale 1ns / 1ps

module fpv_auto_scanner #(
    parameter [19:0] SETTLE_CYCLES  = 20'd216_000, // 8 ms @ 27 MHz
    parameter [19:0] MEASURE_CYCLES = 20'd450_000  // 16.7 ms (1 full NTSC field) @ 27 MHz
)(
    input  wire        clk,              // 27 MHz system clock
    input  wire        rst_n,            // Active-low reset

    // Scan control & triggers
    input  wire        start_scan,       // Trigger pulse to initiate scan sweep

    // Signal quality feedback from demodulator & sync separator
    input  wire [15:0] rssi,             // Current channel signal strength
    input  wire        ntsc_locked,      // High when valid NTSC sync detected

    // Host control transfer interface (to ulpi_enum_test host FSM)
    output reg         retune_req,       // Pulse to request frequency change
    output reg  [15:0] retune_freq_mhz,  // Target frequency in MHz
    input  wire        retune_done,      // Pulse from host when SET_FREQ completed

    // Scanner status outputs
    output reg         is_scanning,      // High while auto-scan is running
    output reg  [5:0]  cur_ch_idx,       // Channel index currently being evaluated (0..39)
    output reg  [5:0]  locked_ch_idx,    // Locked channel index
    output reg  [15:0] locked_freq_mhz,  // Locked channel frequency in MHz
    output reg  [15:0] best_rssi,         // Best measured signal strength
    output reg         locked_sync       // Sync lock status on locked channel
);

    // -------------------------------------------------------------------------
    // 40 Standard 5.8 GHz FPV Frequencies (MHz)
    // -------------------------------------------------------------------------
    function [15:0] get_fpv_freq(input [5:0] idx);
        case (idx)
            // Band A (Boscam A)
            6'd0:  get_fpv_freq = 16'd5865;
            6'd1:  get_fpv_freq = 16'd5845;
            6'd2:  get_fpv_freq = 16'd5825;
            6'd3:  get_fpv_freq = 16'd5805;
            6'd4:  get_fpv_freq = 16'd5785;
            6'd5:  get_fpv_freq = 16'd5765;
            6'd6:  get_fpv_freq = 16'd5745;
            6'd7:  get_fpv_freq = 16'd5725;
            // Band B (Boscam B)
            6'd8:  get_fpv_freq = 16'd5733;
            6'd9:  get_fpv_freq = 16'd5752;
            6'd10: get_fpv_freq = 16'd5771;
            6'd11: get_fpv_freq = 16'd5790;
            6'd12: get_fpv_freq = 16'd5809;
            6'd13: get_fpv_freq = 16'd5828;
            6'd14: get_fpv_freq = 16'd5847;
            6'd15: get_fpv_freq = 16'd5866;
            // Band E (Foxtech / Boscam E)
            6'd16: get_fpv_freq = 16'd5705;
            6'd17: get_fpv_freq = 16'd5685;
            6'd18: get_fpv_freq = 16'd5665;
            6'd19: get_fpv_freq = 16'd5645;
            6'd20: get_fpv_freq = 16'd5885;
            6'd21: get_fpv_freq = 16'd5905;
            6'd22: get_fpv_freq = 16'd5925;
            6'd23: get_fpv_freq = 16'd5945;
            // Band F (FatShark / ImmersionRC)
            6'd24: get_fpv_freq = 16'd5740;
            6'd25: get_fpv_freq = 16'd5760;
            6'd26: get_fpv_freq = 16'd5780;
            6'd27: get_fpv_freq = 16'd5800;
            6'd28: get_fpv_freq = 16'd5820;
            6'd29: get_fpv_freq = 16'd5840;
            6'd30: get_fpv_freq = 16'd5860;
            6'd31: get_fpv_freq = 16'd5880;
            // Band R (RaceBand)
            6'd32: get_fpv_freq = 16'd5658;
            6'd33: get_fpv_freq = 16'd5695;
            6'd34: get_fpv_freq = 16'd5732;
            6'd35: get_fpv_freq = 16'd5769;
            6'd36: get_fpv_freq = 16'd5806;
            6'd37: get_fpv_freq = 16'd5843;
            6'd38: get_fpv_freq = 16'd5880;
            6'd39: get_fpv_freq = 16'd5917;
            default: get_fpv_freq = 16'd5800;
        endcase
    endfunction

    // Scanner States
    localparam S_IDLE     = 3'd0;
    localparam S_REQ_TUNE = 3'd1;
    localparam S_WAIT_TUNE= 3'd2;
    localparam S_SETTLE   = 3'd3;
    localparam S_MEASURE  = 3'd4;
    localparam S_NEXT_CH  = 3'd5;
    localparam S_LOCK_TUNE= 3'd6;
    localparam S_LOCK_WAIT= 3'd7;

    reg [2:0]  state;
    reg [19:0] timer;
    reg [16:0] best_score;
    reg [5:0]  best_ch;
    reg [15:0] best_freq;
    reg        best_sync;

    wire [16:0] cur_score = {ntsc_locked, rssi}; // Sync lock takes absolute priority

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            timer           <= 20'd0;
            retune_req      <= 1'b0;
            retune_freq_mhz <= 16'd5800;
            is_scanning     <= 1'b0;
            cur_ch_idx      <= 6'd0;
            locked_ch_idx   <= 6'd27;    // Default: F4 (5800 MHz)
            locked_freq_mhz <= 16'd5800;
            best_score      <= 17'd0;
            best_rssi       <= 16'd0;
            best_ch         <= 6'd27;
            best_freq       <= 16'd5800;
            best_sync       <= 1'b0;
            locked_sync     <= 1'b0;
        end else begin
            retune_req <= 1'b0;

            case (state)
                S_IDLE: begin
                    is_scanning <= 1'b0;
                    locked_sync <= ntsc_locked;
                    if (start_scan) begin
                        is_scanning <= 1'b1;
                        cur_ch_idx  <= 6'd0;
                        best_score  <= 17'd0;
                        best_rssi   <= 16'd0;
                        best_ch     <= 6'd27;
                        best_freq   <= 16'd5800;
                        best_sync   <= 1'b0;
                        state       <= S_REQ_TUNE;
                    end
                end

                S_REQ_TUNE: begin
                    retune_req      <= 1'b1;
                    retune_freq_mhz <= get_fpv_freq(cur_ch_idx);
                    state           <= S_WAIT_TUNE;
                end

                S_WAIT_TUNE: begin
                    if (retune_done) begin
                        timer <= 20'd0;
                        state <= S_SETTLE;
                    end
                end

                S_SETTLE: begin
                    // Settle for PLL lock and FIFO flush
                    if (timer >= SETTLE_CYCLES) begin
                        timer <= 20'd0;
                        state <= S_MEASURE;
                    end else timer <= timer + 20'd1;
                end

                S_MEASURE: begin
                    // Measure RSSI and sync lock
                    if (timer >= MEASURE_CYCLES) begin
                        // Check if current channel score exceeds previous best
                        if (cur_score > best_score) begin
                            best_score <= cur_score;
                            best_rssi  <= rssi;
                            best_ch    <= cur_ch_idx;
                            best_freq  <= get_fpv_freq(cur_ch_idx);
                            best_sync  <= ntsc_locked;
                        end
                        state <= S_NEXT_CH;
                    end else timer <= timer + 20'd1;
                end

                S_NEXT_CH: begin
                    if (cur_ch_idx == 6'd39) begin
                        // Finished scanning all 40 channels! Now lock to best channel
                        state <= S_LOCK_TUNE;
                    end else begin
                        cur_ch_idx <= cur_ch_idx + 6'd1;
                        state      <= S_REQ_TUNE;
                    end
                end

                S_LOCK_TUNE: begin
                    retune_req      <= 1'b1;
                    retune_freq_mhz <= best_freq;
                    state           <= S_LOCK_WAIT;
                end

                S_LOCK_WAIT: begin
                    if (retune_done) begin
                        locked_ch_idx   <= best_ch;
                        locked_freq_mhz <= best_freq;
                        locked_sync     <= best_sync;
                        is_scanning     <= 1'b0;
                        state           <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
