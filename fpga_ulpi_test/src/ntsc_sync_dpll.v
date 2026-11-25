// =============================================================================
// ntsc_sync_dpll.v
// NTSC sync separator and active-raster generator, drop-in replacement for
// ntsc_sync_separator.v.
//
// Three things are done differently from the threshold-and-window separator it
// replaces. Each was a measured failure of that design on the recorded signal
// in captures/iq_session.txt, not a theoretical improvement:
//
// 1. SLICING LEVEL FROM TWO MEASURED LEVELS.
//    The old slicer tracked the sync tip with a leak of +1 per sample and a
//    ceiling of 80, so the tip climbed from 40 to its ceiling in 2 us and stayed
//    there 87% of the time. The slicing level, tip+25, therefore sat at 105 -
//    inside the picture, whose black level is 85 and median 135 - and dark
//    content was sliced as sync. Here the tip leaks +1 per 512 samples and the
//    blanking level is measured, with the slice halfway between them.
//
// 2. THE BACK PORCH IS SAMPLED FROM THE PULSE, NOT FROM THE LOOP.
//    Gating the blanking-level window on the loop's own phase closes a feedback
//    path: when the loop slips the window lands in the picture, the measured
//    level rises, the slice rises, and the loop never recovers. The window here
//    is armed by the trailing edge of the sync pulse itself.
//
// 3. LINE TIMING FROM A LEARNED PERIOD, NOT A CONSTANT.
//    An NCO free-runs at a period maintained by an integrator, and each sync
//    pulse nudges its phase instead of resetting it. Noise therefore averages
//    out instead of moving the line start, and the free-running period is the
//    transmitter's real one (measured 1269..1272 against a nominal 1271) rather
//    than a hard-coded value whose error walks the picture sideways.
//
// Measured by tools/sync_model.py on the 16 recorded islands of real video:
// line-period jitter 0.20 samples, against 154.5 for the old design.
//
// Reference model: tools/sync_model.py (same state, same arithmetic).
// Test bench:      sim/tb_sync_dpll.v
// =============================================================================

`timescale 1ns / 1ps

module ntsc_sync_dpll #(
    // ---- all timings in input samples; defaults are 20 Msps NTSC ----
    parameter integer LINE_NOM  = 1271,   // 63.5555 us
    parameter integer GLITCH    = 8,      // slicer debounce, 0.4 us
    parameter integer W_HS_MIN  = 60,     // H sync is 4.7 us = 94 samples
    parameter integer W_HS_MAX  = 150,
    parameter integer CAPTURE   = 48,     // phase-detector window once locked
    parameter integer KP        = 3,      // proportional shift
    parameter integer KI        = 12,     // integral shift
    parameter integer ACT_START = 190,    // 9.5 us after the sync leading edge
    parameter integer ACT_LEN   = 1050,   // 52.5 us of active video
    parameter integer PIX_SHIFT = 1,      // 1050 samples -> 525 pixels
    parameter integer VACT_START = 18,
    parameter integer VACT_LINES = 240,
    parameter integer VSYNC_LOW = 600,    // low samples in a line => vertical
    parameter integer VS_GUARD  = 200,    // lines of vsync blanking afterwards
    parameter integer TIP_LEAK  = 512,    // samples per +1 of tip leak
    parameter integer BP_SKIP   = 16,     // back porch window, after sync rise
    parameter integer BP_LEN    = 32,     // must be a power of two
    parameter integer LOCK_ERR  = 8,      // |phase error| that counts as good
    parameter integer LOCK_N    = 3,      // good pulses to declare lock
    parameter integer LOSS_N    = 8,      // lines without sync that drop lock
    parameter integer TIP_INIT  = 48,     // start near the real levels, so the
    parameter integer BLANK_INIT = 96,    // first pulse is already sliced right
    // 8 Msps configuration, for reference:
    //   LINE_NOM 508, GLITCH 3, W_HS_MIN 24, W_HS_MAX 60, CAPTURE 19,
    //   ACT_START 76, ACT_LEN 420, PIX_SHIFT 0, VSYNC_LOW 240,
    //   TIP_LEAK 205, BP_SKIP 6, BP_LEN 16, LOCK_ERR 3
    parameter integer FRAC      = 8       // fractional bits of the NCO phase
)(
    input  wire        clk,
    input  wire        rst_n,

    // Baseband video from fm_demod
    input  wire [7:0]  video_in,
    input  wire        video_valid,

    // A hole in the sample stream: the USB transport delivers IQ in islands and
    // simply loses the time between them, so the phase after a hole is
    // meaningless. Pulse this and the loop re-acquires instead of dragging a
    // stale phase across the seam. Tie low if nothing can report it.
    input  wire        stream_gap,

    output reg         hsync,            // an accepted sync pulse
    output reg         line_advance,     // every line boundary, sync or not
    output reg         vsync,
    output reg         ntsc_locked,
    output reg         field,

    output reg  [9:0]  line_cnt,
    output reg  [9:0]  pixel_x,
    output reg  [8:0]  pixel_y,
    output reg  [7:0]  pixel_val,
    output reg         pixel_valid,

    // diagnostics, for the UART telemetry; leave unconnected if not wanted
    output wire [7:0]  dbg_slice,
    output wire [7:0]  dbg_tip,
    output wire [7:0]  dbg_blank,
    output wire [19:0] dbg_period
);
    localparam integer PW   = 21;                 // phase / period width, signed
    localparam integer PNOM = LINE_NOM * (1 << FRAC);
    localparam integer PLIM = PNOM / 64;          // period may move +-1.6 %
    localparam integer BP_SH = (BP_LEN ==  16) ? 4 :
                               (BP_LEN ==  32) ? 5 :
                               (BP_LEN ==  64) ? 6 : 7;

    // -------------------------------------------------------------------------
    // 1. Level trackers and the slicer
    // -------------------------------------------------------------------------
    reg  [7:0]  tip, blank;
    reg  [9:0]  leak;
    reg  [6:0]  bp_cnt;        // 0 = disarmed, else position in the window
    reg  [13:0] bp_acc;

    wire [8:0] span_raw = {1'b0, blank} - {1'b0, tip};
    wire [7:0] span     = (blank <= tip)       ? 8'd16  :
                          (span_raw > 9'd128)  ? 8'd128 :
                          (span_raw < 9'd16)   ? 8'd16  : span_raw[7:0];
    wire [8:0] slice_w  = {1'b0, tip} + {1'b0, span[7:1]};
    wire [7:0] slice    = slice_w[8] ? 8'd255 : slice_w[7:0];

    assign dbg_slice = slice;
    assign dbg_tip   = tip;
    assign dbg_blank = blank;

    // One IIR step of the blanking level. Every operand is made signed
    // explicitly: a single unsigned operand turns the whole expression unsigned
    // and degrades >>> to a logical shift, which sends a negative difference to
    // a large positive one and walks the level into the picture.
    wire signed [10:0] bp_avg_s  = $signed({2'b00, bp_acc[13:BP_SH]});
    wire signed [10:0] blank_s   = $signed({3'b000, blank});
    wire signed [10:0] blank_sum = blank_s + ((bp_avg_s - blank_s) >>> 3);
    wire [7:0] blank_next = (blank_sum < 11'sd0)   ? 8'd0   :
                            (blank_sum > 11'sd255) ? 8'd255 : blank_sum[7:0];

    // -------------------------------------------------------------------------
    // 2. Debounced sync detection
    // -------------------------------------------------------------------------
    reg        low;                         // debounced state of the slicer
    reg  [3:0] deb;
    reg        fall_evt, rise_evt;
    reg  [9:0] run_len;                     // samples since the last edge
    reg  [9:0] pulse_w;                     // width of the pulse just ended
    reg  [10:0] low_in_line;                // sync integrator, per line

    wire raw_low = (video_in < slice);

    // -------------------------------------------------------------------------
    // 3. The NCO and its loop filter
    // -------------------------------------------------------------------------
    reg  signed [PW-1:0] phase;
    reg  signed [PW-1:0] period;
    reg  signed [PW-1:0] fall_phase;
    reg  signed [31:0]   ierr;
    reg                  line_tick;
    reg  [3:0]           good;
    reg  [3:0]           miss;
    reg  [9:0]           vs_guard;
    reg  [8:0]           pacc;

    assign dbg_period = period[19:0];

    wire signed [PW-1:0] e_raw  = fall_phase;
    wire signed [PW-1:0] e_sgn  = (e_raw >= (period >>> 1)) ? (e_raw - period) : e_raw;
    wire signed [PW-1:0] e_abs  = (e_sgn < 0) ? -e_sgn : e_sgn;
    wire                 in_win = (e_abs <= (CAPTURE * (1 << FRAC)));
    wire                 is_good= (e_abs <= (LOCK_ERR * (1 << FRAC)));
    wire                 is_hs  = (pulse_w >= W_HS_MIN) && (pulse_w <= W_HS_MAX);

    // The phase correction has to be applied on top of this sample's NCO step,
    // not instead of it. Assigning the corrected phase in the same cycle as the
    // step silently drops one increment per line, which is a bias of exactly one
    // sample per line: the loop then settles with a standing error of KP samples
    // instead of zero, the integral never comes inside its gate, and lock is
    // dropped every LOSS_N lines for no visible reason.
    wire signed [PW-1:0] ph_step = phase + (1 <<< FRAC);
    wire signed [PW-1:0] ph_adv  = (ph_step >= period) ? (ph_step - period) : ph_step;
    wire signed [PW-1:0] ph_prop = ph_adv - (e_sgn >>> KP);
    wire signed [PW-1:0] ph_wrap = (ph_prop < 0)       ? (ph_prop + period) :
                                   (ph_prop >= period) ? (ph_prop - period) : ph_prop;

    // Both operands are signed, so the subtraction sign-extends on its own.
    // Padding e_sgn with a concatenation here would make the expression
    // unsigned - the trap that already cost this project a de-emphasis filter.
    wire signed [31:0] ierr_n  = ierr - e_sgn;
    wire signed [31:0] ierr_c  = (ierr_n >  (PLIM <<< KI)) ?  (PLIM <<< KI) :
                                 (ierr_n < -(PLIM <<< KI)) ? -(PLIM <<< KI) : ierr_n;
    wire signed [31:0] per_adj = ierr_c >>> KI;
    wire signed [PW-1:0] per_n = $signed(PNOM[PW-1:0]) + per_adj[PW-1:0];

    // active-video position, from the NCO phase
    wire [PW-1:0] ph_int  = phase >>> FRAC;
    wire signed [11:0] act_x = ph_int[11:0] - ACT_START;
    wire in_act  = (act_x >= 0) && (act_x < ACT_LEN);
    wire in_vact = (line_cnt >= VACT_START) && (line_cnt < VACT_START + VACT_LINES);

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tip <= TIP_INIT[7:0]; blank <= BLANK_INIT[7:0]; leak <= 10'd0;
            bp_cnt <= 7'd0; bp_acc <= 14'd0;
            low <= 1'b0; deb <= 4'd0; run_len <= 10'd0; pulse_w <= 10'd0;
            fall_evt <= 1'b0; rise_evt <= 1'b0; low_in_line <= 11'd0;
            phase <= {PW{1'b0}}; period <= PNOM[PW-1:0]; fall_phase <= {PW{1'b0}};
            ierr <= 32'sd0; line_tick <= 1'b0;
            good <= 4'd0; miss <= 4'd0; vs_guard <= 10'd0; pacc <= 9'd0;
            hsync <= 1'b0; line_advance <= 1'b0; vsync <= 1'b0;
            ntsc_locked <= 1'b0; field <= 1'b0; line_cnt <= 10'd0;
            pixel_x <= 10'd0; pixel_y <= 9'd0; pixel_val <= 8'd0; pixel_valid <= 1'b0;
        end else begin
            hsync        <= 1'b0;
            vsync        <= 1'b0;
            line_advance <= 1'b0;
            pixel_valid  <= 1'b0;
            fall_evt     <= 1'b0;
            rise_evt     <= 1'b0;
            line_tick    <= 1'b0;

            // A hole in the stream: everything about the current phase is stale.
            if (stream_gap) begin
                ntsc_locked <= 1'b0;
                good        <= 4'd0;
                miss        <= 4'd0;
                ierr        <= 32'sd0;
                period      <= PNOM[PW-1:0];
            end else if (video_valid) begin
                // ---- tip tracker ------------------------------------------
                if (video_in < tip) begin
                    tip  <= video_in;
                    leak <= 10'd0;
                end else if (leak >= TIP_LEAK[9:0]) begin
                    leak <= 10'd0;
                    if (tip != 8'd255) tip <= tip + 8'd1;
                end else leak <= leak + 10'd1;

                // ---- back porch sampler, armed by the sync trailing edge ----
                if (bp_cnt != 7'd0) begin
                    bp_cnt <= bp_cnt + 7'd1;
                    if (bp_cnt > BP_SKIP[6:0]) begin
                        bp_acc <= bp_acc + {6'd0, video_in};
                        if (bp_cnt == (BP_SKIP + BP_LEN)) begin
                            blank  <= blank_next;      // blank += (avg-blank)>>3
                            bp_cnt <= 7'd0;
                            bp_acc <= 14'd0;
                        end
                    end
                end

                // ---- debounced slicer -------------------------------------
                run_len <= run_len + 10'd1;
                if (raw_low != low) begin
                    if (deb == GLITCH[3:0] - 4'd1) begin
                        low     <= raw_low;
                        deb     <= 4'd0;
                        run_len <= 10'd0;
                        pulse_w <= run_len - GLITCH[9:0];
                        if (raw_low) fall_evt <= 1'b1;
                        else         rise_evt <= 1'b1;
                    end else deb <= deb + 4'd1;
                end else deb <= 4'd0;
                if (low) low_in_line <= low_in_line + 11'd1;

                // ---- NCO ---------------------------------------------------
                phase <= ph_adv;
                if (ph_step >= period) line_tick <= 1'b1;

                // ---- pulse handling ----------------------------------------
                if (fall_evt) begin
                    // phase of the leading edge, GLITCH samples back
                    fall_phase <= (phase < (GLITCH <<< FRAC))
                                  ? (phase - (GLITCH <<< FRAC) + period)
                                  : (phase - (GLITCH <<< FRAC));
                end

                if (rise_evt && is_hs) begin
                    bp_cnt <= 7'd1;                       // arm the back porch
                    if (!ntsc_locked) begin
                        // Cold acquisition: snap the phase so that phase 0 is
                        // the pulse's leading edge. The debounce costs GLITCH
                        // samples at each edge and the event is registered, so
                        // the edge was pulse_w + 2*GLITCH + 1 samples ago - the
                        // same definition the phase detector uses, or the first
                        // measurement after lock starts a window away from zero.
                        // Always far below one period, so no wrap is needed.
                        phase <= (pulse_w + 2*GLITCH + 1) <<< FRAC;
                        hsync <= 1'b1;
                        miss  <= 4'd0;
                        if (good >= LOCK_N[3:0] - 4'd1) begin
                            ntsc_locked <= 1'b1;
                            good        <= LOCK_N[3:0];
                        end else good <= good + 4'd1;
                    end else if (in_win) begin
                        phase <= ph_wrap;                 // proportional
                        hsync <= 1'b1;
                        if (is_good) begin
                            ierr   <= ierr_c;             // integral
                            period <= per_n;
                            miss   <= 4'd0;
                            if (good != 4'd15) good <= good + 4'd1;
                        end
                    end
                end

                // ---- line and field bookkeeping ----------------------------
                if (line_tick) begin
                    line_advance <= 1'b1;
                    low_in_line  <= 11'd0;
                    if (vs_guard != 10'd0) vs_guard <= vs_guard - 10'd1;

                    if (low_in_line > VSYNC_LOW[10:0]) begin
                        // A line this full of sync is the vertical interval.
                        // Counting integrated sync, not the width of one
                        // serration, is what makes this survive noise: noise
                        // breaks a 27 us serration into fragments but does not
                        // change how much of the line is below the slice.
                        miss <= 4'd0;
                        if (vs_guard == 10'd0) begin
                            vsync    <= 1'b1;
                            field    <= ~field;
                            line_cnt <= 10'd0;
                            vs_guard <= VS_GUARD[9:0];
                        end else line_cnt <= (line_cnt == 10'd524) ? 10'd0
                                                                  : line_cnt + 10'd1;
                    end else begin
                        line_cnt <= (line_cnt == 10'd524) ? 10'd0 : line_cnt + 10'd1;
                        if (ntsc_locked) begin
                            if (miss >= LOSS_N[3:0] - 4'd1) begin
                                ntsc_locked <= 1'b0;
                                good        <= 4'd0;
                                miss        <= 4'd0;
                            end else miss <= miss + 4'd1;
                        end
                    end
                end

                // ---- active video -------------------------------------------
                if (ntsc_locked && in_act && in_vact) begin
                    pixel_x <= act_x[10:PIX_SHIFT];
                    pixel_y <= line_cnt[8:0] - VACT_START[8:0];
                    if (act_x[0] == 1'b0) begin
                        pacc <= {1'b0, video_in};
                    end else begin
                        pixel_val   <= (pacc + {1'b0, video_in}) >> 1;
                        pixel_valid <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
