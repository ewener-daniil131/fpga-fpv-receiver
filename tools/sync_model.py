#!/usr/bin/env python3
"""Bit-level reference model of ntsc_sync_dpll.v.

Integer-only and written with the same state the Verilog holds, so a change can
be tried here in seconds and then transcribed, and so the test bench has
something to compare against. This is the same idea as tools/render.py: whatever
wins on the recording is what gets implemented in Verilog.

    python3 tools/sync_model.py captures/....txt      # a baseband video dump
"""
import numpy as np

FRAC = 8                      # fractional bits of the NCO phase


class SyncDPLL:
    """Horizontal AFC + sync slicer + vertical sync integrator.

    The three things it does differently from a threshold-and-window separator,
    each of which was a measured failure of the previous design:

    1. The slicing level is derived from two *measured* levels - the sync tip and
       the back porch - and sits halfway between them. A fixed offset above the
       tip tracks neither contrast nor gain.
    2. The back porch is sampled in a window gated by the sync pulse itself, not
       by the loop's own phase. Gating it on the phase is a feedback path: when
       the loop slips, the window lands in the picture, the level rises, the
       slice rises, and the loop never recovers.
    3. Line timing comes from an NCO whose period is *learned* by an integrator,
       not from a counter that is reset by each pulse. A pulse nudges the phase
       instead of setting it, so noise averages out, and when pulses go missing
       the NCO free-runs at the period it measured rather than at a constant
       that is only approximately right.
    """

    def __init__(self,
                 LINE_NOM=1271,      # samples per line at 20 Msps
                 GLITCH=8,           # debounce, 0.4 us
                 W_HS_MIN=60, W_HS_MAX=150,     # H sync is 94 samples (4.7 us)
                 W_BROAD=300,
                 CAPTURE=48,         # phase detector window once locked, samples
                 KP=3, KI=12,        # loop filter shifts, swept on tools/ntsc_gen.py output
                 ACT_START=190, ACT_LEN=1050,   # 9.5 us after sync, 52.5 us long
                 VACT_START=18, VACT_LINES=240,
                 VSYNC_LOW=600,      # low samples in one line = vertical interval
                 VS_GUARD=200,
                 TIP_LEAK=512,
                 BP_SKIP=16, BP_LEN=32,         # back porch sampling window
                 LOCK_ERR=8, LOCK_N=3, LOSS_N=8):
        self.__dict__.update(locals())
        del self.self
        self.P = LINE_NOM << FRAC
        self.ONE = 1 << FRAC
        self.period = self.P
        self.ierr = 0
        self.phase = 0
        # Start near the levels this transmitter actually produces, so the
        # very first pulse is already sliced in the right place. Starting at
        # the extremes costs about eight lines of settling, which is most of
        # an island of the gappy USB stream.
        self.tip = 48
        self.blank = 96
        self.leak = 0
        self.low = False
        self.deb = 0
        self.fall_n = 0
        self.fall_phase = 0
        self.lowsum = 0
        self.bp_cnt = -1
        self.bp_acc = 0
        self.locked = False
        self.good = 0
        self.miss = 0
        self.line = 0
        self.vsg = 0
        self.n = 0
        self.pacc = 0
        # observability, mirrored by the Verilog's counters
        self.hs = []
        self.vs = []
        self.pix = []
        self.widths = []
        self.err_log = []
        self.lock_at = None

    # ---- level trackers ---------------------------------------------------
    @property
    def slice_level(self):
        d = self.blank - self.tip
        if d < 16:
            d = 16
        elif d > 128:
            d = 128
        return self.tip + (d >> 1)

    def sample(self, v):
        n = self.n
        self.n += 1

        if v < self.tip:
            self.tip = v
        else:
            self.leak += 1
            if self.leak >= self.TIP_LEAK:
                self.leak = 0
                if self.tip < 255:
                    self.tip += 1

        # back porch sampler, armed by the sync pulse's trailing edge
        if self.bp_cnt >= 0:
            self.bp_cnt += 1
            if self.bp_cnt > self.BP_SKIP:
                self.bp_acc += v
                if self.bp_cnt == self.BP_SKIP + self.BP_LEN:
                    avg = self.bp_acc // self.BP_LEN
                    self.blank += (avg - self.blank) >> 3
                    self.bp_cnt = -1
                    self.bp_acc = 0

        # ---- debounced slicer ---------------------------------------------
        raw = v < self.slice_level
        edge = None
        if raw != self.low:
            self.deb += 1
            if self.deb >= self.GLITCH:
                self.low = raw
                self.deb = 0
                edge = 'fall' if raw else 'rise'
        else:
            self.deb = 0
        if self.low:
            self.lowsum += 1

        # ---- NCO -----------------------------------------------------------
        self.phase += self.ONE
        line_tick = False
        if self.phase >= self.period:
            self.phase -= self.period
            line_tick = True
        ph = self.phase >> FRAC

        # ---- pulse handling --------------------------------------------------
        if edge == 'fall':
            self.fall_n = n - self.GLITCH
            p = self.phase - (self.GLITCH << FRAC)
            self.fall_phase = p + self.period if p < 0 else p
        elif edge == 'rise':
            w = (n - self.GLITCH) - self.fall_n
            self.widths.append(w)
            is_hs = self.W_HS_MIN <= w <= self.W_HS_MAX
            if is_hs:
                self.bp_cnt = 0                      # arm the back porch sampler
                e = self.fall_phase
                if e >= (self.period >> 1):
                    e -= self.period                 # signed, 1/256 of a sample
                self.err_log.append((n, e >> FRAC, w))
                if not self.locked:
                    self.phase = ((n - self.fall_n) << FRAC) % self.period
                    self.good += 1
                    self.miss = 0          # or the first line after lock drops it again
                    self.hs.append(self.fall_n)
                    if self.good >= self.LOCK_N:
                        self.locked = True
                        if self.lock_at is None:
                            self.lock_at = n
                elif abs(e) <= (self.CAPTURE << FRAC):
                    self.phase -= e >> self.KP                  # proportional
                    if self.phase < 0:
                        self.phase += self.period
                    elif self.phase >= self.period:
                        self.phase -= self.period
                    if abs(e) <= (self.LOCK_ERR << FRAC):
                        # Integral: accumulate the error and derive the period
                        # from the accumulator. Shifting each error on its own
                        # leaves a dead zone - any error smaller than 1<<KI
                        # shifts to zero - and the period then never converges
                        # on the transmitter's true line rate, which is not
                        # exactly nominal (measured 1269..1272 against 1271).
                        self.ierr -= e
                        lim = self.P >> 6                       # +-1.6 %
                        adj = self.ierr >> self.KI
                        self.period = max(self.P - lim, min(self.P + lim, self.P + adj))
                        self.good = min(self.good + 1, 15)
                        self.miss = 0
                    self.hs.append(self.fall_n)

        # ---- line and field bookkeeping ---------------------------------------
        if line_tick:
            in_vert = self.lowsum > self.VSYNC_LOW    # sync integrator = vertical
            if in_vert:
                if self.vsg == 0:
                    self.vs.append(n)
                    self.line = 0
                    self.vsg = self.VS_GUARD
            self.lowsum = 0
            if self.vsg:
                self.vsg -= 1
            self.line = self.line + 1 if self.line < 524 else 0
            if self.locked:
                # The vertical interval carries no line-width sync at all, so a
                # miss there is expected and must not be read as signal loss.
                if in_vert:
                    self.miss = 0
                else:
                    self.miss += 1
                    if self.miss >= self.LOSS_N:
                        self.locked = False
                        self.good = 0

        # ---- active video ------------------------------------------------------
        if self.locked:
            x = ph - self.ACT_START
            if (0 <= x < self.ACT_LEN and
                    self.VACT_START <= self.line < self.VACT_START + self.VACT_LINES):
                if (x & 1) == 0:
                    self.pacc = v
                else:
                    self.pix.append((x >> 1, self.line - self.VACT_START,
                                     (self.pacc + v) >> 1))
        return line_tick

    def run(self, video):
        for v in video:
            self.sample(int(v))
        return self


def frame(pix, width=525, height=240):
    """Rebuild the picture from the emitted pixels, NaN where nothing arrived."""
    acc = np.zeros((height, width))
    cnt = np.zeros((height, width))
    for x, y, v in pix:
        if 0 <= x < width and 0 <= y < height:
            acc[y, x] += v
            cnt[y, x] += 1
    return np.where(cnt > 0, acc / np.maximum(cnt, 1), np.nan)


def score(img):
    """Adjacent-line agreement over the rows that are actually filled."""
    rows = [r for r in img if np.isfinite(r).mean() > 0.8]
    if len(rows) < 4:
        return 0.0, len(rows)
    rows = np.nan_to_num(np.array(rows))
    c = np.nanmean([np.corrcoef(rows[i], rows[i + 1])[0, 1]
                    for i in range(len(rows) - 1)])
    return float(c), len(rows)


if __name__ == '__main__':
    import sys
    v = np.loadtxt(sys.argv[1]).astype(int)
    m = SyncDPLL().run(v)
    img = frame(m.pix)
    c, rows = score(img)
    print('samples %d (%.1f lines)' % (len(v), len(v) / 1271))
    print('hsync accepted %d   vsync %d   locked %s   lock after %s samples'
          % (len(m.hs), len(m.vs), m.locked, m.lock_at))
    print('tip %d  blank %d  slice %d   NCO period %.3f samples'
          % (m.tip, m.blank, m.slice_level, m.period / 256))
    print('pixels %d   filled rows %d   adjacent-line correlation %.3f'
          % (len(m.pix), rows, c))
