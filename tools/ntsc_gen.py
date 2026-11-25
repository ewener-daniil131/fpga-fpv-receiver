#!/usr/bin/env python3
"""Synthetic NTSC source for the sync-separator test benches.

Builds a real NTSC field structure - equalizing pulses, broad serrations, the
half-line field offset - FM-modulates it the way an analog FPV transmitter does,
and writes the IQ as a hex file an iverilog test bench can $readmemh.

Why generate instead of only using the recording: the recording is six-line
islands with holes between them (see HANDOFF section on the USB transport), so
it can never exercise the vertical interval or a field boundary. The generator
covers those; the recording covers what the real RF actually looks like. Both
are needed, and the test benches use both.

The modulation constants are the inverse of fm_demod.v, so a generated level of
40 comes back out of the demodulator as 40:

    video = dtheta_sum/4 >> 6 + 128,  dtheta = f/Fs * 65536  (CORDIC pi = 32768)
    => video = 128 + f * 3.836e-5     (f in Hz)

Measured on the air for this transmitter: sync tip -2.30 MHz (video 40),
blanking about -1.12 MHz (video 85), which is what the defaults below produce.

    python3 tools/ntsc_gen.py --fields 3 --snr 10 --out /tmp/iq.hex
"""
import argparse
import numpy as np

FS = 20.0e6
LINE_SAMPLES = 1271          # 63.5555 us at 20 Msps
HZ_PER_LEVEL = 1.0 / 3.836e-5

# 8-bit video levels, matching what fm_demod.v produces from this transmitter
LVL_SYNC = 40
LVL_BLANK = 85
LVL_BLACK = 92
LVL_WHITE = 205

US = FS / 1e6
W_SYNC = int(4.7 * US)       # 94  horizontal sync tip
W_FRONT = int(1.5 * US)      # 30  front porch
W_BACK = int(4.7 * US)       # 94  back porch
W_EQ = int(2.3 * US)         # 46  equalizing pulse
W_BROAD = int(27.1 * US)     # 542 vertical serration


def _active_pattern(n, line_no, kind):
    """Picture content: a pattern with known structure so the test can score it."""
    x = np.arange(n) / n
    if kind == 'bars':
        bars = np.array([1.0, 0.75, 0.5, 0.35, 0.25, 0.15, 0.08, 0.0])
        v = bars[np.minimum((x * 8).astype(int), 7)]
    elif kind == 'ramp':
        v = x
    else:  # 'wedge': vertical position shifts the pattern, so torn lines show up
        v = 0.5 + 0.5 * np.sin(2 * np.pi * (x * 6 + line_no / 40.0))
    return LVL_BLACK + v * (LVL_WHITE - LVL_BLACK)


def _line(line_no, kind):
    """One ordinary video line: front porch, sync, back porch, active."""
    out = np.full(LINE_SAMPLES, LVL_BLANK, dtype=float)
    out[0:W_SYNC] = LVL_SYNC
    act_start = W_SYNC + W_BACK
    act_end = LINE_SAMPLES - W_FRONT
    out[act_start:act_end] = _active_pattern(act_end - act_start, line_no, kind)
    return out


def _half_line_eq(broad):
    """Half a line of the vertical interval: an equalizing or a serration pulse."""
    h = LINE_SAMPLES // 2
    out = np.full(h, LVL_BLANK, dtype=float)
    out[0:(W_BROAD if broad else W_EQ)] = LVL_SYNC
    return out


def field(parity, kind='wedge'):
    """One NTSC field: 262 or 263 lines, vertical interval first."""
    parts = []
    for _ in range(6):
        parts.append(_half_line_eq(False))      # pre-equalizing
    for _ in range(6):
        parts.append(_half_line_eq(True))       # broad serrations = vertical sync
    for _ in range(6):
        parts.append(_half_line_eq(False))      # post-equalizing
    n_blank = 12
    for i in range(n_blank):
        ln = _line(i, kind)
        ln[W_SYNC + W_BACK:LINE_SAMPLES - W_FRONT] = LVL_BLANK
        parts.append(ln)
    for i in range(253 - n_blank):
        parts.append(_line(i, kind))
    v = np.concatenate(parts)
    if parity:                                  # field 2 is offset by half a line
        v = np.concatenate([np.full(LINE_SAMPLES // 2, LVL_BLANK), v])
    return v


def modulate(video, snr_db=None, seed=1):
    """FM-modulate an 8-bit video level stream into int8 IQ."""
    f = (video - 128.0) * HZ_PER_LEVEL
    ph = np.cumsum(2 * np.pi * f / FS)
    amp = 70.0
    z = amp * np.exp(1j * ph)
    if snr_db is not None:
        rng = np.random.default_rng(seed)
        n = rng.normal(size=len(z)) + 1j * rng.normal(size=len(z))
        z = z + n * amp / (10 ** (snr_db / 20.0)) / np.sqrt(2)
    i = np.clip(np.round(z.real), -127, 127).astype(np.int8)
    q = np.clip(np.round(z.imag), -127, 127).astype(np.int8)
    return i, q


def punch_gaps(i, q, every, length):
    """Delete blocks of samples, the way the USB transport loses buffer halves.

    This is the impairment the recording actually shows: the stream is whole
    inside a block and a chunk of real time is simply missing between blocks.
    """
    keep = np.ones(len(i), dtype=bool)
    pos = every
    while pos + length < len(i):
        keep[pos:pos + length] = False
        pos += every + length
    return i[keep], q[keep]


def write_hex(path, i, q):
    """{I[7:0], Q[7:0]} per line, the format usb_iq_fifo hands to fm_demod."""
    w = ((i.astype(np.uint16) & 0xFF) << 8) | (q.astype(np.uint16) & 0xFF)
    with open(path, 'w') as f:
        f.write('\n'.join('%04X' % x for x in w))
        f.write('\n')
    return len(w)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fields', type=int, default=3)
    ap.add_argument('--snr', type=float, default=None, help='dB; omit for clean')
    ap.add_argument('--pattern', default='wedge', choices=['wedge', 'bars', 'ramp'])
    ap.add_argument('--gap-every', type=int, default=0,
                    help='delete a block of samples this often (0 = no gaps)')
    ap.add_argument('--gap-len', type=int, default=0)
    ap.add_argument('--out', default='/tmp/iq_synth.hex')
    ap.add_argument('--video-out', default=None,
                    help='also write the baseband video levels, for the Python model')
    a = ap.parse_args()

    v = np.concatenate([field(k % 2, a.pattern) for k in range(a.fields)])
    i, q = modulate(v, a.snr)
    if a.gap_every:
        i, q = punch_gaps(i, q, a.gap_every, a.gap_len)
    n = write_hex(a.out, i, q)
    print('%d fields, %d samples (%.1f ms), SNR %s -> %s'
          % (a.fields, n, n / FS * 1e3, a.snr if a.snr is not None else 'clean', a.out))
    if a.video_out:
        np.savetxt(a.video_out, np.clip(v, 0, 255).astype(int), fmt='%d')
        print('baseband video -> %s' % a.video_out)


if __name__ == '__main__':
    main()
