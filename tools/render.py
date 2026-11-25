#!/usr/bin/env python3
"""Offline lab for the FPV video chain.

Reads a recorded IQ session (captures/iq_session.txt), demodulates it the way
the FPGA does, and renders the result as a PNG so changes can be judged by eye
without touching the hardware.

The point is to try processing changes here first: whatever wins on the
recording is what gets implemented in Verilog.

    python3 tools/render.py [--out FILE] [--variant NAME]
"""
import re, sys, zlib, struct, argparse
import numpy as np

FS = 20e6
LINE_NOM = 1271

def load_blocks(path):
    raw = open(path, 'rb').read().decode('ascii', 'replace')
    out = []
    for b in raw.split('IQDUMP')[1:]:
        t = re.findall(r'\b([0-9A-F]{4})\b', b)
        if len(t) < 20000:
            continue
        w = np.array([int(x, 16) for x in t[:32768]], dtype=np.uint16)
        I = ((w >> 8) & 0xFF).astype(np.int8).astype(float)
        Q = (w & 0xFF).astype(np.int8).astype(float)
        out.append(I + 1j*Q)
    return out

def score(z):
    """line periodicity of the demodulated signal: >0.5 means real video"""
    f = np.angle(z[1:]*np.conj(z[:-1]))
    sm = np.convolve(f, np.ones(16)/16, mode='valid')
    x = sm - sm.mean()
    ac = np.correlate(x, x, 'full')[len(x)-1:]
    ac /= ac[0]
    lo, hi = int(55e-6*FS), int(75e-6*FS)
    j = lo + int(np.argmax(ac[lo:hi]))
    return ac[j], j

# ---- processing variants: add new ones here and compare -------------------
def v_ma3(f):
    """what the FPGA does now: 3-tap moving average (notches the 6 MHz audio)"""
    return np.convolve(f, np.ones(3)/3, mode='same')

def v_raw(f):
    return f

def v_iir8(f):
    a = 1/8; acc = 0.0; y = np.empty_like(f)
    for i in range(len(f)):
        acc += f[i] - acc*a; y[i] = acc*a
    return y

def v_ma3_ma3(f):
    return np.convolve(v_ma3(f), np.ones(3)/3, mode='same')

def v_ma5(f):
    return np.convolve(f, np.ones(5)/5, mode='same')

VARIANTS = {'ma3': v_ma3, 'raw': v_raw, 'iir8': v_iir8, 'ma3x2': v_ma3_ma3, 'ma5': v_ma5}

def slice_lines(v, report=False):
    """Cut the stream into lines, locked to the true line period.

    Thresholding each sync pulse separately makes every line start at its own
    offset and the picture shears. The autocorrelation peak alone is not precise
    enough either: an error of a fraction of a sample accumulates line over line
    and tilts the image. So sweep the period and keep whichever value makes
    consecutive lines agree best - that is exactly the criterion for "not
    sheared".
    """
    A = int(9.5e-6*FS); B = int(62.0e-6*FS)

    def cut(period_f):
        first = int(np.argmin(v[:int(period_f)]))
        rows, k = [], 0
        while True:
            start = first + int(round(k*period_f))
            if start + B >= len(v):
                break
            rows.append(v[start+A:start+B]); k += 1
        if len(rows) < 6:
            return None
        L = min(len(r) for r in rows)
        return np.array([r[:L] for r in rows])

    best = (None, -2.0, 0.0)
    for period_f in np.arange(1255.0, 1290.0, 0.05):
        img = cut(period_f)
        if img is None:
            continue
        c = np.nanmean([np.corrcoef(img[i], img[i+1])[0, 1]
                        for i in range(len(img)-1)])
        if c > best[1]:
            best = (img, c, period_f)
    img, c, period_f = best
    if img is None:
        return None
    if report:
        print('   line period %.2f samples (%.3f us), line agreement %.3f'
              % (period_f, period_f/FS*1e6, c))
    return img

def normalise(img, lo_pct=2, hi_pct=98):
    lo, hi = np.percentile(img, [lo_pct, hi_pct])
    return np.clip((img-lo)/max(hi-lo, 1e-9)*255, 0, 255).astype(np.uint8)

def write_png(path, img):
    h, w = img.shape
    rows = b''.join(b'\x00' + img[y].tobytes() for y in range(h))
    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        return c + struct.pack('>I', zlib.crc32(tag+data) & 0xffffffff)
    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(rows, 9))
           + chunk(b'IEND', b''))
    open(path, 'wb').write(png)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='captures/iq_session.txt')
    ap.add_argument('--out', default='captures/frame.png')
    ap.add_argument('--variant', default='ma3', choices=list(VARIANTS))
    ap.add_argument('--scale', type=int, default=2)
    ap.add_argument('--all', action='store_true', help='render every variant')
    a = ap.parse_args()

    blocks = load_blocks(a.src)
    print('blocks: %d' % len(blocks))
    scored = [(score(z)[0], i, z) for i, z in enumerate(blocks)]
    scored.sort(reverse=True, key=lambda s: s[0])
    good = [s for s in scored if s[0] > 0.5]
    print('blocks carrying video: %d' % len(good))
    for s, i, z in scored[:8]:
        print('   block %2d  periodicity=%.3f  |z|=%5.1f  clip=%.2f%%'
              % (i, s, np.abs(z).mean(),
                 100*((np.abs(z.real) >= 127) | (np.abs(z.imag) >= 127)).mean()))
    if not good:
        print('no usable block'); return
    _, idx, z = good[0]
    f = np.angle(z[1:]*np.conj(z[:-1]))/(2*np.pi)*FS

    names = list(VARIANTS) if a.all else [a.variant]
    for nm in names:
        img = slice_lines(VARIANTS[nm](f), report=True)
        if img is None:
            print('%-6s : could not slice lines' % nm); continue
        im = normalise(img)
        cs = np.nanmean([np.corrcoef(im[i].astype(float), im[i+1].astype(float))[0,1]
                         for i in range(len(im)-1)])
        det = np.mean([np.std(np.diff(r.astype(float))) for r in im])
        out = a.out if len(names) == 1 else a.out.replace('.png', '_%s.png' % nm)
        big = np.repeat(np.repeat(im, a.scale, axis=0), 1, axis=1)
        write_png(out, big)
        print('%-6s : %dx%d  line-corr=%.3f  detail=%5.1f  -> %s'
              % (nm, im.shape[1], im.shape[0], cs, det, out))

if __name__ == '__main__':
    main()
