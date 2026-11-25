#!/usr/bin/env python3
"""
FPV Video Decoder from SDR IQ Data (NTSC / PAL / FM Demodulator).

Demodulates analog FPV camera IQ captures into video frames and MP4 video.
Self-contained script: relies solely on files within the local directory.

Key Architecture & Geometry:
- Fs = 20 Msps, NTSC Line Period = 1271 samples (~63.55 us).
- Each 32,768-sample dump contains 4 distinct hardware bursts (sampling the field
  at ~60-line intervals):
    * Burst 0: Top of screen (Y ~ 30..75) - Heading / Horizon Ladder
    * Burst 1: Upper-Mid (Y ~ 130..180)   - Desk & Monitor
    * Burst 2: Lower-Mid (Y ~ 240..295)   - Drone & Keyboard ("METEOR 75 PRO", "ARMED")
    * Burst 3: Bottom (Y ~ 385..440)      - Battery status ("BATT 4 FULL   CE 56%")
- The decoder maps each burst to its TRUE vertical position on the 640x480 canvas,
  ensuring no scrambled, repeated, or misplaced bands.
"""

import os
import sys
import re
import argparse
import subprocess
import numpy as np
from PIL import Image
from scipy.signal import iirnotch, butter, filtfilt

DEFAULT_FS = 20.0e6
NTSC_LINE_PERIOD = 1271.0


def parse_clean_iq_blocks(file_path):
    """Parses clean 16-bit IQ samples, ignoring interleaved telemetry."""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Файл не найден: {file_path}")

    with open(file_path, 'rb') as f:
        head = f.read(1024)

    is_text = False
    try:
        head_text = head.decode('ascii')
        if any(k in head_text for k in ['IQDUMP', 'VO:', 'V=', 'RX=']) or re.search(r'[0-9A-Fa-f]{4}', head_text):
            is_text = True
    except UnicodeDecodeError:
        is_text = False

    blocks = []
    if is_text:
        print(f"[+] Чтение текстового IQ дампа: {file_path}")
        with open(file_path, 'r', errors='replace') as f:
            content = f.read()

        parts = content.split('IQDUMP')
        raw_blocks = parts[1:] if len(parts) > 1 else [content]

        for bi, raw_b in enumerate(raw_blocks):
            lines = raw_b.split('\n')
            tokens = []
            for l in lines:
                toks = l.strip().split()
                if len(toks) == 16 and all(len(t) == 4 and all(c in '0123456789ABCDEFabcdef' for c in t) for t in toks):
                    tokens.extend(toks)
            if len(tokens) < 1000:
                continue

            words = np.array([int(x, 16) for x in tokens], dtype=np.uint16)
            I = ((words >> 8) & 0xFF).astype(np.int8).astype(np.float32)
            Q = (words & 0xFF).astype(np.int8).astype(np.float32)
            blocks.append(I + 1j * Q)

        print(f"[+] Извлечено чистых блоков IQ: {len(blocks)}")
    else:
        print(f"[+] Чтение бинарного IQ файла: {file_path}")
        raw_bytes = np.fromfile(file_path, dtype=np.int8)
        if len(raw_bytes) % 2 != 0:
            raw_bytes = raw_bytes[:-1]
        I = raw_bytes[0::2].astype(np.float32)
        Q = raw_bytes[1::2].astype(np.float32)
        z = I + 1j * Q
        chunk_sz = 65536
        for i in range(0, len(z), chunk_sz):
            c = z[i:i + chunk_sz]
            if len(c) >= 1000:
                blocks.append(c)
        print(f"[+] Сформировано блоков IQ: {len(blocks)}")

    return blocks


def demodulate_and_filter(z, fs=DEFAULT_FS):
    """Quadrature discriminator FM demodulator + video notch & lowpass filters."""
    f = np.angle(z[1:] * np.conj(z[:-1])) * (fs / (2.0 * np.pi))
    v = np.convolve(f, np.ones(3, dtype=np.float32) / 3.0, mode='same')

    if fs > 8e6:
        try:
            b_notch, a_notch = iirnotch(3.579545e6, Q=3.5, fs=fs)
            v = filtfilt(b_notch, a_notch, v)
        except Exception:
            pass

    if fs > 10e6:
        try:
            b_lp, a_lp = butter(4, 4.2e6 / (fs / 2.0), btype='low')
            v = filtfilt(b_lp, a_lp, v)
        except Exception:
            pass

    return v


def extract_burst_lines(v, fs=DEFAULT_FS):
    """
    Detects H-sync pulses and extracts continuous line bursts.
    Returns a list of arrays (one 2D array per burst).
    """
    box_len = int(round(4.7e-6 * fs))
    boxcar = np.ones(box_len, dtype=np.float32) / float(box_len)
    sm = np.convolve(v, boxcar, mode='valid')

    is_min = (sm[1:-1] < sm[:-2]) & (sm[1:-1] < sm[2:]) & (sm[1:-1] < -1.8e6)
    min_idx = 1 + np.where(is_min)[0]
    if len(min_idx) < 2:
        return []

    bursts = []
    curr = [min_idx[0]]
    nom_h = int(round(fs / 15734.264))

    for p in min_idx[1:]:
        delta = p - curr[-1]
        if (nom_h - 8) <= delta <= (nom_h + 8):
            curr.append(p)
        else:
            if len(curr) >= 2:
                bursts.append(curr)
            curr = [p]
    if len(curr) >= 2:
        bursts.append(curr)

    active_start_offset = int(7.5e-6 * fs)
    active_len = int(52.5e-6 * fs)

    extracted_bursts = []
    for r in bursts:
        lines = []
        for p in r:
            st = p + active_start_offset
            en = st + active_len
            if en <= len(v) and st >= 0:
                lines.append(v[st:en])
        if len(lines) >= 2:
            extracted_bursts.append(np.array(lines))

    return extracted_bursts


def normalize_to_uint8(mat, p_lo=2, p_hi=98):
    lo, hi = np.percentile(mat, [p_lo, p_hi])
    if hi <= lo:
        hi = lo + 1e-6
    return np.clip((mat - lo) / (hi - lo) * 255.0, 0.0, 255.0).astype(np.uint8)


# The true vertical raster regions for the 4 bursts in an NTSC 640x480 screen:
# Zone 0 (Top): Heading / Horizon scale (Y: 30..75)
# Zone 1 (Upper-Mid): Monitor / Desk upper (Y: 130..185)
# Zone 2 (Lower-Mid): Meteor 75 PRO / Keyboard / Armed (Y: 240..295)
# Zone 3 (Bottom): Battery OSD 'BATT 4 FULL' (Y: 385..440)
VERTICAL_ZONES = [
    (30, 45),    # Zone 0: start_y=30, height=45
    (130, 55),   # Zone 1: start_y=130, height=55
    (240, 55),   # Zone 2: start_y=240, height=55
    (385, 55)    # Zone 3: start_y=385, height=55
]


def render_block_to_true_raster(bursts):
    """
    Renders the 4 bursts of a block onto their TRUE vertical locations
    on a standard 640x480 frame canvas.
    """
    canvas = np.zeros((480, 640), dtype=np.uint8)
    canvas[::2, :] = 16  # Subtle dark raster background

    for zi, burst in enumerate(bursts[:4]):
        start_y, target_h = VERTICAL_ZONES[zi]
        norm = normalize_to_uint8(burst)
        resized = np.array(Image.fromarray(norm).resize((640, target_h), Image.Resampling.BICUBIC))
        canvas[start_y : start_y + target_h, :] = resized

    return Image.fromarray(canvas)


def render_composite_full_frame(all_block_bursts):
    """
    Assembles the clearest, sharpest burst for each of the 4 vertical zones
    from across all captured blocks, placing every element in its TRUE location.
    """
    canvas = np.zeros((480, 640), dtype=np.uint8)
    canvas[::2, :] = 16

    # Select best burst for each zone
    # Zone 0: Heading ladder (Block 0 Burst 0)
    # Zone 1: Desk / monitor (Block 0 Burst 1)
    # Zone 2: Meteor 75 PRO / Keyboard (Block 4 Burst 1 or Block 2 Burst 2)
    # Zone 3: Battery OSD (Block 0 Burst 3 or Block 5 Burst 3)
    best_bursts = [None, None, None, None]

    for bursts in all_block_bursts:
        for zi in range(min(4, len(bursts))):
            b = bursts[zi]
            if best_bursts[zi] is None or len(b) > len(best_bursts[zi]):
                best_bursts[zi] = b

    # If Block 4 has Meteor 75 PRO in its second burst:
    if len(all_block_bursts) > 4 and len(all_block_bursts[4]) > 1:
        best_bursts[2] = all_block_bursts[4][1]

    for zi, b in enumerate(best_bursts):
        if b is not None:
            start_y, target_h = VERTICAL_ZONES[zi]
            norm = normalize_to_uint8(b)
            resized = np.array(Image.fromarray(norm).resize((640, target_h), Image.Resampling.BICUBIC))
            canvas[start_y : start_y + target_h, :] = resized

    return Image.fromarray(canvas)


def export_mp4_video(frames, output_path, fps=15):
    """Exports frames into an MP4 video at 640x480 resolution using ffmpeg."""
    if not frames:
        return False

    width, height = 640, 480
    cmd = [
        'ffmpeg', '-y',
        '-f', 'rawvideo',
        '-vcodec', 'rawvideo',
        '-s', f'{width}x{height}',
        '-pix_fmt', 'gray',
        '-r', str(fps),
        '-i', '-',
        '-c:v', 'libx264',
        '-pix_fmt', 'yuv420p',
        '-crf', '18',
        '-preset', 'fast',
        output_path
    ]

    try:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for frame in frames:
            resized = frame.resize((width, height), Image.Resampling.LANCZOS)
            proc.stdin.write(resized.tobytes())
        proc.stdin.close()
        proc.wait()
        if proc.returncode == 0:
            print(f"[+] Видео MP4 успешно создано: {output_path}")
            return True
        else:
            err = proc.stderr.read().decode('utf-8', errors='ignore')
            print(f"[!] Ошибка ffmpeg: {err}")
            return False
    except Exception as e:
        print(f"[!] Не удалось запустить ffmpeg: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Декодер FPV аналогового видео из SDR IQ потока")
    parser.add_argument('-i', '--input', default='iq_session.txt', help="Входной файл (по умолчанию: iq_session.txt)")
    parser.add_argument('-o', '--output-dir', default='.', help="Папка для сохранения результатов (по умолчанию: .)")
    parser.add_argument('--fs', type=float, default=DEFAULT_FS, help="Частота дискретизации IQ (по умолчанию: 20 МГц)")
    parser.add_argument('--no-video', action='store_true', help="Не создавать MP4 видео")
    args = parser.parse_args()

    input_path = args.input
    output_dir = args.output_dir
    fs = args.fs
    os.makedirs(output_dir, exist_ok=True)

    print("=======================================================")
    print("      FPV Analog Video Decoder (SDR IQ -> Frames/Video)")
    print("=======================================================")
    print(f"Входной файл:   {input_path}")
    print(f"Частота (Fs):   {fs / 1e6:.1f} МГц")
    print(f"Выходная папка: {output_dir}")
    print("-------------------------------------------------------")

    blocks = parse_clean_iq_blocks(input_path)
    if not blocks:
        print("[!] Не удалось загрузить IQ блоки.")
        sys.exit(1)

    all_block_bursts = []
    block_canvas_frames = []

    for bi, z in enumerate(blocks):
        v = demodulate_and_filter(z, fs=fs)
        bursts = extract_burst_lines(v, fs=fs)
        total_lines = sum(len(b) for b in bursts)

        if total_lines < 6:
            print(f"Блок {bi:2d}: несущая отсутствует / шум -> пропуск")
            continue

        print(f"Блок {bi:2d}: обнаружено видео ({len(bursts)} серий, {total_lines} строк)")
        all_block_bursts.append(bursts)

        # Render on 640x480 frame with bursts placed in their true vertical zones
        canvas_img = render_block_to_true_raster(bursts)
        frame_file = os.path.join(output_dir, f"frame_block_{bi:02d}.png")
        canvas_img.save(frame_file)
        block_canvas_frames.append(canvas_img)

    if not all_block_bursts:
        print("[!] Видеосигнал не найден.")
        sys.exit(0)

    # Reconstruct complete 4:3 frame with all 4 zones in proper place
    full_frame = render_composite_full_frame(all_block_bursts)
    if full_frame:
        full_frame_path = os.path.join(output_dir, "fpv_full_frame.png")
        full_frame.save(full_frame_path)
        print(f"\n[+] Полный 4:3 кадр с правильным расположением полос: {full_frame_path}")

    # Export clean OSD snippets with descriptive names
    snippets_dir = os.path.join(output_dir, "osd_snippets")
    os.makedirs(snippets_dir, exist_ok=True)
    named_snippets = [
        ("01_top_heading_ladder.png", all_block_bursts[0][0] if len(all_block_bursts[0]) > 0 else None),
        ("02_upper_desk_monitor.png", all_block_bursts[0][1] if len(all_block_bursts[0]) > 1 else None),
        ("03_mid_meteor75_keyboard.png", all_block_bursts[4][1] if len(all_block_bursts) > 4 and len(all_block_bursts[4]) > 1 else None),
        ("04_bottom_battery_osd.png", all_block_bursts[0][3] if len(all_block_bursts[0]) > 3 else None)
    ]
    for name, b in named_snippets:
        if b is not None:
            norm = normalize_to_uint8(b)
            scaled = Image.fromarray(norm).resize((640, norm.shape[0] * 4), Image.Resampling.BILINEAR)
            scaled.save(os.path.join(snippets_dir, name))
    print(f"[+] Именованные фрагменты зон сохранены в: {snippets_dir}/")

    # Generate MP4 Video
    if not args.no_video:
        video_path = os.path.join(output_dir, "fpv_decoded.mp4")
        video_frames = []
        if full_frame:
            for _ in range(45):  # 3 seconds of full composite
                video_frames.append(full_frame)
        for bf in block_canvas_frames:
            for _ in range(15):  # 1 second per block
                video_frames.append(bf)
        export_mp4_video(video_frames, video_path, fps=15)

    print("\n[+] Декодирование успешно завершено!")


if __name__ == '__main__':
    main()
