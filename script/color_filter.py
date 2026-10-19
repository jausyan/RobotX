#!/usr/bin/env python3
"""
color_filter.py  –  colour filter for sea buoys AND close objects (delivery circles)
====================================================================================
Based on the GIMP settings Color Balance (midtones, Cyan-Red +62.9) + Saturation x3.065,
made safe for close objects:

1. auto white : when a big bright object fills part of the frame (board up close), its
                brightest pixels are taken as white and the channels are corrected
                (camera WB 3141 K records white as blue). Sea frames: nothing that big
                and bright -> no change. Smoothed over time in videos (no flicker).
2. red shift  : GIMP midtone colour balance, but only on teal/cyan (water) and red
                hues -> water loses the cyan cast, red buoys get redder, blue / green
                objects are NOT pushed to violet.
3. saturation : colour pushed `sat` times away from grey (Rec.709 luminance kept).

--gimp = exactly the GIMP filter (no auto white, no hue gate).
Pixel positions are not changed. Use the SAME settings for training data and live.

Usage:
    python3 script/color_filter.py ~/Documents/Dataset_RobotX/5.mp4          # -> 5_color.mp4
    python3 script/color_filter.py <image_dir> [<output_dir>]                 # -> <image_dir>_color
    python3 script/color_filter.py <input> --cyan-red 62.9 --sat 3.065 --compare
"""

import argparse
import os
import shutil
import subprocess

import cv2
import numpy as np

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')
VID_EXT = ('.mp4', '.avi', '.mkv', '.mov')
LUMA = np.array([0.0722, 0.7152, 0.2126], np.float32)   # B, G, R (Rec.709)
RED_SHIFT_HUES = ((165.0, 200.0), (300.0, 360.0), (-1.0, 20.0))   # water teal/cyan + reds [deg]


def hue_weight(H, lo, hi, soft=15.0):
    """1 for hue lo..hi [deg], ramps to 0 over `soft` degrees outside."""
    return np.clip(np.minimum(H - (lo - soft), (hi + soft) - H) / soft, 0, 1)


def white_gains(img, bright_L=0.45, frac_lo=0.03, frac_hi=0.10, top=1.0, max_gain=2.5):
    """Per-channel gains (B, G, R) that make the brightest pixels neutral, faded in when
    a big bright object covers frac_lo..frac_hi of the frame. 1, 1, 1 otherwise."""
    f = img.astype(np.float32) / 255.0
    L = (f.max(axis=2) + f.min(axis=2)) * 0.5
    on = np.clip(((L > bright_L).mean() - frac_lo) / (frac_hi - frac_lo), 0, 1)
    if on == 0:
        return np.ones(3, np.float32)
    w = f[L >= np.percentile(L, 100 - top)].mean(axis=0)
    g = np.clip(w.max() / np.maximum(w, 1e-3), 1.0, max_gain)
    return (1.0 + on * (g - 1.0)).astype(np.float32)


def color_filter(img, cyan_red=62.9, magenta_green=0.0, yellow_blue=0.0,
                 preserve_luminosity=True, sat=3.065, gains=None, hue_gate=True):
    f = img.astype(np.float32) / 255.0                   # BGR 0..1
    if gains is not None:
        f = np.clip(f * gains, 0, 1)
    L = (f.max(axis=2) + f.min(axis=2)) * 0.5            # HSL lightness

    # GIMP midtones mask: 1 for lightness ~0.46..0.54, 0 below ~0.21 / above ~0.79
    a, b, k = 0.25, 0.333, 0.7
    w = (np.clip((L - b) / a + 0.5, 0, 1) * np.clip((L + b - 1) / -a + 0.5, 0, 1) * k)
    if hue_gate:
        H = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)[..., 0]   # 0..360
        w = w * np.max([hue_weight(H, lo, hi) for lo, hi in RED_SHIFT_HUES], axis=0)
    shift = np.array([yellow_blue, magenta_green, cyan_red], np.float32) / 100.0
    out = np.clip(f + w[..., None] * shift, 0, 1)

    if preserve_luminosity:                              # back to the input lightness
        hls = cv2.cvtColor(out, cv2.COLOR_BGR2HLS)
        hls[..., 1] = L
        out = cv2.cvtColor(hls, cv2.COLOR_HLS2BGR)

    if sat != 1.0:
        Y = (out @ LUMA)[..., None]
        out = Y + sat * (out - Y)
    return (np.clip(out, 0, 1) * 255.0 + 0.5).astype(np.uint8)


class Filter:
    """Applies the filter frame by frame; in a video the auto-white gains are smoothed."""

    def __init__(self, args, smooth=0.1):
        self.args, self.smooth, self.gains = args, smooth, None

    def __call__(self, img, video=False):
        a = self.args
        gains = None
        if not a.gimp and not a.no_auto_white:
            target = white_gains(img)
            if video and self.gains is not None:
                self.gains = self.gains + self.smooth * (target - self.gains)
            else:
                self.gains = target
            gains = self.gains
        return color_filter(img, a.cyan_red, a.magenta_green, a.yellow_blue,
                            not a.no_preserve_luminosity, a.sat, gains, hue_gate=not a.gimp)


def process_video(src, dst, fn, compare):
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        raise SystemExit(f'cannot open {src}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) * (2 if compare else 1)
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    enc = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
         '-s', f'{w}x{h}', '-r', f'{fps}', '-i', '-',
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', dst],
        stdin=subprocess.PIPE)
    n = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        out = fn(img, video=True)
        enc.stdin.write((np.hstack([img, out]) if compare else out).tobytes())
        n += 1
        if n % 2000 == 0:
            print(f'{n}/{total} frames ...')
    cap.release()
    enc.stdin.close()
    enc.wait()
    print(f'done: {n} frames -> {dst}')


def main():
    ap = argparse.ArgumentParser(description='Colour filter for sea buoys and close objects')
    ap.add_argument('input', help='video file or image folder')
    ap.add_argument('output', nargs='?', help='default: <input>_color(.mp4)')
    ap.add_argument('--cyan-red', type=float, default=62.9, help='-100..100, + = more red')
    ap.add_argument('--magenta-green', type=float, default=0.0, help='-100..100, + = more green')
    ap.add_argument('--yellow-blue', type=float, default=0.0, help='-100..100, + = more blue')
    ap.add_argument('--no-preserve-luminosity', action='store_true')
    ap.add_argument('--sat', type=float, default=3.065, help='saturation scale (1 = unchanged)')
    ap.add_argument('--no-auto-white', action='store_true', help='no white correction for close objects')
    ap.add_argument('--gimp', action='store_true', help='exactly the GIMP filter (no auto white, no hue gate)')
    ap.add_argument('--compare', action='store_true', help='write original | filtered side by side')
    args = ap.parse_args()
    fn = Filter(args)

    src = os.path.abspath(os.path.expanduser(args.input)).rstrip('/')
    if os.path.isfile(src) and src.lower().endswith(VID_EXT):
        dst = (os.path.abspath(os.path.expanduser(args.output)) if args.output
               else os.path.splitext(src)[0] + '_color.mp4')
        if dst == src:
            raise SystemExit('output must be a different file than input')
        process_video(src, dst, fn, args.compare)
        return

    dst = os.path.abspath(os.path.expanduser(args.output)) if args.output else src + '_color'
    if dst == src:
        raise SystemExit('output must be a different folder than input')
    n = 0
    for root, _, files in os.walk(src):
        out_root = os.path.join(dst, os.path.relpath(root, src))
        os.makedirs(out_root, exist_ok=True)
        for name in sorted(files):
            in_path, out_path = os.path.join(root, name), os.path.join(out_root, name)
            if not name.lower().endswith(IMG_EXT):
                shutil.copy2(in_path, out_path)   # keep labels (.txt) etc.
                continue
            img = cv2.imread(in_path)
            if img is None:
                print(f'skip (unreadable): {in_path}')
                continue
            out = fn(img)
            cv2.imwrite(out_path, np.hstack([img, out]) if args.compare else out,
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            n += 1
    print(f'done: {n} images -> {dst}')


if __name__ == '__main__':
    main()
