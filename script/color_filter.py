#!/usr/bin/env python3
"""
color_filter.py  –  GIMP "Color Balance (midtones) + Saturation" for sea images
================================================================================
Same maths as GIMP 2.10:
1. Color Balance, midtones: add cyan_red / magenta_green / yellow_blue (-100..100) to the
   R / G / B channels of mid-lightness pixels (the water), keeping the lightness
   (GIMP "Preserve luminosity") → removes the cyan cast
2. Saturation: colour pushed `sat` times away from grey (Rec.709 luminance kept)
   → red / green / blue buoys stand out from the water

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


def color_filter(img, cyan_red=62.9, magenta_green=0.0, yellow_blue=0.0,
                 preserve_luminosity=True, sat=3.065):
    f = img.astype(np.float32) / 255.0                   # BGR 0..1
    L = (f.max(axis=2) + f.min(axis=2)) * 0.5            # HSL lightness

    # GIMP midtones mask: 1 for lightness ~0.46..0.54, 0 below ~0.21 / above ~0.79
    a, b, k = 0.25, 0.333, 0.7
    mid = (np.clip((L - b) / a + 0.5, 0, 1) * np.clip((L + b - 1) / -a + 0.5, 0, 1) * k)
    shift = np.array([yellow_blue, magenta_green, cyan_red], np.float32) / 100.0
    out = np.clip(f + mid[..., None] * shift, 0, 1)

    if preserve_luminosity:                              # back to the input lightness
        hls = cv2.cvtColor(out, cv2.COLOR_BGR2HLS)
        hls[..., 1] = L
        out = cv2.cvtColor(hls, cv2.COLOR_HLS2BGR)

    if sat != 1.0:
        Y = (out @ LUMA)[..., None]
        out = Y + sat * (out - Y)
    return (np.clip(out, 0, 1) * 255.0 + 0.5).astype(np.uint8)


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
        out = fn(img)
        enc.stdin.write((np.hstack([img, out]) if compare else out).tobytes())
        n += 1
        if n % 2000 == 0:
            print(f'{n}/{total} frames ...')
    cap.release()
    enc.stdin.close()
    enc.wait()
    print(f'done: {n} frames -> {dst}')


def main():
    ap = argparse.ArgumentParser(description='GIMP colour balance (midtones) + saturation')
    ap.add_argument('input', help='video file or image folder')
    ap.add_argument('output', nargs='?', help='default: <input>_color(.mp4)')
    ap.add_argument('--cyan-red', type=float, default=62.9, help='-100..100, + = more red')
    ap.add_argument('--magenta-green', type=float, default=0.0, help='-100..100, + = more green')
    ap.add_argument('--yellow-blue', type=float, default=0.0, help='-100..100, + = more blue')
    ap.add_argument('--no-preserve-luminosity', action='store_true')
    ap.add_argument('--sat', type=float, default=3.065, help='saturation scale (1 = unchanged)')
    ap.add_argument('--compare', action='store_true', help='write original | filtered side by side')
    args = ap.parse_args()

    def fn(img):
        return color_filter(img, args.cyan_red, args.magenta_green, args.yellow_blue,
                            not args.no_preserve_luminosity, args.sat)

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
