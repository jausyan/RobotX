#!/usr/bin/env python3
"""
glare_removal.py  –  remove sun glare / haze from sea images
============================================================
1. glow removal : divide by a heavily blurred copy (kills the big sun glow, keeps small buoys)
2. CLAHE        : local contrast on the L channel only (hue is untouched)
3. saturation   : boost so red / green / blue buoys separate from the water

Pixel positions are not changed, so labels, PnP and GPS projection stay valid.
Use the SAME settings for training data and for the live pipeline.

Usage:
    python3 script/glare_removal.py ~/Documents/Dataset_RobotX/4                # -> ~/Documents/Dataset_RobotX/4_glare
    python3 script/glare_removal.py <input_dir> <output_dir>
    python3 script/glare_removal.py <input_dir> --compare                       # side by side original | processed
    python3 script/glare_removal.py <input_dir> --sigma 40 --clip 2.0 --sat 1.6
"""

import argparse
import os
import shutil

import cv2
import numpy as np

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def remove_glare(img, sigma=40.0, clip=2.0, tile=8, sat=1.6):
    f = img.astype(np.float32)
    bg = cv2.GaussianBlur(f, (0, 0), sigma)
    flat = np.clip(f / (bg + 1.0) * bg.mean(axis=(0, 1)), 0, 255).astype(np.uint8)

    lab = cv2.cvtColor(flat, cv2.COLOR_BGR2LAB)
    lab[..., 0] = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile)).apply(lab[..., 0])
    out = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    hsv = cv2.cvtColor(out, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 1] = np.clip(hsv[..., 1] * sat, 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def main():
    ap = argparse.ArgumentParser(description='Remove sun glare from a folder of images')
    ap.add_argument('input')
    ap.add_argument('output', nargs='?', help='default: <input>_glare')
    ap.add_argument('--sigma', type=float, default=40.0, help='glow blur sigma (px)')
    ap.add_argument('--clip', type=float, default=2.0, help='CLAHE clip limit')
    ap.add_argument('--tile', type=int, default=8, help='CLAHE tile grid')
    ap.add_argument('--sat', type=float, default=1.6, help='saturation gain')
    ap.add_argument('--compare', action='store_true', help='write original | processed side by side')
    args = ap.parse_args()

    src = os.path.abspath(os.path.expanduser(args.input)).rstrip('/')
    dst = os.path.abspath(os.path.expanduser(args.output)) if args.output else src + '_glare'
    if dst == src:
        raise SystemExit('output must be a different folder than input')

    n = 0
    for root, _, files in os.walk(src):
        out_root = os.path.join(dst, os.path.relpath(root, src))
        os.makedirs(out_root, exist_ok=True)
        for name in sorted(files):
            in_path = os.path.join(root, name)
            out_path = os.path.join(out_root, name)
            if not name.lower().endswith(IMG_EXT):
                shutil.copy2(in_path, out_path)   # keep labels (.txt) etc.
                continue
            img = cv2.imread(in_path)
            if img is None:
                print(f'skip (unreadable): {in_path}')
                continue
            out = remove_glare(img, args.sigma, args.clip, args.tile, args.sat)
            if args.compare:
                out = np.hstack([img, out])
            cv2.imwrite(out_path, out, [cv2.IMWRITE_JPEG_QUALITY, 95])
            n += 1
            if n % 200 == 0:
                print(f'{n} images ...')
    print(f'done: {n} images -> {dst}')


if __name__ == '__main__':
    main()
