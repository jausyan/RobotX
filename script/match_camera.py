#!/usr/bin/env python3
"""
match_camera.py  –  make a video look like it was recorded with config/startup.json
=====================================================================================
Camera controls (exposure, white balance, contrast, ...) act inside the camera, so they
cannot be re-applied to a recorded video. Instead this fits ONE fixed gain + offset per colour
channel (B, G, R) — what exposure / white balance / contrast do — that maps the video onto
reference images that WERE taken with startup.json, and applies it to every frame
(like a camera setting: same for the whole video).

Fit on frames of the same kind of scene (sea). Sun glare / saturated pixels are left out.

Usage:
    python3 script/match_camera.py ~/Documents/Dataset_RobotX/5.mp4 \\
        --ref ~/Documents/Dataset_RobotX/4 --fit-start 40 --fit-end 160
        # -> ~/Documents/Dataset_RobotX/5_startup.mp4  (+ 5_startup_lut.npz)
    python3 script/match_camera.py <video> --lut 5_startup_lut.npz      # reuse a fitted curve
"""

import argparse
import glob
import os
import subprocess

import cv2
import numpy as np

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def channel_hist(imgs, glare=235):
    """Per-channel histograms, without glare / saturated pixels (any channel >= glare)."""
    h = np.zeros((3, 256), np.float64)
    for img in imgs:
        keep = img.max(axis=2) < glare
        for c in range(3):
            h[c] += np.bincount(img[..., c][keep], minlength=256)
    return h


def percentile(hist, q):
    c = np.cumsum(hist); c /= c[-1]
    return float(np.searchsorted(c, q / 100.0))


def fit_lut(src_hist, ref_hist, lo=5.0, hi=95.0):
    """Per channel: map the src lo..hi percentile range onto the ref one (straight line),
    black stays black and white stays white. The fit only knows the sea range; darker /
    brighter values are ramped to 0 / 255 so dark objects get no colour cast."""
    lut = np.zeros((3, 256), np.uint8)
    x = np.arange(256, dtype=np.float64)
    for c in range(3):
        s0, s1 = percentile(src_hist[c], lo), percentile(src_hist[c], hi)
        r0, r1 = percentile(ref_hist[c], lo), percentile(ref_hist[c], hi)
        s0 = max(s0, 16.0)                     # src channel clipped near 0 (e.g. red under water)
        s1 = max(s1, s0 + 1.0)
        lut[c] = np.clip(np.round(np.interp(x, [0, s0, s1, 255], [0, r0, r1, 255])), 0, 255).astype(np.uint8)
    return lut


def apply_lut(img, lut):
    return cv2.merge([cv2.LUT(img[..., c], lut[c]) for c in range(3)])


def ref_images(path, n=300):
    files = sorted(f for f in glob.glob(os.path.join(path, '**', '*'), recursive=True)
                   if f.lower().endswith(IMG_EXT))
    files = [files[i] for i in np.linspace(0, len(files) - 1, min(n, len(files))).astype(int)]
    imgs = [im for im in (cv2.imread(f) for f in files) if im is not None]
    # typical frames only (mean colour near the median) — skips ground / grass / concrete shots
    means = np.array([im.reshape(-1, 3).mean(axis=0) for im in imgs])
    dist = np.linalg.norm(means - np.median(means, axis=0), axis=1)
    return [im for im, d in zip(imgs, dist) if d <= np.percentile(dist, 70)]


def video_frames(path, start, end, n=150):
    cap = cv2.VideoCapture(path)
    out = []
    for t in np.linspace(start, end, n):
        cap.set(cv2.CAP_PROP_POS_MSEC, t * 1000.0)
        ok, im = cap.read()
        if ok:
            out.append(im)
    cap.release()
    return out


def main():
    ap = argparse.ArgumentParser(description='Match a video to the startup.json camera look')
    ap.add_argument('input', help='video file')
    ap.add_argument('output', nargs='?', help='default: <input>_startup.mp4')
    ap.add_argument('--ref', help='folder of images taken with startup.json')
    ap.add_argument('--fit-start', type=float, default=0.0, help='[s] sea part of the video used for the fit')
    ap.add_argument('--fit-end', type=float, default=60.0)
    ap.add_argument('--lut', help='use this fitted curve (.npz) instead of fitting')
    args = ap.parse_args()

    src = os.path.abspath(os.path.expanduser(args.input))
    base = os.path.splitext(src)[0]
    dst = os.path.abspath(os.path.expanduser(args.output)) if args.output else base + '_startup.mp4'
    if dst == src:
        raise SystemExit('output must be a different file than input')

    if args.lut:
        lut = np.load(os.path.expanduser(args.lut))['lut']
    else:
        if not args.ref:
            raise SystemExit('need --ref <startup.json images> or --lut <file.npz>')
        refs = ref_images(os.path.expanduser(args.ref))
        frames = video_frames(src, args.fit_start, args.fit_end)
        if not refs or not frames:
            raise SystemExit('no reference images / video frames to fit')
        lut = fit_lut(channel_hist(frames), channel_hist(refs))
        lut_path = os.path.splitext(dst)[0] + '_lut.npz'
        np.savez(lut_path, lut=lut)
        print(f'fitted on {len(frames)} video frames vs {len(refs)} reference images -> {lut_path}')

    cap = cv2.VideoCapture(src)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
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
        enc.stdin.write(apply_lut(img, lut).tobytes())
        n += 1
        if n % 2000 == 0:
            print(f'{n}/{total} frames ...')
    cap.release()
    enc.stdin.close()
    enc.wait()
    print(f'done: {n} frames -> {dst}')


if __name__ == '__main__':
    main()
