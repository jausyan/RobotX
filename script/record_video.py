#!/usr/bin/env python3
"""
record_video.py  –  dataset recorder matching vision_video pipeline exactly
============================================================================
Captures raw frames from /dev/video0 at native 640×480, then applies
undistortion using the same camera matrix & distortion coeffs as
vision_video (vision_video.yaml).

NOTE: vision_video itself does NOT undistort the raw frame before inference.
      The undistortion here is applied so the SAVED frames are geometrically
      corrected — useful for dataset labelling. If you want truly identical
      raw frames (no undistort), pass --no-undistort.

Usage:
    python3 record_video.py                  # undistorted, native 640x480
    python3 record_video.py --no-undistort   # raw frames (same as vision_video input)
    python3 record_video.py --duration 60    # stop after 60 s
    python3 record_video.py --output out.avi # custom filename

Output saved to: <script folder>/video/
Press Ctrl+C to stop.
"""

import cv2
import numpy as np
import sys
import time
import signal
import argparse
from datetime import datetime
from pathlib import Path

# ── Output folder ────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
VIDEO_DIR  = SCRIPT_DIR / "video"

# ── Camera calibration — from vision_video.yaml ──────────────────────────────
# Calibrated at 640×480 (native camera resolution)
CAMERA_MATRIX = np.array([
    [504.99132615,   0.0,         318.4012728 ],
    [  0.0,        506.13168687,  243.99035432],
    [  0.0,          0.0,           1.0       ]
], dtype=np.float64)

DIST_COEFFS = np.array(
    [0.07771398, -0.18711237, -0.00182709, 0.00026326, -0.06212367],
    dtype=np.float64
)

# ── Args ─────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(
        description="Record dataset video matching vision_video input pipeline."
    )
    p.add_argument("--device",       default="/dev/video0",
                   help="Camera device (default: /dev/video0)")
    p.add_argument("--output",       default=None,
                   help="Output filename (default: auto-timestamp .avi)")
    p.add_argument("--duration",     type=int, default=0,
                   help="Max recording seconds (0 = unlimited)")
    p.add_argument("--no-undistort", action="store_true",
                   help="Save raw frames without undistortion (same as vision_video input)")
    return p.parse_args()


def main():
    args = parse_args()

    # ── Open camera (same as vision_video: V4L2, no forced resolution) ───────
    dev = args.device
    try:
        idx = int(dev.replace("/dev/video", ""))
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    except ValueError:
        cap = cv2.VideoCapture(dev)

    if not cap.isOpened():
        cap = cv2.VideoCapture(dev)  # fallback without V4L2 flag

    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera: {dev}")
        sys.exit(1)

    # Read actual resolution (do NOT force width/height — match vision_video)
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    print(f"[INFO] Device       : {dev}  ({w}x{h} @ {fps:.0f} fps)")

    # ── Pre-compute undistortion maps (once) ──────────────────────────────────
    if not args.no_undistort:
        # alpha=0 → no black borders (crops to valid pixels)
        new_K, roi = cv2.getOptimalNewCameraMatrix(
            CAMERA_MATRIX, DIST_COEFFS, (w, h), alpha=0, newImgSize=(w, h)
        )
        map1, map2 = cv2.initUndistortRectifyMap(
            CAMERA_MATRIX, DIST_COEFFS, None, new_K, (w, h), cv2.CV_16SC2
        )
        rx, ry, rw, rh = roi
        print(f"[INFO] Undistort    : ON  (valid ROI {rw}x{rh} @ {rx},{ry})")
    else:
        map1 = map2 = None
        print("[INFO] Undistort    : OFF  (raw frames — identical to vision_video input)")

    # ── Prepare output ────────────────────────────────────────────────────────
    VIDEO_DIR.mkdir(parents=True, exist_ok=True)
    out_name = args.output or f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    out_path = VIDEO_DIR / out_name

    fourcc = cv2.VideoWriter_fourcc(*"MJPG")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
    if not writer.isOpened():
        print("[ERROR] Cannot open VideoWriter (MJPG). Trying XVID…")
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
    if not writer.isOpened():
        print("[ERROR] VideoWriter failed. Check OpenCV codec support.")
        cap.release()
        sys.exit(1)

    print(f"[INFO] Output       : {out_path}")
    print(f"[INFO] Duration     : {'unlimited' if args.duration == 0 else str(args.duration) + 's'}")
    print("[INFO] Recording … Ctrl+C to stop\n")

    # ── Graceful stop ─────────────────────────────────────────────────────────
    running = [True]
    def _stop(*_): running[0] = False
    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    frames  = 0
    dropped = 0
    t0      = time.time()

    while running[0]:
        ok, frame = cap.read()
        if not ok or frame is None:
            dropped += 1
            if dropped > 30:
                print("\n[ERROR] Too many dropped frames — camera lost.")
                break
            time.sleep(0.01)
            continue

        dropped = 0  # reset on success

        # Apply undistortion
        if map1 is not None:
            frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

        writer.write(frame)
        frames  += 1
        elapsed  = time.time() - t0

        # Progress every ~1 second
        if frames % max(1, int(fps)) == 0:
            print(f"\r  {int(elapsed//60):02d}:{int(elapsed%60):02d}  |  {frames} frames", end="", flush=True)

        if args.duration > 0 and elapsed >= args.duration:
            print(f"\n[INFO] Duration limit reached ({args.duration}s).")
            break

    cap.release()
    writer.release()

    elapsed = time.time() - t0
    size_mb = out_path.stat().st_size / 1e6 if out_path.exists() else 0
    print(f"\n\n[INFO] Saved  → {out_path}")
    print(f"[INFO] Frames : {frames}  |  Duration : {elapsed:.1f}s  |  Size : {size_mb:.1f} MB")


if __name__ == "__main__":
    main()
