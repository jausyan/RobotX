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
    python3 record_video.py --width 1280 --height 720 --fourcc MJPG   # camera mode (default: driver default)
    python3 record_video.py --stream-host 192.168.0.99                # also stream to GStreamer (H.264 RTP/UDP)

Stream receiver (same as vision_geo / vision_video stream):
    gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
        ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false

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
CALIB_SIZE = (640, 480)
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
    p.add_argument("--width",        type=int, default=0,
                   help="Camera width  (0 = driver default), e.g. 640 / 1280")
    p.add_argument("--height",       type=int, default=0,
                   help="Camera height (0 = driver default), e.g. 480 / 720")
    p.add_argument("--fourcc",       default="",
                   help="Camera pixel format, e.g. MJPG (needed for 720p @ 30 fps); empty = driver default")
    p.add_argument("--stream-host",  default="",
                   help="Also stream the recorded frames to this host (H.264 RTP/UDP); empty = no stream")
    p.add_argument("--stream-port",  type=int, default=5000)
    p.add_argument("--stream-bitrate", type=int, default=2000, help="Stream bitrate [kbps]")
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

    # Camera mode: same as vision_video camera_fourcc / camera_width / camera_height
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # Actual resolution (the driver picks the nearest mode it supports)
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    print(f"[INFO] Device       : {dev}  ({w}x{h} @ {fps:.0f} fps)")
    if (args.width and args.width != w) or (args.height and args.height != h):
        print(f"[WARN] Asked {args.width}x{args.height}, camera gives {w}x{h}")

    # ── Pre-compute undistortion maps (once) ──────────────────────────────────
    if not args.no_undistort and (w, h) != CALIB_SIZE:
        # calibration is for 640x480; other modes have a different field of view
        print(f"[WARN] Undistort    : OFF  (calibrated at {CALIB_SIZE[0]}x{CALIB_SIZE[1]}, camera is {w}x{h})")
        map1 = map2 = None
    elif not args.no_undistort:
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

    # ── GStreamer stream (same pipeline as vision_geo / vision_video) ─────────
    stream = None
    if args.stream_host:
        fps_i = max(1, int(round(fps)))
        pipeline = (
            "appsrc is-live=true do-timestamp=true format=time ! "
            f"video/x-raw,format=BGR,width={w},height={h},framerate={fps_i}/1 ! "
            "videoconvert ! "
            f"x264enc tune=zerolatency bitrate={args.stream_bitrate} speed-preset=superfast key-int-max={fps_i} byte-stream=true ! "
            "rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={args.stream_host} port={args.stream_port} sync=false async=false")
        stream = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(fps_i), (w, h), True)
        if not stream.isOpened():
            print(f"[ERROR] Cannot open GStreamer stream. Pipeline: {pipeline}")
            cap.release()
            writer.release()
            sys.exit(1)
        print(f"[INFO] Stream       : udp://{args.stream_host}:{args.stream_port}  ({w}x{h} @ {fps_i} fps, {args.stream_bitrate} kbps)")
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
        if stream is not None:
            stream.write(frame)
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
    if stream is not None:
        stream.release()

    elapsed = time.time() - t0
    size_mb = out_path.stat().st_size / 1e6 if out_path.exists() else 0
    print(f"\n\n[INFO] Saved  → {out_path}")
    print(f"[INFO] Frames : {frames}  |  Duration : {elapsed:.1f}s  |  Size : {size_mb:.1f} MB")


if __name__ == "__main__":
    main()
