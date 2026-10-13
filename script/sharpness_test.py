#!/usr/bin/env python3
"""Live sharpness check for the downward camera (tune camera_setup.sh on the ground).

Point the camera at something FAR away (> 20 m, e.g. out of a window or across a field) and
compare scores: higher = sharper. Use it to pick EXPOSURE / GAIN / FOCUS, or to compare cameras
(e.g. Logitech C922 vs Yahboom) in the same spot and light.

  python3 script/sharpness_test.py                                  # 1280x720 MJPG, /dev/video0
  python3 script/sharpness_test.py --stream 100.95.74.67:5000       # also view it on the ground station
  python3 script/sharpness_test.py --device 2 --width 1280 --height 720

Ground station viewer for --stream:
  gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
    ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
"""
import argparse
import time

import cv2


def sharpness(gray):
    """Variance of the Laplacian: more fine detail (edges) = higher."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--device', default='0', help='camera index or /dev/videoN (default 0)')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fourcc', default='MJPG')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--stream', default='', help='host:port to stream the annotated view (H.264 over UDP)')
    a = ap.parse_args()

    dev = int(a.device.replace('/dev/video', '')) if a.device.replace('/dev/video', '').isdigit() else a.device
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit(f'cannot read from camera {a.device}')
    h, w = frame.shape[:2]
    print(f'camera {a.device}: {w}x{h} (asked {a.width}x{a.height} {a.fourcc} @ {a.fps}) — Ctrl+C to stop')

    writer = None
    if a.stream:
        host, port = a.stream.split(':')
        sw, sh = 848, 480
        pipeline = ('appsrc is-live=true do-timestamp=true format=time ! '
                    f'video/x-raw,format=BGR,width={sw},height={sh},framerate=15/1 ! videoconvert ! '
                    'x264enc tune=zerolatency bitrate=2000 speed-preset=superfast key-int-max=15 ! '
                    f'rtph264pay config-interval=1 pt=96 ! udpsink host={host} port={port} sync=false async=false')
        writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, 15.0, (sw, sh), True)
        print('streaming to', a.stream if writer.isOpened() else 'FAILED (no GStreamer in cv2?)')

    n, t0, last_print, last_stream = 0, time.time(), 0.0, 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            n += 1
            now = time.time()
            if now - last_print >= 0.5:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                cy, cx = h // 2, w // 2
                center = gray[cy - h // 6:cy + h // 6, cx - w // 6:cx + w // 6]
                fps = n / (now - t0)
                print(f'sharpness  full {sharpness(gray):7.1f}   center {sharpness(center):7.1f}   '
                      f'brightness {gray.mean():5.1f}   fps {fps:4.1f}', flush=True)
                last_print, n, t0 = now, 0, now
            if writer is not None and writer.isOpened() and now - last_stream >= 1 / 15:
                view = cv2.resize(frame, (848, 480))
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                cv2.putText(view, f'sharpness {sharpness(gray):.0f}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                writer.write(view)
                last_stream = now
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if writer is not None:
            writer.release()


if __name__ == '__main__':
    main()
