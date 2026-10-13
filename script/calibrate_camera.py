#!/usr/bin/env python3
"""Camera calibration (checkerboard) in the FLIGHT camera mode -> writes vision_geo / vision_video configs.

1. Print a checkerboard and glue it FLAT on a stiff board. Count the INNER corners (where 4 squares
   meet) = squares - 1 in each direction, e.g. an 11x8-square board has 10x7 inner corners
   (--cols 10 --rows 7). Measure one square in mm.
2. Run camera_setup.sh first (focus locked at infinity, like in flight).
3. Run this on the drone and move the board slowly: near and far, tilted left/right/up/down and into
   ALL corners of the image. A frame is captured automatically when the board is sharp and in a NEW spot.
4. After --count captures it calibrates, prints the error (< 0.5 px good, < 1.0 px ok) and writes
   camera_matrix / distortion_coeffs into the configs (backup: <config>.bak-calib).

  python3 script/calibrate_camera.py --cols 9 --rows 6 --square-mm 25
  python3 script/calibrate_camera.py --cols 9 --rows 6 --square-mm 25 --stream 100.95.74.67:5000   # live preview
  python3 script/calibrate_camera.py --cols 9 --rows 6 --square-mm 25 --from-dir calib/2026-10-10_1200  # redo from saved images
  python3 script/calibrate_camera.py --cols 9 --rows 6 --square-mm 25 --device board.mp4   # from a recorded video

Preview on the ground station (with --stream):
  gst-launch-1.0 udpsrc port=5000 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
    ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
"""
import argparse
import glob
import os
import re
import shutil
import time

import cv2
import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIGS = [os.path.join(WS, 'src/vision_geo/config/vision_geo.yaml'),
                   os.path.join(WS, 'src/vision_video/config/vision_video.yaml')]
FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
SUBPIX = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def find_board(gray, pattern):
    ok, corners = cv2.findChessboardCorners(gray, pattern, FIND_FLAGS)
    if not ok:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX)


def board_signature(corners, w, h):
    """(centre x, centre y, size) normalised to the image, to tell 'new spot' from 'same spot'."""
    pts = corners.reshape(-1, 2)
    (x0, y0), (x1, y1) = pts.min(0), pts.max(0)
    return np.array([(x0 + x1) / 2 / w, (y0 + y1) / 2 / h, np.sqrt((x1 - x0) * (y1 - y0) / (w * h))])


def coverage(sigs):
    """3x3 grid of the image: which cells already have a board centre in them."""
    grid = np.zeros((3, 3), int)
    for s in sigs:
        grid[min(2, int(s[1] * 3)), min(2, int(s[0] * 3))] += 1
    return grid


def calibrate(obj_pts, img_pts, size):
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    errs = []
    for o, i, r, t in zip(obj_pts, img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, D)
        errs.append(cv2.norm(i, proj, cv2.NORM_L2) / np.sqrt(len(proj)))
    return rms, K, D.reshape(-1)[:5], np.array(errs)


def write_configs(paths, K, D, size):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    k_re = re.compile(r'^(\s*)camera_matrix:\s*\[[^\]]*\]', re.M)
    d_re = re.compile(r'^(\s*)distortion_coeffs:\s*\[[^\]]*\]', re.M)

    def k_text(m):
        ind = m.group(1)
        pad = ' ' * (len(ind) + len('camera_matrix: ['))
        return (f'{ind}camera_matrix: [{fx:.8f}, 0.0, {cx:.8f},\n'
                f'{pad}0.0, {fy:.8f}, {cy:.8f},\n{pad}0.0, 0.0, 1.0]')

    def d_text(m):
        return f'{m.group(1)}distortion_coeffs: [' + ', '.join(f'{v:.8f}' for v in D) + ']'

    for path in paths:
        if not os.path.isfile(path):
            print(f'  skip (not found): {path}')
            continue
        s = open(path).read()
        if not (k_re.search(s) and d_re.search(s)):
            print(f'  skip (no camera_matrix / distortion_coeffs): {path}')
            continue
        shutil.copy(path, path + '.bak-calib')
        s = k_re.sub(k_text, s, count=1)
        s = d_re.sub(d_text, s, count=1)
        s = re.sub(r'^(\s*use_camera_info:\s*)\S+', r'\g<1>false', s, flags=re.M)
        open(path, 'w').write(s)
        cw = re.search(r'^\s*camera_width:\s*(\d+)', s, re.M)
        ch = re.search(r'^\s*camera_height:\s*(\d+)', s, re.M)
        note = ''
        if cw and ch and (int(cw.group(1)), int(ch.group(1))) != size:
            note = f'   !! camera_width/height there = {cw.group(1)}x{ch.group(1)}, calibration = {size[0]}x{size[1]}'
        print(f'  updated: {path}  (backup .bak-calib){note}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cols', type=int, required=True, help='inner corners along the long side, e.g. 9')
    ap.add_argument('--rows', type=int, required=True, help='inner corners along the short side, e.g. 6')
    ap.add_argument('--square-mm', type=float, required=True, help='size of one square in mm')
    ap.add_argument('--count', type=int, default=25, help='captures to collect (default 25)')
    ap.add_argument('--device', default='0')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--fourcc', default='MJPG')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--configs', nargs='*', default=DEFAULT_CONFIGS, help='yaml files to update')
    ap.add_argument('--no-write', action='store_true', help='only report, do not touch the configs')
    ap.add_argument('--from-dir', default='', help='calibrate from images saved by an earlier run')
    ap.add_argument('--stream', default='', help='host:port live preview (H.264 over UDP)')
    ap.add_argument('--show', action='store_true', help='local preview window (needs a display)')
    a = ap.parse_args()

    pattern = (a.cols, a.rows)
    objp = np.zeros((a.cols * a.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:a.cols, 0:a.rows].T.reshape(-1, 2) * (a.square_mm / 1000.0)
    obj_pts, img_pts, sigs = [], [], []

    if a.from_dir:
        files = sorted(glob.glob(os.path.join(a.from_dir, '*.png')))
        size = None
        for f in files:
            gray = cv2.cvtColor(cv2.imread(f), cv2.COLOR_BGR2GRAY)
            size = (gray.shape[1], gray.shape[0])
            c = find_board(gray, pattern)
            if c is not None:
                obj_pts.append(objp); img_pts.append(c); sigs.append(board_signature(c, *size))
        print(f'{len(img_pts)}/{len(files)} images usable from {a.from_dir}')
        save_dir = a.from_dir
    else:
        is_file = os.path.isfile(a.device)            # a recorded video works too
        if is_file:
            cap = cv2.VideoCapture(a.device)
        else:
            dev = a.device.replace('/dev/video', '')
            cap = cv2.VideoCapture(int(dev) if dev.isdigit() else a.device, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*a.fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
            cap.set(cv2.CAP_PROP_FPS, a.fps)
        ok, frame = cap.read()
        if not ok:
            raise SystemExit(f'cannot read from camera {a.device}')
        size = (frame.shape[1], frame.shape[0])
        if not is_file and size != (a.width, a.height):
            print(f'!! camera gives {size[0]}x{size[1]}, asked {a.width}x{a.height} — calibrating what it gives')
        save_dir = os.path.join(WS, 'calib', time.strftime('%Y-%m-%d_%H%M'))
        os.makedirs(save_dir, exist_ok=True)
        print(f'camera {a.device}: {size[0]}x{size[1]} {a.fourcc} | board {a.cols}x{a.rows} inner corners, '
              f'{a.square_mm} mm | saving to {save_dir}\nmove the board slowly; Ctrl+C to stop early')

        writer = None
        if a.stream:
            host, port = a.stream.split(':')
            pipe = ('appsrc is-live=true do-timestamp=true format=time ! '
                    'video/x-raw,format=BGR,width=848,height=480,framerate=15/1 ! videoconvert ! '
                    'x264enc tune=zerolatency bitrate=2000 speed-preset=superfast key-int-max=15 ! '
                    f'rtph264pay config-interval=1 pt=96 ! udpsink host={host} port={port} sync=false async=false')
            writer = cv2.VideoWriter(pipe, cv2.CAP_GSTREAMER, 0, 15.0, (848, 480), True)
            print('preview stream:', a.stream if writer.isOpened() else 'FAILED')

        last_capture, last_view, failed_reads = 0.0, 0.0, 0
        last_found, last_hint = None, -1e9      # in video time for files, wall time for cameras
        try:
            while len(img_pts) < a.count:
                ok, frame = cap.read()
                if not ok:
                    failed_reads += 1
                    if is_file or failed_reads > 30:    # end of the video / camera gone: stop, do not hang
                        print('end of video' if is_file else 'camera stopped delivering frames')
                        break
                    continue
                failed_reads = 0
                now = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0 if is_file else time.time()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                small = cv2.resize(gray, None, fx=0.5, fy=0.5)          # fast "is there a board" check
                found_small, _ = cv2.findChessboardCorners(small, pattern, FIND_FLAGS)
                corners, status = None, 'no board'
                if found_small:
                    corners = find_board(gray, pattern)
                    status = 'board' if corners is not None else 'no board'
                if last_found is None:
                    last_found = now
                if corners is not None:
                    last_found = now
                elif now - last_found > 5.0 and now - last_hint > 5.0:
                    # most common mistake: SQUARES counted instead of INNER corners -> try one less
                    last_hint = now
                    alt = (a.cols - 1, a.rows - 1)
                    if cv2.findChessboardCorners(small, alt, FIND_FLAGS)[0]:
                        print(f'!! no board with {a.cols}x{a.rows} inner corners, but FOUND with {alt[0]}x{alt[1]}: '
                              f'you counted squares. Restart with --cols {alt[0]} --rows {alt[1]}', flush=True)
                        status = f'counted squares? use --cols {alt[0]} --rows {alt[1]}'
                if corners is not None and now - last_capture > 1.0:
                    sig = board_signature(corners, *size)
                    sharp = cv2.Laplacian(gray, cv2.CV_64F).var()
                    new_spot = all(np.linalg.norm(sig - s) > 0.08 for s in sigs)
                    if not new_spot:
                        status = 'same spot - move/tilt the board'
                    elif sharp < 20:
                        status = 'blurry - hold still'
                    else:
                        obj_pts.append(objp); img_pts.append(corners); sigs.append(sig)
                        cv2.imwrite(os.path.join(save_dir, f'{len(img_pts):02d}.png'), frame)
                        last_capture = now
                        g = coverage(sigs)
                        print(f'captured {len(img_pts)}/{a.count}  coverage (3x3, top row first): '
                              f'{g.tolist()}  empty cells: {int((g == 0).sum())}', flush=True)
                        status = 'CAPTURED'
                if (writer is not None or a.show) and now - last_view > 1 / 15:
                    view = frame.copy()
                    if corners is not None:
                        cv2.drawChessboardCorners(view, pattern, corners, True)
                    for s in sigs:
                        cv2.circle(view, (int(s[0] * size[0]), int(s[1] * size[1])), 8, (0, 255, 0), -1)
                    cv2.putText(view, f'{len(img_pts)}/{a.count}  {status}', (15, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 255), 2)
                    view = cv2.resize(view, (848, 480))
                    if writer is not None and writer.isOpened():
                        writer.write(view)
                    if a.show:
                        cv2.imshow('calibration', view)
                        cv2.waitKey(1)
                    last_view = now
        except KeyboardInterrupt:
            print('\nstopped early')
        finally:
            cap.release()
            if writer is not None:
                writer.release()

    if len(img_pts) < 10:
        raise SystemExit(f'only {len(img_pts)} usable views — need at least 10 (25 recommended)')

    rms, K, D, errs = calibrate(obj_pts, img_pts, size)
    worst = np.argsort(errs)[::-1]
    print(f'\nRMS reprojection error: {rms:.3f} px  ({"good" if rms < 0.5 else "ok" if rms < 1.0 else "POOR — redo"})')
    print('worst views:', ', '.join(f'#{i + 1}: {errs[i]:.2f}px' for i in worst[:3]))
    keep = [i for i in range(len(errs)) if errs[i] <= 2.5 * np.median(errs)]
    if len(keep) < len(errs) and len(keep) >= 10:
        rms2, K2, D2, _ = calibrate([obj_pts[i] for i in keep], [img_pts[i] for i in keep], size)
        if rms2 < rms:
            print(f'dropped {len(errs) - len(keep)} outlier view(s) -> RMS {rms2:.3f} px')
            rms, K, D = rms2, K2, D2
    g = coverage(sigs)
    if (g == 0).any():
        print(f'!! image areas without any board (3x3 grid): {g.tolist()} — edges may be less accurate')

    print(f'\nimage size: {size[0]}x{size[1]}')
    print(f'camera_matrix: [{K[0,0]:.8f}, 0.0, {K[0,2]:.8f}, 0.0, {K[1,1]:.8f}, {K[1,2]:.8f}, 0.0, 0.0, 1.0]')
    print('distortion_coeffs: [' + ', '.join(f'{v:.8f}' for v in D) + ']')

    out = os.path.join(save_dir, f'camera_{size[0]}x{size[1]}.yaml')
    fs = cv2.FileStorage(out, cv2.FILE_STORAGE_WRITE)
    fs.write('image_width', size[0]); fs.write('image_height', size[1])
    fs.write('camera_matrix', K); fs.write('distortion_coeffs', D.reshape(1, -1)); fs.write('rms', rms)
    fs.release()
    print(f'saved: {out}')

    if a.no_write:
        return
    if rms >= 1.0:
        print('RMS >= 1.0 px — configs NOT updated. Redo with a flatter board, more angles and all corners.')
        return
    write_configs(a.configs, K, D, size)
    print('done. Rebuild vision_geo if you launch it from install/ (launch_vision_geo.sh uses the installed config).')


if __name__ == '__main__':
    main()
