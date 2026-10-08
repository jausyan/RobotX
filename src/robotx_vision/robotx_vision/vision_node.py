#!/usr/bin/env python3
"""RobotX UAV vision node - Python counterpart of the C++ `vision_video` package.

Run like vision_video:
  ros2 run robotx_vision robotx_vision --ros-args \
      --params-file src/robotx_vision/config/robotx_vision.yaml

Video in : input_mode "device" (V4L2 camera / video file) or "topic" (sensor_msgs/Image)
Inference: YOLO ONNX via OpenVINO (NPU/CPU/GPU) or OpenCV DNN
Video out: annotated frames over GStreamer (H.264/RTP/UDP) to stream_host:stream_port,
           plus an optional local window. The stream runs from launch, mission or not.

Data out, driven by String messages on the order topic (/mission/order):
  UAV-GO #1 -> MISSION 1: task1 model, map lights (incl. blinking), publish one
               DetectedObjectArray per class; after `mapping_time_s` publish
               "MISSION-DONE" on the order topic and go idle.
  UAV-GO #2 -> MISSION 2: task2 model, publish circle_* / tin_* / green_light
  UAV-GO #3 -> MISSION 3: same as 2
Nothing is published on the target topics while idle. Detections are still drawn on
the stream while idle (using the next mission's model) unless infer_when_idle=false.

Targets go to <target_topic_prefix><name>, e.g. /vision_geo/target/circle_red.
`altitude_m` = drone height above the target plane used for the projection;
north/east_offset_m = metres from the drone to the object.
"""
import math
import os
import re
import time
from collections import namedtuple

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
from vision_msgs.msg import DetectedObject, DetectedObjectArray

from .utils.detector import Detector, read_onnx_names
from .utils.geo import (body_to_ne, latlon_to_offset, norm_label,
                        normalized_to_body, offset_to_latlon, parse_order,
                        topic_suffix)
from .utils.light_mapper import BLINK, LightMapper

Pose = namedtuple('Pose', 'lat lon agl heading')
GREEN, RED, YELLOW = (0, 255, 0), (0, 0, 255), (0, 255, 255)


class RobotxVisionNode(Node):
    def __init__(self):
        super().__init__('robotx_vision_node')
        self._declare_params()
        g = self._get

        # ---- input / inference params ---------------------------------------
        self.input_mode = g('input_mode')
        self.input_source = g('input_source')
        self.frame_id = g('frame_id')
        self.in_w, self.in_h = int(g('input_width')), int(g('input_height'))
        self.infer_when_idle = bool(g('infer_when_idle'))

        # ---- camera intrinsics (like vision_video: params or CameraInfo topic) --
        self.K = np.array(g('camera_matrix'), np.float64).reshape(3, 3)
        self.D = np.array(g('distortion_coeffs'), np.float64).reshape(1, -1)
        self.calib_size = (float(g('calib_width')), float(g('calib_height')))
        self.use_camera_info = bool(g('use_camera_info'))
        self.camera_info_received = False
        if self.use_camera_info:
            self.create_subscription(CameraInfo, g('camera_info_topic'),
                                     self.camera_info_cb, 10)
            self.get_logger().info(f'Waiting for camera_info on: {g("camera_info_topic")}')

        # ---- geo / mission params ---------------------------------------------
        self.cam_yaw = float(g('camera_yaw_offset_deg'))
        self.target_height = float(g('target_height_m'))
        self.max_age = float(g('max_sensor_age_s'))
        self.mapping_time = float(g('mapping_time_s'))
        self.only_ordered = bool(g('only_publish_ordered'))
        self.active_timeout = float(g('active_timeout_s'))
        self.prefix = g('target_topic_prefix').rstrip('/') + '/'

        # ---- stream / window ---------------------------------------------------
        self.show_window = bool(g('show_window'))
        self.window_name = g('window_name')
        self.enable_stream = bool(g('enable_stream'))
        self.stream_host, self.stream_port = g('stream_host'), int(g('stream_port'))
        self.stream_w, self.stream_h = int(g('stream_width')), int(g('stream_height'))
        self.stream_fps, self.stream_kbps = int(g('stream_fps')), int(g('stream_bitrate_kbps'))
        self.writer, self.writer_init = None, False

        # ---- models ----------------------------------------------------------
        self.det1, self.names1 = self._make_detector('task1')
        self.det2, self.names2 = self._make_detector('task2')
        self.blink_id = self.names1.index(BLINK) if BLINK in self.names1 else -1
        self.mapper = LightMapper(
            self.get_logger(),
            dist_tol_m=float(g('dist_tol_m')), dedup_tol_m=float(g('dedup_tol_m')),
            blink_min_s=float(g('blink_min_s')), blink_max_s=float(g('blink_max_s')))

        # ---- state -------------------------------------------------------------
        self.next_mission = int(g('start_mission'))   # started by the next UAV-GO
        self.active = 0                                # 0 = idle
        self.mission_t0 = 0.0
        self.order = (None, None)                      # (tin, circle)
        self.last_meta = None
        self.lat = self.lon = self.rel_alt = self.hdg = None
        self.t_gps = self.t_alt = self.t_hdg = 0.0
        self.cap, self.using_camera = None, False
        self.fps, self._frames, self._fps_t0 = 0.0, 0, time.monotonic()

        # ---- publishers (created up-front so DDS discovery is done in time) -------
        self.pubs = {}
        labels = {topic_suffix(n) for n in self.names1 + self.names2} | {BLINK}
        for s in sorted(labels):
            self._get_pub(s)
        self.order_pub = self.create_publisher(String, g('order_topic'), 10)

        # ---- drone state subscriptions -----------------------------------------
        self.create_subscription(NavSatFix, g('gps_topic'), self.gps_cb, qos_profile_sensor_data)
        if g('use_odometry'):       # simulation: altitude + heading from odometry
            self.create_subscription(Odometry, g('odom_topic'), self.odom_cb, qos_profile_sensor_data)
        else:                       # real drone: MAVROS
            self.create_subscription(Float64, g('rel_alt_topic'), self.alt_cb, qos_profile_sensor_data)
            self.create_subscription(Float64, g('heading_topic'), self.hdg_cb, qos_profile_sensor_data)
        self.create_subscription(String, g('order_topic'), self.order_cb, 10)
        self.create_timer(0.1, self._tick)

        # ---- video input -----------------------------------------------------------
        if self.input_mode == 'topic':
            self.create_subscription(Image, self.input_source, self.image_cb,
                                     qos_profile_sensor_data)
            self.get_logger().info(f'Subscribing to image topic: {self.input_source}')
        else:
            self._open_source()
            self.create_timer(0.001, self._device_tick,
                              clock=Clock(clock_type=ClockType.STEADY_TIME))

        if self.show_window:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            except cv2.error as exc:
                self.show_window = False
                self.get_logger().error(f'Cannot create window, display disabled: {exc}')

        self.get_logger().info(
            f'Ready. Waiting for UAV-GO on {g("order_topic")} (next mission: '
            f'{self.next_mission}). Targets -> {self.prefix}<name>')

    # ======================================================================
    # Setup helpers
    # ======================================================================
    def _declare_params(self):
        d = self.declare_parameter
        # input
        d('input_mode', 'device')                 # "device" | "topic"
        d('input_source', '/dev/video0')          # device path, video file or Image topic
        d('frame_id', 'camera')
        # models (one ONNX per phase; class lists optional if names are embedded)
        d('task1_model_path', '')
        d('task2_model_path', '')
        d('task1_classes', [''])                  # [''] = read from ONNX metadata
        d('task2_classes', [''])
        d('input_width', 320)
        d('input_height', 320)
        d('conf_threshold', 0.4)
        d('nms_threshold', 0.45)
        d('letterbox', True)
        d('use_openvino', False)
        d('openvino_device', 'CPU')
        d('infer_when_idle', True)
        # camera
        d('use_camera_info', False)
        d('camera_info_topic', '/camera/camera_info')
        d('camera_matrix', [504.99132615, 0.0, 318.4012728,
                            0.0, 506.13168687, 243.99035432,
                            0.0, 0.0, 1.0])
        d('distortion_coeffs', [0.07771398, -0.18711237, -0.00182709, 0.00026326, -0.06212367])
        d('calib_width', 640.0)                   # resolution camera_matrix belongs to
        d('calib_height', 480.0)
        d('camera_yaw_offset_deg', 0.0)           # camera "up" vs drone nose, CW positive
        # drone state
        d('gps_topic', '/mavros/global_position/global')            # NavSatFix
        d('rel_alt_topic', '/mavros/global_position/rel_alt')       # Float64 [m]
        d('heading_topic', '/mavros/global_position/compass_hdg')   # Float64 [deg CW from N]
        d('use_odometry', False)
        d('odom_topic', '/ap/v1/odometry')
        d('target_height_m', 0.0)
        d('max_sensor_age_s', 1.0)
        # mission I/O
        d('order_topic', '/mission/order')                          # communication.order_topic
        d('target_topic_prefix', '/vision_geo/target/')             # vision.target_topic_prefix
        d('start_mission', 1)                                       # mission the next UAV-GO starts
        d('mapping_time_s', 60.0)                                   # keep == mission.survey_time
        d('dist_tol_m', 3.0)
        d('dedup_tol_m', 1.5)
        d('blink_min_s', 0.3)
        d('blink_max_s', 3.0)
        d('only_publish_ordered', False)
        d('active_timeout_s', 0.0)                                  # 0 = until next UAV-GO
        # display / stream
        d('show_window', False)
        d('window_name', 'robotx_vision')
        d('enable_stream', False)
        d('stream_host', '127.0.0.1')
        d('stream_port', 5000)
        d('stream_width', 640)
        d('stream_height', 480)
        d('stream_fps', 30)
        d('stream_bitrate_kbps', 500)

    def _get(self, name):
        return self.get_parameter(name).value

    def _make_detector(self, tag):
        path = self._get(f'{tag}_model_path')
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(f'{tag}_model_path not found: "{path}"')
        names = [c for c in self._get(f'{tag}_classes') if c] or read_onnx_names(path)
        if not names:
            raise ValueError(f'{tag}_classes is empty and {path} has no embedded class names')
        det = Detector(path, self.in_w, self.in_h, self._get('conf_threshold'),
                       self._get('nms_threshold'), bool(self._get('use_openvino')),
                       self._get('openvino_device'), bool(self._get('letterbox')))
        if det.num_classes != len(names):
            raise ValueError(f'{tag}: model has {det.num_classes} classes but '
                             f'{len(names)} names were given/found')
        self.get_logger().info(f'{tag}: {path} [{det.backend}] classes={names}')
        return det, [norm_label(n) for n in names]

    def _open_source(self):
        src = self.input_source
        cap = None
        if src.startswith('/dev/video'):
            try:
                cap = cv2.VideoCapture(int(src[len('/dev/video'):]), cv2.CAP_V4L2)
                self.using_camera = cap.isOpened()
            except ValueError:
                cap = None
        if cap is None or not cap.isOpened():
            cap = cv2.VideoCapture(src)
            self.using_camera = src.startswith('/dev/video')
        if not cap.isOpened():
            raise RuntimeError(f'failed to open input source: {src}')
        self.cap = cap
        self.get_logger().info(
            f'Using {"camera device" if self.using_camera else "video file"} input: {src}')

    def _get_pub(self, suffix):
        pub = self.pubs.get(suffix)
        if pub is None:
            pub = self.create_publisher(DetectedObjectArray, self.prefix + suffix, 10)
            self.pubs[suffix] = pub
        return pub

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        if self.cap is not None:
            self.cap.release()
        if self.show_window:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass
        super().destroy_node()

    # ======================================================================
    # Drone-state callbacks
    # ======================================================================
    def camera_info_cb(self, msg):
        if self.camera_info_received:
            return
        self.K = np.array(msg.k, np.float64).reshape(3, 3)
        self.D = np.array(list(msg.d) or [0.0] * 5, np.float64).reshape(1, -1)
        self.calib_size = (float(msg.width), float(msg.height))
        self.camera_info_received = True
        self.get_logger().info(f'Camera intrinsics from topic: fx={msg.k[0]:.2f} '
                               f'fy={msg.k[4]:.2f} cx={msg.k[2]:.2f} cy={msg.k[5]:.2f}')

    def gps_cb(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.lat, self.lon, self.t_gps = msg.latitude, msg.longitude, self._now()

    def alt_cb(self, msg):
        if math.isfinite(msg.data):
            self.rel_alt, self.t_alt = msg.data, self._now()

    def hdg_cb(self, msg):
        if math.isfinite(msg.data):
            self.hdg, self.t_hdg = msg.data, self._now()

    def odom_cb(self, msg):      # same math as the simulation scripts
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.hdg = math.degrees(yaw)
        self.rel_alt = msg.pose.pose.position.z
        self.t_alt = self.t_hdg = self._now()

    def _pose(self):
        if None in (self.lat, self.rel_alt, self.hdg):
            return None
        now = self._now()
        if max(now - self.t_gps, now - self.t_alt, now - self.t_hdg) > self.max_age:
            return None
        return Pose(self.lat, self.lon, max(0.1, self.rel_alt - self.target_height), self.hdg)

    # ======================================================================
    # Mission sequencing (/mission/order)
    # ======================================================================
    def order_cb(self, msg):
        text = msg.data.strip()
        up = text.upper()
        if 'MISSION-DONE' in up or 'UAV-GO' not in up:
            return                       # also ignores our own MISSION-DONE echo
        n = self.next_mission
        self.active, self.mission_t0 = n, self._now()
        self.next_mission = min(n + 1, 3)
        if n == 1:
            self.mapper.reset()
            self.get_logger().info(f'MISSION 1 START: mapping for {self.mapping_time:.0f}s')
        else:
            self.order = parse_order(text)
            self.get_logger().info(f'MISSION {n} START: order="{text}" -> '
                                   f'tin={self.order[0]}, circle={self.order[1]}')

    def _tick(self):
        if self.active == 0:
            return
        elapsed = self._now() - self.mission_t0
        if self.active == 1 and elapsed >= self.mapping_time:
            self._finish_mission1()
        elif (self.active in (2, 3) and self.active_timeout > 0
              and elapsed >= self.active_timeout):
            self.get_logger().info(f'MISSION {self.active} timeout -> idle')
            self.active = 0

    def _finish_mission1(self):
        self._publish_map()
        counts = dict(self.mapper.counts())
        if self.mapper.arena_complete():
            self.get_logger().info(f'MISSION 1 DONE. Map: {counts}')
        else:
            self.get_logger().warning(f'MISSION 1 DONE but map looks incomplete: {counts}')
        self.active = 0
        self.order_pub.publish(String(data='MISSION-DONE'))

    # ======================================================================
    # Frame sources
    # ======================================================================
    def _device_tick(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            if self.using_camera:
                self.get_logger().error(f'Failed to read frame from {self.input_source}',
                                        throttle_duration_sec=1.0)
                return
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)      # loop video files
            ok, frame = self.cap.read()
            if not ok:
                return
        self.process_frame(frame, self.get_clock().now().to_msg(), self.frame_id)

    def image_cb(self, msg):
        if self.use_camera_info and not self.camera_info_received:
            self.get_logger().warning('Waiting for camera_info...', throttle_duration_sec=2.0)
            return
        ch = {'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(msg.encoding)
        if ch is None:
            self.get_logger().error(f'Unsupported image encoding: {msg.encoding}',
                                    throttle_duration_sec=2.0)
            return
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        arr = arr[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'mono8':
            frame = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        else:
            frame = arr.copy()
        stamp = msg.header.stamp
        if not (stamp.sec or stamp.nanosec):
            stamp = self.get_clock().now().to_msg()
        self.process_frame(frame, stamp, msg.header.frame_id or self.frame_id)

    # ======================================================================
    # Core loop
    # ======================================================================
    def process_frame(self, frame, stamp, frame_id):
        try:
            self._update_fps()
            active = self.active
            if not active and not self.infer_when_idle:
                self._render(frame, [])
                return
            det, names = ((self.det1, self.names1) if (active or self.next_mission) == 1
                          else (self.det2, self.names2))
            dets = [self._make_det(r, names) for r in det.infer(frame)]
            overlay = [(topic_suffix(d['label']), d) for d in dets]

            if active:
                pose = self._pose()
                if pose is None:
                    self.get_logger().warning('Waiting for fresh GPS / altitude / heading...',
                                              throttle_duration_sec=2.0)
                else:
                    self._add_geo(dets, pose, frame.shape)
                    meta = (stamp, frame_id, pose)
                    self.last_meta = meta
                    overlay = (self._mission1(dets, meta) if active == 1
                               else self._mission23(dets, meta))
            self._render(frame, overlay)
            self.get_logger().info(f'FPS={self.fps:.1f} | {self._state_text()} | {len(dets)} obj',
                                   throttle_duration_sec=1.0)
        except Exception as exc:     # keep the node alive in flight
            self.get_logger().error(f'Frame processing failed: {exc!r}',
                                    throttle_duration_sec=2.0)

    def _update_fps(self):
        self._frames += 1
        t = time.monotonic()
        if t - self._fps_t0 >= 1.0:
            self.fps = self._frames / (t - self._fps_t0)
            self._frames, self._fps_t0 = 0, t

    def _state_text(self):
        return f'MISSION {self.active}' if self.active else f'IDLE (next M{self.next_mission})'

    @staticmethod
    def _make_det(raw, names):
        cid, conf, u, v, w, h = raw
        label = names[cid] if cid < len(names) else f'class_{cid}'
        return {'label': label, 'class_id': cid, 'conf': conf,
                'u': u, 'v': v, 'w': w, 'h': h}

    def _intrinsics(self, w, h):
        K = self.K.copy()
        cw, ch = self.calib_size
        if cw > 0 and ch > 0 and (w != cw or h != ch):
            K[0, :] *= w / cw
            K[1, :] *= h / ch
        return K

    def _add_geo(self, dets, pose, shape):
        if not dets:
            return
        K = self._intrinsics(shape[1], shape[0])
        pts = np.array([[d['u'], d['v']] for d in dets], np.float64).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, K, self.D).reshape(-1, 2)
        heading = pose.heading + self.cam_yaw
        for d, (xn, yn) in zip(dets, norm):
            fwd, right = normalized_to_body(float(xn), float(yn), pose.agl)
            d['north'], d['east'] = body_to_ne(fwd, right, heading)
            d['lat'], d['lon'] = offset_to_latlon(pose.lat, pose.lon, d['north'], d['east'])

    # ---- message builders -----------------------------------------------------------
    @staticmethod
    def _obj(stamp, frame_id, label, class_id, conf, u, v, lat, lon, alt, north, east):
        o = DetectedObject()
        o.header.stamp, o.header.frame_id = stamp, frame_id
        o.class_id, o.label, o.confidence = int(class_id), label, float(conf)
        o.pixel_u, o.pixel_v = float(u), float(v)
        o.latitude, o.longitude, o.altitude_m = float(lat), float(lon), float(alt)
        o.north_offset_m, o.east_offset_m = float(north), float(east)
        return o

    @staticmethod
    def _array(stamp, frame_id):
        arr = DetectedObjectArray()
        arr.header.stamp, arr.header.frame_id = stamp, frame_id
        return arr

    # ---- mission 1: map lights ---------------------------------------------------------
    def _mission1(self, dets, meta):
        pairs = self.mapper.update(dets, self._now())
        self._publish_map(meta)
        final = {id(d): t['final'] for d, t in pairs}
        return [(final.get(id(d), d['label']), d) for d in dets]

    def _publish_map(self, meta=None):
        meta = meta or self.last_meta
        if meta is None:
            return
        stamp, frame_id, pose = meta
        for label, targets in self.mapper.snapshot().items():
            arr = self._array(stamp, frame_id)
            for t in targets:
                north, east = latlon_to_offset(pose.lat, pose.lon, t['lat'], t['lon'])
                cid = self.blink_id if label == BLINK else t['class_id']
                arr.objects.append(self._obj(stamp, frame_id, label, cid, t['conf'],
                                             t['u'], t['v'], t['lat'], t['lon'],
                                             pose.agl, north, east))
            self._get_pub(label).publish(arr)

    # ---- missions 2 & 3: circles / tins ------------------------------------------------
    def _mission23(self, dets, meta):
        stamp, frame_id, pose = meta
        groups = {}
        for d in dets:
            groups.setdefault(topic_suffix(d['label']), []).append(d)
        tin, circle = self.order
        allowed = ({f'circle_{circle}', f'tin_{tin}'}
                   if self.only_ordered and tin and circle else None)
        overlay = []
        for suffix, items in groups.items():
            if allowed is not None and suffix not in allowed:
                continue
            items.sort(key=lambda d: d['w'] * d['h'], reverse=True)    # largest first
            arr = self._array(stamp, frame_id)
            for d in items:
                arr.objects.append(self._obj(
                    stamp, frame_id, suffix, d['class_id'], d['conf'], d['u'], d['v'],
                    d['lat'], d['lon'], pose.agl, d['north'], d['east']))
                overlay.append((suffix, d))
            self._get_pub(suffix).publish(arr)
        return overlay

    # ======================================================================
    # Display & GStreamer stream (same pipeline as vision_video)
    # ======================================================================
    def _ensure_writer(self):
        if self.writer_init:
            return self.writer
        self.writer_init = True
        pipeline = (
            'appsrc is-live=true do-timestamp=true format=time ! '
            f'video/x-raw,format=BGR,width={self.stream_w},height={self.stream_h},'
            f'framerate={self.stream_fps}/1 ! videoconvert ! '
            f'x264enc tune=zerolatency bitrate={self.stream_kbps} speed-preset=superfast '
            f'key-int-max={self.stream_fps} byte-stream=true ! '
            'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={self.stream_host} port={self.stream_port} sync=false async=false')
        writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(self.stream_fps),
                                 (self.stream_w, self.stream_h), True)
        if not writer.isOpened():
            hint = ''
            if not re.search(r'GStreamer:\s+YES', cv2.getBuildInformation()):
                hint = (' This OpenCV build has no GStreamer support (pip wheels never do);'
                        ' use the apt python3-opencv.')
            self.get_logger().error(f'Failed to open GStreamer pipeline, stream disabled.{hint} '
                                    f'Pipeline: {pipeline}')
            self.enable_stream = False
            return None
        self.writer = writer
        self.get_logger().info(f'Streaming to udp://{self.stream_host}:{self.stream_port} '
                               f'({self.stream_w}x{self.stream_h} @ {self.stream_fps} fps)')
        return writer

    def _render(self, frame, overlay):
        writer = self._ensure_writer() if self.enable_stream else None
        if writer is None and not self.show_window:
            return
        img = frame.copy()
        h, w = img.shape[:2]
        cv2.line(img, (0, h // 2), (w, h // 2), GREEN, 1)          # crosshair
        cv2.line(img, (w // 2, 0), (w // 2, h), GREEN, 1)
        for label, d in overlay:
            x1, y1 = int(d['u'] - d['w'] / 2), int(d['v'] - d['h'] / 2)
            x2, y2 = int(d['u'] + d['w'] / 2), int(d['v'] + d['h'] / 2)
            hot = label == BLINK
            color, thick = (RED, 2) if hot else (GREEN, 1)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, thick)
            text = f'{label.upper() if hot else label} {d["conf"]:.2f}'
            cv2.putText(img, text, (x1, max(12, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, thick)
        cv2.putText(img, f'{self._state_text()}  {self.fps:.0f} fps', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, YELLOW, 2)

        if writer is not None:      # boxes are drawn before resizing, so they stay aligned
            out = img if (w, h) == (self.stream_w, self.stream_h) else \
                cv2.resize(img, (self.stream_w, self.stream_h))
            writer.write(out)
        if self.show_window:
            try:
                cv2.imshow(self.window_name, img)
                if cv2.waitKey(1) in (ord('q'), 27):
                    self.get_logger().info('Window exit key pressed, shutting down')
                    rclpy.shutdown()
            except cv2.error as exc:
                self.show_window = False
                self.get_logger().error(f'Display failed, disabled: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotxVisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
