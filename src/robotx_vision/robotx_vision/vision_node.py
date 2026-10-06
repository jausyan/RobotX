#!/usr/bin/env python3
"""RobotX UAV vision node (Python twin of vision_geo — same topics, same behaviour).

Driven by String messages on the order topic (default /mission/order).
The mission is chosen from the message CONTENT, so repeated orders are harmless:

  "UAV-GO"               -> MISSION 1 (only from idle): task1 model, buoy map.
                            Keeps mapping until a delivery order arrives
                            (MISSION-DONE comes from the GCS, not from vision).
  "UAV-GO:<TIN>:<CIRCLE>" -> MISSION 2/3: task2 model, centering targets.
                            The same order again is ignored; a different one
                            starts the next delivery.
                            task23.delivery_trigger = 'ack' (default): start on
                            control's "ACK:UAV-GO:<TIN>:<CIRCLE>", i.e. when control
                            really starts the delivery (uav_bridge may queue an
                            order while Task 1 is still running). 'order': start on
                            the order itself (bench tests without control).

While idle nothing is inferred and nothing is published.

Output topics (identical to vision_geo):
  /vision_geo/target/<class>        geometry_msgs/PoseStamped, every frame per class of
                                    the active model: closest detection, camera frame
                                    x = right, y = down, z = height (m); zeros = not seen.
                                    centering_red() reads this.
  /vision_geo/map/buoy_<state>      vision_msgs/DetectedObjectArray, MISSION 1, every
                                    frame: red / green / entry / exit / off.
  /vision_geo/detections            vision_msgs/DetectedObjectArray, raw detections
                                    of the frame with lat/lon.

Optional GStreamer stream (same as vision_hailo): annotated frames as RTP/H.264
over UDP to stream.stream_host:stream.stream_port.
"""
import math
import os
from collections import namedtuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
from ultralytics import YOLO
from vision_msgs.msg import DetectedObject, DetectedObjectArray

from .geo import (body_to_ne, latlon_to_offset, norm_label, offset_to_latlon,
                  parse_order, pixel_to_body, topic_suffix)
from .light_mapper import STATES, LightMapper

Pose = namedtuple('Pose', 'lat lon agl heading')


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self._declare_params()
        g = self._get

        # ---- parameters -> attributes -------------------------------------
        self.conf = float(g('model.conf'))
        self.imgsz = int(g('model.imgsz'))
        self.device = g('model.device')
        self.calib = (float(g('camera.fx')), float(g('camera.fy')),
                      float(g('camera.cx')), float(g('camera.cy')),
                      float(g('camera.calib_width')), float(g('camera.calib_height')))
        self.cam_yaw = float(g('camera.yaw_offset_deg'))
        self.target_height = float(g('geo.target_height_m'))
        self.max_age = float(g('geo.max_sensor_age_s'))
        self.mapping_time = float(g('task1.mapping_time_s'))
        self.publish_done = bool(g('task1.publish_mission_done'))
        self.only_ordered = bool(g('task23.only_publish_ordered'))
        self.active_timeout = float(g('task23.active_timeout_s'))
        self.delivery_trigger = str(g('task23.delivery_trigger')).lower()
        self.publish_debug = bool(g('topics.publish_debug'))
        self.prefix = g('topics.target_prefix').rstrip('/') + '/'
        self.map_prefix = g('topics.map_prefix').rstrip('/') + '/'
        self.frame_id = g('topics.pose_frame_id')
        self.stream_cfg = {k: g('stream.' + k) for k in (
            'enable_stream', 'stream_host', 'stream_port', 'stream_width',
            'stream_height', 'stream_fps', 'stream_bitrate_kbps')}
        self.stream_writer = None
        self.stream_ready = False      # pipeline tried once (like vision_hailo)

        # ---- models ---------------------------------------------------------
        self.bridge = CvBridge()
        self.model1 = self._load(g('model.task1_weights'))
        self.model2 = self._load(g('model.task2_weights'))

        self.mapper = LightMapper(
            self.get_logger(),
            dist_tol_m=float(g('task1.dist_tol_m')),
            dedup_tol_m=float(g('task1.dedup_tol_m')),
            state_window_s=float(g('task1.state_window_s')),
            flash_gap_min_s=float(g('task1.flash_gap_min_s')),
            flash_gap_max_s=float(g('task1.flash_gap_max_s')),
            solid_min_s=float(g('task1.solid_min_s')))

        # ---- mission state ----------------------------------------------------
        self.active = 0                 # 0 = idle, 1 = mapping, 2/3 = delivery
        self.mission_t0 = 0.0
        self.order = (None, None)       # (tin, circle)
        self.last_meta = None

        # ---- sensors ----------------------------------------------------------
        self.lat = self.lon = self.rel_alt = self.hdg = None
        self.home = None                # first GPS fix: origin of the map offsets
        self.t_gps = self.t_alt = self.t_hdg = 0.0

        # ---- publishers / subscribers ----------------------------------------
        # created up-front so DDS discovery is done before the first frame
        self.classes = {1: sorted({topic_suffix(n) for n in self.model1.names.values()}),
                        2: sorted({topic_suffix(n) for n in self.model2.names.values()})}
        self.pose_pubs = {s: self.create_publisher(PoseStamped, self.prefix + s, 10)
                          for s in set(self.classes[1]) | set(self.classes[2])}
        self.map_pubs = {s: self.create_publisher(DetectedObjectArray,
                                                  self.map_prefix + 'buoy_' + s, 10)
                         for s in STATES}
        self.det_pub = self.create_publisher(DetectedObjectArray, g('topics.detections'), 10)
        self.order_pub = self.create_publisher(String, g('topics.order'), 10)
        self.debug_pub = self.create_publisher(Image, g('topics.debug_image'), 10)

        cam_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, g('topics.camera'), self.image_cb, cam_qos)
        self.create_subscription(NavSatFix, g('topics.gps'), self.gps_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, g('topics.rel_alt'), self.alt_cb, qos_profile_sensor_data)
        self.create_subscription(Float64, g('topics.heading'), self.hdg_cb, qos_profile_sensor_data)
        self.create_subscription(String, g('topics.order'), self.order_cb, 10)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'Vision node ready. Waiting for UAV-GO on {g("topics.order")}. '
            f'Targets -> {self.prefix}<class> (PoseStamped), map -> {self.map_prefix}buoy_<state>')

    # ======================================================================
    # Parameters / setup helpers
    # ======================================================================
    def _declare_params(self):
        try:
            share = get_package_share_directory('robotx_vision')
        except Exception:
            share = ''
        d = self.declare_parameter
        d('topics.camera', '/camera/image_raw')
        d('topics.gps', '/mavros/global_position/global')        # NavSatFix
        d('topics.rel_alt', '/mavros/global_position/rel_alt')   # Float64, m above home
        d('topics.heading', '/mavros/global_position/compass_hdg')  # Float64, deg CW from N
        d('topics.order', '/mission/order')                      # String
        d('topics.target_prefix', '/vision_geo/target/')         # PoseStamped per class
        d('topics.map_prefix', '/vision_geo/map/')               # buoy_<state> arrays
        d('topics.detections', '/vision_geo/detections')
        d('topics.pose_frame_id', 'camera')
        d('topics.debug_image', '/vision_geo/debug_image')
        d('topics.publish_debug', True)
        # GStreamer stream (same params as vision_hailo)
        d('stream.enable_stream', False)
        d('stream.stream_host', '192.168.0.127')
        d('stream.stream_port', 5000)
        d('stream.stream_width', 640)
        d('stream.stream_height', 480)
        d('stream.stream_fps', 30)
        d('stream.stream_bitrate_kbps', 500)
        d('model.task1_weights', os.path.join(share, 'weights', 'task1.pt'))
        d('model.task2_weights', os.path.join(share, 'weights', 'task2.pt'))
        d('model.conf', 0.25)
        d('model.imgsz', 640)
        d('model.device', '')                                    # '' = auto, e.g. 'cuda:0'
        d('camera.fx', 205.47)       # REPLACE with calibration values
        d('camera.fy', 205.47)
        d('camera.cx', 320.0)        # <= 0 -> use image centre
        d('camera.cy', 240.0)
        d('camera.calib_width', 640.0)
        d('camera.calib_height', 480.0)
        d('camera.yaw_offset_deg', 0.0)   # camera "up" vs drone nose, CW positive
        d('geo.target_height_m', 0.0)     # height of the target plane above home
        d('geo.max_sensor_age_s', 1.0)
        d('task1.publish_mission_done', False)   # True = old behaviour (self MISSION-DONE)
        d('task1.mapping_time_s', 60.0)          # only used with publish_mission_done
        d('task1.dist_tol_m', 2.0)
        d('task1.dedup_tol_m', 1.5)
        d('task1.state_window_s', 6.0)
        d('task1.flash_gap_min_s', 0.6)
        d('task1.flash_gap_max_s', 1.6)
        d('task1.solid_min_s', 3.0)
        d('task23.only_publish_ordered', False)
        d('task23.active_timeout_s', 0.0)  # 0 = stay active until the next order
        d('task23.delivery_trigger', 'ack')  # 'ack' = control's ACK:UAV-GO:..., 'order' = the order

    def _get(self, name):
        return self.get_parameter(name).value

    def _infer(self, model, frame):
        return model(frame, conf=self.conf, imgsz=self.imgsz,
                     device=self.device or None, verbose=False)[0]

    def _load(self, path):
        if not path or not os.path.isfile(path):
            raise FileNotFoundError(
                f'YOLO weights not found: "{path}" '
                '(set model.task1_weights / model.task2_weights)')
        model = YOLO(path)
        self._infer(model, np.zeros((480, 640, 3), np.uint8))   # warm-up
        self.get_logger().info(f'Loaded {path}: {list(model.names.values())}')
        return model

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ======================================================================
    # Sensor callbacks
    # ======================================================================
    def gps_cb(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        self.lat, self.lon, self.t_gps = msg.latitude, msg.longitude, self._now()
        if self.home is None:
            self.home = (self.lat, self.lon)

    def alt_cb(self, msg):
        if math.isfinite(msg.data):
            self.rel_alt, self.t_alt = msg.data, self._now()

    def hdg_cb(self, msg):
        if math.isfinite(msg.data):
            self.hdg, self.t_hdg = msg.data, self._now()

    def _pose(self):
        if None in (self.lat, self.rel_alt, self.hdg):
            return None
        now = self._now()
        if max(now - self.t_gps, now - self.t_alt, now - self.t_hdg) > self.max_age:
            return None
        agl = max(0.1, self.rel_alt - self.target_height)
        return Pose(self.lat, self.lon, agl, self.hdg)

    # ======================================================================
    # Mission sequencing (/mission/order) — decided by content, not by count
    # ======================================================================
    def order_cb(self, msg):
        text = msg.data.strip()
        acked = text.upper().startswith('ACK:')
        if acked:
            text = text[4:].strip()
        if not text.upper().startswith('UAV-GO'):
            return                              # MISSION-DONE, RUN-START, UAV-HOLD, ...
        tin, circle = parse_order(text)
        if tin and circle and acked != (self.delivery_trigger == 'ack'):
            return                              # delivery starts on the configured trigger only
        if acked and not (tin and circle):
            return

        if not (tin and circle):                # plain "UAV-GO" -> mapping
            if self.active != 0:
                return                          # repeat / late GO: keep current mission
            self.mapper.reset()
            self._start(1)
            self.get_logger().info('MISSION 1 START: buoy mapping')
            return

        if self.active in (2, 3) and self.order == (tin, circle):
            return                              # same order again
        n = 2 if self.active in (0, 1) else 3
        self.order = (tin, circle)
        self._start(n)
        self.get_logger().info(f'MISSION {n} START: order="{text}" -> tin={tin}, circle={circle}')

    def _start(self, n):
        if self.active != 0:
            self._publish_lost(self.classes[1 if self.active == 1 else 2])
        self.active = n
        self.mission_t0 = self._now()

    def _stop(self):
        self._publish_lost(self.classes[1 if self.active == 1 else 2])
        self.active = 0

    def _tick(self):
        if self.active == 0:
            return
        elapsed = self._now() - self.mission_t0
        if self.active == 1 and self.publish_done and elapsed >= self.mapping_time:
            self._finish_mission1()
        elif (self.active in (2, 3) and self.active_timeout > 0
              and elapsed >= self.active_timeout):
            self.get_logger().info(f'MISSION {self.active} timeout -> idle')
            self._stop()

    def _finish_mission1(self):
        self._publish_map()
        counts = dict(self.mapper.counts())
        if self.mapper.arena_complete():
            self.get_logger().info(f'MISSION 1 DONE. Map: {counts}')
        else:
            self.get_logger().warning(f'MISSION 1 DONE but map looks incomplete: {counts}')
        self._stop()
        self.order_pub.publish(String(data='MISSION-DONE'))

    # ======================================================================
    # Image pipeline
    # ======================================================================
    def image_cb(self, msg):
        if self.active == 0:
            return
        pose = self._pose()
        if pose is None:
            self.get_logger().warning('Waiting for fresh GPS / rel_alt / heading...',
                                      throttle_duration_sec=2.0)
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            model = self.model1 if self.active == 1 else self.model2
            dets = self._detections(self._infer(model, frame), model.names,
                                    frame.shape, pose)
            stamp = msg.header.stamp
            if not (stamp.sec or stamp.nanosec):
                stamp = self.get_clock().now().to_msg()
            meta = (stamp, msg.header.frame_id, pose)
            self.last_meta = meta

            self._publish_detections(dets, meta)
            self._publish_targets(dets, meta)
            if self.active == 1:
                pairs = self.mapper.update(dets, self._now())
                self._publish_map(meta)
                state = {id(d): 'buoy_' + t['state'] for d, t in pairs}
                overlay = [(state.get(id(d), d['label']), d) for d in dets]
            else:
                overlay = [(d['label'], d) for d in dets]
            self._debug(frame, overlay)
        except Exception as exc:   # keep the node alive in flight
            self.get_logger().error(f'Frame processing failed: {exc!r}',
                                    throttle_duration_sec=2.0)

    def _intrinsics(self, w, h):
        fx, fy, cx, cy, cw, ch = self.calib
        sx, sy = w / cw, h / ch
        return (fx * sx, fy * sy,
                cx * sx if cx > 0 else w / 2.0,
                cy * sy if cy > 0 else h / 2.0)

    def _detections(self, res, names, shape, pose):
        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            return []
        fx, fy, cx, cy = self._intrinsics(shape[1], shape[0])
        heading = pose.heading + self.cam_yaw
        xywh = boxes.xywh.cpu().numpy()
        cls = boxes.cls.cpu().numpy().astype(int)
        conf = boxes.conf.cpu().numpy()
        dets = []
        for (u, v, w, h), c, s in zip(xywh, cls, conf):
            fwd, right = pixel_to_body(u, v, fx, fy, cx, cy, pose.agl)
            north, east = body_to_ne(fwd, right, heading)
            lat, lon = offset_to_latlon(pose.lat, pose.lon, north, east)
            dets.append({'label': topic_suffix(norm_label(names[int(c)])), 'class_id': int(c),
                         'conf': float(s), 'u': float(u), 'v': float(v),
                         'w': float(w), 'h': float(h),
                         'fwd': float(fwd), 'right': float(right),
                         'north': float(north), 'east': float(east),
                         'lat': lat, 'lon': lon})
        return dets

    # ---- message builders ---------------------------------------------------
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

    def _pose_msg(self, stamp, x=0.0, y=0.0, z=0.0):
        p = PoseStamped()
        p.header.stamp, p.header.frame_id = stamp, self.frame_id
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
        p.pose.orientation.w = 1.0
        return p

    # ---- /vision_geo/detections: raw detections of this frame ----------------
    def _publish_detections(self, dets, meta):
        stamp, _, pose = meta
        arr = self._array(stamp, 'map')
        for d in dets:
            arr.objects.append(self._obj(stamp, 'map', d['label'], d['class_id'], d['conf'],
                                         d['u'], d['v'], d['lat'], d['lon'], pose.agl,
                                         d['north'], d['east']))
        self.det_pub.publish(arr)

    # ---- /vision_geo/target/<class>: closest detection, zeros when not seen ---
    def _publish_targets(self, dets, meta):
        stamp, _, pose = meta
        tin, circle = self.order
        allowed = ({f'circle_{circle}', f'tin_{tin}'}
                   if self.active in (2, 3) and self.only_ordered and tin and circle else None)
        for suffix in self.classes[1 if self.active == 1 else 2]:
            best = None
            if allowed is None or suffix in allowed:
                hits = [d for d in dets if d['label'] == suffix]
                if hits:
                    best = min(hits, key=lambda d: math.hypot(d['fwd'], d['right']))
            if best is None:
                self.pose_pubs[suffix].publish(self._pose_msg(stamp))
            else:
                # camera frame like vision_geo's solvePnP: x = right, y = down, z = height
                self.pose_pubs[suffix].publish(
                    self._pose_msg(stamp, best['right'], -best['fwd'], pose.agl))

    def _publish_lost(self, suffixes):
        stamp = self.get_clock().now().to_msg()
        for s in suffixes:
            self.pose_pubs[s].publish(self._pose_msg(stamp))

    # ---- /vision_geo/map/buoy_<state>: Task 1 map, whole map every frame -----
    def _publish_map(self, meta=None):
        meta = meta or self.last_meta
        if meta is None or self.home is None:
            return
        stamp = meta[0]
        for state, targets in self.mapper.snapshot().items():
            arr = self._array(stamp, 'map')
            for t in targets:
                north, east = latlon_to_offset(self.home[0], self.home[1], t['lat'], t['lon'])
                arr.objects.append(self._obj(stamp, 'map', 'buoy_' + state, t['id'], t['conf'],
                                             0.0, 0.0, t['lat'], t['lon'], 0.0, north, east))
            self.map_pubs[state].publish(arr)

    # ---- debug image + GStreamer stream ------------------------------------------
    def _debug(self, frame, overlay):
        want_topic = self.publish_debug and self.debug_pub.get_subscription_count() > 0
        want_stream = self._ensure_stream()
        if not (want_topic or want_stream):
            return
        img = self._annotate(frame, overlay)
        if want_topic:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        if want_stream:
            c = self.stream_cfg
            if img.shape[1] != c['stream_width'] or img.shape[0] != c['stream_height']:
                img = cv2.resize(img, (c['stream_width'], c['stream_height']))
            self.stream_writer.write(img)

    def _ensure_stream(self):
        """Open the GStreamer pipeline once (same pipeline as vision_hailo)."""
        c = self.stream_cfg
        if not c['enable_stream']:
            return False
        if self.stream_ready:
            return self.stream_writer is not None
        self.stream_ready = True
        pipeline = (
            'appsrc is-live=true do-timestamp=true format=time ! '
            f'video/x-raw,format=BGR,width={c["stream_width"]},height={c["stream_height"]},'
            f'framerate={c["stream_fps"]}/1 ! '
            'videoconvert ! '
            f'x264enc tune=zerolatency bitrate={c["stream_bitrate_kbps"]} '
            f'speed-preset=superfast key-int-max={c["stream_fps"]} byte-stream=true ! '
            'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={c["stream_host"]} port={c["stream_port"]} sync=false async=false')
        writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, float(c['stream_fps']),
                                 (c['stream_width'], c['stream_height']), True)
        if not writer.isOpened():
            self.get_logger().error(
                f'Failed to open GStreamer stream pipeline. Disabling stream. Pipeline: {pipeline}')
            c['enable_stream'] = False
            return False
        self.stream_writer = writer
        self.get_logger().info(
            f'Streaming enabled to udp://{c["stream_host"]}:{c["stream_port"]} '
            f'({c["stream_width"]}x{c["stream_height"]} @ {c["stream_fps"]} fps)')
        return True

    def _annotate(self, frame, overlay):
        img = frame.copy()
        h, w = img.shape[:2]
        cv2.line(img, (0, h // 2), (w, h // 2), (0, 255, 0), 1)       # crosshair
        cv2.line(img, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
        for label, d in overlay:
            x1, y1 = int(d['u'] - d['w'] / 2), int(d['v'] - d['h'] / 2)
            x2, y2 = int(d['u'] + d['w'] / 2), int(d['v'] + d['h'] / 2)
            hot = label in ('buoy_entry', 'buoy_exit')
            color = (0, 0, 255) if hot else (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2 if hot else 1)
            cv2.putText(img, label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2 if hot else 1)
        cv2.putText(img, f'MISSION {self.active}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return img


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stream_writer is not None:
            node.stream_writer.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
