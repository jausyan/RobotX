#!/usr/bin/env python3
"""RobotX UAV vision node (real-drone port of the ArduPilot simulation scripts).

Driven entirely by String messages on the order topic (default /mission/order):

  UAV-GO #1  -> MISSION 1: run the task1 model, map the lights (incl. blinking),
                publish one DetectedObjectArray per class. After
                `task1.mapping_time_s` it publishes "MISSION-DONE" on the order
                topic (the C++ node waits for it) and goes idle.
  UAV-GO #2  -> MISSION 2: run the task2 model, publish circle_* / tin_* /
                green_light detections (geo-referenced).
  UAV-GO #3  -> MISSION 3: same as mission 2.

While idle nothing is inferred and nothing is published.

Output topics: <target_prefix><name>, e.g. /vision_geo/target/circle_red,
/vision_geo/target/tin_blue, /vision_geo/target/green_light, type
vision_msgs/DetectedObjectArray. `altitude_m` is the drone's height above the
target plane used for the projection; north/east_offset_m are metres from the
drone to the object.
"""
import math
import os
from collections import namedtuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
from ultralytics import YOLO
from vision_msgs.msg import DetectedObject, DetectedObjectArray

from .geo import (body_to_ne, latlon_to_offset, norm_label, offset_to_latlon,
                  parse_order, pixel_to_body, topic_suffix)
from .light_mapper import BLINK, LightMapper

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
        self.only_ordered = bool(g('task23.only_publish_ordered'))
        self.active_timeout = float(g('task23.active_timeout_s'))
        self.publish_debug = bool(g('topics.publish_debug'))
        self.prefix = g('topics.target_prefix').rstrip('/') + '/'

        # ---- models ---------------------------------------------------------
        self.bridge = CvBridge()
        self.model1 = self._load(g('model.task1_weights'))
        self.model2 = self._load(g('model.task2_weights'))
        self.blink_id = next((i for i, n in self.model1.names.items()
                              if topic_suffix(n) == BLINK), -1)

        self.mapper = LightMapper(
            self.get_logger(),
            dist_tol_m=float(g('task1.dist_tol_m')),
            dedup_tol_m=float(g('task1.dedup_tol_m')),
            blink_min_s=float(g('task1.blink_min_s')),
            blink_max_s=float(g('task1.blink_max_s')))

        # ---- mission state ----------------------------------------------------
        self.next_mission = int(g('mission.start_index'))   # started by next UAV-GO
        self.active = 0                                      # 0 = idle
        self.mission_t0 = 0.0
        self.order = (None, None)                            # (tin, circle)
        self.last_meta = None

        # ---- sensors ----------------------------------------------------------
        self.lat = self.lon = self.rel_alt = self.hdg = None
        self.t_gps = self.t_alt = self.t_hdg = 0.0

        # ---- publishers / subscribers ----------------------------------------
        self.pubs = {}
        labels = {topic_suffix(n) for m in (self.model1, self.model2)
                  for n in m.names.values()} | {BLINK}
        for s in sorted(labels):          # create up-front so DDS discovery is done
            self._get_pub(s)
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
            f'Vision node ready. Waiting for UAV-GO on {g("topics.order")} '
            f'(next mission: {self.next_mission}). Targets -> {self.prefix}<name>')

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
        d('topics.target_prefix', '/vision_geo/target/')
        d('topics.debug_image', '/vision_geo/debug_image')
        d('topics.publish_debug', True)
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
        d('task1.mapping_time_s', 60.0)   # keep equal to mission.survey_time
        d('task1.dist_tol_m', 3.0)
        d('task1.dedup_tol_m', 1.5)
        d('task1.blink_min_s', 0.3)
        d('task1.blink_max_s', 3.0)
        d('task23.only_publish_ordered', False)
        d('task23.active_timeout_s', 0.0)  # 0 = stay active until next UAV-GO
        d('mission.start_index', 1)        # 1..3, which mission the next UAV-GO starts

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

    def _get_pub(self, suffix):
        pub = self.pubs.get(suffix)
        if pub is None:
            pub = self.create_publisher(DetectedObjectArray, self.prefix + suffix, 10)
            self.pubs[suffix] = pub
        return pub

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
    # Mission sequencing (/mission/order)
    # ======================================================================
    def order_cb(self, msg):
        text = msg.data.strip()
        up = text.upper()
        if 'MISSION-DONE' in up or 'UAV-GO' not in up:
            return                      # includes our own MISSION-DONE echo
        n = self.next_mission
        self.active = n
        self.mission_t0 = self._now()
        self.next_mission = min(n + 1, 3)
        if n == 1:
            self.mapper.reset()
            self.get_logger().info(f'MISSION 1 START: mapping for {self.mapping_time:.0f}s')
        else:
            self.order = parse_order(text)
            self.get_logger().info(
                f'MISSION {n} START: order="{text}" -> tin={self.order[0]}, '
                f'circle={self.order[1]}')

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
            overlay = (self._mission1(dets, meta) if self.active == 1
                       else self._mission23(dets, meta))
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
            dets.append({'label': norm_label(names[int(c)]), 'class_id': int(c),
                         'conf': float(s), 'u': float(u), 'v': float(v),
                         'w': float(w), 'h': float(h), 'north': float(north),
                         'east': float(east), 'lat': lat, 'lon': lon})
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

    # ---- mission 1 --------------------------------------------------------------
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

    # ---- missions 2 & 3 ---------------------------------------------------------
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
            items.sort(key=lambda d: d['w'] * d['h'], reverse=True)   # largest first
            arr = self._array(stamp, frame_id)
            for d in items:
                arr.objects.append(self._obj(
                    stamp, frame_id, suffix, d['class_id'], d['conf'], d['u'], d['v'],
                    d['lat'], d['lon'], pose.agl, d['north'], d['east']))
                overlay.append((suffix, d))
            self._get_pub(suffix).publish(arr)
        return overlay

    # ---- debug overlay -------------------------------------------------------------
    def _debug(self, frame, overlay):
        if not self.publish_debug or self.debug_pub.get_subscription_count() == 0:
            return
        img = frame.copy()
        for label, d in overlay:
            x1, y1 = int(d['u'] - d['w'] / 2), int(d['v'] - d['h'] / 2)
            x2, y2 = int(d['u'] + d['w'] / 2), int(d['v'] + d['h'] / 2)
            hot = label == BLINK
            color = (0, 0, 255) if hot else (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2 if hot else 1)
            cv2.putText(img, label.upper() if hot else label, (x1, max(12, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2 if hot else 1)
        cv2.putText(img, f'MISSION {self.active}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
