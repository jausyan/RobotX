#!/usr/bin/env python3
"""UAV <-> GCS bridge (team communication format, rx_msgs).

Only /system/... topics cross to the GCS / other vehicles (DDS Router). Everything
else stays inside the UAV domain. This node translates between the two:

  GCS -> UAV
    /system/mission/command (rx_msgs/Command, for the UAV or for all vehicles)
        CMD_RUN_START    -> "RUN-START"               on /mission/order
        CMD_GO           -> "UAV-GO"                  (starts vision)
        CMD_DELIVERY     -> "UAV-GO:<TIN>:<CIRCLE>"   + delivery echo (ResourceDelivery)
        CMD_MISSION_DONE -> "MISSION-DONE"
      Orders are re-sent every resend_period_s until control confirms them with
      "ACK:<order>" (waitCommand()), so an order that arrives while control is busy
      with another task is not lost.
    /system/vehicle/usv/heartbeat -> USV position as NavSatFix on /USV/global_position/global

  UAV -> GCS
    /system/vehicle/uav/heartbeat           rx_msgs/Heartbeat, 2 Hz, from MAVROS + current task
    /system/mission/status                  rx_msgs/MissionStatus, 1 Hz and on change
    /system/vehicle/uav/task1/safe_passage  rx_msgs/SafePassage, from /vision_geo/map/buoy_*,
                                            on change + every safe_passage_period_s
    /system/vehicle/uav/task{2,3}/delivery  rx_msgs/ResourceDelivery echo, when CMD_DELIVERY arrives
"""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rx_msgs.msg import (Command, Course, Heartbeat, LatLng, MissionStatus,
                         ResourceDelivery, SafePassage, BuoyDetection)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String, UInt8
from vision_msgs.msg import DetectedObjectArray

from . import translate as tr


class UavBridge(Node):
    def __init__(self):
        super().__init__('uav_bridge')
        d = self.declare_parameter
        # internal (UAV domain) topics
        d('topics.order', '/mission/order')
        d('topics.current_task', '/mission/current_task')
        d('topics.usv_gps', '/USV/global_position/global')
        d('topics.buoy_map_prefix', '/vision_geo/map/')
        d('topics.mavros_ns', '/rian')                  # MAVROS namespace (same as control)
        # /system/... topics (team interface)
        d('system.command', '/system/mission/command')
        d('system.course', '/system/mission/course')
        d('system.status', '/system/mission/status')
        d('system.vehicle_prefix', '/system/vehicle/uav')
        d('system.usv_heartbeat', '/system/vehicle/usv/heartbeat')
        d('vehicle_id', 'uav')
        d('heartbeat_hz', 2.0)
        d('resend_period_s', 1.0)
        d('go_repeat', 3)                                # "UAV-GO" (no ACK) is sent this many times
        d('safe_passage_period_s', 1.0)
        d('safe_passage_move_tol_m', 0.5)
        d('airborne_alt_m', 0.5)
        g = lambda n: self.get_parameter(n).value

        ns = g('topics.mavros_ns').rstrip('/')
        veh = g('system.vehicle_prefix').rstrip('/')
        self.vehicle_id = g('vehicle_id')
        self.move_tol = float(g('safe_passage_move_tol_m'))
        self.airborne_alt = float(g('airborne_alt_m'))
        self.go_repeat = int(g('go_repeat'))

        # ---- state ------------------------------------------------------------
        self.run_id = 0
        self.current_task = tr.TASK_NONE
        self.seen_cmds = set()             # (command_seq, command_type) already handled
        self.pending = deque()             # orders waiting for "ACK:<order>"
        self.go_left = 0                   # remaining "UAV-GO" sends
        self.mav = {'connected': False, 'armed': False, 'mode': '', 'fix': None,
                    'rel_alt': 0.0, 'hdg': 0.0, 'speed': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self.buoys = {s: [] for s in tr.BEACON_STATE}
        self.last_field = None
        self.field_dirty = False
        self.last_status = None

        # ---- publishers ---------------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.order_pub = self.create_publisher(String, g('topics.order'), 10)
        self.usv_gps_pub = self.create_publisher(NavSatFix, g('topics.usv_gps'), 10)
        self.hb_pub = self.create_publisher(Heartbeat, veh + '/heartbeat', 10)
        self.status_pub = self.create_publisher(MissionStatus, g('system.status'), 10)
        self.sp_pub = self.create_publisher(SafePassage, veh + '/task1/safe_passage', 10)
        self.delivery_pubs = {task: self.create_publisher(
            ResourceDelivery, f'{veh}/{name}/delivery', 10)
            for task, name in tr.DELIVERY_TOPIC.items()}

        # ---- subscribers --------------------------------------------------------
        self.create_subscription(Command, g('system.command'), self.command_cb, 10)
        self.create_subscription(Course, g('system.course'), self.course_cb, latched)
        self.create_subscription(Heartbeat, g('system.usv_heartbeat'), self.usv_cb, 10)
        self.create_subscription(String, g('topics.order'), self.order_cb, 10)
        self.create_subscription(UInt8, g('topics.current_task'), self.task_cb, latched)
        prefix = g('topics.buoy_map_prefix').rstrip('/') + '/'
        for state in tr.BEACON_STATE:
            self.create_subscription(DetectedObjectArray, prefix + 'buoy_' + state,
                                     lambda m, s=state: self.buoy_cb(s, m), 10)
        sd = qos_profile_sensor_data
        self.create_subscription(State, ns + '/state', self.state_cb, 10)
        self.create_subscription(NavSatFix, ns + '/global_position/global', self.fix_cb, sd)
        self.create_subscription(Float64, ns + '/global_position/rel_alt', self.alt_cb, sd)
        self.create_subscription(Float64, ns + '/global_position/compass_hdg', self.hdg_cb, sd)
        self.create_subscription(TwistStamped, ns + '/local_position/velocity_local', self.vel_cb, sd)
        self.create_subscription(PoseStamped, ns + '/local_position/pose', self.pose_cb, sd)

        # ---- timers -------------------------------------------------------------
        self.create_timer(1.0 / float(g('heartbeat_hz')), self.publish_heartbeat)
        self.create_timer(float(g('resend_period_s')), self.resend_orders)
        self.create_timer(float(g('safe_passage_period_s')), lambda: self.publish_safe_passage(True))
        # vision sends the 5 buoy_<state> topics one after another; collect them, then publish once
        self.create_timer(0.2, self.flush_safe_passage)
        self.create_timer(1.0, lambda: self.publish_status(force=True))

        self.get_logger().info(
            f'UAV bridge ready: {g("system.command")} -> {g("topics.order")}, '
            f'heartbeat -> {veh}/heartbeat, MAVROS ns {ns}')

    # ======================================================================
    # GCS -> UAV
    # ======================================================================
    def command_cb(self, msg):
        if not tr.is_for_uav(msg.target_vehicle):
            return
        key = (msg.command_seq, msg.command_type)
        if key in self.seen_cmds:
            return                                  # GCS re-sent the same command
        self.seen_cmds.add(key)

        if msg.command_type in tr.CMD_TASK4:
            self.get_logger().warning(
                f'{tr.CMD_TASK4[msg.command_type]} (seq {msg.command_seq}) not supported yet - ignored')
            return
        order = tr.command_to_order(msg.command_type, msg.resource_color,
                                    msg.delivery_circle_color)
        if order is None:
            self.get_logger().warning(f'Command type {msg.command_type} (seq {msg.command_seq}) ignored')
            return

        if msg.command_type == tr.CMD_RUN_START:
            self.run_id = msg.run_id
        if msg.command_type == tr.CMD_DELIVERY:
            self.echo_delivery(msg)

        self.get_logger().info(f'Command seq {msg.command_seq} -> "{order}"')
        if tr.needs_ack(order):
            self.pending.append(order)
        else:
            self.go_left = self.go_repeat
        self.resend_orders()

    def echo_delivery(self, cmd):
        pub = self.delivery_pubs.get(cmd.task)
        if pub is None:
            self.get_logger().warning(f'CMD_DELIVERY with unknown task {cmd.task}, no echo')
            return
        echo = ResourceDelivery()
        echo.header.stamp = self.get_clock().now().to_msg()
        echo.task = cmd.task
        echo.resource_color = cmd.resource_color
        echo.delivery_circle_color = cmd.delivery_circle_color
        pub.publish(echo)

    def resend_orders(self):
        # kept until control sends ACK:<order>. Every non-delivery order, but only the
        # OLDEST delivery, so the Task 2 order is always consumed before the Task 3 one.
        oldest_delivery = next((o for o in self.pending if o.startswith('UAV-GO:')), None)
        for order in self.pending:
            if not order.startswith('UAV-GO:') or order is oldest_delivery:
                self.order_pub.publish(String(data=order))
        if self.go_left > 0:
            self.order_pub.publish(String(data='UAV-GO'))
            self.go_left -= 1

    def order_cb(self, msg):
        text = msg.data.strip().upper()
        if not text.startswith('ACK:'):
            return
        acked = text[4:]
        if acked in self.pending:
            self.pending.remove(acked)
            self.get_logger().info(f'"{acked}" confirmed by control')

    def course_cb(self, msg):
        self.get_logger().info(
            f'Course {msg.course_id}: boundary {len(msg.boundary)} pts, '
            f'UAV geofence {len(msg.uav_geofence)} pts')

    def usv_cb(self, msg):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'usv'
        fix.status.status = 0
        fix.latitude, fix.longitude = msg.position.latitude, msg.position.longitude
        fix.altitude = float(msg.altitude_hae_m)
        self.usv_gps_pub.publish(fix)

    # ======================================================================
    # UAV -> GCS
    # ======================================================================
    def task_cb(self, msg):
        if msg.data != self.current_task:
            self.get_logger().info(f'current_task {self.current_task} -> {msg.data}')
            self.current_task = msg.data
            self.publish_heartbeat()
            self.publish_status(force=True)

    def state_cb(self, msg):
        self.mav.update(connected=msg.connected, armed=msg.armed, mode=msg.mode)

    def fix_cb(self, msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.mav['fix'] = msg

    def alt_cb(self, msg):
        self.mav['rel_alt'] = msg.data

    def hdg_cb(self, msg):
        self.mav['hdg'] = msg.data

    def vel_cb(self, msg):
        v = msg.twist.linear
        self.mav['speed'] = math.hypot(v.x, v.y)

    def pose_cb(self, msg):
        q = msg.pose.orientation
        self.mav['roll'], self.mav['pitch'] = tr.roll_pitch_deg(q.x, q.y, q.z, q.w)

    def publish_heartbeat(self):
        m = self.mav
        hb = Heartbeat()
        hb.header.stamp = self.get_clock().now().to_msg()
        hb.state = tr.robot_state(m['connected'], m['mode'])
        if m['fix'] is not None:
            hb.position = LatLng(latitude=m['fix'].latitude, longitude=m['fix'].longitude)
            hb.altitude_hae_m = float(m['fix'].altitude)   # MAVROS global fix altitude = ellipsoid
        hb.spd_mps = float(m['speed'])
        hb.heading_deg = float(m['hdg'])
        hb.roll_deg = float(m['roll'])
        hb.pitch_deg = float(m['pitch'])
        hb.current_task = self.current_task
        hb.vehicle_type = tr.TYPE_UAV
        hb.flight_phase = tr.flight_phase(m['armed'], m['rel_alt'], self.airborne_alt)
        self.hb_pub.publish(hb)

    def publish_status(self, force=False):
        state = tr.robot_state(self.mav['connected'], self.mav['mode'])
        key = (self.run_id, state, self.current_task)
        if not force and key == self.last_status:
            return
        self.last_status = key
        st = MissionStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.run_id, st.vehicle_id = self.run_id, self.vehicle_id
        st.state, st.current_task = state, self.current_task
        st.message = f'mode={self.mav["mode"]} armed={self.mav["armed"]} pending={len(self.pending)}'
        self.status_pub.publish(st)

    def buoy_cb(self, state, msg):
        self.buoys[state] = [(o.latitude, o.longitude) for o in msg.objects]
        self.field_dirty = True

    def flush_safe_passage(self):
        if self.field_dirty:
            self.field_dirty = False
            self.publish_safe_passage(periodic=False)

    def publish_safe_passage(self, periodic):
        entry, exit_, buoys = tr.buoy_field(self.buoys)
        if not buoys:
            return                                   # nothing mapped yet
        if not periodic and not tr.field_changed(self.last_field, buoys, self.move_tol):
            return
        if not periodic:
            self.get_logger().info(
                f'SafePassage update: {len(buoys)} buoys, entry={entry}, exit={exit_}')
        self.last_field = buoys
        sp = SafePassage()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.entry_position = LatLng(latitude=entry[0], longitude=entry[1])
        sp.exit_position = LatLng(latitude=exit_[0], longitude=exit_[1])
        sp.buoys = [BuoyDetection(position=LatLng(latitude=la, longitude=lo), state=s)
                    for la, lo, s in buoys]
        self.sp_pub.publish(sp)


def main(args=None):
    rclpy.init(args=args)
    node = UavBridge()
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
