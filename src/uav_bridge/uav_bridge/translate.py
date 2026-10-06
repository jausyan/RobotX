"""Pure translation helpers (no ROS dependency, easy to unit-test).

Constant values mirror rx_msgs (= official RoboCommand schemas).
"""
import math

# rx_msgs/Command
CMD_RUN_START, CMD_DELIVERY, CMD_GO, CMD_MISSION_DONE = 1, 2, 8, 9
CMD_TASK4 = {3: 'TASK4_ASSISTANCE', 4: 'TASK4_KEEP_OUT_ZONE', 5: 'TASK4_ALL_CLEAR',
             6: 'TASK4_MOVING_OBJECT', 7: 'TASK4_READINESS_CONFIRM'}
# rx_msgs/VehicleType
TYPE_UNKNOWN, TYPE_UAV = 0, 3
# rx_msgs/RxTask
TASK_NONE, TASK_SAFE_PASSAGE, TASK_INFRA_SURVEY_REPAIR, TASK_COORDINATED_LOGISTICS = 1, 2, 3, 4
# rx_msgs/RobotState
STATE_KILLED, STATE_MANUAL, STATE_AUTO = 1, 2, 3
# rx_msgs/FlightPhase
FLIGHT_PHASE_GROUNDED, FLIGHT_PHASE_AIRBORNE = 1, 2
# rx_msgs/Color
COLOR_NAMES = {1: 'RED', 2: 'GREEN', 3: 'BLUE', 4: 'ANY'}
# rx_msgs/BeaconState, keyed by our buoy state (topic /vision_geo/map/buoy_<state>)
BEACON_STATE = {'off': 1, 'red': 2, 'green': 3, 'entry': 4, 'exit': 5}

# ArduPilot Copter modes in which the vehicle flies itself
AUTO_MODES = ('GUIDED', 'AUTO', 'RTL', 'SMART_RTL', 'LAND', 'BRAKE')

DELIVERY_TOPIC = {TASK_INFRA_SURVEY_REPAIR: 'task2', TASK_COORDINATED_LOGISTICS: 'task3'}


def is_for_uav(target_vehicle):
    return target_vehicle in (TYPE_UNKNOWN, TYPE_UAV)


def command_to_order(command_type, resource_color=0, delivery_circle_color=0):
    """rx_msgs/Command -> legacy /mission/order string, or None (not for the order topic).

    CMD_RUN_START    -> "RUN-START"
    CMD_GO           -> "UAV-GO"              (starts vision)
    CMD_DELIVERY     -> "UAV-GO:<TIN>:<CIRCLE>"
    CMD_MISSION_DONE -> "MISSION-DONE"
    """
    if command_type == CMD_RUN_START:
        return 'RUN-START'
    if command_type == CMD_GO:
        return 'UAV-GO'
    if command_type == CMD_MISSION_DONE:
        return 'MISSION-DONE'
    if command_type == CMD_DELIVERY:
        tin = COLOR_NAMES.get(resource_color)
        circle = COLOR_NAMES.get(delivery_circle_color)
        if tin is None or circle is None:
            return None
        return f'UAV-GO:{tin}:{circle}'
    return None


def needs_ack(order):
    """Orders that control consumes with waitCommand() and confirms with "ACK:<order>".

    "UAV-GO" alone only starts vision (nobody acknowledges it), so it is sent a
    few times instead of being kept until ACK.
    """
    return order != 'UAV-GO'


def robot_state(connected, mode):
    if not connected:
        return STATE_KILLED
    return STATE_AUTO if mode.upper() in AUTO_MODES else STATE_MANUAL


def flight_phase(armed, rel_alt_m, airborne_alt_m=0.5):
    return FLIGHT_PHASE_AIRBORNE if armed and rel_alt_m > airborne_alt_m else FLIGHT_PHASE_GROUNDED


def roll_pitch_deg(qx, qy, qz, qw):
    """MAVROS local pose quaternion (ENU / FLU body) -> aviation roll, pitch in degrees.

    roll  > 0 = right wing down, pitch > 0 = nose up (FLU pitch is the opposite sign).
    """
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch_flu = math.asin(sinp)
    return math.degrees(roll), -math.degrees(pitch_flu)


def buoy_field(buoys_by_state):
    """{state: [(lat, lon), ...]} -> (entry, exit, [(lat, lon, beacon_state), ...]).

    entry / exit = first buoy of that state, or (0.0, 0.0) = not found yet
    (agreed convention with the USV).
    """
    entry = (buoys_by_state.get('entry') or [(0.0, 0.0)])[0]
    exit_ = (buoys_by_state.get('exit') or [(0.0, 0.0)])[0]
    buoys = [(lat, lon, BEACON_STATE[state])
             for state, points in sorted(buoys_by_state.items()) if state in BEACON_STATE
             for lat, lon in points]
    return entry, exit_, buoys


def field_changed(old, new, move_tol_m=0.5):
    """True when SafePassage must be republished: a buoy appeared / disappeared,
    changed state, or moved more than move_tol_m."""
    if old is None or len(old) != len(new):
        return True
    for (la1, lo1, s1), (la2, lo2, s2) in zip(old, new):
        if s1 != s2:
            return True
        north = (la2 - la1) * 111320.0
        east = (lo2 - lo1) * 111320.0 * math.cos(math.radians(la1))
        if math.hypot(north, east) > move_tol_m:
            return True
    return False
