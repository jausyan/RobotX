import math

from uav_bridge import translate as tr


def test_command_to_order():
    assert tr.command_to_order(tr.CMD_RUN_START) == 'RUN-START'
    assert tr.command_to_order(tr.CMD_GO) == 'UAV-GO'
    assert tr.command_to_order(tr.CMD_MISSION_DONE) == 'MISSION-DONE'
    assert tr.command_to_order(tr.CMD_DELIVERY, 1, 2) == 'UAV-GO:RED:GREEN'
    assert tr.command_to_order(tr.CMD_DELIVERY, 3, 1) == 'UAV-GO:BLUE:RED'
    assert tr.command_to_order(tr.CMD_DELIVERY, 0, 1) is None        # COLOR_UNKNOWN
    assert tr.command_to_order(3) is None                             # Task 4


def test_target_and_ack():
    assert tr.is_for_uav(tr.TYPE_UAV) and tr.is_for_uav(tr.TYPE_UNKNOWN)
    assert not tr.is_for_uav(1) and not tr.is_for_uav(2)              # USV, UUV
    assert not tr.needs_ack('UAV-GO') and tr.needs_ack('UAV-GO:RED:GREEN')
    assert tr.needs_ack('MISSION-DONE') and tr.needs_ack('RUN-START')


def test_state_phase():
    assert tr.robot_state(True, 'GUIDED') == tr.STATE_AUTO
    assert tr.robot_state(True, 'auto') == tr.STATE_AUTO
    assert tr.robot_state(True, 'STABILIZE') == tr.STATE_MANUAL
    assert tr.robot_state(False, 'GUIDED') == tr.STATE_KILLED
    assert tr.flight_phase(True, 10.0) == tr.FLIGHT_PHASE_AIRBORNE
    assert tr.flight_phase(False, 10.0) == tr.FLIGHT_PHASE_GROUNDED
    assert tr.flight_phase(True, 0.1) == tr.FLIGHT_PHASE_GROUNDED


def test_roll_pitch():
    r, p = tr.roll_pitch_deg(0, 0, 0, 1)
    assert math.isclose(r, 0, abs_tol=1e-9) and math.isclose(p, 0, abs_tol=1e-9)
    a = math.radians(10) / 2                                   # +10 deg about body x (FLU)
    r, p = tr.roll_pitch_deg(math.sin(a), 0, 0, math.cos(a))
    assert math.isclose(r, 10, abs_tol=1e-6)
    r, p = tr.roll_pitch_deg(0, math.sin(a), 0, math.cos(a))   # +10 about FLU y = nose down
    assert math.isclose(p, -10, abs_tol=1e-6)


def test_buoy_field():
    entry, exit_, buoys = tr.buoy_field({'red': [(1.0, 2.0)], 'entry': [(3.0, 4.0)],
                                         'exit': [], 'green': [], 'off': [(5.0, 6.0)]})
    assert entry == (3.0, 4.0) and exit_ == (0.0, 0.0)              # exit not found yet
    assert sorted(b[2] for b in buoys) == [1, 2, 4]                 # OFF, FLASHING_RED, FLASHING_BLUE


def test_field_changed():
    a = [(-35.0, 149.0, 2)]
    assert tr.field_changed(None, a)
    assert not tr.field_changed(a, [(-35.0, 149.000001, 2)])       # ~0.09 m
    assert tr.field_changed(a, [(-35.0, 149.00001, 2)])            # ~0.9 m
    assert tr.field_changed(a, [(-35.0, 149.0, 1)])                # red -> off
    assert tr.field_changed(a, a + [(-35.1, 149.0, 3)])            # new buoy
