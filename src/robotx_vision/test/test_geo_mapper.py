import math

from robotx_vision.geo import (body_to_ne, dist_m, offset_to_latlon,
                               parse_order, pixel_to_body, topic_suffix)
from robotx_vision.light_mapper import LightMapper, light_color


class Log:
    def info(self, *_): pass
    def warning(self, *_): pass


def test_topic_suffix():
    assert topic_suffix('red_circle') == 'circle_red'
    assert topic_suffix('blue_tin') == 'tin_blue'
    assert topic_suffix('green_light') == 'green_light'
    assert topic_suffix('blinking-light') == 'blinking_light'


def test_projection_heading():
    fwd, right = pixel_to_body(320, 140, 200, 200, 320, 240, 10.0)   # 100px up
    assert math.isclose(fwd, 5.0) and math.isclose(right, 0.0)
    n, e = body_to_ne(0.0, 1.0, 90.0)        # facing east, right = south
    assert math.isclose(n, -1.0, abs_tol=1e-9) and math.isclose(e, 0.0, abs_tol=1e-9)
    lat, lon = offset_to_latlon(-7.28, 112.79, 3.0, 4.0)
    assert math.isclose(dist_m(-7.28, 112.79, lat, lon), 5.0, rel_tol=1e-3)


def test_parse_order():
    assert parse_order('UAV-GO:RED:GREEN') == ('red', 'green')     # our command format
    assert parse_order('UAV-GO:BLUE:RED') == ('blue', 'red')
    assert parse_order('UAV-GO,tin:red,circle:blue') == ('red', 'blue')
    assert parse_order('UAV-GO') == (None, None)


def test_light_color():
    assert light_color('red_light') == 'red'
    assert light_color('blue_light') == 'blue'
    assert light_color('grey_light') == 'off'
    assert light_color('blinking_light') == 'blue'
    assert light_color('circle_red') == 'red'       # filtered out of Task 1 by the model
    assert light_color('tin') is None


# ---- buoy state: same cases as the vision_geo (C++) check ----------------------
def _det(label, lat=-7.28, lon=112.79):
    return {'label': label, 'class_id': 0, 'conf': 0.9, 'u': 0, 'v': 0, 'lat': lat, 'lon': lon}


def _run(light):
    """light(t) -> label or None (not detected); 10 s at 10 fps, one buoy."""
    m = LightMapper(Log())
    for i in range(100):
        t = i * 0.1
        lbl = light(t)
        m.update([_det(lbl)] if lbl else [], t)
    assert len(m.targets) == 1
    return m.targets[0]['state']


def _on(t):
    return int(t) % 2 == 0          # 1 s on / 1 s off


def test_flashing_red():
    assert _run(lambda t: 'red_light' if _on(t) else 'grey_light') == 'red'


def test_flashing_blue_is_entry():
    assert _run(lambda t: 'blue_light' if _on(t) else 'grey_light') == 'entry'


def test_flashing_blue_off_missed_is_entry():
    assert _run(lambda t: 'blue_light' if _on(t) else None) == 'entry'


def test_solid_blue_is_exit():
    assert _run(lambda t: 'blue_light') == 'exit'


def test_unlit_is_off():
    assert _run(lambda t: 'grey_light') == 'off'


def test_hazard_change_red_to_off():
    # Disruptive: same buoy changes, no stale red copy is left
    assert _run(lambda t: ('red_light' if _on(t) else 'grey_light') if t < 3 else 'grey_light') == 'off'


def test_snapshot_groups_by_state():
    m = LightMapper(Log())
    for i in range(60):
        t = i * 0.1
        m.update([_det('red_light', lat=-7.28), _det('blue_light', lat=-7.2801)], t)
    snap = m.snapshot()
    assert set(snap) == {'red', 'green', 'entry', 'exit', 'off'}
    assert len(snap['red']) == 1 and len(snap['exit']) == 1
