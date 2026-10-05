import math

from robotx_vision.geo import (body_to_ne, dist_m, offset_to_latlon,
                               parse_order, pixel_to_body, topic_suffix)
from robotx_vision.light_mapper import LightMapper


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
    assert parse_order('UAV-GO,tin:red,circle:blue') == ('red', 'blue')
    assert parse_order('UAV-GO circle_green tin_blue') == ('blue', 'green')
    assert parse_order('UAV-GO') == (None, None)


def _det(label, lat=-7.28, lon=112.79):
    return {'label': label, 'class_id': 0, 'conf': 0.9, 'u': 0, 'v': 0, 'lat': lat, 'lon': lon}


def test_blink_confirmed_and_colour_lock():
    m = LightMapper(Log())
    t = 0.0
    for lbl in ['grey_light', 'blue_light', 'grey_light', 'blue_light']:
        m.update([_det(lbl)], t)
        t += 1.0
    assert m.snapshot().keys() == {'blinking_light'}
    m.update([_det('red_light')], t)          # red must not hijack the blinking target
    assert set(m.snapshot()) == {'blinking_light', 'red_light'}


def test_static_light_not_blinking():
    m = LightMapper(Log())
    for i in range(20):
        m.update([_det('grey_light')], i * 0.1)
    assert set(m.snapshot()) == {'grey_light'}
