import math

import numpy as np

from robotx_vision.utils.detector import Detector
from robotx_vision.utils.geo import (body_to_ne, dist_m, normalized_to_body,
                                     offset_to_latlon, parse_order,
                                     pixel_to_body, topic_suffix)
from robotx_vision.utils.light_mapper import LightMapper


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
    f2, r2 = normalized_to_body((320 - 320) / 200, (140 - 240) / 200, 10.0)
    assert math.isclose(f2, fwd) and math.isclose(r2, right, abs_tol=1e-12)
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


def _bare_detector(letterbox):
    d = Detector.__new__(Detector)
    d.in_w = d.in_h = 320
    d.conf, d.nms, d.letterbox = 0.4, 0.45, letterbox
    return d


def test_detector_decode_letterbox_roundtrip():
    # 640x480 frame, object centred at (400, 120), 80x60 px, class 1 of 3
    d = _bare_detector(True)
    frame = np.zeros((480, 640, 3), np.uint8)
    _, tf = d._preprocess(frame)
    x0, y0, sx, sy = tf
    cx_m, cy_m = 400 / sx + x0, 120 / sy + y0
    out = np.zeros((1, 7, 10), np.float32)          # channels-first: 4 box + 3 classes, 10 anchors
    out[0, :4, 3] = [cx_m, cy_m, 80 / sx, 60 / sy]
    out[0, 4 + 1, 3] = 0.9
    res = d._decode(out, tf, frame.shape)
    assert d.num_classes == 3 and len(res) == 1
    cid, conf, u, v, w, h = res[0]
    assert cid == 1 and math.isclose(conf, 0.9, rel_tol=1e-5)
    assert math.isclose(u, 400, abs_tol=1e-3) and math.isclose(v, 120, abs_tol=1e-3)
    assert math.isclose(w, 80, abs_tol=1e-3) and math.isclose(h, 60, abs_tol=1e-3)


def test_detector_decode_channels_last_and_threshold():
    d = _bare_detector(False)
    frame = np.zeros((480, 640, 3), np.uint8)
    _, tf = d._preprocess(frame)
    out = np.zeros((1, 300, 6), np.float32)         # channels-last
    out[0, 5, :4] = [160, 160, 40, 40]; out[0, 5, 4] = 0.2   # below conf_threshold
    assert d._decode(out, tf, frame.shape) == []
    out[0, 5, 4] = 0.7
    res = d._decode(out, tf, frame.shape)
    assert len(res) == 1 and math.isclose(res[0][2], 320, abs_tol=1e-3)   # stretched x2
