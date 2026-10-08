"""Pure helpers with no ROS dependency (easy to unit-test).

Conventions
-----------
* Camera is nadir (pointing straight down). Image "up" = drone forward,
  image "right" = drone right.
* heading_deg is a compass heading: degrees clockwise from north.
* north / east offsets are metres from the drone to the target.
"""
import math
import re

COLORS = ('red', 'green', 'blue')
M_PER_DEG = 111320.0


# --------------------------------------------------------------------------
# Naming
# --------------------------------------------------------------------------
def norm_label(name):
    """'Blue-Light ' -> 'blue_light' (hyphens are illegal in ROS topic names)."""
    return name.strip().lower().replace('-', '_').replace(' ', '_')


def topic_suffix(label):
    """Model class name -> topic suffix used by the C++ control node.

    red_circle -> circle_red, blue_tin -> tin_blue, green_light -> green_light
    """
    parts = norm_label(label).split('_')
    if len(parts) == 2 and parts[0] in COLORS and parts[1] in ('circle', 'tin'):
        return f'{parts[1]}_{parts[0]}'
    return '_'.join(parts)


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------
def pixel_to_body(u, v, fx, fy, cx, cy, height_m):
    """Pinhole projection of a pixel onto the ground plane -> (forward_m, right_m)."""
    forward = (cy - v) * height_m / fy
    right = (u - cx) * height_m / fx
    return forward, right


def normalized_to_body(xn, yn, height_m):
    """Same as pixel_to_body but from undistorted normalized coords ((u-cx)/fx, (v-cy)/fy)."""
    return -yn * height_m, xn * height_m


def body_to_ne(forward, right, heading_deg):
    """Rotate body-frame (forward, right) offsets into (north, east)."""
    h = math.radians(heading_deg)
    north = forward * math.cos(h) - right * math.sin(h)
    east = forward * math.sin(h) + right * math.cos(h)
    return north, east


def offset_to_latlon(lat, lon, north, east):
    dlat = north / M_PER_DEG
    dlon = east / (M_PER_DEG * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def latlon_to_offset(lat0, lon0, lat, lon):
    """(north_m, east_m) from point 0 to point 1."""
    north = (lat - lat0) * M_PER_DEG
    east = (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0))
    return north, east


def dist_m(lat1, lon1, lat2, lon2):
    north, east = latlon_to_offset(lat1, lon1, lat2, lon2)
    return math.hypot(north, east)


# --------------------------------------------------------------------------
# Order parsing (best effort - align with the C++ parseCommand())
# --------------------------------------------------------------------------
def _find_color(key, text):
    c = '|'.join(COLORS)
    m = (re.search(rf'{key}[\s_:=,\-]*({c})', text)
         or re.search(rf'({c})[\s_:=,\-]*{key}', text))
    return m.group(1) if m else None


def parse_order(text):
    """Return (tin_color, circle_color) or (None, None).

    Understands e.g. 'UAV-GO,tin:red,circle:blue', 'tin_red circle_blue'.
    Falls back to "first colour = tin, second colour = circle".
    """
    t = text.lower()
    tin, circle = _find_color('tin', t), _find_color('circle', t)
    if tin and circle:
        return tin, circle
    colors = re.findall('|'.join(COLORS), t)
    if len(colors) >= 2:
        return colors[0], colors[1]
    return None, None
