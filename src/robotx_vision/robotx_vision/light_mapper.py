"""Mission 1 buoy map + light state (no ROS dependency).

Same behaviour as vision_geo (C++):
  * one target per PHYSICAL buoy: any light detection within dist_tol_m is the
    same buoy, even when its colour changes (a flashing light looks like
    "red, off, red, off ..."; a Disruptive hazard change turns red -> off)
  * position = running mean of the sightings
  * state from the last state_window_s of (time, colour) sightings:
      lit red / green                       -> 'red' / 'green'
      lit blue + flashing                   -> 'entry'  (off between lit sightings,
                                                         or a flash-sized gap)
      lit blue, steady for solid_min_s      -> 'exit'
      only off, for flash_gap_max_s         -> 'off'
    not enough evidence -> keep the previous state ('unknown' at first)
"""
from collections import Counter, deque

from .geo import dist_m

STATES = ('red', 'green', 'entry', 'exit', 'off')


def light_color(label):
    """Model label -> 'red' / 'green' / 'blue' / 'off', or None (not a buoy light)."""
    lbl = label.lower()
    for color in ('red', 'green', 'blue'):
        if color in lbl:
            return color
    if 'blink' in lbl:
        return 'blue'                      # flashing beacons are blue
    if any(k in lbl for k in ('grey', 'gray', 'off', 'black')):
        return 'off'
    return None


class LightMapper:
    def __init__(self, logger, dist_tol_m=2.0, dedup_tol_m=1.5, state_window_s=6.0,
                 flash_gap_min_s=0.6, flash_gap_max_s=1.6, solid_min_s=3.0,
                 max_samples=50):
        self.log = logger
        self.dist_tol = dist_tol_m
        self.dedup_tol = dedup_tol_m
        self.window = state_window_s
        self.gap_min = flash_gap_min_s
        self.gap_max = flash_gap_max_s
        self.solid_min = solid_min_s
        self.max_samples = max_samples
        self.targets = []
        self.next_id = 1

    def reset(self):
        self.targets = []
        self.next_id = 1

    def _best_match(self, det, taken):
        best, best_d = None, self.dist_tol
        for idx, t in enumerate(self.targets):
            if idx in taken:
                continue
            d = dist_m(t['lat'], t['lon'], det['lat'], det['lon'])
            if d < best_d:
                best, best_d = idx, d
        return best

    def update(self, dets, now):
        """dets: list of dicts (label, class_id, conf, u, v, lat, lon, ...).

        Non-light detections are ignored. Returns [(det, target), ...].
        """
        lights = [d for d in dets if light_color(d['label']) is not None]
        unique = []
        for d in lights:
            if all(dist_m(u['lat'], u['lon'], d['lat'], d['lon']) >= self.dedup_tol
                   for u in unique):
                unique.append(d)

        taken, pairs = set(), []
        for d in unique:
            idx = self._best_match(d, taken)
            if idx is None:
                self.targets.append({
                    'id': self.next_id, 'lat': d['lat'], 'lon': d['lon'], 'n': 1,
                    'u': d['u'], 'v': d['v'], 'conf': d['conf'],
                    'hist': deque(), 'state': 'unknown'})
                self.next_id += 1
                idx = len(self.targets) - 1
                self.log.info(f"[map] buoy#{self.targets[idx]['id']} NEW "
                              f"lat={d['lat']:.6f} lon={d['lon']:.6f}")
            else:
                self._refine(self.targets[idx], d)
            self.targets[idx]['hist'].append((now, light_color(d['label'])))
            taken.add(idx)
            pairs.append((d, self.targets[idx]))

        for t in self.targets:
            self._update_state(t, now)
        return pairs

    def _refine(self, t, d):
        # running mean (capped so the target can still follow slow drift)
        w = min(t['n'], self.max_samples) + 1
        t['lat'] += (d['lat'] - t['lat']) / w
        t['lon'] += (d['lon'] - t['lon']) / w
        t['n'] += 1
        t['u'], t['v'] = d['u'], d['v']
        t['conf'] = max(t['conf'], d['conf'])

    def _update_state(self, t, now):
        hist = t['hist']
        while hist and hist[0][0] < now - self.window:
            hist.popleft()
        if not hist:
            return                                  # out of view: keep last state

        lit = [(ts, c) for ts, c in hist if c != 'off']
        new = t['state']
        if not lit:
            if hist[-1][0] - hist[0][0] >= self.gap_max:
                new = 'off'
        else:
            first, last = lit[0][0], lit[-1][0]
            flashing = any(self.gap_min <= b[0] - a[0] <= self.gap_max
                           for a, b in zip(lit, lit[1:]))
            flashing = flashing or any(c == 'off' and first < ts < last for ts, c in hist)
            color = Counter(c for _, c in lit).most_common(1)[0][0]
            if color != 'blue':
                new = color
            elif flashing:
                new = 'entry'
            elif last - first >= self.solid_min:
                new = 'exit'

        if new != t['state']:
            self.log.info(f"[map] buoy#{t['id']} state {t['state']} -> {new} "
                          f"(lat={t['lat']:.6f} lon={t['lon']:.6f})")
            t['state'] = new

    def snapshot(self):
        """{state: [targets]} for every state in STATES (unknown buoys left out)."""
        out = {s: [] for s in STATES}
        for t in self.targets:
            if t['state'] in out:
                out[t['state']].append(t)
        return out

    def counts(self):
        return Counter({k: len(v) for k, v in self.snapshot().items()})

    def arena_complete(self):
        """Sanity check: at least one of each light kind found."""
        c = self.counts()
        return all(c[s] >= 1 for s in ('red', 'green', 'entry', 'exit'))
