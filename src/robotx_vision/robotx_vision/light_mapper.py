"""Mission 1 light mapping + blink detection (no ROS dependency).

Ported from the simulation Task1MappingNode:
  * spatial de-duplication inside a frame
  * tracking of mapped targets by geo-position with a colour lock
    (blue/grey/blinking targets only accept blue/grey detections, so a red
    light can never hijack a blinking one)
  * a target is promoted to `blinking_light` when it flips blue<->grey after
    being stable for blink_min_s .. blink_max_s
"""
from collections import Counter

from .geo import dist_m

BLUE, GREY, BLINK = 'blue_light', 'grey_light', 'blinking_light'
SWAPPABLE = {BLUE, GREY, BLINK}


class LightMapper:
    def __init__(self, logger, dist_tol_m=3.0, dedup_tol_m=1.5,
                 blink_min_s=0.3, blink_max_s=3.0, max_samples=50):
        self.log = logger
        self.dist_tol = dist_tol_m
        self.dedup_tol = dedup_tol_m
        self.blink_min = blink_min_s
        self.blink_max = blink_max_s
        self.max_samples = max_samples
        self.targets = []

    def reset(self):
        self.targets = []

    @staticmethod
    def _compatible(target_class, det_class):
        if target_class in SWAPPABLE:
            return det_class in (BLUE, GREY)
        return target_class == det_class

    def _best_match(self, det, taken):
        best, best_d = None, self.dist_tol
        for idx, t in enumerate(self.targets):
            if idx in taken or not self._compatible(t['final'], det['label']):
                continue
            d = dist_m(t['lat'], t['lon'], det['lat'], det['lon'])
            if d < best_d:
                best, best_d = idx, d
        return best

    def update(self, dets, now):
        """dets: list of dicts (label, class_id, conf, u, v, lat, lon, ...).

        Returns [(det, target), ...] for the de-duplicated detections.
        """
        unique = []
        for d in dets:
            if all(dist_m(u['lat'], u['lon'], d['lat'], d['lon']) >= self.dedup_tol
                   for u in unique):
                unique.append(d)

        taken, pairs = set(), []
        for d in unique:
            idx = self._best_match(d, taken)
            if idx is None:
                self.targets.append({
                    'lat': d['lat'], 'lon': d['lon'], 'n': 1,
                    'u': d['u'], 'v': d['v'],
                    'class_id': d['class_id'], 'conf': d['conf'],
                    'final': d['label'], 'current': d['label'], 'since': now,
                })
                idx = len(self.targets) - 1
            else:
                self._refine(self.targets[idx], d, now)
            taken.add(idx)
            pairs.append((d, self.targets[idx]))
        return pairs

    def _refine(self, t, d, now):
        # running mean (capped so the target can still follow slow drift)
        w = min(t['n'], self.max_samples) + 1
        t['lat'] += (d['lat'] - t['lat']) / w
        t['lon'] += (d['lon'] - t['lon']) / w
        t['n'] += 1
        t['u'], t['v'] = d['u'], d['v']
        t['class_id'], t['conf'] = d['class_id'], d['conf']

        if d['label'] == t['current']:
            return
        stable = now - t['since']
        if {t['current'], d['label']} == {BLUE, GREY}:
            self.log.info(f"[BLINK CHECK] {t['current']} -> {d['label']} | stable {stable:.2f}s")
            if self.blink_min <= stable <= self.blink_max:
                if t['final'] != BLINK:
                    t['final'] = BLINK
                    self.log.info('>>> BLINKING LIGHT CONFIRMED <<<')
            else:
                self.log.warning(f'Transition rejected: {stable:.2f}s outside '
                                 f'[{self.blink_min}, {self.blink_max}]')
        t['current'], t['since'] = d['label'], now

    def snapshot(self):
        """{final_label: [targets]} with same-class targets closer than dist_tol merged."""
        out = {}
        for t in self.targets:
            group = out.setdefault(t['final'], [])
            if all(dist_m(g['lat'], g['lon'], t['lat'], t['lon']) >= self.dist_tol
                   for g in group):
                group.append(t)
        return out

    def counts(self):
        return Counter({k: len(v) for k, v in self.snapshot().items()})

    def arena_complete(self):
        """Sanity check on the final map (blinking lights are blue/grey ones)."""
        c = self.counts()
        return (c['red_light'] >= 2 and c['green_light'] >= 2
                and c[GREY] + c[BLUE] + c[BLINK] >= 5)
