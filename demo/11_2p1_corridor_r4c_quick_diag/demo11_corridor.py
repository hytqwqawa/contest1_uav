#!/usr/bin/env python3
"""Contest 2.1 Demo11 R2 review-only straight-corridor geometry audit.

Pure Python, with NO flight/ROS/mavros import and no actuator control.
The curved area is intentionally not constrained beyond the Demo5 fence.
This guard checks preview/replan paths; it is NOT a real-time collision shield.
"""
from dataclasses import dataclass
from pathlib import Path
import math
import yaml

class CorridorViolation(ValueError):
    pass

@dataclass(frozen=True)
class StraightCorridor:
    f_start: float
    f_end: float
    l_min: float
    l_max: float
    body_radius: float
    tracking_allowance: float
    reserve: float

    @property
    def clearance(self):
        return self.body_radius + self.tracking_allowance + self.reserve

    @property
    def center_l_min(self):
        return self.l_min + self.clearance

    @property
    def center_l_max(self):
        return self.l_max - self.clearance

    @property
    def f_guard_min(self):
        return self.f_start - self.body_radius

    @property
    def f_guard_max(self):
        return self.f_end + self.body_radius

    def _near_wall(self, f):
        return self.f_guard_min <= f <= self.f_guard_max

    def check_center(self, f, l, label='center'):
        if not (math.isfinite(float(f)) and math.isfinite(float(l))):
            raise CorridorViolation('%s non-finite coordinate' % label)
        if self._near_wall(f) and not (self.center_l_min <= l <= self.center_l_max):
            raise CorridorViolation(
                '%s violates straight corridor at F%+.3f L%+.3f; '
                'required L[%+.3f,%+.3f] while F[%+.3f,%+.3f]' %
                (label, f, l, self.center_l_min, self.center_l_max,
                 self.f_guard_min, self.f_guard_max))

    def check_path(self, xy, label='sampled B-spline'):
        if not xy:
            raise CorridorViolation('%s empty path' % label)
        for idx, (f, l) in enumerate(xy):
            self.check_center(float(f), float(l), '%s[%d]' % (label, idx))

    def certify_control_hulls(self, control_xy, knots, degree):
        """Conservative B-spline convex-hull check over each nonempty knot span.

        For span i, its curve is inside the convex hull of control points
        [i-degree, i]. Reject spans whose bounding box could touch the straight
        wall guard zone but whose control-point L extent exceeds safe bounds.
        May reject some valid curves; NEVER deforms or clips a planned curve.
        """
        cp = [(float(f), float(l)) for f, l in control_xy]
        knots = [float(k) for k in knots]
        degree = int(degree)
        n = len(cp) - 1
        if (degree < 1 or n < degree or len(knots) != n + degree + 2 or
                not all(math.isfinite(k) for k in knots) or
                any(knots[i] > knots[i+1] for i in range(len(knots)-1)) or
                any(not (math.isfinite(f) and math.isfinite(l)) for f, l in cp)):
            raise CorridorViolation('invalid B-spline knots/degree/control points')
        spans = 0
        for i in range(degree, n + 1):
            if knots[i+1] - knots[i] <= 1e-9:
                continue
            spans += 1
            affected = cp[i-degree:i+1]
            f_low = min(f for f, _ in affected)
            f_high = max(f for f, _ in affected)
            if f_low <= self.f_guard_max and f_high >= self.f_guard_min:
                l_low = min(l for _, l in affected)
                l_high = max(l for _, l in affected)
                if l_low < self.center_l_min or l_high > self.center_l_max:
                    raise CorridorViolation(
                        'B-spline control hull span %d intersects straight region; '
                        'F[%+.3f,%+.3f] L[%+.3f,%+.3f], '
                        'required L[%+.3f,%+.3f]' %
                        (i, f_low, f_high, l_low, l_high,
                         self.center_l_min, self.center_l_max))
        if not spans:
            raise CorridorViolation('B-spline contains no nonzero knot span')
        return spans


def load_corridor(path):
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict) or 'straight' not in raw:
        raise CorridorViolation('corridor config must contain straight: mapping')
    s = raw['straight']
    required = ('f_start', 'f_end', 'l_min', 'l_max', 'body_radius',
                'tracking_allowance', 'reserve')
    if not isinstance(s, dict) or set(s) != set(required):
        raise CorridorViolation('corridor config unexpected/missing keys')
    v = {}
    for k in required:
        x = s[k]
        if isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(float(x)):
            raise CorridorViolation('invalid corridor number '+k)
        v[k] = float(x)
    c = StraightCorridor(**v)
    if (c.f_end <= c.f_start or c.l_max <= c.l_min or
            c.body_radius <= 0 or c.tracking_allowance < 0 or c.reserve < 0 or
            c.center_l_min >= c.center_l_max):
        raise CorridorViolation('invalid corridor/clearance geometry')
    # Do not silently change agreed 1.5m corridor or assumed clearance.
    expected = {'f_start':0.,'f_end':3.,'l_min':-0.5,'l_max':1.,
                'body_radius':0.20,'tracking_allowance':0.15,'reserve':0.10}
    if any(abs(v[k] - required_value)>1e-9 for k, required_value in expected.items()):
        raise CorridorViolation('R2 reviewed corridor/clearance constants changed')
    return c
