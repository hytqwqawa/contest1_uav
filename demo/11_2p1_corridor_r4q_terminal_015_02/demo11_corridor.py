#!/usr/bin/env python3
"""Contest 2.1 Demo11 R2 review-only straight-corridor geometry audit.

Pure Python, with NO flight/ROS/mavros import and no actuator control.
The curved area is intentionally not constrained beyond the Demo5 fence.
This guard checks preview/replan paths; it is NOT a real-time collision shield.
R4F: refine coarse hull only via conservative derivative bounds, fail closed.
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
    lmin_tracking_allowance: float
    reserve: float

    @property
    def clearance(self):
        return self.body_radius + self.tracking_allowance + self.reserve

    @property
    def center_l_min(self):
        # Right (negative L): user-defined open-area reference, not a measured wall.
        return self.l_min + (self.body_radius + self.lmin_tracking_allowance + self.reserve)

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

    @staticmethod
    def _deboor_span_point(control, knots, degree, span, t):
        """de Boor position strictly inside one positive knot span."""
        p = degree
        d = [list(control[span-p+j]) for j in range(p+1)]
        for r in range(1, p+1):
            for j in range(p, r-1, -1):
                k = span-p+j
                den = knots[k+p-r+1]-knots[k]
                if den <= 0.0:
                    raise CorridorViolation('degenerate_deboor_knot')
                a = (t-knots[k])/den
                if not math.isfinite(a) or a < -1e-8 or a > 1.00000001:
                    raise CorridorViolation('invalid_deboor_weight')
                d[j][0] = (1.0-a)*d[j-1][0]+a*d[j][0]
                d[j][1] = (1.0-a)*d[j-1][1]+a*d[j][1]
        f, l = d[p]
        if not math.isfinite(f) or not math.isfinite(l):
            raise CorridorViolation('invalid_deboor_evaluation')
        return f, l

    def _certify_span_by_derivative_enclosure(self, cp, knots, degree, span):
        """Fail-closed enclosure of an actual B-spline knot span.

        C'(u) is a degree-(p-1) B-spline whose control points are
        D_j=p*(P_{j+1}-P_j)/(U_{j+p+1}-U_{j+1}). The derivative basis is
        nonnegative and a partition of unity, so componentwise |C'|
        is bounded by max |D_j| for the active controls on this span.
        At any subinterval's midpoint, the whole subcurve lies within
        C(mid) +/- max|C'| * width/2 (plus outward numeric rounding pad).
        Subdivide only ambiguous intervals; NEVER accept by sampling alone.
        """
        a, b = knots[span], knots[span+1]
        p = degree
        vf = vl = 0.0
        for j in range(span-p, span):
            den = knots[j+p+1]-knots[j+1]
            if den <= 0.0 or not math.isfinite(den):
                raise CorridorViolation('degenerate_derivative_knot')
            scale = p/den
            vf = max(vf, abs(scale*(cp[j+1][0]-cp[j][0])))
            vl = max(vl, abs(scale*(cp[j+1][1]-cp[j][1])))
        if not all(math.isfinite(x) for x in (vf,vl)):
            raise CorridorViolation('nonfinite_derivative_bound')

        # Avoid accepting a barely tangent/grazing curve on floating-point
        # roundoff. The safety envelope is left EXACTLY as in R4E.
        pad = 1e-8
        max_depth = 15
        work_budget = 20000
        pending = [(a,b,0)]
        checked = 0
        while pending:
            lo,hi,depth=pending.pop()
            checked += 1
            if checked > work_budget:
                raise CorridorViolation('R4F refined span %d budget exhausted; fail closed' % span)
            mid=lo+(hi-lo)*0.5
            if not (lo<mid<hi):
                raise CorridorViolation('R4F refined span %d precision exhausted' % span)
            f,l=self._deboor_span_point(cp,knots,p,span,mid)
            df=vf*(hi-lo)*0.5+pad
            dl=vl*(hi-lo)*0.5+pad
            low_f,high_f=f-df,f+df
            low_l,high_l=l-dl,l+dl
            if high_f < self.f_guard_min or low_f > self.f_guard_max:
                continue
            if low_l >= self.center_l_min and high_l <= self.center_l_max:
                continue
            if depth >= max_depth:
                raise CorridorViolation(
                    'B-spline refined proof FAILED span %d: t[%.7f,%.7f] '
                    'F[%+.4f,%+.4f] L[%+.4f,%+.4f] may violate '
                    'center L[%+.3f,%+.3f] while F[%+.3f,%+.3f]' %
                    (span,lo,hi,low_f,high_f,low_l,high_l,
                     self.center_l_min,self.center_l_max,
                     self.f_guard_min,self.f_guard_max))
            pending.append((mid,hi,depth+1))
            pending.append((lo,mid,depth+1))

    def certify_control_hulls(self, control_xy, knots, degree):
        """Conservative B-spline convex-hull check over each nonempty knot span.

        For span i, its curve is inside the convex hull of control points
        [i-degree, i]. Reject spans whose bounding box could touch the straight
        wall guard zone but whose control-point L extent exceeds safe bounds.
        Ambiguous control hulls receive a derivative-enclosed span proof;
        if still ambiguous, FAIL CLOSED. Never edits a planned curve.
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
                    # R4F: a whole B-spline control hull can straddle the
                    # corridor EXIT even when the actual curve crosses the
                    # L boundary only beyond F_guard_max. Prove the span
                    # safe with a conservative derivative enclosure; when
                    # this cannot be proved, reject (never approve on samples).
                    self._certify_span_by_derivative_enclosure(cp, knots, degree, i)
        if not spans:
            raise CorridorViolation('B-spline contains no nonzero knot span')
        return spans


def load_corridor(path):
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict) or 'straight' not in raw:
        raise CorridorViolation('corridor config must contain straight: mapping')
    s = raw['straight']
    required = ('f_start', 'f_end', 'l_min', 'l_max', 'body_radius',
                'tracking_allowance', 'lmin_tracking_allowance', 'reserve')
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
            c.body_radius <= 0 or c.tracking_allowance < 0 or
            c.lmin_tracking_allowance < 0 or c.reserve < 0 or
            c.center_l_min >= c.center_l_max):
        raise CorridorViolation('invalid corridor/clearance geometry')
    # Same F zone; symmetric center-line bounds +/-0.75 from open-area references.
    # Right tracking .10; left tracking .15; EGO obstacle inflation unchanged.
    expected = {'f_start':1.,'f_end':3.,'l_min':-1.05,'l_max':1.10,
                'body_radius':0.20,'tracking_allowance':0.15,
                'lmin_tracking_allowance':0.10,'reserve':0.0}
    if any(abs(v[k] - required_value)>1e-9 for k, required_value in expected.items()):
        raise CorridorViolation('R2 reviewed corridor/clearance constants changed')
    return c
