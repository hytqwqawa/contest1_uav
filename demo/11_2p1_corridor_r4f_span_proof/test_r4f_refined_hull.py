#!/usr/bin/env python3
"""R4F proof-only patch: pure Python, ROS-free, no actuator controls."""
import unittest,random,math
from pathlib import Path
from demo11_corridor import load_corridor,CorridorViolation
C=load_corridor(Path(__file__).resolve().parent/'demo11_corridor.yaml')
U=[0,0,0,0,1,1,1,1]

def sample_curve(cp,t):
    return C._deboor_span_point(cp,U,3,3,t)

class SpanProofTests(unittest.TestCase):
    def test_inherited_physical_geometry(self):
        for a,b in zip((C.f_guard_min,C.f_guard_max,C.center_l_min,C.center_l_max),(.8,3.2,-.15,.65)):
            self.assertAlmostEqual(a,b)
    def test_old_coarse_hull_would_reject_but_actual_curve_safe(self):
        # negative control points belong to F>3.2 turn area, not the wall.
        pts=[(3.02,.0),(3.07,-.01),(3.35,-.20),(3.612,-.236)]
        assert min(v for _,v in pts)<C.center_l_min
        assert min(v for f,v in pts if f<=3.2)>C.center_l_min
        C.certify_control_hulls(pts,U,3)
        for i in range(1,1000):
            f,l=sample_curve(pts,i/1000)
            C.check_center(f,l,'old_hull_false_positive')
    def test_same_hull_with_real_unsafe_turn_is_rejected(self):
        pts=[(3.02,.0),(3.07,-.29),(3.25,-.35),(3.612,-.236)]
        # Explicit counterexample: even if a few coarse samples look safe,
        # no unsafe continuous curve may be admitted by this proof.
        self.assertTrue(any(C.f_guard_min<=sample_curve(pts,t/300)[0]<=C.f_guard_max
              and sample_curve(pts,t/300)[1]<C.center_l_min
              for t in range(1,300)))
        with self.assertRaises(CorridorViolation): C.certify_control_hulls(pts,U,3)
    def test_known_wide_bypass_remains_rejected(self):
        with self.assertRaises(CorridorViolation):
            C.certify_control_hulls([(0.,0.),(1.,-.6),(2.,-.6),(3.,0.)],U,3)
    def test_curve_outside_wall_only(self):
        C.certify_control_hulls([(3.4,-1.),(3.6,-1.1),(3.8,-1.3),(4.5,-1.5)],U,3)
    def test_midpoint_eval_matches_bernstein_cubic(self):
        pts=[(3.02,0.),(3.07,-.01),(3.35,-.20),(3.612,-.236)]
        for t in (.001,.123,.5,.987,.999):
            q=sample_curve(pts,t)
            e=tuple((1-t)**3*pts[0][k]+3*(1-t)**2*t*pts[1][k]
                    +3*(1-t)*t*t*pts[2][k]+t**3*pts[3][k] for k in (0,1))
            for x,y in zip(q,e):self.assertAlmostEqual(x,y,places=11)
    def test_nonuniform_multispan_deboor_independent_basis(self):
        # Includes a repeated interior knot, which changes continuity.
        pts=[(3.02,0.),(3.07,-.01),(3.18,-.05),(3.30,-.09),
             (3.52,-.19),(3.70,-.27),(3.90,-.35),(4.1,-.48)]
        knots=[0.,0.,0.,0.,.2,.4,.4,.8,1.,1.,1.,1.]
        p=3
        def basis(j,d,t):
            if d==0:
                return float(knots[j] <= t < knots[j+1])
            a=b=0.0
            da=knots[j+d]-knots[j]
            db=knots[j+d+1]-knots[j+1]
            if da>0:a=(t-knots[j])/da*basis(j,d-1,t)
            if db>0:b=(knots[j+d+1]-t)/db*basis(j+1,d-1,t)
            return a+b
        for span in range(p,len(pts)):
            if knots[span+1]==knots[span]:continue
            for z in range(1,30):
                t=knots[span]+(knots[span+1]-knots[span])*z/30
                got=C._deboor_span_point(pts,knots,p,span,t)
                exp=[sum(pts[k][d]*basis(k,p,t) for k in range(len(pts))) for d in (0,1)]
                self.assertAlmostEqual(got[0],exp[0],places=10)
                self.assertAlmostEqual(got[1],exp[1],places=10)
        # If the refined certificate accepts any span, every dense sample
        # across ALL knot spans must satisfy the original R4E center rule.
        try:C.certify_control_hulls(pts,knots,p)
        except CorridorViolation:pass
        else:
            for span in range(p,len(pts)):
                if knots[span+1]==knots[span]:continue
                for z in range(1,200):
                    t=knots[span]+(knots[span+1]-knots[span])*z/200
                    f,l=C._deboor_span_point(pts,knots,p,span,t)
                    C.check_center(f,l)

    def test_derivative_enclosure_randomized_fails_closed(self):
        rng=random.Random(1149)
        for n in range(130):
            pts=[(rng.uniform(2.65,3.65),rng.uniform(-.5,.45)) for _ in range(4)]
            try:
                C.certify_control_hulls(pts,U,3)
            except CorridorViolation:
                continue
            for k in range(1,800):
                f,l=sample_curve(pts,k/800)
                if C.f_guard_min<=f<=C.f_guard_max:
                    self.assertGreaterEqual(l,C.center_l_min-1e-9,(n,k,pts,f,l))
                    self.assertLessEqual(l,C.center_l_max+1e-9,(n,k,pts,f,l))
    def test_degenerate_knots_fails_closed(self):
        with self.assertRaises(CorridorViolation):
            C.certify_control_hulls([(0.,0.)],[0.,1.],3)
    def test_downstream_mavros_safety_unmodified(self):
        p=Path(__file__).resolve().parent
        iface=(p/'demo8_interface_overlay/scripts/trajectory_interface_demo8.py').read_text()
        mission=(p/'uav2_demo5_waypoints.py').read_text()
        assert 'self.r3_gate.check_target(r3_candidate)' in iface
        assert 'self.r3_abort(str(exc))' in iface
        assert 'self.r2_corridor.certify_control_hulls' in mission
        assert 'HOLD_R3_CORRIDOR_ABORT' in iface

if __name__=='__main__': unittest.main(verbosity=2)
