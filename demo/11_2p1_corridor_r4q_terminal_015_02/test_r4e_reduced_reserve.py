#!/usr/bin/env python3
"""R4E centered-flight-envelope regression. Absolutely no ROS/flight I/O."""
import unittest
from pathlib import Path
from demo11_corridor import load_corridor, CorridorViolation
ROOT = Path(__file__).resolve().parent

class ReducedReserveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c=load_corridor(ROOT/'demo11_corridor.yaml')

    def test_walls_unchanged_and_finite_clearance_retained(self):
        c=self.c
        self.assertEqual((c.f_start,c.f_end,c.l_min,c.l_max),(1.,3.,-.5,1.))
        self.assertEqual((c.body_radius,c.tracking_allowance,c.reserve),(.2,.15,0.))
        self.assertAlmostEqual(c.clearance,.35)
        self.assertAlmostEqual(c.center_l_min,-.15)
        self.assertAlmostEqual(c.center_l_max,.65)
        self.assertAlmostEqual(c.f_guard_min,.8)
        self.assertAlmostEqual(c.f_guard_max,3.2)

    def test_measured_p1_yaw_positions_now_accepted(self):
        # Previous HARD ABORT while yaw target fixed near F3.178/L-0.050.
        for f,l in [(2.995,-.017),(3.098,-.038),(3.161,-.047),
                    (3.178,-.0501),(3.178,-.07),(3.2,-.07)]:
            with self.subTest(f=f,l=l):self.c.check_center(f,l)

    def test_near_wall_real_risk_still_rejected(self):
        for f,l in [(.801,-.151),(1.,-.25),(1.94,-.904),(3.178,-.16),
                    (3.,+.651),(3.199,+.8)]:
            with self.subTest(f=f,l=l),self.assertRaises(CorridorViolation):
                self.c.check_center(f,l)

    def test_center_free_before_entry_and_after_curve(self):
        for f,l in [(0.,-.9),(.799,-.9),(3.201,-1.5),(4.5,-1.5)]:
            self.c.check_center(f,l)

    def test_planner_control_hull_uses_same_clearance(self):
        knots=[0,0,0,0,1,1,1,1]
        c=self.c
        c.certify_control_hulls([(0.,0.),(1.,-.08),(2.,-.08),(3.,0.)],knots,3)
        with self.assertRaises(CorridorViolation):
            c.certify_control_hulls([(0.,0.),(1.,-.2),(2.,-.2),(3.,0.)],knots,3)

    def test_downstream_gate_reuses_same_file_and_pilot_abort(self):
        last=(ROOT/'demo8_interface_overlay/scripts/trajectory_interface_demo8.py').read_text()
        monitor=(ROOT/'demo8_interface_overlay/scripts/demo11_runtime_corridor_gate.py').read_text()
        mission=(ROOT/'uav2_demo5_waypoints.py').read_text()
        assert 'load_corridor(corridor_file)' in monitor
        assert 'self.corridor.check_center(f, l, label)' in monitor
        assert 'self.r3_gate.check_target(r3_candidate)' in last
        assert "self.r3_gate.check_xy(p.x, p.y, 'actual_aircraft')" in last
        assert 'self.r3_abort(str(exc))' in last
        assert 'self.r2_corridor.certify_control_hulls' in mission
        assert 'HOLD_R3_CORRIDOR_ABORT' in last

if __name__=='__main__':unittest.main(verbosity=2)
