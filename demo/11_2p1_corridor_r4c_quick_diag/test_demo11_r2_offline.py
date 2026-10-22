#!/usr/bin/env python3
"""ROS-free fail-closed geometry and launcher checks for R2 review candidate."""
import pathlib
import unittest
from demo11_corridor import CorridorViolation, load_corridor

ROOT = pathlib.Path(__file__).resolve().parent

class CorridorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = load_corridor(ROOT/'demo11_corridor.yaml')

    def test_agreed_numeric_walls(self):
        self.assertEqual((self.c.f_start,self.c.f_end,self.c.l_min,self.c.l_max),
                         (0.,3.,-.5,1.))
        self.assertAlmostEqual(self.c.clearance,.45)
        self.assertAlmostEqual(self.c.center_l_min,-.05)
        self.assertAlmostEqual(self.c.center_l_max,.55)

    def test_observed_wide_bypass_rejected(self):
        with self.assertRaises(CorridorViolation):
            self.c.check_path([(0.,0.), (1.94,-.904), (3.,0.)])
        with self.assertRaises(CorridorViolation):
            self.c.check_path([(0.,0.), (1.9,-.878), (3.,0.)])

    def test_clear_path_allowed(self):
        self.c.check_path([(0.,0.),(1.5,.05),(3.,0.)])

    def test_unsafe_near_wall_guard_end_rejected(self):
        with self.assertRaises(CorridorViolation):
            self.c.check_center(3.15,-.9)
        self.c.check_center(3.201,-1.5)
        self.c.check_center(4.5,-1.5)

    def test_zero_margin_wall_collision_rejected(self):
        with self.assertRaises(CorridorViolation):
            self.c.check_center(1.,-.25)
        with self.assertRaises(CorridorViolation):
            self.c.check_center(1.,.85)

    def test_convex_hull_clear(self):
        pts=[(0.,0.),(1.,0.1),(2.,0.1),(3.,0.)]
        self.assertEqual(self.c.certify_control_hulls(pts,[0,0,0,0,1,1,1,1],3),1)

    def test_convex_hull_blocks_hidden_overshoot(self):
        pts=[(0.,0.),(1.,-.6),(2.,-.6),(3.,0.)]
        with self.assertRaises(CorridorViolation):
            self.c.certify_control_hulls(pts,[0,0,0,0,1,1,1,1],3)

    def test_curved_region_not_restricted_by_straight_wall(self):
        pts=[(3.3,-1.0),(3.7,-1.2),(4.,-1.4),(4.5,-1.5)]
        self.assertEqual(self.c.certify_control_hulls(pts,[0,0,0,0,1,1,1,1],3),1)

    def test_invalid_data_rejected(self):
        with self.assertRaises(CorridorViolation):
            self.c.check_center(float('nan'),0.)
        with self.assertRaises(CorridorViolation):
            self.c.certify_control_hulls([(0.,0.)],[0.,1.],3)

    def test_live_flight_entrypoints_locked(self):
        for name in ['run_demo11.sh','run_demo9.sh','start_demo9.sh','start_demo5.sh']:
            src=(ROOT/name).read_text()
            self.assertIn('r2_flight_lock.sh',src,name)
        self.assertIn('exit 97',(ROOT/'r2_flight_lock.sh').read_text())

    def test_unchanged_original_fence(self):
        cfg=(ROOT/'demo5_waypoints.yaml').read_text()
        self.assertIn('left_min: -2.0',cfg)
        self.assertIn('left_max: 3.0',cfg)

if __name__=='__main__':
    unittest.main(verbosity=2)
