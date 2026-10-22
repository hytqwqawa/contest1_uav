#!/usr/bin/env python3
"""Offline R4E corridor entrance proof, never starts ROS or MAVROS."""
from pathlib import Path
import hashlib, unittest
from demo11_corridor import load_corridor, CorridorViolation

ROOT=Path(__file__).resolve().parent

class EntryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c=load_corridor(ROOT/'demo11_corridor.yaml')

    def test_physical_entrance_and_finite_body(self):
        self.assertEqual((self.c.f_start,self.c.f_end),(1.,3.))
        self.assertAlmostEqual(self.c.f_guard_min,.8)
        self.assertAlmostEqual(self.c.f_guard_max,3.2)

    def test_original_launch_positions_allowed(self):
        for f,l in ((.011,-.065),(.001,-.060),(0.,-.904),(.799,-.904)):
            with self.subTest(f=f,l=l):self.c.check_center(f,l)

    def test_entrance_margin_is_preserved(self):
        for f in (.8001,1.,1.94,3.,3.2):
            for l in (-.16,-.904,+.651):
                with self.subTest(f=f,l=l),self.assertRaises(CorridorViolation):
                    self.c.check_center(f,l)

    def test_valid_centers_enter(self):
        for f,l in ((.801,0.),(1.,-.05),(1.5,.05),(3.,+.55)):
            with self.subTest(f=f,l=l):self.c.check_center(f,l)

    def test_curve_after_end_open(self):
        self.c.check_center(3.201,-1.5)
        self.c.check_center(4.5,-1.5)

    def test_hidden_bypass_hull_always_rejected(self):
        with self.assertRaises(CorridorViolation):
            self.c.certify_control_hulls([(0.,0.),(1.,-.6),(2.,-.6),(3.,0.)],
                                         [0,0,0,0,1,1,1,1],3)

    def test_hull_outside_before_entrance_permitted(self):
        self.c.certify_control_hulls([(0.,-.904),(.2,-.904),(.4,-.904),(.6,-.904)],
                                     [0,0,0,0,1,1,1,1],3)

    def test_runtime_interface_reuses_geometry(self):
        iface=(ROOT/'demo8_interface_overlay/scripts/trajectory_interface_demo8.py').read_text()
        gate=(ROOT/'demo8_interface_overlay/scripts/demo11_runtime_corridor_gate.py').read_text()
        plan=(ROOT/'uav2_demo5_waypoints.py').read_text()
        assert "self.r3_gate.check_xy(p.x, p.y, 'actual_before_enable')" in iface
        assert 'self.r3_gate.check_target(r3_candidate)' in iface
        assert 'self.corridor.check_center(f, l, label)' in gate
        assert 'self.r2_corridor.certify_control_hulls' in plan

if __name__=='__main__':unittest.main(verbosity=2)
