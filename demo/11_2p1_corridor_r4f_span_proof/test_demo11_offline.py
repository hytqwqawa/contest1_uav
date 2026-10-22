#!/usr/bin/env python3
"""Demo11 geometry/YAML/guard regression; no ROS, ARM or actuator calls."""
import ast
import math
from pathlib import Path
import unittest
import yaml

from demo5_config import load_config
from demo8_terminal_yaw import TerminalYawGate, demo_yaw_to_world_deg
import demo11_verify

ROOT = Path(__file__).resolve().parent


class Demo11Tests(unittest.TestCase):
    def test_exact_three_ego_points(self):
        cfg = load_config(ROOT / 'demo5_waypoints.yaml')
        self.assertEqual(cfg['waypoints'], demo11_verify.ROUTE)
        self.assertEqual(cfg['flight_height'], 0.8)
        self.assertEqual(len(cfg['waypoints']), 3)
        self.assertTrue(all(p['mode'] == 'EGO' for p in cfg['waypoints']))

    def test_midpoint_on_quarter_circle(self):
        w = demo11_verify.ROUTE[1]
        c = (3.0, -1.5)
        radius = math.hypot(w['forward']-c[0], w['left']-c[1])
        self.assertAlmostEqual(radius, 1.5, delta=.001)
        f = 3.0 + 1.5/math.sqrt(2.0)
        l = -1.5 + 1.5/math.sqrt(2.0)
        self.assertAlmostEqual(w['forward'], f, delta=.001)
        self.assertAlmostEqual(w['left'], l, delta=.001)

    def test_yaw_commands_are_endpoint_observations(self):
        self.assertEqual([x['yaw_deg'] for x in demo11_verify.ROUTE],
                         [0., -135., -180.])
        self.assertAlmostEqual(demo_yaw_to_world_deg((1., 0.), -135), -135)
        gate = TerminalYawGate((4.061, -0.439, .8), -135.)
        odom_xyz = (4.061, -0.439, .8)
        vel = (0., 0., 0.)
        yaw = math.radians(-135.)
        self.assertFalse(gate.update(0., odom_xyz, vel, yaw, True)[0])
        self.assertTrue(gate.update(1.1, odom_xyz, vel, yaw, True)[0])
        self.assertFalse(gate.update(1.2, odom_xyz, vel, yaw, False)[0])

    def test_legacy_core_algorithm_and_posctl_intact(self):
        src = (ROOT/'uav2_demo5_waypoints.py').read_text()
        self.assertIn('self.execute_terminal_yaw(', src)
        self.assertIn('self.restore_point_hold()', src)
        self.assertIn('self.start_land()', src)
        handoff = (ROOT/'demo9_posctl_handoff.py').read_text()
        self.assertIn("custom_mode='POSCTL'", handoff)
        self.assertNotIn("custom_mode='AUTO.LAND'", handoff)
        self.assertNotIn('CommandBool', handoff)

    def test_wrappers_reject_old_routes(self):
        mission=(ROOT/'start_demo5.sh').read_text()
        self.assertIn('DEMO11_THREE_EGO_GUARD', mission)
        self.assertIn("'yaw_deg':-135.0", mission)
        self.assertIn('demo11_verify.py" --mux-only', mission)
        backend=(ROOT/'start_backend.sh').read_text()
        self.assertIn('V4_PKG="$DEMO/ros_src/ego_uav_interface"', backend)
        self.assertIn('ROS_PACKAGE_PATH="$DEMO/ros_src"', backend)
        self.assertIn('data/logs/demo11_2p1_corridor_r4f_span_proof', backend)
        run=(ROOT/'run_demo9.sh').read_text()
        self.assertIn('V4_MUX="$D/ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py"',run)
        self.assertIn('data/bags/demo11_2p1_corridor_r4f_span_proof',run)
        self.assertNotIn('/home/yundrone/contest1_uav/demo/10_demo9_v4_success', run)
        ground=(ROOT/'demo11_verify.py').read_text()
        self.assertIn("'grid_map/obstacles_inflation': 0.299", ground)
        for fn in ('run_demo10.sh','run_mission.sh','start_demo7.sh'):
            self.assertIn('exit 96', (ROOT/fn).read_text())

    def test_immutable_sha_manifest(self):
        demo11_verify.static_check(quiet=True)


if __name__ == '__main__':
    unittest.main(verbosity=2)
