#!/usr/bin/env python3
import json, hashlib, unittest, ast, re
from pathlib import Path
ROOT=Path(__file__).resolve().parent
class R4HTests(unittest.TestCase):
    def test_manifest(self):
        data=json.loads((ROOT/'DEMO11_SOURCE_MANIFEST.json').read_text())['source_sha256']
        self.assertEqual(len(data),182)
        for rel,h in data.items():
            with self.subTest(file=rel):
                self.assertEqual(hashlib.sha256((ROOT/rel).read_bytes()).hexdigest(),h)
    def test_slow_yaw_both_launches(self):
        for name in ('uav2_demo5_multi_waypoint_interface.launch',
          'ros_src/ego_uav_interface/launch/uav2_obstacle_hover_test.launch'):
            txt=(ROOT/name).read_text()
            self.assertIn('name="yaw_rate_limit" value="0.35"',txt)
            self.assertIn('name="max_yaw_lead" value="0.18"',txt)
            self.assertNotIn('name="yaw_rate_limit" value="0.80"',txt)
    def test_state_monitor_not_relaxed(self):
        txt=(ROOT/'uav2_core_bringup_dev.launch').read_text()
        self.assertIn('name="yaw_error_max_deg"',txt)
        self.assertIn('value="5.0"',txt)
        self.assertNotIn('ready_loss_time',txt)
        src=(ROOT/'ros_src/ego_uav_interface/src/state_monitor_node.cpp').read_text()
        self.assertIn('ready_loss_time_',src)
        self.assertIn('0.40',src)
    def test_fail_closed_restore(self):
        code=(ROOT/'uav2_demo5_waypoints.py').read_text()
        ast.parse(code)
        self.assertIn("not self.flight_healthy or not self.mission_allowed",code)
        self.assertIn('settle_until = time.monotonic() + 1.0',code)
        self.assertIn('safe_to_rejoin = (',code)
        self.assertIn('if safe_to_rejoin:',code)
        self.assertIn('no second POINT rejoin',code)
        self.assertIn('R4H POINT HOLD stream/state lost',code)
    def test_frozen_geometry_and_planner(self):
        c=(ROOT/'demo11_corridor.yaml').read_text()
        self.assertIn('tracking_allowance: 0.15',c)
        self.assertIn('reserve: 0.0',c)
        v=(ROOT/'demo11_verify.py').read_text()
        self.assertIn("{'forward':3.5, 'left':0.25",v)
        self.assertIn("{'forward':4.061, 'left':-0.439",v)
        self.assertIn("{'forward':4.5, 'left':-1.5",v)
        for name in ('run_demo9.sh','start_demo9.sh','start_demo5.sh','run_demo11.sh'):
            self.assertIn('r4_flight_gate.sh',(ROOT/name).read_text())
    def test_no_arm_software(self):
        for rel in ('run_r4_flight.sh','r4_flight_gate.py','r4_flight_gate.sh'):
            txt=(ROOT/rel).read_text()
            self.assertNotIn('CommandBool',txt)
            self.assertNotIn('mavros/cmd/arming',txt)
if __name__=='__main__':unittest.main(verbosity=2)
