#!/usr/bin/env python3
"""ROS-free R4B source and no-regression tests.  Does not start flight."""
import hashlib
import json
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parent
class CandidateTests(unittest.TestCase):
    def test_hashes(self):
        files=json.loads((ROOT/'DEMO11_SOURCE_MANIFEST.json').read_text())['source_sha256']
        self.assertEqual(len(files),156)
        for rel,digest in files.items():
            with self.subTest(rel=rel):
                self.assertEqual(hashlib.sha256((ROOT/rel).read_bytes()).hexdigest(),digest)
    def test_all_entry_gated(self):
        for name in ('run_demo11.sh','run_demo9.sh','start_demo9.sh','start_demo5.sh'):
            self.assertIn('r4_flight_gate.sh',(ROOT/name).read_text())
    def test_no_rc_preflight_acceptance(self):
        gate=(ROOT/'r4_flight_gate.py').read_text()
        for removed in ('rc_switch_challenge(', 'PILOT_SIGNOFF', 'PROPS_REMOVED',
                        'rc_switch_observed', 'verify_approval(', "'--prepare'"):
            self.assertNotIn(removed,gate)
        self.assertFalse((ROOT/'prepare_r4_noprop.sh').exists())
    def test_runtime_rc_and_px4_fences_remain(self):
        gate=(ROOT/'r4_flight_gate.py').read_text()
        for required in ('rc.channels[6] > 1300', 'rc.channels[5] < 1700',
                         'flight_healthy', 'rc_mission_allowed','HOLD_RC_BLOCKED',
                         'airborne_authorized(', 'SESSION_READY','session_frame.yaml',
                         'grid_map/obstacles_inflation','DEMO11_P2_GUARD'):
            self.assertIn(required,gate)
        wp=(ROOT/'uav2_demo5_waypoints.py').read_text()
        self.assertIn('airborne_authorized(rospy)',wp)
        self.assertIn('wait_airborne_hold()',wp)
    def test_demo11_flight_session_and_launch_paths(self):
        launch=(ROOT/'demo8_interface_overlay/launch/trajectory_interface_demo8.launch').read_text()
        self.assertEqual(launch.count('11_2p1_corridor_r4d_entry_f1'),3)
        self.assertNotIn('11_2p1_corridor_r4_flight_candidate',launch)
        self.assertEqual((ROOT/'demo11_corridor.yaml').is_file(),True)
    def test_r4c_fast_diagnostics_not_gate_removal(self):
        point=(ROOT/'ros_src/ego_uav_interface/scripts/point_traj_node.py').read_text()
        gate=(ROOT/'r4_flight_gate.py').read_text()
        flight=(ROOT/'uav2_flight_ops_dev.py').read_text()
        assert "self.last_enable_error" in point
        assert "getattr(self, 'last_enable_error', 'unknown')" in point
        assert 'ThreadPoolExecutor' in gate
        assert 'rc_mission_allowed' in gate and 'offboard' in gate.lower()
        assert 'flight_healthy' in flight and 'self.state.armed' in flight
        assert 'if not self.ensure_trajectory_interface_enabled()' in point
    def test_does_not_software_arm(self):
        for name in ('r4_flight_gate.sh','r4_flight_gate.py','run_r4_flight.sh'):
            s=(ROOT/name).read_text()
            self.assertNotIn('CommandBool',s)
            self.assertNotIn('SetMode(',s)
            self.assertNotIn('mavros/cmd/arming',s)
if __name__=='__main__':unittest.main(verbosity=2)
