#!/usr/bin/env python3
"""ROS-free test on staged candidate. Must NOT start a flight."""
import hashlib
import json
from pathlib import Path
import unittest
import sys
ROOT=Path(__file__).resolve().parent

class CandidateTests(unittest.TestCase):
    def test_hashes(self):
        files=json.loads((ROOT/'DEMO11_SOURCE_MANIFEST.json').read_text())['source_sha256']
        self.assertEqual(len(files), 155)
        for rel,digest in files.items():
            with self.subTest(rel=rel):
                self.assertEqual(hashlib.sha256((ROOT/rel).read_bytes()).hexdigest(),digest)
    def test_all_entry_gate(self):
        for name in ('run_demo11.sh','run_demo9.sh','start_demo9.sh','start_demo5.sh'):
            text=(ROOT/name).read_text()
            self.assertIn('r4_flight_gate.sh',text)
            self.assertNotIn('r2_flight_lock.sh',text)
    def test_module_mission_guard(self):
        x=(ROOT/'uav2_demo5_waypoints.py').read_text()
        self.assertIn('airborne_authorized(rospy)', x)
        self.assertNotIn('raise SystemExit(\'[R2 REVIEW',x)
    def test_real_hardware_params(self):
        x=(ROOT/'demo8_interface_overlay/launch/trajectory_interface_demo8.launch').read_text()
        self.assertEqual(x.count('11_2p1_corridor_r4_flight_candidate'),3)
        self.assertNotIn('11_2p1_corridor_r3_ground_review',x)
    def test_legacy_frozen(self):
        x=(ROOT/'demo11_corridor.yaml').read_text()
        self.assertIn('l_min:', x)
        self.assertIn('l_max:', x)
        self.assertTrue((ROOT/'r2_flight_lock.sh').is_file())
    def test_no_auto_arm(self):
        for name in ('r4_flight_gate.sh','r4_flight_gate.py','run_r4_flight.sh'):
            s=(ROOT/name).read_text()
            self.assertNotIn('CommandBool',s)
            self.assertNotIn('SetMode(',s)
    def test_binary_user_gate_requirements(self):
        x=(ROOT/'r4_flight_gate.py').read_text()
        for thing in ('--prepare','ROS_MASTER_URI','r4_corridor_status','PILOT_SIGNOFF','SITE_CLEAR'):
            # 'r3_corridor_status' stays in real R3 final interface on purpose.
            if thing == 'r4_corridor_status': thing='r3_corridor_status'
            self.assertIn(thing,x)

if __name__=='__main__':
    unittest.main(verbosity=2)
