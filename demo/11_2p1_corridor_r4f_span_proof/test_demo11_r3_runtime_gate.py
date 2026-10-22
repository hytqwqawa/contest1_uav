#!/usr/bin/env python3
"""ROS-free validation of last-hop R3 guard; cannot publish flight commands."""
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
import time
import types
import unittest
import yaml

ROOT = Path(__file__).resolve().parent
SCRIPT = ROOT/'demo8_interface_overlay/scripts/demo11_runtime_corridor_gate.py'
spec = importlib.util.spec_from_file_location('demo11_runtime_corridor_gate', SCRIPT)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class Command:
    def __init__(self,x,y,z=0.8,yaw=0,mask=0):
        self.coordinate_frame=1
        self.type_mask=mask
        self.position=types.SimpleNamespace(x=x,y=y,z=z)
        self.yaw=yaw

class GateTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.session = Path(self.tmp.name)/'session.yaml'
        from demo5_config import load_config
        self.plan = load_config(ROOT/'demo5_waypoints.yaml')
        self.data = {
            'demo5_plan_snapshot':self.plan, 'demo_fence':self.plan['fence'],
            'origin_world':{'x':10.0,'y':20.0,'z':0},
            'forward_world':{'x':0.0,'y':1.0},
            'left_world':{'x':-1.0,'y':0.0},
        }
        self.session.write_text(yaml.safe_dump(self.data))
        self.g = module.R3CorridorGate(ROOT/'demo11_corridor.yaml',
                                      ROOT/'demo5_waypoints.yaml', self.session)
        self.g.bind_session()

    def at(self, f,l,yaw=0):
        return Command(10.0-l,20.0+f,yaw=yaw)

    def test_correct_rotated_frame_and_rejected_wide_bypass(self):
        self.g.check_target(self.at(1.8,0.0))
        for v in (-0.878,-0.904,-0.16,0.66):
            with self.subTest(v=v),self.assertRaises(module.GateError):
                self.g.check_target(self.at(1.94,v))

    def test_admissible_straight(self):
        for f,l in [(0,-.065),(.79,-.065),(.8,0.),(1.5,-.0499),(2,.5499),(3,0)]:
            self.g.check_target(self.at(f,l))

    def test_entry_rejects_out_of_range_lateral(self):
        self.g.check_target(self.at(.011,-.065))
        self.g.check_target(self.at(.799,-.904))
        for f in (.801,1.,2.):
            with self.subTest(f=f), self.assertRaises(module.GateError):
                self.g.check_target(self.at(f,-.16))

    def test_no_turn_wall_outside_guard_window(self):
        self.g.check_target(self.at(3.201,-1.5))
        with self.assertRaises(module.GateError):
            self.g.check_target(self.at(3.199,-1.5))

    def test_stale_session_file_refused(self):
        stale=module.R3CorridorGate(ROOT/'demo11_corridor.yaml',
                ROOT/'demo5_waypoints.yaml',self.session,started_at=time.time()+5)
        with self.assertRaisesRegex(module.GateError,'stale_session'):
            stale.bind_session()

    def test_snapshot_changed_refused(self):
        self.data['demo5_plan_snapshot']['waypoints'][0]['forward'] = 4.0
        self.session.write_text(yaml.safe_dump(self.data))
        not_bound=module.R3CorridorGate(ROOT/'demo11_corridor.yaml',
                                       ROOT/'demo5_waypoints.yaml',self.session)
        with self.assertRaisesRegex(module.GateError,'waypoints'):
            not_bound.bind_session()

    def test_inverted_axes_refused(self):
        self.data['left_world']['x'] = 1.0
        self.session.write_text(yaml.safe_dump(self.data))
        not_bound=module.R3CorridorGate(ROOT/'demo11_corridor.yaml',
                                       ROOT/'demo5_waypoints.yaml',self.session)
        with self.assertRaisesRegex(module.GateError,'inverted'):
            not_bound.bind_session()

    def test_nonfinite_command_rejected(self):
        for value in (float('nan'),float('inf')):
            with self.assertRaises(module.GateError):
                self.g.check_target(self.at(1,value))
        with self.assertRaises(module.GateError):
            self.g.check_target(Command(10,22,yaw=float('nan')))

    def test_ignore_xyz_yaw_refused(self):
        for mask in (1,2,4,1024):
            with self.assertRaises(module.GateError):
                self.g.check_target(Command(10,22,mask=mask))

    def test_bad_coordinate_frame_refused(self):
        c=self.at(2,0)
        c.coordinate_frame=7
        with self.assertRaises(module.GateError):self.g.check_target(c)

    def test_jump_caught_even_if_both_positions_inside(self):
        a,b=self.at(1,0),self.at(1.35,0)
        with self.assertRaisesRegex(module.GateError,'position_step'):
            self.g.check_continuity(a,b)

    def test_yaw_spike_after_replan_refused(self):
        a,b=self.at(2,0,0),self.at(2,0,math.radians(32))
        with self.assertRaisesRegex(module.GateError,'yaw_step'):
            self.g.check_continuity(a,b)

    def test_smooth_bypass_yaw_wrap(self):
        self.g.check_continuity(self.at(1.0,0,math.radians(179)),
                                self.at(1.01,0,math.radians(-179)))

    def test_flight_script_keeps_live_gate(self):
        for script in ('run_demo11.sh','run_demo9.sh','start_demo9.sh','start_demo5.sh'):
            s=(ROOT/script).read_text()
            self.assertIn('r4_flight_gate.sh',s)
        self.assertIn('exit 97',(ROOT/'r4_flight_gate.sh').read_text())

    def test_final_interface_has_abort_and_holding_path(self):
        src=(ROOT/'demo8_interface_overlay/scripts/trajectory_interface_demo8.py').read_text()
        self.assertIn('DEMO11_R3_FINAL_SETPOINT_CORRIDOR_GUARD',src)
        self.assertIn('self.r3_gate.check_target(r3_candidate)',src)
        self.assertIn('self.r3_gate.check_continuity',src)
        self.assertIn('self.r3_abort(str(exc))',src)
        self.assertIn('self.capture_hold()',src)
        self.assertIn('self.output_pub.publish(cmd)',src)
        self.assertIn('HOLD_R3_CORRIDOR_ABORT',src)

    def test_private_planner_overlay_not_runtime(self):
        text=(ROOT/'start_backend.sh').read_text()
        self.assertIn('demo11_p2_guard_ws',text)
        self.assertIn('PRIVATE_PLANNER_PACKAGE',text)
        self.assertIn('source "$R3_PRIVATE_WS/devel/setup.bash"',text)
        self.assertIn('rospack find ego_planner',text)

if __name__=='__main__':
    unittest.main(verbosity=2)
