#!/usr/bin/env python3
"""Offline FINAL_WAYPOINT hold regression with fake ROS clock. Never actuates."""
import ast
import math
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
source = (ROOT/'uav2_demo5_waypoints.py').read_text()
mod_ast = ast.parse(source)
class_ast = next(n for n in mod_ast.body if isinstance(n,ast.ClassDef) and n.name=='TangentPreview')
method_ast = next(n for n in class_ast.body if isinstance(n,ast.FunctionDef) and n.name=='demo9_verified_hold')
land_ast = next(n for n in class_ast.body if isinstance(n,ast.FunctionDef) and n.name=='start_land')

class FakeTime:
    t=0.0
    def monotonic(self): return self.t
class FakeRospy:
    def __init__(self,clock): self.clock=clock; self.logged=[]
    def is_shutdown(self): return False
    def sleep(self,seconds): self.clock.t+=seconds
    class Time:
        @staticmethod
        def now(): return FakeStamp()
    def loginfo(self,*args): self.logged.append(('info',args))
    def logwarn(self,*args): self.logged.append(('warn',args))
class FakeStamp:
    def __sub__(self,other): return SimpleNamespace(to_sec=lambda: 0.005)

def execute_fn(self,phase,seconds):
    clock=FakeTime(); ros=FakeRospy(clock)
    code=compile(ast.fix_missing_locations(ast.Module(body=[method_ast],type_ignores=[])),str(ROOT/'uav2_demo5_waypoints.py'),'exec')
    ns={'rospy':ros,'time':clock,'math':math}
    exec(code,ns)
    ns['demo9_verified_hold'](self,phase,seconds)
    return clock.t,ros

def fixture(height=.948,vz=.25,xy=.025,healthy=True):
    pos=SimpleNamespace(x=0.,y=0.,z=height)
    vel=SimpleNamespace(x=0.,y=0.,z=vz)
    cmd=SimpleNamespace(x=xy,y=0.,z=.8)
    return SimpleNamespace(
      odom=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=pos)),
                           twist=SimpleNamespace(twist=SimpleNamespace(linear=vel))),
      point_cmd=SimpleNamespace(position=cmd),
      state=SimpleNamespace(connected=True,armed=True,mode='OFFBOARD'),
      flight_healthy=healthy,mission_allowed=True,demo_safe=True,
      point_status='AIRBORNE_HOLD',mux_status='POINT_ACTIVE',
      odom_time=FakeStamp(),point_cmd_time=FakeStamp(),
      demo5_cfg={'flight_height':.8},oz=0.
    )

class TerminalHoldTests(unittest.TestCase):
    def test_final_call_is_one_second(self):
        calls=[n for n in ast.walk(land_ast) if isinstance(n,ast.Call) and
               isinstance(n.func,ast.Attribute) and n.func.attr=='demo9_verified_hold']
        self.assertEqual(len(calls),1)
        self.assertEqual(ast.literal_eval(calls[0].args[0]),'FINAL_WAYPOINT')
        self.assertEqual(ast.literal_eval(calls[0].args[1]),1.0)
    def test_measured_failure_sample_now_allowed(self):
        elapsed, ros=execute_fn(fixture(height=.948,vz=.007,xy=.025),'FINAL_WAYPOINT',1.)
        self.assertGreaterEqual(elapsed,1.0)
        self.assertLess(elapsed,1.1)
    def test_final_no_height_or_vertical_velocity_limit(self):
        elapsed,_=execute_fn(fixture(height=1.02,vz=.35,xy=.05),'FINAL_WAYPOINT',1.0)
        self.assertGreaterEqual(elapsed,1.)
    def test_final_xy_unchanged(self):
        with self.assertRaisesRegex(RuntimeError,'XY not stable'):
            execute_fn(fixture(height=.948,vz=.007,xy=.221),'FINAL_WAYPOINT',1.)
    def test_health_still_blocks(self):
        with self.assertRaisesRegex(RuntimeError,'health/RC state invalid'):
            execute_fn(fixture(healthy=False),'FINAL_WAYPOINT',1.)
    def test_invalid_mux_still_blocks(self):
        f=fixture();f.mux_status='EGO_ACTIVE'
        with self.assertRaisesRegex(RuntimeError,'health/RC state invalid'):
            execute_fn(f,'FINAL_WAYPOINT',1.)
    def test_nonfinite_still_blocks(self):
        with self.assertRaisesRegex(RuntimeError,'non-finite'):
            execute_fn(fixture(vz=float('nan')),'FINAL_WAYPOINT',1.)
    def test_takeoff_three_seconds_and_all_thresholds_retained(self):
        elapsed,_=execute_fn(fixture(height=.8,vz=.01,xy=.01),'AFTER_TAKEOFF',3.0)
        self.assertGreaterEqual(elapsed,3.)
        with self.assertRaisesRegex(RuntimeError,'height/VZ not stable'):
            execute_fn(fixture(height=.95,vz=.01),'AFTER_TAKEOFF',3.0)
        with self.assertRaisesRegex(RuntimeError,'height/VZ not stable'):
            execute_fn(fixture(height=.8,vz=.181),'AFTER_TAKEOFF',3.0)
    def test_posctl_handoff_no_second_height_vz_gate(self):
        code=(ROOT/'demo9_posctl_handoff.py').read_text()
        self.assertNotIn('if abs(dz-self.height) > .14 or abs(vel.z) > .18',code)
        self.assertIn('if errxy > .22:',code)
        self.assertIn("Final waypoint complete; 1s XY/health verified HOLD",code)
        self.assertIn("for key in ('healthy', 'mission', 'safe')",code)
        self.assertIn("if not self.fresh('rc', .30):",code)
        self.assertIn("self.require_airborne_hold()",code)
    def test_invalid_durations_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'Unexpected FINAL_WAYPOINT'):
            execute_fn(fixture(),'FINAL_WAYPOINT',3.0)
        with self.assertRaisesRegex(RuntimeError,'Unexpected hover duration'):
            execute_fn(fixture(),'AFTER_TAKEOFF',1.0)

if __name__=='__main__':unittest.main(verbosity=2)
