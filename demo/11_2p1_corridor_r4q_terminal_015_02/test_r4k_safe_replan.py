#!/usr/bin/env python3
"""R4O supersedes R4K automatic replanning; hold-only regression. No ROS/PX4."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

R=Path(__file__).resolve().parent
SRC=(R/'uav2_demo5_waypoints.py').read_text()
TREE=ast.parse(SRC)
CL=next(n for n in TREE.body if isinstance(n,ast.ClassDef) and n.name=='TangentPreview')

def method(name):
    m=next(n for n in CL.body if isinstance(n,ast.FunctionDef) and n.name==name)
    ns=ast.Module(body=[m],type_ignores=[])
    ast.fix_missing_locations(ns)
    code=compile(ns,str(R/'uav2_demo5_waypoints.py'),'exec')
    env=dict(rospy=ROS,Demo9EgoStartGap=Demo9EgoStartGap,
             Demo11RecoverableCorridorReject=Demo11RecoverableCorridorReject,
             Demo11RecoverableAltitudeReject=Demo11RecoverableAltitudeReject,
             RuntimeError=RuntimeError, String=lambda data:SimpleNamespace(data=data),
             PositionTarget=SimpleNamespace(IGNORE_PX=1,IGNORE_PY=2,IGNORE_PZ=4,
                                            IGNORE_YAW=8,FRAME_LOCAL_NED=1),
             math=math,wrap=lambda a:(a+math.pi)%(2*math.pi)-math.pi)
    exec(code,env)
    return env[name]

class Demo9EgoStartGap(RuntimeError): pass
class Demo11RecoverableCorridorReject(RuntimeError): pass
class Demo11RecoverableAltitudeReject(RuntimeError): pass
class Stamp:
    def __init__(self,v): self.v=v
    def __sub__(self,other): return SimpleNamespace(to_sec=lambda:self.v-other.v)
class Ros:
    def __init__(self): self.logged=[]; self.sleep_count=0
    def is_shutdown(self): return False
    def logwarn(self,*args): self.logged.append(('warn',args))
    def logerr(self,*args): self.logged.append(('err',args))
    def sleep(self,s): self.sleep_count+=1
    class Time:
        @staticmethod
        def now(): return Stamp(10.0)
ROS=Ros()
class Pub:
    def __init__(self): self.sent=[]
    def publish(self,msg): self.sent.append(msg.data)

class Runner:
    plan_execute_waypoint=None
    def __init__(self,attempts,verified=False,health=True):
        self.attempts=list(attempts)
        self.calls=[]; self.handoffs=0; self.hold_waits=0
        self._r4k_rejoin_verified=verified
        self.active_wp_pub=Pub(); self.yaw_mode_pub=Pub()
        self.health=health
    def _r4k_require_point_for_plan(self):
        if not self.health: raise RuntimeError('health failed: never retry')
        self.calls.append('check')
    def _demo9_plan_execute_once(self,index,total,wp):
        self.calls.append('plan')
        x=self.attempts.pop(0)
        if isinstance(x,Exception):
            if getattr(x,'already_handoff',False):
                self._r4k_rejoin_verified=True
            raise x
        return x
    def restore_point_hold(self):
        self.handoffs+=1; self._r4k_rejoin_verified=True
    def _demo9_wait_gap_replan_hold(self,timeout_s):
        self.hold_waits+=1

Runner.plan_execute_waypoint=method('plan_execute_waypoint')

class Tests(unittest.TestCase):
    def setUp(self): ROS.logged=[];ROS.sleep_count=0
    def test_single_success_attempt(self):
        r=Runner(['DONE'])
        self.assertEqual(r.plan_execute_waypoint(1,3,{}),'DONE')
        self.assertEqual(r.calls.count('plan'),1)
    def test_preview_corridor_refused_once_then_hold(self):
        r=Runner([Demo11RecoverableCorridorReject('bad'),'NEVER'])
        with self.assertRaisesRegex(RuntimeError,'one-plan attempt rejected'):
            r.plan_execute_waypoint(1,3,{})
        self.assertEqual(r.calls.count('plan'),1)
        self.assertEqual(r.handoffs,1)
        self.assertEqual(r.hold_waits,0)
    def test_gap_refused_once(self):
        r=Runner([Demo9EgoStartGap('gap'),'NEVER'])
        with self.assertRaisesRegex(RuntimeError,'one-plan attempt rejected'):
            r.plan_execute_waypoint(2,3,{})
        self.assertEqual(r.calls.count('plan'),1)
    def test_handoff_already_verified_not_repeated(self):
        e=Demo11RecoverableCorridorReject('bad');e.already_handoff=True
        r=Runner([e,'NEVER'])
        with self.assertRaisesRegex(RuntimeError,'one-plan attempt rejected'):
            r.plan_execute_waypoint(2,3,{})
        self.assertEqual(r.handoffs,0)
    def test_unhealthy_vehicle_does_not_plan(self):
        r=Runner(['NEVER'],health=False)
        with self.assertRaisesRegex(RuntimeError,'health failed'):
            r.plan_execute_waypoint(1,3,{})
        self.assertEqual(r.calls.count('plan'),0)
    def test_handoff_failure_fails_closed(self):
        r=Runner([Demo11RecoverableCorridorReject('bad')])
        def broken():raise RuntimeError('POINT HOLD not verified')
        r.restore_point_hold=broken
        with self.assertRaisesRegex(RuntimeError,'POINT HOLD not verified'):
            r.plan_execute_waypoint(1,3,{})
        self.assertEqual(r.calls.count('plan'),1)
    def test_inflight_failure_still_calls_safety_handoff(self):
        body=next(n for n in CL.body if isinstance(n,ast.FunctionDef) and
                  n.name=='_demo9_plan_execute_once')
        code=ast.get_source_segment(SRC,body)
        self.assertIn('handoff_verified = False',code)
        self.assertIn('self.restore_point_hold()',code)
        self.assertIn('if safe_to_rejoin:',code)
    def test_guard_fences_and_commands_retained(self):
        self.assertIn('self.r2_corridor.check_path',SRC)
        self.assertIn('self.r2_corridor.certify_control_hulls',SRC)
        self.assertIn('self.call_service(\n            "/contest1/ego_uav_interface/enable_trajectory"',SRC)
        self.assertNotIn('mavros/cmd/arming',SRC)

if __name__=='__main__':
    unittest.main(verbosity=2)
