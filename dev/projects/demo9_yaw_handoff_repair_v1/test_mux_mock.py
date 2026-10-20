"""No-ROS mux integration tests: real patched method body, fake ROS transport."""
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from prepare_dev import patch_mux

class Stamp:
    clock=100.0
    def __init__(self,t): self.t=t
    @classmethod
    def now(cls): return Stamp(cls.clock)
    def __sub__(self,other): return Stamp(self.t-other.t)
    def __add__(self,other): return Stamp(self.t+other.t)
    def to_sec(self): return self.t
    def __le__(self,other): return self.t<=other.t

class Publisher:
    def __init__(self,*args,**kwargs): self.messages=[]
    def publish(self,msg): self.messages.append(msg)

class PositionTarget:
    def __init__(self,x=0,y=0,z=0,yaw=0,mask=0):
        self.position=types.SimpleNamespace(x=x,y=y,z=z)
        self.yaw=math.radians(yaw)
        self.type_mask=mask
        self.coordinate_frame=1

class String:
    def __init__(self,data=''):self.data=data

class Response:
    def __init__(self,success,message):self.success=success;self.message=message

fake_ros=types.ModuleType('rospy')
fake_ros.Time=Stamp
fake_ros.Duration=lambda t:Stamp(t)
fake_ros.Publisher=Publisher
fake_ros.Subscriber=lambda *a,**kw:None
fake_ros.Service=lambda *a,**kw:None
fake_ros.Timer=lambda *a,**kw:None
fake_ros.logerr=lambda *a,**kw:None
fake_ros.logerr_throttle=lambda *a,**kw:None
sys.modules['rospy']=fake_ros
msg_module=types.ModuleType('mavros_msgs.msg');msg_module.PositionTarget=PositionTarget
sys.modules['mavros_msgs']=types.ModuleType('mavros_msgs')
sys.modules['mavros_msgs.msg']=msg_module
s_module=types.ModuleType('std_msgs.msg');s_module.String=String
sys.modules['std_msgs']=types.ModuleType('std_msgs')
sys.modules['std_msgs.msg']=s_module
svc=types.ModuleType('std_srvs.srv')
svc.Trigger=object;svc.TriggerResponse=Response
sys.modules['std_srvs']=types.ModuleType('std_srvs')
sys.modules['std_srvs.srv']=svc

legacy=Path(__file__).parent/'fixtures/trajectory_mux_reference_20261015.py'
patched=patch_mux(legacy.read_text())
code=compile(patched,'<patched mux regression>','exec')
scope={'__name__':'regression_mux'}
exec(code,scope)
Mux=scope['TrajectoryMux']

class TestMux(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.m=Mux()
    def publish_point(self,msg):
        self.m.point_cb(msg)
        self.m.timer_cb(None)
    def test_reject_old_point_in_incident(self):
        old_point=PositionTarget(-4.719796658,3.517995596,.70103693,-175.01)
        ego=PositionTarget(-4.776221752,4.471584797,.719291449,93.56,2048)
        self.publish_point(PositionTarget(-4.77,4.47,.72,93.0))
        self.m.ego_cb(ego)
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.m.point_cb(old_point)
        n=len(self.m.pub.messages)
        resp=self.m.use_point(None)
        self.assertFalse(resp.success,resp.message)
        self.assertEqual(self.m.source,'EGO')
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n+1)
        self.assertAlmostEqual(self.m.pub.messages[-1].yaw,ego.yaw)
    def test_accept_refreshed_point(self):
        ego=PositionTarget(-4.776,4.471,.719,93.56,2048)
        self.publish_point(PositionTarget(-4.77,4.47,.72,93))
        self.m.ego_cb(ego)
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.m.point_cb(PositionTarget(-4.78,4.475,.72,93.0))
        self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_ACTIVE')
    def test_post_switch_bad_point_is_vetoed(self):
        self.publish_point(PositionTarget(0,0,1,0))
        self.m.ego_cb(PositionTarget(0.01,0,1,0,2048))
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.m.point_cb(PositionTarget(0,0,1,1))
        self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        n=len(self.m.pub.messages)
        self.m.point_cb(PositionTarget(0,0,1,92))
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_GUARD_REJECTED')
    def test_initial_point_takeoff_reset_not_blocked(self):
        self.publish_point(PositionTarget(0,0,0,179))
        n=len(self.m.pub.messages)
        self.publish_point(PositionTarget(0,0,1.692,0))
        self.assertEqual(len(self.m.pub.messages),n+1)
    def test_ordinary_normal_source_switch_is_allowed(self):
        self.publish_point(PositionTarget(-1.665,3.742,.772,-177.39))
        self.m.ego_cb(PositionTarget(-1.606,3.798,.749,-177.30,2048))
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.assertEqual(self.m.status_pub.messages[-1].data,'EGO_ACTIVE')

if __name__=='__main__':unittest.main(verbosity=2)
