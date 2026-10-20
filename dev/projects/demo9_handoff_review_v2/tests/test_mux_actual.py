"""Run the user's actual V2 MUX candidate with an isolated fake rospy transport."""
import math
import sys
import types
import unittest
from pathlib import Path

class Stamp:
    clock = 100.0
    def __init__(self, t): self.t=float(t)
    @classmethod
    def now(cls): return Stamp(cls.clock)
    def __sub__(self, other): return Stamp(self.t-other.t)
    def __add__(self, other): return Stamp(self.t+other.t)
    def to_sec(self): return self.t
    def __le__(self, other): return self.t<=other.t

class Publisher:
    def __init__(self,*a,**k): self.messages=[]
    def publish(self, message): self.messages.append(message)
class PositionTarget:
    def __init__(self,x=0,y=0,z=0,yaw=0,mask=0,frame=1):
        self.position=types.SimpleNamespace(x=x,y=y,z=z)
        self.yaw=math.radians(yaw)
        self.type_mask=mask
        self.coordinate_frame=frame
class String:
    def __init__(self,data=''):self.data=data
class Response:
    def __init__(self,success,message): self.success=success; self.message=message

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'candidates'))
ros=types.ModuleType('rospy')
ros.Time=Stamp
ros.Duration=lambda secs: Stamp(secs)
ros.Publisher=Publisher
ros.Subscriber=lambda *a,**k: None
ros.Service=lambda *a,**k: None
ros.Timer=lambda *a,**k: None
ros.logerr=lambda *a,**k: None
ros.logerr_throttle=lambda *a,**k: None
sys.modules['rospy']=ros
for pkg,attr,cls in [('mavros_msgs.msg','PositionTarget',PositionTarget),
                     ('std_msgs.msg','String',String),
                     ('std_srvs.srv','TriggerResponse',Response)]:
    group,name=pkg.split('.')
    sys.modules[group]=types.ModuleType(group)
    module=types.ModuleType(pkg)
    setattr(module,attr,cls)
    if pkg=='std_srvs.srv': module.Trigger=object
    sys.modules[pkg]=module
ns={'__name__':'offline_mux_v2'}
path=Path(__file__).resolve().parents[1]/'candidates/trajectory_mux.py'
exec(compile(path.read_text(),str(path),'exec'),ns)
Mux=ns['TrajectoryMux']

class ActualMuxTests(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.m=Mux()
    def pt(self,msg):
        self.m.point_cb(msg)
        self.m.timer_cb(None)
    def test_old_incident_point_veto_no_source_change(self):
        self.pt(PositionTarget(-4.77,4.47,.72,93))
        ego=PositionTarget(-4.77622,4.47158,.71929,93.56,2048)
        self.m.ego_cb(ego)
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.m.point_cb(PositionTarget(-4.7198,3.518,.701,-175.01))
        n=len(self.m.pub.messages)
        resp=self.m.use_point(None)
        self.assertFalse(resp.success)
        self.assertEqual(self.m.source,'EGO')
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n+1)
        self.assertAlmostEqual(self.m.pub.messages[-1].yaw,ego.yaw)
    def test_valid_refreshed_point(self):
        self.pt(PositionTarget(-4.77,4.47,.72,93))
        self.m.ego_cb(PositionTarget(-4.776,4.471,.719,93.56,2048))
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.m.point_cb(PositionTarget(-4.777,4.472,.72,93.0))
        self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_ACTIVE')
    def test_invalid_post_handoff_frame_is_rejected(self):
        self.pt(PositionTarget(0,0,1,0))
        self.m.ego_cb(PositionTarget(0.01,0,1,0,2048))
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
        self.m.point_cb(PositionTarget(0,0,1,1))
        self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        n=len(self.m.pub.messages)
        self.m.point_cb(PositionTarget(0,0,1,1,frame=2))
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_GUARD_REJECTED')
    def test_post_handoff_yaw_spike_is_not_published(self):
        self.pt(PositionTarget(0,0,1,0))
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
    def test_no_clamp_inside_normal_point_takeoff(self):
        self.pt(PositionTarget(0,0,0,179))
        before=len(self.m.pub.messages)
        self.pt(PositionTarget(0,0,1.692,0))
        self.assertEqual(len(self.m.pub.messages),before+1)
    def test_no_future_timestamp_is_fresh(self):
        self.pt(PositionTarget(0,0,1,0))
        self.m.point_time=Stamp(101)
        self.assertFalse(self.m.fresh(self.m.point_time))
    def test_invalid_candidate_cannot_be_selected(self):
        self.pt(PositionTarget(0,0,1,0))
        self.m.ego_cb(PositionTarget(0,0,1,float('nan')))
        r=self.m.use_ego(None)
        self.assertFalse(r.success)
        self.assertEqual(self.m.source,'POINT')
    def test_ordinary_source_switch(self):
        self.pt(PositionTarget(-1.665,3.742,.772,-177.39))
        self.m.ego_cb(PositionTarget(-1.606,3.798,.749,-177.30,2048))
        self.assertTrue(self.m.use_ego(None).success)

if __name__=='__main__': unittest.main()
