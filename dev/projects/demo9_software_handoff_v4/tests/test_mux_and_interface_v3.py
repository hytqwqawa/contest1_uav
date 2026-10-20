"""Execute real V3 MUX and the uploaded runtime trajectory_interface in fake rospy.

This validates state logic and wiring contracts, not PX4/SITL/hardware timing.
"""
import math
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'candidates'))

class Stamp:
    clock = 100.0
    def __init__(self, seconds): self.s=float(seconds)
    @classmethod
    def now(cls): return cls(cls.clock)
    def __sub__(self, other): return Stamp(self.s-other.s)
    def __add__(self, other): return Stamp(self.s+other.s)
    def __lt__(self, other): return self.s < other.s
    def __le__(self, other): return self.s <= other.s
    def __ge__(self, other): return self.s >= other.s
    def __gt__(self, other): return self.s > other.s
    def to_sec(self): return self.s

class Publisher:
    def __init__(self, *args, **kw): self.messages=[]
    def publish(self, msg): self.messages.append(msg)
class PositionTarget:
    FRAME_LOCAL_NED=1
    IGNORE_PX=1;IGNORE_PY=2;IGNORE_PZ=4
    IGNORE_VX=8;IGNORE_VY=16;IGNORE_VZ=32
    IGNORE_AFX=64;IGNORE_AFY=128;IGNORE_AFZ=256
    IGNORE_YAW=1024;IGNORE_YAW_RATE=2048
    def __init__(self, x=0, y=0, z=0, yaw=0, mask=0, frame=1):
        self.header=types.SimpleNamespace(stamp=Stamp.now(),frame_id='map')
        self.position=types.SimpleNamespace(x=x,y=y,z=z)
        self.velocity=types.SimpleNamespace(x=0.0,y=0.0,z=0.0)
        self.acceleration_or_force=types.SimpleNamespace(x=0.0,y=0.0,z=0.0)
        self.yaw_rate=0.0
        self.yaw=math.radians(yaw)
        self.type_mask=mask
        self.coordinate_frame=frame
class String:
    def __init__(self,data=''):self.data=data
class Bool:
    def __init__(self,data=False):self.data=data
class Response:
    def __init__(self,success=False,message=''):self.success=success;self.message=message
class Odometry:
    def __init__(self,x=0,y=0,z=1):
        self.pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x,y=y,z=z),
            orientation=types.SimpleNamespace(x=0,y=0,z=0,w=1)))
        self.twist=types.SimpleNamespace(twist=types.SimpleNamespace(
            linear=types.SimpleNamespace(x=0,y=0,z=0)))

ros=types.ModuleType('rospy')
ros.Time=Stamp
ros.Duration=lambda n:Stamp(n)
ros.Publisher=Publisher
ros.Subscriber=lambda *a,**k:None
ros.Service=lambda *a,**k:None
ros.Timer=lambda *a,**k:None
ros.get_param=lambda name,default=None:default
ros.logerr=lambda *a,**k:None
ros.logerr_throttle=lambda *a,**k:None
ros.logwarn=lambda *a,**k:None
ros.logwarn_throttle=lambda *a,**k:None
ros.loginfo=lambda *a,**k:None
ros.loginfo_throttle=lambda *a,**k:None
sys.modules['rospy']=ros
for pkg, attrs in {
    'mavros_msgs.msg':{'PositionTarget':PositionTarget},
    'std_msgs.msg':{'String':String,'Bool':Bool},
    'std_srvs.srv':{'Trigger':object,'TriggerResponse':Response},
    'nav_msgs.msg':{'Odometry':Odometry},
    'tf.transformations':{'euler_from_quaternion':lambda q:(0,0,0)},
}.items():
    left,right=pkg.split('.')
    if left not in sys.modules:sys.modules[left]=types.ModuleType(left)
    mod=types.ModuleType(pkg)
    for k,v in attrs.items():setattr(mod,k,v)
    sys.modules[pkg]=mod

mux_ns={'__name__':'dev_mux_v3'}
mux_file=ROOT/'candidates/trajectory_mux.py'
exec(compile(mux_file.read_text(),str(mux_file),'exec'),mux_ns)
Mux=mux_ns['TrajectoryMux']
interface_ns={'__name__':'dev_iface_actual'}
interface_file=ROOT/'reference/trajectory_interface.py'
exec(compile(interface_file.read_text(),str(interface_file),'exec'),interface_ns)
Interface=interface_ns['TrajectoryInterface']

def cmd(x=0,y=0,z=1,yaw=93,mask=0):
    return PositionTarget(x,y,z,yaw,mask)

class MuxV3Tests(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.m=Mux()
        self.m.point_cb(cmd(0,0,1,93))
        self.m.point_status_cb(String('AIRBORNE_HOLD'))
        self.m.timer_cb(None)
        self.m.interface_cb(String('TRAJECTORY_ACTIVE'))
        self.m.ego_cb(cmd(.01,0,1,93.2,2048))
        self.assertTrue(self.m.use_ego(None).success)
        self.m.timer_cb(None)
    def in_hold(self,hold):
        self.m.interface_cb(String('HOLD_ACTIVE'))
        self.m.downstream_cb(hold)
    def test_ego_far_away_but_new_point_matches_real_hold(self):
        # EGO remains near 0, downstream is holding the actual vehicle at x=1.
        # A comparison against stale EGO would incorrectly reject this switch.
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1.01,1,1,93))
        res=self.m.use_point(None)
        self.assertTrue(res.success,res.message)
        self.m.timer_cb(None)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_ACTIVE')
    def test_old_point_yaw_rejected_against_downstream_hold(self):
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1,1,1,-175))
        result=self.m.use_point(None)
        self.assertFalse(result.success)
        self.assertIn('yaw_jump',result.message)
        self.assertEqual(self.m.source,'EGO')
    def test_old_point_position_rejected_against_downstream_hold(self):
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1,0,1,93))
        result=self.m.use_point(None)
        self.assertFalse(result.success)
        self.assertIn('position_jump',result.message)
    def test_stale_downstream_hold_never_switches(self):
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1,1,1,93))
        Stamp.clock+=.20
        self.m.interface_cb(String('HOLD_ACTIVE'))
        self.assertFalse(self.m.use_point(None).success)
    def test_interface_stale_never_switches(self):
        self.in_hold(cmd(1,1,1,93))
        Stamp.clock+=.31
        self.m.point_cb(cmd(1,1,1,93))
        self.assertFalse(self.m.use_point(None).success)
    def test_rejection_remains_after_250_ms(self):
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1,1,1,93))
        self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        published=len(self.m.pub.messages)
        # 90-degree error is repeatedly returned beyond original window.
        for i in range(30):
            Stamp.clock+=.02
            self.m.point_cb(cmd(1,1,1,-175))
            self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),published)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_GUARD_REJECTED')
        self.assertIsNotNone(self.m._guard_until)
    def test_transient_rejection_recovers_only_on_continuity(self):
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1,1,1,93))
        self.assertTrue(self.m.use_point(None).success)
        self.m.timer_cb(None)
        Stamp.clock+=.02
        self.m.point_cb(cmd(1,1,1,-175))
        self.m.timer_cb(None)
        rejected=len(self.m.pub.messages)
        for i in range(18):
            Stamp.clock+=.02
            self.m.point_cb(cmd(1.01,1,1,93))
            self.m.timer_cb(None)
        self.assertGreater(len(self.m.pub.messages),rejected)
        self.assertIsNone(self.m._guard_until)
    def test_point_hold_spike_long_after_switch_is_rejected(self):
        self.in_hold(cmd(1,1,1,93))
        self.m.point_cb(cmd(1,1,1,93))
        self.assertTrue(self.m.use_point(None).success)
        self.m.point_status_cb(String('AIRBORNE_HOLD'))
        for i in range(20):
            Stamp.clock += .02
            self.m.point_status_cb(String('AIRBORNE_HOLD'))
            self.m.point_cb(cmd(1,1,1,93))
            self.m.timer_cb(None)
        self.assertIsNone(self.m._guard_until)
        n=len(self.m.pub.messages)
        self.m.point_cb(cmd(1,1,1,-175))
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_GUARD_REJECTED')
        self.assertIsNotNone(self.m._guard_until)
    def test_ground_takeoff_setpoint_not_clamped(self):
        self.m.source='POINT'
        self.m._guard_until=None
        self.m.point_status_cb(String('HOLD_GROUND'))
        self.m.point_cb(cmd(1,1,0,93))
        self.m.timer_cb(None)
        n=len(self.m.pub.messages)
        self.m.point_cb(cmd(1,1,0.50,93))
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n+1)

    def test_stale_source_does_not_publish(self):
        n=len(self.m.pub.messages)
        Stamp.clock+=.30
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),n)
        self.assertEqual(self.m.status_pub.messages[-1].data,'EGO_STALE')
    def test_normal_switch_when_live_trajectory(self):
        self.m.point_cb(cmd(.02,0,1,93.3))
        result=self.m.use_point(None)
        self.assertTrue(result.success,result.message)

class RuntimeInterfaceTests(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.i=Interface()
        self.i.odom_callback(Odometry())
        self.i.health_callback(Bool(True))
        self.i.mission_permission_callback(Bool(True))
    def test_disabled_trajectory_publishes_independent_hold(self):
        self.i.timer_callback(None)
        self.assertEqual(len(self.i.output_pub.messages),1)
        self.assertEqual(self.i.last_source,'HOLD')
        self.assertFalse(self.i.trajectory_enabled)
        Stamp.clock+=1.60
        self.i.odom_callback(Odometry())
        self.i.health_callback(Bool(True))
        self.i.mission_permission_callback(Bool(True))
        self.i.timer_callback(None)
        self.assertEqual(self.i.status_pub.messages[-1].data,'HOLD_ACTIVE')
    def test_stale_trajectory_returns_hold_without_losing_output(self):
        self.i.timer_callback(None)
        self.i.trajectory_enabled=True
        self.i.command_callback(cmd(0,0,1,93))
        Stamp.clock+=.02
        self.i.timer_callback(None)
        self.assertEqual(self.i.last_source,'TRAJECTORY')
        n=len(self.i.output_pub.messages)
        Stamp.clock+=.24  # greater than command_timeout=0.20
        self.i.odom_callback(Odometry(0.02,0,1))
        self.i.health_callback(Bool(True))
        self.i.mission_permission_callback(Bool(True))
        self.i.timer_callback(None)
        self.assertEqual(len(self.i.output_pub.messages),n+1)
        self.assertEqual(self.i.last_source,'HOLD')
        self.assertAlmostEqual(self.i.output_pub.messages[-1].position.x,.02)
    def test_no_odom_never_promises_hold_stream(self):
        self.i.timer_callback(None)
        n=len(self.i.output_pub.messages)
        Stamp.clock+=.25
        self.i.timer_callback(None)
        self.assertEqual(len(self.i.output_pub.messages),n)
        self.assertEqual(self.i.status_pub.messages[-1], 'WAIT_ODOM')

class ContractChainTests(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.i=Interface()
        self.i.odom_callback(Odometry(1,1,1))
        self.i.health_callback(Bool(True))
        self.i.mission_permission_callback(Bool(True))
        self.i.timer_callback(None)
        Stamp.clock+=1.60
        self.i.odom_callback(Odometry(1,1,1))
        self.i.health_callback(Bool(True))
        self.i.mission_permission_callback(Bool(True))
        self.i.timer_callback(None)
        self.assertEqual(self.i.status_pub.messages[-1].data,'HOLD_ACTIVE')
        self.m=Mux()
        self.m.ego_cb(cmd(0,0,1,0,2048))
        self.m.source='EGO'
        self.m.timer_cb(None)
        self.assertEqual(self.m.status_pub.messages[-1].data,'EGO_ACTIVE')
    def sync_downstream(self):
        self.i.timer_callback(None)
        self.m.downstream_cb(self.i.output_pub.messages[-1])
        self.m.interface_cb(String(self.i.status_pub.messages[-1].data))
    def test_rejoin_e2e_uses_real_interface_hold(self):
        self.i.disable_trajectory(None)
        self.sync_downstream()
        self.m.point_cb(cmd(1,1,1,0))
        self.m.point_status_cb(String('AIRBORNE_HOLD'))
        result=self.m.use_point(None)
        self.assertTrue(result.success,result.message)
        self.m.timer_cb(None)
        self.i.command_callback(self.m.pub.messages[-1])
        self.assertTrue(self.i.enable_trajectory(None).success)
        self.i.timer_callback(None)
        self.assertEqual(self.i.status_pub.messages[-1].data,'TRAJECTORY_ACTIVE')
        self.assertAlmostEqual(self.i.output_pub.messages[-1].position.x,1.0)
    def test_old_point_rejection_keeps_independent_hold(self):
        self.i.disable_trajectory(None)
        self.sync_downstream()
        self.m.point_cb(cmd(1,0,1,-90))
        self.m.point_status_cb(String('AIRBORNE_HOLD'))
        self.assertFalse(self.m.use_point(None).success)
        for _ in range(12):
            Stamp.clock+=.02
            self.i.odom_callback(Odometry(1,1,1))
            self.i.health_callback(Bool(True))
            self.i.mission_permission_callback(Bool(True))
            self.i.timer_callback(None)
        self.assertEqual(self.i.last_source,'HOLD')
        self.assertAlmostEqual(self.i.output_pub.messages[-1].position.y,1.0)
        self.assertAlmostEqual(self.i.output_pub.messages[-1].yaw,0)

if __name__=='__main__':unittest.main()
