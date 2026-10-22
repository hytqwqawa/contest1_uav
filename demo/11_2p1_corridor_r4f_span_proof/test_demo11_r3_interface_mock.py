#!/usr/bin/env python3
"""No-ROS control-path test: final TrajectoryInterface trajectory -> HOLD.

Only Python stubs are used. Absolutely no ROS master/MAVROS/actuator output.
"""
import sys, types, math, time, tempfile, unittest, importlib.util
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parent
SCRIPTS=ROOT/'demo8_interface_overlay/scripts'
sys.path.insert(0,str(SCRIPTS))

class MockDuration:
    def __init__(self,s):self.s=float(s)
    def to_sec(self):return self.s
class MockTime:
    now_sec=100.0
    def __init__(self,s):self.s=s
    def __sub__(self,other):return MockDuration(self.s-other.s)
    @staticmethod
    def now():return MockTime(MockTime.now_sec)
class Pub:
    def __init__(self,*args,**kwargs):self.values=[]
    def publish(self,obj):self.values.append(obj)
class Message:
    def __init__(self,data=None):self.data=data
class XY:
    def __init__(self,x=0.,y=0.,z=0.):self.x,self.y,self.z=x,y,z
class Quaternion:
    def __init__(self):self.x=self.y=self.z=0.;self.w=1.
class PositionTarget:
    FRAME_LOCAL_NED=1
    IGNORE_VX=8;IGNORE_VY=16;IGNORE_VZ=32
    IGNORE_AFX=64;IGNORE_AFY=128;IGNORE_AFZ=256
    IGNORE_YAW_RATE=2048
    def __init__(self):
        self.header=types.SimpleNamespace(stamp=None,frame_id='')
        self.coordinate_frame=1;self.type_mask=0
        self.position=XY();self.velocity=XY();self.acceleration_or_force=XY()
        self.yaw=0.;self.yaw_rate=0.
class Odom:
    def __init__(self,x,y,z):
        self.pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=XY(x,y,z),orientation=Quaternion()))
class Response:
    def __init__(self,success,message):self.success=success;self.message=message

rospy=types.ModuleType('rospy')
rospy.Publisher=Pub
rospy.Subscriber=lambda *a,**k: None
rospy.Service=lambda *a,**k: None
rospy.Timer=lambda *a,**k: None
rospy.Duration=MockDuration
rospy.Time=MockTime
rospy.get_param=lambda name,default=None: rospy.params.get(name,default)
for m in ('loginfo','logwarn','logerr','logwarn_throttle'):
    setattr(rospy,m,lambda *a,**k:None)
sys.modules['rospy']=rospy
nav=types.ModuleType('nav_msgs');navmsg=types.ModuleType('nav_msgs.msg');navmsg.Odometry=Odom;nav.msg=navmsg
std=types.ModuleType('std_msgs');stdmsg=types.ModuleType('std_msgs.msg');stdmsg.Bool=Message;stdmsg.String=Message;std.msg=stdmsg
srvs=types.ModuleType('std_srvs');srvsm=types.ModuleType('std_srvs.srv');srvsm.Trigger=object;srvsm.TriggerResponse=Response;srvs.srv=srvsm
mav=types.ModuleType('mavros_msgs');mavm=types.ModuleType('mavros_msgs.msg');mavm.PositionTarget=PositionTarget;mav.msg=mavm
tf=types.ModuleType('tf');tft=types.ModuleType('tf.transformations');tft.euler_from_quaternion=lambda q:(0.,0.,0.);tf.transformations=tft
sys.modules.update({'nav_msgs':nav,'nav_msgs.msg':navmsg,'std_msgs':std,
                    'std_msgs.msg':stdmsg,'std_srvs':srvs,'std_srvs.srv':srvsm,
                    'mavros_msgs':mav,'mavros_msgs.msg':mavm,
                    'tf':tf,'tf.transformations':tft})

spec=importlib.util.spec_from_file_location('r3_if', SCRIPTS/'trajectory_interface_demo8.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class OutputTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        from demo5_config import load_config
        plan=load_config(ROOT/'demo5_waypoints.yaml')
        self.path=Path(self.tmp.name)/'session_frame.yaml'
        self.path.write_text(yaml.safe_dump(dict(
            demo5_plan_snapshot=plan,demo_fence=plan['fence'],
            origin_world={'x':0.,'y':0.,'z':0.},
            forward_world={'x':1.,'y':0.}, left_world={'x':0.,'y':1.})))
        rospy.params={'~r3_corridor_yaml':str(ROOT/'demo11_corridor.yaml'),
                      '~r3_mission_yaml':str(ROOT/'demo5_waypoints.yaml'),
                      '~r3_session_yaml':str(self.path)}
        MockTime.now_sec=100.
        self.iface=module.TrajectoryInterface()
        self.iface.odom_callback(Odom(1.,0.,.8))
        self.iface.health_callback(Message(True))
        self.iface.mission_permission_callback(Message(True))
        self.iface.r3_safe_cb(Message(True))
        self.iface.r3_session_cb(Message(True))
        self.assertTrue(self.iface.r3_frame_ok())
        self.iface.timer_callback(None)
        self.assertEqual(self.iface.last_source,'HOLD')

    def send(self,x,y,yaw=0):
        cmd=PositionTarget();cmd.position=XY(x,y,.8);cmd.yaw=yaw
        self.iface.command_callback(cmd)
        MockTime.now_sec+=.02
        self.iface.r3_safe_cb(Message(True))
        self.iface.timer_callback(None)
        return self.iface.output_pub.values[-1]

    def enable(self):
        response=self.iface.enable_trajectory(None)
        self.assertTrue(response.success,response.message)

    def test_takeoff_outside_lateral_corridor_admitted(self):
        # Actual original denied launch at (F ~0.011, L~-0.065).
        self.iface.odom_callback(Odom(.011,-.065,.8))
        resp=self.iface.enable_trajectory(None)
        self.assertTrue(resp.success,resp.message)
        cmd=self.send(.021,-.065)
        self.assertFalse(self.iface.r3_abort_latched)
        self.assertAlmostEqual(cmd.position.y,-.065)

    def test_entry_unsafe_lateral_aborts_last_hop(self):
        self.iface.odom_callback(Odom(.011,-.065,.8))
        self.enable()
        self.send(.021,-.065)
        self.iface.odom_callback(Odom(.82,-.16,.8))
        self.send(.83,-.16)
        self.assertTrue(self.iface.r3_abort_latched)
        self.assertFalse(self.iface.trajectory_enabled)
        self.assertEqual(self.iface.last_source,'HOLD')

    def test_p1_yaw_endpoint_inside_relaxed_guard_remains_active(self):
        # Real R4D abort was F3.178 L~-0.050 while yaw FIXED.
        # R4E must admit tracking variation to at least L-0.070.
        self.iface.odom_callback(Odom(3.178,-.070,.8))
        self.enable()
        o=self.send(3.18,-.070)
        self.assertFalse(self.iface.r3_abort_latched)
        self.assertEqual(self.iface.last_source,'TRAJECTORY')
        self.assertAlmostEqual(o.position.y,-.070)

    def test_p1_actual_beyond_reduced_margin_still_aborts(self):
        self.enable()
        self.iface.odom_callback(Odom(3.178,-.151,.8))
        self.send(3.18,-.12)
        self.assertTrue(self.iface.r3_abort_latched)
        self.assertIn('actual_aircraft',self.iface.r3_abort_reason)
        self.assertEqual(self.iface.last_source,'HOLD')

    def test_normal_position_accepted(self):
        self.enable()
        o=self.send(1.01,0.)
        self.assertEqual(self.iface.last_source,'TRAJECTORY')
        self.assertAlmostEqual(o.position.x,1.01)
        self.assertFalse(self.iface.r3_abort_latched)

    def test_late_replan_outside_wall_holds_current_position(self):
        self.enable()
        self.send(1.01,0.)
        output=self.send(1.05,-.904)
        self.assertTrue(self.iface.r3_abort_latched)
        self.assertFalse(self.iface.trajectory_enabled)
        self.assertEqual(self.iface.last_source,'HOLD')
        self.assertAlmostEqual(output.position.x,1.)
        self.assertAlmostEqual(output.position.y,0.)
        self.assertFalse(self.iface.enable_trajectory(None).success)
        latest=len(self.iface.output_pub.values)
        MockTime.now_sec+=.02
        self.iface.r3_safe_cb(Message(True))
        self.iface.timer_callback(None)
        self.assertEqual(len(self.iface.output_pub.values),latest+1)
        self.assertEqual(self.iface.last_source,'HOLD')

    def test_yaw_spike_while_xy_safe_goes_to_hold(self):
        self.enable()
        self.send(1.01,0.,0.)
        self.send(1.02,0.,math.radians(32))
        self.assertTrue(self.iface.r3_abort_latched)
        self.assertIn('yaw_step',self.iface.r3_abort_reason)

    def test_missing_frame_rejects_enable(self):
        self.iface.r3_session_ready=False
        self.assertFalse(self.iface.enable_trajectory(None).success)

    def test_actual_position_beyond_wall_holds(self):
        self.enable()
        self.send(1.01,0.)
        self.iface.odom_callback(Odom(1.1,-.9,.8))
        self.send(1.04,0.0)
        self.assertTrue(self.iface.r3_abort_latched)
        self.assertIn('actual_aircraft',self.iface.r3_abort_reason)

if __name__=='__main__':unittest.main(verbosity=2)
