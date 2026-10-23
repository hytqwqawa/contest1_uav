#!/usr/bin/env python3
"""R4N pure-Python regression. No rospy import, ROS master, or flight commands."""
import ast
import math
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as S

ROOT=Path(__file__).resolve().parent
MISSION=(ROOT/'uav2_demo5_waypoints.py').read_text()
BRIDGE=(ROOT/'ros_src/ego_uav_interface/scripts/ego_trajectory_bridge.py').read_text()
M_TREE=ast.parse(MISSION)
B_TREE=ast.parse(BRIDGE)
M=next(x for x in M_TREE.body if isinstance(x,ast.ClassDef) and x.name=='TangentPreview')
B=next(x for x in B_TREE.body if isinstance(x,ast.ClassDef) and x.name=='EgoTrajectoryBridge')

def method(cls,name):return next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
def code(cls,name,src):return ast.get_source_segment(src,method(cls,name))

class Clock:
    t=100.0
    def __init__(self,v=0.0):self.v=float(v)
    @staticmethod
    def now():return Clock(Clock.t)
    def __sub__(self,other):return S(to_sec=lambda:self.v-other.v)
class ROS:
    Time=Clock
    @staticmethod
    def logwarn(*args):pass
    @staticmethod
    def logerr(*args):pass
    @staticmethod
    def logwarn_throttle(*args):pass
    @staticmethod
    def logerr_throttle(*args):pass
class Target:
    FRAME_LOCAL_NED=1
    IGNORE_YAW_RATE=2048
    def __init__(self):
        self.header=S(stamp=None,frame_id='')
        self.position=S(x=0.,y=0.,z=0.)
        self.velocity=S(x=0.,y=0.,z=0.)
        self.acceleration_or_force=S(x=0.,y=0.,z=0.)
        self.yaw=0.;self.yaw_rate=0.
class Pub:
    def __init__(self):self.sent=[]
    def publish(self,x):self.sent.append(x)
def fake_bridge():
    clazz=ast.ClassDef(name='BridgeFake',bases=[],keywords=[],
                       body=[method(B,'command_cb')],decorator_list=[])
    ast.fix_missing_locations(clazz)
    env=dict(rospy=ROS,math=math,wrap_angle=lambda v:math.atan2(math.sin(v),math.cos(v)),
             PositionTarget=Target,String=lambda **kw:S(**kw))
    exec(compile(ast.Module(body=[clazz],type_ignores=[]),'<bridge>','exec'),env)
    b=env['BridgeFake']()
    for name,val in dict(current_yaw=0.,current_position=(1.,2.,.771),
        _r4m_world_z=.771,_r4m_world_z_time=Clock(100),_r4m_approved_id=7,
        active_traj_id=None,yaw_cmd=0.,last_time=None,
        mux_ego_active=True,min_tangent_speed=.06,yaw_mode='TANGENT',
        tangent_yaw=None,hold_yaw=None,fixed_yaw=None,
        yaw_rate_limit=.35,max_yaw_lead=.18,
        output_pub=Pub(),status_pub=Pub()).items():setattr(b,name,val)
    return b
def command(z,tid=7,vz=.14,az=.04):
    return S(trajectory_id=tid,position=S(x=1.5,y=2.3,z=z),
             velocity=S(x=.2,y=0.,z=vz),acceleration=S(x=.1,y=0.,z=az))

class R4NTests(unittest.TestCase):
    def test_waypoint_targets_remain_h08(self):
        import yaml
        cfg=yaml.safe_load((ROOT/'demo5_waypoints.yaml').read_text())
        self.assertEqual(cfg['flight_height'],.8)
        self.assertEqual([(w['forward'],w['left']) for w in cfg['waypoints']],
                         [(3.5,.25),(4.061,-.439),(4.5,-1.5)])
        s=code(M,'_demo9_plan_execute_once',MISSION)
        self.assertIn('gz = self._r4m_world_z',s)
        self.assertIn('msg.pose.position.z = gz',s)
        self.assertNotIn('unsafe fixed-height start gap',s)
    def test_nominal_height_rejects_removed(self):
        self.assertNotIn('def _r4m_check_control_heights',MISSION)
        self.assertNotIn('[R4M PLANAR REJECT]',MISSION)
        self.assertNotIn('offset=+%.3fm > 0.015m',MISSION)
        self.assertNotIn('allowed_h_min =',MISSION)
        self.assertNotIn('h_target - 0.25',MISSION)
        self.assertNotIn('cannot plan fixed-height EGO',MISSION)
        self.assertNotIn('abs(dz)>0.020',BRIDGE)
        self.assertNotIn('FIXED height drift',BRIDGE)
        self.assertIn('planned B-spline Demo-H',MISSION)
        self.assertIn('INFO only; 3D EGO preserved',MISSION)
    def test_fences_and_approval_remaining(self):
        preview=code(M,'_demo9_plan_execute_once',MISSION)
        self.assertIn('self.r2_certify_bspline_corridor(planned_bspline, pts)',preview)
        self.assertIn('self.h_min <= h <= self.h_max',preview)
        self.assertIn('self._r4m_approved_pub.publish',preview)
        self.assertLess(preview.index('self.r2_certify_bspline_corridor(planned_bspline, pts)'),
                        preview.index('self._r4m_approved_pub.publish'))
        live=code(M,'validated_ego_path',MISSION)
        self.assertIn('self.r2_certify_bspline_corridor(msg, pts)',live)
        self.assertIn('self.h_min <= h <= self.h_max',live)
        self.assertIn('return pts',live)
        self.assertNotIn('self.validated_ego_path(candidate)',code(M,'execute_direct',MISSION))
        self.assertNotIn('self._r4m_approved_pub.publish',code(M,'execute_direct',MISSION))
        self.assertIn('self._r4o_confirm_one_plan(traj_id)',code(M,'execute_direct',MISSION))
    def test_specific_real_r4m_first_plan_no_nominal_reject(self):
        # The R4M incident first plan had H0.788..0.841, and was rejected
        # solely because its *control point* was 4.1cm off locked Z.
        env={'math':math,'RuntimeError':RuntimeError}
        fn=method(M,'validated_ego_path')
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'<validate>','exec'),env)
        points=[(1.0,.2,.788),(2.0,.23,.800),(3.5,.25,.841)]
        runner=S(x_min=-1.,x_max=6.,y_min=-2.,y_max=3.,h_min=.4,h_max=1.3,
                 sample_bspline=lambda msg:(points,0.,1.),
                 to_demo=lambda p:p,
                 r2_certify_bspline_corridor=lambda msg,pts:True)
        self.assertEqual(env['validated_ego_path'](runner,object()),points)
        runner.h_min=.55
        points2=[(1.,.2,.429),(2.,.23,.800)]
        runner.sample_bspline=lambda msg:(points2,0.,1.)
        with self.assertRaisesRegex(RuntimeError,'outside Demo5 fence'):
            env['validated_ego_path'](runner,object())
    def test_bridge_retains_original_z_vz_az(self):
        b=fake_bridge()
        b.command_cb(command(.813,vz=.17,az=.11))
        self.assertEqual(len(b.output_pub.sent),1)
        out=b.output_pub.sent[0]
        self.assertAlmostEqual(out.position.z,.813)
        self.assertAlmostEqual(out.velocity.z,.17)
        self.assertAlmostEqual(out.acceleration_or_force.z,.11)
        self.assertEqual(out.position.x,1.5)
        # No lock means a prior control point can be 7.6cm below the target.
        b.command_cb(command(.695,vz=-.12,az=-.10))
        self.assertEqual(len(b.output_pub.sent),2)
        self.assertAlmostEqual(b.output_pub.sent[-1].position.z,.695)
    def test_bridge_refuses_unauthorized_or_stale(self):
        b=fake_bridge()
        b.command_cb(command(.8,tid=19))
        self.assertEqual(len(b.output_pub.sent),0)
        b._r4m_world_z_time=Clock(99.)
        b.command_cb(command(.8))
        self.assertEqual(len(b.output_pub.sent),0)
        b._r4m_world_z_time=Clock(100.)
        b.command_cb(command(float('nan')))
        self.assertEqual(len(b.output_pub.sent),0)
    def test_fixed_terminal_yaw_captures_actual_z(self):
        b=fake_bridge()
        b.yaw_mode='FIXED'
        b.command_cb(command(.79))
        self.assertEqual(len(b.output_pub.sent),1)
        out=b.output_pub.sent[0]
        self.assertAlmostEqual(out.position.z,.771)
        self.assertAlmostEqual(out.velocity.z,0.)
        self.assertAlmostEqual(out.acceleration_or_force.z,0.)
    def test_physical_takeoff_safety_retained(self):
        takeoff=code(M,'wait_airborne_hold',MISSION)
        self.assertIn('height_ok',takeoff)
        self.assertIn('vz_ok',takeoff)
        after=code(M,'demo9_verified_hold',MISSION)
        self.assertIn('abs(relative_h-',after)
        self.assertIn('not final_handoff',after)
        self.assertIn('flight_healthy',after)
        self.assertIn('mission_allowed',after)
    def test_version_isolation_and_no_arming(self):
        self.assertIn('/contest1/demo11_r4p/locked_world_z',MISSION)
        self.assertIn('/contest1/demo11_r4p/approved_traj_id',BRIDGE)
        self.assertNotIn('/contest1/demo11_r4m/',BRIDGE)
        self.assertNotIn('mavros/cmd/arming',MISSION)
        self.assertNotIn('CommandBool',MISSION)
        self.assertIn('tracking_allowance: 0.15',
                      (ROOT/'demo11_corridor.yaml').read_text())
        self.assertIn('DEMO11_P2_GUARD',
                      (ROOT/'start_backend.sh').read_text())
    def test_rviz_and_bag_preserved(self):
        self.assertTrue((ROOT/'rviz_corridor_tubes.py').is_file())
        record=(ROOT/'run_demo9.sh').read_text()
        self.assertIn('/contest1/drone_0_planning/bspline',record)
        self.assertIn('/contest1/position_cmd',record)
        self.assertIn('/contest1/demo11_r4p/approved_traj_id',record)

if __name__=='__main__': unittest.main(verbosity=2)
