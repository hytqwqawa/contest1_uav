"""Reproduce an actual POINT abort + timer publication with a stale 10-ms MUX buffer.

Uses extracted methods from the uploaded POINT source, the full V4 MUX,
and the full uploaded TrajectoryInterface in a mocked ROS bus.
No flight hardware, ROS master or PX4 is controlled.
"""
import ast
import math
import types
import unittest
from pathlib import Path
from test_mux_and_interface_v3 import (
    ROOT, Mux, Interface, Stamp, Odometry, PositionTarget,
    String, Publisher, Response, ros, cmd,
)

source_file=ROOT/'reference/point_traj_node.py'
tree=ast.parse(source_file.read_text())
point_class=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='PointTrajectoryNode')
methods=('current_position','current_yaw','capture_current_hold','is_airborne_by_height',
         'abort_to_current_hold','abort_trajectory','publish_command','timer_callback')
method_nodes=[n for n in point_class.body if isinstance(n,ast.FunctionDef) and n.name in methods]
method_ast=ast.Module(body=method_nodes,type_ignores=[])
ast.fix_missing_locations(method_ast)
env=dict(__name__='offline_real_point',rospy=ros,math=math,
         PositionTarget=PositionTarget,String=String,TriggerResponse=Response,
         euler_from_quaternion=lambda q:(0,0,0))
exec(compile(method_ast,str(source_file),'exec'),env)

class RealPoint:
    def __init__(self):
        self.odom=Odometry(1,1,1)
        self.initialized=True
        self.home_p=[0.,0.,0.]
        self.airborne_height_threshold=.08
        self.hold_p=[0.,0.,1.]
        self.hold_yaw=math.radians(-90)
        self.mode='AIRBORNE_HOLD'
        self.traj_active=False
        self.traj_kind=''
        self.traj_t0=None
        self.takeoff_completed=True
        self.cmd_pub=Publisher()
        self.status_pub=Publisher()
        self.odom_fresh=lambda:True
        self.health_ok=lambda:True
        self.mission_permission_ok=lambda:True
        self.px4_execution_ok=lambda:True
        self.final_drop_grace_active=lambda:False
        self.evaluate_trajectory=lambda: (_ for _ in ()).throw(AssertionError('trajectory not active'))
for fn in methods:
    setattr(RealPoint,fn,env[fn])

class TenMsRace(unittest.TestCase):
    def setUp(self):
        Stamp.clock=100.0
        self.i=Interface()
        self.i.odom_callback(Odometry(1,1,1))
        self.i.health_callback(types.SimpleNamespace(data=True))
        self.i.mission_permission_callback(types.SimpleNamespace(data=True))
        self.i.disable_trajectory(None)
        self.i.timer_callback(None)
        self.m=Mux()
        self.m.source='EGO'
        self.m.ego_cb(cmd(1,1,1,0))
        self.m.timer_cb(None)
        self.m.interface_cb(String('HOLD_ACTIVE'))
        self.m.downstream_cb(self.i.output_pub.messages[-1])
        self.p=RealPoint()
        self.m.point_cb(cmd(0,0,1,-90))  # OLD POINT: 1.41 m & 90 deg away
        self.m.point_status_cb(String('AIRBORNE_HOLD'))

    def test_10ms_old_point_veto_then_new_20ms_hold_accepted(self):
        Stamp.clock=100.010
        response=self.p.abort_trajectory(None)
        self.assertTrue(response.success)
        self.assertEqual(self.p.mode,'AIRBORNE_HOLD')
        # Service succeeded, but timer has not published refreshed POINT.
        verdict=self.m.use_point(None)
        self.assertFalse(verdict.success)
        self.assertEqual(self.m.source,'EGO')
        self.assertEqual(self.i.trajectory_enabled,False)
        Stamp.clock=100.030
        self.p.timer_callback(None)  # Real POINT's next 50-Hz HOLD publish.
        updated=self.p.cmd_pub.messages[-1]
        self.assertAlmostEqual(updated.position.x,1)
        self.assertAlmostEqual(updated.yaw,0)
        self.assertGreater(updated.header.stamp.s,100.010)
        self.m.point_cb(updated)
        self.m.point_status_cb(self.p.status_pub.messages[-1])
        self.m.downstream_cb(self.i.output_pub.messages[-1])
        self.m.interface_cb(String('HOLD_ACTIVE'))
        verdict=self.m.use_point(None)
        self.assertTrue(verdict.success,verdict.message)
        self.m.timer_cb(None)
        self.i.command_callback(self.m.pub.messages[-1])
        self.assertTrue(self.i.enable_trajectory(None).success)
        self.i.timer_callback(None)
        self.assertTrue(self.i.status_pub.messages[-1].data.endswith('TRAJECTORY_ACTIVE'))
        self.assertAlmostEqual(self.i.output_pub.messages[-1].position.x,1)
        self.assertAlmostEqual(self.i.output_pub.messages[-1].yaw,0)

    def test_delayed_old_packet_after_new_packet_cannot_replace_it(self):
        Stamp.clock=100.020
        self.p.abort_trajectory(None)
        self.p.timer_callback(None)
        self.m.point_cb(self.p.cmd_pub.messages[-1])
        last=self.m.point_msg.header.stamp
        # An out-of-order packet arrives later with an older generation stamp.
        old=cmd(0,0,1,-90)
        old.header.stamp=Stamp(100.005)
        Stamp.clock=100.050
        self.m.point_cb(old)
        self.assertEqual(self.m.point_msg.header.stamp.s,last.s)
        self.assertAlmostEqual(self.m.point_msg.position.x,1)

    def test_old_packet_received_after_abort_is_not_post_abort_generated(self):
        Stamp.clock=100.010
        reset=Stamp.now()
        old=cmd(1,1,1,0)
        old.header.stamp=Stamp(100.001)
        Stamp.clock=100.015
        self.m.point_cb(old)
        self.assertGreater(self.m.point_time.s,reset.s)
        # ROS receive time > reset, but ROS generation time < reset.
        self.assertLess(self.m.point_msg.header.stamp.s,reset.s)

    def test_normal_point_landing_trajectory_after_hold_remains_allowed(self):
        Stamp.clock=100.020
        self.p.abort_trajectory(None)
        self.p.timer_callback(None)
        self.m.point_cb(self.p.cmd_pub.messages[-1]);self.m.point_status_cb(String('AIRBORNE_HOLD'))
        self.assertTrue(self.m.use_point(None).success)
        for i in range(17):
            Stamp.clock+=.02
            self.p.timer_callback(None)
            self.m.point_cb(self.p.cmd_pub.messages[-1]);self.m.point_status_cb(String('AIRBORNE_HOLD'))
            self.m.timer_cb(None)
        self.assertIsNone(self.m._guard_until)
        # A new legitimate POINT landing trajectory is not treated as HOLD.
        self.m.point_status_cb(String('POINT_TRAJECTORY'))
        land=cmd(1,1,.98,0)
        land.velocity.z=-.2
        land.acceleration_or_force.z=-.3
        self.m.point_cb(land)
        before=len(self.m.pub.messages)
        self.m.timer_cb(None)
        self.assertEqual(len(self.m.pub.messages),before+1)
        self.assertEqual(self.m.status_pub.messages[-1].data,'POINT_ACTIVE')

if __name__=='__main__':unittest.main()
