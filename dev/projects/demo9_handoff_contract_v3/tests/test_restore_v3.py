"""Run actual V3 mission method extracted by AST, using offline fake ROS."""
import ast
import math
import types
import unittest
from pathlib import Path

class Delta:
    def __init__(self,n):self.n=n
    def to_sec(self):return self.n
class Stamp:
    clock=100.0
    def __init__(self,n):self.n=float(n)
    @staticmethod
    def now():return Stamp(Stamp.clock)
    def __sub__(self,other):return Delta(self.n-other.n)
    def __gt__(self,other):return self.n>other.n
    def __eq__(self,other):return isinstance(other,Stamp) and self.n==other.n
class PositionTarget:
    FRAME_LOCAL_NED=1
    IGNORE_PX=1;IGNORE_PY=2;IGNORE_PZ=4;IGNORE_YAW=1024

class Trial:
    def __init__(self,scenario):
        self.scenario=scenario
        self.services=[]
        self.state=types.SimpleNamespace(connected=True,armed=True,mode='OFFBOARD')
        self.flight_healthy=True; self.mission_allowed=True;self.demo_safe=True
        self.state_rx_time=Stamp.now();self.health_rx_time=Stamp.now()
        self.permission_rx_time=Stamp.now()
        self.odom=types.SimpleNamespace(
            pose=types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0,y=0.0,z=1.0))),
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0,y=0.0,z=0.0))))
        self.odom_time=Stamp.now()
        self.point_status='AIRBORNE_HOLD'
        self.point_cmd=None;self.point_cmd_time=None
        self.mux_status='EGO_ACTIVE';self.mux_status_time=Stamp.now()
        self.if_status='TRAJECTORY_ACTIVE';self.if_status_time=Stamp.now()
        self.mavros_sp=None;self.mavros_sp_time=None
        self.epoch=0;self.sample_count=0
        self._enabled=False
        self._ticks=0
    def current_yaw(self): return math.radians(93.0)
    def call_service(self,name):
        verb=name.rsplit('/',1)[-1]
        self.services.append(verb)
        if verb=='disable_trajectory':
            self._enabled=False
            self.if_status='HOLD_ACTIVE'
            self.if_status_time=Stamp.now()
        elif verb=='abort_trajectory':
            self.epoch+=1
            self.point_cmd_time=None
            self.sample_count=0
            if self.scenario=='abort_refusal':
                raise RuntimeError('abort failed')
        elif verb=='use_point':
            if self.scenario=='service_refusal':raise RuntimeError('switch rejected')
            if self.scenario!='no_mux_ack':
                self.mux_status='POINT_ACTIVE'
                # Post-service ACK should be updated on next timer tick.
                self.mux_status_time=None
        elif verb=='enable_trajectory':
            self._enabled=True
            if self.scenario=='enable_ack_loss':
                raise RuntimeError('enable service ACK lost')
        return True
    def tick(self):
        Stamp.clock += .02
        self._ticks+=1
        self.odom_time=Stamp.now()
        self.if_status_time=Stamp.now()
        self.state_rx_time=Stamp.now();self.health_rx_time=Stamp.now()
        self.permission_rx_time=Stamp.now()
        if self.scenario=='lost_offboard' and self._ticks==3: self.state.mode='POSCTL'
        if self.scenario=='health_drop' and self._ticks==3: self.flight_healthy=False
        if self.scenario=='rc_drop' and self._ticks==3: self.mission_allowed=False
        if self.scenario=='stale_odom':self.odom_time=Stamp(Stamp.clock-1)
        if self.scenario=='rc_drop_after_enable' and self._enabled:
            self.mission_allowed=False
        if self.mux_status=='POINT_ACTIVE': self.mux_status_time=Stamp.now()
        if not self._enabled:
            self.if_status='HOLD_ACTIVE'
            if self.scenario != 'missing_hold_stream':
                x=1.0 if self.scenario=='hold_target_mismatch' else .01
                self.mavros_sp=types.SimpleNamespace(
                    coordinate_frame=1,type_mask=0,
                    position=types.SimpleNamespace(x=x,y=.01,z=1.0),
                    yaw=math.radians(93.0))
                self.mavros_sp_time=Stamp.now()
            if self.scenario in ('stale_point','wrong_yaw','no_mux_ack','no_output','delayed_first_capture'):
                if self.scenario=='stale_point':return
                if self.scenario=='delayed_first_capture' and self.epoch==1:return
            self.sample_count+=1
            yaw=-175.0 if self.scenario=='wrong_yaw' else 93.0
            self.point_cmd=types.SimpleNamespace(
                coordinate_frame=1,type_mask=0,
                position=types.SimpleNamespace(x=.01,y=.01,z=1.0),
                yaw=math.radians(yaw))
            self.point_cmd_time=Stamp.now()
            return
        if self.scenario=='no_output':
            self.if_status='HOLD_ACTIVE'
            return
        self.if_status='TRAJECTORY_ACTIVE'
        self.mavros_sp=types.SimpleNamespace(
            coordinate_frame=1,type_mask=0,
            position=types.SimpleNamespace(x=.01,y=.01,z=1.0),
            yaw=math.radians(93.0))
        self.mavros_sp_time=Stamp.now()

SOURCE=Path(__file__).resolve().parents[1]/'candidates/uav2_demo5_waypoints.py'
T=ast.parse(SOURCE.read_text())
K=next(n for n in T.body if isinstance(n,ast.ClassDef) and n.name=='TangentPreview')
F=next(n for n in K.body if isinstance(n,ast.FunctionDef) and n.name=='restore_point_hold')
E=ast.Module(body=[F],type_ignores=[])
ast.fix_missing_locations(E)
CODE=compile(E,str(SOURCE),'exec')

class RestoreActualTests(unittest.TestCase):
    def make(self,scenario):
        Stamp.clock=100.0
        t=Trial(scenario)
        ros=types.SimpleNamespace(
            Time=types.SimpleNamespace(now=Stamp.now),
            is_shutdown=lambda:False,
            logwarn=lambda *a,**k:None,
            logerr=lambda *a,**k:None,
            sleep=lambda _:t.tick())
        fake_time=types.SimpleNamespace(monotonic=lambda:Stamp.clock)
        env=dict(rospy=ros,PositionTarget=PositionTarget,time=fake_time,math=math,
                 wrap=lambda a:(a+math.pi)%(2*math.pi)-math.pi)
        exec(CODE,env)
        return t, lambda:env['restore_point_hold'](t)
    def test_good_auto_rejoin_verified_through_mavros(self):
        t,fn=self.make('good');fn()
        self.assertEqual(t.services,['disable_trajectory','abort_trajectory',
                      'use_point','enable_trajectory'])
        self.assertEqual(t.if_status,'TRAJECTORY_ACTIVE')
    def test_no_point_never_enables(self):
        t,fn=self.make('stale_point')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'): fn()
        self.assertEqual(t.services,['disable_trajectory','abort_trajectory','abort_trajectory'])
    def test_wrong_yaw_never_enables(self):
        t,fn=self.make('wrong_yaw')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('use_point',t.services)
        self.assertNotIn('enable_trajectory',t.services)
    def test_stale_odom_never_enables(self):
        t,fn=self.make('stale_odom')
        with self.assertRaisesRegex(RuntimeError,'odometry stale'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_offboard_loss_cancels(self):
        t,fn=self.make('lost_offboard')
        with self.assertRaisesRegex(RuntimeError,'ARMED OFFBOARD'):fn()
        self.assertNotIn('use_point',t.services)
    def test_health_failure_cancels(self):
        t,fn=self.make('health_drop')
        with self.assertRaisesRegex(RuntimeError,'flight_healthy=False'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_rc_cancel_cancels(self):
        t,fn=self.make('rc_drop')
        with self.assertRaisesRegex(RuntimeError,'RC permission lost'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_use_point_refusal_does_not_enable(self):
        t,fn=self.make('service_refusal')
        with self.assertRaisesRegex(RuntimeError,'switch rejected'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_missing_mux_ack_does_not_enable(self):
        t,fn=self.make('no_mux_ack')
        with self.assertRaisesRegex(RuntimeError,'POINT_ACTIVE not confirmed'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_missing_mavros_echo_disables_again(self):
        t,fn=self.make('no_output')
        with self.assertRaisesRegex(RuntimeError,'MAVROS POINT output not verified'):fn()
        self.assertEqual(t.services[-1],'disable_trajectory')
    def test_enable_ack_loss_restores_hold(self):
        t,fn=self.make('enable_ack_loss')
        with self.assertRaisesRegex(RuntimeError,'enable service ACK lost'):fn()
        self.assertEqual(t.services[-1],'disable_trajectory')
    def test_rc_drop_after_enable_restores_hold(self):
        t,fn=self.make('rc_drop_after_enable')
        with self.assertRaisesRegex(RuntimeError,'RC permission lost'):fn()
        self.assertEqual(t.services[-1],'disable_trajectory')
    def test_missing_downstream_hold_blocks_switch(self):
        t,fn=self.make('missing_hold_stream')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('use_point',t.services)
    def test_mismatched_downstream_hold_blocks_switch(self):
        t,fn=self.make('hold_target_mismatch')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('use_point',t.services)
    def test_one_capture_retry_then_success(self):
        t,fn=self.make('delayed_first_capture');fn()
        self.assertEqual(t.services.count('abort_trajectory'),2)
        self.assertEqual(t.services[-1],'enable_trajectory')

if __name__=='__main__':unittest.main()
