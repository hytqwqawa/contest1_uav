"""Exercise restored POINT handoff method outside ROS, with fake flight inputs."""
import ast
import math
import types
import unittest
from pathlib import Path
from prepare_dev import patch_mission

CLOCK=100.0
class Stamp:
    def __init__(self,x):self.x=x
    def __gt__(self,other):return self.x>other.x
    def __sub__(self,other):return Delta(self.x-other.x)
    @staticmethod
    def now():return Stamp(CLOCK)
class Delta:
    def __init__(self,x):self.x=x
    def to_sec(self):return self.x

class Msg:
    IGNORE_PX=1; IGNORE_PY=2; IGNORE_PZ=4; IGNORE_YAW=1024

class Trial:
    def __init__(self,scenario):
        self.scenario=scenario
        self.state=types.SimpleNamespace(connected=True,armed=True,mode='OFFBOARD')
        self.odom=types.SimpleNamespace(pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0,y=0,z=1))))
        self.point_cmd=None;self.point_cmd_time=None
        self.mux_status='EGO_ACTIVE';self.services=[]
        self.seq=0
    def current_yaw(self):return math.radians(93.)
    def call_service(self,name):
        self.services.append(name.rsplit('/',1)[-1])
        if name.endswith('use_point'):
            if self.scenario=='service_refuses':
                raise RuntimeError('switch rejected')
            self.mux_status='POINT_ACTIVE'
    def tick(self):
        global CLOCK
        CLOCK+=0.02
        self.seq+=1
        if self.scenario=='stale':return
        if self.scenario=='lost_offboard':
            self.state.mode='POSCTL'
            return
        yaw=(-175. if self.scenario=='wrong_yaw' else 93.)
        self.point_cmd=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.01,y=0.01,z=1.00),
            yaw=math.radians(yaw),type_mask=0)
        self.point_cmd_time=Stamp(CLOCK)

legacy=Path(__file__).parent.parent/'demo9_nextstage_src'  # fallback not used
# This fixture is *not* deployed: historical mission source for AST-method tests.
source_path=Path(__file__).parent/'fixtures/uav2_demo5_waypoints_reference_20261015.py'
src=patch_mission(source_path.read_text())
tree=ast.parse(src)
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='TangentPreview')
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='restore_point_hold')
module=ast.Module(body=[method],type_ignores=[])
ast.fix_missing_locations(module)
wrapped=compile(module,'<isolated POINT restore method>','exec')

class RestoreTests(unittest.TestCase):
    def run_restore(self, scenario):
        global CLOCK
        CLOCK=100.0
        trial=Trial(scenario)
        fake_ros=types.SimpleNamespace(
            Time=types.SimpleNamespace(now=Stamp.now),
            is_shutdown=lambda:False,
            logwarn=lambda *a,**kw:None,
            sleep=lambda x:trial.tick())
        env=dict(rospy=fake_ros,math=math,time=types.SimpleNamespace(time=lambda:CLOCK),
                 PositionTarget=Msg,wrap=lambda a:(a+math.pi)%(2*math.pi)-math.pi)
        exec(wrapped,env)
        return trial,lambda:env['restore_point_hold'](trial)
    def test_stale_point_cannot_reenable(self):
        t,fn=self.run_restore('stale')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):
            fn()
        self.assertEqual(t.services,['disable_trajectory','abort_trajectory'])
    def test_wrong_yaw_cannot_reenable(self):
        t,fn=self.run_restore('wrong_yaw')
        with self.assertRaisesRegex(RuntimeError,'fresh current HOLD'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_offboard_mode_lost(self):
        t,fn=self.run_restore('lost_offboard')
        with self.assertRaisesRegex(RuntimeError,'ARMED OFFBOARD'):fn()
        self.assertNotIn('enable_trajectory',t.services)
    def test_point_service_refusal_cannot_reenable(self):
        t,fn=self.run_restore('service_refuses')
        with self.assertRaisesRegex(RuntimeError,'switch rejected'):fn()
        self.assertIn('use_point',t.services)
        self.assertNotIn('enable_trajectory',t.services)
    def test_fresh_current_hold_can_reenable(self):
        t,fn=self.run_restore('good')
        fn()
        self.assertEqual(t.services,
             ['disable_trajectory','abort_trajectory','use_point','enable_trajectory'])

if __name__=='__main__':unittest.main(verbosity=2)
