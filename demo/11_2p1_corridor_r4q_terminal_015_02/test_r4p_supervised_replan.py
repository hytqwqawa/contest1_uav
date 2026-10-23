#!/usr/bin/env python3
"""Ground-only gate/mission tests; no ROS master or aircraft I/O."""
import ast
import math
import sys
import time
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as S

ROOT=Path(__file__).resolve().parent
GATE=ROOT/'ros_src/ego_uav_interface/scripts/bspline_single_plan_gate.py'
MISSION=(ROOT/'uav2_demo5_waypoints.py').read_text()
BRIDGE=(ROOT/'ros_src/ego_uav_interface/scripts/ego_trajectory_bridge.py').read_text()
LAUNCH=(ROOT/'ros_src/ego_uav_interface/launch/ego_uav2.launch').read_text()

class Pub:
    def __init__(self):self.sent=[];self.onpublish=None;self.connections=1
    def publish(self,msg):
        self.sent.append(msg)
        if self.onpublish:self.onpublish(msg)
    def get_num_connections(self):return self.connections
class Clock:
    def __init__(self,t=20.):self.t=t
    @staticmethod
    def now():return Clock(20.)
    def __sub__(self,other):return S(to_sec=lambda: self.t-other.t)
class FakeRos:
    Time=Clock
    def __init__(self):self.pub={};self.services={}
    def Publisher(self,topic,*args,**kwargs):
        p=Pub();self.pub[topic]=p;return p
    def Subscriber(self,*args,**kwargs):return None
    def Service(self,topic,kind,handler):self.services[topic]=handler
    def Timer(self,*args,**kwargs):return None
    def Duration(self,sec):return sec
    @staticmethod
    def sleep(sec):pass
    @staticmethod
    def is_shutdown():return False
    @staticmethod
    def logwarn(*args):pass
    @staticmethod
    def logerr(*args):pass
class Bspline:
    def __init__(self,tid=5,extra=0.0,stamp=(17,1)):
        self.traj_id=tid;self.order=3
        self.start_time=S(secs=stamp[0],nsecs=stamp[1])
        self.pos_pts=[S(x=float(i)+extra,y=0.0,z=0.8) for i in range(4)]
        self.knots=[0.,0.,0.,0.,1.,1.,1.,1.]

ROS=FakeRos()
backup={k:sys.modules.get(k) for k in ('rospy','std_msgs','std_msgs.msg','std_srvs','std_srvs.srv','traj_utils','traj_utils.msg')}
sys.modules['rospy']=ROS
for name in ('std_msgs','std_msgs.msg','std_srvs','std_srvs.srv','traj_utils','traj_utils.msg'):
    sys.modules[name]=types.ModuleType(name)
sys.modules['std_msgs.msg'].String=lambda data:S(data=data)
sys.modules['std_msgs.msg'].Int32=lambda data:S(data=data)
sys.modules['std_srvs.srv'].Trigger=object
sys.modules['std_srvs.srv'].TriggerResponse=lambda a,b:S(success=a,message=b)
sys.modules['traj_utils.msg'].Bspline=Bspline
try:
    gate_mod=types.ModuleType('r4p_gate')
    exec(compile(GATE.read_text(),str(GATE),'exec'),gate_mod.__dict__)
finally:
    for k,v in backup.items():
        if v is None:sys.modules.pop(k,None)
        else:sys.modules[k]=v

def gate():
    ROS.pub.clear();ROS.services.clear()
    g=gate_mod.SupervisedGate()
    g.bspline_cb(Bspline())
    assert g.freeze(None).success
    return g

# Compile only the live mission function and inject fake ROS bindings.
tree=ast.parse(MISSION)
cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='TangentPreview')
method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_r4p_accept_avoiding_replan')
env=dict(rospy=ROS,time=time,String=lambda data:S(data=data),
         spline_ticket=gate_mod.spline_ticket)
exec(compile(ast.Module(body=[method],type_ignores=[]),'<r4p mission>','exec'),env)
accept=env['_r4p_accept_avoiding_replan']

class Tests(unittest.TestCase):
    def test_first_preview_then_freeze(self):
        g=gate();self.assertEqual(g.state,'FROZEN:5')
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
    def test_new_candidate_staged_not_forwarded(self):
        g=gate();g.bspline_cb(Bspline(6))
        self.assertEqual(g.state,'PENDING:'+gate_mod.spline_ticket(Bspline(6)))
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
    def test_commit_approves_before_forwarding(self):
        g=gate();g.bspline_cb(Bspline(6))
        events=[]
        g.approved_pub.onpublish=lambda m:events.append(('approve',m.data))
        g.pub.onpublish=lambda m:events.append(('forward',m.traj_id))
        g.commit_cb(S(data=gate_mod.spline_ticket(Bspline(6))))
        self.assertEqual(events,[('approve',6),('forward',6)])
        self.assertEqual(g.state,'FROZEN:6')
    def test_invalid_ticket_does_not_release(self):
        g=gate();g.bspline_cb(Bspline(6))
        g.commit_cb(S(data='6:bad'))
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
    def test_old_candidate_cannot_replace_newer(self):
        g=gate();g.bspline_cb(Bspline(6));old=gate_mod.spline_ticket(Bspline(6))
        g.bspline_cb(Bspline(7));g.commit_cb(S(data=old))
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
        self.assertTrue(g.state.startswith('PENDING:7:'))
    def test_same_id_changed_geometry_is_staged(self):
        g=gate();g.bspline_cb(Bspline(5,extra=.1))
        self.assertTrue(g.state.startswith('PENDING:5:'))
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
    def test_unmodified_duplicate_does_not_replan(self):
        g=gate();g.bspline_cb(Bspline())
        self.assertEqual(g.state,'FROZEN:5')
        self.assertEqual(len(g.pub.sent),1)
    def test_timestamp_changes_ticket(self):
        self.assertNotEqual(gate_mod.spline_ticket(Bspline(6,stamp=(17,1))),
                            gate_mod.spline_ticket(Bspline(6,stamp=(17,2))))
    def test_disconnected_traj_server_refuses_commit(self):
        g=gate();g.bspline_cb(Bspline(6));g.pub.connections=0
        g.commit_cb(S(data=gate_mod.spline_ticket(Bspline(6))))
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
    def test_next_waypoint_rearms(self):
        g=gate();g.bspline_cb(Bspline(6));g.commit_cb(S(data=gate_mod.spline_ticket(Bspline(6))))
        self.assertTrue(g.arm_preview(None).success)
        g.bspline_cb(Bspline(7))
        self.assertEqual([m.traj_id for m in g.pub.sent],[5,6,7])
    def mission(self,valid=True):
        g=gate();candidate=Bspline(6);g.bspline_cb(candidate)
        ticket=gate_mod.spline_ticket(candidate)
        r=S(_r4o_gate_state='PENDING:'+ticket,_r4o_gate_time=Clock.now(),
            _r4p_candidate_time=time.monotonic(),bspline=candidate,
            state=S(connected=True,armed=True,mode='OFFBOARD'),
            flight_healthy=True,mission_allowed=True,demo_safe=True,
            mux_status='EGO_ACTIVE',_r4p_committed_ticket='',
            _r4o_locked_id=5,cmd=S(trajectory_id=5),cmd_time=Clock.now(),
            publish_traj_marker=lambda pts,tid:None)
        def validate(m):
            if not valid:raise RuntimeError('EGO path/replan outside Demo5 fence')
            return [(0.,0.,0.8)]
        r.validated_ego_path=validate
        def update_cmd(msg):
            if msg.traj_id!=5:r.cmd=S(trajectory_id=msg.traj_id);r.cmd_time=Clock.now()
        g.pub.onpublish=update_cmd
        g.committed_pub.onpublish=lambda m:setattr(r,'_r4p_committed_ticket',m.data)
        g.status_pub.onpublish=lambda m:setattr(r,'_r4o_gate_state',m.data)
        r._r4p_commit_pub=S(publish=g.commit_cb)
        return g,r
    def test_mission_rejects_ambiguous_same_id_geometry(self):
        g=gate();candidate=Bspline(5,extra=.1);g.bspline_cb(candidate)
        ticket=gate_mod.spline_ticket(candidate)
        _,r=self.mission()
        r.bspline=candidate;r._r4o_gate_state='PENDING:'+ticket
        with self.assertRaisesRegex(RuntimeError,'reused an executing traj_id'):
            accept(r,5)
        self.assertEqual(len(g.pub.sent),1)

    def test_mission_promotes_safe_new_spline(self):
        g,r=self.mission()
        self.assertEqual(accept(r,5),6)
        self.assertEqual(g.state,'FROZEN:6')
        self.assertEqual(r._r4o_locked_id,6)
    def test_mission_does_not_commit_invalid_spline(self):
        g,r=self.mission(valid=False)
        with self.assertRaisesRegex(RuntimeError,'outside Demo5 fence'):accept(r,5)
        self.assertEqual([m.traj_id for m in g.pub.sent],[5])
    def test_mission_returns_existing_id_if_frozen(self):
        g,r=self.mission()
        r._r4o_gate_state='FROZEN:5'
        self.assertEqual(accept(r,5),5)
    def test_mission_rejects_lost_gate(self):
        g,r=self.mission()
        r._r4o_gate_time=Clock(19.)
        with self.assertRaisesRegex(RuntimeError,'heartbeat lost'):accept(r,5)
    def test_no_unconditional_blocked_abort(self):
        self.assertNotIn('BLOCKED:',GATE.read_text())
        exec_direct=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='execute_direct')
        txt=ast.get_source_segment(MISSION,exec_direct)
        self.assertIn('traj_id=self._r4p_accept_avoiding_replan(traj_id)',txt)
        self.assertNotIn('if tid != traj_id:',txt[:txt.index('traj_id=self._r4p_accept_avoiding_replan')])
        self.assertNotIn('no in-flight replanning; stop EGO',txt)
    def test_no_automatic_waypoint_restart(self):
        fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='plan_execute_waypoint')
        txt=ast.get_source_segment(MISSION,fn)
        self.assertEqual(txt.count('self._demo9_plan_execute_once('),1)
        self.assertNotIn('for attempt',txt)
    def test_private_server_remapped_and_yaw_slew_not_reset(self):
        self.assertIn('demo11_r4p/bspline_to_traj_server',LAUNCH)
        self.assertIn('demo11_r4p_bspline_gate',LAUNCH)
        chunk=BRIDGE[BRIDGE.index('if msg.trajectory_id != self.active_traj_id:'):BRIDGE.index('vx = msg.velocity.x')]
        self.assertNotIn('self.yaw_cmd = self.current_yaw',chunk)
    def test_both_yaw_phases_process_safe_candidates(self):
        fixed=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='execute_terminal_yaw')
        txt=ast.get_source_segment(MISSION,fixed)
        self.assertIn('self._r4p_accept_avoiding_replan',txt)
    def test_real_waypoints_and_corridor_unchanged(self):
        import yaml
        cfg=yaml.safe_load((ROOT/'demo5_waypoints.yaml').read_text())
        self.assertEqual(cfg['flight_height'],.8)
        self.assertEqual([(w['forward'],w['left']) for w in cfg['waypoints']],[(3.5,.25),(4.061,-.439),(4.5,-1.5)])
        self.assertIn('self.r2_corridor.certify_control_hulls',MISSION)
        self.assertIn('self.h_min <= h <= self.h_max',MISSION)
        self.assertNotIn('mavros/cmd/arming',MISSION)

if __name__=='__main__':unittest.main(verbosity=2)
