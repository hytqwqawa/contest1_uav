#!/usr/bin/env python3
"""Copy and patch current files ONLY into a new dev/projects folder.
Refuse changed/unrecognized interfaces; NEVER write original paths.
"""
import argparse, ast, difflib, hashlib, shutil, textwrap
from pathlib import Path


def locate(src, cls, func):
    root=ast.parse(src)
    classes=[n for n in root.body if isinstance(n,ast.ClassDef) and n.name==cls]
    if len(classes)!=1: raise ValueError('unknown class '+cls)
    nodes=[n for n in classes[0].body if isinstance(n,ast.FunctionDef) and n.name==func]
    if len(nodes)!=1: raise ValueError('unknown method '+cls+'.'+func)
    return nodes[0]

def replace_method(src, cls, name, code, must_contain=()):
    node=locate(src,cls,name); lines=src.splitlines(keepends=True)
    old=''.join(lines[node.lineno-1:node.end_lineno])
    for marker in must_contain:
        if marker not in old: raise ValueError('unknown '+name+' method lacks '+marker)
    fresh=textwrap.indent(textwrap.dedent(code).strip()+'\n','    ')
    new=''.join(lines[:node.lineno-1])+fresh+''.join(lines[node.end_lineno:])
    ast.parse(new)
    return new

MUX_POINT_CB='''
def point_cb(self, msg):
    with self._guard_lock:
        self.point_msg = msg
        self.point_time = rospy.Time.now()
'''
MUX_EGO_CB='''
def ego_cb(self, msg):
    with self._guard_lock:
        self.ego_msg = msg
        self.ego_time = rospy.Time.now()
'''
MUX_POINT='''
def use_point(self, req):
    return self._guard_select('POINT', self.point_msg, self.point_time)
'''
MUX_EGO='''
def use_ego(self, req):
    return self._guard_select('EGO', self.ego_msg, self.ego_time)
'''
MUX_EXTRA='''
def _guard_check(self, cmd):
    if self._guard_last_msg is None or self._guard_last_time is None:
        return 'no_published_reference'
    age = (rospy.Time.now()-self._guard_last_time).to_sec()
    if not 0.0 <= age <= 0.30:
        return 'reference_stale'
    return check(self._guard_last_msg, cmd)

def _guard_select(self, desired, cmd, timestamp):
    with self._guard_lock:
        if not self.fresh(timestamp):
            return TriggerResponse(False, desired+' source missing/stale')
        if desired != self.source:
            reason = self._guard_check(cmd)
            if reason:
                rospy.logerr('[HANDOFF_GUARD] %s refused: %s',desired,reason)
                return TriggerResponse(False, 'continuity veto: '+reason)
        if desired != self.source:
            self._guard_window_until = rospy.Time.now() + rospy.Duration(0.25)
        self.source = desired
        return TriggerResponse(True, 'Trajectory source -> '+desired)
'''
MUX_TIMER='''
def timer_cb(self, event):
    with self._guard_lock:
        cmd, timestamp = ((self.point_msg,self.point_time) if self.source=='POINT'
                         else (self.ego_msg,self.ego_time))
        if not self.fresh(timestamp):
            self.status_pub.publish(String(data=self.source+'_STALE'))
            return
        # Ordinary in-source trajectory updates (including TAKEOFF init)
        # are *not* clamped. The guard applies at source handoff only.
        reason = check(None, cmd)
        if (not reason and self._guard_window_until is not None and
                rospy.Time.now() <= self._guard_window_until):
            reason = self._guard_check(cmd)
        if reason:
            rospy.logerr_throttle(1.0,'[HANDOFF_GUARD] %s command veto: %s' % (
                self.source,reason))
            self.status_pub.publish(String(data=self.source+'_GUARD_REJECTED'))
            return
        self.pub.publish(cmd)
        self._guard_last_msg=copy.deepcopy(cmd)
        self._guard_last_time=rospy.Time.now()
        self.status_pub.publish(String(data=self.source+'_ACTIVE'))
'''

def patch_mux(src):
    if 'yaw_guard_core import check' in src: raise ValueError('already patched mux')
    if 'import rospy\n' not in src or 'self.source = "POINT"' not in src:
        raise ValueError('unknown MUX import/init')
    for topic in ('/contest1/ego_uav_interface/point_setpoint',
                  '/contest1/ego_uav_interface/ego_setpoint',
                  '/contest1/ego_uav_interface/trajectory_setpoint'):
        if topic not in src: raise ValueError('unknown MUX topic: '+topic)
    src=src.replace('import rospy\n','import rospy\nimport threading\nimport copy\nfrom yaw_guard_core import check\n',1)
    src=src.replace('self.source = "POINT"','''self.source = "POINT"
        self._guard_lock = threading.RLock()
        self._guard_last_msg = None
        self._guard_last_time = None
        self._guard_window_until = None''',1)
    src=replace_method(src,'TrajectoryMux','point_cb',MUX_POINT_CB,('self.point_msg','self.point_time'))
    src=replace_method(src,'TrajectoryMux','ego_cb',MUX_EGO_CB,('self.ego_msg','self.ego_time'))
    src=replace_method(src,'TrajectoryMux','use_point',MUX_POINT,('POINT','self.source'))
    src=replace_method(src,'TrajectoryMux','use_ego',MUX_EGO,('EGO','self.source'))
    src=replace_method(src,'TrajectoryMux','timer_cb',MUX_TIMER,('self.pub.publish','self.status_pub.publish'))
    node=locate(src,'TrajectoryMux','use_point'); lines=src.splitlines(keepends=True)
    extra=textwrap.indent(textwrap.dedent(MUX_EXTRA).strip()+'\n\n','    ')
    src=''.join(lines[:node.lineno-1])+extra+''.join(lines[node.lineno-1:])
    ast.parse(src)
    return src

MISSION_RESTORE='''
def restore_point_hold(self):
    # Must disable EGO output before updating POINT. If any step fails,
    # leave trajectory_interface in physical-position HOLD, not stale EGO.
    rospy.logwarn('[HANDOFF_GUARD] Begin EGO -> POINT HOLD rejoin')
    self.call_service('/contest1/ego_uav_interface/disable_trajectory')
    reset_started = rospy.Time.now()
    self.call_service('/contest1/ego_uav_interface/abort_trajectory')
    deadline=time.time()+2.0
    last_good_stamp=None
    good=0
    while not rospy.is_shutdown() and time.time()<deadline:
        if (self.state is None or not self.state.connected or not self.state.armed
                or self.state.mode!='OFFBOARD'):
            raise RuntimeError('POINT rejoin aborted: not ARMED OFFBOARD')
        cmd=self.point_cmd
        stamp=self.point_cmd_time
        if (cmd is not None and stamp is not None and self.odom is not None
            and stamp>reset_started
            and (rospy.Time.now()-stamp).to_sec()<0.15
            and (cmd.type_mask & (PositionTarget.IGNORE_PX |
                  PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                  PositionTarget.IGNORE_YAW))==0):
            p=self.odom.pose.pose.position
            dist=math.sqrt((cmd.position.x-p.x)**2 +
                           (cmd.position.y-p.y)**2 +
                           (cmd.position.z-p.z)**2)
            dyaw=abs(wrap(cmd.yaw-self.current_yaw()))
            if (math.isfinite(dist) and math.isfinite(dyaw) and
                dist<=0.12 and dyaw<=math.radians(12.0)):
                if stamp!=last_good_stamp:
                    good+=1
                    last_good_stamp=stamp
            else:
                good=0
                last_good_stamp=None
        else:
            good=0
            last_good_stamp=None
        if good>=3: break
        rospy.sleep(0.02)
    if good<3:
        raise RuntimeError('POINT rejoin veto: fresh current HOLD was not verified')
    self.call_service('/contest1/ego_uav_interface/use_point')
    deadline=time.time()+0.6
    while not rospy.is_shutdown() and time.time()<deadline:
        if self.mux_status=='POINT_ACTIVE': break
        rospy.sleep(0.02)
    if self.mux_status!='POINT_ACTIVE':
        raise RuntimeError('POINT rejoin veto: MUX POINT_ACTIVE not confirmed')
    self.call_service('/contest1/ego_uav_interface/enable_trajectory')
    rospy.logwarn('[HANDOFF_GUARD] POINT current HOLD confirmed')
'''

def patch_mission(src):
    if 'POINT rejoin veto:' in src: raise ValueError('already patched mission')
    if 'from mavros_msgs.msg import State, PositionTarget' not in src:
        raise ValueError('unknown mission PositionTarget import')
    if 'def point_cmd_cb' not in src or 'self.point_cmd_time' not in src:
        raise ValueError('unknown POINT callback')
    for topic in ('/contest1/ego_uav_interface/disable_trajectory',
                  '/contest1/ego_uav_interface/abort_trajectory',
                  '/contest1/ego_uav_interface/use_point',
                  '/contest1/ego_uav_interface/enable_trajectory'):
        if topic not in src: raise ValueError('unknown mission service: '+topic)
    return replace_method(src,'TangentPreview','restore_point_hold',MISSION_RESTORE,
                          ('use_point','enable_trajectory'))


def ensure_known_methods(source, cls, names, fixture):
    """Refuse to overwrite any newer, unknown safety logic in these methods."""
    historic=fixture.read_text(encoding='utf-8')
    for name in names:
        current_method=locate(source,cls,name)
        historic_method=locate(historic,cls,name)
        if ast.dump(current_method,include_attributes=False) != ast.dump(
                historic_method,include_attributes=False):
            raise ValueError('Live %s.%s has changed since historical snapshot; '
                             'manual merge required, no automated replacement' % (cls,name))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mux',type=Path,required=True)
    parser.add_argument('--mission',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    a=parser.parse_args()
    allowed=(Path.home()/'contest1_uav/dev/projects').resolve()
    out=a.out.expanduser().resolve()
    if out==allowed or allowed not in out.parents or out.exists():
        raise SystemExit('[STOP] New --out must be inside '+str(allowed))
    if not a.mux.is_file() or not a.mission.is_file() or a.mux.resolve()==a.mission.resolve():
        raise SystemExit('[STOP] Input missing or duplicated')
    old_mux=a.mux.read_text(encoding='utf-8')
    old_mission=a.mission.read_text(encoding='utf-8')
    # Important: verify patched methods have NOT acquired newer safety logic.
    # This historical snapshot is only a fingerprint, NEVER deployed as code.
    try:
        fixture=Path(__file__).resolve().parent/'fixtures'
        ensure_known_methods(old_mux,'TrajectoryMux',
            ['point_cb','ego_cb','use_point','use_ego','timer_cb'],
            fixture/'trajectory_mux_reference_20261015.py')
        ensure_known_methods(old_mission,'TangentPreview',
            ['restore_point_hold'],
            fixture/'uav2_demo5_waypoints_reference_20261015.py')
        new_mux=patch_mux(old_mux)
        new_mission=patch_mission(old_mission)
        compile(new_mux,'<mux_dev>','exec')
        compile(new_mission,'<mission_dev>','exec')
    except Exception as exc:
        raise SystemExit('[STOP] Source mismatch, no changes: '+str(exc))
    out.mkdir(parents=True)
    (out/'originals').mkdir()
    (out/'candidates').mkdir()
    (out/'originals/trajectory_mux.py').write_text(old_mux,encoding='utf-8')
    (out/'originals/uav2_demo5_waypoints.py').write_text(old_mission,encoding='utf-8')
    (out/'candidates/trajectory_mux.py').write_text(new_mux,encoding='utf-8')
    (out/'candidates/uav2_demo5_waypoints.py').write_text(new_mission,encoding='utf-8')
    shutil.copyfile(Path(__file__).with_name('yaw_guard_core.py'),
                    out/'candidates/yaw_guard_core.py')
    diff=[]
    for filename,before,after in [('trajectory_mux.py',old_mux,new_mux),
                                   ('uav2_demo5_waypoints.py',old_mission,new_mission)]:
        diff.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),
                    fromfile=filename+'.original',tofile=filename+'.dev'))
    (out/'patch_review.diff').write_text(''.join(diff),encoding='utf-8')
    (out/'sha256_original.txt').write_text(
        'mux '+hashlib.sha256(old_mux.encode()).hexdigest()+'\n'+
        'mission '+hashlib.sha256(old_mission.encode()).hexdigest()+'\n',encoding='utf-8')
    print('[OK] Created DEV copies:',out)
    print('[OK] Candidate scripts and diff syntax verified')
    print('[REVIEW]',out/'patch_review.diff')
    print('[UNCHANGED] Runtime and Demo9 remain untouched')

if __name__=='__main__': main()
