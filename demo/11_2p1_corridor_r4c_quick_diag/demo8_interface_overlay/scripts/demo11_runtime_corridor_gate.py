#!/usr/bin/env python3
"""Demo11 R3 read-only geometry/continuity admission checks.

No ROS/MAVROS dependency; all checks fail closed and NEVER clip the command.
Used by the *private* final TrajectoryInterface overlay only.
"""
import math
import sys
import time
from pathlib import Path
import yaml

DEMO_ROOT = Path(__file__).resolve().parents[2]
if str(DEMO_ROOT) not in sys.path:
    sys.path.insert(0, str(DEMO_ROOT))
from demo11_corridor import load_corridor, CorridorViolation
from demo5_config import load_config


class GateError(ValueError):
    pass


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(float(value)):
        raise GateError('%s_not_finite' % label)
    return float(value)


class R3CorridorGate:
    def __init__(self, corridor_file, mission_file, session_file, started_at=None):
        self.corridor = load_corridor(corridor_file)
        self.expected_plan = load_config(mission_file)
        self.session_file = Path(session_file)
        self.started_at = time.time() if started_at is None else float(started_at)
        self.frame = None
        self.session_mtime = None

    @property
    def bound(self):
        return self.frame is not None

    def bind_session(self):
        """Bind exactly one fresh Frame Guard session after its ready signal.

        Must be called from the ROS ready subscriber, not the 50-Hz control timer.
        Do not let a leftover session YAML from an old flight be used.
        """
        if self.bound:
            return
        try:
            meta = self.session_file.stat()
            if meta.st_mtime < self.started_at - 0.5:
                raise GateError('stale_session_file')
            data = yaml.safe_load(self.session_file.read_text())
        except (OSError, yaml.YAMLError) as exc:
            raise GateError('session_unavailable:%s' % exc)
        if not isinstance(data, dict) or data.get('demo5_plan_snapshot') != self.expected_plan:
            raise GateError('session_waypoints_or_fence_changed')
        if data.get('demo_fence') != self.expected_plan['fence']:
            raise GateError('session_fence_mismatch')
        try:
            ori = data['origin_world']
            fw = data['forward_world']
            left = data['left_world']
            o = tuple(_finite(ori[k], 'origin_'+k) for k in ('x', 'y', 'z'))
            f = tuple(_finite(fw[k], 'forward_'+k) for k in ('x', 'y'))
            l = tuple(_finite(left[k], 'left_'+k) for k in ('x', 'y'))
        except (KeyError, TypeError) as exc:
            raise GateError('invalid_session_frame:%s' % exc)
        if abs(math.hypot(*f)-1.) > 0.005 or abs(math.hypot(*l)-1.) > 0.005:
            raise GateError('nonunit_session_frame')
        if abs(f[0]*l[0] + f[1]*l[1]) > 0.005:
            raise GateError('nonorthogonal_session_frame')
        if abs(f[0]*l[1] - f[1]*l[0] - 1.) > 0.005:
            raise GateError('inverted_session_frame')
        self.frame = (o, f, l)
        self.session_mtime = meta.st_mtime

    def demo(self, x, y):
        if not self.bound:
            raise GateError('session_not_bound')
        x, y = _finite(x,'x'), _finite(y,'y')
        origin, f, l = self.frame
        dx, dy = x-origin[0], y-origin[1]
        return f[0]*dx+f[1]*dy, l[0]*dx+l[1]*dy

    def check_xy(self, x, y, label='command'):
        f, l = self.demo(x, y)
        try:
            self.corridor.check_center(f, l, label)
        except CorridorViolation as exc:
            raise GateError(str(exc))
        return f,l

    def check_target(self, cmd):
        """Validate PositionTarget in world ENU (MAVROS converts on publish)."""
        if int(cmd.coordinate_frame) != 1:
            raise GateError('wrong_coordinate_frame')
        if int(cmd.type_mask) & (1|2|4|1024):
            raise GateError('position_or_yaw_ignored')
        for label, value in (('z',cmd.position.z),('yaw',cmd.yaw)):
            _finite(value,label)
        return self.check_xy(cmd.position.x, cmd.position.y)

    @staticmethod
    def check_continuity(prev, cmd):
        """Last-hop V4-equivalent check on consecutive trajectory commands."""
        if prev is None:
            return
        for candidate in (prev,cmd):
            if int(candidate.coordinate_frame) != 1 or int(candidate.type_mask) & (1|2|4|1024):
                raise GateError('continuity_invalid_fields')
            for n, v in (('x',candidate.position.x),('y',candidate.position.y),
                         ('z',candidate.position.z),('yaw',candidate.yaw)):
                _finite(v,n)
        d = math.sqrt(sum((float(getattr(cmd.position,k))-float(getattr(prev.position,k)))**2
                          for k in ('x','y','z')))
        yaw_err = abs(math.atan2(math.sin(cmd.yaw-prev.yaw),math.cos(cmd.yaw-prev.yaw)))
        if d > 0.25:
            raise GateError('replan_position_step_%.3fm' % d)
        if yaw_err > math.radians(12.):
            raise GateError('replan_yaw_step_%.2fdeg' % math.degrees(yaw_err))
