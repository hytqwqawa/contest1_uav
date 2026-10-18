#!/usr/bin/env python3
"""Pure, passive Demo9 V2 phase/quality assessment; NEVER controls the UAV.

Accepting a phase means the data support an observation, not permission to fly,
land, change mode, or disarm. Python 3.8 compatible, no ROS dependency.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Thresholds:
    height_m: float = 0.80
    altitude_error_m: float = 0.06
    vertical_speed_max_mps: float = 0.10
    stable_for_s: float = 0.80
    post_hold_timeout_s: float = 15.0
    odom_max_age_s: float = 0.25
    point_max_age_s: float = 0.50
    interface_max_age_s: float = 0.50
    health_max_age_s: float = 0.80
    mission_max_age_s: float = 0.80
    state_max_age_s: float = 2.50
    setpoint_max_age_s: float = 0.25
    max_horizontal_tracking_error_m: float = 0.35
    range_max_age_s: float = 0.30


@dataclass(frozen=True)
class FlightSample:
    t: float
    z: float
    vz: float
    x: float
    y: float
    desired_x: float
    desired_y: float
    desired_z: float
    point_status: str
    interface_status: str
    px4_mode: str
    connected: bool
    armed: bool
    healthy: bool
    mission_allowed: bool
    ages: dict


class TakeoffPhaseMonitor:
    """Observe a protected POINT takeoff without commanding anything.

    Important: the already-proven takeoff gate ALSO waits for real altitude
    within +/-0.06m, abs(vz)<=0.10m/s for 0.8s. This monitor adds
    independently inspectable status and fault reasons; it does not replace it.
    """
    def __init__(self, ground_z, thresholds=None):
        self.ground_z = float(ground_z)
        self.params = thresholds or Thresholds()
        self.state = 'WAIT_COMMAND'
        self.started_at = None
        self.point_hold_at = None
        self.stable_since = None
        self.accepted_at = None
        self.first_10cm_at = None
        self.first_50cm_at = None
        self.first_70cm_at = None
        self.fault = None
        self.max_xy_tracking = 0.0
        self.max_xy_from_start = 0.0
        self.start_xy = None

    def begin(self, t):
        if self.started_at is not None:
            raise ValueError('takeoff monitor was already started')
        self.started_at = float(t)
        self.state = 'TRAJECTORY_RUNNING'

    def _failure(self, reason):
        self.state = 'FAULT'
        self.fault = reason
        self.stable_since = None
        return self.state

    def update(self, s):
        if self.state in ('FAULT', 'ALTITUDE_CONFIRMED'):
            return self.state
        if self.started_at is None:
            return 'WAIT_COMMAND'
        p = self.params
        if not all(math.isfinite(v) for v in (
                s.t, s.z, s.vz, s.x, s.y,
                s.desired_x, s.desired_y, s.desired_z)):
            return self._failure('NON_FINITE_ODOMETRY_OR_SETPOINT')
        if s.t < self.started_at:
            return self._failure('NONMONOTONIC_SAMPLE_TIME')
        requirements = (
            ('odom', p.odom_max_age_s),
            ('point', p.point_max_age_s),
            ('interface', p.interface_max_age_s),
            ('health', p.health_max_age_s),
            ('mission', p.mission_max_age_s),
            ('state', p.state_max_age_s),
            ('setpoint', p.setpoint_max_age_s),
        )
        for key, max_age in requirements:
            age = s.ages.get(key, float('inf'))
            if not math.isfinite(age) or age < 0.0 or age > max_age:
                return self._failure('STALE_' + key.upper())
        if not s.connected or not s.armed or s.px4_mode != 'OFFBOARD':
            return self._failure('PX4_LINK_ARM_OR_MODE')
        if not s.healthy or not s.mission_allowed:
            return self._failure('HEALTH_OR_CH7')
        if s.interface_status != 'TRAJECTORY_ACTIVE':
            return self._failure('INTERFACE_NOT_ACTIVE')

        d_xy = math.hypot(s.x - s.desired_x, s.y - s.desired_y)
        self.max_xy_tracking = max(self.max_xy_tracking, d_xy)
        if self.start_xy is None:
            self.start_xy = (s.x, s.y)
        self.max_xy_from_start = max(
            self.max_xy_from_start,
            math.hypot(s.x - self.start_xy[0], s.y - self.start_xy[1]))
        if d_xy > p.max_horizontal_tracking_error_m:
            return self._failure('XY_TRACKING_ENVELOPE')

        delta_z = s.z - self.ground_z
        for limit, attr in ((0.10, 'first_10cm_at'),
                            (0.50, 'first_50cm_at'),
                            (0.70, 'first_70cm_at')):
            if delta_z >= limit and getattr(self, attr) is None:
                setattr(self, attr, s.t)

        if s.point_status == 'AIRBORNE_HOLD':
            if self.point_hold_at is None:
                self.point_hold_at = s.t
        elif s.point_status != 'TAKEOFF_TRAJECTORY':
            return self._failure('UNEXPECTED_POINT_STATE')

        if self.point_hold_at is None:
            self.stable_since = None
            self.state = 'TRAJECTORY_RUNNING'
            return self.state
        if s.t - self.point_hold_at > p.post_hold_timeout_s:
            return self._failure('ALTITUDE_SETTLE_TIMEOUT')

        within_target = (abs(delta_z - p.height_m) <= p.altitude_error_m)
        stopped = (abs(s.vz) <= p.vertical_speed_max_mps)
        setpoint_aligned = (abs((s.desired_z - self.ground_z) - p.height_m)
                            <= p.altitude_error_m)
        if within_target and stopped and setpoint_aligned:
            if self.stable_since is None:
                self.stable_since = s.t
            if s.t - self.stable_since >= p.stable_for_s:
                self.accepted_at = s.t
                self.state = 'ALTITUDE_CONFIRMED'
                return self.state
            self.state = 'SETTLING'
        else:
            self.stable_since = None
            self.state = 'WAIT_REAL_ALTITUDE'
        return self.state


@dataclass(frozen=True)
class RangeResult:
    state: str
    value_m: float
    detail: str
    can_authorize_touchdown: bool = False


def assess_range(value, min_range, max_range, age_s, max_age_s=0.30):
    """Range metadata must be respected. NEVER authorizes motor-stop/contact."""
    vals = (value, min_range, max_range, age_s)
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
        return RangeResult('INVALID', float('nan'), 'non-finite/missing fields')
    if min_range <= 0 or max_range <= min_range:
        return RangeResult('INVALID_METADATA', value, 'bad min_range/max_range')
    if age_s < 0 or age_s > max_age_s:
        return RangeResult('STALE', value, 'range age exceeds threshold')
    if value < min_range:
        return RangeResult('BELOW_SPEC', value, 'reading below advertised min_range')
    if value > max_range:
        return RangeResult('ABOVE_SPEC', value, 'reading above advertised max_range')
    return RangeResult('IN_SPEC', value,
                       'range plausible, NOT verified contact clearance')


def touchdown_observation_policy(range_result, px4_on_ground, low_vz):
    """V2 pilot-review policy: never return an automatic stop authorization.

    A valid range alone cannot establish contact; an out-of-spec range cannot
    support a touchdown claim. This is a diagnostic result, not a control API.
    """
    if range_result.state != 'IN_SPEC':
        return 'BLOCK_CONTACT_INFERENCE_INVALID_RANGE'
    if px4_on_ground and low_vz:
        return 'REVIEW_PX4_LANDED_AND_CONTACT_INDEPENDENTLY'
    return 'NO_CONTACT_EVIDENCE'
