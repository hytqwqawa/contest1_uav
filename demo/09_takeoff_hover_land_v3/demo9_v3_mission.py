#!/usr/bin/env python3
"""Demo9 V3 flight-candidate: guarded takeoff, verified hover, PILOT landing.

No direct setpoint publisher or service client here. The already-flown V1
physical-pilot takeoff gate remains intact. V3 *does not* command an
automatic landing because the observed ranging topic advertises >=0.40m
minimum distance while the V1 final approach relies on 0.08-0.10m values.

V3 flight path: DISARM/POSCTL -> physical pilot authorization -> V1 TAKEOFF ->
altitude-verified HOLD -> 6s POINT HOLD -> pilot CH7 LOW + CH6 POSCTL ->
manual pilot landing -> physical CH5 DISARM / PX4 ground-confirmed auto-disarm.
"""
import math
import statistics
import subprocess
import time
from pathlib import Path

import rospy
import yaml
from mavros_msgs.msg import ExtendedState, PositionTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String

from demo9_v2_phase import assess_range

ROOT = Path(__file__).resolve().parent


class UnsafeFlightState(RuntimeError):
    pass


class Demo9V3:
    def __init__(self):
        settings = yaml.safe_load((ROOT / 'demo9_settings.yaml').read_text())
        self.hover_seconds = float(settings['hover_seconds'])
        self.max_xy = float(settings['max_xy_displacement_m'])
        self.value = {}
        self.stamp = {}
        self.ground_xyz = None
        self.max_xy_during_hover = 0.0
        self.max_xy_error_during_hover = 0.0
        self.max_range_log = 0.0

        subjects = (
            ('state', '/contest1/mavros/state', State),
            ('odom', '/contest1/mavros/local_position/odom', Odometry),
            ('extended', '/contest1/mavros/extended_state', ExtendedState),
            ('range', '/contest1/mavros/distance_sensor/hrlv_ez4_pub', Range),
            ('healthy', '/contest1/ego_uav_interface/flight_healthy', Bool),
            ('stream', '/contest1/ego_uav_interface/offboard_stream_ready', Bool),
            ('mission', '/contest1/ego_uav_interface/rc_mission_allowed', Bool),
            ('point', '/contest1/ego_uav_interface/point_traj_status', String),
            ('mux', '/contest1/ego_uav_interface/trajectory_mux_status', String),
            ('interface', '/contest1/ego_uav_interface/trajectory_interface_status', String),
            ('point_cmd', '/contest1/ego_uav_interface/point_setpoint', PositionTarget),
        )
        self.subs = [rospy.Subscriber(topic, cls, self.receive,
                                     callback_args=key, queue_size=30)
                     for key, topic, cls in subjects]

    def receive(self, msg, key):
        self.value[key] = msg
        self.stamp[key] = time.monotonic()

    def data(self, key):
        msg = self.value.get(key)
        return getattr(msg, 'data', None) if msg is not None else None

    def fresh(self, key, limit):
        return key in self.value and 0 <= time.monotonic()-self.stamp.get(key, -1e12) < limit

    def require_airborne(self):
        s = self.value.get('state')
        if not self.fresh('state', 2.5) or s is None or not s.connected:
            raise UnsafeFlightState('PX4 state unavailable')
        if not s.armed or s.mode != 'OFFBOARD':
            raise UnsafeFlightState('PX4 no longer ARMED/OFFBOARD')
        for key, age in (('healthy', .8), ('mission', .8)):
            if not self.fresh(key, age) or self.data(key) is not True:
                raise UnsafeFlightState('flight health/CH7 missing: ' + key)
        for key, age, expected in (
            ('point', .5, 'AIRBORNE_HOLD'),
            ('mux', .5, 'POINT_ACTIVE'),
            ('interface', .5, 'TRAJECTORY_ACTIVE'),
        ):
            if not self.fresh(key, age) or self.data(key) != expected:
                raise UnsafeFlightState('%s not active: %s' % (key, self.data(key)))
        if not self.fresh('odom', .25) or not self.fresh('point_cmd', .25):
            raise UnsafeFlightState('ODOM/POINT setpoint stale')
        odom = self.value['odom'].pose.pose.position
        cmd = self.value['point_cmd'].position
        for v in (odom.x, odom.y, odom.z, cmd.x, cmd.y, cmd.z,
                  self.value['odom'].twist.twist.linear.z):
            if not math.isfinite(v):
                raise UnsafeFlightState('Non-finite odometry/setpoint')

    def check_ground_start(self):
        stop = time.monotonic() + 15.0
        keys = ('state', 'odom', 'healthy', 'stream', 'mission', 'point',
                'mux', 'interface', 'extended', 'range')
        while not rospy.is_shutdown() and time.monotonic() < stop:
            if all(k in self.value for k in keys):
                break
            rospy.sleep(0.05)
        if not all(k in self.value for k in keys):
            raise UnsafeFlightState('Required core/backend topics missing')
        state = self.value['state']
        if not self.fresh('state', 2.5) or not state.connected or state.armed or state.mode != 'POSCTL':
            raise UnsafeFlightState('Initial state must be connected, DISARM, POSCTL')
        if not self.fresh('extended', 2.0) or self.value['extended'].landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            raise UnsafeFlightState('PX4 has not confirmed ON_GROUND')
        if not self.fresh('healthy', .8) or self.data('healthy') is not True:
            raise UnsafeFlightState('Ground health not ready')
        if not self.fresh('stream', .8) or self.data('stream') is not True:
            raise UnsafeFlightState('OFFBOARD stream not ready')
        if not self.fresh('mission', .8) or self.data('mission') is not False:
            raise UnsafeFlightState('CH7 must initially be LOW')
        if (self.data('point') not in ('HOLD_GROUND', 'LANDED_HOLD') or
                self.data('mux') != 'POINT_ACTIVE' or
                self.data('interface') != 'HOLD_RC_BLOCKED'):
            raise UnsafeFlightState('POINT/MUX/interface ground safety latch not ready')

        # Robust, observed-only ground reference. No localization resets.
        samples = []
        end = time.monotonic()+1.2
        last_stamp = -1
        while time.monotonic() < end and not rospy.is_shutdown():
            state = self.value.get('state')
            if (state is None or state.armed or state.mode != 'POSCTL' or
                    self.data('mission') is not False):
                raise UnsafeFlightState('Initial pilot gate changed during ground sampling')
            if self.fresh('odom', .25) and self.stamp['odom'] != last_stamp:
                p = self.value['odom'].pose.pose.position
                if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                    raise UnsafeFlightState('Bad ground position estimate')
                samples.append((p.x, p.y, p.z))
                last_stamp = self.stamp['odom']
            rospy.sleep(.02)
        if len(samples) < 18:
            raise UnsafeFlightState('Insufficient fresh ground odometry samples')
        ref = tuple(statistics.median(p[i] for p in samples) for i in range(3))
        jitter_xy = max(math.hypot(p[0]-ref[0], p[1]-ref[1]) for p in samples)
        jitter_z = max(abs(p[2]-ref[2]) for p in samples)
        if jitter_xy > .08 or jitter_z > .05:
            raise UnsafeFlightState('Ground position estimate unstable')
        self.ground_xyz = ref
        print('[GROUND REFERENCE] XYZ=(%.3f %.3f %.3f) n=%d jitter_xy=%.3f jitter_z=%.3f' %
              (*ref, len(samples), jitter_xy, jitter_z), flush=True)
        print('[OK] DISARM/POSCTL/CH7 LOW and landing detector ON_GROUND confirmed.', flush=True)
        self.print_range_quality(prefix='[GROUND RANGE]')

    def print_range_quality(self, prefix='[RANGE]'):
        msg = self.value.get('range')
        if msg is None or not self.fresh('range', .30):
            print(prefix, 'missing/stale; not used for touchdown authorization', flush=True)
            return
        age = time.monotonic() - self.stamp['range']
        r = assess_range(msg.range, msg.min_range, msg.max_range, age)
        print('%s value=%.3fm spec=[%.3f,%.3f] %s (observation only)' %
              (prefix, msg.range, msg.min_range, msg.max_range, r.state), flush=True)

    def call_v1_takeoff(self):
        print('[TAKEOFF] Calling unchanged V1 pilot gate (CH5 physical ARM only).', flush=True)
        code = subprocess.call(['python3', '-B', '-u', str(ROOT/'uav2_5m_takeoff_gate.py')])
        if code != 0:
            raise UnsafeFlightState('V1 takeoff gate failed: %d' % code)

    def verify_hover_altitude(self):
        # Independent confirmation; V1 gate already waits for physical settle.
        stable_at = None
        deadline = time.monotonic() + 5.0
        print('[VERIFY] POINT trajectory completion alone does not mean takeoff complete.', flush=True)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.require_airborne()
            p = self.value['odom'].pose.pose.position
            vz = self.value['odom'].twist.twist.linear.z
            cmd = self.value['point_cmd'].position
            dz = p.z-self.ground_xyz[2]
            within = (abs(dz-.80) <= .06 and abs(vz) <= .10 and
                      abs(cmd.z-(self.ground_xyz[2]+.80)) <= .06)
            now = time.monotonic()
            if within:
                if stable_at is None:
                    stable_at = now
                if now-stable_at >= .80:
                    print('[OK] Altitude verified at dz=%.3fm vz=%.3fm/s' % (dz, vz), flush=True)
                    return
            else:
                stable_at = None
            rospy.sleep(.05)
        raise UnsafeFlightState('Independent hover-altitude verification timed out')

    def hover(self):
        t_end = time.monotonic()+self.hover_seconds
        print('[HOVER] Verified POINT HOLD for %.1f seconds' % self.hover_seconds, flush=True)
        next_print = 0.
        while not rospy.is_shutdown() and time.monotonic() < t_end:
            self.require_airborne()
            p = self.value['odom'].pose.pose.position
            cmd = self.value['point_cmd'].position
            d0 = math.hypot(p.x-self.ground_xyz[0], p.y-self.ground_xyz[1])
            de = math.hypot(p.x-cmd.x, p.y-cmd.y)
            self.max_xy_during_hover = max(self.max_xy_during_hover, d0)
            self.max_xy_error_during_hover = max(self.max_xy_error_during_hover, de)
            if d0 > self.max_xy:
                raise UnsafeFlightState('XY displacement exceeds %.2fm diagnostic envelope' % self.max_xy)
            now = time.monotonic()
            if now > next_print:
                print('[HOVER] from_ground_XY=%.3fm tracking_XY=%.3fm relZ=%.3fm' %
                      (d0, de, p.z-self.ground_xyz[2]), flush=True)
                next_print = now+.5
            rospy.sleep(.05)
        print('[HOVER DONE] max_ground_XY=%.3fm max_tracking_XY=%.3fm' %
              (self.max_xy_during_hover, self.max_xy_error_during_hover), flush=True)

    def pilot_landing(self):
        print('\n[PILOT LANDING REQUIRED] The last 10cm range is outside sensor specification.', flush=True)
        print('[PILOT] CH7 -> LOW; CH6 -> POSCTL using original verified RC procedure.', flush=True)
        print('[PILOT] Confirm actual PX4 POSCTL, then manually land and physically DISARM (CH5).', flush=True)
        print('[PILOT] V3 does not request a mode change, LAND, ARM or DISARM.', flush=True)
        last_print = 0.
        got_posctl = False
        while not rospy.is_shutdown():
            s = self.value.get('state')
            if self.fresh('state', 2.5) and s is not None and s.connected:
                if not s.armed:
                    ext = self.value.get('extended')
                    confirmed = (self.fresh('extended', 2.0) and ext is not None and
                                 ext.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
                    if not confirmed:
                        raise UnsafeFlightState(
                            'PX4 DISARMED but ON_GROUND not confirmed; pilot must visually check vehicle')
                    print('[DONE] PX4 DISARMED and reports ON_GROUND. Pilot verify physical contact.',
                          flush=True)
                    return
                if s.mode == 'POSCTL':
                    if not got_posctl:
                        print('[PILOT] POSCTL confirmed; descend and land under pilot control.', flush=True)
                        got_posctl = True
                elif got_posctl:
                    print('[WARN] PX4 unexpectedly left POSCTL; pilot should follow emergency procedure.', flush=True)
                    got_posctl = False
            now = time.monotonic()
            if now-last_print >= 2.0:
                print('[WAIT] PX4 mode=%s armed=%s | manual landing | monitor continues' %
                      ((s.mode if s else '?'), (s.armed if s else '?')), flush=True)
                self.print_range_quality()
                last_print = now
            rospy.sleep(.1)
        raise UnsafeFlightState('ROS shutdown during pilot landing')

    def run(self):
        try:
            self.check_ground_start()
            print('[PLAN] Protected V1 vertical TAKEOFF -> verified 6s HOLD -> PILOT LANDING', flush=True)
            self.call_v1_takeoff()
            self.verify_hover_altitude()
            self.hover()
            self.pilot_landing()
            return 0
        except (Exception, KeyboardInterrupt) as exc:
            print('\n[ABORT] V3: %s' % exc, flush=True)
            s = self.value.get('state')
            if s is not None and s.armed:
                print('[PILOT] If airborne: CH7 LOW, CH6 POSCTL, land manually, physical CH5 DISARM.', flush=True)
                print('[PILOT] Keep Core/Backend running; Ctrl+C is not a flight safety action.', flush=True)
                try:
                    self.pilot_landing()
                except Exception as exc2:
                    print('[WARN] Pilot monitoring ended: %s' % exc2, flush=True)
            return 1


def main():
    rospy.init_node('demo9_v3_takeoff_hover_pilot_land', anonymous=False)
    return Demo9V3().run()


if __name__ == '__main__':
    raise SystemExit(main())
