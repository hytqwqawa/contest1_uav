#!/usr/bin/env python3
"""Demo6 V2: proven POINT takeoff -> F0.5 L-0.5 -> QR -> HOME -> LAND.

The proven Demo5 POINT leg and landing implementations are unchanged.
This subclass inserts a QR scan after leg 1, then calls the same existing POINT
leg for homecoming. It does not arm or change PX4 mode on its own.
"""
import hashlib
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rospy
from std_msgs.msg import String

from demo5_config import demo_to_world
from demo6_config import load_demo6
from demo6_scan_policy import ScanPolicy
from uav2_demo5_waypoints import TangentPreview

TOPIC = '/contest1/demo6/qr/live_result'


class Demo6QrReturnHome(TangentPreview):
    def __init__(self):
        super().__init__()
        root = Path(__file__).resolve().parent
        expected_hash = os.environ.get('DEMO6_QR_HASH', '')
        actual_hash = hashlib.sha256((root / 'demo6_qr_mission.yaml').read_bytes()).hexdigest()
        if not expected_hash or expected_hash != actual_hash:
            raise RuntimeError('Demo6 YAML changed or approved start_demo6.sh not used')
        self.demo6 = load_demo6(root)
        self.qr_proc = None
        self.qr_data = ''
        self.qr_last_received = None
        self.qr_ok = False
        self.qr_reason = 'QR scan not started'
        self.scan_finished = False
        self.return_started = False
        self.return_complete = False
        self.landing_complete = False
        self.qr_sub = rospy.Subscriber(TOPIC, String, self.on_qr, queue_size=10)

    def on_qr(self, msg):
        # This publisher is launched only AFTER physical POINT arrival; no latch.
        self.qr_data = msg.data
        self.qr_last_received = time.monotonic()

    def require_point_hold(self, f, l):
        if (self.state is None or not self.state.connected or
                not self.state.armed or self.state.mode != 'OFFBOARD' or
                not self.flight_healthy or not self.mission_allowed or
                not self.demo_safe or self.mux_status != 'POINT_ACTIVE' or
                self.point_status != 'AIRBORNE_HOLD' or self.odom is None):
            raise RuntimeError(
                'DEMO6 POINT HOLD safety lost; pilot CH7 LOW / CH6 POSCTL takeover')
        if (self.point_cmd is None or self.point_cmd_time is None or
                (rospy.Time.now() - self.point_cmd_time).to_sec() >= 0.25):
            raise RuntimeError('DEMO6 POINT setpoint missing/stale')
        p = self.odom.pose.pose.position
        gx, gy, gz = demo_to_world(
            (self.ox, self.oy, self.oz), (self.fx, self.fy),
            (self.lx, self.ly), f, l, self.demo6['height'])
        xy = math.hypot(p.x - gx, p.y - gy)
        dz = abs(p.z - gz)
        if not all(math.isfinite(value) for value in
                   (p.x, p.y, p.z, gx, gy, gz, xy, dz)):
            raise RuntimeError('DEMO6 POINT HOLD contains NaN/Inf')
        if xy > 0.25 or dz > 0.18:
            raise RuntimeError(
                'DEMO6 POINT HOLD drift dXY=%.3f dZ=%.3f; pilot takeover' %
                (xy, dz))

    def observe_qr(self):
        self.require_point_hold(0.5, -0.5)
        script = Path(__file__).resolve().parent / 'qr_flight_detector.py'
        if not script.is_file():
            raise RuntimeError('Demo6 QR flight detector missing')
        self.qr_data = ''
        self.qr_last_received = None
        self.qr_proc = subprocess.Popen(
            [sys.executable, '-u', str(script), '_process_rate:=5.0',
             '_stable_count:=%d' % self.demo6['stable_required']],
            cwd=str(script.parent))
        rospy.logwarn('[DEMO6] P1 POINT arrived F+0.50 L-0.50; QR scan started')
        policy = ScanPolicy(time.monotonic(), self.demo6['timeout'],
                            self.demo6['hold_time'])
        last_handled_rx = None
        last_logged_tick = -1
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                self.require_point_hold(0.5, -0.5)
                now = time.monotonic()
                if self.qr_proc.poll() is not None:
                    self.qr_reason = 'QR detector unexpectedly exited'
                    rospy.logerr('[DEMO6] %s', self.qr_reason)
                    return False
                rx = self.qr_last_received
                if rx is not None and rx != last_handled_rx:
                    last_handled_rx = rx
                    previous_code = policy.code
                    policy.push(self.qr_data, now)
                    if not previous_code and policy.code:
                        rospy.logwarn(
                            '[DEMO6] QR DETECTED: %r; POINT HOLD for 5.0 seconds',
                            policy.code)
                state = policy.state(now)
                if state == 'FAIL':
                    self.qr_reason = policy.failure
                    rospy.logerr('[DEMO6] QR FAIL: %s', self.qr_reason)
                    return False
                if state == 'TIMEOUT':
                    self.qr_reason = 'QR TIMEOUT (nothing stably decoded)'
                    rospy.logerr('[DEMO6] %s; returning home if safe', self.qr_reason)
                    return False
                if policy.code:
                    whole = int(now - policy.detected_at)
                    if whole != last_logged_tick:
                        last_logged_tick = whole
                        rospy.logwarn(
                            '[DEMO6] QR result=%r | return-home in %.1fs',
                            policy.code, policy.remaining(now))
                if state == 'READY':
                    self.qr_ok = True
                    self.qr_reason = policy.code
                    rospy.logwarn(
                        '[DEMO6] QR HOLD 5s COMPLETE: %r; RETURN HOME F0 L0',
                        policy.code)
                    return True
                rate.sleep()
            raise RuntimeError('ROS shutdown while waiting for QR')
        finally:
            if self.qr_proc is not None and self.qr_proc.poll() is None:
                self.qr_proc.terminate()
                try:
                    self.qr_proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.qr_proc.kill()
                    self.qr_proc.wait(timeout=3)
            self.qr_proc = None

    def execute_point_waypoint(self, index, total, waypoint):
        if total != 2:
            raise RuntimeError('Demo6 V2 expected exactly two POINT legs')
        if index == 1:
            # Proven POINT trajectory, physical arrival, yaw hold and 0.8s settle.
            super().execute_point_waypoint(index, total, waypoint)
            self.observe_qr()
            self.scan_finished = True
            # Even if QR was not found before timeout, return and land through
            # the proven POINT chain while the safety conditions still permit.
            self.require_point_hold(0.5, -0.5)
            rospy.logwarn('[DEMO6] P1 QR phase ended; beginning POINT return')
            return
        if index == 2:
            if not self.scan_finished:
                raise RuntimeError('Demo6 cannot return without finishing scan phase')
            self.return_started = True
            rospy.logwarn('[DEMO6] RETURN HOME START: POINT F0 L0, yaw held')
            super().execute_point_waypoint(index, total, waypoint)
            self.return_complete = True
            rospy.logwarn('[DEMO6] RETURN HOME REACHED; landing completed')
            return
        raise RuntimeError('Invalid Demo6 POINT waypoint index')

    def start_land(self):
        # Invoked only after proven POINT return leg arrival (0.8s stable).
        if not self.scan_finished or not self.return_started:
            raise RuntimeError('Refusing to land before QR scan and homecoming')
        self.require_point_hold(0.0, 0.0)
        rospy.logwarn('[DEMO6] HOME HOLD CONFIRMED; PROVEN AUTO LAND CHAIN')
        super().start_land()
        self.landing_complete = True

    def run(self):
        rospy.logwarn(
            '[DEMO6] POINT F+0.50 L-0.50 -> QR -> HOLD 5s -> '
            'POINT HOME F0 L0 -> LAND; NO software ARM')
        super().run()
        if not self.return_complete or not self.landing_complete:
            raise RuntimeError('Demo6 return home / auto landing incomplete')
        if not self.qr_ok:
            raise RuntimeError(
                'DEMO6 FAIL: returned home and landed, but no confirmed QR: %s' %
                self.qr_reason)
        rospy.logwarn(
            '[DEMO6] SUCCESS: QR %r + POINT RETURN HOME + LAND/DISARM',
            self.qr_reason)


if __name__ == '__main__':
    rospy.init_node('uav2_demo6_qr_waypoint', anonymous=False)
    Demo6QrReturnHome().run()
