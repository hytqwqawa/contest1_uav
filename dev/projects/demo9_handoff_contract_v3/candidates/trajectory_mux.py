#!/usr/bin/env python3
"""Development-only trajectory source switch guard.

Never deploy without ROS/PX4-in-loop and no-prop verification.
"""
import copy
import threading
import rospy
from yaw_guard_core import check

from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class TrajectoryMux:
    def __init__(self):
        self.timeout = 0.25
        self.source = 'POINT'
        self._lock = threading.RLock()
        self.point_msg = None
        self.point_time = None
        self.ego_msg = None
        self.ego_time = None
        self.point_status = None
        self.point_status_time = None
        self.interface_status = None
        self.interface_time = None
        self.downstream_msg = None
        self.downstream_time = None
        self._last_msg = None
        self._last_time = None
        self._guard_until = None

        self.pub = rospy.Publisher(
            '/contest1/ego_uav_interface/trajectory_setpoint',
            PositionTarget, queue_size=10)
        self.status_pub = rospy.Publisher(
            '/contest1/ego_uav_interface/trajectory_mux_status',
            String, queue_size=1, latch=True)
        rospy.Subscriber('/contest1/ego_uav_interface/point_setpoint',
                         PositionTarget, self.point_cb, queue_size=20)
        rospy.Subscriber('/contest1/ego_uav_interface/ego_setpoint',
                         PositionTarget, self.ego_cb, queue_size=20)
        rospy.Subscriber('/contest1/ego_uav_interface/point_traj_status',
                         String, self.point_status_cb, queue_size=20)
        rospy.Subscriber('/contest1/ego_uav_interface/trajectory_interface_status',
                         String, self.interface_cb, queue_size=20)
        rospy.Subscriber('/contest1/mavros/setpoint_raw/local',
                         PositionTarget, self.downstream_cb, queue_size=20)
        rospy.Service('/contest1/ego_uav_interface/use_point',
                      Trigger, self.use_point)
        rospy.Service('/contest1/ego_uav_interface/use_ego',
                      Trigger, self.use_ego)
        rospy.Timer(rospy.Duration(0.02), self.timer_cb)

    def point_cb(self, msg):
        with self._lock:
            self.point_msg = copy.deepcopy(msg)
            self.point_time = rospy.Time.now()

    def ego_cb(self, msg):
        with self._lock:
            self.ego_msg = copy.deepcopy(msg)
            self.ego_time = rospy.Time.now()

    def point_status_cb(self, msg):
        with self._lock:
            self.point_status = msg.data
            self.point_status_time = rospy.Time.now()

    def interface_cb(self, msg):
        with self._lock:
            self.interface_status = msg.data
            self.interface_time = rospy.Time.now()

    def downstream_cb(self, msg):
        with self._lock:
            self.downstream_msg = copy.deepcopy(msg)
            self.downstream_time = rospy.Time.now()

    @staticmethod
    def fresh(stamp, maximum=0.25):
        if stamp is None:
            return False
        age = (rospy.Time.now() - stamp).to_sec()
        return 0.0 <= age < maximum

    def _switch_anchor(self):
        # The downstream interface can independently output HOLD even while
        # MUX still selects EGO. Compare to *what PX4 is actually receiving*.
        if not self.fresh(self.interface_time, 0.30):
            return None, 'interface_status_stale'
        if self.interface_status == 'HOLD_ACTIVE':
            if not self.fresh(self.downstream_time, 0.15):
                return None, 'downstream_hold_stale'
            reason = check(None, self.downstream_msg)
            if reason:
                return None, 'downstream_hold_invalid:' + reason
            return self.downstream_msg, None
        if self.interface_status == 'TRAJECTORY_ACTIVE':
            if not self.fresh(self._last_time, 0.30):
                return None, 'selected_source_reference_stale'
            reason = check(None, self._last_msg)
            if reason:
                return None, 'selected_source_reference_invalid:' + reason
            return self._last_msg, None
        return None, 'interface_not_eligible:' + str(self.interface_status)

    def _select(self, desired):
        with self._lock:
            cmd, stamp = ((self.point_msg, self.point_time)
                          if desired == 'POINT' else (self.ego_msg, self.ego_time))
            if not self.fresh(stamp, self.timeout):
                return TriggerResponse(False, desired + ' source missing/stale')
            reason = check(None, cmd)
            if reason:
                return TriggerResponse(False, desired + ' invalid: ' + reason)
            if desired != self.source:
                anchor, reason = self._switch_anchor()
                if not reason:
                    reason = check(anchor, cmd)
                if reason:
                    rospy.logerr('[HANDOFF_V3] %s refused: %s', desired, reason)
                    return TriggerResponse(False, 'continuity veto: ' + reason)
                # Seed the exact downstream reference before accepting the
                # first command. Do not reuse a stale EGO MUX output.
                self._last_msg = copy.deepcopy(anchor)
                self._last_time = rospy.Time.now()
                self._guard_until = rospy.Time.now() + rospy.Duration(0.25)
            self.source = desired
            return TriggerResponse(True, 'Trajectory source -> ' + desired)

    def use_point(self, req):
        return self._select('POINT')

    def use_ego(self, req):
        return self._select('EGO')

    def timer_cb(self, event):
        with self._lock:
            cmd, stamp = ((self.point_msg, self.point_time)
                          if self.source == 'POINT' else (self.ego_msg, self.ego_time))
            if not self.fresh(stamp, self.timeout):
                self.status_pub.publish(String(data=self.source + '_STALE'))
                return
            reason = check(None, cmd)
            guard_active = self._guard_until is not None
            # POINT AIRBORNE_HOLD should never jump by a metre or ~90 degrees
            # within one update, including long AFTER the handoff window.
            steady_hold = (self.source == 'POINT' and
                self.point_status == 'AIRBORNE_HOLD' and
                self.fresh(self.point_status_time, 0.30) and
                self.fresh(self._last_time, 0.30))
            if not reason and (guard_active or steady_hold):
                reason = check(self._last_msg, cmd)
            if reason:
                # A rejected command must NEVER be released just because the
                # initial 250-ms handoff window elapsed. Extend the window
                # until fresh, continuous commands have resumed.
                if guard_active or steady_hold:
                    self._guard_until = rospy.Time.now() + rospy.Duration(0.25)
                rospy.logerr_throttle(1.0,
                    '[HANDOFF_V3] %s command veto: %s' % (self.source, reason))
                self.status_pub.publish(String(data=self.source + '_GUARD_REJECTED'))
                return
            self.pub.publish(cmd)
            self._last_msg = copy.deepcopy(cmd)
            self._last_time = rospy.Time.now()
            if self._guard_until is not None and rospy.Time.now() >= self._guard_until:
                self._guard_until = None
            self.status_pub.publish(String(data=self.source + '_ACTIVE'))


if __name__ == '__main__':
    rospy.init_node('trajectory_mux')
    TrajectoryMux()
    rospy.spin()
