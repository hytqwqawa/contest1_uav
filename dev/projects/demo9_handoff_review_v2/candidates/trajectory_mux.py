#!/usr/bin/env python3
import rospy
import threading
import copy
from yaw_guard_core import check

from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


class TrajectoryMux:

    def __init__(self):

        self.timeout = 0.25

        self.source = "POINT"
        self._guard_lock = threading.RLock()
        self._guard_last_msg = None
        self._guard_last_time = None
        self._guard_window_until = None

        self.point_msg = None
        self.point_time = None

        self.ego_msg = None
        self.ego_time = None

        self.pub = rospy.Publisher(
            "/contest1/ego_uav_interface/trajectory_setpoint",
            PositionTarget,
            queue_size=10
        )

        self.status_pub = rospy.Publisher(
            "/contest1/ego_uav_interface/trajectory_mux_status",
            String,
            queue_size=1,
            latch=True
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/point_setpoint",
            PositionTarget,
            self.point_cb,
            queue_size=20
        )

        rospy.Subscriber(
            "/contest1/ego_uav_interface/ego_setpoint",
            PositionTarget,
            self.ego_cb,
            queue_size=20
        )

        rospy.Service(
            "/contest1/ego_uav_interface/use_point",
            Trigger,
            self.use_point
        )

        rospy.Service(
            "/contest1/ego_uav_interface/use_ego",
            Trigger,
            self.use_ego
        )

        rospy.Timer(
            rospy.Duration(0.02),
            self.timer_cb
        )

    def point_cb(self, msg):
        with self._guard_lock:
            self.point_msg = msg
            self.point_time = rospy.Time.now()

    def ego_cb(self, msg):
        with self._guard_lock:
            self.ego_msg = msg
            self.ego_time = rospy.Time.now()

    def fresh(self, t):
        if t is None:
            return False
        age = (rospy.Time.now() - t).to_sec()
        return 0.0 <= age < self.timeout

    def _guard_check(self, cmd):
        if self._guard_last_msg is None or self._guard_last_time is None:
            return 'no_published_reference'
        age = (rospy.Time.now()-self._guard_last_time).to_sec()
        if not 0.0 <= age <= 0.30:
            return 'reference_stale'
        return check(self._guard_last_msg, cmd)

    def _guard_select(self, desired):
        # Read the message and its receipt timestamp UNDER the same lock.
        # A callback may update them concurrently with the service call.
        with self._guard_lock:
            if desired == 'POINT':
                cmd, timestamp = self.point_msg, self.point_time
            else:
                cmd, timestamp = self.ego_msg, self.ego_time
            if not self.fresh(timestamp):
                return TriggerResponse(False, desired + ' source missing/stale')
            reason = check(None, cmd)
            if reason:
                return TriggerResponse(False, desired + ' invalid: ' + reason)
            if desired != self.source:
                reason = self._guard_check(cmd)
                if reason:
                    rospy.logerr('[HANDOFF_GUARD] %s refused: %s', desired, reason)
                    return TriggerResponse(False, 'continuity veto: ' + reason)
                self._guard_window_until = rospy.Time.now() + rospy.Duration(0.25)
            self.source = desired
            return TriggerResponse(True, 'Trajectory source -> ' + desired)

    def use_point(self, req):
        return self._guard_select('POINT')

    def use_ego(self, req):
        return self._guard_select('EGO')

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


if __name__ == "__main__":
    rospy.init_node("trajectory_mux")
    TrajectoryMux()
    rospy.spin()
