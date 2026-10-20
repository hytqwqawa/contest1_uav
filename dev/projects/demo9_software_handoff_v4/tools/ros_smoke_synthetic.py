#!/usr/bin/env python3
"""Synthetic ROS1 loopback smoke for the V4 MUX and real trajectory interface.

This file must be launched ONLY by isolated_ros_smoke.sh, which switches to a
private loopback roscore. It cannot arm, set modes, or contact a flight control.
"""
import math
import os
import threading
import time
import urllib.parse
import rospy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

EXPECTED_MASTER='http://127.0.0.1:11457'
if os.environ.get('ROS_MASTER_URI')!=EXPECTED_MASTER:
    raise SystemExit('[STOP] Must use loopback-only private ROS master: '+EXPECTED_MASTER)

class Simulator:
    def __init__(self):
        self.lock=threading.RLock()
        self.good_point=False
        self.yaw_spike=False
        self.status={}
        self.messages={}
        self.publishers={
            'odom':rospy.Publisher('/contest1/mavros/local_position/odom',Odometry,queue_size=10),
            'health':rospy.Publisher('/contest1/ego_uav_interface/flight_healthy',Bool,queue_size=10),
            'rc':rospy.Publisher('/contest1/ego_uav_interface/rc_mission_allowed',Bool,queue_size=10),
            'point':rospy.Publisher('/contest1/ego_uav_interface/point_setpoint',PositionTarget,queue_size=10),
            'point_status':rospy.Publisher('/contest1/ego_uav_interface/point_traj_status',String,queue_size=10),
            'ego':rospy.Publisher('/contest1/ego_uav_interface/ego_setpoint',PositionTarget,queue_size=10),
        }
        rospy.Subscriber('/contest1/ego_uav_interface/trajectory_mux_status',String,
                         lambda msg:self.on_status('mux',msg.data),queue_size=20)
        rospy.Subscriber('/contest1/ego_uav_interface/trajectory_interface_status',String,
                         lambda msg:self.on_status('interface',msg.data),queue_size=20)
        rospy.Subscriber('/contest1/mavros/setpoint_raw/local',PositionTarget,
                         lambda msg:self.on_command(msg),queue_size=20)
        self.worker=threading.Thread(target=self.publisher_loop,daemon=True)
        self.worker.start()
    def on_status(self,key,data):
        with self.lock:self.status[key]=data
    def on_command(self,msg):
        with self.lock:self.messages['last']=msg
    @staticmethod
    def command(x,y,z,yaw):
        c=PositionTarget()
        c.header.stamp=rospy.Time.now()
        c.header.frame_id='map'
        c.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
        c.type_mask=0
        c.position.x=x;c.position.y=y;c.position.z=z
        c.yaw=math.radians(yaw)
        return c
    def publisher_loop(self):
        rate=rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                good=self.good_point
                spike=self.yaw_spike
            odom=Odometry()
            odom.header.stamp=rospy.Time.now()
            odom.pose.pose.position.x=1.0
            odom.pose.pose.position.y=1.0
            odom.pose.pose.position.z=1.0
            a=math.radians(93.0)/2.0
            odom.pose.pose.orientation.z=math.sin(a)
            odom.pose.pose.orientation.w=math.cos(a)
            self.publishers['odom'].publish(odom)
            self.publishers['health'].publish(Bool(data=True))
            self.publishers['rc'].publish(Bool(data=True))
            self.publishers['point_status'].publish(String(data='AIRBORNE_HOLD'))
            self.publishers['ego'].publish(self.command(1.,1.,1.,93.))
            if not good or spike:
                point=self.command(1.,0.,1.,-175.)
            else:
                point=self.command(1.,1.,1.,93.)
            self.publishers['point'].publish(point)
            rate.sleep()
    def wait_status(self,key,expected,seconds=5):
        end=time.monotonic()+seconds
        while not rospy.is_shutdown() and time.monotonic()<end:
            with self.lock:val=self.status.get(key,'')
            if val==expected:return True
            time.sleep(.02)
        raise AssertionError('Timeout waiting %s=%s, observed %s'%(key,expected,val))
    def final_matches(self,seconds=5):
        end=time.monotonic()+seconds
        while not rospy.is_shutdown() and time.monotonic()<end:
            with self.lock: c=self.messages.get('last')
            if c is not None and abs(c.position.x-1)<.02 and abs(c.position.y-1)<.02 and abs(c.yaw-math.radians(93))<.05:
                return True
            time.sleep(.02)
        raise AssertionError('No correct final MAVROS local setpoint')

def call(path):
    rospy.wait_for_service(path,timeout=5)
    return rospy.ServiceProxy(path,Trigger)()

def main():
    rospy.init_node('v4_synthetic_ros_smoke',anonymous=True)
    sim=Simulator()
    sim.wait_status('interface','HOLD_ACTIVE',seconds=6)
    result=call('/contest1/ego_uav_interface/use_ego')
    assert result.success, 'Cannot choose normal EGO: '+result.message
    sim.wait_status('mux','EGO_ACTIVE')
    # Recreate 10-ms stale POINT selection; current HOLD is at (1,1,1), yaw 93.
    old=call('/contest1/ego_uav_interface/use_point')
    assert not old.success, 'Old POINT was wrongly accepted'
    print('[OK] Old POINT -> switch refused:',old.message)
    # Mimic the new POINT publish after abort: now in actual vehicle HOLD.
    with sim.lock:sim.good_point=True
    time.sleep(.15)
    good=call('/contest1/ego_uav_interface/use_point')
    assert good.success, 'New POINT was wrongly rejected: '+good.message
    sim.wait_status('mux','POINT_ACTIVE')
    response=call('/contest1/ego_uav_interface/enable_trajectory')
    assert response.success, response.message
    sim.wait_status('interface','TRAJECTORY_ACTIVE')
    sim.final_matches()
    print('[OK] Fresh POINT -> MUX -> TrajectoryInterface -> synthetic MAVROS target')
    with sim.lock:sim.yaw_spike=True
    sim.wait_status('mux','POINT_GUARD_REJECTED')
    sim.wait_status('interface','HOLD_WAIT_TRAJECTORY')
    print('[OK] Late POINT yaw spike refused, downstream falls back to HOLD')
    with sim.lock:sim.yaw_spike=False
    sim.wait_status('mux','POINT_ACTIVE')
    sim.wait_status('interface','TRAJECTORY_ACTIVE')
    print('[OK] Continuity restored without re-enabling old trajectory')
    print('PASS: isolated ROS synthetic handoff and downstream HOLD')

if __name__=='__main__':
    main()
