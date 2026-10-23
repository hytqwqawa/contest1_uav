#!/usr/bin/env python3
"""Dedicated local test only: fake odom/health/permission/command into R3 IF.

No arm, no MAVROS node. Fails if master not http://127.0.0.1:11674.
"""
import os,sys,threading,time,math
from urllib.parse import urlparse
from pathlib import Path
u=urlparse(os.environ.get('ROS_MASTER_URI',''))
if u.hostname!='127.0.0.1' or u.port!=11674 or os.environ.get('ROS_IP')!='127.0.0.1':
    raise SystemExit('[STOP] R3 loopback REQUIRES local-only 11674 master')

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from mavros_msgs.msg import PositionTarget
from demo5_config import load_config
import yaml

ROOT=Path(__file__).resolve().parent
session=Path(sys.argv[1]); stopped=False; lock=threading.RLock(); outputs=[]; statuses=[]
rospy.init_node('demo11_r3_final_loopback',anonymous=True,disable_signals=True)
od_pub=rospy.Publisher('/contest1/mavros/local_position/odom',Odometry,queue_size=10)
health_pub=rospy.Publisher('/contest1/ego_uav_interface/flight_healthy',Bool,queue_size=10)
perm_pub=rospy.Publisher('/contest1/ego_uav_interface/rc_mission_allowed',Bool,queue_size=10)
safe_pub=rospy.Publisher('/contest1/ego_uav_interface/demo_frame_safe',Bool,queue_size=10)
ready_pub=rospy.Publisher('/contest1/yaw_tangent_demo/session_ready',Bool,queue_size=1,latch=True)
cmd_pub=rospy.Publisher('/contest1/ego_uav_interface/trajectory_setpoint',PositionTarget,queue_size=10)

def output_cb(m):
    with lock: outputs.append((time.monotonic(),m.position.x,m.position.y,m.yaw))
def status_cb(m):
    with lock: statuses.append(m.data)
rospy.Subscriber('/contest1/mavros/setpoint_raw/local',PositionTarget,output_cb,queue_size=100)
rospy.Subscriber('/contest1/ego_uav_interface/trajectory_interface_status',String,status_cb,queue_size=20)

def tick(_):
    m=Odometry();m.header.stamp=rospy.Time.now();m.header.frame_id='world'
    m.pose.pose.position.x=1.;m.pose.pose.position.y=0.;m.pose.pose.position.z=.8
    m.pose.pose.orientation.w=1.
    od_pub.publish(m)
    health_pub.publish(Bool(data=True))
    perm_pub.publish(Bool(data=True))
    safe_pub.publish(Bool(data=True))
rospy.Timer(rospy.Duration(.035),tick)

def until(fn,seconds,reason):
    end=time.monotonic()+seconds
    while not rospy.is_shutdown() and time.monotonic()<end:
        if fn():return
        time.sleep(.04)
    raise RuntimeError('timeout: '+reason)

def publish_cmd(x,y,yaw=0.,count=12):
    for _ in range(count):
        m=PositionTarget();m.header.stamp=rospy.Time.now()
        m.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
        m.type_mask=(PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|
            PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|
            PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE)
        m.position.x=x;m.position.y=y;m.position.z=.8;m.yaw=yaw
        cmd_pub.publish(m);time.sleep(.03)

try:
    until(lambda:cmd_pub.get_num_connections()>=1,8,'interface command subscription')
    plan=load_config(ROOT/'demo5_waypoints.yaml')
    frame={'demo5_plan_snapshot':plan,'demo_fence':plan['fence'],
           'origin_world':{'x':0.,'y':0.,'z':0.},
           'forward_world':{'x':1.,'y':0.},
           'left_world':{'x':0.,'y':1.}}
    session.write_text(yaml.safe_dump(frame))
    ready_pub.publish(Bool(data=True))
    until(lambda:rospy.get_published_topics() is not None,1,'ROS Master')
    rospy.wait_for_service('/contest1/ego_uav_interface/enable_trajectory',timeout=8)
    enable=rospy.ServiceProxy('/contest1/ego_uav_interface/enable_trajectory',Trigger)
    until(lambda:len(outputs)>8,6,'HOLD stream')
    time.sleep(.2)
    success=enable()
    if not success.success:raise RuntimeError('enable refused: '+success.message)
    print('[PASS] Fresh session + frame + health allow trajectory enable',flush=True)
    publish_cmd(1.02,0.,count=14)
    with lock:
        if not any(abs(y)<.015 and abs(x-1.02)<.015 for _,x,y,_ in outputs):
            raise RuntimeError('valid trajectory did not reach final MAVROS setpoint topic')
    print('[PASS] Valid trajectory reaches final setpoint publisher',flush=True)
    with lock: before=len(outputs)
    publish_cmd(1.04,-.904,count=11)
    until(lambda:any('HOLD_R3_CORRIDOR_ABORT' in s for s in statuses),4,'corridor abort status')
    with lock:
        after=outputs[before:]
        if any(y<-.5 for _,_,y,_ in after):
            raise RuntimeError('UNSAFE downstream setpoint published')
        if len(after)<7:
            raise RuntimeError('output stream stopped after abort')
        if not any(abs(x-1.)<.001 and abs(y)<.001 for _,x,y,_ in after):
            raise RuntimeError('HOLD did not capture actual aircraft position')
    if enable().success:raise RuntimeError('abort latch allowed automatic reenable')
    print('[PASS] Unsafe L=-0.904 command never reached MAVROS output',flush=True)
    print('[PASS] Continuous measured-pose HOLD + no auto-reenable after rejection',flush=True)
except Exception as exc:
    print('[FAIL] '+str(exc),flush=True)
    raise SystemExit(3)
