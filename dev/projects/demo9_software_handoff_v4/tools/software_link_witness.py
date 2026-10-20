#!/usr/bin/env python3
"""Passive ROS1 Demo9 link witness, never publishes, calls services, or controls PX4.

Run as a fourth, optional read-only ground diagnostic process. Produces a
separately-fsynced JSONL heartbeat trail, independent of rosbag's close/index.
It is NOT a watchdog that can keep OFFBOARD alive when the host fails.
"""
import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
from datetime import datetime

import rospy
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

TOPICS={
    'state':('/contest1/mavros/state',State,2.8),
    'odom':('/contest1/mavros/local_position/odom',Odometry,.45),
    'setpoint':('/contest1/mavros/setpoint_raw/local',PositionTarget,.45),
    'mux':('/contest1/ego_uav_interface/trajectory_mux_status',String,.8),
    'interface':('/contest1/ego_uav_interface/trajectory_interface_status',String,.8),
    'health':('/contest1/ego_uav_interface/flight_healthy',Bool,.8),
    'rc_mission':('/contest1/ego_uav_interface/rc_mission_allowed',Bool,1.7),
}

class Witness:
    def __init__(self,path):
        self.path=path
        self.lock=threading.RLock()
        self.known={}
        self.first=time.monotonic()
        self.prev_yaw=None
        # Open in exclusive mode to avoid replacing any earlier evidence.
        self.file=open(path,'x',encoding='utf8',buffering=1)
        self.write({'event':'started','pid':os.getpid(),'path':str(path)})
        self.subscribers=[]
        for key,(name,cls,limit) in TOPICS.items():
            self.subscribers.append(rospy.Subscriber(
                name,cls,lambda msg,key=key:self.callback(key,msg),queue_size=50))
        self.timer=rospy.Timer(rospy.Duration(1.0),self.report)

    def write(self,obj):
        obj={'unix_time':round(time.time(),3),'monotonic':round(time.monotonic(),3),**obj}
        self.file.write(json.dumps(obj,ensure_ascii=False,separators=(',',':'))+'\n')
        self.file.flush()
        os.fsync(self.file.fileno())

    def callback(self,key,msg):
        now=time.monotonic()
        with self.lock:
            entry=self.known.setdefault(key,{'count':0,'last':now,'value':None})
            entry['count']+=1
            entry['last']=now
            if key=='state':
                val=(bool(msg.connected),bool(msg.armed),str(msg.mode))
            elif key in ('mux','interface'):
                val=msg.data
            elif key in ('health','rc_mission'):
                val=bool(msg.data)
            else:
                val=None
            if val is not None and val!=entry['value']:
                entry['value']=val
                self.write({'event':'change','topic':key,'value':val})
            if key=='setpoint':
                yaw=float(msg.yaw)
                if math.isfinite(yaw):
                    if self.prev_yaw is not None:
                        change=math.degrees(math.atan2(math.sin(yaw-self.prev_yaw),
                                                        math.cos(yaw-self.prev_yaw)))
                        if abs(change)>25.0:
                            self.write({'event':'setpoint_yaw_jump',
                                        'change_deg':round(change,2)})
                    self.prev_yaw=yaw

    def report(self,event):
        now=time.monotonic()
        with self.lock:
            status={}
            missing=[]
            for key,(_,__,limit) in TOPICS.items():
                entry=self.known.get(key)
                if entry is None:
                    status[key]={'count':0,'age_s':None}
                    if now-self.first>5.0:missing.append(key)
                else:
                    age=round(max(0.0,now-entry['last']),3)
                    status[key]={'count':entry['count'],'age_s':age}
                    if age>limit:missing.append(key)
            self.write({'event':'heartbeat','streams':status,'missing':missing})

    def close(self):
        with self.lock:
            self.write({'event':'clean_exit'})
            self.file.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,
        default=Path.home()/'Desktop'/(
            'demo9_link_witness_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl'))
    args=parser.parse_args()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    rospy.init_node('demo9_software_link_witness',anonymous=True)
    witness=Witness(args.out)
    rospy.loginfo('[LINK WITNESS] passive recorder: %s',args.out)
    try:
        rospy.spin()
    finally:
        witness.close()

if __name__=='__main__':
    main()
