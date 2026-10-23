#!/usr/bin/env python3
"""R4P: stage in-flight obstacle replans until geometry-certified by mission.

Initial full-goal preview goes to private traj_server unchanged; freeze after
mission certifies it. In-flight EGO updates are staged and NEVER forwarded
until a one-time exact-ticket commit from the active mission. This node does
not publish MAVROS commands or ARM. Invalid candidates cannot reach traj_server.
"""
import hashlib
import threading
import rospy
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerResponse
from traj_utils.msg import Bspline

PREFIX='/contest1/demo11_r4p'


def spline_signature(msg):
    stamp=getattr(msg,'start_time',None)
    time_pair=(int(stamp.secs),int(stamp.nsecs)) if stamp is not None else (0,0)
    return (int(msg.traj_id),int(msg.order),time_pair,
            tuple((float(p.x),float(p.y),float(p.z)) for p in msg.pos_pts),
            tuple(float(k) for k in msg.knots))


def spline_ticket(msg):
    digest=hashlib.sha256(repr(spline_signature(msg)).encode('ascii')).hexdigest()[:20]
    return '%d:%s'%(int(msg.traj_id),digest)


class SupervisedGate:
    def __init__(self):
        self.lock=threading.RLock()
        self.state='PREVIEW'
        self.frozen_id=None
        self.preview_msg=None
        self.preview_signature=None
        self.pending_msg=None
        self.pending_ticket=None
        self.pub=rospy.Publisher(PREFIX+'/bspline_to_traj_server',Bspline,queue_size=10)
        self.status_pub=rospy.Publisher(PREFIX+'/gate_state',String,queue_size=5,latch=True)
        self.committed_pub=rospy.Publisher(PREFIX+'/committed_ticket',String,queue_size=5,latch=True)
        # Dynamic approval and traj_server admission are paired in this node.
        self.approved_pub=rospy.Publisher(PREFIX+'/approved_traj_id',Int32,queue_size=10)
        rospy.Subscriber('/contest1/drone_0_planning/bspline',Bspline,self.bspline_cb,queue_size=20)
        rospy.Subscriber(PREFIX+'/commit_ticket',String,self.commit_cb,queue_size=10)
        rospy.Service(PREFIX+'/arm_preview',Trigger,self.arm_preview)
        rospy.Service(PREFIX+'/freeze',Trigger,self.freeze)
        rospy.Timer(rospy.Duration(0.10),self.heartbeat)
        self._publish_state()

    def _publish_state(self):
        self.status_pub.publish(String(data=self.state))

    def bspline_cb(self,msg):
        with self.lock:
            if self.frozen_id is None:
                self.preview_msg=msg
                self.preview_signature=spline_signature(msg)
                self.pub.publish(msg)
                return
            if int(msg.traj_id)==self.frozen_id and spline_signature(msg)==self.preview_signature:
                return
            ticket=spline_ticket(msg)
            if ticket==self.pending_ticket:
                return
            self.pending_msg=msg
            self.pending_ticket=ticket
            self.state='PENDING:'+ticket
            rospy.logwarn('[R4P] candidate staged %s; awaiting existing 3D/corridor certification',ticket)
            self._publish_state()
            # Do not forward a candidate just because Planner published it.

    def commit_cb(self,msg):
        with self.lock:
            ticket=str(msg.data).strip()
            if not ticket or ticket!=self.pending_ticket or self.pending_msg is None:
                rospy.logwarn('[R4P] stale/unmatched certificate ignored: %s',ticket)
                return
            if self.pub.get_num_connections()<1:
                rospy.logerr('[R4P] traj_server disconnected; candidate not committed')
                return
            chosen=self.pending_msg
            chosen_signature=spline_signature(chosen)
            if ticket!=spline_ticket(chosen):
                rospy.logerr('[R4P] pending spline mutated; refusing')
                return
            # Approve the exact ID before traj_server publishes that ID.
            # The existing Bridge still suppresses any non-approved ID.
            self.approved_pub.publish(Int32(data=int(chosen.traj_id)))
            rospy.sleep(0.035)
            self.pub.publish(chosen)
            self.frozen_id=int(chosen.traj_id)
            self.preview_signature=chosen_signature
            self.pending_msg=None
            self.pending_ticket=None
            self.state='FROZEN:%d'%self.frozen_id
            self.committed_pub.publish(String(data=ticket))
            self._publish_state()
            rospy.logwarn('[R4P] certified in-flight replan COMMITTED %s; yaw slew unchanged',ticket)

    def arm_preview(self,req):
        with self.lock:
            self.frozen_id=None
            self.preview_msg=None
            self.preview_signature=None
            self.pending_msg=None
            self.pending_ticket=None
            self.state='PREVIEW'
            self._publish_state()
            return TriggerResponse(True,'PREVIEW')

    def freeze(self,req):
        with self.lock:
            if self.frozen_id is not None:return TriggerResponse(False,'already frozen')
            if self.preview_msg is None:return TriggerResponse(False,'no B-spline received')
            if self.pub.get_num_connections()<1:return TriggerResponse(False,'traj_server disconnected')
            self.frozen_id=int(self.preview_msg.traj_id)
            self.state='FROZEN:%d'%self.frozen_id
            self._publish_state()
            return TriggerResponse(True,self.state)

    def heartbeat(self,event):
        with self.lock:self._publish_state()


if __name__=='__main__':
    rospy.init_node('demo11_r4p_bspline_gate')
    SupervisedGate()
    rospy.spin()
