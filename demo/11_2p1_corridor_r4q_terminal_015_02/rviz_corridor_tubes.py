#!/usr/bin/env python3
"""VISUALIZATION ONLY: 1.5m straight prism and schematic round 90deg bend.

No MAVROS services, mode commands, setpoints, or flight-controller topics.
Requires a fresh Frame Guard session whose YAML plan matches this demo's YAML.
"""
import json
import math
from pathlib import Path

import rospy
import yaml
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from demo5_config import load_config
from demo11_corridor import load_corridor
from demo11_rviz_geometry import world_point, quarter_arc, round_tube_ring

ROOT = Path(__file__).resolve().parent
TOPIC = '/contest1/demo11_r4i/geometry_markers'


class CorridorView:
    def __init__(self):
        self.plan = load_config(ROOT / 'demo5_waypoints.yaml')
        self.c = load_corridor(ROOT / 'demo11_corridor.yaml')
        self.arc = json.loads((ROOT/'DEMO11_SOURCE_MANIFEST.json').read_text())['reference_arc']
        self.pub = rospy.Publisher(TOPIC, MarkerArray, queue_size=1, latch=True)
        self.session_path = ROOT / 'session_frame.yaml'
        self.session_signature = None
        self.last_wait_message = 0
        self.loaded = False

    def mark(self, session, ident, group, kind, color, width=0.025):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = rospy.Time.now()
        m.ns = group
        m.id = ident
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = width
        m.scale.y = width
        m.scale.z = width
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.lifetime = rospy.Duration(0)
        return m

    @staticmethod
    def point(session, f, l, h):
        x, y, z = world_point(session, f, l, h)
        return Point(x=x, y=y, z=z)

    def line(self, out, session, idx, group, pts, color, thickness=0.035,
             kind=Marker.LINE_STRIP):
        m = self.mark(session, idx, group, kind, color, thickness)
        m.points = [self.point(session, *p) for p in pts]
        out.append(m)

    def text(self, out, session, idx, group, f, l, h, message,
             color=(1.0,1.0,1.0,1.0), scale=.16):
        m = self.mark(session, idx, group, Marker.TEXT_VIEW_FACING, color, .01)
        m.pose.position = self.point(session, f, l, h)
        m.scale.z = scale
        m.text = message
        out.append(m)

    def sphere(self, out, session, idx, f, l, h, color):
        m = self.mark(session, idx, 'waypoints', Marker.SPHERE, color)
        m.pose.position = self.point(session, f, l, h)
        m.scale.x = m.scale.y = m.scale.z = 0.12
        out.append(m)

    def build(self, session):
        c = self.c
        a = self.arc
        height = float(self.plan['flight_height'])
        wire_top = float(self.plan['fence']['height_max'])
        out = []
        # Straight physical wall: ground/top rails + end frames (illustration
        # height comes from Demo fence, NOT measured physical wall height).
        yellow = (1.0, 0.79, 0.16, .95)
        for i, L in enumerate((c.l_min, c.l_max)):
            for j, z in enumerate((0.0, wire_top)):
                self.line(out,session,10+i*2+j,'straight_wall',
                          [(c.f_start,L,z),(c.f_end,L,z)],yellow,.055)
            for j,f in enumerate((c.f_start,c.f_end)):
                self.line(out,session,20+i*2+j,'straight_wall',
                          [(f,L,0),(f,L,wire_top)],yellow,.023)
        for i,f in enumerate((c.f_start,c.f_end)):
            for j,z in enumerate((0.0, wire_top)):
                self.line(out,session,30+i*2+j,'straight_end',
                          [(f,c.l_min,z),(f,c.l_max,z)],yellow,.035)
        # Original reviewed body-center guard (actual enforcement elsewhere).
        green = (.25, 1.0, .30, .9)
        for i,L in enumerate((c.center_l_min,c.center_l_max)):
            self.line(out,session,40+i,'protected_center',
                      [(c.f_guard_min,L,height),(c.f_guard_max,L,height)],green,.026)
        for i,f in enumerate((c.f_guard_min,c.f_guard_max)):
            self.line(out,session,50+i,'protected_center',
                      [(f,c.center_l_min,height),(f,c.center_l_max,height)],green,.021)
        self.text(out,session,60,'labels',c.f_start+.4,c.l_max+.32,.65,
                  'STRAIGHT: F1..3 / L[-0.5,+1.0] (schematic)',(1.,.84,.4,1.),.16)

        # Round curved tube is a REFERENCE ARC ONLY and is NOT a true fence.
        cf,cl,r = float(a['center_f']),float(a['center_l']),float(a['radius_m'])
        # The illustrated 1.5m-diameter round tunnel (0.75m tube radius) is
        # not a measurement of any competition wall / obstacle / safety envelope.
        tube_r = .75
        cyan = (.15,.78,1.0,.72)
        for i,ang in enumerate(range(0,91,15)):
            pts = round_tube_ring(cf,cl,r,tube_r,ang,height,36)
            self.line(out,session,100+i,'ROUND_BEND_ILLUSTRATION',pts,cyan,.019)
        for i,rho in enumerate((r-tube_r,r+tube_r)):
            planar = quarter_arc(cf,cl,rho,72)
            for j,z in enumerate((height,)):
                self.line(out,session,120+i,'ROUND_BEND_ILLUSTRATION',
                          [(f,l,z) for f,l in planar],cyan,.035)
        for j,z in enumerate((height-tube_r,height+tube_r)):
            planar = quarter_arc(cf,cl,r,72)
            self.line(out,session,130+j,'ROUND_BEND_ILLUSTRATION',
                      [(f,l,z) for f,l in planar],cyan,.018)
        self.line(out,session,140,'arc_reference_center',
                  [(f,l,height) for f,l in quarter_arc(cf,cl,r,72)],
                  (.12, .98, 1.0, 1.0),.044)
        self.text(out,session,150,'labels',3.7,-1.4,1.72,
                  'ROUND BEND: 90deg R1.5 / SCHEMATIC ONLY',(.42,.85,1.,1.),.17)

        for i,wp in enumerate(self.plan['waypoints']):
            f,l,h = float(wp['forward']),float(wp['left']),height
            color = (1.,.45,.12,1.) if i==0 else (.97,.34,.54,1.)
            self.sphere(out,session,200+i,f,l,h,color)
            name = 'P1 = P1B' if i==0 else 'P%d'%(i+1)
            self.text(out,session,210+i,'labels',f,l,h+.25,
                      '%s F%+.3f L%+.3f'%(name,f,l),color,.16)
        self.text(out,session,250,'labels',1.0,-1.15,1.64,
                  'RViz illustration != collision sensor / flight fence',
                  (1.0,.55,.48,1.), .16)
        return MarkerArray(markers=out)

    def tick(self, _event):
        try:
            if not self.session_path.is_file():
                self.loaded = False
                rospy.logwarn_throttle(5,'[R4I RVIZ] Waiting for new session_frame.yaml')
                return
            stat = self.session_path.stat()
            signature = (stat.st_mtime_ns,stat.st_size)
            if self.loaded and signature == self.session_signature:
                return
            session = yaml.safe_load(self.session_path.read_text())
            if (not isinstance(session,dict) or
                    session.get('demo5_plan_snapshot') != self.plan):
                self.loaded = False
                rospy.logwarn_throttle(5,'[R4I RVIZ] Session plan mismatch / stale; markers NOT published')
                return
            view = self.build(session)
            self.pub.publish(view)
            self.loaded = True
            self.session_signature = signature
            rospy.logwarn('[R4I RVIZ] Straight+round schematic and 3 goals published in world (%d markers)',
                          len(view.markers))
        except Exception as exc:
            self.loaded = False
            rospy.logerr_throttle(3,'[R4I RVIZ] Visualization only failed: %s',str(exc))


def main():
    rospy.init_node('demo11_r4i_tube_visualization',anonymous=False)
    app = CorridorView()
    rospy.Timer(rospy.Duration(.5),app.tick)
    rospy.spin()


if __name__=='__main__':
    main()
