#!/usr/bin/env python3
"""ROS-free source/geometry safety invariants; does not launch an aircraft."""
import hashlib
import json
import math
import unittest
from pathlib import Path

import yaml
from demo5_config import load_config
from demo11_corridor import load_corridor
from demo11_rviz_geometry import world_point, quarter_arc, round_tube_ring, static_scene_definitions

ROOT=Path(__file__).resolve().parent

class ViewTests(unittest.TestCase):
    def test_exact_three_points(self):
        points=load_config(ROOT/'demo5_waypoints.yaml')['waypoints']
        self.assertEqual(points,[
            {'forward':3.5,'left':0.25,'mode':'EGO','yaw_deg':0.0},
            {'forward':4.061,'left':-0.439,'mode':'EGO','yaw_deg':-135.0},
            {'forward':4.5,'left':-1.5,'mode':'EGO','yaw_deg':-180.0}])
    def test_physical_center_limits_not_changed(self):
        c=load_corridor(ROOT/'demo11_corridor.yaml')
        self.assertEqual((c.f_start,c.f_end,c.l_min,c.l_max),(1.,3.,-.5,1.))
        self.assertAlmostEqual(c.center_l_min,-.15)
        self.assertAlmostEqual(c.center_l_max,.65)
        self.assertAlmostEqual(c.f_guard_max,3.2)
    def test_arc_reference_not_safety_boundary(self):
        c=load_corridor(ROOT/'demo11_corridor.yaml')
        m=json.loads((ROOT/'DEMO11_SOURCE_MANIFEST.json').read_text())
        g=static_scene_definitions(c,m['reference_arc'])
        self.assertFalse(g['arc_is_physical_safety_boundary'])
        self.assertEqual(g['arc_reference_center'],(3.,-1.5))
        self.assertEqual(g['arc_reference_radius'],1.5)
    def test_90deg_arc_geometry(self):
        pts=quarter_arc()
        self.assertEqual(len(pts),73)
        self.assertAlmostEqual(pts[0][0],3.,places=7)
        self.assertAlmostEqual(pts[0][1],0.,places=7)
        self.assertAlmostEqual(pts[-1][0],4.5,places=7)
        self.assertAlmostEqual(pts[-1][1],-1.5,places=7)
        for f,l in pts:
            self.assertAlmostEqual(math.hypot(f-3,l+1.5),1.5,places=7)
    def test_round_tube_ribs(self):
        pts=round_tube_ring(3.,-1.5,1.5,.75,45.,.8,24)
        self.assertEqual(len(pts),25)
        self.assertAlmostEqual(pts[0][0],pts[-1][0],places=7)
        self.assertAlmostEqual(pts[0][1],pts[-1][1],places=7)
    def test_world_transform_current_session(self):
        session={'origin_world':{'x':12.,'y':-2.,'z':.03},
                 'forward_world':{'x':0.,'y':1.},
                 'left_world':{'x':-1.,'y':0.}}
        value=world_point(session,3.5,.25,.8)
        for found,target in zip(value,(11.75,1.5,.83)):
            self.assertAlmostEqual(found,target,places=9)
    def test_visualizer_no_flight_commands(self):
        vis=(ROOT/'rviz_corridor_tubes.py').read_text()
        self.assertIn('MarkerArray',vis)
        self.assertIn("m.header.frame_id = 'world'",vis)
        self.assertIn('demo5_plan_snapshot',vis)
        for forbidden in ('mavros/cmd/arming','/setpoint_raw/local','ServiceProxy(',
                          'CommandBool','SetMode(', 'request_offboard('):
            self.assertNotIn(forbidden,vis)
    def test_standalone_rviz_display(self):
        d=yaml.safe_load((ROOT/'yaw_tangent_demo.rviz').read_text())
        self.assertEqual(d['Visualization Manager']['Global Options']['Fixed Frame'],'world')
        displays=d['Visualization Manager']['Displays']
        topics=[x.get('Marker Topic') for x in displays]
        self.assertIn('/contest1/demo11_r4i/geometry_markers',topics)
    def test_startup_autopublish_then_original_rviz(self):
        s=(ROOT/'start_demo5.sh').read_text()
        self.assertIn('rviz_corridor_tubes.py',s)
        self.assertIn('CORRIDOR_VIZ_PID',s)
        self.assertIn('yaw_tangent_demo.rviz',s)
        self.assertIn('r4_flight_gate.sh',s)
    def test_frozen_manifest(self):
        m=json.loads((ROOT/'DEMO11_SOURCE_MANIFEST.json').read_text())
        self.assertEqual(m['route'][0]['forward'],3.5)
        for path,h in m['source_sha256'].items():
            with self.subTest(path=path):
                self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(),h)

if __name__=='__main__':unittest.main(verbosity=2)
