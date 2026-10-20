"""Static assertions against the launch and scripts collected from contest1."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
class WiringTests(unittest.TestCase):
    def test_real_launch_has_three_source_nodes(self):
        root=ET.parse(ROOT/'reference/uav2_demo5_multi_waypoint_interface.launch').getroot()
        nodes={(n.get('name'),n.get('type'),n.get('pkg')) for n in root.findall('node')}
        self.assertIn(('trajectory_mux','trajectory_mux.py','ego_uav_interface'),nodes)
        self.assertIn(('point_traj_node','point_traj_node.py','ego_uav_interface'),nodes)
        self.assertIn(('ego_trajectory_bridge','ego_trajectory_bridge.py','ego_uav_interface'),nodes)
    def test_mux_and_downstream_topics_match(self):
        mux=(ROOT/'candidates/trajectory_mux.py').read_text()
        itf=(ROOT/'reference/trajectory_interface.py').read_text()
        self.assertIn('/contest1/ego_uav_interface/trajectory_setpoint',mux)
        self.assertIn('/contest1/ego_uav_interface/trajectory_setpoint',itf)
        self.assertIn('/contest1/mavros/setpoint_raw/local',mux)
        self.assertIn('/contest1/mavros/setpoint_raw/local',itf)
        self.assertIn('/contest1/ego_uav_interface/trajectory_interface_status',mux)
        self.assertIn('/contest1/ego_uav_interface/point_traj_status',mux)
    def test_existing_independent_hold_still_owned_by_interface(self):
        itf=(ROOT/'reference/trajectory_interface.py').read_text()
        self.assertIn('def make_hold_command(self):',itf)
        self.assertIn('def disable_trajectory(self, req):',itf)
        self.assertIn('self.trajectory_enabled = False',itf)
        self.assertIn('self.output_pub.publish(cmd)',itf)
    def test_runtime_package_path_was_identified(self):
        resolution=(ROOT/'reference/resolution.txt').read_text()
        self.assertIn('PKG=/home/yundrone/contest1_uav/runtime/ego_uav_ws/src/ego_uav_interface',resolution)

if __name__=='__main__':unittest.main()
