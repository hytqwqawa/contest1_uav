"""Static safety contract for the optional loopback-only ROS smoke runner."""
import ast
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]

class LoopbackSafety(unittest.TestCase):
    def test_wrapper_forces_isolated_master(self):
        shell=(ROOT/'tools/isolated_ros_smoke.sh').read_text()
        self.assertIn('127.0.0.1',shell)
        self.assertIn('ROS_MASTER_URI',shell)
        self.assertIn('11457',shell)
        self.assertIn('trap cleanup',shell)
        for s in ('roslaunch mavros','rosrun mavros','/mavros/cmd/arming',
                  'set_mode','killall','pkill','sudo '):
            self.assertNotIn(s,shell)
    def test_simulator_never_commands_px4(self):
        src=(ROOT/'tools/ros_smoke_synthetic.py').read_text()
        tree=ast.parse(src)
        names=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertIn('Publisher',names)
        self.assertNotIn('Popen',names)
        for s in ("'/contest1/mavros/cmd/arming'","'/contest1/mavros/set_mode'",'roslaunch px4'):
            self.assertNotIn(s,src)
        self.assertIn("EXPECTED_MASTER='http://127.0.0.1:11457'",src)

if __name__=='__main__':unittest.main()
