import math
import sys
import types
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'candidates'))
from yaw_guard_core import angle_delta,check

def pt(x,y,z,yaw,mask=0,frame=1):
    return types.SimpleNamespace(position=types.SimpleNamespace(x=x,y=y,z=z),yaw=math.radians(yaw),type_mask=mask,coordinate_frame=frame)
class ActualGuardTests(unittest.TestCase):
    def test_old_point_incident(self):
        ego=pt(-4.77622,4.47158,.71929,93.56)
        old=pt(-4.7198,3.518,.701,-175.01)
        self.assertTrue(check(ego,old).startswith('position_jump_'))
    def test_good_point_switch(self):
        self.assertIsNone(check(pt(0,0,1,93),pt(.02,0,1,93)))
    def test_finite(self):
        self.assertEqual(check(None,pt(float('nan'),0,1,0)),'nonfinite')
    def test_frame(self):
        self.assertEqual(check(pt(0,0,1,0),pt(0,0,1,0,frame=2)),'unsupported_coordinate_frame')
    def test_yaw_wrap(self):
        self.assertAlmostEqual(math.degrees(abs(angle_delta(math.radians(-179),math.radians(179)))),2.0)
    def test_yaw_spike(self):
        self.assertTrue(check(pt(0,0,1,0),pt(0,0,1,92)).startswith('yaw_jump_'))
if __name__=='__main__':unittest.main()
