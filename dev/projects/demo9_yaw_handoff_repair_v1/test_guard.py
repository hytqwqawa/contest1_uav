import math, unittest
from types import SimpleNamespace as Obj
from yaw_guard_core import check, angle_delta

def msg(x,y,z,deg,mask=0,frame=1):
    return Obj(position=Obj(x=x,y=y,z=z),yaw=math.radians(deg),type_mask=mask,
               coordinate_frame=frame)

class GuardTests(unittest.TestCase):
    def test_recovered_bag_spike(self):
        ego=msg(-4.776221752,4.471584797,0.719291449,93.56,2048)
        point=msg(-4.719796658,3.517995596,0.70103693,-175.01)
        self.assertTrue(check(ego,point).startswith('position_jump_'))
    def test_bad_yaw_only(self):
        self.assertTrue(check(msg(0,0,1,93.56),msg(0,0,1,-175.01)).startswith('yaw_jump_'))
    def test_fresh_hold_ok(self):
        self.assertIsNone(check(msg(0,0,1,93.56),msg(0.02,0,1.01,92.0)))
    def test_wrap(self):
        self.assertAlmostEqual(math.degrees(angle_delta(math.radians(-179),math.radians(179))),2.)
        self.assertIsNone(check(msg(0,0,1,179),msg(0,0,1,-179)))
    def test_pos_jump(self):
        self.assertTrue(check(msg(0,0,1,0),msg(.5,0,1,0)).startswith('position_jump_'))
    def test_mask(self):
        self.assertEqual(check(msg(0,0,1,0),msg(0,0,1,0,1024)),'xyz_or_yaw_ignored')
    def test_frame(self):
        self.assertEqual(check(msg(0,0,1,0),msg(0,0,1,0,frame=8)),'frame_mismatch')
    def test_nan(self):
        self.assertEqual(check(msg(0,0,1,0),msg(float('nan'),0,1,0)),'nonfinite')

if __name__=='__main__': unittest.main(verbosity=2)
