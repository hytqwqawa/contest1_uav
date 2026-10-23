#!/usr/bin/env python3
"""R4Q terminal translation gate invariants. Never connects to ROS/PX4."""
import ast, math, pathlib, unittest
SRC=pathlib.Path(__file__).with_name('uav2_demo5_waypoints.py')
TREE=ast.parse(SRC.read_text())
CLASS=next(x for x in TREE.body if isinstance(x,ast.ClassDef) and x.name=='TangentPreview')
METHOD=next(x for x in CLASS.body if isinstance(x,ast.FunctionDef) and x.name=='execute_direct')

# Execute ONLY assignment expressions relevant to the actual arrival gate.
assigns=[]
for node in ast.walk(METHOD):
    if isinstance(node,ast.Assign):
        names=[t.id for t in node.targets if isinstance(t,ast.Name)]
        if any(n in ('final_yaw_handoff','arrival_xy_tol','arrival_stable_s','arrived') for n in names):
            assigns.append(node)
assert len(assigns)==4, 'R4Q gate changed; revise tests'
compiled=compile(ast.Module(body=sorted(assigns,key=lambda x:x.lineno),type_ignores=[]),str(SRC),'exec')

class Waypoint:
    def __init__(self,index,total):self.active_wp_num=index;self.wp_total=total

def calc(index,total,has_yaw,dxy,dz=0.03,speed=0.06):
    loc=dict(self=Waypoint(index,total),terminal_yaw_world_deg=0.0 if has_yaw else None,
             dxy=dxy,dz=dz,speed=speed)
    exec(compiled,{},loc)
    return loc['arrived'],loc['arrival_stable_s'],loc['arrival_xy_tol']

class GateTests(unittest.TestCase):
    def test_final_entry_0148(self):
        self.assertEqual(calc(3,3,True,0.148),(True,0.20,0.15))
    def test_final_outside_limit(self):
        self.assertFalse(calc(3,3,True,0.152)[0])
    def test_final_height_speed_unchanged(self):
        self.assertFalse(calc(3,3,True,0.13,dz=0.081)[0])
        self.assertFalse(calc(3,3,True,0.13,speed=0.101)[0])
    def test_prior_yaw_legs_unchanged(self):
        self.assertEqual(calc(1,3,True,0.13),(False,0.80,0.12))
        self.assertEqual(calc(2,3,True,0.11),(True,0.80,0.12))
    def test_unconstrained_legacy_unchanged(self):
        self.assertEqual(calc(3,3,False,0.14),(True,0.80,0.15))
    def test_terminal_yaw_gate_still_one_second(self):
        from demo8_terminal_yaw import TerminalYawGate
        self.assertAlmostEqual(TerminalYawGate((0,0,.8),0).stable_s,1.0)
    def test_original_protection_not_edited(self):
        s=SRC.read_text()
        self.assertIn('if stable >= arrival_stable_s:',s)
        self.assertIn('stable=%.2f/%.2f',s)
        self.assertIn('flight_healthy=False during execution',s)
        self.assertIn('mission permission lost',s)
        self.assertIn('Demo-frame guard UNSAFE',s)

if __name__=='__main__':unittest.main(verbosity=2)
