"""Test V4 staging without touching an actual contest1 install."""
import ast
import contextlib
import hashlib
import importlib.util
import io
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('stage_dev_only_v4',ROOT/'tools/stage_dev_only.py')
stage=importlib.util.module_from_spec(spec)
spec.loader.exec_module(stage)

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

class StageSafety(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.root=Path(self.tmp.name)
        self.runtime=self.root/'runtime/ego_uav_interface'
        (self.runtime/'scripts').mkdir(parents=True)
        original=ROOT/'reference/reviewed_originals'
        shutil.copy2(original/'trajectory_mux.py',self.runtime/'scripts/trajectory_mux.py')
        shutil.copy2(ROOT/'reference/point_traj_node.py',self.runtime/'scripts/point_traj_node.py')
        shutil.copy2(ROOT/'reference/trajectory_interface.py',self.runtime/'scripts/trajectory_interface.py')
        self.demo=self.root/'demo9'
        self.demo.mkdir()
        shutil.copy2(original/'uav2_demo5_waypoints.py',self.demo/'uav2_demo5_waypoints.py')
        shutil.copy2(ROOT/'reference/uav2_demo5_multi_waypoint_interface.launch',
                     self.demo/'uav2_demo5_multi_waypoint_interface.launch')
        self.dev=self.root/'dev/projects'
        self.dev.mkdir(parents=True)
        self.old_dev=stage.DEV_ROOT
        stage.DEV_ROOT=self.dev
        self.old_argv=sys.argv
    def tearDown(self):
        stage.DEV_ROOT=self.old_dev
        sys.argv=self.old_argv
        self.tmp.cleanup()
    def invoke(self,out):
        sys.argv=['stage_dev_only.py','--runtime',str(self.runtime),
                  '--demo',str(self.demo),'--out',str(out)]
        with contextlib.redirect_stdout(io.StringIO()):stage.main()
    def test_creates_isolated_v4_stage_preserving_inputs(self):
        before=sha(self.runtime/'scripts/trajectory_mux.py'),sha(self.demo/'uav2_demo5_waypoints.py')
        out=self.dev/'candidate'
        self.invoke(out)
        self.assertTrue((out/'dev_manifest.json').is_file())
        self.assertEqual((sha(self.runtime/'scripts/trajectory_mux.py'),
                          sha(self.demo/'uav2_demo5_waypoints.py')),before)
        self.assertEqual(sha(out/'ros_src/ego_uav_interface/scripts/trajectory_mux.py'),
                         sha(ROOT/'candidates/trajectory_mux.py'))
        self.assertEqual(sha(out/'demo9_app/uav2_demo5_waypoints.py'),
                         sha(ROOT/'candidates/uav2_demo5_waypoints.py'))
    def test_refuses_to_overwrite_existing_directory(self):
        out=self.dev/'candidate';out.mkdir()
        with self.assertRaisesRegex(SystemExit,'already exists'):
            self.invoke(out)
    def test_refuses_active_runtime_destination(self):
        with self.assertRaisesRegex(SystemExit,'BELOW'):
            self.invoke(self.runtime)
    def test_refuses_source_mismatch_without_writing(self):
        with open(self.runtime/'scripts/trajectory_mux.py','a') as fp:fp.write('# a changed site version\n')
        out=self.dev/'candidate'
        with self.assertRaisesRegex(SystemExit,'SHA differs'):
            self.invoke(out)
        self.assertFalse(out.exists())
    def test_staging_tool_is_not_a_flight_launcher(self):
        tree=ast.parse((ROOT/'tools/stage_dev_only.py').read_text())
        calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        for prohibited in ('ServiceProxy','roslaunch','system','Popen','run'):
            self.assertNotIn(prohibited,calls)
    def test_link_witness_only_subscribes(self):
        code=(ROOT/'tools/software_link_witness.py').read_text()
        tree=ast.parse(code)
        names=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
        self.assertIn('Subscriber',names)
        for prohibited in ('Publisher','Service','ServiceProxy','set_mode','arming'):
            self.assertNotIn(prohibited,names)

if __name__=='__main__':unittest.main()
