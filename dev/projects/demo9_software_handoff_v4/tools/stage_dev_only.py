#!/usr/bin/env python3
"""Stage V4 into a separate DEV copy; cannot edit the live runtime/Demo9.

This script intentionally performs no roslaunch, param writes, ROS service
calls, arming, or writes outside ~/contest1_uav/dev/projects.
"""
import argparse
import hashlib
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOME = Path.home()
DEV_ROOT = (HOME/'contest1_uav/dev/projects').resolve()
SOURCE_RUNTIME = HOME/'contest1_uav/runtime/ego_uav_ws/src/ego_uav_interface'
SOURCE_DEMO = HOME/'contest1_uav/demo/09_demo8_posctl_integrated_v1'
RUNTIME_SHA256 = '9f470c0268b542b30813351c841467a9b3acb11d71fa98cea6fc12e35bf9c17a'
MISSION_SHA256 = '5909777db45dccbfec13086e5e23f4220e5d2cb068be21d8371ccb5858f0e0d4'
POINT_SHA256 = 'a7432b439064429cc6c46eb393eaea224bdc60d0bd622b30b7369f1a7cb1b72f'
INTERFACE_SHA256 = '1bf2063afb421a68a285d157bcec8488dfa07f962b62fad6d6dbabe5a9bd2376'
LAUNCH_SHA256 = 'a17f6483d76e58cc71fba2de659b8dd52039c46980c5c98eebb699bd67cdab4f'

def sha256(path):
    obj = hashlib.sha256()
    with open(path,'rb') as handle:
        for block in iter(lambda:handle.read(1024*1024),b''):
            obj.update(block)
    return obj.hexdigest()

def inside(path, parent):
    try:
        path.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime',type=Path,default=SOURCE_RUNTIME)
    parser.add_argument('--demo',type=Path,default=SOURCE_DEMO)
    parser.add_argument('--out',type=Path,default=(DEV_ROOT/(
        'demo9_v4_stage_'+datetime.now().strftime('%Y%m%d_%H%M%S'))))
    args=parser.parse_args()
    runtime=args.runtime.resolve()
    demo=args.demo.resolve()
    out=args.out.resolve()
    if not inside(out,DEV_ROOT) or out == DEV_ROOT:
        raise SystemExit('[STOP] Output must be BELOW ~/contest1_uav/dev/projects')
    if out.exists():
        raise SystemExit('[STOP] Destination already exists; never overwrite: '+str(out))
    mux_orig=runtime/'scripts/trajectory_mux.py'
    mission_orig=demo/'uav2_demo5_waypoints.py'
    if not mux_orig.is_file() or not mission_orig.is_file():
        raise SystemExit('[STOP] Expected runtime/Demo9 files missing')
    checks=(('live runtime MUX',mux_orig,RUNTIME_SHA256),
            ('live POINT node',runtime/'scripts/point_traj_node.py',POINT_SHA256),
            ('live trajectory interface',runtime/'scripts/trajectory_interface.py',INTERFACE_SHA256),
            ('live Demo9 mission',mission_orig,MISSION_SHA256),
            ('live Demo9 launch',demo/'uav2_demo5_multi_waypoint_interface.launch',LAUNCH_SHA256))
    for label,p,expected in checks:
        if not p.is_file():
            raise SystemExit('[STOP] Missing reviewed source: '+str(p))
        actual=sha256(p)
        if actual!=expected:
            raise SystemExit('[STOP] %s SHA differs from reviewed copy: %s' % (label,actual))
    for filename in ('trajectory_mux.py','yaw_guard_core.py','uav2_demo5_waypoints.py'):
        candidate=ROOT/'candidates'/filename
        if not candidate.is_file():
            raise SystemExit('[STOP] Missing candidate '+filename)
        compile(candidate.read_bytes(),str(candidate),'exec')
    DEV_ROOT.mkdir(parents=True,exist_ok=True)
    out.mkdir()
    try:
        pkg=out/'ros_src/ego_uav_interface'
        shutil.copytree(runtime,pkg,ignore=shutil.ignore_patterns('__pycache__','*.pyc', '.git'))
        app=out/'demo9_app'
        app.mkdir()
        shutil.copy2(ROOT/'candidates/trajectory_mux.py',pkg/'scripts/trajectory_mux.py')
        shutil.copy2(ROOT/'candidates/yaw_guard_core.py',pkg/'scripts/yaw_guard_core.py')
        shutil.copy2(ROOT/'candidates/uav2_demo5_waypoints.py',app/'uav2_demo5_waypoints.py')
        # Non-flight dependencies are copied so Python imports can later be
        # inspected in DEV. Do not clone a running launch script by default.
        for name in ('demo5_config.py','demo8_terminal_yaw.py','demo5_waypoints.yaml'):
            source=demo/name
            if source.exists():
                shutil.copy2(source,app/name)
        manifest={
            'status':'DEV_SOURCE_ONLY_NOT_DEPLOYED',
            'runtime_source':str(runtime),
            'mission_source':str(demo),
            'sha_before':{str(p):expected for _,p,expected in checks},
            'dev_outputs':{
                'mux':str(pkg/'scripts/trajectory_mux.py'),
                'guard':str(pkg/'scripts/yaw_guard_core.py'),
                'mission':str(app/'uav2_demo5_waypoints.py')},
        }
        (out/'dev_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        (out/'READ_FIRST.txt').write_text(
            'DEV SOURCE ONLY. DO NOT launch on actual aircraft.\n'
            'This is a separate ROS package snapshot and mission script.\n'
            'It is not installed in the active ROS_PACKAGE_PATH.\n'
            'The official Demo9 and runtime were not changed.\n'
            'No power-on, propeller, OFFBOARD, ARM, or flight tests were run.\n')
        assert all(sha256(p)==expected for _,p,expected in checks)
    except Exception:
        # Destination was newly created; leave no half-staged package.
        shutil.rmtree(out,ignore_errors=True)
        raise
    print('[OK] Separate DEV source stage:',out)
    print('[OK] Reviewed live runtime/Demo9 SHA matched and stayed unchanged')
    print('[INFO] No active code was deployed. No commands sent to PX4.')
    print('[REVIEW]',out/'dev_manifest.json')

if __name__=='__main__':
    main()
