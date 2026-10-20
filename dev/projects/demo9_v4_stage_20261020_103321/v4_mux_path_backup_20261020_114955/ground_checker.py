#!/usr/bin/env python3
"""Reversible V4 shadow launch preparation and DISARMED software gate.

Usage (on contest1):
  python3 -B ~/Desktop/demo9_v4_fast_live_prep.py prepare
  source ~/contest1_uav/runtime/env.sh
  python3 -B ~/Desktop/demo9_v4_fast_live_prep.py check --seconds 120

NEVER arms, changes PX4 parameters, or runs a flight mission.
No writes to ~/contest1_uav/runtime or the original Demo9 directory.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

HOME = Path.home()
BASE = HOME / 'contest1_uav/demo/09_demo8_posctl_integrated_v1'
RUNTIME = HOME / 'contest1_uav/runtime/ego_uav_ws/src/ego_uav_interface'
DEVROOT = HOME / 'contest1_uav/dev/projects'
EXPECTED = {
    'trajectory_mux.py': 'ea1f5a12fc04d55abe883ec9cebe9203fdd0f45b67e885b4e7a0d43f80689bb0',
    'yaw_guard_core.py': 'f36dcf84faac9fcd9ee39ad4ae6f5e16e27b4d40a32a088b20146d1f7a51ebc0',
    'uav2_demo5_waypoints.py': '5c7a473305ec25ac2312acabdbb04534f810a40e36a214b9bed61354ccb4a705',
}


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def die(msg):
    raise SystemExit('[STOP] ' + msg)


def stage_path():
    pref = 'demo9_v4_stage_'
    stages = sorted((p for p in DEVROOT.glob(pref + '*') if p.is_dir()), reverse=True)
    if not stages:
        die('V4 DEV stage missing')
    return stages[0].resolve()


def stage_files(stage):
    pkg = stage/'ros_src/ego_uav_interface'
    paths = {
        'trajectory_mux.py': pkg/'scripts/trajectory_mux.py',
        'yaw_guard_core.py': pkg/'scripts/yaw_guard_core.py',
        'uav2_demo5_waypoints.py': stage/'demo9_app/uav2_demo5_waypoints.py',
    }
    for name, f in paths.items():
        if not f.is_file() or sha(f) != EXPECTED[name]:
            die('V4 source missing or hash mismatch: ' + str(f))
    manifest = stage/'dev_manifest.json'
    if not manifest.is_file():
        die('DEV manifest missing')
    m = json.loads(manifest.read_text())
    if m.get('status') != 'DEV_SOURCE_ONLY_NOT_DEPLOYED':
        die('Unknown DEV manifest')
    for original, expected in m.get('sha_before', {}).items():
        p = Path(original)
        if not p.is_file() or sha(p) != expected:
            die('Reviewed source changed: ' + str(p))
    return paths


def replace_live_path(txt, shadow):
    """Never modify original launchers; substitute literal base paths in copies."""
    candidates = (
        str(BASE),
        '$HOME/contest1_uav/demo/09_demo8_posctl_integrated_v1',
        '${HOME}/contest1_uav/demo/09_demo8_posctl_integrated_v1',
    )
    for item in candidates:
        txt = txt.replace(item, str(shadow))
    return txt


def prepare():
    stage = stage_path()
    files = stage_files(stage)
    if not BASE.is_dir():
        die('Original Demo9 directory missing')
    launcher_names = ('start_core.sh', 'start_backend.sh', 'run_demo9.sh',
                      'start_demo9.sh', 'start_demo5.sh')
    originals = {}
    for name in launcher_names:
        p = BASE/name
        if not p.is_file():
            die('Original launcher missing: ' + str(p))
        originals[name] = p.read_text()
    if '"$D/start_demo9.sh"' not in originals['run_demo9.sh']:
        die('run_demo9.sh entrypoint not recognized')
    if '"$D/start_demo5.sh"' not in originals['start_demo9.sh']:
        die('start_demo9.sh entrypoint not recognized')
    mission_runner = originals['start_demo5.sh']
    # A copied wrapper must resolve the mission via the copied wrapper's
    # own directory; never silently run the original competition mission.
    if not any(tok in mission_runner for tok in (
            '$D/uav2_demo5_waypoints.py',
            '${D}/uav2_demo5_waypoints.py',
            '$DEMO/uav2_demo5_waypoints.py',
            '${DEMO}/uav2_demo5_waypoints.py',
            '$DBG/uav2_demo5_waypoints.py',
            '${DBG}/uav2_demo5_waypoints.py')):
        die('start_demo5.sh mission reference is not proven DEV-relative. '
            'Show: grep -n -E "uav2_demo5_waypoints|^D=|^DEMO=" '
            + str(BASE/'start_demo5.sh'))
    if 'BASH_SOURCE' not in mission_runner and 'dirname "$0"' not in mission_runner:
        die('start_demo5.sh does not establish a script-relative directory')
    backend = originals['start_backend.sh']
    marker = 'source "$ROOT/runtime/env.sh"'
    if backend.count(marker) != 1:
        die('Unexpected backend source env layout')
    if 'roslaunch \\' not in backend or 'uav2_demo5_multi_waypoint_interface.launch' not in backend:
        die('Backend launch chain changed')
    if 'ROS_PACKAGE_PATH' in backend:
        die('Backend already customizes ROS_PACKAGE_PATH; review required')

    shadow = stage/'live_candidate_demo9'
    if shadow.exists():
        die('Shadow already exists. Refusing to overwrite: ' + str(shadow))
    tmp = stage/'live_candidate_demo9.tmp'
    if tmp.exists():
        die('Incomplete prior temporary shadow exists: ' + str(tmp))
    tmp.mkdir()
    try:
        for entry in BASE.iterdir():
            target = tmp/entry.name
            if entry.name in launcher_names:
                shutil.copy2(entry, target)
            elif entry.name == 'uav2_demo5_waypoints.py':
                shutil.copy2(files['uav2_demo5_waypoints.py'], target)
            else:
                target.symlink_to(entry)

        # The backend must use V4 ROS package, not the original runtime package.
        # Inject AFTER env.sh to prevent its setup from overriding the overlay.
        overlay = '\n'.join([
            'V4_PKG='+json.dumps(str(stage/'ros_src/ego_uav_interface')),
            'export ROS_PACKAGE_PATH='+json.dumps(str(stage/'ros_src'))+':${ROS_PACKAGE_PATH:-}',
            'rospack profile >/dev/null 2>&1 || { echo "[STOP] rospack profile failed"; exit 96; }',
            'V4_FOUND="$(rospack find ego_uav_interface 2>/dev/null)"',
            '[ "$V4_FOUND" = "$V4_PKG" ] || { echo "[STOP] MUX package is NOT V4: $V4_FOUND"; exit 96; }',
            'echo "[V4 VERIFIED] ROS package: $V4_FOUND"',
        ])
        patched_backend = replace_live_path(backend.replace(marker, marker+'\n'+overlay, 1), shadow)
        (tmp/'start_backend.sh').write_text(patched_backend)
        (tmp/'start_backend.sh').chmod((BASE/'start_backend.sh').stat().st_mode & 0o777)

        # Mission activation must abort if the backend still runs the original MUX.
        run = replace_live_path(originals['run_demo9.sh'], shadow)
        needle = 'source "$ROOT/runtime/env.sh"'
        if run.count(needle) != 1:
            die('Unexpected mission runner env source layout')
        mux_path = str(files['trajectory_mux.py'])
        gate = '\n'.join([
            'V4_MUX='+json.dumps(mux_path),
            'if ! pgrep -fa "$V4_MUX" >/dev/null 2>&1; then',
            '  echo "[STOP] Running backend is not V4 MUX; mission NOT started"',
            '  exit 96',
            'fi',
            'echo "[V4 VERIFIED] Live MUX process uses $V4_MUX"',
        ])
        run = run.replace(needle, needle+'\n'+gate, 1)
        (tmp/'run_demo9.sh').write_text(run)
        (tmp/'run_demo9.sh').chmod((BASE/'run_demo9.sh').stat().st_mode & 0o777)

        # Path references in the mission wrapper are redirected only in shadow.
        for name in ('start_demo9.sh', 'start_demo5.sh'):
            path = tmp/name
            path.write_text(replace_live_path(originals[name], shadow))
            path.chmod((BASE/name).stat().st_mode & 0o777)
        subprocess.run(['bash','-n',*[str(tmp/n) for n in launcher_names]], check=True)
        for name, original in originals.items():
            if sha(BASE/name) != hashlib.sha256(original.encode()).hexdigest():
                die('Original script changed while preparing: ' + name)
        if sha(tmp/'uav2_demo5_waypoints.py') != EXPECTED['uav2_demo5_waypoints.py']:
            die('Shadow mission version mismatch')
        (tmp/'SHADOW_READY.txt').write_text(
            'V4 DEV shadow launchers. No original runtime or Demo9 source was changed.\n'
            'Core: original Demo9/start_core.sh.\n'
            'Backend: this directory/start_backend.sh (V4 package).\n'
            'Mission: this directory/run_demo9.sh (V4 mission).\n'
            'NOT an airworthiness approval. Ground tests and independent flight safety review required.\n')
        tmp.rename(shadow)
    except Exception:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    print('[OK] Reversible V4 launch shadow:', shadow)
    print('[OK] Original runtime/Demo9 untouched')
    print('TERMINAL 1:', BASE/'start_core.sh')
    print('TERMINAL 2:', shadow/'start_backend.sh')
    print('GROUND CHECK: python3 -B ~/Desktop/demo9_v4_fast_live_prep.py check --seconds 120')
    print('TERMINAL 3 (ONLY after independent flight approval):', shadow/'run_demo9.sh')
    print('ROLLBACK: stop the 3 terminals, then use the original Demo9 launch scripts.')


def check(seconds):
    stage = stage_path()
    stage_files(stage)
    shadow = stage/'live_candidate_demo9'
    if not (shadow/'SHADOW_READY.txt').is_file():
        die('No prepared V4 shadow')
    expected = str(stage/'ros_src/ego_uav_interface/scripts/trajectory_mux.py')
    procs = []
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():
            continue
        try:
            argv = (p/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if expected in argv and 'trajectory_mux.py' in argv:
                procs.append((p.name, argv[:220]))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    if len(procs) != 1:
        die('Need exactly one ACTIVE V4 MUX process; found %d: %s' % (len(procs), procs))
    print('[OK] Active V4 MUX process:', procs[0][0], flush=True)
    try:
        import rospy
        from mavros_msgs.msg import State, PositionTarget
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Bool, String
    except ImportError:
        die('Run `source ~/contest1_uav/runtime/env.sh` before ground check')

    topics = {
        'px4': ('/contest1/mavros/state', State),
        'odom': ('/contest1/mavros/local_position/odom', Odometry),
        't265': ('/contest1/t265/odom/filtered', Odometry),
        'imu': ('/contest1/mavros/imu/data', Imu),
        'target': ('/contest1/mavros/setpoint_raw/local', PositionTarget),
        'health': ('/contest1/ego_uav_interface/flight_healthy', Bool),
        'frame': ('/contest1/ego_uav_interface/demo_frame_safe', Bool),
        'mux': ('/contest1/ego_uav_interface/trajectory_mux_status', String),
        'interface': ('/contest1/ego_uav_interface/trajectory_interface_status', String),
    }
    samples = {key: [] for key in topics}
    lastmsg = {}
    start = time.monotonic()
    rospy.init_node('demo9_v4_ground_gate', anonymous=True, disable_signals=True)
    subs = []
    for key, (topic, msgtype) in topics.items():
        def callback(msg, k=key):
            now = time.monotonic()
            samples[k].append(now)
            lastmsg[k] = (now, msg)
        subs.append(rospy.Subscriber(topic, msgtype, callback, queue_size=50))
    print('[CHECK] Monitoring live software for %.0fs; propellers REMOVED, DISARMED...' % seconds, flush=True)
    while time.monotonic() - start < seconds:
        time.sleep(0.2)
        if rospy.is_shutdown():
            die('ROS shutdown during ground verification')
        if 'px4' in lastmsg and lastmsg['px4'][1].armed:
            die('Aircraft ARMED during ground-only test')
    now = time.monotonic()
    elapsed = now - start
    failures = []
    for key, (topic, _) in topics.items():
        arr = samples[key]
        hz = len(arr)/elapsed
        age = now - lastmsg[key][0] if key in lastmsg else float('inf')
        val = lastmsg[key][1] if key in lastmsg else None
        display = getattr(val, 'data', '') if key in ('health','frame','mux','interface') else ''
        print('%-10s count=%5d Hz=%5.1f age=%4.2fs  %s' % (key, len(arr), hz, age, display))
        if age > (1.5 if key == 'px4' else 0.5):
            failures.append('%s missing/stale' % key)
        if key in ('odom','t265','target') and hz < 15:
            failures.append('%s rate below 15Hz' % key)
        if key == 'imu' and hz < 20:
            failures.append('IMU rate below 20Hz')
    px = lastmsg.get('px4', (None,None))[1]
    if px is None or not px.connected or px.armed or px.mode != 'POSCTL':
        failures.append('PX4 must be connected DISARMED POSCTL')
    for key in ('health','frame'):
        obj = lastmsg.get(key,(None,None))[1]
        if obj is None or obj.data is not True:
            failures.append('%s is not True' % key)
    # Ground-only control-source status can be STALE until a mission is running,
    # but guard rejection at idle is suspicious and must not be ignored.
    mux = lastmsg.get('mux',(None,None))[1]
    if mux is not None and 'GUARD_REJECTED' in mux.data:
        failures.append('MUX guard is rejecting commands')
    if failures:
        for f in failures:
            print('[STOP]', f)
        raise SystemExit(2)
    print('[PASS] Live V4 shadow, ROS stream, frame and flight-health GROUND checks')
    print('This is NOT a flight approval or proof against the prior OFFBOARD dropout.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('prepare')
    ck = sub.add_parser('check')
    ck.add_argument('--seconds', type=int, default=120)
    args = p.parse_args()
    if args.command == 'prepare':
        prepare()
    else:
        if not 15 <= args.seconds <= 900:
            die('Ground sample seconds must be between 15 and 900')
        check(args.seconds)

if __name__ == '__main__':
    main()
