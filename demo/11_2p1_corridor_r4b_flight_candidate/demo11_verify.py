#!/usr/bin/env python3
"""Demo11 immutable-source, route and optionally DISARMED ground checker.

No ARM, flight-mode change, ROS launch or MAVROS publication is performed.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml
from demo5_config import load_config
from demo11_corridor import load_corridor

ROOT = Path(__file__).resolve().parent
MANIFEST = ROOT / 'DEMO11_SOURCE_MANIFEST.json'
ROUTE = [
    {'forward':3.0, 'left':0.0, 'mode':'EGO', 'yaw_deg':0.0},
    {'forward':4.061, 'left':-0.439, 'mode':'EGO', 'yaw_deg':-135.0},
    {'forward':4.5, 'left':-1.5, 'mode':'EGO', 'yaw_deg':-180.0},
]


def fail(s):
    raise SystemExit('[STOP] Demo11 verification: ' + s)


def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def static_check(quiet=False):
    try:
        m = json.loads(MANIFEST.read_text())
        expected = m['source_sha256']
        for rel, digest in expected.items():
            p = ROOT / rel
            if (not p.is_file()) or p.is_symlink() or sha(p) != digest:
                fail('missing/modified source: ' + rel)
        cfg = load_config(ROOT / 'demo5_waypoints.yaml')
        if cfg['waypoints'] != ROUTE:
            fail('three EGO waypoint plan changed')
        if cfg['flight_height'] != 0.8:
            fail('flight height changed')
        c = load_corridor(ROOT / 'demo11_corridor.yaml')
        # Reported P1 bypass L=-0.878/-0.904 must remain rejected.
        try:
            c.check_center(1.94, -0.904)
        except ValueError:
            pass
        else:
            fail('known unsafe obstacle bypass unexpectedly accepted')
        c.check_center(3.0, 0.0)
        c.check_center(4.5, -1.5)
        for script in ('run_demo11.sh','run_demo9.sh','start_demo9.sh','start_demo5.sh'):
            if 'r4_flight_gate.sh' not in (ROOT / script).read_text():
                fail('R4 pilot/site gate missing from '+script)
        # 90-degree clockwise arc: C(F=3,L=-1.5), R=1.5.
        f2 = 3.0 + 1.5 / math.sqrt(2.0)
        l2 = -1.5 + 1.5 / math.sqrt(2.0)
        if math.hypot(ROUTE[1]['forward'] - f2,
                      ROUTE[1]['left'] - l2) > 0.001:
            fail('P2 not at agreed arc midpoint')
    except (KeyError, OSError, ValueError, TypeError, yaml.YAMLError) as e:
        fail(str(e))
    if not quiet:
        print('[PASS] Demo11 R3 GROUND REVIEW corridor/route/integrity: %d SHA256 files' % len(expected))
        for i, w in enumerate(ROUTE, 1):
            print('  P%d F%+.3f L%+.3f H0.800 EGO yaw=%+.1f deg' %
                  (i, w['forward'], w['left'], w['yaw_deg']))
        print('  [LIVE-GATED R4B CANDIDATE] RC procedural approval removed; no automatic ARM')


def mux_check():
    expected = str(ROOT / 'ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py')
    hits = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            argv = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if 'trajectory_mux_v4_only.py' in argv or '/scripts/trajectory_mux.py' in argv:
            if 'python' in argv and ('ego_uav_interface' in argv or expected in argv):
                hits.append((proc.name, argv))
    if len(hits) != 1 or expected not in hits[0][1]:
        fail('exactly ONE LIVE Demo11 V4 MUX required; found ' + repr([(x[0], x[1][:220]) for x in hits[:5]]))
    print('[PASS] Exactly one live Demo11 V4 MUX, pid ' + hits[0][0])


def ground_check():
    try:
        import rospy
        from mavros_msgs.msg import State
        from std_msgs.msg import Bool, String
    except ImportError as e:
        fail('ROS Noetic environment not sourced: ' + str(e))
    rospy.init_node('demo11_safe_ground_check', anonymous=True, disable_signals=True)

    def msg(topic, typ):
        try:
            return rospy.wait_for_message(topic, typ, timeout=5)
        except Exception as e:
            fail('missing ROS topic ' + topic + ': ' + str(e))

    state = msg('/contest1/mavros/state', State)
    rc = msg('/contest1/ego_uav_interface/rc_mission_allowed', Bool)
    healthy = msg('/contest1/ego_uav_interface/flight_healthy', Bool)
    mux = msg('/contest1/ego_uav_interface/trajectory_mux_status', String)
    interface = msg('/contest1/ego_uav_interface/trajectory_interface_status', String)
    if not state.connected or state.armed or state.mode != 'POSCTL':
        fail('PX4 must be CONNECTED + DISARMED + POSCTL')
    if rc.data or not healthy.data:
        fail('CH7 mission permission must be LOW and flight health must be True')
    if mux.data != 'POINT_ACTIVE' or interface.data != 'HOLD_RC_BLOCKED':
        fail('unexpected MUX/interface: %r / %r' % (mux.data, interface.data))
    # Live planner values MUST be re-read; the exported state snapshot did not
    # contain running Contest1 planner ROS parameters.
    base = '/contest1/drone_0_ego_planner_node/'
    params = {
        'grid_map/resolution': 0.15,
        'grid_map/obstacles_inflation': 0.299,
        'optimization/dist0': 0.30,
        'manager/max_vel': 0.20,
    }
    for suffix, desired in params.items():
        name = base + suffix
        try:
            found = rospy.get_param(name)
        except KeyError:
            fail('planner ROS parameter missing: ' + name)
        if (isinstance(found, bool) or not isinstance(found, (float, int)) or
                not math.isfinite(float(found)) or abs(float(found)-desired) > 1e-6):
            fail('planner ROS parameter unexpected: %s = %r, required %s' %
                 (name, found, desired))
    print('[PASS] Live EGO params: resolution=.15, inflation=.299, dist0=.30, speed=.20')
    print('[PASS] Ground PX4 DISARM/POSCTL, CH7 LOW, health True, POINT_ACTIVE/HOLD_RC_BLOCKED')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--quiet', action='store_true')
    ap.add_argument('--mux-only', action='store_true')
    ap.add_argument('--ground', action='store_true')
    args = ap.parse_args()
    static_check(args.quiet)
    if args.mux_only or args.ground:
        mux_check()
    if args.ground:
        ground_check()
