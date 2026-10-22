#!/usr/bin/env python3
"""R4B candidate runtime preflight gate. No software ARM or mode control.

--check uses ONLY ROS subscriber telemetry and XMLRPC process introspection.
--check is invoked before the R4B flight mission starts.
Does not perform the procedural no-prop RC-switch challenge; retains live RC and PX4 state gates.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import xmlrpc.client

D = Path(__file__).resolve().parent
ROOT = Path.home() / 'contest1_uav'
WS = ROOT / 'dev/projects/demo11_p2_guard_ws'
STAMP = D / '.r4b_launch_intent.json'
MASTER = 'http://192.168.8.101:11311'
BRIDGE_HASH = '4834c2964b7ef191940c492da8297146f97185597773910bea5c07e6c21c5769'
INTENT_MAX_AGE = 20 * 60


def stop(message):
    raise RuntimeError('[BLOCK] ' + message)


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def verify_static():
    check = subprocess.run([sys.executable, '-B', str(D / 'demo11_verify.py'), '--quiet'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
    if check.returncode:
        stop('R4 source integrity check failed: ' + check.stdout[-800:])
    expected = json.loads((D / 'DEMO11_SOURCE_MANIFEST.json').read_text())['source_sha256']
    if not all((D / x).is_file() and sha(D / x) == dig for x, dig in expected.items()):
        stop('Source altered after candidate installation')
    for x in ('run_demo11.sh', 'run_demo9.sh', 'start_demo9.sh', 'start_demo5.sh'):
        if 'r4_flight_gate.sh' not in (D / x).read_text():
            stop('Flight entrypoint missing live gate ' + x)


def node_command(name):
    import rosgraph
    uri = rosgraph.Master('/r4_flight_gate').lookupNode(name)
    answer = xmlrpc.client.ServerProxy(uri).getPid('/r4_flight_gate')
    if answer[0] != 1:
        stop('Cannot inspect ROS node ' + name)
    proc = Path('/proc') / str(answer[2])
    arguments = [x.decode('utf-8', 'replace') for x in (proc / 'cmdline').read_bytes().split(b'\0') if x]
    return arguments, os.path.realpath(str(proc / 'exe'))


def live_state():
    import rospy
    import rosgraph
    from mavros_msgs.msg import State, ExtendedState, RCIn
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Bool, String

    rospy.init_node('r4_site_gate', anonymous=True, disable_signals=True)
    def receive(topic, typ, timeout=4):
        try:
            return rospy.wait_for_message(topic, typ, timeout=timeout)
        except Exception as e:
            stop('ROS topic unavailable ' + topic + ': ' + repr(e))

    state = receive('/contest1/mavros/state', State)
    landed = receive('/contest1/mavros/extended_state', ExtendedState)
    battery = receive('/contest1/mavros/battery', BatteryState)
    rc = receive('/contest1/mavros/rc/in', RCIn)
    allow = receive('/contest1/ego_uav_interface/rc_mission_allowed', Bool)
    health = receive('/contest1/ego_uav_interface/flight_healthy', Bool)
    mux = receive('/contest1/ego_uav_interface/trajectory_mux_status', String)
    iface = receive('/contest1/ego_uav_interface/trajectory_interface_status', String)
    if not (state.connected and not state.armed and state.mode == 'POSCTL'):
        stop('PX4 must be CONNECTED DISARM POSCTL')
    if landed.landed_state != landed.LANDED_STATE_ON_GROUND:
        stop('PX4 must be ON_GROUND')
    if len(rc.channels) < 8 or rc.channels[6] > 1300 or rc.channels[5] < 1700:
        stop('CH6 HIGH(POSCTL) and CH7 LOW required; verify RC mapping')
    if allow.data or not health.data:
        stop('CH7 mission permission must be OFF and flight_healthy True')
    if mux.data != 'POINT_ACTIVE' or iface.data != 'HOLD_RC_BLOCKED':
        stop('Initial MUX/last-hop HOLD not safe: %r / %r' % (mux.data, iface.data))
    if not (battery.present and math.isfinite(battery.voltage)
            and 23.2 <= battery.voltage <= 25.5
            and math.isfinite(battery.percentage) and battery.percentage >= 0.5):
        stop('Battery telemetry outside REVIEWED 6S trial range; do not fake battery readings')
    graph = rosgraph.Master('/r4_flight_gate')
    pubs = dict(graph.getSystemState()[0])
    if pubs.get('/contest1/mavros/setpoint_raw/local', []) != ['/contest1/trajectory_interface']:
        stop('Multiple/incorrect MAVROS setpoint publishers')
    if pubs.get('/contest1/ego_uav_interface/trajectory_setpoint', []) != ['/contest1/trajectory_mux']:
        stop('Multiple/incorrect MUX setpoint publishers')

    paths = {
        '/contest1/trajectory_interface': D / 'demo8_interface_overlay/scripts/trajectory_interface_demo8.py',
        '/contest1/trajectory_mux': D / 'ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py',
    }
    for name, expected in paths.items():
        args, _ = node_command(name)
        if str(expected) not in args:
            stop(name + ' is not the candidate R4 process: ' + repr(args))
    _, planner = node_command('/contest1/drone_0_ego_planner_node')
    _, server = node_command('/contest1/drone_0_traj_server')
    if planner != str(WS / 'devel/lib/ego_planner/ego_planner_node') or server != str(WS / 'devel/lib/ego_planner/traj_server'):
        stop('Planner or Traj Server is not private guard build')
    if b'DEMO11_P2_GUARD' not in (WS / 'devel/lib/ego_planner/ego_planner_node').read_bytes():
        stop('Private Planner missing guarded sampler')
    args, _ = node_command('/contest1/ego_trajectory_bridge')
    src = [v for v in args if v.endswith('/ego_trajectory_bridge.py')]
    if len(src) != 1 or os.path.realpath(src[0]) not in {
        os.path.realpath(str(D / 'ros_src/ego_uav_interface/scripts/ego_trajectory_bridge.py')),
        os.path.realpath(str(ROOT / 'runtime/ego_uav_ws/src/ego_uav_interface/scripts/ego_trajectory_bridge.py')),
    } or sha(src[0]) != BRIDGE_HASH:
        stop('Bridge is not frozen reviewed source')
    base = '/contest1/drone_0_ego_planner_node/'
    for key, desired in [('grid_map/resolution', 0.15),
                         ('grid_map/obstacles_inflation', 0.299),
                         ('optimization/dist0', 0.30),
                         ('manager/max_vel', 0.20)]:
        val = rospy.get_param(base + key, None)
        if type(val) not in (int, float) or not math.isfinite(float(val)) or abs(float(val) - desired) > 1e-5:
            stop('Planner parameter mismatch %s=%s' % (key, val))
    print('[PASS] Live R4B private planner, bridge, MUX, final publisher, PX4 DISARM, RC, 6S telemetry')
    return state, battery, rc, receive




def record_launch_intent():
    # Not a pilot approval; ties frame data to this actual launch attempt.
    record = {
        'schema': 1,
        'started_unix': time.time(),
        'master_uri': os.environ.get('ROS_MASTER_URI'),
        'gate_sha256': sha(D / 'r4_flight_gate.py'),
        'manifest_sha256': sha(D / 'DEMO11_SOURCE_MANIFEST.json'),
    }
    temp = STAMP.with_name(STAMP.name + '.pending')
    temp.write_text(json.dumps(record, sort_keys=True) + '\n')
    os.chmod(temp, 0o600)
    os.replace(temp, STAMP)


def verify_launch_intent():
    if not STAMP.is_file() or STAMP.is_symlink():
        stop('No fresh R4B mission launch intent')
    record = json.loads(STAMP.read_text())
    if record.get('schema') != 1 or record.get('master_uri') != MASTER:
        stop('R4B mission launch intent invalid')
    if record.get('gate_sha256') != sha(D / 'r4_flight_gate.py') or \
       record.get('manifest_sha256') != sha(D / 'DEMO11_SOURCE_MANIFEST.json'):
        stop('R4B launch intent does not match current checked sources')
    age = time.time() - record['started_unix']
    if not (0 <= age <= INTENT_MAX_AGE):
        stop('R4B launch intent expired')
    return record['started_unix']


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--check', action='store_true')
    group.add_argument('--inspect', action='store_true')
    args = parser.parse_args()
    try:
        verify_static()
        if args.inspect:
            print('[PASS] R4B integrity and flight-gate scripts verified; flight not started')
            return 0
        if os.environ.get('ROS_MASTER_URI') != MASTER:
            stop('Unexpected ROS Master; require reviewed real hardware master')
        if args.check:
            live_state()
            record_launch_intent()
            print('[PASS] R4B live preflight; no RC switch challenge or no-prop approval required')
            print('[INFO] No automatic ARM. Pilot-controlled takeoff and all runtime RC protections unchanged.')
        return 0
    except (RuntimeError, ValueError, OSError, KeyError, TypeError,
            json.JSONDecodeError, AttributeError) as exc:
        print(str(exc), file=sys.stderr)
        return 97


if __name__ == '__main__':
    sys.exit(main())


def airborne_authorized(rospy):
    """Verify actual pilot ARM/OFFBOARD, fresh frame, and live safety inputs."""
    from mavros_msgs.msg import State
    from std_msgs.msg import Bool, String
    verify_static()
    start = verify_launch_intent()
    def get(topic, kind):
        try:
            return rospy.wait_for_message(topic, kind, timeout=3)
        except Exception as exc:
            stop('airborne authorization topic missing ' + topic + ': ' + str(exc))
    state = get('/contest1/mavros/state', State)
    if not state.connected or not state.armed or state.mode != 'OFFBOARD':
        stop('Mission only permitted after physical pilot takeoff (ARMED/OFFBOARD)')
    for topic in ('/contest1/ego_uav_interface/flight_healthy',
                  '/contest1/ego_uav_interface/rc_mission_allowed',
                  '/contest1/ego_uav_interface/demo_frame_safe',
                  '/contest1/yaw_tangent_demo/session_ready'):
        if not get(topic, Bool).data:
            stop('Airborne health/frame/pilot authorization not ready: ' + topic)
    if get('/contest1/ego_uav_interface/r3_corridor_status', String).data != 'SESSION_READY':
        stop('Final-hop corridor not bound to fresh session')
    session = D / 'session_frame.yaml'
    if not session.is_file() or session.is_symlink():
        stop('Mission frame missing')
    age = time.time() - session.stat().st_mtime
    if not (0 <= age < 900) or session.stat().st_mtime + 5 < start:
        stop('Flight frame stale or predates this R4B launch')
    print('[PASS] R4B flight: pilot takeoff, fresh frame, downstream gate and RC permissions ready')
