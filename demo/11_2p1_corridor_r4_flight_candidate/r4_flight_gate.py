#!/usr/bin/env python3
"""R4 candidate ground/pilot release gate. No software ARM or mode control.

--prepare uses ONLY subscribers and ROS XMLRPC read-only introspection.
--check is invoked repeatedly before the R4 flight mission starts.
Real props/pilot/site control CANNOT be verified in software; site signoff is required.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import xmlrpc.client

D = Path(__file__).resolve().parent
ROOT = Path.home() / 'contest1_uav'
WS = ROOT / 'dev/projects/demo11_p2_guard_ws'
STAMP = D / '.r4_site_release.json'
MASTER = 'http://192.168.8.101:11311'
BRIDGE_HASH = '4834c2964b7ef191940c492da8297146f97185597773910bea5c07e6c21c5769'
APPROVAL_MAX_AGE = 45 * 60


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
            stop('Flight entrypoint missing site gate ' + x)


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
    print('[PASS] Live R4 private planner, bridge, MUX, final publisher, PX4 DISARM, RC, 6S telemetry')
    return state, battery, rc, receive


def classify(pwm):
    if pwm <= 1300: return 'LOW'
    if pwm >= 1700: return 'HIGH'
    return 'MID'


def has_subsequence(seq, expected):
    i = 0
    for x in seq:
        if x == expected[i]:
            i += 1
            if i == len(expected):
                return True
    return False


def rc_switch_challenge():
    """Record actual RC switch transitions without arming or mode commands."""
    import rospy
    from mavros_msgs.msg import State, RCIn
    trace = {6: [], 7: []}
    last_class = {6: None, 7: None}
    candidate = {6: None, 7: None}
    candidate_at = {6: 0, 7: 0}
    latest_rc = None
    latest_state = None
    armed_seen = False
    def state_cb(msg):
        nonlocal latest_state, armed_seen
        latest_state = msg
        if msg.armed:
            armed_seen = True
    def rc_cb(msg):
        nonlocal latest_rc
        latest_rc = msg
        if len(msg.channels) < 8:
            return
        for ch, index in ((6, 5), (7, 6)):
            kind = classify(msg.channels[index])
            t = time.monotonic()
            if candidate[ch] != kind:
                candidate[ch] = kind
                candidate_at[ch] = t
            elif last_class[ch] != kind and t - candidate_at[ch] > 0.25:
                last_class[ch] = kind
                trace[ch].append(kind)
                print('[RC] CH%d %s' % (ch, kind), flush=True)
    sub1 = rospy.Subscriber('/contest1/mavros/state', State, state_cb, queue_size=20)
    sub2 = rospy.Subscriber('/contest1/mavros/rc/in', RCIn, rc_cb, queue_size=30)
    try:
        print('[NO-PROPS] Do not ARM. Demonstrate CH6 HIGH -> LOW -> HIGH, then CH7 LOW -> MID -> HIGH -> LOW.')
        print('[NO-PROPS] Stop immediately if motors move or PX4 state becomes ARMED.')
        expiry = time.monotonic() + 70
        while time.monotonic() < expiry:
            if armed_seen:
                stop('PX4 ARMED during no-props RC test')
            if has_subsequence(trace[6], ('HIGH', 'LOW', 'HIGH')) and \
               has_subsequence(trace[7], ('LOW', 'MID', 'HIGH', 'LOW')):
                break
            time.sleep(.08)
        else:
            stop('Did not observe full CH6/CH7 physical switch sequence within 70s')
        # Require stable final pilot-safe state, no arming.
        time.sleep(.5)
        if (latest_state is None or latest_state.armed or latest_state.mode != 'POSCTL'
                or latest_rc is None or classify(latest_rc.channels[5]) != 'HIGH'
                or classify(latest_rc.channels[6]) != 'LOW'):
            stop('RC switches/PX4 did not return to CH6 HIGH, CH7 LOW, DISARM/POSCTL')
        print('[PASS] Physical RC switch transitions detected, PX4 remained DISARMED')
        return {'ch6': trace[6], 'ch7': trace[7]}
    finally:
        sub1.unregister()
        sub2.unregister()


def authorize():
    verify_static()
    print('WARNING: Test only with PROPS REMOVED, aircraft secured, pilot present.')
    phrase = input('Type PROPS_REMOVED to begin read-only physical RC switch test: ').strip()
    if phrase != 'PROPS_REMOVED': stop('No-props affirmation missing')
    state, battery, rc, _ = live_state()
    observed = rc_switch_challenge()
    print('The RC switch test does NOT prove airborne POSCTL recovery.')
    print('Only the on-site flight team can attest PX4 failsafe, RC takeover, site clearance, and physical voltage.')
    pilot = input('Enter pilot initials (nonblank): ').strip()
    spotter = input('Enter safety observer initials (nonblank): ').strip()
    if not pilot or not spotter or pilot == spotter:
        stop('Two distinct on-site operator initials required')
    phrase = input('After genuine no-props PX4 takeover/failsafe verification, enter PILOT_SIGNOFF: ').strip()
    if phrase != 'PILOT_SIGNOFF': stop('No physical control takeover signoff')
    other = input('Confirm obstacle-free 4.5m route, props/airframe clearances, QGC vs meter voltage: type SITE_CLEAR: ').strip()
    if other != 'SITE_CLEAR': stop('No flight site clearance signoff')
    # After operator interaction, recheck current PX4/RC state.
    live_state()
    record = {
        'schema': 1, 'created_unix': time.time(), 'hostname': socket.gethostname(),
        'ros_master_uri': os.environ.get('ROS_MASTER_URI'),
        'r4_gate_sha256': sha(D / 'r4_flight_gate.py'),
        'candidate_manifest_sha256': sha(D / 'DEMO11_SOURCE_MANIFEST.json'),
        'pilot_initials': pilot, 'spotter_initials': spotter,
        'rc_switch_observed': observed, 'no_props_test': True,
        'takeover_claim': 'physical pilot signoff; NOT measured in software',
        'flight_intent': 'single UAV, obstacle free, F3/F4.061/F4.5, 0.8m, 0.20m/s',
    }
    tmp = STAMP.with_suffix('.pending')
    tmp.write_text(json.dumps(record, indent=2) + '\n')
    os.chmod(tmp, 0o600)
    os.replace(tmp, STAMP)
    print('[PASS] One-site approval created (45 min max). Flight NOT started.')


def verify_approval():
    if not STAMP.is_file() or STAMP.is_symlink():
        stop('R4 approval missing: run --prepare with props removed first')
    record = json.loads(STAMP.read_text())
    if record.get('schema') != 1 or not record.get('no_props_test'):
        stop('Malformed / invalid R4 approval')
    if not (0 <= time.time() - record['created_unix'] <= APPROVAL_MAX_AGE):
        stop('On-site approval expired; repeat no-props check')
    if record['hostname'] != socket.gethostname() or record['ros_master_uri'] != MASTER:
        stop('Site approval belongs to other machine or ROS Master')
    if record['r4_gate_sha256'] != sha(D / 'r4_flight_gate.py') or record['candidate_manifest_sha256'] != sha(D / 'DEMO11_SOURCE_MANIFEST.json'):
        stop('Gate code/source changed since approval')
    if not (record.get('pilot_initials') and record.get('spotter_initials')):
        stop('Pilot/observer signoff missing')
    if not has_subsequence(record['rc_switch_observed']['ch6'], ('HIGH', 'LOW', 'HIGH')) or not has_subsequence(record['rc_switch_observed']['ch7'], ('LOW', 'MID', 'HIGH', 'LOW')):
        stop('No valid RC switch takeover evidence')
    print('[PASS] Active site approval from %s / %s' % (record['pilot_initials'], record['spotter_initials']))


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--prepare', action='store_true')
    group.add_argument('--check', action='store_true')
    group.add_argument('--inspect', action='store_true')
    args = parser.parse_args()
    try:
        verify_static()
        if args.inspect:
            print('[PASS] Source integrity and four independent flight gates present; NO flight release')
            return 0
        if os.environ.get('ROS_MASTER_URI') != MASTER:
            stop('Unexpected ROS Master; use reviewed real hardware master')
        if args.prepare:
            authorize()
        if args.check:
            verify_approval()
            live_state()
            print('[PASS] R4 gated launcher preconditions, now awaiting pilot via existing Demo11 mission')
        return 0
    except (RuntimeError, ValueError, OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(str(exc), file=sys.stderr)
        return 97


if __name__ == '__main__':
    sys.exit(main())


def airborne_authorized(rospy):
    """Guard against bypassing the supervised multi-waypoint mission process.

    Called by the *real* waypoint module after the pilot-controlled takeoff.
    Only subscribes; does not issue MAVROS/flight commands.
    """
    from mavros_msgs.msg import State
    from std_msgs.msg import Bool, String
    verify_static()
    verify_approval()
    def get(topic, kind):
        try:
            return rospy.wait_for_message(topic, kind, timeout=3)
        except Exception as exc:
            stop('airborne authorization topic missing ' + topic + ': ' + str(exc))
    state = get('/contest1/mavros/state', State)
    if not state.connected or not state.armed or state.mode != 'OFFBOARD':
        stop('Mission process only permitted after original physical takeoff gate (ARMED/OFFBOARD)')
    for topic in ('/contest1/ego_uav_interface/flight_healthy',
                  '/contest1/ego_uav_interface/rc_mission_allowed',
                  '/contest1/ego_uav_interface/demo_frame_safe',
                  '/contest1/yaw_tangent_demo/session_ready'):
        if not get(topic, Bool).data:
            stop('Airborne health / frame / pilot authorization NOT ready: ' + topic)
    if get('/contest1/ego_uav_interface/r3_corridor_status', String).data != 'SESSION_READY':
        stop('Final-hop corridor did not bind fresh frame session')
    session = D / 'session_frame.yaml'
    if not session.is_file() or session.is_symlink():
        stop('New flight session frame missing')
    recorded = json.loads(STAMP.read_text())['created_unix']
    age = time.time() - session.stat().st_mtime
    if not (0 <= age < 900) or session.stat().st_mtime + 5 < recorded:
        stop('Flight frame is stale or predates R4 no-props site approval')
    print('[PASS] Real flight mission entered through pilot takeoff and FRESH R4 final guard')
