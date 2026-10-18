#!/usr/bin/env python3
"""V3 offline regression. No ROS dependency, never starts flight."""
import ast
import hashlib
import importlib.util
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


def manifest_check(root, manifest):
    for line in manifest.read_text().splitlines():
        checksum, relative = line.split('  ', 1)
        f = root / relative
        if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest() != checksum:
            raise AssertionError('Source changed since V3 installation: '+str(f))


def main(d):
    v1 = Path.home()/'contest1_uav/demo/09_takeoff_hover_land_v1'
    d8 = Path.home()/'contest1_uav/demo/08_ego_terminal_yaw_v1'
    manifest_check(v1, d/'v1_origin.sha256')
    manifest_check(d8, d/'demo8_origin.sha256')

    # Only V3 launch/mission/entrypoint is different from flight-successful V1.
    for name in ('uav2_5m_takeoff_gate.py', 't265_landing_sanitizer_dev.py',
                 'demo9_point_backend.launch'):
        assert (d/name).read_bytes() == (v1/name).read_bytes(), name
    assert ((d/'demo9_v3_interface_overlay/scripts/trajectory_interface_demo9_v3.py')
            .read_bytes() ==
            (v1/'demo9_interface_overlay/scripts/trajectory_interface_demo9.py').read_bytes()), 'Controller must remain byte-identical'

    # V3 copied flight-ops is baseline except for the exact entrypoint denial.
    baseline = (v1/'uav2_flight_ops_dev.py').read_text()
    original = '    if command == "land":\n        return node.land()'
    assert baseline.count(original) == 1, 'Unexpected V1 landing entrypoint'
    modified = baseline.replace(original,
        '    if command == "land":\n'
        '        print("[BLOCK] Demo9 V3: auto LAND disabled; pilot POSCTL landing only")\n'
        '        return 2', 1)
    assert (d/'uav2_flight_ops_dev.py').read_text() == modified, 'Unexpected change to proven takeoff ops'

    for rel in ('demo9_v3_mission.py', 'analyze_demo9_v3.py', 'demo9_v2_phase.py',
                'uav2_5m_takeoff_gate.py', 'uav2_flight_ops_dev.py',
                'demo9_v3_interface_overlay/scripts/trajectory_interface_demo9_v3.py'):
        ast.parse((d/rel).read_text(), filename=rel)
    for rel in ('uav2_core_bringup_dev.launch', 'demo9_point_backend.launch',
                'demo9_v3_interface_overlay/launch/trajectory_interface_demo9_v3.launch'):
        ET.parse(str(d/rel))

    m = (d/'demo9_v3_mission.py').read_text()
    assert 'rospy.Publisher(' not in m and 'rospy.ServiceProxy(' not in m
    assert '.land()' not in m and "'land'" not in m, 'No automatic landing in V3 task'
    assert "'uav2_5m_takeoff_gate.py'" in m, 'Proven pilot takeoff required'
    assert 'self.pilot_landing()' in m and "s.mode == 'POSCTL'" in m
    assert "state.armed or state.mode != 'POSCTL'" in m, 'Initial physical DISARM/POSCTL check required'

    core = (d/'start_core.sh').read_text()
    assert 'demo9_v3_interface_overlay' in core
    assert 'demo9_interface_overlay' not in core.replace('demo9_v3_interface_overlay','')
    assert 'core_09_takeoff_hover_land_v3' in core
    launch = (d/'uav2_core_bringup_dev.launch').read_text()
    assert 'demo9_v3_interface_overlay' in launch
    assert 'demo9_interface_overlay' not in launch.replace('demo9_v3_interface_overlay','')
    for key, value in (('takeoff_height','0.80'),('takeoff_speed','0.40'),
                       ('land_speed','0.30'),('land_terminal_speed','0.20'),
                       ('land_terminal_range','0.10')):
        import re
        assert re.search(r'name="%s"\s+value="%s"' % (key, value),
                         (d/'demo9_point_backend.launch').read_text()), key

    sys.path.insert(0, str(d))
    import demo9_v2_phase as phase
    p = phase.assess_range(.09, .4, 12.0, .02)
    assert p.state == 'BELOW_SPEC' and not p.can_authorize_touchdown
    p = phase.assess_range(.60, .4, 12.0, .02)
    assert p.state == 'IN_SPEC' and not p.can_authorize_touchdown
    print('[OK] Original V1 and Demo8 fingerprints intact; V3 Core/controller isolated.')
    print('[OK] Proven TAKEOFF code unchanged; AUTO LAND entrypoint blocked in V3.')
    print('[OK] Flight code AST, launch XML, output routing, sensor validity logic.')


if __name__ == '__main__':
    main(Path(sys.argv[1]).resolve())
