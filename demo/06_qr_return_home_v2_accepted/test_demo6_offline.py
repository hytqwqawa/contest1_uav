#!/usr/bin/env python3
"""ROS-free tests for new two-leg config and QR -> 5 seconds -> home policy."""
from pathlib import Path
import tempfile
import yaml
from demo6_config import load_demo6
from demo6_scan_policy import ScanPolicy


def check_config(root):
    cfg = load_demo6(root)
    assert cfg['point'] == {'forward': .5, 'left': -.5, 'mode': 'POINT'}
    assert cfg['home'] == {'forward': 0., 'left': 0., 'mode': 'POINT'}
    assert cfg['hold_time'] == 5.0
    with tempfile.TemporaryDirectory() as path:
        temp = Path(path)
        for fn in ('demo5_waypoints.yaml', 'demo6_qr_mission.yaml'):
            (temp / fn).write_bytes((root / fn).read_bytes())

        def reject(fn, alter):
            file = temp / fn
            original = file.read_text()
            data = yaml.safe_load(original)
            alter(data)
            file.write_text(yaml.safe_dump(data))
            try:
                load_demo6(temp)
            except (ValueError, KeyError):
                pass
            else:
                raise AssertionError('Unsafe YAML was accepted: %s' % fn)
            file.write_text(original)

        reject('demo5_waypoints.yaml', lambda d: d['waypoints'].pop())
        reject('demo5_waypoints.yaml', lambda d: d['waypoints'][0].update(mode='EGO'))
        reject('demo5_waypoints.yaml', lambda d: d['waypoints'][1].update(mode='EGO'))
        reject('demo5_waypoints.yaml', lambda d: d['waypoints'][1].update(forward=.3))
        reject('demo5_waypoints.yaml', lambda d: d['waypoints'][0].update(left=-1.))
        reject('demo6_qr_mission.yaml', lambda d: d['target'].update(forward=1.))
        reject('demo6_qr_mission.yaml', lambda d: d['mission'].update(hold_time=4.))
        reject('demo6_qr_mission.yaml', lambda d: d['qr'].update(stable_required=0))
        reject('demo6_qr_mission.yaml', lambda d: d['qr'].update(timeout=float('nan')))


def check_policy():
    # QR is from detector's already 3-frame-stable fresh result.
    p = ScanPolicy(start=0., timeout=30., hold_time=5.)
    assert p.state(0.) == 'SEARCH'
    assert p.state(1.) == 'SEARCH'
    p.push('HIT', 10.)
    assert p.state(10.) == 'HOLD'
    assert p.state(14.99) == 'HOLD'
    # Losing image does not erase an already confirmed QR.
    p.push('', 11.)
    assert p.state(15.) == 'READY'
    assert p.code == 'HIT'
    assert p.remaining(15.) == 0.0
    p2 = ScanPolicy(start=1., timeout=30., hold_time=5.)
    assert p2.state(31.) == 'TIMEOUT'
    p3 = ScanPolicy(start=0., timeout=30., hold_time=5.)
    p3.push('HIT', 4.)
    p3.push('WRONG', 4.5)
    assert p3.state(5.) == 'FAIL'


def check_implemented_hook(root):
    import ast
    tree = ast.parse((root / 'uav2_demo6_qr_waypoint.py').read_text())
    main = next(n for n in tree.body if isinstance(n, ast.ClassDef) and
                n.name == 'Demo6QrReturnHome')
    methods = {n.name: n for n in main.body if isinstance(n, ast.FunctionDef)}
    assert {'observe_qr', 'execute_point_waypoint', 'start_land', 'run'} <= set(methods)
    code = (root / 'uav2_demo6_qr_waypoint.py').read_text()
    assert 'super().execute_point_waypoint(index, total, waypoint)' in code
    assert 'super().start_land()' in code
    assert 'self.observe_qr()' in code
    assert 'self.require_point_hold(0.0, 0.0)' in code
    assert 'POINT RETURN HOME + LAND/DISARM' in code


if __name__ == '__main__':
    here = Path(__file__).resolve().parent
    check_config(here)
    check_policy()
    check_implemented_hook(here)
    print('[ OK ] Offline: scan P1, home P2, fixed yaw, QR hold 5s, timeout, unsafe configs')
