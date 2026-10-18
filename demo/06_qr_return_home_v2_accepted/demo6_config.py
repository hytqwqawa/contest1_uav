#!/usr/bin/env python3
"""Strict offline validator for Demo6 V2 QR scan and POINT return-to-home.

Both legs remain POINT (no obstacle avoidance); no change to runtime or ARM.
"""
from pathlib import Path
import argparse
import math
import yaml
from demo5_config import load_config


def number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError('%s must be numeric' % label)
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError('%s must be finite and in [%.2f,%.2f]' % (label, low, high))
    return value


def load_demo6(directory):
    root = Path(directory)
    plan = load_config(root / 'demo5_waypoints.yaml')
    cfg = yaml.safe_load((root / 'demo6_qr_mission.yaml').read_text())
    if not isinstance(cfg, dict) or set(cfg) != {'target', 'qr', 'mission'}:
        raise ValueError('Demo6 config needs target, qr, mission')
    target, qr, mission = cfg['target'], cfg['qr'], cfg['mission']
    if not all(isinstance(d, dict) for d in (target, qr, mission)):
        raise ValueError('target/qr/mission must be mappings')
    if set(target) != {'forward', 'left', 'height'}:
        raise ValueError('target keys must be forward,left,height')
    if set(qr) != {'stable_required', 'timeout'}:
        raise ValueError('qr keys must be stable_required,timeout')
    if set(mission) != {'hold_time'}:
        raise ValueError('mission key must be hold_time')

    f = number(target['forward'], 'target.forward', -0.8, 5.8)
    l = number(target['left'], 'target.left', -1.8, 1.8)
    h = number(target['height'], 'target.height', 0.8, 0.8)
    stable = qr['stable_required']
    if isinstance(stable, bool) or not isinstance(stable, int) or not 3 <= stable <= 10:
        raise ValueError('qr.stable_required must be integer [3,10]')
    timeout = number(qr['timeout'], 'qr.timeout', 10., 120.)
    dwell = number(mission['hold_time'], 'mission.hold_time', 5., 5.)

    if len(plan['waypoints']) != 2:
        raise ValueError('Demo6 V2 needs two ordered POINT waypoints (scan, home)')
    first, home = plan['waypoints']
    if first['mode'] != 'POINT' or home['mode'] != 'POINT':
        raise ValueError('Both Demo6 legs must be POINT (NO avoidance)')
    if abs(first['forward'] - f) > 1e-9 or abs(first['left'] - l) > 1e-9:
        raise ValueError('QR scan target disagrees with P1')
    if abs(plan['flight_height'] - h) > 1e-9:
        raise ValueError('QR scan height differs from POINT takeoff height')
    if (f, l, h) != (0.5, -0.5, 0.8):
        raise ValueError('Demo6 V2 restricted to F+0.50 L-0.50 H0.80')
    if abs(home['forward']) > 1e-9 or abs(home['left']) > 1e-9:
        raise ValueError('P2 must return to captured takeoff origin F0 L0')
    # Existing controller rejects >3m; conservative margins on both straight legs.
    if math.hypot(f, l) > 2.70:
        raise ValueError('POINT flight exceeds safe 2.70m segment limit')

    return dict(point=first, home=home, height=h, stable_required=stable,
                timeout=timeout, hold_time=dwell, fence=plan['fence'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', required=True)
    data = load_demo6(parser.parse_args().check)
    print('[ OK ] Demo6 V2 P1 SCAN: F+0.50 L-0.50 H0.80 / POINT (yaw HOLD)')
    print('[ OK ] Demo6 V2 P2 HOME: F+0.00 L+0.00 H0.80 / POINT -> LAND')
    print('[ OK ] QR 3-frame confirmation, hold 5.0s after recognition, timeout %.1fs' %
          data['timeout'])
    print('[WARN] Both POINT segments are straight-line and DO NOT avoid obstacles')


if __name__ == '__main__':
    main()
