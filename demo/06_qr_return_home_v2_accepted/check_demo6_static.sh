#!/usr/bin/env bash
# Offline only. Does not call ROS, ARM, send goals, or start flight nodes.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$D"
python3 -B demo5_config.py --check "$D/demo5_waypoints.yaml"
python3 -B demo6_config.py --check "$D"
python3 -B test_demo6_offline.py
for f in start_core.sh start_backend.sh start_demo6.sh run_demo6.sh; do
    bash -n "$f"
done
python3 -B - <<'PY'
import ast
from pathlib import Path
import xml.etree.ElementTree as ET
for name in ('demo5_config.py', 'demo6_config.py', 'demo6_scan_policy.py',
             'qr_stability.py', 'qr_flight_detector.py',
             'uav2_demo6_qr_waypoint.py', 'uav2_demo5_waypoints.py',
             'uav2_flight_ops_dev.py', 'uav2_5m_takeoff_gate.py'):
    ast.parse(Path(name).read_text(), filename=name)
for p in Path('.').glob('*.launch'):
    ET.parse(p)
backend = Path('start_backend.sh').read_text()
assert 'enable_color:=true' in backend and 'enable_depth:=true' in backend
start = Path('start_demo6.sh').read_text()
for key in ('uav2_demo6_qr_waypoint.py', 'demo6_config.py',
            'uav2_demo_frame_guard_v2.py', 'uav2_5m_takeoff_gate.py',
            'uav2_flight_ops_dev.py', 'DEMO6_QR_HASH'):
    assert key in start, key
assert 'start_demo6.sh' in Path('run_demo6.sh').read_text()
assert 'demo6_qr_return_home_v2' in Path('run_demo6.sh').read_text()
mission = Path('uav2_demo6_qr_waypoint.py').read_text()
assert 'super().execute_point_waypoint(index, total, waypoint)' in mission
assert 'self.require_point_hold(0.0, 0.0)' in mission
assert 'super().start_land()' in mission
assert 'QR HOLD 5s COMPLETE' in mission
print('[ OK ] V2 AST/XML, POINT return, QR scan, original pilot gate and landing checked')
PY
printf '\n[ OK ] Demo6 V2 static checks passed; NOT flight acceptance\n'
