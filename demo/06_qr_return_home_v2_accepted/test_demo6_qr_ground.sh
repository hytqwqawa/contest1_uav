#!/usr/bin/env bash
# Ground-only QR test WITH Demo6 Terminal 2 camera (RGB + depth).
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HOME/contest1_uav/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
export ROS_NAMESPACE=/contest1
"$D/check_demo6_static.sh"
STATE="$(timeout -s KILL 5s rostopic echo -n 1 /contest1/mavros/state 2>/dev/null || true)"
if ! printf '%s\n' "$STATE" | grep -Fxq 'armed: False'; then
  echo '[STOP] Cannot confirm PX4 DISARMED; ground test denied'
  exit 1
fi
if ! printf '%s\n' "$STATE" | grep -Fxq 'mode: "POSCTL"'; then
  echo '[STOP] Cannot confirm POSCTL; ground test denied'
  exit 1
fi
if rosnode list 2>/dev/null | grep -Fxq /contest1/demo6_qr_flight_detector; then
  echo '[STOP] Another Demo6 QR node is running'; exit 1
fi
if ! timeout -s KILL 10s rostopic echo -n 1 \
  /contest1/camera/color/image_raw >/dev/null 2>&1; then
  echo '[FAIL] RGB color topic unavailable'; exit 1
fi
if ! timeout -s KILL 10s rostopic echo -n 1 \
  /contest1/camera/depth/image_rect_raw >/dev/null 2>&1; then
  echo '[FAIL] Depth topic unavailable'; exit 1
fi
echo '[ OK ] DISARM/POSCTL, RGB + Depth active'
echo '[QR] Hold a QR code in front of D435i. Output is 1Hz.'
echo '[QR] This test NEVER calls flight services or publishes POINT goals.'
exec python3 -u "$D/qr_flight_detector.py" _process_rate:=5.0 \
  _stable_count:=3
