#!/usr/bin/env bash
# DISARM ground-only simultaneous QR CPU + RGB/depth throughput diagnostic.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HOME/contest1_uav/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.5.20:11311
export ROS_IP=192.168.5.12
export ROS_NAMESPACE=/contest1

STATE="$(timeout -s KILL 5s rostopic echo -n 1 /contest1/mavros/state 2>/dev/null || true)"
if ! printf '%s\n' "$STATE" | grep -Fxq 'armed: False'; then
  echo '[STOP] Must be PX4 DISARMED'; exit 1
fi
if ! printf '%s\n' "$STATE" | grep -Fxq 'mode: "POSCTL"'; then
  echo '[STOP] Must be PX4 POSCTL'; exit 1
fi
if rosnode list 2>/dev/null | grep -Fxq /contest1/demo6_qr_flight_detector; then
  echo '[STOP] Stop the other QR test before measuring load'; exit 1
fi
LOG="/tmp/demo6_qr_camera_load_$(date +%Y%m%d_%H%M%S).log"
python3 -u "$D/qr_flight_detector.py" _process_rate:=5.0 _stable_count:=3 >"$LOG" 2>&1 &
PID=$!
cleanup() {
  trap - EXIT INT TERM
  kill -INT "$PID" 2>/dev/null || true
  wait "$PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep 2
if ! kill -0 "$PID" 2>/dev/null; then
  cat "$LOG"; echo '[FAIL] QR node exited early'; exit 1
fi
echo '[INFO] Keep QR visible during rate checks. Detector is running concurrently.'
for item in \
  '/contest1/camera/depth/image_rect_raw' \
  '/contest1/camera/color/image_raw' \
  '/contest1/mavros/local_position/odom'
do
  name="$(basename "$item")"
  F="/tmp/demo6_rate_${name}.log"
  echo "=== $item ==="
  timeout -s INT -k 3s 12s env PYTHONUNBUFFERED=1 rostopic hz "$item" >"$F" 2>&1 || true
  grep 'average rate:' "$F" | tail -n 1 || true
  rate="$(awk '/average rate:/{val=$3} END{print val+0}' "$F")"
  if ! awk -v h="$rate" 'BEGIN{exit !(h>=25.0)}'; then
    echo "[FAIL] $item average rate below 25Hz or unavailable"
    echo "QR log: $LOG"
    exit 1
  fi
done
echo
if grep -q '\[DEMO6 QR\] result=' "$LOG"; then
  echo '[ OK ] QR detector running concurrently (inspect decoded results below)'
else
  echo '[WARN] No QR 1Hz output; inspect detector log'
fi
grep '\[DEMO6 QR\] result=' "$LOG" | tail -n 6 || true
echo '[ OK ] Depth, RGB and odom >=25Hz during 5Hz QR decode'
echo "QR log: $LOG"
