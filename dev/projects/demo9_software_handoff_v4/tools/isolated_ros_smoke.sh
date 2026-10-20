#!/usr/bin/env bash
# Software-only integration smoke with a LOOPBACK-ONLY NEW ROS master.
# It does not use a real PX4, MAVROS bridge or actual camera/localization.
set -euo pipefail

STAGE="${1:-}"
if [ -z "$STAGE" ]; then
  echo '[STOP] Usage: isolated_ros_smoke.sh <path-to-V4-DEV-stage>'
  exit 2
fi
STAGE="$(realpath "$STAGE")"
SRC="$(cd "$(dirname "$0")/.." && pwd)"
MUX="$STAGE/ros_src/ego_uav_interface/scripts/trajectory_mux.py"
IFACE="$STAGE/ros_src/ego_uav_interface/scripts/trajectory_interface.py"
PROBE="$SRC/tools/ros_smoke_synthetic.py"
[ -f "$MUX" ] && [ -f "$IFACE" ] && [ -f "$PROBE" ] || {
  echo '[STOP] Required staged scripts are missing'; exit 2;
}
[ -f "$STAGE/dev_manifest.json" ] || {
  echo '[STOP] Stage manifest missing'; exit 2;
}

# Avoid inheriting a remote ROS_MASTER_URI, ROS_IP or the active ROS log dir.
if ! command -v roscore >/dev/null 2>&1; then
  echo '[STOP] ROS Noetic is not sourced; source /opt/ros/noetic/setup.bash'; exit 2
fi
PORT=11457
python3 - "$PORT" <<'PY'
import socket,sys
s=socket.socket()
try:
    s.bind(('127.0.0.1',int(sys.argv[1])))
except OSError:
    raise SystemExit('[STOP] Isolated ROS port occupied; no processes started')
finally:
    s.close()
PY
export ROS_MASTER_URI="http://127.0.0.1:${PORT}"
unset ROS_IP
export ROS_HOSTNAME=127.0.0.1
LOGDIR="$STAGE/isolated_smoke_logs"
mkdir -p "$LOGDIR"
export ROS_HOME="$LOGDIR/ros_home"
mkdir -p "$ROS_HOME"
CORE_PID=''; IF_PID=''; MUX_PID=''
cleanup() {
  set +e
  for PID in "$MUX_PID" "$IF_PID" "$CORE_PID"; do
    if [ -n "$PID" ]; then kill "$PID" 2>/dev/null || true; fi
  done
  for PID in "$MUX_PID" "$IF_PID" "$CORE_PID"; do
    if [ -n "$PID" ]; then wait "$PID" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM

roscore -p "$PORT" >"$LOGDIR/roscore.log" 2>&1 & CORE_PID=$!
READY=0
for _ in $(seq 1 40); do
  if rosparam list >/dev/null 2>&1; then READY=1;break;fi
  sleep 0.10
done
[ "$READY" = 1 ] || { echo '[STOP] Isolated roscore did not start'; exit 2; }
python3 -B "$IFACE" >"$LOGDIR/trajectory_interface.log" 2>&1 & IF_PID=$!
python3 -B "$MUX" >"$LOGDIR/trajectory_mux.log" 2>&1 & MUX_PID=$!
sleep .5
kill -0 "$IF_PID" 2>/dev/null && kill -0 "$MUX_PID" 2>/dev/null || {
  echo '[STOP] A ROS node exited early; inspect staged isolated_smoke_logs'; exit 2;
}
# Only synthetic publisher / subscriber. No px4/arming or mode services.
python3 -B "$PROBE" | tee "$LOGDIR/result.txt"
echo '[OK] Isolated ROS integration smoke passed.'
echo "[LOGS] $LOGDIR"
