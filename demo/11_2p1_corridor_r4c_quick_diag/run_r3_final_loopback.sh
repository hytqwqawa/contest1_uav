#!/usr/bin/env bash
# Standalone localhost ROS loopback. NEVER talks to the real flight master.
set -Eeuo pipefail
D="$HOME/contest1_uav/demo/11_2p1_corridor_r3_ground_review"
WS="$HOME/contest1_uav/dev/projects/demo11_p2_guard_ws"
PORT=11674
OUT="$HOME/Desktop/demo11_r3_final_loopback_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT/ros_home" "$OUT/ros_log"
ROS_PID=''; IF_PID=''
cleanup(){
  rc=$?
  trap - EXIT INT TERM
  if [[ -n "$IF_PID" ]]; then kill -INT "$IF_PID" 2>/dev/null || true; wait "$IF_PID" 2>/dev/null || true; fi
  if [[ -n "$ROS_PID" ]]; then kill -INT "$ROS_PID" 2>/dev/null || true; wait "$ROS_PID" 2>/dev/null || true; fi
  echo "[INFO] Isolated output logs: $OUT"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

[[ -f "$D/r2_flight_lock.sh" && -f "$D/demo8_interface_overlay/scripts/demo11_runtime_corridor_gate.py" ]] || { echo '[STOP] R3 not installed';exit 4; }
[[ -f "$WS/devel/setup.bash" ]] || { echo '[STOP] private Planner workspace missing';exit 4; }

unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH ROS_NAMESPACE ROS_HOSTNAME
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export ROS_PACKAGE_PATH="$D:${ROS_PACKAGE_PATH:-}"
export ROS_MASTER_URI="http://127.0.0.1:$PORT"
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME="$OUT/ros_home"
export ROS_LOG_DIR="$OUT/ros_log"
rospack profile >/dev/null
if timeout 2 rostopic list >/dev/null 2>&1; then
  echo '[STOP] localhost:11674 ROS Master is already in use';exit 5;
fi
[[ "$(rospack find demo8_interface_overlay)" == "$D/demo8_interface_overlay" ]] || { echo '[STOP] wrong ROS overlay';exit 5; }
python3 -B "$D/demo11_verify.py" --quiet
python3 -B "$D/test_demo11_r3_interface_mock.py" >"$OUT/offline.txt" 2>&1 || { cat "$OUT/offline.txt";exit 5; }

[[ ! -e "$OUT/session_frame.yaml" ]] || { echo '[STOP] stale test session file';exit 5; }
echo '[INFO] Starting dedicated 127.0.0.1:11674 ROS Master (no MAVROS)'
roscore -p "$PORT" >"$OUT/roscore.log" 2>&1 &
ROS_PID=$!
for i in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then break; fi
  kill -0 "$ROS_PID" 2>/dev/null || { echo '[STOP] roscore exited'; exit 6; }
  sleep 0.2
done
rosnode list 2>/dev/null | grep -Fxq /rosout || { echo '[STOP] no isolated ROS Master';exit 6; }

echo '[INFO] Starting R3-only TrajectoryInterface (simulated ROS topics)'
python3 -u "$D/demo8_interface_overlay/scripts/trajectory_interface_demo8.py" \
  "_r3_corridor_yaml:=$D/demo11_corridor.yaml" \
  "_r3_mission_yaml:=$D/demo5_waypoints.yaml" \
  "_r3_session_yaml:=$OUT/session_frame.yaml" \
  >"$OUT/interface.log" 2>&1 &
IF_PID=$!
for i in $(seq 1 50); do
  if rosnode list 2>/dev/null | grep -Fxq /trajectory_interface; then break; fi
  kill -0 "$IF_PID" 2>/dev/null || { echo '[STOP] interface exited';tail -n 40 "$OUT/interface.log";exit 7; }
  sleep 0.2
done

set +e
timeout 28 python3 -u -B "$D/r3_final_loopback_replay.py" "$OUT/session_frame.yaml" \
  >"$OUT/results.txt" 2>&1
CODE=$?
set -e
cat "$OUT/results.txt"
if [[ $CODE -ne 0 ]]; then
  echo '[FAIL] R3 real ROS output-path test failed'
  tail -n 60 "$OUT/interface.log"
  exit "$CODE"
fi
echo '[PASS] R3 final-hop interface rejected invalid live ROS setpoint and continued HOLD'
echo '[NOTE] Loopback does not certify PX4, depth perception or physical clearances; R3 flight remains LOCKED.'
