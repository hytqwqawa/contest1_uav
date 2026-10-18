#!/usr/bin/env bash
# Demo9 V3 flight candidate. Pilot physical arming and pilot landing only.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="$HOME/contest1_uav"
source "$ROOT/runtime/env.sh"
export ROS_NAMESPACE=/contest1

bash "$D/check_demo9_v3_static.sh"

nodes="$(timeout 5s rosnode list 2>/dev/null || true)"
for node in /contest1/demo9_v3_takeoff_hover_pilot_land \
  /contest1/demo9_takeoff_hover_land /contest1/demo04_5m_takeoff_gate \
  /contest1/flight_ops_test; do
  if grep -Fxq "$node" <<<"$nodes"; then
    echo "[ABORT] Duplicate flight mission: $node"; exit 2
  fi
done

# Require the known V3 standalone Core/backend topology before opening bag.
if ! pgrep -af "$D/demo9_v3_interface_overlay/scripts/trajectory_interface_demo9_v3.py" >/dev/null; then
    echo '[ABORT] V3 local trajectory interface not running'; exit 2
fi
for node in /contest1/mavros /contest1/trajectory_interface \
  /contest1/point_traj_node /contest1/trajectory_mux; do
  if ! grep -Fxq "$node" <<<"$nodes"; then
    echo "[ABORT] Missing V3 Core/Backend node: $node"; exit 2
  fi
done

BAG_DIR="$ROOT/data/bags/demo9_takeoff_hover_land_v3"
LOG_DIR="$ROOT/data/logs/demo9_takeoff_hover_land_v3"
mkdir -p "$BAG_DIR" "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$BAG_DIR/demo9_v3_$STAMP.bag"
BAG_LOG="$LOG_DIR/rosbag_$STAMP.log"
echo "[DEMO9 V3] Flight bag: $BAG"

rosbag record -O "$BAG" \
  /contest1/mavros/state \
  /contest1/mavros/extended_state \
  /contest1/mavros/local_position/odom \
  /contest1/mavros/distance_sensor/hrlv_ez4_pub \
  /contest1/mavros/imu/data \
  /contest1/mavros/estimator_status \
  /contest1/t265/odom/filtered \
  /contest1/ego_uav_interface/point_setpoint \
  /contest1/ego_uav_interface/trajectory_setpoint \
  /contest1/mavros/setpoint_raw/local \
  /contest1/ego_uav_interface/point_traj_status \
  /contest1/ego_uav_interface/trajectory_mux_status \
  /contest1/ego_uav_interface/trajectory_interface_status \
  /contest1/ego_uav_interface/flight_healthy \
  /contest1/ego_uav_interface/offboard_stream_ready \
  /contest1/ego_uav_interface/rc_mission_allowed \
  /contest1/ego_uav_interface/rc_supervisor_status \
  /rosout_agg >"$BAG_LOG" 2>&1 &
BAG_PID=$!
cleanup() {
  trap - EXIT INT TERM
  if kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep 1
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo '[ABORT] Bag process did not start'; tail -n 30 "$BAG_LOG"; exit 2
fi
python3 -B -u "$D/demo9_v3_mission.py"
