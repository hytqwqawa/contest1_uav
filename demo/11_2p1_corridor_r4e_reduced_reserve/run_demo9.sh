#!/usr/bin/env bash
# R2 deliberately stops before launching mission/PX4 mode changes.
R2_LOCK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$R2_LOCK_DIR/r4_flight_gate.sh" || exit 97
DEMO11_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$DEMO11_SCRIPT_DIR/demo11_verify.py" --quiet || {
  echo "[STOP] Demo11 source integrity check failed"
  exit 96
}
# Terminal 3: Demo9 integrated EGO yaw mission + advisory Demo7 QR observer. No software ARM.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest1_uav"
source "$ROOT/runtime/env.sh"
V4_MUX="$D/ros_src/ego_uav_interface/scripts/trajectory_mux_v4_only.py"
if ! pgrep -fa "$V4_MUX" >/dev/null 2>&1; then
  echo "[STOP] Running backend is not V4 MUX; mission NOT started"
  exit 96
fi
echo "[V4 VERIFIED] Live MUX process uses $V4_MUX"
python3 -B "$D/demo11_verify.py" --mux-only
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
export ROS_NAMESPACE=/contest1
if rosnode list 2>/dev/null | grep -Fxq /contest1/demo7_qr_monitor; then
    echo '[STOP] Existing Demo7 QR node detected; stop it first.'
    exit 1
fi
BAG_DIR="$ROOT/data/bags/demo11_2p1_corridor_r4e_reduced_reserve"
mkdir -p "$BAG_DIR"
BAG="$BAG_DIR/demo11_$(date +%Y%m%d_%H%M%S).bag"
echo "[DEMO11] Flight bag: $BAG"
rosbag record -O "$BAG" \
  /contest1/mavros/state \
  /contest1/mavros/extended_state \
  /contest1/mavros/local_position/odom \
  /contest1/mavros/distance_sensor/hrlv_ez4_pub \
  /contest1/mavros/setpoint_raw/local \
  /contest1/mavros/rc/in \
  /contest1/mavros/battery \
  /contest1/mavros/imu/data \
  /contest1/mavros/estimator_status \
  /contest1/t265/odom/filtered \
  /contest1/ego_uav_interface/point_setpoint \
  /contest1/ego_uav_interface/trajectory_setpoint \
  /contest1/demo9_integrated/mission_event \
  /contest1/ego_uav_interface/point_traj_status \
  /contest1/ego_uav_interface/trajectory_mux_status \
  /contest1/ego_uav_interface/flight_healthy \
  /contest1/ego_uav_interface/rc_mission_allowed \
  /contest1/ego_uav_interface/demo_frame_safe \
  /contest1/position_cmd \
  /contest1/demo7/qr/result \
  /contest1/demo7/qr/status \
  /contest1/ego_uav_interface/yaw_mode \
  /contest1/ego_uav_interface/yaw_target_deg \
  /contest1/ego_uav_interface/ego_bridge_status \
  /contest1/ego_uav_interface/trajectory_interface_status \
  /contest1/ego_uav_interface/ego_setpoint \
  /contest1/demo5/active_waypoint \
  >/tmp/demo11_arc_midpoint_rosbag.log 2>&1 &
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
    echo '[ABORT] Demo9 rosbag failed to start'
    tail -n 40 /tmp/demo11_arc_midpoint_rosbag.log
    exit 1
fi
"$D/start_demo9.sh"
