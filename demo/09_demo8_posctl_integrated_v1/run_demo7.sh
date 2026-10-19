#!/usr/bin/env bash
# Terminal 3: Demo5 V2 mission + advisory Demo7 QR observer. No software ARM.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest1_uav"
source "$ROOT/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
export ROS_NAMESPACE=/contest1
if rosnode list 2>/dev/null | grep -Fxq /contest1/demo7_qr_monitor; then
    echo '[STOP] Existing Demo7 QR node detected; stop it first.'
    exit 1
fi
BAG_DIR="$ROOT/data/bags/demo7_qr_overlay_v1"
mkdir -p "$BAG_DIR"
BAG="$BAG_DIR/demo7_$(date +%Y%m%d_%H%M%S).bag"
echo "[DEMO7] Flight bag: $BAG"
rosbag record -O "$BAG" \
  /contest1/mavros/state \
  /contest1/mavros/extended_state \
  /contest1/mavros/local_position/odom \
  /contest1/mavros/distance_sensor/hrlv_ez4_pub \
  /contest1/mavros/setpoint_raw/local \
  /contest1/ego_uav_interface/point_traj_status \
  /contest1/ego_uav_interface/trajectory_mux_status \
  /contest1/ego_uav_interface/flight_healthy \
  /contest1/ego_uav_interface/rc_mission_allowed \
  /contest1/ego_uav_interface/demo_frame_safe \
  /contest1/position_cmd \
  /contest1/demo7/qr/result \
  /contest1/demo7/qr/status \
  >/tmp/demo7_qr_rosbag.log 2>&1 &
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
    echo '[ABORT] Demo7 rosbag failed to start'
    tail -n 40 /tmp/demo7_qr_rosbag.log
    exit 1
fi
"$D/start_demo7.sh"
