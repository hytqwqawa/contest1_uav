#!/usr/bin/env bash
# RViz view only. No flight launches, ROS services, or actuator commands.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest1_uav"
source /opt/ros/noetic/setup.bash
source "$ROOT/runtime/ego_uav_ws/devel/setup.bash" --extend
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
if [ ! -s "$D/session_frame.yaml" ]; then
  echo '[WAIT] A DISARMED Frame Guard session is needed; start the regular Demo11 mission wrapper first.'
  echo '[INFO] This viewer does NOT ARM or send setpoints.'
  exit 2
fi
python3 -B "$D/rviz_corridor_tubes.py" &
PID=$!
trap 'kill -INT "$PID" 2>/dev/null || true; wait "$PID" 2>/dev/null || true' EXIT INT TERM
rviz -d "$D/yaw_tangent_demo.rviz" /tf:=/contest1/tf /tf_static:=/contest1/tf_static
