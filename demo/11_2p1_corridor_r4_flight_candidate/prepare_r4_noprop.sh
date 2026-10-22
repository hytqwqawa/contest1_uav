#!/usr/bin/env bash
# Run with actual R4 Core & Backend alive, WITHOUT propellers.
set -eo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest1_uav"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
unset ROS_HOSTNAME || true
source "$ROOT/runtime/env.sh"
source "$ROOT/dev/projects/demo11_p2_guard_ws/devel/setup.bash"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
export ROS_PACKAGE_PATH="$D/ros_src:$D:${ROS_PACKAGE_PATH:-}"
python3 -B "$D/r4_flight_gate.py" --prepare
