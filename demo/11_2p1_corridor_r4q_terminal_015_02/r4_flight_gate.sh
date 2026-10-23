#!/usr/bin/env bash
# Invoked by EVERY flight entrypoint, before any mission starts.
set -eo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest1_uav"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
unset ROS_HOSTNAME || true
# ROS Noetic setup scripts are not universally nounset-safe.
source "$ROOT/runtime/env.sh"
source "$ROOT/dev/projects/demo11_p2_guard_ws/devel/setup.bash"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
export ROS_PACKAGE_PATH="$D/ros_src:$D:${ROS_PACKAGE_PATH:-}"
python3 -B "$D/r4_flight_gate.py" --check || {
  echo '[FLIGHT LOCK] R4H live PX4/RC/trajectory safety gate not satisfied; no mission started.' >&2
  exit 97
}
