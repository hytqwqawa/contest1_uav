#!/usr/bin/env bash

set -u

ROOT="$HOME/contest1_uav"
DEMO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest1

export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101

BAG_DIR="$ROOT/data/bags/demo5_mixed_point_ego_v2"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="$BAG_DIR/demo5_waypoints_${STAMP}.bag"

mkdir -p "$BAG_DIR"

echo
echo "=========================================="
echo " CONTEST1 DEMO05 V2 MIXED POINT/EGO"
echo "=========================================="
echo " POINT takeoff"
echo " EGO avoidance / POINT no avoidance"
echo " EGO tangent yaw / POINT held yaw"
echo " config-driven ordered goals"
echo " automatic V4 landing"
echo
echo " NO SOFTWARE ARM"
echo
echo " bag:"
echo " $BAG"
echo "=========================================="
echo

rosbag record \
-O "$BAG" \
/contest1/t265/odom/sample \
/contest1/t265/odom/filtered \
/contest1/t265/odom/landing_safe_dev \
/contest1/mavros/local_position/odom \
/contest1/mavros/state \
/contest1/mavros/extended_state \
/contest1/mavros/distance_sensor/hrlv_ez4_pub \
/contest1/mavros/imu/data \
/contest1/mavros/setpoint_raw/local \
/contest1/ego_uav_interface/point_setpoint \
/contest1/ego_uav_interface/flight_healthy \
/contest1/position_cmd \
/contest1/traj_start_trigger \
/contest1/drone_0_planning/bspline \
/contest1/ego_uav_interface/trajectory_mux_status \
/contest1/ego_uav_interface/demo_frame_guard_status \
/contest1/demo5/active_waypoint \
/contest1/tf \
/contest1/tf_static \
/rosout_agg \
>/tmp/uav2_final_demo_rosbag.log 2>&1 &

BAG_PID=$!

cleanup()
{
    if kill -0 "$BAG_PID" 2>/dev/null; then
        kill -INT "$BAG_PID" 2>/dev/null
        wait "$BAG_PID" 2>/dev/null
    fi
}

trap cleanup INT TERM EXIT

sleep 1

echo "[ OK ] rosbag recording started"
echo

"$DEMO/start_demo5.sh"

RET=$?

cleanup
trap - INT TERM EXIT

echo
echo "=========================================="
echo " FINAL DEMO PROCESS ENDED"
echo " BAG: $BAG"
echo "=========================================="

exit "$RET"
