#!/usr/bin/env bash

set -u

ROOT="$HOME/contest1_uav"
DEMO="$ROOT/demo/02_waypoint_pole"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest1

BAGDIR="$ROOT/data/bags/waypoint_pole"
STAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BAGDIR"

BAG="$BAGDIR/waypoint_pole_demo_${STAMP}.bag"

echo
echo "========================================"
echo " UAV2 WAYPOINT-POLE FULL DEMO"
echo "========================================"
echo
echo " P0 -> P1 -> P2(obstacle) -> P3 -> LAND"
echo
echo "NO SOFTWARE ARM"
echo
echo "bag:"
echo "$BAG"
echo


rosbag record \
-O "$BAG" \
/contest1/t265/odom/filtered \
/contest1/mavros/local_position/odom \
/contest1/mavros/state \
/contest1/mavros/extended_state \
/contest1/mavros/distance_sensor/hrlv_ez4_pub \
/contest1/mavros/setpoint_raw/local \
/contest1/ego_uav_interface/point_setpoint \
/contest1/ego_uav_interface/ego_setpoint \
/contest1/ego_uav_interface/trajectory_mux_status \
/contest1/ego_uav_interface/rviz_goal_status \
/contest1/ego_uav_interface/demo_frame_safe \
/contest1/ego_uav_interface/yaw_mode \
/contest1/ego_uav_interface/yaw_target_deg \
/contest1/drone_0_planning/bspline \
/contest1/position_cmd \
/contest1/traj_start_trigger \
/rosout_agg \
>/tmp/waypoint_pole_bag.log 2>&1 &

BAG_PID=$!

cleanup()
{
    if kill -0 "$BAG_PID" 2>/dev/null; then
        kill -INT "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

sleep 1

echo "[ LIVE PATH ] resetting previous mission paths"

rostopic pub -1 /contest1/waypoint_pole/reset_live_paths std_msgs/Empty "{}" >/dev/null 2>&1 || true

sleep 0.2

PYTHONUNBUFFERED=1 \
python3 \
"$DEMO/uav2_waypoint_pole_mission.py"

RET=$?

cleanup
trap - EXIT INT TERM

echo
echo "========================================"
echo " TEST FINISHED"
echo " BAG:"
echo " $BAG"
echo "========================================"

exit "$RET"
