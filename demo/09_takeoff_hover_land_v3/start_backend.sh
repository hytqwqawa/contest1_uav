#!/usr/bin/env bash
# Demo9 V3: minimal POINT-only backend. Never ARM/DISARM and never request OFFBOARD.
set -euo pipefail
ROOT="$HOME/contest1_uav"
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/runtime/env.sh"
export ROS_NAMESPACE=/contest1
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101

STATE="$(timeout 5s rostopic echo -n 1 /contest1/mavros/state 2>/dev/null || true)"
if ! grep -Fq 'connected: True' <<<"$STATE" || \
   ! grep -Fq 'armed: False' <<<"$STATE" || \
   ! grep -Fq 'mode: "POSCTL"' <<<"$STATE"; then
    echo '[ABORT] Core must be connected, DISARM, POSCTL before starting Demo9 V3 Backend.'
    printf '%s\n' "$STATE"
    exit 1
fi
nodes="$(timeout 5s rosnode list 2>/dev/null || true)"
for node in /contest1/point_traj_node /contest1/trajectory_mux \
    /contest1/ego_trajectory_bridge /contest1/drone_0_ego_planner_node \
    /contest1/demo9_takeoff_hover_land /contest1/demo04_5m_takeoff_gate; do
    if grep -Fxq "$node" <<<"$nodes"; then
        echo "[ABORT] Existing flight/backend node: $node"
        exit 2
    fi
done

echo '[DEMO9] POINT-only backend; no D435i/EGO/QR.'
echo '[DEMO9] No ARM, OFFBOARD, or flight task is issued here.'
exec roslaunch "$D/demo9_point_backend.launch"
