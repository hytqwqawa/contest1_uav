#!/usr/bin/env bash
# READ ONLY to flight code, ROS state, and parameters. File copies only.
set -euo pipefail
ROOT="$HOME/contest1_uav"
BASE="$ROOT/demo/09_demo8_posctl_integrated_v1"
source "$ROOT/runtime/env.sh"
PKG="$(rospack find ego_uav_interface)"
OUT="$HOME/Desktop/demo9_runtime_contract.tar.gz"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/demo9" "$TMP/ros_pkg_scripts"
{
    echo "ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
    echo "PKG=$PKG"
    echo "BASE=$BASE"
    echo "DATE=$(date -Is)"
} > "$TMP/resolution.txt"
for f in start_demo9.sh uav2_demo5_multi_waypoint_interface.launch; do
    if [ -f "$BASE/$f" ]; then
        cp -p "$BASE/$f" "$TMP/demo9/"
    else
        echo "MISSING: $BASE/$f" >> "$TMP/resolution.txt"
    fi
done
for f in trajectory_mux.py trajectory_interface.py point_traj_node.py ego_trajectory_bridge.py; do
    if [ -f "$PKG/scripts/$f" ]; then
        cp -p "$PKG/scripts/$f" "$TMP/ros_pkg_scripts/"
    else
        echo "MISSING: $PKG/scripts/$f" >> "$TMP/resolution.txt"
    fi
done
mkdir -p "$HOME/Desktop"
tar -czf "$OUT" -C "$TMP" .
echo "[OK] Read-only snapshot: $OUT"
cat "$TMP/resolution.txt"
ls -lh "$OUT"
