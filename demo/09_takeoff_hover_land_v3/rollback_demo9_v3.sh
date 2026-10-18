#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
[ "$D" = "$HOME/contest1_uav/demo/09_takeoff_hover_land_v3" ] || {
  echo '[ABORT] Unexpected folder path'; exit 2;
}
nodes="$(timeout 5s rosnode list 2>/dev/null || true)"
for node in /contest1/mavros /contest1/trajectory_interface /contest1/point_traj_node \
 /contest1/trajectory_mux /contest1/demo9_v3_takeoff_hover_pilot_land; do
  if grep -Fxq "$node" <<<"$nodes"; then
    echo "[ABORT] Flight/control stack still running: $node"; exit 2
  fi
done
# Match Python process comm+script path, not a parent shell command containing
# the script names as literal diagnostic text.
if ps -eo comm=,args= | awk '
  $1 ~ /^python(3(\.[0-9]+)?)?$/ && $0 ~ /\/(trajectory_interface_demo9_v3|demo9_v3_mission|uav2_5m_takeoff_gate)\.py([[:space:]]|$)/ { found=1; exit }
  END { exit(found ? 0 : 1) }
'; then
    echo '[ABORT] V3 control/mission process remains'; exit 2
fi
OUT="$HOME/uav_handoff_exports"
mkdir -p "$OUT"
DEST="$OUT/demo9_v3_removed_$(date +%Y%m%d_%H%M%S)_$$"
[ ! -e "$DEST" ] || { echo '[ABORT] Backup path collision'; exit 2; }
mv "$D" "$DEST"
echo "[OK] V3 archived: $DEST"
echo '[OK] V1, frozen Demo8 and runtime unchanged.'
