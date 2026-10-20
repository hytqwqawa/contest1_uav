#!/usr/bin/env bash
# Read-only incident diagnostics; no ROS/PX4 command or parameter changes.
set -eu
BOOT_ID="ecc265b01b6c4194b8d324535969e79e"
OUT="$HOME/Desktop/demo9_20261019_pc_fault.txt"
mkdir -p "$HOME/Desktop"
{
  echo "===== INCIDENT BOOT ====="
  journalctl -b "$BOOT_ID" -o short-iso --no-pager \
    --since '2026-10-19 21:07:30' --until '2026-10-19 21:10:30' 2>&1 || true
  echo "===== INCIDENT KERNEL ====="
  journalctl -b "$BOOT_ID" -k -o short-iso --no-pager \
    --since '2026-10-19 21:07:30' --until '2026-10-19 21:10:30' 2>&1 || true
  echo "===== REPORTED ROS NODELET CRASH ====="
  ls -lh /var/crash/_opt_ros_noetic_lib_nodelet_nodelet.1000.crash 2>&1 || true
  echo "===== REBOOT HISTORY ====="
  last -x reboot shutdown 2>&1 | head -n 20 || true
} > "$OUT"
echo "Read-only diagnostic report: $OUT"
