#!/usr/bin/env bash
DEMO11_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$DEMO11_SCRIPT_DIR/demo11_verify.py" --quiet || {
  echo "[STOP] Demo11 source integrity check failed"
  exit 96
}
# Thin QR observer wrapper; calls UNMODIFIED successful Demo5 V2 pilot+flight mission.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$HOME/contest1_uav"
source "$ROOT/runtime/env.sh"
export ROS_MASTER_URI=http://192.168.8.101:11311
export ROS_IP=192.168.8.101
export ROS_NAMESPACE=/contest1
python3 -B "$D/demo11_verify.py" --mux-only || exit 96

if rosnode list 2>/dev/null | grep -Fxq /contest1/demo7_qr_monitor; then
    echo '[ABORT] Demo9 QR monitor already running; stop duplicate.'
    exit 1
fi
# Fail BEFORE pilot gate if the RGB pipeline is not ready.
if ! timeout -s KILL 10s rostopic echo -n 1 \
    /contest1/camera/color/image_raw >/dev/null 2>&1; then
    echo '[ABORT] D435i RGB topic has no frames.'
    exit 1
fi

echo '[DEMO11] Starting non-controlling QR observer (yellow 1Hz during ARMED/OFFBOARD).'
PYTHONUNBUFFERED=1 python3 -u "$D/demo7_qr_monitor.py" \
    _decode_hz:=5.0 _stable_frames:=3 _ground_test:=false &
QR_PID=$!
cleanup() {
    trap - EXIT INT TERM
    if kill -0 "$QR_PID" 2>/dev/null; then
        kill -INT "$QR_PID" 2>/dev/null || true
        wait "$QR_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

sleep 2
if ! kill -0 "$QR_PID" 2>/dev/null; then
    echo '[ABORT] Demo9 QR monitor failed before pilot gate.'
    exit 1
fi
if ! timeout -s KILL 6s rostopic echo -n 1 \
    /contest1/demo7/qr/status >/dev/null 2>&1; then
    echo '[ABORT] Demo11 QR node not publishing status.'
    exit 1
fi

echo '[ OK ] QR monitor online; Demo11 route control remains independent.'
echo '[DEMO11] QR observations DO NOT alter POINT/EGO/landing or pilot permissions.'
"$D/start_demo5.sh"
