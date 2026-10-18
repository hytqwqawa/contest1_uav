#!/usr/bin/env bash

set -u

ROOT="$HOME/contest1_uav"
DBG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ROOT/runtime/env.sh"

export ROS_NAMESPACE=/contest1

# Pure Python config validation; never ARM or launch ROS from here.
if ! python3 "$DBG/demo5_config.py" --check "$DBG/demo5_waypoints.yaml"; then
    echo "[ABORT] Invalid Demo6 QR waypoints/fence YAML."
    exit 1
fi

if ! python3 -B "$DBG/demo6_config.py" --check "$DBG"; then
    echo "[ABORT] Invalid Demo6 QR mission configuration."
    exit 1
fi
DEMO6_QR_HASH="$(sha256sum "$DBG/demo6_qr_mission.yaml" | awk '{print $1}')"
export DEMO6_QR_HASH


echo "[CHECK] DISARMED aircraft nose must face NEW forward (old LEFT)."
if rosnode list 2>/dev/null | grep -Eq '/(uav2_demo_frame_guard_v2|uav2_demo5_waypoints|uav2_demo6_qr_waypoint|uav2_tangent_preview)$'; then
    echo "[ABORT] Existing Demo-frame or waypoint mission node detected."
    exit 1
fi
rm -f "$DBG/alignment_target.yaml"

GUARD_PID=""
PREVIEW_PID=""
RVIZ_PID=""

cleanup()
{
    trap - EXIT INT TERM

    echo
    echo "[ CLEANUP ] stopping Demo06 auxiliary nodes..."

    [ -n "${PREVIEW_PID:-}" ] && \
        kill -INT "$PREVIEW_PID" 2>/dev/null || true

    [ -n "${RVIZ_PID:-}" ] && \
        kill -INT "$RVIZ_PID" 2>/dev/null || true

    [ -n "${GUARD_PID:-}" ] && \
        kill -INT "$GUARD_PID" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo
echo "========================================"
echo " CONTEST1 DEMO06 - POINT QR + RETURN HOME"
echo " YAML POINT/EGO legs + Demo3 POINT takeoff/LAND"
echo "========================================"
echo
echo "Required at startup:"
echo "  aircraft at fixed takeoff point"
echo "  CH5 DISARM"
echo "  CH6 POSCTL"
echo "  CH7 LOW"
echo "  CH8 SAFE"
echo

for F in \
"$DBG/uav2_demo_frame_guard_v2.py" \
"$DBG/demo5_config.py" \
"$DBG/demo5_waypoints.yaml" \
"$DBG/uav2_demo6_qr_waypoint.py" \
"$DBG/qr_flight_detector.py" \
"$DBG/demo6_qr_mission.yaml" \
"$DBG/uav2_5m_takeoff_gate.py" \
"$DBG/uav2_flight_ops_dev.py" \
"$DBG/demo_frame.yaml" \
"$DBG/yaw_tangent_demo.rviz"
do
    if [ ! -f "$F" ]; then
        echo "[ABORT] missing:"
        echo "        $F"
        exit 1
    fi
done

chmod +x \
"$DBG/uav2_demo_frame_guard_v2.py" \
"$DBG/uav2_demo6_qr_waypoint.py" \
"$DBG/uav2_5m_takeoff_gate.py" \
"$DBG/uav2_flight_ops_dev.py"

rm -f \
"$DBG/demo_frame_guard_v2.log" \
"$DBG/demo6_qr_flight.log" \
"$DBG/preview_summary.txt" \
"$DBG/session_frame.yaml"

echo "[1/4] Capturing new-nose Demo frame..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_demo_frame_guard_v2.py" \
>"$DBG/demo_frame_guard_v2.log" \
2>&1 &

GUARD_PID=$!

READY=0

for i in $(seq 1 50); do

    if ! kill -0 "$GUARD_PID" 2>/dev/null; then
        echo
        echo "[ABORT] Demo-frame guard exited:"
        cat "$DBG/demo_frame_guard_v2.log"
        exit 1
    fi

    V="$(
        timeout 1 \
        rostopic echo -n 1 \
        /contest1/yaw_tangent_demo/session_ready \
        2>/dev/null |
        awk '/data:/ {print $2; exit}'
    )"

    if [ "$V" = "True" ]; then
        READY=1
        break
    fi

    sleep 0.2
done

if [ "$READY" -ne 1 ]; then
    echo
    echo "[ABORT] Demo-frame session not ready."
    cat "$DBG/demo_frame_guard_v2.log"
    exit 1
fi

echo "[ OK ] Demo-frame session ready"
echo
cat "$DBG/session_frame.yaml"
echo

echo "[2/4] Starting preset RViz..."

rviz \
-d "$DBG/yaw_tangent_demo.rviz" \
/tf:=/contest1/tf \
/tf_static:=/contest1/tf_static &

RVIZ_PID=$!

sleep 1

if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[ABORT] RViz exited immediately."
    exit 1
fi

echo
echo "========================================"
echo " CHECK ALL WAYPOINTS IN RVIZ BEFORE ARM"
echo "========================================"
echo "Demo5: NOSE MUST POINT NEW FORWARD (= Demo4 old LEFT)."
echo "Fence: F [-1.00,+6.00] L [-2.00,+2.00]"
echo "Check EVERY numbered waypoint marker against field and obstacles."
echo "The final numbered point is the landing destination."
echo
echo "[INFO] Visually verify RViz before CH7 permission / CH5 ARM."
echo "[INFO] Pilot gate still requires physical authorization; no software ARM."

# Refuse a last-minute waypoint edit after Guard captured the frame.
if ! python3 -B - "$DBG" <<'PYHASH'
import hashlib, sys, yaml
from pathlib import Path
root = Path(sys.argv[1])
session = yaml.safe_load((root / 'session_frame.yaml').read_text())
current = hashlib.sha256((root / 'demo5_waypoints.yaml').read_bytes()).hexdigest()
if session.get('demo5_plan_sha256') != current:
    raise SystemExit('[ABORT] Demo5 YAML changed after RViz capture')
print('[ OK ] RViz/Guard waypoint configuration hash matches')
PYHASH
then
    exit 1
fi

# No takeoff until the RGB stream and scan configuration are confirmed.
if [ "$DEMO6_QR_HASH" != "$(sha256sum "$DBG/demo6_qr_mission.yaml" | awk '{print $1}')" ]; then
    echo "[ABORT] Demo6 QR YAML changed after initial review."
    exit 1
fi
if ! timeout -s KILL 10s rostopic echo -n 1 \
  /contest1/camera/color/image_raw >/dev/null 2>&1; then
    echo "[ABORT] D435 RGB not delivering frames; do not ARM."
    exit 1
fi

echo "[3/4] Waiting for pilot + Demo3 takeoff..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_5m_takeoff_gate.py"

TAKEOFF_RET=$?

# Keep Guard + RViz alive on abnormal airborne abort until pilot takeover.
wait_safe_pilot_takeover()
{
    echo
    echo "[PILOT] If airborne: CH7 LOW, CH6 POSCTL; do not software DISARM."
    echo "[PILOT] Leave this terminal running until aircraft is safely handled."
    while true; do
        RAW="$(timeout 3 rostopic echo -n 1 /contest1/mavros/state 2>/dev/null || true)"
        ARMED="$(printf '%s\n' "$RAW" | awk '/^armed:/ {print $2; exit}')"
        MODE="$(printf '%s\n' "$RAW" | awk '/^mode:/ {print $2; exit}')"
        if [ "$ARMED" = "False" ]; then
            echo "[ OK ] PX4 DISARMED; auxiliary nodes may stop."
            return 0
        fi
        if [ "$ARMED" = "True" ] && [ "$MODE" = "POSCTL" ]; then
            echo "[ OK ] CH6/POSCTL pilot takeover active; ending mission stack."
            return 0
        fi
        echo "[WAIT] armed=${ARMED:-UNKNOWN} mode=${MODE:-UNKNOWN} | CH7 LOW + CH6 POSCTL"
        sleep 1
    done
}

if [ "$TAKEOFF_RET" -ne 0 ]; then
    echo
    echo "[ABORT] Demo06 takeoff gate failed."
    wait_safe_pilot_takeover
    exit 1
fi

echo
echo "[ OK ] Demo3 takeoff physically settled"
echo

echo "[4/4] Starting POINT F+0.50 L-0.50 -> QR hold 5s -> HOME -> LAND..."

PYTHONUNBUFFERED=1 \
python3 \
"$DBG/uav2_demo6_qr_waypoint.py" \
> >(tee "$DBG/demo6_qr_flight.log") \
2>&1 &

PREVIEW_PID=$!

sleep 1

if ! kill -0 "$PREVIEW_PID" 2>/dev/null; then
    echo
    echo "[ABORT] Demo6 QR mission exited immediately:"
    cat "$DBG/demo6_qr_flight.log"
    exit 1
fi

MISSION_RET=0

wait "$PREVIEW_PID" || MISSION_RET=$?

if [ "$MISSION_RET" -ne 0 ]; then
    echo
    echo "[FAIL] Demo06 waypoint mission returned $MISSION_RET"
    tail -n 100 "$DBG/demo6_qr_flight.log"
    wait_safe_pilot_takeover
    exit 1
fi

echo
echo "========================================"
echo " DEMO05 MULTI-WAYPOINT MISSION COMPLETE"
echo "========================================"
echo
