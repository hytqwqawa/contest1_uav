# Demo6 — Development candidate (NOT ACCEPTED)

Goal: operator physically ARM + authorized POINT takeoff to 0.8m; single fixed-yaw
POINT goal F+1 L-1 in DISARM-captured Demo frame; stay in POINT AIRBORNE_HOLD;
start RGB QR decoder *only after confirmed arrival*; print fresh QR result
(or NOT_FOUND) every second; land using unchanged proven Demo5 landing chain.

The flight algorithms in `uav2_demo5_waypoints.py`, `uav2_flight_ops_dev.py`,
`uav2_5m_takeoff_gate.py` are **untouched**. Demo6 subclasses the mission's
`start_land()` hook so that scan occurs only after the POINT arrival gate.

* `demo5_waypoints.yaml`: one POINT F1 L-1, unchanged safety fence.
* `demo6_qr_mission.yaml`: scan configuration.
* `qr_flight_detector.py`: 5Hz color decoding; 1Hz unlatched live result.
* `uav2_demo6_qr_waypoint.py`: QR scan + hover safety + proven landing.
* `start_backend.sh`: enable D435i RGB **and keep depth enabled**.
* `start_demo6.sh` / `run_demo6.sh`: dev Terminal 3; original Demo5 files remain.

On QR timeout/detector crash, the mission attempts the proven landing **only
if POINT, RC permission and flight health remain valid**. It subsequently
returns an error instead of calling the QR task successful. If flight-health,
RC permission or POSITION HOLD are lost, **pilot must take over** (CH7 LOW,
CH6 POSCTL); no software ARM/DISARM or unsafe forced landing is attempted.

POINT has NO obstacle avoidance. This initial development mission is for a
clear, controlled path only. RGB and depth share camera/USB/CPU resources;
verify depth FPS with both streams and QR active before any airborne test.

## Stage A: offline

```bash
cd ~/contest1_uav/dev/projects/06_qr_flight_v1
./check_demo6_static.sh
```

## Stage B: DISARM ground RGB + depth check

With NO other RealSense/ROS nodes running, CH5 DISARM, CH6 POSCTL, CH7 LOW,
CH8 SAFE:

Terminal 1: `./start_core.sh`

Terminal 2: `./start_backend.sh`

Terminal 3: `./test_demo6_qr_ground.sh`

In terminal 3 observe `[DEMO6 QR] result='...'` every second, or NOT_FOUND.
Stop ground QR with Ctrl+C while still DISARMED. Then run
`./test_demo6_camera_load.sh` in Terminal 3 to check RGB, depth and odom
rates **while** QR decoding is running. Require each average >=25Hz before any
flight planning. A D435i RGB+Depth combination can compete for USB/CPU bandwidth.

## Flight

**Not yet ground-integration validated or approved for flight.** After ground
validation and safety review, Terminal 3 would use `./run_demo6.sh`; requires
pilot-operated CH5 ARM, CH7 authorization and CH6 takeover capability. Do not
use `qr_mission_incomplete.py.disabled` as a flight program.
