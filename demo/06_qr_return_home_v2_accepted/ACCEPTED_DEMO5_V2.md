# Accepted Demo5 V2: Mixed POINT/EGO Waypoints

**Status:** user-reported successful real flight, corroborated by mission log and MAVROS flight bag.

- Flight date: 2026-10-09
- P1: F+1, L+1, POINT / heading hold / no EGO obstacle avoidance
- P2: F+3, L-1, EGO / tangent yaw / obstacle-aware planner
- P3: F+5, L0, EGO / final goal
- Final phase: original POINT landing; MAVROS flight bag confirms PX4 DISARM
- Takeoff target: 0.80m; commanded EGO/POINT maximum nominal speed: 0.20m/s
- First-gap and CH5/CH6/CH7/CH8 protection unchanged
- No software ARM; CH5 physical ARM/DISARM; CH6 manual POSCTL takeover

**Evidence:** `flight_evidence/demo5_v2_success_20261009_220734.log` plus MAVROS bag state-transition verification.
The 69MB flight rosbag remains under `data/bags/demo5_mixed_point_ego_v2/`.
If available, its path/checksum is captured in `flight_evidence/rosbag.sha256`.
Real obstacle-avoidance performance requires separate obstacle-specific evidence.

**Official entries, within this directory:**

Terminal 1: `./start_core.sh`
Terminal 2: `./start_backend.sh`
Terminal 3: `./check_demo5_static.sh && ./run_mission.sh`

This is a file-exact snapshot of the flown DEV implementation (except removed
transient outputs, added acceptance metadata and runtime provenance copies).
Existing `runtime/` code and ROS workspaces are NOT modified. Execution still
depends on live `~/contest1_uav/runtime/`, so verify runtime drift separately.
Do not edit accepted `.py`, `.sh`, `.launch`, or `.yaml` files.
New runs can generate temporary session-frame data and logs in this directory;
those transient outputs are not part of `ACCEPTED.sha256`.
