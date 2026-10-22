# Demo11 R2 -- straight corridor / P2 FSM investigation (REVIEW ONLY)

**Do not fly R2.** This candidate refuses `run_demo11.sh`, `run_demo9.sh`,
`start_demo9.sh`, `start_demo5.sh`, and direct execution of
`uav2_demo5_waypoints.py`. It never ARM/launches a mission via install.

## Numeric corridor geometry

The user-supplied numeric values (despite swapped left/right naming):
F=0..3.0m, L=-0.5..+1.0m. L+ is Demo-frame left.
Turn region F>3m is NOT assigned any new walls, but the existing
Demo10 global fence stays in force everywhere.

For candidate geometric protection, assume **body radius 0.20m**,
**tracking-error allowance 0.15m**, and **additional reserve 0.10m**.
Then center L limits are -0.05..+0.55m near the straight walls.
For physical overlap near the ends, guard F=-0.20..+3.20m.
These are unvalidated safety assumptions and may be insufficient for your
particular propeller diameter, payload, wall thickness and localization drift.

## What R2 actually implements

- Pure Python `demo11_corridor.py` tests 301 sampled spline points and
  *conservative* convex hull of each active B-spline knot span. Known
  P1 bypass L=-0.878 / observed L=-0.904 will be rejected.
- R2 hooks are added in preview and in the waypoint task's replan monitor.
- Mission runner and all obvious entrypoints are flight-locked.
- ROS package, PX4 parameters, EGO obstacle inflation, Frame Guard,
  V4 MUX and underlying runtime are NOT changed by the installer.
- The geometry checker DOES NOT supply virtual walls to the planner. It
  does NOT manufacture a passable path; if EGO proposes outside, reject.
- A planning-time check can be TOO LATE for an already published mid-flight
  replan. Real-time validation before MUX/PX4 is still required for flight.

## P2 failure: NOT YET FIXED

From 2026-10-21 21:29-ish logs: EGO remained in EXEC_TRAJ while executing P1;
P2 showed `PREVIEW REQUEST` via the trajectory server but no new
`MANUAL goal accepted`, `GEN_NEW_TRAJ`, or B-spline. The exact underlying
FSM defect is **not proven**. The supplementary roslogs.tar.gz contains zero
files; the exported Demo10 ZIP did not include the planner FSM .cpp file.
Do not bypass the 8s B-spline timeout. Do not publish a fake trajectory.

## Offline usage

In the R2 new directory: `python3 -B demo11_verify.py` and
`python3 -B test_demo11_r2_offline.py` are ROS-free and do not fly.
The installer checks the current Demo11 V1 frozen 137-file hash list and
refuses installation if V1 was changed or R2 destination already exists.

Find planner FSM sources using the no-ROS script:
`python3 -B collect_p2_planner_sources.py`
Upload the resulting `~/Desktop/demo11_p2_fsm_sources_*.zip`.

Review next: 1) inspect the FSM source and determine the P2 reset bug;
2) add downstream/live EGO replan admission guard before PX4, with
continuous safe HOLD on rejection; 3) static tests + DISARMED no-prop ROS
simulation, then carefully staged tests only after site safety review.

## Rollback

R2 does not touch Demo10, Demo11 V1, or `runtime/`. Stop using the new
R2 directory to revert. Do not delete historical experimental data.
