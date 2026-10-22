# Demo11 R3: final MAVROS-position-setpoint corridor gate (GROUND REVIEW)

**This is NOT a flight release.** Demo10, Demo11 V1, Demo11 R2, PX4 and runtime are untouched.
The R3 mission wrappers still exit with code 97 before any flight action.

## What changed

- `trajectory_interface_demo8.py`: at 50Hz, before emitting a trajectory
  `PositionTarget` to `/contest1/mavros/setpoint_raw/local`, verify its
  world position against the freshly bound Demo F/L frame and the agreed
  straight corridor (F -0.20..3.20 / center L -0.05..+0.55).
- Check aircraft measured XY as well as commanded XY, finite active XYZ/YAW,
  and consecutive active trajectory commands (maximum 0.25m / 12deg jumps).
- On rejection: latch abort, disable trajectory, capture current measured
  position and keep issuing the baseline HOLD at 50Hz while health/odom permit.
  NEVER clip the command, freeze a stale EGO command or auto-resume.
- Frame Guard `session_ready` must bind a fresh session file matching the
  active Demo11 YAML and basis; stale / missing / rotated incorrectly fails closed.
- The backend startup now resolves BOTH private EGO executables in the isolated
  build `~/contest1_uav/dev/projects/demo11_p2_guard_ws`, after sourcing that
  workspace, rather than silently running the old runtime Planner.
- Source and flight-entrypoint SHA verification remains in the R3 copy.

## Key limitation

This last-hop guard is an additional gate, not a guarantee of collision
avoidance. Its HOLD fallback needs healthy odometry and position control.
It is NOT an independent emergency system if the main computer, power, or
PX4 control link fails. An invalid trajectory may already be physically
unavoidable when detected; measured wall boundaries/clearance are unverified.
Do not assume an acceptable path exists around F1.5 L+0.3.

## Offline test only

Run `python3 -B ~/contest1_uav/demo/11_2p1_corridor_r3_ground_review/test_demo11_r3_runtime_gate.py`
(or `python3 -B .../demo11_verify.py`). No ROS nodes will launch.

Next required: independent localhost playback through the *actual modified*
TrajectoryInterface using `bash run_r3_final_loopback.sh` (isolated 127.0.0.1:11674,
no MAVROS or controller). Then a disarmed/no-prop complete-chain check
and inspection of observed final setpoints; obstacle map/clearance assessment.
Until then do not unlock `run_demo11.sh` or attempt a live flight.

Rollback: do not run this R3 version. Existing R2 / V1 / Demo10 unchanged.
