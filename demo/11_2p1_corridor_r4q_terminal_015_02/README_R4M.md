# Demo11 R4M — H0.800 locked EGO setpoints, waypoint goals and safe retry

This self-contained demo directory installs from **existing R4K**; R4L DOES NOT
need to have been installed. All files stay in a NEW Demo11 demo directory.
R4K / R4I / R4J, Demo10, runtime, private EGO C++ and the safety corridor
are NOT modified. Uses the R4L height-reject retry improvement.

## User-requested fixed altitude: exact scope

Three airborne EGO waypoint targets: **relative H=0.800 m** to the **live
session origin**, i.e. absolute world Z=`session_frame.origin_world.z + 0.800`.
Unlike R4K, the planner goal is NOT set to current odometry Z.

Every accepted airborne EGO bridge **PositionTarget setpoint** has:
`position.z=Z_locked`, `velocity.z=0`, `acceleration_or_force.z=0`, including
FIXED terminal yaw holds. The live mission publishes the actual world-Z lock
at 10 Hz; stale/missing lock fails closed. The bridge ignores uncertified
`PositionCommand.trajectory_id`s. Mission accepts and certifies each plan
BEFORE releasing EGO. Actual measured aircraft Z is subject to PX4 tracking
error, and cannot be mathematically guaranteed to equal 0.800 at all times.
POINT takeoff and pilot POSCTL landing are not clamped (doing so would prevent
takeoff and landing).

### CRITICAL: no blind flattening of obstacle-avoidance replans

The private EGO Planner still **optimizes in 3D**, because its C++ internals
are unchanged. The original 3D B-spline is checked BEFORE certification.
All position control points must lie within **15mm** of locked world Z,
conservatively bounding all B-spline points. Any candidate that deviates
farther is REJECTED and triggers bounded safe POINT-HOLD replan. No large
vertical dip is forcibly projected to 0.8m, because that could cause a
collision with obstacles that required a 3D avoidance route.

The bridge independently checks raw PositionCommand Z within 20mm, then
locks EGO command z to H=0.8 and zeros vertical feedforward. The software
retains PX4 estimation/RC/frame guard, R2 corridor and R3 final output fences.
No software ARM or software LAND is introduced. The existing R4K 4 extra
geometry-replans (shared with altitude) and 2 gap retries stay bounded.

**R4K example** original B-spline min H=0.429 would still be rejected, not
silently flown at H0.800. Without modifying/rebuilding EGO to optimize
STRICTLY in 2D and doing collision validation at H0.800, one cannot honestly
promise that every raw candidate will be flat or accepted.

### Offline test & runtime cautions

Tests cover constant-z goals, bounded control-hull gate, bridge suppression
of uncertified/unsafe commands, setpoint z locking and R4K/R4L retry paths.
No rosbag-only or offline test proves the live PX4 timing under real replans.
Validate in SITL/HIL or propeller-free ROS/PX4 closed loop before real flight.
If R4M rejects a candidate, do not weaken the planar gate or obstacle safety.

## Installation (Ubuntu, aircraft DISARM, old processes stopped)

Unzip `demo11_r4m_fixed_height.zip` on Desktop and run:

```bash
cd ~/Desktop
unzip -n demo11_r4m_fixed_height.zip
python3 -B ~/Desktop/demo11_r4m_fixed_height_package/install_r4m.py
```

Installed directory:
`~/contest1_uav/demo/11_2p1_corridor_r4m_fixed_height`

Ground checks (NO flight):

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4m_fixed_height
python3 -B "$D/demo11_verify.py" --quiet
python3 -B "$D/test_r4m_height_lock.py"
```

Flight launch entry points remain `start_core.sh`, `start_backend.sh`, and
`run_r4_flight.sh` in the NEW directory, but **do not use for an actual flight
before live SITL/HIL verification**. Existing RViz straight/circular markers
are preserved. No edits are made to the old installed Demo11 versions.
