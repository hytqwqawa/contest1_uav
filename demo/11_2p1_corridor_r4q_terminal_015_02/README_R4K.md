# Demo11 R4K — guarded in-air EGO corridor replanning

An isolated sibling under `~/contest1_uav/demo/`, cloned from the hash-verified
R4J. No changes to the installed R4J, Demo10, shared runtime or private Planner.

## What changes

This version targets the real failure in R4J:
`execute_direct() -> validated_ego_path() -> [R2 CORRIDOR REJECT]` when a
new **in-flight** EGO B-spline is outside the physical straight-center corridor.
The previous waypoint's `POINT verified through MAVROS output` was successful;
it does not certify the next replan.

- A genuine R2 B-spline corridor **planning rejection** is a recoverable
  exception type. The command is NEVER admitted or declared safe.
- If the rejection occurs **during EGO execution**, immediately begin the
  existing guarded `disable_trajectory -> abort_trajectory -> use_point ->
  enable_trajectory` chain and require three fresh, consistent physical
  MAVROS POINT output samples before allowing any new EGO goal.
- If it occurs in **pre-execution preview**, perform the same full POINT HOLD
  recapture and verification before re-planning.
- Wait until POINT HOLD is stationary for 0.80s, with fresh odometry/POINT
  samples, then require fresh PX4, RC, health, MUX, interface, and MAVROS
  output again. Retry the **same** physical waypoint (do not skip a leg).
- Require a distinct fresh trajectory ID and the pre-existing Traj Server ACK
  and entire B-spline/corridor validation on EVERY attempt.
- Up to **4 additional corridor planning retries** per waypoint (5 plans
  total); retains existing 2 retries for first-command gap. Attempt counters
  never reset until that waypoint genuinely completes. Backoff 0.35–1.2s.
- If the safe HOLD cannot be confirmed, health/RC/vision/R3 abort is bad, or
  no safe trajectory exists within the retry budget, DO NOT proceed. Hold
  with the existing controller / require pilot takeover, never skip waypoints,
  never re-arm or force a trajectory.

## What does not change

Exact 3 waypoint positions P1=F3.500 L+0.250, P2=F4.061 L-0.439,
P3=F4.500 L-1.500; 0.8m target; .20m/s cap, R4H .35rad/s yaw limit,
R4F conservative B-spline proof, R3 real-time corridor guard,
original physical walls L[-.50,+1.00] and center L[-.15,+.65], RViz geometry,
CH5 manual ARM/CH6 POSCTL/CH7 pilot permission, R4J 1s XY-only final
handoff and human POSCTL landing. The software does not force ARM or auto LAND.

## Limits / real-world safety

**NOT validated on a real aircraft or ROS/PX4 HIL.** Static and mocked offline
unit tests cannot certify collision safety or the actual ordering of EGO
planner, trajectory server, MUX and PX4 callbacks. In-flight EGO can already
publish a rejected replanned trajectory before the Python monitor observes
it; the pre-existing R3 output guard is the final safety barrier. Do not treat
this package as authorized for competition flight until a representative
SITL / hardware-in-loop recovery test and physical clearance check succeed.

Repeatedly replanning exactly the same goal from the same stationary state may
produce the same invalid path. Automatic retry must be bounded and will NOT
guarantee that a flight can complete after arbitrary hardware/geometry errors.

## Installation (DISARM; stop old R4J processes)

Copy ZIP to Ubuntu Desktop and use:

```bash
cd ~/Desktop
unzip -n demo11_r4k_safe_replan.zip
python3 -B ~/Desktop/demo11_r4k_safe_replan_package/install_r4k.py
```

Installs a new `~/contest1_uav/demo/11_2p1_corridor_r4k_safe_replan` only
when **all 170 R4J tracked files** match their original SHA256. No running
nodes, system settings or runtime code are changed by the installer.

Offline checks after install:

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4k_safe_replan
python3 -B "$D/demo11_verify.py" --quiet
python3 -B "$D/test_r4k_safe_replan.py"
```

Use the existing physical-pilot controlled mission entrypoints only after a
controlled in-air recovery validation:

```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4k_safe_replan/start_core.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4k_safe_replan/start_backend.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4k_safe_replan/run_r4_flight.sh
```
