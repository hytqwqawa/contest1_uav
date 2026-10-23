# Demo11 R4N — Remove nominal altitude gates, retain 3D EGO obstacle avoidance

## Why R4M repeatedly rejected P1

R4M launched and settled at ~0.8 m, but EGO remained a 3D optimizer.
Its valid first B-spline had Demo H[0.788,0.841]m. R4M's new
"every B-spline control point within 0.015m of exact H0.800" vetoed it;
subsequent HOLD capture/replans descended and exhausted retries.

## R4N behavior

- Same **P1B (F3.500,L+0.250), P2 (F4.061,L-0.439), P3 (F4.500,L-1.500)**.
- All **waypoint TARGETS** remain Demo **H0.800m**, using fresh session
  origin Z + 0.800m rather than actual current Z.
- **No additional nominal-flight-height tolerance checks** on planned EGO
  control points, preview trajectory samples, live replans, or EGO bridge.
  Specifically removed R4M ±15mm, Bridge ±20mm, start-Z ±60mm,
  original mission H±0.25m nominal band. Measured heights are still logged.
- **Important deliberate difference from R4M:** executing EGO follows the
  original 3D B-spline P/V/A including Z; it is NOT forcibly flattened to a
  constant H0.800m plane. This preserves EGO's 3D obstacle-avoidance intent.
- The site-wide 3D physical **global fence**, R2 straight body corridor,
  R3 final MAVROS hard fencing, PX4/visual health gates, physical takeoff
  settlement, manual RC authority and land handoff are **unchanged**.
- All live EGO replans must pass global 3D fence and R2 corridor before
  their trajectory ID is permitted through the EGO bridge. A miss causes
  bounded safe HOLD/replanning, not silent acceptance.
- Runtime now uses R4N-exclusive lease+trajectory-ID topics. Old R4M
  processes must be stopped before starting R4N.

**This is not a zero-altitude-protection build.** Removing PX4/R3/global
altitude safety fencing was intentionally NOT done. Completely removing
height bounds while forcing every 3D obstacle-avoidance path to H0.800m
would risk collisions that cannot be ruled out by the user's previous flights.
The physics cannot guarantee actual aircraft altitude exactly 0.800m.

## Installation (DISARMED, old R4M ROS nodes stopped)

Place `demo11_r4n_ego3d_no_height_gate.zip` on Desktop:

```bash
cd ~/Desktop
unzip -n demo11_r4n_ego3d_no_height_gate.zip
python3 -B ~/Desktop/demo11_r4n_ego3d_no_height_gate_package/install_r4n.py
```

Destination:
`~/contest1_uav/demo/11_2p1_corridor_r4p_supervised_continuity`.
Installed R4M and all R4K/R4J/Demo10/runtime files are NOT changed.

## Ground-only checks

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4p_supervised_continuity
python3 -B "$D/demo11_verify.py" --quiet
python3 -B "$D/test_r4n_ego3d_policy.py"
```

Previously verified entry scripts remain in the independent folder:
`start_core.sh`, `start_backend.sh`, `run_r4_flight.sh`. The R4N
bridge/mission recovery sequence is **not yet flight tested**. Do SITL/HIL
and a non-flying closed-loop test, especially the transition from preview
POINT to approved EGO and dynamic replans, before a human-exposed flight.
