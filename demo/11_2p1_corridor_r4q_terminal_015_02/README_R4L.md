# Demo11 R4L — bounded altitude-preview replan + raw Bspline logging

A separate, independently verified demo folder cloned from R4K. No edits to
R4K, R4I/R4J, Demo10, runtime, or the private compiled Planner.

## Confirmed R4K incident

The first waypoint was F3.500 L+0.250 H0.8. PX4 odometry and published
goal both had world z=0.777 m (Demo 0.802 m). But EGO's **planned**
B-spline sampled Demo altitude [0.429, 0.803] m, below the required
[0.550, 1.050] m. Rejection was mandatory and correct. `traj_id=2`
ACK just meant trajectory-server accepted/froze its PREVIEW, not that
its 3-D geometry passed. R4K retried only corridor/gap faults;
`RuntimeError('B-spline altitude invalid')` therefore stopped the mission.

**Unknown from supplied logs:** why private EGO dipped ~37cm despite
same-height endpoints. Possible avoidance/optimization or initial-state
mismatch; do not assume one until raw spline and Planner logs are inspected.
The R4K recorded bag did not include the /planning/bspline topic.

## R4L behavior

- A planned path outside 0.8m +/-0.25m altitude envelope becomes a
  **recoverable, but NEVER executable**, altitude-reject exception.
- If it happens before release: verify EGO->POINT HOLD output, fresh RC,
  PX4, odometry, health and stationary POINT, then plan the SAME waypoint
  from the current actual odometry. At most 4 EXTRA geometry retries.
- Altitude and corridor retries SHARE the same budget. Start-gap
  retains its separate 2-retry budget; maximum total 6 extras.
- For in-flight replans, rejoin POINT and verify physical MAVROS output
  BEFORE the next planning attempt, as in R4K.
- If global fence/flight-health/CH7 permission/RC/HOLD/R3 corridor guard
  fails, don't retry. After budget exhaustion, keep verified HOLD and
  require physical pilot takeover. No software ARM or autonomous LAND.
- Print the height-outlier sample index, its F/L/H, and bounds.
- Record `/contest1/drone_0_planning/bspline` in future flight bags so
  the actual spline can be inspected instead of inferring from extrema.

## Unchanged protections

The allowed altitude range, original Demo5 global fence, straight-tube
physical and swept-body geofences, R3 runtime output guard, EGO inflation,
T265/PX4 health rules, CH5 physical ARM, CH6 POSCTL and CH7 mission permission,
P1B/P2/P3 route, yaw settings, and R4J 1s final XY-only handoff stay exactly
as in R4K. No forced overflight of an unsafe spline.

## Install

Copy ZIP to Ubuntu Desktop, DISARM, stop old mission processes, then:

```bash
cd ~/Desktop
unzip -n demo11_r4l_altitude_replan.zip
python3 -B ~/Desktop/demo11_r4l_altitude_replan_package/install_r4l.py
```

Files install into:
`~/contest1_uav/demo/11_2p1_corridor_r4l_altitude_replan/`

Offline checks:

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4l_altitude_replan
python3 -B "$D/demo11_verify.py" --quiet
python3 -B "$D/test_r4l_altitude_replan.py"
```

**R4L has NOT been tested on actual PX4, ROS HIL or SITL.** It fixes the
missing altitude-retry branch, not the root cause of the vertical dip. Check
raw B-spline and EGO logs, then validate recovery in SITL/HIL before flight.
