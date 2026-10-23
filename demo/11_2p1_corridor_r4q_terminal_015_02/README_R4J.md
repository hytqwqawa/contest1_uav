# R4J - FINAL_WAYPOINT XY-only, 1-second HOLD

This is an independent copy of the installed R4I, under `~/contest1_uav/demo/`.
No changes are applied to R4I, Demo10, shared runtime, or private EGO Planner.

## Requested change

At the **final waypoint only**, after EGO -> POINT HOLD has been verified:

- The `FINAL_WAYPOINT` continuous hold is **1.0 seconds** (formerly 3.0).
- Remove the final hold **relative height 0.8 +/- 0.14 m** rejection.
- Remove the final hold **|VZ| <= 0.18 m/s** rejection.
- Remove the duplicate relative-height and VZ rejection inside `demo9_posctl_handoff.py`
  (the one-shot POSCTL transfer also checks the final aircraft estimates).
- Retain **POINT XY error <= 0.22 m**, finite values, fresh odometry and
  POINT setpoint, PX4 connection, ARMED/OFFBOARD, flight health, RC permission,
  `demo_safe`, `AIRBORNE_HOLD`, and `POINT_ACTIVE` throughout that second.
- Keep the *AFTER_TAKEOFF* 3-second height/VZ/XY verification unchanged.
- Keep original route P1B=F3.5/L+0.25; P2/P3, 0.8m target height,
  0.20m/s EGO speed, R4H yaw cap 0.35 rad/s, R4F B-spline proof,
  R3 output corridor guard, physical CH5 ARM/CH6 POSCTL/CH7 permission,
  and pilot POSCTL landing unchanged.

**Safety note**: This is deliberately a reduced terminal verification. Height
and vertical velocity are still logged, but the terminal gate will no longer
refuse the pilot landing handoff based on these two quantities. The software
fence, upstream checks, estimator-health failsafes and pilot responsibility
remain. The offline reproduction of this user's values (relZ=.948m,
VZ=.007m/s, XYerr=.025m) passes the new terminal gate, subject to all other
live conditions. That result is not a real-flight validation.

The RViz straight+round geometry stays enabled. It still publishes to the
legacy `/contest1/demo11_r4i/geometry_markers` MarkerArray topic, matching the
existing RViz preset; this is visualization only.

## Install

DISARM and stop old R4I processes first. Move zip to Ubuntu Desktop:

```bash
cd ~/Desktop
unzip -n demo11_r4j_final_xy_1s.zip
python3 -B ~/Desktop/demo11_r4j_final_xy_1s_package/install_r4j.py
```

The installer requires 168 hash-verified *original* R4I source files, validates
its immutable safety files, and clones to:

`~/contest1_uav/demo/11_2p1_corridor_r4j_final_xy_1s`

It refuses destination overwrite and does not start ROS/PX4/flight processes.
An extra untracked diagnostic `.py` file inside R4I is not copied and is
not modified.

The pre-existing safe flight entrypoints are:

```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4j_final_xy_1s/start_core.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4j_final_xy_1s/start_backend.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4j_final_xy_1s/run_r4_flight.sh
```

Only use the final command following a real-world safety review and physical
pilot authorization. The package is **not flight tested**.
