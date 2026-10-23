# Demo11 R4I: P1B first + RViz straight/round corridor schematic

This is an **isolated clone of installed, checksum-verified R4H**. It replaces
P1 with the requested earlier suggestion P1B; it does **not** insert a fourth
waypoint. R4H yaw-rate mitigation and post-handoff fail-closed behavior are
retained. Never edit Demo10, shared runtime or the existing R4H directory.

## Flight route (Demo F/L meters)

- **P1=P1B**: F+3.500, L+0.250, H0.800, EGO, yaw 0 deg
- P2: F+4.061, L-0.439, H0.800, EGO, yaw -135 deg
- P3: F+4.500, L-1.500, H0.800, EGO, yaw -180 deg

The new P1 is outside the straight wall end (wall F1..3, *body-center guard*
F0.8..3.2). It is a new **unflown** start point for the bend. It does **not**
prove the P2 trajectory is feasible: existing R4F B-spline certification and R3
final MAVROS setpoint guard still reject unsafe sampled/control trajectories.

## RViz display (automatically shown at mission pre-arm)

`start_demo5.sh` starts the read-only `rviz_corridor_tubes.py` node after a NEW
`session_frame.yaml` is recorded, and opens the existing RViz preset with the
added marker display. The drawing is in RViz Fixed Frame **world**, using the
actual origin, forward and left vectors recorded at DISARM by Frame Guard.
The publisher is `/contest1/demo11_r4i/geometry_markers` (MarkerArray).

- **Yellow wireframe**: straight physical wall XY footprint, F1..3,
  L[-0.5,+1.0]. The displayed vertical height uses the software fence's
  1.3m limit; it is *not* a measured wall height.
- **Green wireframe**: protected **aircraft center** region in the straight,
  F0.8..3.2, L[-0.15,+0.65]. These are the existing runtime limits; no new
  safety decision is made by this visualization.
- **Cyan circular 3D quarter-tube**: the **reference** clockwise 90-degree
  circle, center F3.0/L-1.5, R1.5, and a decorative 1.5m diameter tube
  cross section. This tube is **schematic only**, not a measured curved wall
  or actual obstacle map; **no curved corridor is enforced by this marker**.
- **Orange/pink point spheres**: P1=P1B, P2, P3. Existing actual trajectory,
  EGO preview, pointclouds and yaw visuals in RViz are retained.

The circular arc reference begins at F3,L0; P1B=F3.5,L+.25 is **not** on this
ideal circle. EGO is free to plan a different but guard-compliant curve.

`session_frame.yaml` is removed and regenerated per mission by the existing
frame guard. The MarkerArray node only publishes once it finds a session whose
`demo5_plan_snapshot` exactly matches this version's route. It never publishes
MAVROS setpoints, takes control, or bypasses flight authorization. For a
separate viewer once Frame Guard is active, run `bash view_r4i_rviz.sh`.

## Install (DISARM; R4H already installed)

Move `demo11_r4i_p1b_rviz.zip` to Ubuntu Desktop and run:

```bash
cd ~/Desktop
unzip -n demo11_r4i_p1b_rviz.zip
python3 -B ~/Desktop/demo11_r4i_p1b_rviz_package/install_r4i.py
```

New directory: `~/contest1_uav/demo/11_2p1_corridor_r4i_p1b_rviz`.
The installer requires the original R4H's 163 tracked file hashes and refuses
an existing destination. It copies only inside `~/contest1_uav/demo` and makes
no ROS or PX4 calls. It preserves all original physical RC ARM/takeover,
OFFBOARD, watchdog, B-spline and collision gates.

Core / Backend / mission (only after independent field review and a safe site):

```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4i_p1b_rviz/start_core.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4i_p1b_rviz/start_backend.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4i_p1b_rviz/run_r4_flight.sh
```

**Safety:** These commands run a real flight candidate once the physical pilot
has authorized it. R4I is not flight validated; review clearances and the
actual P2 B-spline before attempting flight. On failure, do not override
HARD ABORT or `flight_healthy=False`; use CH7 LOW + CH6 POSCTL pilot takeover.
