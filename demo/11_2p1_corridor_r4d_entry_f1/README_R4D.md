# Demo11 R4D: straight walls start at F=1 m

This release is an **independent copy of the installed R4C**, leaving Demo10,
R2/R3/R4/R4B/R4C, and shared runtime untouched.

## Agreed physical geometry

- The launch point is outside the physical straight corridor.
- Physical straight wall segment: **F=1.0..3.0m**, **L=-0.5..+1.0m**.
- The machine has nonzero size. With the previously reviewed 0.20m body
  radius, last-hop and preview guards apply from **F=0.8..3.2m**.
- Lateral center range in that section remains **L=-0.05..+0.55m**
  (0.20m body + 0.15m tracking allowance + 0.10m reserve per wall).
- From takeoff F≈0 to before F=0.8m, this *specific straight wall rule*
  is inactive. All prior overall Demo5 fence, RC/PX4 safety, yaw,
  mode/handoff, altitude, and continuity restrictions remain.
- F>3.2m: no extra straight-wall constraint, original global fence applies.

This fixes the actual ground refusal at F+0.011 L-0.065 and retry
F+0.001 L-0.060 while keeping F>=0.8m blocked if L<-0.05.

**Do not treat the pre-entrance section as clearance for the body at F=1m.**
As the body approaches the entry (F>=0.8m), its CENTER must be within
L[-0.05,+0.55]. The planner may reject a trajectory earlier at the preview
stage if its conservative B-spline control hull does not prove the entrance
safe. Do NOT bypass that rejection; diagnose its actual message.

## Installation (safe; no ROS or ARM)

Put `demo11_r4d_entry_f1.zip` on Ubuntu Desktop:

```bash
cd ~/Desktop
unzip -n demo11_r4d_entry_f1.zip
python3 -B ~/Desktop/demo11_r4d_entry_f1_package/install_r4d.py
```

Creates `~/contest1_uav/demo/11_2p1_corridor_r4d_entry_f1` once;
refuses overwrite and verifies every one of the 154 frozen R4C files.

## Starting (only after stopping older running ROS nodes, while DISARM)

Terminal 1:
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4d_entry_f1/start_core.sh
```
Terminal 2:
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4d_entry_f1/start_backend.sh
```
Terminal 3 (physical pilot authorization and safety gates remain):
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4d_entry_f1/run_r4_flight.sh
```

The candidate does NOT ARM automatically. Physical pilot/manual takeoff and
intervention remain unchanged. No-prop procedural CH6/CH7 challenge does not
return. This patch does not validate outdoor walls or physical clearance.

On a gate rejection, leave DISARM, inspect the *specific* error. Do not edit
`demo11_corridor.yaml` after install: source SHA would become inconsistent.
For an in-air anomaly, follow the previously reviewed pilot handover.
