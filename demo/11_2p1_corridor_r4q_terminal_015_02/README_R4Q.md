# Demo11 R4Q | last waypoint translation-to-yaw handoff

Built as an independent `demo` copy of R4P. No changes to private EGO Planner,
bridge, traj_server, obstacle-replanning gate, waypoints or site fences.

## The only flight behavior changed

* **Final waypoint P3 only**, just before switching from `TANGENT` to `FIXED` yaw:
  XY arrival requirement `dXY < 0.120 m` -> `dXY < 0.150 m`.
* At the same P3 translation arrival gate only, required uninterrupted stable
  time `0.80 s` -> `0.20 s`.
* The console displays `stable=x/0.20` at P3.
* P1/P2 translation thresholds, height/speed thresholds, actual terminal FIXED
  yaw completion check (still 1.0s), mission permissions, physical fences,
  PX4, POINT HOLD and final landing checks are all unchanged.
* Manual landing/CH7 revocation seen after user landing is not a control fault.

The modified gate may allow FIXED yaw to start with a bit more residual XY
motion; it does not guarantee yaw can finish or that the physical aircraft
stays within the specified tolerance. This build is not physically flight tested.

## Ubuntu installation

Copy ZIP to `~/Desktop`, then run:

```bash
cd ~/Desktop
unzip -n demo11_r4q_terminal_015_02.zip
python3 -B ~/Desktop/demo11_r4q_terminal_015_02_package/install_r4q.py
```

Installs only to
`~/contest1_uav/demo/11_2p1_corridor_r4q_terminal_015_02`
without modifying R4P, earlier demos or runtime. The installer verifies
original R4P code and refuses overwrites.

## Offline test (no PX4 commands)

```bash
D=~/contest1_uav/demo/11_2p1_corridor_r4q_terminal_015_02
python3 -B "$D/demo11_verify.py" --quiet
python3 -B "$D/test_r4q_terminal_gate.py"
```

## ROS launch sequence (only after controlled simulation verification)

1. `bash ~/contest1_uav/demo/11_2p1_corridor_r4q_terminal_015_02/start_core.sh`
2. `bash ~/contest1_uav/demo/11_2p1_corridor_r4q_terminal_015_02/start_backend.sh`
3. `bash ~/contest1_uav/demo/11_2p1_corridor_r4q_terminal_015_02/run_r4_flight.sh`

No new flight checks were added. Only the stated two endpoint acceptance
parameters were relaxed. Site/hardware safeguards retained.
