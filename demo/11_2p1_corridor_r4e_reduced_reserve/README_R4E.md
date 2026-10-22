# Demo11 R4E: small and bounded straight-corridor allowance adjustment

Independent, hash-checked copy of R4D. **No changes to Demo10, runtime,
R4B/C/D, flight waypoint coordinates, PX4 parameters, or other guard logic.**

## EXACT change

Physical walls: F=1.0..3.0, L=-0.5..+1.0m (unchanged).
Center check zone: F=0.8..3.2m (unchanged, includes airframe length).

| component | R4D | R4E |
| --- | --- | --- |
| airframe assumed halfwidth | 0.20m | 0.20m |
| tracking allowance | 0.15m | 0.15m |
| **extra reserve** | **0.10m** | **0.00m** |
| cumulative lateral clearance | 0.45m | 0.35m |
| accepted center L | [-0.05,+0.55]m | **[-0.15,+0.65]m** |

**WARNING**: L[-0.5,+1.0]m describes the wall inner faces, NOT a safe
range for the vehicle center. Enforcing only those wall coordinates would
allow propellers to intersect the walls. This version does NOT do that.
Body/tracking allowances remain assumptions, not a verified physical guarantee.

The real previous R4D P1 yaw abort was at F3.178, L about -0.050,
with subsequent observations around L=-0.070. Those positions fall inside
R4E's new range; the old unsafe bypass L=-0.904 still fails closed.
This change does not guarantee P1->P2 safety. At an actual abort, HOLD remains
latched and the pilot MUST take over. The POINT handoff error after a HARD ABORT
is a secondary status mismatch that remains to be corrected separately.

## Install while DISARM (no ROS services/ARM used)

Place demo11_r4e_reduced_reserve.zip on Desktop and run:

```bash
cd ~/Desktop
unzip -n demo11_r4e_reduced_reserve.zip
python3 -B ~/Desktop/demo11_r4e_no_extra_reserve_package/install_r4e.py
```

Installation uses only tracked 156 SHA256 R4D source files; fails closed if
original differs, refuses overwrites, verifies all new 158 files and tests.
It also restores execute permission for 3 shell wrappers that R4D had missed.

R4E installed at:
`~/contest1_uav/demo/11_2p1_corridor_r4e_reduced_reserve`

After stopping the old R4D Core and Backend and confirming DISARM, the
candidate launch paths are:

Terminal 1:
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4e_reduced_reserve/start_core.sh
```
Terminal 2:
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4e_reduced_reserve/start_backend.sh
```
Terminal 3 (only after site pilot signoff):
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4e_reduced_reserve/run_r4_flight.sh
```

The code never software-ARMs and retains live RC/PX4 checks. **Not flight
certified; no live test of R4E has been performed.** If genuine physical walls
are erected, measure actual blade circle and tracking performance first;
do not attempt obstacle passage until clearance is demonstrated.
