# DEMO 3 — ACCEPTED

Accepted: 2026-10-05

Capability:
- successful takeoff
- multi-waypoint P1 -> P2 -> P3
- EGO obstacle avoidance
- corrected large-yaw alignment behavior
- successful landing
- near-ground T265 landing sanitizer

Accepted operating configuration:
- EGO max_vel: 0.20 m/s
- takeoff speed: 0.40 m/s
- takeoff acceleration: 0.80 m/s^2
- landing terminal speed: 0.20 m/s
- terminal range trigger: 0.10 m
- first-gap dXY <= 0.08 m
- first-gap dZ <= 0.08 m

Safety:
- NO software ARM
- CH5 physical ARM / DISARM
- CH6 manual POSCTL takeover
- CH7 autonomy permission
- landing sanitizer locks near touchdown
- original NaN guard retained

Official three-terminal operation:

Terminal 1:
    ./start_core.sh

Terminal 2:
    ./start_backend.sh

Terminal 3:
    ./run_mission.sh

runtime_snapshot/ contains the relevant runtime-source snapshot
used when Demo 3 was frozen. Stable runtime is not deleted or modified.
