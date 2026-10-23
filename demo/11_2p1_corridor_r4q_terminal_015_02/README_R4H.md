# Demo11 R4H - Slower yaw / fail-closed handoff candidate

The 2026-10-22 P2 flight log showed external pose vs PX4 yaw disagreement
increase -0.37, -2.23, -3.88, -5.05 degrees during P2. The C++ state monitor
lost READY after >5 degrees persisted for its existing 0.40 seconds.
PX4 odometry/health became unavailable to the trajectory interface, causing
post-handoff failure and an unhelpful second `flight_healthy=False` traceback.

R4H does NOT relax localization, flight, corridor or RC limits. Its two changes:

1. The EGO bridge yaw reference rate is reduced from 0.80 rad/s to 0.35 rad/s;
   max yaw lead from 0.30 to 0.18 rad. Its hard sensor/PX4 yaw disagreement
   threshold remains 5 degrees. This may reduce estimator lag during rotation,
   but it is not verified to cure the underlying disagreement in hardware.
2. An EGO->POINT HOLD handoff still requires fresh POINT/MUX/MAVROS data.
   After a successful handoff, a one-second settle verifies continuing
   health, OFFBOARD, RC, POINT ownership and active output. When health is
   already lost, an exception does NOT try to re-enable POINT a second time.
   The original failure is preserved; pilot takeover is required.

R4G route unchanged: P1 F3.000 L+0.250, P2 F4.061 L-0.439,
P3 F4.500 L-1.500; height 0.8 m, velocity 0.2 m/s.
Wall F1..3, guard F0.8..3.2, body-center L[-0.15,+0.65] unchanged.
Planner, spline proof, hard-abort, final interface, V4 MUX, frame guard,
RC hardware ARM/takeover, and all 5-degree localization thresholds unchanged.

This candidate has passed OFFLINE source and behavioral model tests only.
It has NOT been tested on real PX4 and is NOT flight-certified. A repeated
`READY lost` means investigate T265/external yaw vs PX4 EKF and timing or
calibration, rather than further weakening the ready monitor.

Install (DISARM; stop old Core and Backend):
  cd ~/Desktop && unzip -n demo11_r4h_yaw_handoff.zip
  python3 -B ~/Desktop/demo11_r4h_yaw_handoff_package/install_r4h.py

Independent directory:
  ~/contest1_uav/demo/11_2p1_corridor_r4h_yaw_handoff

After independent bench validation, Core/Backend/Mission are respectively:
  bash ~/contest1_uav/demo/11_2p1_corridor_r4h_yaw_handoff/start_core.sh
  bash ~/contest1_uav/demo/11_2p1_corridor_r4h_yaw_handoff/start_backend.sh
  bash ~/contest1_uav/demo/11_2p1_corridor_r4h_yaw_handoff/run_r4_flight.sh

Never bypass a live `flight_healthy=False` or `[R3 CORRIDOR] HARD ABORT`.
Old R4G, prior successful versions, and shared runtime are untouched.
