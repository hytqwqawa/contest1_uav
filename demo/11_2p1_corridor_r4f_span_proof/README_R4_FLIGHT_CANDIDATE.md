# Demo11 R4F: one-command terminal 3, no procedural RC takeover acceptance

R4F is an independent candidate cloned from installed R4D after checking **156 SHA256** tracked files. Earlier R4B/C removed only the redundant procedural RC acceptance. It removes ONLY the additional preflight CH6/CH7 switch choreography, `PROPS_REMOVED` response, `PILOT_SIGNOFF`, and 45-minute manual approval file. It does **not** disable the actual RC Supervisor, CH6/CH7 live switch state checks, physical pilot ARM, PX4 failsafe, flight_healthy, V4 MUX, fresh Frame Guard, R3 final MAVROS setpoint corridor monitor, P2 private Planner, ROS odometry/vision inputs or emergency HOLD. Any rejected live safety check still exits 97.

**Not flight certified.** Prior R3 hardware read-only PASS was achieved with backend running and power connected; an onboard no-props PX4 mode transition/takeover test, with the final version, has not been demonstrated. Removing a mandatory step from the software is the operator's decision and does not verify manual takeover is safe. Pilot is responsible for site clearance, physical RC behavior and battery pack/individual cell health. Do not place the obstacle F1.5/L+0.3; geometric clearance may be insufficient.

## Install (NO flight)

```bash
cd ~/Desktop
unzip -n demo11_r4b_no_rc_acceptance.zip
python3 -B ~/Desktop/demo11_r4b_no_rc_acceptance/install_r4b.py
```

R4F target: `~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/`.

## Startup

Only one version can own Core/Backend. Shut down old R4/R3 processes in a controlled fashion **while DISARMED**, then start R4F:

Terminal 1:
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/start_core.sh
```

Terminal 2:
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/start_backend.sh
```

Terminal 3 (once Core/Backend ready and pilot/site ready):
```bash
bash ~/contest1_uav/demo/11_2p1_corridor_r4f_span_proof/run_r4_flight.sh
```

Terminal 3 requires no `--prepare`, no special CH6/CH7 switch cycle and no confirmation prompts before handing off to the original Demo11 pilot-controlled takeoff. **It does not ARM autonomously**. For pre-start state: PX4 CONNECTED, DISARM, POSCTL, CH6 HIGH, CH7 LOW, health true, battery valid and within existing 6S trial bounds, R4F process identities and exclusive setpoint publisher. Flight always uses P1 F3 L0, P2 F4.061 L-0.439, P3 F4.5 L-1.5, H0.8m, max 0.20m/s, pilot landing. All post-ARM frame/session checks remain intact.

If `BLOCK` or exit 97: stop and diagnose; never bypass the remaining runtime gates. If abnormal airborne state: flight pilot uses CH7 LOW and CH6 POSCTL for supervised descent. Do not switch versions while airborne. Demo10, Demo11 V1/R2/R3, R4, and runtime are not modified.
