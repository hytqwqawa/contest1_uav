# Demo11 R4G - P1 corridor-center approach (not flight-certified)

**Cause observed:** R4F P2 traj_id=2 succeeded on one sortie but on another
real sampled B-spline point [117] at F+3.157 L-0.151 exceeded the fixed
center limit -0.150 by 0.001m, and was CORRECTLY rejected. The other flight
proves the stack can traverse, NOT that clearance is repeatably robust.

**Single route change:** P1 F+3.000, L+0.000 -> F+3.000, L+0.250.
P1 ends centered between straight walls L=-0.5 and L=+1.0. P2 remains
F+4.061 L-0.439 yaw=-135; P3 remains F+4.500 L-1.500 yaw=-180.
Height H0.80m, velocity 0.20m/s, physical walls F1..3, swept body
F0.8..3.2, and center gate L[-0.150,+0.650] remain UNCHANGED.
The EGO planner and R4F mathematical spline check remain UNCHANGED.
All RC/PX4 HOLD/hard-abort/MUX gate implementations remain UNCHANGED.

**Why route is more robust:** P2 begins from the middle of the corridor rather
than near the negative-L safety limit. This makes a near-wall turn less
likely, but it cannot guarantee actual EGO B-spline feasibility. Any actual
out-of-bounds spline is still rejected by R4F's exact sampled+bounded proof.
Do not weaken the corridor check or forcibly proceed after a rejection.

**Installation** (PX4 DISARMED and old Core/Backend stopped):
1. Place `demo11_r4g_center_p1.zip` on Ubuntu Desktop.
2. `cd ~/Desktop && unzip -n demo11_r4g_center_p1.zip`
3. `python3 -B ~/Desktop/demo11_r4g_center_p1_package/install_r4g.py`

Independent target: `~/contest1_uav/demo/11_2p1_corridor_r4g_center_p1`
Frozen R4F/R4E/R4D/Demo10/runtime untouched. Install does NOT ARM or start ROS.

**Before any new physical flight:** inspect the actual measured exit/corner
clearance for L+0.25 at F3.0 and capture a no-prop P1/P2 trajectory preview;
review its continuous corridor proof and the airframe clearance. The new route
has ONLY software/offline tests so far. Pilot retains physical ARM and
CH6/CH7 immediate takeover; use an open obstacle-free area first.

Terminal 1: `bash ~/contest1_uav/demo/11_2p1_corridor_r4g_center_p1/start_core.sh`
Terminal 2: `bash ~/contest1_uav/demo/11_2p1_corridor_r4g_center_p1/start_backend.sh`
Terminal 3: `bash ~/contest1_uav/demo/11_2p1_corridor_r4g_center_p1/run_r4_flight.sh`

The launch gate remains real-time; terminal commands alone do not confer
flight safety approval. This candidate is NOT flight-tested.
