# Demo11 R4O: single validated trajectory per waypoint

* Independent demo directory; based on R4N.
* For EACH waypoint: receive a new EGO B-spline, verify original geometry and existing physical gates, freeze exactly the certified `traj_id` at the `traj_server` input, immediately release that single trajectory after the pre/post-trigger gap checks.
* Disable task-level retry loops and the continuous in-flight trajectory approval loop. Any new B-spline while executing causes a **STOP and pilot POINT HOLD/POSCTL takeover**, not execution of an unverified replacement.
* The planner's **collision-safety behavior is NOT defeated**; it can still publish changed trajectories. The freeze gate makes that a terminal safety event, not permission to ignore obstacles and keep following the frozen trajectory blindly.
* EGO bridge no longer resets the tangent yaw slew merely because a trajectory id changed; normal POINT/yaw-mode handoff still seeds actual yaw.
* The existing CH5 physical ARM, CH6 pilot POSCTL takeover, CH7 RC revocation, R3 runtime output corridor guard, B-spline geometric fence, healthy localization and MUX remain unchanged.
* No added height checks, widened fences, new waypoint, or software ARM. New gate status watchdog is required solely to ensure the new frozen-trajectory mediator is alive; it does not impose a new geometry threshold.

**Important:** Private EGO Planner already suppresses routine periodic replanning for a manual full-goal trajectory. Multiple replacements in a few seconds may be an **actual collision warning** from the D435i map. In that case R4O will stop instead of claiming to complete the route. It must NOT be flown before simulation tests or after ignored collision warnings.

## Ubuntu installation

Place `demo11_r4o_single_plan.zip` in `~/Desktop`:

```bash
cd ~/Desktop
unzip -n demo11_r4o_single_plan.zip
python3 -B ~/Desktop/demo11_r4o_single_plan_package/install_r4o.py
```

Installation directory: `~/contest1_uav/demo/11_2p1_corridor_r4p_supervised_continuity`

Simulation-only startup:

```
bash ~/contest1_uav/demo/11_2p1_corridor_r4p_supervised_continuity/start_core.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4p_supervised_continuity/start_backend.sh
bash ~/contest1_uav/demo/11_2p1_corridor_r4p_supervised_continuity/run_r4_flight.sh
```

Do not test the new relay in flight before full ROS/PX4 SITL/HIL trajectory and forced-collision tests. Only one gate must run, and this demo's launch should remap the private traj_server's input onto `/contest1/demo11_r4p/bspline_to_traj_server`.

Verification logs of a single leg must show: `FROZEN:id` and `RELEASED` followed by a stable unchanging traj id until terminal HOLD. `BLOCKED:n` must cause HOLD, never be ignored.
