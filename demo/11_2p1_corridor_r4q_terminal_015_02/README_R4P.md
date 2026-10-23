# Demo11 R4P - supervised in-flight EGO trajectory continuity

**Purpose:** Complete each waypoint without stopping simply because the private
EGO Planner publishes a new collision-avoidance B-spline. The first approved
full-goal trajectory is released immediately, unchanged. Later trajectories
are staged in a demo-only ROS node until the mission checks them against the
existing global three-dimensional fence and swept-body straight-corridor rule.
Accepted candidates become the running `traj_server` trajectory with the
original target waypoint, unchanged EGO/POINT mux, and continuous yaw slew.

## Behaviour changes from R4O

* REMOVE R4O's `BLOCKED:n -> unconditional STOP` design.
* One initial preflight preview -> immediately EXECUTE, no software ARM.
* EGO **does not** plan extra waypoint or restart the mission on in-flight
  obstacle requests; it may still produce emergency replans if a collision is
  predicted. These MUST be handled, not ignored.
* New in-flight Bspline is held at the staging gate and only delivered to
  `traj_server` after passing the ALREADY EXISTING 3D/corridor checks.
* Gate publishes approved traj_id before release, preserving the bridge's
  original source authorisation. Bridge yaw is **never reset** on traj-id change.
* If a candidate fails any existing geometry/health check, or gate cannot
  confirm a safe handoff promptly, enter POINT HOLD and pilot takeover. No
  promise of 100% completion with physical obstacles or loss of localisation.
* Three EGO goals H=0.8, P1 F3.5 L+0.25; P2 F4.061 L-0.439;
  P3 F4.5 L-1.5; no additional waypoint.

## Check delta

* **Added geometric thresholds: NONE.** No new height, lateral, time-stable
  hover threshold or additional takeoff check.
* Existing B-spline altitude is INFO only, original EGO 3D dynamics retained.
* **Preserved:** global 3D site fence; existing R2 swept corridor proof; R3
  actual output hard gate; fresh PX4, CH6/CH7 RC, flight healthy; verified
  POINT HOLD / manual POSCTL takeover.
* Staged obstacle replan is given the **existing 0.60s command freshness
  window** to gain certification; if it cannot, we must not continue following
  the possibly blocked older trajectory indefinitely.

## Notes

No new private Planner compilation; unchanged runtime and immutable Demo10,
R4O. The new gate/mission handling requires real ROS/PX4 SIL/HIL validation
of staging and dynamic handoff before use in the competition. There is no
claim of a successful physical flight.
