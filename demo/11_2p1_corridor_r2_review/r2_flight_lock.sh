#!/usr/bin/env bash
# Do not remove the lock until the planner FSM and downstream guard are fixed,
# reviewed, and verified in DISARMED/no-prop/in-loop tests by the flight team.
echo '[R2 REVIEW-ONLY / FLIGHT LOCKED] P2 EGO-FSM reset is unresolved.' >&2
echo '[R2 REVIEW-ONLY / FLIGHT LOCKED] Preview/replan audit alone cannot prevent a live downstream command from briefly violating the corridor.' >&2
echo '[R2 REVIEW-ONLY / FLIGHT LOCKED] Do NOT arm/fly this R2 clone. Demo10/V1 are unchanged.' >&2
exit 97
