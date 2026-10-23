#!/usr/bin/env bash
echo '[R2 REVIEW-ONLY / FLIGHT LOCKED] R3 ground-review clone remains flight locked.' >&2
echo '[R3] Planner loopback and final-hop guard have not been validated together with real sensors/PX4.' >&2
echo '[R3] Narrow obstacle geometry may be impassable; never bypass walls or health gates.' >&2
exit 97
