#!/usr/bin/env bash
# Pure static validation: no ROS publishers, no service calls, no flight.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
python3 -B "$D/test_demo9_v3_offline.py" "$D"
bash -n "$D/start_core.sh" "$D/start_backend.sh" "$D/run_demo9_v3.sh" "$D/rollback_demo9_v3.sh"
echo '[OK] Demo9 V3 STATIC/OFFLINE PASS. Does NOT authorize flight.'
