#!/usr/bin/env bash
# R2 deliberately stops before launching mission/PX4 mode changes.
R2_LOCK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$R2_LOCK_DIR/r2_flight_lock.sh" || exit 97
# ACTUAL FLIGHT entrypoint. This never software-ARMs, but physical pilot gates
# downstream CAN launch a mission when the pilot authorizes it.
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$D/demo11_verify.py"
echo '[DEMO11] Flight launcher; ONLY for reviewed conditions + on-site pilot.'
exec "$D/run_demo9.sh"
