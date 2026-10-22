#!/usr/bin/env bash
# Single documented flight candidate entry; NO software ARM.
set -eo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -B "$D/demo11_verify.py" --quiet
bash "$D/r4_flight_gate.sh" || exit 97
exec bash "$D/run_demo11.sh"
