#!/usr/bin/env bash
set -euo pipefail
D="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HOME/contest1_uav"
sha256sum -c "$D/RUNTIME.sha256"
