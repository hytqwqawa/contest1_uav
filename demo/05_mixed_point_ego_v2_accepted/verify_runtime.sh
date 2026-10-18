#!/usr/bin/env bash
set -euo pipefail
ROOT="${CONTEST1_ROOT:-$HOME/contest1_uav}"
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
sha256sum -c "$HERE/RUNTIME_SOURCES.sha256"
