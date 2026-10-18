#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
[ -f flight_evidence/rosbag.sha256 ] || {
  echo '[STOP] Flight bag hash was unavailable when frozen.' >&2; exit 1;
}
sha256sum -c flight_evidence/rosbag.sha256
