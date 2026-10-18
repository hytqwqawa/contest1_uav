#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
python3 -B - <<'PYCHECK'
from pathlib import Path
import hashlib, json
j=json.loads(Path("flight_evidence/FLIGHT_BAG.json").read_text())
p=Path(j["bag_path"])
assert p.is_file(), "Flight bag missing"
h=hashlib.sha256()
with p.open("rb") as f:
    for data in iter(lambda: f.read(4*1024*1024), b""):
        h.update(data)
assert h.hexdigest()==j["bag_sha256"], "Flight bag SHA256 mismatch"
print("[ OK ] Flight bag SHA256 verified:",p)
PYCHECK
