#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
python3 -B - <<'PY_VERIFY'
import hashlib, json
from pathlib import Path
data=json.loads(Path("flight_evidence/FLIGHT_BAG.json").read_text())
p=Path(data["bag_path"])
assert p.is_file(), "Flight bag missing"
h=hashlib.sha256()
with p.open("rb") as f:
    for chunk in iter(lambda: f.read(4*1024*1024), b""):
        h.update(chunk)
assert h.hexdigest()==data["bag_sha256"], "Flight bag hash mismatch"
print("[ OK ] Flight bag SHA256 verified:",p)
PY_VERIFY
