#!/usr/bin/env bash
# Readiness transport probe: full ROS graph checks belong to the E2E runner.
set -euo pipefail
python3 - <<'PY'
import socket
try:
    with socket.create_connection(('127.0.0.1', 9090), timeout=3):
        pass
except OSError as exc:
    print(f'rosbridge 127.0.0.1:9090 unavailable: {exc}', flush=True)
    raise SystemExit(1)
PY
