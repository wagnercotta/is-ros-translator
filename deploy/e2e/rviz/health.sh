#!/usr/bin/env bash
set -euo pipefail
xdpyinfo -display :99 >/dev/null
pgrep -x rviz2 >/dev/null
python3 - <<'PY'
import socket
with socket.create_connection(('127.0.0.1', 6080), timeout=3):
    pass
PY
