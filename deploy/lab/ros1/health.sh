#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/noetic/setup.bash
set -u
rosservice list | grep -qx '/rosapi/topics'
rosnode list | grep -qx '/ros1_simulated_robot'
python3 - <<'PY'
import socket
for port in (9091, 6081):
    with socket.create_connection(('127.0.0.1', port), timeout=3):
        pass
PY
