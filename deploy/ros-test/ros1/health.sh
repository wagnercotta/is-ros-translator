#!/usr/bin/env bash
# ROS setup scripts may expand unset AMENT_* / CATKIN_* variables.
# Source without nounset, then restore strict variable checking.
set -eo pipefail
# Explicitly clear nounset even if inherited via SHELLOPTS or bash -u.
set +u
# ament uses this variable before it is defined in some ROS versions.
: "${AMENT_TRACE_SETUP_FILES:=}"
export AMENT_TRACE_SETUP_FILES
source /opt/ros/noetic/setup.bash
set -u
rosservice list | grep -q '^/rosapi/topics$'
python3 - <<'PY'
import socket
socket.create_connection(('127.0.0.1', 9090), 2).close()
PY
