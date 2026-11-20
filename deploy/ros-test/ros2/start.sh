#!/usr/bin/env bash
# ROS 2 Jazzy robot + rosbridge / rosapi for Docker labs.
# ROS setup scripts may expand unset AMENT_* / CATKIN_* variables.
# Source without nounset, then restore strict variable checking.
set -eo pipefail
echo "[ros2] is-ros-translator E2E startup v0.3.4" >&2
# Explicitly clear nounset even if inherited via SHELLOPTS or bash -u.
set +u
# ament uses this variable before it is defined in some ROS versions.
: "${AMENT_TRACE_SETUP_FILES:=}"
export AMENT_TRACE_SETUP_FILES
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# rosbridge_suite versions shipped for some Jazzy installations mistakenly use
# integer 0 for a DOUBLE parameter, causing InvalidParameterTypeException.
# Passing 0.0 explicitly avoids that crash (rosbridge_suite issue #1141).
# Bind to all container interfaces so that gateway/tester can reach port 9090.
echo '[ros2] Starting rosbridge + rosapi on 0.0.0.0:9090 (delay_between_messages=0.0)' \
  >&2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  port:=9090 address:=0.0.0.0 delay_between_messages:=0.0 &
BRIDGE_PID=$!

echo '[ros2] Starting the kinematic robot simulation' >&2
python3 -u /opt/ros-lab/robot.py &
ROBOT_PID=$!

cleanup() {
  trap - EXIT INT TERM
  kill "$BRIDGE_PID" "$ROBOT_PID" 2>/dev/null || true
  wait "$BRIDGE_PID" "$ROBOT_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Fail loudly if either important process terminates: don't leave a running
# but permanently unhealthy container where the logs are inconclusive.
status=0
wait -n "$BRIDGE_PID" "$ROBOT_PID" || status=$?
echo "[ros2] A required process exited (status=${status}); stopping container." >&2
exit 1
