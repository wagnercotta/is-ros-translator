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
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roscore &
MASTER_PID=$!
BRIDGE_PID=''
cleanup() {
  trap - TERM INT EXIT
  if [[ -n "$BRIDGE_PID" ]]; then kill "$BRIDGE_PID" 2>/dev/null || true; fi
  kill "$MASTER_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT TERM INT
for _ in $(seq 1 45); do
  if rosparam get /rosdistro >/dev/null 2>&1; then break; fi
  sleep 1
done
rosparam get /rosdistro >/dev/null 2>&1 || { echo 'ROS1 master did not start' >&2; exit 1; }
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090 &
BRIDGE_PID=$!
python3 /opt/ros-lab/robot.py &
ROBOT_PID=$!
# Exit if the robot or ROS bridge dies unexpectedly.
while kill -0 "$ROBOT_PID" 2>/dev/null && kill -0 "$BRIDGE_PID" 2>/dev/null && kill -0 "$MASTER_PID" 2>/dev/null; do sleep 2; done
echo 'ROS1 lab process exited' >&2
exit 1
