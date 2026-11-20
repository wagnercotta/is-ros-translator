#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
: "${ROS_MASTER_URI:?Set ROS_MASTER_URI to the ROS 1 master address}"

exec roslaunch rosbridge_server rosbridge_websocket.launch \
  address:="${ROSBRIDGE_BIND_ADDRESS:-127.0.0.1}" \
  port:="${ROSBRIDGE_PORT:-9091}"
