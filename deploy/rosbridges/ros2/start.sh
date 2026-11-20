#!/usr/bin/env bash
set -eo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set by the ROS image}"
# The generated ROS setup script references optional variables that are not
# defined in a clean shell. Enable nounset only after sourcing it.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:="${ROSBRIDGE_BIND_ADDRESS:-127.0.0.1}" \
  port:="${ROSBRIDGE_PORT:-9090}"
