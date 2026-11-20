#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=127.0.0.1 port:=9090 &
rosbridge_pid=$!

is-ros-translator --config /app/configs/create3.yaml &
gateway_pid=$!

terminate() {
  trap - EXIT INT TERM
  kill -TERM "$gateway_pid" "$rosbridge_pid" 2>/dev/null || true
  wait "$gateway_pid" 2>/dev/null || true
  wait "$rosbridge_pid" 2>/dev/null || true
}

trap terminate EXIT
trap 'exit 0' INT TERM

wait -n "$rosbridge_pid" "$gateway_pid"
