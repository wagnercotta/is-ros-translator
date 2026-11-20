#!/usr/bin/env bash
# ROS setup scripts may expand unset AMENT_* / CATKIN_* variables.
# Source without nounset, then restore strict variable checking.
set -eo pipefail
echo "[rviz] is-ros-translator E2E startup v0.3.4" >&2
# Explicitly clear nounset even if inherited via SHELLOPTS or bash -u.
set +u
# ament uses this variable before it is defined in some ROS versions.
: "${AMENT_TRACE_SETUP_FILES:=}"
export AMENT_TRACE_SETUP_FILES
source /opt/ros/jazzy/setup.bash
set -u
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
# Uses Mesa's software rendering (llvmpipe). GPU passthrough is not necessary.
Xvfb :99 -screen 0 1600x900x24 -ac +extension GLX +render -noreset > /tmp/xvfb.log 2>&1 &
XVFB_PID=$!
for attempt in $(seq 1 60); do
  if xdpyinfo -display :99 >/dev/null 2>&1; then break; fi
  if ! kill -0 "$XVFB_PID" 2>/dev/null; then cat /tmp/xvfb.log >&2; exit 1; fi
  sleep 0.3
done
xdpyinfo -display :99 >/dev/null
fluxbox > /tmp/fluxbox.log 2>&1 &
WM_PID=$!
x11vnc -display :99 -rfbport 5900 -listen 127.0.0.1 -nopw -forever -shared \
    -noxdamage -noxrecord -noxfixes > /tmp/x11vnc.log 2>&1 &
VNC_PID=$!
websockify --web=/usr/share/novnc 0.0.0.0:6080 127.0.0.1:5900 > /tmp/websockify.log 2>&1 &
WEB_PID=$!
rviz2 -d /opt/ros-lab/demo.rviz > /tmp/rviz2.log 2>&1 &
RVIZ_PID=$!
cleanup() {
  trap - EXIT INT TERM
  kill "$RVIZ_PID" "$WEB_PID" "$VNC_PID" "$WM_PID" "$XVFB_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
while kill -0 "$RVIZ_PID" 2>/dev/null && kill -0 "$WEB_PID" 2>/dev/null \
      && kill -0 "$VNC_PID" 2>/dev/null && kill -0 "$XVFB_PID" 2>/dev/null; do
  sleep 2
done
echo 'RViz/VNC process exited unexpectedly; recent logs:' >&2
for file in /tmp/rviz2.log /tmp/websockify.log /tmp/xvfb.log /tmp/x11vnc.log; do
  echo "---- $file ----" >&2
  tail -n 30 "$file" >&2 || true
done
exit 1
