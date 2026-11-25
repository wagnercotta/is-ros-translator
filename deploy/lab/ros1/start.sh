#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/noetic/setup.bash
set -u

export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export DISPLAY=:98
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

roscore > /tmp/ros1-master.log 2>&1 &
master_pid=$!

pids=("$master_pid")
cleanup() {
    trap - EXIT INT TERM
    for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for attempt in $(seq 1 60); do
    rosparam get /rosdistro >/dev/null 2>&1 && break
    kill -0 "$master_pid" 2>/dev/null || { tail -n 30 /tmp/ros1-master.log; exit 1; }
    sleep 0.25
done
rosparam get /rosdistro >/dev/null 2>&1 || { echo 'ROS1 master unavailable' >&2; exit 1; }

roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9091 \
    > /tmp/ros1-bridge.log 2>&1 &
pids+=("$!")

python3 /opt/ros1-lab/simulator.py > /tmp/ros1-simulator.log 2>&1 &
pids+=("$!")

Xvfb :98 -screen 0 1600x900x24 -ac +extension GLX +render -noreset \
    > /tmp/ros1-xvfb.log 2>&1 &
xvfb_pid=$!
pids+=("$xvfb_pid")
for attempt in $(seq 1 60); do
    xdpyinfo -display :98 >/dev/null 2>&1 && break
    kill -0 "$xvfb_pid" 2>/dev/null || { tail -n 30 /tmp/ros1-xvfb.log; exit 1; }
    sleep 0.25
done
xdpyinfo -display :98 >/dev/null

fluxbox > /tmp/ros1-fluxbox.log 2>&1 &
pids+=("$!")
x11vnc -display :98 -rfbport 5901 -listen 127.0.0.1 -nopw -forever -shared \
    -noxdamage -noxrecord -noxfixes > /tmp/ros1-vnc.log 2>&1 &
pids+=("$!")
websockify --web=/usr/share/novnc 127.0.0.1:6081 127.0.0.1:5901 \
    > /tmp/ros1-websockify.log 2>&1 &
pids+=("$!")
rviz -d /opt/ros1-lab/rviz.rviz > /tmp/ros1-rviz.log 2>&1 &
pids+=("$!")

echo '[ros1-lab] ROS1 Noetic + /move_base simulator + RViz readying on localhost:6081' >&2
while true; do
    for pid in "${pids[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "ROS1 lab process exited: pid=$pid" >&2
            for name in ros1-master ros1-bridge ros1-simulator ros1-rviz ros1-websockify; do
                tail -n 25 "/tmp/$name.log" >&2 || true
            done
            exit 1
        fi
    done
    sleep 2
done
