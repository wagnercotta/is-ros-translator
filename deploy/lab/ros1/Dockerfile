FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_MASTER_URI=http://127.0.0.1:11311 \
    ROS_IP=127.0.0.1 \
    DISPLAY=:98 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-suite \
    ros-noetic-move-base-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-actionlib \
    ros-noetic-rviz \
    ros-noetic-visualization-msgs \
    ros-noetic-tf2-ros \
    xvfb fluxbox x11vnc novnc websockify \
    mesa-utils libgl1-mesa-dri libglu1-mesa \
    x11-utils procps python3 \
    && rm -rf /var/lib/apt/lists/*

COPY deploy/lab/ros1/simulator.py /opt/ros1-lab/simulator.py
COPY deploy/lab/ros1/rviz.rviz /opt/ros1-lab/rviz.rviz
COPY deploy/lab/ros1/start.sh /opt/ros1-lab/start.sh
COPY deploy/lab/ros1/health.sh /opt/ros1-lab/health.sh
RUN chmod +x /opt/ros1-lab/start.sh /opt/ros1-lab/health.sh

# Uses host networking. Ports are bound to localhost by start.sh.
EXPOSE 9091 6081
ENTRYPOINT ["/opt/ros1-lab/start.sh"]
