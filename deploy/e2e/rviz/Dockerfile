FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    ROS_DOMAIN_ID=31
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 ros-jazzy-rviz-default-plugins \
    ros-jazzy-nav-msgs ros-jazzy-tf2-ros \
    xvfb fluxbox x11vnc novnc websockify \
    mesa-utils libgl1-mesa-dri libglu1-mesa \
    x11-utils procps python3 \
    && rm -rf /var/lib/apt/lists/*
COPY deploy/e2e/rviz/demo.rviz /opt/ros-lab/demo.rviz
COPY deploy/e2e/rviz/start.sh /opt/ros-lab/start.sh
COPY deploy/e2e/rviz/health.sh /opt/ros-lab/health.sh
RUN chmod +x /opt/ros-lab/start.sh /opt/ros-lab/health.sh
EXPOSE 6080
ENTRYPOINT ["/opt/ros-lab/start.sh"]
