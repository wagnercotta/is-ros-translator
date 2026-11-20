FROM ros:jazzy-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-visualization-msgs \
    python3 && rm -rf /var/lib/apt/lists/*
COPY examples/ros2_simulator_node.py /opt/ros-lab/robot.py
COPY deploy/ros-test/ros2/start.sh /opt/ros-lab/start.sh
COPY deploy/ros-test/ros2/health.sh /opt/ros-lab/health.sh
RUN chmod +x /opt/ros-lab/start.sh /opt/ros-lab/health.sh
EXPOSE 9090
ENTRYPOINT ["/opt/ros-lab/start.sh"]
