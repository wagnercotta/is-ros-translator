# Legacy ROS1 test fixture. The image intentionally remains isolated on the Docker lab network.
FROM ros:noetic-ros-base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-suite \
    ros-noetic-move-base-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-std-msgs \
    ros-noetic-actionlib \
    python3 && rm -rf /var/lib/apt/lists/*
COPY examples/ros1_simulator_node.py /opt/ros-lab/robot.py
COPY deploy/ros-test/ros1/start.sh /opt/ros-lab/start.sh
COPY deploy/ros-test/ros1/health.sh /opt/ros-lab/health.sh
RUN chmod +x /opt/ros-lab/start.sh /opt/ros-lab/health.sh
EXPOSE 9090
ENTRYPOINT ["/opt/ros-lab/start.sh"]
