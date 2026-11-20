ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-irobot-create-msgs \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY deploy/rosbridges/ros2/start.sh /usr/local/bin/start-ros2-bridge
RUN chmod 755 /usr/local/bin/start-ros2-bridge

EXPOSE 9090
ENTRYPOINT ["/usr/local/bin/start-ros2-bridge"]
