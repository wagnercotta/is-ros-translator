FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-suite \
    ros-noetic-move-base-msgs \
    ros-noetic-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY deploy/rosbridges/ros1-noetic/start.sh /usr/local/bin/start-ros1-bridge
RUN chmod 755 /usr/local/bin/start-ros1-bridge

EXPOSE 9091
ENTRYPOINT ["/usr/local/bin/start-ros1-bridge"]
