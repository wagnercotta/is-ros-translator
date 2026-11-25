ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
# Optional for generic ROSBridge images; required by deploy/lab/compose.yaml.
ARG PI_WIFI_IP=""

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-irobot-create-msgs \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# A build-time IPv4 is injected into the Fast DDS peer configuration.
# The physical lab fails fast if LAB_PI_WIFI_IP is missing; generic builds
# can omit PI_WIFI_IP when they do not use a Fast DDS profile.
COPY configs/fastdds-unicast.example.xml /etc/fastdds/unicast.xml
RUN if [ -n "$PI_WIFI_IP" ]; then \
      python3 -c 'import ipaddress,pathlib,sys; ip=str(ipaddress.IPv4Address(sys.argv[1])); path=pathlib.Path("/etc/fastdds/unicast.xml"); path.write_text(path.read_text().replace("PI_WIFI_IP",ip))' "$PI_WIFI_IP"; \
    fi

COPY deploy/rosbridges/ros2/start.sh /usr/local/bin/start-ros2-bridge
RUN chmod 755 /usr/local/bin/start-ros2-bridge

EXPOSE 9090
ENTRYPOINT ["/usr/local/bin/start-ros2-bridge"]
