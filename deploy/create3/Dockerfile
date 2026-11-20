FROM ros:jazzy-ros-base

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-irobot-create-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/rostranslator-venv
ENV PATH="/opt/rostranslator-venv/bin:${PATH}"

WORKDIR /app
COPY pyproject.toml README.md LICENSE NOTICE.md ./
COPY src ./src
RUN python3 -m pip install --no-cache-dir .
RUN python3 -c "from is_wire.core import Channel, Subscription, Message; from is_msgs.robot_pb2 import RobotTaskRequest; from is_msgs.common_pb2 import Pose"
COPY configs ./configs
COPY deploy/create3/start.sh /usr/local/bin/start-create3-gateway
RUN chmod 755 /usr/local/bin/start-create3-gateway

EXPOSE 8080 9090
ENTRYPOINT ["/usr/local/bin/start-create3-gateway"]
