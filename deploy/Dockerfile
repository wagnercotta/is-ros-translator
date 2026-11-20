FROM python:3.10-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /app
COPY pyproject.toml README.md LICENSE NOTICE.md ./
COPY src ./src
RUN python -m pip install --no-cache-dir .
# Fail during image build if the native PrIS SDK imports are incomplete.
RUN python -c "from is_wire.core import Channel, Subscription, Message; from is_msgs.robot_pb2 import RobotTaskRequest; from is_msgs.common_pb2 import Pose"
COPY configs ./configs
COPY examples ./examples
RUN useradd --create-home --uid 10001 appuser
USER appuser
EXPOSE 8080
CMD ["is-ros-translator", "--config", "configs/vm-demo.yaml"]
