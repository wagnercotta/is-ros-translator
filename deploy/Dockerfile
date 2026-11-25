# Keep build-time docs and licence placeholders out of the image.
FROM python:3.10-slim AS builder
WORKDIR /build
COPY pyproject.toml ./
COPY src ./src
# The branch metadata references README/LICENSE/NOTICE files not present here.
# Drop only these build-stage metadata entries without changing pyproject.toml.
RUN sed -i '/^readme = /d; /^license = /d; /^license-files = /d' pyproject.toml \
    && python -m pip install --no-cache-dir --prefix=/install .
RUN PYTHONPATH=/install/lib/python3.10/site-packages \
    python -c "from is_wire.core import Channel, Subscription, Message; from is_msgs.robot_pb2 import RobotTaskRequest; from is_msgs.common_pb2 import Pose"

FROM python:3.10-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /app
COPY --from=builder /install /usr/local
COPY configs ./configs
RUN useradd --create-home --uid 10001 appuser
USER appuser
EXPOSE 8080
CMD ["is-ros-translator", "--config", "configs/vm-demo.yaml"]
