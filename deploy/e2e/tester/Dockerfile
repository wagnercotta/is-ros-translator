FROM python:3.10-slim
WORKDIR /e2e
RUN python -m pip install --no-cache-dir 'is-wire==1.2.1' 'is-msgs==1.1.18' 'protobuf==3.20.3' 'six>=1.16,<2' 'websocket-client>=1.8,<2'
RUN python -c "from is_wire.core import Channel, Subscription, Message; from is_msgs.robot_pb2 import RobotTaskRequest; from is_msgs.common_pb2 import Pose"
COPY tests/integration/rviz_e2e.py /e2e/rviz_e2e.py
COPY tests/integration/pris_multi_robot_e2e.py /e2e/pris_multi_robot_e2e.py
CMD ["python", "-u", "/e2e/pris_multi_robot_e2e.py"]
