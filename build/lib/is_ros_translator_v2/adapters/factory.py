"""Select protocol adapters and bind robot IDs to ROSBridge instances."""

from __future__ import annotations

from ..config import RobotConfig, Settings
from ..rosbridges.client import RosBridge
from .ros1 import ROS1Adapter
from .ros2 import ROS2Adapter


def create_adapter(robot: RobotConfig):
    return ROS1Adapter(robot) if robot.ros_version == 1 else ROS2Adapter(robot)


def create_adapters(robots: dict[str, RobotConfig]):
    return {robot_id: create_adapter(robot) for robot_id, robot in robots.items()}


def create_transports(settings: Settings):
    return {robot_id: RosBridge(settings.resolved_robot(robot_id))
            for robot_id in settings.robots}
