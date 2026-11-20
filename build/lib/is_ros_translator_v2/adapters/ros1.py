"""ROS 1 wire types and move_base navigation goal translation."""

from __future__ import annotations

from collections.abc import Mapping

from ..config import RobotConfig
from ..errors import GatewayError, OperationTimeout
from ..models import Target
from .base import pose_stamped


class ROS1Adapter:
    def __init__(self, robot: RobotConfig):
        self.robot = robot

    @property
    def map_service_type(self) -> str:
        return "nav_msgs/GetMap"

    @property
    def odometry_type(self) -> str:
        return "nav_msgs/Odometry"

    def navigation_goal(self, target: Target) -> dict:
        return {"target_pose": pose_stamped(target, self.robot.navigation.frame_id)}

    async def validate_action_type(self, bridge: object, endpoint: str, ros_type: str) -> None:
        # ROS 1 rosapi has no portable action-type introspection endpoint.
        # The configured type remains mandatory; payload schema is checked by Core.
        return None

    @staticmethod
    def send_action(conn, name, ros_type, goal_dict, timeout, feedback):
        """Encode ROS 1 actionlib request and interpret its terminal status."""
        import roslibpy
        from roslibpy.ros1.actionlib import ActionClient, Goal

        client = ActionClient(conn, name, ros_type)
        goal = Goal(client, roslibpy.Message(goal_dict))
        if feedback:
            goal.on("feedback", lambda msg: feedback(dict(msg) if isinstance(msg, Mapping)
                                                    else {"value": str(msg)}))
        try:
            goal.send(timeout=timeout)
            result = goal.wait(timeout=timeout)
            status = goal.status or {}
            status_value = status.get("status") if isinstance(status, dict) else status
            if not goal.is_finished:
                goal.cancel()
                raise OperationTimeout(f"ROS 1 action {name} exceeded {timeout}s; cancellation requested")
            if status_value != 3:
                raise GatewayError(f"ROS 1 action {name} ended with status {status_value}", "action_failed", 502)
            return {"status": status_value, "result": dict(result) if isinstance(result, Mapping) else result}
        finally:
            client.dispose()
