"""ROS 2 wire types, Nav2/Create 3 navigation and action type verification."""

from __future__ import annotations

from collections.abc import Mapping
import logging
import threading

from ..config import RobotConfig
from ..errors import GatewayError, InvalidRequest, OperationTimeout, ROSUnavailable
from ..models import Target
from .base import pose_stamped

LOG = logging.getLogger(__name__)

# action_msgs/msg/GoalStatus: 4 is SUCCEEDED; 5 and 6 are CANCELED/ABORTED.
_ROS2_GOAL_STATUSES = {
    "UNKNOWN": 0,
    "ACCEPTED": 1,
    "EXECUTING": 2,
    "CANCELING": 3,
    "SUCCEEDED": 4,
    "CANCELED": 5,
    "ABORTED": 6,
}


def _normalise_goal_status(value: object) -> int | None:
    """Accept numeric ROS 2 statuses and roslibpy's textual enum representation."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if value in _ROS2_GOAL_STATUSES.values() else None
    if isinstance(value, float):
        return int(value) if value.is_integer() and int(value) in _ROS2_GOAL_STATUSES.values() else None
    if isinstance(value, str):
        name = value.strip()
        if name.startswith("GoalStatus."):
            name = name[len("GoalStatus."):]
        if name.startswith("STATUS_"):
            name = name[len("STATUS_"):]
        if name in _ROS2_GOAL_STATUSES:
            return _ROS2_GOAL_STATUSES[name]
        if name in ("0", "1", "2", "3", "4", "5", "6"):
            return int(name)
    return None


class ROS2Adapter:
    def __init__(self, robot: RobotConfig):
        self.robot = robot

    @property
    def map_service_type(self) -> str:
        return "nav_msgs/srv/GetMap"

    @property
    def odometry_type(self) -> str:
        return "nav_msgs/msg/Odometry"

    def navigation_goal(self, target: Target) -> dict:
        pose = pose_stamped(target, self.robot.navigation.frame_id)
        ros_type = self.robot.navigation.action_type
        if ros_type == "irobot_create_msgs/action/NavigateToPosition":
            if pose["header"]["frame_id"] != "odom":
                raise InvalidRequest("Create 3 navigation requires the odom frame")
            return {"goal_pose": pose, "achieve_goal_heading": True}
        if ros_type == "nav2_msgs/action/NavigateToPose":
            return {"pose": pose, "behavior_tree": ""}
        raise InvalidRequest(f"Navigation action type {ros_type} is not supported by navigate")

    async def validate_action_type(self, bridge: object, endpoint: str, ros_type: str) -> None:
        advertised = await bridge.get_action_type(endpoint)
        if not advertised or advertised != ros_type:
            raise InvalidRequest(f"ROS action type mismatch: advertised {advertised}, requested {ros_type}")

    @staticmethod
    def send_action(conn, name, ros_type, goal_dict, timeout, feedback):
        """Encode ROS 2 action goal and handle DDS action terminal states."""
        import roslibpy

        client = roslibpy.ActionClient(conn, name, ros_type)
        done = threading.Event()
        outcome = {}

        def result_cb(value):
            outcome["result"] = dict(value) if isinstance(value, Mapping) else value
            done.set()

        def error_cb(error):
            outcome["error"] = str(error)
            done.set()

        def feedback_cb(value):
            if feedback:
                feedback(dict(value) if isinstance(value, Mapping) else {"value": str(value)})

        goal_id = client.send_goal(roslibpy.Goal(goal_dict), result_cb, feedback_cb, error_cb)
        if not goal_id:
            raise ROSUnavailable(f"ROS 2 action {name} did not accept the goal")
        if not done.wait(timeout):
            try:
                client.cancel_goal(goal_id)
            except Exception:
                LOG.warning("Could not cancel ROS 2 action %s", name, exc_info=True)
            raise OperationTimeout(f"ROS 2 action {name} exceeded {timeout}s; cancellation best-effort")
        if "error" in outcome:
            raise GatewayError(outcome["error"], "action_failed", 502)
        result = outcome["result"]
        raw_status = result.get("status") if isinstance(result, Mapping) else None
        status = _normalise_goal_status(raw_status)
        if status is None:
            raise GatewayError(
                f"ROS 2 action {name} returned an invalid terminal status: {raw_status!r}",
                "action_failed",
                502,
            )
        if status != 4:
            raise GatewayError(f"ROS 2 action {name} finished with status {raw_status}", "action_failed", 502)
        if result.get("result") is False:
            raise GatewayError(f"ROS 2 action {name} reported failure", "action_failed", 502)
        # Keep the original representation for diagnostics, but use the ROS 2
        # numeric code in *both* fields consumed by downstream IS-Wire clients.
        normalised_result = dict(result)
        normalised_result["raw_status"] = raw_status
        normalised_result["status"] = status
        return {"goal_id": str(goal_id), "result": normalised_result, "status": status}
