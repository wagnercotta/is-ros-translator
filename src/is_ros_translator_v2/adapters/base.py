"""ROS wire-format conversions shared by the ROS 1 and ROS 2 adapters."""

from __future__ import annotations

import math
from typing import Protocol

from ..config import RobotConfig
from ..errors import InvalidRequest
from ..models import Target


def pose_stamped(target: Target, default_frame: str) -> dict:
    """Translate a semantic navigation target to a ROS PoseStamped structure."""
    p = target.position
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, target.yaw)):
        raise InvalidRequest("Position and yaw must be finite")
    return {"header": {"frame_id": target.frame_id or default_frame},
            "pose": {"position": {"x": p.x, "y": p.y, "z": p.z},
                     "orientation": {"x": 0.0, "y": 0.0,
                                     "z": math.sin(target.yaw / 2), "w": math.cos(target.yaw / 2)}}}


class ROSAdapter(Protocol):
    robot: RobotConfig

    @property
    def map_service_type(self) -> str: ...

    @property
    def odometry_type(self) -> str: ...

    def navigation_goal(self, target: Target) -> dict: ...

    async def validate_action_type(self, bridge: object, endpoint: str, ros_type: str) -> None: ...
