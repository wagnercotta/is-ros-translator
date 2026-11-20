"""Default-deny endpoint policy. Patterns are anchored fnmatch expressions."""
from fnmatch import fnmatchcase
from .config import RobotConfig
from .errors import AccessDenied, InvalidRequest


def require_allowed(robot: RobotConfig, kind: str, endpoint: str) -> None:
    if not isinstance(endpoint, str) or not endpoint.startswith("/") or len(endpoint) > 240:
        raise InvalidRequest("ROS endpoint must be an absolute name shorter than 241 characters")
    patterns = getattr(robot.allowed, kind)
    if not any(fnmatchcase(endpoint, pattern) for pattern in patterns):
        raise AccessDenied(f"{kind} is not authorised for ROS endpoint {endpoint}")


def check_type(ros_type: str):
    if not isinstance(ros_type, str) or not ros_type or len(ros_type) > 160:
        raise InvalidRequest("ROS message type is required and must be under 161 characters")
    if not all(c.isalnum() or c in "/_" for c in ros_type):
        raise InvalidRequest("Invalid ROS message type")
