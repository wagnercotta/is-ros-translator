"""Version-neutral PrIS application contracts."""
from typing import Any
from pydantic import BaseModel, Field, field_validator


BUILTIN_OPERATIONS = frozenset({
    "navigate", "follow_waypoints", "get_map", "get_odometry",
    "ros.publish", "ros.service", "ros.action", "ros.schema",
    "ros.topics", "ros.services", "ros.subscribe", "ros.unsubscribe",
})


class Command(BaseModel):
    robot_id: str = Field(min_length=1, max_length=80)
    operation: str = Field(min_length=1, max_length=80, pattern=r"^[a-z][a-z0-9_.]*$")
    args: dict[str, Any] = Field(default_factory=dict)
    timeout_seconds: float | None = Field(default=None, gt=0, le=300)
    request_id: str | None = Field(default=None, max_length=128)


class SubscriptionSpec(BaseModel):
    robot_id: str = Field(min_length=1)
    endpoint: str = Field(min_length=1)
    subscriber_id: str = Field(min_length=1, max_length=128)
    message_type: str | None = None
    lease_seconds: int | None = Field(default=None, ge=10, le=86400)


class Position(BaseModel):
    x: float
    y: float
    z: float = 0.0


class Target(BaseModel):
    position: Position
    yaw: float = 0.0
    frame_id: str | None = None


class Waypoints(BaseModel):
    waypoints: list[Target] = Field(min_length=1, max_length=100)

    @field_validator("waypoints")
    @classmethod
    def only_finite(cls, v):
        return v
