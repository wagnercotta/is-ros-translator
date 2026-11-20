"""Configuration is explicit and typed; no ROS imports required."""
from __future__ import annotations
import os
from fnmatch import fnmatchcase
from pathlib import Path
import re
from typing import Any, Literal
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
import yaml
from .models import BUILTIN_OPERATIONS


class Allowed(BaseModel):
    publish: list[str] = Field(default_factory=list)
    subscribe: list[str] = Field(default_factory=list)
    services: list[str] = Field(default_factory=list)
    actions: list[str] = Field(default_factory=list)


class NavConfig(BaseModel):
    action: str = "/navigate_to_pose"
    action_type: str = "nav2_msgs/action/NavigateToPose"
    frame_id: str = "map"
    enabled: bool = True


class CapabilityConfig(BaseModel):
    model_config = ConfigDict(extra="forbid")

    operation: Literal["ros.publish", "ros.service", "ros.action",
                       "ros.subscribe", "ros.unsubscribe"]
    endpoint: str = Field(min_length=2, max_length=240, pattern=r"^/[a-zA-Z0-9_/]+$")
    ros_type: str | None = Field(default=None, min_length=1, max_length=160,
                                 pattern=r"^[a-zA-Z0-9_/]+$")
    default_payload: dict[str, Any] = Field(default_factory=dict)
    allow_overrides: bool = True
    description: str = ""

    @model_validator(mode="after")
    def require_action_type(self):
        if self.operation == "ros.action" and not self.ros_type:
            raise ValueError("ros_type is mandatory for action capabilities")
        return self


class BridgeInstanceConfig(BaseModel):
    """Remote rosbridge endpoint; ROS distribution/domain are deployment metadata."""

    model_config = ConfigDict(extra="forbid")

    ros_version: Literal[1, 2]
    distribution: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9_]*$")
    host: str = Field(min_length=1)
    port: int = Field(default=9090, ge=1, le=65535)
    domain_id: int | None = Field(default=None, ge=0, le=232)
    rmw_implementation: str | None = None

    @model_validator(mode="after")
    def validate_domain(self):
        if self.ros_version == 1 and (self.domain_id is not None or self.rmw_implementation):
            raise ValueError("ROS 1 bridge cannot specify a ROS 2 domain or RMW")
        return self


class RobotConfig(BaseModel):
    ros_version: int
    host: str | None = None
    port: int = Field(default=9090, ge=1, le=65535)
    bridge_id: str | None = None
    navigation: NavConfig = Field(default_factory=NavConfig)
    map_service: str | None = "/map_server/map"
    odom_topic: str = "/odom"
    allowed: Allowed = Field(default_factory=Allowed)
    capabilities: dict[str, CapabilityConfig] = Field(default_factory=dict)

    @field_validator("ros_version")
    @classmethod
    def version_supported(cls, value: int) -> int:
        if value not in (1, 2):
            raise ValueError("ros_version must be 1 or 2")
        return value


class HTTPConfig(BaseModel):
    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = Field(default=8080, ge=1, le=65535)


class ISWireConfig(BaseModel):
    enabled: bool = False
    url_env: str = "ISWIRE_BROKER_URI"
    consumer_name: str = "is-ros-translator-v2"
    robot_task_topic: str = "ROSTranslator.v2.RobotTask"
    pose_event_prefix: str = "ROSTranslator.v2.Pose"
    request_topic: str = "ROSTranslator.v2.Request"
    event_prefix: str = "ROSTranslator.v2.Event"
    legacy_robot_ids: dict[str, str] = Field(default_factory=dict)


class SecurityConfig(BaseModel):
    require_api_key: bool = True
    api_key_env: str = "GATEWAY_API_KEY"
    max_request_bytes: int = Field(default=1048576, ge=1024)
    max_response_bytes: int = Field(default=4194304, ge=1024)
    allow_live_schema_fallback: bool = False


class Settings(BaseModel):
    model_config = ConfigDict(extra="forbid")
    http: HTTPConfig = Field(default_factory=HTTPConfig)
    iswire: ISWireConfig = Field(default_factory=ISWireConfig)
    security: SecurityConfig = Field(default_factory=SecurityConfig)
    schema_ttl_seconds: int = Field(default=300, ge=1)
    subscription_lease_seconds: int = Field(default=300, ge=10, le=86400)
    max_subscriptions: int = Field(default=100, ge=1)
    default_timeout_seconds: float = Field(default=10, gt=0, le=300)
    bridges: dict[str, BridgeInstanceConfig] = Field(default_factory=dict)
    robots: dict[str, RobotConfig]

    @model_validator(mode="after")
    def validate_presence(self):
        if not self.robots:
            raise ValueError("At least one robot must be configured")
        for robot_id, robot in self.robots.items():
            if robot.bridge_id:
                if robot.bridge_id not in self.bridges:
                    raise ValueError(f"{robot_id}: unknown bridge_id {robot.bridge_id!r}")
                instance = self.bridges[robot.bridge_id]
                if instance.ros_version != robot.ros_version:
                    raise ValueError(f"{robot_id}: ROS version differs from bridge {robot.bridge_id!r}")
                if robot.host is not None:
                    raise ValueError(f"{robot_id}: choose either bridge_id or a direct host")
            elif not robot.host:
                raise ValueError(f"{robot_id}: host or bridge_id is required")
            for name, capability in robot.capabilities.items():
                if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
                    raise ValueError(f"{robot_id}: invalid capability name {name!r}")
                if name in BUILTIN_OPERATIONS:
                    raise ValueError(f"{robot_id}: capability {name!r} conflicts with a built-in operation")
                kind = {"ros.publish": "publish", "ros.service": "services",
                        "ros.action": "actions", "ros.subscribe": "subscribe",
                        "ros.unsubscribe": "subscribe"}[capability.operation]
                if not any(fnmatchcase(capability.endpoint, pattern)
                           for pattern in getattr(robot.allowed, kind)):
                    raise ValueError(f"{robot_id}: capability {name!r} must be permitted in allowed.{kind}")
        return self

    def resolved_robot(self, robot_id: str) -> RobotConfig:
        """Resolve reusable bridge endpoint without mutating the robot catalogue."""
        robot = self.robots[robot_id]
        if not robot.bridge_id:
            return robot
        bridge = self.bridges[robot.bridge_id]
        return robot.model_copy(update={"host": bridge.host, "port": bridge.port})

    def api_key(self) -> str:
        key = os.environ.get(self.security.api_key_env, "")
        if self.security.require_api_key and not key:
            raise RuntimeError(f"Set {self.security.api_key_env} to a strong API key before starting")
        return key


def load_settings(path: str | Path) -> Settings:
    with open(path, encoding="utf-8") as stream:
        return Settings.model_validate(yaml.safe_load(stream))
