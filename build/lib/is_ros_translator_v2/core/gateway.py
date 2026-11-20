"""ROSTranslator Core: PrIS commands, routing, policy, events and validation."""
from __future__ import annotations
import asyncio
from copy import deepcopy
import json
import time
import uuid
from collections.abc import Mapping
from ..adapters.base import pose_stamped
from ..adapters.factory import create_adapter, create_transports
from ..config import Settings
from ..errors import InvalidRequest, NoRobot, ROSUnavailable, GatewayError, OperationTimeout
from ..events import EventBus
from ..models import Command, Target, Waypoints, SubscriptionSpec
from ..policy import require_allowed, check_type
from ..schema import SchemaCache, validate_ros_payload
from ..subscriptions import SubscriptionManager


def ros_type_normalized(t: str) -> str:
    return t.replace("/msg/", "/").replace("/srv/", "/")


def ensure_mapping(obj, label="payload") -> dict:
    if not isinstance(obj, dict):
        raise InvalidRequest(f"{label} must be a JSON object")
    return obj


def merge_payload(default: dict, overrides: dict) -> dict:
    merged = deepcopy(default)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_payload(merged[key], value)
        else:
            merged[key] = deepcopy(value)
    return merged


class Gateway:
    def __init__(self, settings: Settings, bridges: dict[str, object] | None = None):
        self.settings = settings
        self.events = EventBus()
        self._bridges = bridges if bridges is not None else create_transports(settings)
        self.schemas = SchemaCache(settings.schema_ttl_seconds)
        self.subscriptions = SubscriptionManager(self)
        self._task_locks = {rid: asyncio.Lock() for rid in settings.robots}

    async def start(self):
        self.events.start()
        self.subscriptions.start()

    async def stop(self):
        await self.subscriptions.stop()
        await self.events.stop()
        for bridge in self._bridges.values():
            await bridge.close()

    def robot(self, rid):
        if rid not in self.settings.robots:
            raise NoRobot(rid)
        return self.settings.robots[rid]

    def bridge(self, rid):
        self.robot(rid)
        return self._bridges[rid]

    def adapter(self, rid):
        # Adapter selection stays consistent if a robot configuration is changed.
        return create_adapter(self.robot(rid))

    @staticmethod
    def authorize(robot, kind, endpoint):
        require_allowed(robot, kind, endpoint)

    async def topic_type(self, rid: str, endpoint: str, provided: str | None = None):
        try:
            actual = await self.bridge(rid).get_topic_type(endpoint)
        except Exception as exc:
            if not self.settings.security.allow_live_schema_fallback or not provided:
                raise ROSUnavailable(f"rosapi topic discovery failed for {endpoint}: {exc}") from exc
            actual = provided
        if not actual:
            raise InvalidRequest(f"ROS topic not found: {endpoint}")
        if provided and ros_type_normalized(provided) != ros_type_normalized(actual):
            raise InvalidRequest(f"ROS type mismatch: request {provided}, advertised {actual}")
        check_type(actual)
        return actual

    async def service_type(self, rid: str, endpoint: str, provided: str | None = None):
        try:
            actual = await self.bridge(rid).get_service_type(endpoint)
        except Exception as exc:
            if not self.settings.security.allow_live_schema_fallback or not provided:
                raise ROSUnavailable(f"rosapi service discovery failed for {endpoint}: {exc}") from exc
            actual = provided
        if not actual:
            raise InvalidRequest(f"ROS service not found: {endpoint}")
        if provided and ros_type_normalized(provided) != ros_type_normalized(actual):
            raise InvalidRequest(f"ROS service type mismatch: {provided} != {actual}")
        check_type(actual)
        return actual

    async def get_schema(self, rid, kind, ros_type, allow_missing=False):
        cached = self.schemas.get(rid, kind, ros_type)
        if cached is not None:
            return cached
        bridge = self.bridge(rid)
        try:
            schema = await (bridge.service_details(ros_type) if kind == "service"
                            else bridge.action_details(ros_type) if kind == "action"
                            else bridge.message_details(ros_type))
            if not isinstance(schema, Mapping) or not schema.get("typedefs"):
                raise ValueError("rosapi returned no typedefs")
            schema = dict(schema)
        except Exception as exc:
            if not allow_missing:
                raise ROSUnavailable(f"Could not discover schema for {ros_type}: {exc}") from exc
            return None
        self.schemas.put(rid, kind, ros_type, schema)
        return schema

    async def execute(self, command: Command | dict) -> dict:
        if not isinstance(command, Command):
            command = Command.model_validate(command)
        robot = self.robot(command.robot_id)
        bridge = self.bridge(command.robot_id)
        args = command.args
        timeout = command.timeout_seconds or self.settings.default_timeout_seconds
        operation = command.operation
        req_id = command.request_id or str(uuid.uuid4())
        try:
            if operation == "navigate":
                target = Target.model_validate(args)
                async with self._task_locks[command.robot_id]:
                    result = await self._navigate(command.robot_id, target, timeout, req_id)
            elif operation == "follow_waypoints":
                steps = Waypoints.model_validate(args).waypoints
                async with self._task_locks[command.robot_id]:
                    start = time.monotonic()
                    completed = []
                    for index, target in enumerate(steps):
                        remaining = timeout - (time.monotonic() - start)
                        if remaining <= 0:
                            raise OperationTimeout(f"Waypoint sequence timed out after {len(completed)} waypoints")
                        res = await self._navigate(command.robot_id, target, remaining, req_id)
                        completed.append({"index": index, "result": res})
                    result = {"completed_waypoints": len(completed), "outcomes": completed}
            elif operation in ("ros.subscribe", "ros.unsubscribe"):
                spec = SubscriptionSpec.model_validate({"robot_id": command.robot_id, **args})
                result = (await self.subscriptions.add(spec) if operation == "ros.subscribe"
                          else await self.subscriptions.remove(spec))
            elif operation == "get_map":
                if not robot.map_service:
                    raise InvalidRequest(f"Map service is not configured for {command.robot_id}")
                ros_type = self.adapter(command.robot_id).map_service_type
                result = await bridge.service(robot.map_service, ros_type, {}, timeout)
            elif operation == "get_odometry":
                result = await self._one_odometry(command.robot_id, timeout)
            elif operation == "ros.topics":
                result = {"topics": await bridge.get_topics()}
            elif operation == "ros.services":
                result = {"services": await bridge.get_services()}
            elif operation == "ros.schema":
                kind = args.get("kind", "topic")
                endpoint = args.get("endpoint", "")
                if kind not in ("topic", "service", "action"):
                    raise InvalidRequest("Schema discovery supports topic, service or action")
                self.authorize(robot, {"topic": "subscribe", "service": "services", "action": "actions"}[kind], endpoint)
                if kind == "topic":
                    ros_type = await self.topic_type(command.robot_id, endpoint, args.get("ros_type"))
                elif kind == "service":
                    ros_type = await self.service_type(command.robot_id, endpoint, args.get("ros_type"))
                else:
                    ros_type = args.get("ros_type")
                    check_type(ros_type)
                    await self.adapter(command.robot_id).validate_action_type(bridge, endpoint, ros_type)
                result = {"kind": kind, "endpoint": endpoint, "ros_type": ros_type,
                          "schema": await self.get_schema(command.robot_id, kind, ros_type)}
            elif operation in ("ros.publish", "ros.service", "ros.action"):
                endpoint = args.get("endpoint", "")
                kind = {"ros.publish": "publish", "ros.service": "services", "ros.action": "actions"}[operation]
                self.authorize(robot, kind, endpoint)
                payload = ensure_mapping(args.get("payload", {}))
                provided = args.get("ros_type")
                if operation == "ros.action":
                    check_type(provided)
                    await self.adapter(command.robot_id).validate_action_type(bridge, endpoint, provided)
                    schema = await self.get_schema(command.robot_id, "action", provided,
                                                   allow_missing=self.settings.security.allow_live_schema_fallback)
                    if schema is not None:
                        validate_ros_payload(payload, schema, provided)
                    result = await bridge.action(endpoint, provided, payload, timeout,
                        feedback=lambda val: self.events.emit({"kind": "action_feedback", "robot_id": command.robot_id,
                            "endpoint": endpoint, "request_id": req_id, "payload": val}))
                else:
                    ros_type = (await self.topic_type(command.robot_id, endpoint, provided) if operation == "ros.publish"
                                else await self.service_type(command.robot_id, endpoint, provided))
                    schema = await self.get_schema(command.robot_id,
                                                   "message" if operation == "ros.publish" else "service", ros_type,
                                                   allow_missing=self.settings.security.allow_live_schema_fallback)
                    if schema is not None:
                        validate_ros_payload(payload, schema, ros_type)
                    elif not self.settings.security.allow_live_schema_fallback:
                        raise InvalidRequest("ROS message schema unavailable")
                    result = (await bridge.publish(endpoint, ros_type, payload) if operation == "ros.publish"
                              else await bridge.service(endpoint, ros_type, payload, timeout))
            elif operation in robot.capabilities:
                capability = robot.capabilities[operation]
                if args and not capability.allow_overrides:
                    raise InvalidRequest(f"Capability {operation} does not accept request arguments")
                payload = merge_payload(capability.default_payload, args)
                if capability.operation in ("ros.subscribe", "ros.unsubscribe"):
                    translated = {**payload, "endpoint": capability.endpoint}
                    if capability.ros_type:
                        translated["message_type"] = capability.ros_type
                else:
                    translated = {"endpoint": capability.endpoint, "payload": payload}
                    if capability.ros_type:
                        translated["ros_type"] = capability.ros_type
                response = await self.execute(Command(
                    robot_id=command.robot_id,
                    operation=capability.operation,
                    args=translated,
                    timeout_seconds=timeout,
                    request_id=req_id,
                ))
                result = response["result"]
            else:
                raise InvalidRequest(f"Operation unsupported: {operation}")
            if len(json.dumps(result, ensure_ascii=False, default=str).encode()) > self.settings.security.max_response_bytes:
                raise InvalidRequest("Response larger than max_response_bytes; reduce map size or use ROS-native map transport")
            return {"ok": True, "request_id": req_id, "robot_id": command.robot_id,
                    "operation": operation, "result": result}
        except (GatewayError, ValueError):
            raise
        except Exception as exc:
            raise ROSUnavailable(f"ROS operation {operation} failed: {exc}") from exc

    async def _navigate(self, rid: str, target: Target, timeout: float, request_id: str):
        robot = self.robot(rid)
        if not robot.navigation.enabled:
            raise InvalidRequest(f"Navigation is disabled for {rid}")
        goal = self.adapter(rid).navigation_goal(target)
        return await self.bridge(rid).action(robot.navigation.action, robot.navigation.action_type, goal, timeout,
            feedback=lambda val: self.events.emit({"kind": "navigation_feedback", "robot_id": rid,
                                                   "endpoint": robot.navigation.action, "request_id": request_id,
                                                   "payload": val}))

    async def _one_odometry(self, rid: str, timeout: float):
        robot = self.robot(rid)
        bridge = self.bridge(rid)
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        def callback(value):
            def deliver():
                if not future.done():
                    future.set_result(value)
            loop.call_soon_threadsafe(deliver)
        handle = await bridge.subscribe(robot.odom_topic, self.adapter(rid).odometry_type, callback)
        try:
            data = await asyncio.wait_for(future, timeout=timeout)
            return {"pose": data.get("pose", {}), "twist": data.get("twist", {}),
                    "header": data.get("header", {})}
        except asyncio.TimeoutError as exc:
            raise OperationTimeout(f"No odometry received on {robot.odom_topic} within {timeout}s") from exc
        finally:
            await bridge.unsubscribe(handle)
