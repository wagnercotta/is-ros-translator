"""Single AMQP transport for PrIS, implemented strictly with is-wire/is-msgs.

The official is-wire Channel has a blocking consumer. It belongs to one worker
thread, while all gateway / ROS calls execute on the asyncio main loop.
Connection loss does not terminate the FastAPI process: each new Channel is
resubscribed with bounded backoff. The SDK is responsible for the AMQP driver.

RPC topics:
    ROSTranslator.v2.Request                google.protobuf.Struct -> Struct
    ROSTranslator.v2.RobotTask.<robot_id>   is_msgs.RobotTaskRequest -> RobotTaskReply
    ROSTranslator.<legacy-id>.GetInfo|Translate (optional legacy contract)
Telemetry topics:
    ROSTranslator.v2.Event.<robot_id>.<kind>    google.protobuf.Struct
    ROSTranslator.v2.Pose.<robot_id>            is_msgs.common.Pose (for /odom)
"""
from __future__ import annotations

import asyncio
import concurrent.futures
import importlib.metadata
import logging
import math
import os
import platform
import queue
import socket
import threading
from typing import Any

from google.protobuf.json_format import MessageToDict, ParseDict
from google.protobuf.struct_pb2 import Struct

from .errors import GatewayError
from .evidence import evidence, evidence_scope, navigation_args
from .models import Command

LOG = logging.getLogger(__name__)


def struct_message(payload: dict) -> Struct:
    value = Struct()
    ParseDict(payload, value)
    return value


def wire_rpc(uri: str, topic: str, payload, timeout: float = 30.0, channel_cls=None):
    """Synchronous is-wire RPC used by integration tests and external examples.

    Accepts a protobuf object (Struct or an is-msgs object). The response type
    is intentionally caller-selected; this helper only verifies correlation.
    """
    from is_wire.core import Channel, Message, Subscription

    channel = (channel_cls or Channel)(uri)
    try:
        replies = Subscription(channel)
        outgoing = Message(content=payload, reply_to=replies)
        channel.publish(outgoing, topic=topic)
        import time
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"No is-wire response from {topic} within {timeout}s")
            try:
                response = channel.consume(timeout=remaining)
            except socket.timeout as exc:
                raise TimeoutError(f"No is-wire response from {topic} within {timeout}s") from exc
            if response is not None and response.correlation_id == outgoing.correlation_id:
                return response
    finally:
        channel.close()


class ISWireTransport:
    def __init__(self, settings):
        self.settings = settings
        self.cfg = settings.iswire
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._events = queue.Queue(maxsize=512)
        self._loop = None
        self._gateway = None
        self.last_error = None

    @property
    def ready(self) -> bool:
        return self._ready.is_set()

    def health(self):
        return {"transport": "is-wire", "connected": self.ready,
                "last_error": self.last_error, "pending_events": self._events.qsize()}

    async def start(self, gateway):
        self._gateway = gateway
        uri = os.environ.get(self.cfg.url_env, "")
        if not uri:
            raise RuntimeError(f"Set {self.cfg.url_env}; is-wire is enabled")
        # Fail fast on missing libraries; network failures must be recovered.
        from is_wire.core import Channel, Subscription, Message  # noqa: F401
        from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply  # noqa: F401
        from is_msgs.common_pb2 import Pose  # noqa: F401
        self._loop = asyncio.get_running_loop()
        try:
            package_version = importlib.metadata.version("is-ros-translator-v2")
        except importlib.metadata.PackageNotFoundError:
            package_version = None
        evidence("core", "session_started", python=platform.python_version(),
                 gateway_package_version=package_version,
                 robots={rid: {"ros_version": cfg.ros_version,
                                "navigation_action": cfg.navigation.action,
                                "navigation_action_type": cfg.navigation.action_type}
                         for rid, cfg in self.settings.robots.items()})
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, args=(uri,), daemon=True,
                                        name="is-wire-rpc")
        self._thread.start()
        gateway.events.add_sink(self.send_event)

    async def send_event(self, event):
        try:
            self._events.put_nowait(event)
        except queue.Full:
            LOG.warning("Dropped is-wire ROS telemetry: event queue is full")

    def _publish_events(self, channel):
        from is_wire.core import Message
        for _ in range(16):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            rid = str(event.get("robot_id", "unknown")).replace(".", "_")
            kind = str(event.get("kind", "event")).replace(".", "_")
            event_proto = struct_message(event)
            if event_proto.ByteSize() > self.settings.security.max_response_bytes:
                LOG.warning("Dropped oversize is-wire event: %s", rid)
                continue
            try:
                channel.publish(Message(content=event_proto),
                                topic=f"{self.cfg.event_prefix}.{rid}.{kind}")
                if rid in self.settings.robots and event.get("endpoint") == self.settings.robots[rid].odom_topic:
                    pose = self._pose_from_odom(event)
                    if pose is not None:
                        channel.publish(Message(content=pose),
                                        topic=f"{self.cfg.pose_event_prefix}.{rid}")
            except Exception:
                # Keep one event available for the next connection when possible.
                try:
                    self._events.put_nowait(event)
                except queue.Full:
                    pass
                raise

    @staticmethod
    def _pose_from_odom(event):
        """Translate nav_msgs/Odometry into standard is-msgs common.Pose."""
        from is_msgs.common_pb2 import Pose
        try:
            ros_pose = event["payload"]["pose"]["pose"]
            pos, q = ros_pose["position"], ros_pose["orientation"]
            x, y, z, w = (float(q[k]) for k in ("x", "y", "z", "w"))
            out = Pose()
            out.position.x = float(pos["x"])
            out.position.y = float(pos["y"])
            out.position.z = float(pos.get("z", 0))
            out.orientation.yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
            out.orientation.pitch = math.asin(max(-1., min(1., 2 * (w*y - z*x))))
            out.orientation.roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
            return out
        except (KeyError, ValueError, TypeError, AttributeError):
            return None

    def _execute(self, command: Command):
        async def execute_traced():
            with evidence_scope(robot_id=command.robot_id,
                                request_id=command.request_id,
                                operation=command.operation):
                return await self._gateway.execute(command)

        future = asyncio.run_coroutine_threadsafe(execute_traced(), self._loop)
        try:
            timeout = (command.timeout_seconds or self.settings.default_timeout_seconds) + 10
            return future.result(timeout=timeout)
        except concurrent.futures.TimeoutError:
            future.cancel()
            return {"ok": False, "error": "timeout", "detail": "Gateway command timed out"}

    def _handle_struct_request(self, incoming):
        correlation_id = str(getattr(incoming, "correlation_id", None))
        request_id = None
        robot_id = None
        operation = None
        try:
            if len(incoming.body) > self.settings.security.max_request_bytes:
                raise ValueError("is-wire request too large")
            request = MessageToDict(incoming.unpack(Struct), preserving_proto_field_name=True)
            command = Command.model_validate(request)
            request_id, robot_id, operation = (command.request_id, command.robot_id,
                                               command.operation)
            evidence("iswire", "request_received", correlation_id=correlation_id,
                     request_id=request_id, robot_id=robot_id, operation=operation,
                     request_topic=self.cfg.request_topic,
                     navigation_args=navigation_args(command.args)
                         if operation in ("navigate", "follow_waypoints") else None)
            response = self._execute(command)
            evidence("iswire", "reply_prepared", correlation_id=correlation_id,
                     request_id=request_id, robot_id=robot_id, operation=operation,
                     ok=response.get("ok"), error=response.get("error"))
            return struct_message(response)
        except GatewayError as exc:
            evidence("iswire", "request_rejected", correlation_id=correlation_id,
                     request_id=request_id, robot_id=robot_id, operation=operation,
                     error=exc.code)
            return struct_message({"ok": False, "error": exc.code, "detail": str(exc)})
        except Exception as exc:
            LOG.warning("is-wire Struct command failed: %s", exc)
            evidence("iswire", "request_rejected", correlation_id=correlation_id,
                     request_id=request_id, robot_id=robot_id, operation=operation,
                     error=type(exc).__name__)
            return struct_message({"ok": False, "error": "invalid_request", "detail": str(exc)})

    def _handle_robot_task(self, incoming):
        """An is-msgs RobotTaskRequest is translated into Nav2/move_base actions."""
        from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply
        from is_wire.core import Status, StatusCode
        rid = incoming.topic.removeprefix(self.cfg.robot_task_topic + ".")
        if rid not in self.settings.robots:
            raise ValueError(f"Unconfigured robot {rid!r}")
        if len(incoming.body) > self.settings.security.max_request_bytes:
            raise ValueError("RobotTaskRequest too large")
        task = incoming.unpack(RobotTaskRequest)
        if task.WhichOneof("Task") != "basic_move_task":
            raise ValueError("Only is-msgs BasicMoveTask is supported")
        motion = task.basic_move_task
        if not motion.positions:
            raise ValueError("BasicMoveTask.positions must contain at least one point")
        if motion.allowed_error < 0 or motion.rate < 0:
            raise ValueError("Negative allowed_error / rate is invalid")
        yaw = float(motion.final_orientation.yaw)
        waypoints = [{"position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)}, "yaw": yaw}
                     for p in motion.positions]
        operation = "navigate" if len(waypoints) == 1 else "follow_waypoints"
        args = waypoints[0] if operation == "navigate" else {"waypoints": waypoints}
        command = Command(robot_id=rid, operation=operation, args=args,
                          timeout_seconds=self.settings.default_timeout_seconds)
        result = self._execute(command)
        if not result.get("ok"):
            code = StatusCode.INVALID_ARGUMENT if result.get("error") in ("invalid_request", "forbidden") else StatusCode.INTERNAL_ERROR
            return RobotTaskReply(id=task.id), Status(code, str(result.get("detail", result.get("error"))))
        return RobotTaskReply(id=task.id), Status(StatusCode.OK)

    def _handle_legacy(self, incoming):
        from .legacy_rpc import process_legacy
        from .legacy_proto import legacy_types
        segments = incoming.topic.split(".")
        _, robot_alias, operation = segments
        rid = self.cfg.legacy_robot_ids.get(robot_alias)
        if not rid:
            raise ValueError(f"Legacy robot ID {robot_alias} not configured")
        future = asyncio.run_coroutine_threadsafe(
            process_legacy(self._gateway, rid, operation, incoming), self._loop)
        try:
            return future.result(timeout=190)
        except concurrent.futures.TimeoutError:
            future.cancel()
            raise TimeoutError("Legacy ROS request timed out")

    def _build_reply(self, incoming):
        from is_wire.core import Status, StatusCode
        if not incoming.reply_to:
            LOG.warning("is-wire RPC request without reply_to on %s discarded", incoming.topic)
            return None
        reply = incoming.create_reply()  # preserves is-wire integer correlation_id
        topic = incoming.topic or ""
        try:
            if topic == self.cfg.request_topic:
                reply.pack(self._handle_struct_request(incoming))
                reply.status = Status(StatusCode.OK)
            elif topic.startswith(self.cfg.robot_task_topic + "."):
                payload, status = self._handle_robot_task(incoming)
                reply.pack(payload)
                reply.status = status
            elif topic.startswith("ROSTranslator."):
                reply.pack(self._handle_legacy(incoming))
                reply.status = Status(StatusCode.OK)
            else:
                raise ValueError(f"Unexpected RPC topic: {topic}")
        except Exception as exc:
            LOG.warning("is-wire request %s failed: %s", topic, exc)
            reply.status = Status(StatusCode.INVALID_ARGUMENT if isinstance(exc, ValueError)
                                  else StatusCode.INTERNAL_ERROR, str(exc))
            # Protobuf response type remains consistent for typed RobotTask RPC.
            if topic.startswith(self.cfg.robot_task_topic + "."):
                from is_msgs.robot_pb2 import RobotTaskReply
                reply.pack(RobotTaskReply())
            elif topic.startswith("ROSTranslator.") and topic not in (self.cfg.request_topic,):
                from .legacy_proto import legacy_types
                _, Reply = legacy_types()
                fallback = Reply()
                fallback.data = str(exc)
                reply.pack(fallback)
            else:
                reply.pack(struct_message({"ok": False, "error": "invalid_request", "detail": str(exc)}))
        return reply

    def _handle_incoming(self, channel, incoming):
        reply = self._build_reply(incoming)
        if reply is not None:
            channel.publish(reply)

    def _run(self, uri):
        from is_wire.core import Channel, Subscription
        backoff = 1.0
        while not self._stop.is_set():
            channel = None
            executor = None
            try:
                channel = Channel(uri)
                subscription = Subscription(channel, name=self.cfg.consumer_name)
                subscription.subscribe(topic=self.cfg.request_topic)
                subscription.subscribe(topic=self.cfg.robot_task_topic + ".*")
                if self.cfg.legacy_robot_ids:
                    subscription.subscribe(topic="ROSTranslator.*.GetInfo")
                    subscription.subscribe(topic="ROSTranslator.*.Translate")
                self.last_error = None
                self._ready.set()
                backoff = 1.0
                LOG.info("is-wire connected; request=%s; task=%s.*", self.cfg.request_topic, self.cfg.robot_task_topic)
                executor = concurrent.futures.ThreadPoolExecutor(max_workers=8,
                    thread_name_prefix="is-wire-command")
                pending = set()
                while not self._stop.is_set():
                    self._publish_events(channel)
                    for future in tuple(pending):
                        if future.done():
                            pending.remove(future)
                            reply = future.result()
                            if reply is not None:
                                channel.publish(reply)
                    if len(pending) >= 16:
                        self._stop.wait(0.05)
                        continue
                    try:
                        msg = channel.consume(timeout=0.2)
                    except socket.timeout:
                        continue
                    if msg is not None:
                        pending.add(executor.submit(self._build_reply, msg))
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                self._ready.clear()
                if not self._stop.is_set():
                    LOG.warning("is-wire broker unavailable: %s; reconnecting in %.1fs", self.last_error, backoff)
                    self._stop.wait(backoff)
                    backoff = min(backoff * 2.0, 20.0)
            finally:
                self._ready.clear()
                if executor is not None:
                    executor.shutdown(wait=False, cancel_futures=True)
                if channel is not None:
                    try:
                        channel.close()
                    except Exception:
                        LOG.debug("Failed closing previous is-wire channel", exc_info=True)

    async def stop(self):
        evidence("core", "session_stopping")
        self._stop.set()
        if self._thread is not None:
            await asyncio.to_thread(self._thread.join, 3.0)
