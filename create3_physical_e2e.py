#!/usr/bin/env python3
"""Physical Create 3 E2E client: orchestrate via IS-Wire messages only.

ROSTranslator Core must already be running and receiving IS-Wire requests.
It forwards them through its ROS 2 Adapter to the physical Create 3.
This client never starts the gateway, Docker, ROSBridge, or a ROS 2 node.

  python3 create3_physical_e2e.py --seed 2027
  python3 create3_physical_e2e.py --check-only
  python3 create3_physical_e2e.py --execute --confirm-area-clear --seed 2027
  python3 create3_physical_e2e.py --dock-only --confirm-area-clear
  python3 create3_physical_e2e.py --execute --confirm-area-clear --ros1-mirror
  python3 create3_physical_e2e.py --execute --confirm-area-clear --ros1-mirror --validate-follow-waypoints
  python3 create3_physical_e2e.py --distinct-commands-only

Robot commands use ONLY ROSTranslator.v2.Request and IS-Wire telemetry. No
rclpy, direct ROSBridge calls, or automatic Docker startup are performed.
Waypoints are expressed in the physical FLOOR frame. The charging dock is
at floor (1.5, -1.0) and coincides with odometry (0, 0). The frame axes
have the same orientation, so only a translation is needed:
odom_xy = floor_xy - dock_floor_xy.

Optional --ros1-mirror sends every navigation waypoint first to a separate
ROS 1 /move_base simulator via the SAME IS-Wire translator, then to Create 3.
ROS 1 uses world XY directly in map; Create 3 uses translated odom XY.
The ROS 1 mirror also validates distinct read-only PrIS operations before
physical movement: ROS 1 get_map (service) and ROS 2 get_odometry (topic).
Use --distinct-commands-only to run that stage without robot motion.

IMPORTANT: Create 3 NavigateToPosition is NOT an obstacle-avoiding planner.
An operator must supervise the robot and keep a physical stop available.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import random
import secrets
import socket
import subprocess
import sys
import time
import uuid


LOOPS = 5
COORD_LIMIT = 1.2
DEFAULT_DOCK_WORLD = (1.5, -1.0)
WORLD_ORIGIN = (0.0, 0.0)
DEFAULT_DOCK_APPROACH_DISTANCE = 0.20
DOCK_APPROACH_TOLERANCE = 0.075
REQUEST_TOPIC = "ROSTranslator.v2.Request"
EVENT_PREFIX = "ROSTranslator.v2.Event"
DOCK_STATUS_TYPE = "irobot_create_msgs/msg/DockStatus"


class ExperimentError(RuntimeError):
    pass


def now_utc() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def provenance() -> dict:
    """Capture runtime versions and source revision, when locally available."""
    versions = {}
    for distribution in ("is-wire", "is-msgs", "protobuf"):
        try:
            versions[distribution] = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            versions[distribution] = None
    try:
        revision = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                                  text=True, check=True, timeout=3).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], capture_output=True,
                                    text=True, check=True, timeout=3).stdout.strip())
    except (FileNotFoundError, subprocess.SubprocessError):
        revision = None
        dirty = None
    return {"python": platform.python_version(), "platform": platform.platform(),
            "packages": versions, "git_commit": revision, "git_dirty": dirty}


def random_waypoints(seed: int) -> list[tuple[float, float]]:
    """Five reproducible offsets in the experimental +/-1.2 metre square."""
    rng = random.Random(seed)
    return [(round(rng.uniform(-COORD_LIMIT, COORD_LIMIT), 3),
             round(rng.uniform(-COORD_LIMIT, COORD_LIMIT), 3)) for _ in range(LOOPS)]


def world_to_odom(world: tuple[float, float],
                  dock_world: tuple[float, float]) -> tuple[float, float]:
    """Convert floor coordinates to Create 3 odometry (aligned XY axes)."""
    return (round(world[0] - dock_world[0], 4),
            round(world[1] - dock_world[1], 4))


def odom_to_world(odom: tuple[float, float],
                  dock_world: tuple[float, float]) -> tuple[float, float]:
    """Convert Create 3 odometry to measured floor coordinates."""
    return (round(odom[0] + dock_world[0], 4),
            round(odom[1] + dock_world[1], 4))


def dock_approach_odom(distance: float) -> tuple[float, float]:
    """Approach along -X, the observed Create 3 undocking direction.

    This assumes odom (0, 0) is the dock and its entry is from negative X.
    The client remains responsible for choosing the approach pose.
    """
    return (-distance, 0.0)


def odometry_pose(reply: dict) -> tuple[float, float, float]:
    """Read position and yaw from get_odometry's ROS 2 Odometry payload."""
    try:
        pose = reply["result"]["pose"]["pose"]
        pos = pose["position"]
        orientation = pose["orientation"]
        x, y = float(pos["x"]), float(pos["y"])
        qx, qy = float(orientation.get("x", 0.0)), float(orientation.get("y", 0.0))
        qz, qw = float(orientation.get("z", 0.0)), float(orientation["w"])
        yaw = math.atan2(2 * (qw * qz + qx * qy),
                         1 - 2 * (qy * qy + qz * qz))
    except (TypeError, KeyError, ValueError) as exc:
        raise ExperimentError("get_odometry returned an invalid pose") from exc
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ExperimentError("get_odometry returned a non-finite pose")
    return x, y, yaw


def validate_action_result(reply: dict, label: str, *, expected_status: int = 4) -> None:
    """Do not treat an accepted goal or a missing terminal code as success."""
    result = reply.get("result")
    if not isinstance(result, dict):
        raise ExperimentError(f"{label}: missing action result")
    try:
        code = int(result["status"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ExperimentError(f"{label}: missing/invalid terminal action status") from exc
    if code != expected_status or float(result["status"]) != expected_status:
        raise ExperimentError(f"{label}: action finished with status {result['status']}; "
                              f"expected {expected_status}")
    nested = result.get("result")
    if isinstance(nested, dict) and nested.get("result") is False:
        raise ExperimentError(f"{label}: ROS 2 action reported failure")


def validate_waypoint_results(reply: dict, label: str, *, expected_status: int) -> None:
    """Require two individually successful, ordered goals in ONE PrIS response."""
    result = reply.get("result")
    if not isinstance(result, dict) or result.get("completed_waypoints") != 2:
        raise ExperimentError(f"{label}: expected completed_waypoints=2, received {result}")
    outcomes = result.get("outcomes")
    if not isinstance(outcomes, list) or len(outcomes) != 2:
        raise ExperimentError(f"{label}: expected two individual terminal outcomes")
    for index, outcome in enumerate(outcomes):
        if not isinstance(outcome, dict) or outcome.get("index") != index:
            raise ExperimentError(f"{label}: goal outcomes not ordered 0,1")
        validate_action_result({"result": outcome.get("result")},
                               f"{label} goal {index + 1}", expected_status=expected_status)


def find_goal_ids(value: object) -> list[str]:
    """Collect only goal IDs actually supplied by the backend, never fabricate them."""
    found = []
    if isinstance(value, dict):
        for key, item in value.items():
            if key in ("goal_id", "goal_uuid") and item is not None:
                found.append(str(item))
            else:
                found.extend(find_goal_ids(item))
    elif isinstance(value, list):
        for item in value:
            found.extend(find_goal_ids(item))
    return found


def event_dock_state(event: dict, robot_id: str, endpoint: str,
                     subscriber_id: str) -> dict | None:
    if (event.get("kind") != "topic" or event.get("robot_id") != robot_id
            or event.get("endpoint") != endpoint
            or subscriber_id not in event.get("subscribers", [])):
        return None
    payload = event.get("payload")
    if not isinstance(payload, dict) or type(payload.get("is_docked")) is not bool:
        return None
    return {"is_docked": payload["is_docked"],
            "dock_visible": payload.get("dock_visible"),
            "observed_at_utc": now_utc(), "source": "IS-Wire ROS telemetry"}


class ISWireClient:
    """Blocking IS-Wire client; no direct ROS, rosbridge, or HTTP calls."""

    def __init__(self, uri: str, robot_id: str, dock_topic: str,
                 request_topic: str = REQUEST_TOPIC, evidence: Evidence | None = None):
        self.uri = uri
        self.robot_id = robot_id
        self.dock_topic = dock_topic
        self.request_topic = request_topic
        self.evidence = evidence

    def request(self, operation: str, args: dict | None = None,
                *, timeout: float = 30) -> dict:
        from google.protobuf.json_format import MessageToDict, ParseDict
        from google.protobuf.struct_pb2 import Struct
        from is_wire.core import Channel, Message, Subscription

        if not 0 < timeout <= 300:
            raise ValueError("IS-Wire request timeout must be between 0 and 300 seconds")
        channel = Channel(self.uri)
        body = None
        outgoing = None
        reply = None
        error = None
        started = time.monotonic_ns()
        try:
            response_sub = Subscription(channel)
            request_id = str(uuid.uuid4())
            body = {"robot_id": self.robot_id, "operation": operation,
                    "args": args or {}, "request_id": request_id,
                    "timeout_seconds": timeout}
            proto = Struct()
            ParseDict(body, proto)
            outgoing = Message(content=proto, reply_to=response_sub)
            channel.publish(outgoing, topic=self.request_topic)
            deadline = time.monotonic() + timeout + 12
            while time.monotonic() < deadline:
                try:
                    incoming = channel.consume(timeout=min(3.0, max(0.1, deadline - time.monotonic())))
                except socket.timeout:
                    continue
                if incoming is None or incoming.correlation_id != outgoing.correlation_id:
                    continue
                if incoming.status is not None and not incoming.status.ok():
                    raise ExperimentError(f"IS-Wire rejected {operation}: {incoming.status}")
                reply = MessageToDict(incoming.unpack(Struct), preserving_proto_field_name=True)
                if (not reply.get("ok") or reply.get("request_id") != request_id
                        or reply.get("robot_id") != self.robot_id):
                    raise ExperimentError(f"PrIS {operation} failed: "
                                          f"{reply.get('error', 'invalid reply')} "
                                          f"{reply.get('detail', '')}")
                return reply
            raise ExperimentError(f"No correlated IS-Wire reply for {operation} "
                                  f"within {timeout + 12:.0f}s")
        except BaseException as exc:
            error = f"{type(exc).__name__}: {exc}"
            raise
        finally:
            if self.evidence is not None and body is not None:
                self.evidence.record_rpc(
                    request_topic=self.request_topic,
                    request=body,
                    correlation_id=str(outgoing.correlation_id) if outgoing is not None else None,
                    reply=reply,
                    duration_ms=round((time.monotonic_ns() - started) / 1_000_000, 3),
                    error=error,
                )
            channel.close()

    def dock_status(self, *, expected: bool | None = None, timeout: float = 30) -> dict:
        """Subscribe to the ROS dock topic via IS-Wire; consume its IS-Wire event.

        The telemetry subscription starts before the ROS subscription RPC,
        avoiding a race with the first dock status publication.
        """
        from google.protobuf.json_format import MessageToDict
        from google.protobuf.struct_pb2 import Struct
        from is_wire.core import Channel, Subscription

        subscriber_id = "create3-e2e-" + uuid.uuid4().hex
        channel = Channel(self.uri)
        subscribed = False
        latest = None
        try:
            events = Subscription(channel)
            events.subscribe(topic=f"{EVENT_PREFIX}.{self.robot_id}.topic")
            response = self.request("ros.subscribe", {
                "endpoint": self.dock_topic,
                "message_type": DOCK_STATUS_TYPE,
                "subscriber_id": subscriber_id,
                "lease_seconds": max(60, math.ceil(timeout + 30)),
            }, timeout=20)
            subscribed = bool(response.get("result", {}).get("subscribed"))
            if not subscribed:
                raise ExperimentError("IS-Wire did not establish the dock status subscription")
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                try:
                    message = channel.consume(timeout=min(2.0, max(0.1, deadline - time.monotonic())))
                except socket.timeout:
                    continue
                if message is None:
                    continue
                try:
                    event = MessageToDict(message.unpack(Struct), preserving_proto_field_name=True)
                except Exception:
                    continue
                state = event_dock_state(event, self.robot_id, self.dock_topic, subscriber_id)
                if state is not None:
                    latest = state
                    if expected is None or state["is_docked"] is expected:
                        return state
            raise ExperimentError(f"No fresh dock status confirming {expected!r}; "
                                  f"last received={latest}")
        finally:
            if subscribed:
                try:
                    self.request("ros.unsubscribe", {"endpoint": self.dock_topic,
                        "subscriber_id": subscriber_id}, timeout=12)
                except Exception as exc:
                    print(f"WARNING: could not release dock subscription: {exc}", file=sys.stderr)
            channel.close()


class Evidence:
    def __init__(self, output: Path, *, verbose: bool = False):
        self.output = output
        self.output.mkdir(parents=True, exist_ok=False)
        self.log = self.output / "events.jsonl"
        self.wire_log = self.output / "wire_requests.jsonl"
        self.motion_attempted = False
        self.verbose = verbose
        self.requests = []
        self.navigations = {"ros1": 0, "ros2": 0}
        self.waypoint_sequences = {"ros1": 0, "ros2": 0}
        self.distinct_commands_verified = False

    def record_rpc(self, *, request_topic: str, request: dict,
                   correlation_id: str | None, reply: dict | None,
                   duration_ms: float, error: str | None) -> None:
        record = {"at_utc": now_utc(), "transport": "IS-Wire",
                  "request_topic": request_topic, "correlation_id": correlation_id,
                  "request": request, "reply": reply, "duration_ms": duration_ms,
                  "goal_ids_returned": find_goal_ids(reply),
                  "status": "failed" if error else "passed", "error": error}
        with self.wire_log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
        self.requests.append({"robot_id": request["robot_id"],
                              "operation": request["operation"],
                              "correlated": bool(reply is not None and
                                                 reply.get("request_id") == request["request_id"] and
                                                 correlation_id is not None),
                              "goal_ids_returned": record["goal_ids_returned"],
                              "status": record["status"]})

    def emit(self, kind: str, **values) -> None:
        event = {"at_utc": now_utc(), "kind": kind, **values}
        with self.log.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event, ensure_ascii=False, default=str) + "\n")
        if self.verbose:
            print(f"[{event['at_utc']}] {kind}: "
                  + ", ".join(f"{k}={v}" for k, v in values.items() if k != "reply"), flush=True)
        else:
            compact = {
                "started": f"[START] {values.get('mode')} | seed={values.get('seed')}",
                "undocked_confirmed": "[PASS] Create3 undocked",
                "undock_skipped": "[INFO] Create3 already undocked",
                "loop_pass": f"[PASS] Loop {values.get('index')}/{LOOPS}",
                "follow_waypoints_verified":
                    f"[PASS] E2 follow_waypoints | {values.get('robot_id')} | 2/2 goals",
                "distinct_commands_verified":
                    "[PASS] E3 distinct operations | ROS1 get_map (service) "
                    "| ROS2 get_odometry (topic)",
                "dock_approach_verified": "[PASS] Dock approach verified",
                "docked_confirmed": "[PASS] Create3 docked (telemetry confirmed)",
                "docking_warning": f"[WARN] {values.get('detail')}",
                "experiment_pass":
                    "[RESULT] E3 distinct-commands validation PASSED"
                    if values.get("mode") == "distinct-commands-only"
                    else "[RESULT] Physical experiment PASSED",
                "experiment_failed": f"[FAIL] {values.get('error')}",
                "emergency_stop_requested": "[WARN] Emergency stop requested",
                "emergency_stop_unavailable": "[WARN] Emergency stop unavailable",
                "dock_status": f"[INFO] Docked: {values.get('is_docked')}",
            }.get(kind)
            if compact:
                print(compact, flush=True)

    def validation_results(self, *, ros1_mirror: bool,
                           follow_requested: bool,
                           distinct_requested: bool = False) -> dict:
        """Only mark what this client can actually observe; avoid inferred PASS."""
        nav = [r for r in self.requests if r["operation"] == "navigate"]
        seq = [r for r in self.requests if r["operation"] == "follow_waypoints"]
        distinct = [r for r in self.requests if r["operation"] in
                    ("get_map", "get_odometry")]
        goal_relevant = nav + seq
        relevant = goal_relevant + distinct
        correlation = bool(relevant) and all(r["correlated"] and r["status"] == "passed"
                                                  for r in relevant)
        goal_ids = bool(goal_relevant) and all(r["goal_ids_returned"] for r in goal_relevant)
        return {
            "E1_navigate_ros1": "PASS" if self.navigations["ros1"] else
                                ("NOT_EXECUTED" if not ros1_mirror or not nav else "NOT_VERIFIED"),
            "E1_navigate_ros2": "PASS" if self.navigations["ros2"] else "NOT_EXECUTED",
            "E2_follow_waypoints_ros1": "PASS" if self.waypoint_sequences["ros1"] else
                                        ("FAIL" if follow_requested and seq else "NOT_EXECUTED"),
            "E2_follow_waypoints_ros2": "PASS" if self.waypoint_sequences["ros2"] else
                                        ("FAIL" if follow_requested and len(seq) >= 2 else
                                         "NOT_EXECUTED"),
            "E3_distinct_commands": "PASS" if self.distinct_commands_verified else
                                     ("FAIL" if distinct_requested else "NOT_EXECUTED"),
            "IS_Wire_request_reply_correlation": "PASS" if correlation else "NOT_VERIFIED",
            "backend_goal_ids": "PASS" if goal_ids else "NOT_AVAILABLE_IN_REPLY",
            "independent_adapter_routing": "NOT_VERIFIED_FROM_CLIENT_ALONE",
        }

    def finish(self, summary: dict) -> None:
        (self.output / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, default=str) + "\n", encoding="utf-8")


def check_odometry(client: ISWireClient, target: tuple[float, float],
                   tolerance: float, *, timeout: float = 8) -> dict:
    deadline = time.monotonic() + timeout
    latest = None
    while True:
        reply = client.request("get_odometry", timeout=10)
        x, y, yaw = odometry_pose(reply)
        error = math.hypot(x - target[0], y - target[1])
        latest = {"x": x, "y": y, "yaw": yaw, "error_m": round(error, 4)}
        if error <= tolerance:
            return latest
        if time.monotonic() >= deadline:
            raise ExperimentError(f"Robot is {error:.3f}m from requested odom target "
                                  f"{target}; tolerance={tolerance:.3f}m; last pose={latest}")
        time.sleep(0.4)


def run_distinct_commands(client: ISWireClient, ros1_client: ISWireClient,
                          evidence: Evidence) -> dict:
    """E3: exercise different ROS service/topic mechanisms through one PrIS API.

    Both commands are read-only. Neither drives, docks, or undocks the robot.
    The raw correlated RPC replies are preserved by ISWireClient; only compact
    response metadata is emitted to the human-readable experiment event log.
    """
    if ros1_client is None or client.robot_id == ros1_client.robot_id:
        raise ExperimentError("E3 requires distinct configured ROS1 and ROS2 robot IDs")
    evidence.emit("distinct_commands_started", operations=[
        {"robot_id": ros1_client.robot_id, "operation": "get_map", "endpoint": "/dynamic_map"},
        {"robot_id": client.robot_id, "operation": "get_odometry", "endpoint": "/create3/odom"},
    ])

    map_reply = ros1_client.request("get_map", timeout=30)
    try:
        grid = map_reply["result"]["map"]
        info = grid["info"]
        width = int(info["width"])
        height = int(info["height"])
        resolution = float(info["resolution"])
        cells = grid["data"]
        if (float(info["width"]) != width or float(info["height"]) != height
                or width <= 0 or height <= 0 or width * height != len(cells)
                or not math.isfinite(resolution) or resolution <= 0):
            raise ValueError("invalid map dimensions or resolution")
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ExperimentError("E3 ROS1 get_map returned an invalid OccupancyGrid") from exc
    evidence.emit("distinct_command_response", robot_id=ros1_client.robot_id,
                  operation="get_map", endpoint="/dynamic_map",
                  ros_type="nav_msgs/GetMap", request_id=map_reply["request_id"],
                  width=width, height=height, resolution=resolution, cells=len(cells))

    odom_reply = client.request("get_odometry", timeout=20)
    # Validate the message structure, not its coordinates or positioning error.
    odometry_pose(odom_reply)
    if map_reply["request_id"] == odom_reply["request_id"]:
        raise ExperimentError("E3 expected distinct request IDs for independent commands")
    evidence.emit("distinct_command_response", robot_id=client.robot_id,
                  operation="get_odometry", endpoint="/create3/odom",
                  ros_type="nav_msgs/msg/Odometry", request_id=odom_reply["request_id"],
                  pose_received=True)

    evidence.distinct_commands_verified = True
    evidence.emit("distinct_commands_verified", ros1_operation="get_map",
                  ros2_operation="get_odometry", ros1_robot_id=ros1_client.robot_id,
                  ros2_robot_id=client.robot_id,
                  ros1_request_id=map_reply["request_id"],
                  ros2_request_id=odom_reply["request_id"])
    return {"distinct_commands": {
        "ros1": {"robot_id": ros1_client.robot_id, "operation": "get_map",
                 "request_id": map_reply["request_id"], "map_width": width,
                 "map_height": height, "map_cells": len(cells)},
        "ros2": {"robot_id": client.robot_id, "operation": "get_odometry",
                 "request_id": odom_reply["request_id"], "pose_received": True},
    }}


def send_action(client: ISWireClient, evidence: Evidence, action: str,
                args: dict | None, timeout: float, *,
                expected_status: int = 4, physical_motion: bool = True) -> dict:
    evidence.emit("action_start", operation=action, robot_id=client.robot_id, args=args or {})
    if physical_motion:
        evidence.motion_attempted = True
    start = time.monotonic_ns()
    response = client.request(action, args or {}, timeout=timeout)
    if action == "follow_waypoints":
        validate_waypoint_results(response, action, expected_status=expected_status)
    else:
        validate_action_result(response, action, expected_status=expected_status)
    elapsed_ms = round((time.monotonic_ns() - start) / 1_000_000, 3)
    evidence.emit("action_pass", operation=action, robot_id=client.robot_id,
                  duration_ms=elapsed_ms, reply=response)
    return response


def move(client: ISWireClient, evidence: Evidence, *, label: str,
         target_world: tuple[float, float], dock_world: tuple[float, float],
         yaw: float, timeout: float,
         tolerance: float, ros1_client: ISWireClient | None = None) -> dict:
    target = world_to_odom(target_world, dock_world)
    args = {"position": {"x": target[0], "y": target[1], "z": 0.0},
            "yaw": yaw, "frame_id": "odom"}
    evidence.emit("waypoint", label=label,
                  target_world=target_world, target_odom=target)
    if ros1_client is not None:
        # The ROS 1 robot's map is the physical floor frame. Mirror the same
        # floor point, not the Create 3-specific translated odom coordinates.
        ros1_args = {"position": {"x": target_world[0], "y": target_world[1], "z": 0.0},
                     "yaw": yaw, "frame_id": "map"}
        evidence.emit("ros1_mirror_waypoint", label=label,
                      robot_id=ros1_client.robot_id,
                      target_world=target_world, target_map=target_world)
        send_action(ros1_client, evidence, "navigate", ros1_args, timeout,
                    expected_status=3, physical_motion=False)
        ros1_measured = check_odometry(ros1_client, target_world, tolerance)
        evidence.emit("ros1_mirror_verified", label=label,
                      target_map=target_world, measured_map=ros1_measured)
        evidence.navigations["ros1"] += 1
    send_action(client, evidence, "navigate", args, timeout)
    measured = check_odometry(client, target, tolerance)
    evidence.emit("odometry_verified", label=label,
                  target_world=target_world, target_odom=target,
                  measured_world=odom_to_world((measured["x"], measured["y"]), dock_world),
                  measured=measured)
    evidence.navigations["ros2"] += 1
    return measured


def follow_two_waypoints(client: ISWireClient, evidence: Evidence, *,
                         robot_version: str, goals_world: list[tuple[float, float]],
                         dock_world: tuple[float, float], yaw: float,
                         timeout: float, tolerance: float) -> dict:
    """One correlated PrIS call, two ordered ROS goals, same route as navigate."""
    if len(goals_world) != 2:
        raise ExperimentError("E2 requires exactly two goals")
    ros1 = robot_version == "ros1"
    frame = "map" if ros1 else "odom"
    positions = goals_world if ros1 else [world_to_odom(p, dock_world) for p in goals_world]
    args = {"waypoints": [
        {"position": {"x": p[0], "y": p[1], "z": 0.0},
         "yaw": yaw, "frame_id": frame} for p in positions]}
    # Execution is deliberately sequential, and the Core owns both steps.
    response = send_action(client, evidence, "follow_waypoints", args, timeout,
                           expected_status=3 if ros1 else 4,
                           physical_motion=not ros1)
    validate_waypoint_results(response, f"{robot_version} E2",
                              expected_status=3 if ros1 else 4)
    measured = check_odometry(client, positions[-1], tolerance)
    evidence.emit("follow_waypoints_verified", robot_id=client.robot_id,
                  ros_version=robot_version, goal_world=goals_world,
                  goal_backend=positions, final_pose=measured,
                  completed_waypoints=2, outcomes=response["result"]["outcomes"])
    evidence.waypoint_sequences[robot_version] += 1
    return measured


def run_mission(client: ISWireClient, evidence: Evidence, *, waypoints: list[tuple[float, float]],
                dock_world: tuple[float, float], action_timeout: float, status_timeout: float,
                tolerance: float, dock_approach_distance: float,
                ros1_client: ISWireClient | None = None,
                validate_follow_waypoints: bool = False) -> dict:
    # The dock approach is a separate final operation, never a sixth random
    # waypoint; every loop also returns to the world origin by design.
    if len(waypoints) != LOOPS:
        raise ExperimentError(f"Exactly {LOOPS} random waypoints are required; "
                              "the charging-dock approach must not be appended to them")
    dock = client.dock_status(timeout=status_timeout)
    evidence.emit("initial_dock_status", **dock)
    if dock["is_docked"]:
        # This experiment measures communication and action completion, not
        # absolute odometry calibration at the charging station.
        send_action(client, evidence, "undock", {}, action_timeout)
        undocked = client.dock_status(expected=False, timeout=status_timeout)
        evidence.emit("undocked_confirmed", **undocked)
    else:
        evidence.emit("undock_skipped", reason="robot already undocked")

    pose_reply = client.request("get_odometry", timeout=15)
    x, y, yaw = odometry_pose(pose_reply)
    return_odom = world_to_odom(WORLD_ORIGIN, dock_world)
    evidence.emit("coordinate_frames",
                  odom_pose={"x": x, "y": y, "yaw": yaw},
                  world_pose=odom_to_world((x, y), dock_world),
                  dock_world=dock_world, dock_odom=WORLD_ORIGIN,
                  return_world=WORLD_ORIGIN, return_odom=return_odom)
    for index, target_world in enumerate(waypoints, 1):
        evidence.emit("loop_started", index=index, target_world=target_world,
                      target_odom=world_to_odom(target_world, dock_world))
        if validate_follow_waypoints and index == LOOPS:
            goals = [target_world, WORLD_ORIGIN]
            # ROS 1 mirror still runs before the physical Create3.
            follow_two_waypoints(ros1_client, evidence, robot_version="ros1",
                                 goals_world=goals, dock_world=dock_world, yaw=yaw,
                                 timeout=action_timeout, tolerance=tolerance)
            follow_two_waypoints(client, evidence, robot_version="ros2",
                                 goals_world=goals, dock_world=dock_world, yaw=yaw,
                                 timeout=action_timeout, tolerance=tolerance)
        else:
            move(client, evidence, label=f"loop-{index}-random",
                 target_world=target_world, dock_world=dock_world,
                 yaw=yaw, timeout=action_timeout, tolerance=tolerance,
                 ros1_client=ros1_client)
            move(client, evidence, label=f"loop-{index}-world-origin",
                 target_world=WORLD_ORIGIN, dock_world=dock_world,
                 yaw=yaw, timeout=action_timeout, tolerance=tolerance,
                 ros1_client=ros1_client)
        evidence.emit("loop_pass", index=index)

    state = approach_and_dock(client, evidence, dock_world=dock_world, yaw=yaw,
                              action_timeout=action_timeout, status_timeout=status_timeout,
                              tolerance=tolerance, dock_approach_distance=dock_approach_distance,
                              ros1_client=ros1_client)
    return {"completed_loops": LOOPS, "dock_world": [*dock_world],
            "return_world": [*WORLD_ORIGIN], "return_odom": [*return_odom],
            "dock_approach_world": [*odom_to_world(
                dock_approach_odom(dock_approach_distance), dock_world)],
            "dock_approach_odom": [*dock_approach_odom(dock_approach_distance)],
            "final_dock_status": state}


def approach_and_dock(client: ISWireClient, evidence: Evidence, *,
                      dock_world: tuple[float, float], yaw: float,
                      action_timeout: float, status_timeout: float,
                      tolerance: float, dock_approach_distance: float,
                      ros1_client: ISWireClient | None = None) -> dict:
    """Navigate to the dock's -X side and then dock; never return to world (0,0)."""
    final_status = client.dock_status(timeout=status_timeout)
    evidence.emit("before_docking", **final_status)
    if not final_status["is_docked"]:
        approach_odom = dock_approach_odom(dock_approach_distance)
        approach_world = odom_to_world(approach_odom, dock_world)
        if world_to_odom(approach_world, dock_world) != approach_odom:
            raise ExperimentError("Dock approach frame transform is inconsistent")
        evidence.emit("dock_approach_planned",
                      target_world=approach_world, target_odom=approach_odom,
                      distance_from_dock_m=dock_approach_distance)
        # The general waypoint tolerance is 25 cm by default, which is too
        # permissive for a point only 20 cm from the charging station.
        measured_approach = move(
            client, evidence, label="dock-approach",
            target_world=approach_world, dock_world=dock_world,
            yaw=yaw, timeout=action_timeout,
            tolerance=min(tolerance, DOCK_APPROACH_TOLERANCE),
            ros1_client=ros1_client,
        )
        evidence.emit("dock_approach_verified",
                      target_odom=approach_odom,
                      measured_odom=(measured_approach["x"], measured_approach["y"]),
                      distance_from_dock_m=round(math.hypot(measured_approach["x"],
                                                             measured_approach["y"]), 4))
        approach_status = client.dock_status(timeout=status_timeout)
        evidence.emit("before_dock_action", **approach_status)
        if approach_status["is_docked"]:
            evidence.emit("dock_skipped", reason="robot already docked after approach")
        else:
            if approach_status.get("dock_visible") is False:
                evidence.emit("docking_warning", detail="dock not currently visible "
                              "even at the verified approach position; docking may fail")
            send_action(client, evidence, "dock", {}, action_timeout)
    else:
        evidence.emit("dock_skipped", reason="robot already docked")
    state = client.dock_status(expected=True, timeout=status_timeout)
    evidence.emit("docked_confirmed", **state)
    return state


def run_dock_only(client: ISWireClient, evidence: Evidence, *,
                  dock_world: tuple[float, float], action_timeout: float,
                  status_timeout: float, tolerance: float,
                  dock_approach_distance: float,
                  ros1_client: ISWireClient | None = None) -> dict:
    """Recover a previously undocked robot without repeating five movement loops."""
    pose = client.request("get_odometry", timeout=15)
    x, y, yaw = odometry_pose(pose)
    evidence.emit("dock_only_start_pose", odom=(x, y, yaw),
                  world=odom_to_world((x, y), dock_world))
    state = approach_and_dock(client, evidence, dock_world=dock_world, yaw=yaw,
                              action_timeout=action_timeout, status_timeout=status_timeout,
                              tolerance=tolerance, dock_approach_distance=dock_approach_distance,
                              ros1_client=ros1_client)
    return {"completed_loops": 0, "final_dock_status": state,
            "dock_approach_odom": [*dock_approach_odom(dock_approach_distance)]}


def try_emergency_stop(client: ISWireClient, evidence: Evidence, endpoint: str) -> None:
    """Best-effort stop over IS-Wire; requires /e_stop to be allowed by Core config."""
    try:
        response = client.request("ros.service", {
            "endpoint": endpoint,
            "ros_type": "irobot_create_msgs/srv/EStop",
            "payload": {"e_stop_on": True},
        }, timeout=10)
        evidence.emit("emergency_stop_requested", endpoint=endpoint, reply=response)
    except Exception as exc:
        evidence.emit("emergency_stop_unavailable", endpoint=endpoint, error=str(exc))
        print("WARNING: remote E-Stop was NOT confirmed; stop the robot physically!", file=sys.stderr)


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    run = parser.add_mutually_exclusive_group()
    run.add_argument("--execute", action="store_true", help="Send the E2E command sequence over IS-Wire")
    run.add_argument("--check-only", action="store_true", help="Read physical dock status only")
    run.add_argument("--dock-only", action="store_true",
                     help="Navigate to odom (-0.2,0), then dock; skip all five loops")
    run.add_argument("--distinct-commands-only", action="store_true",
                     help="Run only E3: ROS1 get_map and ROS2 get_odometry over IS-Wire, without motion")
    parser.add_argument("--confirm-area-clear", action="store_true",
                        help="Confirm a supervised, obstacle-free 2.4m x 2.4m area "
                             "plus robot clearance, dock, and travel paths")
    parser.add_argument("--robot-id", default="create_3")
    parser.add_argument("--ros1-mirror", action="store_true",
                        help="Mirror all navigation waypoints to the ROS 1 /move_base simulator via IS-Wire")
    parser.add_argument("--ros1-robot-id", default="ros1_sim",
                        help="ROS 1 simulator robot_id configured in the Core")
    parser.add_argument("--validate-follow-waypoints", action="store_true",
                        help="In loop 5, replace its two navigate calls with ONE 2-goal "
                             "follow_waypoints request per backend; requires --execute --ros1-mirror")
    parser.add_argument("--verbose", action="store_true",
                        help="Show every event on the terminal; detailed evidence is always saved")
    parser.add_argument("--dock-topic", default="/create3/dock_status",
                        help="Change to /dock_status for a non-namespaced Create 3")
    parser.add_argument("--broker-uri", default=os.environ.get("ISWIRE_BROKER_URI"),
                        help="IS-Wire broker URI, defaults to ISWIRE_BROKER_URI")
    parser.add_argument("--seed", type=int, default=None, help="Reproducible random sequence")
    parser.add_argument("--dock-world-x", type=float, default=DEFAULT_DOCK_WORLD[0],
                        help="Charging dock X in the physical floor frame (metres)")
    parser.add_argument("--dock-world-y", type=float, default=DEFAULT_DOCK_WORLD[1],
                        help="Charging dock Y in the physical floor frame (metres)")
    parser.add_argument("--dock-approach-distance", type=float,
                        default=DEFAULT_DOCK_APPROACH_DISTANCE,
                        help="Approach distance from dock in metres along odom -X "
                             "(default: 0.20, given observed undocking direction)")
    parser.add_argument("--action-timeout", type=float, default=180.0)
    parser.add_argument("--status-timeout", type=float, default=30.0)
    parser.add_argument("--position-tolerance", type=float, default=0.25,
                        help="Maximum post-action odometry position error in metres")
    parser.add_argument("--emergency-stop-endpoint", default="/create3/e_stop",
                        help="Best-effort emergency stop endpoint; must be authorised in Core")
    parser.add_argument("--output-root", type=Path,
                        default=Path.home() / "rostranslator-create3-results")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = argument_parser()
    args = parser.parse_args(argv)
    if (args.execute or args.dock_only) and not args.confirm_area_clear:
        parser.error("--execute and --dock-only require --confirm-area-clear and physical supervision")
    dock_world = (args.dock_world_x, args.dock_world_y)
    if not all(math.isfinite(v) for v in dock_world):
        parser.error("Dock floor coordinates must be finite")
    if args.action_timeout <= 0 or args.action_timeout > 300:
        parser.error("--action-timeout must be within (0,300] seconds")
    if args.status_timeout <= 0 or args.status_timeout > 300:
        parser.error("--status-timeout must be within (0,300] seconds")
    if not 0 < args.position_tolerance <= 0.5:
        parser.error("--position-tolerance must be within (0,0.5] metres")
    if not math.isfinite(args.dock_approach_distance) or not 0.15 <= args.dock_approach_distance <= 0.35:
        parser.error("--dock-approach-distance must be within [0.15,0.35] metres")
    if not args.robot_id or not args.dock_topic.startswith("/"):
        parser.error("--robot-id and an absolute --dock-topic are required")
    if (args.ros1_mirror or args.distinct_commands_only) and (
            not args.ros1_robot_id or args.ros1_robot_id == args.robot_id):
        parser.error("--ros1-robot-id must identify a different, configured ROS 1 robot")
    if args.validate_follow_waypoints and (not args.execute or not args.ros1_mirror):
        parser.error("--validate-follow-waypoints requires --execute --ros1-mirror")

    seed = args.seed if args.seed is not None else secrets.randbits(31)
    points_world = random_waypoints(seed)
    if len(points_world) != LOOPS:
        parser.error(f"random_waypoints must return exactly {LOOPS} points; "
                     "do not append the dock approach to this list")
    return_odom = world_to_odom(WORLD_ORIGIN, dock_world)
    approach_odom = dock_approach_odom(args.dock_approach_distance)
    approach_world = odom_to_world(approach_odom, dock_world)
    read_only = args.distinct_commands_only
    plan = {"seed": seed, "loops": 0 if args.dock_only or read_only else LOOPS,
            "coordinate_frame": "physical_floor",
            "world_bounds_m": [-COORD_LIMIT, COORD_LIMIT],
            "dock_world_xy_m": [*dock_world],
            "dock_odom_xy_m": [0.0, 0.0],
            "return_world_xy_m": [*WORLD_ORIGIN],
            "return_odom_xy_m": [*return_odom],
            "dock_approach_distance_m": args.dock_approach_distance,
            "dock_approach_world_xy_m": [*approach_world],
            "dock_approach_odom_xy_m": [*approach_odom],
            "ros1_mirror_enabled": args.ros1_mirror,
            "validate_follow_waypoints": args.validate_follow_waypoints,
            "validate_distinct_commands": (args.ros1_mirror and args.execute) or read_only,
            "ros1_robot_id": args.ros1_robot_id if args.ros1_mirror or read_only else None,
            "ros1_navigation_frame": "map" if args.ros1_mirror else None,
            "waypoints": [
                {"loop": i, "random_world_xy_m": [*point],
                 "random_odom_xy_m": [*world_to_odom(point, dock_world)],
                 "return_world_xy_m": [*WORLD_ORIGIN],
                 "return_odom_xy_m": [*return_odom]}
                for i, point in enumerate(points_world, 1)] if not args.dock_only and not read_only else [],
            "final_actions": [] if read_only else ["navigate_to_dock_approach", "dock"],
            "robot_id": args.robot_id}
    if not args.execute and not args.check_only and not args.dock_only and not read_only:
        print(json.dumps({"mode": "DRY_RUN (NO robot communication)", **plan}, indent=2))
        return 0
    if not args.broker_uri:
        parser.error("ISWIRE_BROKER_URI (or --broker-uri) is required")

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
    evidence = Evidence(args.output_root.expanduser().resolve() / run_id,
                        verbose=args.verbose)
    # Never write the broker URI: it may contain credentials.
    summary = {"run_id": run_id, "start_utc": now_utc(), "status": "running",
               "mode": "check-only" if args.check_only else
                       ("dock-only" if args.dock_only else
                        ("distinct-commands-only" if read_only else "physical-e2e")),
               "provenance": provenance(), **plan}
    client = ISWireClient(args.broker_uri, args.robot_id, args.dock_topic,
                          evidence=evidence)
    ros1_client = (ISWireClient(args.broker_uri, args.ros1_robot_id, args.dock_topic,
                                evidence=evidence)
                   if args.ros1_mirror or read_only else None)
    try:
        evidence.emit("started", mode=summary["mode"], seed=seed,
                      robot_id=args.robot_id, dock_world=dock_world)
        if args.check_only:
            state = client.dock_status(timeout=args.status_timeout)
            evidence.emit("dock_status", **state)
            summary["dock_status"] = state
        elif read_only:
            summary.update(run_distinct_commands(client, ros1_client, evidence))
        elif args.dock_only:
            evidence.emit("physical_supervision_confirmed", acknowledged=True)
            summary.update(run_dock_only(client, evidence, dock_world=dock_world,
                                         action_timeout=args.action_timeout,
                                         status_timeout=args.status_timeout,
                                         tolerance=args.position_tolerance,
                                         dock_approach_distance=args.dock_approach_distance,
                                         ros1_client=ros1_client))
        else:
            evidence.emit("physical_supervision_confirmed", acknowledged=True)
            if args.ros1_mirror:
                summary.update(run_distinct_commands(client, ros1_client, evidence))
            summary.update(run_mission(client, evidence, waypoints=points_world,
                                       dock_world=dock_world, action_timeout=args.action_timeout,
                                       status_timeout=args.status_timeout,
                                       tolerance=args.position_tolerance,
                                       dock_approach_distance=args.dock_approach_distance,
                                       ros1_client=ros1_client,
                                       validate_follow_waypoints=args.validate_follow_waypoints))
        summary["status"] = "passed"
        evidence.emit("experiment_pass", mode=summary["mode"])
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        summary["status"] = "failed"
        summary["error"] = f"{type(exc).__name__}: {exc}"
        evidence.emit("experiment_failed", error=summary["error"])
        if evidence.motion_attempted:
            try_emergency_stop(client, evidence, args.emergency_stop_endpoint)
        print(f"FAILED: {summary['error']}", file=sys.stderr)
        return 130 if isinstance(exc, KeyboardInterrupt) else 1
    finally:
        summary["validation"] = evidence.validation_results(
            ros1_mirror=args.ros1_mirror,
            follow_requested=args.validate_follow_waypoints,
            distinct_requested=(args.ros1_mirror and args.execute) or read_only)
        summary["observability_limits"] = [
            "IS-Wire correlation and backend odometry are observed by the client; "
            "adapter forwarding needs independently captured Core/bridge logs.",
            "Goal IDs are reported only when actually present in the PrIS response.",
            "E2 outcome ordering is returned by the Core, not independently timestamped "
            "by each ROS action server.",
        ]
        summary["finished_at_utc"] = now_utc()
        evidence.finish(summary)
        if (args.execute and args.ros1_mirror) or read_only:
            print("[VALIDATION] " + " | ".join(
                f"{key}={value}" for key, value in summary["validation"].items()
                if key.startswith("E1_") or key.startswith("E2_")
                or key.startswith("E3_")), flush=True)
        print(f"Evidence: {evidence.output}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
