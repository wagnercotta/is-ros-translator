"""Partial backward-compatible RPC adapter for the 2022 viniciusbaltoe contract.

Supports GetInfo (topic list) and RobotTaskRequest / BasicMoveTask waypoint
navigation. Does NOT claim the unpublished T_MapRequest workflow or custom
allowed_error tuning of the original implementation.
"""
from __future__ import annotations
import json
from .legacy_proto import legacy_types
from .errors import InvalidRequest
from .core.gateway import Gateway


async def process_legacy(gateway: Gateway, robot_id: str, operation: str, incoming):
    Request, Reply = legacy_types()
    req = incoming.unpack(Request)
    reply = Reply()
    if operation == "GetInfo":
        if req.topic:
            # Inspect a topic's type; do not consume telemetric samples unexpectedly.
            typ = await gateway.topic_type(robot_id, req.topic)
            reply.data = json.dumps({"topic": req.topic, "ros_type": typ})
        else:
            topics = await gateway.bridge(robot_id).get_topics()
            reply.data = json.dumps({"topics": topics})
        return reply
    if operation != "Translate":
        raise InvalidRequest("Unknown legacy operation")
    if req.function not in ("", "RTR_Translate"):
        raise InvalidRequest(f"Legacy function {req.function!r} not implemented in v2")
    from is_msgs.robot_pb2 import RobotTaskRequest, BasicMoveTask
    type_name = req.any.type_url.split("/")[-1].split(".")[-1]
    if type_name == "RobotTaskRequest":
        task = RobotTaskRequest()
        if not req.any.Unpack(task):
            raise InvalidRequest("Invalid RobotTaskRequest Any payload")
        motion = task.basic_move_task
    elif type_name == "BasicMoveTask":
        motion = BasicMoveTask()
        if not req.any.Unpack(motion):
            raise InvalidRequest("Invalid BasicMoveTask Any payload")
    else:
        raise InvalidRequest(f"Legacy message type {type_name} not supported")
    if hasattr(motion, "allowed_error") and motion.allowed_error:
        raise InvalidRequest("Legacy allowed_error planner tuning is not supported by v2")
    if not motion.positions:
        raise InvalidRequest("No positions in legacy navigation request")
    yaw = 0.0
    if motion.HasField("final_orientation"):
        yaw = float(motion.final_orientation.yaw)
    from .models import Command
    result = await gateway.execute(Command(robot_id=robot_id, operation="follow_waypoints",
        args={"waypoints": [{"position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)}, "yaw": yaw}
                            for p in motion.positions]}, timeout_seconds=180))
    reply.data = json.dumps(result, separators=(",", ":"))
    return reply
