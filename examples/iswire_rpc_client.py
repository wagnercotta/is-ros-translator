#!/usr/bin/env python3
"""Native is-wire request client for ROSTranslator (no ROS install required)."""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
import uuid


def build_command(operation: str, robot: str, x: float | None, y: float | None,
                  yaw: float, timeout: float) -> dict:
    aliases = {"odom": "get_odometry", "topics": "ros.topics"}
    command = {
        "robot_id": robot,
        "operation": aliases.get(operation, operation),
        "request_id": str(uuid.uuid4()),
        "timeout_seconds": timeout,
        "args": {},
    }
    if operation == "navigate":
        if x is None or y is None:
            raise ValueError("navigate requires --x and --y")
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError("x, y and yaw must be finite")
        command["args"] = {
            "position": {"x": x, "y": y, "z": 0.0},
            "yaw": yaw,
            "frame_id": "odom",
        }
    return command


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("navigate", "odom", "topics", "dock", "undock"))
    parser.add_argument("--robot", default="create_3", help="Robot identifier in the Core YAML")
    parser.add_argument("--x", type=float, help="Absolute odom X coordinate (metres)")
    parser.add_argument("--y", type=float, help="Absolute odom Y coordinate (metres)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Final heading (radians)")
    parser.add_argument("--timeout", type=float, default=180.0, help="Operation timeout (seconds)")
    parser.add_argument("--dry-run", action="store_true", help="Print the request without sending it")
    args = parser.parse_args()

    if not 0 < args.timeout <= 300:
        parser.error("--timeout must be between 0 and 300 seconds")
    try:
        command = build_command(args.operation, args.robot, args.x, args.y, args.yaw, args.timeout)
    except ValueError as exc:
        parser.error(str(exc))

    if args.dry_run:
        print(json.dumps(command, indent=2))
        return 0

    broker_uri = os.environ.get("ISWIRE_BROKER_URI")
    if not broker_uri:
        parser.error("ISWIRE_BROKER_URI is required")

    # Imports are intentionally lazy so --dry-run requires no installed SDK.
    from google.protobuf.json_format import MessageToDict, ParseDict
    from google.protobuf.struct_pb2 import Struct
    from is_wire.core import Channel, Message, Subscription

    channel = Channel(broker_uri)
    try:
        reply_to = Subscription(channel)
        payload = Struct()
        ParseDict(command, payload)
        request = Message(content=payload, reply_to=reply_to)
        channel.publish(request, topic="ROSTranslator.v2.Request")
        print(f"Sent {command['operation']} for {args.robot}; waiting for the result...", flush=True)

        deadline = time.monotonic() + args.timeout + 15
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("No matching is-wire reply before the deadline")
            try:
                response = channel.consume(timeout=min(10.0, remaining))
            except socket.timeout:
                continue
            if response is None or response.correlation_id != request.correlation_id:
                continue
            if response.status is not None and not response.status.ok():
                raise RuntimeError(f"is-wire transport error: {response.status}")
            result = MessageToDict(response.unpack(Struct), preserving_proto_field_name=True)
            print(json.dumps(result, indent=2, ensure_ascii=False))
            return 0 if result.get("ok") else 1
    except (TimeoutError, RuntimeError, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        channel.close()


if __name__ == "__main__":
    raise SystemExit(main())
