#!/usr/bin/env python3
"""Black-box ROS1 Noetic + ROS2 Jazzy integration checks for the Docker lab.

No ROS Python library is imported. Tests call the gateway over HTTP/WebSocket
and is-wire, so every response flows through real rosbridge containers.
"""
from __future__ import annotations
import json
import os
import sys
import time
import uuid
import urllib.error
import urllib.request
import websocket

GATEWAY = os.getenv("GATEWAY_URL", "http://gateway:8080").rstrip("/")
KEY = os.environ["GATEWAY_API_KEY"]
ISWIRE_BROKER_URI = os.environ["ISWIRE_BROKER_URI"]
CASES = (
    ("jackal_ros1", "/dynamic_map", "std_msgs/String"),
    ("turtlebot_ros2", "/map_server/map", "std_msgs/msg/String"),
)
FAILURES = []
PASS = 0


def req(method: str, path: str, data=None, expect: int = 200, timeout: float = 75.0):
    raw = None if data is None else json.dumps(data).encode()
    request = urllib.request.Request(
        GATEWAY + path, data=raw, method=method,
        headers={"X-API-Key": KEY, "Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            status, body = reply.status, reply.read().decode()
    except urllib.error.HTTPError as exc:
        status, body = exc.code, exc.read().decode()
    parsed = json.loads(body)
    if status != expect:
        raise AssertionError(f"HTTP {method} {path}: {status} != {expect}: {body[:800]}")
    return parsed


def command(robot: str, operation: str, args=None, timeout: int = 45):
    response = req("POST", "/v1/commands", {
        "robot_id": robot, "operation": operation,
        "args": args or {}, "timeout_seconds": timeout,
        "request_id": str(uuid.uuid4()),
    }, timeout=timeout + 15)
    if not response.get("ok"):
        raise AssertionError(f"{robot}/{operation}: {json.dumps(response)[:800]}")
    return response["result"]


def run(label, fn):
    global PASS
    print(f"[RUN ] {label}", flush=True)
    try:
        fn()
    except Exception as exc:
        FAILURES.append((label, str(exc)))
        print(f"[FAIL] {label}: {exc}", flush=True)
    else:
        PASS += 1
        print(f"[PASS] {label}", flush=True)


def check_ready():
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        try:
            value = req("GET", "/health/ready", timeout=5)
            if value.get("status") == "ready":
                return
        except Exception:
            pass
        time.sleep(2)
    raise AssertionError("Gateway /health/ready did not respond")


def connect_iswire_channel(seconds=120, retry_seconds=2.0):
    """Use only is-wire to connect; retry while the broker starts."""
    from is_wire.core import Channel

    deadline = time.monotonic() + seconds
    while True:
        try:
            return Channel(ISWIRE_BROKER_URI)
        except Exception as exc:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ConnectionError(
                    f'is-wire broker unavailable after {seconds}s: {type(exc).__name__}: {exc}'
                ) from exc
            print(f'[WAIT] is-wire AMQP broker: {type(exc).__name__}: {exc}', flush=True)
            time.sleep(min(retry_seconds, remaining))


def check_topics(robot):
    value = command(robot, "ros.topics")
    assert "/odom" in value["topics"], value
    assert "/chatter" in value["topics"], value


def check_services(robot, map_endpoint):
    value = command(robot, "ros.services")
    assert map_endpoint in value["services"], value


def check_odometry(robot):
    value = command(robot, "get_odometry", timeout=15)
    position = value.get("pose", {}).get("pose", {}).get("position", {})
    assert "x" in position and "y" in position, value


def check_map(robot):
    value = command(robot, "get_map")
    m = value.get("map", {})
    assert int(m["info"]["width"]) == 20, str(value)[:600]
    assert int(m["info"]["height"]) == 20, str(value)[:600]
    assert len(m["data"]) == 400, str(value)[:600]


def check_navigation(robot):
    value = command(robot, "navigate", {"position": {"x": 1.5, "y": -0.5}}, timeout=25)
    assert value is not None
    check_odometry(robot)


def check_waypoints(robot):
    result = command(robot, "follow_waypoints", {"waypoints": [
        {"position": {"x": 0.5, "y": 0.5}},
        {"position": {"x": -0.5, "y": 0.0}},
    ]}, timeout=45)
    assert result["completed_waypoints"] == 2, result


def check_discovery(robot):
    value = command(robot, "ros.schema", {"kind": "topic", "endpoint": "/chatter"})
    assert value["schema"]["typedefs"], value
    assert "String" in value["ros_type"], value


def check_generic_publish(robot, ros_type):
    val = command(robot, "ros.publish", {"endpoint": "/chatter", "ros_type": ros_type,
                                          "payload": {"data": "from-PrIS-docker-lab"}})
    assert val["published"] is True, val


def check_default_deny(robot):
    result = req("POST", "/v1/commands", {
        "robot_id": robot, "operation": "ros.publish",
        "args": {"endpoint": "/cmd_vel", "payload": {}, "ros_type": "geometry_msgs/msg/Twist"}
    }, expect=403)
    assert result["error"] == "forbidden", result


def check_event_subscription(robot):
    ws_url = ("wss" if GATEWAY.startswith("https") else "ws") + GATEWAY.split("://", 1)[1] + "/v1/events"
    sock = websocket.create_connection(ws_url, header=[f"X-API-Key: {KEY}"], timeout=20)
    subscriber_id = "docker-lab-" + uuid.uuid4().hex[:8]
    spec = {"robot_id": robot, "endpoint": "/odom", "subscriber_id": subscriber_id}
    try:
        registered = req("POST", "/v1/subscriptions", spec)
        assert registered["subscribed"] is True
        start = time.monotonic()
        while time.monotonic() - start < 16:
            msg = json.loads(sock.recv())
            if msg.get("robot_id") == robot and msg.get("kind") == "topic" and msg.get("endpoint") == "/odom":
                assert "pose" in msg.get("payload", {}), msg
                return
        raise AssertionError("Subscribed to /odom but no matching WebSocket events received")
    finally:
        try:
            req("DELETE", "/v1/subscriptions", spec)
        finally:
            sock.close()


def check_iswire():
    """Prove native is-wire request/reply and integer correlation id."""
    import socket
    from is_wire.core import Message, Subscription
    from google.protobuf.struct_pb2 import Struct
    from google.protobuf.json_format import ParseDict, MessageToDict
    channel = connect_iswire_channel()
    try:
        replies = Subscription(channel)
        payload = Struct()
        ParseDict({"robot_id": "turtlebot_ros2", "operation": "ros.topics", "args": {}}, payload)
        request_message = Message(content=payload, reply_to=replies)
        channel.publish(request_message, topic='ROSTranslator.v2.Request')
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            try:
                reply = channel.consume(timeout=max(0.1, deadline - time.monotonic()))
            except socket.timeout:
                break
            if reply.correlation_id != request_message.correlation_id:
                continue
            if reply.status is not None and not reply.status.ok():
                raise AssertionError(str(reply.status))
            obj = MessageToDict(reply.unpack(Struct), preserving_proto_field_name=True)
            assert obj["ok"] is True, obj
            assert "/odom" in obj["result"]["topics"], obj
            return
        raise AssertionError("No correlated is-wire RPC reply")
    finally:
        channel.close()


def main():
    run("Gateway ready", check_ready)
    if FAILURES:
        return 1
    robot_ids = {x["id"] for x in req("GET", "/v1/robots")}
    assert robot_ids == {x[0] for x in CASES}, robot_ids
    for robot, map_endpoint, string_type in CASES:
        run(f"{robot}: ROS topics discovered", lambda r=robot: check_topics(r))
        run(f"{robot}: ROS services discovered", lambda r=robot,e=map_endpoint: check_services(r,e))
        run(f"{robot}: generic schema discovery", lambda r=robot: check_discovery(r))
        run(f"{robot}: generic typed publish", lambda r=robot,t=string_type: check_generic_publish(r,t))
        run(f"{robot}: map service", lambda r=robot: check_map(r))
        run(f"{robot}: odometry", lambda r=robot: check_odometry(r))
        run(f"{robot}: navigate action", lambda r=robot: check_navigation(r))
        run(f"{robot}: two waypoints", lambda r=robot: check_waypoints(r))
        run(f"{robot}: events over WebSocket", lambda r=robot: check_event_subscription(r))
        run(f"{robot}: deny unauthorised topic", lambda r=robot: check_default_deny(r))
    run("PrIS Protobuf RPC over is-wire", check_iswire)
    print(f"\nRESULTS: {PASS} passed; {len(FAILURES)} failed", flush=True)
    if FAILURES:
        print("Failures:", flush=True)
        for label, error in FAILURES:
            print(f" - {label}: {error}", flush=True)
    return int(bool(FAILURES))


if __name__ == "__main__":
    sys.exit(main())
