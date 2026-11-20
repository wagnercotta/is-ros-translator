#!/usr/bin/env python3
"""No-dependency gateway client. Example: python examples/http_client.py robots"""
import argparse
import json
import os
import urllib.request
import urllib.error


def send(method, path, data=None):
    base = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080")
    key = os.environ.get("GATEWAY_API_KEY", "")
    payload = None if data is None else json.dumps(data).encode()
    request = urllib.request.Request(base + path, data=payload, method=method,
                  headers={"Content-Type": "application/json", "X-API-Key": key})
    try:
        with urllib.request.urlopen(request, timeout=180) as response:
            print(json.dumps(json.load(response), indent=2))
    except urllib.error.HTTPError as exc:
        print(exc.code, exc.read().decode())
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=["robots", "topics", "services", "navigate", "waypoints", "odometry", "map", "subscribe", "unsubscribe", "schema", "publish"])
    parser.add_argument("--robot", default="turtlebot_ros2")
    parser.add_argument("--x", type=float, default=1)
    parser.add_argument("--y", type=float, default=2)
    parser.add_argument("--topic", default="/odom")
    parser.add_argument("--subscriber", default="vm-demo")
    args = parser.parse_args()
    if args.operation == "robots":
        return send("GET", "/v1/robots")
    if args.operation in ("subscribe", "unsubscribe"):
        spec = {"robot_id": args.robot, "endpoint": args.topic, "subscriber_id": args.subscriber}
        return send("POST" if args.operation == "subscribe" else "DELETE", "/v1/subscriptions", spec)
    operations = {"topics": "ros.topics", "services": "ros.services", "navigate": "navigate",
                  "waypoints": "follow_waypoints", "odometry": "get_odometry", "map": "get_map",
                  "schema": "ros.schema", "publish": "ros.publish"}
    options = {}
    if args.operation == "navigate":
        options = {"position": {"x": args.x, "y": args.y}}
    elif args.operation == "waypoints":
        options = {"waypoints": [{"position": {"x": 1.0, "y": 2.0}}, {"position": {"x": args.x, "y": args.y}}]}
    elif args.operation == "schema":
        options = {"endpoint": args.topic, "kind": "topic"}
    elif args.operation == "publish":
        if args.topic != "/chatter":
            parser.error("The demo publish example is intentionally restricted to /chatter")
        options = {"endpoint": "/chatter", "ros_type": "std_msgs/msg/String" if args.robot == "turtlebot_ros2" else "std_msgs/String",
                   "payload": {"data": "Hello from PrIS"}}
    send("POST", "/v1/commands", {"robot_id": args.robot, "operation": operations[args.operation],
                                     "args": options, "timeout_seconds": 30})


if __name__ == "__main__":
    main()
