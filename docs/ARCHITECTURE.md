# Architecture, contract, and security notes

## Logical flow

```
PrIS client ─ HTTP JSON / is-wire Protobuf RPC ─┐
                                                             ▼
  Gateway → RBAC-like endpoint allowlists → command dispatcher
                       │                ├→ Semantic translator: navigate/waypoints/map/odom
                       │                └→ Generic resolver: rosapi schema discovery/cache/validation
                       ▼                                    │
            Robot connection manager (1 roslibpy per robot)  │
                 ├→ ROS1 adapter: ROS1 actionlib             │
                 └→ ROS2 adapter: ROS2 action calls          │
                             │ ROS WebSocket                 │
                 Remote VM's rosbridge + rosapi              │
                 ROS topics / services / actions ◄───────────┘
                 ROS topic callbacks → leased subscribers → event bus
                                             ├→ AMQP topic exchange
                                             ├→ is-wire events
                                             └→ HTTP WebSocket /v1/events
```

A core rule: the PrIS contract must not import `rclpy`, `rospy`, or packages specific to a particular robot. Only the ROS VM needs ROS message packages. `roslibpy` transports JSON-compatible structures to that VM's rosbridge.

## Contracts

### Request

```json
{
  "robot_id": "turtlebot_ros2",
  "operation": "navigate",
  "args": {"position": {"x": 2.5, "y": 3.0, "z": 0.0}, "yaw": 0.0, "frame_id": "map"},
  "request_id": "optional-unique-id",
  "timeout_seconds": 30
}
```

Supported `operation` values:

| Operation | Args | Meaning |
|---|---|---|
| `navigate` | `position`, optional `yaw`, `frame_id` | ROS1 move_base or ROS2 NavigateToPose action |
| `follow_waypoints` | `waypoints: [{position, yaw?}, ...]` | Sequential action goals, stop on failure |
| `get_map` | `{}` | ROS1/ROS2 `nav_msgs/GetMap` using configured service |
| `get_odometry` | `{}` | Sample one `nav_msgs/Odometry` message |
| `ros.publish` | `endpoint`, `payload`, optional `ros_type` | Allowlisted topic publication and rosapi schema check |
| `ros.subscribe` / `ros.unsubscribe` | `endpoint`, `subscriber_id`, optional `message_type` and `lease_seconds` | Leased ROS topic subscription from any request transport |
| `ros.service` | `endpoint`, `payload`, optional `ros_type` | Allowlisted service call and schema check |
| `ros.action` | `endpoint`, `ros_type`, `payload` | Allowlisted ROS1/ROS2 action (ROS2 type discovery) |
| `ros.schema` | `kind: topic/service/action`, `endpoint`, action `ros_type` | Get inspected ROS schema |
| `ros.topics` | `{}` | List active topics via rosapi |
| `ros.services` | `{}` | List active services via rosapi |

The supported semantic navigation config (`navigation.action`, `.action_type`, `.frame_id`) is not inferred from random ROS topic names: it is configured **per robot**. This prevents assuming all robots implement Nav2.

### Response

```json
{"ok":true,"request_id":"...","robot_id":"turtlebot_ros2","operation":"get_odometry","result":{"header":{},"pose":{},"twist":{}}}
```

Failures are structured: `{"ok": false, "error": "forbidden|invalid_request|ros_unavailable|timeout|action_failed", "detail": "..."}`. In HTTP these map to HTTP errors; in is-wire responses they are returned in the payload. A topic `ros.publish` response indicates only that rosbridge was asked to publish, **not delivery to a robot or successful motion**.

### Native is-wire/is-msgs (sole AMQP transport)

The gateway uses the official LabVisio `is-wire==1.2.1` API. All AMQP RPC,
event subscriptions, broker connections and published replies are performed by
`is_wire.core.Channel`, `Message` and `Subscription`. No direct RabbitMQ or
alternative AMQP client code is used. `is-wire` itself uses the `amqp` library
internally; that dependency is owned by the SDK.

| Topic | Request / event Protobuf | Response |
|---|---|---|
| `ROSTranslator.v2.Request` | `google.protobuf.Struct` | `Struct` with `ok`, `result` / `error` |
| `ROSTranslator.v2.RobotTask.<robot_id>` | `is_msgs.robot_pb2.RobotTaskRequest` | `is_msgs.robot_pb2.RobotTaskReply` and `Status` |
| `ROSTranslator.v2.Event.<robot_id>.<kind>` | `google.protobuf.Struct` | Pub/sub only |
| `ROSTranslator.v2.Pose.<robot_id>` | `is_msgs.common_pb2.Pose` | Pub/sub only |

`Struct` supports semantic operations (including `navigate`) and generic ROS
operations. The `RobotTaskRequest` route provides a typed `is-msgs` alternative
for standard navigation tasks.

A worker thread owns a blocking `is-wire` Channel, with all ROS operations
submitted back to the asyncio loop. Broker disconnects cause a fresh Channel,
resubscription and bounded exponential backoff; no FastAPI startup failure.
`incoming.create_reply()` preserves the original integer correlation ID.

**Limitation:** `BasicMoveTask.allowed_error` and `.rate` are checked for valid
nonnegative values but do not reconfigure Nav2/move_base planners. Nav2 goal
acceptance uses the robot's own navigation-controller settings. No distributed
exactly-once execution is guaranteed.

**Legacy support:** With `legacy_robot_ids`, historical
`ROSTranslator.<id>.GetInfo/Translate` accepts `ROSTranslateRequest` and the
partial `RobotTaskRequest` workflow. See `proto/is_ros_legacy.proto`.

The is-wire SDK uses its own exchange (`is`) and routing topics. Configure
RabbitMQ permissions for these topics.

## Schema discovery and validation

1. Verify endpoint against per-robot allowlist.
2. Resolve advertised topic/service ROS type through rosapi; reject caller-type mismatch.
3. Obtain message/service request typedefs; cache for `schema_ttl_seconds`.
4. Check unknown fields, primitive types/ranges, fixed arrays, nested messages (depth bounded).
5. Send JSON-compatible data to rosbridge; rosbridge and the installed ROS packages perform native serialisation.
6. ROS 2 generic actions also resolve the advertised action type and query rosapi action_goal_details. ROS 1 action goals use `FooGoal` definitions.

This is **structural**, not semantic, validation. Frame transforms, coordinate systems, bounded acceleration, safe velocity, collision avoidance, arbitrary enum meanings and physical safety remain the robot application's responsibility. A custom type unavailable on the ROS VM cannot be used merely because a JSON payload can be built.

Type definitions may change across ROS deployments. Restart or invalidate the cache on interface changes. Default config is *fail-closed* when schemas cannot be discovered. The `allow_live_schema_fallback` option weakens the guarantee and is only suitable for controlled debugging.

## ROS-specific details

| Feature | ROS 1 | ROS 2 |
|---|---|---|
| Navigation action | `move_base_msgs/MoveBaseAction` | `nav2_msgs/action/NavigateToPose` or Create 3 `irobot_create_msgs/action/NavigateToPosition` |
| Goal payload | `{"target_pose": PoseStamped}` | Nav2: `{"pose": PoseStamped, "behavior_tree":""}`; Create 3: `{"goal_pose": PoseStamped, "achieve_goal_heading": true}` |
| Map service type | `nav_msgs/GetMap` | `nav_msgs/srv/GetMap` |
| Odometry message | `nav_msgs/Odometry` | `nav_msgs/msg/Odometry` |
| Action completion | actionlib status `3 = SUCCEEDED` | action status `4 = SUCCEEDED` |
| Transient-local subscriptions | ROS latch | QoS specific; not tuned in this preview |

Navigation uses an action and waits for a terminal result; it does not equate reaching a waypoint with a single-axis odometry tolerance. `follow_waypoints` serialises goals per robot using an in-process lock, but allows different robots to operate simultaneously. With multiple gateway replicas, use a distributed lock.

## Security and reliability

- REST endpoints require the `X-API-Key` header by default. Health probes are unauthenticated.
- Generic operations are denied unless the exact endpoint or a configured `fnmatch` glob is allowed. Avoid wildcards in production.
- Semantic `navigate`, `get_map` and `get_odometry` use configured endpoints; they are not subject to the generic allowlist because they are explicitly declared capabilities. Disable navigation for a robot with `navigation.enabled=false`.
- **Never** expose rosbridge (port 9090) directly to the internet. It grants powerful ROS graph access. Use an isolated host-only network, VPN and firewall rules.
- Plain WebSocket ROS bridge traffic is unencrypted on the VM demonstration network. Add TLS at the endpoint/tunnel for real deployments.
- Keyed HTTP authentication is for a controlled lab, not a complete multi-tenant RBAC solution.
- Topic events use bounded buffers and may be dropped under overload. Command RPC is not an exactly-once or transactional channel.
- ROS 2 QoS negotiation and the native Nav2 multi-waypoint action are not implemented; for these cases extend the transport/adapter.
- `get_map` is bounded by `max_response_bytes`; large maps should use object storage or chunked/binary transfer.

## Relevant external APIs

- roslibpy 2.1 API reference: https://roslibpy.readthedocs.io/en/latest/reference/index.html
- rosbridge protocol specification: https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md
- ROS 2 rosapi service types: https://index.ros.org/p/rosapi/
- Original implementation repositories: see README and NOTICE.
