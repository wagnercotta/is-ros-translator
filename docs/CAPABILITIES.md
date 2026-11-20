# Per-robot PrIS capability catalogue

The gateway supports named semantic capabilities defined **per robot** in its
YAML configuration. A capability maps a friendly PrIS operation name to a
standard ROS publish, service or action. There is no need to modify the Python
dispatcher to introduce another name, provided the target ROS interface already
exists and is supported by rosbridge/rosapi.

## Example

Add these settings under a robot in `deploy/kubernetes/configmap.yaml` (the
`gateway.yaml` value), or in any other gateway configuration:

```yaml
robots:
  robot_example:
    ros_version: 2
    host: rosbridge.ros2-a.svc
    port: 9090
    allowed:
      publish: [/chatter]
      subscribe: [/odom]
      services: [/camera/capture]
      actions: [/dock]
    capabilities:
      announce:
        operation: ros.publish
        endpoint: /chatter
        ros_type: std_msgs/msg/String
        default_payload: {data: "Hello"}
        description: Publishes a message on chatter
      capture_image:
        operation: ros.service
        endpoint: /camera/capture
        ros_type: std_srvs/srv/Trigger
        default_payload: {}
        description: Triggers the robot's camera capture service
      dock:
        operation: ros.action
        endpoint: /dock
        ros_type: irobot_create_msgs/action/Dock
        default_payload: {}
        allow_overrides: false
        description: Docks the robot at its home station
      monitor_odometry:
        operation: ros.subscribe
        endpoint: /odom
        ros_type: nav_msgs/msg/Odometry
      stop_monitoring_odometry:
        operation: ros.unsubscribe
        endpoint: /odom
```

The `/camera/capture` service, `/dock` action and their types above are only
illustrative. Install those interfaces in the robot's ROS graph first and
adjust their names/types for the actual device. To invoke a capability, both
its `endpoint` and its `operation` must appear in the corresponding
`allowed.publish`, `allowed.services` or `allowed.actions` list. The gateway
refuses to start when a configured capability is outside that allowlist.

## PrIS commands

The same command JSON works over `POST /v1/commands` or in a native is-wire
`google.protobuf.Struct` request on topic `ROSTranslator.v2.Request`:

```json
{"robot_id": "robot_example", "operation": "announce", "args": {"data": "Hello from PrIS"}}
```

```json
{"robot_id": "robot_example", "operation": "capture_image"}
```

```json
{"robot_id": "robot_example", "operation": "dock"}
```

Topic subscriptions use `subscriber_id` in `args` and deliver incoming events
through the gateway's standard PrIS event transport. A matching unsubscribe
removes the lease:

```json
{"robot_id": "robot_example", "operation": "monitor_odometry", "args": {"subscriber_id": "lab-observer", "lease_seconds": 300}}
```

```json
{"robot_id": "robot_example", "operation": "stop_monitoring_odometry", "args": {"subscriber_id": "lab-observer"}}
```

The command `args` object is the ROS payload. Keys supplied in `args` override
the configured `default_payload`; nested objects are merged recursively, so a
single position component does not remove the other configured components.
For a capability with `allow_overrides: false`, `args` must be empty. All
commands retain their original PrIS operation, robot id and correlation id in
the reply. Action completion and failures are propagated through the existing
ROS adapter.

ROS message/service schema validation and action type checking remain active:
the catalogue does **not** bypass the `allowed` endpoint policy. Generic
`ros.publish`, `ros.service`, `ros.action`, `ros.subscribe` and `ros.unsubscribe` commands remain available for
clients that need raw ROS access. The built-in semantic commands (`navigate`,
`follow_waypoints`, `get_odometry`, `get_map`) also remain unchanged. Reserved
operation names cannot be reused as catalogue names.

## Discover available robot capabilities

The authenticated REST API exposes:

```text
GET /v1/robots
GET /v1/robots/{robot_id}/capabilities
```

For example:

```bash
curl -s -H 'x-api-key: local-e2e-api-key' \
  http://127.0.0.1:18080/v1/robots/turtlebot_ros2/capabilities
```

`/v1/robots` lists the names of each robot's capabilities; the detailed
endpoint returns their type, endpoint, description and default payload.

## Update a catalogue in Kubernetes

Edit `deploy/kubernetes/configmap.yaml`, then reload the gateway:

```bash
kubectl apply -k deploy/kubernetes
kubectl -n is-ros-translator rollout restart deployment/is-ros-translator
```

The catalogue is loaded at startup, so changes to the ConfigMap require a
gateway restart. This feature does **not** provide runtime registration or
automatically create ROS functions; it only maps named PrIS operations to
existing ROS interfaces.
