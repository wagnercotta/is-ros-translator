# One PrIS gateway for ROS 1 and ROS 2

The ROSTranslator is a single PrIS/is-wire server that can connect to any
number of robot-specific rosbridge WebSocket endpoints. It normalises PrIS
requests, selects the `robot_id` entry from the catalogue, validates the
requested operation against that robot's allowlist and routes it using the
appropriate ROS 1 or ROS 2 encoding.

There is **no ROS master and no ROS 1 installation inside the gateway**.
For example, an independent Jazzy ROSBridge container can discover the Create 3
through the Raspberry Pi, while a Noetic ROSBridge connects to a ROS 1 master.
The ROS-independent Core connects to both over WebSockets.

Example robot entries to merge into a gateway configuration's `robots` key
(replace the ROS 1 host and namespace with real values):

```yaml
robots:
  create_3:
    ros_version: 2
    host: 127.0.0.1
    port: 9090
    navigation:
      action: /create3/navigate_to_position
      action_type: irobot_create_msgs/action/NavigateToPosition
      frame_id: odom
    map_service: null
    allowed:
      actions: [/create3/navigate_to_position, /create3/dock, /create3/undock]
      subscribe: [/create3/odom]
    capabilities:
      dock:
        operation: ros.action
        endpoint: /create3/dock
        ros_type: irobot_create_msgs/action/Dock
        allow_overrides: false
      undock:
        operation: ros.action
        endpoint: /create3/undock
        ros_type: irobot_create_msgs/action/Undock
        allow_overrides: false

  jackal_1:
    ros_version: 1
    host: REPLACE_WITH_ROS1_ROSBRIDGE_HOST
    port: 9090
    navigation:
      action: /jackal_1/move_base
      action_type: move_base_msgs/MoveBaseAction
      frame_id: map
    odom_topic: /jackal_1/odom
    allowed:
      actions: [/jackal_1/move_base]
      subscribe: [/jackal_1/odom]
```

The same PrIS/is-wire request structure works for either robot:

```json
{"robot_id": "create_3", "operation": "navigate", "args": {"position": {"x": 0.3, "y": 0.0}}}
```

```json
{"robot_id": "jackal_1", "operation": "navigate", "args": {"position": {"x": 0.3, "y": 0.0}}}
```

The gateway converts the first to the Create 3 `NavigateToPosition` action
goal, and the second to the ROS 1 `move_base` action goal. The ROS 1
rosbridge needs to run alongside the robot's own ROS 1 master and motion
stack, whereas the ROS 2 rosbridge participates in DDS discovery.

For several ROS 1 masters or several isolated ROS 2 DDS domains, run a
separate rosbridge process/container for each graph and give each gateway
robot configuration the corresponding host/port. `ROS_DOMAIN_ID` is set per
ROS 2 process and cannot be changed per individual gateway request.

The gateway does not auto-discover ROS version from an is-wire request:
`robot_id` selects an explicit, auditable configuration. The current
adapters already cover ROS 1 and ROS 2 action clients and native type names;
the existing `capabilities` mapping makes new operations configurable.
