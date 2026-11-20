# ROSTranslator

Connect **LabVisio's Programmable Intelligent Space (PrIS)** to robots running
**ROS 1 or ROS 2** through native **is-wire/is-msgs**. Applications send
version-neutral requests; ROSTranslator selects the correct ROS environment,
converts the payload and returns results and telemetry.

## Architecture

```text
PrIS application --is-wire/AMQP--> ROSTranslator Core (ROS-independent)
                                      |
                                  ROS adapters
                                      |
                         +------------+------------+
                         |            |            |
                    ROSBridge     ROSBridge     ROSBridge
                    Noetic        Jazzy         Humble
                         |            |            |
                      ROS 1        Create 3      ROS 2
```

- **Core:** broker, routing by `robot_id`, validation, allowlists, capability
  catalogue, request/reply and events. It does **not** require a ROS install.
- **Adapters:** convert semantic operations, topic messages, services and
  actions to ROS 1/ROS 2 wire contracts.
- **ROSBridge instances:** separate ROS environments. Multiple instances of
  the same ROS distribution are supported; each has a distinct endpoint.

The physical Create 3 example below runs **two containers on an Ubuntu PC-Host**
(Core, Jazzy ROSBridge). The bridge communicates through DDS with
the Raspberry Pi's `create3_republisher`, which communicates with the robot.
**No ROSBridge server needs to run on the Raspberry Pi.**

## Complete walkthrough: physical Create 3 + ROS 2 Jazzy

### 0. Requirements

- Linux with Docker Engine and Docker Compose plugin. The network adapter must be able to reach the Raspberry Pi.
- Raspberry Pi with ROS 2 Jazzy and the iRobot `create3_republisher` running.
- Create 3 accessible to the Pi (e.g., over USB), using `ROS_DOMAIN_ID=0` and Fast DDS, as in this example.

### 1. Check the robot from the Raspberry Pi

On `robot@robot1`:

```bash
source /opt/ros/jazzy/setup.bash
ros2 node list | grep create3_repub
ros2 topic list -t | grep /create3/odom
ros2 action list -t | grep /create3/navigate_to_position
```

Expected action:

```text
/create3/navigate_to_position [irobot_create_msgs/action/NavigateToPosition]
```

If this action is missing on the Pi, correct the Pi/Create 3 configuration
before starting the PC-Host services. Do not use the internal
`/robot1/_do_not_use/*` action endpoints.

### 2. Configure the PC-Host

On Ubuntu, from the cloned repository:

```bash
cd is-ros-translator
cp deploy/lab/.env.example deploy/lab/.env
cp configs/fastdds-unicast.example.xml configs/fastdds-unicast.xml
```

Edit `deploy/lab/.env` and **replace both placeholder secrets**.
`LAB_ROS_DOMAIN_ID` must match the robot (the example uses `0`).
The broker username and password must be valid for an AMQP URI;
alphanumeric passwords are simplest.

Configure Fast DDS to discover the Pi:

```bash
sed -i 's/PI_WIFI_IP/10.20.9.3/' configs/fastdds-unicast.xml
```

The XML file has exactly one `<address>` per `<udpv4>` locator.
Edit the file again if the Pi's Wi-Fi address changes.
The generated `deploy/lab/.env` and `configs/fastdds-unicast.xml` are
excluded from Git.

### 3. Start RabbitMQ, ROSBridge and Core

```bash
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml up -d --build
```

Inspect the three containers:

```bash
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml ps
docker logs --tail 50 rostranslator-bridge-jazzy
docker logs --tail 50 rostranslator-core
```

The Jazzy container uses `--network host`, `rmw_fastrtps_cpp`,
`ROS_DOMAIN_ID=0` and the Fast DDS unicast XML. Its ROSBridge WebSocket
listens on **127.0.0.1:9090**. The Core listens on **127.0.0.1:8080**.
RabbitMQ's AMQP and management ports are published only to the PC-Host's loopback.

If you already started manual containers named `rostranslator-core` or
`rostranslator-bridge-jazzy`, stop and remove **those specific old containers**
before running Compose; their existing files and host configuration remain
untouched.

### 4. Verify discovery and Core readiness

```bash
docker exec rostranslator-bridge-jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 daemon stop && ros2 topic list -t'
```

Look for `/create3/odom` and `/create3/dock_status`.
Then check the **action server**, not just the topic list:

```bash
docker exec rostranslator-bridge-jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 action info /create3/navigate_to_position'
```

The result should include **Action servers: 1** (or more).
Check the Core:

```bash
curl -fsS http://127.0.0.1:8080/health/ready
```

Proceed only if the JSON status is `ready`. See troubleshooting below if
it is `degraded` or the action server is missing.

### 5. Send real is-wire requests (no direct ROS action CLI)

The Core image includes an example client with the project's `is-wire`
dependencies. Run it from the Ubuntu PC-Host:

```bash
# Read odometry through is-wire -> Core -> ROSBridge -> Create 3
docker exec rostranslator-core python3 /app/examples/iswire_rpc_client.py odom
```

```bash
# Leave the dock, if currently docked
docker exec rostranslator-core python3 /app/examples/iswire_rpc_client.py undock
```

```bash
# Move to ABSOLUTE odom coordinates x=0 m, y=-5 m, final yaw=0 rad
docker exec rostranslator-core python3 /app/examples/iswire_rpc_client.py \
  navigate --x 0 --y -5 --yaw 0 --timeout 180
```

Monitor the position in another terminal:

```bash
docker exec rostranslator-bridge-jazzy bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /create3/odom'
```

The requested pose is **absolute in `odom`**, not a relative 5-metre move.
Reaching `x=0, y=-5` depends on the robot's starting odometry pose and on
there being a clear path.

Return to the charging dock when safe:

```bash
docker exec rostranslator-core python3 /app/examples/iswire_rpc_client.py dock
```

The example publishes a `google.protobuf.Struct` through
`is_wire.core.Channel` to topic `ROSTranslator.v2.Request` and waits for
the correlated reply. A navigation request has this shape:

```json
{
  "robot_id": "create_3",
  "operation": "navigate",
  "timeout_seconds": 180,
  "args": {
    "position": {"x": 0.0, "y": -5.0, "z": 0.0},
    "yaw": 0.0,
    "frame_id": "odom"
  }
}
```

The Core maps this to `/create3/navigate_to_position` using
`irobot_create_msgs/action/NavigateToPosition`. The same broker supports the
typed `is_msgs.robot_pb2.RobotTaskRequest` contract on
`ROSTranslator.v2.RobotTask.<robot_id>`.

### 6. Stop and restart the lab

```bash
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml stop
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml start
```

To remove only this lab's containers and network (not host configuration):

```bash
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml down
```

## ROS 1, ROS 2 and multiple distributions

Add a `bridge_id` for each ROS environment in the Core YAML, for example:

```yaml
bridges:
  jazzy_local: {ros_version: 2, distribution: jazzy, host: 127.0.0.1, port: 9090}
  humble_local: {ros_version: 2, distribution: humble, host: 127.0.0.1, port: 9092}
  noetic_local: {ros_version: 1, distribution: noetic, host: 127.0.0.1, port: 9091}
robots:
  create_3:
    ros_version: 2
    bridge_id: jazzy_local
    navigation:
      action: /create3/navigate_to_position
      action_type: irobot_create_msgs/action/NavigateToPosition
      frame_id: odom
```

Use `configs/multi-distro.example.yaml` as a complete catalogue example.
The Core does **not** install/switch ROS distributions; you must run the
corresponding ROSBridge image in the right ROS graph. Instance `domain_id`
and `rmw_implementation` are deployment metadata in the Core YAML; set
the real environment variables on each ROSBridge container.

Build optional bridge images:

```bash
docker build -f deploy/rosbridges/ros2/Dockerfile \
  --build-arg ROS_DISTRO=humble -t rostranslator-rosbridge:humble .
docker build -f deploy/rosbridges/ros1-noetic/Dockerfile \
  -t rostranslator-rosbridge:noetic .
```

See [multi-ROS setup](docs/MULTI_ROS.md) and
[capability configuration](docs/CAPABILITIES.md) for additional examples.

## Automated tests

Unit tests (Python 3.10 or compatible environment):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[test]'
pytest -q
```

The repository also includes an isolated Docker E2E lab with simulated ROS 1
and ROS 2 robots:

```bash
scripts/docker-ros-e2e.sh up
scripts/docker-ros-e2e.sh test
scripts/docker-ros-e2e.sh down
```

These simulation tests do not prove that a physical robot, its network or its
motion controller is safe or correctly configured.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `No action server available` | The Core must use **`/create3/navigate_to_position`**, and `ros2 action info` in the long-running bridge must report an action server. |
| Only `/rosout` and ROSBridge topics appear | Verify `FASTRTPS_DEFAULT_PROFILES_FILE` exists **inside the running bridge**, the Pi's actual Wi-Fi IP, the ROS domain and Fast DDS RMW. |
| Topics appear, but navigation fails | Check `ros2 action list -t`, the exact action name/type, ROSBridge logs and the result returned to the is-wire client. |
| Fast DDS XML reports duplicate `address` | Put a single `<address>` inside each `<udpv4>` element; see the provided XML example. |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | Rebuild the ROSBridge image; the startup script sources ROS before enabling `set -u`. |
| Core returns `degraded` | Check RabbitMQ and `docker logs rostranslator-core`; the Core retries is-wire broker connections. |
| Docker name/port already exists | Stop the previous manually-created instance before launching this Compose lab. |
