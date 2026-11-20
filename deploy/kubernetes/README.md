# Kubernetes: PrIS is-ros-translator v0.4.10

Named PrIS capabilities can be assigned independently to each robot directly
inside `configmap.yaml`, alongside its rosbridge host and `allowed` ROS
endpoints. See [the capability guide](../../docs/CAPABILITIES.md) for examples
of `announce`, `capture_image`, `dock` and named ROS actions.

This deployment installs **only the translator gateway**; it does not install
RabbitMQ, ROS 1, ROS 2, RViz, Gazebo, or the old Docker E2E stack. Deploy each
ROS installation independently and expose its **rosbridge WebSocket (9090)**
through a reachable Kubernetes Service or stable hostname. The gateway does
not require ROS installed locally. `rosapi` must also run in each ROS system.

## Topology

```
PrIS is-wire/is-msgs client -> RabbitMQ (existing installation)
                                  |
                          is-ros-translator (Deployment, 1 replica)
                                  |
                           independent rosbridges
                                  |
                   +--------------+-------------+
                   |              |             |
                 ROS1 A         ROS1 B       ROS2 A / ROS2 B
```

The default configuration provides four example mappings:

| Robot id | ROS | rosbridge DNS | Navigation action |
| --- | --- | --- | --- |
| `jackal_ros1_a` | Noetic | `rosbridge.ros1-a.svc:9090` | `/move_base` |
| `jackal_ros1_b` | Noetic | `rosbridge.ros1-b.svc:9090` | `/move_base` |
| `turtlebot_ros2` | Jazzy | `rosbridge.ros2-a.svc:9090` | `/navigate_to_pose` |
| `turtlebot_ros2_b` | Jazzy | `rosbridge.ros2-b.svc:9090` | `/navigate_to_pose` |

These Service names are **examples**, not Services created by this package.
Edit `configmap.yaml` to match the names, namespaces, ports, ROS topic
allowlists, installed navigation actions and coordinate frames in your ROS
installations. Each ROS instance should use its own graph/ROS master or DDS
domain so identical topic/action names do not collide. For ROS 2, the Nav2
stack must be active to accept `/navigate_to_pose`; for ROS 1, a `move_base`
action server must be active. ROS 1 and ROS 2 rosbridge servers should expose
their own `rosapi` services in the respective graph.

If ROS runs on a VM outside Kubernetes, replace `host` with its reachable IP
or DNS name and allow TCP 9090 from the gateway Pod. The gateway communicates
with ROS over TCP WebSockets; it does not require DDS across namespaces.

## Install

1. Edit `deployment.yaml`: replace `registry.example.com/labvisio/is-ros-translator:v0.4.10`
   with an image available to **every Kubernetes node**.
2. Build and push that image from the **repository root** (substitute your registry):

   ```bash
   docker build -f deploy/Dockerfile -t REGISTRY/labvisio/is-ros-translator:v0.4.10 .
   docker push REGISTRY/labvisio/is-ros-translator:v0.4.10
   ```

3. Edit `configmap.yaml` with your actual ROS hostnames/ports and endpoints.
4. Edit `secret-local.yaml` with the address of your existing RabbitMQ (and
   credentials). The included values `guest/guest` and `local-e2e-api-key` are
   for a **private lab only**. RabbitMQ's default `guest` account cannot log in
   from another host/Pod unless you configure RabbitMQ to permit it; for a
   shared cluster, create a proper RabbitMQ user instead.
5. Apply and inspect:

   ```bash
   kubectl apply -k deploy/kubernetes
   kubectl -n is-ros-translator rollout status deploy/is-ros-translator
   kubectl -n is-ros-translator get pods,svc
   kubectl -n is-ros-translator logs -f deploy/is-ros-translator
   ```

6. Test the gateway without exposing its insecure lab API on the network:

   ```bash
   kubectl -n is-ros-translator port-forward svc/is-ros-translator 18080:8080
   ```

   In another terminal:

   ```bash
   curl -s http://127.0.0.1:18080/health/ready
   curl -s -H 'x-api-key: local-e2e-api-key' http://127.0.0.1:18080/v1/robots
   curl -s -H 'x-api-key: local-e2e-api-key' http://127.0.0.1:18080/v1/robots/turtlebot_ros2/capabilities
   ```

   A `connected: false` robot is **not necessarily unhealthy**: the gateway
   connects to each rosbridge lazily, upon its first request. Probe a robot:

   ```bash
   curl -s -H 'x-api-key: local-e2e-api-key' \
     -H 'content-type: application/json' \
     -d '{"robot_id":"jackal_ros1_a","operation":"ros.topics","args":{}}' \
     http://127.0.0.1:18080/v1/commands
   ```

   Test a named PrIS capability through the same command endpoint:

   ```bash
   curl -s -H 'x-api-key: local-e2e-api-key' \
     -H 'content-type: application/json' \
     -d '{"robot_id":"turtlebot_ros2","operation":"announce","args":{"data":"Hello PrIS"}}' \
     http://127.0.0.1:18080/v1/commands
   ```

The `ClusterIP` API inside Kubernetes is
`http://is-ros-translator.is-ros-translator.svc:8080`. PrIS AMQP clients
publish to RabbitMQ on `ROSTranslator.v2.Request` (generic commands) or
`ROSTranslator.v2.RobotTask.<robot_id>` (native `is-msgs` tasks).

## Operational notes

- Keep **one gateway replica** while using the existing is-wire consumer and
  process-local event subscriptions. The Deployment uses `Recreate` to prevent
  concurrent old/new consumers during rollouts. This is not a stateless HTTP
  API suitable for horizontal scaling without additional coordination.
- `/health/live` is liveness; `/health/ready` reports AMQP status in JSON but
  always returns HTTP 200, so Kubernetes uses an **exec readiness probe** that
  explicitly checks `status == ready`. The probe does not require all ROS
  installations to be online, allowing independent ROS maintenance.
- After editing ConfigMap or Secret, restart the Deployment to reload values:

  ```bash
  kubectl apply -k deploy/kubernetes
  kubectl -n is-ros-translator rollout restart deploy/is-ros-translator
  ```

- The old Docker four-robot random E2E remains in `deploy/e2e` for simulation;
  **it is not started by these manifests**. Do not send unbounded random goals
  to real robots without environment-specific safety bounds and supervision.
- For full graphical access install RViz/rviz2 and optional Gazebo separately
  alongside the corresponding ROS distributions. The translator provides
  transport, not graphical rendering.
