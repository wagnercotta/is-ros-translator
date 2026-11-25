# Physical Create 3 and ROS 1 RViz mirror

This is an optional extension to the existing Create 3 lab. The ROS Translator
Core stays a translation/router service. The E2E **client** sends the same floor
waypoints separately, using `robot_id=create_3` (ROS 2) and `robot_id=ros1_sim`
(ROS 1). There is no ROS 1-to-ROS 2 transport or cross-robot logic in Core.

The ROS 1 backend is a simple simulated `move_base` action server, not a
physical robot or a navigation planner. It publishes `/odom`, `/map`,
`/ros1_sim/path` and `/ros1_sim/goal_marker` for ROS 1 RViz.

## Setup (after the current physical experiment has finished)

Prepare a local environment file:

```bash
cp deploy/lab/.env.example deploy/lab/.env
```

Set the RabbitMQ password, API key and `LAB_PI_WIFI_IP` to your real values.
Uncomment `LAB_CORE_CONFIG=../../configs/create3-ros1-lab.yaml` in the local
`.env` file when running with the ROS 1 profile. The ROS 2 bridge image
automatically replaces the `PI_WIFI_IP` placeholder in the tracked
`configs/fastdds-unicast.example.xml` during its build; no generated XML needs
to be copied from an older VM or committed to GitHub.

From the repository root:

```bash
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml \
  --profile ros1 build ros1-lab core
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml \
  --profile ros1 up -d --no-deps ros1-lab
docker compose --env-file deploy/lab/.env -f deploy/lab/compose.yaml \
  --profile ros1 up -d --no-deps --force-recreate core
docker exec rostranslator-ros1-lab /opt/ros1-lab/health.sh
```

RViz (noVNC): `http://127.0.0.1:6081/vnc.html`. For a remote VM,
forward `6081` using SSH. The ROS 1 RosBridge listens on
`127.0.0.1:9091`; ROS 2 continues on `127.0.0.1:9090`.

Use the versioned physical client at the repository root and inspect its dry run:

```bash
python3 create3_physical_e2e.py --seed 2027 --ros1-mirror
```

Only after clearing the physical travel and docking area, and ensuring a real
manual stop is available:

```bash
python3 create3_physical_e2e.py --execute --confirm-area-clear --ros1-mirror
```

The ROS 1 simulator receives the same **world XY target** in `map` that the
ROS 2 Create 3 receives after translating to `odom`:

- ROS 2 `odom=(world_x-1.5, world_y+1.0)`.
- ROS 1 `map=(world_x, world_y)`.

For physical safety, the client completes and checks the ROS 1 simulated goal
before sending its counterpart to Create 3. It is a mirrored **sequence**,
not concurrent robot execution. The dock approach is also mirrored as a waypoint
20 cm before the station; only the real Create 3 receives the final `dock` action.

## Persistent Core/bridge evidence

To enable scientific evidence, make a dedicated host directory writable by
the Core container user (UID 10001), and set the variable below before Compose:

```bash
export LAB_EVIDENCE_HOST_DIR="$HOME/rostranslator-evidence"
```

Then include the tracked overlay in the usual Compose command:

```bash
docker compose --env-file deploy/lab/.env \
  -f deploy/lab/compose.yaml -f deploy/lab/compose.evidence.yaml \
  --profile ros1 up -d --build
```

The resulting file is `$LAB_EVIDENCE_HOST_DIR/core_bridge_events.jsonl`.
See `docs/SCIENTIFIC_EVIDENCE.md` for request IDs, goal IDs, ordering and
evidence verification. The physical client has an optional
`--validate-follow-waypoints` mode for E2 in its fifth loop.
