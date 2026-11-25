# Core/bridge evidence for the ROS 1 / ROS 2 experiments

The gateway can save **actual** IS-Wire, Core, bridge and action-adapter events
to one append-only JSONL file. This instrumentation is disabled unless the
environment variable below is set **before the gateway starts**:

```bash
export ROSTRANSLATOR_EVIDENCE_DIR="$PWD/evidence/core"
```

Start the existing gateway as usual, from this instrumented source version.
The file is created automatically at:

```text
evidence/core/core_bridge_events.jsonl
```

Directories and the file are created with restrictive permissions (0700 and
0600 respectively). For Docker or Kubernetes, pass the variable to the Core
container and mount a persistent writable directory at the specified path.
**Do not configure the evidence path to a temporary container filesystem** if
the results must survive a restart. Do not publish the raw evidence on a
public website without reviewing it; goals can reveal physical coordinates.

The evidence writer intentionally does not capture the broker address,
credentials, HTTP headers, arbitrary `ros.service` payloads or other
application data. Writes are thread-safe, and an evidence file I/O error
generates a warning but cannot fail a robot action. Check the file exists
and contains all expected records before concluding an experiment.

## Recorded events

| Component | Event | What it proves |
| --- | --- | --- |
| `iswire` | `request_received`, `reply_prepared` | The Core received a request/replied, with client correlation ID and request ID |
| `core` | `navigation_goal_prepared` | The configured adapter, action type, endpoint and translated goal |
| `bridge` | `action_dispatched`, `action_completed`/`action_failed` | The actual ROSBridge call and outcome |
| `ros1_adapter` | `goal_sent`, `goal_terminal`/`goal_timeout` | ROS 1 actionlib goal ID (when provided), server result/status and time |
| `ros2_adapter` | `goal_sent`, `goal_terminal`/`goal_timeout` | ROS 2 goal ID, server result/status and time |
| `core` | `sequence_started`, `sequence_step_started`, `sequence_step_completed`, `sequence_completed` | The Core sequenced the two goals within one `follow_waypoints` request |

Each event has `at_utc` and `monotonic_ns`. Every E1/E2 event is matched
through `request_id`; the IS-Wire receive/reply records also carry
`correlation_id`. In E2, `waypoint_index=0` must finish before
`waypoint_index=1` starts, using the server-side monotonic timestamps.

Both ROS versions now return their actual `goal_id` in a successful action
result when supplied by the underlying client. The ROS 1 client (`roslibpy`)
normally provides `Goal.goal_id`; if another version does not, the field is
left missing rather than synthesized.

## Run and preserve the experiment

With Core running with tracing enabled, run the physical client on the
supervised laboratory machine:

```bash
python3 create3_physical_e2e.py --execute --confirm-area-clear \
    --ros1-mirror --validate-follow-waypoints --seed 2027
```

The client creates `events.jsonl`, `wire_requests.jsonl`, and `summary.json`
under its own run directory. Preserve those three files together with the
Core's `core_bridge_events.jsonl`, the Git commit SHA, any uncommitted diff,
Core/ROSBridge container logs, and relevant software versions.

To link records, compare the client's `wire_requests.jsonl` `request.request_id`
and `correlation_id` with Core `request_id` and `iswire.request_received`
`correlation_id`. Then follow the same ID through Core/bridge/adapter events.

For E2, a strong functional result includes:

1. Exactly **one** client request `follow_waypoints` per backend containing
   two goal positions and one `request_id`.
2. One `iswire.request_received`, one Core `sequence_started`, and two
   `sequence_step_started` records with indices `0,1`.
3. Two bridge dispatches and two adapter `goal_sent`/`goal_terminal` pairs,
   with real goal IDs and successful ROS terminal statuses (3 for ROS 1,
   4 for ROS 2).
4. The first goal's `sequence_step_completed` timestamp preceding the second
   goal's `sequence_step_started` timestamp.
5. Core `sequence_completed` with 2 goals, correlated PrIS reply with
   `completed_waypoints=2`, and externally observed final odometry.

This proves the implemented order of ROS task dispatch and completion. It
does not prove simultaneous multi-robot operation, autonomous obstacle
avoidance, SLAM, or direct ROS 1 ↔ ROS 2 communication. It also does not
replace a physical operator's safety supervision.

## E3: distinct commands to different ROS backends

E1 and E2 use the same semantic navigation contract on ROS 1 and ROS 2. E3
complements them by exercising **different PrIS operations and ROS mechanisms**
through the same IS-Wire request channel:

| Destination | PrIS operation | Backend mechanism | Read-only response check |
| --- | --- | --- | --- |
| `ros1_sim` | `get_map` | ROS 1 `/dynamic_map` service (`nav_msgs/GetMap`) | `OccupancyGrid` dimensions, resolution and cell count |
| `create_3` | `get_odometry` | ROS 2 `/create3/odom` topic (`nav_msgs/msg/Odometry`) | Valid pose message structure |

The requests have separate `robot_id`, `request_id` and IS-Wire
`correlation_id`. The client saves both complete request/reply envelopes in
`wire_requests.jsonl`, and a compact `distinct_commands_verified` event in
`events.jsonl`. Server-side `request_received` and `reply_prepared` can be
matched by those IDs in `core_bridge_events.jsonl`.

Run E3 alone **without moving the robot** from the Ubuntu VM, using the same
`ISWIRE_BROKER_URI` as the other client modes:

```bash
python3 create3_physical_e2e.py --distinct-commands-only --seed 2027
```

When using `--execute --ros1-mirror`, E3 also runs automatically **before**
any undock or navigation action. The experimental output reports
`E3_distinct_commands=PASS` only after valid responses from both backends.

E3 demonstrates heterogeneous operation dispatch to distinct ROS versions,
not direct ROS 1 ↔ ROS 2 communication. It is a functional check, not a
measurement of map quality, localisation accuracy or network performance.

## Quick checks

If `jq` is installed:

```bash
jq -c 'select(.event=="request_received" or .event=="goal_sent" or .event=="goal_terminal") | {at_utc,component,event,robot_id,request_id,waypoint_index,goal_id,terminal_status}' \
  evidence/core/core_bridge_events.jsonl
```

If the Core has not been restarted since this change, the file will not
contain action-level evidence from the previous experiment. Previous logs
cannot be retroactively enriched with missing ROS goal IDs or timestamps.
