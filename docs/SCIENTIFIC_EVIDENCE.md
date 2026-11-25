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

## H1: optional read-only heterogeneous command diagnostic

H1 checks two *different types of operation* over the same PrIS/IS-Wire API:

| Destination | PrIS operation | Backend mechanism | Read-only check |
| --- | --- | --- | --- |
| `ros1_sim` | `get_map` | ROS 1 `/dynamic_map` service (`nav_msgs/GetMap`) | Valid `OccupancyGrid` size and resolution |
| `create_3` | `get_odometry` | ROS 2 `/create3/odom` topic (`nav_msgs/msg/Odometry`) | Valid pose structure |

```bash
python3 create3_physical_e2e.py --distinct-commands-only --seed 2027
```

H1 is automatically performed before motion under `--execute --ros1-mirror`.
It saves separate `request_id`/`correlation_id` records and reports
`H1_distinct_commands=PASS`. This diagnostic is **not** the parallel E3 test.

## E3: concurrent different navigation destinations in ROS 1 and ROS 2

E1/E2 demonstrate a *shared* semantic contract using corresponding goals,
normally executed sequentially across the backends. E3 repeats the familiar
**destination → origin** workflow for **five complete loops**, but gives
ROS 1 and ROS 2 **different destinations** and dispatches each pair of
commands concurrently. The next phase only begins after both preceding
responses have completed.

Each E3 loop follows this order:

1. ROS 1 goes to its own target **concurrently** with ROS 2 going to a
   different target.
2. Both reach a successful terminal action result.
3. ROS 1 goes to `map=(0,0)` **concurrently** with ROS 2 going to the
   corresponding floor origin, `odom=(-1.5,+1.0)`.
4. Both return actions finish before the next loop is dispatched.

This produces **ten synchronized pairs, 20 independent `navigate` requests**
and five origin returns on each backend. No positioning-accuracy requirement
is added to E3: an action's successful ROS status confirms the communication
path, not the achieved physical pose.

| Backend | PrIS operation | Loop 1 world/floor target (m) | ROS action frame and sent coordinates (m) |
| --- | --- | --- | --- |
| ROS 1 simulator (`ros1_sim`) | `navigate` | **(-0.8, +0.6)** | `map`: **(-0.8, +0.6)** via `/move_base` |
| ROS 2 physical Create 3 (`create_3`) | `navigate` | **(+0.8, -0.6)** | `odom`: **(-0.7, +0.4)** via `/create3/navigate_to_position` |

The ROS 2 odom coordinates result from the **configured** dock offset
`(1.5, -1.0)` m, not from any claim about navigation accuracy. The first
destinations are the default targets above, and loops 2–5 use deterministic
but distinct random destinations derived from `--seed`. Only the first pair
can be customised via `--parallel-ros1-x`, `--parallel-ros1-y`,
`--parallel-ros2-x` and `--parallel-ros2-y`. Every destination pair is
separated by at least 0.5 m within the configured floor bounds.

On the **Ubuntu host, outside Docker**, with Core/ROSBridge already running,
the IS-Wire broker URI configured, a physically clear area and a supervisor
holding a reliable **physical** stop control:

```bash
# Preview all five target/return pairs first (no IS-Wire, no motion).
python3 create3_physical_e2e.py --plan-parallel-goals --seed 2027

# Five E3 loops in isolation; actual Create 3 movement (undocks if necessary).
python3 create3_physical_e2e.py \
  --parallel-goals-only --confirm-area-clear --seed 2027

# Or five E3 loops after the five E1/E2 loops, before docking.
python3 create3_physical_e2e.py \
  --execute --confirm-area-clear --ros1-mirror \
  --validate-follow-waypoints --validate-parallel-goals --seed 2027
```

**The isolated E3 run leaves the Create 3 undocked.** Do not perform these
commands unattended; Create 3 NavigateToPosition does not provide obstacle
avoidance. The simulator and physical robot do not need to be collocated.
E3 tests *routing/dispatch*, not geometry or trajectory accuracy.

The client uses two worker threads and a start barrier for **each pair**, and
each worker creates a separate IS-Wire request/channel. Its `events.jsonl`
records `parallel_loop_started`, `parallel_request_started`,
`parallel_request_completed`, `parallel_pair_verified`, `parallel_loop_pass`
and `parallel_cycles_verified`, with independent IDs and monotonic start/end
times. `summary.json` includes `parallel_e3_waypoints` and
`parallel_distinct_cycles` (per loop: `target`, then `return`). It distinguishes
`E3_concurrent_dispatch=PASS` only after all ten pairs finish successfully
from `E3_backend_action_overlap=NOT_VERIFIED_FROM_CLIENT_ALONE`. **Concurrent
client RPCs alone do not establish simultaneous ROS actions.**

To corroborate E3 using *Core* evidence from the same run, execute:

```bash
python3 verify_e3_overlap.py \
  --client-run "$HOME/rostranslator-create3-results/<RUN_ID>" \
  --core-evidence "$HOME/rostranslator-evidence/<SESSION_ID>/core_bridge_events.jsonl"
```

Replace `<RUN_ID>` and `<SESSION_ID>` with actual directories displayed by
the scripts and Compose evidence mount. Verify that the Core evidence file
is **writable by the current Core UID** before the experiment. The verifier
checks **all 20** client `request_id`/`correlation_id` pairs, destination/return
coordinates and successful replies against `iswire.request_received`, `navigation_goal_prepared`,
`bridge.action_dispatched`, and each adapter's `goal_sent`/`goal_terminal`.
Using the **same Core monotonic clock**, it computes:

`overlap = min(goal_terminal_ros1, goal_terminal_ros2) - max(goal_sent_ros1, goal_sent_ros2)`

Only a **strictly positive** interval in **every one of the ten pairs**, with
both successful backend terminal statuses and the phases in order, produces
`[PASS]` and a saved `e3_overlap_verification.json`. A zero/negative interval,
missing return, overlapping successive phases, incomplete logs or ID mismatch
produces `[FAIL]`.
This is evidence of overlapping ROS action-client intervals, not necessarily
simultaneous robot motion at the millisecond level. Preserve all four client
files plus the Core JSONL for the scientific report. Do not retroactively
attribute E3 to the earlier 2026-11-25 run, which had sequential goals only.

### Observed E3 result — 2026-11-25 17:05 UTC

The Ubuntu-host run `20261125T170547Z-c28397` reported **5/5 completed
cycles and 10/10 overlapping client request pairs**, with 20 independent
navigation requests. Its server-side verifier completed at 17:08:44 UTC:

- `status=PASS`, `criterion=server_side_action_interval_overlap`
- `pairs_verified=10`, `cycles_verified=5`
- Minimum Core action interval overlap: **2506.458 ms**, across all ten pairs;
  maximum: **2521.805 ms**
- ROS 1 terminal status `3` and ROS 2 terminal status `4` for each pair
- Ordered target → return → next target transitions validated per backend

The complete `e3_overlap_verification.json` was supplied in the research
conversation and is summarised in
[`docs/EXPERIMENT_REPORT_2026-11-25.md`](EXPERIMENT_REPORT_2026-11-25.md).
The original client/Core JSONL files reside on the Ubuntu VM and have not
been published with the report. Their preservation is needed for independent
reproduction of the audit; this is **functional evidence from one E3 run**,
not a statistical reliability or performance benchmark.

## Quick checks

If `jq` is installed:

```bash
jq -c 'select(.event=="request_received" or .event=="goal_sent" or .event=="goal_terminal") | {at_utc,component,event,robot_id,request_id,waypoint_index,goal_id,terminal_status}' \
  evidence/core/core_bridge_events.jsonl
```

If the Core has not been restarted since this change, the file will not
contain action-level evidence from the previous experiment. Previous logs
cannot be retroactively enriched with missing ROS goal IDs or timestamps.
