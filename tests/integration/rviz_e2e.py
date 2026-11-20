#!/usr/bin/env python3
"""Real inter-container E2E: PrIS/is-wire -> gateway -> rosbridge -> ROS2 -> RViz.

Runs in a standalone Python container without any ROS installation. The live
ROS graph is monitored via a SECOND rosbridge WebSocket, independent of the
PrIS gateway, so an is-wire RPC reply alone cannot make this test pass.
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import random
import signal
import statistics
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections import Counter, deque
from datetime import datetime, timezone

import websocket

GATEWAY = os.getenv('GATEWAY_URL', 'http://gateway:8080').rstrip('/')
ROSBRIDGE = os.getenv('ROSBRIDGE_URL', 'ws://ros2:9090')
RVIZ = os.getenv('RVIZ_URL', 'http://ros2:6080/vnc.html')
ISWIRE_BROKER_URI = os.environ['ISWIRE_BROKER_URI']
REPORT_FILE = Path(os.getenv('REPORT_FILE', '/reports/e2e-report.json'))
ROBOT = 'turtlebot_ros2'
TOPICS = ('/pris_demo/goal', '/pris_demo/path', '/pris_demo/marker',
          '/pris_demo/state', '/odom', '/tf', '/map')
TARGETS = ((1.8, 0.8), (-1.1, 1.2))


def log(message):
    print(f'[E2E] {message}', flush=True)


def request(method, path, data=None, timeout=12):
    raw = json.dumps(data).encode() if data is not None else None
    http = urllib.request.Request(
        GATEWAY + path, method=method, data=raw,
        headers={'X-API-Key': os.getenv('GATEWAY_API_KEY', ''),
                 'Content-Type': 'application/json'},
    )
    with urllib.request.urlopen(http, timeout=timeout) as response:
        body = response.read()
        return json.loads(body.decode())


def command(operation, args=None, timeout=15):
    body = {'robot_id': ROBOT, 'operation': operation, 'args': args or {},
            'timeout_seconds': timeout, 'request_id': str(uuid.uuid4())}
    reply = request('POST', '/v1/commands', body, timeout=timeout + 7)
    if not reply.get('ok'):
        raise AssertionError(f'Gateway {operation} returned: {reply}')
    return reply['result']


def wait_for_graph(seconds=100):
    deadline = time.monotonic() + seconds
    last = ''
    while time.monotonic() < deadline:
        try:
            available = set(command('ros.topics')['topics'])
            missing = set(TOPICS) - available
            if not missing:
                return sorted(available)
            last = f'ROS graph missing topics: {sorted(missing)}'
        except Exception as exc:
            last = f'{type(exc).__name__}: {exc}'
        time.sleep(2)
    raise AssertionError(f'ROS graph not ready within {seconds}s: {last}')


def check_novnc(seconds=75):
    """RViz's Xvfb, VNC and HTTP frontend start asynchronously."""
    deadline = time.monotonic() + seconds
    error = 'not yet reachable'
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(RVIZ, timeout=5) as resp:
                body = resp.read(300000).decode(errors='ignore').lower()
                if resp.status == 200 and 'vnc' in body:
                    return True
                error = f'unexpected noVNC response: HTTP {resp.status}'
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
        time.sleep(2)
    raise AssertionError(f'noVNC did not become ready in {seconds}s: {error}')


class RosGraphObserver:
    """Observe ROS topics DIRECTLY at rosbridge, outside the gateway RPC path."""
    def __init__(self):
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.last_error = None
        self.reconnect_count = 0
        self.connected = False
        self.last_message_at = None
        self.last_path_size = 0
        self.path_last = None
        self.tf_last = None
        self.state_events = []
        self.message_count = {'/pris_demo/path': 0, '/tf': 0, '/pris_demo/state': 0}
        self.ws = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        reconnect_delay = 1.0
        while not self.stop.is_set():
            socket = None
            try:
                socket = websocket.create_connection(ROSBRIDGE, timeout=6)
                with self.lock:
                    self.ws = socket
                    self.connected = True
                    self.last_error = None
                reconnect_delay = 1.0
                socket.settimeout(1)
                for topic in self.message_count:
                    socket.send(json.dumps({'op': 'subscribe', 'id': 'pris-e2e-' + topic,
                                            'topic': topic, 'throttle_rate': 200,
                                            'queue_length': 1}))
                while not self.stop.is_set():
                    try:
                        data = json.loads(socket.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
                    if data.get('op') != 'publish':
                        continue
                    topic = data.get('topic')
                    msg = data.get('msg', {})
                    with self.lock:
                        if topic not in self.message_count:
                            continue
                        self.last_message_at = time.monotonic()
                        self.message_count[topic] += 1
                        if topic == '/pris_demo/path':
                            poses = msg.get('poses') or []
                            self.last_path_size = max(self.last_path_size, len(poses))
                            if poses:
                                self.path_last = poses[-1]['pose']['position']
                        elif topic == '/tf':
                            for transform in msg.get('transforms', []):
                                if transform.get('child_frame_id') == 'base_link':
                                    self.tf_last = transform['transform']['translation']
                        elif topic == '/pris_demo/state':
                            try:
                                self.state_events.append(json.loads(msg.get('data', '{}')))
                            except ValueError:
                                pass
                            self.state_events = self.state_events[-50:]
            except Exception as exc:
                if not self.stop.is_set():
                    with self.lock:
                        self.last_error = f'{type(exc).__name__}: {exc}'
                        self.reconnect_count += 1
                    log(f'ROS2 observer disconnected: {exc}; reconnecting')
            finally:
                with self.lock:
                    self.connected = False
                    if self.ws is socket:
                        self.ws = None
                if socket is not None:
                    try:
                        socket.close()
                    except Exception:
                        pass
            if not self.stop.is_set():
                self.stop.wait(reconnect_delay)
                reconnect_delay = min(5.0, reconnect_delay * 2.0)

    def snapshot(self):
        with self.lock:
            return {'path_samples': self.last_path_size, 'path_last': self.path_last,
                    'tf_last': self.tf_last, 'states': list(self.state_events),
                    'messages': dict(self.message_count), 'observer_error': self.last_error,
                    'observer_connected': self.connected,
                    'observer_reconnections': self.reconnect_count,
                    'last_message_age_seconds': (round(time.monotonic() - self.last_message_at, 2)
                                                 if self.last_message_at is not None else None)}

    def close(self):
        self.stop.set()
        if self.ws is not None:
            try:
                self.ws.close()
            except Exception:
                pass
        self.thread.join(timeout=3)


def near(point, target, tolerance=0.16):
    return bool(point is not None and
                math.hypot(float(point['x']) - target[0],
                           float(point['y']) - target[1]) <= tolerance)


def wait_for_observer(observer, seconds=20):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        sample = observer.snapshot()
        if (sample['path_samples'] >= 1 and sample['tf_last'] is not None and
                sample['observer_connected'] and
                sample['last_message_age_seconds'] is not None and
                sample['last_message_age_seconds'] < 5):
            return
        time.sleep(0.25)
    raise AssertionError('ROS2 rosbridge never delivered Path and TF: ' + str(observer.snapshot()))


def connect_iswire_channel(seconds=120, retry_seconds=2.0):
    """Bounded retry for AMQP startup races; only the native is-wire SDK is used."""
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
            log(f'Waiting for is-wire AMQP broker ({type(exc).__name__}): {exc}')
            time.sleep(min(retry_seconds, remaining))


def wire_rpc(command_payload, timeout_seconds=30):
    """Native Protobuf Struct RPC using is-wire; preserves correlation id."""
    from is_wire.core import Message, Subscription
    from google.protobuf.json_format import ParseDict, MessageToDict
    from google.protobuf.struct_pb2 import Struct
    import socket

    channel = connect_iswire_channel()
    try:
        reply_subscription = Subscription(channel)
        data = Struct()
        ParseDict(command_payload, data)
        outgoing = Message(content=data, reply_to=reply_subscription)
        channel.publish(outgoing, topic='ROSTranslator.v2.Request')
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            try:
                incoming = channel.consume(timeout=max(0.1, deadline - time.monotonic()))
            except socket.timeout:
                break
            if incoming.correlation_id == outgoing.correlation_id:
                response = MessageToDict(incoming.unpack(Struct), preserving_proto_field_name=True)
                if not response.get('ok'):
                    raise AssertionError('PrIS is-wire RPC failed: ' + str(response))
                return response
        raise AssertionError('No correlated is-wire Struct RPC reply')
    finally:
        channel.close()


def robot_task_rpc(x, y):
    """Standard is-msgs RobotTaskRequest through is-wire, not ROS-specific types."""
    import socket
    from is_wire.core import Message, Subscription
    from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply

    channel = connect_iswire_channel()
    try:
        sub = Subscription(channel)
        task = RobotTaskRequest()
        task.id = uuid.uuid4().int & ((1 << 64) - 1)
        waypoint = task.basic_move_task.positions.add()
        waypoint.x, waypoint.y, waypoint.z = x, y, 0.0
        task.basic_move_task.allowed_error = 0.15
        task.basic_move_task.rate = 10.0
        request_message = Message(content=task, reply_to=sub)
        channel.publish(request_message, topic='ROSTranslator.v2.RobotTask.' + ROBOT)
        deadline = time.monotonic() + 50
        while time.monotonic() < deadline:
            try:
                reply = channel.consume(timeout=max(0.1, deadline - time.monotonic()))
            except socket.timeout:
                break
            if reply.correlation_id != request_message.correlation_id:
                continue
            if reply.status is not None and not reply.status.ok():
                raise AssertionError('RobotTask RPC status: ' + str(reply.status))
            result = reply.unpack(RobotTaskReply)
            if result.id != task.id:
                raise AssertionError('is-msgs RobotTaskReply.id did not match request')
            return {'ok': True, 'request_id': str(task.id), 'result': {'navigation_completed': True}}
        raise AssertionError('No correlated is-msgs RobotTaskReply')
    finally:
        channel.close()


def publish_goal_via_pris_iswire(x, y):
    # This is a ROS2 PoseStamped, but the PrIS-facing transport only handles JSON.
    payload = {
        'robot_id': ROBOT,
        'operation': 'ros.publish',
        'request_id': str(uuid.uuid4()),
        'args': {
            'endpoint': '/pris_demo/goal',
            'ros_type': 'geometry_msgs/msg/PoseStamped',
            'payload': {
                'header': {'frame_id': 'map'},
                'pose': {
                    'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
            },
        },
    }
    return wire_rpc(payload)


def get_position():
    odom = command('get_odometry', timeout=8)
    return odom['pose']['pose']['position']


def wait_for_goal(observer, target, seconds=35):
    deadline = time.monotonic() + seconds
    last_odom = None
    while time.monotonic() < deadline:
        sample = observer.snapshot()
        if (sample['observer_connected'] and sample['last_message_age_seconds'] is not None
                and sample['last_message_age_seconds'] < 5 and
                near(sample['path_last'], target) and near(sample['tf_last'], target)):
            try:
                last_odom = get_position()
                if near(last_odom, target):
                    return last_odom, sample
            except Exception:
                pass
        time.sleep(0.35)
    raise AssertionError('Robot did not reach target %s; odom=%s; observer=%s' %
                         (target, last_odom, observer.snapshot()))


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def environment_float(name, default, minimum=0, maximum=300):
    value = float(os.getenv(name, str(default)))
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be between {minimum} and {maximum}: {value}')
    return value


def random_goal(rng, current_position=None, limit=2.5, minimum_distance=0.8):
    """Generate non-trivial, reproducible goals inside the ROS demo's safe area.

    This only applies to the kinematic demo: there is no collision avoidance.
    """
    for _ in range(100):
        goal = (round(rng.uniform(-limit, limit), 2),
                round(rng.uniform(-limit, limit), 2))
        if current_position is None or not near(current_position, goal, minimum_distance):
            return goal
    raise RuntimeError('Cannot select a distinct random target in demo bounds')


def loop_request(operation, args=None, timeout_seconds=45):
    """Send a semantic or generic command exclusively via the native PrIS SDK."""
    return wire_rpc({
        'robot_id': ROBOT,
        'request_id': str(uuid.uuid4()),
        'operation': operation,
        'args': args or {},
        'timeout_seconds': timeout_seconds,
    }, timeout_seconds=timeout_seconds + 10)


def run_random_command(rng, current_position, limit, operation=None, target=None):
    """Select a varied PrIS transport / ROS2 command and its final destination."""
    operation = operation or rng.choice(('ros.publish goal', 'is-msgs RobotTask',
                                         'navigate', 'follow_waypoints'))
    target = target or random_goal(rng, current_position, limit)
    if operation == 'ros.publish goal':
        response = publish_goal_via_pris_iswire(*target)
        if not response.get('result', {}).get('published'):
            raise AssertionError('ROS2 goal was not published: ' + str(response))
    elif operation == 'is-msgs RobotTask':
        response = robot_task_rpc(*target)
        if not response.get('result', {}).get('navigation_completed'):
            raise AssertionError('RobotTask was not completed: ' + str(response))
    elif operation == 'navigate':
        response = loop_request('navigate', {'position': {'x': target[0], 'y': target[1]}},
                                timeout_seconds=50)
    else:
        intermediate = random_goal(rng, current_position, limit)
        response = loop_request('follow_waypoints', {'waypoints': [
            {'position': {'x': intermediate[0], 'y': intermediate[1]}},
            {'position': {'x': target[0], 'y': target[1]}},
        ]}, timeout_seconds=90)
        if response.get('result', {}).get('completed_waypoints') != 2:
            raise AssertionError('Two ROS2 waypoints were not completed: ' + str(response))
    return operation, target, response


def run_random_probe(rng):
    """Exercise additional gateway operations between movement requests."""
    operation = rng.choice(('ros.topics', 'ros.services', 'get_odometry',
                            'ros.publish chatter'))
    if operation == 'ros.publish chatter':
        response = loop_request('ros.publish', {
            'endpoint': '/chatter', 'ros_type': 'std_msgs/msg/String',
            'payload': {'data': 'PrIS continuous E2E ' + str(uuid.uuid4())},
        })
        if not response.get('result', {}).get('published'):
            raise AssertionError('Chatter was not published: ' + str(response))
    else:
        response = loop_request(operation)
    return operation, response


def atomic_report_write(path, report):
    """Keep the latest report valid while the tester is still running."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str) + '\n')
    temporary.replace(path)


def append_bounded_history(path, entry, max_bytes):
    """Rotate one history file to avoid growing forever on an unattended VM."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.stat().st_size >= max_bytes:
        path.replace(path.with_name(path.name + '.1'))
    with path.open('a') as output:
        output.write(json.dumps(entry, ensure_ascii=False, default=str) + '\n')


def run_loop():
    """Run varied random E2E commands until SIGINT/SIGTERM, without failing closed."""
    report_path = Path(os.getenv('E2E_LOOP_REPORT_FILE', '/reports/e2e-loop-report.json'))
    history_path = Path(os.getenv('E2E_LOOP_HISTORY_FILE', '/reports/e2e-loop-history.jsonl'))
    report = {
        'started_at_utc': utc_now(),
        'flow': 'is-wire/is-msgs -> PrIS gateway -> ROS2 rosbridge -> simulated robot -> RViz',
        'mode': 'continuous_random', 'status': 'starting',
        'attempts': 0, 'passed': 0, 'failed': 0, 'operations': {}, 'recent': [],
    }
    stopped = threading.Event()
    original_handlers = {}
    observer = None

    def stop_handler(signum, _frame):
        log(f'Received signal {signum}: stopping after the active request')
        stopped.set()

    for signum in (signal.SIGTERM, signal.SIGINT):
        original_handlers[signum] = signal.signal(signum, stop_handler)
    try:
        interval = environment_float('E2E_LOOP_INTERVAL_SECONDS', 1.0, 0, 60)
        retry_delay = environment_float('E2E_LOOP_RETRY_DELAY_SECONDS', 5.0, 0.5, 60)
        limit = environment_float('E2E_LOOP_GOAL_LIMIT', 2.5, 1.0, 3.5)
        history_max_bytes = int(os.getenv('E2E_LOOP_HISTORY_MAX_BYTES', '10485760'))
        if not 1024 <= history_max_bytes <= 100_000_000:
            raise ValueError('E2E_LOOP_HISTORY_MAX_BYTES must be between 1024 and 100000000')
        seed = os.getenv('E2E_LOOP_SEED', '').strip()
        rng = random.Random(seed if seed else None)
        report['seed'] = seed or 'random'
        report['goal_limit'] = limit
        report['interval_seconds'] = interval
        durations = deque(maxlen=100)
        operation_stats = Counter()
        failure_stats = Counter()
        recent = deque(maxlen=30)

        log('Waiting for ROS2 graph and RViz before entering continuous random E2E...')
        report['available_topics'] = wait_for_graph()
        report['novnc_available'] = check_novnc()
        observer = RosGraphObserver()
        observer.start()
        wait_for_observer(observer)
        report['status'] = 'running'
        atomic_report_write(report_path, report)
        log(f'Running indefinitely; goals in [-{limit}, {limit}], ' 
            f'interval={interval}s; Ctrl+C or "loop-stop" to stop')

        while not stopped.is_set():
            started = time.monotonic()
            entry = {'timestamp_utc': utc_now(), 'iteration': report['attempts'] + 1}
            try:
                current = get_position()
                operation = rng.choice(('ros.publish goal', 'is-msgs RobotTask',
                                        'navigate', 'follow_waypoints'))
                goal = random_goal(rng, current, limit)
                entry['operation'] = operation
                entry['target'] = list(goal)
                operation, goal, response = run_random_command(rng, current, limit,
                                                               operation=operation, target=goal)
                entry['request_id'] = response.get('request_id')
                odom, observation = wait_for_goal(observer, goal, seconds=45)
                if observation['path_samples'] < 4:
                    raise AssertionError('ROS2 Path has fewer than four samples')
                entry['odometry'] = odom
                entry['tf'] = observation['tf_last']
                entry['path_points'] = observation['path_samples']

                # Probe read operations and occasionally publish additional
                # non-navigation traffic over the same PrIS is-wire contract.
                probe, _probe_response = run_random_probe(rng)
                entry['probe'] = probe
                entry['ok'] = True
                report['passed'] += 1
                operation_stats[operation] += 1
                operation_stats[probe] += 1
                log(f'PASS #{entry["iteration"]}: {operation} -> {goal}; '
                    f'probe={probe}; odom=({odom["x"]:.2f},{odom["y"]:.2f})')
            except Exception as exc:
                entry['ok'] = False
                entry['error'] = f'{type(exc).__name__}: {exc}'
                report['failed'] += 1
                failure_stats[entry.get('operation', 'setup')] += 1
                log(f'FAIL #{entry["iteration"]}: {entry["error"]} (will retry)')

            entry['duration_seconds'] = round(time.monotonic() - started, 3)
            durations.append(entry['duration_seconds'])
            report['attempts'] += 1
            report['operations'] = dict(operation_stats)
            report['failures_by_operation'] = dict(failure_stats)
            recent.append(entry)
            report['recent'] = list(recent)
            report['last_iteration_at_utc'] = utc_now()
            report['last_observer'] = observer.snapshot()
            report['mean_duration_seconds_last_100'] = round(statistics.mean(durations), 3)
            append_bounded_history(history_path, entry, history_max_bytes)
            atomic_report_write(report_path, report)
            stopped.wait(interval if entry['ok'] else retry_delay)

        report['status'] = 'stopped'
        report['stopped_at_utc'] = utc_now()
        log(f'Stopped: {report["passed"]} passed, {report["failed"]} failed')
        return 0
    except Exception as exc:
        report['status'] = 'failed'
        report['fatal_error'] = f'{type(exc).__name__}: {exc}'
        log(f'Continuous E2E startup failed: {report["fatal_error"]}')
        return 1
    finally:
        if observer is not None:
            report['last_observer'] = observer.snapshot()
            observer.close()
        atomic_report_write(report_path, report)
        for signum, handler in original_handlers.items():
            signal.signal(signum, handler)


def run():
    report = {'started_at_utc': __import__('datetime').datetime.now(
        __import__('datetime').timezone.utc).isoformat(),
              'flow': 'is-wire/is-msgs client -> PrIS gateway -> ROS2 rosbridge -> simulated robot -> RViz',
              'targets': [], 'status': 'failed'}
    observer = None
    try:
        log('Waiting for the live ROS2 graph through the gateway...')
        topics = wait_for_graph()
        report['available_topics'] = topics
        log('PASS: ROS2 graph published all required RViz topics')

        check_novnc()
        log('PASS: noVNC web interface is reachable on ROS2 shared network')
        report['novnc_available'] = True

        observer = RosGraphObserver()
        observer.start()
        wait_for_observer(observer)
        log('PASS: independent rosbridge WebSocket sees ROS2 Path and TF')

        for index, target in enumerate(TARGETS, 1):
            log(f'[{index}/{len(TARGETS)}] is-wire -> gateway -> ROS2 goal {target}')
            response = publish_goal_via_pris_iswire(*target) if index == 1 else robot_task_rpc(*target)
            outcome = 'published' if index == 1 else 'navigation_completed'
            if not response.get('result', {}).get(outcome):
                raise AssertionError(f'Gateway did not confirm {outcome}: {response}')
            odom, sample = wait_for_goal(observer, target)
            if sample['path_samples'] < 4:
                raise AssertionError('Path publisher did not record the movement')
            log('PASS: odom=(%.3f,%.3f), TF=(%.3f,%.3f), path=%d points' %
                (odom['x'], odom['y'], sample['tf_last']['x'],
                 sample['tf_last']['y'], sample['path_samples']))
            report['targets'].append({'requested': list(target), 'odometry': odom,
                                      'path_points': sample['path_samples'],
                                      'last_path_position': sample['path_last'],
                                      'last_tf_translation': sample['tf_last'],
                                      'iswire_request_id': response['request_id'],
                                      'contract': 'Struct' if index == 1 else 'is-msgs RobotTaskRequest/RobotTaskReply'})

        observed = observer.snapshot()
        if not observed['states'] or not any(e.get('state') == 'goal_received'
                                                 for e in observed['states']):
            raise AssertionError('No goal_received event observed on ROS2 state topic')
        report['observed_graph'] = observed
        log('PASS: ROS2 confirmed goal reception using a separate state topic')

        if os.getenv('E2E_ACTION_TEST', 'false').lower() in ('1', 'yes', 'true'):
            log('Additional semantic NavigateToPose action test...')
            nav_result = command('navigate', {'position': {'x': 0.0, 'y': 0.0}}, timeout=40)
            odom, sample = wait_for_goal(observer, (0.0, 0.0), 25)
            report['semantic_action'] = {'result': nav_result, 'odometry': odom,
                                         'path_points': sample['path_samples']}
            log('PASS: ROS2 NavigateToPose action and odometry')

        report['status'] = 'passed'
        log('E2E PASSED: is-wire Struct + is-msgs RobotTask RPC, real ROS2 movement, Path, TF, odometry, noVNC')
        return 0
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        log(f'E2E FAILED: {report["error"]}')
        return 1
    finally:
        if observer is not None:
            report['last_observer'] = observer.snapshot()
            observer.close()
        REPORT_FILE.parent.mkdir(parents=True, exist_ok=True)
        REPORT_FILE.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str) + '\n')
        log(f'Report: {REPORT_FILE}')


if __name__ == '__main__':
    if os.getenv('E2E_LOOP', 'false').lower() in ('1', 'yes', 'true'):
        raise SystemExit(run_loop())
    raise SystemExit(run())
