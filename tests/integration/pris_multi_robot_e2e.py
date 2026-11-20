#!/usr/bin/env python3
"""Independent native is-wire E2E workers for two ROS 1 and two ROS 2 robots."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import random
import signal
import socket
import statistics
import threading
import time
import urllib.request
import uuid
from collections import Counter, deque
from datetime import datetime, timezone

import websocket


BROKER = 'amqp://guest:guest@rabbitmq:5672/'
REQUEST_TOPIC = 'ROSTranslator.v2.Request'
RVIZ_URL = 'http://ros2:6080/vnc.html'
ROBOTS = {
    'jackal_ros1_a': {'version': 1, 'host': 'ros1a', 'topics': ('/odom', '/chatter')},
    'jackal_ros1_b': {'version': 1, 'host': 'ros1b', 'topics': ('/odom', '/chatter')},
    'turtlebot_ros2': {'version': 2, 'host': 'ros2',
                       'topics': ('/odom', '/tf', '/pris_demo/path', '/pris_demo/state', '/chatter')},
    'turtlebot_ros2_b': {'version': 2, 'host': 'ros2b',
                         'topics': ('/odom', '/tf', '/pris_demo/path', '/pris_demo/state', '/chatter')},
}


def log(robot, message):
    print(f'[E2E][{robot}] {message}', flush=True)


def now_utc():
    return datetime.now(timezone.utc).isoformat()


def position_near(position, goal, tolerance=0.18):
    return (position is not None and
            math.hypot(float(position['x']) - goal[0], float(position['y']) - goal[1]) <= tolerance)


def random_goal(rng, current, limit=2.5, minimum_distance=0.8):
    for _ in range(100):
        goal = (round(rng.uniform(-limit, limit), 2),
                round(rng.uniform(-limit, limit), 2))
        if current is None or not position_near(current, goal, minimum_distance):
            return goal
    raise AssertionError('Could not generate an adequately separated random destination')


def channel_with_retry(seconds=60):
    from is_wire.core import Channel
    deadline = time.monotonic() + seconds
    while True:
        try:
            return Channel(BROKER)
        except Exception:
            if time.monotonic() >= deadline:
                raise
            time.sleep(1)


def struct_rpc(robot, operation, args=None, timeout=60):
    from is_wire.core import Message, Subscription
    from google.protobuf.struct_pb2 import Struct
    from google.protobuf.json_format import MessageToDict, ParseDict

    channel = channel_with_retry()
    try:
        reply_to = Subscription(channel)
        payload = Struct()
        ParseDict({'robot_id': robot, 'operation': operation, 'args': args or {},
                   'request_id': str(uuid.uuid4()), 'timeout_seconds': timeout}, payload)
        outgoing = Message(content=payload, reply_to=reply_to)
        channel.publish(outgoing, topic=REQUEST_TOPIC)
        deadline = time.monotonic() + timeout + 12
        while time.monotonic() < deadline:
            try:
                incoming = channel.consume(timeout=max(0.1, deadline - time.monotonic()))
            except socket.timeout:
                break
            if incoming is None or incoming.correlation_id != outgoing.correlation_id:
                continue
            if incoming.status is not None and not incoming.status.ok():
                raise AssertionError(f'is-wire status for {robot}/{operation}: {incoming.status}')
            answer = MessageToDict(incoming.unpack(Struct), preserving_proto_field_name=True)
            if not answer.get('ok'):
                raise AssertionError(f'PrIS {robot}/{operation} failed: {answer}')
            return answer
        raise TimeoutError(f'No correlated is-wire reply for {robot}/{operation}')
    finally:
        channel.close()


def robot_task_rpc(robot, goal):
    from is_wire.core import Message, Subscription
    from is_msgs.robot_pb2 import RobotTaskRequest, RobotTaskReply

    channel = channel_with_retry()
    try:
        sub = Subscription(channel)
        task = RobotTaskRequest()
        task.id = uuid.uuid4().int & ((1 << 64) - 1)
        waypoint = task.basic_move_task.positions.add()
        waypoint.x, waypoint.y, waypoint.z = float(goal[0]), float(goal[1]), 0.0
        task.basic_move_task.allowed_error = 0.15
        task.basic_move_task.rate = 10.0
        outgoing = Message(content=task, reply_to=sub)
        channel.publish(outgoing, topic='ROSTranslator.v2.RobotTask.' + robot)
        deadline = time.monotonic() + 65
        while time.monotonic() < deadline:
            try:
                incoming = channel.consume(timeout=max(0.1, deadline - time.monotonic()))
            except socket.timeout:
                break
            if incoming is None or incoming.correlation_id != outgoing.correlation_id:
                continue
            if incoming.status is not None and not incoming.status.ok():
                raise AssertionError(f'is-msgs RobotTask failed for {robot}: {incoming.status}')
            result = incoming.unpack(RobotTaskReply)
            if result.id != task.id:
                raise AssertionError('RobotTaskReply.id does not match the request')
            return {'request_id': str(task.id), 'result': {'navigation_completed': True}}
        raise TimeoutError(f'No correlated is-msgs RobotTaskReply for {robot}')
    finally:
        channel.close()


class RobotObserver:
    def __init__(self, robot, config):
        self.robot = robot
        self.config = config
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.connection = None
        self.connected = False
        self.last_error = None
        self.reconnections = 0
        self.last_message_at = None
        self.odom = None
        self.path = None
        self.tf = None
        self.path_size = 0
        self.message_counts = Counter()
        self.thread = threading.Thread(target=self.run, name='observer-' + robot, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        delay = 1.0
        while not self.stop.is_set():
            connection = None
            try:
                connection = websocket.create_connection(
                    f'ws://{self.config["host"]}:9090', timeout=6)
                connection.settimeout(1)
                topics = ['/odom']
                if self.config['version'] == 2:
                    topics.extend(('/tf', '/pris_demo/path'))
                for topic in topics:
                    connection.send(json.dumps({'op': 'subscribe',
                        'id': f'{self.robot}-{topic}', 'topic': topic,
                        'throttle_rate': 200, 'queue_length': 1}))
                with self.lock:
                    self.connection = connection
                    self.connected = True
                    self.last_error = None
                delay = 1.0
                while not self.stop.is_set():
                    try:
                        message = json.loads(connection.recv())
                    except websocket.WebSocketTimeoutException:
                        continue
                    if message.get('op') != 'publish':
                        continue
                    topic = message.get('topic')
                    data = message.get('msg', {})
                    with self.lock:
                        self.message_counts[topic] += 1
                        self.last_message_at = time.monotonic()
                        if topic == '/odom':
                            self.odom = data.get('pose', {}).get('pose', {}).get('position')
                        elif topic == '/pris_demo/path':
                            poses = data.get('poses') or []
                            self.path_size = max(self.path_size, len(poses))
                            if poses:
                                self.path = poses[-1]['pose']['position']
                        elif topic == '/tf':
                            for transform in data.get('transforms', []):
                                if transform.get('child_frame_id') == 'base_link':
                                    self.tf = transform['transform']['translation']
            except Exception as exc:
                if not self.stop.is_set():
                    with self.lock:
                        self.last_error = f'{type(exc).__name__}: {exc}'
                        self.reconnections += 1
                    log(self.robot, f'observer disconnected: {exc}; reconnecting')
            finally:
                with self.lock:
                    self.connected = False
                    if self.connection is connection:
                        self.connection = None
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
            if not self.stop.is_set():
                self.stop.wait(delay)
                delay = min(5.0, 2 * delay)

    def snapshot(self):
        with self.lock:
            age = None if self.last_message_at is None else round(
                time.monotonic() - self.last_message_at, 2)
            return {'connected': self.connected, 'error': self.last_error,
                    'reconnections': self.reconnections, 'age_seconds': age,
                    'odom': self.odom, 'path': self.path, 'tf': self.tf,
                    'path_samples': self.path_size, 'messages': dict(self.message_counts)}

    def close(self):
        self.stop.set()
        with self.lock:
            connection = self.connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        self.thread.join(timeout=3)


def wait_for_graph(robot, config, seconds=90):
    deadline = time.monotonic() + seconds
    error = 'waiting for topics'
    while time.monotonic() < deadline:
        try:
            reply = struct_rpc(robot, 'ros.topics', timeout=12)
            available = set(reply['result']['topics'])
            missing = set(config['topics']) - available
            if not missing:
                return sorted(available)
            error = f'missing topics: {sorted(missing)}'
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
        time.sleep(2)
    raise AssertionError(f'ROS graph for {robot} unavailable: {error}')


def wait_for_observer(observer, config, seconds=30):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        sample = observer.snapshot()
        if (sample['connected'] and sample['age_seconds'] is not None and
                sample['age_seconds'] < 5 and sample['odom'] is not None and
                (config['version'] == 1 or
                 (sample['path'] is not None and sample['tf'] is not None))):
            return sample
        time.sleep(0.25)
    raise AssertionError(f'Observer {observer.robot} not ready: {observer.snapshot()}')


def wire_odometry(robot):
    answer = struct_rpc(robot, 'get_odometry', timeout=12)
    return answer['result']['pose']['pose']['position']


def wait_for_goal(robot, config, observer, goal, seconds=60):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        sample = observer.snapshot()
        if (sample['connected'] and sample['age_seconds'] is not None and
                sample['age_seconds'] < 5 and position_near(sample['odom'], goal) and
                (config['version'] == 1 or
                 (position_near(sample['path'], goal) and position_near(sample['tf'], goal)))):
            position = wire_odometry(robot)
            if position_near(position, goal):
                return position, sample
        time.sleep(0.3)
    raise AssertionError(f'{robot} did not reach {goal}; observer={observer.snapshot()}')


def available_operations(config, action_only=False):
    operations = ['is-msgs RobotTask', 'navigate', 'follow_waypoints']
    if config['version'] == 2:
        operations.append('ros.publish goal')
    if action_only:
        operations = ['navigate', 'follow_waypoints']
    return operations


def send_random_movement(robot, config, rng, current, limit, action_only=False,
                         operation=None, goal=None):
    operation = operation or rng.choice(available_operations(config, action_only))
    goal = goal or random_goal(rng, current, limit)
    if operation == 'is-msgs RobotTask':
        response = robot_task_rpc(robot, goal)
    elif operation == 'navigate':
        response = struct_rpc(robot, 'navigate',
            {'position': {'x': goal[0], 'y': goal[1]}}, timeout=60)
    elif operation == 'follow_waypoints':
        midpoint = random_goal(rng, current, limit)
        response = struct_rpc(robot, 'follow_waypoints', {
            'waypoints': [
                {'position': {'x': midpoint[0], 'y': midpoint[1]}},
                {'position': {'x': goal[0], 'y': goal[1]}},
            ]}, timeout=110)
        if response.get('result', {}).get('completed_waypoints') != 2:
            raise AssertionError(f'Only some waypoints were completed: {response}')
    else:
        response = struct_rpc(robot, 'ros.publish', {
            'endpoint': '/pris_demo/goal',
            'ros_type': 'geometry_msgs/msg/PoseStamped',
            'payload': {'header': {'frame_id': 'map'}, 'pose': {
                'position': {'x': goal[0], 'y': goal[1], 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            }},
        }, timeout=30)
        if not response.get('result', {}).get('published'):
            raise AssertionError(f'ROS goal was not published: {response}')
    return operation, goal, response


def random_probe(robot, config, rng):
    operation = rng.choice(('ros.topics', 'ros.services', 'get_odometry',
                            'ros.publish chatter', 'capability announce'))
    if operation == 'ros.publish chatter':
        result = struct_rpc(robot, 'ros.publish', {
            'endpoint': '/chatter',
            'ros_type': 'std_msgs/String' if config['version'] == 1 else 'std_msgs/msg/String',
            'payload': {'data': f'PrIS E2E {robot} {uuid.uuid4()}'},
        }, timeout=20)
        if not result.get('result', {}).get('published'):
            raise AssertionError(f'PrIS chatter publication failed for {robot}')
    elif operation == 'capability announce':
        result = struct_rpc(robot, 'announce', {
            'data': f'PrIS catalogue E2E {robot} {uuid.uuid4()}'}, timeout=20)
        if not result.get('result', {}).get('published'):
            raise AssertionError(f'PrIS capability announce failed for {robot}')
    else:
        struct_rpc(robot, operation, timeout=20)
    return operation


class Report:
    def __init__(self, loop):
        self.lock = threading.Lock()
        self.report_path = Path(os.getenv('E2E_LOOP_REPORT_FILE' if loop else 'REPORT_FILE',
            '/reports/e2e-loop-report.json' if loop else '/reports/e2e-report.json'))
        self.history_path = Path(os.getenv('E2E_LOOP_HISTORY_FILE', '/reports/e2e-loop-history.jsonl'))
        self.max_history_bytes = int(os.getenv('E2E_LOOP_HISTORY_MAX_BYTES', '10485760'))
        self.recent = deque(maxlen=40)
        self.durations = deque(maxlen=100)
        self.data = {'started_at_utc': now_utc(), 'mode': 'continuous_random' if loop else 'once',
                     'flow': 'PrIS is-wire/is-msgs -> gateway -> ROS 1/2 -> four robots',
                     'status': 'starting', 'robots': {robot: {'ros_version': cfg['version'],
                         'passed': 0, 'failed': 0, 'attempts': 0, 'operations': {}}
                         for robot, cfg in ROBOTS.items()},
                     'attempts': 0, 'passed': 0, 'failed': 0, 'recent': []}
        self._write()

    def _write(self):
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.report_path.with_name(self.report_path.name + '.tmp')
        tmp.write_text(json.dumps(self.data, indent=2, default=str) + '\n')
        tmp.replace(self.report_path)

    def set_status(self, status):
        with self.lock:
            self.data['status'] = status
            self.data['updated_at_utc'] = now_utc()
            self._write()

    def record(self, record):
        robot = record['robot_id']
        with self.lock:
            stats = self.data['robots'][robot]
            stats['attempts'] += 1
            self.data['attempts'] += 1
            key = 'passed' if record['ok'] else 'failed'
            stats[key] += 1
            self.data[key] += 1
            operation = record.get('operation', 'preflight')
            stats['operations'][operation] = stats['operations'].get(operation, 0) + 1
            self.recent.append(record)
            self.data['recent'] = list(self.recent)
            self.durations.append(record['duration_seconds'])
            self.data['mean_duration_seconds_last_100'] = round(statistics.mean(self.durations), 3)
            self.data['updated_at_utc'] = now_utc()
            self.history_path.parent.mkdir(parents=True, exist_ok=True)
            if self.history_path.exists() and self.history_path.stat().st_size >= self.max_history_bytes:
                self.history_path.replace(self.history_path.with_suffix('.jsonl.1'))
            with self.history_path.open('a') as output:
                output.write(json.dumps(record, default=str) + '\n')
            self._write()


def check_rviz(seconds=100):
    deadline = time.monotonic() + seconds
    last_error = None
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(RVIZ_URL, timeout=5) as response:
                if response.status == 200 and b'vnc' in response.read(200000).lower():
                    return
        except Exception as exc:
            last_error = str(exc)
        time.sleep(2)
    raise AssertionError('ROS2 RViz/noVNC unavailable: ' + str(last_error))


def worker(robot, config, rng, stop, report, once, goal_limit, interval, retry_delay,
           action_only=False):
    observer = RobotObserver(robot, config)
    observer.start()
    ready = False
    iteration = 0
    try:
        while not stop.is_set():
            iteration += 1
            started = time.monotonic()
            entry = {'robot_id': robot, 'ros_version': config['version'],
                     'iteration': iteration, 'timestamp_utc': now_utc()}
            try:
                if not ready:
                    entry['available_topics'] = wait_for_graph(robot, config)
                    wait_for_observer(observer, config)
                    ready = True
                    log(robot, f'graph ready, ROS {config["version"]}, independent rosbridge')
                current = wire_odometry(robot)
                operation = rng.choice(available_operations(config, action_only))
                goal = random_goal(rng, current, goal_limit)
                entry['operation'] = operation
                entry['target'] = list(goal)
                operation, goal, response = send_random_movement(
                    robot, config, rng, current, goal_limit, action_only=action_only,
                    operation=operation, goal=goal)
                entry['request_id'] = response.get('request_id')
                position, observation = wait_for_goal(robot, config, observer, goal)
                entry['odometry'] = position
                entry['observer'] = observation
                entry['probe'] = random_probe(robot, config, rng)
                entry['ok'] = True
                log(robot, f'PASS #{iteration}: {operation} -> {goal}, probe={entry["probe"]}')
            except Exception as exc:
                entry['ok'] = False
                entry['error'] = f'{type(exc).__name__}: {exc}'
                entry['observer'] = observer.snapshot()
                log(robot, f'FAIL #{iteration}: {entry["error"]}; retrying')
            entry['duration_seconds'] = round(time.monotonic() - started, 3)
            report.record(entry)
            if once:
                return
            stop.wait(interval if entry['ok'] else retry_delay)
    finally:
        observer.close()


def run():
    loop = os.getenv('E2E_LOOP', 'false').lower() in ('1', 'yes', 'true')
    action_only = os.getenv('E2E_ACTION_TEST', 'false').lower() in ('1', 'yes', 'true')
    interval = float(os.getenv('E2E_LOOP_INTERVAL_SECONDS', '1.0'))
    retry_delay = float(os.getenv('E2E_LOOP_RETRY_DELAY_SECONDS', '5.0'))
    goal_limit = float(os.getenv('E2E_LOOP_GOAL_LIMIT', '2.5'))
    if not (0 <= interval <= 60 and 0.5 <= retry_delay <= 60 and 1 <= goal_limit <= 3.5):
        raise ValueError('Invalid E2E loop interval, retry delay, or goal limit')
    report = Report(loop)
    stop = threading.Event()

    def stop_handler(signum, _frame):
        log('all', f'received signal {signum}, stopping after active movements')
        stop.set()

    handlers = {signum: signal.signal(signum, stop_handler)
                for signum in (signal.SIGINT, signal.SIGTERM)}
    try:
        log('all', 'checking the RViz/noVNC display on the first ROS 2 robot')
        check_rviz()
        report.set_status('running')
        seed = os.getenv('E2E_LOOP_SEED', '').strip()
        log('all', f'starting {len(ROBOTS)} independent is-wire workers '
            f'({sum(c["version"] == 1 for c in ROBOTS.values())} ROS 1, '
            f'{sum(c["version"] == 2 for c in ROBOTS.values())} ROS 2)')
        threads = []
        for robot, config in ROBOTS.items():
            rng = random.Random(seed + robot if seed else None)
            thread = threading.Thread(target=worker,
                args=(robot, config, rng, stop, report, not loop, goal_limit, interval,
                      retry_delay, action_only),
                name='e2e-' + robot)
            thread.start()
            threads.append(thread)
        while any(thread.is_alive() for thread in threads):
            for thread in threads:
                thread.join(timeout=0.5)
        if loop:
            report.set_status('stopped')
            return 0
        passed = report.data['passed'] == len(ROBOTS) and report.data['failed'] == 0
        report.set_status('passed' if passed else 'failed')
        return 0 if passed else 1
    except Exception as exc:
        log('all', f'FATAL: {type(exc).__name__}: {exc}')
        report.set_status('failed')
        return 1
    finally:
        stop.set()
        for signum, original in handlers.items():
            signal.signal(signum, original)


if __name__ == '__main__':
    raise SystemExit(run())
