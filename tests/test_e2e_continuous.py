"""Deterministic unit checks for the indefinite RViz/PrIS E2E mode (no Docker)."""
import ast
import importlib.util
import json
from pathlib import Path
import random
import sys
import types

import pytest


def load_runner(monkeypatch):
    monkeypatch.setenv('ISWIRE_BROKER_URI', 'amqp://test')
    monkeypatch.setitem(sys.modules, 'websocket', types.ModuleType('websocket'))
    filename = Path(__file__).parent / 'integration' / 'rviz_e2e.py'
    spec = importlib.util.spec_from_file_location('rviz_loop_test', filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_ros2_simulator_never_passes_printf_args_to_rclpy_logger():
    source = Path(__file__).resolve().parents[1] / 'examples' / 'ros2_simulator_node.py'
    tree = ast.parse(source.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr not in ('info', 'warn', 'error', 'debug', 'fatal'):
            continue
        logger = node.func.value
        if isinstance(logger, ast.Call) and isinstance(logger.func, ast.Attribute):
            if logger.func.attr == 'get_logger':
                assert len(node.args) == 1, 'rclpy logger expects one formatted string'


def test_random_goals_stay_in_demo_bounds_and_are_distinct(monkeypatch):
    runner = load_runner(monkeypatch)
    rng = random.Random('e2e-test')
    current = {'x': 0.0, 'y': 0.0}
    for _ in range(200):
        target = runner.random_goal(rng, current)
        assert all(-2.5 <= value <= 2.5 for value in target)
        assert not runner.near(current, target, tolerance=0.8)
        current = {'x': target[0], 'y': target[1]}


@pytest.mark.parametrize('operation', (
    'ros.publish goal', 'is-msgs RobotTask', 'navigate', 'follow_waypoints',
))
def test_random_movement_operations_reach_native_pris_transport(monkeypatch, operation):
    runner = load_runner(monkeypatch)
    sent = []

    def fake_wire(payload, timeout_seconds=30):
        sent.append(('wire', payload, timeout_seconds))
        return {'ok': True, 'request_id': payload['request_id'], 'result': {
            'published': True, 'completed_waypoints': 2,
        }}

    def fake_task(x, y):
        sent.append(('task', x, y))
        return {'ok': True, 'request_id': 'task',
                'result': {'navigation_completed': True}}

    monkeypatch.setattr(runner, 'wire_rpc', fake_wire)
    monkeypatch.setattr(runner, 'robot_task_rpc', fake_task)
    goal = (1.25, -1.5)
    operation_result, target, response = runner.run_random_command(
        random.Random(7), {'x': 0, 'y': 0}, 2.5, operation=operation, target=goal
    )
    assert (operation_result, target) == (operation, goal)
    assert response['ok']
    if operation == 'is-msgs RobotTask':
        assert sent == [('task', *goal)]
    elif operation == 'ros.publish goal':
        assert sent[0][1]['operation'] == 'ros.publish'
        assert sent[0][1]['args']['payload']['pose']['position']['x'] == goal[0]
    elif operation == 'navigate':
        assert sent[0][1]['operation'] == 'navigate'
        assert sent[0][1]['args']['position'] == {'x': goal[0], 'y': goal[1]}
    else:
        assert sent[0][1]['operation'] == 'follow_waypoints'
        assert len(sent[0][1]['args']['waypoints']) == 2
        assert sent[0][1]['args']['waypoints'][-1]['position']['y'] == goal[1]


@pytest.mark.parametrize('operation', (
    'ros.topics', 'ros.services', 'get_odometry', 'ros.publish chatter',
))
def test_random_probe_uses_iswire(monkeypatch, operation):
    runner = load_runner(monkeypatch)
    monkeypatch.setattr(runner, 'loop_request', lambda action, args=None: {
        'ok': True, 'result': {'published': True, 'operation': action},
    })

    class Choose:
        def choice(self, values):
            assert operation in values
            return operation

    actual, reply = runner.run_random_probe(Choose())
    assert actual == operation
    assert reply['ok']


def test_live_report_is_atomic_and_history_rotates(monkeypatch, tmp_path):
    runner = load_runner(monkeypatch)
    report = tmp_path / 'reports' / 'current.json'
    runner.atomic_report_write(report, {'status': 'running', 'passed': 2})
    assert json.loads(report.read_text())['passed'] == 2
    runner.atomic_report_write(report, {'status': 'stopped', 'passed': 3})
    assert json.loads(report.read_text()) == {'status': 'stopped', 'passed': 3}
    assert not report.with_name('current.json.tmp').exists()

    history = report.parent / 'history.jsonl'
    for attempt in range(20):
        runner.append_bounded_history(history, {'attempt': attempt, 'payload': 'x' * 200},
                                      max_bytes=1024)
    assert history.exists()
    assert history.with_name('history.jsonl.1').exists()
    assert json.loads(history.read_text().splitlines()[-1])['attempt'] == 19


def test_wait_for_goal_tolerates_temporary_rosbridge_disconnect(monkeypatch):
    runner = load_runner(monkeypatch)

    class Observer:
        count = 0

        def snapshot(self):
            self.count += 1
            if self.count == 1:
                return {'observer_connected': False, 'observer_error': 'Connection lost',
                        'last_message_age_seconds': 8, 'path_last': None, 'tf_last': None}
            return {'observer_connected': True, 'observer_error': None,
                    'last_message_age_seconds': 0.2,
                    'path_last': {'x': 1.25, 'y': 0.5},
                    'tf_last': {'x': 1.25, 'y': 0.5}}

    monkeypatch.setattr(runner, 'get_position', lambda: {'x': 1.25, 'y': 0.5})
    monkeypatch.setattr(runner.time, 'sleep', lambda _: None)
    obs = Observer()
    position, sample = runner.wait_for_goal(obs, (1.25, 0.5), seconds=2)
    assert position['x'] == 1.25
    assert sample['observer_connected']
    assert obs.count == 2


def test_observer_recovers_after_websocket_close(monkeypatch):
    runner = load_runner(monkeypatch)

    class Timeout(Exception):
        pass

    class FakeSocket:
        def __init__(self):
            self.messages = [
                {'op': 'publish', 'topic': '/pris_demo/path',
                 'msg': {'poses': [{'pose': {'position': {'x': 1.0, 'y': 0.0}}}]}},
                {'op': 'publish', 'topic': '/tf',
                 'msg': {'transforms': [{'child_frame_id': 'base_link',
                                         'transform': {'translation': {'x': 1.0, 'y': 0.0}}}]}},
            ]

        def settimeout(self, timeout):
            pass

        def send(self, data):
            pass

        def recv(self):
            if self.messages:
                return json.dumps(self.messages.pop(0))
            raise Timeout()

        def close(self):
            pass

    attempts = []

    def connect(*args, **kwargs):
        attempts.append(True)
        if len(attempts) == 1:
            raise ConnectionResetError('temporary websocket reset')
        return FakeSocket()

    monkeypatch.setattr(runner.websocket, 'create_connection', connect, raising=False)
    monkeypatch.setattr(runner.websocket, 'WebSocketTimeoutException', Timeout, raising=False)
    observer = runner.RosGraphObserver()
    observer.start()
    try:
        runner.wait_for_observer(observer, seconds=4)
        sample = observer.snapshot()
        assert sample['observer_connected'] is True
        assert sample['observer_error'] is None
        assert sample['observer_reconnections'] == 1
        assert sample['path_last']['x'] == 1.0
    finally:
        observer.close()
