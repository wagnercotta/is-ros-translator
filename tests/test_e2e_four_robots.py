"""Standard-library regression tests for the four-robot PrIS E2E lab."""
import importlib.util
import json
import os
from pathlib import Path
import random
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
try:
    import websocket
except ImportError:
    sys.modules['websocket'] = types.ModuleType('websocket')
spec = importlib.util.spec_from_file_location('pris_multi_e2e',
    ROOT / 'tests/integration/pris_multi_robot_e2e.py')
e2e = importlib.util.module_from_spec(spec)
spec.loader.exec_module(e2e)


class FourRobotE2ETests(unittest.TestCase):
    def test_four_independent_configured_robots(self):
        from is_ros_translator_v2.config import load_settings
        settings = load_settings(ROOT / 'configs/e2e-rviz.yaml')
        self.assertEqual(set(settings.robots), set(e2e.ROBOTS))
        self.assertEqual([robot.ros_version for robot in settings.robots.values()].count(1), 2)
        self.assertEqual([robot.ros_version for robot in settings.robots.values()].count(2), 2)
        for name, cfg in e2e.ROBOTS.items():
            self.assertEqual(settings.robots[name].host, cfg['host'])
            action = '/move_base' if cfg['version'] == 1 else '/navigate_to_pose'
            self.assertEqual(settings.robots[name].navigation.action, action)

    def test_docker_compose_has_four_independent_simulators(self):
        services = yaml.safe_load((ROOT / 'deploy/e2e/compose.yaml').read_text())['services']
        for simulator in ('ros1a', 'ros1b', 'ros2', 'ros2b'):
            self.assertIn(simulator, services)
            self.assertNotIn('network_mode', services[simulator])
            self.assertEqual(services['gateway']['depends_on'][simulator]['condition'],
                             'service_healthy')
        self.assertEqual(services['rviz']['network_mode'], 'service:ros2')
        self.assertEqual(services['e2e-loop']['environment']['E2E_LOOP'], 'true')

    def test_random_destinations_are_valid_for_every_robot(self):
        for robot in e2e.ROBOTS:
            rng = random.Random(robot)
            position = {'x': 0, 'y': 0}
            for _ in range(40):
                goal = e2e.random_goal(rng, position)
                self.assertTrue(all(-2.5 <= p <= 2.5 for p in goal))
                self.assertFalse(e2e.position_near(position, goal, 0.8))
                position = {'x': goal[0], 'y': goal[1]}

    def test_all_four_robots_use_iswire_for_commands(self):
        for robot, config in e2e.ROBOTS.items():
            sent = []

            def fake_rpc(rid, operation, args=None, timeout=60):
                sent.append((rid, operation, args))
                return {'request_id': 'testing', 'result': {'published': True,
                                                            'completed_waypoints': 2}}

            def fake_task(rid, goal):
                sent.append((rid, 'RobotTask', goal))
                return {'request_id': 'testing', 'result': {'navigation_completed': True}}

            class Pick:
                def __init__(self, operation):
                    self.operation = operation

                def choice(self, options):
                    self_options = {'RobotTask': 'is-msgs RobotTask'}
                    name = self_options.get(self.operation, self.operation)
                    assert name in options
                    return name

                def uniform(self, low, high):
                    return high

            ops = ['is-msgs RobotTask', 'navigate', 'follow_waypoints']
            if config['version'] == 2:
                ops.append('ros.publish goal')
            with patch.object(e2e, 'struct_rpc', fake_rpc), patch.object(e2e, 'robot_task_rpc', fake_task):
                for operation in ops:
                    sent.clear()
                    e2e.send_random_movement(robot, config, Pick(operation), {'x': 0, 'y': 0}, 2.5)
                    self.assertTrue(sent)
                    self.assertEqual(sent[0][0], robot)

    def test_reports_have_per_robot_results_and_history(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(os.environ, {'REPORT_FILE': str(Path(directory) / 'report.json'),
                                         'E2E_LOOP_HISTORY_FILE': str(Path(directory) / 'history.jsonl')}):
                report = e2e.Report(loop=False)
                for robot in e2e.ROBOTS:
                    report.record({'robot_id': robot, 'duration_seconds': 0.1,
                                   'operation': 'navigate', 'ok': True})
                self.assertEqual(report.data['passed'], 4)
                self.assertEqual(report.data['failed'], 0)
                self.assertEqual(len(report.data['robots']), 4)
                entries = (Path(directory) / 'history.jsonl').read_text().splitlines()
                self.assertEqual(len(entries), 4)
                self.assertEqual({json.loads(line)['robot_id'] for line in entries},
                                 set(e2e.ROBOTS))

    def test_finite_e2e_starts_all_four_workers_concurrently(self):
        barrier = threading.Barrier(4, timeout=4)

        def fake_worker(robot, config, rng, stop, report, once, goal_limit,
                        interval, retry_delay, action_only):
            self.assertTrue(once)
            barrier.wait()
            report.record({'robot_id': robot, 'duration_seconds': 0.1,
                           'operation': 'navigate', 'ok': True})

        with tempfile.TemporaryDirectory() as directory:
            with patch.dict(os.environ, {'REPORT_FILE': str(Path(directory) / 'report.json'),
                                         'E2E_LOOP_HISTORY_FILE': str(Path(directory) / 'history.jsonl'),
                                         'E2E_LOOP': 'false'}):
                with patch.object(e2e, 'worker', fake_worker), patch.object(e2e, 'check_rviz'):
                    self.assertEqual(e2e.run(), 0)
                result = json.loads((Path(directory) / 'report.json').read_text())
                self.assertEqual(result['status'], 'passed')
                self.assertEqual(result['passed'], 4)

    def test_gateway_processes_iswire_requests_concurrently(self):
        source = (ROOT / 'src/is_ros_translator_v2/iswire_transport.py').read_text()
        self.assertIn('ThreadPoolExecutor(max_workers=8', source)
        self.assertIn('pending.add(executor.submit(self._build_reply, msg))', source)
        self.assertIn('channel.publish(reply)', source)


if __name__ == '__main__':
    unittest.main()
