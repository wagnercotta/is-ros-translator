"""Unit coverage for configuration-defined semantic PrIS capabilities."""

import asyncio
from pathlib import Path
import sys
import unittest

from pydantic import ValidationError
from fastapi.testclient import TestClient
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from is_ros_translator_v2.config import Settings  # noqa: E402
from is_ros_translator_v2.api import create_app  # noqa: E402
from is_ros_translator_v2.errors import InvalidRequest  # noqa: E402
from is_ros_translator_v2.gateway import Gateway, merge_payload  # noqa: E402
from is_ros_translator_v2.models import Command  # noqa: E402


class FakeBridge:
    def __init__(self, ros_version):
        self.ros_version = ros_version
        self.calls = []

    def connected(self):
        return True

    async def close(self):
        return None

    async def get_topic_type(self, endpoint):
        if endpoint == '/odom':
            return ('nav_msgs/Odometry' if self.ros_version == 1
                    else 'nav_msgs/msg/Odometry')
        return ('std_msgs/String' if self.ros_version == 1
                else 'std_msgs/msg/String')

    async def message_details(self, ros_type):
        return {'typedefs': [{'type': ros_type, 'fieldnames': ['data'],
                             'fieldtypes': ['string'], 'fieldarraylen': [-1]}]}

    async def get_service_type(self, endpoint):
        return 'std_srvs/Trigger' if self.ros_version == 1 else 'std_srvs/srv/Trigger'

    async def service_details(self, ros_type):
        return {'typedefs': [{'type': ros_type, 'fieldnames': [],
                             'fieldtypes': [], 'fieldarraylen': []}]}

    async def get_action_type(self, endpoint):
        return 'example_msgs/action/Dock'

    async def action_details(self, ros_type):
        return {'typedefs': [{'type': ros_type, 'fieldnames': [],
                             'fieldtypes': [], 'fieldarraylen': []}]}

    async def publish(self, endpoint, ros_type, payload):
        self.calls.append(('publish', endpoint, ros_type, payload))
        return {'published': True}

    async def service(self, endpoint, ros_type, payload, timeout):
        self.calls.append(('service', endpoint, ros_type, payload))
        return {'called': True}

    async def action(self, endpoint, ros_type, payload, timeout, feedback=None):
        self.calls.append(('action', endpoint, ros_type, payload))
        return {'status': 4}

    async def subscribe(self, endpoint, ros_type, callback):
        self.calls.append(('subscribe', endpoint, ros_type, None))
        return endpoint

    async def unsubscribe(self, handle):
        self.calls.append(('unsubscribe', handle, '', None))


def settings_for(ros_version=2, capabilities=None, allowed=None):
    return Settings.model_validate({
        'security': {'require_api_key': False},
        'robots': {
            'robot_a': {
                'ros_version': ros_version,
                'host': 'localhost',
                'allowed': allowed or {
                    'publish': ['/chatter'],
                    'subscribe': ['/odom'],
                    'services': ['/capture_image'],
                    'actions': ['/dock'],
                },
                'capabilities': capabilities or {},
            },
        },
    })


class CapabilityCatalogTests(unittest.TestCase):
    def test_named_subscribe_and_unsubscribe_preserve_endpoint(self):
        settings = settings_for(capabilities={
            'monitor_odometry': {
                'operation': 'ros.subscribe', 'endpoint': '/odom',
                'ros_type': 'nav_msgs/msg/Odometry',
            },
            'stop_monitoring_odometry': {
                'operation': 'ros.unsubscribe', 'endpoint': '/odom',
            },
        })
        bridge = FakeBridge(2)
        gateway = Gateway(settings, bridges={'robot_a': bridge})

        async def run():
            await gateway.start()
            try:
                added = await gateway.execute({
                    'robot_id': 'robot_a', 'operation': 'monitor_odometry',
                    'args': {'subscriber_id': 'alice', 'endpoint': '/not_allowed'},
                })
                self.assertEqual(added['result']['endpoint'], '/odom')
                self.assertEqual(added['operation'], 'monitor_odometry')
                removed = await gateway.execute({
                    'robot_id': 'robot_a', 'operation': 'stop_monitoring_odometry',
                    'args': {'subscriber_id': 'alice'},
                })
                self.assertFalse(removed['result']['subscribed'])
            finally:
                await gateway.stop()

        asyncio.run(run())
        self.assertEqual([call[0] for call in bridge.calls], ['subscribe', 'unsubscribe'])

    def test_api_lists_capability_names_and_descriptions(self):
        settings = settings_for(capabilities={
            'announce': {'operation': 'ros.publish', 'endpoint': '/chatter',
                         'default_payload': {'data': 'hello'},
                         'description': 'Test announcement'},
        })
        gateway = Gateway(settings, bridges={'robot_a': FakeBridge(2)})
        with TestClient(create_app(settings, gateway)) as client:
            robots = client.get('/v1/robots').json()
            self.assertEqual(robots[0]['capabilities'], ['announce'])
            response = client.get('/v1/robots/robot_a/capabilities')
            self.assertEqual(response.status_code, 200)
            announce = response.json()['capabilities']['announce']
            self.assertEqual(announce['description'], 'Test announcement')
            self.assertEqual(announce['default_payload'], {'data': 'hello'})
            self.assertEqual(client.get('/v1/robots/missing/capabilities').status_code, 404)

    def test_publish_catalog_name_retains_request_identity(self):
        settings = settings_for(capabilities={
            'announce': {'operation': 'ros.publish', 'endpoint': '/chatter',
                         'ros_type': 'std_msgs/msg/String',
                         'default_payload': {'data': 'default'}},
        })
        bridge = FakeBridge(2)
        gateway = Gateway(settings, bridges={'robot_a': bridge})
        result = asyncio.run(gateway.execute({
            'robot_id': 'robot_a', 'operation': 'announce',
            'args': {'data': 'Hello PrIS'}, 'request_id': 'capability-test',
        }))
        self.assertEqual(result['operation'], 'announce')
        self.assertEqual(result['request_id'], 'capability-test')
        self.assertEqual(bridge.calls, [('publish', '/chatter', 'std_msgs/msg/String',
                                         {'data': 'Hello PrIS'})])
        self.assertEqual(settings.robots['robot_a'].capabilities['announce'].default_payload,
                         {'data': 'default'})

    def test_ros1_uses_its_configured_type(self):
        settings = settings_for(ros_version=1, capabilities={
            'announce': {'operation': 'ros.publish', 'endpoint': '/chatter',
                         'ros_type': 'std_msgs/String', 'default_payload': {'data': 'ROS1'}},
        })
        bridge = FakeBridge(1)
        asyncio.run(Gateway(settings, bridges={'robot_a': bridge}).execute({
            'robot_id': 'robot_a', 'operation': 'announce',
        }))
        self.assertEqual(bridge.calls[-1], ('publish', '/chatter', 'std_msgs/String',
                                            {'data': 'ROS1'}))

    def test_service_and_action_dispatch(self):
        settings = settings_for(capabilities={
            'capture_image': {'operation': 'ros.service', 'endpoint': '/capture_image',
                              'ros_type': 'std_srvs/srv/Trigger'},
            'dock': {'operation': 'ros.action', 'endpoint': '/dock',
                     'ros_type': 'example_msgs/action/Dock', 'allow_overrides': False},
        })
        bridge = FakeBridge(2)
        gateway = Gateway(settings, bridges={'robot_a': bridge})

        async def run():
            a = await gateway.execute({'robot_id': 'robot_a', 'operation': 'capture_image'})
            b = await gateway.execute({'robot_id': 'robot_a', 'operation': 'dock'})
            return a, b

        a, b = asyncio.run(run())
        self.assertTrue(a['result']['called'])
        self.assertEqual(b['result']['status'], 4)
        self.assertEqual([x[0] for x in bridge.calls], ['service', 'action'])

        with self.assertRaises(InvalidRequest):
            asyncio.run(gateway.execute({'robot_id': 'robot_a', 'operation': 'dock',
                                         'args': {'override': 'not allowed'}}))

    def test_invalid_payload_is_rejected_by_ros_schema(self):
        settings = settings_for(capabilities={
            'announce': {'operation': 'ros.publish', 'endpoint': '/chatter',
                         'default_payload': {'data': 'hello'}},
        })
        bridge = FakeBridge(2)
        gateway = Gateway(settings, bridges={'robot_a': bridge})
        with self.assertRaises(InvalidRequest):
            asyncio.run(gateway.execute({'robot_id': 'robot_a', 'operation': 'announce',
                                         'args': {'data': {'unsafe': 'not a string'}}}))
        self.assertEqual(bridge.calls, [])

    def test_catalog_requires_the_existing_allowlist(self):
        with self.assertRaisesRegex(ValidationError, 'allowed.actions'):
            settings_for(capabilities={
                'dock': {'operation': 'ros.action', 'endpoint': '/not_allowed',
                         'ros_type': 'example_msgs/action/Dock'},
            })

    def test_action_needs_type_and_reserved_names_cannot_be_shadowed(self):
        with self.assertRaisesRegex(ValidationError, 'ros_type'):
            settings_for(capabilities={'dock': {
                'operation': 'ros.action', 'endpoint': '/dock',
            }})
        with self.assertRaisesRegex(ValidationError, 'conflicts'):
            settings_for(capabilities={'navigate': {
                'operation': 'ros.publish', 'endpoint': '/chatter',
            }})

    def test_unknown_operation_stays_invalid(self):
        settings = settings_for()
        gateway = Gateway(settings, bridges={'robot_a': FakeBridge(2)})
        with self.assertRaises(InvalidRequest):
            asyncio.run(gateway.execute({'robot_id': 'robot_a',
                                         'operation': 'arbitrary_unknown_function'}))

    def test_nested_defaults_do_not_mutate_or_drop_siblings(self):
        base = {'pose': {'position': {'x': 0.0, 'y': 1.0}, 'orientation': {'w': 1.0}}}
        result = merge_payload(base, {'pose': {'position': {'x': 2.0}}})
        self.assertEqual(result['pose']['position'], {'x': 2.0, 'y': 1.0})
        self.assertEqual(result['pose']['orientation'], {'w': 1.0})
        self.assertEqual(base['pose']['position']['x'], 0.0)

    def test_iswire_contract_accepts_named_operations(self):
        command = Command.model_validate({'robot_id': 'robot_a', 'operation': 'capture_image'})
        self.assertEqual(command.operation, 'capture_image')

    def test_kubernetes_and_docker_example_catalogues_validate(self):
        for path in (ROOT / 'configs/e2e-rviz.yaml',
                     ROOT / 'deploy/kubernetes/configmap.yaml'):
            data = yaml.safe_load(path.read_text())
            if data.get('kind') == 'ConfigMap':
                data = yaml.safe_load(data['data']['gateway.yaml'])
            settings = Settings.model_validate(data)
            self.assertEqual(len(settings.robots), 4)
            self.assertTrue(all('announce' in robot.capabilities
                                for robot in settings.robots.values()))


if __name__ == '__main__':
    unittest.main()
