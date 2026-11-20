"""Protocol-level tests for the sole AMQP interface without requiring Docker.

These stubs reproduce the public is-wire Message/Channel API, including the
integer correlation_id and create_reply semantics inspected in upstream 1.2.1.
They do not replace a live broker integration test.
"""
import asyncio
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import Mock

import pytest
from google.protobuf.struct_pb2 import Struct
from google.protobuf.json_format import ParseDict, MessageToDict

from is_ros_translator_v2.config import Settings
from is_ros_translator_v2.iswire_transport import ISWireTransport, wire_rpc


class StubMessage:
    counter = 100

    def __init__(self, content=None, reply_to=None):
        self.body = b''
        self.topic = None
        self.correlation_id = None
        self.reply_to = None
        self.status = None
        self.content = None
        if content is not None:
            self.pack(content)
        if reply_to is not None:
            self.reply_to = reply_to.name
            StubMessage.counter += 1
            self.correlation_id = StubMessage.counter

    def pack(self, value):
        self.content = value
        if hasattr(value, 'SerializeToString'):
            self.body = value.SerializeToString()
        else:
            self.body = b'FAKE'

    def unpack(self, cls):
        if isinstance(self.content, cls):
            return self.content
        value = cls()
        if hasattr(value, 'ParseFromString'):
            value.ParseFromString(self.body)
        return value

    def create_reply(self):
        result = StubMessage()
        result.topic = self.reply_to
        result.correlation_id = self.correlation_id
        return result


class StubSubscription:
    def __init__(self, channel, name=None):
        self.name = name or 'reply.sink'
        self.topics = []

    def subscribe(self, topic):
        self.topics.append(topic)


class StubStatus:
    def __init__(self, code, why=''):
        self.code = code
        self.why = why

    def ok(self):
        return self.code == 'OK'


class StubStatusCode:
    OK = 'OK'
    INVALID_ARGUMENT = 'INVALID_ARGUMENT'
    INTERNAL_ERROR = 'INTERNAL_ERROR'


@dataclass
class StubRobotTaskReply:
    id: int = 0


class StubPose:
    def __init__(self):
        self.position = types.SimpleNamespace(x=0., y=0., z=0.)
        self.orientation = types.SimpleNamespace(yaw=0., pitch=0., roll=0.)


@pytest.fixture
def stub_sdk(monkeypatch):
    core = types.ModuleType('is_wire.core')
    core.Message = StubMessage
    core.Channel = object
    core.Subscription = StubSubscription
    core.Status = StubStatus
    core.StatusCode = StubStatusCode
    wire = types.ModuleType('is_wire')
    wire.core = core
    msgs = types.ModuleType('is_msgs')
    robot = types.ModuleType('is_msgs.robot_pb2')
    robot.RobotTaskRequest = object
    robot.RobotTaskReply = StubRobotTaskReply
    common = types.ModuleType('is_msgs.common_pb2')
    common.Pose = StubPose
    msgs.robot_pb2 = robot
    msgs.common_pb2 = common
    monkeypatch.setitem(sys.modules, 'is_wire', wire)
    monkeypatch.setitem(sys.modules, 'is_wire.core', core)
    monkeypatch.setitem(sys.modules, 'is_msgs', msgs)
    monkeypatch.setitem(sys.modules, 'is_msgs.robot_pb2', robot)
    monkeypatch.setitem(sys.modules, 'is_msgs.common_pb2', common)
    return core, robot


def settings():
    return Settings.model_validate({
        'http': {'enabled': True}, 'iswire': {'enabled': True},
        'security': {'require_api_key': False},
        'robots': {'turtlebot_ros2': {'ros_version': 2, 'host': 'sim'}}
    })


def test_wire_rpc_preserves_integer_correlation_id(stub_sdk):
    core, _ = stub_sdk

    class StubChannel:
        def __init__(self, uri):
            self.uri = uri
            self.received = []
            self.closed = False

        def publish(self, request, topic):
            assert topic == 'ROSTranslator.v2.Request'
            assert isinstance(request.correlation_id, int)
            reply = request.create_reply()
            reply.pack(ParseDict({'ok': True, 'result': {'passed': True}}, Struct()))
            self.received.append(reply)

        def consume(self, timeout):
            return self.received.pop(0)

        def close(self):
            self.closed = True

    request = ParseDict({'operation': 'ros.topics'}, Struct())
    response = wire_rpc('amqp://test', 'ROSTranslator.v2.Request', request,
                        channel_cls=StubChannel)
    assert MessageToDict(response.unpack(Struct))['result']['passed']
    assert isinstance(response.correlation_id, int)


def test_struct_request_parses_and_executes_real_command(stub_sdk):
    transport = ISWireTransport(settings())
    called = []
    transport._execute = lambda cmd: called.append(cmd) or {'ok': True, 'result': {'topics': ['/odom']}}
    request = StubMessage(ParseDict({'robot_id': 'turtlebot_ros2',
                                    'operation': 'ros.topics', 'args': {}}, Struct()))
    result = transport._handle_struct_request(request)
    assert called[0].operation == 'ros.topics'
    assert MessageToDict(result)['result']['topics'] == ['/odom']


def test_robot_task_uses_is_msgs_and_returns_typed_reply(stub_sdk):
    _, robot = stub_sdk

    class Request:
        def __init__(self):
            self.id = 987
            self.basic_move_task = types.SimpleNamespace(
                positions=[types.SimpleNamespace(x=1.1, y=2.2, z=0.)],
                final_orientation=types.SimpleNamespace(yaw=0.5),
                allowed_error=0.15, rate=10.0)

        def WhichOneof(self, name):
            assert name == 'Task'
            return 'basic_move_task'

    robot.RobotTaskRequest = Request
    request = StubMessage()
    request.topic = 'ROSTranslator.v2.RobotTask.turtlebot_ros2'
    request.body = b'FAKE'
    request.unpack = lambda cls: Request()
    transport = ISWireTransport(settings())
    commands = []
    transport._execute = lambda cmd: commands.append(cmd) or {'ok': True}
    result, status = transport._handle_robot_task(request)
    assert status.ok()
    assert isinstance(result, StubRobotTaskReply) and result.id == 987
    assert commands[0].operation == 'navigate'
    assert commands[0].args['position']['x'] == pytest.approx(1.1)


def test_native_rpc_reply_uses_iswire_create_reply_not_manual_correlation(stub_sdk):
    core, _ = stub_sdk
    transport = ISWireTransport(settings())
    transport._execute = lambda cmd: {'ok': True, 'robot_id': cmd.robot_id, 'result': {'topics': ['/odom']}}
    request = StubMessage(ParseDict({'robot_id': 'turtlebot_ros2',
                                    'operation': 'ros.topics'}, Struct()),
                          reply_to=types.SimpleNamespace(name='reply.instance'))
    request.topic = 'ROSTranslator.v2.Request'
    outgoing = []
    channel = types.SimpleNamespace(publish=lambda reply: outgoing.append(reply))
    transport._handle_incoming(channel, request)
    assert len(outgoing) == 1
    assert outgoing[0].correlation_id == request.correlation_id
    assert outgoing[0].topic == 'reply.instance'
    assert outgoing[0].status.ok()
    assert MessageToDict(outgoing[0].unpack(Struct))['ok']


def test_odom_event_converts_to_standard_is_msgs_pose(stub_sdk):
    transport = ISWireTransport(settings())
    event = {'payload': {'pose': {'pose': {
        'position': {'x': 1.0, 'y': 2.0, 'z': 0.0},
        'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}
    }}}}
    pose = transport._pose_from_odom(event)
    assert isinstance(pose, StubPose)
    assert pose.position.x == 1.0 and pose.position.y == 2.0
    assert pose.orientation.yaw == pytest.approx(0.)


def test_only_native_transport_has_amqp_in_repository():
    root = Path(__file__).resolve().parents[1]
    src = root / 'src/is_ros_translator_v2'
    assert not (src / 'amqp_transport.py').exists()
    # No direct RabbitMQ drivers are permitted in the application or test clients.
    forbidden = ['aio' + '_pika', 'aiormq', 'import ' + 'pika']
    tracked = list(src.glob('*.py')) + list((root / 'tests/integration').glob('*.py'))
    tracked += list((root / 'examples').glob('*.py'))
    for file in tracked:
        body = file.read_text()
        for name in forbidden:
            assert name not in body, (file, name)
    packaging = (root / 'pyproject.toml').read_text()
    assert '"is-wire==1.2.1"' in packaging
    assert '"is-msgs==1.1.18"' in packaging
    assert '"protobuf==3.20.3"' in packaging
    assert '"six>=1.16,<2"' in packaging
    assert '"aio-pika' not in packaging


def test_broker_not_ready_does_not_block_fastapi_startup(stub_sdk, monkeypatch):
    core, _ = stub_sdk
    monkeypatch.setenv('ISWIRE_BROKER_URI', 'amqp://testing')

    class FailingChannel:
        def __init__(self, uri):
            raise ConnectionRefusedError('Broker is starting')

    core.Channel = FailingChannel
    transport = ISWireTransport(settings())
    fake_gateway = types.SimpleNamespace(events=types.SimpleNamespace(add_sink=lambda sink: None))

    async def scenario():
        await transport.start(fake_gateway)
        await asyncio.sleep(0.08)
        assert transport._thread.is_alive()
        assert not transport.ready
        await transport.stop()

    asyncio.run(scenario())


def test_docker_lab_requires_native_iswire_for_all_amqp():
    import yaml
    root = Path(__file__).resolve().parents[1]
    for name in ('deploy/e2e/compose.yaml', 'deploy/ros-test/compose.yaml'):
        compose = yaml.safe_load((root / name).read_text())
        services = compose['services']
        assert 'ISWIRE_BROKER_URI' in services['gateway']['environment']
        clients = services['e2e' if 'e2e' in services else 'tester']
        assert 'ISWIRE_BROKER_URI' in clients['environment']
        assert 'AMQP_URL' not in services['gateway']['environment']
        assert 'AMQP_URL' not in clients['environment']
        dockerfile = root / ('deploy/e2e/tester/Dockerfile' if 'e2e' in services
                             else 'deploy/ros-test/tester/Dockerfile')
        data = dockerfile.read_text()
        assert 'is-wire==1.2.1' in data and 'is-msgs==1.1.18' in data
        assert 'six>=1.16,<2' in data
        assert 'from is_wire.core import Channel, Subscription, Message' in data
        assert 'from is_msgs.robot_pb2 import RobotTaskRequest' in data
        assert 'python:3.10-slim' in data
        assert 'iswire' in (root / ('configs/e2e-rviz.yaml' if 'e2e' in services
                                    else 'configs/docker-integration.yaml')).read_text()


def test_gateway_image_checks_native_sdk_imports():
    root = Path(__file__).resolve().parents[1]
    dockerfile = (root / 'deploy/Dockerfile').read_text()
    assert 'from is_wire.core import Channel, Subscription, Message' in dockerfile
    assert 'from is_msgs.robot_pb2 import RobotTaskRequest' in dockerfile
    assert 'from is_msgs.common_pb2 import Pose' in dockerfile
