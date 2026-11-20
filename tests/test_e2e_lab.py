"""Non-Docker checks of the real E2E lab's wiring and persistent publishers."""
from __future__ import annotations
import asyncio
import importlib.util
from pathlib import Path
import sys
import types
import yaml

from is_ros_translator_v2.bridge import RosBridge
from is_ros_translator_v2.config import RobotConfig, load_settings

ROOT = Path(__file__).resolve().parents[1]


def read_yaml(path):
    return yaml.safe_load((ROOT / path).read_text())


def test_docker_e2e_config_exposes_only_local_ports():
    lab = read_yaml('deploy/e2e/compose.yaml')['services']
    assert set(lab) == {'rabbitmq', 'gateway', 'ros2', 'rviz', 'e2e', 'e2e-loop'}
    assert lab['rviz']['network_mode'] == 'service:ros2'
    assert lab['rviz']['ipc'] == 'service:ros2'
    assert lab['ros2']['ipc'] == 'shareable'  # required donor IPC mode
    assert lab['ros2']['shm_size'] == '512m'
    for service in lab.values():
        assert all(str(port).startswith('127.0.0.1:') for port in service.get('ports', []))
    assert lab['ros2']['ports'] == ['127.0.0.1:6080:6080']
    assert lab['gateway']['ports'] == ['127.0.0.1:18080:8080']
    assert lab['gateway']['depends_on']['ros2']['condition'] == 'service_started'
    assert lab['e2e']['depends_on']['rviz']['condition'] == 'service_started'
    assert lab['e2e-loop']['profiles'] == ['loop']
    assert lab['e2e-loop']['environment']['E2E_LOOP'] == 'true'
    assert lab['e2e-loop']['restart'] == 'unless-stopped'
    assert lab['e2e-loop']['depends_on']['gateway']['condition'] == 'service_healthy'
    assert lab['e2e-loop']['volumes'] == ['./reports:/reports']
    assert lab['rabbitmq']['environment'] == {'RABBITMQ_DEFAULT_USER': 'guest', 'RABBITMQ_DEFAULT_PASS': 'guest'}
    assert lab['rabbitmq']['volumes'] == ['./rabbitmq.conf:/etc/rabbitmq/rabbitmq.conf:ro']
    assert (ROOT / 'deploy/e2e/rabbitmq.conf').read_text().strip() == 'loopback_users = none'
    for name in ('gateway', 'e2e', 'e2e-loop'):
        assert lab[name]['environment']['ISWIRE_BROKER_URI'] == 'amqp://guest:guest@rabbitmq:5672/'
        assert lab[name]['environment']['GATEWAY_API_KEY'] == 'local-e2e-api-key'
    assert not (ROOT / 'deploy/e2e/.env.example').exists()
    assert '--env-file' not in (ROOT / 'scripts/docker-ros-e2e.sh').read_text()


def test_amqp_healthchecks_wait_for_real_broker_readiness():
    for name in ('deploy/e2e/compose.yaml', 'deploy/ros-test/compose.yaml'):
        services = read_yaml(name)['services']
        assert services['rabbitmq']['healthcheck']['test'] == [
            'CMD', 'rabbitmq-diagnostics', '-q', 'check_port_connectivity'
        ]
        gateway_probe = services['gateway']['healthcheck']['test']
        assert '/health/ready' in gateway_probe[-1]
        assert "data.get('status') == 'ready'" in gateway_probe[-1]
        tester = services['e2e' if 'e2e' in services else 'tester']
        assert tester['depends_on']['gateway']['condition'] == 'service_healthy'


def test_e2e_clients_retry_iswire_connection_refused(monkeypatch):
    """Reproduce a broker refusing connections twice, without a live broker."""
    monkeypatch.setenv('ISWIRE_BROKER_URI', 'amqp://testing')
    monkeypatch.setenv('GATEWAY_API_KEY', 'testing')
    monkeypatch.setitem(sys.modules, 'websocket', types.ModuleType('websocket'))
    wire = types.ModuleType('is_wire')
    core = types.ModuleType('is_wire.core')
    wire.core = core
    monkeypatch.setitem(sys.modules, 'is_wire', wire)
    monkeypatch.setitem(sys.modules, 'is_wire.core', core)

    for index, file in enumerate(('rviz_e2e.py', 'docker_smoke.py')):
        source = ROOT / 'tests/integration' / file
        spec = importlib.util.spec_from_file_location(f'retry_{index}', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        clock = [0.0]
        attempts = [0]

        def channel(_uri):
            attempts[0] += 1
            if attempts[0] <= 2:
                raise ConnectionRefusedError('broker not listening')
            return 'connected'

        core.Channel = channel
        module.time = types.SimpleNamespace(
            monotonic=lambda: clock[0], sleep=lambda sec: clock.__setitem__(0, clock[0] + sec)
        )
        assert module.connect_iswire_channel(seconds=10) == 'connected'
        assert attempts[0] == 3
        assert clock[0] == 4.0

        core.Channel = lambda _uri: (_ for _ in ()).throw(ConnectionRefusedError('still down'))
        clock[0] = 0.0
        try:
            module.connect_iswire_channel(seconds=3)
        except ConnectionError as exc:
            assert 'after 3s' in str(exc)
        else:
            raise AssertionError('E2E client must fail after retry deadline')


def test_e2e_uses_semantic_ros2_and_generic_goals():
    settings = load_settings(ROOT / 'configs/e2e-rviz.yaml')
    cfg = settings.robots['turtlebot_ros2']
    assert cfg.ros_version == 2
    assert cfg.host == 'ros2'
    assert cfg.navigation.action == '/navigate_to_pose'
    assert '/pris_demo/goal' in cfg.allowed.publish
    assert '/tf' in cfg.allowed.subscribe
    assert '/pris_demo/path' in cfg.allowed.subscribe
    assert settings.iswire.enabled and settings.security.require_api_key
    assert not settings.security.allow_live_schema_fallback


def test_rviz_visualization_displays():
    cfg = read_yaml('deploy/e2e/rviz/demo.rviz')['Visualization Manager']
    assert cfg['Global Options']['Fixed Frame'] == 'map'
    displays = {display['Class']: display for display in cfg['Displays']}
    for p in ('Map', 'Path', 'TF', 'Marker', 'Odometry'):
        assert f'rviz_default_plugins/{p}' in displays
    assert displays['rviz_default_plugins/Path']['Topic']['Value'] == '/pris_demo/path'
    assert displays['rviz_default_plugins/Marker']['Topic']['Value'] == '/pris_demo/marker'


def test_e2e_simulator_has_ros2_visualization_published():
    source = (ROOT / 'examples/ros2_simulator_node.py').read_text()
    for string in ('TransformBroadcaster', 'OccupancyGrid', 'Path', 'Marker',
                   "'/pris_demo/goal'", "'/pris_demo/path'", "'/odom'"):
        assert string in source
    runner = (ROOT / 'tests/integration/rviz_e2e.py').read_text()
    assert 'wire_rpc(' in runner
    assert 'websocket.create_connection' in runner
    assert 'near(sample[\'path_last\'], target)' in runner
    assert 'near(sample[\'tf_last\'], target)' in runner


def test_first_ros2_publication_keeps_publisher_alive(monkeypatch):
    class FakeTopic:
        all_topics = []
        def __init__(self, conn, name, msg_type, **kwargs):
            self.conn, self.name, self.msg_type = conn, name, msg_type
            self.advertises = 0
            self.publishes = []
            self.unadvertises = 0
            self.all_topics.append(self)
        def advertise(self):
            self.advertises += 1
        def publish(self, msg):
            self.publishes.append(msg)
        def unadvertise(self):
            self.unadvertises += 1
    fake_roslibpy = types.SimpleNamespace(Topic=FakeTopic, Message=dict)
    monkeypatch.setitem(sys.modules, 'roslibpy', fake_roslibpy)
    monkeypatch.setattr('is_ros_translator_v2.bridge.time.sleep', lambda _: None)
    adapter = RosBridge(RobotConfig(ros_version=2, host='robot', port=9090))

    async def connected():
        return object()
    monkeypatch.setattr(adapter, 'ensure_connected', connected)

    async def scenario():
        a = await adapter.publish('/pris_demo/goal', 'geometry_msgs/msg/PoseStamped', {'header': {}})
        b = await adapter.publish('/pris_demo/goal', 'geometry_msgs/msg/PoseStamped', {'header': {'frame_id': 'map'}})
        assert a['published'] and b['published']
        assert len(FakeTopic.all_topics) == 1
        topic = FakeTopic.all_topics[0]
        assert topic.advertises == 1
        assert len(topic.publishes) == 2
        assert topic.unadvertises == 0  # not during publish (DDS needs persistent endpoints)
        await adapter.close()
        assert topic.unadvertises == 1

    asyncio.run(scenario())


def test_ros2_known_jazzy_launch_regression_and_e2e_auth():
    start = (ROOT / 'deploy/ros-test/ros2/start.sh').read_text()
    assert 'delay_between_messages:=0.0' in start
    assert 'address:=0.0.0.0' in start
    lab = read_yaml('deploy/e2e/compose.yaml')['services']
    assert lab['e2e']['environment']['GATEWAY_API_KEY'] == 'local-e2e-api-key'
    assert lab['rviz']['depends_on']['ros2']['condition'] == 'service_started'
    assert lab['e2e']['depends_on']['gateway']['condition'] == 'service_healthy'


def test_ros2_docker_lab_shares_fixed_startup():
    lab = read_yaml('deploy/ros-test/compose.yaml')['services']
    assert lab['ros2']['build']['dockerfile'] == 'deploy/ros-test/ros2/Dockerfile'
    dockerfile = (ROOT / 'deploy/ros-test/ros2/Dockerfile').read_text()
    assert 'deploy/ros-test/ros2/start.sh' in dockerfile


def test_ros_environment_setup_does_not_inherit_nounset(tmp_path):
    """Reproduces ROS Jazzy AMENT_TRACE_SETUP_FILES: unbound variable."""
    import subprocess

    fake_setup = tmp_path / 'setup.bash'
    fake_setup.write_text(
        '[[ -z "$AMENT_TRACE_SETUP_FILES" ]]\n'
        'export MOCK_ROS_SETUP_LOADED=yes\n'
    )
    for relative in (
        'deploy/ros-test/ros2/start.sh',
        'deploy/e2e/rviz/start.sh',
        'deploy/ros-test/ros1/start.sh',
        'deploy/ros-test/ros1/health.sh',
    ):
        source = (ROOT / relative).read_text()
        lines = source.splitlines()
        start = lines.index('set -eo pipefail')
        setup = next(i for i in range(start, len(lines))
                     if lines[i].startswith('source /opt/ros/'))
        end = lines.index('set -u', setup)
        snippet = '\n'.join(lines[start:end + 1])
        snippet = snippet.replace(lines[setup], f'source "{fake_setup}"')
        snippet += '\n[[ "$MOCK_ROS_SETUP_LOADED" = yes ]]\n[[ $- == *u* ]]\n'
        # Simulate environments where the entrypoint inherits nounset.
        result = subprocess.run(['bash', '-u', '-c', snippet], capture_output=True, text=True)
        assert result.returncode == 0, (relative, result.stderr)


def test_v034_start_scripts_explicitly_disable_inherited_nounset():
    for relative in (
        'deploy/ros-test/ros2/start.sh',
        'deploy/e2e/rviz/start.sh',
        'deploy/ros-test/ros1/start.sh',
        'deploy/ros-test/ros1/health.sh',
    ):
        contents = (ROOT / relative).read_text()
        assert contents.index('set +u') < contents.index('source /opt/ros/')
        assert contents.index('source /opt/ros/') < contents.index('set -u')
    assert 'is-ros-translator E2E startup v0.3.4' in (
        ROOT / 'deploy/ros-test/ros2/start.sh').read_text()
