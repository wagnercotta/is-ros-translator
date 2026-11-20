"""Static invariants for the Docker-only ROS 1 + ROS 2 test lab."""
from pathlib import Path
import yaml
from is_ros_translator_v2.config import load_settings

ROOT = Path(__file__).resolve().parents[1]


def test_docker_lab_config_robot_contracts():
    cfg = load_settings(ROOT / "configs/docker-integration.yaml")
    assert cfg.robots["jackal_ros1"].host == "ros1"
    assert cfg.robots["jackal_ros1"].ros_version == 1
    assert cfg.robots["turtlebot_ros2"].host == "ros2"
    assert cfg.robots["turtlebot_ros2"].ros_version == 2
    assert cfg.security.allow_live_schema_fallback is False
    for r in cfg.robots.values():
        assert r.allowed.publish == ["/chatter"]
        assert "/cmd_vel" not in r.allowed.publish


def test_compose_has_private_rosbridge_and_test_runner():
    compose = yaml.safe_load((ROOT / "deploy/ros-test/compose.yaml").read_text())
    services = compose["services"]
    assert {"ros1", "ros2", "rabbitmq", "gateway", "tester"} == set(services)
    assert "ports" not in services["ros1"]
    assert "ports" not in services["ros2"]
    assert "ports" not in services["rabbitmq"]
    assert services["gateway"]["ports"] == ["127.0.0.1:${GATEWAY_HTTP_PORT:-18080}:8080"]
    assert services["tester"]["profiles"] == ["test"]
    for ros in ("ros1", "ros2"):
        assert services[ros]["healthcheck"]["test"] == ["CMD", "/opt/ros-lab/health.sh"]


def test_package_and_image_sources_are_present():
    assert "NOTICE.md" in (ROOT / "deploy/Dockerfile").read_text()
    for ros, label in (("ros1", "noetic"), ("ros2", "jazzy")):
        dockerfile = ROOT / f"deploy/ros-test/{ros}/Dockerfile"
        assert f"ros:{label}-ros-base" in dockerfile.read_text()
        assert (ROOT / f"deploy/ros-test/{ros}/health.sh").exists()
        assert (ROOT / f"deploy/ros-test/{ros}/start.sh").exists()
