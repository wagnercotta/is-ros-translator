"""Static constraints for the optional ROS 1 mirror of physical Create 3 E2E."""

import importlib.util
from pathlib import Path

import pytest
import yaml

from is_ros_translator_v2.config import load_settings


ROOT = Path(__file__).resolve().parents[1]


def test_optional_ros1_lab_preserves_existing_create3():
    physical = load_settings(ROOT / "configs/create3-lab.yaml")
    mirrored = load_settings(ROOT / "configs/create3-ros1-lab.yaml")
    assert mirrored.robots["create_3"] == physical.robots["create_3"]
    assert mirrored.bridges["jazzy_local"] == physical.bridges["jazzy_local"]
    assert mirrored.robots["ros1_sim"].ros_version == 1
    assert mirrored.robots["ros1_sim"].bridge_id == "noetic_local"
    assert mirrored.robots["ros1_sim"].navigation.frame_id == "map"
    assert mirrored.robots["ros1_sim"].navigation.action == "/move_base"
    assert mirrored.robots["ros1_sim"].allowed.actions == ["/move_base"]
    assert not mirrored.robots["ros1_sim"].allowed.publish


def test_ros1_lab_is_opt_in_and_not_publicly_exposed():
    compose = yaml.safe_load((ROOT / "deploy/lab/compose.yaml").read_text())
    services = compose["services"]
    assert services["ros1-lab"]["profiles"] == ["ros1"]
    assert services["ros1-lab"]["network_mode"] == "host"
    assert "ports" not in services["ros1-lab"]
    assert "LAB_CORE_CONFIG" in services["core"]["volumes"][0]
    assert "../../configs/create3-lab.yaml" in services["core"]["volumes"][0]

    start = (ROOT / "deploy/lab/ros1/start.sh").read_text()
    assert "127.0.0.1:6081" in start
    assert "address:=127.0.0.1 port:=9091" in start
    assert "roscore" in start
    assert "rviz -d" in start


def test_ros1_visualisation_endpoints_match_simulator():
    simulator = (ROOT / "deploy/lab/ros1/simulator.py").read_text()
    rviz = yaml.safe_load((ROOT / "deploy/lab/ros1/rviz.rviz").read_text())
    assert "'/move_base'" in simulator
    assert "'/odom'" in simulator
    assert "'/ros1_sim/path'" in simulator
    assert "'/ros1_sim/goal_marker'" in simulator
    displays = rviz["Visualization Manager"]["Displays"]
    topics = {disp.get("Topic") or disp.get("Marker Topic") for disp in displays}
    assert {"/odom", "/ros1_sim/path", "/ros1_sim/goal_marker"} <= topics


def test_communication_e2e_does_not_block_undock_on_dock_odom_offset(monkeypatch):
    """Dock origin precision is not an acceptance criterion for the communication E2E."""
    spec = importlib.util.spec_from_file_location(
        "create3_physical_e2e", ROOT / "create3_physical_e2e.py")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)

    class Client:
        def dock_status(self, *, timeout):
            return {"is_docked": True, "dock_visible": True}

        def request(self, *args, **kwargs):
            pytest.fail("Initial odometry must not gate the undock command")

    class Evidence:
        events = []

        def emit(self, event, **details):
            self.events.append(event)

    class UndockReached(Exception):
        pass

    def stop_at_undock(client, evidence, action, args, timeout):
        assert action == "undock"
        raise UndockReached

    monkeypatch.setattr(runner, "send_action", stop_at_undock)
    evidence = Evidence()
    with pytest.raises(UndockReached):
        runner.run_mission(Client(), evidence, waypoints=[(0.0, 0.0)] * 5,
                           dock_world=(1.5, -1.0), action_timeout=180,
                           status_timeout=30, tolerance=0.25,
                           dock_approach_distance=0.2)
    assert evidence.events == ["initial_dock_status"]
