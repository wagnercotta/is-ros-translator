"""Compose regression for the RViz Docker IPC namespace boot failure."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_rviz_joins_ros2_shareable_ipc_namespace():
    services = yaml.safe_load((ROOT / "deploy/e2e/compose.yaml").read_text())["services"]
    donor = services["ros2"]
    rviz = services["rviz"]
    assert donor["ipc"] == "shareable", "Docker requires the donor container to have shareable IPC"
    assert donor["shm_size"] == "512m"
    assert rviz["ipc"] == "service:ros2"
    assert rviz["network_mode"] == "service:ros2"
    assert rviz["depends_on"]["ros2"]["condition"] == "service_started"
    assert donor["ports"] == ["127.0.0.1:6080:6080"]
