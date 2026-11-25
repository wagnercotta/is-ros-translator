"""Read-only E3: distinct PrIS operations for the ROS1 and ROS2 backends."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def runner():
    spec = importlib.util.spec_from_file_location(
        "create3_physical_e2e_e3", ROOT / "create3_physical_e2e.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Client:
    def __init__(self, robot_id, operation, reply):
        self.robot_id = robot_id
        self.operation = operation
        self.reply = reply
        self.calls = []

    def request(self, operation, *, timeout):
        self.calls.append((operation, timeout))
        assert operation == self.operation, "E3 must not trigger another command"
        return self.reply


def map_response(data=None):
    return {"request_id": "req-map", "result": {"map": {
        "info": {"width": 2.0, "height": 2.0, "resolution": 0.1},
        "data": [0, 0, 0, 0] if data is None else data,
    }}}


def odom_response():
    return {"request_id": "req-odom", "result": {"pose": {"pose": {
        "position": {"x": 0.1, "y": 0.2, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    }}}}


def test_e3_sends_different_read_only_commands_to_distinct_backends(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "run")
    ros1 = Client("ros1_sim", "get_map", map_response())
    ros2 = Client("create_3", "get_odometry", odom_response())

    result = runner.run_distinct_commands(ros2, ros1, evidence)

    assert ros1.calls == [("get_map", 30)]
    assert ros2.calls == [("get_odometry", 20)]
    assert evidence.distinct_commands_verified
    assert result["distinct_commands"]["ros1"]["map_cells"] == 4
    assert result["distinct_commands"]["ros2"]["pose_received"] is True
    assert result["distinct_commands"]["ros1"]["request_id"] != result["distinct_commands"]["ros2"]["request_id"]
    logged = [item for item in evidence.log.read_text().splitlines()]
    assert len(logged) == 4  # start, ROS1 response, ROS2 response, verified
    assert '"kind": "distinct_commands_verified"' in logged[-1]
    # The E3 status is reported separately from goal IDs, which do not apply.
    evidence.requests = [
        {"operation": op, "correlated": True, "status": "passed", "goal_ids_returned": []}
        for op in ("get_map", "get_odometry")
    ]
    validation = evidence.validation_results(ros1_mirror=False,
                                             follow_requested=False,
                                             distinct_requested=True)
    assert validation["E3_distinct_commands"] == "PASS"
    assert validation["IS_Wire_request_reply_correlation"] == "PASS"
    assert validation["backend_goal_ids"] == "NOT_AVAILABLE_IN_REPLY"


def test_e3_rejects_invalid_ros1_map_before_contacting_ros2(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "run")
    ros1 = Client("ros1_sim", "get_map", map_response(data=[0]))
    ros2 = Client("create_3", "get_odometry", odom_response())

    with pytest.raises(runner.ExperimentError, match="invalid OccupancyGrid"):
        runner.run_distinct_commands(ros2, ros1, evidence)
    assert ros2.calls == []
    assert not evidence.distinct_commands_verified


def test_e3_only_mode_does_not_require_motion_confirmation(runner):
    args = runner.argument_parser().parse_args(["--distinct-commands-only", "--seed", "2027"])
    assert args.distinct_commands_only
    assert not args.execute and not args.confirm_area_clear


def test_full_mirror_runs_e3_before_physical_mission(runner, monkeypatch, tmp_path):
    order = []
    monkeypatch.setattr(runner, "ISWireClient", lambda *args, **kwargs: object())

    def e3(client, ros1_client, evidence):
        order.append("E3")
        evidence.distinct_commands_verified = True
        return {"distinct_commands": {"tested": True}}

    def mission(*args, **kwargs):
        order.append("mission")
        return {"completed_loops": 5}

    monkeypatch.setattr(runner, "run_distinct_commands", e3)
    monkeypatch.setattr(runner, "run_mission", mission)
    result = runner.main(["--execute", "--confirm-area-clear", "--ros1-mirror",
                          "--broker-uri", "amqp://unused", "--seed", "2027",
                          "--output-root", str(tmp_path)])
    assert result == 0
    assert order == ["E3", "mission"]
