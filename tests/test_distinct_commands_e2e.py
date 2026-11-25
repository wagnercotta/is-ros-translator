"""H1 read-only diagnostics and E3 concurrent ROS1/ROS2 navigation."""

import importlib.util
import json
from pathlib import Path
import time

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


def test_h1_sends_different_read_only_commands_to_distinct_backends(runner, tmp_path):
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
    # H1 is a read-only diagnostic, separate from E3 navigation concurrency.
    evidence.requests = [
        {"operation": op, "correlated": True, "status": "passed", "goal_ids_returned": []}
        for op in ("get_map", "get_odometry")
    ]
    validation = evidence.validation_results(ros1_mirror=False,
                                             follow_requested=False,
                                             distinct_requested=True)
    assert validation["H1_distinct_commands"] == "PASS"
    assert validation["IS_Wire_request_reply_correlation"] == "PASS"
    assert validation["backend_goal_ids"] == "NOT_AVAILABLE_IN_REPLY"


def test_h1_rejects_invalid_ros1_map_before_contacting_ros2(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "run")
    ros1 = Client("ros1_sim", "get_map", map_response(data=[0]))
    ros2 = Client("create_3", "get_odometry", odom_response())

    with pytest.raises(runner.ExperimentError, match="invalid OccupancyGrid"):
        runner.run_distinct_commands(ros2, ros1, evidence)
    assert ros2.calls == []
    assert not evidence.distinct_commands_verified


def test_h1_only_mode_does_not_require_motion_confirmation(runner):
    args = runner.argument_parser().parse_args(["--distinct-commands-only", "--seed", "2027"])
    assert args.distinct_commands_only
    assert not args.execute and not args.confirm_area_clear


def test_full_mirror_runs_h1_before_physical_mission(runner, monkeypatch, tmp_path):
    order = []
    monkeypatch.setattr(runner, "ISWireClient", lambda *args, **kwargs: object())

    def h1(client, ros1_client, evidence):
        order.append("H1")
        evidence.distinct_commands_verified = True
        return {"distinct_commands": {"tested": True}}

    def mission(*args, **kwargs):
        order.append("mission")
        return {"completed_loops": 5}

    monkeypatch.setattr(runner, "run_distinct_commands", h1)
    monkeypatch.setattr(runner, "run_mission", mission)
    result = runner.main(["--execute", "--confirm-area-clear", "--ros1-mirror",
                          "--broker-uri", "amqp://unused", "--seed", "2027",
                          "--output-root", str(tmp_path)])
    assert result == 0
    assert order == ["H1", "mission"]


def test_e3_parallel_goals_have_distinct_positions_and_overlapping_rpcs(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "parallel-run")

    class NavigateClient:
        def __init__(self, robot_id, status):
            self.robot_id = robot_id
            self.status = status
            self.calls = []

        def request(self, operation, args, *, timeout):
            assert operation == "navigate"
            self.calls.append(args)
            time.sleep(0.04)
            return {"request_id": f"req-{self.robot_id}", "result": {
                "goal_id": f"goal-{self.robot_id}", "status": self.status}}

    ros1 = NavigateClient("ros1_sim", 3)
    ros2 = NavigateClient("create_3", 4)
    result = runner.run_parallel_distinct_goals(
        ros2, ros1, evidence, ros1_world=(-0.8, 0.6),
        ros2_world=(0.8, -0.6), dock_world=(1.5, -1.0),
        yaw=0.0, timeout=120)

    assert ros1.calls[0]["frame_id"] == "map"
    assert ros1.calls[0]["position"]["x"] == -0.8
    assert ros2.calls[0]["frame_id"] == "odom"
    assert ros2.calls[0]["position"]["x"] == -0.7
    assert ros2.calls[0]["position"]["y"] == 0.4
    assert result["parallel_distinct_goals"]["client_rpc_overlap_ms"] > 0
    assert result["parallel_distinct_goals"]["server_action_overlap"] == "NOT_VERIFIED_FROM_CLIENT_ALONE"
    assert evidence.parallel_goals_verified
    assert evidence.motion_attempted
    validation = evidence.validation_results(
        ros1_mirror=True, follow_requested=False, parallel_requested=True)
    assert validation["E3_concurrent_dispatch"] == "PASS"
    assert validation["E3_backend_action_overlap"] == "NOT_VERIFIED_FROM_CLIENT_ALONE"
    events = [json.loads(line) for line in evidence.log.read_text().splitlines()]
    starts = [event for event in events if event["kind"] == "parallel_request_started"]
    ends = [event for event in events if event["kind"] == "parallel_request_completed"]
    assert {event["robot_id"] for event in starts} == {"ros1_sim", "create_3"}
    assert {event["robot_id"] for event in ends} == {"ros1_sim", "create_3"}


def test_e3_rejects_same_or_near_targets_without_movement(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "parallel-run")
    ros1 = Client("ros1_sim", "navigate", {})
    ros2 = Client("create_3", "navigate", {})
    with pytest.raises(runner.ExperimentError, match="at least 0.5m apart"):
        runner.run_parallel_distinct_goals(
            ros2, ros1, evidence, ros1_world=(0.0, 0.0),
            ros2_world=(0.2, 0.2), dock_world=(1.5, -1.0),
            yaw=0.0, timeout=120)
    assert ros1.calls == ros2.calls == []


def test_e3_generates_five_deterministic_distinct_pairs(runner):
    a = runner.parallel_waypoint_pairs(2027)
    b = runner.parallel_waypoint_pairs(2027)
    assert a == b
    assert len(a) == 5
    assert a[0] == ((-0.8, 0.6), (0.8, -0.6))
    assert all(runner.math.dist(ros1, ros2) >= 0.5 for ros1, ros2 in a)
    assert len({target for pair in a for target in pair}) >= 6


def test_e3_plan_only_prints_all_five_pairs_without_robot_calls(runner, monkeypatch, capsys):
    def no_client(*args, **kwargs):
        pytest.fail("E3 plan must not instantiate an IS-Wire client")

    monkeypatch.setattr(runner, "ISWireClient", no_client)
    assert runner.main(["--plan-parallel-goals", "--seed", "2027"]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["mode"].startswith("E3_PLAN_ONLY")
    assert len(plan["parallel_e3_waypoints"]) == 5
    assert plan["parallel_e3_waypoints"][1]["return_world_xy_m"] == [0.0, 0.0]
    assert plan["waypoints"] == []
    assert plan["final_actions"] == []


def test_e3_rejects_an_invalid_later_pair_before_any_navigation(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "invalid-cycles")
    targets = runner.parallel_waypoint_pairs(2027)
    targets[-1] = ((0.1, 0.1), (0.2, 0.2))
    with pytest.raises(runner.ExperimentError, match="Invalid E3 loop 5"):
        runner.run_parallel_distinct_cycles(
            Client("create_3", "navigate", {}), Client("ros1_sim", "navigate", {}),
            evidence, target_pairs=targets, dock_world=(1.5, -1.0),
            yaw=0.0, timeout=120)
    assert not evidence.motion_attempted
    assert not evidence.parallel_goals_verified


def test_e3_runs_each_distinct_pair_then_both_origins(runner, tmp_path):
    evidence = runner.Evidence(tmp_path / "five-cycles")
    order = []

    class NavigateClient:
        def __init__(self, rid, status):
            self.robot_id = rid
            self.status = status
            self.count = 0

        def request(self, operation, args, *, timeout):
            assert operation == "navigate"
            self.count += 1
            order.append((self.robot_id, args["position"]["x"], args["position"]["y"]))
            time.sleep(0.01)
            return {"request_id": f"{self.robot_id}-{self.count}", "result": {
                "goal_id": f"goal-{self.robot_id}-{self.count}", "status": self.status}}

    ros1 = NavigateClient("ros1_sim", 3)
    ros2 = NavigateClient("create_3", 4)
    pairs = runner.parallel_waypoint_pairs(2027)
    result = runner.run_parallel_distinct_cycles(
        ros2, ros1, evidence, target_pairs=pairs,
        dock_world=(1.5, -1.0), yaw=0.0, timeout=120)

    assert ros1.count == ros2.count == 10
    assert len(order) == 20
    assert result["parallel_completed_loops"] == 5
    assert result["parallel_command_pairs"] == 10
    assert evidence.parallel_goals_verified
    assert len(result["parallel_distinct_cycles"]) == 5
    for i, cycle in enumerate(result["parallel_distinct_cycles"]):
        assert cycle["loop"] == i + 1
        assert cycle["target"]["ros1"]["target_world"] == list(pairs[i][0])
        assert cycle["target"]["ros2"]["target_world"] == list(pairs[i][1])
        assert cycle["return"]["ros1"]["target_world"] == [0.0, 0.0]
        assert cycle["return"]["ros2"]["target_world"] == [0.0, 0.0]
        assert cycle["target"]["client_rpc_overlap_ms"] > 0
        assert cycle["return"]["client_rpc_overlap_ms"] > 0
    events = [json.loads(row) for row in evidence.log.read_text().splitlines()]
    assert len([x for x in events if x["kind"] == "parallel_pair_verified"]) == 10
    assert len([x for x in events if x["kind"] == "parallel_loop_pass"]) == 5
    assert len([x for x in events if x["kind"] == "parallel_cycles_verified"]) == 1


def test_e3_physical_mode_requires_explicit_supervision(runner):
    with pytest.raises(SystemExit) as exc:
        runner.main(["--parallel-goals-only", "--broker-uri", "amqp://unused"])
    assert exc.value.code == 2


def test_e3_only_mode_does_not_redock_after_parallel_goals(runner, monkeypatch, tmp_path):
    calls = []

    class StubClient:
        def __init__(self, uri, rid, dock_topic, *, evidence):
            self.robot_id = rid

        def dock_status(self, *, timeout):
            calls.append("dock-status")
            return {"is_docked": False}

        def request(self, operation, *, timeout):
            calls.append(operation)
            assert operation == "get_odometry"
            return odom_response()

    def issue_parallel(client, ros1_client, evidence, **kwargs):
        calls.append("parallel")
        assert client.robot_id == "create_3"
        assert ros1_client.robot_id == "ros1_sim"
        assert len(kwargs["target_pairs"]) == 5
        evidence.parallel_goals_verified = True
        return {"parallel_distinct_cycles": [], "parallel_completed_loops": 5}

    monkeypatch.setattr(runner, "ISWireClient", StubClient)
    monkeypatch.setattr(runner, "run_parallel_distinct_cycles", issue_parallel)
    result = runner.main(["--parallel-goals-only", "--confirm-area-clear",
                          "--broker-uri", "amqp://unused", "--output-root", str(tmp_path)])
    assert result == 0
    assert calls == ["dock-status", "get_odometry", "parallel"]
    summary_path = next(tmp_path.glob("*/summary.json"))
    summary = json.loads(summary_path.read_text())
    assert summary["status"] == "passed"
    assert summary["final_actions"] == []
    assert summary["validation"]["E3_concurrent_dispatch"] == "PASS"


def test_full_e3_runs_after_five_loops_and_before_docking(runner, monkeypatch, tmp_path):
    evidence = runner.Evidence(tmp_path / "full-run")
    observed = []

    class StubClient:
        robot_id = "create_3"

        def dock_status(self, *, timeout):
            return {"is_docked": False}

        def request(self, operation, *, timeout):
            assert operation == "get_odometry"
            return odom_response()

    def move(*args, **kwargs):
        observed.append("navigate")
        return {}

    def parallel(*args, **kwargs):
        observed.append("parallel")
        assert len(kwargs["target_pairs"]) == 5
        return {"parallel_distinct_cycles": [], "parallel_completed_loops": 5}

    def dock(*args, **kwargs):
        observed.append("dock")
        return {"is_docked": True}

    monkeypatch.setattr(runner, "move", move)
    monkeypatch.setattr(runner, "run_parallel_distinct_cycles", parallel)
    monkeypatch.setattr(runner, "approach_and_dock", dock)
    runner.run_mission(StubClient(), evidence, waypoints=[(0.0, 0.0)] * 5,
                       dock_world=(1.5, -1.0), action_timeout=180,
                       status_timeout=30, tolerance=0.25,
                       dock_approach_distance=0.2,
                       ros1_client=object(),
                       parallel_target_pairs=runner.parallel_waypoint_pairs(2027))
    assert observed == ["navigate"] * 10 + ["parallel", "dock"]
