"""Server-side corroboration for separate concurrent ROS1/ROS2 goals."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def checker():
    spec = importlib.util.spec_from_file_location("e3_checker", ROOT / "verify_e3_overlap.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fixture_evidence(tmp_path, *, ros1_end=200, ros2_start=150, ros2_end=300):
    run = tmp_path / "client"
    run.mkdir()
    details = {"ros1": {"robot_id": "ros1_sim", "request_id": "req1",
                         "goal_id": "g1", "terminal_status": 3,
                         "target_world": [-0.8, 0.6], "frame_id": "map"},
               "ros2": {"robot_id": "create_3", "request_id": "req2",
                         "goal_id": "g2", "terminal_status": 4,
                         "target_world": [0.8, -0.6], "frame_id": "odom"}}
    summary = {"run_id": "run123", "parallel_distinct_goals": {
        **details, "client_rpc_overlap_ms": 40.0}}
    (run / "summary.json").write_text(json.dumps(summary))
    records = []
    server = []
    for version, component, number, start, end in (
            ("ros1", "ros1_adapter", 1, 100, ros1_end),
            ("ros2", "ros2_adapter", 2, ros2_start, ros2_end)):
        goal = details[version]
        rid, req = goal["robot_id"], goal["request_id"]
        correlation = f"cid-{number}"
        args = {"frame_id": goal["frame_id"], "yaw": 0.0,
                "position": {"x": goal["target_world"][0] if number == 1 else -0.7,
                             "y": goal["target_world"][1] if number == 1 else 0.4,
                             "z": 0.0}}
        records.append({"status": "passed", "correlation_id": correlation,
                        "request": {"request_id": req, "robot_id": rid,
                                    "operation": "navigate", "args": args},
                        "reply": {"ok": True, "request_id": req}})
        common = {"request_id": req, "robot_id": rid}
        server.extend([
            {**common, "component": "iswire", "event": "request_received",
             "correlation_id": correlation, "navigation_args": args, "operation": "navigate"},
            {**common, "component": "core", "event": "navigation_goal_prepared",
             "ros_version": number},
            {**common, "component": "bridge", "event": "action_dispatched",
             "ros_version": number},
            {**common, "component": component, "event": "goal_sent", "goal_id": goal["goal_id"],
             "monotonic_ns": start * 1_000_000},
            {**common, "component": component, "event": "goal_terminal", "goal_id": goal["goal_id"],
             "terminal_status": goal["terminal_status"], "monotonic_ns": end * 1_000_000},
            {**common, "component": "iswire", "event": "reply_prepared", "ok": True},
        ])
    (run / "wire_requests.jsonl").write_text("".join(json.dumps(row) + "\n" for row in records))
    core = tmp_path / "core_bridge_events.jsonl"
    core.write_text("".join(json.dumps(row) + "\n" for row in server))
    return run, core


def test_e3_corroborates_overlapping_backend_intervals(checker, tmp_path):
    run, core = fixture_evidence(tmp_path)
    result = checker.verify(run, core)
    assert result["status"] == "PASS"
    assert result["core_overlap_ms"] > 0
    assert result["goals"]["ros1"]["request_id"] == "req1"
    assert result["goals"]["ros2"]["request_id"] == "req2"


def test_e3_rejects_sequential_backend_intervals_even_if_client_overlaps(checker, tmp_path):
    run, core = fixture_evidence(tmp_path, ros2_start=250)
    with pytest.raises(checker.VerificationError, match="no overlap"):
        checker.verify(run, core)


def test_e3_rejects_missing_adapter_proof(checker, tmp_path):
    run, core = fixture_evidence(tmp_path)
    rows = [json.loads(line) for line in core.read_text().splitlines()]
    rows = [row for row in rows if not (row["component"] == "ros1_adapter"
               and row["event"] == "goal_sent")]
    core.write_text("".join(json.dumps(row) + "\n" for row in rows))
    with pytest.raises(checker.VerificationError, match="expected one ros1_adapter.goal_sent"):
        checker.verify(run, core)
