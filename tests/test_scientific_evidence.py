"""Scientific tracing is opt-in, correlated and does not alter ROS behavior."""

import json

import pytest

from is_ros_translator_v2 import evidence as evidence_module
from is_ros_translator_v2.evidence import EvidenceRecorder, evidence_scope, navigation_args
from is_ros_translator_v2.bridge import RosBridge
from is_ros_translator_v2.iswire_transport import ISWireTransport


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_whitelisted_navigation_evidence_no_credentials():
    result = navigation_args({"position": {"x": 1, "y": 2, "token": "secret"},
                              "frame_id": "odom", "password": "secret"})
    assert result == {"position": {"x": 1, "y": 2}, "frame_id": "odom"}


@pytest.mark.asyncio
async def test_e2_core_sequence_has_correlated_ordered_goal_events(
        gateway, tmp_path, monkeypatch):
    recorder = EvidenceRecorder(tmp_path)
    monkeypatch.setattr(evidence_module, "RECORDER", recorder)
    await gateway.start()
    try:
        reply = await gateway.execute({
            "robot_id": "r2", "operation": "follow_waypoints", "request_id": "e2-correlation",
            "args": {"waypoints": [
                {"position": {"x": 0.5, "y": 0.2}},
                {"position": {"x": 0.0, "y": 0.0}},
            ]},
        })
    finally:
        await gateway.stop()
    assert reply["result"]["completed_waypoints"] == 2
    items = records(tmp_path / "core_bridge_events.jsonl")
    steps = [row for row in items if row["event"] == "sequence_step_started"]
    completed = [row for row in items if row["event"] == "sequence_step_completed"]
    prepared = [row for row in items if row["event"] == "navigation_goal_prepared"]
    assert [r["waypoint_index"] for r in steps] == [0, 1]
    assert [r["waypoint_index"] for r in completed] == [0, 1]
    assert [r["waypoint_index"] for r in prepared] == [0, 1]
    assert completed[0]["monotonic_ns"] <= steps[1]["monotonic_ns"]
    assert all(r["request_id"] == "e2-correlation" for r in steps + completed + prepared)
    assert all(r["terminal_status"] == 4 for r in completed)


def test_iswire_correlation_captured_on_struct_request(
        settings, tmp_path, monkeypatch):
    from google.protobuf.json_format import ParseDict
    from google.protobuf.struct_pb2 import Struct

    recorder = EvidenceRecorder(tmp_path)
    monkeypatch.setattr(evidence_module, "RECORDER", recorder)
    transport = ISWireTransport(settings)
    transport._execute = lambda cmd: {"ok": True, "request_id": cmd.request_id,
                                      "robot_id": cmd.robot_id, "result": {"status": 4}}
    class Incoming:
        correlation_id = 987

        def __init__(self, body):
            self.payload = body
            self.body = body.SerializeToString()

        def unpack(self, cls):
            assert cls is Struct
            return self.payload

    incoming = Incoming(ParseDict({"robot_id": "r2", "operation": "navigate",
                   "request_id": "req-7", "args": {"position": {"x": 1, "y": 2}}}, Struct()))
    transport._handle_struct_request(incoming)
    items = records(tmp_path / "core_bridge_events.jsonl")
    assert [r["event"] for r in items] == ["request_received", "reply_prepared"]
    assert all(r["correlation_id"] == "987" for r in items)
    assert all(r["request_id"] == "req-7" for r in items)


def test_disabled_recorder_writes_nothing(tmp_path):
    recorder = EvidenceRecorder(None)
    recorder.emit("core", "test", request_id="unused")
    assert not list(tmp_path.iterdir())


def test_ros2_adapter_goal_id_is_correlated(tmp_path, monkeypatch):
    from test_bridge_actions import install_stub

    install_stub(monkeypatch)
    monkeypatch.setattr(evidence_module, "RECORDER", EvidenceRecorder(tmp_path))
    with evidence_scope(robot_id="r2", request_id="req-ros2", waypoint_index=1):
        result = RosBridge._ros2_action(object(), "/navigate_to_pose",
                 "nav2_msgs/action/NavigateToPose", {"pose": {}}, 2, None)
    assert result["goal_id"] == "goal-1"
    rows = records(tmp_path / "core_bridge_events.jsonl")
    assert [r["event"] for r in rows] == ["goal_sent", "goal_terminal"]
    assert all(r["request_id"] == "req-ros2" for r in rows)
    assert all(r["goal_id"] == "goal-1" for r in rows)
