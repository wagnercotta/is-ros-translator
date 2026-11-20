"""Adapter-level unit tests use a roslibpy stub, not a real ROS VM."""
import sys
import types
import pytest
from is_ros_translator_v2.bridge import RosBridge
from is_ros_translator_v2.errors import GatewayError


class DummyRosMessage(dict):
    pass


class DummyRos2Goal(dict):
    pass


class DummyRos2ActionClient:
    def __init__(self, conn, name, action_type):
        self.name = name
        self.action_type = action_type

    def send_goal(self, goal, resultback, feedback, errback):
        feedback({"distance_remaining": 2.0})
        resultback({"status": 4, "result": {"error_code": 0}})
        return "goal-1"

    def cancel_goal(self, id):
        pass


class DummyRos1Goal:
    def __init__(self, client, message):
        self.status = {"status": 3}
        self.is_finished = True
        self.callbacks = {}

    def on(self, kind, callback):
        self.callbacks[kind] = callback

    def send(self, timeout=None):
        pass

    def wait(self, timeout=None):
        return {"done": True}

    def cancel(self):
        pass


class DummyRos1ActionClient:
    def __init__(self, conn, name, type):
        self.name = name

    def dispose(self):
        pass


def install_stub(monkeypatch):
    lib = types.ModuleType("roslibpy")
    lib.ActionClient = DummyRos2ActionClient
    lib.Goal = DummyRos2Goal
    lib.Message = DummyRosMessage
    lib.__path__ = []
    ros1 = types.ModuleType("roslibpy.ros1")
    ros1.__path__ = []
    actionlib = types.ModuleType("roslibpy.ros1.actionlib")
    actionlib.ActionClient = DummyRos1ActionClient
    actionlib.Goal = DummyRos1Goal
    monkeypatch.setitem(sys.modules, "roslibpy", lib)
    monkeypatch.setitem(sys.modules, "roslibpy.ros1", ros1)
    monkeypatch.setitem(sys.modules, "roslibpy.ros1.actionlib", actionlib)


def test_ros2_action_callback_path(monkeypatch):
    install_stub(monkeypatch)
    feedback = []
    result = RosBridge._ros2_action(object(), "/navigate_to_pose", "nav2_msgs/action/NavigateToPose",
                                      {"pose": {}}, timeout=2, feedback=feedback.append)
    assert result["goal_id"] == "goal-1"
    assert result["status"] == 4
    assert feedback == [{"distance_remaining": 2.0}]


def test_ros1_action_result(monkeypatch):
    install_stub(monkeypatch)
    result = RosBridge._ros1_action(object(), "/move_base", "move_base_msgs/MoveBaseAction", {}, 2, None)
    assert result["status"] == 3
    assert result["result"]["done"]
