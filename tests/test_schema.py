import pytest
from is_ros_translator_v2.schema import validate_ros_payload, SchemaCache
from is_ros_translator_v2.errors import InvalidRequest
from is_ros_translator_v2.policy import require_allowed
from is_ros_translator_v2.config import RobotConfig

SCHEMA = {"typedefs": [
    {"type": "custom/Thing", "fieldnames": ["name", "count", "values", "location"],
     "fieldtypes": ["string", "uint8", "float64[]", "custom/Point"], "fieldarraylen": [-1, -1, 0, -1]},
    {"type": "custom/Point", "fieldnames": ["x", "y"],
     "fieldtypes": ["float64", "float64"], "fieldarraylen": [-1, -1]},
]}


def test_schema_accepts_nested_and_arrays():
    validate_ros_payload({"name": "thing", "count": 2, "values": [1.4, 2.5],
                          "location": {"x": 1.1, "y": 2.2}}, SCHEMA, "custom/msg/Thing")


ROS2_POSE_STAMPED = {"typedefs": [
    {"type": "geometry_msgs/msg/PoseStamped", "fieldnames": ["header", "pose"],
     "fieldtypes": ["std_msgs/msg/Header", "geometry_msgs/msg/Pose"], "fieldarraylen": [-1, -1]},
    {"type": "std_msgs/msg/Header", "fieldnames": ["frame_id"],
     "fieldtypes": ["string"], "fieldarraylen": [-1]},
    {"type": "geometry_msgs/msg/Pose", "fieldnames": ["position", "orientation"],
     "fieldtypes": ["geometry_msgs/msg/Point", "geometry_msgs/msg/Quaternion"], "fieldarraylen": [-1, -1]},
    {"type": "geometry_msgs/msg/Point", "fieldnames": ["x", "y", "z"],
     "fieldtypes": ["double", "double", "double"], "fieldarraylen": [-1, -1, -1]},
    {"type": "geometry_msgs/msg/Quaternion", "fieldnames": ["x", "y", "z", "w"],
     "fieldtypes": ["double", "double", "double", "double"], "fieldarraylen": [-1, -1, -1, -1]},
]}


def test_ros2_idl_double_fields_are_validated_as_numbers():
    """Regression for v0.4.1 E2E: orientation.x was interpreted as an object."""
    payload = {
        "header": {"frame_id": "map"},
        "pose": {"position": {"x": 1.25, "y": -2.0, "z": 0.0},
                 "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}},
    }
    validate_ros_payload(payload, ROS2_POSE_STAMPED, "geometry_msgs/msg/PoseStamped")
    payload["pose"]["orientation"]["x"] = "not-a-number"
    with pytest.raises(InvalidRequest, match="expected a finite number"):
        validate_ros_payload(payload, ROS2_POSE_STAMPED, "geometry_msgs/msg/PoseStamped")


def test_ros2_idl_float_boolean_and_octet_aliases():
    schema = {"typedefs": [{"type": "custom/msg/Sensor", "fieldnames": ["value", "ok", "raw"],
                           "fieldtypes": ["float", "boolean", "octet"], "fieldarraylen": [-1] * 3}]}
    validate_ros_payload({"value": 1.5, "ok": True, "raw": 255}, schema, "custom/msg/Sensor")
    with pytest.raises(InvalidRequest):
        validate_ros_payload({"value": float("nan")}, schema, "custom/msg/Sensor")
    with pytest.raises(InvalidRequest):
        validate_ros_payload({"raw": 256}, schema, "custom/msg/Sensor")


@pytest.mark.parametrize("payload", [
    {"unexpected": "x"}, {"count": -1}, {"count": 256},
    {"values": "not-list"}, {"location": {"x": "wrong"}},
])
def test_schema_rejects_bad_payload(payload):
    with pytest.raises(InvalidRequest):
        validate_ros_payload(payload, SCHEMA, "custom/Thing")


def test_schema_fails_closed():
    with pytest.raises(InvalidRequest):
        validate_ros_payload({"x": 2}, {}, "custom/Thing")


def test_default_deny():
    robot = RobotConfig(ros_version=2, host="localhost")
    with pytest.raises(Exception):
        require_allowed(robot, "publish", "/cmd_vel")


def test_cache_robot_isolation():
    cache = SchemaCache(300)
    cache.put("r1", "message", "std_msgs/String", SCHEMA)
    assert cache.get("r1", "message", "std_msgs/String") is not None
    assert cache.get("r2", "message", "std_msgs/String") is None
    cache.invalidate_robot("r1")
    assert cache.get("r1", "message", "std_msgs/String") is None
