from is_ros_translator_v2.legacy_proto import legacy_types
from google.protobuf.any_pb2 import Any


def test_legacy_descriptors_roundtrip():
    Request, Reply = legacy_types()
    req = Request()
    req.topic = "/move_base/goal"
    req.function = "RTR_Translate"
    raw = req.SerializeToString()
    restored = Request.FromString(raw)
    assert restored.topic == req.topic
    assert restored.function == req.function
    assert hasattr(restored, "any")
    rep = Reply(data="{}")
    assert Reply.FromString(rep.SerializeToString()).data == "{}"
