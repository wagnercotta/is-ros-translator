"""Load historical is.ros RPC descriptors without a protoc build step.

Matches proto/is_ros_legacy.proto (the original source used `is_ros.proto`).
Only a migration shim; newer clients should use Command/Struct contracts.
"""
from google.protobuf import any_pb2, descriptor_pb2, descriptor_pool, message_factory


def legacy_types():
    spec = descriptor_pb2.FileDescriptorProto()
    spec.name = "is_ros_gateway_legacy_compat.proto"
    spec.package = "is.ros"
    spec.syntax = "proto3"
    spec.dependency.append("google/protobuf/any.proto")
    for msg_name, fields in (
        ("ROSTranslateRequest", [("topic", 1, 9), ("any", 2, 11), ("function", 3, 9)]),
        ("ROSTranslateReply", [("data", 1, 9), ("any", 2, 11)]),
    ):
        msg = spec.message_type.add()
        msg.name = msg_name
        for field_name, field_num, field_type in fields:
            field = msg.field.add()
            field.name = field_name
            field.number = field_num
            field.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
            field.type = field_type
            if field_type == descriptor_pb2.FieldDescriptorProto.TYPE_MESSAGE:
                field.type_name = ".google.protobuf.Any"
    pool = descriptor_pool.Default()
    try:
        pool.AddSerializedFile(spec.SerializeToString())
    except Exception:
        # Can be called more than once after hot-reload.
        pass
    def get_class(name):
        descriptor = pool.FindMessageTypeByName(name)
        if hasattr(message_factory, "GetMessageClass"):
            return message_factory.GetMessageClass(descriptor)
        # is-wire/is-msgs currently pin protobuf 3.20.3.
        return message_factory.MessageFactory(pool).GetPrototype(descriptor)

    return get_class("is.ros.ROSTranslateRequest"), get_class("is.ros.ROSTranslateReply")
