"""Best-effort structural ROS schema validation using rosapi typedefs.

Schema validation is deliberately fail-closed for generic calls if rosapi cannot
supply details. It does NOT establish semantic safety or validate ROS-specific
constraints (e.g. frames, enumerations or navigation limits).
"""
from __future__ import annotations
import time
from .errors import InvalidRequest

_INTS = ("int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "byte", "char")
_FLOATS = ("float32", "float64")
_BASE = set(_INTS + _FLOATS + ("bool", "string", "wstring", "time", "duration"))
# ROS 1 typedefs commonly use float32/float64. ROS 2 rosapi may expose their
# IDL names (float/double); interpret them as primitives, not message types.
_IDL_PRIMITIVE_ALIASES = {
    "float": "float32",
    "double": "float64",
    "boolean": "bool",
    "octet": "uint8",
}


class SchemaCache:
    def __init__(self, ttl: int = 300):
        self.ttl = ttl
        self.cache = {}

    def get(self, robot: str, kind: str, ros_type: str):
        item = self.cache.get((robot, kind, ros_type))
        if item and item[0] > time.monotonic():
            return item[1]
        return None

    def put(self, robot: str, kind: str, ros_type: str, schema: dict):
        self.cache[(robot, kind, ros_type)] = (time.monotonic() + self.ttl, schema)

    def invalidate_robot(self, robot: str):
        for key in list(self.cache):
            if key[0] == robot:
                self.cache.pop(key)


def _short(name: str) -> str:
    return name.split("/")[-1]


def _normalise(type_name: str) -> str:
    return type_name.replace("/msg/", "/").replace("/srv/", "/")


def _primitive_check(value, kind: str, path: str):
    if kind in _INTS:
        if isinstance(value, bool) or not isinstance(value, int):
            raise InvalidRequest(f"{path}: expected {kind}")
        bits = 8 if kind in ("byte", "char") else int("".join(c for c in kind if c.isdigit()) or "8")
        lo = -(2 ** (bits - 1)) if kind.startswith("int") else 0
        hi = 2 ** (bits - 1) - 1 if kind.startswith("int") else 2 ** bits - 1
        if not lo <= value <= hi:
            raise InvalidRequest(f"{path}: integer outside {kind} range")
    elif kind in _FLOATS:
        import math
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(float(value)):
            raise InvalidRequest(f"{path}: expected a finite number")
    elif kind == "bool" and not isinstance(value, bool):
        raise InvalidRequest(f"{path}: expected bool")
    elif kind in ("string", "wstring") and not isinstance(value, str):
        raise InvalidRequest(f"{path}: expected string")
    elif kind in ("time", "duration") and (not isinstance(value, dict) or not set(value).issubset({"secs", "nsecs", "sec", "nanosec"})):
        raise InvalidRequest(f"{path}: expected ROS time mapping")


def validate_ros_payload(payload: dict, schema: dict, ros_type: str):
    """Validate listed fields when rosapi returns typedefs; omitted fields use ROS defaults.

    ROS2 rosapi versions may use different response shapes. Unsupported shapes
    fail with a clear error rather than permitting unchecked arbitrary messages.
    """
    if not isinstance(payload, dict):
        raise InvalidRequest("ROS payload must be a JSON object")
    typedefs = schema.get("typedefs", []) if isinstance(schema, dict) else []
    if not isinstance(typedefs, list) or not typedefs:
        raise InvalidRequest("rosapi schema missing typedefs; generic operation denied")
    specs = {_normalise(x["type"]): x for x in typedefs if isinstance(x, dict) and "type" in x}

    def validate_map(item: dict, type_name: str, path: str, depth: int = 0):
        if depth > 15:
            raise InvalidRequest("ROS message nested too deeply")
        found = specs.get(_normalise(type_name))
        if found is None:
            # ROS definitions may carry only the package-less type at root.
            matches = [v for k, v in specs.items() if _short(k) == _short(type_name)]
            found = matches[0] if len(matches) == 1 else None
        if found is None:
            raise InvalidRequest(f"Missing typedef for {type_name}")
        names = found.get("fieldnames", [])
        types = found.get("fieldtypes", [])
        lengths = found.get("fieldarraylen", [])
        if len(names) != len(types):
            raise InvalidRequest("Invalid rosapi typedef format")
        unknown = set(item) - set(names)
        if unknown:
            raise InvalidRequest(f"{path}: unknown ROS field(s): {', '.join(sorted(unknown))}")
        for key, val in item.items():
            i = names.index(key)
            fieldtype = types[i]
            array = fieldtype.endswith("[]") or (i < len(lengths) and lengths[i] != -1)
            if fieldtype.endswith("[]"):
                fieldtype = fieldtype[:-2]
            elif "[" in fieldtype:
                fieldtype = fieldtype.split("[", 1)[0]
                array = True
            def check_value(v, p):
                simple = _IDL_PRIMITIVE_ALIASES.get(_short(fieldtype), _short(fieldtype))
                if simple in _BASE:
                    _primitive_check(v, simple, p)
                else:
                    if not isinstance(v, dict):
                        raise InvalidRequest(f"{p}: expected nested object {fieldtype}")
                    validate_map(v, fieldtype, p, depth + 1)
            if array:
                if not isinstance(val, list):
                    raise InvalidRequest(f"{path}.{key}: expected array")
                if i < len(lengths) and lengths[i] > 0 and len(val) != lengths[i]:
                    raise InvalidRequest(f"{path}.{key}: expected length {lengths[i]}")
                for j, each in enumerate(val):
                    check_value(each, f"{path}.{key}[{j}]")
            else:
                check_value(val, f"{path}.{key}")

    root_type = ros_type
    if _normalise(ros_type) not in specs:
        # ROS 2 rosapi action_goal_details/service_request_details return their
        # corresponding *_Goal / *_Request typedef as the first entry.
        first = typedefs[0].get("type")
        if first and (_short(first).endswith(("_Request", "_Goal", "Request", "Goal"))
                      or _short(ros_type) in _short(first)):
            root_type = first
    validate_map(payload, root_type, "payload")
