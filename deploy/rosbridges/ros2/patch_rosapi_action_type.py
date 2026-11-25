#!/usr/bin/env python3
"""Fix ROSAPI action discovery in affected ROS 2 Jazzy packages.

Some rosapi versions import get_action_names_and_types from ros2action.api.
That implementation calls a method which is missing from the Jazzy rclpy Node
class and crashes the entire rosapi process. The supported rclpy.action
implementation queries the node's handle.

This Docker-build-time compatibility patch changes only the import.
It is idempotent and fails closed on an unknown package layout.
"""

from __future__ import annotations

from pathlib import Path
import sys


LEGACY_IMPORT = "from ros2action.api import get_action_names_and_types"
CORRECT_IMPORT = "from rclpy.action import get_action_names_and_types"


def patch_rosapi(ros_install: Path) -> str:
    candidates = sorted(ros_install.glob("lib/python*/site-packages/rosapi/proxy.py"))
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected one rosapi/proxy.py under {ros_install}, found {len(candidates)}"
        )
    proxy = candidates[0]
    original = proxy.read_text(encoding="utf-8")
    if CORRECT_IMPORT in original and LEGACY_IMPORT not in original:
        return f"Already compatible: {proxy}"
    if original.count(LEGACY_IMPORT) != 1:
        raise RuntimeError(
            f"Unknown ROSAPI action discovery implementation in {proxy}; "
            "refusing to patch a different version"
        )
    if CORRECT_IMPORT in original:
        raise RuntimeError(f"Conflicting ROSAPI action imports in {proxy}")
    proxy.write_text(original.replace(LEGACY_IMPORT, CORRECT_IMPORT), encoding="utf-8")
    return f"Patched ROSAPI action discovery: {proxy}"


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"Usage: {argv[0]} /opt/ros/jazzy", file=sys.stderr)
        return 2
    try:
        print(patch_rosapi(Path(argv[1]).resolve()))
    except (OSError, RuntimeError) as exc:
        print(f"ROSAPI patch failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
