"""Backward-compatible import for the ROSBridge transport client."""

import time  # Legacy tests patch bridge.time.sleep; this is the same module as client.time.

from .rosbridges.client import RosBridge

__all__ = ["RosBridge"]
