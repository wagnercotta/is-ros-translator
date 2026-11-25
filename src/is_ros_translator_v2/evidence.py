"""Optional append-only scientific evidence for Core → bridge → ROS actions.

Enable with ROSTRANSLATOR_EVIDENCE_DIR=/absolute/path. No broker URL,
credentials, HTTP headers, or arbitrary service payloads are recorded.
Failures to write evidence never alter robot command execution.
"""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from datetime import datetime, timezone
import json
import logging
import os
from pathlib import Path
import threading
import time
from typing import Any

LOG = logging.getLogger(__name__)
_CONTEXT: ContextVar[dict] = ContextVar("rostranslator_evidence_context", default={})


def navigation_args(args: object) -> dict | None:
    """Whitelist coordinates used by E1/E2; avoid logging arbitrary request data."""
    if not isinstance(args, dict):
        return None
    if isinstance(args.get("waypoints"), list):
        return {"waypoints": [navigation_args(item) for item in args["waypoints"]]}
    position = args.get("position")
    if not isinstance(position, dict):
        return None
    safe_position = {key: position[key] for key in ("x", "y", "z") if key in position}
    return {**{key: args[key] for key in ("yaw", "frame_id") if key in args},
            "position": safe_position}


class EvidenceRecorder:
    def __init__(self, directory: str | Path | None):
        self.path = Path(directory).expanduser() / "core_bridge_events.jsonl" if directory else None
        self._lock = threading.Lock()
        self._warned = False

    def emit(self, component: str, event: str, **fields: Any) -> None:
        if self.path is None:
            return
        record = {"at_utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
                  "monotonic_ns": time.monotonic_ns(),
                  "component": component, "event": event,
                  **_CONTEXT.get(), **fields}
        try:
            with self._lock:
                self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
                fd = os.open(self.path, os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o600)
                with os.fdopen(fd, "a", encoding="utf-8") as output:
                    output.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
        except OSError as exc:
            if not self._warned:
                LOG.warning("Could not write Core/bridge evidence: %s", exc)
                self._warned = True


RECORDER = EvidenceRecorder(os.environ.get("ROSTRANSLATOR_EVIDENCE_DIR"))


def evidence(component: str, event: str, **fields: Any) -> None:
    RECORDER.emit(component, event, **fields)


@contextmanager
def evidence_scope(**context: Any):
    """Context propagates automatically into asyncio.to_thread adapter calls."""
    token = _CONTEXT.set({**_CONTEXT.get(), **context})
    try:
        yield
    finally:
        _CONTEXT.reset(token)


def evidence_context() -> dict:
    return dict(_CONTEXT.get())
