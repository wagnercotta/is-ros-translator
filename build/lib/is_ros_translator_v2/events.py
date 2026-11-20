"""Bounded in-memory event bus with multiple sinks and websocket consumers."""
from __future__ import annotations
import asyncio
import logging
from datetime import datetime, timezone
from typing import Callable
import uuid

LOG = logging.getLogger(__name__)


class EventBus:
    def __init__(self, maxsize=512):
        self._queue = asyncio.Queue(maxsize=maxsize)
        self._sinks = []
        self._listeners: dict[str, asyncio.Queue] = {}
        self._loop = None
        self._task = None
        self.dropped = 0

    def start(self):
        self._loop = asyncio.get_running_loop()
        self._task = asyncio.create_task(self._run(), name="event-bus")

    async def stop(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass

    def add_sink(self, callback):
        self._sinks.append(callback)

    def emit(self, event: dict):
        """Thread-safe entry point for roslibpy callbacks."""
        if not self._loop or self._loop.is_closed():
            return
        self._loop.call_soon_threadsafe(self._enqueue, event)

    def _enqueue(self, event):
        event = {**event, "timestamp": datetime.now(timezone.utc).isoformat()}
        if self._queue.full():
            try:
                self._queue.get_nowait()
                self.dropped += 1
            except asyncio.QueueEmpty:
                pass
        self._queue.put_nowait(event)

    def register(self) -> tuple[str, asyncio.Queue]:
        key = str(uuid.uuid4())
        queue = asyncio.Queue(maxsize=128)
        self._listeners[key] = queue
        return key, queue

    def unregister(self, key: str):
        self._listeners.pop(key, None)

    async def _run(self):
        while True:
            event = await self._queue.get()
            for queue in list(self._listeners.values()):
                if queue.full():
                    try:
                        queue.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                queue.put_nowait(event)
            for sink in self._sinks:
                try:
                    await sink(event)
                except Exception:
                    LOG.exception("Failed to send event to sink")
