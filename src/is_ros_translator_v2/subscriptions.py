"""Leased subscriptions shared among clients; bounded and ref-counted."""
from __future__ import annotations
import asyncio
import time
from dataclasses import dataclass, field
import logging
LOG = logging.getLogger(__name__)
from .models import SubscriptionSpec
from .errors import InvalidRequest


@dataclass
class ActiveSubscription:
    message_type: str
    handle: object
    subscribers: dict[str, float] = field(default_factory=dict)


class SubscriptionManager:
    def __init__(self, gateway):
        self.gateway = gateway
        self._active: dict[tuple[str, str], ActiveSubscription] = {}
        self._lock = asyncio.Lock()
        self._janitor = None

    def start(self):
        self._janitor = asyncio.create_task(self._reap_loop())

    async def stop(self):
        if self._janitor:
            self._janitor.cancel()
            try:
                await self._janitor
            except asyncio.CancelledError:
                pass
        async with self._lock:
            for (rid, _), active in list(self._active.items()):
                try:
                    await self.gateway.bridge(rid).unsubscribe(active.handle)
                except Exception:
                    pass
            self._active.clear()

    async def add(self, spec: SubscriptionSpec):
        robot = self.gateway.robot(spec.robot_id)
        self.gateway.authorize(robot, "subscribe", spec.endpoint)
        key = (spec.robot_id, spec.endpoint)
        lease = spec.lease_seconds or self.gateway.settings.subscription_lease_seconds
        expiry = time.monotonic() + lease
        bridge = self.gateway.bridge(spec.robot_id)
        msg_type = await self.gateway.topic_type(spec.robot_id, spec.endpoint, spec.message_type)
        async with self._lock:
            active = self._active.get(key)
            if active is None:
                if len(self._active) >= self.gateway.settings.max_subscriptions:
                    raise InvalidRequest("Maximum number of ROS topic subscriptions reached")
                def callback(data, robot_id=spec.robot_id, endpoint=spec.endpoint):
                    current = self._active.get((robot_id, endpoint))
                    if current:
                        clients = [s for s, t in list(current.subscribers.items()) if t > time.monotonic()]
                        if clients:
                            self.gateway.events.emit({"kind": "topic", "robot_id": robot_id,
                                                      "endpoint": endpoint, "subscribers": clients,
                                                      "ros_type": current.message_type, "payload": data})
                handle = await bridge.subscribe(spec.endpoint, msg_type, callback)
                active = ActiveSubscription(msg_type, handle)
                self._active[key] = active
            elif active.message_type != msg_type:
                raise InvalidRequest("Topic already subscribed using a different ROS type")
            active.subscribers[spec.subscriber_id] = expiry
        return {"subscribed": True, "robot_id": spec.robot_id, "endpoint": spec.endpoint,
                "subscriber_id": spec.subscriber_id, "lease_seconds": lease, "ros_type": msg_type}

    async def remove(self, spec: SubscriptionSpec):
        key = (spec.robot_id, spec.endpoint)
        async with self._lock:
            active = self._active.get(key)
            if active:
                active.subscribers.pop(spec.subscriber_id, None)
                if not active.subscribers:
                    await self.gateway.bridge(spec.robot_id).unsubscribe(active.handle)
                    self._active.pop(key, None)
        return {"subscribed": False, "robot_id": spec.robot_id, "endpoint": spec.endpoint}

    def summary(self):
        return [{"robot_id": rid, "endpoint": endpoint, "ros_type": value.message_type,
                 "subscribers": sorted(value.subscribers)}
                for (rid, endpoint), value in self._active.items()]

    async def _reap_loop(self):
        while True:
            await asyncio.sleep(5)
            async with self._lock:
                now = time.monotonic()
                for (rid, endpoint), active in list(self._active.items()):
                    for subscriber, expiry in list(active.subscribers.items()):
                        if expiry <= now:
                            active.subscribers.pop(subscriber, None)
                    if not active.subscribers:
                        try:
                            await self.gateway.bridge(rid).unsubscribe(active.handle)
                        except Exception:
                            LOG.exception("Could not unsubscribe expired ROS topic %s", endpoint)
                        finally:
                            self._active.pop((rid, endpoint), None)
