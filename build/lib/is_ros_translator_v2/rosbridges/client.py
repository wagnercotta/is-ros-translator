"""ROSBridge WebSocket transport client; no local ROS installation is required.

Each robot has exactly one long-lived WebSocket connection, shared by its
publisher, subscribers, service clients and action clients. Blocking roslibpy
calls run on worker threads; callbacks can come from roslibpy's event loop.
"""
from __future__ import annotations
import asyncio
import logging
import threading
import time
from typing import Callable
from ..config import RobotConfig
from ..errors import OperationTimeout, ROSUnavailable

LOG = logging.getLogger(__name__)


class RosBridge:
    def __init__(self, robot: RobotConfig):
        self.robot = robot
        self._ros = None
        self._lock = asyncio.Lock()
        # Keep ROS2 publishers advertised: advertising and immediately
        # unadvertising can drop the first message during DDS discovery.
        self._publishers = {}
        self._publish_lock = threading.Lock()

    async def ensure_connected(self):
        if self._ros is not None and self._ros.is_connected:
            return self._ros
        async with self._lock:
            if self._ros is not None and self._ros.is_connected:
                return self._ros
            def connect():
                import roslibpy
                if self._ros is not None:
                    # Prefer recovering the same Ros object: existing Topic
                    # handles use reconnect_on_close and are bound to it.
                    try:
                        self._ros.connect()
                        self._ros.run(timeout=8)
                        if self._ros.is_connected:
                            return self._ros
                    except Exception:
                        LOG.warning("Could not reconnect existing Ros client", exc_info=True)
                    try:
                        self._ros.close(timeout=2)
                    except Exception:
                        pass
                conn = roslibpy.Ros(host=self.robot.host, port=self.robot.port)
                conn.run(timeout=8)
                with self._publish_lock:
                    self._publishers.clear()  # old handles refer to another Ros client
                return conn
            try:
                self._ros = await asyncio.to_thread(connect)
                return self._ros
            except Exception as exc:
                raise ROSUnavailable(f"Could not connect to {self.robot.host}:{self.robot.port}: {exc}") from exc

    def connected(self) -> bool:
        return bool(self._ros and self._ros.is_connected)

    async def close(self):
        with self._publish_lock:
            publishers = list(self._publishers.values())
            self._publishers.clear()
        for topic in publishers:
            try:
                await asyncio.to_thread(topic.unadvertise)
            except Exception:
                LOG.warning("Could not unadvertise ROS topic", exc_info=True)
        if self._ros:
            conn, self._ros = self._ros, None
            try:
                await asyncio.to_thread(conn.close, timeout=3)
            except Exception:
                LOG.exception("Failed to close rosbridge")

    async def get_topics(self):
        conn = await self.ensure_connected()
        return await asyncio.to_thread(conn.get_topics)

    async def get_services(self):
        conn = await self.ensure_connected()
        return await asyncio.to_thread(conn.get_services)

    async def get_topic_type(self, name):
        conn = await self.ensure_connected()
        return await asyncio.to_thread(conn.get_topic_type, name)

    async def get_service_type(self, name):
        conn = await self.ensure_connected()
        return await asyncio.to_thread(conn.get_service_type, name)

    async def message_details(self, ros_type):
        conn = await self.ensure_connected()
        result = await asyncio.to_thread(conn.get_message_details, ros_type)
        return dict(result)

    async def service_details(self, ros_type):
        conn = await self.ensure_connected()
        result = await asyncio.to_thread(conn.get_service_request_details, ros_type)
        return dict(result)

    async def get_action_type(self, name: str):
        if self.robot.ros_version == 1:
            # ROS1 rosapi has no uniformly portable action_type service.
            return None
        conn = await self.ensure_connected()
        def operation():
            import roslibpy
            service = roslibpy.Service(conn, "/rosapi/action_type", "rosapi_msgs/srv/ActionType")
            response = service.call(roslibpy.ServiceRequest({"action": name}), timeout=5)
            return response.get("type")
        return await asyncio.to_thread(operation)

    async def action_details(self, ros_type: str):
        conn = await self.ensure_connected()
        if self.robot.ros_version == 1:
            # For ROS1 action FooAction, the goal payload has type FooGoal.
            if not ros_type.endswith("Action"):
                raise ValueError("ROS1 action type must end with Action")
            return await self.message_details(ros_type[:-6] + "Goal")
        def operation():
            import roslibpy
            service = roslibpy.Service(conn, "/rosapi/action_goal_details",
                                       "rosapi_msgs/srv/ActionGoalDetails")
            return dict(service.call(roslibpy.ServiceRequest({"type": ros_type}), timeout=5))
        return await asyncio.to_thread(operation)

    async def publish(self, name: str, ros_type: str, data: dict):
        conn = await self.ensure_connected()
        def operation():
            import roslibpy
            key = (name, ros_type)
            with self._publish_lock:
                topic = self._publishers.get(key)
                if topic is None:
                    topic = roslibpy.Topic(conn, name, ros_type,
                                          latch=False, reconnect_on_close=True)
                    topic.advertise()
                    self._publishers[key] = topic
                    # ROS2 DDS needs time to match a newly created publisher
                    # with an existing remote subscription. A persistent
                    # publisher avoids subsequent first-message losses.
                    time.sleep(1.0 if self.robot.ros_version == 2 else 0.15)
                topic.publish(roslibpy.Message(data))
        await asyncio.to_thread(operation)
        # ROS topics are fire-and-forget. The E2E runner independently checks
        # movement, odometry, TF, and Path; this flag is NOT proof of delivery.
        return {"published": True, "delivery_confirmed": False, "topic": name}

    async def service(self, name: str, ros_type: str, data: dict, timeout: float):
        conn = await self.ensure_connected()
        def operation():
            import roslibpy
            client = roslibpy.Service(conn, name, ros_type)
            return dict(client.call(roslibpy.ServiceRequest(data), timeout=timeout))
        try:
            return await asyncio.wait_for(asyncio.to_thread(operation), timeout=timeout + 2)
        except TimeoutError as exc:
            raise OperationTimeout(f"ROS service {name} exceeded {timeout}s") from exc

    async def subscribe(self, name: str, ros_type: str, callback: Callable[[dict], None]):
        conn = await self.ensure_connected()
        def operation():
            import roslibpy
            topic = roslibpy.Topic(conn, name, ros_type, reconnect_on_close=True)
            topic.subscribe(lambda msg: callback(dict(msg)))
            return topic
        return await asyncio.to_thread(operation)

    async def unsubscribe(self, handle):
        await asyncio.to_thread(handle.unsubscribe)

    async def action(self, name: str, ros_type: str, goal_dict: dict, timeout: float,
                     feedback: Callable[[dict], None] | None = None):
        conn = await self.ensure_connected()
        if self.robot.ros_version == 1:
            return await asyncio.to_thread(self._ros1_action, conn, name, ros_type, goal_dict, timeout, feedback)
        return await asyncio.to_thread(self._ros2_action, conn, name, ros_type, goal_dict, timeout, feedback)

    @staticmethod
    def _ros1_action(conn, name, ros_type, goal_dict, timeout, feedback):
        # Legacy entry point retained for existing callers; protocol mapping
        # and action result interpretation belong to the ROS 1 adapter.
        from ..adapters.ros1 import ROS1Adapter
        return ROS1Adapter.send_action(conn, name, ros_type, goal_dict, timeout, feedback)

    @staticmethod
    def _ros2_action(conn, name, ros_type, goal_dict, timeout, feedback):
        from ..adapters.ros2 import ROS2Adapter
        return ROS2Adapter.send_action(conn, name, ros_type, goal_dict, timeout, feedback)
