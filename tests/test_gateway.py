import asyncio
import math
import pytest
from is_ros_translator_v2.gateway import pose_stamped
from is_ros_translator_v2.models import Command, Target, SubscriptionSpec
from is_ros_translator_v2.errors import AccessDenied, InvalidRequest, NoRobot


@pytest.mark.asyncio
async def test_ros1_navigation(gateway):
    await gateway.start()
    result = await gateway.execute({"robot_id": "r1", "operation": "navigate", "args": {"position": {"x": 1, "y": 2}, "yaw": 1.57}})
    assert result["ok"]
    _, endpoint, action_type, goal = gateway.bridge("r1").sent[0]
    assert endpoint == "/move_base"
    assert action_type == "move_base_msgs/MoveBaseAction"
    assert goal["target_pose"]["pose"]["position"]["x"] == 1
    await gateway.stop()


@pytest.mark.asyncio
async def test_ros2_navigation(gateway):
    await gateway.start()
    result = await gateway.execute({"robot_id": "r2", "operation": "navigate", "args": {"position": {"x": 3, "y": 4}, "yaw": math.pi}})
    _, endpoint, action_type, goal = gateway.bridge("r2").sent[0]
    assert endpoint == "/navigate_to_pose"
    assert action_type == "nav2_msgs/action/NavigateToPose"
    assert goal["pose"]["pose"]["orientation"]["w"] == pytest.approx(0, abs=1e-9)
    await gateway.stop()


@pytest.mark.asyncio
async def test_waypoint_sequencing(gateway):
    await gateway.start()
    result = await gateway.execute({"robot_id": "r2", "operation": "follow_waypoints",
              "args": {"waypoints": [{"position": {"x": 1, "y": 1}}, {"position": {"x": 2, "y": 2}}]}})
    assert result["result"]["completed_waypoints"] == 2
    assert len(gateway.bridge("r2").sent) == 2
    await gateway.stop()


@pytest.mark.asyncio
async def test_semantic_map(gateway):
    await gateway.start()
    result = await gateway.execute({"robot_id": "r1", "operation": "get_map"})
    assert result["result"]["map"]["info"]["resolution"] == 0.2
    await gateway.stop()


@pytest.mark.asyncio
async def test_semantic_odometry(gateway):
    await gateway.start()
    result = await gateway.execute({"robot_id": "r2", "operation": "get_odometry"})
    assert result["result"]["pose"]["pose"]["position"]["x"] == 4
    assert gateway.bridge("r2").listeners == {}
    await gateway.stop()


@pytest.mark.asyncio
async def test_generic_publish_schema(gateway):
    await gateway.start()
    result = await gateway.execute({"robot_id": "r2", "operation": "ros.publish",
         "args": {"endpoint": "/chatter", "ros_type": "std_msgs/msg/String", "payload": {"data": "Hello"}}})
    assert result["result"]["published"]
    with pytest.raises(InvalidRequest):
        await gateway.execute({"robot_id": "r2", "operation": "ros.publish", "args": {
            "endpoint": "/chatter", "payload": {"bogus": 2}}})
    await gateway.stop()


@pytest.mark.asyncio
async def test_deny_arbitrary_topic_and_actions(gateway):
    await gateway.start()
    with pytest.raises(AccessDenied):
        await gateway.execute({"robot_id": "r2", "operation": "ros.publish", "args": {
            "endpoint": "/cmd_vel", "payload": {"linear": {"x": 3}}}})
    with pytest.raises(AccessDenied):
        await gateway.execute({"robot_id": "r2", "operation": "ros.action", "args": {
            "endpoint": "/dangerous", "ros_type": "x/action/Do", "payload": {}}})
    with pytest.raises(NoRobot):
        await gateway.execute({"robot_id": "missing", "operation": "get_map"})
    await gateway.stop()


@pytest.mark.asyncio
async def test_subscription_shared_and_unsubscribe(gateway):
    await gateway.start()
    a = SubscriptionSpec(robot_id="r2", endpoint="/chatter", subscriber_id="alice")
    b = SubscriptionSpec(robot_id="r2", endpoint="/chatter", subscriber_id="bob")
    await gateway.subscriptions.add(a)
    await gateway.subscriptions.add(b)
    assert len(gateway.subscriptions.summary()) == 1
    assert len(gateway.subscriptions.summary()[0]["subscribers"]) == 2
    await gateway.subscriptions.remove(a)
    assert "/chatter" in gateway.bridge("r2").listeners
    await gateway.subscriptions.remove(b)
    assert "/chatter" not in gateway.bridge("r2").listeners
    await gateway.stop()


@pytest.mark.asyncio
async def test_subscription_event(gateway):
    await gateway.start()
    spec = SubscriptionSpec(robot_id="r1", endpoint="/chatter", subscriber_id="a")
    await gateway.subscriptions.add(spec)
    key, events = gateway.events.register()
    gateway.bridge("r1").listeners["/chatter"]({"data": "reply"})
    event = await asyncio.wait_for(events.get(), timeout=1)
    assert event["payload"]["data"] == "reply"
    assert event["subscribers"] == ["a"]
    gateway.events.unregister(key)
    await gateway.stop()


@pytest.mark.asyncio
async def test_subscribe_from_command_transport(gateway):
    await gateway.start()
    added = await gateway.execute({"robot_id": "r2", "operation": "ros.subscribe",
                                   "args": {"endpoint": "/chatter", "subscriber_id": "remote"}})
    assert added["result"]["subscribed"]
    removed = await gateway.execute({"robot_id": "r2", "operation": "ros.unsubscribe",
                                   "args": {"endpoint": "/chatter", "subscriber_id": "remote"}})
    assert not removed["result"]["subscribed"]
    await gateway.stop()
