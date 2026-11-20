"""Native Create 3 navigation and catalogue contracts."""

import asyncio
from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from is_ros_translator_v2.config import Settings  # noqa: E402
from is_ros_translator_v2.errors import InvalidRequest  # noqa: E402
from is_ros_translator_v2.gateway import Gateway  # noqa: E402


class BridgeStub:
    def __init__(self):
        self.actions = []

    async def action(self, endpoint, ros_type, goal, timeout, feedback=None):
        self.actions.append((endpoint, ros_type, goal))
        return {"status": 4}


class Create3Tests(unittest.TestCase):
    def setUp(self):
        data = yaml.safe_load((ROOT / "configs/create3.yaml").read_text())
        self.settings = Settings.model_validate(data)
        self.bridge = BridgeStub()
        self.gateway = Gateway(self.settings, bridges={"create_3": self.bridge})

    def test_config_has_native_capabilities(self):
        robot = self.settings.robots["create_3"]
        self.assertEqual(robot.navigation.action, "/navigate_to_position")
        self.assertEqual(robot.navigation.frame_id, "odom")
        self.assertIsNone(robot.map_service)
        self.assertIn("emergency_stop", robot.capabilities)
        self.assertIn("drive_distance", robot.capabilities)
        self.assertNotIn("announce", robot.capabilities)

    def test_generic_navigate_uses_create3_goal_shape(self):
        asyncio.run(self.gateway.execute({
            "robot_id": "create_3", "operation": "navigate",
            "args": {"position": {"x": 0.5, "y": 0.25}, "yaw": 1.57},
        }))
        endpoint, ros_type, goal = self.bridge.actions[-1]
        self.assertEqual(endpoint, "/navigate_to_position")
        self.assertEqual(ros_type, "irobot_create_msgs/action/NavigateToPosition")
        self.assertEqual(goal["goal_pose"]["header"]["frame_id"], "odom")
        self.assertEqual(goal["goal_pose"]["pose"]["position"]["x"], 0.5)
        self.assertTrue(goal["achieve_goal_heading"])
        self.assertNotIn("behavior_tree", goal)

    def test_navigate_rejects_map_frame_without_map(self):
        with self.assertRaisesRegex(InvalidRequest, "odom frame"):
            asyncio.run(self.gateway.execute({
                "robot_id": "create_3", "operation": "navigate",
                "args": {"position": {"x": 0.5, "y": 0.25}, "frame_id": "map"},
            }))
        self.assertEqual(self.bridge.actions, [])

    def test_waypoints_use_the_same_create3_goal_adapter(self):
        result = asyncio.run(self.gateway.execute({
            "robot_id": "create_3", "operation": "follow_waypoints",
            "args": {"waypoints": [
                {"position": {"x": 0.1, "y": 0.0}},
                {"position": {"x": 0.2, "y": 0.0}},
            ]},
        }))
        self.assertEqual(result["result"]["completed_waypoints"], 2)
        self.assertEqual(len(self.bridge.actions), 2)
        self.assertTrue(all("goal_pose" in call[2] for call in self.bridge.actions))

    def test_standard_nav2_and_ros1_goal_formats_are_preserved(self):
        robot = self.settings.robots["create_3"]
        robot.navigation.frame_id = "map"
        robot.navigation.action_type = "nav2_msgs/action/NavigateToPose"
        asyncio.run(self.gateway.execute({
            "robot_id": "create_3", "operation": "navigate",
            "args": {"position": {"x": 0.1, "y": 0.2}},
        }))
        self.assertIn("pose", self.bridge.actions[-1][2])
        self.assertIn("behavior_tree", self.bridge.actions[-1][2])
        robot.ros_version = 1
        robot.navigation.action_type = "move_base_msgs/MoveBaseAction"
        asyncio.run(self.gateway.execute({
            "robot_id": "create_3", "operation": "navigate",
            "args": {"position": {"x": 0.1, "y": 0.2}},
        }))
        self.assertIn("target_pose", self.bridge.actions[-1][2])

    def test_get_map_is_disabled(self):
        with self.assertRaisesRegex(InvalidRequest, "not configured"):
            asyncio.run(self.gateway.execute({"robot_id": "create_3", "operation": "get_map"}))


if __name__ == "__main__":
    unittest.main()
