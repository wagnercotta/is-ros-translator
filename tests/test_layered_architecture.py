"""Regression contracts for Core, ROS Adapters and ROSBridge instances."""

import asyncio
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import yaml
from pydantic import ValidationError


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from is_ros_translator_v2.adapters.ros1 import ROS1Adapter  # noqa: E402
from is_ros_translator_v2.adapters.ros2 import ROS2Adapter  # noqa: E402
from is_ros_translator_v2.config import Settings  # noqa: E402
from is_ros_translator_v2.core.gateway import Gateway  # noqa: E402
from is_ros_translator_v2.rosbridges.client import RosBridge  # noqa: E402


class FakeBridge:
    def __init__(self):
        self.actions = []

    async def action(self, endpoint, ros_type, goal, timeout, feedback=None):
        self.actions.append((endpoint, ros_type, goal))
        return {"status": 4}


class LayeredArchitectureTests(unittest.TestCase):
    def setUp(self):
        self.data = yaml.safe_load((ROOT / "configs/multi-distro.example.yaml").read_text())
        self.settings = Settings.model_validate(self.data)

    def test_bridge_registry_with_three_distributions(self):
        self.assertEqual(set(self.settings.bridges), {"jazzy_local", "humble_local", "noetic_local"})
        self.assertEqual(self.settings.bridges["jazzy_local"].distribution, "jazzy")
        self.assertEqual(self.settings.bridges["humble_local"].domain_id, 10)
        self.assertIsNone(self.settings.bridges["noetic_local"].domain_id)

    def test_robot_endpoint_resolution_uses_registry(self):
        for robot_id, port in (("create_3", 9090), ("ros1_robot", 9091), ("ros2_humble_robot", 9092)):
            self.assertIsNone(self.settings.robots[robot_id].host)
            resolved = self.settings.resolved_robot(robot_id)
            self.assertEqual(resolved.host, "127.0.0.1")
            self.assertEqual(resolved.port, port)

        gateway = Gateway(self.settings)
        self.assertTrue(all(isinstance(gateway.bridge(rid), RosBridge) for rid in self.settings.robots))
        self.assertEqual(gateway.bridge("ros1_robot").robot.port, 9091)

    def test_adapter_selection_is_independent_of_ros_distribution(self):
        gateway = Gateway(self.settings)
        self.assertIsInstance(gateway.adapter("ros1_robot"), ROS1Adapter)
        self.assertIsInstance(gateway.adapter("create_3"), ROS2Adapter)
        self.assertIsInstance(gateway.adapter("ros2_humble_robot"), ROS2Adapter)

    def test_semantic_navigation_dispatches_three_distinct_action_formats(self):
        fakes = {rid: FakeBridge() for rid in self.settings.robots}
        gateway = Gateway(self.settings, bridges=fakes)

        async def run():
            for rid in fakes:
                result = await gateway.execute({"robot_id": rid, "operation": "navigate",
                                                "args": {"position": {"x": 0.2, "y": 0.1}}})
                self.assertTrue(result["ok"])

        asyncio.run(run())
        self.assertIn("goal_pose", fakes["create_3"].actions[0][2])
        self.assertIn("target_pose", fakes["ros1_robot"].actions[0][2])
        self.assertIn("pose", fakes["ros2_humble_robot"].actions[0][2])

    def test_several_robots_can_share_an_instance(self):
        self.data["robots"]["create_3_copy"] = self.data["robots"]["create_3"].copy()
        settings = Settings.model_validate(self.data)
        self.assertEqual(settings.resolved_robot("create_3").port,
                         settings.resolved_robot("create_3_copy").port)

    def test_multiple_instances_of_one_distribution_are_isolated(self):
        second = self.data["bridges"]["jazzy_local"].copy()
        second.update({"port": 9093, "domain_id": 20})
        self.data["bridges"]["jazzy_second"] = second
        self.data["robots"]["create_3_second"] = self.data["robots"]["create_3"].copy()
        self.data["robots"]["create_3_second"]["bridge_id"] = "jazzy_second"
        settings = Settings.model_validate(self.data)
        gateway = Gateway(settings)

        self.assertEqual(settings.bridges["jazzy_local"].distribution,
                         settings.bridges["jazzy_second"].distribution)
        self.assertEqual(gateway.bridge("create_3").robot.port, 9090)
        self.assertEqual(gateway.bridge("create_3_second").robot.port, 9093)
        self.assertIsInstance(gateway.adapter("create_3_second"), ROS2Adapter)

    def test_action_protocol_translations_are_in_adapters(self):
        sentinel1, sentinel2 = object(), object()
        with patch.object(ROS1Adapter, "send_action", return_value=sentinel1) as ros1:
            self.assertIs(RosBridge._ros1_action(None, "/move_base", "MoveBaseAction",
                                                   {}, 1.0, None), sentinel1)
            ros1.assert_called_once()
        with patch.object(ROS2Adapter, "send_action", return_value=sentinel2) as ros2:
            self.assertIs(RosBridge._ros2_action(None, "/dock", "Dock",
                                                   {}, 1.0, None), sentinel2)
            ros2.assert_called_once()

    def test_unknown_bridge_mismatched_family_and_host_conflicts_fail(self):
        bad = yaml.safe_load((ROOT / "configs/multi-distro.example.yaml").read_text())
        bad["robots"]["create_3"]["bridge_id"] = "does_not_exist"
        with self.assertRaisesRegex(ValidationError, "unknown bridge_id"):
            Settings.model_validate(bad)

        bad["robots"]["create_3"]["bridge_id"] = "noetic_local"
        with self.assertRaisesRegex(ValidationError, "ROS version differs"):
            Settings.model_validate(bad)

        bad["robots"]["create_3"]["bridge_id"] = "jazzy_local"
        bad["robots"]["create_3"]["host"] = "remote-host"
        with self.assertRaisesRegex(ValidationError, "choose either bridge_id or a direct host"):
            Settings.model_validate(bad)

    def test_bridge_deployments_cover_noetic_humble_and_jazzy(self):
        ros1 = (ROOT / "deploy/rosbridges/ros1-noetic/Dockerfile").read_text()
        ros2 = (ROOT / "deploy/rosbridges/ros2/Dockerfile").read_text()
        self.assertIn("FROM ros:noetic-ros-base", ros1)
        self.assertIn("ros-noetic-rosbridge-suite", ros1)
        self.assertIn("FROM ros:${ROS_DISTRO}-ros-base", ros2)
        self.assertIn("ros-${ROS_DISTRO}-rosbridge-suite", ros2)
        self.assertIn("ros2 launch rosbridge_server", (ROOT / "deploy/rosbridges/ros2/start.sh").read_text())


if __name__ == "__main__":
    unittest.main()
