"""Contracts for the ROS 2 + ROSBridge + gateway single-container deployment."""

from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class Create3DockerTests(unittest.TestCase):
    def test_image_contains_ros2_rosbridge_and_gateway(self):
        dockerfile = (ROOT / "deploy/create3/Dockerfile").read_text()
        self.assertIn("FROM ros:jazzy-ros-base", dockerfile)
        self.assertIn("ros-jazzy-rosbridge-suite", dockerfile)
        self.assertIn("ros-jazzy-irobot-create-msgs", dockerfile)
        self.assertIn("ros-jazzy-rmw-cyclonedds-cpp", dockerfile)
        self.assertIn("python3 -m venv /opt/rostranslator-venv", dockerfile)
        self.assertIn('ENV PATH="/opt/rostranslator-venv/bin:${PATH}"', dockerfile)
        self.assertIn("python3 -m pip install --no-cache-dir .", dockerfile)
        self.assertIn('ENTRYPOINT ["/usr/local/bin/start-create3-gateway"]', dockerfile)

    def test_entrypoint_starts_rosbridge_and_gateway(self):
        script = (ROOT / "deploy/create3/start.sh").read_text()
        self.assertIn("source /opt/ros/jazzy/setup.bash", script)
        self.assertIn("ros2 launch rosbridge_server rosbridge_websocket_launch.xml", script)
        self.assertIn("address:=127.0.0.1 port:=9090", script)
        self.assertIn("is-ros-translator --config /app/configs/create3.yaml", script)
        self.assertIn('wait -n "$rosbridge_pid" "$gateway_pid"', script)

    def test_gateway_uses_local_bridge_and_native_create3_interfaces(self):
        config = yaml.safe_load((ROOT / "configs/create3.yaml").read_text())
        robot = config["robots"]["create_3"]
        self.assertEqual(robot["host"], "127.0.0.1")
        self.assertEqual(robot["port"], 9090)
        self.assertEqual(robot["navigation"]["action"], "/navigate_to_position")
        self.assertIn("/undock", robot["allowed"]["actions"])
        self.assertIn("/dock", robot["allowed"]["actions"])
        self.assertTrue(config["iswire"]["enabled"])

    def test_multi_ros_example_has_independent_ros1_ros2_routes(self):
        from is_ros_translator_v2.config import Settings

        documentation = (ROOT / "docs/MULTI_ROS.md").read_text()
        example = yaml.safe_load(documentation.split("```yaml\n", 1)[1].split("\n```", 1)[0])
        settings = Settings.model_validate(example)
        self.assertEqual(settings.robots["create_3"].ros_version, 2)
        self.assertEqual(settings.robots["jackal_1"].ros_version, 1)
        self.assertNotEqual(settings.robots["create_3"].host, settings.robots["jackal_1"].host)


if __name__ == "__main__":
    unittest.main()
