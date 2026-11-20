"""Keep the public quick-start aligned with the physical Create 3 configuration."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

from is_ros_translator_v2.config import load_settings


ROOT = Path(__file__).resolve().parents[1]


class PublicLabTests(unittest.TestCase):
    def test_create3_lab_uses_real_republished_endpoints(self):
        settings = load_settings(ROOT / "configs/create3-lab.yaml")
        robot = settings.robots["create_3"]
        self.assertEqual(robot.navigation.action, "/create3/navigate_to_position")
        self.assertEqual(robot.navigation.frame_id, "odom")
        self.assertEqual(robot.odom_topic, "/create3/odom")
        self.assertEqual(robot.bridge_id, "jazzy_local")
        self.assertEqual(settings.bridges["jazzy_local"].rmw_implementation,
                         "rmw_fastrtps_cpp")
        for action in ("dock", "undock"):
            self.assertEqual(robot.capabilities[action].endpoint, f"/create3/{action}")

    def test_fastdds_example_has_one_peer_address(self):
        tree = ET.parse(ROOT / "configs/fastdds-unicast.example.xml")
        addresses = [node.text for node in tree.iter() if node.tag.endswith("address")]
        self.assertEqual(addresses, ["PI_WIFI_IP"])

    def test_compose_declares_three_separate_services(self):
        compose = yaml.safe_load((ROOT / "deploy/lab/compose.yaml").read_text())
        self.assertEqual(set(compose["services"]), {"rabbitmq", "bridge-jazzy", "core"})
        self.assertEqual(compose["services"]["bridge-jazzy"]["network_mode"], "host")
        self.assertEqual(compose["services"]["core"]["network_mode"], "host")

    def test_example_wire_command_targets_absolute_odom_position(self):
        spec = importlib.util.spec_from_file_location(
            "iswire_client", ROOT / "examples/iswire_rpc_client.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        command = module.build_command("navigate", "create_3", 0, -5, 0, 180)
        self.assertEqual(command["operation"], "navigate")
        self.assertEqual(command["args"]["position"]["y"], -5)
        self.assertEqual(command["args"]["frame_id"], "odom")
        self.assertEqual(command["timeout_seconds"], 180)


if __name__ == "__main__":
    unittest.main()
