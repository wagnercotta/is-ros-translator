"""Offline regression checks for the Jazzy ROSAPI action graph compatibility patch."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


PATCH_SCRIPT = (Path(__file__).resolve().parents[1] /
                "deploy/rosbridges/ros2/patch_rosapi_action_type.py")
SPEC = importlib.util.spec_from_file_location("rosapi_action_patch", PATCH_SCRIPT)
assert SPEC and SPEC.loader
PATCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PATCH)


class RosapiPatchTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tempdir.cleanup)
        self.root = Path(self.tempdir.name) / "jazzy"
        self.proxy = self.root / "lib/python3.12/site-packages/rosapi/proxy.py"
        self.proxy.parent.mkdir(parents=True)

    def test_legacy_import_is_replaced_and_idempotent(self):
        self.proxy.write_text(
            "from ros2action.api import get_action_names_and_types\n"
            "def get_type():\n"
            "    return get_action_names_and_types(node=_node)\n"
        )
        self.assertIn("Patched", PATCH.patch_rosapi(self.root))
        expected = self.proxy.read_text()
        self.assertIn(PATCH.CORRECT_IMPORT, expected)
        self.assertNotIn(PATCH.LEGACY_IMPORT, expected)
        self.assertIn("get_action_names_and_types(node=_node)", expected)
        self.assertIn("Already compatible", PATCH.patch_rosapi(self.root))
        self.assertEqual(self.proxy.read_text(), expected)

    def test_unknown_layout_fails_without_modifying_file(self):
        original = "from some_other_api import get_action_names_and_types\n"
        self.proxy.write_text(original)
        with self.assertRaisesRegex(RuntimeError, "Unknown ROSAPI"):
            PATCH.patch_rosapi(self.root)
        self.assertEqual(self.proxy.read_text(), original)

    def test_missing_implementation_fails(self):
        with self.assertRaisesRegex(RuntimeError, "Expected one"):
            PATCH.patch_rosapi(self.root)


if __name__ == "__main__":
    unittest.main()
