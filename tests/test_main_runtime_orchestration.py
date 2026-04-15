import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import parse_config
import main as sim_main


class _FakeLowCmdManager:
    def __init__(self, command):
        self.latest_lowcmd = command


class MainRuntimeOrchestrationTests(unittest.TestCase):
    def test_default_runtime_selects_ros2_observation_and_sdk2py_control(self):
        config = parse_config([])

        mode = sim_main.resolve_dds_runtime_mode(config)

        self.assertTrue(mode.ros2_dds_enabled)
        self.assertFalse(mode.native_unitree_enabled)
        self.assertTrue(mode.sdk2py_enabled)
        self.assertEqual(mode.lowcmd_authority, "sdk2py")

    def test_default_runtime_instantiates_sdk2py_not_native(self):
        config = parse_config([])
        mode = sim_main.resolve_dds_runtime_mode(config)

        with (
            patch("main.DdsManager", return_value="ros2") as ros2_manager,
            patch("main.NativeUnitreeDdsManager", return_value="native") as native_manager,
            patch("main.UnitreeSdk2PyDdsManager", return_value="sdk2py") as sdk2py_manager,
        ):
            managers = sim_main.create_dds_runtime_managers(config, mode)

        self.assertEqual(managers.ros2, "ros2")
        self.assertIsNone(managers.native_unitree)
        self.assertEqual(managers.sdk2py, "sdk2py")
        ros2_manager.assert_called_once_with(config)
        native_manager.assert_not_called()
        sdk2py_manager.assert_called_once_with(config)

    def test_native_runtime_instantiates_native_when_sdk2py_is_disabled(self):
        config = parse_config(
            [
                "--no-enable-unitree-sdk2py-lowstate",
                "--no-enable-unitree-sdk2py-lowcmd",
                "--enable-native-unitree-lowstate",
                "--enable-native-unitree-lowcmd",
            ]
        )
        mode = sim_main.resolve_dds_runtime_mode(config)

        with (
            patch("main.DdsManager", return_value="ros2") as ros2_manager,
            patch("main.NativeUnitreeDdsManager", return_value="native") as native_manager,
            patch("main.UnitreeSdk2PyDdsManager", return_value="sdk2py") as sdk2py_manager,
        ):
            managers = sim_main.create_dds_runtime_managers(config, mode)

        self.assertTrue(mode.ros2_dds_enabled)
        self.assertTrue(mode.native_unitree_enabled)
        self.assertFalse(mode.sdk2py_enabled)
        self.assertEqual(mode.lowcmd_authority, "native")
        self.assertEqual(managers.ros2, "ros2")
        self.assertEqual(managers.native_unitree, "native")
        self.assertIsNone(managers.sdk2py)
        ros2_manager.assert_called_once_with(config)
        native_manager.assert_called_once_with(config)
        sdk2py_manager.assert_not_called()

    def test_runtime_mode_log_identifies_selected_authority(self):
        config = parse_config([])
        mode = sim_main.resolve_dds_runtime_mode(config)

        with self.assertLogs("unitree_g1_isaac_sim.main", level="INFO") as captured:
            sim_main.log_dds_runtime_mode(config, mode)

        rendered = "\n".join(captured.output)
        self.assertIn("unitree_runtime=sdk2py", rendered)
        self.assertIn("lowcmd_authority=sdk2py", rendered)
        self.assertIn("ros2_lowcmd=disabled", rendered)

    def test_resolve_active_lowcmd_checks_sdk2py_native_then_ros2(self):
        ros2_command = object()
        native_command = object()
        sdk2py_command = object()

        self.assertIs(
            sim_main.resolve_active_lowcmd(
                _FakeLowCmdManager(ros2_command),
                _FakeLowCmdManager(native_command),
                _FakeLowCmdManager(sdk2py_command),
            ),
            sdk2py_command,
        )
        self.assertIs(
            sim_main.resolve_active_lowcmd(
                _FakeLowCmdManager(ros2_command),
                _FakeLowCmdManager(native_command),
                None,
            ),
            native_command,
        )
        self.assertIs(
            sim_main.resolve_active_lowcmd(
                _FakeLowCmdManager(ros2_command),
                None,
                None,
            ),
            ros2_command,
        )


if __name__ == "__main__":
    unittest.main()
