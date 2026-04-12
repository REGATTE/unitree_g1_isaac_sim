import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from sensors.livox_mid360 import (  # noqa: E402
    MID360_RTX_ATTRIBUTES,
    MID360_RPY_IN_TORSO,
    MID360_TRANSLATION_IN_TORSO,
    _is_incompatible_ros_python_path,
    _optional_isaac_string,
    _strip_ros_topic_prefix,
)


class LivoxMid360ConfigTests(unittest.TestCase):
    def test_mid360_mount_matches_urdf_sensor_frame(self):
        self.assertEqual(MID360_TRANSLATION_IN_TORSO, (0.0002835, 0.00003, 0.40618))
        self.assertEqual(MID360_RPY_IN_TORSO, (0.0, 0.04014257279586953, 0.0))

    def test_mid360_rtx_pattern_has_consistent_emitter_width(self):
        self.assertEqual(MID360_RTX_ATTRIBUTES["omni:sensor:Core:numberOfEmitters"], 40)
        self.assertEqual(MID360_RTX_ATTRIBUTES["omni:sensor:Core:numberOfChannels"], 40)
        self.assertEqual(len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:azimuthDeg"]), 40)
        self.assertEqual(len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:elevationDeg"]), 40)
        self.assertEqual(len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:fireTimeNs"]), 40)
        self.assertEqual(len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:channelId"]), 40)

    def test_optional_isaac_string_accepts_none_spellings(self):
        self.assertIsNone(_optional_isaac_string("none"))
        self.assertIsNone(_optional_isaac_string(" null "))
        self.assertEqual(_optional_isaac_string("OS1"), "OS1")

    def test_ros_topic_prefix_is_normalized_for_writer(self):
        self.assertEqual(_strip_ros_topic_prefix("/livox/lidar"), "livox/lidar")
        self.assertEqual(_strip_ros_topic_prefix("livox/lidar"), "livox/lidar")

    def test_ros_python_310_paths_are_filtered_for_isaac_python(self):
        self.assertTrue(
            _is_incompatible_ros_python_path(
                "/opt/ros/humble/lib/python3.10/site-packages"
            )
        )
        self.assertTrue(
            _is_incompatible_ros_python_path(
                "/home/me/Workspaces/unitree_ros2/cyclonedds_ws/install/unitree_hg/local/lib/python3.10/dist-packages"
            )
        )
        self.assertFalse(
            _is_incompatible_ros_python_path(
                "/home/cleohumanoid/Omniverse/isaac_sim/exts/isaacsim.ros2.bridge/humble/rclpy"
            )
        )


if __name__ == "__main__":
    unittest.main()
