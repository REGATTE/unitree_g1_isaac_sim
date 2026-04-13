import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from sensors.livox_mid360 import (  # noqa: E402
    MID360_CHANNEL_COUNT,
    MID360_FRAME_RATE_HZ,
    MID360_POINT_RATE_HZ,
    MID360_POINTS_PER_FRAME,
    MID360_RTX_ATTRIBUTES,
    MID360_RPY_IN_TORSO,
    MID360_TRANSLATION_IN_TORSO,
    MID360_VERTICAL_FOV_DEG,
    _iter_mid360_scan_angles,
    _is_incompatible_ros_python_path,
    _optional_isaac_string,
    _strip_ros_topic_prefix,
)
import numpy as np  # noqa: E402


class LivoxMid360ConfigTests(unittest.TestCase):
    def test_mid360_mount_matches_urdf_sensor_frame(self):
        self.assertEqual(MID360_TRANSLATION_IN_TORSO, (0.0002835, 0.00003, 0.40618))
        self.assertEqual(MID360_RPY_IN_TORSO, (0.0, 0.04014257279586953, 0.0))

    def test_mid360_rtx_pattern_has_consistent_emitter_width(self):
        self.assertEqual(MID360_RTX_ATTRIBUTES["omni:sensor:Core:numberOfEmitters"], MID360_CHANNEL_COUNT)
        self.assertEqual(MID360_RTX_ATTRIBUTES["omni:sensor:Core:numberOfChannels"], MID360_CHANNEL_COUNT)
        self.assertEqual(
            len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:azimuthDeg"]),
            MID360_CHANNEL_COUNT,
        )
        self.assertEqual(
            len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:elevationDeg"]),
            MID360_CHANNEL_COUNT,
        )
        self.assertEqual(
            len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:fireTimeNs"]),
            MID360_CHANNEL_COUNT,
        )
        self.assertEqual(
            len(MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:channelId"]),
            MID360_CHANNEL_COUNT,
        )

    def test_mid360_raycast_density_matches_expected_frame_density(self):
        self.assertEqual(MID360_POINT_RATE_HZ, 200_000)
        self.assertEqual(MID360_FRAME_RATE_HZ, 10.0)
        self.assertEqual(MID360_POINTS_PER_FRAME, 20_000)
        self.assertEqual(
            MID360_RTX_ATTRIBUTES["omni:sensor:Core:reportRateBaseHz"],
            MID360_POINT_RATE_HZ,
        )
        self.assertEqual(
            MID360_RTX_ATTRIBUTES["omni:sensor:Core:scanRateBaseHz"],
            MID360_FRAME_RATE_HZ,
        )

    def test_mid360_scan_iterator_covers_full_vertical_fov_and_frame_density(self):
        elevations = np.deg2rad(
            np.asarray(
                MID360_RTX_ATTRIBUTES["omni:sensor:Core:emitterState:s001:elevationDeg"],
                dtype=np.float64,
            )
        )
        angles = list(_iter_mid360_scan_angles(0, MID360_POINTS_PER_FRAME, elevations))
        self.assertEqual(len(angles), MID360_POINTS_PER_FRAME)
        self.assertAlmostEqual(min(elevation for elevation, _ in angles), np.deg2rad(MID360_VERTICAL_FOV_DEG[0]))
        self.assertAlmostEqual(max(elevation for elevation, _ in angles), np.deg2rad(MID360_VERTICAL_FOV_DEG[1]))
        self.assertGreater(len({round(azimuth, 6) for _, azimuth in angles}), 400)

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
