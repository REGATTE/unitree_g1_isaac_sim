import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import DEFAULT_WORLD_PATH, parse_config, resolve_unitree_ros2_install_prefix


class ConfigDefaultsTests(unittest.TestCase):
    def test_phase0_defaults_enable_both_lowstate_paths_and_only_native_lowcmd(self):
        config = parse_config([])

        self.assertTrue(config.enable_dds)
        self.assertTrue(config.enable_ros2_lowstate)
        self.assertFalse(config.enable_ros2_lowcmd)
        self.assertTrue(config.enable_native_unitree_lowstate)
        self.assertTrue(config.enable_native_unitree_lowcmd)
        self.assertEqual(config.native_unitree_domain_id, config.dds_domain_id)
        self.assertEqual(config.dds_domain_id, 1)
        self.assertAlmostEqual(config.physics_dt, 1.0 / 500.0)
        self.assertFalse(config.use_world)
        self.assertEqual(config.world_path, DEFAULT_WORLD_PATH)
        self.assertEqual(config.world_prim_path, "/World/Environment")
        self.assertTrue(config.enable_follow_camera)
        self.assertEqual(config.follow_camera_prim_path, "/World/FollowCamera")
        self.assertEqual(config.follow_camera_distance, 4.0)
        self.assertEqual(config.follow_camera_height, 0.6)
        self.assertEqual(config.follow_camera_target_height, 0.3)
        self.assertEqual(config.lowstate_publish_hz, 500.0)
        self.assertEqual(config.lowcmd_max_position_delta_rad, 0.25)
        self.assertEqual(config.bridge_lowstate_port, 35501)
        self.assertEqual(config.bridge_lowcmd_port, 35502)
        self.assertTrue(config.enable_livox_lidar)
        self.assertEqual(config.livox_lidar_topic, "livox/lidar")
        self.assertEqual(config.livox_lidar_frame_id, "mid360_link")
        self.assertEqual(config.livox_lidar_parent_link_name, "torso_link")

    def test_boolean_optional_flags_can_disable_default_dds_paths(self):
        config = parse_config(
            [
                "--no-enable-dds",
                "--no-enable-ros2-lowstate",
                "--no-enable-native-unitree-lowstate",
                "--no-enable-native-unitree-lowcmd",
            ]
        )

        self.assertFalse(config.enable_dds)
        self.assertFalse(config.enable_ros2_lowstate)
        self.assertFalse(config.enable_native_unitree_lowstate)
        self.assertFalse(config.enable_native_unitree_lowcmd)

    def test_ros2_lowcmd_can_be_enabled_when_native_lowcmd_is_disabled(self):
        config = parse_config(
            [
                "--enable-ros2-lowcmd",
                "--no-enable-native-unitree-lowcmd",
            ]
        )

        self.assertTrue(config.enable_ros2_lowcmd)
        self.assertFalse(config.enable_native_unitree_lowcmd)

    def test_both_lowcmd_sources_enabled_fails_preflight(self):
        with self.assertRaises(SystemExit):
            parse_config(["--enable-ros2-lowcmd"])

    def test_livox_lidar_can_be_disabled_and_overridden(self):
        config = parse_config(
            [
                "--no-enable-livox-lidar",
                "--livox-lidar-topic",
                "points",
                "--livox-lidar-frame-id",
                "custom_mid360",
                "--livox-lidar-rtx-config",
                "none",
                "--livox-lidar-rtx-variant",
                "none",
            ]
        )

        self.assertFalse(config.enable_livox_lidar)
        self.assertEqual(config.livox_lidar_topic, "points")
        self.assertEqual(config.livox_lidar_frame_id, "custom_mid360")
        self.assertEqual(config.livox_lidar_rtx_config, "none")
        self.assertEqual(config.livox_lidar_rtx_variant, "none")

    def test_world_can_be_enabled_with_explicit_true_false_value(self):
        enabled = parse_config(["--use-world", "true"])
        disabled = parse_config(["--use-world", "false"])

        self.assertTrue(enabled.use_world)
        self.assertFalse(disabled.use_world)

    def test_world_can_be_enabled_without_value(self):
        config = parse_config(["--use-world"])

        self.assertTrue(config.use_world)

    def test_world_path_can_be_overridden(self):
        config = parse_config(["--use-world", "true", "--world-path", "/tmp/world.usd"])

        self.assertTrue(config.use_world)
        self.assertEqual(config.world_path, Path("/tmp/world.usd"))

    def test_follow_camera_can_be_disabled(self):
        config = parse_config(["--no-enable-follow-camera"])

        self.assertFalse(config.enable_follow_camera)

    def test_follow_camera_offsets_can_be_overridden(self):
        config = parse_config(
            [
                "--follow-camera-prim-path",
                "/World/CameraRig/Follow",
                "--follow-camera-distance",
                "4.5",
                "--follow-camera-height",
                "2.0",
                "--follow-camera-target-height",
                "1.1",
            ]
        )

        self.assertEqual(config.follow_camera_prim_path, "/World/CameraRig/Follow")
        self.assertEqual(config.follow_camera_distance, 4.5)
        self.assertEqual(config.follow_camera_height, 2.0)
        self.assertEqual(config.follow_camera_target_height, 1.1)

    def test_unitree_ros2_install_prefix_can_be_provided_explicitly(self):
        config = parse_config(["--unitree-ros2-install-prefix", "/tmp"])

        self.assertEqual(str(config.unitree_ros2_install_prefix), "/tmp")

    def test_shallow_checkout_path_does_not_crash_prefix_resolution(self):
        with patch("config.PROJECT_ROOT", Path("/workspace/unitree_g1_isaac_sim")):
            resolved = resolve_unitree_ros2_install_prefix(None)

        self.assertTrue(resolved is None or isinstance(resolved, Path))

    def test_lowstate_publish_rate_cannot_exceed_physics_rate(self):
        with self.assertRaises(SystemExit):
            parse_config(["--physics-dt", "0.01", "--lowstate-publish-hz", "500"])


if __name__ == "__main__":
    unittest.main()
