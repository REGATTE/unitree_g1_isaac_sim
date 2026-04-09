import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import parse_config


class ConfigDefaultsTests(unittest.TestCase):
    def test_dds_and_lowcmd_are_enabled_by_default(self):
        config = parse_config([])

        self.assertTrue(config.enable_dds)
        self.assertTrue(config.enable_lowcmd_subscriber)
        self.assertEqual(config.dds_domain_id, 1)
        self.assertAlmostEqual(config.physics_dt, 1.0 / 500.0)
        self.assertEqual(config.lowstate_publish_hz, 500.0)
        self.assertEqual(config.lowcmd_max_position_delta_rad, 0.25)
        self.assertEqual(config.bridge_lowstate_port, 35501)
        self.assertEqual(config.bridge_lowcmd_port, 35502)

    def test_boolean_optional_flags_can_disable_default_dds_path(self):
        config = parse_config(["--no-enable-dds", "--no-enable-lowcmd-subscriber"])

        self.assertFalse(config.enable_dds)
        self.assertFalse(config.enable_lowcmd_subscriber)

    def test_unitree_ros2_install_prefix_can_be_provided_explicitly(self):
        config = parse_config(["--unitree-ros2-install-prefix", "/tmp"])

        self.assertEqual(str(config.unitree_ros2_install_prefix), "/tmp")

    def test_lowstate_publish_rate_cannot_exceed_physics_rate(self):
        with self.assertRaises(SystemExit):
            parse_config(["--physics-dt", "0.01", "--lowstate-publish-hz", "500"])


if __name__ == "__main__":
    unittest.main()
