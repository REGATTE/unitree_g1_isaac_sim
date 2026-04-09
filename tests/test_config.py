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

    def test_boolean_optional_flags_can_disable_default_dds_path(self):
        config = parse_config(["--no-enable-dds", "--no-enable-lowcmd-subscriber"])

        self.assertFalse(config.enable_dds)
        self.assertFalse(config.enable_lowcmd_subscriber)


if __name__ == "__main__":
    unittest.main()
