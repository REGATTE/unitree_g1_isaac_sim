import json
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.native.lowcmd import NativeLowCmdUdpSubscriber
from mapping.joints import BODY_JOINT_COUNT


def _packet(count=BODY_JOINT_COUNT):
    return json.dumps(
        {
            "mode_pr": 1,
            "mode_machine": 2,
            "joint_positions_dds": [float(index) for index in range(count)],
            "joint_velocities_dds": [float(index) + 0.1 for index in range(count)],
            "joint_torques_dds": [float(index) + 0.2 for index in range(count)],
            "joint_kp_dds": [float(index) + 0.3 for index in range(count)],
            "joint_kd_dds": [float(index) + 0.4 for index in range(count)],
        }
    ).encode("utf-8")


class NativeLowCmdUdpSubscriberTests(unittest.TestCase):
    def test_on_packet_caches_body_joint_command(self):
        subscriber = NativeLowCmdUdpSubscriber("127.0.0.1", 0)

        subscriber._on_packet(_packet(), received_at_monotonic=123.0)

        cached = subscriber.latest_command
        self.assertIsNotNone(cached)
        self.assertEqual(cached.mode_pr, 1)
        self.assertEqual(cached.mode_machine, 2)
        self.assertEqual(len(cached.joint_positions_dds), BODY_JOINT_COUNT)
        self.assertEqual(cached.joint_positions_dds[3], 3.0)
        self.assertEqual(cached.joint_kd_dds[3], 3.4)
        self.assertEqual(cached.received_at_monotonic, 123.0)

    def test_on_packet_truncates_extra_unitree_slots(self):
        subscriber = NativeLowCmdUdpSubscriber("127.0.0.1", 0)

        subscriber._on_packet(_packet(count=35), received_at_monotonic=123.0)

        cached = subscriber.latest_command
        self.assertIsNotNone(cached)
        self.assertEqual(len(cached.joint_positions_dds), BODY_JOINT_COUNT)
        self.assertEqual(cached.joint_positions_dds[-1], float(BODY_JOINT_COUNT - 1))

    def test_on_packet_drops_short_command(self):
        subscriber = NativeLowCmdUdpSubscriber("127.0.0.1", 0)

        subscriber._on_packet(_packet(count=BODY_JOINT_COUNT - 1), received_at_monotonic=123.0)

        self.assertIsNone(subscriber.latest_command)

    def test_on_packet_drops_incomplete_fields(self):
        subscriber = NativeLowCmdUdpSubscriber("127.0.0.1", 0)
        payload = json.loads(_packet().decode("utf-8"))
        payload["joint_kd_dds"] = payload["joint_kd_dds"][:-1]

        subscriber._on_packet(json.dumps(payload).encode("utf-8"), received_at_monotonic=123.0)

        self.assertIsNone(subscriber.latest_command)


if __name__ == "__main__":
    unittest.main()
