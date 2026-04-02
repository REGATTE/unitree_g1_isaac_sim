import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.g1_lowcmd import G1LowCmdSubscriber
from mapping.joints import BODY_JOINT_COUNT, G1_29DOF_JOINT_MAPPING


def _make_motor_command(index: int) -> SimpleNamespace:
    return SimpleNamespace(
        q=1000.0 + index,
        dq=2000.0 + index,
        tau=3000.0 + index,
        kp=4000.0 + index,
        kd=5000.0 + index,
    )


def _make_lowcmd_message(slot_count: int) -> SimpleNamespace:
    return SimpleNamespace(
        mode_pr=7,
        mode_machine=11,
        motor_cmd=[_make_motor_command(index) for index in range(slot_count)],
        crc=12345,
    )


class _AcceptAllCrc:
    def Crc(self, msg) -> int:
        return msg.crc


class G1LowCmdSubscriberTests(unittest.TestCase):
    def test_wider_messages_are_clamped_to_body_joint_count(self) -> None:
        subscriber = G1LowCmdSubscriber()
        subscriber._crc_helper = _AcceptAllCrc()
        message = _make_lowcmd_message(slot_count=35)

        subscriber._on_message(message)

        cached = subscriber.latest_command
        self.assertIsNotNone(cached)
        self.assertEqual(len(cached.joint_positions_dds), BODY_JOINT_COUNT)
        self.assertEqual(cached.joint_positions_dds[0], 1000.0)
        self.assertEqual(cached.joint_positions_dds[-1], 1000.0 + BODY_JOINT_COUNT - 1)

        sim_order = cached.to_sim_order()
        sim_first_joint = G1_29DOF_JOINT_MAPPING.sim_joint_names[0]
        dds_index = G1_29DOF_JOINT_MAPPING.dds_name_to_index[sim_first_joint]
        self.assertEqual(sim_order.positions[0], 1000.0 + dds_index)
        self.assertEqual(len(sim_order.positions), BODY_JOINT_COUNT)

    def test_short_messages_are_rejected(self) -> None:
        subscriber = G1LowCmdSubscriber()
        subscriber._crc_helper = _AcceptAllCrc()
        message = _make_lowcmd_message(slot_count=BODY_JOINT_COUNT - 1)

        subscriber._on_message(message)

        self.assertIsNone(subscriber.latest_command)


if __name__ == "__main__":
    unittest.main()
