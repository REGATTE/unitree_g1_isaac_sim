import sys
import time
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.bridge_protocol import encode_lowcmd_packet
from dds.g1_lowcmd import G1LowCmdSubscriber
from mapping.joints import BODY_JOINT_COUNT, G1_29DOF_JOINT_MAPPING


class G1LowCmdSubscriberTests(unittest.TestCase):
    def test_packet_is_clamped_to_body_joint_count(self) -> None:
        subscriber = G1LowCmdSubscriber(bind_host="127.0.0.1", bind_port=5502)
        packet = encode_lowcmd_packet(
            mode_pr=7,
            mode_machine=11,
            positions=[1000.0 + index for index in range(35)],
            velocities=[2000.0 + index for index in range(35)],
            torques=[3000.0 + index for index in range(35)],
            kp=[4000.0 + index for index in range(35)],
            kd=[5000.0 + index for index in range(35)],
        )

        subscriber._on_packet(packet, received_at_monotonic=time.monotonic())

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

    def test_short_packets_are_rejected(self) -> None:
        subscriber = G1LowCmdSubscriber(bind_host="127.0.0.1", bind_port=5502)
        packet = encode_lowcmd_packet(
            mode_pr=7,
            mode_machine=11,
            positions=[float(index) for index in range(BODY_JOINT_COUNT - 1)],
            velocities=[float(index) for index in range(BODY_JOINT_COUNT - 1)],
            torques=[float(index) for index in range(BODY_JOINT_COUNT - 1)],
            kp=[float(index) for index in range(BODY_JOINT_COUNT - 1)],
            kd=[float(index) for index in range(BODY_JOINT_COUNT - 1)],
        )

        subscriber._on_packet(packet, received_at_monotonic=time.monotonic())

        self.assertIsNone(subscriber.latest_command)


if __name__ == "__main__":
    unittest.main()
