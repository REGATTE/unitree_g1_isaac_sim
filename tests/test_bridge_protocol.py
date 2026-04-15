import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.ros2.bridge_protocol import decode_lowstate_packet, encode_lowstate_packet
from mapping.joints import SIM_G1_29DOF_JOINT_NAMES
from robot_state import RobotKinematicSnapshot


class BridgeProtocolTests(unittest.TestCase):
    def test_lowstate_packet_preserves_wxyz_quaternion(self):
        snapshot = RobotKinematicSnapshot(
            joint_names=tuple(SIM_G1_29DOF_JOINT_NAMES),
            joint_positions=tuple(0.1 * index for index in range(len(SIM_G1_29DOF_JOINT_NAMES))),
            joint_velocities=tuple(-0.01 * index for index in range(len(SIM_G1_29DOF_JOINT_NAMES))),
            joint_efforts=tuple(1.0 + index for index in range(len(SIM_G1_29DOF_JOINT_NAMES))),
            base_position_world=(0.0, 0.0, 0.8),
            base_quaternion_wxyz=(1.0, 0.1, 0.2, 0.3),
            base_linear_velocity_world=(0.0, 0.0, 0.0),
            base_angular_velocity_world=(0.0, 0.0, 0.0),
            imu_quaternion_wxyz=(0.5, 0.5, 0.5, 0.5),
            imu_linear_acceleration_body=(0.0, 0.0, 9.81),
            imu_angular_velocity_body=(0.0, 0.0, 0.0),
        )

        packet = encode_lowstate_packet(snapshot, tick=42)
        payload = decode_lowstate_packet(packet)

        self.assertEqual(payload["tick"], 42)
        self.assertEqual(payload["imu_quaternion_wxyz"], [0.5, 0.5, 0.5, 0.5])


if __name__ == "__main__":
    unittest.main()
