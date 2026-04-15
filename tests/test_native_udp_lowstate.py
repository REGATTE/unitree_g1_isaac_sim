import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.native.bridge_protocol import decode_native_lowstate_packet, encode_native_lowstate_packet
from mapping.joints import DDS_G1_29DOF_JOINT_NAMES, SIM_G1_29DOF_JOINT_NAMES
from robot_state import RobotKinematicSnapshot


class NativeLowStateProtocolTests(unittest.TestCase):
    def test_native_lowstate_packet_uses_dds_joint_order(self):
        snapshot = RobotKinematicSnapshot(
            joint_names=tuple(SIM_G1_29DOF_JOINT_NAMES),
            joint_positions=tuple(100.0 + index for index in range(len(SIM_G1_29DOF_JOINT_NAMES))),
            joint_velocities=tuple(200.0 + index for index in range(len(SIM_G1_29DOF_JOINT_NAMES))),
            joint_efforts=tuple(300.0 + index for index in range(len(SIM_G1_29DOF_JOINT_NAMES))),
            base_position_world=(0.0, 0.0, 0.8),
            base_quaternion_wxyz=(1.0, 0.1, 0.2, 0.3),
            base_linear_velocity_world=(0.0, 0.0, 0.0),
            base_angular_velocity_world=(0.0, 0.0, 0.0),
            imu_quaternion_wxyz=(1.0, 0.1, 0.2, 0.3),
            imu_linear_acceleration_body=(0.0, 0.0, 9.81),
            imu_angular_velocity_body=(0.1, 0.2, 0.3),
            secondary_imu_quaternion_wxyz=(0.5, 0.5, 0.5, 0.5),
        )

        packet = encode_native_lowstate_packet(snapshot, tick=7)
        payload = decode_native_lowstate_packet(packet)

        self.assertEqual(payload["tick"], 7)
        self.assertEqual(payload["imu_quaternion_wxyz"], [1.0, 0.1, 0.2, 0.3])
        self.assertEqual(payload["imu_accelerometer_body"], [0.0, 0.0, 9.81])
        self.assertEqual(payload["imu_gyroscope_body"], [0.1, 0.2, 0.3])

        expected_first_dds_joint = DDS_G1_29DOF_JOINT_NAMES[0]
        expected_first_sim_index = SIM_G1_29DOF_JOINT_NAMES.index(expected_first_dds_joint)
        self.assertEqual(payload["joint_positions_dds"][0], 100.0 + expected_first_sim_index)
        self.assertEqual(payload["joint_velocities_dds"][0], 200.0 + expected_first_sim_index)
        self.assertEqual(payload["joint_efforts_dds"][0], 300.0 + expected_first_sim_index)


if __name__ == "__main__":
    unittest.main()
