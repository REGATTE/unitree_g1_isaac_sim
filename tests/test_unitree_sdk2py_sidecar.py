import importlib.util
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
SIDECAR_PATH = REPO_ROOT / "scripts" / "unitree_sdk2py_sidecar.py"
spec = importlib.util.spec_from_file_location("unitree_sdk2py_sidecar", SIDECAR_PATH)
unitree_sdk2py_sidecar = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(unitree_sdk2py_sidecar)


class _FakeImuState:
    def __init__(self):
        self.quaternion = [0.0, 0.0, 0.0, 0.0]
        self.gyroscope = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.temperature = 0


class _FakeMotorState:
    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.ddq = 0.0
        self.tau_est = 0.0
        self.temperature = [0, 0]
        self.vol = 0.0
        self.sensor = [0, 0]
        self.motorstate = 0
        self.reserve = [0, 0, 0, 0]


class _FakeLowState:
    def __init__(self):
        self.version = [0, 0]
        self.mode_pr = 0
        self.mode_machine = 0
        self.tick = 0
        self.imu_state = _FakeImuState()
        self.motor_state = [_FakeMotorState() for _ in range(35)]
        self.wireless_remote = [0 for _ in range(40)]
        self.reserve = [0, 0, 0, 0]
        self.crc = 0


class _FakeCrc:
    def Crc(self, message):
        self.message = message
        return 123456


class UnitreeSdk2PySidecarTests(unittest.TestCase):
    def test_build_lowstate_message_maps_body_joints_and_crc(self):
        crc = _FakeCrc()
        payload = {
            "tick": 42,
            "imu_quaternion_wxyz": [1.0, 0.1, 0.2, 0.3],
            "imu_gyroscope_body": [0.4, 0.5, 0.6],
            "imu_accelerometer_body": [0.7, 0.8, 0.9],
            "joint_positions_dds": [float(index) for index in range(29)],
            "joint_velocities_dds": [float(index) + 0.1 for index in range(29)],
            "joint_efforts_dds": [float(index) + 0.2 for index in range(29)],
        }

        message = unitree_sdk2py_sidecar.build_lowstate_message(payload, _FakeLowState, crc)

        self.assertEqual(message.version, [1, 0])
        self.assertEqual(message.tick, 42)
        self.assertEqual(message.imu_state.quaternion, [1.0, 0.1, 0.2, 0.3])
        self.assertEqual(message.imu_state.gyroscope, [0.4, 0.5, 0.6])
        self.assertEqual(message.imu_state.accelerometer, [0.7, 0.8, 0.9])
        self.assertEqual(message.motor_state[0].q, 0.0)
        self.assertEqual(message.motor_state[0].dq, 0.1)
        self.assertEqual(message.motor_state[0].tau_est, 0.2)
        self.assertEqual(message.motor_state[28].q, 28.0)
        self.assertEqual(message.motor_state[34].q, 0.0)
        self.assertEqual(message.crc, 123456)
        self.assertIs(crc.message, message)


if __name__ == "__main__":
    unittest.main()
