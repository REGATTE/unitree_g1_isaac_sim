import sys
import time
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.g1_lowcmd import LowCmdCache
from robot_control import RobotCommandApplier


class _FakeStateReader:
    def __init__(self):
        self.last_positions = None
        self.last_velocities = None
        self.last_torques = None
        self.last_kp = None
        self.last_kd = None

    def apply_joint_position_targets(self, values):
        self.last_positions = list(values)
        return True

    def apply_joint_velocity_targets(self, values):
        self.last_velocities = list(values)
        return True

    def apply_joint_efforts(self, values):
        self.last_torques = list(values)
        return True

    def apply_joint_gains(self, joint_kp, joint_kd):
        self.last_kp = list(joint_kp)
        self.last_kd = list(joint_kd)
        return True


class RobotCommandApplierTests(unittest.TestCase):
    def test_lowcmd_applies_dynamic_gains(self):
        state_reader = _FakeStateReader()
        applier = RobotCommandApplier(state_reader)
        lowcmd = LowCmdCache(
            mode_pr=0,
            mode_machine=0,
            joint_positions_dds=tuple(float(index) for index in range(29)),
            joint_velocities_dds=tuple(float(index + 100) for index in range(29)),
            joint_torques_dds=tuple(float(index + 200) for index in range(29)),
            joint_kp_dds=tuple(float(index + 300) for index in range(29)),
            joint_kd_dds=tuple(float(index + 400) for index in range(29)),
            received_at_monotonic=time.monotonic(),
        )

        result = applier.apply_lowcmd(lowcmd)

        self.assertTrue(result.command_seen)
        self.assertTrue(result.position_applied)
        self.assertTrue(result.velocity_applied)
        self.assertTrue(result.effort_applied)
        self.assertTrue(result.gains_applied)
        self.assertIsNotNone(state_reader.last_kp)
        self.assertIsNotNone(state_reader.last_kd)
        self.assertEqual(len(state_reader.last_kp), 29)
        self.assertEqual(len(state_reader.last_kd), 29)


if __name__ == "__main__":
    unittest.main()
