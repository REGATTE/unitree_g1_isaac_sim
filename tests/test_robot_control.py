import sys
import time
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dds.common.lowcmd_types import LowCmdCache
from robot_control import RobotCommandApplier
from robot_state import JointStateSnapshot


class _FakeStateReader:
    def __init__(self):
        self.current_positions = [0.0] * 29
        self.last_positions = None
        self.last_velocities = None
        self.last_torques = None
        self.last_kp = None
        self.last_kd = None

    def read_snapshot(self):
        return JointStateSnapshot(
            joint_names=[f"joint_{index}" for index in range(29)],
            joint_positions=list(self.current_positions),
            joint_velocities=[0.0] * 29,
            joint_efforts=[0.0] * 29,
        )

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
        applier = RobotCommandApplier(state_reader, max_position_delta_rad=100.0)
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
        self.assertFalse(result.rejected_by_safety)

    def test_lowcmd_rejects_large_position_jump(self):
        state_reader = _FakeStateReader()
        applier = RobotCommandApplier(state_reader, max_position_delta_rad=0.1)
        lowcmd = LowCmdCache(
            mode_pr=0,
            mode_machine=0,
            joint_positions_dds=tuple(1.0 for _ in range(29)),
            joint_velocities_dds=tuple(0.0 for _ in range(29)),
            joint_torques_dds=tuple(0.0 for _ in range(29)),
            joint_kp_dds=tuple(0.0 for _ in range(29)),
            joint_kd_dds=tuple(0.0 for _ in range(29)),
            received_at_monotonic=time.monotonic(),
        )

        result = applier.apply_lowcmd(lowcmd)

        self.assertTrue(result.command_seen)
        self.assertTrue(result.rejected_by_safety)
        self.assertFalse(result.position_applied)
        self.assertIsNone(state_reader.last_positions)


if __name__ == "__main__":
    unittest.main()
