import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from robot_state import PhysicsViewUnavailableError, RobotStateReader


class _FakeArticulation:
    def __init__(self, dof_names, positions, velocities, efforts=None):
        self.dof_names = list(dof_names)
        self._positions = positions
        self._velocities = velocities
        self._efforts = efforts
        self.last_kps = None
        self.last_kds = None

    def get_joint_positions(self):
        return self._positions

    def get_joint_velocities(self):
        return self._velocities

    def get_measured_joint_efforts(self):
        return self._efforts

    def set_gains(self, kps=None, kds=None):
        self.last_kps = kps
        self.last_kds = kds


class RobotStatePauseSafeTests(unittest.TestCase):
    def _make_reader(self, articulation):
        reader = object.__new__(RobotStateReader)
        reader._articulation = articulation
        reader._initialized = True
        reader._warned_physics_view_unavailable = False
        return reader

    def test_empty_joint_state_raises_recoverable_error(self):
        reader = self._make_reader(
            _FakeArticulation(
                dof_names=["joint_a", "joint_b"],
                positions=[],
                velocities=[],
                efforts=[],
            )
        )

        with self.assertRaises(PhysicsViewUnavailableError):
            reader._read_joint_state()

    def test_joint_state_recovery_clears_warning_latch(self):
        reader = self._make_reader(
            _FakeArticulation(
                dof_names=["joint_a", "joint_b"],
                positions=[0.1, 0.2],
                velocities=[0.0, 0.0],
                efforts=[0.0, 0.0],
            )
        )
        reader._warned_physics_view_unavailable = True

        positions, velocities, efforts = reader._read_joint_state()

        self.assertEqual(positions, [0.1, 0.2])
        self.assertEqual(velocities, [0.0, 0.0])
        self.assertEqual(efforts, [0.0, 0.0])
        self.assertFalse(reader._warned_physics_view_unavailable)

    def test_joint_gains_apply_through_articulation_set_gains(self):
        articulation = _FakeArticulation(
            dof_names=["joint_a", "joint_b"],
            positions=[0.1, 0.2],
            velocities=[0.0, 0.0],
            efforts=[0.0, 0.0],
        )
        reader = self._make_reader(articulation)

        applied = reader.apply_joint_gains([10.0, 20.0], [1.0, 2.0])

        self.assertTrue(applied)
        self.assertEqual(articulation.last_kps.shape, (1, 2))
        self.assertEqual(articulation.last_kds.shape, (1, 2))
        self.assertEqual(articulation.last_kps.tolist(), [[10.0, 20.0]])
        self.assertEqual(articulation.last_kds.tolist(), [[1.0, 2.0]])


if __name__ == "__main__":
    unittest.main()
