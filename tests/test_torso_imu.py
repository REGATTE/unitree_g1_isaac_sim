import math
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from sensors import torso_imu


class TorsoImuMathTests(unittest.TestCase):
    def test_gravity_only_acceleration_matches_identity_orientation(self):
        acceleration = torso_imu._gravity_only_acceleration_body([1.0, 0.0, 0.0, 0.0])

        self.assertEqual(acceleration, [0.0, 0.0, 9.81])

    def test_estimate_body_angular_velocity_tracks_yaw_rotation(self):
        sample_dt = 0.1
        half_angle = 0.05
        angular_velocity = torso_imu._estimate_body_angular_velocity(
            [1.0, 0.0, 0.0, 0.0],
            [math.cos(half_angle), 0.0, 0.0, math.sin(half_angle)],
            sample_dt,
        )

        self.assertAlmostEqual(angular_velocity[0], 0.0, places=6)
        self.assertAlmostEqual(angular_velocity[1], 0.0, places=6)
        self.assertAlmostEqual(angular_velocity[2], 1.0, places=6)

    def test_reset_clears_cached_state(self):
        sensor = torso_imu.TorsoImuSensor("/World/G1")
        sensor._last_position_world = [1.0, 2.0, 3.0]
        sensor._last_linear_velocity_world = [0.1, 0.2, 0.3]
        sensor._last_quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]

        sensor.reset()

        self.assertIsNone(sensor._last_position_world)
        self.assertIsNone(sensor._last_linear_velocity_world)
        self.assertIsNone(sensor._last_quaternion_wxyz)


if __name__ == "__main__":
    unittest.main()
