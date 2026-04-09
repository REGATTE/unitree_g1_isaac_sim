import csv
import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.lowstate_listener import (
    LowStateCapture,
    LowStateSample,
    print_lowstate_capture_summary,
    summarize_joint_history,
    write_joint_history_csv,
)


def _make_sample(received_at_monotonic: float, tick: int, joint_index: int, q: float, dq: float, tau: float) -> LowStateSample:
    positions = [0.0] * 29
    velocities = [0.0] * 29
    torques = [0.0] * 29
    positions[joint_index] = q
    velocities[joint_index] = dq
    torques[joint_index] = tau
    return LowStateSample(
        received_at_monotonic=received_at_monotonic,
        tick=tick,
        positions=positions,
        velocities=velocities,
        torques=torques,
        imu_quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
        imu_accelerometer=[0.0, 0.0, 9.81],
        imu_gyroscope=[0.0, 0.0, 0.0],
    )


class LowStateListenerHelperTests(unittest.TestCase):
    def test_summarize_joint_history_returns_none_for_empty_history(self):
        summary = summarize_joint_history([], "left_shoulder_pitch_joint")

        self.assertIsNone(summary)

    def test_summarize_joint_history_reports_basic_extrema(self):
        history = [
            _make_sample(10.0, 100, 15, 0.10, 0.20, -1.0),
            _make_sample(10.1, 101, 15, 0.25, -0.40, 2.5),
            _make_sample(10.2, 102, 15, -0.05, 0.10, -0.5),
        ]

        summary = summarize_joint_history(history, "left_shoulder_pitch_joint")

        self.assertIsNotNone(summary)
        self.assertEqual(summary.sample_count, 3)
        self.assertAlmostEqual(summary.duration_seconds, 0.2)
        self.assertAlmostEqual(summary.position_min, -0.05)
        self.assertAlmostEqual(summary.position_max, 0.25)
        self.assertAlmostEqual(summary.velocity_peak_abs, 0.40)
        self.assertAlmostEqual(summary.torque_peak_abs, 2.5)

    def test_summarize_joint_history_uses_only_valid_filtered_samples(self):
        history = [
            _make_sample(10.0, 100, 15, 0.10, 0.20, -1.0),
            LowStateSample(
                received_at_monotonic=10.1,
                tick=101,
                positions=[],
                velocities=[],
                torques=[],
                imu_quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
                imu_accelerometer=[0.0, 0.0, 9.81],
                imu_gyroscope=[0.0, 0.0, 0.0],
            ),
            _make_sample(10.2, 102, 15, -0.05, 0.10, -0.5),
        ]

        summary = summarize_joint_history(history, "left_shoulder_pitch_joint")

        self.assertIsNotNone(summary)
        self.assertEqual(summary.sample_count, 2)
        self.assertAlmostEqual(summary.duration_seconds, 0.2)
        self.assertAlmostEqual(summary.position_min, -0.05)
        self.assertAlmostEqual(summary.position_max, 0.10)

    def test_write_joint_history_csv_outputs_target_joint_rows(self):
        history = [
            _make_sample(20.0, 200, 15, 0.10, 0.20, -1.0),
            _make_sample(20.1, 201, 15, 0.25, -0.40, 2.5),
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "left_shoulder_pitch_joint.csv"
            exported_sample_count = write_joint_history_csv(history, "left_shoulder_pitch_joint", output_path)

            with output_path.open(newline="") as csv_file:
                rows = list(csv.reader(csv_file))

        self.assertEqual(exported_sample_count, 2)
        self.assertEqual(rows[0], ["time_s", "tick", "joint_name", "q", "dq", "tau"])
        self.assertEqual(rows[1][1], "200")
        self.assertEqual(rows[1][2], "left_shoulder_pitch_joint")
        self.assertEqual(rows[2][1], "201")
        self.assertEqual(rows[2][2], "left_shoulder_pitch_joint")

    def test_write_joint_history_csv_skips_samples_missing_target_joint(self):
        history = [
            _make_sample(20.0, 200, 15, 0.10, 0.20, -1.0),
            LowStateSample(
                received_at_monotonic=20.1,
                tick=201,
                positions=[],
                velocities=[],
                torques=[],
                imu_quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
                imu_accelerometer=[0.0, 0.0, 9.81],
                imu_gyroscope=[0.0, 0.0, 0.0],
            ),
            _make_sample(20.2, 202, 15, 0.25, -0.40, 2.5),
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "left_shoulder_pitch_joint.csv"
            exported_sample_count = write_joint_history_csv(history, "left_shoulder_pitch_joint", output_path)

            with output_path.open(newline="") as csv_file:
                rows = list(csv.reader(csv_file))

        self.assertEqual(exported_sample_count, 2)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[1][1], "200")
        self.assertEqual(rows[2][1], "202")

    def test_print_lowstate_capture_summary_uses_provided_capture_window(self):
        capture = LowStateCapture(
            latest=_make_sample(30.1, 301, 15, 0.25, -0.40, 2.5),
            history=[
                _make_sample(30.0, 300, 15, 0.10, 0.20, -1.0),
                _make_sample(30.1, 301, 15, 0.25, -0.40, 2.5),
            ],
            messages_seen=2,
            messages_rejected=1,
        )

        output = io.StringIO()
        with redirect_stdout(output):
            print_lowstate_capture_summary(
                capture,
                preview_joints=0,
                target_joint_name="left_shoulder_pitch_joint",
            )

        rendered = output.getvalue()
        self.assertIn("tick=301 valid_messages=2 transport_rejected=1", rendered)
        self.assertIn("target_history left_shoulder_pitch_joint: samples=2 duration_s=0.100", rendered)
        self.assertIn("q_max=0.25000", rendered)

    def test_print_lowstate_capture_summary_handles_missing_latest_sample(self):
        capture = LowStateCapture(
            latest=None,
            history=[],
            messages_seen=0,
            messages_rejected=0,
        )

        output = io.StringIO()
        with redirect_stdout(output):
            print_lowstate_capture_summary(
                capture,
                preview_joints=0,
                target_joint_name="left_shoulder_pitch_joint",
            )

        self.assertIn("No valid lowstate sample received yet.", output.getvalue())


if __name__ == "__main__":
    unittest.main()
