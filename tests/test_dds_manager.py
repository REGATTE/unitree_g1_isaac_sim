import io
import sys
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import AppConfig
from dds.g1_lowcmd import LowCmdCache
from dds.manager import DdsManager


class _FakeLowCmdSubscriber:
    def __init__(self, latest_command=None):
        self.latest_command = latest_command


class _FakeLowStatePublisher:
    def __init__(self):
        self.publish_calls = 0

    def publish(self, snapshot):
        self.publish_calls += 1
        return True


def _build_config(**overrides) -> AppConfig:
    config = AppConfig(
        robot_variant="29dof",
        asset_path=None,
        robot_prim_path="/World/G1",
        robot_height=0.8,
        physics_dt=1.0 / 120.0,
        headless=False,
        renderer="RayTracedLighting",
        width=1280,
        height=720,
        max_frames=0,
        print_all_joints=False,
        enable_dds=True,
        dds_domain_id=1,
        lowstate_topic="rt/lowstate",
        lowcmd_topic="rt/lowcmd",
        lowstate_publish_hz=100.0,
        enable_lowcmd_subscriber=True,
        lowcmd_timeout_seconds=0.5,
        lowstate_cadence_report_interval=3,
        lowstate_cadence_warn_ratio=0.05,
    )
    if not overrides:
        return config
    return AppConfig(**{**config.__dict__, **overrides})


class DdsManagerTests(unittest.TestCase):
    def test_stale_lowcmd_is_not_returned(self):
        manager = DdsManager(_build_config())
        manager._lowcmd_subscriber = _FakeLowCmdSubscriber(
            latest_command=LowCmdCache(
                mode_pr=0,
                mode_machine=0,
                joint_positions_dds=tuple([0.0] * 29),
                joint_velocities_dds=tuple([0.0] * 29),
                joint_torques_dds=tuple([0.0] * 29),
                joint_kp_dds=tuple([0.0] * 29),
                joint_kd_dds=tuple([0.0] * 29),
                received_at_monotonic=time.monotonic() - 1.0,
            )
        )

        resolved = manager._resolve_latest_lowcmd(now_monotonic=time.monotonic())

        self.assertIsNone(resolved)

    def test_fresh_lowcmd_is_returned(self):
        manager = DdsManager(_build_config())
        cached = LowCmdCache(
            mode_pr=0,
            mode_machine=0,
            joint_positions_dds=tuple([0.0] * 29),
            joint_velocities_dds=tuple([0.0] * 29),
            joint_torques_dds=tuple([0.0] * 29),
            joint_kp_dds=tuple([0.0] * 29),
            joint_kd_dds=tuple([0.0] * 29),
            received_at_monotonic=time.monotonic(),
        )
        manager._lowcmd_subscriber = _FakeLowCmdSubscriber(latest_command=cached)

        resolved = manager._resolve_latest_lowcmd(now_monotonic=time.monotonic())

        self.assertIs(resolved, cached)

    def test_lowcmd_timeout_disabled_keeps_old_command_fresh(self):
        manager = DdsManager(_build_config(lowcmd_timeout_seconds=0.0))
        cached = LowCmdCache(
            mode_pr=0,
            mode_machine=0,
            joint_positions_dds=tuple([0.0] * 29),
            joint_velocities_dds=tuple([0.0] * 29),
            joint_torques_dds=tuple([0.0] * 29),
            joint_kp_dds=tuple([0.0] * 29),
            joint_kd_dds=tuple([0.0] * 29),
            received_at_monotonic=0.0,
        )
        manager._lowcmd_subscriber = _FakeLowCmdSubscriber(latest_command=cached)

        resolved = manager._resolve_latest_lowcmd(now_monotonic=10_000.0)

        self.assertIs(resolved, cached)
        self.assertFalse(manager._warned_stale_lowcmd)

    def test_cadence_reporting_emits_observed_rate(self):
        manager = DdsManager(_build_config())
        output = io.StringIO()

        with redirect_stdout(output):
            manager._cadence.record(0.00, expected_hz=100.0, interval=3, warn_ratio=0.05)
            manager._cadence.record(0.01, expected_hz=100.0, interval=3, warn_ratio=0.05)
            manager._cadence.record(0.02, expected_hz=100.0, interval=3, warn_ratio=0.05)

        self.assertIn("lowstate cadence check", output.getvalue())
        self.assertIn("observed=100.000Hz", output.getvalue())

    def test_lowstate_schedule_does_not_reanchor_to_current_frame(self):
        manager = DdsManager(_build_config())
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeLowStatePublisher()
        manager._lowcmd_subscriber = _FakeLowCmdSubscriber()

        snapshot = object()
        published_frames = []
        for frame_index in range(1, 13):
            simulation_time_seconds = frame_index * (1.0 / 120.0)
            result = manager.step(simulation_time_seconds, snapshot)
            if result.lowstate_published:
                published_frames.append(frame_index)

        self.assertEqual(
            published_frames,
            [1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12],
        )
        self.assertEqual(manager._lowstate_publisher.publish_calls, len(published_frames))


if __name__ == "__main__":
    unittest.main()
