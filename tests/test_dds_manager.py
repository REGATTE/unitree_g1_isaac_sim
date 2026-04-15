import os
import sys
import time
import unittest
from unittest.mock import patch
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import AppConfig
from dds.common.lowcmd_types import LowCmdCache
from dds.ros2.manager import DdsManager


class _FakeLowCmdSubscriber:
    def __init__(self, latest_command=None):
        self.latest_command = latest_command
        self.clear_calls = 0
        self.poll_calls = 0
        self.bound_port = 35502

    def poll(self):
        self.poll_calls += 1
        return None

    def clear_cached_command(self):
        self.latest_command = None
        self.clear_calls += 1

    def initialize(self):
        return True

    def close(self):
        return None


class _FakeLowStatePublisher:
    def __init__(self):
        self.publish_calls = 0

    def initialize(self):
        return True

    def publish(self, snapshot):
        self.publish_calls += 1
        return True

    def close(self):
        return None


class _FakeSecondaryImuPublisher:
    def __init__(self):
        self.publish_calls = 0

    def initialize(self):
        return True

    def publish(self, snapshot):
        self.publish_calls += 1
        return True

    def close(self):
        return None


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
        reset_after_frames=0,
        print_all_joints=False,
        enable_dds=True,
        dds_domain_id=1,
        enable_ros2_lowstate=True,
        enable_ros2_lowcmd=True,
        lowstate_topic="rt/lowstate",
        lowcmd_topic="rt/lowcmd",
        lowstate_publish_hz=100.0,
        lowcmd_max_position_delta_rad=0.25,
        enable_native_unitree_lowstate=True,
        enable_native_unitree_lowcmd=False,
        native_unitree_domain_id=1,
        native_unitree_lowstate_topic="rt/lowstate",
        native_unitree_lowcmd_topic="rt/lowcmd",
        native_unitree_bridge_exe=Path("/tmp/unitree_g1_native_bridge"),
        enable_unitree_sdk2py_lowstate=False,
        enable_unitree_sdk2py_lowcmd=False,
        unitree_sdk2py_domain_id=1,
        unitree_sdk2py_lowstate_topic="rt/lowstate",
        unitree_sdk2py_lowcmd_topic="rt/lowcmd",
        unitree_sdk2py_network_interface="lo",
        lowcmd_timeout_seconds=0.5,
        lowstate_cadence_report_interval=3,
        lowstate_cadence_warn_ratio=0.05,
        unitree_ros2_install_prefix=None,
        ros2_python_exe="/usr/bin/python3",
        bridge_bind_host="127.0.0.1",
        bridge_lowstate_port=35501,
        bridge_lowcmd_port=35502,
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

        with self.assertLogs("unitree_g1_isaac_sim.dds.timing", level="INFO") as captured:
            manager._simulation_cadence.record(0.00, expected_hz=100.0, interval=3, warn_ratio=0.05)
            manager._simulation_cadence.record(0.01, expected_hz=100.0, interval=3, warn_ratio=0.05)
            manager._simulation_cadence.record(0.02, expected_hz=100.0, interval=3, warn_ratio=0.05)

        output = "\n".join(captured.output)
        self.assertIn("lowstate cadence check (simulation_time)", output)
        self.assertIn("observed=100.000Hz", output)

    def test_wall_clock_cadence_reporting_emits_distinct_label(self):
        manager = DdsManager(_build_config())

        with self.assertLogs("unitree_g1_isaac_sim.dds.timing", level="INFO") as captured:
            manager._wall_clock_cadence.record(10.00, expected_hz=100.0, interval=3, warn_ratio=0.05)
            manager._wall_clock_cadence.record(10.01, expected_hz=100.0, interval=3, warn_ratio=0.05)
            manager._wall_clock_cadence.record(10.02, expected_hz=100.0, interval=3, warn_ratio=0.05)

        output = "\n".join(captured.output)
        self.assertIn("lowstate cadence check (wall_clock)", output)
        self.assertIn("observed=100.000Hz", output)

    def test_lowstate_schedule_does_not_reanchor_to_current_frame(self):
        manager = DdsManager(_build_config())
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeLowStatePublisher()
        manager._secondary_imu_publisher = _FakeSecondaryImuPublisher()
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
        self.assertEqual(manager._secondary_imu_publisher.publish_calls, len(published_frames))

    def test_reset_runtime_state_clears_transient_dds_state(self):
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
        subscriber = _FakeLowCmdSubscriber(latest_command=cached)
        manager._lowcmd_subscriber = subscriber
        manager._warned_stale_lowcmd = True
        manager._next_lowstate_publish_time = 12.5
        manager._simulation_cadence.window_start_time = 5.0
        manager._simulation_cadence.publish_count = 10
        manager._wall_clock_cadence.window_start_time = 6.0
        manager._wall_clock_cadence.publish_count = 11

        manager.reset_runtime_state()

        self.assertIsNone(subscriber.latest_command)
        self.assertEqual(subscriber.clear_calls, 1)
        self.assertFalse(manager._warned_stale_lowcmd)
        self.assertEqual(manager._next_lowstate_publish_time, 0.0)
        self.assertIsNone(manager._simulation_cadence.window_start_time)
        self.assertEqual(manager._simulation_cadence.publish_count, 0)
        self.assertIsNone(manager._wall_clock_cadence.window_start_time)
        self.assertEqual(manager._wall_clock_cadence.publish_count, 0)

    def test_step_does_not_poll_lowcmd_when_subscriber_disabled(self):
        manager = DdsManager(_build_config(enable_ros2_lowcmd=False))
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeLowStatePublisher()
        manager._secondary_imu_publisher = _FakeSecondaryImuPublisher()
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
        subscriber = _FakeLowCmdSubscriber(latest_command=cached)
        manager._lowcmd_subscriber = subscriber

        result = manager.step(0.01, object())

        self.assertEqual(subscriber.poll_calls, 0)
        self.assertFalse(result.lowcmd_available)
        self.assertFalse(result.lowcmd_fresh)

    def test_find_stale_sidecar_pids_filters_current_processes(self):
        manager = DdsManager(_build_config())
        sidecar_script = REPO_ROOT / "scripts" / "ros2_cyclonedds_sidecar.py"
        manager._bridge_process = type("Proc", (), {"pid": 22222, "poll": lambda self: None})()
        ps_output = "\n".join(
            [
                f"{os.getpid()} /usr/bin/python3 {sidecar_script}",
                f"22222 /usr/bin/python3 {sidecar_script}",
                f"33333 /usr/bin/python3 {sidecar_script}",
                "44444 /usr/bin/python3 some_other_script.py",
            ]
        )

        with patch("dds.ros2.manager.subprocess.run") as run_mock:
            run_mock.return_value.stdout = ps_output
            stale = manager._find_stale_sidecar_pids(sidecar_script)

        self.assertEqual(stale, [33333])

    def test_sidecar_launch_omits_lowcmd_subscription_when_ros2_lowcmd_disabled(self):
        manager = DdsManager(
            _build_config(
                enable_ros2_lowcmd=False,
                unitree_ros2_install_prefix=Path("/tmp/unitree_ros2_install"),
            )
        )

        with patch.object(manager, "_cleanup_stale_sidecars"):
            with patch("dds.ros2.manager.subprocess.Popen") as popen_mock:
                popen_mock.return_value.pid = 12345
                manager._start_sidecar_bridge()

        command = popen_mock.call_args.args[0][2]
        self.assertNotIn("--enable-lowcmd", command)
        self.assertIn("--secondary-imu-topic 'rt/secondary_imu'", command)
        self.assertIn("--secondary-imu-port 35503", command)
        self.assertIn("--enable-secondary-imu", command)
        self.assertIn("--lowcmd-port 35502", command)

    def test_sidecar_launch_enables_lowcmd_subscription_when_ros2_lowcmd_enabled(self):
        manager = DdsManager(
            _build_config(
                enable_ros2_lowcmd=True,
                unitree_ros2_install_prefix=Path("/tmp/unitree_ros2_install"),
            )
        )

        with patch.object(manager, "_cleanup_stale_sidecars"):
            with patch("dds.ros2.manager.subprocess.Popen") as popen_mock:
                popen_mock.return_value.pid = 12345
                manager._start_sidecar_bridge()

        command = popen_mock.call_args.args[0][2]
        self.assertIn("--secondary-imu-topic 'rt/secondary_imu'", command)
        self.assertIn("--secondary-imu-port 35503", command)
        self.assertIn("--enable-secondary-imu", command)
        self.assertIn("--enable-lowcmd", command)

    def test_secondary_imu_still_publishes_when_ros2_lowstate_is_disabled(self):
        manager = DdsManager(_build_config(enable_ros2_lowstate=False, enable_ros2_lowcmd=True))
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeLowStatePublisher()
        manager._secondary_imu_publisher = _FakeSecondaryImuPublisher()
        manager._lowcmd_subscriber = _FakeLowCmdSubscriber()

        result = manager.step(1.0 / 120.0, object())

        self.assertFalse(result.lowstate_published)
        self.assertEqual(manager._lowstate_publisher.publish_calls, 0)
        self.assertEqual(manager._secondary_imu_publisher.publish_calls, 1)


if __name__ == "__main__":
    unittest.main()
