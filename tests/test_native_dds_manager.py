import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import AppConfig
from dds.common.lowcmd_types import LowCmdCache
from dds.native.manager import NativeUnitreeDdsManager


class _FakeNativeLowStatePublisher:
    def __init__(self):
        self.publish_calls = 0

    def initialize(self):
        return True

    def publish(self, snapshot):
        self.publish_calls += 1
        return True

    def close(self):
        return None


class _FakeNativeLowCmdSubscriber:
    def __init__(self, command=None):
        self.latest_command = command
        self.poll_calls = 0
        self.clear_calls = 0
        self.close_calls = 0

    @property
    def bound_port(self):
        return 45678

    def initialize(self):
        return True

    def poll(self):
        self.poll_calls += 1

    def clear_cached_command(self):
        self.clear_calls += 1
        self.latest_command = None

    def close(self):
        self.close_calls += 1


def _lowcmd(received_at=100.0):
    return LowCmdCache(
        mode_pr=0,
        mode_machine=0,
        joint_positions_dds=tuple(0.0 for _ in range(29)),
        joint_velocities_dds=tuple(0.0 for _ in range(29)),
        joint_torques_dds=tuple(0.0 for _ in range(29)),
        joint_kp_dds=tuple(0.0 for _ in range(29)),
        joint_kd_dds=tuple(0.0 for _ in range(29)),
        received_at_monotonic=received_at,
    )


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
        enable_ros2_lowcmd=False,
        lowstate_topic="rt/lowstate",
        lowcmd_topic="rt/lowcmd",
        lowstate_publish_hz=100.0,
        lowcmd_max_position_delta_rad=0.25,
        enable_native_unitree_lowstate=True,
        enable_native_unitree_lowcmd=True,
        native_unitree_domain_id=1,
        native_unitree_lowstate_topic="rt/lowstate",
        native_unitree_lowcmd_topic="rt/lowcmd",
        native_unitree_bridge_exe=Path("/tmp/unitree_g1_native_bridge"),
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


class NativeUnitreeDdsManagerTests(unittest.TestCase):
    def test_initialize_returns_false_when_bridge_binary_is_missing(self):
        manager = NativeUnitreeDdsManager(_build_config(native_unitree_bridge_exe=Path("/tmp/does_not_exist")))

        initialized = manager.initialize()

        self.assertFalse(initialized)

    def test_lowstate_schedule_does_not_reanchor_to_current_frame(self):
        manager = NativeUnitreeDdsManager(_build_config())
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeNativeLowStatePublisher()
        manager._lowcmd_subscriber = _FakeNativeLowCmdSubscriber()

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

    def test_reset_runtime_state_clears_native_cadence_state(self):
        manager = NativeUnitreeDdsManager(_build_config())
        manager._lowcmd_subscriber = _FakeNativeLowCmdSubscriber(command=_lowcmd())
        manager._next_lowstate_publish_time = 12.5
        manager._simulation_cadence.window_start_time = 5.0
        manager._simulation_cadence.publish_count = 10
        manager._wall_clock_cadence.window_start_time = 6.0
        manager._wall_clock_cadence.publish_count = 11

        manager.reset_runtime_state()

        self.assertEqual(manager._next_lowstate_publish_time, 0.0)
        self.assertIsNone(manager._lowcmd_subscriber.latest_command)
        self.assertEqual(manager._lowcmd_subscriber.clear_calls, 1)
        self.assertIsNone(manager._simulation_cadence.window_start_time)
        self.assertEqual(manager._simulation_cadence.publish_count, 0)
        self.assertIsNone(manager._wall_clock_cadence.window_start_time)
        self.assertEqual(manager._wall_clock_cadence.publish_count, 0)

    def test_resolve_latest_lowcmd_returns_fresh_native_command(self):
        command = _lowcmd(received_at=10.0)
        manager = NativeUnitreeDdsManager(_build_config(lowcmd_timeout_seconds=0.5))
        manager._lowcmd_subscriber = _FakeNativeLowCmdSubscriber(command=command)

        resolved = manager._resolve_latest_lowcmd(now_monotonic=10.25)

        self.assertIs(resolved, command)

    def test_resolve_latest_lowcmd_drops_stale_native_command(self):
        command = _lowcmd(received_at=10.0)
        manager = NativeUnitreeDdsManager(_build_config(lowcmd_timeout_seconds=0.5))
        manager._lowcmd_subscriber = _FakeNativeLowCmdSubscriber(command=command)

        resolved = manager._resolve_latest_lowcmd(now_monotonic=11.0)

        self.assertIsNone(resolved)

    def test_step_polls_native_lowcmd_when_enabled(self):
        command = _lowcmd(received_at=10.0)
        manager = NativeUnitreeDdsManager(_build_config(lowcmd_timeout_seconds=0.0))
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeNativeLowStatePublisher()
        manager._lowcmd_subscriber = _FakeNativeLowCmdSubscriber(command=command)

        result = manager.step(1.0 / 120.0, object())

        self.assertEqual(manager._lowcmd_subscriber.poll_calls, 1)
        self.assertTrue(result.lowcmd_available)
        self.assertTrue(result.lowcmd_fresh)

    def test_find_stale_bridge_pids_matches_bridge_binary_and_skips_current_processes(self):
        manager = NativeUnitreeDdsManager(_build_config())
        bridge_exe = Path("/tmp/unitree_g1_native_bridge")
        ps_output = "\n".join(
            [
                "111 /tmp/unitree_g1_native_bridge --domain-id 1",
                "222 /usr/bin/python3 other_process.py",
                f"333 {sys.executable} {bridge_exe.name}-test-helper.py",
            ]
        )

        with patch("dds.native.manager.os.getpid", return_value=333):
            with patch("dds.native.manager.subprocess.run") as run_mock:
                run_mock.return_value.stdout = ps_output
                pids = manager._find_stale_bridge_pids(bridge_exe)

        self.assertEqual(pids, [111])

    def test_cleanup_stale_bridges_terminates_then_kills_remaining_processes(self):
        manager = NativeUnitreeDdsManager(_build_config())
        bridge_exe = Path("/tmp/unitree_g1_native_bridge")
        exists_calls = []

        def fake_pid_exists(pid):
            exists_calls.append(pid)
            return True

        with patch.object(manager, "_find_stale_bridge_pids", return_value=[111]):
            with patch.object(manager, "_pid_exists", side_effect=fake_pid_exists):
                with patch("dds.native.manager.os.kill") as kill_mock:
                    with patch("dds.native.manager.time.monotonic", side_effect=[0.0, 1.0, 3.0]):
                        manager._cleanup_stale_bridges(bridge_exe)

        kill_calls = [call.args for call in kill_mock.call_args_list]
        self.assertEqual(kill_calls[0], (111, 15))
        self.assertEqual(kill_calls[1], (111, 9))
        self.assertEqual(exists_calls, [111])


if __name__ == "__main__":
    unittest.main()
