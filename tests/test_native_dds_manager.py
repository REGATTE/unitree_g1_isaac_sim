import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import AppConfig
from dds.native_manager import NativeUnitreeDdsManager


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
        manager._next_lowstate_publish_time = 12.5
        manager._simulation_cadence.window_start_time = 5.0
        manager._simulation_cadence.publish_count = 10
        manager._wall_clock_cadence.window_start_time = 6.0
        manager._wall_clock_cadence.publish_count = 11

        manager.reset_runtime_state()

        self.assertEqual(manager._next_lowstate_publish_time, 0.0)
        self.assertIsNone(manager._simulation_cadence.window_start_time)
        self.assertEqual(manager._simulation_cadence.publish_count, 0)
        self.assertIsNone(manager._wall_clock_cadence.window_start_time)
        self.assertEqual(manager._wall_clock_cadence.publish_count, 0)


if __name__ == "__main__":
    unittest.main()
