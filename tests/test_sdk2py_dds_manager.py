import os
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from config import AppConfig
from dds.sdk2py.manager import SDK2PY_UDP_LOWSTATE_PORT, UnitreeSdk2PyDdsManager


class _FakeSdk2PyLowStatePublisher:
    def __init__(self):
        self.publish_calls = 0
        self.initialize_calls = 0
        self.close_calls = 0

    def initialize(self):
        self.initialize_calls += 1
        return True

    def publish(self, snapshot):
        self.publish_calls += 1
        return True

    def close(self):
        self.close_calls += 1


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
        enable_native_unitree_lowstate=False,
        enable_native_unitree_lowcmd=False,
        native_unitree_domain_id=1,
        native_unitree_lowstate_topic="rt/lowstate",
        native_unitree_lowcmd_topic="rt/lowcmd",
        native_unitree_bridge_exe=Path("/tmp/unitree_g1_native_bridge"),
        enable_unitree_sdk2py_lowstate=True,
        enable_unitree_sdk2py_lowcmd=True,
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


class UnitreeSdk2PyDdsManagerTests(unittest.TestCase):
    def test_initialize_starts_sdk2py_sidecar_for_lowstate(self):
        manager = UnitreeSdk2PyDdsManager(
            _build_config(
                unitree_sdk2py_domain_id=7,
                unitree_sdk2py_lowstate_topic="rt/custom_lowstate",
                unitree_sdk2py_network_interface="eth0",
            )
        )
        manager._lowstate_publisher = _FakeSdk2PyLowStatePublisher()

        with patch.object(manager, "_cleanup_stale_sidecars"):
            with patch("dds.sdk2py.manager.subprocess.Popen") as popen_mock:
                popen_mock.return_value.pid = 12345
                initialized = manager.initialize()

        self.assertTrue(initialized)
        command = popen_mock.call_args.args[0]
        self.assertEqual(command[0], "python3")
        self.assertIn("unitree_sdk2py_sidecar.py", command[1])
        self.assertIn("--domain-id", command)
        self.assertIn("7", command)
        self.assertIn("--lowstate-topic", command)
        self.assertIn("rt/custom_lowstate", command)
        self.assertIn("--network-interface", command)
        self.assertIn("eth0", command)
        self.assertIn("--lowstate-port", command)
        self.assertIn(str(SDK2PY_UDP_LOWSTATE_PORT), command)
        self.assertEqual(manager._lowstate_publisher.initialize_calls, 1)

    def test_initialize_does_not_start_sidecar_when_only_phase2_lowcmd_is_selected(self):
        manager = UnitreeSdk2PyDdsManager(
            _build_config(
                enable_unitree_sdk2py_lowstate=False,
                enable_unitree_sdk2py_lowcmd=True,
            )
        )

        with patch("dds.sdk2py.manager.subprocess.Popen") as popen_mock:
            initialized = manager.initialize()

        self.assertFalse(initialized)
        popen_mock.assert_not_called()

    def test_lowstate_schedule_does_not_reanchor_to_current_frame(self):
        manager = UnitreeSdk2PyDdsManager(_build_config())
        manager._initialized = True
        manager._sdk_enabled = True
        manager._lowstate_publisher = _FakeSdk2PyLowStatePublisher()

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

    def test_reset_runtime_state_clears_sdk2py_cadence_state(self):
        manager = UnitreeSdk2PyDdsManager(_build_config())
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

    def test_find_stale_sidecar_pids_matches_sidecar_script_and_skips_current_processes(self):
        manager = UnitreeSdk2PyDdsManager(_build_config())
        sidecar_script = REPO_ROOT / "scripts" / "unitree_sdk2py_sidecar.py"
        manager._bridge_process = type("Proc", (), {"pid": 22222, "poll": lambda self: None})()
        ps_output = "\n".join(
            [
                f"{os.getpid()} /usr/bin/python3 {sidecar_script}",
                f"22222 /usr/bin/python3 {sidecar_script}",
                f"33333 /usr/bin/python3 {sidecar_script}",
                "44444 /usr/bin/python3 some_other_script.py",
            ]
        )

        with patch("dds.sdk2py.manager.subprocess.run") as run_mock:
            run_mock.return_value.stdout = ps_output
            stale = manager._find_stale_sidecar_pids(sidecar_script)

        self.assertEqual(stale, [33333])

    def test_bridge_environment_removes_isaac_python_runtime_variables(self):
        manager = UnitreeSdk2PyDdsManager(_build_config())

        with patch(
            "dds.sdk2py.manager.os.environ",
            {
                "PYTHONHOME": "/tmp/isaac/python",
                "PYTHONPATH": "/tmp/isaac/python/lib",
                "LD_PRELOAD": "/tmp/preload.so",
                "KEEP_ME": "1",
            },
        ):
            env = manager._bridge_environment()

        self.assertNotIn("PYTHONHOME", env)
        self.assertNotIn("LD_PRELOAD", env)
        self.assertEqual(env["KEEP_ME"], "1")
        if (Path.home() / "unitree_sdk2_python").exists():
            self.assertEqual(env["PYTHONPATH"], str(Path.home() / "unitree_sdk2_python"))
        else:
            self.assertNotIn("PYTHONPATH", env)


if __name__ == "__main__":
    unittest.main()
