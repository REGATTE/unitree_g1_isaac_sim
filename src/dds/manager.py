"""Top-level DDS lifecycle management for the Unitree G1 simulator."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import subprocess
import time

from config import AppConfig
from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger

from .g1_lowcmd import G1LowCmdSubscriber, LowCmdCache
from .g1_lowstate import G1LowStatePublisher


LOGGER = get_logger("dds.manager")


@dataclass(frozen=True)
class DdsStepResult:
    """Outcome of one DDS update from the simulation loop."""

    lowstate_published: bool
    lowcmd_available: bool
    lowcmd_fresh: bool


@dataclass
class CadenceTracker:
    """Track rolling lowstate cadence diagnostics."""

    window_start_time: float | None = None
    publish_count: int = 0

    def record(self, t: float, expected_hz: float, interval: int, warn_ratio: float) -> None:
        if interval <= 0:
            return
        if self.window_start_time is None:
            self.window_start_time = t
            self.publish_count = 1
            return

        self.publish_count += 1
        if self.publish_count < interval:
            return

        elapsed = t - self.window_start_time
        publishes = self.publish_count
        observed_hz = 0.0 if elapsed <= 0.0 else (publishes - 1) / elapsed
        relative_error = 0.0 if expected_hz <= 0.0 else abs(observed_hz - expected_hz) / expected_hz
        log_method = LOGGER.warning if relative_error > warn_ratio else LOGGER.info
        log_method(
            "lowstate cadence check: observed=%.3fHz expected=%.3fHz publishes=%s "
            "window_dt=%.3fs rel_error=%.3f%%",
            observed_hz,
            expected_hz,
            publishes,
            elapsed,
            relative_error * 100.0,
        )

        self.window_start_time = t
        self.publish_count = 1


class DdsManager:
    """Own the DDS bridge lifecycle for the single-process simulator runtime."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._initialized = False
        self._sdk_enabled = False
        self._bridge_process: subprocess.Popen[str] | None = None
        self._next_lowstate_publish_time = 0.0
        lowstate_hz = config.lowstate_publish_hz
        if lowstate_hz <= 0.0 or not math.isfinite(lowstate_hz):
            raise ValueError(
                f"Invalid AppConfig.lowstate_publish_hz={lowstate_hz!r}; "
                "expected a positive finite frequency in Hz (check --lowstate-publish-hz)."
            )
        self._lowstate_publish_period = _compute_publish_period(lowstate_hz)
        self._lowstate_publisher = G1LowStatePublisher(
            host=config.bridge_bind_host,
            port=config.bridge_lowstate_port,
        )
        self._lowcmd_subscriber = G1LowCmdSubscriber(
            bind_host=config.bridge_bind_host,
            bind_port=config.bridge_lowcmd_port,
        )
        self._warned_stale_lowcmd = False
        self._cadence = CadenceTracker()

    @property
    def latest_lowcmd(self) -> LowCmdCache | None:
        return self._resolve_latest_lowcmd(now_monotonic=time.monotonic())

    def initialize(self) -> bool:
        """Initialize the localhost bridge plus ROS 2 sidecar process."""
        if self._initialized:
            return self._sdk_enabled
        self._initialized = True

        if not self._config.enable_dds:
            LOGGER.info("DDS bridge disabled")
            return False
        if self._config.unitree_ros2_install_prefix is None:
            LOGGER.warning(
                "DDS bridge enabled but the Unitree ROS 2 install prefix is unavailable. "
                "The simulator will continue without external DDS I/O.",
            )
            return False

        self._start_sidecar_bridge()
        self._lowstate_publisher.initialize()
        if self._config.enable_lowcmd_subscriber:
            self._lowcmd_subscriber.initialize()
        else:
            LOGGER.info("lowcmd subscriber disabled for this run")
        self._sdk_enabled = True
        LOGGER.info(
            "initialized localhost DDS sidecar bridge (domain_id=%s)",
            self._config.dds_domain_id,
        )
        return True

    def step(self, simulation_time_seconds: float, snapshot: RobotKinematicSnapshot) -> DdsStepResult:
        """Run one DDS update from the main simulation loop.

        The publish cadence is derived from simulation time, not host wall
        time, so the DDS state stream stays aligned with the physics loop.
        """
        self.initialize()

        if self._sdk_enabled:
            self._poll_sidecar_health()
            self._lowcmd_subscriber.poll()

        lowstate_published = False
        if self._sdk_enabled and simulation_time_seconds >= self._next_lowstate_publish_time:
            lowstate_published = self._lowstate_publisher.publish(snapshot)
            self._advance_lowstate_publish_schedule(simulation_time_seconds)
            if lowstate_published:
                self._cadence.record(
                    simulation_time_seconds,
                    expected_hz=self._config.lowstate_publish_hz,
                    interval=self._config.lowstate_cadence_report_interval,
                    warn_ratio=self._config.lowstate_cadence_warn_ratio,
                )

        active_lowcmd = self._resolve_latest_lowcmd(now_monotonic=time.monotonic())

        return DdsStepResult(
            lowstate_published=lowstate_published,
            lowcmd_available=self._lowcmd_subscriber.latest_command is not None,
            lowcmd_fresh=active_lowcmd is not None,
        )

    def shutdown(self) -> None:
        """Release DDS-facing objects owned by the manager."""
        if self._bridge_process is not None and self._bridge_process.poll() is None:
            self._bridge_process.terminate()
            try:
                self._bridge_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bridge_process.kill()
                self._bridge_process.wait(timeout=5.0)
        self._bridge_process = None
        self._lowstate_publisher = G1LowStatePublisher(
            host=self._config.bridge_bind_host,
            port=self._config.bridge_lowstate_port,
        )
        self._lowcmd_subscriber = G1LowCmdSubscriber(
            bind_host=self._config.bridge_bind_host,
            bind_port=self._config.bridge_lowcmd_port,
        )
        self._sdk_enabled = False
        self._initialized = False
        self._warned_stale_lowcmd = False
        self._next_lowstate_publish_time = 0.0
        self._cadence = CadenceTracker()

    def reset_runtime_state(self) -> None:
        """Clear transient DDS runtime state after an in-session simulator reset."""
        if hasattr(self._lowcmd_subscriber, "clear_cached_command"):
            self._lowcmd_subscriber.clear_cached_command()
        self._warned_stale_lowcmd = False
        self._next_lowstate_publish_time = 0.0
        self._cadence = CadenceTracker()
        LOGGER.info("cleared DDS runtime state after simulator reset")

    def _resolve_latest_lowcmd(self, now_monotonic: float) -> LowCmdCache | None:
        """Return the current fresh lowcmd sample or `None` if stale/absent."""
        cached = self._lowcmd_subscriber.latest_command
        is_fresh = _is_fresh(now_monotonic, cached, self._config.lowcmd_timeout_seconds)
        if not is_fresh:
            if cached is None:
                self._warned_stale_lowcmd = False
                return None
            if not self._warned_stale_lowcmd:
                age_seconds = now_monotonic - cached.received_at_monotonic
                LOGGER.warning(
                    "cached `rt/lowcmd` sample went stale after %.3fs; stopping command "
                    "reapplication until a fresh sample arrives.",
                    age_seconds,
                )
                self._warned_stale_lowcmd = True
            return None

        if cached is None:
            self._warned_stale_lowcmd = False
            return None
        if self._warned_stale_lowcmd:
            LOGGER.info("received fresh `rt/lowcmd` again; resuming command application.")
        self._warned_stale_lowcmd = False
        return cached

    def _advance_lowstate_publish_schedule(self, simulation_time_seconds: float) -> None:
        """Advance the next publish target from the prior schedule, not the current frame."""
        while self._next_lowstate_publish_time <= simulation_time_seconds:
            self._next_lowstate_publish_time += self._lowstate_publish_period

    def _poll_sidecar_health(self) -> None:
        if self._bridge_process is None:
            return
        return_code = self._bridge_process.poll()
        if return_code is not None and self._sdk_enabled:
            LOGGER.error("ROS 2 sidecar bridge exited unexpectedly with code %s", return_code)
            self._sdk_enabled = False

    def _start_sidecar_bridge(self) -> None:
        if self._bridge_process is not None and self._bridge_process.poll() is None:
            return

        install_prefix = Path(self._config.unitree_ros2_install_prefix)
        sidecar_script = Path(__file__).resolve().parents[2] / "scripts" / "ros2_cyclonedds_sidecar.py"
        ros2_python_exe = self._config.ros2_python_exe or "/usr/bin/python3"
        clean_env = {
            key: value
            for key, value in os.environ.items()
            if key
            not in {
                "PYTHONHOME",
                "PYTHONPATH",
                "LD_PRELOAD",
            }
        }
        clean_env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        clean_env["ROS_DOMAIN_ID"] = str(self._config.dds_domain_id)
        command = (
            f"source /opt/ros/humble/setup.bash >/dev/null 2>&1 && "
            f"source '{install_prefix / 'setup.bash'}' >/dev/null 2>&1 && "
            f"exec '{ros2_python_exe}' '{sidecar_script}' "
            f"--lowstate-topic '{self._config.lowstate_topic}' "
            f"--lowcmd-topic '{self._config.lowcmd_topic}' "
            f"--bind-host '{self._config.bridge_bind_host}' "
            f"--lowstate-port {self._config.bridge_lowstate_port} "
            f"--lowcmd-port {self._config.bridge_lowcmd_port}"
        )
        self._bridge_process = subprocess.Popen(
            ["bash", "-lc", command],
            cwd=Path(__file__).resolve().parents[2],
            env=clean_env,
        )
        LOGGER.info(
            "started ROS 2 sidecar bridge pid=%s using install_prefix=%s",
            self._bridge_process.pid,
            install_prefix,
        )


def _compute_publish_period(publish_hz: float) -> float:
    """Convert a positive publish rate into a simulation-time period."""
    if publish_hz <= 0.0:
        raise ValueError(f"lowstate publish rate must be positive, got {publish_hz}")
    if not math.isfinite(publish_hz):
        raise ValueError(f"lowstate publish rate must be finite, got {publish_hz}")
    return 1.0 / publish_hz


def _is_fresh(now_monotonic: float, cached: LowCmdCache | None, timeout_seconds: float) -> bool:
    """Return whether a cached lowcmd sample should be treated as fresh."""
    if cached is None:
        return False
    if timeout_seconds <= 0.0:
        return True
    return (now_monotonic - cached.received_at_monotonic) <= timeout_seconds
