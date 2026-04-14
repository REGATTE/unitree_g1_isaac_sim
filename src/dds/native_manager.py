"""Native Unitree SDK bridge lifecycle for the publish-only Phase 1 path."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import subprocess
import time

from config import AppConfig
from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger

from .lowcmd_types import LowCmdCache
from .manager import CadenceTracker, _compute_publish_period
from .native_udp_lowstate import NativeLowStateUdpPublisher


LOGGER = get_logger("dds.native_manager")

NATIVE_UDP_LOWSTATE_PORT = 35511


@dataclass(frozen=True)
class NativeDdsStepResult:
    """Outcome of one native DDS update from the simulation loop."""

    lowstate_published: bool
    lowcmd_available: bool
    lowcmd_fresh: bool


class NativeUnitreeDdsManager:
    """Own the native Unitree SDK bridge lifecycle for Isaac Sim."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._initialized = False
        self._sdk_enabled = False
        self._bridge_process: subprocess.Popen[str] | None = None
        lowstate_hz = config.lowstate_publish_hz
        if lowstate_hz <= 0.0 or not math.isfinite(lowstate_hz):
            raise ValueError(
                f"Invalid AppConfig.lowstate_publish_hz={lowstate_hz!r}; "
                "expected a positive finite frequency in Hz."
            )
        self._next_lowstate_publish_time = 0.0
        self._lowstate_publish_period = _compute_publish_period(lowstate_hz)
        self._lowstate_publisher = NativeLowStateUdpPublisher(
            host=config.bridge_bind_host,
            port=NATIVE_UDP_LOWSTATE_PORT,
        )
        self._simulation_cadence = CadenceTracker(label="native_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="native_wall_clock")
        self._warned_missing_bridge = False
        self._warned_lowcmd_unimplemented = False

    @property
    def latest_lowcmd(self) -> LowCmdCache | None:
        return None

    def initialize(self) -> bool:
        """Initialize the publish-only native DDS bridge."""
        if self._initialized:
            return self._sdk_enabled
        self._initialized = True

        if not self._config.enable_native_unitree_lowstate:
            LOGGER.info("native lowstate publisher disabled for this run")
            return False

        bridge_executable = self._config.native_unitree_bridge_exe.expanduser().resolve()
        if not bridge_executable.exists():
            if not self._warned_missing_bridge:
                LOGGER.warning(
                    "native Unitree bridge executable is unavailable at %s; "
                    "continuing without native DDS I/O.",
                    bridge_executable,
                )
                self._warned_missing_bridge = True
            return False

        try:
            self._start_bridge(bridge_executable)
            self._lowstate_publisher.initialize()
            self._sdk_enabled = True
            if self._config.enable_native_unitree_lowcmd and not self._warned_lowcmd_unimplemented:
                LOGGER.info(
                    "native lowcmd is configured but not implemented in Phase 1; "
                    "native bridge is running in lowstate publish-only mode."
                )
                self._warned_lowcmd_unimplemented = True
            LOGGER.info(
                "initialized native Unitree bridge (domain_id=%s topic=%s)",
                self._config.native_unitree_domain_id,
                self._config.native_unitree_lowstate_topic,
            )
            return True
        except Exception:
            self.shutdown()
            raise

    def step(self, simulation_time_seconds: float, snapshot: RobotKinematicSnapshot) -> NativeDdsStepResult:
        self.initialize()

        if self._sdk_enabled:
            self._poll_bridge_health()

        lowstate_published = False
        if self._sdk_enabled and simulation_time_seconds >= self._next_lowstate_publish_time:
            lowstate_published = self._lowstate_publisher.publish(snapshot)
            self._advance_lowstate_publish_schedule(simulation_time_seconds)
            if lowstate_published:
                wall_clock_seconds = time.monotonic()
                self._simulation_cadence.record(
                    simulation_time_seconds,
                    expected_hz=self._config.lowstate_publish_hz,
                    interval=self._config.lowstate_cadence_report_interval,
                    warn_ratio=self._config.lowstate_cadence_warn_ratio,
                )
                self._wall_clock_cadence.record(
                    wall_clock_seconds,
                    expected_hz=self._config.lowstate_publish_hz,
                    interval=self._config.lowstate_cadence_report_interval,
                    warn_ratio=self._config.lowstate_cadence_warn_ratio,
                )

        return NativeDdsStepResult(
            lowstate_published=lowstate_published,
            lowcmd_available=False,
            lowcmd_fresh=False,
        )

    def reset_runtime_state(self) -> None:
        self._next_lowstate_publish_time = 0.0
        self._simulation_cadence = CadenceTracker(label="native_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="native_wall_clock")
        LOGGER.info("cleared native DDS runtime state after simulator reset")

    def shutdown(self) -> None:
        if self._bridge_process is not None and self._bridge_process.poll() is None:
            self._bridge_process.terminate()
            try:
                self._bridge_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._bridge_process.kill()
                self._bridge_process.wait(timeout=5.0)
        self._bridge_process = None
        self._lowstate_publisher.close()
        self._lowstate_publisher = NativeLowStateUdpPublisher(
            host=self._config.bridge_bind_host,
            port=NATIVE_UDP_LOWSTATE_PORT,
        )
        self._sdk_enabled = False
        self._initialized = False
        self._next_lowstate_publish_time = 0.0
        self._simulation_cadence = CadenceTracker(label="native_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="native_wall_clock")

    def _start_bridge(self, bridge_executable: Path) -> None:
        if self._bridge_process is not None and self._bridge_process.poll() is None:
            return
        command = [
            str(bridge_executable),
            "--domain-id",
            str(self._config.native_unitree_domain_id),
            "--lowstate-topic",
            self._config.native_unitree_lowstate_topic,
            "--bind-host",
            self._config.bridge_bind_host,
            "--lowstate-port",
            str(NATIVE_UDP_LOWSTATE_PORT),
        ]
        self._bridge_process = subprocess.Popen(
            command,
            cwd=Path(__file__).resolve().parents[2],
            text=True,
        )

    def _poll_bridge_health(self) -> None:
        if self._bridge_process is None:
            return
        return_code = self._bridge_process.poll()
        if return_code is not None and self._sdk_enabled:
            LOGGER.error("native Unitree bridge exited unexpectedly with code %s", return_code)
            self._sdk_enabled = False

    def _advance_lowstate_publish_schedule(self, simulation_time_seconds: float) -> None:
        while self._next_lowstate_publish_time <= simulation_time_seconds:
            self._next_lowstate_publish_time += self._lowstate_publish_period
