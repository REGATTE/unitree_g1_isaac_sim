"""Unitree SDK2 Python bridge lifecycle for lowstate publication."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from config import AppConfig
from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger

from dds.common.lowcmd_types import LowCmdCache
from dds.common.timing import CadenceTracker, compute_publish_period

from .lowstate import Sdk2PyLowStateUdpPublisher


LOGGER = get_logger("dds.sdk2py_manager")

SDK2PY_UDP_LOWSTATE_PORT = 35521


@dataclass(frozen=True)
class Sdk2PyDdsStepResult:
    """Outcome of one SDK2 Python DDS update from the simulation loop."""

    lowstate_published: bool
    lowcmd_available: bool
    lowcmd_fresh: bool


class UnitreeSdk2PyDdsManager:
    """Own the Unitree SDK2 Python bridge lifecycle for Isaac Sim."""

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
        self._lowstate_publish_period = compute_publish_period(lowstate_hz)
        self._lowstate_publisher = Sdk2PyLowStateUdpPublisher(
            host=config.bridge_bind_host,
            port=SDK2PY_UDP_LOWSTATE_PORT,
        )
        self._simulation_cadence = CadenceTracker(label="sdk2py_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="sdk2py_wall_clock")

    @property
    def latest_lowcmd(self) -> LowCmdCache | None:
        """SDK2 Python lowcmd ingress lands in Phase 2."""
        return None

    def initialize(self) -> bool:
        """Initialize the SDK2 Python sidecar bridge."""
        if self._initialized:
            return self._sdk_enabled
        self._initialized = True

        if not self._config.enable_unitree_sdk2py_lowstate and not self._config.enable_unitree_sdk2py_lowcmd:
            LOGGER.info("Unitree SDK2 Python bridge disabled for this run")
            return False

        try:
            if self._config.enable_unitree_sdk2py_lowcmd:
                LOGGER.info("Unitree SDK2 Python lowcmd ingress is reserved for Phase 2")
            if self._config.enable_unitree_sdk2py_lowstate:
                self._start_bridge()
                self._lowstate_publisher.initialize()
            else:
                LOGGER.info("Unitree SDK2 Python lowstate publisher disabled for this run")
                return False
            self._sdk_enabled = True
            LOGGER.info(
                "initialized Unitree SDK2 Python bridge (domain_id=%s lowstate_topic=%s)",
                self._config.unitree_sdk2py_domain_id,
                self._config.unitree_sdk2py_lowstate_topic,
            )
            return True
        except Exception:
            self.shutdown()
            raise

    def step(self, simulation_time_seconds: float, snapshot: RobotKinematicSnapshot) -> Sdk2PyDdsStepResult:
        self.initialize()

        if self._sdk_enabled:
            self._poll_bridge_health()

        lowstate_published = False
        if (
            self._sdk_enabled
            and self._config.enable_unitree_sdk2py_lowstate
            and simulation_time_seconds >= self._next_lowstate_publish_time
        ):
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

        return Sdk2PyDdsStepResult(
            lowstate_published=lowstate_published,
            lowcmd_available=False,
            lowcmd_fresh=False,
        )

    def reset_runtime_state(self) -> None:
        self._next_lowstate_publish_time = 0.0
        self._simulation_cadence = CadenceTracker(label="sdk2py_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="sdk2py_wall_clock")
        LOGGER.info("cleared SDK2 Python DDS runtime state after simulator reset")

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
        self._lowstate_publisher = Sdk2PyLowStateUdpPublisher(
            host=self._config.bridge_bind_host,
            port=SDK2PY_UDP_LOWSTATE_PORT,
        )
        self._sdk_enabled = False
        self._initialized = False
        self._next_lowstate_publish_time = 0.0
        self._simulation_cadence = CadenceTracker(label="sdk2py_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="sdk2py_wall_clock")

    def _start_bridge(self) -> None:
        if self._bridge_process is not None and self._bridge_process.poll() is None:
            return
        sidecar_script = Path(__file__).resolve().parents[3] / "scripts" / "unitree_sdk2py_sidecar.py"
        self._cleanup_stale_sidecars(sidecar_script)
        command = [
            os.environ.get("UNITREE_SDK2PY_PYTHON_EXE", "python3"),
            str(sidecar_script),
            "--domain-id",
            str(self._config.unitree_sdk2py_domain_id),
            "--lowstate-topic",
            self._config.unitree_sdk2py_lowstate_topic,
            "--network-interface",
            self._config.unitree_sdk2py_network_interface,
            "--bind-host",
            self._config.bridge_bind_host,
            "--lowstate-port",
            str(SDK2PY_UDP_LOWSTATE_PORT),
        ]
        self._bridge_process = subprocess.Popen(
            command,
            cwd=Path(__file__).resolve().parents[3],
            env=self._bridge_environment(),
            text=True,
        )

    def _bridge_environment(self) -> dict[str, str]:
        env = {
            key: value
            for key, value in os.environ.items()
            if key
            not in {
                "PYTHONHOME",
                "PYTHONPATH",
                "LD_PRELOAD",
            }
        }
        sdk2py_root = Path.home() / "unitree_sdk2_python"
        if sdk2py_root.exists():
            env["PYTHONPATH"] = str(sdk2py_root)
        return env

    def _poll_bridge_health(self) -> None:
        if self._bridge_process is None:
            return
        return_code = self._bridge_process.poll()
        if return_code is not None and self._sdk_enabled:
            LOGGER.error("Unitree SDK2 Python bridge exited unexpectedly with code %s", return_code)
            self._sdk_enabled = False

    def _cleanup_stale_sidecars(self, sidecar_script: Path) -> None:
        """Terminate leftover SDK2 Python sidecar processes from prior simulator runs."""
        candidate_pids = self._find_stale_sidecar_pids(sidecar_script)
        if not candidate_pids:
            return

        LOGGER.warning(
            "terminating %s stale Unitree SDK2 Python sidecar process(es) before startup: %s",
            len(candidate_pids),
            ", ".join(str(pid) for pid in candidate_pids),
        )
        for pid in candidate_pids:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue

        deadline = time.monotonic() + 2.0
        remaining = set(candidate_pids)
        while remaining and time.monotonic() < deadline:
            time.sleep(0.05)
            remaining = {pid for pid in remaining if self._pid_exists(pid)}

        for pid in sorted(remaining):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                continue

    def _find_stale_sidecar_pids(self, sidecar_script: Path) -> list[int]:
        """Return PIDs for matching SDK2 Python sidecars that predate this manager."""
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                check=True,
                text=True,
                stdout=subprocess.PIPE,
            )
        except (OSError, subprocess.CalledProcessError):
            return []

        script_name = sidecar_script.name
        own_pid = os.getpid()
        bridge_pid = self._bridge_process.pid if self._bridge_process is not None else None
        candidate_pids: list[int] = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            pid_text, _, command = stripped.partition(" ")
            try:
                pid = int(pid_text)
            except ValueError:
                continue
            if pid in {own_pid, bridge_pid}:
                continue
            if script_name in command:
                candidate_pids.append(pid)
        return candidate_pids

    @staticmethod
    def _pid_exists(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _advance_lowstate_publish_schedule(self, simulation_time_seconds: float) -> None:
        while self._next_lowstate_publish_time <= simulation_time_seconds:
            self._next_lowstate_publish_time += self._lowstate_publish_period
