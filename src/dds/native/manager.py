"""Native Unitree SDK bridge lifecycle for lowstate publication and lowcmd ingress."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import platform
from pathlib import Path
import signal
import subprocess
import time

from config import AppConfig
from robot_state import RobotKinematicSnapshot
from runtime_logging import get_logger

from dds.common.lowcmd_types import LowCmdCache
from dds.common.timing import CadenceTracker, compute_publish_period, is_fresh

from .lowcmd import NativeLowCmdUdpSubscriber
from .lowstate import NativeLowStateUdpPublisher


LOGGER = get_logger("dds.native_manager")

NATIVE_UDP_LOWSTATE_PORT = 35511
NATIVE_UDP_LOWCMD_PORT = 35512


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
        self._lowstate_publish_period = compute_publish_period(lowstate_hz)
        self._lowstate_publisher = NativeLowStateUdpPublisher(
            host=config.bridge_bind_host,
            port=NATIVE_UDP_LOWSTATE_PORT,
        )
        self._lowcmd_subscriber = NativeLowCmdUdpSubscriber(
            bind_host=config.bridge_bind_host,
            bind_port=NATIVE_UDP_LOWCMD_PORT,
        )
        self._simulation_cadence = CadenceTracker(label="native_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="native_wall_clock")
        self._warned_missing_bridge = False
        self._warned_stale_lowcmd = False

    @property
    def latest_lowcmd(self) -> LowCmdCache | None:
        if not self._config.enable_native_unitree_lowcmd:
            return None
        return self._resolve_latest_lowcmd(now_monotonic=time.monotonic())

    def initialize(self) -> bool:
        """Initialize the publish-only native DDS bridge."""
        if self._initialized:
            return self._sdk_enabled
        self._initialized = True

        if not self._config.enable_native_unitree_lowstate and not self._config.enable_native_unitree_lowcmd:
            LOGGER.info("native Unitree bridge disabled for this run")
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
            if self._config.enable_native_unitree_lowcmd:
                self._lowcmd_subscriber.initialize()
            else:
                LOGGER.info("native lowcmd subscriber disabled for this run")
            self._start_bridge(bridge_executable)
            if self._config.enable_native_unitree_lowstate:
                self._lowstate_publisher.initialize()
            else:
                LOGGER.info("native lowstate publisher disabled for this run")
            self._sdk_enabled = True
            LOGGER.info(
                "initialized native Unitree bridge (domain_id=%s lowstate_topic=%s lowcmd_topic=%s)",
                self._config.native_unitree_domain_id,
                self._config.native_unitree_lowstate_topic,
                self._config.native_unitree_lowcmd_topic,
            )
            return True
        except Exception:
            self.shutdown()
            raise

    def step(self, simulation_time_seconds: float, snapshot: RobotKinematicSnapshot) -> NativeDdsStepResult:
        self.initialize()

        if self._sdk_enabled:
            self._poll_bridge_health()
            if self._config.enable_native_unitree_lowcmd:
                self._lowcmd_subscriber.poll()

        lowstate_published = False
        if (
            self._sdk_enabled
            and self._config.enable_native_unitree_lowstate
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

        active_lowcmd = self._resolve_latest_lowcmd(now_monotonic=time.monotonic())

        return NativeDdsStepResult(
            lowstate_published=lowstate_published,
            lowcmd_available=(
                self._config.enable_native_unitree_lowcmd
                and self._lowcmd_subscriber.latest_command is not None
            ),
            lowcmd_fresh=active_lowcmd is not None,
        )

    def reset_runtime_state(self) -> None:
        if hasattr(self._lowcmd_subscriber, "clear_cached_command"):
            self._lowcmd_subscriber.clear_cached_command()
        self._warned_stale_lowcmd = False
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
        self._lowcmd_subscriber.close()
        self._lowstate_publisher = NativeLowStateUdpPublisher(
            host=self._config.bridge_bind_host,
            port=NATIVE_UDP_LOWSTATE_PORT,
        )
        self._lowcmd_subscriber = NativeLowCmdUdpSubscriber(
            bind_host=self._config.bridge_bind_host,
            bind_port=NATIVE_UDP_LOWCMD_PORT,
        )
        self._sdk_enabled = False
        self._initialized = False
        self._warned_stale_lowcmd = False
        self._next_lowstate_publish_time = 0.0
        self._simulation_cadence = CadenceTracker(label="native_simulation_time")
        self._wall_clock_cadence = CadenceTracker(label="native_wall_clock")

    def _start_bridge(self, bridge_executable: Path) -> None:
        if self._bridge_process is not None and self._bridge_process.poll() is None:
            return
        self._cleanup_stale_bridges(bridge_executable)
        command = [
            str(bridge_executable),
            "--domain-id",
            str(self._config.native_unitree_domain_id),
            "--enable-lowstate" if self._config.enable_native_unitree_lowstate else "--disable-lowstate",
            "--lowstate-topic",
            self._config.native_unitree_lowstate_topic,
            "--bind-host",
            self._config.bridge_bind_host,
            "--lowstate-port",
            str(NATIVE_UDP_LOWSTATE_PORT),
        ]
        if self._config.enable_native_unitree_lowcmd:
            command.extend(
                [
                    "--enable-lowcmd",
                    "--lowcmd-topic",
                    self._config.native_unitree_lowcmd_topic,
                    "--lowcmd-port",
                    str(self._lowcmd_subscriber.bound_port),
                ]
            )
        self._bridge_process = subprocess.Popen(
            command,
            cwd=Path(__file__).resolve().parents[3],
            env=self._bridge_environment(bridge_executable),
            text=True,
        )

    def _bridge_environment(self, bridge_executable: Path) -> dict[str, str]:
        env = dict(os.environ)
        sdk_thirdparty_lib = _resolve_unitree_sdk2_thirdparty_lib(bridge_executable)
        if sdk_thirdparty_lib is None:
            return env
        existing = env.get("LD_LIBRARY_PATH", "")
        entries = [str(sdk_thirdparty_lib)]
        if existing:
            entries.append(existing)
        env["LD_LIBRARY_PATH"] = os.pathsep.join(entries)
        return env

    def _poll_bridge_health(self) -> None:
        if self._bridge_process is None:
            return
        return_code = self._bridge_process.poll()
        if return_code is not None and self._sdk_enabled:
            LOGGER.error("native Unitree bridge exited unexpectedly with code %s", return_code)
            self._sdk_enabled = False

    def _cleanup_stale_bridges(self, bridge_executable: Path) -> None:
        """Terminate leftover native bridge processes from prior simulator runs."""
        candidate_pids = self._find_stale_bridge_pids(bridge_executable)
        if not candidate_pids:
            return

        LOGGER.warning(
            "terminating %s stale native Unitree bridge process(es) before startup: %s",
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

    def _find_stale_bridge_pids(self, bridge_executable: Path) -> list[int]:
        """Return PIDs for matching native bridge binaries that predate this manager."""
        try:
            result = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                check=True,
                capture_output=True,
                text=True,
            )
        except (OSError, subprocess.SubprocessError):
            return []

        bridge_tokens = {str(bridge_executable)}
        try:
            bridge_tokens.add(str(bridge_executable.resolve()))
        except OSError:
            pass
        bridge_tokens.add(bridge_executable.name)
        current_pid = os.getpid()
        active_bridge_pid = self._bridge_process.pid if self._bridge_process is not None else None
        pids: list[int] = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped or not any(token in stripped for token in bridge_tokens):
                continue
            pid_text, _, command = stripped.partition(" ")
            if not any(token in command for token in bridge_tokens):
                continue
            try:
                pid = int(pid_text)
            except ValueError:
                continue
            if pid == current_pid or pid == active_bridge_pid:
                continue
            pids.append(pid)
        return pids

    @staticmethod
    def _pid_exists(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _resolve_latest_lowcmd(self, now_monotonic: float) -> LowCmdCache | None:
        if not self._config.enable_native_unitree_lowcmd:
            self._warned_stale_lowcmd = False
            return None
        cached = self._lowcmd_subscriber.latest_command
        fresh = is_fresh(now_monotonic, cached, self._config.lowcmd_timeout_seconds)
        if not fresh:
            if cached is None:
                self._warned_stale_lowcmd = False
                return None
            if not self._warned_stale_lowcmd:
                age_seconds = now_monotonic - cached.received_at_monotonic
                LOGGER.warning(
                    "cached native `rt/lowcmd` sample went stale after %.3fs; stopping command "
                    "reapplication until a fresh sample arrives.",
                    age_seconds,
                )
                self._warned_stale_lowcmd = True
            return None

        if cached is None:
            self._warned_stale_lowcmd = False
            return None
        if self._warned_stale_lowcmd:
            LOGGER.info("received fresh native `rt/lowcmd` again; resuming command application.")
        self._warned_stale_lowcmd = False
        return cached

    def _advance_lowstate_publish_schedule(self, simulation_time_seconds: float) -> None:
        while self._next_lowstate_publish_time <= simulation_time_seconds:
            self._next_lowstate_publish_time += self._lowstate_publish_period


def _resolve_unitree_sdk2_thirdparty_lib(bridge_executable: Path) -> Path | None:
    """Find the bundled Unitree SDK2 DDS library directory for bridge startup."""
    arch = platform.machine()
    candidates = [
        Path.home() / "unitree_sdk2" / "thirdparty" / "lib" / arch,
        bridge_executable.parents[2] / "thirdparty" / "lib" / arch
        if len(bridge_executable.parents) > 2
        else None,
    ]
    for candidate in candidates:
        if candidate is not None and (candidate / "libddsc.so").exists() and (candidate / "libddscxx.so").exists():
            return candidate
    return None
