"""Top-level DDS lifecycle management for the Unitree G1 simulator.

This manager deliberately avoids the heavier shared-memory/threaded structure
used by the Isaac Lab reference. In this repository the simulator already runs
as a single Isaac Sim process, so the clearest first integration is:

1. initialize DDS once at startup
2. let the main simulation loop produce kinematic snapshots
3. publish `rt/lowstate` at a fixed simulation-time cadence
4. treat `rt/lowcmd` freshness explicitly so stale commands stop being
   re-applied without crashing the runtime
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any

from config import AppConfig
from robot_state import RobotKinematicSnapshot

from .g1_lowcmd import G1LowCmdSubscriber, LowCmdCache
from .g1_lowstate import G1LowStatePublisher, unitree_sdk_is_available


@dataclass(frozen=True)
class DdsStepResult:
    """Outcome of one DDS update from the simulation loop."""

    lowstate_published: bool
    lowcmd_available: bool
    lowcmd_fresh: bool


class DdsManager:
    """Own the DDS bridge lifecycle for the single-process simulator runtime."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._initialized = False
        self._sdk_enabled = False
        self._next_lowstate_publish_time = 0.0
        self._lowstate_publish_period = _compute_publish_period(config.lowstate_publish_hz)
        self._lowstate_publisher = G1LowStatePublisher(topic_name=config.lowstate_topic)
        self._lowcmd_subscriber = G1LowCmdSubscriber(topic_name=config.lowcmd_topic)
        self._warned_stale_lowcmd = False
        self._cadence_window_start_time: float | None = None
        self._cadence_window_publish_count = 0

    @property
    def latest_lowcmd(self) -> LowCmdCache | None:
        return self._resolve_latest_lowcmd(now_monotonic=time.monotonic())

    def initialize(self) -> bool:
        """Initialize the Unitree DDS channel factory and requested bridges."""
        if self._initialized:
            return self._sdk_enabled
        self._initialized = True

        if not self._config.enable_dds:
            print("[unitree_g1_isaac_sim] DDS bridge disabled")
            return False
        if not unitree_sdk_is_available():
            print(
                "[unitree_g1_isaac_sim] DDS bridge enabled but `unitree_sdk2py` is unavailable. "
                "The simulator will continue without external DDS I/O."
            )
            return False

        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(self._config.dds_domain_id)
        self._sdk_enabled = True
        print(
            "[unitree_g1_isaac_sim] initialized Unitree DDS channel factory "
            f"(domain_id={self._config.dds_domain_id})"
        )

        self._lowstate_publisher.initialize()
        if self._config.enable_lowcmd_subscriber:
            self._lowcmd_subscriber.initialize()
        else:
            print("[unitree_g1_isaac_sim] lowcmd subscriber disabled for this run")
        return True

    def step(self, simulation_time_seconds: float, snapshot: RobotKinematicSnapshot) -> DdsStepResult:
        """Run one DDS update from the main simulation loop.

        The publish cadence is derived from simulation time, not host wall
        time, so the DDS state stream stays aligned with the physics loop.
        """
        self.initialize()

        lowstate_published = False
        if self._sdk_enabled and simulation_time_seconds >= self._next_lowstate_publish_time:
            lowstate_published = self._lowstate_publisher.publish(snapshot)
            self._next_lowstate_publish_time = simulation_time_seconds + self._lowstate_publish_period
            if lowstate_published:
                self._record_lowstate_publish(simulation_time_seconds)

        active_lowcmd = self._resolve_latest_lowcmd(now_monotonic=time.monotonic())

        return DdsStepResult(
            lowstate_published=lowstate_published,
            lowcmd_available=self._lowcmd_subscriber.latest_command is not None,
            lowcmd_fresh=active_lowcmd is not None,
        )

    def shutdown(self) -> None:
        """Release DDS-facing objects owned by the manager."""
        self._lowstate_publisher = G1LowStatePublisher(topic_name=self._config.lowstate_topic)
        self._lowcmd_subscriber = G1LowCmdSubscriber(topic_name=self._config.lowcmd_topic)
        self._sdk_enabled = False
        self._initialized = False
        self._warned_stale_lowcmd = False
        self._cadence_window_start_time = None
        self._cadence_window_publish_count = 0

    def _resolve_latest_lowcmd(self, now_monotonic: float) -> LowCmdCache | None:
        """Return the current fresh lowcmd sample or `None` if stale/absent."""
        cached = self._lowcmd_subscriber.latest_command
        if cached is None:
            self._warned_stale_lowcmd = False
            return None

        timeout_seconds = self._config.lowcmd_timeout_seconds
        if timeout_seconds <= 0.0:
            self._warned_stale_lowcmd = False
            return cached

        age_seconds = now_monotonic - cached.received_at_monotonic
        if age_seconds <= timeout_seconds:
            if self._warned_stale_lowcmd:
                print("[unitree_g1_isaac_sim] received fresh `rt/lowcmd` again; resuming command application.")
            self._warned_stale_lowcmd = False
            return cached

        if not self._warned_stale_lowcmd:
            print(
                "[unitree_g1_isaac_sim] cached `rt/lowcmd` sample went stale "
                f"after {age_seconds:.3f}s; stopping command reapplication until a fresh sample arrives."
            )
            self._warned_stale_lowcmd = True
        return None

    def _record_lowstate_publish(self, simulation_time_seconds: float) -> None:
        """Accumulate cadence diagnostics for simulation-time lowstate publishes."""
        interval = self._config.lowstate_cadence_report_interval
        if interval <= 0:
            return

        if self._cadence_window_start_time is None:
            self._cadence_window_start_time = simulation_time_seconds
            self._cadence_window_publish_count = 1
            return

        self._cadence_window_publish_count += 1
        if self._cadence_window_publish_count < interval:
            return

        elapsed = simulation_time_seconds - self._cadence_window_start_time
        publish_count = self._cadence_window_publish_count
        observed_hz = 0.0 if elapsed <= 0.0 else (publish_count - 1) / elapsed
        expected_hz = self._config.lowstate_publish_hz
        relative_error = 0.0 if expected_hz <= 0.0 else abs(observed_hz - expected_hz) / expected_hz
        log_prefix = "WARNING" if relative_error > self._config.lowstate_cadence_warn_ratio else "INFO"
        print(
            "[unitree_g1_isaac_sim] "
            f"{log_prefix} lowstate cadence check: observed={observed_hz:.3f}Hz "
            f"expected={expected_hz:.3f}Hz publishes={publish_count} "
            f"window_dt={elapsed:.3f}s rel_error={relative_error:.3%}"
        )
        self._cadence_window_start_time = simulation_time_seconds
        self._cadence_window_publish_count = 1


def _compute_publish_period(publish_hz: float) -> float:
    """Convert a positive publish rate into a simulation-time period."""
    if publish_hz <= 0.0:
        raise ValueError(f"lowstate publish rate must be positive, got {publish_hz}")
    if not math.isfinite(publish_hz):
        raise ValueError(f"lowstate publish rate must be finite, got {publish_hz}")
    return 1.0 / publish_hz
