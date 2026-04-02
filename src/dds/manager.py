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
        log_prefix = "WARNING" if relative_error > warn_ratio else "INFO"
        print(
            "[unitree_g1_isaac_sim] "
            f"{log_prefix} lowstate cadence check: observed={observed_hz:.3f}Hz "
            f"expected={expected_hz:.3f}Hz publishes={publishes} "
            f"window_dt={elapsed:.3f}s rel_error={relative_error:.3%}"
        )

        self.window_start_time = t
        self.publish_count = 1


class DdsManager:
    """Own the DDS bridge lifecycle for the single-process simulator runtime."""

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._initialized = False
        self._sdk_enabled = False
        self._next_lowstate_publish_time = 0.0
        lowstate_hz = config.lowstate_publish_hz
        if lowstate_hz <= 0.0 or not math.isfinite(lowstate_hz):
            raise ValueError(
                f"Invalid AppConfig.lowstate_publish_hz={lowstate_hz!r}; "
                "expected a positive finite frequency in Hz (check --lowstate-publish-hz)."
            )
        self._lowstate_publish_period = _compute_publish_period(lowstate_hz)
        self._lowstate_publisher = G1LowStatePublisher(topic_name=config.lowstate_topic)
        self._lowcmd_subscriber = G1LowCmdSubscriber(topic_name=config.lowcmd_topic)
        self._warned_stale_lowcmd = False
        self._cadence = CadenceTracker()

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
        self._lowstate_publisher = G1LowStatePublisher(topic_name=self._config.lowstate_topic)
        self._lowcmd_subscriber = G1LowCmdSubscriber(topic_name=self._config.lowcmd_topic)
        self._sdk_enabled = False
        self._initialized = False
        self._warned_stale_lowcmd = False
        self._next_lowstate_publish_time = 0.0
        self._cadence = CadenceTracker()

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
                print(
                    "[unitree_g1_isaac_sim] cached `rt/lowcmd` sample went stale "
                    f"after {age_seconds:.3f}s; stopping command reapplication until a fresh sample arrives."
                )
                self._warned_stale_lowcmd = True
            return None

        if cached is None:
            self._warned_stale_lowcmd = False
            return None
        if self._warned_stale_lowcmd:
            print("[unitree_g1_isaac_sim] received fresh `rt/lowcmd` again; resuming command application.")
        self._warned_stale_lowcmd = False
        return cached

    def _advance_lowstate_publish_schedule(self, simulation_time_seconds: float) -> None:
        """Advance the next publish target from the prior schedule, not the current frame."""
        while self._next_lowstate_publish_time <= simulation_time_seconds:
            self._next_lowstate_publish_time += self._lowstate_publish_period


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
