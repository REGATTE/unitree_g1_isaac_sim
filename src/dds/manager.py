"""Top-level DDS lifecycle management for the Unitree G1 simulator.

This manager deliberately avoids the heavier shared-memory/threaded structure
used by the Isaac Lab reference. In this repository the simulator already runs
as a single Isaac Sim process, so the clearest first integration is:

1. initialize DDS once at startup
2. let the main simulation loop produce kinematic snapshots
3. publish `rt/lowstate` at a fixed simulation-time cadence
4. keep `rt/lowcmd` initialization explicit but optional until command
   application into Isaac Sim is implemented
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from config import AppConfig
from robot_state import RobotKinematicSnapshot

from .g1_lowcmd import G1LowCmdSubscriber
from .g1_lowstate import G1LowStatePublisher, unitree_sdk_is_available


@dataclass(frozen=True)
class DdsStepResult:
    """Outcome of one DDS update from the simulation loop."""

    lowstate_published: bool
    lowcmd_available: bool


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

    @property
    def latest_lowcmd(self):
        return self._lowcmd_subscriber.latest_command

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

        return DdsStepResult(
            lowstate_published=lowstate_published,
            lowcmd_available=self._lowcmd_subscriber.latest_command is not None,
        )

    def shutdown(self) -> None:
        """Release DDS-facing objects owned by the manager."""
        self._lowstate_publisher = G1LowStatePublisher(topic_name=self._config.lowstate_topic)
        self._lowcmd_subscriber = G1LowCmdSubscriber(topic_name=self._config.lowcmd_topic)
        self._sdk_enabled = False
        self._initialized = False


def _compute_publish_period(publish_hz: float) -> float:
    """Convert a positive publish rate into a simulation-time period."""
    if publish_hz <= 0.0:
        raise ValueError(f"lowstate publish rate must be positive, got {publish_hz}")
    return 1.0 / publish_hz
